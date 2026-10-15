/*
 * Portions of this file are derived from the Sopiro Physics Engine.
 *
 * Copyright (c) 2022 Sopiro
 * Licensed under the MIT License
 *
 * Original project:
 * https://github.com/Sopiro
 */
import * as CCD from '../collision/CCD';
import { canCollide } from '../collision/CollisionFilter';
import { ContactManifold } from '../collision/ContactManifold';
import * as NarrowPhase from '../collision/NarrowPhase';
import { PairCache } from '../collision/PairCache';
import { applyWeightForce } from '../force/Gravity';
import { Joint } from '../joint/Joint';
import { Vec2 } from '../math/Vec2';
import { MIN_BULLET_SPEED_SQUARED, SETTINGS } from './Constants';
import { RigidBody } from './RigidBody';
import { SolverBodies } from './SolverBodies';

const DEFAULT_MAX_BODIES = 5_000;

/** The sweep axis changes only when the other axis spreads bodies this much more, to avoid flip-flopping */
const SWEEP_AXIS_HYSTERESIS = 1.5;

export interface WorldOptions {
    /** Maximum live bodies. Defaults to 5,000; Infinity disables the limit. */
    maxBodies?: number;
}

export class World {
    readonly maxBodies: number;
    private readonly up = new Vec2(0, 1);
    private G: number;

    private bodies: RigidBody[] = [];
    /** Pairs are allocated in blocks of 2 */
    private potentialPairs: RigidBody[] = [];

    private joints: Joint[] = [];

    private manifolds: ContactManifold[] = [];
    private manifoldsNext: ContactManifold[] = [];

    private manifoldMap = new PairCache<ContactManifold>();
    private manifoldMapNext = new PairCache<ContactManifold>();

    private readonly manifoldPool = NarrowPhase.manifoldPool;

    private forces: Vec2[] = [];
    private torques: number[] = [];

    private dtFractions: number[] = [];

    private sweepAxisIsX = true;

    private readonly solverBodies = new SolverBodies();

    constructor(gravity: number, options: WorldOptions = {}) {
        const maxBodies = options.maxBodies ?? DEFAULT_MAX_BODIES;
        if (maxBodies !== Infinity && (!Number.isSafeInteger(maxBodies) || maxBodies < 0)) {
            throw new RangeError('maxBodies must be a non-negative safe integer or Infinity');
        }

        this.maxBodies = maxBodies;
        this.G = -gravity;
    }

    addBody(body: RigidBody): void {
        if (this.bodies.length >= this.maxBodies) throw new Error('Max number of bodies exceeded');

        this.bodies.push(body);
    }

    removeBody(body: RigidBody): void {
        for (let i = 0; i < this.bodies.length; i++) {
            const current = this.bodies[i];

            // It suffices to look for the position going below the screen
            if (body.id === current.id) {
                this.bodies[i] = this.bodies[this.bodies.length - 1];
                this.bodies.pop();
                return;
            }
        }
    }

    getBodies(): readonly RigidBody[] {
        return this.bodies;
    }

    getManifolds(): ContactManifold[] {
        return this.manifolds;
    }

    addJoint(joint: Joint): void {
        this.joints.push(joint);
    }

    removeJoint(joint: Joint): void {
        for (let i = 0; i < this.joints.length; i++) {
            const current = this.joints[i];

            if (joint.id === current.id) {
                this.joints[i] = this.joints[this.joints.length - 1];
                this.joints.pop();
                return;
            }
        }
    }

    getJoints(): Joint[] {
        return this.joints;
    }

    addForce(force: Vec2): void {
        this.forces.push(force);
    }

    addTorque(torque: number): void {
        this.torques.push(torque);
    }

    update(beforeSubStep?: (dt: number) => void): void {
        const settings = SETTINGS;
        const dt = settings.dt;
        const subSteps = settings.subSteps;

        const bodies = this.bodies;

        for (let i = 0; i < subSteps; i++) {
            beforeSubStep?.(dt);

            // Loop all bodies of the world applying forces
            for (let i = 0; i < bodies.length; i++) {
                const body = bodies[i];
                if (SETTINGS.applyGravity) {
                    // Apply the weight force to all bodies
                    applyWeightForce(body, this.G * body.gravityScale);
                }

                // Apply forces to all bodies
                const forces = this.forces;
                for (let j = 0; j < forces.length; j++) {
                    body.addForce(forces[j]);
                }

                // Apply torque to all bodiesx
                const torques = this.torques;
                for (let j = 0; j < torques.length; j++) {
                    body.addTorque(torques[j]);
                }

                // Update last grounded time
                if (body.isGrounded) {
                    body.lastGroundedTime = 0;
                } else {
                    // Since we have fixed dt we can safely assume that the last frame dt is the same as this one
                    body.lastGroundedTime += dt;
                }

                // Reset grounded value at the beginning of each frame
                body.isGrounded = false;
            }

            // Integrate all the forces
            for (let i = 0; i < bodies.length; i++) {
                const body = bodies[i];
                body.integrateForces(dt);
            }

            if (SETTINGS.ccd) {
                this.ccd(dt);

                for (let i = 0; i < this.dtFractions.length; i++) {
                    const dtFraction = this.dtFractions[i];
                    this.step(dtFraction);
                }
            } else {
                this.step(dt);
            }
        }
    }

    private ccd(dt: number) {
        const fractions = this.dtFractions;
        fractions.length = 0;

        const bodies = this.bodies;

        for (let i = 0; i < bodies.length; i++) {
            const body = bodies[i];

            if (!body.isBullet) continue;

            if (body.velocity.magnitudeSquared() <= MIN_BULLET_SPEED_SQUARED) {
                body.isBullet = false;
                continue;
            }

            const fraction = CCD.resolveCCD(body, bodies, dt);

            if (fraction != null) {
                fractions[fractions.length] = fraction;
            }
        }

        fractions[fractions.length] = 1;
        fractions.sort((a, b) => a - b);

        let previous = 0;

        // Convert fractions to dt slices, the sum of the slices will be equal to dt
        for (let i = 0; i < fractions.length; i++) {
            const current = fractions[i] * dt;
            fractions[i] = current - previous;
            previous = current;
        }
    }

    private step(dt: number) {
        const bodies = this.bodies;
        // Bias terms always use the full sub-step: CCD slices can be arbitrarily small and
        // 1 / slice would turn position correction into huge velocities
        const invDt = dt === 0 ? 0 : SETTINGS.invDt;

        this.broadPhase();

        this.narrowPhase();

        this.solveConstraints(invDt);

        // Integrate all the velocities
        if (dt !== 0) {
            for (let i = 0; i < bodies.length; i++) {
                const body = bodies[i];
                body.integrateVelocities(dt);
            }
        }
    }

    private broadPhase() {
        const bodies = this.bodies;
        const axisChanged = this.updateSweepAxis();

        if (axisChanged) {
            bodies.sort((a, b) => a.sweepMin - b.sweepMin);
        } else {
            // Use insertion sort instead of Array.sort to exploit temporal coherence:
            // between frames, bodies move only slightly, so the array is already nearly sorted.
            // In this case insertion sort runs in ~O(n) (only small local swaps),
            // while a full sort would still cost O(n log n).
            for (let i = 1; i < bodies.length; i++) {
                const current = bodies[i];
                let j = i - 1;

                while (j >= 0 && bodies[j].sweepMin > current.sweepMin) {
                    bodies[j + 1] = bodies[j];
                    j--;
                }

                bodies[j + 1] = current;
            }
        }

        this.potentialPairs.length = 0;

        // Broad phase check with prune & sweep algorithm
        for (let i = 0, len = bodies.length; i < len; i++) {
            const a = bodies[i];

            for (let j = i + 1; j < len; j++) {
                const b = bodies[j];

                // If objects don't overlap on the sweep axis they cannot collide
                if (b.sweepMin > a.sweepMax) break;

                // If objects overlap on the sweep axis but don't overlap on the other axis the cannot collide
                if (a.crossMax < b.crossMin || a.crossMin > b.crossMax) {
                    continue;
                }

                if (!canCollide(a, b)) {
                    continue;
                }

                // Objects may be colliding
                this.potentialPairs.push(a, b);
            }
        }
    }

    /**
     * Sweeping along a single axis degrades to O(n^2) when many bodies share the same range on that axis
     * (e.g. tall piles when sweeping on X). Sweep along the axis where body centers are more spread out.
     * Also caches the AABB projections used by the sweep. Returns true when the sweep axis changed.
     */
    private updateSweepAxis(): boolean {
        const bodies = this.bodies;
        const n = bodies.length;

        let sumX = 0;
        let sumY = 0;
        let sumXX = 0;
        let sumYY = 0;

        for (let i = 0; i < n; i++) {
            const body = bodies[i];
            const centerX = (body.minX + body.maxX) * 0.5;
            const centerY = (body.minY + body.maxY) * 0.5;
            sumX += centerX;
            sumY += centerY;
            sumXX += centerX * centerX;
            sumYY += centerY * centerY;
        }

        let axisChanged = false;

        if (n > 1) {
            const varianceX = sumXX / n - (sumX / n) ** 2;
            const varianceY = sumYY / n - (sumY / n) ** 2;

            if (this.sweepAxisIsX && varianceY > varianceX * SWEEP_AXIS_HYSTERESIS) {
                this.sweepAxisIsX = false;
                axisChanged = true;
            } else if (!this.sweepAxisIsX && varianceX > varianceY * SWEEP_AXIS_HYSTERESIS) {
                this.sweepAxisIsX = true;
                axisChanged = true;
            }
        }

        if (this.sweepAxisIsX) {
            for (let i = 0; i < n; i++) {
                const body = bodies[i];
                body.sweepMin = body.minX;
                body.sweepMax = body.maxX;
                body.crossMin = body.minY;
                body.crossMax = body.maxY;
            }
        } else {
            for (let i = 0; i < n; i++) {
                const body = bodies[i];
                body.sweepMin = body.minY;
                body.sweepMax = body.maxY;
                body.crossMin = body.minX;
                body.crossMax = body.maxX;
            }
        }

        return axisChanged;
    }

    private narrowPhase() {
        const oldManifolds = this.manifolds;

        const newManifolds = this.manifoldsNext;
        const newMap = this.manifoldMapNext;

        newManifolds.length = 0;
        newMap.clear();

        const pairs = this.potentialPairs;
        const oldMap = this.manifoldMap;
        const warmStarting = SETTINGS.warmStarting;

        // Narrow phase check, potential pairs may still not collide
        for (let i = 0; i < pairs.length; i += 2) {
            let a = pairs[i];
            let b = pairs[i + 1];

            if (a.isStatic() && b.isStatic()) continue;

            // Improve coherence
            if (a.id > b.id) {
                const tmp = a;
                a = b;
                b = tmp;
            }

            const newManifold = NarrowPhase.detectCollision(a, b);
            if (newManifold == null) continue;

            if (warmStarting) {
                const oldManifold = oldMap.get(a.id, b.id);
                if (oldManifold !== undefined) {
                    newManifold.tryWarmStart(oldManifold);
                }
            }

            newMap.set(a.id, b.id, newManifold);
            newManifolds.push(newManifold);

            this.setGrounded(newManifold);
        }

        for (let i = 0; i < oldManifolds.length; i++) {
            this.manifoldPool.release(oldManifolds[i]);
        }

        this.manifolds = newManifolds;
        this.manifoldsNext = oldManifolds;

        this.manifoldMap = newMap;
        this.manifoldMapNext = oldMap;
    }

    private solveConstraints(invDt: number) {
        const manifolds = this.manifolds;
        const joints = this.joints;
        const solverBodies = this.solverBodies;

        // Stage body velocities in a packed array for the solver, joints may also reference bodies
        // that are not part of the world
        solverBodies.begin();

        const bodies = this.bodies;
        for (let i = 0; i < bodies.length; i++) solverBodies.add(bodies[i]);

        for (let i = 0; i < joints.length; i++) {
            solverBodies.add(joints[i].bodyA);
            solverBodies.add(joints[i].bodyB);
        }

        // Read after staging, the array grows while bodies are added
        const v = solverBodies.velocities;

        // Presolve constraints
        for (let i = 0; i < manifolds.length; i++) manifolds[i].preSolve(invDt, v);

        for (let i = 0; i < joints.length; i++) joints[i].preSolve(invDt, v);

        // Solve constraints
        const iterations = SETTINGS.solverIterations;
        for (let i = 0; i < iterations; i++) {
            for (let j = 0; j < manifolds.length; j++) manifolds[j].solve(v);

            for (let j = 0; j < joints.length; j++) joints[j].solve(v);
        }

        solverBodies.end();

        // Run contact callbacks
        for (let i = 0; i < this.manifolds.length; i++) {
            const manifold = this.manifolds[i];
            const bodyA = manifold.bodyA;
            const bodyB = manifold.bodyB;

            if (bodyA.onContact) {
                bodyA.onContact(manifold.contactInfo);
            }

            if (bodyB.onContact) {
                bodyB.onContact(manifold.contactInfo);
            }
        }
    }

    private setGrounded(manifold: ContactManifold) {
        const bodyA = manifold.bodyA;
        const bodyB = manifold.bodyB;
        const normalY = manifold.contactNormalY;

        if (!bodyA.isStatic() && normalY < -0.5) bodyA.isGrounded = true;
        if (!bodyB.isStatic() && normalY > 0.5) bodyB.isGrounded = true;
    }

    clear() {
        for (let i = 0; i < this.manifolds.length; i++) {
            this.manifoldPool.release(this.manifolds[i]);
        }

        this.bodies.length = 0;
        this.potentialPairs.length = 0;
        this.manifolds.length = 0;
        this.manifoldMap.clear();
        this.joints.length = 0;
        this.forces.length = 0;
        this.torques.length = 0;
    }
}
