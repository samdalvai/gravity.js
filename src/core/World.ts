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
import { applyWeightForce } from '../force/Gravity';
import { Joint } from '../joint/Joint';
import { Vec2 } from '../math/Vec2';
import * as Utils from '../utils/Utils';
import { makeSoft, MAX_BODIES, MIN_BULLET_SPEED_SQUARED, SETTINGS } from './Constants';
import { RigidBody } from './RigidBody';

export interface WorldOptions {
    /** Maximum live bodies. Defaults to MAX_BODIES; Infinity disables the limit. */
    maxBodies?: number;
    /** Combines the two body friction coefficients for each contact. */
    frictionCallback?: ContactMaterialCallback;
    /** Combines the two body restitution coefficients for each contact. */
    restitutionCallback?: ContactMaterialCallback;
}

export type ContactMaterialCallback = (materialA: number, materialB: number, bodyA: RigidBody, bodyB: RigidBody) => number;

interface PersistentContact {
    bodyA: RigidBody;
    bodyB: RigidBody;
    manifold: ContactManifold | null;
    touching: boolean;
    seen: boolean;
}

/** Per-update counters and timings. Enable with `SETTINGS.collectMetrics`. */
export interface WorldMetrics {
    updateMs: number;
    broadPhaseMs: number;
    narrowPhaseMs: number;
    solveMs: number;
    integrationMs: number;
    broadPhaseCalls: number;
    narrowPhaseCalls: number;
    insertionSortShifts: number;
    potentialPairCount: number;
    narrowPhaseTests: number;
    manifoldCount: number;
    constraintIterations: number;
    maxPenetrationDepth: number;
}

const emptyMetrics = (): WorldMetrics => ({
    updateMs: 0,
    broadPhaseMs: 0,
    narrowPhaseMs: 0,
    solveMs: 0,
    integrationMs: 0,
    broadPhaseCalls: 0,
    narrowPhaseCalls: 0,
    insertionSortShifts: 0,
    potentialPairCount: 0,
    narrowPhaseTests: 0,
    manifoldCount: 0,
    constraintIterations: 0,
    maxPenetrationDepth: 0,
});

const now = (): number => (typeof performance === 'undefined' ? Date.now() : performance.now());

export class World {
    readonly maxBodies: number;
    private readonly up = new Vec2(0, 1);
    private G: number;

    private bodies: RigidBody[] = [];
    /** Pairs are allocated in blocks of 2 */
    private potentialPairs: RigidBody[] = [];

    private joints: Joint[] = [];

    private manifolds: ContactManifold[] = [];
    private readonly contacts: Map<Utils.PairKey, PersistentContact> = new Map();

    private readonly manifoldPool = NarrowPhase.manifoldPool;

    private forces: Vec2[] = [];
    private torques: number[] = [];

    private dtFractions: number[] = [];
    private readonly frictionCallback: ContactMaterialCallback;
    private readonly restitutionCallback: ContactMaterialCallback;

    // TODO: evaluate to delete this after implementation
    private metrics: WorldMetrics = emptyMetrics();

    constructor(gravity: number, options: WorldOptions = {}) {
        const maxBodies = options.maxBodies ?? MAX_BODIES;
        if (maxBodies !== Infinity && (!Number.isSafeInteger(maxBodies) || maxBodies < 0)) {
            throw new RangeError('maxBodies must be a non-negative safe integer or Infinity');
        }

        this.maxBodies = maxBodies;
        this.G = -gravity;
        this.frictionCallback = options.frictionCallback ?? ((a, b) => Math.sqrt(a * b));
        this.restitutionCallback = options.restitutionCallback ?? ((a, b) => Math.max(a, b));
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
                this.destroyContactsForBody(body);
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

    /** Returns a snapshot of the counters from the most recent `update()`. */
    getMetrics(): Readonly<WorldMetrics> {
        return { ...this.metrics };
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
        const collectMetrics = settings.collectMetrics;
        const updateStart = collectMetrics ? now() : 0;

        if (collectMetrics) {
            this.metrics = emptyMetrics();
        }

        const bodies = this.bodies;
        let start = collectMetrics ? now() : 0;
        this.broadPhase();
        if (collectMetrics) {
            this.metrics.broadPhaseCalls++;
            this.metrics.broadPhaseMs += now() - start;
        }

        start = collectMetrics ? now() : 0;
        this.narrowPhase();
        if (collectMetrics) {
            this.metrics.narrowPhaseCalls++;
            this.metrics.narrowPhaseMs += now() - start;
        }

        let applyWarmStarting = true;
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
                    this.step(dtFraction, applyWarmStarting);
                    applyWarmStarting = false;
                }
            } else {
                this.step(dt, applyWarmStarting);
                applyWarmStarting = false;
            }
        }

        if (collectMetrics) {
            this.metrics.updateMs = now() - updateStart;
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

    private step(dt: number, applyWarmStarting: boolean) {
        const bodies = this.bodies;
        const invDt = dt === 0 ? 0 : 1 / dt;
        const collectMetrics = SETTINGS.collectMetrics;

        let start = collectMetrics ? now() : 0;
        for (let i = 0; i < this.manifolds.length; i++) this.setGrounded(this.manifolds[i]);
        this.solveConstraints(invDt, applyWarmStarting);
        if (collectMetrics) {
            this.metrics.solveMs += now() - start;
        }

        // Integrate all the velocities
        start = collectMetrics ? now() : 0;
        if (dt !== 0) {
            for (let i = 0; i < bodies.length; i++) {
                const body = bodies[i];
                body.integrateVelocities(dt);
            }
        }
        this.solveContactRestitutionAndFriction();
        if (collectMetrics) {
            this.metrics.integrationMs += now() - start;
        }
    }

    private broadPhase() {
        // Use insertion sort instead of Array.sort to exploit temporal coherence:
        // between frames, bodies move only slightly, so the array is already nearly sorted by minX.
        // In this case insertion sort runs in ~O(n) (only small local swaps),
        // while a full sort would still cost O(n log n).
        const bodies = this.bodies;

        for (let i = 1; i < bodies.length; i++) {
            const current = bodies[i];
            let j = i - 1;

            while (j >= 0 && bodies[j].minX > current.minX) {
                bodies[j + 1] = bodies[j];
                j--;
                if (SETTINGS.collectMetrics) this.metrics.insertionSortShifts++;
            }

            bodies[j + 1] = current;
        }

        this.potentialPairs.length = 0;

        // Broad phase check with prune & sweep algorithm
        for (let i = 0, len = bodies.length; i < len; i++) {
            const a = bodies[i];

            for (let j = i + 1; j < len; j++) {
                const b = bodies[j];

                // If objects don't overlap on X axis they cannot collide
                if (b.minX > a.maxX) break;

                // If objects overlap on X axis but don't overlap on Y axis the cannot collide
                if (a.maxY < b.minY || a.minY > b.maxY) {
                    continue;
                }

                if (!canCollide(a, b)) {
                    continue;
                }

                // Objects may be colliding
                this.potentialPairs.push(a, b);
            }
        }

        if (SETTINGS.collectMetrics) {
            this.metrics.potentialPairCount += this.potentialPairs.length / 2;
        }
    }

    private narrowPhase() {
        const pairs = this.potentialPairs;
        const warmStarting = SETTINGS.warmStarting;

        for (const contact of this.contacts.values()) contact.seen = false;
        this.manifolds.length = 0;

        for (let i = 0; i < pairs.length; i += 2) {
            let a = pairs[i];
            let b = pairs[i + 1];
            if (a.isStatic() && b.isStatic()) continue;
            if (SETTINGS.collectMetrics) this.metrics.narrowPhaseTests++;

            if (a.id > b.id) {
                const tmp = a;
                a = b;
                b = tmp;
            }

            const key = Utils.pairKey(a, b);
            let contact = this.contacts.get(key);
            if (contact === undefined) {
                contact = { bodyA: a, bodyB: b, manifold: null, touching: false, seen: true };
                this.contacts.set(key, contact);
            } else {
                contact.seen = true;
            }

            const oldManifold = contact.manifold;
            const newManifold = NarrowPhase.detectCollision(a, b);
            if (newManifold == null) {
                if (contact.touching && oldManifold != null) this.emitContactEnd(oldManifold);
                contact.touching = false;
                if (oldManifold != null) this.manifoldPool.release(oldManifold);
                contact.manifold = null;
                continue;
            }

            newManifold.setMaterialProperties(
                this.frictionCallback(a.friction, b.friction, a, b),
                this.restitutionCallback(a.restitution, b.restitution, a, b),
            );
            let manifold = newManifold;
            if (oldManifold != null) {
                if (warmStarting) newManifold.tryWarmStart(oldManifold);
                oldManifold.updateFrom(newManifold, warmStarting);
                this.manifoldPool.release(newManifold);
                manifold = oldManifold;
            }

            contact.manifold = manifold;
            this.manifolds.push(manifold);
            if (!contact.touching) this.emitContactBegin(manifold);
            contact.touching = true;
            this.setGrounded(manifold);
        }

        for (const [key, contact] of this.contacts) {
            if (contact.seen) continue;
            if (contact.touching && contact.manifold != null) this.emitContactEnd(contact.manifold);
            if (contact.manifold != null) this.manifoldPool.release(contact.manifold);
            this.contacts.delete(key);
        }

        if (SETTINGS.collectMetrics) {
            this.metrics.manifoldCount = this.manifolds.length;
            for (let i = 0; i < this.manifolds.length; i++) {
                this.metrics.maxPenetrationDepth = Math.max(
                    this.metrics.maxPenetrationDepth,
                    this.manifolds[i].penetrationDepth,
                );
            }
        }
    }

    private solveConstraints(invDt: number, applyWarmStarting: boolean) {
        // Presolve constraints
        const hertz = Math.min(SETTINGS.contactHertz, 0.125 * invDt);
        const contactSoftness = makeSoft(hertz, SETTINGS.contactDampingRatio, invDt > 0 ? 1 / invDt : 0);
        const staticSoftness = makeSoft(2 * hertz, SETTINGS.contactDampingRatio, invDt > 0 ? 1 / invDt : 0);
        for (let i = 0; i < this.manifolds.length; i++) {
            const manifold = this.manifolds[i];
            if (applyWarmStarting) {
                manifold.preSolve(invDt, contactSoftness, staticSoftness, true);
            } else {
                manifold.refreshForSubStep(invDt, contactSoftness, staticSoftness);
            }
        }

        for (let i = 0; i < this.joints.length; i++) this.joints[i].preSolve(invDt);

        // Solve constraints
        for (let i = 0; i < SETTINGS.solverIterations; i++) {
            for (let j = 0; j < this.manifolds.length; j++) this.manifolds[j].solveBias();

            for (let j = 0; j < this.joints.length; j++) this.joints[j].solve();
        }

        if (SETTINGS.collectMetrics) {
            this.metrics.constraintIterations += SETTINGS.solverIterations * 2;
        }

    }

    private runContactCallbacks() {
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

    private emitContactBegin(manifold: ContactManifold): void {
        manifold.bodyA.onContactBegin?.(manifold.contactInfo);
        manifold.bodyB.onContactBegin?.(manifold.contactInfo);
    }

    private emitContactEnd(manifold: ContactManifold): void {
        manifold.bodyA.onContactEnd?.(manifold.contactInfo);
        manifold.bodyB.onContactEnd?.(manifold.contactInfo);
    }

    private destroyContactsForBody(body: RigidBody): void {
        for (const [key, contact] of this.contacts) {
            if (contact.bodyA !== body && contact.bodyB !== body) continue;
            if (contact.touching && contact.manifold != null) this.emitContactEnd(contact.manifold);
            if (contact.manifold != null) {
                const index = this.manifolds.indexOf(contact.manifold);
                if (index >= 0) this.manifolds.splice(index, 1);
                this.manifoldPool.release(contact.manifold);
            }
            this.contacts.delete(key);
        }
    }

    private solveContactRestitutionAndFriction() {
        for (let i = 0; i < SETTINGS.solverIterations; i++) {
            for (let j = 0; j < this.manifolds.length; j++) {
                this.manifolds[j].solveRestitutionAndFriction();
            }
        }
        this.runContactCallbacks();
    }

    private setGrounded(manifold: ContactManifold) {
        const bodyA = manifold.bodyA;
        const bodyB = manifold.bodyB;
        const normalY = manifold.contactNormalY;

        if (!bodyA.isStatic() && normalY < -0.5) bodyA.isGrounded = true;
        if (!bodyB.isStatic() && normalY > 0.5) bodyB.isGrounded = true;
    }

    clear() {
        for (const contact of this.contacts.values()) {
            if (contact.touching && contact.manifold != null) this.emitContactEnd(contact.manifold);
            if (contact.manifold != null) this.manifoldPool.release(contact.manifold);
        }

        this.bodies.length = 0;
        this.potentialPairs.length = 0;
        this.manifolds.length = 0;
        this.contacts.clear();
        this.joints.length = 0;
        this.forces.length = 0;
        this.torques.length = 0;
    }
}
