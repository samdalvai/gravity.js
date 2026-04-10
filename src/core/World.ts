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
import { ContactManifold } from '../collision/ContactManifold';
import * as NarrowPhase from '../collision/NarrowPhase';
import { Force } from '../force/Force';
import { Joint } from '../joint/Joint';
import { Vec2 } from '../math/Vec2';
import * as Utils from '../utils/Utils';
import { MIN_BULLET_SPEED, SETTINGS } from './Constants';
import { RigidBody } from './RigidBody';

export class World {
    private readonly up = new Vec2(0, 1);
    private G: number;

    private bodies: RigidBody[] = [];

    public manifolds: ContactManifold[] = [];
    public joints: Joint[] = [];

    public potentialPairs: [RigidBody, RigidBody][] = [];
    public manifoldMap: Map<number, ContactManifold> = new Map();

    private forces: Vec2[] = [];
    private torques: number[] = [];

    private dtFractions: number[] = [];

    constructor(gravity: number) {
        this.G = -gravity;
    }

    addBody(body: RigidBody): void {
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

    getBodies(): RigidBody[] {
        return this.bodies;
    }

    getManifolds(): ContactManifold[] {
        return this.manifolds;
    }

    addJoint(constraint: Joint): void {
        this.joints.push(constraint);
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

    update(dt: number): void {
        // Loop all bodies of the world applying forces
        for (let i = 0; i < this.bodies.length; i++) {
            const body = this.bodies[i];
            if (SETTINGS.applyGravity) {
                // Apply the weight force to all bodies
                const weightForce = Force.generateWeightForce(body, this.G);
                body.addForce(weightForce);
            }

            // Apply forces to all bodies
            for (const force of this.forces) {
                body.addForce(force);
            }

            // Apply torque to all bodiesx
            for (const torque of this.torques) {
                body.addTorque(torque);
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
        for (let i = 0; i < this.bodies.length; i++) {
            this.bodies[i].integrateForces(dt);
        }

        this.ccd(dt);

        for (let i = 0; i < this.dtFractions.length; i++) {
            const fraction = this.dtFractions[i];
            const invDt = 1 / fraction;

            this.broadPhase();

            this.narrowPhase();

            this.solveConstraints(invDt);

            // Integrate all the velocities
            for (let i = 0; i < this.bodies.length; i++) {
                this.bodies[i].integrateVelocities(fraction);
            }
        }
    }

    private ccd(dt: number) {
        const fractions = this.dtFractions;
        fractions.length = 0;

        for (let i = 0; i < this.bodies.length; i++) {
            const body = this.bodies[i];

            if (!body.isBullet) continue;

            if (body.velocity.magnitudeSquared() <= MIN_BULLET_SPEED) {
                body.isBullet = false;
                continue;
            }

            const fraction = CCD.resolveCCD(body, this.bodies, dt);

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

    private broadPhase() {
        this.bodies.sort((a, b) => a.minX - b.minX);
        this.potentialPairs.length = 0;

        // Broad phase check with prune & sweep algorithm
        // TODO: some collisions are not correclty found, try to set gravity to 0
        for (let i = 0; i < this.bodies.length; i++) {
            const a = this.bodies[i];

            for (let j = i + 1; j < this.bodies.length; j++) {
                const b = this.bodies[j];

                // If objects don't overlap on X axis they cannot collide
                if (b.minX > a.maxX) break;

                // If objects overlap on X axis but don't overlap on Y axis the cannot collide
                if (a.maxY < b.minY || a.minY > b.maxY) {
                    continue;
                }

                // Objects may be colliding
                this.potentialPairs.push([a, b]);
            }
        }
    }

    private narrowPhase() {
        const newManifolds: ContactManifold[] = [];
        const newManifoldMap: Map<number, ContactManifold> = new Map();

        // Narrow phase check, potential pairs may still not collide
        for (let [a, b] of this.potentialPairs) {
            // TODO: verify if it is appropriate that bullets cannot collide with each other
            // if (a.isStatic() && b.isStatic()) continue;
            if ((a.isStatic() && b.isStatic()) || (a.isBullet && b.isBullet)) continue;

            // Improve coherence
            if (a.id > b.id) {
                [a, b] = [b, a];
            }

            const newManifold = NarrowPhase.detectCollision(a, b);
            if (newManifold == null) continue;

            const key = Utils.pairKey(a, b);
            if (SETTINGS.warmStarting && this.manifoldMap.has(key)) {
                const oldManifold = this.manifoldMap.get(key)!;
                newManifold.tryWarmStart(oldManifold);
            }

            newManifoldMap.set(key, newManifold);
            newManifolds.push(newManifold);

            this.setGrounded(newManifold);
        }

        this.manifoldMap = newManifoldMap;
        this.manifolds = newManifolds;
    }

    private solveConstraints(invDt: number) {
        // Presolve constraints
        for (let i = 0; i < this.manifolds.length; i++) this.manifolds[i].preSolve(invDt);

        for (let i = 0; i < this.joints.length; i++) this.joints[i].preSolve(invDt);

        // Solve constraints
        for (let i = 0; i < SETTINGS.solverIterations; i++) {
            for (let j = 0; j < this.manifolds.length; j++) this.manifolds[j].solve();

            for (let j = 0; j < this.joints.length; j++) this.joints[j].solve();
        }
    }

    private setGrounded(manifold: ContactManifold) {
        const bodyA = manifold.bodyA;
        const bodyB = manifold.bodyB;
        const contactNormal = manifold.contactNormal;

        const dotA = contactNormal.negateNew().dot(this.up);
        const dotB = contactNormal.dot(this.up);

        if (!bodyA.isStatic() && dotA > 0.5) bodyA.isGrounded = true;
        if (!bodyB.isStatic() && dotB > 0.5) bodyB.isGrounded = true;
    }

    clear() {
        this.bodies.length = 0;
        this.manifolds.length = 0;
        this.joints.length = 0;
        this.manifoldMap.clear();
        this.forces.length = 0;
        this.torques.length = 0;
    }
}
