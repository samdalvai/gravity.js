/*
 * Portions of this file are derived from the Sopiro Physics Engine.
 *
 * Copyright (c) 2022 Sopiro
 * Licensed under the MIT License
 *
 * Original project:
 * https://github.com/Sopiro
 */
import Vec2 from '../math/Vec2';
import CollisionDetection from './CollisionDetection';
import { BODY_REMOVAL_THRESHOLD, SETTINGS } from './Constants';
import { ContactManifold } from './Contact';
import Force from './Force';
import { Joint } from './Joint';
import RigidBody from './RigidBody';

export default class World {
    private readonly up = new Vec2(0, 1);
    private G: number;

    private bodies: RigidBody[] = [];

    public manifolds: ContactManifold[] = [];
    public joints: Joint[] = [];

    public manifoldMap: Map<number, ContactManifold> = new Map();

    private forces: Vec2[] = [];
    private torques: number[] = [];

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
        const invDt = 1 / dt;
        const newManifolds: ContactManifold[] = [];
        const newManifoldMap: Map<number, ContactManifold> = new Map();

        // Loop all bodies of the world applying forces
        for (const body of this.bodies) {
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
        for (const body of this.bodies) {
            body.integrateForces(dt);
        }

        this.bodies.sort((a, b) => a.minX - b.minX);
        const potentialPairs: [RigidBody, RigidBody][] = [];

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
                potentialPairs.push([a, b]);
            }
        }

        // Narrow phase check, potential pairs may still not collide
        for (let [a, b] of potentialPairs) {
            if (a.isStatic() && b.isStatic()) continue;

            // Improve coherence
            if (a.id > b.id) {
                [a, b] = [b, a];
            }

            const newManifold = CollisionDetection.detectCollision(a, b);
            if (newManifold == null) continue;

            const key = RigidBody.pairKey(a, b);
            if (SETTINGS.warmStarting && this.manifoldMap.has(key)) {
                const oldManifold = this.manifoldMap.get(key)!;
                newManifold.tryWarmStart(oldManifold);
            }

            newManifoldMap.set(key, newManifold);
            newManifolds.push(newManifold);

            this.setGrounded(a, b, newManifold.contactNormal);
        }

        this.manifoldMap = newManifoldMap;
        this.manifolds = newManifolds;

        // Presolve constraints
        for (let i = 0; i < this.manifolds.length; i++) this.manifolds[i].preSolve(invDt);

        for (let i = 0; i < this.joints.length; i++) this.joints[i].preSolve(invDt);

        // Solve constraints
        for (let i = 0; i < SETTINGS.solverIterations; i++) {
            for (let j = 0; j < this.manifolds.length; j++) this.manifolds[j].solve();

            for (let j = 0; j < this.joints.length; j++) this.joints[j].solve();
        }

        // Integrate all the velocities
        for (const body of this.bodies) {
            body.integrateVelocities(dt);
        }

        // Kill objects that went out of the screen
        for (let i = 0; i < this.bodies.length; i++) {
            const body = this.bodies[i];

            // It suffices to look for the position going below the screen
            if (body.position.y < BODY_REMOVAL_THRESHOLD) {
                this.bodies[i] = this.bodies[this.bodies.length - 1];
                this.bodies.pop();
            }
        }
    }

    clear() {
        this.bodies.length = 0;
        this.joints.length = 0;
        this.forces.length = 0;
        this.torques.length = 0;
    }

    setGrounded(bodyA: RigidBody, bodyB: RigidBody, contactNormal: Vec2) {
        const dotA = contactNormal.negateNew().dot(this.up);
        const dotB = contactNormal.dot(this.up);

        if (dotA > 0.5) bodyA.isGrounded = true;
        if (dotB > 0.5) bodyB.isGrounded = true;
    }
}
