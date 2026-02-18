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
import { BODY_REMOVAL_THRESHOLD, DELTA_TIME, INVERSE_DELTA_TIME, SETTINGS } from './Constants';
import { ContactManifold } from './Contact';
import Force from './Force';
import { Joint } from './Joint';
import RigidBody from './RigidBody';

export default class World {
    private G: number;
    private iterations: number;

    private bodies: RigidBody[] = [];

    public manifolds: ContactManifold[] = [];
    public joints: Joint[] = [];

    public manifoldMap: Map<number, ContactManifold> = new Map();

    private forces: Vec2[] = [];
    private torques: number[] = [];

    constructor(gravity: number, iterations = 10) {
        this.G = -gravity;
        this.iterations = iterations;
    }

    addBody = (body: RigidBody): void => {
        this.bodies.push(body);
    };

    removeBody = (body: RigidBody): void => {
        for (let i = 0; i < this.bodies.length; i++) {
            const current = this.bodies[i];

            // It suffices to look for the position going below the screen
            if (body.id === current.id) {
                this.bodies[i] = this.bodies[this.bodies.length - 1];
                this.bodies.pop();
                return;
            }
        }
    };

    getBodies = (): RigidBody[] => {
        return this.bodies;
    };

    getManifolds = (): ContactManifold[] => {
        return this.manifolds;
    };

    addJoint = (constraint: Joint): void => {
        this.joints.push(constraint);
    };

    getJoints = (): Joint[] => {
        return this.joints;
    };

    addForce = (force: Vec2): void => {
        this.forces.push(force);
    };

    addTorque = (torque: number): void => {
        this.torques.push(torque);
    };

    update = (): void => {
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

            // Apply torque to all bodies
            for (const torque of this.torques) {
                body.addTorque(torque);
            }
        }

        // Integrate all the forces
        for (const body of this.bodies) {
            body.integrateForces(DELTA_TIME);
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
        for (let i = 0; i < this.manifolds.length; i++) this.manifolds[i].preSolve(INVERSE_DELTA_TIME);

        for (let i = 0; i < this.joints.length; i++) this.joints[i].preSolve(INVERSE_DELTA_TIME);

        // Solve constraints
        for (let i = 0; i < this.iterations; i++) {
            for (let j = 0; j < this.manifolds.length; j++) this.manifolds[j].solve();

            for (let j = 0; j < this.joints.length; j++) this.joints[j].solve();
        }

        // Integrate all the velocities
        for (const body of this.bodies) {
            body.integrateVelocities(DELTA_TIME);
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
    };

    clear = () => {
        this.bodies.length = 0;
        this.joints.length = 0;
        this.forces.length = 0;
        this.torques.length = 0;
    };

    private setGrounded(bodyA: RigidBody, bodyB: RigidBody, contactNormal: Vec2): boolean {
        const up = new Vec2(0, 1);

        if (contactNormal.negateNew().dot(up) > 0) {
            bodyA.isGrounded = true;
        } else {
            bodyA.isGrounded = false;
        }

        if (contactNormal.dot(up) > 0) {
            bodyB.isGrounded = true;
        } else {
            bodyB.isGrounded = false;
        }
        
        return false;
    }
}
