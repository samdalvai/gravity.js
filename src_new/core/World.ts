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
import * as Collision from '../collision/Collision';
import { ContactManifold } from '../collision/ContactManifold';
import Force from '../force/Force';
import { Joint } from '../joint/Joint';
import { Vec2 } from '../math/Vec2';
import * as Utils from '../utils/Utils';
import { MIN_BULLET_SPEED, SETTINGS } from './Constants';
import { RigidBody } from './RigidBody';

export interface WorldStats {
    numBodies: number;
    numPotentialPairs: number;
    numManifolds: number;
    numContacts: number;
    numJoints: number;
}

interface ConstraintIsland {
    bodies: RigidBody[];
    manifolds: ContactManifold[];
    joints: Joint[];
}

export class World {
    private readonly up = new Vec2(0, 1);
    private G: number;
    private _blackHole: RigidBody | null = null;

    private bodies: RigidBody[] = [];

    public manifolds: ContactManifold[] = [];
    public joints: Joint[] = [];

    public potentialPairs: [RigidBody, RigidBody][] = [];
    public manifoldMap: Map<number, ContactManifold> = new Map();
    private constraintIslands: ConstraintIsland[] = [];
    private bodyIslandMap: Map<number, number> = new Map();

    private forces: Vec2[] = [];
    private torques: number[] = [];

    constructor(gravity: number) {
        this.G = -gravity;
    }

    get blackHole(): RigidBody | null {
        return this._blackHole;
    }

    set blackHole(body: RigidBody | null) {
        this._blackHole = body;
    }

    addBody(body: RigidBody): void {
        this.bodies.push(body);
    }

    removeBody(body: RigidBody): void {
        for (let i = 0; i < this.bodies.length; i++) {
            const current = this.bodies[i];

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

    getStats(): WorldStats {
        let numContacts = 0;

        for (let i = 0; i < this.manifolds.length; i++) {
            numContacts += this.manifolds[i].numContacts;
        }

        return {
            numBodies: this.bodies.length,
            numPotentialPairs: this.potentialPairs.length,
            numManifolds: this.manifolds.length,
            numContacts,
            numJoints: this.joints.length,
        };
    }

    getNumConstraintIslands(): number {
        return this.constraintIslands.length;
    }

    getNumAwakeConstraintIslands(): number {
        let awakeIslands = 0;

        for (let i = 0; i < this.constraintIslands.length; i++) {
            const island = this.constraintIslands[i];

            for (let j = 0; j < island.bodies.length; j++) {
                if (!island.bodies[j].isSleeping) {
                    awakeIslands++;
                    break;
                }
            }
        }

        return awakeIslands;
    }

    getBodyIslandId(body: RigidBody): number | null {
        const islandId = this.bodyIslandMap.get(body.id);
        return islandId === undefined ? null : islandId;
    }

    addForce(force: Vec2): void {
        this.forces.push(force);
    }

    addTorque(torque: number): void {
        this.torques.push(torque);
    }

    step(dt: number): void {
        const invDt = 1 / dt;

        // Loop all bodies of the world applying forces
        for (const body of this.bodies) {
            if (SETTINGS.applyGravity) {
                // Apply the weight force to all bodies
                const weightForce = Force.generateWeightForce(body, this.G);
                body.addForce(weightForce, false);
            }

            if (this.blackHole) {
                const blackHole = this.blackHole;

                const attraction = Force.generateGravitationalForce(body, blackHole, -this.G, 1, 200);
                body.addForce(attraction, false);
            }

            // Apply forces to all bodies
            for (const force of this.forces) {
                body.addForce(force, false);
            }

            // Apply torque to all bodiesx
            for (const torque of this.torques) {
                body.addTorque(torque, false);
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
            body.hasCCD = false;
        }

        // Integrate all the forces
        for (const body of this.bodies) {
            body.integrateForces(dt);
        }

        this.ccd(dt);

        this.broadPhase();

        this.narrowPhase();

        this.buildConstraintIslands();

        // Presolve constraints
        for (let i = 0; i < this.constraintIslands.length; i++) {
            const island = this.constraintIslands[i];
            if (!this.prepareIslandForSolving(island)) continue;

            for (let j = 0; j < island.manifolds.length; j++) island.manifolds[j].preSolve(invDt);
            for (let j = 0; j < island.joints.length; j++) island.joints[j].preSolve(invDt);
        }

        // Solve constraints
        for (let i = 0; i < SETTINGS.solverIterations; i++) {
            for (let j = 0; j < this.constraintIslands.length; j++) {
                const island = this.constraintIslands[j];
                if (this.isIslandSleeping(island)) continue;

                for (let k = 0; k < island.manifolds.length; k++) island.manifolds[k].solve();
                for (let k = 0; k < island.joints.length; k++) island.joints[k].solve();
            }
        }

        // Apply restitution
        for (let i = 0; i < SETTINGS.solverIterations; i++) {
            for (let j = 0; j < this.constraintIslands.length; j++) {
                const island = this.constraintIslands[j];
                if (this.isIslandSleeping(island)) continue;

                for (let k = 0; k < island.manifolds.length; k++) island.manifolds[k].applyRestitution();
            }
        }

        this.updateIslandSleepStates(dt);

        // Integrate all the velocities
        for (const body of this.bodies) {
            body.integrateVelocities(dt);
        }
    }

    update(dt: number): void {
        this.step(dt);
    }

    ccd(dt: number) {
        for (const body of this.bodies) {
            if (body.isBullet) {
                if (body.velocity.magnitudeSquared() > MIN_BULLET_SPEED) {
                    CCD.resolveCCD(body, this.bodies, dt);
                } else {
                    // If a bullet stopped moving fast enugh downgrade to normal dynamic body
                    body.isBullet = false;
                }
            }
        }
    }

    private sortBodiesByMinX(): void {
        const bodies = this.bodies;

        // Broad phase updates tend to keep this array nearly sorted frame-to-frame.
        for (let i = 1; i < bodies.length; ++i) {
            const body = bodies[i];
            const minX = body.minX;
            let j = i - 1;

            while (j >= 0 && bodies[j].minX > minX) {
                bodies[j + 1] = bodies[j];
                --j;
            }

            bodies[j + 1] = body;
        }
    }

    broadPhase() {
        this.sortBodiesByMinX();
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

    narrowPhase() {
        const newManifolds: ContactManifold[] = [];
        const newManifoldMap: Map<number, ContactManifold> = new Map();

        // Narrow phase check, potential pairs may still not collide
        for (let [a, b] of this.potentialPairs) {
            if (a.isStatic() && b.isStatic()) continue;

            // Improve coherence
            if (a.id > b.id) {
                [a, b] = [b, a];
            }

            const newManifold = Collision.detectCollision(a, b);
            if (newManifold == null) continue;

            const key = Utils.pairKey(a, b);
            if (SETTINGS.warmStarting && this.manifoldMap.has(key)) {
                const oldManifold = this.manifoldMap.get(key)!;
                newManifold.tryWarmStart(oldManifold);
            }

            newManifoldMap.set(key, newManifold);
            newManifolds.push(newManifold);

            if (newManifold.hasTouchingContact()) {
                this.setGrounded(a, b, newManifold.contactNormal);
            }
        }

        this.manifoldMap = newManifoldMap;
        this.manifolds = newManifolds;
    }

    buildConstraintIslands() {
        this.constraintIslands.length = 0;
        this.bodyIslandMap.clear();

        if (this.manifolds.length === 0 && this.joints.length === 0) {
            return;
        }

        const bodyIndexById = new Map<number, number>();
        const manifoldsByBody: number[][] = Array.from({ length: this.bodies.length }, () => []);
        const jointsByBody: number[][] = Array.from({ length: this.bodies.length }, () => []);

        for (let i = 0; i < this.bodies.length; i++) {
            bodyIndexById.set(this.bodies[i].id, i);
        }

        for (let i = 0; i < this.manifolds.length; i++) {
            const manifold = this.manifolds[i];
            const bodyAIndex = bodyIndexById.get(manifold.bodyA.id);
            const bodyBIndex = bodyIndexById.get(manifold.bodyB.id);

            if (bodyAIndex !== undefined) manifoldsByBody[bodyAIndex].push(i);
            if (bodyBIndex !== undefined && bodyBIndex !== bodyAIndex) manifoldsByBody[bodyBIndex].push(i);
        }

        for (let i = 0; i < this.joints.length; i++) {
            const joint = this.joints[i];
            const bodyAIndex = bodyIndexById.get(joint.bodyA.id);
            const bodyBIndex = bodyIndexById.get(joint.bodyB.id);

            if (bodyAIndex !== undefined) jointsByBody[bodyAIndex].push(i);
            if (bodyBIndex !== undefined && bodyBIndex !== bodyAIndex) jointsByBody[bodyBIndex].push(i);
        }

        const visitedBodies = new Array(this.bodies.length).fill(false);
        const visitedManifolds = new Array(this.manifolds.length).fill(false);
        const visitedJoints = new Array(this.joints.length).fill(false);
        const stack: number[] = [];

        for (let i = 0; i < this.bodies.length; i++) {
            const body = this.bodies[i];

            if (visitedBodies[i] || body.isStatic()) {
                continue;
            }

            if (manifoldsByBody[i].length === 0 && jointsByBody[i].length === 0) {
                visitedBodies[i] = true;
                const islandId = this.constraintIslands.length;
                this.constraintIslands.push({
                    bodies: [body],
                    manifolds: [],
                    joints: [],
                });
                this.bodyIslandMap.set(body.id, islandId);
                continue;
            }

            const island: ConstraintIsland = {
                bodies: [],
                manifolds: [],
                joints: [],
            };

            visitedBodies[i] = true;
            stack.push(i);

            while (stack.length > 0) {
                const bodyIndex = stack.pop()!;
                const islandBody = this.bodies[bodyIndex];
                island.bodies.push(islandBody);
                this.bodyIslandMap.set(islandBody.id, this.constraintIslands.length);

                for (let j = 0; j < manifoldsByBody[bodyIndex].length; j++) {
                    const manifoldIndex = manifoldsByBody[bodyIndex][j];
                    if (visitedManifolds[manifoldIndex]) continue;

                    visitedManifolds[manifoldIndex] = true;
                    const manifold = this.manifolds[manifoldIndex];
                    island.manifolds.push(manifold);

                    const nextBodyAIndex = bodyIndexById.get(manifold.bodyA.id);
                    const nextBodyBIndex = bodyIndexById.get(manifold.bodyB.id);

                    if (
                        nextBodyAIndex !== undefined &&
                        !visitedBodies[nextBodyAIndex] &&
                        !this.bodies[nextBodyAIndex].isStatic()
                    ) {
                        visitedBodies[nextBodyAIndex] = true;
                        stack.push(nextBodyAIndex);
                    }

                    if (
                        nextBodyBIndex !== undefined &&
                        !visitedBodies[nextBodyBIndex] &&
                        !this.bodies[nextBodyBIndex].isStatic()
                    ) {
                        visitedBodies[nextBodyBIndex] = true;
                        stack.push(nextBodyBIndex);
                    }
                }

                for (let j = 0; j < jointsByBody[bodyIndex].length; j++) {
                    const jointIndex = jointsByBody[bodyIndex][j];
                    if (visitedJoints[jointIndex]) continue;

                    visitedJoints[jointIndex] = true;
                    const joint = this.joints[jointIndex];
                    island.joints.push(joint);

                    const nextBodyAIndex = bodyIndexById.get(joint.bodyA.id);
                    const nextBodyBIndex = bodyIndexById.get(joint.bodyB.id);

                    if (
                        nextBodyAIndex !== undefined &&
                        !visitedBodies[nextBodyAIndex] &&
                        !this.bodies[nextBodyAIndex].isStatic()
                    ) {
                        visitedBodies[nextBodyAIndex] = true;
                        stack.push(nextBodyAIndex);
                    }

                    if (
                        nextBodyBIndex !== undefined &&
                        !visitedBodies[nextBodyBIndex] &&
                        !this.bodies[nextBodyBIndex].isStatic()
                    ) {
                        visitedBodies[nextBodyBIndex] = true;
                        stack.push(nextBodyBIndex);
                    }
                }
            }

            this.constraintIslands.push(island);
        }

        // Constraints that involve only static bodies are not reachable from a dynamic-body seed.
        for (let i = 0; i < this.manifolds.length; i++) {
            if (!visitedManifolds[i]) {
                this.constraintIslands.push({
                    bodies: [],
                    manifolds: [this.manifolds[i]],
                    joints: [],
                });
            }
        }

        for (let i = 0; i < this.joints.length; i++) {
            if (!visitedJoints[i]) {
                this.constraintIslands.push({
                    bodies: [],
                    manifolds: [],
                    joints: [this.joints[i]],
                });
            }
        }
    }

    private isIslandSleeping(island: ConstraintIsland): boolean {
        if (!SETTINGS.sleepEnabled) {
            return false;
        }

        if (island.bodies.length === 0) {
            return false;
        }

        for (let i = 0; i < island.bodies.length; i++) {
            if (!island.bodies[i].isSleeping) {
                return false;
            }
        }

        return true;
    }

    private prepareIslandForSolving(island: ConstraintIsland): boolean {
        if (!SETTINGS.sleepEnabled || island.bodies.length === 0) {
            return true;
        }

        let hasAwakeBody = false;

        for (let i = 0; i < island.bodies.length; i++) {
            if (!island.bodies[i].isSleeping) {
                hasAwakeBody = true;
                break;
            }
        }

        if (!hasAwakeBody) {
            return false;
        }

        for (let i = 0; i < island.bodies.length; i++) {
            if (island.bodies[i].isSleeping) {
                island.bodies[i].wakeUp();
            }
        }

        return true;
    }

    private updateIslandSleepStates(dt: number): void {
        if (!SETTINGS.sleepEnabled) {
            return;
        }

        const linearSleepVelocitySquared = SETTINGS.sleepLinearVelocity * SETTINGS.sleepLinearVelocity;

        for (let i = 0; i < this.constraintIslands.length; i++) {
            const island = this.constraintIslands[i];
            if (island.bodies.length === 0) {
                continue;
            }

            let canSleep = true;

            for (let j = 0; j < island.bodies.length; j++) {
                const body = island.bodies[j];
                const linearVelocitySquared = body.velocity.magnitudeSquared();

                if (
                    linearVelocitySquared > linearSleepVelocitySquared ||
                    Math.abs(body.angularVelocity) > SETTINGS.sleepAngularVelocity ||
                    body.hasCCD
                ) {
                    canSleep = false;
                    break;
                }
            }

            if (!canSleep) {
                for (let j = 0; j < island.bodies.length; j++) {
                    island.bodies[j].sleepTime = 0;
                }
                continue;
            }

            let shouldSleepIsland = true;
            for (let j = 0; j < island.bodies.length; j++) {
                const body = island.bodies[j];
                body.sleepTime += dt;

                if (body.sleepTime < SETTINGS.sleepTimeThreshold) {
                    shouldSleepIsland = false;
                }
            }

            if (shouldSleepIsland) {
                for (let j = 0; j < island.bodies.length; j++) {
                    island.bodies[j].sleep();
                }
            }
        }
    }

    setGrounded(bodyA: RigidBody, bodyB: RigidBody, contactNormal: Vec2) {
        const dotA = contactNormal.negateNew().dot(this.up);
        const dotB = contactNormal.dot(this.up);

        if (dotA > 0.5) bodyA.isGrounded = true;
        if (dotB > 0.5) bodyB.isGrounded = true;
    }

    clear() {
        this.bodies.length = 0;
        this.manifolds.length = 0;
        this.joints.length = 0;
        this.constraintIslands.length = 0;
        this.bodyIslandMap.clear();
        this.manifoldMap.clear();
        this.forces.length = 0;
        this.torques.length = 0;

        this._blackHole = null;
    }
}
