/*
 * Portions of this file are derived from the Sopiro Physics Engine.
 *
 * Copyright (c) 2022 Sopiro
 * Licensed under the MIT License
 *
 * Original project:
 * https://github.com/Sopiro
 */
import * as Collision from '../collision/Collision';
import { ContactManifold } from '../collision/ContactManifold';
import { Joint } from '../joint/Joint';
import Vec2 from '../math/Vec2';
import Force from '../physics/Force';
import { CircleShape } from '../shapes/CircleShape';
import { edgeIntersection } from '../shapes/Edge';
import { PolygonShape } from '../shapes/PolygonShape';
import { ShapeType } from '../shapes/Shape';
import * as Utils from '../utils/Utils';
import { BODY_REMOVAL_THRESHOLD, MIN_BULLET_SPEED, SETTINGS } from './Constants';
import RigidBody from './RigidBody';

export default class World {
    private readonly up = new Vec2(0, 1);
    private G: number;

    private bodies: RigidBody[] = [];

    public manifolds: ContactManifold[] = [];
    public joints: Joint[] = [];

    public potentialPairs: [RigidBody, RigidBody][] = [];
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
            body.hasCCD = false;
        }

        // Integrate all the forces
        for (const body of this.bodies) {
            body.integrateForces(dt);
        }

        this.ccd(dt);

        this.broadPhase();

        this.narrowPhase();

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

    ccd(dt: number) {
        for (const body of this.bodies) {
            if (body.isBullet && body.velocity.magnitudeSquared() > MIN_BULLET_SPEED) {
                const bulletShape = body.shape as CircleShape;
                const currentPos = body.position.copy();
                const nextPos = currentPos.addNew(body.velocity.scaleNew(dt));

                // TODO: We could cast two rays instead of one or check intersection by shifting up and down by radius
                for (const other of this.bodies) {
                    if (other.isStatic()) {
                        if (other.shapeType === ShapeType.BOX || other.shapeType === ShapeType.POLYGON) {
                            const polygonShape = other.shape as PolygonShape;
                            const vertices = polygonShape.worldVertices;

                            let minDistanceSquared = Infinity;
                            let closestIntersection: Vec2 | undefined;
                            let hitEdge: [Vec2, Vec2] | undefined;

                            for (let i = 0; i < vertices.length; i++) {
                                const v0 = vertices[i];
                                const v1 = vertices[(i + 1) % vertices.length];

                                const intersection = edgeIntersection(currentPos, nextPos, v0, v1);

                                if (intersection) {
                                    const distanceSquared = intersection.subNew(currentPos).magnitudeSquared();

                                    if (distanceSquared < minDistanceSquared) {
                                        closestIntersection = intersection.copy();
                                        minDistanceSquared = distanceSquared;
                                        hitEdge = [v0, v1];
                                    }
                                }
                            }

                            if (closestIntersection && hitEdge) {
                                const [v0, v1] = hitEdge;
                                const edgeVector = v0.subNew(v1);
                                const edgeNormal = edgeVector.perpNew().unitVector();

                                // Make sure normal points away from the bullet's current position
                                const toBullet = currentPos.subNew(closestIntersection).unitVector();
                                if (edgeNormal.dot(toBullet) < 0) {
                                    edgeNormal.negate(); // flip to point outward
                                }

                                const bulletNewPos = closestIntersection.addNew(
                                    edgeNormal.scaleNew(bulletShape.radius),
                                );
                                body.position = bulletNewPos.copy();
                                body.shape.updateAABB(body);
                                body.hasCCD = true;
                            }
                        }
                    }
                }
            }
        }
    }

    broadPhase() {
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

            this.setGrounded(a, b, newManifold.contactNormal);
        }

        this.manifoldMap = newManifoldMap;
        this.manifolds = newManifolds;
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
        this.manifoldMap.clear();
        this.forces.length = 0;
        this.torques.length = 0;
    }
}
