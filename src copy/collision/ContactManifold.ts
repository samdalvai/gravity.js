/*
 * Portions of this file are derived from Box2D and Phaser Box2D.
 *
 * Copyright (c) 2023 Erin Catto
 * Copyright (c) 2024 Phaser Studio Inc
 * Licensed under the MIT License
 */
import { Constraint } from '../constraint/Constraint';
import { Softness, makeSoftness } from '../constraint/Softness';
import { SETTINGS } from '../core/Constants';
import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';

export interface ContactPoint {
    point: Vec2;
    id: number;
}

export interface ContactPointData extends ContactPoint {
    anchorA: Vec2;
    anchorB: Vec2;
    separation: number;
    normalImpulse?: number;
    tangentImpulse?: number;
    maxNormalImpulse?: number;
    normalVelocity?: number;
    persisted?: boolean;
}

export interface CollisionManifoldData {
    normal: Vec2;
    points: ContactPointData[];
}

interface SolverContactPoint extends ContactPointData {
    normalImpulse: number;
    tangentImpulse: number;
    maxNormalImpulse: number;
    normalVelocity: number;
    normalMass: number;
    tangentMass: number;
    baseSeparation: number;
}

function mixFriction(frictionA: number, frictionB: number): number {
    return Math.sqrt(frictionA * frictionB);
}

function mixRestitution(restitutionA: number, restitutionB: number): number {
    return Math.max(restitutionA, restitutionB);
}

export class ContactManifold extends Constraint {
    public readonly contactNormal: Vec2;
    public readonly contactTangent: Vec2;
    public readonly contactPoints: ContactPoint[];
    public penetrationDepth: number;

    private readonly friction: number;
    private readonly restitution: number;
    private readonly solverPoints: SolverContactPoint[];

    private inverseDeltaTime = 0;
    private softness: Softness = {
        biasRate: 0,
        massScale: 1,
        impulseScale: 0,
    };

    constructor(bodyA: RigidBody, bodyB: RigidBody, contactNormal: Vec2, contactPoints: ContactPointData[]) {
        super(bodyA, bodyB);

        this.contactNormal = contactNormal.copy();
        this.contactTangent = new Vec2(this.contactNormal.y, -this.contactNormal.x);
        this.friction = mixFriction(bodyA.friction, bodyB.friction);
        this.restitution = mixRestitution(bodyA.restitution, bodyB.restitution);
        const solverPoints: SolverContactPoint[] = new Array(contactPoints.length);

        for (let i = 0; i < contactPoints.length; ++i) {
            const point = contactPoints[i];
            solverPoints[i] = {
                point: point.point.copy(),
                anchorA: point.anchorA.copy(),
                anchorB: point.anchorB.copy(),
                separation: point.separation,
                id: point.id,
                normalImpulse: point.normalImpulse ?? 0,
                tangentImpulse: point.tangentImpulse ?? 0,
                maxNormalImpulse: point.maxNormalImpulse ?? 0,
                normalVelocity: point.normalVelocity ?? 0,
                normalMass: 0,
                tangentMass: 0,
                baseSeparation: point.separation,
                persisted: point.persisted ?? false,
            };
        }

        this.solverPoints = solverPoints;

        const publicContactPoints: ContactPoint[] = new Array(solverPoints.length);

        for (let i = 0; i < solverPoints.length; ++i) {
            const point = solverPoints[i];
            publicContactPoints[i] = {
                point: point.point,
                id: point.id,
            };
        }

        this.contactPoints = publicContactPoints;
        this.penetrationDepth = this.computePenetrationDepth();
    }

    override preSolve(inverseDeltaTime: number): void {
        this.inverseDeltaTime = Math.abs(inverseDeltaTime);

        const dt = this.inverseDeltaTime > 0 ? 1 / this.inverseDeltaTime : 0;
        const targetHertz = Math.min(SETTINGS.contactHertz, this.inverseDeltaTime * 0.25);
        const hertz = this.hasStaticBody() ? targetHertz * 2 : targetHertz;
        this.softness = makeSoftness(hertz, SETTINGS.contactDampingRatio, dt);
        this.penetrationDepth = this.computePenetrationDepth();

        for (const point of this.solverPoints) {
            point.baseSeparation = point.separation;
            point.maxNormalImpulse = 0;

            const rnA = point.anchorA.cross(this.contactNormal);
            const rnB = point.anchorB.cross(this.contactNormal);
            const kNormal =
                this.bodyA.invMass + this.bodyB.invMass + this.bodyA.invI * rnA * rnA + this.bodyB.invI * rnB * rnB;
            point.normalMass = kNormal > 0 ? 1 / kNormal : 0;

            const rtA = point.anchorA.cross(this.contactTangent);
            const rtB = point.anchorB.cross(this.contactTangent);
            const kTangent =
                this.bodyA.invMass + this.bodyB.invMass + this.bodyA.invI * rtA * rtA + this.bodyB.invI * rtB * rtB;
            point.tangentMass = kTangent > 0 ? 1 / kTangent : 0;

            const vrAX = this.bodyA.velocity.x - this.bodyA.angularVelocity * point.anchorA.y;
            const vrAY = this.bodyA.velocity.y + this.bodyA.angularVelocity * point.anchorA.x;
            const vrBX = this.bodyB.velocity.x - this.bodyB.angularVelocity * point.anchorB.y;
            const vrBY = this.bodyB.velocity.y + this.bodyB.angularVelocity * point.anchorB.x;

            point.normalVelocity = this.contactNormal.x * (vrBX - vrAX) + this.contactNormal.y * (vrBY - vrAY);

            if (!SETTINGS.warmStarting) {
                point.normalImpulse = 0;
                point.tangentImpulse = 0;
            }
        }

        if (SETTINGS.warmStarting) {
            for (const point of this.solverPoints) {
                const impulseX =
                    point.normalImpulse * this.contactNormal.x + point.tangentImpulse * this.contactTangent.x;
                const impulseY =
                    point.normalImpulse * this.contactNormal.y + point.tangentImpulse * this.contactTangent.y;

                this.bodyA.velocity.x -= this.bodyA.invMass * impulseX;
                this.bodyA.velocity.y -= this.bodyA.invMass * impulseY;
                this.bodyA.angularVelocity -=
                    this.bodyA.invI * (point.anchorA.x * impulseY - point.anchorA.y * impulseX);

                this.bodyB.velocity.x += this.bodyB.invMass * impulseX;
                this.bodyB.velocity.y += this.bodyB.invMass * impulseY;
                this.bodyB.angularVelocity +=
                    this.bodyB.invI * (point.anchorB.x * impulseY - point.anchorB.y * impulseX);
            }
        }
    }

    override solve(): void {
        let vAX = this.bodyA.velocity.x;
        let vAY = this.bodyA.velocity.y;
        let wA = this.bodyA.angularVelocity;
        let vBX = this.bodyB.velocity.x;
        let vBY = this.bodyB.velocity.y;
        let wB = this.bodyB.angularVelocity;

        const normalX = this.contactNormal.x;
        const normalY = this.contactNormal.y;
        const tangentX = this.contactTangent.x;
        const tangentY = this.contactTangent.y;

        for (const point of this.solverPoints) {
            let velocityBias = 0.0;
            let massScale = 1.0;
            let impulseScale = 0.0;

            if (point.baseSeparation > 0.0) {
                velocityBias = point.baseSeparation * this.inverseDeltaTime;
            } else {
                velocityBias = Math.max(
                    this.softness.biasRate * point.baseSeparation,
                    -SETTINGS.contactPushoutVelocity,
                );
                massScale = this.softness.massScale;
                impulseScale = this.softness.impulseScale;
            }

            const vn =
                (vBX - vAX + wB * -point.anchorB.y - wA * -point.anchorA.y) * normalX +
                (vBY - vAY + wB * point.anchorB.x - wA * point.anchorA.x) * normalY;

            let impulse = -point.normalMass * massScale * (vn + velocityBias) - impulseScale * point.normalImpulse;
            const newImpulse = Math.max(point.normalImpulse + impulse, 0.0);
            impulse = newImpulse - point.normalImpulse;
            point.normalImpulse = newImpulse;
            point.maxNormalImpulse = Math.max(point.maxNormalImpulse, impulse);

            const impulseX = impulse * normalX;
            const impulseY = impulse * normalY;
            vAX -= this.bodyA.invMass * impulseX;
            vAY -= this.bodyA.invMass * impulseY;
            wA -= this.bodyA.invI * (point.anchorA.x * impulseY - point.anchorA.y * impulseX);
            vBX += this.bodyB.invMass * impulseX;
            vBY += this.bodyB.invMass * impulseY;
            wB += this.bodyB.invI * (point.anchorB.x * impulseY - point.anchorB.y * impulseX);
        }

        for (const point of this.solverPoints) {
            const vtX = vBX - wB * point.anchorB.y - (vAX - wA * point.anchorA.y);
            const vtY = vBY + wB * point.anchorB.x - (vAY + wA * point.anchorA.x);
            const vt = vtX * tangentX + vtY * tangentY;

            let impulse = -point.tangentMass * vt;
            const maxFriction = this.friction * point.normalImpulse;
            const oldImpulse = point.tangentImpulse;
            point.tangentImpulse = Math.max(-maxFriction, Math.min(oldImpulse + impulse, maxFriction));
            impulse = point.tangentImpulse - oldImpulse;

            const impulseX = impulse * tangentX;
            const impulseY = impulse * tangentY;
            vAX -= this.bodyA.invMass * impulseX;
            vAY -= this.bodyA.invMass * impulseY;
            wA -= this.bodyA.invI * (point.anchorA.x * impulseY - point.anchorA.y * impulseX);
            vBX += this.bodyB.invMass * impulseX;
            vBY += this.bodyB.invMass * impulseY;
            wB += this.bodyB.invI * (point.anchorB.x * impulseY - point.anchorB.y * impulseX);
        }

        this.bodyA.velocity.x = vAX;
        this.bodyA.velocity.y = vAY;
        this.bodyA.angularVelocity = wA;
        this.bodyB.velocity.x = vBX;
        this.bodyB.velocity.y = vBY;
        this.bodyB.angularVelocity = wB;
    }

    applyRestitution(): void {
        if (this.restitution === 0.0) {
            return;
        }

        let vAX = this.bodyA.velocity.x;
        let vAY = this.bodyA.velocity.y;
        let wA = this.bodyA.angularVelocity;
        let vBX = this.bodyB.velocity.x;
        let vBY = this.bodyB.velocity.y;
        let wB = this.bodyB.angularVelocity;

        const normalX = this.contactNormal.x;
        const normalY = this.contactNormal.y;

        for (const point of this.solverPoints) {
            if (point.normalVelocity > -SETTINGS.restitutionThreshold || point.maxNormalImpulse === 0.0) {
                continue;
            }

            const vrAX = vAX - wA * point.anchorA.y;
            const vrAY = vAY + wA * point.anchorA.x;
            const vrBX = vBX - wB * point.anchorB.y;
            const vrBY = vBY + wB * point.anchorB.x;
            const vn = normalX * (vrBX - vrAX) + normalY * (vrBY - vrAY);

            let impulse = -point.normalMass * (vn + this.restitution * point.normalVelocity);
            const newImpulse = Math.max(point.normalImpulse + impulse, 0.0);
            impulse = newImpulse - point.normalImpulse;
            point.normalImpulse = newImpulse;
            point.maxNormalImpulse = Math.max(point.maxNormalImpulse, impulse);

            const impulseX = impulse * normalX;
            const impulseY = impulse * normalY;
            vAX -= this.bodyA.invMass * impulseX;
            vAY -= this.bodyA.invMass * impulseY;
            wA -= this.bodyA.invI * (point.anchorA.x * impulseY - point.anchorA.y * impulseX);
            vBX += this.bodyB.invMass * impulseX;
            vBY += this.bodyB.invMass * impulseY;
            wB += this.bodyB.invI * (point.anchorB.x * impulseY - point.anchorB.y * impulseX);
        }

        this.bodyA.velocity.x = vAX;
        this.bodyA.velocity.y = vAY;
        this.bodyA.angularVelocity = wA;
        this.bodyB.velocity.x = vBX;
        this.bodyB.velocity.y = vBY;
        this.bodyB.angularVelocity = wB;
    }

    tryWarmStart(oldManifold: ContactManifold) {
        for (const point of this.solverPoints) {
            let oldPoint: SolverContactPoint | null = null;

            for (let i = 0; i < oldManifold.solverPoints.length; ++i) {
                const candidate = oldManifold.solverPoints[i];
                if (candidate.id === point.id) {
                    oldPoint = candidate;
                    break;
                }
            }

            if (oldPoint === null) {
                continue;
            }

            point.normalImpulse = oldPoint.normalImpulse;
            point.tangentImpulse = oldPoint.tangentImpulse;
            point.persisted = true;
        }
    }

    get numContacts() {
        return this.contactPoints.length;
    }

    hasTouchingContact(maxSeparation = 0.5): boolean {
        for (let i = 0; i < this.solverPoints.length; ++i) {
            if (this.solverPoints[i].separation <= maxSeparation) {
                return true;
            }
        }

        return false;
    }

    private computePenetrationDepth(): number {
        let depth = 0;

        for (const point of this.solverPoints) {
            depth = Math.max(depth, -point.separation);
        }

        return depth;
    }

    private hasStaticBody(): boolean {
        return this.bodyA.isStatic() || this.bodyB.isStatic();
    }
}
