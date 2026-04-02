/*
 * Portions of this file are derived from Box2D and Phaser Box2D.
 *
 * Copyright (c) 2023 Erin Catto
 * Copyright (c) 2024 Phaser Studio Inc
 * Licensed under the MIT License
 */
import { SETTINGS } from '../core/Constants';
import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { Joint } from './Joint';

const AXIS_EPSILON = 1.0e-10;

export class DistanceJoint extends Joint {
    public localAnchorA: Vec2;
    public localAnchorB: Vec2;

    private readonly length: number;

    private anchorA = new Vec2();
    private anchorB = new Vec2();
    private axis = new Vec2(0, 1);
    private axialMass = 0;
    private currentLength = 0;
    private inverseDeltaTime = 0;
    private impulse = 0;

    constructor(
        bodyA: RigidBody,
        bodyB: RigidBody,
        anchorA: Vec2 = bodyA.position,
        anchorB: Vec2 = bodyB.position,
        length: number = -1,
        frequency = 15,
        dampingRatio = 1.0,
        jointMass = -1,
    ) {
        super(bodyA, bodyB, frequency, dampingRatio, jointMass);

        this.localAnchorA = this.bodyA.worldPointToLocal(anchorA);
        this.localAnchorB = this.bodyB.worldPointToLocal(anchorB);
        this.length = length <= 0 ? anchorB.subNew(anchorA).magnitude() : Math.max(0, length);
    }

    override preSolve(inverseDeltaTime: number): void {
        this.inverseDeltaTime = Math.abs(inverseDeltaTime);
        this.updateSoftness(this.inverseDeltaTime);

        const worldAnchorA = this.bodyA.localPointToWorld(this.localAnchorA);
        const worldAnchorB = this.bodyB.localPointToWorld(this.localAnchorB);

        this.anchorA = worldAnchorA.subNew(this.bodyA.position);
        this.anchorB = worldAnchorB.subNew(this.bodyB.position);

        const separation = worldAnchorB.subNew(worldAnchorA);
        this.currentLength = separation.magnitude();

        if (this.currentLength > AXIS_EPSILON) {
            this.axis = separation.divNew(this.currentLength);
        }

        const crA = this.anchorA.cross(this.axis);
        const crB = this.anchorB.cross(this.axis);
        const k = this.bodyA.invMass + this.bodyB.invMass + this.bodyA.invI * crA * crA + this.bodyB.invI * crB * crB;

        this.axialMass = k > 0.0 ? 1.0 / k : 0.0;

        if (!SETTINGS.warmStarting) {
            this.impulse = 0.0;
            return;
        }

        this.applyImpulse(this.impulse);
    }

    override solve(): void {
        const relativeVelocity =
            this.axis.x *
                (this.bodyB.velocity.x -
                    this.bodyA.velocity.x -
                    this.bodyB.angularVelocity * this.anchorB.y +
                    this.bodyA.angularVelocity * this.anchorA.y) +
            this.axis.y *
                (this.bodyB.velocity.y -
                    this.bodyA.velocity.y +
                    this.bodyB.angularVelocity * this.anchorB.x -
                    this.bodyA.angularVelocity * this.anchorA.x);

        const error = this.currentLength - this.length;
        let impulse: number;

        if (this.isSpringEnabled()) {
            impulse =
                -this.softness.massScale * this.axialMass * (relativeVelocity + this.softness.biasRate * error) -
                this.softness.impulseScale * this.impulse;
            this.impulse += impulse;
        } else {
            const bias =
                SETTINGS.positionCorrection && this.inverseDeltaTime > 0
                    ? SETTINGS.positionCorrectionBeta * this.inverseDeltaTime * error
                    : 0.0;

            impulse = -this.axialMass * (relativeVelocity + bias);
            this.impulse += impulse;
        }

        this.applyImpulse(impulse);
    }

    private applyImpulse(impulse: number): void {
        const impulseX = impulse * this.axis.x;
        const impulseY = impulse * this.axis.y;

        this.bodyA.velocity.x -= this.bodyA.invMass * impulseX;
        this.bodyA.velocity.y -= this.bodyA.invMass * impulseY;
        this.bodyA.angularVelocity -= this.bodyA.invI * (this.anchorA.x * impulseY - this.anchorA.y * impulseX);

        this.bodyB.velocity.x += this.bodyB.invMass * impulseX;
        this.bodyB.velocity.y += this.bodyB.invMass * impulseY;
        this.bodyB.angularVelocity += this.bodyB.invI * (this.anchorB.x * impulseY - this.anchorB.y * impulseX);
    }
}
