/*
 * Portions of this file are derived from the Sopiro Physics Engine.
 *
 * Copyright (c) 2022 Sopiro
 * Licensed under the MIT License
 *
 * Original project:
 * https://github.com/Sopiro
 */
import { SETTINGS } from '../core/Constants';
import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { Joint } from './Joint';

export class DistanceJoint extends Joint {
    public localAnchorA: Vec2;
    public localAnchorB: Vec2;

    private length: number;

    private ra!: Vec2;
    private rb!: Vec2;
    private m!: number;
    private n!: Vec2;
    private bias!: number;
    private impulseSum: number = 0.0;

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
        this.length = length <= 0 ? anchorB.subNew(anchorA).magnitude() : length;
    }

    override preSolve(inverseDeltaTime: number): void {
        // Calculate Jacobian J and effective mass M
        const pa = this.bodyA.localPointToWorld(this.localAnchorA);
        const pb = this.bodyB.localPointToWorld(this.localAnchorB);
        this.ra = pa.subNew(this.bodyA.position);
        this.rb = pb.subNew(this.bodyB.position);

        const u = pb.subNew(pa);

        this.n = u.normalizeNew();

        const k =
            this.bodyA.invMass +
            this.bodyB.invMass +
            this.bodyA.invI * this.n.cross(this.ra) * this.n.cross(this.ra) +
            this.bodyB.invI * this.n.cross(this.rb) * this.n.cross(this.rb) +
            this.gamma;

        this.m = 1.0 / k;

        const error = u.magnitude() - this.length;

        if (SETTINGS.positionCorrection) {
            this.bias = error * this.beta * inverseDeltaTime;
        } else {
            this.bias = 0.0;
        }

        if (SETTINGS.warmStarting) this.applyImpulse(this.impulseSum);
    }

    override solve(): void {
        // Calculate corrective impulse: Pc

        const jv = this.bodyB.velocity
            .addNew(this.rb.crossScalar(this.bodyB.angularVelocity))
            .subNew(this.bodyA.velocity.addNew(this.ra.crossScalar(this.bodyA.angularVelocity)))
            .dot(this.n);

        // Check out below for the reason why the (accumulated impulse * gamma) term is on the right hand side
        // https://pybullet.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
        const lambda = this.m * -(jv + this.bias + this.impulseSum * this.gamma);

        this.applyImpulse(lambda);

        if (SETTINGS.warmStarting) this.impulseSum += lambda;
    }

    private applyImpulse(lambda: number): void {
        this.bodyA.velocity = this.bodyA.velocity.subNew(this.n.scaleNew(lambda * this.bodyA.invMass));
        this.bodyA.angularVelocity =
            this.bodyA.angularVelocity - this.n.dot(this.ra.crossScalar(lambda)) * this.bodyA.invI;

        this.bodyB.velocity = this.bodyB.velocity.addNew(this.n.scaleNew(lambda * this.bodyB.invMass));
        this.bodyB.angularVelocity =
            this.bodyB.angularVelocity + this.n.dot(this.rb.crossScalar(lambda)) * this.bodyB.invI;
    }
}
