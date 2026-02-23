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
import { SETTINGS } from '../core/Constants';
import { Joint } from './Joint';
import RigidBody from '../core/RigidBody';

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

        /*
        const jv = this.bodyB.linearVelocity
            .addNew(Util.cross(this.bodyB.angularVelocity, this.rb))
            .subNew(this.bodyA.linearVelocity.addNew(Util.cross(this.bodyA.angularVelocity, this.ra)))
            .dot(this.n);
        */
        const jv =
            (this.bodyB.velocity.x +
                -this.bodyB.angularVelocity * this.rb.y -
                (this.bodyA.velocity.x + -this.bodyA.angularVelocity)) *
                this.n.x +
            (this.bodyB.velocity.y +
                this.bodyB.angularVelocity * this.rb.x -
                (this.bodyA.velocity.y + this.bodyA.angularVelocity * this.ra.x)) *
                this.n.y;

        const lambda = this.m * -(jv + this.bias + this.impulseSum * this.gamma);

        this.applyImpulse(lambda);

        if (SETTINGS.warmStarting) this.impulseSum += lambda;
    }

    private applyImpulse(lambda: number): void {
        /*
        this.bodyA.linearVelocity = this.bodyA.linearVelocity.subNew(this.n.scaleNew(lambda * this.bodyA.inverseMass));
        this.bodyA.angularVelocity =
            this.bodyA.angularVelocity - this.n.dot(Util.cross(lambda, this.ra)) * this.bodyA.inverseInertia;
        */
        this.bodyA.velocity.x = this.bodyA.velocity.x - this.n.x * (lambda * this.bodyA.invMass);
        this.bodyA.velocity.y = this.bodyA.velocity.y - this.n.y * (lambda * this.bodyA.invMass);
        this.bodyA.angularVelocity =
            this.bodyA.angularVelocity -
            (this.n.x * (-lambda * this.ra.y) + this.n.y * (lambda * this.ra.x)) * this.bodyA.invI;

        /*
        this.bodyB.linearVelocity = this.bodyB.linearVelocity.addNew(this.n.scaleNew(lambda * this.bodyB.inverseMass));
        this.bodyB.angularVelocity =
            this.bodyB.angularVelocity + this.n.dot(Util.cross(lambda, this.rb)) * this.bodyB.inverseInertia;
        */
        this.bodyB.velocity.x = this.bodyB.velocity.x + this.n.x * (lambda * this.bodyB.invMass);
        this.bodyB.velocity.y = this.bodyB.velocity.y + this.n.y * (lambda * this.bodyB.invMass);
        this.bodyB.angularVelocity =
            this.bodyB.angularVelocity +
            (this.n.x * (-lambda * this.rb.y) + this.n.y * (lambda * this.rb.x)) * this.bodyB.invI;
    }
}
