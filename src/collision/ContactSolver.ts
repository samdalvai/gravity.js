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
import RigidBody from '../core/RigidBody';
import Vec2 from '../math/Vec2';
import * as Utils from '../utils/Utils';
import { ContactManifold, ContactType, Jacobian } from './ContactManifold';

export class ContactSolver {
    private manifold: ContactManifold;

    private bodyA: RigidBody;
    private bodyB: RigidBody;
    private contactPoint: Vec2;
    private contactType!: ContactType;

    private beta: number;
    private restitution: number;
    private friction: number;

    private ra!: Vec2;
    private rb!: Vec2;

    public jacobian!: Jacobian;
    public bias!: number;
    public effectiveMass!: number;

    public impulseSum: number = 0.0; // For accumulated impulse

    constructor(manifold: ContactManifold, contactPoint: Vec2) {
        this.manifold = manifold;
        this.bodyA = manifold.bodyA;
        this.bodyB = manifold.bodyB;
        this.contactPoint = contactPoint;

        this.beta = SETTINGS.positionCorrectionBeta;
        this.restitution = this.bodyA.restitution * this.bodyB.restitution;
        this.friction = this.bodyA.friction * this.bodyB.friction;
    }

    preSolve(dir: Vec2, contactType: ContactType, featureFlipped: boolean, inverseDeltaTime: number) {
        // Calculate Jacobian J and effective mass M
        this.contactType = contactType;

        this.ra = this.contactPoint.subNew(this.bodyA.position);
        this.rb = this.contactPoint.subNew(this.bodyB.position);

        this.jacobian = {
            va: dir.negateNew(),
            wa: -this.ra!.cross(dir),
            vb: dir,
            wb: this.rb!.cross(dir),
        };

        this.bias = 0.0;
        if (this.contactType == ContactType.Normal) {
            // Relative velocity at contact point
            const relativeVelocity = this.bodyB.velocity
                .addNew(this.rb.crossScalar(this.bodyB.angularVelocity))
                .subNew(this.bodyA.velocity.addNew(this.ra.crossScalar(this.bodyA.angularVelocity)));
            const normalVelocity = this.manifold.contactNormal.dot(relativeVelocity);

            if (SETTINGS.positionCorrection) {
                this.bias =
                    -(this.beta * inverseDeltaTime) *
                    Math.max(this.manifold.penetrationDepth! - SETTINGS.penetrationSlop, 0.0);
            }

            this.bias += this.restitution * Math.min(normalVelocity + SETTINGS.restitutionSlop, 0.0);
        } else {
            // Bias for surface speed that enables the conveyor belt-like behavior
            this.bias = -(this.bodyB.surfaceSpeed - this.bodyA.surfaceSpeed);
            if (featureFlipped) this.bias *= -1;
        }

        const k: number =
            +this.bodyA.invMass +
            this.jacobian.wa * this.bodyA.invI * this.jacobian.wa +
            this.bodyB.invMass +
            this.jacobian.wb * this.bodyB.invI * this.jacobian.wb;

        this.effectiveMass = k > 0.0 ? 1.0 / k : 0.0;

        // Apply the old impulse calculated in the previous time step
        if (SETTINGS.warmStarting) this.applyImpulse(this.impulseSum);
    }

    solve(normalContact?: ContactSolver) {
        // Calculate corrective impulse: Pc
        // Jacobian * velocity vector (Normal velocity)

        /*
        const jv: number =
            +this.jacobian.va.dot(this.bodyA.linearVelocity) +
            this.jacobian.wa * this.bodyA.angularVelocity +
            this.jacobian.vb.dot(this.bodyB.linearVelocity) +
            this.jacobian.wb * this.bodyB.angularVelocity;
        */
        const jv: number =
            this.jacobian.va.x * this.bodyA.velocity.x +
            this.jacobian.va.y * this.bodyA.velocity.y +
            this.jacobian.wa * this.bodyA.angularVelocity +
            this.jacobian.vb.x * this.bodyB.velocity.x +
            this.jacobian.vb.y * this.bodyB.velocity.y +
            this.jacobian.wb * this.bodyB.angularVelocity;

        let lambda = this.effectiveMass * -(jv + this.bias);

        const oldImpulseSum = this.impulseSum;
        switch (this.contactType) {
            case ContactType.Normal: {
                if (SETTINGS.impulseAccumulation) {
                    this.impulseSum = Math.max(0.0, this.impulseSum + lambda);
                } else {
                    this.impulseSum = Math.max(0.0, lambda);
                }
                break;
            }
            case ContactType.Tangent: {
                const maxFriction = this.friction * normalContact!.impulseSum;
                if (SETTINGS.impulseAccumulation) {
                    this.impulseSum = Utils.clamp(this.impulseSum + lambda, -maxFriction, maxFriction);
                } else {
                    this.impulseSum = Utils.clamp(lambda, -maxFriction, maxFriction);
                }
                break;
            }
        }

        if (SETTINGS.impulseAccumulation) {
            lambda = this.impulseSum - oldImpulseSum;
        } else {
            lambda = this.impulseSum;
        }

        // Apply impulse
        this.applyImpulse(lambda);
    }

    private applyImpulse(lambda: number) {
        /*
        this.bodyA.linearVelocity = this.bodyA.linearVelocity.addNew(
            this.jacobian.va.scaleNew(this.bodyA.inverseMass * lambda),
        );
        this.bodyA.angularVelocity = this.bodyA.angularVelocity + this.bodyA.inverseInertia * this.jacobian.wa * lambda;
        */
        this.bodyA.velocity.x = this.bodyA.velocity.x + this.jacobian.va.x * this.bodyA.invMass * lambda;
        this.bodyA.velocity.y = this.bodyA.velocity.y + this.jacobian.va.y * this.bodyA.invMass * lambda;
        this.bodyA.angularVelocity = this.bodyA.angularVelocity + this.bodyA.invI * this.jacobian.wa * lambda;

        /*
        this.bodyB.linearVelocity = this.bodyB.linearVelocity.addNew(
            this.jacobian.vb.scaleNew(this.bodyB.inverseMass * lambda),
        );
        this.bodyB.angularVelocity = this.bodyB.angularVelocity + this.bodyB.inverseInertia * this.jacobian.wb * lambda;
        */
        this.bodyB.velocity.x = this.bodyB.velocity.x + this.jacobian.vb.x * this.bodyB.invMass * lambda;
        this.bodyB.velocity.y = this.bodyB.velocity.y + this.jacobian.vb.y * this.bodyB.invMass * lambda;
        this.bodyB.angularVelocity = this.bodyB.angularVelocity + this.bodyB.invI * this.jacobian.wb * lambda;
    }
}
