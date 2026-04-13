import { SETTINGS } from '../core/Constants';
import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import * as Utils from '../utils/Utils';
import { ContactManifold, ContactType } from './ContactManifold';

export class ContactSolver {
    private readonly manifold: ContactManifold;

    private readonly bodyA: RigidBody;
    private readonly bodyB: RigidBody;
    private readonly contactPointX: number;
    private readonly contactPointY: number;
    private contactType: ContactType = ContactType.Normal;

    private readonly beta: number;
    private readonly restitution: number;
    private readonly friction: number;

    private raX = 0.0;
    private raY = 0.0;
    private rbX = 0.0;
    private rbY = 0.0;

    public jvaX = 0.0;
    public jvaY = 0.0;
    public jwa = 0.0;
    public jvbX = 0.0;
    public jvbY = 0.0;
    public jwb = 0.0;
    public bias = 0.0;
    public effectiveMass = 0.0;

    public impulseSum: number = 0.0; // For accumulated impulse

    constructor(manifold: ContactManifold, contactPoint: Vec2) {
        this.manifold = manifold;
        this.bodyA = manifold.bodyA;
        this.bodyB = manifold.bodyB;
        this.contactPointX = contactPoint.x;
        this.contactPointY = contactPoint.y;

        this.beta = SETTINGS.positionCorrectionBeta;
        this.restitution = this.bodyA.restitution * this.bodyB.restitution;
        this.friction = this.bodyA.friction * this.bodyB.friction;
    }

    preSolve(dir: Vec2, contactType: ContactType, featureFlipped: boolean, invDt: number) {
        // Calculate Jacobian J and effective mass M
        // J = [-dir, -ra × dir, dir, rb × dir] (dir: Contact vector, normal or tangent)
        // M = (J · M^-1 · J^t)^-1

        this.contactType = contactType;

        const bodyAPosition = this.bodyA.position;
        const bodyBPosition = this.bodyB.position;
        this.raX = this.contactPointX - bodyAPosition.x;
        this.raY = this.contactPointY - bodyAPosition.y;
        this.rbX = this.contactPointX - bodyBPosition.x;
        this.rbY = this.contactPointY - bodyBPosition.y;

        const dirX = dir.x;
        const dirY = dir.y;
        this.jvaX = -dirX;
        this.jvaY = -dirY;
        this.jwa = this.raY * dirX - this.raX * dirY;
        this.jvbX = dirX;
        this.jvbY = dirY;
        this.jwb = this.rbX * dirY - this.rbY * dirX;

        this.bias = 0.0;
        if (this.contactType == ContactType.Normal) {
            // Relative velocity at contact point
            const bodyAVelocity = this.bodyA.velocity;
            const bodyBVelocity = this.bodyB.velocity;
            const bodyAAngularVelocity = this.bodyA.angularVelocity;
            const bodyBAngularVelocity = this.bodyB.angularVelocity;
            const relativeVelocityX =
                bodyBVelocity.x - bodyBAngularVelocity * this.rbY - (bodyAVelocity.x - bodyAAngularVelocity * this.raY);
            const relativeVelocityY =
                bodyBVelocity.y + bodyBAngularVelocity * this.rbX - (bodyAVelocity.y + bodyAAngularVelocity * this.raX);
            const contactNormal = this.manifold.contactNormal;
            const normalVelocity = contactNormal.x * relativeVelocityX + contactNormal.y * relativeVelocityY;

            // We skip positional correction for 0 inverse delta time because CCD can generate 0 dt
            if (SETTINGS.positionCorrection && invDt > 0) {
                this.bias =
                    -(this.beta * invDt) * Math.max(this.manifold.penetrationDepth! - SETTINGS.penetrationSlop, 0.0);
            }

            // Only apply restitution on the initial impact. Reapplying it on persistent
            // contacts can inject energy into resting stacks when warm starting is enabled.
            if (!this.manifold.persistent && normalVelocity + SETTINGS.restitutionSlop < 0.0) {
                this.bias += this.restitution * normalVelocity;
            }
        } else {
            // Bias for surface speed that enables the conveyor belt-like behavior
            this.bias = this.bodyA.surfaceSpeed - this.bodyB.surfaceSpeed;
            if (featureFlipped) this.bias *= -1;
        }

        const k: number =
            this.bodyA.invMass +
            this.jwa * this.bodyA.invI * this.jwa +
            this.bodyB.invMass +
            this.jwb * this.bodyB.invI * this.jwb;

        this.effectiveMass = k > 0.0 ? 1.0 / k : 0.0;

        // Apply the old impulse calculated in the previous time step
        if (SETTINGS.warmStarting && this.impulseSum !== 0.0) this.applyImpulse(this.impulseSum);
    }

    solve(normalContact?: ContactSolver) {
        // Calculate corrective impulse: Pc
        // Pc = J^t * λ (λ: lagrangian multiplier)
        // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

        // Jacobian * velocity vector (Normal velocity)
        const bodyAVelocity = this.bodyA.velocity;
        const bodyBVelocity = this.bodyB.velocity;
        const jv: number =
            this.jvaX * bodyAVelocity.x +
            this.jvaY * bodyAVelocity.y +
            this.jwa * this.bodyA.angularVelocity +
            this.jvbX * bodyBVelocity.x +
            this.jvbY * bodyBVelocity.y +
            this.jwb * this.bodyB.angularVelocity;

        let lambda = this.effectiveMass * -(jv + this.bias);

        const oldImpulseSum = this.impulseSum;
        switch (this.contactType) {
            case ContactType.Normal: {
                if (SETTINGS.impulseAccumulation) this.impulseSum = Math.max(0.0, this.impulseSum + lambda);
                else this.impulseSum = Math.max(0.0, lambda);
                break;
            }
            case ContactType.Tangent: {
                const maxFriction = this.friction * normalContact!.impulseSum;
                if (SETTINGS.impulseAccumulation)
                    this.impulseSum = Utils.clamp(this.impulseSum + lambda, -maxFriction, maxFriction);
                else this.impulseSum = Utils.clamp(lambda, -maxFriction, maxFriction);
                break;
            }
        }

        if (SETTINGS.impulseAccumulation) lambda = this.impulseSum - oldImpulseSum;
        else lambda = this.impulseSum;

        // Apply impulse
        this.applyImpulse(lambda);
    }

    private applyImpulse(lambda: number) {
        // V2 = V2' + M^-1 ⋅ Pc
        // Pc = J^t ⋅ λ

        if (lambda === 0.0) {
            return;
        }

        const bodyAImpulseScale = this.bodyA.invMass * lambda;
        this.bodyA.velocity.x += this.jvaX * bodyAImpulseScale;
        this.bodyA.velocity.y += this.jvaY * bodyAImpulseScale;
        this.bodyA.angularVelocity += this.bodyA.invI * this.jwa * lambda;

        const bodyBImpulseScale = this.bodyB.invMass * lambda;
        this.bodyB.velocity.x += this.jvbX * bodyBImpulseScale;
        this.bodyB.velocity.y += this.jvbY * bodyBImpulseScale;
        this.bodyB.angularVelocity += this.bodyB.invI * this.jwb * lambda;
    }
}
