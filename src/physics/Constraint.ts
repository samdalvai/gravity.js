import Utils from '../math/Utils';
import Vec2 from '../math/Vec2';
import Body from './Body';

export abstract class Constraint {
    bodyA: Body;
    bodyB: Body;

    aPointLocal: Vec2; // The constraint point in A's local space
    bPointLocal: Vec2; // The constraint point in B's local space

    constructor(a: Body, b: Body, aPointWorld: Vec2, bPointWorld: Vec2) {
        this.bodyA = a;
        this.bodyB = b;

        this.aPointLocal = a.worldSpaceToLocalSpace(aPointWorld);
        this.bPointLocal = b.worldSpaceToLocalSpace(bPointWorld);
    }

    abstract preSolve(dt: number): void;
    abstract solve(): void;
    abstract postSolve(): void;
}

export class JointConstraint extends Constraint {
    // Elements of the 2x2 effective mass matrix (inverse of the constraint mass matrix) for x-x, x-y, and y-y components.
    private effectiveMassXX = 0;
    private effectiveMassXY = 0;
    private effectiveMassYY = 0;

    // Lever arms (vectors from centers of mass to anchor points in world space)
    private leverArmAX = 0;
    private leverArmAY = 0;
    private leverArmBX = 0;
    private leverArmBY = 0;

    // The position correction bias Baumgarte stabilization).
    private positionBiasX = 0;
    private positionBiasY = 0;

    // The accumulated impulse for the x-direction (for warm starting and sequential solving).
    private accumulatedImpulseX = 0;
    private accumulatedImpulseY = 0;

    // Strength of position correction (typically between 0 and 1)
    private biasFactor: number;

    // Softness parameter allowing some constraint compliance (for stability in soft constraints).
    private softness: number;

    constructor(bodyA: Body, bodyB: Body, worldAnchorPoint: Vec2, softness = 0.01, biasFactor = 0.2) {
        super(bodyA, bodyB, worldAnchorPoint, worldAnchorPoint);

        this.softness = softness;
        this.biasFactor = biasFactor;
    }

    preSolve(inverseDeltaTime: number): void {
        const bodyA = this.bodyA;
        const bodyB = this.bodyB;

        // Transform local anchor points to world space using current rotation and position
        const cosA = Math.cos(bodyA.rotation);
        const sinA = Math.sin(bodyA.rotation);
        const anchorWorldAX = this.aPointLocal.x * cosA - this.aPointLocal.y * sinA + bodyA.position.x;
        const anchorWorldAY = this.aPointLocal.x * sinA + this.aPointLocal.y * cosA + bodyA.position.y;

        const cosB = Math.cos(bodyB.rotation);
        const sinB = Math.sin(bodyB.rotation);
        const anchorWorldBX = this.bPointLocal.x * cosB - this.bPointLocal.y * sinB + bodyB.position.x;
        const anchorWorldBY = this.bPointLocal.x * sinB + this.bPointLocal.y * cosB + bodyB.position.y;

        // Compute lever arms (vectors from centers of mass to anchor points in world space)
        this.leverArmAX = anchorWorldAX - bodyA.position.x;
        this.leverArmAY = anchorWorldAY - bodyA.position.y;
        this.leverArmBX = anchorWorldBX - bodyB.position.x;
        this.leverArmBY = anchorWorldBY - bodyB.position.y;

        // Compute the constraint mass matrix K (including softness on diagonal)
        const massMatrixXX =
            bodyA.invMass +
            bodyB.invMass +
            bodyA.invI * this.leverArmAY * this.leverArmAY +
            bodyB.invI * this.leverArmBY * this.leverArmBY +
            this.softness;
        const massMatrixXY =
            -bodyA.invI * this.leverArmAX * this.leverArmAY - bodyB.invI * this.leverArmBX * this.leverArmBY;
        const massMatrixYY =
            bodyA.invMass +
            bodyB.invMass +
            bodyA.invI * this.leverArmAX * this.leverArmAX +
            bodyB.invI * this.leverArmBX * this.leverArmBX +
            this.softness;

        // Compute determinant for inverting the 2x2 matrix
        const determinant = 1.0 / (massMatrixXX * massMatrixYY - massMatrixXY * massMatrixXY);

        // Compute effective mass matrix as inverse of K
        this.effectiveMassXX = determinant * massMatrixYY;
        this.effectiveMassXY = -determinant * massMatrixXY;
        this.effectiveMassYY = determinant * massMatrixXX;

        // Compute current world positions of anchors (position correction)
        const anchorPositionAX = bodyA.position.x + this.leverArmAX;
        const anchorPositionAY = bodyA.position.y + this.leverArmAY;
        const anchorPositionBX = bodyB.position.x + this.leverArmBX;
        const anchorPositionBY = bodyB.position.y + this.leverArmBY;

        // Compute relative position error (anchorB - anchorA)
        const relativePositionX = anchorPositionBX - anchorPositionAX;
        const relativePositionY = anchorPositionBY - anchorPositionAY;

        // Compute position biases using Baumgarte stabilization
        this.positionBiasX = relativePositionX * (-this.biasFactor * inverseDeltaTime);
        this.positionBiasY = relativePositionY * (-this.biasFactor * inverseDeltaTime);

        // Apply accumulated impulses from previous frame to velocities for faster convergence (Warm starting)
        bodyA.velocity.x -= this.accumulatedImpulseX * bodyA.invMass;
        bodyA.velocity.y -= this.accumulatedImpulseY * bodyA.invMass;
        bodyA.angularVelocity -=
            bodyA.invI * (this.leverArmAX * this.accumulatedImpulseY - this.leverArmAY * this.accumulatedImpulseX);

        bodyB.velocity.x += this.accumulatedImpulseX * bodyB.invMass;
        bodyB.velocity.y += this.accumulatedImpulseY * bodyB.invMass;
        bodyB.angularVelocity +=
            bodyB.invI * (this.leverArmBX * this.accumulatedImpulseY - this.leverArmBY * this.accumulatedImpulseX);
    }

    solve(): void {
        const bodyA = this.bodyA;
        const bodyB = this.bodyB;

        // Compute velocities at anchor points (linear velocity plus angular contribution)
        const velocityAtAnchorAX = bodyA.velocity.x - bodyA.angularVelocity * this.leverArmAY;
        const velocityAtAnchorAY = bodyA.velocity.y + bodyA.angularVelocity * this.leverArmAX;

        const velocityAtAnchorBX = bodyB.velocity.x - bodyB.angularVelocity * this.leverArmBY;
        const velocityAtAnchorBY = bodyB.velocity.y + bodyB.angularVelocity * this.leverArmBX;

        // Compute relative velocities (B - A)
        const relativeVelocityX = velocityAtAnchorBX - velocityAtAnchorAX;
        const relativeVelocityY = velocityAtAnchorBY - velocityAtAnchorAY;

        // Compute the constraint violation vector (bias - relative velocity - softness * accumulated)
        const constraintViolationX = this.positionBiasX - relativeVelocityX - this.accumulatedImpulseX * this.softness;
        const constraintViolationY = this.positionBiasY - relativeVelocityY - this.accumulatedImpulseY * this.softness;

        // Compute delta impulse using effective mass matrix
        const deltaImpulseX = this.effectiveMassXX * constraintViolationX + this.effectiveMassXY * constraintViolationY;
        const deltaImpulseY = this.effectiveMassXY * constraintViolationX + this.effectiveMassYY * constraintViolationY;

        // Apply delta impulses to bodies (negative on A, positive on B)
        bodyA.velocity.x -= deltaImpulseX * bodyA.invMass;
        bodyA.velocity.y -= deltaImpulseY * bodyA.invMass;
        bodyA.angularVelocity -= bodyA.invI * (this.leverArmAX * deltaImpulseY - this.leverArmAY * deltaImpulseX);

        bodyB.velocity.x += deltaImpulseX * bodyB.invMass;
        bodyB.velocity.y += deltaImpulseY * bodyB.invMass;
        bodyB.angularVelocity += bodyB.invI * (this.leverArmBX * deltaImpulseY - this.leverArmBY * deltaImpulseX);

        // Accumulate the delta impulses
        this.accumulatedImpulseX += deltaImpulseX;
        this.accumulatedImpulseY += deltaImpulseY;
    }

    postSolve(): void {
        // Optional: Clamp the magnitude of accumulated impulses to prevent numerical instability
        const maxImpulseMagnitude = 1000;
        const impulseMagnitude = Math.sqrt(
            this.accumulatedImpulseX * this.accumulatedImpulseX + this.accumulatedImpulseY * this.accumulatedImpulseY,
        );

        if (impulseMagnitude > maxImpulseMagnitude) {
            this.accumulatedImpulseX *= maxImpulseMagnitude / impulseMagnitude;
            this.accumulatedImpulseY *= maxImpulseMagnitude / impulseMagnitude;
        }
    }
}

export class ContactConstraint extends Constraint {
    private normalWorldX = 0;
    private normalWorldY = 0;

    penetrationDepth: number;

    // Lever arms (vectors from centers of mass to contact points in world space)
    private leverArmAX = 0;
    private leverArmAY = 0;
    private leverArmBX = 0;
    private leverArmBY = 0;

    private tangentWorldX = 0;
    private tangentWorldY = 0;

    // Effective masses (inverse of the reduced mass for the constraint directions)
    private effectiveMassNormal = 0;
    private effectiveMassTangent = 0;

    // Bias terms (for position correction and restitution)
    private positionBias = 0;
    private restitutionVelocityBias = 0;

    // Accumulated impulses (for warm starting and sequential impulse solving)
    private accumulatedNormalImpulse = 0;
    private accumulatedTangentImpulse = 0;

    constructor(
        bodyA: Body,
        bodyB: Body,
        localContactPointA: Vec2,
        localContactPointB: Vec2,
        normalWorld: Vec2,
        penetrationDepth: number,
    ) {
        super(bodyA, bodyB, localContactPointA, localContactPointB);

        // Ensure normal always points A → B
        // TODO: Why do we need to negate the normal?
        this.normalWorldX = -normalWorld.x;
        this.normalWorldY = -normalWorld.y;
        this.penetrationDepth = penetrationDepth;
    }

    // TODO: removing angular velocity change make the solver much stabler for stack of boxes, investigate why and how to improve

    preSolve(inverseDeltaTime: number): void {
        const bodyA = this.bodyA;
        const bodyB = this.bodyB;

        // Transform local contact points to world space using current rotation and position
        const cosA = Math.cos(bodyA.rotation);
        const sinA = Math.sin(bodyA.rotation);
        const contactWorldAX = this.aPointLocal.x * cosA - this.aPointLocal.y * sinA + bodyA.position.x;
        const contactWorldAY = this.aPointLocal.x * sinA + this.aPointLocal.y * cosA + bodyA.position.y;

        const cosB = Math.cos(bodyB.rotation);
        const sinB = Math.sin(bodyB.rotation);
        const contactWorldBX = this.bPointLocal.x * cosB - this.bPointLocal.y * sinB + bodyB.position.x;
        const contactWorldBY = this.bPointLocal.x * sinB + this.bPointLocal.y * cosB + bodyB.position.y;

        // Compute lever arms (vectors from centers of mass to contact points)
        this.leverArmAX = contactWorldAX - bodyA.position.x;
        this.leverArmAY = contactWorldAY - bodyA.position.y;
        this.leverArmBX = contactWorldBX - bodyB.position.x;
        this.leverArmBY = contactWorldBY - bodyB.position.y;

        // Compute tangent vector as the left-perpendicular to the normal (for friction direction)
        this.tangentWorldX = this.normalWorldY;
        this.tangentWorldY = -this.normalWorldX;

        // Compute torque arms for normal direction (cross product of lever arm and normal)
        const normalTorqueArmA = this.leverArmAX * this.normalWorldY - this.leverArmAY * this.normalWorldX;
        const normalTorqueArmB = this.leverArmBX * this.normalWorldY - this.leverArmBY * this.normalWorldX;

        // Compute effective mass for normal direction (accounts for linear and angular contributions)
        this.effectiveMassNormal =
            1 /
            (bodyA.invMass +
                bodyB.invMass +
                normalTorqueArmA * normalTorqueArmA * bodyA.invI +
                normalTorqueArmB * normalTorqueArmB * bodyB.invI);

        // Compute torque arms for tangent direction (cross product of lever arm and tangent)
        const tangentTorqueArmA = this.leverArmAX * this.tangentWorldY - this.leverArmAY * this.tangentWorldX;
        const tangentTorqueArmB = this.leverArmBX * this.tangentWorldY - this.leverArmBY * this.tangentWorldX;

        // Compute effective mass for tangent direction (for friction)
        this.effectiveMassTangent =
            1 /
            (bodyA.invMass +
                bodyB.invMass +
                tangentTorqueArmA * tangentTorqueArmA * bodyA.invI +
                tangentTorqueArmB * tangentTorqueArmB * bodyB.invI);

        // Compute position bias using Baumgarte stabilization to gradually correct penetration
        const allowedPenetration = 0.01; // Small allowance for penetration to avoid jitter
        const stabilizationFactor = 0.2; // Beta factor controlling correction speed
        this.positionBias =
            Math.max(this.penetrationDepth - allowedPenetration, 0) * stabilizationFactor * inverseDeltaTime;

        // Compute velocity at contact points (linear velocity plus angular contribution)
        const velocityAtContactAX = bodyA.velocity.x - bodyA.angularVelocity * this.leverArmAY;
        const velocityAtContactAY = bodyA.velocity.y + bodyA.angularVelocity * this.leverArmAX;
        const velocityAtContactBX = bodyB.velocity.x - bodyB.angularVelocity * this.leverArmBY;
        const velocityAtContactBY = bodyB.velocity.y + bodyB.angularVelocity * this.leverArmBX;

        // Compute relative velocity between contact points
        const relativeVelocityX = velocityAtContactAX - velocityAtContactBX;
        const relativeVelocityY = velocityAtContactAY - velocityAtContactBY;

        // Project relative velocity onto normal to get separating velocity
        const normalRelativeVelocity = relativeVelocityX * this.normalWorldX + relativeVelocityY * this.normalWorldY;

        // Compute combined restitution (bounciness) as minimum of both bodies
        const restitution = Math.min(bodyA.restitution, bodyB.restitution);

        // Apply restitution only if separating velocity exceeds threshold (to avoid unnecessary bounce)
        const restitutionThreshold = 0.5 * inverseDeltaTime;
        this.restitutionVelocityBias =
            normalRelativeVelocity < -restitutionThreshold ? -restitution * normalRelativeVelocity : 0;

        // Warm starting: Apply accumulated impulses from previous frame to velocities for faster convergence
        const impulseX =
            this.normalWorldX * this.accumulatedNormalImpulse + this.tangentWorldX * this.accumulatedTangentImpulse;
        const impulseY =
            this.normalWorldY * this.accumulatedNormalImpulse + this.tangentWorldY * this.accumulatedTangentImpulse;

        bodyA.velocity.x += impulseX * bodyA.invMass;
        bodyA.velocity.y += impulseY * bodyA.invMass;
        bodyA.angularVelocity += (this.leverArmAX * impulseY - this.leverArmAY * impulseX) * bodyA.invI;

        bodyB.velocity.x += -impulseX * bodyB.invMass;
        bodyB.velocity.y += -impulseY * bodyB.invMass;
        bodyB.angularVelocity += (this.leverArmBX * -impulseY - this.leverArmBY * -impulseX) * bodyB.invI;
    }

    solve(): void {
        const bodyA = this.bodyA;
        const bodyB = this.bodyB;

        // Recompute velocity at contact points (may have changed from other constraints)
        const velocityAtContactAX = bodyA.velocity.x - bodyA.angularVelocity * this.leverArmAY;
        const velocityAtContactAY = bodyA.velocity.y + bodyA.angularVelocity * this.leverArmAX;
        const velocityAtContactBX = bodyB.velocity.x - bodyB.angularVelocity * this.leverArmBY;
        const velocityAtContactBY = bodyB.velocity.y + bodyB.angularVelocity * this.leverArmBX;

        // Compute relative velocity between contact points
        const relativeVelocityX = velocityAtContactAX - velocityAtContactBX;
        const relativeVelocityY = velocityAtContactAY - velocityAtContactBY;

        // Project relative velocity onto normal
        const normalRelativeVelocity = relativeVelocityX * this.normalWorldX + relativeVelocityY * this.normalWorldY;

        // Compute delta impulse for normal direction (corrects velocity violation plus biases)
        let deltaNormalImpulse =
            this.effectiveMassNormal * (-normalRelativeVelocity + this.positionBias + this.restitutionVelocityBias);

        // Accumulate normal impulse (non-negative to prevent pulling)
        const oldNormalImpulse = this.accumulatedNormalImpulse;
        this.accumulatedNormalImpulse = Math.max(oldNormalImpulse + deltaNormalImpulse, 0);
        deltaNormalImpulse = this.accumulatedNormalImpulse - oldNormalImpulse;

        // Compute delta impulse vector for normal
        const normalDeltaImpulseX = this.normalWorldX * deltaNormalImpulse;
        const normalDeltaImpulseY = this.normalWorldY * deltaNormalImpulse;

        // Apply normal delta impulse to velocities and angular velocities
        bodyA.velocity.x += normalDeltaImpulseX * bodyA.invMass;
        bodyA.velocity.y += normalDeltaImpulseY * bodyA.invMass;
        bodyA.angularVelocity +=
            (this.leverArmAX * normalDeltaImpulseY - this.leverArmAY * normalDeltaImpulseX) * bodyA.invI;

        bodyB.velocity.x += -normalDeltaImpulseX * bodyB.invMass;
        bodyB.velocity.y += -normalDeltaImpulseY * bodyB.invMass;
        bodyB.angularVelocity +=
            (this.leverArmBX * -normalDeltaImpulseY - this.leverArmBY * -normalDeltaImpulseX) * bodyB.invI;

        // Project relative velocity onto tangent for friction
        const tangentRelativeVelocity = relativeVelocityX * this.tangentWorldX + relativeVelocityY * this.tangentWorldY;

        // Compute delta impulse for tangent direction (opposes sliding)
        let deltaTangentImpulse = -tangentRelativeVelocity * this.effectiveMassTangent;

        // Compute combined friction coefficient as minimum of both bodies
        const frictionCoefficient = Math.min(bodyA.friction, bodyB.friction);
        // Maximum friction impulse limited by normal impulse (Coulomb friction model)
        const maxFrictionImpulse = frictionCoefficient * this.accumulatedNormalImpulse;

        // TODO: add slop for tangent impulse to avoid circles rolling forever?
        // Accumulate tangent impulse (clamped to friction cone)
        const oldTangentImpulse = this.accumulatedTangentImpulse;
        this.accumulatedTangentImpulse = Utils.clamp(
            oldTangentImpulse + deltaTangentImpulse,
            -maxFrictionImpulse,
            maxFrictionImpulse,
        );
        deltaTangentImpulse = this.accumulatedTangentImpulse - oldTangentImpulse;

        // Compute delta impulse vector for tangent
        const tangentDeltaImpulseX = this.tangentWorldX * deltaTangentImpulse;
        const tangentDeltaImpulseY = this.tangentWorldY * deltaTangentImpulse;

        // Apply tangent delta impulse to velocities and angular velocities
        bodyA.velocity.x += tangentDeltaImpulseX * bodyA.invMass;
        bodyA.velocity.y += tangentDeltaImpulseY * bodyA.invMass;
        bodyA.angularVelocity +=
            (this.leverArmAX * tangentDeltaImpulseY - this.leverArmAY * tangentDeltaImpulseX) * bodyA.invI;

        bodyB.velocity.x += -tangentDeltaImpulseX * bodyB.invMass;
        bodyB.velocity.y += -tangentDeltaImpulseY * bodyB.invMass;
        bodyB.angularVelocity +=
            (this.leverArmBX * -tangentDeltaImpulseY - this.leverArmBY * -tangentDeltaImpulseX) * bodyB.invI;
    }

    postSolve(): void {
        // Optional: Clamp accumulated impulses to prevent numerical blow-up or instability
        const maxAccumulatedImpulse = 1e6;
        this.accumulatedNormalImpulse = Math.min(this.accumulatedNormalImpulse, maxAccumulatedImpulse);
        this.accumulatedTangentImpulse = Utils.clamp(
            this.accumulatedTangentImpulse,
            -maxAccumulatedImpulse,
            maxAccumulatedImpulse,
        );
    }
}
