import Mat22 from '../math/Mat22';
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
    m00 = 0;
    m01 = 0;
    m11 = 0;

    rAx = 0;
    rAy = 0;
    rBx = 0;
    rBy = 0;

    biasX = 0;
    biasY = 0;
    cachedLambdaX = 0;
    cachedLambdaY = 0;

    biasFactor: number;
    softness: number;

    constructor(a: Body, b: Body, anchorWorld: Vec2, softness = 0.01, biasFactor = 0.2) {
        super(a, b, anchorWorld, anchorWorld);

        this.softness = softness;
        this.biasFactor = biasFactor;
    }

    preSolve(invDt: number): void {
        const bodyA = this.bodyA;
        const bodyB = this.bodyB;

        // Transform local contact points to world space using current rotation and position
        const cosA = Math.cos(bodyA.rotation);
        const sinA = Math.sin(bodyA.rotation);
        const anchorWorldAX = this.aPointLocal.x * cosA - this.aPointLocal.y * sinA + bodyA.position.x;
        const anchorWorldAY = this.aPointLocal.x * sinA + this.aPointLocal.y * cosA + bodyA.position.y;

        const cosB = Math.cos(bodyB.rotation);
        const sinB = Math.sin(bodyB.rotation);
        const anchorWorldBX = this.bPointLocal.x * cosB - this.bPointLocal.y * sinB + bodyB.position.x;
        const anchorWorldBY = this.bPointLocal.x * sinB + this.bPointLocal.y * cosB + bodyB.position.y;

        this.rAx = anchorWorldAX - this.bodyA.position.x;
        this.rAy = anchorWorldAY - this.bodyA.position.y;
        this.rBx = anchorWorldBX - this.bodyB.position.x;
        this.rBy = anchorWorldBY - this.bodyB.position.y;

        const Km00 =
            this.bodyA.invMass +
            this.bodyB.invMass +
            this.bodyA.invI * this.rAy * this.rAy +
            this.bodyB.invI * this.rBy * this.rBy +
            this.softness;
        const Km01 = -this.bodyA.invI * this.rAx * this.rAy + -this.bodyB.invI * this.rBx * this.rBy;
        const Km11 =
            this.bodyA.invMass +
            this.bodyB.invMass +
            this.bodyA.invI * this.rAx * this.rAx +
            this.bodyB.invI * this.rBx * this.rBx +
            this.softness;

        const det = 1.0 / (Km00 * Km11 - Km01 * Km01);

        this.m00 = det * Km11;
        this.m01 = -det * Km01;
        this.m11 = det * Km00;

        // ---- Bias (position correction) ----
        const pAX = this.bodyA.position.x + this.rAx;
        const pAY = this.bodyA.position.y + this.rAy;
        const pbX = this.bodyB.position.x + this.rBx;
        const pbY = this.bodyB.position.y + this.rBy;

        const relPosX = pbX - pAX;
        const relPosY = pbY - pAY;

        this.biasX = relPosX * (-this.biasFactor * invDt);
        this.biasY = relPosY * (-this.biasFactor * invDt);

        // ---- Warm starting ----
        this.bodyA.velocity.x -= this.cachedLambdaX * this.bodyA.invMass;
        this.bodyA.velocity.y -= this.cachedLambdaY * this.bodyA.invMass;
        this.bodyA.angularVelocity -= this.bodyA.invI * (this.rAx * this.cachedLambdaY - this.rAy * this.cachedLambdaX);

        this.bodyB.velocity.x += this.cachedLambdaX * this.bodyB.invMass;
        this.bodyB.velocity.y += this.cachedLambdaY * this.bodyB.invMass;
        this.bodyB.angularVelocity += this.bodyB.invI * (this.rBx * this.cachedLambdaY - this.rBy * this.cachedLambdaX);
    }

    solve(): void {
        const vAx = this.bodyA.velocity.x + -this.bodyA.angularVelocity * this.rAy;
        const vAy = this.bodyA.velocity.y + this.bodyA.angularVelocity * this.rAx;

        const vBx = this.bodyB.velocity.x + -this.bodyB.angularVelocity * this.rBy;
        const vBy = this.bodyB.velocity.y + this.bodyB.angularVelocity * this.rBx;

        const dvx = vBx - vAx;
        const dvy = vBy - vAy;

        const lambdaVectorX = this.biasX - dvx - this.cachedLambdaX * this.softness;
        const lambdaVectorY = this.biasY - dvy - this.cachedLambdaY * this.softness;

        const impulseX = this.m00 * lambdaVectorX + this.m01 * lambdaVectorY;
        const impulseY = this.m01 * lambdaVectorX + this.m11 * lambdaVectorY;

        this.bodyA.velocity.x -= impulseX * this.bodyA.invMass;
        this.bodyA.velocity.y -= impulseY * this.bodyA.invMass;
        this.bodyA.angularVelocity -= this.bodyA.invI * (this.rAx * impulseY - this.rAy * impulseX);

        this.bodyB.velocity.x += impulseX * this.bodyB.invMass;
        this.bodyB.velocity.y += impulseY * this.bodyB.invMass;
        this.bodyB.angularVelocity += this.bodyB.invI * (this.rBx * impulseY - this.rBy * impulseX);

        this.cachedLambdaX += impulseX;
        this.cachedLambdaY += impulseY;
    }

    postSolve(): void {
        // Optional: clamp accumulated impulse (recommended for stability)
        const maxImpulse = 1000;
        const mag = Math.sqrt(this.cachedLambdaX * this.cachedLambdaX + this.cachedLambdaY * this.cachedLambdaY);

        if (mag > maxImpulse) {
            this.cachedLambdaX *= maxImpulse / mag;
            this.cachedLambdaY *= maxImpulse / mag;
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
        const restitutionThreshold = 10;
        this.restitutionVelocityBias =
            normalRelativeVelocity < -restitutionThreshold ? -restitution * normalRelativeVelocity : 0;

        // Warm starting: Apply accumulated impulses from previous frame to velocities for faster convergence
        const impulseX =
            this.normalWorldX * this.accumulatedNormalImpulse + this.tangentWorldX * this.accumulatedTangentImpulse;
        const impulseY =
            this.normalWorldY * this.accumulatedNormalImpulse + this.tangentWorldY * this.accumulatedTangentImpulse;

        bodyA.velocity.x += impulseX * bodyA.invMass;
        bodyA.velocity.y += impulseY * bodyA.invMass;
        bodyA.angularVelocity += this.leverArmAX * impulseY - this.leverArmAY * impulseX;

        bodyB.velocity.x += -impulseX * bodyB.invMass;
        bodyB.velocity.y += -impulseY * bodyB.invMass;
        bodyB.angularVelocity += this.leverArmBX * -impulseY - this.leverArmBY * -impulseX;
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
