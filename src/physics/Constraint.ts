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
    M: Mat22;

    rA: Vec2;
    rB: Vec2;
    rAx = 0;
    rAy = 0;
    rBx = 0;
    rBy = 0;

    bias: Vec2;
    cachedLambda: Vec2; // accumulated impulse

    biasFactor: number;
    softness: number;

    constructor(a: Body, b: Body, anchorWorld: Vec2, softness = 0.01, biasFactor = 0.2) {
        super(a, b, anchorWorld, anchorWorld);

        this.M = new Mat22();

        this.rA = new Vec2();
        this.rB = new Vec2();

        this.bias = new Vec2();
        this.cachedLambda = new Vec2();

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

        this.rA.x = anchorWorldAX - this.bodyA.position.x;
        this.rA.y = anchorWorldAY - this.bodyA.position.y;
        this.rB.x = anchorWorldBX - this.bodyB.position.x;
        this.rB.y = anchorWorldBY - this.bodyB.position.y;

        // ---- Effective mass matrix ----
        const K = new Mat22();
        K.col1.x =
            this.bodyA.invMass +
            this.bodyB.invMass +
            this.bodyA.invI * this.rA.y * this.rA.y +
            this.bodyB.invI * this.rB.y * this.rB.y +
            this.softness;
        K.col1.y = 0 + -this.bodyA.invI * this.rA.x * this.rA.y + -this.bodyB.invI * this.rB.x * this.rB.y;
        K.col2.x = 0 + -this.bodyA.invI * this.rA.x * this.rA.y + -this.bodyB.invI * this.rB.x * this.rB.y;
        K.col2.y =
            this.bodyA.invMass +
            this.bodyB.invMass +
            this.bodyA.invI * this.rA.x * this.rA.x +
            this.bodyB.invI * this.rB.x * this.rB.x +
            this.softness;

        this.M = K.invert();

        // ---- Bias (position correction) ----
        const pAX = this.bodyA.position.x + this.rA.x;
        const pAY = this.bodyA.position.y + this.rA.y;
        const pbX = this.bodyB.position.x + this.rB.x;
        const pbY = this.bodyB.position.y + this.rB.y;

        const relPosX = pbX - pAX;
        const relPosY = pbY - pAY;

        this.bias.x = relPosX * (-this.biasFactor * invDt);
        this.bias.y = relPosY * (-this.biasFactor * invDt);

        // ---- Warm starting ----
        this.bodyA.velocity.x -= this.cachedLambda.x * this.bodyA.invMass;
        this.bodyA.velocity.y -= this.cachedLambda.y * this.bodyA.invMass;
        this.bodyA.angularVelocity -=
            this.bodyA.invI * (this.rA.x * this.cachedLambda.y - this.rA.y * this.cachedLambda.x);

        this.bodyB.velocity.x += this.cachedLambda.x * this.bodyB.invMass;
        this.bodyB.velocity.y += this.cachedLambda.y * this.bodyB.invMass;
        this.bodyB.angularVelocity +=
            this.bodyB.invI * (this.rB.x * this.cachedLambda.y - this.rB.y * this.cachedLambda.x);
    }

    solve(): void {
        const vA = this.bodyA.velocity.addNew(this.rA.crossScalar(this.bodyA.angularVelocity));
        const vB = this.bodyB.velocity.addNew(this.rB.crossScalar(this.bodyB.angularVelocity));

        const dv = vB.subNew(vA);

        const impulse = Mat22.multiply(this.M, this.bias.subNew(dv).subNew(this.cachedLambda.scaleNew(this.softness)));

        this.bodyA.velocity.x -= impulse.x * this.bodyA.invMass;
        this.bodyA.velocity.y -= impulse.y * this.bodyA.invMass;
        this.bodyA.angularVelocity -= this.bodyA.invI * (this.rA.x * impulse.y - this.rA.y * impulse.x);

        this.bodyB.velocity.x += impulse.x * this.bodyB.invMass;
        this.bodyB.velocity.y += impulse.y * this.bodyB.invMass;
        this.bodyB.angularVelocity += this.bodyB.invI * (this.rB.x * impulse.y - this.rB.y * impulse.x);

        this.cachedLambda.add(impulse);
    }

    postSolve(): void {
        // Optional: clamp accumulated impulse (recommended for stability)
        const maxImpulse = 1000;
        const mag = this.cachedLambda.magnitude();
        if (mag > maxImpulse) {
            this.cachedLambda.scaleAssign(maxImpulse / mag);
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
