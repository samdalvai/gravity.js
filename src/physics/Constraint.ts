import Vec2 from '../math/Vec2';
import Body from './Body';

export abstract class Constraint {
    bodyA: Body;
    bodyB: Body;

    constructor(a: Body, b: Body) {
        this.bodyA = a;
        this.bodyB = b;
    }

    abstract preSolve(dt: number): void;
    abstract solve(): void;
}

// TODO: investigate separate velocity and position solver
export class JointConstraint extends Constraint {
    aPointLocal: Vec2; // The constraint point in A's local space
    bPointLocal: Vec2; // The constraint point in B's local space

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
        super(bodyA, bodyB);

        this.aPointLocal = bodyA.worldPointToLocal(worldAnchorPoint);
        this.bPointLocal = bodyB.worldPointToLocal(worldAnchorPoint);

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
}
