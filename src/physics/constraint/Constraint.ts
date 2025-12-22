import Utils from '../../math/Utils';
import Vec2 from '../../math/Vec2';
import Body from '../body/Body';

export abstract class Constraint {
    a: Body;
    b: Body;

    aPoint: Vec2; // The constraint point in A's local space
    bPoint: Vec2; // The constraint point in B's local space

    constructor(a: Body, b: Body, aPointWorld: Vec2, bPointWorld: Vec2) {
        this.a = a;
        this.b = b;

        this.aPoint = a.worldSpaceToLocalSpace(aPointWorld);
        this.bPoint = b.worldSpaceToLocalSpace(bPointWorld);
    }

    abstract preSolve(dt: number): void;
    abstract solve(): void;
    abstract postSolve(): void;
}

export class JointConstraint extends Constraint {
    private cachedLambda: Vec2;
    private bias: Vec2;

    constructor(a: Body, b: Body, anchorPoint: Vec2) {
        super(a, b, anchorPoint, anchorPoint);

        this.cachedLambda = new Vec2(0, 0);
        this.bias = new Vec2(0, 0);
    }

    preSolve(dt: number): void {
        // World-space positions and lever arms
        const pa = this.a.localSpaceToWorldSpace(this.aPoint);
        const pb = this.b.localSpaceToWorldSpace(this.bPoint);
        const ra = pa.subNew(this.a.position);
        const rb = pb.subNew(this.b.position);

        // Warm starting (apply cached lambda)
        const lambda = this.cachedLambda;
        const impulseA = new Vec2(lambda.x, lambda.y);
        const angularImpulseA = -ra.y * lambda.x + ra.x * lambda.y;
        const impulseB = new Vec2(-lambda.x, -lambda.y);
        const angularImpulseB = rb.y * lambda.x - rb.x * lambda.y;

        this.a.applyImpulseLinear(impulseA);
        this.a.applyImpulseAngular(angularImpulseA);
        this.b.applyImpulseLinear(impulseB);
        this.b.applyImpulseAngular(angularImpulseB);

        // Compute the bias term (Baumgarte stabilization)
        const beta = 0.02; // Stabilization factor
        const C = pa.subNew(pb); // Positional error vector (Vec2)
        const C_len = C.magnitude();
        const slop = 0.5; // Small tolerance (e.g., 5mm)

        if (C_len > slop) {
            // Apply correction only if error is greater than slop
            const correction = Math.max(0, C_len - slop);
            const C_normalized = C.normalize();
            const bias_magnitude = (beta / dt) * correction;

            // Bias vector pointing in the direction of correction
            this.bias = C_normalized.scaleNew(bias_magnitude);
        } else {
            this.bias = new Vec2(0, 0);
        }
    }

    solve(): void {
        // Recompute ra and rb as positions may have changed slightly from warm start
        const pa = this.a.localSpaceToWorldSpace(this.aPoint);
        const pb = this.b.localSpaceToWorldSpace(this.bPoint);
        const ra = pa.subNew(this.a.position);
        const rb = pb.subNew(this.b.position);

        // Compute relative velocity
        const perpRa = new Vec2(-ra.y, ra.x);
        const perpRb = new Vec2(-rb.y, rb.x);
        const velAP = this.a.velocity.addNew(perpRa.scaleNew(this.a.angularVelocity));
        const velBP = this.b.velocity.addNew(perpRb.scaleNew(this.b.angularVelocity));
        const relVel = velAP.subNew(velBP);

        // b = -relVel - bias
        const bVec = relVel.scaleNew(-1).subNew(this.bias);

        // Compute effective mass components (K matrix as scalars)
        const invMa = this.a.invMass;
        const invMb = this.b.invMass;
        const invIa = this.a.invI;
        const invIb = this.b.invI;

        const kxx = invMa + invMb + ra.y * ra.y * invIa + rb.y * rb.y * invIb;
        const kyy = invMa + invMb + ra.x * ra.x * invIa + rb.x * rb.x * invIb;
        const kxy = -ra.x * ra.y * invIa + rb.x * rb.y * invIb;

        // Solve 2x2 system for delta lambda
        const det = kxx * kyy - kxy * kxy;
        let deltaLambda = new Vec2(0, 0);
        if (det !== 0) {
            const deltaX = (kyy * bVec.x - kxy * bVec.y) / det;
            const deltaY = (kxx * bVec.y - kxy * bVec.x) / det;
            deltaLambda = new Vec2(deltaX, deltaY);
        }

        // Accumulate lambda
        this.cachedLambda.addAssign(deltaLambda);

        // Apply impulses based on delta lambda
        const impulseA = new Vec2(deltaLambda.x, deltaLambda.y);
        const angularImpulseA = -ra.y * deltaLambda.x + ra.x * deltaLambda.y;
        const impulseB = new Vec2(-deltaLambda.x, -deltaLambda.y);
        const angularImpulseB = rb.y * deltaLambda.x - rb.x * deltaLambda.y;

        this.a.applyImpulseLinear(impulseA);
        this.a.applyImpulseAngular(angularImpulseA);
        this.b.applyImpulseLinear(impulseB);
        this.b.applyImpulseAngular(angularImpulseB);
    }

    postSolve(): void {
        // Clamp components of cached lambda
        const maxImpulse = 10000;
        this.cachedLambda.x = Utils.clamp(this.cachedLambda.x, -maxImpulse, maxImpulse);
        this.cachedLambda.y = Utils.clamp(this.cachedLambda.y, -maxImpulse, maxImpulse);
    }
}

export class PenetrationConstraint extends Constraint {
    private cachedLambdaNormal: number;
    private cachedLambdaFriction: number;
    private bias: number;
    private normal: Vec2; // Normal direction of the penetration in A's local space
    private friction: number; // Friction coefficient between the two penetrating bodies

    constructor(a: Body, b: Body, aCollisionPoint: Vec2, bCollisionPoint: Vec2, normal: Vec2) {
        super(a, b, aCollisionPoint, bCollisionPoint);
        this.normal = a.worldSpaceToLocalSpace(normal);

        this.cachedLambdaNormal = 0;
        this.cachedLambdaFriction = 0;
        this.bias = 0;
        this.friction = 0;
    }

    preSolve(dt: number): void {
        // Get the collision points and normal in world space
        const pa = this.a.localSpaceToWorldSpace(this.aPoint);
        const pb = this.b.localSpaceToWorldSpace(this.bPoint);
        const n = this.a.localSpaceToWorldSpace(this.normal); // The normal vector in world space

        const ra = pa.subNew(this.a.position);
        const rb = pb.subNew(this.b.position);

        // Set friction
        this.friction = Math.min(this.a.friction, this.b.friction);

        // Compute tangent if friction > 0
        let t = new Vec2(0, 0);
        if (this.friction > 0) {
            t = n.normal(); // Assuming normal() returns perpendicular vector, e.g., new Vec2(-n.y, n.x)
        }

        // Warm starting (apply cached lambda)
        const totalDir = n.scaleNew(this.cachedLambdaNormal).addNew(t.scaleNew(this.cachedLambdaFriction));
        const impulseA = totalDir.scaleNew(-1);
        const angularImpulseA = -ra.cross(totalDir);
        const impulseB = totalDir;
        const angularImpulseB = rb.cross(totalDir);

        this.a.applyImpulseLinear(impulseA);
        this.a.applyImpulseAngular(angularImpulseA);
        this.b.applyImpulseLinear(impulseB);
        this.b.applyImpulseAngular(angularImpulseB);

        // Compute the bias term (Baumgarte stabilization)
        const beta = 0.2;
        let C = pb.subNew(pa).dot(n.scaleNew(-1));
        C = Math.min(0.0, C + 0.01);

        // Calculate relative velocity
        const perpRa = new Vec2(-ra.y, ra.x);
        const perpRb = new Vec2(-rb.y, rb.x);
        const va = this.a.velocity.addNew(perpRa.scaleNew(this.a.angularVelocity));
        const vb = this.b.velocity.addNew(perpRb.scaleNew(this.b.angularVelocity));
        const relVel = va.subNew(vb);
        const vrelDotNormal = relVel.dot(n);

        // Get the restitution between the two bodies
        const e = Math.min(this.a.restitution, this.b.restitution);

        // Calculate bias term considering elasticity (restitution)
        this.bias = (beta / dt) * C + e * vrelDotNormal;
    }

    solve(): void {
        // Recompute ra, rb, n as positions/velocities may have changed
        const pa = this.a.localSpaceToWorldSpace(this.aPoint);
        const pb = this.b.localSpaceToWorldSpace(this.bPoint);
        const n = this.a.localSpaceToWorldSpace(this.normal);
        const ra = pa.subNew(this.a.position);
        const rb = pb.subNew(this.b.position);

        // Compute tangent if friction > 0
        let t = new Vec2(0, 0);
        if (this.friction > 0) {
            t = n.normal();
        }

        // Compute relative velocity
        const perpRa = new Vec2(-ra.y, ra.x);
        const perpRb = new Vec2(-rb.y, rb.x);
        const velAP = this.a.velocity.addNew(perpRa.scaleNew(this.a.angularVelocity));
        const velBP = this.b.velocity.addNew(perpRb.scaleNew(this.b.angularVelocity));
        const relVel = velAP.subNew(velBP);

        const relVelN = relVel.dot(n);
        const relVelT = this.friction > 0 ? relVel.dot(t) : 0;

        // Compute effective mass components
        const invMa = this.a.invMass;
        const invMb = this.b.invMass;
        const invIa = this.a.invI;
        const invIb = this.b.invI;

        const crossRaN = ra.cross(n);
        const crossRbN = rb.cross(n);
        const knn = invMa + invMb + crossRaN * crossRaN * invIa + crossRbN * crossRbN * invIb;

        let ktt = 0;
        let knt = 0;
        let crossRaT = 0;
        let crossRbT = 0;
        if (this.friction > 0) {
            crossRaT = ra.cross(t);
            crossRbT = rb.cross(t);
            ktt = invMa + invMb + crossRaT * crossRaT * invIa + crossRbT * crossRbT * invIb;
            knt = crossRaN * crossRaT * invIa + crossRbN * crossRbT * invIb;
        }

        // Compute rhs
        const rhsN = relVelN - this.bias;
        const rhsT = relVelT;

        // Solve for delta lambda
        let deltaLambdaN = 0;
        let deltaLambdaT = 0;
        const oldLambdaN = this.cachedLambdaNormal;
        const oldLambdaT = this.cachedLambdaFriction;

        if (this.friction > 0) {
            // 2x2 solve
            const det = knn * ktt - knt * knt;
            if (det !== 0) {
                deltaLambdaN = (ktt * rhsN - knt * rhsT) / det;
                deltaLambdaT = (knn * rhsT - knt * rhsN) / det;
            }
        } else {
            // 1D solve for normal only
            if (knn !== 0) {
                deltaLambdaN = rhsN / knn;
            }
        }

        // Accumulate and clamp
        this.cachedLambdaNormal += deltaLambdaN;
        this.cachedLambdaFriction += deltaLambdaT;

        this.cachedLambdaNormal = Math.max(0, this.cachedLambdaNormal);

        if (this.friction > 0) {
            const maxFriction = this.cachedLambdaNormal * this.friction;
            this.cachedLambdaFriction = Utils.clamp(this.cachedLambdaFriction, -maxFriction, maxFriction);
        }

        // Compute effective delta after clamping
        deltaLambdaN = this.cachedLambdaNormal - oldLambdaN;
        deltaLambdaT = this.cachedLambdaFriction - oldLambdaT;

        // Apply impulses
        const totalDir = n.scaleNew(deltaLambdaN).addNew(t.scaleNew(deltaLambdaT));
        const impulseA = totalDir.scaleNew(-1);
        const angularImpulseA = -ra.cross(totalDir);
        const impulseB = totalDir;
        const angularImpulseB = rb.cross(totalDir);

        this.a.applyImpulseLinear(impulseA);
        this.a.applyImpulseAngular(angularImpulseA);
        this.b.applyImpulseLinear(impulseB);
        this.b.applyImpulseAngular(angularImpulseB);
    }

    postSolve(): void {
        // TODO: to be implemented
    }
}
