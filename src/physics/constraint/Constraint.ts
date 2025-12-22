import Mat22 from '../../math/Mat22';
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
    M: Mat22;

    rA: Vec2;
    rB: Vec2;

    bias: Vec2;
    P: Vec2; // accumulated impulse

    biasFactor: number;
    softness: number;

    constructor(a: Body, b: Body, anchorWorld: Vec2, softness = 0.01, biasFactor = 0.2) {
        super(a, b, anchorWorld, anchorWorld);

        this.M = new Mat22();

        this.rA = new Vec2();
        this.rB = new Vec2();

        this.bias = new Vec2();
        this.P = new Vec2();

        this.softness = softness;
        this.biasFactor = biasFactor;
    }

    preSolve(dt: number): void {
        const pa = this.a.localSpaceToWorldSpace(this.aPoint);
        const pb = this.b.localSpaceToWorldSpace(this.bPoint);
        this.rA = pa.subNew(this.a.position);
        this.rB = pb.subNew(this.b.position);

        // ---- Effective mass matrix ----
        const K1 = new Mat22();
        K1.col1.x = this.a.invMass + this.b.invMass;
        K1.col1.y = 0;
        K1.col2.x = 0;
        K1.col2.y = this.a.invMass + this.b.invMass;

        const K2 = new Mat22();
        K2.col1.x = this.a.invI * this.rA.y * this.rA.y;
        K2.col1.y = -this.a.invI * this.rA.x * this.rA.y;
        K2.col2.x = -this.a.invI * this.rA.x * this.rA.y;
        K2.col2.y = this.a.invI * this.rA.x * this.rA.x;

        const K3 = new Mat22();
        K3.col1.x = this.b.invI * this.rB.y * this.rB.y;
        K3.col1.y = -this.b.invI * this.rB.x * this.rB.y;
        K3.col2.x = -this.b.invI * this.rB.x * this.rB.y;
        K3.col2.y = this.b.invI * this.rB.x * this.rB.x;

        const K = Mat22.add(Mat22.add(K1, K2), K3);

        // softness (CFM)
        K.col1.x += this.softness;
        K.col2.y += this.softness;

        this.M = K.invert();

        // ---- Bias (position correction) ----
        const pA = Vec2.add(this.a.position, this.rA);
        const pB = Vec2.add(this.b.position, this.rB);
        const C = Vec2.sub(pB, pA);

        this.bias = Vec2.scale(-this.biasFactor / dt, C);

        // ---- Warm starting ----
        this.a.velocity.sub(Vec2.scale(this.a.invMass, this.P));
        this.a.angularVelocity -= this.a.invI * Vec2.cross(this.rA, this.P);

        this.b.velocity.add(Vec2.scale(this.b.invMass, this.P));
        this.b.angularVelocity += this.b.invI * Vec2.cross(this.rB, this.P);
    }

    solve(): void {
        const vA = Vec2.add(this.a.velocity, Vec2.cross(this.a.angularVelocity, this.rA));
        const vB = Vec2.add(this.b.velocity, Vec2.cross(this.b.angularVelocity, this.rB));

        const dv = Vec2.sub(vB, vA);

        const impulse = Mat22.multiply(this.M, Vec2.sub(Vec2.sub(this.bias, dv), Vec2.scale(this.softness, this.P)));

        this.a.velocity.sub(Vec2.scale(this.a.invMass, impulse));
        this.a.angularVelocity -= this.a.invI * Vec2.cross(this.rA, impulse);

        this.b.velocity.add(Vec2.scale(this.b.invMass, impulse));
        this.b.angularVelocity += this.b.invI * Vec2.cross(this.rB, impulse);

        this.P.add(impulse);
    }

    postSolve(): void {
        // Optional: clamp accumulated impulse (recommended for stability)
        const maxImpulse = 10000;
        const mag = this.P.magnitude();
        if (mag > maxImpulse) {
            this.P.scaleAssign(maxImpulse / mag);
        }
    }
}

export class PenetrationConstraint extends Constraint {
    cachedLambdaNormal: number;
    cachedLambdaFriction: number;
    bias: number;
    normal: Vec2; // Normal direction of the penetration in A's local space
    friction: number; // Friction coefficient between the two penetrating bodies

    rA: Vec2;
    rB: Vec2;

    constructor(a: Body, b: Body, aCollisionPoint: Vec2, bCollisionPoint: Vec2, normal: Vec2) {
        super(a, b, aCollisionPoint, bCollisionPoint);
        this.normal = a.worldSpaceToLocalSpace(normal);

        this.cachedLambdaNormal = 0;
        this.cachedLambdaFriction = 0;
        this.bias = 0;
        this.friction = 0;

        this.rA = new Vec2();
        this.rB = new Vec2();
    }

    preSolve(dt: number): void {
        // Get the collision points and normal in world space
        const pa = this.a.localSpaceToWorldSpace(this.aPoint);
        const pb = this.b.localSpaceToWorldSpace(this.bPoint);
        const n = this.a.localSpaceToWorldSpace(this.normal); // The normal vector in world space

        this.rA = pa.subNew(this.a.position);
        this.rB = pb.subNew(this.b.position);

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
        const angularImpulseA = -this.rA.cross(totalDir);
        const impulseB = totalDir;
        const angularImpulseB = this.rB.cross(totalDir);

        this.a.applyImpulseLinear(impulseA);
        this.a.applyImpulseAngular(angularImpulseA);
        this.b.applyImpulseLinear(impulseB);
        this.b.applyImpulseAngular(angularImpulseB);

        // Compute the bias term (Baumgarte stabilization)
        const beta = 0.2;
        let c = pb.subNew(pa).dot(n.scaleNew(-1));
        c = Math.min(0.0, c + 0.01);

        // Calculate relative velocity
        const perpRa = new Vec2(-this.rA.y, this.rA.x);
        const perpRb = new Vec2(-this.rB.y, this.rB.x);
        const va = this.a.velocity.addNew(perpRa.scaleNew(this.a.angularVelocity));
        const vb = this.b.velocity.addNew(perpRb.scaleNew(this.b.angularVelocity));
        const relVel = va.subNew(vb);
        const vrelDotNormal = relVel.dot(n);

        // Get the restitution between the two bodies
        const e = Math.min(this.a.restitution, this.b.restitution);

        // Calculate bias term considering elasticity (restitution)
        this.bias = (beta / dt) * c + e * vrelDotNormal;
    }

    solve(): void {
        // Recompute ra, rb, n as positions/velocities may have changed
        const n = this.a.localSpaceToWorldSpace(this.normal);

        // Compute tangent if friction > 0
        let t = new Vec2(0, 0);
        if (this.friction > 0) {
            t = n.normal();
        }

        // Compute relative velocity
        const perpRa = new Vec2(-this.rA.y, this.rA.x);
        const perpRb = new Vec2(-this.rB.y, this.rB.x);
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

        const crossRaN = this.rA.cross(n);
        const crossRbN = this.rB.cross(n);
        const knn = invMa + invMb + crossRaN * crossRaN * invIa + crossRbN * crossRbN * invIb;

        let ktt = 0;
        let knt = 0;
        let crossRaT = 0;
        let crossRbT = 0;
        if (this.friction > 0) {
            crossRaT = this.rA.cross(t);
            crossRbT = this.rB.cross(t);
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
        const angularImpulseA = -this.rA.cross(totalDir);
        const impulseB = totalDir;
        const angularImpulseB = this.rB.cross(totalDir);

        this.a.applyImpulseLinear(impulseA);
        this.a.applyImpulseAngular(angularImpulseA);
        this.b.applyImpulseLinear(impulseB);
        this.b.applyImpulseAngular(angularImpulseB);
    }

    postSolve(): void {
        // TODO: to be implemented
    }
}
