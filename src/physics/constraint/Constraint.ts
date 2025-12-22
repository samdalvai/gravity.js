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

export class JointConstraint {
    body1: Body;
    body2: Body;

    localAnchor1: Vec2;
    localAnchor2: Vec2;

    M: Mat22;
    r1: Vec2;
    r2: Vec2;

    bias: Vec2;
    P: Vec2; // accumulated impulse
    biasFactor: number;
    softness: number;

    constructor(b1: Body, b2: Body, anchor: Vec2, softness = 0.01, biasFactor = 0.2) {
        this.body1 = b1;
        this.body2 = b2;

        const Rot1 = new Mat22(this.body1.rotation);
        const Rot2 = new Mat22(this.body2.rotation);
        const Rot1T = Rot1.transpose();
        const Rot2T = Rot2.transpose();

        this.localAnchor1 = Mat22.multiply(Rot1T, anchor.subNew(this.body1.position));
        this.localAnchor2 = Mat22.multiply(Rot2T, anchor.subNew(this.body2.position));

        this.M = new Mat22();
        this.r1 = new Vec2();
        this.r2 = new Vec2();
        this.bias = new Vec2();
        this.P = new Vec2();

        // Is 0.01 as softness a good default?
        this.softness = softness;
        this.biasFactor = biasFactor;
    }

    preSolve = (dt: number): void => {
        // Pre-compute anchors, mass matrix, and bias.
        const Rot1 = new Mat22(this.body1.rotation);
        const Rot2 = new Mat22(this.body2.rotation);

        this.r1 = Mat22.multiply(Rot1, this.localAnchor1);
        this.r2 = Mat22.multiply(Rot2, this.localAnchor2);

        // deltaV = deltaV0 + K * impulse
        // invM = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
        //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
        //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
        const K1 = new Mat22();
        K1.col1.x = this.body1.invMass + this.body2.invMass;
        K1.col1.y = 0.0;
        K1.col2.x = 0.0;
        K1.col2.y = this.body1.invMass + this.body2.invMass;

        const K2 = new Mat22();
        K2.col1.x = this.body1.invI * this.r1.y * this.r1.y;
        K2.col1.y = -this.body1.invI * this.r1.x * this.r1.y;
        K2.col2.x = -this.body1.invI * this.r1.x * this.r1.y;
        K2.col2.y = this.body1.invI * this.r1.x * this.r1.x;

        const K3 = new Mat22();
        K3.col1.x = this.body2.invI * this.r2.y * this.r2.y;
        K3.col1.y = -this.body2.invI * this.r2.x * this.r2.y;
        K3.col2.x = -this.body2.invI * this.r2.x * this.r2.y;
        K3.col2.y = this.body2.invI * this.r2.x * this.r2.x;

        const K = Mat22.add(Mat22.add(K1, K2), K3);
        K.col1.x += this.softness;
        K.col2.y += this.softness;

        this.M = K.invert();

        const p1 = Vec2.add(this.body1.position, this.r1);
        const p2 = Vec2.add(this.body2.position, this.r2);
        const dp = Vec2.sub(p2, p1);

        this.bias = Vec2.scale(-this.biasFactor * 1 / dt, dp);

        // Apply accumulated impulse.
        this.body1.velocity.sub(Vec2.scale(this.body1.invMass, this.P));
        this.body1.angularVelocity -= this.body1.invI * Vec2.cross(this.r1, this.P);

        this.body2.velocity.add(Vec2.scale(this.body2.invMass, this.P));
        this.body2.angularVelocity += this.body2.invI * Vec2.cross(this.r2, this.P);
    };

    solve = (): void => {
        // Linear velocity of the center of mass of body 1 and 2.
        const lv1 = Vec2.add(this.body1.velocity, Vec2.cross(this.body1.angularVelocity, this.r1));
        const lv2 = Vec2.add(this.body2.velocity, Vec2.cross(this.body2.angularVelocity, this.r2));

        // Relative velocity at contact
        const dv = Vec2.sub(lv2, lv1);

        const impulse = Mat22.multiply(this.M, Vec2.sub(Vec2.sub(this.bias, dv), Vec2.scale(this.softness, this.P)));

        this.body1.velocity.sub(Vec2.scale(this.body1.invMass, impulse));
        this.body1.angularVelocity -= this.body1.invI * Vec2.cross(this.r1, impulse);

        this.body2.velocity.add(Vec2.scale(this.body2.invMass, impulse));
        this.body2.angularVelocity += this.body2.invI * Vec2.cross(this.r2, impulse);

        this.P.add(impulse);
    };
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
        let c = pb.subNew(pa).dot(n.scaleNew(-1));
        c = Math.min(0.0, c + 0.01);

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
        this.bias = (beta / dt) * c + e * vrelDotNormal;
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
