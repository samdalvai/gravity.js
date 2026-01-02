import Mat22 from '../math/Mat22';
import Utils from '../math/Utils';
import Vec2 from '../math/Vec2';
import Body from './Body';
import { GRAVITY, PIXELS_PER_METER } from './Constants';

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
        const pA = this.a.position.addNew(this.rA);
        const pB = this.b.position.addNew(this.rB);
        const relPos = pB.subNew(pA);

        this.bias = relPos.scaleNew(-this.biasFactor * invDt);

        // ---- Warm starting ----
        this.a.velocity.sub(this.cachedLambda.scaleNew(this.a.invMass));
        this.a.angularVelocity -= this.a.invI * this.rA.cross(this.cachedLambda);

        this.b.velocity.add(this.cachedLambda.scaleNew(this.b.invMass));
        this.b.angularVelocity += this.b.invI * this.rB.cross(this.cachedLambda);
    }

    solve(): void {
        const vA = this.a.velocity.addNew(this.rA.crossScalar(this.a.angularVelocity));
        const vB = this.b.velocity.addNew(this.rB.crossScalar(this.b.angularVelocity));

        const dv = vB.subNew(vA);

        const impulse = Mat22.multiply(this.M, this.bias.subNew(dv).subNew(this.cachedLambda.scaleNew(this.softness)));

        this.a.velocity.sub(impulse.scaleNew(this.a.invMass));
        this.a.angularVelocity -= this.a.invI * this.rA.cross(impulse);

        this.b.velocity.add(impulse.scaleNew(this.b.invMass));
        this.b.angularVelocity += this.b.invI * this.rB.cross(impulse);

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
    normal: Vec2;
    depth: number;

    // rA / rB (world space)
    private rAx = 0;
    private rAy = 0;
    private rBx = 0;
    private rBy = 0;

    private tangentX = 0;
    private tangentY = 0;

    // Effective mass
    private normalMass = 0;
    private tangentMass = 0;

    // Bias terms
    private bias = 0;
    private restitutionBias = 0;

    // Warm starting
    private normalImpulse = 0;
    private tangentImpulse = 0;

    constructor(a: Body, b: Body, start: Vec2, end: Vec2, normalWorld: Vec2, depth: number) {
        super(a, b, start, end);

        // Ensure normal always points A → B
        this.normal = normalWorld.negate();
        this.depth = depth;
    }

    preSolve(invDt: number): void {
        const a = this.a;
        const b = this.b;

        // World-space contact points
        const cosA = Math.cos(a.rotation);
        const sinA = Math.sin(a.rotation);
        const pAx = this.aPoint.x * cosA - this.aPoint.y * sinA + a.position.x;
        const pAy = this.aPoint.x * sinA + this.aPoint.y * cosA + a.position.y;

        const cosB = Math.cos(b.rotation);
        const sinB = Math.sin(b.rotation);
        const pBx = this.bPoint.x * cosB - this.bPoint.y * sinB + b.position.x;
        const pBy = this.bPoint.x * sinB + this.bPoint.y * cosB + b.position.y;

        this.rAx = pAx - a.position.x;
        this.rAy = pAy - a.position.y;
        this.rBx = pBx - b.position.x;
        this.rBy = pBy - b.position.y;

        // Tangent
        this.tangentX = this.normal.y;
        this.tangentY = -this.normal.x;

        // Effective mass (normal)
        const rnA = this.rAx * this.normal.y - this.rAy * this.normal.x;
        const rnB = this.rBx * this.normal.y - this.rBy * this.normal.x;

        this.normalMass = 1 / (a.invMass + b.invMass + rnA * rnA * a.invI + rnB * rnB * b.invI);

        // Effective mass (tangent)
        const rtA = this.rAx * this.tangentY - this.rAy * this.tangentX;
        const rtB = this.rBx * this.tangentY - this.rBy * this.tangentX;

        this.tangentMass = 1 / (a.invMass + b.invMass + rtA * rtA * a.invI + rtB * rtB * b.invI);

        // Baumgarte stabilization (penetration → velocity)
        const slop = 0.01;
        const beta = 0.2;

        this.bias = Math.max(this.depth - slop, 0) * beta * invDt;

        // Restitution (bounce only if fast enough)
        const vAx = a.velocity.x + -a.angularVelocity * this.rAy;
        const vAy = a.velocity.y + a.angularVelocity * this.rAx;
        const vBx = b.velocity.x + -b.angularVelocity * this.rBy;
        const vBy = b.velocity.y + b.angularVelocity * this.rBx;

        const vRelx = vAx - vBx;
        const vRely = vAy - vBy;

        const vn = vRelx * this.normal.x + vRely * this.normal.y;

        const e = Math.min(a.restitution, b.restitution);

        const restitutionSlop = 10;
        this.restitutionBias = vn < -restitutionSlop ? -e * vn : 0;

        // Warm starting
        const px = this.normal.x * this.normalImpulse + this.tangentX * this.tangentImpulse;
        const py = this.normal.y * this.normalImpulse + this.tangentY * this.tangentImpulse;

        a.velocity.x += px * a.invMass;
        a.velocity.y += py * a.invMass;
        a.angularVelocity += this.rAx * py - this.rAy * px;

        b.velocity.x += -px * b.invMass;
        b.velocity.y += -py * b.invMass;
        b.angularVelocity += this.rBx * -py - this.rBy * -px;
    }

    solve(): void {
        const a = this.a;
        const b = this.b;

        const vAx = a.velocity.x + -a.angularVelocity * this.rAy;
        const vAy = a.velocity.y + a.angularVelocity * this.rAx;
        const vBx = b.velocity.x + -b.angularVelocity * this.rBy;
        const vBy = b.velocity.y + b.angularVelocity * this.rBx;

        const vRelx = vAx - vBx;
        const vRely = vAy - vBy;

        // Normal impulse
        const vn = vRelx * this.normal.x + vRely * this.normal.y;

        let dPn = this.normalMass * (-vn + this.bias + this.restitutionBias);

        const oldPn = this.normalImpulse;
        this.normalImpulse = Math.max(oldPn + dPn, 0);
        dPn = this.normalImpulse - oldPn;

        const Pnx = this.normal.x * dPn;
        const Pny = this.normal.y * dPn;

        a.velocity.x += Pnx * a.invMass;
        a.velocity.y += Pny * a.invMass;
        a.angularVelocity += (this.rAx * Pny - this.rAy * Pnx) * a.invI;

        b.velocity.x += -Pnx * b.invMass;
        b.velocity.y += -Pny * b.invMass;
        b.angularVelocity += (this.rBx * -Pny - this.rBy * -Pnx) * b.invI;

        // Friction impulse
        const vt = vRelx * this.tangentX + vRely * this.tangentY;

        let dPt = -vt * this.tangentMass;

        const mu = Math.min(a.friction, b.friction);
        const maxPt = mu * this.normalImpulse;

        const oldPt = this.tangentImpulse;
        this.tangentImpulse = Utils.clamp(oldPt + dPt, -maxPt, maxPt);
        dPt = this.tangentImpulse - oldPt;

        const Ptx = this.tangentX * dPt;
        const Pty = this.tangentY * dPt;

        a.velocity.x += Ptx * a.invMass;
        a.velocity.y += Pty * a.invMass;
        a.angularVelocity += (this.rAx * Pty - this.rAy * Ptx) * a.invI;

        b.velocity.x += -Ptx * b.invMass;
        b.velocity.y += -Pty * b.invMass;
        b.angularVelocity += (this.rBx * -Pty - this.rBy * -Ptx) * b.invI;
    }

    postSolve(): void {
        // Optional: clamp cached impulses to prevent blow-up
        const maxImpulse = 1e6;
        this.normalImpulse = Math.min(this.normalImpulse, maxImpulse);
        this.tangentImpulse = Utils.clamp(this.tangentImpulse, -maxImpulse, maxImpulse);
    }
}
