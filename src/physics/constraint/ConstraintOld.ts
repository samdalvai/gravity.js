import MatMN from '../../math/MatMN';
import Utils from '../../math/Utils';
import Vec2 from '../../math/Vec2';
import VecN from '../../math/VecN';
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

    ///////////////////////////////////////////////////////////////////////////////
    // Mat6x6 with the all inverse mass and inverse I of bodies "a" and "b"
    ///////////////////////////////////////////////////////////////////////////////
    //  [ 1/ma  0     0     0     0     0    ]
    //  [ 0     1/ma  0     0     0     0    ]
    //  [ 0     0     1/Ia  0     0     0    ]
    //  [ 0     0     0     1/mb  0     0    ]
    //  [ 0     0     0     0     1/mb  0    ]
    //  [ 0     0     0     0     0     1/Ib ]
    ///////////////////////////////////////////////////////////////////////////////
    getInvM = (): MatMN => {
        const invM = new MatMN(6, 6);
        invM.zero();

        invM.rows[0].set(0, this.a.invMass);
        invM.rows[1].set(1, this.a.invMass);
        invM.rows[2].set(2, this.a.invI);

        invM.rows[3].set(3, this.b.invMass);
        invM.rows[4].set(4, this.b.invMass);
        invM.rows[5].set(5, this.b.invI);

        return invM;
    };

    ///////////////////////////////////////////////////////////////////////////////
    // VecN with the all linear and angular velocities of bodies "a" and "b"
    ///////////////////////////////////////////////////////////////////////////////
    //  [ va.x ]
    //  [ va.y ]
    //  [ ωa   ]
    //  [ vb.x ]
    //  [ vb.y ]
    //  [ ωb   ]
    ///////////////////////////////////////////////////////////////////////////////
    getVelocities = (): VecN => {
        const V = new VecN(6);
        V.zero();

        V.set(0, this.a.velocity.x);
        V.set(1, this.a.velocity.y);
        V.set(2, this.a.angularVelocity);
        V.set(3, this.b.velocity.x);
        V.set(4, this.b.velocity.y);
        V.set(5, this.b.angularVelocity);

        return V;
    };

    abstract preSolve(dt: number): void;
    abstract solve(): void;
    abstract postSolve(): void;
}

export class JointConstraint extends Constraint {
    // Dimensions updated: Jacobian is 2x6, CachedLambda and Bias are 2x1 (VecN)
    private jacobian: MatMN;
    private cachedLambda: VecN;
    private bias: VecN; // Changed from number to VecN (2 elements)

    constructor(a: Body, b: Body, anchorPoint: Vec2) {
        super(a, b, anchorPoint, anchorPoint);

        // J must be 2 rows (for X and Y constraints) and 6 columns
        this.jacobian = new MatMN(2, 6);
        this.cachedLambda = new VecN(2); // Lambda is now a 2D vector
        this.bias = new VecN(2); // Bias is now a 2D vector

        this.cachedLambda.zero();
        this.bias.zero();
    }

    preSolve(dt: number): void {
        // World-space positions and lever arms
        const pa = this.a.localSpaceToWorldSpace(this.aPoint);
        const pb = this.b.localSpaceToWorldSpace(this.bPoint);
        const ra = pa.subNew(this.a.position);
        const rb = pb.subNew(this.b.position);

        // Compute the Jacobian J (2x6) for C = pa - pb = 0
        this.jacobian.zero();

        // J = [ I  ra_perp  -I  -rb_perp ]
        // where ra_perp = (-ra.y, ra.x)

        // Row 0: Constraint X (enforces pa.x - pb.x = 0)
        // [ 1, 0, -ra.y, -1, 0, rb.y ]
        this.jacobian.rows[0].set(0, 1); // A linear x (vax)
        this.jacobian.rows[0].set(1, 0); // A linear y (vay)
        this.jacobian.rows[0].set(2, -ra.y); // A angular (wa) (ra_perp.x)

        this.jacobian.rows[0].set(3, -1); // B linear x (vbx)
        this.jacobian.rows[0].set(4, 0); // B linear y (vby)
        this.jacobian.rows[0].set(5, rb.y); // B angular (wb) (-rb_perp.x)

        // Row 1: Constraint Y (enforces pa.y - pb.y = 0)
        // [ 0, 1, ra.x, 0, -1, -rb.x ]
        this.jacobian.rows[1].set(0, 0); // A linear x (vax)
        this.jacobian.rows[1].set(1, 1); // A linear y (vay)
        this.jacobian.rows[1].set(2, ra.x); // A angular (wa) (ra_perp.y)

        this.jacobian.rows[1].set(3, 0); // B linear x (vbx)
        this.jacobian.rows[1].set(4, -1); // B linear y (vby)
        this.jacobian.rows[1].set(5, -rb.x); // B angular (wb) (-rb_perp.y)

        // Warm starting (apply cached lambda)
        const Jt = this.jacobian.transpose();
        const impulses = Jt.multiplyVec(this.cachedLambda); // impulses is VecN (6)

        // Apply the impulses to both bodies
        this.a.applyImpulseLinear(new Vec2(impulses.get(0), impulses.get(1)));
        this.a.applyImpulseAngular(impulses.get(2));
        this.b.applyImpulseLinear(new Vec2(impulses.get(3), impulses.get(4)));
        this.b.applyImpulseAngular(impulses.get(5));

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
            const biasVec = C_normalized.scaleNew(bias_magnitude);

            this.bias.set(0, biasVec.x);
            this.bias.set(1, biasVec.y);
        } else {
            this.bias.zero();
        }
    }

    solve(): void {
        const V = this.getVelocities(); // V is VecN (6)
        const invM = this.getInvM(); // invM is MatMN (6x6)
        const J = this.jacobian; // J is MatMN (2x6)
        const Jt = this.jacobian.transpose();

        // Compute effective mass K (K = J * invM * Jt)
        const K = J.multiplyMat(invM).multiplyMat(Jt); // K is MatMN (2x2)

        // Compute the right-hand side b (b = -J * V - bias)
        const b = J.multiplyVec(V).scaleNew(-1); // b is VecN (2)

        // b = -J * V - bias
        b.subAssign(this.bias);

        // Solve for lambda (K * lambda = b)
        // Since K is 2x2, MatMN.solveGaussSeidel should handle it,
        // but often an explicit 2x2 inverse or Cramer's rule is used for performance.
        // Assuming MatMN.solveGaussSeidel works for 2x2:
        const lambda = MatMN.solveGaussSeidel(K, b); // lambda is VecN (2)
        this.cachedLambda.addAssign(lambda);

        // Compute and apply impulses (Impulse = Jt * lambda)
        const impulses = Jt.multiplyVec(lambda); // impulses is VecN (6)

        // Apply the impulses to both bodies
        this.a.applyImpulseLinear(new Vec2(impulses.get(0), impulses.get(1)));
        this.a.applyImpulseAngular(impulses.get(2));
        this.b.applyImpulseLinear(new Vec2(impulses.get(3), impulses.get(4)));
        this.b.applyImpulseAngular(impulses.get(5));
    }

    postSolve(): void {
        // Limit the warm starting (clamping a 2D vector's magnitude is better)
        // For simplicity, we clamp the components, but a magnitude clamp is technically better.
        const maxImpulse = 10000;
        this.cachedLambda.set(0, Utils.clamp(this.cachedLambda.get(0), -maxImpulse, maxImpulse));
        this.cachedLambda.set(1, Utils.clamp(this.cachedLambda.get(1), -maxImpulse, maxImpulse));
    }
}

export class PenetrationConstraint extends Constraint {
    private jacobian: MatMN;
    private cachedLambda: VecN;
    private bias: number;
    private normal: Vec2; // Normal direction of the penetration in A's local space
    private friction: number; // Friction coefficient between the two penetrating bodies

    constructor(a: Body, b: Body, aCollisionPoint: Vec2, bCollisionPoint: Vec2, normal: Vec2) {
        super(a, b, aCollisionPoint, bCollisionPoint);
        this.normal = a.worldSpaceToLocalSpace(normal);

        this.jacobian = new MatMN(2, 6);
        this.cachedLambda = new VecN(2);
        this.bias = 0;
        this.friction = 0;

        this.cachedLambda.zero();
    }

    preSolve(dt: number): void {
        // Get the collision points and normal in world space
        const pa = this.a.localSpaceToWorldSpace(this.aPoint);
        const pb = this.b.localSpaceToWorldSpace(this.bPoint);
        const n = this.a.localSpaceToWorldSpace(this.normal); // The normal vector in world space

        const ra = pa.subNew(this.a.position);
        const rb = pb.subNew(this.b.position);

        this.jacobian.zero();

        // Populate the first row of the Jacobian (normal vector)
        this.jacobian.rows[0].set(0, -n.x); // A linear velocity.x
        this.jacobian.rows[0].set(1, -n.y); // A linear velocity.y
        this.jacobian.rows[0].set(2, -ra.cross(n)); // A angular velocity
        this.jacobian.rows[0].set(3, n.x); // B linear velocity.x
        this.jacobian.rows[0].set(4, n.y); // B linear velocity.y
        this.jacobian.rows[0].set(5, rb.cross(n)); // B angular velocity

        // Populate the second row of the Jacobian (tangent vector)
        this.friction = Math.min(this.a.friction, this.b.friction);
        if (this.friction > 0.0) {
            const t = n.normal(); // The tangent is the vector perpendicular to the normal

            this.jacobian.rows[1].set(0, -t.x); // A linear velocity.x
            this.jacobian.rows[1].set(1, -t.y); // A linear velocity.y
            this.jacobian.rows[1].set(2, -ra.cross(t)); // A angular velocity
            this.jacobian.rows[1].set(3, t.x); // B linear velocity.x
            this.jacobian.rows[1].set(4, t.y); // B linear velocity.y
            this.jacobian.rows[1].set(5, rb.cross(t)); // B angular velocity
        }

        // Warm starting (apply cached lambda)
        const Jt = this.jacobian.transpose();
        const impulses = Jt.multiplyVec(this.cachedLambda);

        // Apply the impulses to both bodies
        this.a.applyImpulseLinear(new Vec2(impulses.get(0), impulses.get(1))); // A linear impulse
        this.a.applyImpulseAngular(impulses.get(2)); // A angular impulse
        this.b.applyImpulseLinear(new Vec2(impulses.get(3), impulses.get(4))); // B linear impulse
        this.b.applyImpulseAngular(impulses.get(5)); // B angular impulse

        // Compute the bias term (baumgarte stabilization)
        const beta = 0.2;
        let C = pb.subNew(pa).dot(n.scaleNew(-1));
        C = Math.min(0.0, C + 0.01);

        // Calculate relative velocity pre-impulse normal, which will be used to compute elasticity
        const va = this.a.velocity.addNew(new Vec2(-this.a.angularVelocity * ra.y, this.a.angularVelocity * ra.x));
        const vb = this.b.velocity.addNew(new Vec2(-this.b.angularVelocity * rb.y, this.b.angularVelocity * rb.x));
        const vrelDotNormal = va.subNew(vb).dot(n);

        // Get the restitution between the two bodies
        const e = Math.min(this.a.restitution, this.b.restitution);

        // Calculate bias term considering elasticity (restitution)
        this.bias = (beta / dt) * C + e * vrelDotNormal;
    }

    solve(): void {
        const V = this.getVelocities();
        const invM = this.getInvM();

        const J = this.jacobian;
        const Jt = this.jacobian.transpose();

        // Compute lambda using Ax=b (Gauss-Seidel method)
        const lhs = J.multiplyMat(invM).multiplyMat(Jt); // A
        const rhs = J.multiplyVec(V.scaleNew(-1)); // b
        rhs.set(0, rhs.get(0) - this.bias);

        let lambda = MatMN.solveGaussSeidel(lhs, rhs);

        // Accumulate impulses and clamp it within constraint limits
        const oldLambda = VecN.from(this.cachedLambda);
        this.cachedLambda.addAssign(lambda);
        // Clamp normal impulse to be non-negative
        this.cachedLambda.set(0, Math.max(0, this.cachedLambda.get(0)));

        // Keep friction values between -(λn*µ) and +(λn*µ)
        if (this.friction > 0) {
            const maxFriction = this.cachedLambda.get(0) * this.friction;
            const clampedFriction = Utils.clamp(this.cachedLambda.get(1), -maxFriction, maxFriction);
            this.cachedLambda.set(1, clampedFriction);
        }

        // Compute the change in lambda to get the actual impulses to apply
        lambda = this.cachedLambda.subNew(oldLambda);

        // Compute the impulses with both direction and magnitude
        const impulses = Jt.multiplyVec(lambda);

        // Apply the impulses to both bodies
        this.a.applyImpulseLinear(new Vec2(impulses.get(0), impulses.get(1))); // A linear impulse
        this.a.applyImpulseAngular(impulses.get(2)); // A angular impulse
        this.b.applyImpulseLinear(new Vec2(impulses.get(3), impulses.get(4))); // B linear impulse
        this.b.applyImpulseAngular(impulses.get(5)); // B angular impulse
    }

    postSolve(): void {
        // TODO: to be implemented
    }
}
