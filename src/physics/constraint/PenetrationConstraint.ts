import MatMN from '../../math/MatMN';
import Utils from '../../math/Utils';
import Vec2 from '../../math/Vec2';
import VecN from '../../math/VecN';
import Body from '../body/Body';
import Constraint from './Constraint';

export default class PenetrationConstraint extends Constraint {
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
