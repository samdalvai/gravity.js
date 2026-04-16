import { SETTINGS } from '../core/Constants';
import { RigidBody } from '../core/RigidBody';
import { Mat3 } from '../math/Mat3';
import { Vec2 } from '../math/Vec2';
import { Vec3 } from '../math/Vec3';
import { Joint } from './Joint';

// Revolute joint + Angle joint
export class WeldJoint extends Joint {
    public localAnchorA: Vec2;
    public localAnchorB: Vec2;

    public initialAngleOffset: number;

    private ra!: Vec2;
    private rb!: Vec2;
    private m!: Mat3;
    private bias!: Vec3;
    private impulseSum: Vec3 = new Vec3();

    constructor(
        bodyA: RigidBody,
        bodyB: RigidBody,
        anchor: Vec2 = bodyA.position.lerp(bodyB.position, 0.5),
        frequency = -1,
        dampingRatio = 1.0,
        jointMass = -1,
    ) {
        super(bodyA, bodyB, frequency, dampingRatio, jointMass);

        this.initialAngleOffset = bodyB.rotation - bodyA.rotation;
        this.localAnchorA = this.bodyA.worldPointToLocal(anchor);
        this.localAnchorB = this.bodyB.worldPointToLocal(anchor);

        this.drawAnchor = false;
        this.drawConnectionLine = false;
    }

    override preSolve(invDt: number) {
        // Calculate Jacobian J and effective mass M
        // J = [-I, -skew(ra), I, skew(rb)] // Revolute
        //     [ 0,        -1, 0,        1] // Angle
        // M = (J · M^-1 · J^t)^-1

        this.ra = this.bodyA.localPointToWorld(this.localAnchorA);
        this.rb = this.bodyB.localPointToWorld(this.localAnchorB);

        const k = new Mat3();

        k.m00 =
            this.bodyA.invMass +
            this.bodyB.invMass +
            this.bodyA.invI * this.ra.y * this.ra.y +
            this.bodyB.invI * this.rb.y * this.rb.y;

        k.m01 = -this.bodyA.invI * this.ra.y * this.ra.x - this.bodyB.invI * this.rb.y * this.rb.x;

        k.m10 = -this.bodyA.invI * this.ra.x * this.ra.y - this.bodyB.invI * this.rb.x * this.rb.y;

        k.m11 =
            this.bodyA.invMass +
            this.bodyB.invMass +
            this.bodyA.invI * this.ra.x * this.ra.x +
            this.bodyB.invI * this.rb.x * this.rb.x;

        k.m02 = -this.bodyA.invI * this.ra.y - this.bodyB.invI * this.rb.y;
        k.m12 = this.bodyA.invI * this.ra.x + this.bodyB.invI * this.rb.x;

        k.m20 = -this.bodyA.invI * this.ra.y - this.bodyB.invI * this.rb.y;
        k.m21 = this.bodyA.invI * this.ra.x + this.bodyB.invI * this.rb.x;

        k.m22 = this.bodyA.invI + this.bodyB.invI;

        k.m00 += this.gamma;
        k.m11 += this.gamma;
        k.m22 += this.gamma;

        this.m = k.inverted();

        const pa = this.bodyA.position.addNew(this.ra);
        const pb = this.bodyB.position.addNew(this.rb);

        const error01 = pb.subNew(pa);
        const error2 = this.bodyB.rotation - this.bodyA.rotation - this.initialAngleOffset;

        if (SETTINGS.positionCorrection) this.bias = new Vec3(error01.x, error01.y, error2).mul(this.beta * invDt);
        else this.bias = new Vec3(0.0, 0.0, 0.0);

        if (SETTINGS.warmStarting) this.applyImpulse(this.impulseSum);
    }

    override solve() {
        // Calculate corrective impulse: Pc
        // Pc = J^t * λ (λ: lagrangian multiplier)
        // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

        const jv01 = this.bodyB.velocity
            .add(Util.cross(this.bodyB.angularVelocity, this.rb))
            .sub(this.bodyA.velocity.add(Util.cross(this.bodyA.angularVelocity, this.ra)));
        const jv2 = this.bodyB.angularVelocity - this.bodyA.angularVelocity;

        const jv = new Vec3(jv01.x, jv01.y, jv2);

        const lambda = this.m.mulVec3(jv.add(this.bias).add(this.impulseSum.mul(this.gamma)).inverted());

        this.applyImpulse(lambda);

        if (Settings.warmStarting) this.impulseSum = this.impulseSum.add(lambda);
    }

    private applyImpulse(lambda: Vec3) {
        // V2 = V2' + M^-1 ⋅ Pc
        // Pc = J^t ⋅ λ

        const lambda01 = new Vec2(lambda.x, lambda.y);
        const lambda2 = lambda.z;

        // Solve for point-to-point constraint
        this.bodyA.velocity = this.bodyA.velocity.sub(lambda01.mul(this.bodyA.invMass));
        this.bodyA.angularVelocity = this.bodyA.angularVelocity - this.bodyA.invI * this.ra.cross(lambda01);
        this.bodyB.velocity = this.bodyB.velocity.add(lambda01.mul(this.bodyB.invMass));
        this.bodyB.angularVelocity = this.bodyB.angularVelocity + this.bodyB.invI * this.rb.cross(lambda01);

        // Solve for angle constraint
        this.bodyA.angularVelocity = this.bodyA.angularVelocity - lambda2 * this.bodyA.invI;
        this.bodyB.angularVelocity = this.bodyB.angularVelocity + lambda2 * this.bodyB.invI;
    }
}
