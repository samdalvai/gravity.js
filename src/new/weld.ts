import { Joint } from './joint';
import { Matrix3 } from '../math/matrix3';
import { Vector2 } from '../math/vector2';
import { Vector3 } from '../math/vector3';
import { RigidBody } from '../rigidbody';
import { Settings } from '../settings';
import * as Util from '../util';

// Revolute joint + Angle joint
export class WeldJoint extends Joint {
    public localAnchorA: Vector2;
    public localAnchorB: Vector2;

    public initialAngleOffset: number;

    private ra!: Vector2;
    private rb!: Vector2;
    private m!: Matrix3;
    private bias!: Vector3;
    private impulseSum: Vector3 = new Vector3();

    constructor(
        bodyA: RigidBody,
        bodyB: RigidBody,
        anchor: Vector2 = Util.mid(bodyA.position, bodyB.position),
        frequency = -1,
        dampingRatio = 1.0,
        jointMass = -1,
    ) {
        super(bodyA, bodyB, frequency, dampingRatio, jointMass);

        this.initialAngleOffset = bodyB.rotation - bodyA.rotation;
        this.localAnchorA = this.bodyA.globalToLocal.mulVector2(anchor, 1);
        this.localAnchorB = this.bodyB.globalToLocal.mulVector2(anchor, 1);

        this.drawAnchor = false;
        this.drawConnectionLine = false;
    }

    override prepare(inverseDeltaTime: number) {
        // Calculate Jacobian J and effective mass M
        // J = [-I, -skew(ra), I, skew(rb)] // Revolute
        //     [ 0,        -1, 0,        1] // Angle
        // M = (J · M^-1 · J^t)^-1

        this.ra = this.bodyA.localToGlobal.mulVector2(this.localAnchorA, 0);
        this.rb = this.bodyB.localToGlobal.mulVector2(this.localAnchorB, 0);

        const k = new Matrix3();

        k.m00 =
            this.bodyA.inverseMass +
            this.bodyB.inverseMass +
            this.bodyA.inverseInertia * this.ra.y * this.ra.y +
            this.bodyB.inverseInertia * this.rb.y * this.rb.y;

        k.m01 = -this.bodyA.inverseInertia * this.ra.y * this.ra.x - this.bodyB.inverseInertia * this.rb.y * this.rb.x;

        k.m10 = -this.bodyA.inverseInertia * this.ra.x * this.ra.y - this.bodyB.inverseInertia * this.rb.x * this.rb.y;

        k.m11 =
            this.bodyA.inverseMass +
            this.bodyB.inverseMass +
            this.bodyA.inverseInertia * this.ra.x * this.ra.x +
            this.bodyB.inverseInertia * this.rb.x * this.rb.x;

        k.m02 = -this.bodyA.inverseInertia * this.ra.y - this.bodyB.inverseInertia * this.rb.y;
        k.m12 = this.bodyA.inverseInertia * this.ra.x + this.bodyB.inverseInertia * this.rb.x;

        k.m20 = -this.bodyA.inverseInertia * this.ra.y - this.bodyB.inverseInertia * this.rb.y;
        k.m21 = this.bodyA.inverseInertia * this.ra.x + this.bodyB.inverseInertia * this.rb.x;

        k.m22 = this.bodyA.inverseInertia + this.bodyB.inverseInertia;

        k.m00 += this.gamma;
        k.m11 += this.gamma;
        k.m22 += this.gamma;

        this.m = k.inverted();

        const pa = this.bodyA.position.addNew(this.ra);
        const pb = this.bodyB.position.addNew(this.rb);

        const error01 = pb.subNew(pa);
        const error2 = this.bodyB.rotation - this.bodyA.rotation - this.initialAngleOffset;

        if (Settings.positionCorrection)
            this.bias = new Vector3(error01.x, error01.y, error2).mulNew(this.beta * inverseDeltaTime);
        else this.bias = new Vector3(0.0, 0.0, 0.0);

        if (Settings.warmStarting) this.applyImpulse(this.impulseSum);
    }

    override solve() {
        // Calculate corrective impulse: Pc
        // Pc = J^t * λ (λ: lagrangian multiplier)
        // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

        const jv01 = this.bodyB.linearVelocity
            .addNew(Util.cross(this.bodyB.angularVelocity, this.rb))
            .subNew(this.bodyA.linearVelocity.addNew(Util.cross(this.bodyA.angularVelocity, this.ra)));
        const jv2 = this.bodyB.angularVelocity - this.bodyA.angularVelocity;

        const jv = new Vector3(jv01.x, jv01.y, jv2);

        const lambda = this.m.mulVector3(jv.addNew(this.bias).addNew(this.impulseSum.mulNew(this.gamma)).inverted());

        this.applyImpulse(lambda);

        if (Settings.warmStarting) this.impulseSum = this.impulseSum.addNew(lambda);
    }

    protected override applyImpulse(lambda: Vector3) {
        // V2 = V2' + M^-1 ⋅ Pc
        // Pc = J^t ⋅ λ

        const lambda01 = new Vector2(lambda.x, lambda.y);
        const lambda2 = lambda.z;

        // Solve for point-to-point constraint
        this.bodyA.linearVelocity = this.bodyA.linearVelocity.subNew(lambda01.mulNew(this.bodyA.inverseMass));
        this.bodyA.angularVelocity = this.bodyA.angularVelocity - this.bodyA.inverseInertia * this.ra.cross(lambda01);
        this.bodyB.linearVelocity = this.bodyB.linearVelocity.addNew(lambda01.mulNew(this.bodyB.inverseMass));
        this.bodyB.angularVelocity = this.bodyB.angularVelocity + this.bodyB.inverseInertia * this.rb.cross(lambda01);

        // Solve for angle constraint
        this.bodyA.angularVelocity = this.bodyA.angularVelocity - lambda2 * this.bodyA.inverseInertia;
        this.bodyB.angularVelocity = this.bodyB.angularVelocity + lambda2 * this.bodyB.inverseInertia;
    }
}
