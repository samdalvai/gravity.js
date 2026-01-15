import { Joint } from './joint';
import { Matrix2 } from '../math/matrix2';
import { Vector2 } from '../math/vector2';
import { RigidBody, Type } from '../rigidbody';
import { Settings } from '../settings';
import * as Util from '../util';

// Line joint + Angle joint
export class PrismaticJoint extends Joint {
    public localAnchorA: Vector2;
    public localAnchorB: Vector2;
    public initialAngle: number;

    private t: Vector2;

    private ra!: Vector2;
    private rb!: Vector2;
    private m!: Matrix2;
    private u!: Vector2;
    private bias!: Vector2;
    private impulseSum: Vector2 = new Vector2();

    constructor(
        bodyA: RigidBody,
        bodyB: RigidBody,
        anchorA: Vector2 = bodyA.position,
        anchorB: Vector2 = bodyB.position,
        dir?: Vector2,
        frequency = 30,
        dampingRatio = 1.0,
        jointMass = -1,
    ) {
        super(bodyA, bodyB, frequency, dampingRatio, jointMass);

        if (bodyA.type == Type.Static && bodyB.type == Type.Static)
            throw "Can't make prismatic constraint between static bodies";
        if (bodyA.type == Type.Dynamic && bodyB.type == Type.Dynamic)
            throw "Can't make prismatic constraint between dynamic bodies";
        if (bodyB.type == Type.Static) throw 'Please make prismatic constraint by using the bodyA as a static body';

        this.localAnchorA = this.bodyA.globalToLocal.mulVector2(anchorA, 1);
        this.localAnchorB = this.bodyB.globalToLocal.mulVector2(anchorB, 1);

        this.initialAngle = bodyB.rotation - bodyA.rotation;

        if (dir == undefined) {
            const u = anchorB.subNew(anchorA);
            this.t = new Vector2(-u.y, u.x).normalized();
        } else {
            this.t = new Vector2(-dir.y, dir.x).normalized();
        }

        Util.assert(this.t.squaredLength > 0);
    }

    override prepare(inverseDeltaTime: number): void {
        // Calculate Jacobian J and effective mass M
        // J = [-t^t, -(ra + u)×t, t^t, rb×t] // Line
        //     [   0,          -1,   0,    1] // Angle
        // M = (J · M^-1 · J^t)^-1

        this.ra = this.bodyA.localToGlobal.mulVector2(this.localAnchorA, 0);
        this.rb = this.bodyB.localToGlobal.mulVector2(this.localAnchorB, 0);

        const pa = this.bodyA.position.addNew(this.ra);
        const pb = this.bodyB.position.addNew(this.rb);

        this.u = pb.subNew(pa);

        const sa = this.ra.addNew(this.u).cross(this.t);
        const sb = this.rb.cross(this.t);

        const k = new Matrix2();
        k.m00 =
            this.bodyA.inverseMass +
            sa * sa * this.bodyA.inverseInertia +
            this.bodyB.inverseMass +
            sb * sb * this.bodyB.inverseInertia;
        k.m01 = sa * this.bodyA.inverseInertia + sb * this.bodyB.inverseInertia;
        k.m10 = sa * this.bodyA.inverseInertia + sb * this.bodyB.inverseInertia;
        k.m11 = this.bodyA.inverseInertia + this.bodyB.inverseInertia;

        k.m00 += this.gamma;
        k.m11 += this.gamma;

        this.m = k.inverted();

        const error0 = this.u.dot(this.t);
        const error1 = this.bodyB.rotation - this.bodyA.rotation - this.initialAngle;

        if (Settings.positionCorrection) this.bias = new Vector2(error0, error1).mulNew(this.beta * inverseDeltaTime);
        else this.bias = new Vector2(0.0, 0.0);

        if (Settings.warmStarting) this.applyImpulse(this.impulseSum);
    }

    override solve(): void {
        // Calculate corrective impulse: Pc
        // Pc = J^t · λ (λ: lagrangian multiplier)
        // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

        const jv0 =
            this.t.dot(this.bodyB.linearVelocity) +
            this.rb.cross(this.t) * this.bodyB.angularVelocity -
            (this.t.dot(this.bodyA.linearVelocity) + this.rb.addNew(this.u).cross(this.t) * this.bodyA.angularVelocity) +
            this.gamma;

        const jv1 = this.bodyB.angularVelocity - this.bodyA.angularVelocity;

        const jv = new Vector2(jv0, jv1);

        const lambda = this.m.mulVector(jv.addNew(this.bias).addNew(this.impulseSum.mulNew(this.gamma)).inverted());

        this.applyImpulse(lambda);

        if (Settings.warmStarting) this.impulseSum.addNew(lambda);
    }

    protected override applyImpulse(lambda: Vector2): void {
        // V2 = V2' + M^-1 ⋅ Pc
        // Pc = J^t ⋅ λ

        const lambda0 = lambda.x;
        const lambda1 = lambda.y;

        this.bodyA.linearVelocity = this.bodyA.linearVelocity.subNew(this.t.mulNew(lambda0 * this.bodyA.inverseMass));
        this.bodyA.angularVelocity =
            this.bodyA.angularVelocity - this.ra.addNew(this.u).cross(this.t) * this.bodyA.inverseInertia;
        this.bodyB.linearVelocity = this.bodyB.linearVelocity.addNew(this.t.mulNew(lambda0 * this.bodyB.inverseMass));
        this.bodyB.angularVelocity = this.bodyB.angularVelocity + this.rb.cross(this.t) * this.bodyB.inverseInertia;

        this.bodyA.angularVelocity = this.bodyA.angularVelocity - lambda1 * this.bodyA.inverseInertia;
        this.bodyB.angularVelocity = this.bodyB.angularVelocity + lambda1 * this.bodyB.inverseInertia;
    }
}
