import { Joint } from './joint';
import { Matrix2 } from '../math/matrix2';
import { Vector2 } from '../math/vector2';
import { RigidBody } from '../rigidbody';
import { Settings } from '../settings';
import * as Util from '../util';

export class GrabJoint extends Joint {
    public localAnchor: Vector2;
    public target: Vector2;

    private r!: Vector2;
    private m!: Matrix2;
    private bias!: Vector2;
    private impulseSum: Vector2 = new Vector2();

    constructor(
        body: RigidBody,
        anchor: Vector2,
        target: Vector2,
        frequency = 0.8,
        dampingRatio = 0.6,
        jointMass = -1,
    ) {
        super(body, body, frequency, dampingRatio, jointMass);

        this.localAnchor = body.globalToLocal.mulVector2(anchor, 1);
        this.target = target;
    }

    override prepare(inverseDeltaTime: number): void {
        // Calculate Jacobian J and effective mass M
        // J = [I, skew(r)]
        // M = (J · M^-1 · J^t)^-1

        this.r = this.bodyA.localToGlobal.mulVector2(this.localAnchor, 0);
        const p = this.bodyA.position.addNew(this.r);

        const k = new Matrix2();

        k.m00 = this.bodyA.inverseMass + this.bodyA.inverseInertia * this.r.y * this.r.y;
        k.m01 = -this.bodyA.inverseInertia * this.r.y * this.r.x;
        k.m10 = -this.bodyA.inverseInertia * this.r.x * this.r.y;
        k.m11 = this.bodyA.inverseMass + this.bodyA.inverseInertia * this.r.x * this.r.x;

        k.m00 += this.gamma;
        k.m11 += this.gamma;

        this.m = k.inverted();

        const error = p.subNew(this.target);

        if (Settings.positionCorrection) this.bias = error.mulNew(this.beta * inverseDeltaTime);
        else this.bias = new Vector2(0.0, 0.0);

        if (Settings.warmStarting) this.applyImpulse(this.impulseSum);
    }

    override solve(): void {
        // Calculate corrective impulse: Pc
        // Pc = J^t · λ (λ: lagrangian multiplier)
        // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

        const jv: Vector2 = this.bodyA.linearVelocity.addNew(Util.cross(this.bodyA.angularVelocity, this.r));

        const lambda = this.m.mulVector(jv.addNew(this.bias).addNew(this.impulseSum.mulNew(this.gamma)).inverted());

        this.applyImpulse(lambda);

        if (Settings.warmStarting) this.impulseSum = this.impulseSum.addNew(lambda);
    }

    protected override applyImpulse(lambda: Vector2): void {
        // V2 = V2' + M^-1 ⋅ Pc
        // Pc = J^t ⋅ λ

        this.bodyA.linearVelocity = this.bodyA.linearVelocity.addNew(lambda.mulNew(this.bodyA.inverseMass));
        this.bodyA.angularVelocity = this.bodyA.angularVelocity + this.bodyA.inverseInertia * this.r.cross(lambda);
    }
}
