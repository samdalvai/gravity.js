import { SETTINGS } from '../core/Constants';
import { RigidBody } from '../core/RigidBody';
import { Matrix2 } from '../math/Matrix2';
import { Vec2 } from '../math/Vec2';
import { Joint } from './Joint';

export class GrabJoint extends Joint {
    public localAnchor: Vec2;
    public target: Vec2;

    private r!: Vec2;
    private m!: Matrix2;
    private bias!: Vec2;
    private impulseSum: Vec2 = new Vec2();

    constructor(body: RigidBody, anchor: Vec2, target: Vec2, frequency = 0.8, dampingRatio = 0.6, jointMass = -1) {
        super(body, body, frequency, dampingRatio, jointMass);

        this.localAnchor = this.bodyA.worldPointToLocal(anchor);
        this.target = target.copy();
    }

    setTarget(target: Vec2): void {
        this.target.assign(target);
    }

    override preSolve(invDt: number): void {
        // Calculate Jacobian J and effective mass M
        // J = [I, skew(r)]
        // M = (J · M^-1 · J^t)^-1
        const cos = Math.cos(this.bodyA.rotation);
        const sin = Math.sin(this.bodyA.rotation);
        this.r = new Vec2(
            this.localAnchor.x * cos - this.localAnchor.y * sin,
            this.localAnchor.x * sin + this.localAnchor.y * cos,
        );
        const p = this.bodyA.position.addNew(this.r);

        const k = new Matrix2();

        k.m00 = this.bodyA.invMass + this.bodyA.invI * this.r.y * this.r.y;
        k.m01 = -this.bodyA.invI * this.r.y * this.r.x;
        k.m10 = -this.bodyA.invI * this.r.x * this.r.y;
        k.m11 = this.bodyA.invMass + this.bodyA.invI * this.r.x * this.r.x;

        k.m00 += this.gamma;
        k.m11 += this.gamma;

        this.m = k.inverted();

        const error = p.subNew(this.target);

        if (SETTINGS.positionCorrection && invDt > 0) {
            this.bias = error.scaleNew(this.beta * invDt);
        } else {
            this.bias = new Vec2(0.0, 0.0);
        }

        if (SETTINGS.warmStarting && (this.impulseSum.x !== 0.0 || this.impulseSum.y !== 0.0)) {
            this.applyImpulse(this.impulseSum);
        }
    }

    override solve(): void {
        // Calculate corrective impulse: Pc
        // Pc = J^t · λ (λ: lagrangian multiplier)
        // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

        const jv: Vec2 = this.bodyA.velocity.addNew(this.r.crossScalar(this.bodyA.angularVelocity));

        const lambda = this.m.mulVector(jv.addNew(this.bias).addNew(this.impulseSum.scaleNew(this.gamma)).negateNew());

        this.applyImpulse(lambda);

        if (SETTINGS.warmStarting) this.impulseSum.addAssign(lambda);
    }

    private applyImpulse(lambda: Vec2): void {
        // V2 = V2' + M^-1 ⋅ Pc
        // Pc = J^t ⋅ λ

        this.bodyA.velocity = this.bodyA.velocity.addNew(lambda.scaleNew(this.bodyA.invMass));
        this.bodyA.angularVelocity = this.bodyA.angularVelocity + this.bodyA.invI * this.r.cross(lambda);
    }
}
