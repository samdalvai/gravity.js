import { Joint } from './joint';
import { Vector2 } from '../math/vector2';
import { RigidBody, Type } from '../rigidbody';
import { Settings } from '../settings';
import * as Util from '../util';

export class LineJoint extends Joint {
    public readonly localAnchorA: Vector2;
    public readonly localAnchorB: Vector2;

    private t: Vector2;

    private ra!: Vector2;
    private rb!: Vector2;
    private m!: number;
    private u!: Vector2;
    private bias!: number;
    private impulseSum: number = 0.0;

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
            throw "Can't make line constraint between static bodies";
        if (bodyA.type == Type.Dynamic && bodyB.type == Type.Dynamic)
            throw "Can't make line constraint between dynamic bodies";
        if (bodyB.type == Type.Static) throw 'Please make line constraint by using the bodyA as a static body';

        this.localAnchorA = this.bodyA.globalToLocal.mulVector2(anchorA, 1);
        this.localAnchorB = this.bodyB.globalToLocal.mulVector2(anchorB, 1);

        const u = anchorB.subNew(anchorA);
        this.t = new Vector2(-u.y, u.x).normalized();

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
        // J = [-t^t, -(ra + u)×t, t^t, rb×t]
        // M = (J · M^-1 · J^t)^-1

        this.ra = this.bodyA.localToGlobal.mulVector2(this.localAnchorA, 0);
        this.rb = this.bodyB.localToGlobal.mulVector2(this.localAnchorB, 0);

        const pa = this.bodyA.position.addNew(this.ra);
        const pb = this.bodyB.position.addNew(this.rb);

        this.u = pb.subNew(pa);

        const k =
            this.bodyB.inverseMass +
            this.rb.cross(this.t) * this.bodyB.inverseInertia -
            (this.bodyA.inverseMass + this.ra.addNew(this.u).cross(this.t) * this.bodyA.inverseInertia) +
            this.gamma;

        this.m = 1.0 / k;

        const error = this.u.dot(this.t);

        if (Settings.positionCorrection) this.bias = error * this.beta * inverseDeltaTime;
        else this.bias = 0.0;

        if (Settings.warmStarting) this.applyImpulse(this.impulseSum);
    }

    override solve(): void {
        // Calculate corrective impulse: Pc
        // Pc = J^t · λ (λ: lagrangian multiplier)
        // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

        const jv =
            this.t.dot(this.bodyB.linearVelocity) +
            this.rb.cross(this.t) * this.bodyB.angularVelocity -
            (this.t.dot(this.bodyA.linearVelocity) + this.rb.addNew(this.u).cross(this.t) * this.bodyA.angularVelocity);

        const lambda = this.m * -(jv + this.bias + this.impulseSum * this.gamma);

        this.applyImpulse(lambda);

        if (Settings.warmStarting) this.impulseSum += lambda;
    }

    protected override applyImpulse(lambda: number): void {
        // V2 = V2' + M^-1 ⋅ Pc
        // Pc = J^t ⋅ λ

        this.bodyA.linearVelocity = this.bodyA.linearVelocity.subNew(this.t.mulNew(lambda * this.bodyA.inverseMass));
        this.bodyA.angularVelocity =
            this.bodyA.angularVelocity - this.ra.addNew(this.u).cross(this.t) * this.bodyA.inverseInertia;
        this.bodyB.linearVelocity = this.bodyB.linearVelocity.addNew(this.t.mulNew(lambda * this.bodyB.inverseMass));
        this.bodyB.angularVelocity = this.bodyB.angularVelocity + this.rb.cross(this.t) * this.bodyB.inverseInertia;
    }
}
