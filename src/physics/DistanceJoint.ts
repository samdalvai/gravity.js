import Utils from '../math/Utils';
import Vec2 from '../math/Vec2';
import { SETTINGS } from './Constants';
import { Joint } from './Joint';
import RigidBody from './RigidBody';

export class DistanceJoint extends Joint {
    public localAnchorA: Vec2;
    public localAnchorB: Vec2;

    private _length: number;

    private ra!: Vec2;
    private rb!: Vec2;
    private m!: number;
    private n!: Vec2;
    private bias!: number;
    private impulseSum: number = 0.0;

    constructor(
        bodyA: RigidBody,
        bodyB: RigidBody,
        anchorA: Vec2 = bodyA.position,
        anchorB: Vec2 = bodyB.position,
        length: number = -1,
        frequency = 15,
        dampingRatio = 1.0,
        jointMass = -1,
    ) {
        super(bodyA, bodyB, frequency, dampingRatio, jointMass);

        // this.localAnchorA = this.bodyA.globalToLocal.mulVec2(anchorA, 1);
        this.localAnchorA = this.bodyA.worldPointToLocal(anchorA);
        // this.localAnchorB = this.bodyB.globalToLocal.mulVec2(anchorB, 1);
        this.localAnchorB = this.bodyB.worldPointToLocal(anchorB);
        this._length = length <= 0 ? anchorB.subNew(anchorA).magnitude() : length;
    }

    override preSolve(inverseDeltaTime: number): void {
        // Calculate Jacobian J and effective mass M
        // J = [-n, -n·cross(ra), n, n·cross(rb)] ( n = (anchorB-anchorA) / ||anchorB-anchorA|| )
        // M = (J · M^-1 · J^t)^-1

        // this.ra = this.bodyA.localToGlobal.mulVec2(this.localAnchorA, 0);
        this.ra = this.bodyA.localDirToWorld(this.localAnchorA);
        // this.rb = this.bodyB.localToGlobal.mulVec2(this.localAnchorB, 0);
        this.rb = this.bodyB.localDirToWorld(this.localAnchorB);

        const pa = this.bodyA.position.addNew(this.ra);
        const pb = this.bodyB.position.addNew(this.rb);

        const u = pb.subNew(pa);

        this.n = u.normalizeNew();

        const k =
            this.bodyA.invMass +
            this.bodyB.invMass +
            this.bodyA.invI * this.n.cross(this.ra) * this.n.cross(this.ra) +
            this.bodyB.invI * this.n.cross(this.rb) * this.n.cross(this.rb) +
            this.gamma;

        this.m = 1.0 / k;

        const error = u.magnitude() - this._length;

        if (SETTINGS.positionCorrection) this.bias = error * this.beta * inverseDeltaTime;
        else this.bias = 0.0;

        if (SETTINGS.warmStarting) this.applyImpulse(this.impulseSum);
    }

    override solve(): void {
        // Calculate corrective impulse: Pc
        // Pc = J^t · λ (λ: lagrangian multiplier)
        // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

        const jv = this.bodyB.velocity
            .addNew(this.rb.crossScalar(this.bodyB.angularVelocity))
            .subNew(this.bodyA.velocity.addNew(this.ra.crossScalar(this.bodyA.angularVelocity)))
            .dot(this.n);

        // Check out below for the reason why the (accumulated impulse * gamma) term is on the right hand side
        // https://pybullet.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
        const lambda = this.m * -(jv + this.bias + this.impulseSum * this.gamma);

        this.applyImpulse(lambda);

        if (SETTINGS.warmStarting) this.impulseSum += lambda;
    }

    private applyImpulse(lambda: number): void {
        // V2 = V2' + M^-1 ⋅ Pc
        // Pc = J^t ⋅ λ

        this.bodyA.velocity = this.bodyA.velocity.subNew(this.n.scaleNew(lambda * this.bodyA.invMass));
        this.bodyA.angularVelocity =
            this.bodyA.angularVelocity - this.n.dot(this.ra.crossScalar(lambda)) * this.bodyA.invI;
        this.bodyB.velocity = this.bodyB.velocity.addNew(this.n.scaleNew(lambda * this.bodyB.invMass));
        this.bodyB.angularVelocity =
            this.bodyB.angularVelocity + this.n.dot(this.rb.crossScalar(lambda)) * this.bodyB.invI;
    }

    get length(): number {
        return this._length;
    }

    set length(length: number) {
        this._length = Utils.clamp(length, 0, Number.MAX_VALUE);
    }
}
