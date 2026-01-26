import { Mat2 } from '../math/Mat2';
import Utils from '../math/Utils';
import Vec2 from '../math/Vec2';
import { ContactPoint } from './CollisionDetection';
import { SETTINGS } from './Constants';
import { Constraint } from './Constraint';
import RigidBody from './RigidBody';

enum ContactType {
    Normal,
    Tangent,
}

interface Jacobian {
    va: Vec2;
    wa: number;
    vb: Vec2;
    wb: number;
}

class ContactSolver {
    private manifold: ContactManifold;

    private bodyA: RigidBody;
    private bodyB: RigidBody;
    private contactPoint: Vec2;
    private contactType!: ContactType;

    private beta: number;
    private restitution: number;
    private friction: number;

    private ra: Vec2;
    private rb: Vec2;

    public jacobian!: Jacobian;
    public bias!: number;
    public effectiveMass!: number;

    public impulseSum: number = 0.0; // For accumulated impulse

    constructor(manifold: ContactManifold, contactPoint: Vec2) {
        this.manifold = manifold;
        this.bodyA = manifold.bodyA;
        this.bodyB = manifold.bodyB;
        this.contactPoint = contactPoint;

        this.ra = new Vec2();
        this.rb = new Vec2();

        this.beta = SETTINGS.positionCorrectionBeta;
        this.restitution = this.bodyA.restitution * this.bodyB.restitution;
        this.friction = this.bodyA.friction * this.bodyB.friction;
    }

    preSolve(dir: Vec2, contactType: ContactType, featureFlipped: boolean, inverseDeltaTime: number) {
        // Calculate Jacobian J and effective mass M
        // J = [-dir, -ra × dir, dir, rb × dir] (dir: Contact vector, normal or tangent)
        // M = (J · M^-1 · J^t)^-1
        this.contactType = contactType;

        this.ra.x = this.contactPoint.x - this.bodyA.position.x;
        this.ra.y = this.contactPoint.y - this.bodyA.position.y;
        this.rb.x = this.contactPoint.x - this.bodyB.position.x;
        this.rb.y = this.contactPoint.y - this.bodyB.position.y;

        this.jacobian = {
            va: dir.negateNew(),
            wa: -(this.ra.x * dir.y - this.ra.y * dir.x),
            vb: dir,
            wb: this.rb.x * dir.y - this.rb.y * dir.x,
        };

        this.bias = 0.0;
        if (this.contactType == ContactType.Normal) {
            // Relative velocity at contact point
            const relativeVelocityX =
                this.bodyB.velocity.x +
                -this.bodyB.angularVelocity * this.rb.y -
                (this.bodyA.velocity.x + -this.bodyA.angularVelocity * this.ra.y);
            const relativeVelocityY =
                this.bodyB.velocity.y +
                this.bodyB.angularVelocity * this.rb.x -
                (this.bodyA.velocity.y + this.bodyA.angularVelocity * this.ra.x);

            const normalVelocity =
                this.manifold.contactNormal.x * relativeVelocityX + this.manifold.contactNormal.y * relativeVelocityY;

            if (SETTINGS.positionCorrection) {
                this.bias =
                    -(this.beta * inverseDeltaTime) *
                    Math.max(this.manifold.penetrationDepth! - SETTINGS.penetrationSlop, 0.0);
            }

            this.bias += this.restitution * Math.min(normalVelocity + SETTINGS.restitutionSlop, 0.0);
        } else {
            // Bias for surface speed that enables the conveyor belt-like behavior
            this.bias = -(this.bodyB.surfaceSpeed - this.bodyA.surfaceSpeed);
            if (featureFlipped) this.bias *= -1;
        }

        const k: number =
            this.bodyA.invMass +
            this.jacobian.wa * this.bodyA.invI * this.jacobian.wa +
            this.bodyB.invMass +
            this.jacobian.wb * this.bodyB.invI * this.jacobian.wb;

        this.effectiveMass = k > 0.0 ? 1.0 / k : 0.0;

        // Apply the old impulse calculated in the previous time step
        if (SETTINGS.warmStarting) this.applyImpulse(this.impulseSum);
    }

    solve(normalContact?: ContactSolver) {
        // Calculate corrective impulse: Pc
        // Pc = J^t * λ (λ: lagrangian multiplier)
        // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

        // Jacobian * velocity vector (Normal velocity)
        const jv: number =
            this.jacobian.va.x * this.bodyA.velocity.x +
            this.jacobian.va.y * this.bodyA.velocity.y +
            this.jacobian.wa * this.bodyA.angularVelocity +
            this.jacobian.vb.x * this.bodyB.velocity.x +
            this.jacobian.vb.y * this.bodyB.velocity.y +
            this.jacobian.wb * this.bodyB.angularVelocity;

        let lambda = this.effectiveMass * -(jv + this.bias);

        const oldImpulseSum = this.impulseSum;
        switch (this.contactType) {
            case ContactType.Normal: {
                if (SETTINGS.impulseAccumulation) {
                    this.impulseSum = Math.max(0.0, this.impulseSum + lambda);
                } else {
                    this.impulseSum = Math.max(0.0, lambda);
                }
                break;
            }
            case ContactType.Tangent: {
                const maxFriction = this.friction * normalContact!.impulseSum;
                if (SETTINGS.impulseAccumulation) {
                    this.impulseSum = Utils.clamp(this.impulseSum + lambda, -maxFriction, maxFriction);
                } else {
                    this.impulseSum = Utils.clamp(lambda, -maxFriction, maxFriction);
                }
                break;
            }
        }

        if (SETTINGS.impulseAccumulation) {
            lambda = this.impulseSum - oldImpulseSum;
        } else {
            lambda = this.impulseSum;
        }

        // Apply impulse
        this.applyImpulse(lambda);
    }

    private applyImpulse(lambda: number) {
        // V2 = V2' + M^-1 ⋅ Pc
        // Pc = J^t ⋅ λ

        this.bodyA.velocity.x = this.bodyA.velocity.x + this.jacobian.va.x * this.bodyA.invMass * lambda;
        this.bodyA.velocity.y = this.bodyA.velocity.y + this.jacobian.va.y * this.bodyA.invMass * lambda;
        this.bodyA.angularVelocity = this.bodyA.angularVelocity + this.bodyA.invI * this.jacobian.wa * lambda;
        this.bodyB.velocity.x = this.bodyB.velocity.x + this.jacobian.vb.x * this.bodyB.invMass * lambda;
        this.bodyB.velocity.y = this.bodyB.velocity.y + this.jacobian.vb.y * this.bodyB.invMass * lambda;
        this.bodyB.angularVelocity = this.bodyB.angularVelocity + this.bodyB.invI * this.jacobian.wb * lambda;
    }
}

class BlockSolver {
    private bodyA: RigidBody;
    private bodyB: RigidBody;

    // Normal contacts
    private nc1!: ContactSolver;
    private nc2!: ContactSolver;

    // Jacobians
    private j1vaX!: number;
    private j1vaY!: number;
    private j1wa!: number;
    private j1vbX!: number;
    private j1vbY!: number;
    private j1wb!: number;

    private j2wa!: number;
    private j2wb!: number;

    private km00!: number;
    private km01!: number;
    private km10!: number;
    private km11!: number;

    private mm00!: number;
    private mm01!: number;
    private mm10!: number;
    private mm11!: number;

    constructor(manifold: ContactManifold) {
        this.bodyA = manifold.bodyA;
        this.bodyB = manifold.bodyB;
    }

    preSolve(normalContacts: ContactSolver[]) {
        // Calculate Jacobian J and effective mass M
        // J = [-n, -ra1 × n, n, rb1 × n
        //      -n, -ra2 × n, n, rb2 × n]
        // K = (J · M^-1 · J^t)
        // M = K^-1

        this.nc1 = normalContacts[0];
        this.nc2 = normalContacts[1];

        this.j1vaX = normalContacts[0].jacobian.va.x;
        this.j1vaY = normalContacts[0].jacobian.va.y;
        this.j1wa = normalContacts[0].jacobian.wa;
        this.j1vbX = normalContacts[0].jacobian.vb.x;
        this.j1vbY = normalContacts[0].jacobian.vb.y;
        this.j1wb = normalContacts[0].jacobian.wb;

        this.j2wa = normalContacts[1].jacobian.wa;
        this.j2wb = normalContacts[1].jacobian.wb;

        this.km00 =
            this.bodyA.invMass +
            this.j1wa * this.bodyA.invI * this.j1wa +
            this.bodyB.invMass +
            this.j1wb * this.bodyB.invI * this.j1wb;

        this.km11 =
            this.bodyA.invMass +
            this.j2wa * this.bodyA.invI * this.j2wa +
            this.bodyB.invMass +
            this.j2wb * this.bodyB.invI * this.j2wb;

        this.km01 =
            this.bodyA.invMass +
            this.j1wa * this.bodyA.invI * this.j2wa +
            this.bodyB.invMass +
            this.j1wb * this.bodyB.invI * this.j2wb;

        this.km10 = this.km01;
        const kDeterminant = this.km00 * this.km11 - this.km01 * this.km10;

        Utils.assert(kDeterminant != 0);

        // Invert matrix
        const det = 1.0 / kDeterminant;
        this.mm00 = det * this.km11;
        this.mm01 = -det * this.km01;
        this.mm10 = -det * this.km10;
        this.mm11 = det * this.km00;
    }

    solve() {
        // The comments below are copied from Box2D::b2_contact_solver.cpp
        // Check out Box2D: https://box2d.org
        //
        // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
        // Build the mini LCP for this contact patch
        //
        // vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
        //
        // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
        // b = vn0 - velocityBias
        //
        // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
        // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
        // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
        // solution that satisfies the problem is chosen.
        //
        // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
        // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
        //
        // Substitute:
        //
        // x = a + d
        //
        // a := old total impulse
        // x := new total impulse
        // d := incremental impulse
        //
        // For the current iteration we extend the formula for the incremental impulse
        // to compute the new total impulse:
        //
        // vn = A * d + b
        //     = A * (x - a) + b
        //     = A * x + b - A * a
        //     = A * x + b'
        // b' = b - A * a;

        const ax = this.nc1.impulseSum; // old total impulse x
        const ay = this.nc2.impulseSum; // old total impulse y
        Utils.assert(ax >= 0.0, ay >= 0.0);

        // (Velocity constraint) Normal velocity: Jv = 0
        let vn1: number =
            this.nc1.jacobian.va.x * this.bodyA.velocity.x +
            this.nc1.jacobian.va.y * this.bodyA.velocity.y +
            this.nc1.jacobian.wa * this.bodyA.angularVelocity +
            this.nc1.jacobian.vb.x * this.bodyB.velocity.x +
            this.nc1.jacobian.vb.y * this.bodyB.velocity.y +
            this.nc1.jacobian.wb * this.bodyB.angularVelocity;

        let vn2: number =
            this.nc2.jacobian.va.x * this.bodyA.velocity.x +
            this.nc2.jacobian.va.y * this.bodyA.velocity.y +
            this.nc2.jacobian.wa * this.bodyA.angularVelocity +
            this.nc2.jacobian.vb.x * this.bodyB.velocity.x +
            this.nc2.jacobian.vb.y * this.bodyB.velocity.y +
            this.nc2.jacobian.wb * this.bodyB.angularVelocity;

        // b' = b - K * a
        const bx = vn1 + this.nc1.bias - (this.km00 * ax + this.km01 * ay);
        const by = vn2 + this.nc2.bias - (this.km10 * ax + this.km11 * ay);

        let lambdaX: number;
        let lambdaY: number;

        // eslint-disable-next-line no-constant-condition
        while (true) {
            //
            // Case 1: vn = 0
            // Both constraints are violated
            //
            // 0 = A * x + b'
            //
            // Solve for x:
            //
            // x = - inv(A) * b'
            //
            lambdaX = -(this.mm00 * bx + this.mm01 * by);
            lambdaY = -(this.mm10 * bx + this.mm11 * by);

            if (lambdaX >= 0.0 && lambdaY >= 0.0) break;

            //
            // Case 2: vn1 = 0 and x2 = 0
            // The first constraint is violated and the second constraint is satisfied
            //
            //   0 = a11 * x1 + a12 * 0 + b1'
            // vn2 = a21 * x1 + a22 * 0 + b2'
            //
            lambdaX = this.nc1.effectiveMass * -bx;
            lambdaY = 0.0;
            vn1 = 0.0;
            vn2 = this.km01 * lambdaX + by;
            if (lambdaX >= 0.0 && vn2 >= 0.0) break;

            //
            // Case 3: vn2 = 0 and x1 = 0
            // The first constraint is satisfied and the second constraint is violated
            //
            // vn1 = a11 * 0 + a12 * x2 + b1'
            //   0 = a21 * 0 + a22 * x2 + b2'
            //
            lambdaX = 0.0;
            lambdaY = this.nc2.effectiveMass * -by;
            vn1 = this.km10 * lambdaY + bx;
            vn2 = 0.0;
            if (lambdaY >= 0.0 && vn1 >= 0.0) break;

            //
            // Case 4: x1 = 0 and x2 = 0
            // Both constraints are satisfied
            //
            // vn1 = b1
            // vn2 = b2;
            //
            lambdaX = 0.0;
            lambdaY = 0.0;
            vn1 = bx;
            vn2 = by;
            if (vn1 >= 0.0 && vn2 >= 0.0) break;

            // How did you reach here?! something went wrong!
            // TODO: demo 0 fails here, investigate why
            // Utils.assert(false);
            console.warn('Something wrong with block solver, please investigate');
            break;
        }

        // Get the incremental impulse
        const incrementalImpulseX = lambdaX - ax;
        const incrementalImpulseY = lambdaY - ay;
        this.applyImpulse(incrementalImpulseX, incrementalImpulseY);

        // Accumulate
        this.nc1.impulseSum = lambdaX;
        this.nc2.impulseSum = lambdaY;
    }

    private applyImpulse(lambdaX: number, lambdaY: number): void {
        // V2 = V2' + M^-1 ⋅ Pc
        // Pc = J^t ⋅ λ

        this.bodyA.velocity.x = this.bodyA.velocity.x + this.j1vaX * this.bodyA.invMass * (lambdaX + lambdaY);
        this.bodyA.velocity.y = this.bodyA.velocity.y + this.j1vaY * this.bodyA.invMass * (lambdaX + lambdaY);
        this.bodyA.angularVelocity =
            this.bodyA.angularVelocity + this.bodyA.invI * (this.j1wa * lambdaX + this.j2wa * lambdaY);

        this.bodyB.velocity.x = this.bodyB.velocity.x + this.j1vbX * this.bodyB.invMass * (lambdaX + lambdaY);
        this.bodyB.velocity.y = this.bodyB.velocity.y + this.j1vbY * this.bodyB.invMass * (lambdaX + lambdaY);
        this.bodyB.angularVelocity =
            this.bodyB.angularVelocity + this.bodyB.invI * (this.j1wb * lambdaX + this.j2wb * lambdaY);
    }
}

export class ContactManifold extends Constraint {
    // Contact informations
    public readonly penetrationDepth: number;
    public readonly contactNormal: Vec2;
    public readonly contactTangent: Vec2;
    public readonly contactPoints: ContactPoint[];

    private readonly normalContacts: ContactSolver[] = [];
    private readonly tangentContacts: ContactSolver[] = [];
    private readonly blockSolver!: BlockSolver;

    private readonly featureFlipped;

    constructor(
        bodyA: RigidBody,
        bodyB: RigidBody,
        contactPoints: ContactPoint[],
        penetrationDepth: number,
        contactNormal: Vec2,
        featureFlipped: boolean,
    ) {
        super(bodyA, bodyB);
        this.contactPoints = contactPoints;
        this.penetrationDepth = penetrationDepth;
        this.contactNormal = contactNormal;
        this.contactTangent = new Vec2(-contactNormal.y, contactNormal.x);
        this.featureFlipped = featureFlipped;

        for (let i = 0; i < this.numContacts; i++) {
            this.normalContacts.push(new ContactSolver(this, contactPoints[i].point));
            this.tangentContacts.push(new ContactSolver(this, contactPoints[i].point));
        }

        if (this.numContacts == 2 && SETTINGS.blockSolve) {
            this.blockSolver = new BlockSolver(this);
        }
    }

    override preSolve(inverseDeltaTime: number): void {
        for (let i = 0; i < this.numContacts; i++) {
            this.normalContacts[i].preSolve(
                this.contactNormal,
                ContactType.Normal,
                this.featureFlipped,
                inverseDeltaTime,
            );
            this.tangentContacts[i].preSolve(
                this.contactTangent,
                ContactType.Tangent,
                this.featureFlipped,
                inverseDeltaTime,
            );
        }

        // If we have two contact points, then preSolve the block solver.
        if (this.numContacts == 2 && SETTINGS.blockSolve) {
            this.blockSolver.preSolve(this.normalContacts);
        }
    }

    override solve(): void {
        // Solve tangent constraint first
        for (let i = 0; i < this.numContacts; i++) {
            this.tangentContacts[i].solve(this.normalContacts[i]);
        }

        if (this.numContacts == 1 || !SETTINGS.blockSolve) {
            for (let i = 0; i < this.numContacts; i++) {
                this.normalContacts[i].solve();
            }
        } // Solve two contact constraint in one shot using block solver
        else {
            this.blockSolver.solve();
        }
    }

    tryWarmStart(oldManifold: ContactManifold) {
        for (let n = 0; n < this.numContacts; n++) {
            const id = this.contactPoints[n].id;

            for (let o = 0; o < oldManifold.numContacts; o++) {
                if (oldManifold.contactPoints[o].id === id) {
                    this.normalContacts[n].impulseSum = oldManifold.normalContacts[o].impulseSum;
                    this.tangentContacts[n].impulseSum = oldManifold.tangentContacts[o].impulseSum;
                    break;
                }
            }
        }
    }

    get numContacts() {
        return this.contactPoints.length;
    }
}
