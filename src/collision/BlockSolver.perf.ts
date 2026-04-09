import { RigidBody } from '../core/RigidBody';
import { Mat2 } from '../math/Mat2';
import { Vec2 } from '../math/Vec2';
import * as Utils from '../utils/Utils';
import { ContactManifold } from './ContactManifold.perf';
import { ContactSolver, Jacobian } from './ContactSolver.perf';

export class BlockSolver {
    private bodyA: RigidBody;
    private bodyB: RigidBody;

    // Normal contacts
    private nc1!: ContactSolver;
    private nc2!: ContactSolver;
    // Jacobians
    private j1!: Jacobian;
    private j2!: Jacobian;

    private k!: Mat2;
    private m!: Mat2;

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

        this.j1 = normalContacts[0].jacobian;
        this.j2 = normalContacts[1].jacobian;

        this.k = new Mat2();

        this.k.m00 =
            +this.bodyA.invMass +
            this.j1.wa * this.bodyA.invI * this.j1.wa +
            this.bodyB.invMass +
            this.j1.wb * this.bodyB.invI * this.j1.wb;

        this.k.m11 =
            +this.bodyA.invMass +
            this.j2.wa * this.bodyA.invI * this.j2.wa +
            this.bodyB.invMass +
            this.j2.wb * this.bodyB.invI * this.j2.wb;

        this.k.m01 =
            +this.bodyA.invMass +
            this.j1.wa * this.bodyA.invI * this.j2.wa +
            this.bodyB.invMass +
            this.j1.wb * this.bodyB.invI * this.j2.wb;

        this.k.m10 = this.k.m01;

        Utils.assert(this.k.determinant != 0, 'Determinant is 0');
        this.m = this.k.inverted();
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

        const a = new Vec2(this.nc1.impulseSum, this.nc2.impulseSum); // old total impulse
        Utils.assert(a.x >= 0.0, a.y >= 0.0);

        // (Velocity constraint) Normal velocity: Jv = 0
        let vn1: number =
            +this.nc1.jacobian.va.dot(this.bodyA.velocity) +
            this.nc1.jacobian.wa * this.bodyA.angularVelocity +
            this.nc1.jacobian.vb.dot(this.bodyB.velocity) +
            this.nc1.jacobian.wb * this.bodyB.angularVelocity;

        let vn2: number =
            +this.nc2.jacobian.va.dot(this.bodyA.velocity) +
            this.nc2.jacobian.wa * this.bodyA.angularVelocity +
            this.nc2.jacobian.vb.dot(this.bodyB.velocity) +
            this.nc2.jacobian.wb * this.bodyB.angularVelocity;

        let b = new Vec2(vn1 + this.nc1.bias, vn2 + this.nc2.bias);

        // b' = b - K * a
        b = b.subNew(this.k.mulVector(a));
        let x: Vec2; // Lambda

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
            x = this.m.mulVector(b).negateNew();
            if (x.x >= 0.0 && x.y >= 0.0) break;

            //
            // Case 2: vn1 = 0 and x2 = 0
            // The first constraint is violated and the second constraint is satisfied
            //
            //   0 = a11 * x1 + a12 * 0 + b1'
            // vn2 = a21 * x1 + a22 * 0 + b2'
            //
            x.x = this.nc1.effectiveMass * -b.x;
            x.y = 0.0;
            vn1 = 0.0;
            vn2 = this.k.m01 * x.x + b.y;
            if (x.x >= 0.0 && vn2 >= 0.0) break;

            //
            // Case 3: vn2 = 0 and x1 = 0
            // The first constraint is satisfied and the second constraint is violated
            //
            // vn1 = a11 * 0 + a12 * x2 + b1'
            //   0 = a21 * 0 + a22 * x2 + b2'
            //
            x.x = 0.0;
            x.y = this.nc2.effectiveMass * -b.y;
            vn1 = this.k.m10 * x.y + b.x;
            vn2 = 0.0;
            if (x.y >= 0.0 && vn1 >= 0.0) break;

            //
            // Case 4: x1 = 0 and x2 = 0
            // Both constraints are satisfied
            //
            // vn1 = b1
            // vn2 = b2;
            //
            x.x = 0.0;
            x.y = 0.0;
            vn1 = b.x;
            vn2 = b.y;
            if (vn1 >= 0.0 && vn2 >= 0.0) break;

            // How did you reach here?! something went wrong!
            Utils.assert(false);
            break;
        }

        // Get the incremental impulse
        const d = x.subNew(a);
        this.applyImpulse(d);

        // Accumulate
        this.nc1.impulseSum = x.x;
        this.nc2.impulseSum = x.y;
    }

    private applyImpulse(lambda: Vec2): void {
        // V2 = V2' + M^-1 ⋅ Pc
        // Pc = J^t ⋅ λ

        this.bodyA.velocity = this.bodyA.velocity.addNew(
            this.j1.va.scaleNew(this.bodyA.invMass * (lambda.x + lambda.y)),
        );
        this.bodyA.angularVelocity =
            this.bodyA.angularVelocity + this.bodyA.invI * (this.j1.wa * lambda.x + this.j2.wa * lambda.y);
        this.bodyB.velocity = this.bodyB.velocity.addNew(
            this.j1.vb.scaleNew(this.bodyB.invMass * (lambda.x + lambda.y)),
        );
        this.bodyB.angularVelocity =
            this.bodyB.angularVelocity + this.bodyB.invI * (this.j1.wb * lambda.x + this.j2.wb * lambda.y);
    }
}
