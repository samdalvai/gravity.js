import { RigidBody } from '../core/RigidBody';
import * as Utils from '../utils/Utils';
import { ContactManifold } from './ContactManifold.perf';
import { ContactSolver } from './ContactSolver.perf';

export class BlockSolver {
    private bodyA: RigidBody;
    private bodyB: RigidBody;

    // Normal contacts
    private nc1!: ContactSolver;
    private nc2!: ContactSolver;

    // Cache the two contact rows as plain scalars so solve() can stay on numbers only.
    private j1vaX = 0.0;
    private j1vaY = 0.0;
    private j1wa = 0.0;
    private j1vbX = 0.0;
    private j1vbY = 0.0;
    private j1wb = 0.0;
    private j1bias = 0.0;
    private j1effectiveMass = 0.0;

    private j2vaX = 0.0;
    private j2vaY = 0.0;
    private j2wa = 0.0;
    private j2vbX = 0.0;
    private j2vbY = 0.0;
    private j2wb = 0.0;
    private j2bias = 0.0;
    private j2effectiveMass = 0.0;

    // The 2x2 normal block is symmetric, so one off-diagonal term is enough.
    private k00 = 0.0;
    private k01 = 0.0;
    private k11 = 0.0;
    private m00 = 0.0;
    private m01 = 0.0;
    private m11 = 0.0;

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

        this.j1vaX = this.nc1.jvaX;
        this.j1vaY = this.nc1.jvaY;
        this.j1wa = this.nc1.jwa;
        this.j1vbX = this.nc1.jvbX;
        this.j1vbY = this.nc1.jvbY;
        this.j1wb = this.nc1.jwb;
        this.j1bias = this.nc1.bias;
        this.j1effectiveMass = this.nc1.effectiveMass;

        this.j2vaX = this.nc2.jvaX;
        this.j2vaY = this.nc2.jvaY;
        this.j2wa = this.nc2.jwa;
        this.j2vbX = this.nc2.jvbX;
        this.j2vbY = this.nc2.jvbY;
        this.j2wb = this.nc2.jwb;
        this.j2bias = this.nc2.bias;
        this.j2effectiveMass = this.nc2.effectiveMass;

        const bodyAInvMass = this.bodyA.invMass;
        const bodyAInvI = this.bodyA.invI;
        const bodyBInvMass = this.bodyB.invMass;
        const bodyBInvI = this.bodyB.invI;

        this.k00 = bodyAInvMass + this.j1wa * bodyAInvI * this.j1wa + bodyBInvMass + this.j1wb * bodyBInvI * this.j1wb;

        this.k11 = bodyAInvMass + this.j2wa * bodyAInvI * this.j2wa + bodyBInvMass + this.j2wb * bodyBInvI * this.j2wb;

        this.k01 = bodyAInvMass + this.j1wa * bodyAInvI * this.j2wa + bodyBInvMass + this.j1wb * bodyBInvI * this.j2wb;

        const determinant = this.k00 * this.k11 - this.k01 * this.k01;
        Utils.assert(determinant !== 0.0, 'Determinant is 0');

        const invDeterminant = 1.0 / determinant;
        this.m00 = invDeterminant * this.k11;
        this.m01 = -invDeterminant * this.k01;
        this.m11 = invDeterminant * this.k00;
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

        const aX = this.nc1.impulseSum; // old total impulse
        const aY = this.nc2.impulseSum;
        Utils.assert(aX >= 0.0, aY >= 0.0);

        const bodyAVelocity = this.bodyA.velocity;
        const bodyBVelocity = this.bodyB.velocity;
        const bodyAAngularVelocity = this.bodyA.angularVelocity;
        const bodyBAngularVelocity = this.bodyB.angularVelocity;

        // (Velocity constraint) Normal velocity: Jv = 0
        let vn1: number =
            this.j1vaX * bodyAVelocity.x +
            this.j1vaY * bodyAVelocity.y +
            this.j1wa * bodyAAngularVelocity +
            this.j1vbX * bodyBVelocity.x +
            this.j1vbY * bodyBVelocity.y +
            this.j1wb * bodyBAngularVelocity;

        let vn2: number =
            this.j2vaX * bodyAVelocity.x +
            this.j2vaY * bodyAVelocity.y +
            this.j2wa * bodyAAngularVelocity +
            this.j2vbX * bodyBVelocity.x +
            this.j2vbY * bodyBVelocity.y +
            this.j2wb * bodyBAngularVelocity;

        let bX = vn1 + this.j1bias;
        let bY = vn2 + this.j2bias;

        // b' = b - K * a
        bX -= this.k00 * aX + this.k01 * aY;
        bY -= this.k01 * aX + this.k11 * aY;
        let xX = 0.0;
        let xY = 0.0;

        // The complementarity cases are tested in order and the first valid one wins.
        xX = -(this.m00 * bX + this.m01 * bY);
        xY = -(this.m01 * bX + this.m11 * bY);
        let solved = xX >= 0.0 && xY >= 0.0;

        if (!solved) {
            xX = this.j1effectiveMass * -bX;
            xY = 0.0;
            vn1 = 0.0;
            vn2 = this.k01 * xX + bY;
            solved = xX >= 0.0 && vn2 >= 0.0;
        }

        if (!solved) {
            xX = 0.0;
            xY = this.j2effectiveMass * -bY;
            vn1 = this.k01 * xY + bX;
            vn2 = 0.0;
            solved = xY >= 0.0 && vn1 >= 0.0;
        }

        if (!solved) {
            xX = 0.0;
            xY = 0.0;
            vn1 = bX;
            vn2 = bY;
            solved = vn1 >= 0.0 && vn2 >= 0.0;
        }

        if (!solved) {
            // How did you reach here?! something went wrong!
            Utils.assert(false);
        }

        // Get the incremental impulse
        this.applyImpulse(xX - aX, xY - aY);

        // Accumulate
        this.nc1.impulseSum = xX;
        this.nc2.impulseSum = xY;
    }

    private applyImpulse(lambdaX: number, lambdaY: number): void {
        // V2 = V2' + M^-1 ⋅ Pc
        // Pc = J^t ⋅ λ

        if (lambdaX === 0.0 && lambdaY === 0.0) {
            return;
        }

        const linearImpulse = lambdaX + lambdaY;
        const bodyAImpulseScale = this.bodyA.invMass * linearImpulse;
        // Both normal constraints share the same contact normal, so their linear impulses
        // add before being applied to the bodies.
        this.bodyA.velocity.x += this.j1vaX * bodyAImpulseScale;
        this.bodyA.velocity.y += this.j1vaY * bodyAImpulseScale;
        this.bodyA.angularVelocity += this.bodyA.invI * (this.j1wa * lambdaX + this.j2wa * lambdaY);

        const bodyBImpulseScale = this.bodyB.invMass * linearImpulse;
        this.bodyB.velocity.x += this.j1vbX * bodyBImpulseScale;
        this.bodyB.velocity.y += this.j1vbY * bodyBImpulseScale;
        this.bodyB.angularVelocity += this.bodyB.invI * (this.j1wb * lambdaX + this.j2wb * lambdaY);
    }
}
