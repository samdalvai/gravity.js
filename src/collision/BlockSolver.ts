/*
 * Portions of this file are derived from the Sopiro Physics Engine.
 *
 * Copyright (c) 2022 Sopiro
 * Licensed under the MIT License
 *
 * Original project:
 * https://github.com/Sopiro
 */
import RigidBody from '../core/RigidBody';
import { Mat2 } from '../math/Mat2';
import * as Utils from '../utils/Utils';
import { ContactManifold, Jacobian } from './ContactManifold';
import { ContactSolver } from './ContactSolver';

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

        Utils.assert(this.k.determinant != 0);
        this.m = this.k.inverted();
    }

    solve() {
        const ax = this.nc1.impulseSum; // old total impulse x
        const ay = this.nc2.impulseSum; // old total impulse y
        Utils.assert(ax >= 0.0, ay >= 0.0);

        // (Velocity constraint) Normal velocity: Jv = 0
        /*
        let vn1: number =
            +this.nc1.jacobian.va.dot(this.bodyA.linearVelocity) +
            this.nc1.jacobian.wa * this.bodyA.angularVelocity +
            this.nc1.jacobian.vb.dot(this.bodyB.linearVelocity) +
            this.nc1.jacobian.wb * this.bodyB.angularVelocity;
        */
        let vn1: number =
            this.nc1.jacobian.va.x * this.bodyA.velocity.x +
            this.nc1.jacobian.va.y * this.bodyA.velocity.y +
            this.nc1.jacobian.wa * this.bodyA.angularVelocity +
            this.nc1.jacobian.vb.x * this.bodyB.velocity.x +
            this.nc1.jacobian.vb.y * this.bodyB.velocity.y +
            this.nc1.jacobian.wb * this.bodyB.angularVelocity;

        /*
        let vn2: number =
            +this.nc2.jacobian.va.dot(this.bodyA.linearVelocity) +
            this.nc2.jacobian.wa * this.bodyA.angularVelocity +
            this.nc2.jacobian.vb.dot(this.bodyB.linearVelocity) +
            this.nc2.jacobian.wb * this.bodyB.angularVelocity;
        */
        let vn2: number =
            this.nc2.jacobian.va.x * this.bodyA.velocity.x +
            this.nc2.jacobian.va.y * this.bodyA.velocity.y +
            this.nc2.jacobian.wa * this.bodyA.angularVelocity +
            this.nc2.jacobian.vb.x * this.bodyB.velocity.x +
            this.nc2.jacobian.vb.y * this.bodyB.velocity.y +
            this.nc2.jacobian.wb * this.bodyB.angularVelocity;

        // b' = b - K * a
        // b = b.subNew(this.k.mulVector(a));
        const bx = vn1 + this.nc1.bias - (this.k.m00 * ax + this.k.m01 * ay);
        const by = vn2 + this.nc2.bias - (this.k.m10 * ax + this.k.m11 * ay);

        // let x: Vector2; // Lambda
        let lambdaX: number;
        let lambdaY: number;

        // eslint-disable-next-line no-constant-condition
        while (true) {
            //
            // Case 1: vn = 0
            // Both constraints are violated
            // x = this.m.scaleNew(b).negateNew();
            lambdaX = -(this.m.m00 * bx + this.m.m01 * by);
            lambdaY = -(this.m.m10 * bx + this.m.m11 * by);

            if (lambdaX >= 0.0 && lambdaY >= 0.0) break;

            //
            // Case 2: vn1 = 0 and x2 = 0
            // The first constraint is violated and the second constraint is satisfied
            lambdaX = this.nc1.effectiveMass * -bx;
            lambdaY = 0.0;
            vn1 = 0.0;
            vn2 = this.k.m01 * lambdaX + by;
            if (lambdaX >= 0.0 && vn2 >= 0.0) break;

            //
            // Case 3: vn2 = 0 and x1 = 0
            // The first constraint is satisfied and the second constraint is violated
            lambdaX = 0.0;
            lambdaY = this.nc2.effectiveMass * -by;
            vn1 = this.k.m10 * lambdaY + bx;
            vn2 = 0.0;
            if (lambdaY >= 0.0 && vn1 >= 0.0) break;

            //
            // Case 4: x1 = 0 and x2 = 0
            // Both constraints are satisfied
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
        // const d = x.subNew(a);
        const incrementalImpulseX = lambdaX - ax;
        const incrementalImpulseY = lambdaY - ay;
        this.applyImpulse(incrementalImpulseX, incrementalImpulseY);

        // Accumulate
        this.nc1.impulseSum = lambdaX;
        this.nc2.impulseSum = lambdaY;
    }

    private applyImpulse(lambdaX: number, lambdaY: number): void {
        /*
        this.bodyA.linearVelocity = this.bodyA.linearVelocity.addNew(
            this.j1.va.scaleNew(this.bodyA.inverseMass * (lambda.x + lambda.y)),
        );
        this.bodyA.angularVelocity =
            this.bodyA.angularVelocity + this.bodyA.inverseInertia * (this.j1.wa * lambda.x + this.j2.wa * lambda.y);
        */
        this.bodyA.velocity.x = this.bodyA.velocity.x + this.j1.va.x * this.bodyA.invMass * (lambdaX + lambdaY);
        this.bodyA.velocity.y = this.bodyA.velocity.y + this.j1.va.y * this.bodyA.invMass * (lambdaX + lambdaY);
        this.bodyA.angularVelocity =
            this.bodyA.angularVelocity + this.bodyA.invI * (this.j1.wa * lambdaX + this.j2.wa * lambdaY);

        /*
        this.bodyB.linearVelocity = this.bodyB.linearVelocity.addNew(
            this.j1.vb.scaleNew(this.bodyB.inverseMass * (lambda.x + lambda.y)),
        );
        this.bodyB.angularVelocity =
            this.bodyB.angularVelocity + this.bodyB.inverseInertia * (this.j1.wb * lambda.x + this.j2.wb * lambda.y);
        */
        this.bodyB.velocity.x = this.bodyB.velocity.x + this.j1.vb.x * this.bodyB.invMass * (lambdaX + lambdaY);
        this.bodyB.velocity.y = this.bodyB.velocity.y + this.j1.vb.y * this.bodyB.invMass * (lambdaX + lambdaY);
        this.bodyB.angularVelocity =
            this.bodyB.angularVelocity + this.bodyB.invI * (this.j1.wb * lambdaX + this.j2.wb * lambdaY);
    }
}
