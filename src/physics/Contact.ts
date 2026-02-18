/*
 * Portions of this file are derived from the Sopiro Physics Engine.
 *
 * Copyright (c) 2022 Sopiro
 * Licensed under the MIT License
 *
 * Original project:
 * https://github.com/Sopiro
 */
import { Mat2 } from '../math/Mat2';
import Vec2 from '../math/Vec2';
import { ContactPoint } from './CollisionDetection';
import { SETTINGS } from './Constants';
import { Constraint } from './Constraint';
import RigidBody from './RigidBody';
import Utils from './Utils';

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

    private ra!: Vec2;
    private rb!: Vec2;

    public jacobian!: Jacobian;
    public bias!: number;
    public effectiveMass!: number;

    public impulseSum: number = 0.0; // For accumulated impulse

    constructor(manifold: ContactManifold, contactPoint: Vec2) {
        this.manifold = manifold;
        this.bodyA = manifold.bodyA;
        this.bodyB = manifold.bodyB;
        this.contactPoint = contactPoint;

        this.beta = SETTINGS.positionCorrectionBeta;
        this.restitution = this.bodyA.restitution * this.bodyB.restitution;
        this.friction = this.bodyA.friction * this.bodyB.friction;
    }

    preSolve(dir: Vec2, contactType: ContactType, featureFlipped: boolean, inverseDeltaTime: number) {
        // Calculate Jacobian J and effective mass M
        this.contactType = contactType;

        this.ra = this.contactPoint.subNew(this.bodyA.position);
        this.rb = this.contactPoint.subNew(this.bodyB.position);

        this.jacobian = {
            va: dir.negateNew(),
            wa: -this.ra!.cross(dir),
            vb: dir,
            wb: this.rb!.cross(dir),
        };

        this.bias = 0.0;
        if (this.contactType == ContactType.Normal) {
            // Relative velocity at contact point
            const relativeVelocity = this.bodyB.velocity
                .addNew(this.rb.crossScalar(this.bodyB.angularVelocity))
                .subNew(this.bodyA.velocity.addNew(this.ra.crossScalar(this.bodyA.angularVelocity)));
            const normalVelocity = this.manifold.contactNormal.dot(relativeVelocity);

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
            +this.bodyA.invMass +
            this.jacobian.wa * this.bodyA.invI * this.jacobian.wa +
            this.bodyB.invMass +
            this.jacobian.wb * this.bodyB.invI * this.jacobian.wb;

        this.effectiveMass = k > 0.0 ? 1.0 / k : 0.0;

        // Apply the old impulse calculated in the previous time step
        if (SETTINGS.warmStarting) this.applyImpulse(this.impulseSum);
    }

    solve(normalContact?: ContactSolver) {
        // Calculate corrective impulse: Pc
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
        const bx = vn1 + this.nc1.bias - (this.k.m00 * ax + this.k.m01 * ay);
        const by = vn2 + this.nc2.bias - (this.k.m10 * ax + this.k.m11 * ay);

        let lambdaX: number;
        let lambdaY: number;

        // eslint-disable-next-line no-constant-condition
        while (true) {
            //
            // Case 1: vn = 0
            // Both constraints are violated
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
        const incrementalImpulseX = lambdaX - ax;
        const incrementalImpulseY = lambdaY - ay;
        this.applyImpulse(incrementalImpulseX, incrementalImpulseY);

        // Accumulate
        this.nc1.impulseSum = lambdaX;
        this.nc2.impulseSum = lambdaY;
    }

    private applyImpulse(lambdaX: number, lambdaY: number): void {
        this.bodyA.velocity.x = this.bodyA.velocity.x + this.j1.va.x * this.bodyA.invMass * (lambdaX + lambdaY);
        this.bodyA.velocity.y = this.bodyA.velocity.y + this.j1.va.y * this.bodyA.invMass * (lambdaX + lambdaY);
        this.bodyA.angularVelocity =
            this.bodyA.angularVelocity + this.bodyA.invI * (this.j1.wa * lambdaX + this.j2.wa * lambdaY);

        this.bodyB.velocity.x = this.bodyB.velocity.x + this.j1.vb.x * this.bodyB.invMass * (lambdaX + lambdaY);
        this.bodyB.velocity.y = this.bodyB.velocity.y + this.j1.vb.y * this.bodyB.invMass * (lambdaX + lambdaY);
        this.bodyB.angularVelocity =
            this.bodyB.angularVelocity + this.bodyB.invI * (this.j1.wb * lambdaX + this.j2.wb * lambdaY);
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
        this.contactTangent = contactNormal.perpNew();
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
        } else {
            // Solve two contact constraint in one shot using block solver
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
