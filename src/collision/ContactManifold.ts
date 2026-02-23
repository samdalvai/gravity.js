/*
 * Portions of this file are derived from the Sopiro Physics Engine.
 *
 * Copyright (c) 2022 Sopiro
 * Licensed under the MIT License
 *
 * Original project:
 * https://github.com/Sopiro
 */
import { Constraint } from '../constraint/Constraint';
import { SETTINGS } from '../core/Constants';
import RigidBody from '../core/RigidBody';
import Vec2 from '../math/Vec2';
import { BlockSolver } from './BlockSolver';
import { ContactPoint } from './Collision';
import { ContactSolver } from './ContactSolver';

export enum ContactType {
    Normal,
    Tangent,
}

export interface Jacobian {
    va: Vec2;
    wa: number;
    vb: Vec2;
    wb: number;
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
        // Ensure normal always points upward (y >= 0)
        const shouldFlip = contactNormal.y < 0;

        const finalBodyA = shouldFlip ? bodyB : bodyA;
        const finalBodyB = shouldFlip ? bodyA : bodyB;
        const finalNormal = shouldFlip ? contactNormal.negateNew() : contactNormal;
        const finalFlipped = shouldFlip ? !featureFlipped : featureFlipped;

        super(finalBodyA, finalBodyB);

        this.contactPoints = contactPoints;
        this.penetrationDepth = penetrationDepth;
        this.contactNormal = finalNormal;
        this.contactTangent = finalNormal.perpNew();
        this.featureFlipped = finalFlipped;

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
        // Solve normal constraint first
        if (this.numContacts == 1 || !SETTINGS.blockSolve) {
            for (let i = 0; i < this.numContacts; i++) {
                this.normalContacts[i].solve();
            }
        } else {
            // Solve two contact constraint in one shot using block solver
            this.blockSolver.solve();
        }

        for (let i = 0; i < this.numContacts; i++) {
            this.tangentContacts[i].solve(this.normalContacts[i]);
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
