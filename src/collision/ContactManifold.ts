import { Constraint } from '../constraint/Constraint';
import { SETTINGS } from '../core/Constants';
import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import * as Utils from '../utils/Utils';
import { BlockSolver } from './BlockSolver';
import { ContactSolver } from './ContactSolver';

export enum ContactType {
    Normal,
    Tangent,
}

export interface ContactPoint {
    point: Vec2;
    separation: number;
    id: number;
}

export interface ContactInfo {
    other: RigidBody;
    numContacts: number;
    contactDir: Vec2;
    contactPoints: Vec2[];
    impulse: number;
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
    public persistent = false;

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

    override preSolve(invDt: number): void {
        for (let i = 0; i < this.numContacts; i++) {
            this.normalContacts[i].preSolve(this.contactNormal, ContactType.Normal, this.featureFlipped, invDt);
            this.tangentContacts[i].preSolve(this.contactTangent, ContactType.Tangent, this.featureFlipped, invDt);
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
        } else // Solve two contact constraint in one shot using block solver
        {
            this.blockSolver.solve();
        }
    }

    tryWarmStart(oldManifold: ContactManifold) {
        for (let n = 0; n < this.numContacts; n++) {
            let o = 0;
            for (; o < oldManifold.numContacts; o++) {
                if (this.contactPoints[n].id == oldManifold.contactPoints[o].id) {
                    if (SETTINGS.applyWarmStartingThreshold) {
                        const dist = Utils.squaredDistance(
                            this.contactPoints[n].point,
                            oldManifold.contactPoints[o].point,
                        );
                        // If contact points are close enough, warm start.
                        // Otherwise, it means it's penetrating too deeply, skip the warm starting to prevent the overshoot
                        if (dist < SETTINGS.warmStartingThreshold) break;
                    } else {
                        break;
                    }
                }
            }

            if (o < oldManifold.numContacts) {
                this.normalContacts[n].impulseSum = oldManifold.normalContacts[o].impulseSum;
                this.tangentContacts[n].impulseSum = oldManifold.tangentContacts[o].impulseSum;

                this.persistent = true;
            }
        }
    }

    get points() {
        return this.contactPoints;
    }

    get normal() {
        return this.contactNormal;
    }

    get numContacts() {
        return this.contactPoints.length;
    }

    getContactInfo(flip: boolean): ContactInfo {
        const contactInfo: ContactInfo = {
            other: flip ? this.bodyB : this.bodyA,
            numContacts: this.numContacts,
            contactDir: flip ? this.contactNormal.negateNew() : this.contactNormal.copy(),
            contactPoints: [],
            impulse: 0,
        };

        for (let i = 0; i < this.numContacts; i++) {
            contactInfo.contactPoints.push(this.contactPoints[i].point.copy());
            contactInfo.impulse += this.normalContacts[i].impulseSum;
        }

        return contactInfo;
    }
}
