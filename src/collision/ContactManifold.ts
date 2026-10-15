import { Constraint } from '../constraint/Constraint';
import { SETTINGS } from '../core/Constants';
import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { ContactSolver } from './ContactSolver';

export interface ContactInfo {
    // Relevant contact infos
    bodyA: RigidBody;
    bodyB: RigidBody;
    impulseSum: number;
}

export class ContactManifold extends Constraint {
    // Contact informations
    public penetrationDepth!: number;

    public contactNormalX!: number;
    public contactNormalY!: number;
    public contactTangentX!: number;
    public contactTangentY!: number;

    public contactPoint0X!: number;
    public contactPoint0Y!: number;
    public contactPoint0Id!: number;

    public contactPoint1X!: number;
    public contactPoint1Y!: number;
    public contactPoint1Id!: number;

    private contactCount!: number;

    /** @internal */
    public beta!: number;
    /** @internal */
    public restitution!: number;
    /** @internal */
    public friction!: number;
    /** @internal */
    public featureFlipped!: boolean;
    public persistent = false;

    // Accumulated impulses, kept across steps for warm starting
    public normalImpulseSum0 = 0.0;
    public tangentImpulseSum0 = 0.0;
    public normalImpulseSum1 = 0.0;
    public tangentImpulseSum1 = 0.0;

    /** Used only when a manifold is solved on its own, World solves all its contacts at once */
    private static standaloneSolver: ContactSolver | null = null;
    private static readonly standaloneManifolds: ContactManifold[] = [];

    constructor();
    constructor(
        bodyA: RigidBody,
        bodyB: RigidBody,
        contactCount: number,
        penetrationDepth: number,
        contactNormalX: number,
        contactNormalY: number,
        contactPoint0X: number,
        contactPoint0Y: number,
        contactPoint0Id: number,
        contactPoint1X: number,
        contactPoint1Y: number,
        contactPoint1Id: number,
        featureFlipped: boolean,
    );
    constructor(
        bodyA?: RigidBody,
        bodyB?: RigidBody,
        contactCount?: number,
        penetrationDepth?: number,
        contactNormalX?: number,
        contactNormalY?: number,
        contactPoint0X?: number,
        contactPoint0Y?: number,
        contactPoint0Id?: number,
        contactPoint1X?: number,
        contactPoint1Y?: number,
        contactPoint1Id?: number,
        featureFlipped?: boolean,
    ) {
        super(bodyA as RigidBody, bodyB as RigidBody);

        if (
            bodyA != null &&
            bodyB != null &&
            contactCount != null &&
            penetrationDepth != null &&
            contactNormalX != null &&
            contactNormalY != null &&
            contactPoint0X != null &&
            contactPoint0Y != null &&
            contactPoint0Id != null &&
            contactPoint1X != null &&
            contactPoint1Y != null &&
            contactPoint1Id != null &&
            featureFlipped != null
        ) {
            this.init(
                bodyA,
                bodyB,
                contactCount,
                penetrationDepth,
                contactNormalX,
                contactNormalY,
                contactPoint0X,
                contactPoint0Y,
                contactPoint0Id,
                contactPoint1X,
                contactPoint1Y,
                contactPoint1Id,
                featureFlipped,
            );
        }
    }

    init(
        bodyA: RigidBody,
        bodyB: RigidBody,
        contactCount: number,
        penetrationDepth: number,
        contactNormalX: number,
        contactNormalY: number,
        contactPoint0X: number,
        contactPoint0Y: number,
        contactPoint0Id: number,
        contactPoint1X: number,
        contactPoint1Y: number,
        contactPoint1Id: number,
        featureFlipped: boolean,
    ): void {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.penetrationDepth = penetrationDepth;
        this.contactNormalX = contactNormalX;
        this.contactNormalY = contactNormalY;
        this.contactTangentX = -contactNormalY;
        this.contactTangentY = contactNormalX;

        this.contactCount = contactCount;

        this.contactPoint0X = contactPoint0X;
        this.contactPoint0Y = contactPoint0Y;
        this.contactPoint0Id = contactPoint0Id;

        if (this.contactCount === 2) {
            this.contactPoint1X = contactPoint1X;
            this.contactPoint1Y = contactPoint1Y;
            this.contactPoint1Id = contactPoint1Id;
        } else {
            this.contactPoint1X = 0.0;
            this.contactPoint1Y = 0.0;
            this.contactPoint1Id = 0;
        }

        this.featureFlipped = featureFlipped;

        this.beta = SETTINGS.positionCorrectionBeta;
        this.restitution = this.bodyA.restitution * this.bodyB.restitution;
        this.friction = this.bodyA.friction * this.bodyB.friction;

        this.persistent = false;

        this.normalImpulseSum0 = 0.0;
        this.tangentImpulseSum0 = 0.0;
        this.normalImpulseSum1 = 0.0;
        this.tangentImpulseSum1 = 0.0;
    }

    /** Prepares this contact on its own, World prepares all its contacts at once through ContactSolver */
    override preSolve(invDt: number, v: Float64Array): void {
        const manifolds = ContactManifold.standaloneManifolds;
        manifolds[0] = this;

        ContactManifold.standaloneSolver ??= new ContactSolver();
        ContactManifold.standaloneSolver.prepare(manifolds, invDt, v);
    }

    /** Solves this contact on its own, must follow a call to `preSolve` on the same manifold */
    override solve(v: Float64Array): void {
        const solver = ContactManifold.standaloneSolver as ContactSolver;
        const manifolds = ContactManifold.standaloneManifolds;

        solver.solve(v);
        solver.storeImpulses(manifolds);
    }

    tryWarmStart(oldManifold: ContactManifold) {
        if (
            this.matchesContact(
                this.contactPoint0X,
                this.contactPoint0Y,
                this.contactPoint0Id,
                oldManifold.contactPoint0X,
                oldManifold.contactPoint0Y,
                oldManifold.contactPoint0Id,
            )
        ) {
            this.copyWarmStartImpulse(0, oldManifold, 0);
        } else if (
            oldManifold.numContacts === 2 &&
            this.matchesContact(
                this.contactPoint0X,
                this.contactPoint0Y,
                this.contactPoint0Id,
                oldManifold.contactPoint1X,
                oldManifold.contactPoint1Y,
                oldManifold.contactPoint1Id,
            )
        ) {
            this.copyWarmStartImpulse(0, oldManifold, 1);
        }

        if (this.numContacts === 2) {
            if (
                this.matchesContact(
                    this.contactPoint1X,
                    this.contactPoint1Y,
                    this.contactPoint1Id,
                    oldManifold.contactPoint0X,
                    oldManifold.contactPoint0Y,
                    oldManifold.contactPoint0Id,
                )
            ) {
                this.copyWarmStartImpulse(1, oldManifold, 0);
            } else if (
                oldManifold.numContacts === 2 &&
                this.matchesContact(
                    this.contactPoint1X,
                    this.contactPoint1Y,
                    this.contactPoint1Id,
                    oldManifold.contactPoint1X,
                    oldManifold.contactPoint1Y,
                    oldManifold.contactPoint1Id,
                )
            ) {
                this.copyWarmStartImpulse(1, oldManifold, 1);
            }
        }
    }

    private matchesContact(
        contactPointX: number,
        contactPointY: number,
        contactPointId: number,
        oldContactPointX: number,
        oldContactPointY: number,
        oldContactPointId: number,
    ): boolean {
        if (contactPointId !== oldContactPointId) {
            return false;
        }

        if (!SETTINGS.applyWarmStartingThreshold) {
            return true;
        }

        const dx = contactPointX - oldContactPointX;
        const dy = contactPointY - oldContactPointY;

        // If contact points are close enough, warm start.
        // Otherwise, it means it's penetrating too deeply, skip the warm starting to prevent the overshoot
        return dx * dx + dy * dy < SETTINGS.warmStartingThreshold;
    }

    private copyWarmStartImpulse(index: number, oldManifold: ContactManifold, oldIndex: number): void {
        if (index === 0) {
            if (oldIndex === 0) {
                this.normalImpulseSum0 = oldManifold.normalImpulseSum0;
                this.tangentImpulseSum0 = oldManifold.tangentImpulseSum0;
            } else {
                this.normalImpulseSum0 = oldManifold.normalImpulseSum1;
                this.tangentImpulseSum0 = oldManifold.tangentImpulseSum1;
            }
        } else if (oldIndex === 0) {
            this.normalImpulseSum1 = oldManifold.normalImpulseSum0;
            this.tangentImpulseSum1 = oldManifold.tangentImpulseSum0;
        } else {
            this.normalImpulseSum1 = oldManifold.normalImpulseSum1;
            this.tangentImpulseSum1 = oldManifold.tangentImpulseSum1;
        }

        this.persistent = true;
    }

    get points() {
        if (this.contactCount === 1) {
            return [
                {
                    point: new Vec2(this.contactPoint0X, this.contactPoint0Y),
                    id: this.contactPoint0Id,
                },
            ];
        }

        return [
            {
                point: new Vec2(this.contactPoint0X, this.contactPoint0Y),
                id: this.contactPoint0Id,
            },
            {
                point: new Vec2(this.contactPoint1X, this.contactPoint1Y),
                id: this.contactPoint1Id,
            },
        ];
    }

    get normal() {
        return new Vec2(this.contactNormalX, this.contactNormalY);
    }

    get numContacts() {
        return this.contactCount;
    }

    get contactInfo() {
        // Return relevant info
        return {
            bodyA: this.bodyA,
            bodyB: this.bodyB,
            impulseSum: this.normalImpulseSum0 + this.normalImpulseSum1,
        };
    }
}
