import { Constraint } from '../constraint/Constraint';
import { ContactSoftness, makeSoft, SETTINGS } from '../core/Constants';
import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import * as Utils from '../utils/Utils';

const BLOCK_SOLVER_EPSILON = 1e-12;
const BLOCK_SOLVER_MAX_CONDITION_NUMBER = 1_000;
const NORMAL_LENGTH_TOLERANCE = 1e-6;
const IS_DEVELOPMENT_BUILD =
    (globalThis as { process?: { env?: { NODE_ENV?: string } } }).process?.env?.NODE_ENV !== 'production';

export interface ContactInfo {
    // Relevant contact infos
    bodyA: RigidBody;
    bodyB: RigidBody;
    impulseSum: number;
}

export interface ContactPoint {
    point: Vec2;
    id: number;
    separation: number;
    baseSeparation: number;
    localAnchorA: Vec2;
    localAnchorB: Vec2;
    persisted: boolean;
    normalImpulse: number;
    tangentImpulse: number;
    totalNormalImpulse: number;
    normalVelocity: number;
    restitutionVelocity: number;
}

export class ContactManifold extends Constraint {
    // Contact informations
    public contactNormalX!: number;
    public contactNormalY!: number;
    public contactTangentX!: number;
    public contactTangentY!: number;

    public contactPoint0X!: number;
    public contactPoint0Y!: number;
    public contactPoint0Id!: number;
    public contactPoint0Separation!: number;
    public contactPoint0BaseSeparation!: number;
    public contactPoint0LocalAnchorAX!: number;
    public contactPoint0LocalAnchorAY!: number;
    public contactPoint0LocalAnchorBX!: number;
    public contactPoint0LocalAnchorBY!: number;
    public contactPoint0Persisted = false;
    public contactPoint0NormalVelocity = 0.0;
    public contactPoint0RestitutionVelocity = 0.0;
    public contactPoint0TotalNormalImpulse = 0.0;

    public contactPoint1X!: number;
    public contactPoint1Y!: number;
    public contactPoint1Id!: number;
    public contactPoint1Separation!: number;
    public contactPoint1BaseSeparation!: number;
    public contactPoint1LocalAnchorAX!: number;
    public contactPoint1LocalAnchorAY!: number;
    public contactPoint1LocalAnchorBX!: number;
    public contactPoint1LocalAnchorBY!: number;
    public contactPoint1Persisted = false;
    public contactPoint1NormalVelocity = 0.0;
    public contactPoint1RestitutionVelocity = 0.0;
    public contactPoint1TotalNormalImpulse = 0.0;

    private contactCount!: number;

    private restitution!: number;
    private friction!: number;
    private rollingResistance = 0.0;
    private rollingEffectiveMass = 0.0;
    public rollingImpulseSum = 0.0;
    private featureFlipped!: boolean;
    /** Compatibility view of the per-point persistence state. */
    get persistent(): boolean {
        return this.contactPoint0Persisted || (this.contactCount === 2 && this.contactPoint1Persisted);
    }

    public normalJvaX = 0.0;
    public normalJvaY = 0.0;
    public normalJvbX = 0.0;
    public normalJvbY = 0.0;

    public tangentJvaX = 0.0;
    public tangentJvaY = 0.0;
    public tangentJvbX = 0.0;
    public tangentJvbY = 0.0;

    public normalJwa0 = 0.0;
    public normalJwb0 = 0.0;
    public tangentJwa0 = 0.0;
    public tangentJwb0 = 0.0;
    public normalBias0 = 0.0;
    public tangentBias0 = 0.0;
    public normalEffectiveMass0 = 0.0;
    public normalMassScale0 = 1.0;
    public normalImpulseScale0 = 0.0;
    public tangentEffectiveMass0 = 0.0;
    public normalImpulseSum0 = 0.0;
    public tangentImpulseSum0 = 0.0;

    public normalJwa1 = 0.0;
    public normalJwb1 = 0.0;
    public tangentJwa1 = 0.0;
    public tangentJwb1 = 0.0;
    public normalBias1 = 0.0;
    public tangentBias1 = 0.0;
    public normalEffectiveMass1 = 0.0;
    public normalMassScale1 = 1.0;
    public normalImpulseScale1 = 0.0;
    public tangentEffectiveMass1 = 0.0;
    public normalImpulseSum1 = 0.0;
    public tangentImpulseSum1 = 0.0;

    private blockK00 = 0.0;
    private blockK01 = 0.0;
    private blockK11 = 0.0;
    private blockM00 = 0.0;
    private blockM01 = 0.0;
    private blockM11 = 0.0;
    private blockSolveReady = false;

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
        contactPoint0Separation?: number,
        contactPoint1Separation?: number,
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
        contactPoint0Separation?: number,
        contactPoint1Separation?: number,
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
                contactPoint0Separation,
                contactPoint1Separation,
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
        contactPoint0Separation = -penetrationDepth,
        contactPoint1Separation = -penetrationDepth,
    ): void {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.contactNormalX = contactNormalX;
        this.contactNormalY = contactNormalY;
        this.contactTangentX = -contactNormalY;
        this.contactTangentY = contactNormalX;

        this.contactCount = contactCount;
        const bodyACos = Math.cos(bodyA.rotation);
        const bodyASin = Math.sin(bodyA.rotation);
        const bodyBCos = Math.cos(bodyB.rotation);
        const bodyBSin = Math.sin(bodyB.rotation);

        this.contactPoint0X = contactPoint0X;
        this.contactPoint0Y = contactPoint0Y;
        this.contactPoint0Id = contactPoint0Id;
        this.contactPoint0Separation = contactPoint0Separation;
        this.contactPoint0BaseSeparation = contactPoint0Separation;
        this.setLocalAnchors(0, contactPoint0X, contactPoint0Y, bodyACos, bodyASin, bodyBCos, bodyBSin);

        if (this.contactCount === 2) {
            this.contactPoint1X = contactPoint1X;
            this.contactPoint1Y = contactPoint1Y;
            this.contactPoint1Id = contactPoint1Id;
            this.contactPoint1Separation = contactPoint1Separation;
            this.contactPoint1BaseSeparation = contactPoint1Separation;
            this.setLocalAnchors(1, contactPoint1X, contactPoint1Y, bodyACos, bodyASin, bodyBCos, bodyBSin);
        } else {
            this.contactPoint1X = 0.0;
            this.contactPoint1Y = 0.0;
            this.contactPoint1Id = 0;
            this.contactPoint1Separation = 0.0;
            this.contactPoint1BaseSeparation = 0.0;
            this.contactPoint1LocalAnchorAX = 0.0;
            this.contactPoint1LocalAnchorAY = 0.0;
            this.contactPoint1LocalAnchorBX = 0.0;
            this.contactPoint1LocalAnchorBY = 0.0;
        }

        this.featureFlipped = featureFlipped;

        this.restitution = this.bodyA.restitution * this.bodyB.restitution;
        this.friction = this.bodyA.friction * this.bodyB.friction;
        this.rollingResistance = Math.max(this.bodyA.rollingResistance, this.bodyB.rollingResistance) *
            Math.max(this.bodyA.shape.radius, this.bodyB.shape.radius);
        this.rollingEffectiveMass = 0.0;
        this.rollingImpulseSum = 0.0;

        this.normalJvaX = -this.contactNormalX;
        this.normalJvaY = -this.contactNormalY;
        this.normalJvbX = this.contactNormalX;
        this.normalJvbY = this.contactNormalY;

        this.tangentJvaX = -this.contactTangentX;
        this.tangentJvaY = -this.contactTangentY;
        this.tangentJvbX = this.contactTangentX;
        this.tangentJvbY = this.contactTangentY;

        this.contactPoint0Persisted = false;
        this.contactPoint0NormalVelocity = 0.0;
        this.contactPoint0RestitutionVelocity = 0.0;
        this.contactPoint0TotalNormalImpulse = 0.0;
        this.contactPoint1Persisted = false;
        this.contactPoint1NormalVelocity = 0.0;
        this.contactPoint1RestitutionVelocity = 0.0;
        this.contactPoint1TotalNormalImpulse = 0.0;

        this.normalJwa0 = 0.0;
        this.normalJwb0 = 0.0;
        this.tangentJwa0 = 0.0;
        this.tangentJwb0 = 0.0;
        this.normalBias0 = 0.0;
        this.tangentBias0 = 0.0;
        this.normalEffectiveMass0 = 0.0;
        this.tangentEffectiveMass0 = 0.0;
        this.normalImpulseSum0 = 0.0;
        this.tangentImpulseSum0 = 0.0;

        this.normalJwa1 = 0.0;
        this.normalJwb1 = 0.0;
        this.tangentJwa1 = 0.0;
        this.tangentJwb1 = 0.0;
        this.normalBias1 = 0.0;
        this.tangentBias1 = 0.0;
        this.normalEffectiveMass1 = 0.0;
        this.tangentEffectiveMass1 = 0.0;
        this.normalImpulseSum1 = 0.0;
        this.tangentImpulseSum1 = 0.0;

        this.blockK00 = 0.0;
        this.blockK01 = 0.0;
        this.blockK11 = 0.0;
        this.blockM00 = 0.0;
        this.blockM01 = 0.0;
        this.blockM11 = 0.0;
        this.blockSolveReady = false;

        this.validateGeneratedManifold();
    }

    setMaterialProperties(friction: number, restitution: number): void {
        Utils.assert(Number.isFinite(friction) && friction >= 0.0, 'Invalid contact friction');
        Utils.assert(Number.isFinite(restitution) && restitution >= 0.0, 'Invalid contact restitution');
        this.friction = friction;
        this.restitution = restitution;
    }

    override preSolve(invDt: number, contactSoftness?: ContactSoftness, staticSoftness?: ContactSoftness): void {
        const tangentBias = this.featureFlipped
            ? this.bodyB.surfaceSpeed - this.bodyA.surfaceSpeed
            : this.bodyA.surfaceSpeed - this.bodyB.surfaceSpeed;
        const hertz = Math.min(SETTINGS.contactHertz, 0.125 * invDt);
        const softness = (this.bodyA.isStatic() || this.bodyB.isStatic())
            ? (staticSoftness ?? makeSoft(2 * hertz, SETTINGS.contactDampingRatio, invDt > 0 ? 1 / invDt : 0))
            : (contactSoftness ?? makeSoft(hertz, SETTINGS.contactDampingRatio, invDt > 0 ? 1 / invDt : 0));

        for (let i = 0; i < this.numContacts; i++) {
            this.preSolveContact(i, tangentBias, invDt, softness);
        }

        const rollingK = this.bodyA.invI + this.bodyB.invI;
        this.rollingEffectiveMass = rollingK > 0.0 ? 1.0 / rollingK : 0.0;
        if (SETTINGS.warmStarting) {
            this.applyRollingImpulse(this.rollingImpulseSum);
        }

        if (this.numContacts === 2 && SETTINGS.blockSolve && softness.impulseScale === 0.0) {
            this.preSolveBlock();
        }
    }

    override solve(): void {
        this.solveBias();
        this.solveRestitutionAndFriction();
    }

    /** Solves speculative and penetration correction before position integration. */
    solveBias(): void {
        if (this.numContacts === 1 || !SETTINGS.blockSolve || !this.blockSolveReady) {
            for (let i = 0; i < this.numContacts; i++) {
                this.solveNormalContact(i);
            }
        } else {
            this.solveBlock();
        }
    }

    /** Solves restitution and friction after position integration. */
    solveRestitutionAndFriction(): void {
        for (let i = 0; i < this.numContacts; i++) this.solveRestitutionContact(i);
        this.solveRollingResistance();
        for (let i = 0; i < this.numContacts; i++) this.solveTangentContact(i);
    }

    tryWarmStart(oldManifold: ContactManifold) {
        let matched = false;
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
            matched = true;
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
            matched = true;
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
                matched = true;
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
                matched = true;
            }
        }

        if (matched) {
            this.rollingImpulseSum = oldManifold.rollingImpulseSum;
        }
    }

    private preSolveContact(index: number, tangentBias: number, invDt: number, softness: ContactSoftness): void {
        const contactPointX = index === 0 ? this.contactPoint0X : this.contactPoint1X;
        const contactPointY = index === 0 ? this.contactPoint0Y : this.contactPoint1Y;

        const bodyAPosition = this.bodyA.position;
        const bodyBPosition = this.bodyB.position;
        const raX = contactPointX - bodyAPosition.x;
        const raY = contactPointY - bodyAPosition.y;
        const rbX = contactPointX - bodyBPosition.x;
        const rbY = contactPointY - bodyBPosition.y;

        const normalJwa = raY * this.contactNormalX - raX * this.contactNormalY;
        const normalJwb = rbX * this.contactNormalY - rbY * this.contactNormalX;
        const tangentJwa = raY * this.contactTangentX - raX * this.contactTangentY;
        const tangentJwb = rbX * this.contactTangentY - rbY * this.contactTangentX;
        const bodyAVelocity = this.bodyA.velocity;
        const bodyBVelocity = this.bodyB.velocity;
        const bodyAAngularVelocity = this.bodyA.angularVelocity;
        const bodyBAngularVelocity = this.bodyB.angularVelocity;
        const relativeVelocityX =
            bodyBVelocity.x - bodyBAngularVelocity * rbY - (bodyAVelocity.x - bodyAAngularVelocity * raY);
        const relativeVelocityY =
            bodyBVelocity.y + bodyBAngularVelocity * rbX - (bodyAVelocity.y + bodyAAngularVelocity * raX);
        const normalVelocity = this.contactNormalX * relativeVelocityX + this.contactNormalY * relativeVelocityY;

        if (index === 0) {
            this.contactPoint0NormalVelocity = normalVelocity;
        } else {
            this.contactPoint1NormalVelocity = normalVelocity;
        }

        const separation = index === 0 ? this.contactPoint0Separation : this.contactPoint1Separation;
        let normalBias = 0.0;
        let normalMassScale = 1.0;
        let normalImpulseScale = 0.0;

        if (separation > 0.0) {
            // Speculative contacts allow closing motion only up to the available gap.
            normalBias = separation * invDt;
        } else if (SETTINGS.positionCorrection) {
            normalBias = Math.max(
                softness.massScale * softness.biasRate * separation,
                -SETTINGS.contactPushSpeed,
            );
            normalMassScale = softness.massScale;
            normalImpulseScale = softness.impulseScale;
        }

        const bodyAInvMass = this.bodyA.invMass;
        const bodyAInvI = this.bodyA.invI;
        const bodyBInvMass = this.bodyB.invMass;
        const bodyBInvI = this.bodyB.invI;

        const normalK =
            bodyAInvMass + normalJwa * bodyAInvI * normalJwa + bodyBInvMass + normalJwb * bodyBInvI * normalJwb;

        const tangentK =
            bodyAInvMass + tangentJwa * bodyAInvI * tangentJwa + bodyBInvMass + tangentJwb * bodyBInvI * tangentJwb;

        const normalEffectiveMass = normalK > 0.0 ? 1.0 / normalK : 0.0;
        const tangentEffectiveMass = tangentK > 0.0 ? 1.0 / tangentK : 0.0;

        if (index === 0) {
            this.normalJwa0 = normalJwa;
            this.normalJwb0 = normalJwb;
            this.tangentJwa0 = tangentJwa;
            this.tangentJwb0 = tangentJwb;
            this.normalBias0 = normalBias;
            this.normalMassScale0 = normalMassScale;
            this.normalImpulseScale0 = normalImpulseScale;
            this.tangentBias0 = tangentBias;
            this.normalEffectiveMass0 = normalEffectiveMass;
            this.tangentEffectiveMass0 = tangentEffectiveMass;

            if (SETTINGS.warmStarting) {
                this.contactPoint0TotalNormalImpulse += this.normalImpulseSum0;
                this.applyNormalImpulse(0, this.normalImpulseSum0);
                this.applyTangentImpulse(0, this.tangentImpulseSum0);
            }
        } else {
            this.normalJwa1 = normalJwa;
            this.normalJwb1 = normalJwb;
            this.tangentJwa1 = tangentJwa;
            this.tangentJwb1 = tangentJwb;
            this.normalBias1 = normalBias;
            this.normalMassScale1 = normalMassScale;
            this.normalImpulseScale1 = normalImpulseScale;
            this.tangentBias1 = tangentBias;
            this.normalEffectiveMass1 = normalEffectiveMass;
            this.tangentEffectiveMass1 = tangentEffectiveMass;

            if (SETTINGS.warmStarting) {
                this.contactPoint1TotalNormalImpulse += this.normalImpulseSum1;
                this.applyNormalImpulse(1, this.normalImpulseSum1);
                this.applyTangentImpulse(1, this.tangentImpulseSum1);
            }
        }
    }

    private preSolveBlock(): void {
        const bodyAInvMass = this.bodyA.invMass;
        const bodyAInvI = this.bodyA.invI;
        const bodyBInvMass = this.bodyB.invMass;
        const bodyBInvI = this.bodyB.invI;

        this.blockK00 =
            bodyAInvMass +
            this.normalJwa0 * bodyAInvI * this.normalJwa0 +
            bodyBInvMass +
            this.normalJwb0 * bodyBInvI * this.normalJwb0;

        this.blockK11 =
            bodyAInvMass +
            this.normalJwa1 * bodyAInvI * this.normalJwa1 +
            bodyBInvMass +
            this.normalJwb1 * bodyBInvI * this.normalJwb1;

        this.blockK01 =
            bodyAInvMass +
            this.normalJwa0 * bodyAInvI * this.normalJwa1 +
            bodyBInvMass +
            this.normalJwb0 * bodyBInvI * this.normalJwb1;

        const determinant = this.blockK00 * this.blockK11 - this.blockK01 * this.blockK01;

        this.blockSolveReady =
            Number.isFinite(determinant) &&
            determinant > BLOCK_SOLVER_EPSILON &&
            this.blockK00 * this.blockK00 < BLOCK_SOLVER_MAX_CONDITION_NUMBER * determinant;

        if (!this.blockSolveReady) {
            return;
        }

        const invDeterminant = 1.0 / determinant;
        this.blockM00 = invDeterminant * this.blockK11;
        this.blockM01 = -invDeterminant * this.blockK01;
        this.blockM11 = invDeterminant * this.blockK00;
    }

    // TODO: to be deleted after implementation
    private validateGeneratedManifold(): void {
        if (!IS_DEVELOPMENT_BUILD) {
            return;
        }

        Utils.assert(Number.isInteger(this.contactCount) && this.contactCount >= 1 && this.contactCount <= 2);
        Utils.assert(
            Number.isFinite(this.contactNormalX),
            Number.isFinite(this.contactNormalY),
            Number.isFinite(this.contactPoint0X),
            Number.isFinite(this.contactPoint0Y),
            Number.isFinite(this.contactPoint0Separation),
            Number.isFinite(this.contactPoint0BaseSeparation),
            Number.isFinite(this.contactPoint0LocalAnchorAX),
            Number.isFinite(this.contactPoint0LocalAnchorAY),
            Number.isFinite(this.contactPoint0LocalAnchorBX),
            Number.isFinite(this.contactPoint0LocalAnchorBY),
            Number.isSafeInteger(this.contactPoint0Id),
            'Invalid contact manifold values',
        );

        const normalLengthSquared = this.contactNormalX * this.contactNormalX + this.contactNormalY * this.contactNormalY;
        Utils.assert(
            Math.abs(normalLengthSquared - 1.0) <= NORMAL_LENGTH_TOLERANCE,
            'Contact manifold normal must be unit length',
        );

        if (this.contactCount === 2) {
            Utils.assert(
                Number.isFinite(this.contactPoint1X),
                Number.isFinite(this.contactPoint1Y),
                Number.isSafeInteger(this.contactPoint1Id),
                Number.isFinite(this.contactPoint1Separation),
                Number.isFinite(this.contactPoint1BaseSeparation),
                Number.isFinite(this.contactPoint1LocalAnchorAX),
                Number.isFinite(this.contactPoint1LocalAnchorAY),
                Number.isFinite(this.contactPoint1LocalAnchorBX),
                Number.isFinite(this.contactPoint1LocalAnchorBY),
                this.contactPoint0Id !== this.contactPoint1Id,
                'Invalid two-point contact manifold',
            );
        }
    }

    private solveNormalContact(index: number): void {
        const normalJwa = index === 0 ? this.normalJwa0 : this.normalJwa1;
        const normalJwb = index === 0 ? this.normalJwb0 : this.normalJwb1;
        const normalBias = index === 0 ? this.normalBias0 : this.normalBias1;
        const normalEffectiveMass = index === 0 ? this.normalEffectiveMass0 : this.normalEffectiveMass1;
        const normalMassScale = index === 0 ? this.normalMassScale0 : this.normalMassScale1;
        const normalImpulseScale = index === 0 ? this.normalImpulseScale0 : this.normalImpulseScale1;
        const oldImpulseSum = index === 0 ? this.normalImpulseSum0 : this.normalImpulseSum1;

        const bodyAVelocity = this.bodyA.velocity;
        const bodyBVelocity = this.bodyB.velocity;
        const jv =
            this.normalJvaX * bodyAVelocity.x +
            this.normalJvaY * bodyAVelocity.y +
            normalJwa * this.bodyA.angularVelocity +
            this.normalJvbX * bodyBVelocity.x +
            this.normalJvbY * bodyBVelocity.y +
            normalJwb * this.bodyB.angularVelocity;

        let lambda = -normalEffectiveMass * (normalMassScale * jv + normalBias) - normalImpulseScale * oldImpulseSum;
        let impulseSum = oldImpulseSum;

        if (SETTINGS.impulseAccumulation) {
            impulseSum = Math.max(0.0, oldImpulseSum + lambda);
            lambda = impulseSum - oldImpulseSum;
        } else {
            impulseSum = Math.max(0.0, lambda);
            lambda = impulseSum;
        }

        if (index === 0) {
            this.normalImpulseSum0 = impulseSum;
            this.contactPoint0TotalNormalImpulse += lambda;
        } else {
            this.normalImpulseSum1 = impulseSum;
            this.contactPoint1TotalNormalImpulse += lambda;
        }

        this.applyNormalImpulse(index, lambda);
    }

    private solveRestitutionContact(index: number): void {
        const restitutionVelocity = index === 0
            ? this.contactPoint0RestitutionVelocity
            : this.contactPoint1RestitutionVelocity;
        const totalNormalImpulse = index === 0
            ? this.contactPoint0TotalNormalImpulse
            : this.contactPoint1TotalNormalImpulse;
        const normalVelocity = index === 0
            ? this.contactPoint0NormalVelocity
            : this.contactPoint1NormalVelocity;

        if (restitutionVelocity === 0.0 && totalNormalImpulse > 0.0 && normalVelocity < -SETTINGS.restitutionSlop) {
            const target = -this.restitution * normalVelocity;
            if (index === 0) {
                this.contactPoint0RestitutionVelocity = target;
            } else {
                this.contactPoint1RestitutionVelocity = target;
            }
        }

        const targetVelocity = index === 0 ? this.contactPoint0RestitutionVelocity : this.contactPoint1RestitutionVelocity;
        if (targetVelocity === 0.0) {
            return;
        }

        const normalJwa = index === 0 ? this.normalJwa0 : this.normalJwa1;
        const normalJwb = index === 0 ? this.normalJwb0 : this.normalJwb1;
        const normalEffectiveMass = index === 0 ? this.normalEffectiveMass0 : this.normalEffectiveMass1;
        const oldImpulseSum = index === 0 ? this.normalImpulseSum0 : this.normalImpulseSum1;
        const bodyAVelocity = this.bodyA.velocity;
        const bodyBVelocity = this.bodyB.velocity;
        const jv =
            this.normalJvaX * bodyAVelocity.x +
            this.normalJvaY * bodyAVelocity.y +
            normalJwa * this.bodyA.angularVelocity +
            this.normalJvbX * bodyBVelocity.x +
            this.normalJvbY * bodyBVelocity.y +
            normalJwb * this.bodyB.angularVelocity;
        const lambda = normalEffectiveMass * -(jv - targetVelocity);
        const impulseSum = Math.max(0.0, oldImpulseSum + lambda);
        const appliedImpulse = impulseSum - oldImpulseSum;

        if (index === 0) {
            this.normalImpulseSum0 = impulseSum;
            this.contactPoint0TotalNormalImpulse += appliedImpulse;
        } else {
            this.normalImpulseSum1 = impulseSum;
            this.contactPoint1TotalNormalImpulse += appliedImpulse;
        }

        this.applyNormalImpulse(index, appliedImpulse);
    }

    private solveRollingResistance(): void {
        if (this.rollingEffectiveMass === 0.0 || this.rollingResistance === 0.0) {
            return;
        }

        const totalNormalImpulse = this.normalImpulseSum0 + (this.numContacts === 2 ? this.normalImpulseSum1 : 0.0);
        const limit = this.rollingResistance * totalNormalImpulse;
        const oldImpulse = this.rollingImpulseSum;
        this.rollingImpulseSum = Utils.clamp(
            oldImpulse - this.rollingEffectiveMass * (this.bodyB.angularVelocity - this.bodyA.angularVelocity),
            -limit,
            limit,
        );
        this.applyRollingImpulse(this.rollingImpulseSum - oldImpulse);
    }

    private solveBlock(): void {
        const aX = this.normalImpulseSum0;
        const aY = this.normalImpulseSum1;
        Utils.assert(aX >= 0.0, aY >= 0.0);

        const bodyAVelocity = this.bodyA.velocity;
        const bodyBVelocity = this.bodyB.velocity;
        const bodyAAngularVelocity = this.bodyA.angularVelocity;
        const bodyBAngularVelocity = this.bodyB.angularVelocity;

        let vn1 =
            this.normalJvaX * bodyAVelocity.x +
            this.normalJvaY * bodyAVelocity.y +
            this.normalJwa0 * bodyAAngularVelocity +
            this.normalJvbX * bodyBVelocity.x +
            this.normalJvbY * bodyBVelocity.y +
            this.normalJwb0 * bodyBAngularVelocity;

        let vn2 =
            this.normalJvaX * bodyAVelocity.x +
            this.normalJvaY * bodyAVelocity.y +
            this.normalJwa1 * bodyAAngularVelocity +
            this.normalJvbX * bodyBVelocity.x +
            this.normalJvbY * bodyBVelocity.y +
            this.normalJwb1 * bodyBAngularVelocity;

        let bX = vn1 + this.normalBias0;
        let bY = vn2 + this.normalBias1;

        bX -= this.blockK00 * aX + this.blockK01 * aY;
        bY -= this.blockK01 * aX + this.blockK11 * aY;

        let xX = -(this.blockM00 * bX + this.blockM01 * bY);
        let xY = -(this.blockM01 * bX + this.blockM11 * bY);
        let solved = xX >= 0.0 && xY >= 0.0;

        if (!solved) {
            xX = this.normalEffectiveMass0 * -bX;
            xY = 0.0;
            vn1 = 0.0;
            vn2 = this.blockK01 * xX + bY;
            solved = xX >= 0.0 && vn2 >= 0.0;
        }

        if (!solved) {
            xX = 0.0;
            xY = this.normalEffectiveMass1 * -bY;
            vn1 = this.blockK01 * xY + bX;
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
            console.error('Error solving contact block: ', this);
            Utils.assert(false);
        }

        this.applyBlockImpulse(xX - aX, xY - aY);

        this.normalImpulseSum0 = xX;
        this.normalImpulseSum1 = xY;
        this.contactPoint0TotalNormalImpulse += xX - aX;
        this.contactPoint1TotalNormalImpulse += xY - aY;
    }

    private solveTangentContact(index: number): void {
        const tangentJwa = index === 0 ? this.tangentJwa0 : this.tangentJwa1;
        const tangentJwb = index === 0 ? this.tangentJwb0 : this.tangentJwb1;
        const tangentBias = index === 0 ? this.tangentBias0 : this.tangentBias1;
        const tangentEffectiveMass = index === 0 ? this.tangentEffectiveMass0 : this.tangentEffectiveMass1;
        const oldImpulseSum = index === 0 ? this.tangentImpulseSum0 : this.tangentImpulseSum1;
        const maxFriction = this.friction * (index === 0 ? this.normalImpulseSum0 : this.normalImpulseSum1);

        const bodyAVelocity = this.bodyA.velocity;
        const bodyBVelocity = this.bodyB.velocity;
        const jv =
            this.tangentJvaX * bodyAVelocity.x +
            this.tangentJvaY * bodyAVelocity.y +
            tangentJwa * this.bodyA.angularVelocity +
            this.tangentJvbX * bodyBVelocity.x +
            this.tangentJvbY * bodyBVelocity.y +
            tangentJwb * this.bodyB.angularVelocity;

        let lambda = tangentEffectiveMass * -(jv + tangentBias);
        let impulseSum = oldImpulseSum;

        if (SETTINGS.impulseAccumulation) {
            impulseSum = Utils.clamp(oldImpulseSum + lambda, -maxFriction, maxFriction);
            lambda = impulseSum - oldImpulseSum;
        } else {
            impulseSum = Utils.clamp(lambda, -maxFriction, maxFriction);
            lambda = impulseSum;
        }

        if (index === 0) {
            this.tangentImpulseSum0 = impulseSum;
        } else {
            this.tangentImpulseSum1 = impulseSum;
        }

        this.applyTangentImpulse(index, lambda);
    }

    private applyNormalImpulse(index: number, lambda: number): void {
        if (lambda === 0.0) {
            return;
        }

        const bodyAImpulseScale = this.bodyA.invMass * lambda;
        this.bodyA.velocity.x += this.normalJvaX * bodyAImpulseScale;
        this.bodyA.velocity.y += this.normalJvaY * bodyAImpulseScale;
        this.bodyA.angularVelocity += this.bodyA.invI * (index === 0 ? this.normalJwa0 : this.normalJwa1) * lambda;

        const bodyBImpulseScale = this.bodyB.invMass * lambda;
        this.bodyB.velocity.x += this.normalJvbX * bodyBImpulseScale;
        this.bodyB.velocity.y += this.normalJvbY * bodyBImpulseScale;
        this.bodyB.angularVelocity += this.bodyB.invI * (index === 0 ? this.normalJwb0 : this.normalJwb1) * lambda;
    }

    private applyBlockImpulse(lambdaX: number, lambdaY: number): void {
        if (lambdaX === 0.0 && lambdaY === 0.0) {
            return;
        }

        const linearImpulse = lambdaX + lambdaY;

        const bodyAImpulseScale = this.bodyA.invMass * linearImpulse;
        this.bodyA.velocity.x += this.normalJvaX * bodyAImpulseScale;
        this.bodyA.velocity.y += this.normalJvaY * bodyAImpulseScale;
        this.bodyA.angularVelocity += this.bodyA.invI * (this.normalJwa0 * lambdaX + this.normalJwa1 * lambdaY);

        const bodyBImpulseScale = this.bodyB.invMass * linearImpulse;
        this.bodyB.velocity.x += this.normalJvbX * bodyBImpulseScale;
        this.bodyB.velocity.y += this.normalJvbY * bodyBImpulseScale;
        this.bodyB.angularVelocity += this.bodyB.invI * (this.normalJwb0 * lambdaX + this.normalJwb1 * lambdaY);
    }

    private applyTangentImpulse(index: number, lambda: number): void {
        if (lambda === 0.0) {
            return;
        }

        const bodyAImpulseScale = this.bodyA.invMass * lambda;
        this.bodyA.velocity.x += this.tangentJvaX * bodyAImpulseScale;
        this.bodyA.velocity.y += this.tangentJvaY * bodyAImpulseScale;
        this.bodyA.angularVelocity += this.bodyA.invI * (index === 0 ? this.tangentJwa0 : this.tangentJwa1) * lambda;

        const bodyBImpulseScale = this.bodyB.invMass * lambda;
        this.bodyB.velocity.x += this.tangentJvbX * bodyBImpulseScale;
        this.bodyB.velocity.y += this.tangentJvbY * bodyBImpulseScale;
        this.bodyB.angularVelocity += this.bodyB.invI * (index === 0 ? this.tangentJwb0 : this.tangentJwb1) * lambda;
    }

    private applyRollingImpulse(lambda: number): void {
        if (lambda === 0.0) {
            return;
        }

        this.bodyA.angularVelocity -= this.bodyA.invI * lambda;
        this.bodyB.angularVelocity += this.bodyB.invI * lambda;
    }

    private setLocalAnchors(
        index: number,
        pointX: number,
        pointY: number,
        bodyACos: number,
        bodyASin: number,
        bodyBCos: number,
        bodyBSin: number,
    ): void {
        const bodyAPosition = this.bodyA.position;
        const bodyBPosition = this.bodyB.position;
        const bodyADx = pointX - bodyAPosition.x;
        const bodyADy = pointY - bodyAPosition.y;
        const bodyBDx = pointX - bodyBPosition.x;
        const bodyBDy = pointY - bodyBPosition.y;
        const anchorAX = bodyACos * bodyADx + bodyASin * bodyADy;
        const anchorAY = -bodyASin * bodyADx + bodyACos * bodyADy;
        const anchorBX = bodyBCos * bodyBDx + bodyBSin * bodyBDy;
        const anchorBY = -bodyBSin * bodyBDx + bodyBCos * bodyBDy;

        if (index === 0) {
            this.contactPoint0LocalAnchorAX = anchorAX;
            this.contactPoint0LocalAnchorAY = anchorAY;
            this.contactPoint0LocalAnchorBX = anchorBX;
            this.contactPoint0LocalAnchorBY = anchorBY;
        } else {
            this.contactPoint1LocalAnchorAX = anchorAX;
            this.contactPoint1LocalAnchorAY = anchorAY;
            this.contactPoint1LocalAnchorBX = anchorBX;
            this.contactPoint1LocalAnchorBY = anchorBY;
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

        if (index === 0) {
            this.contactPoint0Persisted = true;
        } else {
            this.contactPoint1Persisted = true;
        }
    }

    get penetrationDepth(): number {
        const minimumSeparation = this.contactCount === 2
            ? Math.min(this.contactPoint0Separation, this.contactPoint1Separation)
            : this.contactPoint0Separation;
        return Math.max(0.0, -minimumSeparation);
    }

    get points(): ContactPoint[] {
        if (this.contactCount === 1) {
            return [
                {
                    point: new Vec2(this.contactPoint0X, this.contactPoint0Y),
                    id: this.contactPoint0Id,
                    separation: this.contactPoint0Separation,
                    baseSeparation: this.contactPoint0BaseSeparation,
                    localAnchorA: new Vec2(this.contactPoint0LocalAnchorAX, this.contactPoint0LocalAnchorAY),
                    localAnchorB: new Vec2(this.contactPoint0LocalAnchorBX, this.contactPoint0LocalAnchorBY),
                    persisted: this.contactPoint0Persisted,
                    normalImpulse: this.normalImpulseSum0,
                    tangentImpulse: this.tangentImpulseSum0,
                    totalNormalImpulse: this.contactPoint0TotalNormalImpulse,
                    normalVelocity: this.contactPoint0NormalVelocity,
                    restitutionVelocity: this.contactPoint0RestitutionVelocity,
                },
            ];
        }

        return [
            {
                point: new Vec2(this.contactPoint0X, this.contactPoint0Y),
                id: this.contactPoint0Id,
                separation: this.contactPoint0Separation,
                baseSeparation: this.contactPoint0BaseSeparation,
                localAnchorA: new Vec2(this.contactPoint0LocalAnchorAX, this.contactPoint0LocalAnchorAY),
                localAnchorB: new Vec2(this.contactPoint0LocalAnchorBX, this.contactPoint0LocalAnchorBY),
                persisted: this.contactPoint0Persisted,
                normalImpulse: this.normalImpulseSum0,
                tangentImpulse: this.tangentImpulseSum0,
                totalNormalImpulse: this.contactPoint0TotalNormalImpulse,
                normalVelocity: this.contactPoint0NormalVelocity,
                restitutionVelocity: this.contactPoint0RestitutionVelocity,
            },
            {
                point: new Vec2(this.contactPoint1X, this.contactPoint1Y),
                id: this.contactPoint1Id,
                separation: this.contactPoint1Separation,
                baseSeparation: this.contactPoint1BaseSeparation,
                localAnchorA: new Vec2(this.contactPoint1LocalAnchorAX, this.contactPoint1LocalAnchorAY),
                localAnchorB: new Vec2(this.contactPoint1LocalAnchorBX, this.contactPoint1LocalAnchorBY),
                persisted: this.contactPoint1Persisted,
                normalImpulse: this.normalImpulseSum1,
                tangentImpulse: this.tangentImpulseSum1,
                totalNormalImpulse: this.contactPoint1TotalNormalImpulse,
                normalVelocity: this.contactPoint1NormalVelocity,
                restitutionVelocity: this.contactPoint1RestitutionVelocity,
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
