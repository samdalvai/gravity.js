import { SETTINGS } from '../core/Constants';
import * as Utils from '../utils/Utils';
import type { ContactManifold } from './ContactManifold';

/** Block solver is used only when the 2x2 effective mass matrix is well conditioned */
const MAX_BLOCK_CONDITION_NUMBER = 1000;

// Layout of one contact constraint in the packed array
const OFFSET_A = 0;
const OFFSET_B = 1;
const INV_MASS_A = 2;
const INV_I_A = 3;
const INV_MASS_B = 4;
const INV_I_B = 5;
const NORMAL_X = 6;
const NORMAL_Y = 7;
const FRICTION = 8;
const TWO_POINTS = 9;
const USE_BLOCK = 10;
const TANGENT_BIAS = 11;
// Two contact points, POINT_STRIDE values each
const POINT_0 = 12;
const POINT_1 = 21;
const POINT_STRIDE = 9;
// Per point values, relative to the point start
const NORMAL_JWA = 0;
const NORMAL_JWB = 1;
const TANGENT_JWA = 2;
const TANGENT_JWB = 3;
const NORMAL_BIAS = 4;
const NORMAL_MASS = 5;
const TANGENT_MASS = 6;
const NORMAL_IMPULSE = 7;
const TANGENT_IMPULSE = 8;
// Block solver matrices
const K00 = POINT_1 + POINT_STRIDE;
const K01 = K00 + 1;
const K11 = K00 + 2;
const M00 = K00 + 3;
const M01 = K00 + 4;
const M11 = K00 + 5;

export const CONTACT_STRIDE = M11 + 1;

/**
 * Solves all the contacts of a step on packed data.
 *
 * Contact manifolds are large objects spread around the heap, iterating over them in every solver
 * iteration is dominated by cache misses. `prepare` computes the data the velocity solver needs
 * (Jacobians, effective masses, biases, accumulated impulses) into one contiguous Float64Array laid out
 * in solve order, `solve` then runs over that array, and `storeImpulses` copies the accumulated impulses
 * back into the manifolds for warm starting the next step.
 *
 * Body velocities are read from and written to the packed solver body array `v`, see SolverBodies.
 */
export class ContactSolver {
    private constraints = new Float64Array(CONTACT_STRIDE * 256);
    private count = 0;

    /** Computes the constraint data of every manifold and applies warm starting impulses to `v` */
    prepare(manifolds: readonly ContactManifold[], invDt: number, v: Float64Array): void {
        const count = manifolds.length;

        if (count * CONTACT_STRIDE > this.constraints.length) {
            let capacity = this.constraints.length;
            while (capacity < count * CONTACT_STRIDE) capacity *= 2;
            this.constraints = new Float64Array(capacity);
        }

        this.count = count;

        const c = this.constraints;
        const positionCorrection = SETTINGS.positionCorrection;
        const warmStarting = SETTINGS.warmStarting;
        const blockSolve = SETTINGS.blockSolve;
        const penetrationSlop = SETTINGS.penetrationSlop;
        const restitutionSlop = SETTINGS.restitutionSlop;

        for (let i = 0; i < count; i++) {
            const manifold = manifolds[i];
            const bodyA = manifold.bodyA;
            const bodyB = manifold.bodyB;
            const base = i * CONTACT_STRIDE;

            const offsetA = bodyA.solverOffset;
            const offsetB = bodyB.solverOffset;
            const invMassA = bodyA.invMass;
            const invIA = bodyA.invI;
            const invMassB = bodyB.invMass;
            const invIB = bodyB.invI;
            const normalX = manifold.contactNormalX;
            const normalY = manifold.contactNormalY;
            const tangentX = manifold.contactTangentX;
            const tangentY = manifold.contactTangentY;
            const twoPoints = manifold.numContacts === 2;
            const persistent = manifold.persistent;

            const tangentBias = manifold.featureFlipped
                ? bodyB.surfaceSpeed - bodyA.surfaceSpeed
                : bodyA.surfaceSpeed - bodyB.surfaceSpeed;

            c[base + OFFSET_A] = offsetA;
            c[base + OFFSET_B] = offsetB;
            c[base + INV_MASS_A] = invMassA;
            c[base + INV_I_A] = invIA;
            c[base + INV_MASS_B] = invMassB;
            c[base + INV_I_B] = invIB;
            c[base + NORMAL_X] = normalX;
            c[base + NORMAL_Y] = normalY;
            c[base + FRICTION] = manifold.friction;
            c[base + TWO_POINTS] = twoPoints ? 1 : 0;
            c[base + TANGENT_BIAS] = tangentBias;

            for (let point = 0; point < (twoPoints ? 2 : 1); point++) {
                const p = base + (point === 0 ? POINT_0 : POINT_1);
                const contactPointX = point === 0 ? manifold.contactPoint0X : manifold.contactPoint1X;
                const contactPointY = point === 0 ? manifold.contactPoint0Y : manifold.contactPoint1Y;

                const raX = contactPointX - bodyA.position.x;
                const raY = contactPointY - bodyA.position.y;
                const rbX = contactPointX - bodyB.position.x;
                const rbY = contactPointY - bodyB.position.y;

                const normalJwa = raY * normalX - raX * normalY;
                const normalJwb = rbX * normalY - rbY * normalX;
                const tangentJwa = raY * tangentX - raX * tangentY;
                const tangentJwb = rbX * tangentY - rbY * tangentX;

                let normalBias = 0.0;
                if (positionCorrection || !persistent) {
                    const bodyAAngularVelocity = v[offsetA + 2];
                    const bodyBAngularVelocity = v[offsetB + 2];
                    const relativeVelocityX =
                        v[offsetB] - bodyBAngularVelocity * rbY - (v[offsetA] - bodyAAngularVelocity * raY);
                    const relativeVelocityY =
                        v[offsetB + 1] + bodyBAngularVelocity * rbX - (v[offsetA + 1] + bodyAAngularVelocity * raX);
                    const normalVelocity = normalX * relativeVelocityX + normalY * relativeVelocityY;

                    if (positionCorrection && invDt > 0.0) {
                        normalBias =
                            -(manifold.beta * invDt) * Math.max(manifold.penetrationDepth - penetrationSlop, 0.0);
                    }

                    if (!persistent && normalVelocity + restitutionSlop < 0.0) {
                        normalBias += manifold.restitution * normalVelocity;
                    }
                }

                const normalK = invMassA + normalJwa * invIA * normalJwa + invMassB + normalJwb * invIB * normalJwb;
                const tangentK =
                    invMassA + tangentJwa * invIA * tangentJwa + invMassB + tangentJwb * invIB * tangentJwb;

                const normalImpulse = point === 0 ? manifold.normalImpulseSum0 : manifold.normalImpulseSum1;
                const tangentImpulse = point === 0 ? manifold.tangentImpulseSum0 : manifold.tangentImpulseSum1;

                c[p + NORMAL_JWA] = normalJwa;
                c[p + NORMAL_JWB] = normalJwb;
                c[p + TANGENT_JWA] = tangentJwa;
                c[p + TANGENT_JWB] = tangentJwb;
                c[p + NORMAL_BIAS] = normalBias;
                c[p + NORMAL_MASS] = normalK > 0.0 ? 1.0 / normalK : 0.0;
                c[p + TANGENT_MASS] = tangentK > 0.0 ? 1.0 / tangentK : 0.0;
                c[p + NORMAL_IMPULSE] = normalImpulse;
                c[p + TANGENT_IMPULSE] = tangentImpulse;

                if (warmStarting) {
                    if (normalImpulse !== 0.0) {
                        const impulseA = invMassA * normalImpulse;
                        v[offsetA] += -normalX * impulseA;
                        v[offsetA + 1] += -normalY * impulseA;
                        v[offsetA + 2] += invIA * normalJwa * normalImpulse;

                        const impulseB = invMassB * normalImpulse;
                        v[offsetB] += normalX * impulseB;
                        v[offsetB + 1] += normalY * impulseB;
                        v[offsetB + 2] += invIB * normalJwb * normalImpulse;
                    }

                    if (tangentImpulse !== 0.0) {
                        const impulseA = invMassA * tangentImpulse;
                        v[offsetA] += -tangentX * impulseA;
                        v[offsetA + 1] += -tangentY * impulseA;
                        v[offsetA + 2] += invIA * tangentJwa * tangentImpulse;

                        const impulseB = invMassB * tangentImpulse;
                        v[offsetB] += tangentX * impulseB;
                        v[offsetB + 1] += tangentY * impulseB;
                        v[offsetB + 2] += invIB * tangentJwb * tangentImpulse;
                    }
                }
            }

            c[base + USE_BLOCK] = twoPoints && blockSolve && prepareBlock(c, base) ? 1 : 0;
        }
    }

    /** One velocity iteration over all the contacts */
    solve(v: Float64Array): void {
        const c = this.constraints;
        const count = this.count;
        const impulseAccumulation = SETTINGS.impulseAccumulation;

        for (let i = 0; i < count; i++) {
            const base = i * CONTACT_STRIDE;
            const offsetA = c[base + OFFSET_A];
            const offsetB = c[base + OFFSET_B];
            const invMassA = c[base + INV_MASS_A];
            const invIA = c[base + INV_I_A];
            const invMassB = c[base + INV_MASS_B];
            const invIB = c[base + INV_I_B];
            const normalX = c[base + NORMAL_X];
            const normalY = c[base + NORMAL_Y];
            const tangentX = -normalY;
            const tangentY = normalX;
            const friction = c[base + FRICTION];
            const tangentBias = c[base + TANGENT_BIAS];
            const twoPoints = c[base + TWO_POINTS] !== 0;
            const p0 = base + POINT_0;
            const p1 = base + POINT_1;

            let vAX = v[offsetA];
            let vAY = v[offsetA + 1];
            let wA = v[offsetA + 2];
            let vBX = v[offsetB];
            let vBY = v[offsetB + 1];
            let wB = v[offsetB + 2];

            // Tangent (friction) constraints
            for (let point = 0; point < (twoPoints ? 2 : 1); point++) {
                const p = point === 0 ? p0 : p1;
                const tangentJwa = c[p + TANGENT_JWA];
                const tangentJwb = c[p + TANGENT_JWB];
                const jv =
                    -tangentX * vAX +
                    -tangentY * vAY +
                    tangentJwa * wA +
                    tangentX * vBX +
                    tangentY * vBY +
                    tangentJwb * wB;
                const maxFriction = friction * c[p + NORMAL_IMPULSE];
                const oldImpulseSum = c[p + TANGENT_IMPULSE];

                let lambda = c[p + TANGENT_MASS] * -(jv + tangentBias);
                let impulseSum: number;

                if (impulseAccumulation) {
                    impulseSum = Utils.clamp(oldImpulseSum + lambda, -maxFriction, maxFriction);
                    lambda = impulseSum - oldImpulseSum;
                } else {
                    impulseSum = Utils.clamp(lambda, -maxFriction, maxFriction);
                    lambda = impulseSum;
                }

                c[p + TANGENT_IMPULSE] = impulseSum;

                const impulseA = invMassA * lambda;
                vAX += -tangentX * impulseA;
                vAY += -tangentY * impulseA;
                wA += invIA * tangentJwa * lambda;

                const impulseB = invMassB * lambda;
                vBX += tangentX * impulseB;
                vBY += tangentY * impulseB;
                wB += invIB * tangentJwb * lambda;
            }

            if (c[base + USE_BLOCK] !== 0) {
                // Normal constraints of both points solved together, see Box2D b2SolveContact
                const normalJwa0 = c[p0 + NORMAL_JWA];
                const normalJwb0 = c[p0 + NORMAL_JWB];
                const normalJwa1 = c[p1 + NORMAL_JWA];
                const normalJwb1 = c[p1 + NORMAL_JWB];
                const k00 = c[base + K00];
                const k01 = c[base + K01];
                const k11 = c[base + K11];
                const aX = c[p0 + NORMAL_IMPULSE];
                const aY = c[p1 + NORMAL_IMPULSE];

                let vn1 =
                    -normalX * vAX + -normalY * vAY + normalJwa0 * wA + normalX * vBX + normalY * vBY + normalJwb0 * wB;
                let vn2 =
                    -normalX * vAX + -normalY * vAY + normalJwa1 * wA + normalX * vBX + normalY * vBY + normalJwb1 * wB;

                let bX = vn1 + c[p0 + NORMAL_BIAS];
                let bY = vn2 + c[p1 + NORMAL_BIAS];

                bX -= k00 * aX + k01 * aY;
                bY -= k01 * aX + k11 * aY;

                let xX = -(c[base + M00] * bX + c[base + M01] * bY);
                let xY = -(c[base + M01] * bX + c[base + M11] * bY);
                let solved = xX >= 0.0 && xY >= 0.0;

                if (!solved) {
                    xX = c[p0 + NORMAL_MASS] * -bX;
                    xY = 0.0;
                    vn1 = 0.0;
                    vn2 = k01 * xX + bY;
                    solved = xX >= 0.0 && vn2 >= 0.0;
                }

                if (!solved) {
                    xX = 0.0;
                    xY = c[p1 + NORMAL_MASS] * -bY;
                    vn1 = k01 * xY + bX;
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

                Utils.assert(solved, 'Error solving contact block');

                const lambdaX = xX - aX;
                const lambdaY = xY - aY;
                const linearImpulse = lambdaX + lambdaY;

                const impulseA = invMassA * linearImpulse;
                vAX += -normalX * impulseA;
                vAY += -normalY * impulseA;
                wA += invIA * (normalJwa0 * lambdaX + normalJwa1 * lambdaY);

                const impulseB = invMassB * linearImpulse;
                vBX += normalX * impulseB;
                vBY += normalY * impulseB;
                wB += invIB * (normalJwb0 * lambdaX + normalJwb1 * lambdaY);

                c[p0 + NORMAL_IMPULSE] = xX;
                c[p1 + NORMAL_IMPULSE] = xY;
            } else {
                for (let point = 0; point < (twoPoints ? 2 : 1); point++) {
                    const p = point === 0 ? p0 : p1;
                    const normalJwa = c[p + NORMAL_JWA];
                    const normalJwb = c[p + NORMAL_JWB];
                    const jv =
                        -normalX * vAX +
                        -normalY * vAY +
                        normalJwa * wA +
                        normalX * vBX +
                        normalY * vBY +
                        normalJwb * wB;
                    const oldImpulseSum = c[p + NORMAL_IMPULSE];

                    let lambda = c[p + NORMAL_MASS] * -(jv + c[p + NORMAL_BIAS]);
                    let impulseSum: number;

                    if (impulseAccumulation) {
                        impulseSum = Math.max(0.0, oldImpulseSum + lambda);
                        lambda = impulseSum - oldImpulseSum;
                    } else {
                        impulseSum = Math.max(0.0, lambda);
                        lambda = impulseSum;
                    }

                    c[p + NORMAL_IMPULSE] = impulseSum;

                    const impulseA = invMassA * lambda;
                    vAX += -normalX * impulseA;
                    vAY += -normalY * impulseA;
                    wA += invIA * normalJwa * lambda;

                    const impulseB = invMassB * lambda;
                    vBX += normalX * impulseB;
                    vBY += normalY * impulseB;
                    wB += invIB * normalJwb * lambda;
                }
            }

            // Impulses on static bodies are always 0, so storing them back unconditionally is safe
            v[offsetA] = vAX;
            v[offsetA + 1] = vAY;
            v[offsetA + 2] = wA;
            v[offsetB] = vBX;
            v[offsetB + 1] = vBY;
            v[offsetB + 2] = wB;
        }
    }

    /** Copies the accumulated impulses back into the manifolds, used for warm starting and contact callbacks */
    storeImpulses(manifolds: readonly ContactManifold[]): void {
        const c = this.constraints;

        for (let i = 0; i < this.count; i++) {
            const manifold = manifolds[i];
            const base = i * CONTACT_STRIDE;

            manifold.normalImpulseSum0 = c[base + POINT_0 + NORMAL_IMPULSE];
            manifold.tangentImpulseSum0 = c[base + POINT_0 + TANGENT_IMPULSE];

            if (c[base + TWO_POINTS] !== 0) {
                manifold.normalImpulseSum1 = c[base + POINT_1 + NORMAL_IMPULSE];
                manifold.tangentImpulseSum1 = c[base + POINT_1 + TANGENT_IMPULSE];
            }
        }
    }
}

/** Computes the block solver matrices, returns false when the matrix is ill conditioned */
function prepareBlock(c: Float64Array, base: number): boolean {
    const invMassA = c[base + INV_MASS_A];
    const invIA = c[base + INV_I_A];
    const invMassB = c[base + INV_MASS_B];
    const invIB = c[base + INV_I_B];
    const normalJwa0 = c[base + POINT_0 + NORMAL_JWA];
    const normalJwb0 = c[base + POINT_0 + NORMAL_JWB];
    const normalJwa1 = c[base + POINT_1 + NORMAL_JWA];
    const normalJwb1 = c[base + POINT_1 + NORMAL_JWB];

    const k00 = invMassA + normalJwa0 * invIA * normalJwa0 + invMassB + normalJwb0 * invIB * normalJwb0;
    const k11 = invMassA + normalJwa1 * invIA * normalJwa1 + invMassB + normalJwb1 * invIB * normalJwb1;
    const k01 = invMassA + normalJwa0 * invIA * normalJwa1 + invMassB + normalJwb0 * invIB * normalJwb1;

    c[base + K00] = k00;
    c[base + K01] = k01;
    c[base + K11] = k11;

    const determinant = k00 * k11 - k01 * k01;

    // Nearly coincident contact points make the matrix singular
    if (k00 * k00 >= MAX_BLOCK_CONDITION_NUMBER * determinant) {
        return false;
    }

    const invDeterminant = 1.0 / determinant;
    c[base + M00] = invDeterminant * k11;
    c[base + M01] = -invDeterminant * k01;
    c[base + M11] = invDeterminant * k00;

    return true;
}
