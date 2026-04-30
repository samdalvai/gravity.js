import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { buildChargeQuadTree } from './ChargeQuadTree';

export function generateExplosionForce(body: RigidBody, explosionSource: Vec2, radius: number, strength: number): Vec2 {
    // Static bodies don't explode
    if (body.invMass === 0) return new Vec2();

    const dir = body.position.subNew(explosionSource);
    const dist = dir.magnitude();

    // If body is farther than radius, explosion force is 0
    if (dist > radius || dist === 0) return new Vec2();

    // Normalize direction
    dir.scaleAssign(1 / dist);

    // Falloff: weaker at distance
    const falloff = 1 - dist / radius;

    const impulseMag = strength * falloff;

    const impulse = dir.scaleNew(impulseMag);
    return impulse;
}

export function generateCoulombForce(bodyA: RigidBody, bodyB: RigidBody, k: number): Vec2 {
    const q1 = bodyA.charge;
    const q2 = bodyB.charge;

    if (q1 === 0 || q2 === 0) {
        return new Vec2(0, 0);
    }

    const dx = bodyB.position.x - bodyA.position.x;
    const dy = bodyB.position.y - bodyA.position.y;

    const distSq = dx * dx + dy * dy;
    if (distSq === 0) {
        return new Vec2(0, 0);
    }

    // Softening to avoid singularity
    const epsilon = 0.01;
    const safeDistSq = distSq + epsilon;

    const invDist = 1 / Math.sqrt(safeDistSq);
    const dirX = dx * invDist;
    const dirY = dy * invDist;

    // Signed scalar handles attraction/repulsion
    const forceScalar = (-k * (q1 * q2)) / safeDistSq;

    return new Vec2(dirX * forceScalar, dirY * forceScalar);
}

/**
 * Convenience version that applies all coulomb forces to all bodies
 */
export function applyCoulombForces(bodies: readonly RigidBody[], k: number): void {
    for (let i = 0; i < bodies.length - 1; i++) {
        const a = bodies[i];
        for (let j = i + 1; j < bodies.length; j++) {
            const b = bodies[j];
            const coulombForce = generateCoulombForce(a, b, k);
            a.addForce(coulombForce);
            b.addForce(coulombForce.negateNew());
        }
    }
}

const DEFAULT_THETA = 0.5;
const DEFAULT_EPSILON = 0.01;

/**
 * Builds the flat charge quadtree and applies one Coulomb force per body.
 */
export function applyBarnesHutCoulombForces(
    bodies: readonly RigidBody[],
    k: number,
    theta = DEFAULT_THETA,
    epsilon = DEFAULT_EPSILON,
): void {
    const tree = buildChargeQuadTree(bodies, theta);
    const force = new Vec2();
    const thetaSquared = theta * theta;

    for (let i = 0; i < bodies.length; i++) {
        const b = bodies[i];
        if (b.charge === 0 || tree === null) continue;
        tree.forceOn(b, k, force, thetaSquared, epsilon);
        b.addForce(force);
    }
}
