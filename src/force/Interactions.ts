import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';

export function generateExplosionForce(
    body: RigidBody,
    explosionSource: Vec2,
    radius: number,
    strength: number,
): Vec2 {
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
