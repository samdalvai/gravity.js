import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';

export class Force {
    static generateDragForce(body: RigidBody, k: number, dt: number): Vec2 {
        const v = body.velocity;

        if (v.magnitudeSquared() === 0) {
            return new Vec2(0, 0);
        }

        const speed = v.magnitude();
        const dragDir = v.scaleNew(-1 / speed); // normalized opposite direction

        // Drag force magnitude: k * v^2
        let dragMagnitude = k * speed * speed;

        // Compute max force that would bring velocity to zero this step
        const maxForce = (body.mass * speed) / dt;

        // Clamp drag so it never reverses velocity
        dragMagnitude = Math.min(dragMagnitude, maxForce);

        return dragDir.scaleNew(dragMagnitude);
    }

    static generateFrictionForce(body: RigidBody, k: number): Vec2 {
        // Calculate the friction direction (inverse of velocity unit vector)
        const frictionDirection = body.velocity.unitVector().scaleNew(-1);

        // Calculate the friction magnitude (just k, for now)
        const frictionMagnitude = k;

        // Calculate the final resulting friction force vector
        return frictionDirection.scaleNew(frictionMagnitude);
    }

    static generateSpringForceBodyAnchor(body: RigidBody, anchor: Vec2, restLength: number, k: number): Vec2 {
        // Calculate the distance between the anchor and the object
        const d = body.position.subNew(anchor);

        // Find the spring displacement considering the rest length
        const displacement = d.magnitude() - restLength;

        // Calculate the direction of the spring force
        const springDirection = d.unitVector();

        // Calculate the magnitude of the spring force
        const sprintMagnitude = -k * displacement;

        // Calculate the final resulting spring force vector
        return springDirection.scaleNew(sprintMagnitude);
    }

    static generateSpringForceBodyBody(a: RigidBody, b: RigidBody, restLength: number, k: number): Vec2 {
        // Calculate the distance between the two bodys
        const d = a.position.subNew(b.position);

        // Find the spring displacement considering the rest length
        const displacement = d.magnitude() - restLength;

        // Calculate the direction of the spring force
        const springDirection = d.unitVector();

        // Calculate the magnitude of the spring force
        const sprintMagnitude = -k * displacement;

        // Calculate the final resulting spring force vector
        return springDirection.scaleNew(sprintMagnitude);
    }

    static generateExplosionForce(body: RigidBody, explosionSource: Vec2, radius: number, strength: number): Vec2 {
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

    static generateCoulombForce(bodyA: RigidBody, bodyB: RigidBody, k: number): Vec2 {
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
}
