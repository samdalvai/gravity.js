import { PIXELS_PER_METER } from '../core/Constants';
import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';

export class Force {
    static generateWeightForce(body: RigidBody, G: number): Vec2 {
        const weightForce = new Vec2(0.0, body.mass * G * PIXELS_PER_METER);

        return weightForce;
    }

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

    /**
     * Generates an inverse-square gravitational attraction force between two bodies.
     *
     * The force is proportional to 1 / r^2, so we clamp `distanceSquared` before dividing by it.
     *
     * Example:
     * - if `minDistanceSquared = 80 * 80` and the bodies are only 20 px apart, we still use 80 px
     * - if `maxDistanceSquared = 950 * 950` and the bodies are 1400 px apart, we still use 950 px
     *
     * This avoids very large forces near the center and very tiny forces far away.
     */
    static generateGravitationalForce(
        a: RigidBody,
        b: RigidBody,
        G: number,
        minDistanceSquared: number,
        maxDistanceSquared: number,
    ): Vec2 {
        // Calculate the distance between the two objects
        const d = b.position.subNew(a.position);

        let distanceSquared = d.magnitudeSquared();

        // Clamp the squared distance so the resulting force stays within a tunable range.
        distanceSquared = Math.min(Math.max(distanceSquared, minDistanceSquared), maxDistanceSquared);

        // Calculate the direction of the attraction force
        const attractionDirection = d.unitVector();

        // Calculate the strength of the attraction force
        const attractionMagnitude = (G * (a.mass * b.mass)) / distanceSquared;

        // Calculate the final resulting attraction force vector
        return attractionDirection.scaleNew(attractionMagnitude);
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

    static generateConvectionForce(
        body: RigidBody,
        ambientTemperature: number,
        strength: number,
        minTemperatureDifference = 0,
    ): Vec2 {
        if (body.isStatic()) {
            return new Vec2(0, 0);
        }

        const deltaT = body.temperature - ambientTemperature;
        if (deltaT <= minTemperatureDifference) {
            return new Vec2(0, 0);
        }

        return new Vec2(0, strength * (deltaT - minTemperatureDifference));
    }

    // TODO: To be improved by computing submerged area and applyin the force at that centroid 
    // instead of the body default centroid
    static generateBuoyancyForce(body: RigidBody, waterSurfaceY: number, liquidDensity: number, gravity: number): Vec2 {
        const maxX = body.maxX;
        const minX = body.minX;
        const maxY = body.maxY;
        const minY = body.minY;

        const width = maxX - minX;
        const height = maxY - minY;

        if (width <= 0 || height <= 0) {
            return new Vec2(0, 0);
        }

        // Amount of the AABB below the water surface
        const submergedHeight = Math.max(0, Math.min(waterSurfaceY - minY, height));

        if (submergedHeight === 0) {
            return new Vec2(0, 0);
        }

        const submergedFraction = submergedHeight / height;
        const submergedArea = body.shape.getArea() * submergedFraction;
        const buoyancyMagnitude = liquidDensity * submergedArea * gravity;

        return new Vec2(0, buoyancyMagnitude);
    }

    static generateLinearWaterDragForce(
        body: RigidBody,
        waterSurfaceY: number,
        dragCoefficient: number,
        dt: number,
    ): Vec2 {
        const maxY = body.maxY;
        const minY = body.minY;

        const height = maxY - minY;

        if (height <= 0) {
            return new Vec2(0, 0);
        }

        // Same submerged logic as buoyancy
        const submergedHeight = Math.max(0, Math.min(waterSurfaceY - minY, height));

        if (submergedHeight === 0) {
            return new Vec2(0, 0);
        }

        const submergedFraction = submergedHeight / height;

        const v = body.velocity;
        const speedSq = v.magnitudeSquared();

        if (speedSq === 0) {
            return new Vec2(0, 0);
        }

        const speed = Math.sqrt(speedSq);

        // Quadratic drag
        let dragMagnitude = dragCoefficient * speedSq * submergedFraction;

        if (dt > 0) {
            // Do not let water drag reverse the velocity in a single step.
            const maxForce = (body.mass * speed) / dt;
            dragMagnitude = Math.min(dragMagnitude, maxForce);
        }

        // Opposite to velocity
        return v.scaleNew(-dragMagnitude / speed);
    }

    static generateAngularWaterDragTorque(
        body: RigidBody,
        waterSurfaceY: number,
        angularDragCoefficient: number,
    ): number {
        const maxY = body.maxY;
        const minY = body.minY;

        const height = maxY - minY;

        if (height <= 0) {
            return 0;
        }

        const submergedHeight = Math.max(0, Math.min(waterSurfaceY - minY, height));

        if (submergedHeight === 0) {
            return 0;
        }

        const submergedFraction = submergedHeight / height;

        // Scale by moment of inertia so the coefficient behaves like an angular damping rate.
        // Otherwise large bodies barely slow down because angular acceleration is torque / I.
        return -body.angularVelocity * angularDragCoefficient * submergedFraction * body.I;
    }
}
