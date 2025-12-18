import Body from './body/Body';
import Vec2 from '../math/Vec2';

export default class Force {
    static generateDragForce = (body: Body, k: number): Vec2 => {
        let dragForce = new Vec2(0, 0);

        if (body.velocity.magnitudeSquared() > 0) {
            // Calculate the drag direction (inverse of velocity unit vector)
            const dragDirection = body.velocity.unitVector().scaleNew(-1);

            // Calculate the drag magnitude, k * |v|^2
            const dragMagnitude = k * body.velocity.magnitudeSquared();

            // Generate the final drag force with direction and magnitude
            dragForce = dragDirection.scaleNew(dragMagnitude);
        }

        return dragForce;
    };

    static generateFrictionForce = (body: Body, k: number): Vec2 => {
        // Calculate the friction direction (inverse of velocity unit vector)
        const frictionDirection = body.velocity.unitVector().scaleNew(-1);

        // Calculate the friction magnitude (just k, for now)
        const frictionMagnitude = k;

        // Calculate the final resulting friction force vector
        return frictionDirection.scaleNew(frictionMagnitude);
    };

    static generateGravitationalForce = (
        a: Body,
        b: Body,
        G: number,
        minDistance: number,
        maxDistance: number,
    ): Vec2 => {
        // Calculate the distance between the two objects
        const d = b.position.subNew(a.position);

        let distanceSquared = d.magnitudeSquared();

        // Clamp the values of the distance (to allow for some insteresting visual effects)
        distanceSquared = Math.min(Math.max(distanceSquared, minDistance), maxDistance);

        // Calculate the direction of the attraction force
        const attractionDirection = d.unitVector();

        // Calculate the strength of the attraction force
        const attractionMagnitude = (G * (a.mass * b.mass)) / distanceSquared;

        // Calculate the final resulting attraction force vector
        return attractionDirection.scaleNew(attractionMagnitude);
    };

    static generateSpringForceBodyAnchor = (body: Body, anchor: Vec2, restLength: number, k: number): Vec2 => {
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
    };

    static generateSpringForceBodyBody = (a: Body, b: Body, restLength: number, k: number): Vec2 => {
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
    };

    static generateExplosionForce = (body: Body, explosionSource: Vec2, radius: number, strength: number): Vec2 => {
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
    };
}
