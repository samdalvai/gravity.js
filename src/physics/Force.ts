import Particle from './Particle';
import Vec2 from './Vec2';

export default class Force {
    static generateDragForce = (particle: Particle, k: number): Vec2 => {
        let dragForce = new Vec2(0, 0);

        if (particle.velocity.magnitudeSquared() > 0) {
            // Calculate the drag direction (inverse of velocity unit vector)
            const dragDirection = particle.velocity.unitVector().scaleNew(-1);

            // Calculate the drag magnitude, k * |v|^2
            const dragMagnitude = k * particle.velocity.magnitudeSquared();

            // Generate the final drag force with direction and magnitude
            dragForce = dragDirection.scaleNew(dragMagnitude);
        }

        return dragForce;
    };
    static generateFrictionForce = (particle: Particle, k: number): Vec2 => {
        // Calculate the friction direction (inverse of velocity unit vector)
        const frictionDirection = particle.velocity.unitVector().scaleNew(-1);

        // Calculate the friction magnitude (just k, for now)
        const frictionMagnitude = k;

        // Calculate the final resulting friction force vector
        return frictionDirection.scaleNew(frictionMagnitude);
    };

    static generateGravitationalForce = (
        a: Particle,
        b: Particle,
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

    static generateSpringForceParticleAnchor = (
        particle: Particle,
        anchor: Vec2,
        restLength: number,
        k: number,
    ): Vec2 => {
        // Calculate the distance between the anchor and the object
        const d = particle.position.subNew(anchor);

        // Find the spring displacement considering the rest length
        const displacement = d.magnitude() - restLength;

        // Calculate the direction of the spring force
        const springDirection = d.unitVector();

        // Calculate the magnitude of the spring force
        const sprintMagnitude = -k * displacement;

        // Calculate the final resulting spring force vector
        return springDirection.scaleNew(sprintMagnitude);
    };

    static generateSpringForceParticleParticle = (a: Particle, b: Particle, restLength: number, k: number): Vec2 => {
        // Calculate the distance between the two particles
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
}
