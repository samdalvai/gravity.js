import { PIXELS_PER_METER } from '../core/Constants';
import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { buildGravityQuadTree } from './GravityQuadTree';

export function generateWeightForce(body: RigidBody, G: number): Vec2 {
    const weightForce = new Vec2(0.0, body.mass * G * PIXELS_PER_METER);
    return weightForce;
}

export function applyWeightForce(body: RigidBody, G: number): void {
    const weigthForce = body.mass * G * PIXELS_PER_METER;
    body.addForceY(weigthForce);
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
export function generateGravitationalForce(
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

/**
 * Convenience version that applies all gravitational forces to all bodies
 */
export function applyGravitationalForces(
    bodies: readonly RigidBody[],
    G: number,
    minDistanceSquared: number,
    maxDistanceSquared: number,
): void {
    for (let i = 0; i < bodies.length - 1; i++) {
        const a = bodies[i];
        for (let j = i + 1; j < bodies.length; j++) {
            const b = bodies[j];
            const attraction = generateGravitationalForce(a, b, G, minDistanceSquared, maxDistanceSquared);
            a.addForce(attraction);
            b.addForce(attraction.negateNew());
        }
    }
}

const DEFAULT_THETA = 0.5;
const DEFAULT_EPSILON = 1;

/**
 * Builds the quadtree and applies one gravitational force per body.
 */
export function applyBarnesHutGravitationalForces(
    bodies: readonly RigidBody[],
    G: number,
    theta = DEFAULT_THETA,
    epsilon = DEFAULT_EPSILON,
): void {
    const tree = buildGravityQuadTree(bodies, theta, epsilon);

    if (tree === null) {
        return;
    }

    const force = new Vec2();
    const thetaSquared = theta * theta;

    for (let i = 0; i < bodies.length; i++) {
        const body = bodies[i];
        if (body.mass === 0) continue;

        tree.forceOn(body, G, force, thetaSquared);
        body.addForce(force);
    }
}
