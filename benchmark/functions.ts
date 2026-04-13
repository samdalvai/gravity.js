import { CircleShape, RigidBody } from '../src';
import { collideCircles, collideCirclesScalar } from '../src/collision/NarrowPhase';

export const ITERATIONS = 10_000;

const a = new RigidBody(new CircleShape(30), 0, 0, 1.0);
const b = new RigidBody(new CircleShape(30), 30, 0, 1.0);

export function runOriginal() {
    for (let i = 0; i < ITERATIONS; i++) {
        collideCircles(a, b);
    }
}

export function runModified() {
    for (let i = 0; i < ITERATIONS; i++) {
        collideCirclesScalar(a, b);
    }
}
