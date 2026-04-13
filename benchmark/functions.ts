import { BoxShape, CapsuleShape, CircleShape, RigidBody, SegmentShape } from '../src';
import { detectCollision } from '../src/collision/NarrowPhase';
import { detectCollision as detectCollisionScalar } from '../src/collision/NarrowPhase.scalar';

export const ITERATIONS = 20_000;

const a = new RigidBody(new CircleShape(30), 0, 0, 1.0);
const b = new RigidBody(new CircleShape(30), 30, 0, 1.0);

// const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
// const b = new RigidBody(new BoxShape(60, 60), 30, 0, 1.0);

// const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
// const b = new RigidBody(new CircleShape(30), 30, 0, 1.0);

// const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
// const b = new RigidBody(new CapsuleShape(40, 10), 0, 20, 1.0);

// const a = new RigidBody(new CircleShape(30), 0, 0, 1.0);
// const b = new RigidBody(new CapsuleShape(40, 10), 0, 20, 1.0);

// const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
// const b = new RigidBody(new SegmentShape(40, true), 0, 20, 1.0);

// const a = new RigidBody(new CircleShape(30), 0, 0, 1.0);
// const b = new RigidBody(new SegmentShape(40, true), 0, 20, 1.0);

// const a = new RigidBody(new CapsuleShape(40, 10), 0, 0, 1.0);
// const b = new RigidBody(new SegmentShape(40, true), 0, 20, 1.0);

export function runOriginal() {
    for (let i = 0; i < ITERATIONS; i++) {
        detectCollision(a, b);
    }
}

export function runModified() {
    for (let i = 0; i < ITERATIONS; i++) {
        detectCollisionScalar(a, b);
    }
}
