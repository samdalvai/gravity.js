import { describe, expect, test } from '@jest/globals';

import { CircleShape, RigidBody } from '../src';
import * as Collision from '../src/collision/NarrowPhase';
import * as CollisionPerf from '../src/collision/NarrowPhase.perf';

const DT = 1 / 60;
const INV_DT = 1 / DT;
const SOLVER_ITERATIONS = 20;
const TEST_ITERATIONS = 100_000;

describe('Performance test', () => {
    test('Implementation 1', () => {
        console.time('normal');

        const a = new RigidBody(new CircleShape(30), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 0, 0, 1.0);
        const manifold = Collision.detectCollision(a, b)!;
        expect(manifold).not.toBeNull();

        manifold.preSolve(INV_DT);

        // Solve constraints
        for (let x = 0; x < TEST_ITERATIONS; x++) {
            for (let i = 0; i < SOLVER_ITERATIONS; i++) {
                manifold.solve();
            }
        }

        console.timeEnd('normal');
    });

    test('Implementation 2', () => {
        console.time('perf');

        const a = new RigidBody(new CircleShape(30), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 0, 0, 1.0);
        const manifold = CollisionPerf.detectCollision(a, b)!;
        expect(manifold).not.toBeNull();

        manifold.preSolve(INV_DT);

        // Solve constraints
        for (let x = 0; x < TEST_ITERATIONS; x++) {
            for (let i = 0; i < SOLVER_ITERATIONS; i++) {
                manifold.solve();
            }
        }

        console.timeEnd('perf');
    });
});
