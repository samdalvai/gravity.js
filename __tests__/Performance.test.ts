import { describe, expect, test } from '@jest/globals';

import { BoxShape, CircleShape, RigidBody } from '../src';
import * as Collision from '../src/collision/NarrowPhase';
import { DistanceJoint } from '../src/joint/DistanceJoint';
import { DistanceJoint as DistanceJointPerf } from '../src/joint/DistanceJoint.perf';

const DT = 1 / 60;
const INV_DT = 1 / DT;
const SOLVER_ITERATIONS = 20;
const TEST_ITERATIONS = 100_000;

describe('Performance test', () => {
    test('Implementation 1', () => {
        console.time('normal');

        const a = new RigidBody(new CircleShape(30), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 0, 0, 1.0);
        // const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        // const b = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const constraint = new DistanceJoint(a, b, a.position, b.position);
        expect(constraint).not.toBeNull();

        // Solve constraints
        for (let x = 0; x < TEST_ITERATIONS; x++) {
            constraint.preSolve(INV_DT);
            for (let i = 0; i < SOLVER_ITERATIONS; i++) {
                constraint.solve();
            }
        }

        console.timeEnd('normal');
    });

    test('Implementation 2', () => {
        console.time('perf');

        const a = new RigidBody(new CircleShape(30), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 0, 0, 1.0);
        // const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        // const b = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const constraint = new DistanceJointPerf(a, b, a.position, b.position);
        expect(constraint).not.toBeNull();

        // Solve constraints
        for (let x = 0; x < TEST_ITERATIONS; x++) {
            constraint.preSolve(INV_DT);
            for (let i = 0; i < SOLVER_ITERATIONS; i++) {
                constraint.solve();
            }
        }

        console.timeEnd('perf');
    });
});
