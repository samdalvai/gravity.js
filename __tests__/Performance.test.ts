import { describe, expect, test } from '@jest/globals';

import { CircleShape, RigidBody } from '../src';
// import * as Collision from '../src/collision/NarrowPhase';
import { DistanceJoint } from '../src/joint/DistanceJoint';
import { DistanceJoint as DistanceJointPerf } from '../src/joint/DistanceJoint';

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

// describe('Performance test for packed arrays', () => {
//     test('Implementation 1', () => {
//         class Entity {
//             velocity: Vec2;

//             constructor(vel: Vec2) {
//                 this.velocity = vel.copy();
//             }
//         }

//         const entities: Entity[] = [];
//         for (let i = 0; i < TEST_ITERATIONS; i++) {
//             entities.push(new Entity(new Vec2(Utils.randomNumber(0, 10), Utils.randomNumber(0, 1))));
//         }

//         console.time('normal');

//         for (let i = 0; i < TEST_ITERATIONS; i++) {
//             const vel = entities[i].velocity;
//             vel.x += 1;
//             vel.y += 1;
//         }

//         console.timeEnd('normal');
//     });

//     test('Implementation 2', () => {
//         // const velocitiesX = new Float64Array(TEST_ITERATIONS);
//         // const velocitiesY = new Float64Array(TEST_ITERATIONS);

//         // Note: Int32Array is faster than Float64Array because there is no conversion to float
//         const velocitiesX = new Int32Array(TEST_ITERATIONS);
//         const velocitiesY = new Int32Array(TEST_ITERATIONS);

//         console.time('perf');

//         for (let i = 0; i < TEST_ITERATIONS; i++) {
//             velocitiesX[i] += 1;
//             velocitiesY[i] += 1;
//         }

//         console.timeEnd('perf');
//     });
// });
