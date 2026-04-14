import { Bodies, Vec2 } from '../src';
import { detectCollision } from '../src/collision/NarrowPhase';
import { detectCollision as detectCollisionV2 } from '../src/collision/NarrowPhaseV2';
import { randomNumber } from '../src/utils/Utils';

// const a = Bodies.circle({
//     radius: 60,
//     x: 0,
//     y: 0,
//     mass: 1,
// });

// const b = Bodies.circle({
//     radius: 60,
//     x: 30,
//     y: 0,
//     mass: 1,
// });

const a = Bodies.box({
    width: 60,
    height: 60,
    x: 0,
    y: 0,
    mass: 1,
});

const b = Bodies.box({
    width: 60,
    height: 60,
    x: 30,
    y: 30,
    mass: 1,
});

const ITERATIONS = 1000;
const SOLVER_ITERATIONS = 20;
const DT = 1 / 60;
const INV_DT = 1 / DT;

export function runOriginal() {
    for (let i = 0; i < ITERATIONS; i++) {
        const manifold = detectCollision(a, b)!;

        manifold.preSolve(INV_DT);

        for (let j = 0; j < SOLVER_ITERATIONS; j++) {
            manifold.solve();
        }
    }
}

export function runModified() {
    for (let i = 0; i < ITERATIONS; i++) {
        const manifold = detectCollisionV2(a, b)!;

        manifold.preSolve(INV_DT);

        for (let j = 0; j < SOLVER_ITERATIONS; j++) {
            manifold.solve();
        }
    }
}
