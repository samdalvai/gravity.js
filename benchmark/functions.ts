import { Bodies, GrabJoint, GrabJointScalar, Vec2 } from '../src';

const a = Bodies.box({
    width: 30,
    height: 30,
    x: 0,
    y: 0,
    mass: 1,
});
const target = new Vec2(100, 100);

const grab = new GrabJoint(a, a.position, target);
const grabScalar = new GrabJointScalar(a, a.position, target);

const SOLVER_ITERATIONS = 20;
const TEST_ITERATIONS = 1000;

const dt = 1 / 60;
const invDt = 1 / dt;

export function runOriginal() {
    for (let i = 0; i < TEST_ITERATIONS; i++) {
        grab.preSolve(invDt);

        for (let j = 0; j < SOLVER_ITERATIONS; j++) {
            grab.solve();
        }
    }
}

export function runModified() {
    for (let i = 0; i < TEST_ITERATIONS; i++) {
        grabScalar.preSolve(invDt);

        for (let j = 0; j < SOLVER_ITERATIONS; j++) {
            grabScalar.solve();
        }
    }
}
