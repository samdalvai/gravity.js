import { Bodies } from '../src';
import { WeldJoint } from '../src/joint/WeldJoint';
import { WeldJointScalar } from '../src/joint/WeldJointScalar';

const bodyA = Bodies.box({
    width: 30,
    height: 30,
    x: 0,
    y: 0,
    mass: 1,
});

const bodyB = Bodies.box({
    width: 30,
    height: 30,
    x: 30,
    y: 0,
    mass: 1,
});

const weldCurrent = new WeldJoint(bodyA, bodyB);
const weldScalar = new WeldJointScalar(bodyA, bodyB);

const ITERATIONS = 100;
const SOLVER_ITERATIONS = 20;
const DT = 1 / 60;
const INV_DT = 1 / DT;

export function runOriginal() {
    for (let i = 0; i < ITERATIONS; i++) {
        weldCurrent.preSolve(INV_DT);
        for (let j = 0; j < SOLVER_ITERATIONS; j++) {
            weldCurrent.solve();
        }
    }
}

export function runModified() {
    for (let i = 0; i < ITERATIONS; i++) {
        weldScalar.preSolve(INV_DT);
        for (let j = 0; j < SOLVER_ITERATIONS; j++) {
            weldScalar.solve();
        }
    }
}
