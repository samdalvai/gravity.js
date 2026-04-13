import { Vec2 } from '../src';
import { randomNumber } from '../src/utils/Utils';

const NUM_BODIES = 10_000;

class TestBody {
    velocity: Vec2;

    constructor(x: number, y: number) {
        this.velocity = new Vec2(x, y);
    }
}

const bodies: TestBody[] = [];

const velocitiesX = new Float64Array(NUM_BODIES);
const velocitiesY = new Float64Array(NUM_BODIES);

for (let i = 0; i < NUM_BODIES; i++) {
    const x = randomNumber(-100, 100);
    const y = randomNumber(-100, 100);

    bodies.push(new TestBody(x, y));
    velocitiesX[i] = x;
    velocitiesY[i] = y;
}

export function runOriginal() {}

export function runModified() {}
