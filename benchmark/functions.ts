import { BodiesFactory, SETTINGS, World } from '../src';

const CIRCLE_COUNT = 1_000;
const COLUMNS = 40;
const ROWS = CIRCLE_COUNT / COLUMNS;
const RADIUS = 10;
const SPACING = RADIUS * 2.05;

SETTINGS.applyGravity = true;

const world = new World(9.8);

world.addBody(BodiesFactory.box({
    width: COLUMNS * SPACING + RADIUS * 4,
    height: RADIUS * 2,
    x: (COLUMNS - 1) * SPACING * 0.5,
    y: -RADIUS,
    mass: 0,
    friction: 0.8,
}));

for (let row = 0; row < ROWS; row++) {
    for (let column = 0; column < COLUMNS; column++) {
        world.addBody(BodiesFactory.circle({
            radius: RADIUS,
            x: column * SPACING,
            y: RADIUS + row * SPACING,
            mass: 1,
            restitution: 0,
            friction: 0.8,
        }));
    }
}

/** Advances the deterministic 1,000-circle stack by one fixed simulation step. */
export function runOriginal() {
    world.update();
}

// export function runModified() {
//     world.update();
// }
