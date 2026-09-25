import { BodiesFactory, SETTINGS, World } from '../src';
import { BodiesFactory as BodiesFactoryOld, SETTINGS as SETTINGS_OLD, World as WorldOld } from '../src_old';

const CIRCLE_COUNT = 1_000;
const COLUMNS = 40;
const ROWS = CIRCLE_COUNT / COLUMNS;
const RADIUS = 10;
const SPACING = RADIUS * 2.05;

SETTINGS.applyGravity = true;
SETTINGS_OLD.applyGravity = true;

const world = new World(9.8);
const world_old = new WorldOld(9.8);

world.addBody(BodiesFactory.box({
    width: COLUMNS * SPACING + RADIUS * 4,
    height: RADIUS * 2,
    x: (COLUMNS - 1) * SPACING * 0.5,
    y: -RADIUS,
    mass: 0,
    friction: 0.8,
}));

world_old.addBody(BodiesFactoryOld.box({
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

        world_old.addBody(BodiesFactoryOld.circle({
            radius: RADIUS,
            x: column * SPACING,
            y: RADIUS + row * SPACING,
            mass: 1,
            restitution: 0,
            friction: 0.8,
        }));
    }
}

export function runOriginal() {
    // Put the old implementation here
    world_old.update();
}

export function runModified() {
    // Put the new implementation here
    world.update();
}
