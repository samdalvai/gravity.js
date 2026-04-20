import { BodiesFactory, SETTINGS } from '../../src';
import type { World } from '../../src';
import type Application from '../Application';
import { defineDemo } from './shared';

function setupDrag(world: World, app: Application): void {
    app.setBackground('darkBackground');
    SETTINGS.applyGravity = false;

    const xStart = 0;

    const pool1 = BodiesFactory.circle({
        radius: 30,
        x: xStart,
        y: 0,
        mass: 1,
        restitution: 0.9,
    });
    world.addBody(pool1);

    const pool2 = BodiesFactory.circle({
        radius: 30,
        x: xStart + 75,
        y: 35,
        mass: 1,
        restitution: 0.9,
    });
    world.addBody(pool2);

    const pool3 = BodiesFactory.circle({
        radius: 30,
        x: xStart + 75,
        y: -35,
        mass: 1,
        restitution: 0.9,
    });
    world.addBody(pool3);

    const movingPool = BodiesFactory.circle({
        radius: 30,
        x: xStart - 500,
        y: 0,
        mass: 1,
        restitution: 0.9,
    });
    movingPool.velocity.x = 1200;
    world.addBody(movingPool);

    app.setDragForce(0.0025);
}

const dragDemo = defineDemo('Drag demo', setupDrag);

export default dragDemo;
