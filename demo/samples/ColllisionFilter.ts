import { BodiesFactory, World } from '../../src';
import Application from '../Application';
import { defineDemo } from './shared';

function setupColllisionFilter(world: World, app: Application): void {
    const barsWidth = 1000;
    const barsHeight = 50;
    const barsStartY = -250;
    const barsOffset = 200;

    const bar1 = BodiesFactory.box({
        width: barsWidth,
        height: barsHeight,
        x: 0,
        y: barsStartY,
        mass: 0,
    });

    const bar2 = BodiesFactory.box({
        width: barsWidth,
        height: barsHeight,
        x: 0,
        y: barsStartY + barsOffset,
        mass: 0,
    });

    const bar3 = BodiesFactory.box({
        width: barsWidth,
        height: barsHeight,
        x: 0,
        y: barsStartY + barsOffset * 2,
        mass: 0,
    });

    world.addBody(bar1);
    world.addBody(bar2);
    world.addBody(bar3);
}

const collisionFilteringDemo = defineDemo('Collision filtering', setupColllisionFilter);

export default collisionFilteringDemo;
