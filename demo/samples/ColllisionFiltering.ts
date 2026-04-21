import { BodiesFactory, World } from '../../src';
import { CollisionCategory } from '../../src/collision/CollisionFilter';
import Application from '../Application';
import { defineDemo } from './shared';

function setupColllisionFiltering(world: World, app: Application): void {
    const layer1Color = '#ff5a5f';
    const layer2Color = '#56c271';
    const layer3Color = '#5aa9ff';
    const ballAllLayersColor = '#f4f7ff';
    const ballLayer1And2Color = '#ffd966';
    const ballLayer1Color = layer1Color;

    const barsWidth = 1000;
    const barsHeight = 25;
    const barsStartY = -250;
    const barsOffset = 200;

    const bar1 = BodiesFactory.box({
        width: barsWidth,
        height: barsHeight,
        x: 0,
        y: barsStartY,
        mass: 0,
        collisionFilter: {
            category: CollisionCategory.LAYER1,
            mask: CollisionCategory.ALL,
        },
    });
    app.setBodyFillColor(bar1, layer1Color);

    const bar2 = BodiesFactory.box({
        width: barsWidth,
        height: barsHeight,
        x: 0,
        y: barsStartY + barsOffset,
        mass: 0,
        collisionFilter: {
            category: CollisionCategory.LAYER2,
            mask: CollisionCategory.ALL,
        },
    });
    app.setBodyFillColor(bar2, layer2Color);

    const bar3 = BodiesFactory.box({
        width: barsWidth,
        height: barsHeight,
        x: 0,
        y: barsStartY + barsOffset * 2,
        mass: 0,
        collisionFilter: {
            category: CollisionCategory.LAYER3,
            mask: CollisionCategory.ALL,
        },
    });
    app.setBodyFillColor(bar3, layer3Color);

    world.addBody(bar1);
    world.addBody(bar2);
    world.addBody(bar3);

    const ball1 = BodiesFactory.circle({
        radius: 30,
        x: -250,
        y: 1000,
        mass: 1,
        collisionFilter: {
            category: CollisionCategory.DEFAULT,
            mask: CollisionCategory.LAYER1 | CollisionCategory.LAYER2 | CollisionCategory.LAYER3,
        },
    });
    app.setBodyFillColor(ball1, ballAllLayersColor);

    const ball2 = BodiesFactory.circle({
        radius: 30,
        x: 0,
        y: 1000,
        mass: 1,
        collisionFilter: {
            category: CollisionCategory.DEFAULT,
            mask: CollisionCategory.LAYER1 | CollisionCategory.LAYER2,
        },
    });
    app.setBodyFillColor(ball2, ballLayer1And2Color);

    const ball3 = BodiesFactory.circle({
        radius: 30,
        x: 250,
        y: 1000,
        mass: 1,
        collisionFilter: {
            category: CollisionCategory.DEFAULT,
            mask: CollisionCategory.LAYER1,
        },
    });
    app.setBodyFillColor(ball3, ballLayer1Color);

    world.addBody(ball1);
    world.addBody(ball2);
    world.addBody(ball3);
}

const collisionFilteringDemo = defineDemo('Collision filtering', setupColllisionFiltering);

export default collisionFilteringDemo;
