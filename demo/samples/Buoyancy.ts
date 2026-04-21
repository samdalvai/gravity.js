import { type World } from '../../src';
import type Application from '../Application';
import Graphics from '../graphics/Graphics';
import { FLOOR_HEIGHT, FLOOR_WIDTH, defineDemo, generateFences, generateFloor } from './shared';

function setupBuoyancy(world: World, app: Application): void {
    app.setBackground('darkBackground');
    Graphics.zoom = 0.75;

    const floor = generateFloor(world, app);
    generateFences(world, app);

    app.setLiquid({
        aabb: {
            minX: floor.position.x - FLOOR_WIDTH / 2,
            minY: floor.position.y + FLOOR_HEIGHT / 2,
            maxX: floor.position.x + FLOOR_WIDTH / 2,
            maxY: 300,
        },
        density: 1,
    });
}

const buoyancyDemo = defineDemo('Buoyancy demo', setupBuoyancy);

export default buoyancyDemo;
