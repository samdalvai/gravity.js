import { type World } from '../../src';
import type Application from '../Application';
import Graphics from '../graphics/Graphics';
import { defineDemo, generateFences, generateFloor } from './shared';

function setupBuoyancy(world: World, app: Application): void {
    app.setBackground('darkBackground');
    Graphics.zoom = 0.75;

    const floor = generateFloor(world, app);
    const fences = generateFences(world, app);
}

const buoyancyDemo = defineDemo('Buoyancy demo', setupBuoyancy);

export default buoyancyDemo;
