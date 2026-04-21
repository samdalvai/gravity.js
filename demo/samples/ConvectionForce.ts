import { BodiesFactory, SETTINGS, Utils } from '../../src';
import type { World } from '../../src';
import type Application from '../Application';
import Graphics from '../graphics/Graphics';
import { defineDemo, generateFences, generateFloor } from './shared';

function setupConvectionForce(world: World, app: Application): void {
    Graphics.zoom = 0.5;

    generateFloor(world, app);
    generateFences(world, app);
}

const convectionDemo = defineDemo('Convection force', setupConvectionForce);

export default convectionDemo;
