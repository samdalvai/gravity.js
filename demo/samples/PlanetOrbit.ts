import { BodiesFactory, GRAVITY, SETTINGS, Utils, Vec2 } from '../../src';
import type { World } from '../../src';
import Graphics from '../graphics/Graphics';
import type Application from '../Application';
import { defineDemo } from './shared';

function setupPlanetOrbit(world: World, app: Application): void {
    app.setBackground('transparent');
    Graphics.zoom = 0.4;

    SETTINGS.applyGravity = false;

    
}

const planetOrbitDemo = defineDemo('Planets orbit', setupPlanetOrbit);

export default planetOrbitDemo;
