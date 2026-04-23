import { BodiesFactory, GRAVITY, SETTINGS, Utils, Vec2 } from '../../src';
import type { World } from '../../src';
import type Application from '../Application';
import Graphics from '../graphics/Graphics';
import { defineDemo } from './shared';

function setupPlanetOrbit(world: World, app: Application): void {
    app.setBackground('transparent');
    Graphics.zoom = 0.1;

    SETTINGS.applyGravity = false;

    const sun = BodiesFactory.circle({
        radius: 50,
        x: 0,
        y: 0,
        mass: 1_000_000,
    });
    app.setBodyFillColor(sun, 'white');
    world.addBody(sun);

    app.setGravitationalForce(true);
}

const planetOrbitDemo = defineDemo('Planets orbit', setupPlanetOrbit);

export default planetOrbitDemo;
