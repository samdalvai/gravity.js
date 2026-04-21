import { BodiesFactory, SETTINGS, Utils } from '../../src';
import type { World } from '../../src';
import type Application from '../Application';
import Graphics from '../graphics/Graphics';
import { defineDemo } from './shared';

function setupElectroStaticForce(world: World, app: Application): void {
    app.setBackground('darkBackground');
    SETTINGS.applyGravity = false;
    Graphics.zoom = 0.75;

    const numOfParticles = 1_000;

    for (let i = 0; i < numOfParticles; i++) {
        const particle = BodiesFactory.circle({
            radius: Utils.randomNumber(5, 15),
            x: Utils.randomNumber(-1500, 1500),
            y: Utils.randomNumber(-1000, 1000),
            mass: Utils.randomNumber(0.1, 1),
        });
        world.addBody(particle);
    }
}

const electroStaticDemo = defineDemo('Electrostatic force', setupElectroStaticForce);

export default electroStaticDemo;
