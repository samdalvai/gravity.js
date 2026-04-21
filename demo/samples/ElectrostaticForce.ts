import { BodiesFactory, SETTINGS, Utils } from '../../src';
import type { World } from '../../src';
import type Application from '../Application';
import Graphics from '../graphics/Graphics';
import { defineDemo } from './shared';

function setupElectroStaticForce(world: World, app: Application): void {
    SETTINGS.applyGravity = false;
    Graphics.zoom = 0.5;

    const numOfParticles = 1_000;

    for (let i = 0; i < numOfParticles; i++) {
        const charge = Utils.randomNumber(-100, 100);
        const radius = Math.max(Math.abs(charge) / 5, 5);
        const particle = BodiesFactory.circle({
            radius: radius,
            x: Utils.randomNumber(-1500, 1500),
            y: Utils.randomNumber(-1000, 1000),
            mass: Utils.randomNumber(0.1, 1),
            charge: charge,
        });

        if (particle.charge > 0) {
            app.setBodyFillColor(particle, 'red');
        } else if (particle.charge < 0) {
            app.setBodyFillColor(particle, 'blue');
        } else {
            app.setBodyFillColor(particle, 'white');
        }
        world.addBody(particle);
    }

    app.setCoulombForce(true);
}

const electroStaticDemo = defineDemo('Electrostatic force', setupElectroStaticForce);

export default electroStaticDemo;
