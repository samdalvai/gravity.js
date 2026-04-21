import { BodiesFactory, Utils, type World } from '../../src';
import type Application from '../Application';
import Graphics from '../graphics/Graphics';
import { FLOOR_HEIGHT, defineDemo, generateFences, generateFloor } from './shared';

function setupConvectionForce(world: World, app: Application): void {
    Graphics.zoom = 0.5;

    const MIN_TEMPERATURE = 0;
    const MAX_TEMPERATURE = 2500;

    const floor = generateFloor(world, app);
    floor.temperature = MAX_TEMPERATURE;

    app.removeBodyTexture(floor);
    app.setBodyFillColor(floor, temperatureToColor(floor.temperature, MIN_TEMPERATURE, MAX_TEMPERATURE));

    const fences = generateFences(world, app);
    fences[0].temperature = MAX_TEMPERATURE;
    fences[1].temperature = MAX_TEMPERATURE;

    app.removeBodyTexture(fences[0]);
    app.removeBodyTexture(fences[1]);
    app.setBodyFillColor(fences[0], temperatureToColor(floor.temperature, MIN_TEMPERATURE, MAX_TEMPERATURE));
    app.setBodyFillColor(fences[1], temperatureToColor(floor.temperature, MIN_TEMPERATURE, MAX_TEMPERATURE));

    const numOfParticles = 2_500;

    for (let i = 0; i < numOfParticles; i++) {
        const radius = 5;
        const particle = BodiesFactory.circle({
            radius: radius,
            x: Utils.randomNumber(-1500, 1500),
            y: Utils.randomNumber(floor.position.y + FLOOR_HEIGHT / 2 + radius, 100),
            mass: 0.1,
            temperature: MIN_TEMPERATURE,
        });

        app.setBodyFillColor(particle, temperatureToColor(particle.temperature, MIN_TEMPERATURE, MAX_TEMPERATURE));

        world.addBody(particle);
    }

    app.setConvectionForce(true);
}

const convectionDemo = defineDemo('Convection force', setupConvectionForce);

export default convectionDemo;

export function temperatureToColor(temperature: number, minTemp: number, maxTemp: number): string {
    const t = Math.max(0, Math.min(1, (temperature - minTemp) / (maxTemp - minTemp)));

    let r = 0;
    let g = 0;
    let b = 0;

    if (t < 0.33) {
        // black -> red
        const k = t / 0.33;
        r = Math.round(255 * k);
    } else if (t < 0.66) {
        // red -> yellow
        const k = (t - 0.33) / 0.33;
        r = 255;
        g = Math.round(180 * k);
    } else {
        // yellow -> white
        const k = (t - 0.66) / 0.34;
        r = 255;
        g = 180 + Math.round(75 * k);
        b = Math.round(220 * k);
    }

    return `rgb(${r}, ${g}, ${b})`;
}
