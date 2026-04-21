import { BodiesFactory, RigidBody, SETTINGS, Utils, type World } from '../../src';
import type Application from '../Application';
import Graphics from '../graphics/Graphics';
import { FENCE_WIDTH, FLOOR_HEIGHT, defineDemo, generateFences, generateFloor } from './shared';

function setupConvectionForce(world: World, app: Application): void {
    Graphics.zoom = 0.5;

    const MIN_TEMPERATURE = 0;
    const MAX_TEMPERATURE = 5_000;
    const FLOOR_WIDTH = 1500;

    const floor = generateFloor(world, app, FLOOR_WIDTH);
    floor.temperature = MAX_TEMPERATURE;

    app.removeBodyTexture(floor);
    app.setBodyFillColor(floor, temperatureToColor(floor.temperature, MIN_TEMPERATURE, MAX_TEMPERATURE));

    const fences = generateFences(world, app, FLOOR_WIDTH);
    fences[0].temperature = MAX_TEMPERATURE;
    fences[1].temperature = MAX_TEMPERATURE;

    app.removeBodyTexture(fences[0]);
    app.removeBodyTexture(fences[1]);
    app.setBodyFillColor(fences[0], temperatureToColor(floor.temperature, MIN_TEMPERATURE, MAX_TEMPERATURE));
    app.setBodyFillColor(fences[1], temperatureToColor(floor.temperature, MIN_TEMPERATURE, MAX_TEMPERATURE));

    const numOfParticles = 2_500;
    const particleRadius = 5;
    const BASE_Y = floor.position.y + FLOOR_HEIGHT / 2 + particleRadius;
    const MAX_X = fences[0].position.x + FENCE_WIDTH / 2 + particleRadius;

    for (let i = 0; i < numOfParticles; i++) {
        const particle = BodiesFactory.circle({
            radius: particleRadius,
            x: Utils.randomNumber(-MAX_X, MAX_X),
            y: Utils.randomNumber(BASE_Y, 250),
            mass: 0.1,
            temperature: MIN_TEMPERATURE,
        });

        app.setBodyFillColor(particle, temperatureToColor(particle.temperature, MIN_TEMPERATURE, MAX_TEMPERATURE));
        particle.onContact = info => {
            const bodyA = info.bodyA;
            const bodyB = info.bodyB;
            const colorA = temperatureToColor(bodyA.temperature, MIN_TEMPERATURE, MAX_TEMPERATURE);
            const colorB = temperatureToColor(bodyB.temperature, MIN_TEMPERATURE, MAX_TEMPERATURE);

            app.setBodyFillColor(bodyA, colorA);
            app.setBodyFillColor(bodyB, colorB);

            exchangeHeat(bodyA, bodyB, SETTINGS.dt);
        };

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

export function exchangeHeat(a: RigidBody, b: RigidBody, dt: number, k = 0.5) {
    const deltaT = a.temperature - b.temperature;
    if (deltaT === 0) return;

    const heat = k * deltaT * dt;

    if (!a.isStatic()) {
        a.temperature -= heat / a.mass;
    }

    if (!b.isStatic()) {
        b.temperature += heat / b.mass;
    }
}

export function dissipateHeat(body: RigidBody, ambientTemperature: number, dt: number, cooling = 0.001) {
    if (body.isStatic()) return;

    const deltaT = body.temperature - ambientTemperature;
    if (deltaT === 0) return;

    const perimeter = body.shape.getPerimeter();
    const heatLoss = cooling * perimeter * deltaT * dt;

    body.temperature -= heatLoss / body.mass;
}
