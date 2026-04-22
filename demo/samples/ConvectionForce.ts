import { BodiesFactory, RigidBody, SETTINGS, Utils, type World } from '../../src';
import type Application from '../Application';
import Graphics from '../graphics/Graphics';
// import { FENCE_WIDTH, FLOOR_HEIGHT, defineDemo, generateCeiling, generateFences, generateFloor } from './shared';
import { defineDemo } from './shared';

export const AMBIENT_TEMPERATURE = 0;
export const MIN_TEMPERATURE = 0;
export const MAX_TEMPERATURE = 5_000;
export const PARTICLE_MASS = 0.015;
export const HEATING_FACTOR = 0.65;
export const DISSIPATION_FACTOR = 0.000001;
export const CONVECTION_FORCE = 0.03;
export const MIN_TEMPERATURE_DIFFERENCE = 1000;

const STATIC_WALL_WIDTH = 50;
const STATIC_WALL_HEIGHT = 500;

function createStaticWall(
    world: World,
    app: Application,
    x: number,
    y: number,
    rotation: number,
    height = STATIC_WALL_HEIGHT,
    temperature = 0,
): RigidBody {
    const wall = BodiesFactory.box({ width: STATIC_WALL_WIDTH, height, x, y, mass: 0.0, rotation, temperature });
    app.setBodyTexture(wall, 'transparent');

    if (temperature > 0) {
        app.removeBodyTexture(wall);
    }
    world.addBody(wall);
    return wall;
}

function setupConvectionForce(world: World, app: Application): void {
    // Simpler version
    // Graphics.zoom = 0.5;

    // const FLOOR_WIDTH = 1500;
    // const WALL_COLOR = 'rgb(90, 45, 20)';

    // const floor = generateFloor(world, app, FLOOR_WIDTH);
    // floor.temperature = MAX_TEMPERATURE;
    // app.removeBodyTexture(floor);
    // app.setBodyFillColor(floor, temperatureToColor(floor.temperature, MIN_TEMPERATURE, MAX_TEMPERATURE));

    // const fences = generateFences(world, app, FLOOR_WIDTH);
    // app.removeBodyTexture(fences[0]);
    // app.removeBodyTexture(fences[1]);
    // app.setBodyFillColor(fences[0], WALL_COLOR);
    // app.setBodyFillColor(fences[1], WALL_COLOR);

    // const ceiling = generateCeiling(world, app, FLOOR_WIDTH);
    // app.removeBodyTexture(ceiling);
    // app.setBodyFillColor(ceiling, WALL_COLOR);
    // world.addBody(ceiling);

    // const numOfParticles = 2_500;
    // const particleRadius = 5;
    // const BASE_Y = floor.position.y + FLOOR_HEIGHT / 2 + particleRadius;
    // const MAX_X = fences[0].position.x + FENCE_WIDTH / 2 + particleRadius;

    // for (let i = 0; i < numOfParticles; i++) {
    //     const particle = BodiesFactory.circle({
    //         radius: particleRadius,
    //         x: Utils.randomNumber(-MAX_X, MAX_X),
    //         y: Utils.randomNumber(BASE_Y, 250),
    //         mass: PARTICLE_MASS,
    //         temperature: MIN_TEMPERATURE,
    //     });

    //     app.setBodyFillColor(particle, temperatureToColor(particle.temperature, MIN_TEMPERATURE, MAX_TEMPERATURE));
    //     particle.onContact = info => {
    //         const bodyA = info.bodyA;
    //         const bodyB = info.bodyB;
    //         exchangeHeat(bodyA, bodyB, SETTINGS.dt, HEATING_FACTOR);
    //     };

    //     world.addBody(particle);
    // }

    // app.setConvectionForce(true);

    // Chimney version
    Graphics.zoom = 0.4;

    const floorWidth = 525;
    const floorHeight = 50;

    const floor = BodiesFactory.box({ width: floorWidth, height: floorHeight, x: 12.5, y: -1000, mass: 0.0 });
    floor.temperature = MAX_TEMPERATURE;
    app.removeBodyTexture(floor);
    app.setBodyFillColor(floor, temperatureToColor(floor.temperature, MIN_TEMPERATURE, MAX_TEMPERATURE));
    world.addBody(floor);

    const staticWallsOptions = [
        { x: -225, y: -390, rotation: 0, height: STATIC_WALL_HEIGHT * 1.55 },
        { x: -225, y: -900, rotation: 0, height: STATIC_WALL_HEIGHT * 0.5, temperature: MAX_TEMPERATURE },
        { x: -145, y: 110, rotation: -0.5, height: STATIC_WALL_HEIGHT / 1.5 },
        { x: 60, y: 350, rotation: -0.95, height: STATIC_WALL_HEIGHT / 1.5 },
        { x: 350, y: 475, rotation: -1.4, height: STATIC_WALL_HEIGHT / 1.5 },
        { x: 675, y: 465, rotation: -1.8, height: STATIC_WALL_HEIGHT / 1.5 },
        { x: 930, y: 310, rotation: -2.5, height: STATIC_WALL_HEIGHT / 1.5 },
        { x: 1035, y: 25, rotation: -0, height: STATIC_WALL_HEIGHT / 1.5 },
        { x: 960, y: -275, rotation: -0.5, height: STATIC_WALL_HEIGHT / 1.5 },
        { x: 770, y: -525, rotation: -0.8, height: STATIC_WALL_HEIGHT / 1.5 },
        { x: 450, y: -720, rotation: -1.2, height: STATIC_WALL_HEIGHT / 1.1 },
        { x: 250, y: -250, rotation: 0, height: STATIC_WALL_HEIGHT * 1.5 },
        { x: 250, y: -900, rotation: 0, height: STATIC_WALL_HEIGHT * 0.5, temperature: MAX_TEMPERATURE },
    ];

    const WALL_COLOR = 'rgb(90, 45, 20)';

    for (const option of staticWallsOptions) {
        const wall = createStaticWall(
            world,
            app,
            option.x,
            option.y,
            option.rotation,
            option.height,
            option.temperature,
        );
        app.setBodyFillColor(wall, WALL_COLOR);
    }

    const numOfParticles = 2_500;
    const particleRadius = 5;
    const BASE_Y = floor.position.y + floorHeight / 2 + particleRadius;
    const MIN_X = floor.position.x - floorWidth / 2 + particleRadius + STATIC_WALL_WIDTH;
    const MAX_X = floor.position.x + floorWidth / 2 + particleRadius - STATIC_WALL_WIDTH;

    for (let i = 0; i < numOfParticles; i++) {
        const particle = BodiesFactory.circle({
            radius: particleRadius,
            x: Utils.randomNumber(MIN_X, MAX_X),
            y: Utils.randomNumber(BASE_Y, BASE_Y + 800),
            mass: PARTICLE_MASS,
            temperature: MIN_TEMPERATURE,
        });

        app.setBodyFillColor(particle, temperatureToColor(particle.temperature, MIN_TEMPERATURE, MAX_TEMPERATURE));
        particle.onContact = info => {
            const bodyA = info.bodyA;
            const bodyB = info.bodyB;
            exchangeHeat(bodyA, bodyB, SETTINGS.dt, HEATING_FACTOR);
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

    a.temperature = Utils.clamp(a.temperature, MIN_TEMPERATURE, MAX_TEMPERATURE);
    b.temperature = Utils.clamp(b.temperature, MIN_TEMPERATURE, MAX_TEMPERATURE);
}

export function dissipateHeat(body: RigidBody, ambientTemperature: number, dt: number, cooling = 0.001) {
    if (body.isStatic()) return;

    const deltaT = body.temperature - ambientTemperature;
    if (deltaT === 0) return;

    const perimeter = body.shape.getPerimeter();
    const heatLoss = cooling * perimeter * deltaT * dt;

    body.temperature = Math.max(ambientTemperature, body.temperature - heatLoss / body.mass);
}
