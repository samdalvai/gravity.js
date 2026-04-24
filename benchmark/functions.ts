import { BodiesFactory, SETTINGS, Utils, Vec2, World } from '../src';
import { applyBarnesHutCoulombForces, applyBarnesHutGravitationalForces } from '../src/force/BarnesHut';
import { applyGravitationalForces } from '../src/force/Gravity';
import { applyCoulombForces } from '../src/force/Interactions';

declare const process: {
    on(event: 'exit', listener: () => void): void;
};

const RADIUS = 2000;
SETTINGS.applyGravity = false;

const numOfParticles = 5000;
const center = new Vec2();

const world = new World(9.8);

for (let i = 0; i < numOfParticles; i++) {
    const charge = Utils.randomNumber(-100, 100);
    const radius = Math.max(Math.abs(charge) / 5, 5);
    const mass = radius / 10;
    const pos = randomPointInRadius(center, RADIUS);
    const particle = BodiesFactory.circle({
        radius: radius,
        x: pos.x,
        y: pos.y,
        mass: mass,
        charge: charge,
    });

    world.addBody(particle);
}

function randomPointInRadius(center: Vec2, radius: number): Vec2 {
    const u = Math.random();
    const v = Math.random();

    const r = radius * Math.sqrt(u);
    const theta = 2 * Math.PI * v;

    return new Vec2(center.x + Math.cos(theta) * r, center.y + Math.sin(theta) * r);
}

const bodies = world.getBodies();
const coulombForceStrength = 100;

export function runOriginal() {
    // applyCoulombForces(bodies, coulombForceStrength);
    applyGravitationalForces(bodies, 9.8, 0, 10_000);
}

export function runModified() {
    // applyBarnesHutCoulombForces(bodies, coulombForceStrength);
    applyBarnesHutGravitationalForces(bodies, 9.8, 0, 10_000);
}

process.on('exit', () => {
    //
});
