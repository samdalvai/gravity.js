import { BodiesFactory, Utils, Vec2, World } from '../src';
import { BodiesFactory as BodiesFactoryOld, Vec2 as Vec2Old, World as WorldOld } from '../src_old';

declare const process: {
    on(event: 'exit', listener: () => void): void;
};

const world = new World(-9.8);
const worldOld = new WorldOld(-9.8);

const numBodies = 1_000;

for (let i = 0; i < numBodies; i++) {
    const x = Utils.randomNumber(-1000, 1000);
    const y = Utils.randomNumber(-500, 500);

    const p = BodiesFactory.circle({
        radius: 20,
        x,
        y,
        mass: 1,
    });

    const pOld = BodiesFactoryOld.circle({
        radius: 20,
        x,
        y,
        mass: 1,
    });

    world.addBody(p);
    worldOld.addBody(pOld);
}

const floor = BodiesFactory.box({ width: 5000, height: 50, x: 0, y: -600, mass: 0 });
world.addBody(floor);

const floorOld = BodiesFactoryOld.box({ width: 5000, height: 50, x: 0, y: -600, mass: 0 });
worldOld.addBody(floorOld);

const DT = 1 / 60;

export function runOriginal() {
    worldOld.update(DT);
}

export function runModified() {
    world.update(DT);
}

process.on('exit', () => {
    console.log('');
    console.log('Vec2 allocations');
    console.log(`original: ${Vec2.allocations.toLocaleString()}`);
    console.log(`modified: ${Vec2Old.allocations.toLocaleString()}`);
});
