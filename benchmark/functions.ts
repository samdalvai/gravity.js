import { BodiesFactory, Utils, Vec2, World as WorldNew } from '../src';
import { BodiesFactory as BodiesFactoryOld, Vec2 as Vec2Old, World as WorldOld } from '../src_old';

declare const process: {
    on(event: 'exit', listener: () => void): void;
};

const gravity = 9.8;

const worldNew = new WorldNew(gravity);
const worldOld = new WorldOld(gravity);

const numBodies = 1_000;

for (let i = 0; i < numBodies; i++) {
    const x = Utils.randomNumber(-500, 500);
    const y = Utils.randomNumber(0, 500);

    const p = BodiesFactory.circle({
        radius: 10,
        x,
        y,
        mass: 1,
    });

    const pOld = BodiesFactoryOld.circle({
        radius: 10,
        x,
        y,
        mass: 1,
    });

    worldNew.addBody(p);
    worldOld.addBody(pOld);
}

const floorNew = BodiesFactory.box({ width: 5000, height: 50, x: 0, y: -600, mass: 0 });
worldNew.addBody(floorNew);

const floorOld = BodiesFactoryOld.box({ width: 5000, height: 50, x: 0, y: -600, mass: 0 });
worldOld.addBody(floorOld);

const DT = 1 / 60;

const warmupIterations = 1_000;

for (let i = 0; i < warmupIterations; i++) {
    worldOld.update(DT);
    worldNew.update(DT);
}

export function runOriginal() {
    worldOld.update(DT);
}

export function runModified() {
    worldNew.update(DT);
}

process.on('exit', () => {
    console.log('');
    console.log('Vec2 allocations');
    console.log(`original: ${Vec2.allocations.toLocaleString()}`);
    console.log(`modified: ${Vec2Old.allocations.toLocaleString()}`);

    console.log('Contacts');
    console.log(`original: ${worldOld.getManifolds().length}`);
    console.log(`modified: ${worldNew.getManifolds().length}`);
});
