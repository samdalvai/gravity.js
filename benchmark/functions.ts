import { BodiesFactory, Utils, Vec2, World as WorldNew } from '../src';
import { BodiesFactory as BodiesFactoryOld, Vec2 as Vec2Old, World as WorldOld } from '../src_old';

declare const process: {
    on(event: 'exit', listener: () => void): void;
};

const gravity = 9.8;

const worldNew = new WorldNew(gravity);
const worldOld = new WorldOld(gravity);

const numBodies = 4_900;

for (let i = 0; i < numBodies; i++) {
    const x = Utils.randomNumber(-500, 500);
    const y = Utils.randomNumber(0, 1000);

    const p = BodiesFactory.circle({
        radius: 10,
        x,
        y,
        mass: 1,
        restitution: 0,
    });

    const pOld = BodiesFactoryOld.circle({
        radius: 10,
        x,
        y,
        mass: 1,
        restitution: 0,
    });

    worldNew.addBody(p);
    worldOld.addBody(pOld);
}

const floorNew = BodiesFactory.box({ width: 5000, height: 50, x: 0, y: -200, mass: 0 });
const leftFenceNew = BodiesFactory.box({ width: 50, height: 5000, x: -600, y: 0, mass: 0 });
const rightFenceNew = BodiesFactory.box({ width: 50, height: 5000, x: 600, y: 0, mass: 0 });
worldNew.addBody(floorNew);
worldNew.addBody(leftFenceNew);
worldNew.addBody(rightFenceNew);

const floorOld = BodiesFactoryOld.box({ width: 5000, height: 50, x: 0, y: -200, mass: 0 });
const leftFenceOld = BodiesFactoryOld.box({ width: 50, height: 5000, x: -600, y: 0, mass: 0 });
const rightFenceOld = BodiesFactoryOld.box({ width: 50, height: 5000, x: 600, y: 0, mass: 0 });
worldOld.addBody(floorOld);
worldOld.addBody(leftFenceOld);
worldOld.addBody(rightFenceOld);

const DT = 1 / 60;

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
