import { BodiesFactory, DistanceJoint, RigidBody, Vec2, World } from '../../src';

export type Scene = {
    world: World;
    /** Optional per-step hook, called before every `world.update()` */
    tick?: (step: number) => void;
    /** Bodies outside these bounds are counted as escaped */
    bounds: { minX: number; maxX: number; minY: number; maxY: number };
};

export type SceneBuilder = (n: number) => Scene;

const SIZE = 20;

/** Small deterministic PRNG so every run builds exactly the same scene */
function mulberry32(seed: number) {
    return () => {
        seed |= 0;
        seed = (seed + 0x6d2b79f5) | 0;
        let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
        t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
}

function addContainer(world: World, width: number, height: number) {
    const thickness = SIZE * 2;

    world.addBody(
        BodiesFactory.box({
            width: width + thickness * 2,
            height: thickness,
            x: width / 2,
            y: -thickness / 2,
            mass: 0,
        }),
    );
    world.addBody(BodiesFactory.box({ width: thickness, height, x: -thickness / 2, y: height / 2, mass: 0 }));
    world.addBody(BodiesFactory.box({ width: thickness, height, x: width + thickness / 2, y: height / 2, mass: 0 }));
}

/** Wide, low stack of touching circles resting on a floor */
function wideCircles(n: number): Scene {
    const world = new World(9.8, { maxBodies: Infinity });
    const cols = Math.ceil(Math.sqrt(n) * 4);
    const rows = Math.ceil(n / cols);
    const width = cols * SIZE;

    addContainer(world, width, rows * SIZE * 2);

    for (let i = 0; i < n; i++) {
        const col = i % cols;
        const row = Math.floor(i / cols);
        world.addBody(
            BodiesFactory.circle({
                radius: SIZE / 2,
                x: SIZE / 2 + col * SIZE,
                y: SIZE / 2 + row * SIZE,
                mass: 1,
                restitution: 0,
            }),
        );
    }

    return { world, bounds: { minX: -SIZE * 4, maxX: width + SIZE * 4, minY: -SIZE * 4, maxY: rows * SIZE * 4 } };
}

/** Tall pile of boxes in a narrow container, dense contacts and many bodies sharing the same X range */
function tallBoxes(n: number): Scene {
    const world = new World(9.8, { maxBodies: Infinity });
    const cols = Math.max(10, Math.ceil(Math.sqrt(n) / 2));
    const rows = Math.ceil(n / cols);
    const spacing = SIZE * 1.05;
    const width = cols * spacing;

    addContainer(world, width, rows * spacing * 2);

    for (let i = 0; i < n; i++) {
        const col = i % cols;
        const row = Math.floor(i / cols);
        world.addBody(
            BodiesFactory.box({
                width: SIZE,
                height: SIZE,
                x: spacing / 2 + col * spacing,
                y: spacing / 2 + row * spacing,
                mass: 1,
                restitution: 0,
            }),
        );
    }

    return { world, bounds: { minX: -SIZE * 4, maxX: width + SIZE * 4, minY: -SIZE * 4, maxY: rows * spacing * 4 } };
}

const HEXAGON = Array.from({ length: 6 }, (_, i) => {
    const angle = (i / 6) * Math.PI * 2;
    return new Vec2(Math.cos(angle) * SIZE * 0.5, Math.sin(angle) * SIZE * 0.5);
});

/** Mixed shapes falling into a container, contacts are created and destroyed continuously */
function mixedRain(n: number): Scene {
    const world = new World(9.8, { maxBodies: Infinity });
    const random = mulberry32(1234);
    const cols = Math.max(10, Math.ceil(Math.sqrt(n)));
    const rows = Math.ceil(n / cols);
    const spacing = SIZE * 1.6;
    const width = cols * spacing;
    const baseY = SIZE * 10;

    addContainer(world, width, baseY + rows * spacing * 2);

    for (let i = 0; i < n; i++) {
        const col = i % cols;
        const row = Math.floor(i / cols);
        const x = spacing / 2 + col * spacing + (random() - 0.5) * SIZE * 0.3;
        const y = baseY + row * spacing + (random() - 0.5) * SIZE * 0.3;
        let body: RigidBody;

        switch (i % 4) {
            case 0:
                body = BodiesFactory.circle({ radius: SIZE / 2, x, y, mass: 1 });
                break;
            case 1:
                body = BodiesFactory.box({ width: SIZE, height: SIZE * 0.8, x, y, mass: 1 });
                break;
            case 2:
                body = BodiesFactory.capsule({ halfHeight: SIZE * 0.25, radius: SIZE * 0.3, x, y, mass: 1 });
                break;
            default:
                body = BodiesFactory.polygon({ vertices: HEXAGON, x, y, mass: 1 });
                break;
        }

        body.rotation = random() * Math.PI;
        body.velocity.x = (random() - 0.5) * 200;
        world.addBody(body);
    }

    return {
        world,
        bounds: { minX: -SIZE * 4, maxX: width + SIZE * 4, minY: -SIZE * 4, maxY: baseY + rows * spacing * 4 },
    };
}

/** Grids of particles connected by distance joints (cloths), stresses the joint solver */
function jointChains(n: number): Scene {
    const world = new World(9.8, { maxBodies: Infinity });
    const clothSize = 30;
    const perCloth = clothSize * clothSize;
    const clothCount = Math.max(1, Math.round(n / perCloth));
    const spacing = 25;
    const clothWidth = clothSize * spacing + 100;

    addContainer(world, clothCount * clothWidth, clothSize * spacing * 4);

    for (let c = 0; c < clothCount; c++) {
        const particles: RigidBody[] = [];
        const startX = c * clothWidth + 50;
        const topY = clothSize * spacing * 2;

        for (let row = 0; row < clothSize; row++) {
            for (let col = 0; col < clothSize; col++) {
                const particle = BodiesFactory.circle({
                    radius: 1,
                    x: startX + col * spacing,
                    y: topY - row * spacing,
                    mass: row === 0 ? 0 : 1,
                });
                world.addBody(particle);
                particles.push(particle);
            }
        }

        for (let row = 0; row < clothSize; row++) {
            for (let col = 0; col < clothSize; col++) {
                const particle = particles[row * clothSize + col];

                if (row > 0) {
                    world.addJoint(new DistanceJoint(particles[(row - 1) * clothSize + col], particle));
                }

                if (col < clothSize - 1 && row > 0) {
                    world.addJoint(new DistanceJoint(particle, particles[row * clothSize + col + 1]));
                }
            }
        }
    }

    return {
        world,
        bounds: {
            minX: -SIZE * 4,
            maxX: clothCount * clothWidth + SIZE * 4,
            minY: -SIZE * 4,
            maxY: clothSize * spacing * 4,
        },
    };
}

/** A box pile continuously hit by fast CCD bullets */
function bullets(n: number): Scene {
    const scene = wideBoxes(n);
    const world = scene.world;
    const random = mulberry32(42);
    const bulletCount = 50;
    const width = scene.bounds.maxX - scene.bounds.minX;
    const launchY = scene.bounds.maxY * 0.5;
    const projectiles: RigidBody[] = [];

    for (let i = 0; i < bulletCount; i++) {
        const bullet = BodiesFactory.circle({ radius: 4, x: (i / bulletCount) * width, y: launchY, mass: 0.2 });
        world.addBody(bullet);
        projectiles.push(bullet);
    }

    const tick = (step: number) => {
        // Relaunch one bullet every other step
        if (step % 2 !== 0) return;

        const bullet = projectiles[(step / 2) % bulletCount];
        bullet.position.x = scene.bounds.minX + random() * width;
        bullet.position.y = launchY;
        bullet.velocity.x = 0;
        bullet.velocity.y = -3000;
        bullet.angularVelocity = 0;
        bullet.isBullet = true;
    };

    return { world, tick, bounds: scene.bounds };
}

function wideBoxes(n: number): Scene {
    const world = new World(9.8, { maxBodies: Infinity });
    const cols = Math.ceil(Math.sqrt(n) * 4);
    const rows = Math.ceil(n / cols);
    const width = cols * SIZE;

    addContainer(world, width, rows * SIZE * 8);

    for (let i = 0; i < n; i++) {
        const col = i % cols;
        const row = Math.floor(i / cols);
        world.addBody(
            BodiesFactory.box({
                width: SIZE,
                height: SIZE,
                x: SIZE / 2 + col * SIZE,
                y: SIZE / 2 + row * SIZE,
                mass: 1,
                restitution: 0,
            }),
        );
    }

    return { world, bounds: { minX: -SIZE * 4, maxX: width + SIZE * 4, minY: -SIZE * 4, maxY: rows * SIZE * 8 } };
}

export const SCENES: Record<string, SceneBuilder> = {
    wideCircles,
    tallBoxes,
    mixedRain,
    jointChains,
    bullets,
};
