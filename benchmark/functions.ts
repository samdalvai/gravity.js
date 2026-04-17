import { Bodies as BodiesCurrent, GRAVITY, World as WorldCurrent } from '../src';
import { Bodies as BodiesNew, World as WorldNew } from '../src';

const CIRCLE_COUNT = 1000;
const CIRCLE_RADIUS = 10;
const CIRCLE_MASS = 1;
const WALL_THICKNESS = 50;
const SQUARE_CAGE_INNER_SIZE = 1000;
const FLOOR_POSITION_Y = -350;
const WORLD_SEED = 0x51c1e5ed;

interface CageBounds {
    innerLeft: number;
    innerRight: number;
    innerBottom: number;
    innerTop: number;
}

interface BodyOptions {
    x: number;
    y: number;
    mass: number;
}

interface BoxBodyOptions extends BodyOptions {
    width: number;
    height: number;
}

interface CircleBodyOptions extends BodyOptions {
    radius: number;
}

interface StressBody {
    friction: number;
    restitution: number;
}

interface BodiesFactory<Body extends StressBody> {
    box(options: BoxBodyOptions): Body;
    circle(options: CircleBodyOptions): Body;
}

interface StressWorld<Body> {
    addBody(body: Body): void;
}

type WorldConstructor<World> = new (gravity: number) => World;

export function createCircleStressWorld<Body extends StressBody, World extends StressWorld<Body>>(
    WorldImplementation: WorldConstructor<World>,
    BodiesImplementation: BodiesFactory<Body>,
): World {
    const world = new WorldImplementation(GRAVITY);
    const cage = generateSquareCage(world, BodiesImplementation);

    populateCircleStressWorld(world, BodiesImplementation, cage);

    return world;
}

function generateSquareCage<Body extends StressBody>(
    world: StressWorld<Body>,
    bodies: BodiesFactory<Body>,
): CageBounds {
    const innerLeft = -SQUARE_CAGE_INNER_SIZE / 2;
    const innerRight = SQUARE_CAGE_INNER_SIZE / 2;
    const innerBottom = FLOOR_POSITION_Y + WALL_THICKNESS / 2;
    const innerTop = innerBottom + SQUARE_CAGE_INNER_SIZE;
    const centerY = (innerBottom + innerTop) / 2;
    const wallSpanVertical = SQUARE_CAGE_INNER_SIZE + WALL_THICKNESS * 2;
    const wallOptions: BoxBodyOptions[] = [
        {
            width: SQUARE_CAGE_INNER_SIZE,
            height: WALL_THICKNESS,
            x: 0,
            y: innerBottom - WALL_THICKNESS / 2,
            mass: 0,
        },
        {
            width: SQUARE_CAGE_INNER_SIZE,
            height: WALL_THICKNESS,
            x: 0,
            y: innerTop + WALL_THICKNESS / 2,
            mass: 0,
        },
        {
            width: WALL_THICKNESS,
            height: wallSpanVertical,
            x: innerLeft - WALL_THICKNESS / 2,
            y: centerY,
            mass: 0,
        },
        {
            width: WALL_THICKNESS,
            height: wallSpanVertical,
            x: innerRight + WALL_THICKNESS / 2,
            y: centerY,
            mass: 0,
        },
    ];

    for (const options of wallOptions) {
        world.addBody(bodies.box(options));
    }

    return { innerLeft, innerRight, innerBottom, innerTop };
}

function populateCircleStressWorld<Body extends StressBody>(
    world: StressWorld<Body>,
    bodies: BodiesFactory<Body>,
    bounds: CageBounds,
): void {
    const random = createSeededRandom(WORLD_SEED);
    const gridSize = Math.ceil(Math.sqrt(CIRCLE_COUNT));
    const minX = bounds.innerLeft + CIRCLE_RADIUS;
    const maxX = bounds.innerRight - CIRCLE_RADIUS;
    const minY = bounds.innerBottom + CIRCLE_RADIUS;
    const maxY = bounds.innerTop - CIRCLE_RADIUS;
    const cellWidth = (maxX - minX) / gridSize;
    const cellHeight = (maxY - minY) / gridSize;
    const jitterX = Math.max(0, cellWidth - CIRCLE_RADIUS * 2) * 0.65;
    const jitterY = Math.max(0, cellHeight - CIRCLE_RADIUS * 2) * 0.65;
    const cells: [number, number][] = [];
    const rowOffsets: number[] = [];

    for (let row = 0; row < gridSize; row++) {
        rowOffsets.push(((row % 2 === 0 ? -0.22 : 0.22) + (random() - 0.5) * 0.08) * cellWidth);

        for (let col = 0; col < gridSize; col++) {
            cells.push([row, col]);
        }
    }

    shuffle(cells, random);

    for (let i = 0; i < CIRCLE_COUNT; i++) {
        const [row, col] = cells[i];
        const x = minX + (col + 0.5) * cellWidth + rowOffsets[row] + (random() - 0.5) * jitterX;
        const y = minY + (row + 0.5) * cellHeight + (random() - 0.5) * jitterY;
        const body = bodies.circle({
            radius: CIRCLE_RADIUS,
            x: clamp(x, minX, maxX),
            y: clamp(y, minY, maxY),
            mass: CIRCLE_MASS,
        });

        body.restitution = 0.05;
        body.friction = 0.6;
        world.addBody(body);
    }
}

function createSeededRandom(seed: number): () => number {
    let state = seed >>> 0;

    return () => {
        state = (Math.imul(state, 1664525) + 1013904223) >>> 0;
        return state / 0x100000000;
    };
}

function shuffle<T>(items: T[], random: () => number): void {
    for (let i = items.length - 1; i > 0; i--) {
        const j = Math.floor(random() * (i + 1));
        const item = items[i];
        items[i] = items[j];
        items[j] = item;
    }
}

function clamp(value: number, min: number, max: number): number {
    return Math.max(min, Math.min(max, value));
}

const worldCurrent = createCircleStressWorld(WorldCurrent, BodiesCurrent);
const worldNew = createCircleStressWorld(WorldNew, BodiesNew);

const WARMUP_ITERATIONS = 1_000;
const DT = 1 / 60;

for (let i = 0; i < WARMUP_ITERATIONS; i++) {
    worldCurrent.update(DT);
    worldNew.update(DT);
}

export function runOriginal() {
    worldCurrent.update(DT);
}

export function runModified() {
    worldNew.update(DT);
}
