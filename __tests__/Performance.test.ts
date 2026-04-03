import { describe, expect, test } from '@jest/globals';

import {
    BoxShape as CurrentBoxShape,
    CapsuleShape as CurrentCapsuleShape,
    CircleShape as CurrentCircleShape,
    RigidBody as CurrentRigidBody,
    SegmentShape as CurrentSegmentShape,
    Vec2 as CurrentVec2,
} from '../src';
import * as CurrentCollision from '../src/collision/Collision_old';
import {
    BoxShape as NewBoxShape,
    CapsuleShape as NewCapsuleShape,
    CircleShape as NewCircleShape,
    RigidBody as NewRigidBody,
    SegmentShape as NewSegmentShape,
    Vec2 as NewVec2,
} from '../src_new';
import * as NewCollision from '../src_new/collision/Collision';

interface BenchmarkBodyPair {
    bodyA: unknown;
    bodyB: unknown;
}

interface BenchmarkManifold {
    numContacts: number;
    preSolve(inverseDeltaTime: number): void;
    solve(): void;
}

interface BenchmarkApi {
    BoxShape: new (width: number, height: number) => unknown;
    CapsuleShape: new (length: number, radius: number) => unknown;
    CircleShape: new (radius: number) => unknown;
    SegmentShape: new (start: unknown, end: unknown) => unknown;
    RigidBody: new (shape: unknown, x: number, y: number, density: number) => unknown;
    Vec2: new (x: number, y: number) => unknown;
    detectCollision(bodyA: unknown, bodyB: unknown): BenchmarkManifold | null;
}

interface BenchmarkScenario {
    name: string;
    createBodies(api: BenchmarkApi): BenchmarkBodyPair;
}

interface InspectionResult {
    hasCollision: boolean;
    contactCount: number;
}

interface BenchmarkStats {
    averageMilliseconds: number;
    microsecondsPerIteration: number;
}

interface ImplementationResult {
    inspection: InspectionResult;
    collision: BenchmarkStats;
    resolution: BenchmarkStats | null;
}

interface ComparisonResult {
    name: string;
    current: ImplementationResult;
    next: ImplementationResult;
}

const RUN_PERF = process.env.RUN_PERF === '1';
const describePerformance = RUN_PERF ? describe : describe.skip;
const PERF_TIE_THRESHOLD = 0.05;

const PERF_SAMPLES = readPositiveInteger('PERF_SAMPLES', 3);
const PERF_WARMUP_SAMPLES = readNonNegativeInteger('PERF_WARMUP_SAMPLES', 1);
const PERF_COLLISION_ITERATIONS = readPositiveInteger('PERF_COLLISION_ITERATIONS', 10_000);
const PERF_RESOLUTION_ITERATIONS = readPositiveInteger('PERF_RESOLUTION_ITERATIONS', 5_000);
const PERF_SOLVER_ITERATIONS = readPositiveInteger('PERF_SOLVER_ITERATIONS', 10);
const PERF_TIMEOUT_MS = readPositiveInteger('PERF_TIMEOUT_MS', 120_000);
const INVERSE_DELTA_TIME = 60;

const CURRENT_API: BenchmarkApi = {
    BoxShape: CurrentBoxShape,
    CapsuleShape: CurrentCapsuleShape,
    CircleShape: CurrentCircleShape,
    SegmentShape: CurrentSegmentShape,
    RigidBody: CurrentRigidBody,
    Vec2: CurrentVec2,
    detectCollision: CurrentCollision.detectCollision,
};

const NEW_API: BenchmarkApi = {
    BoxShape: NewBoxShape,
    CapsuleShape: NewCapsuleShape,
    CircleShape: NewCircleShape,
    SegmentShape: NewSegmentShape,
    RigidBody: NewRigidBody,
    Vec2: NewVec2,
    detectCollision: NewCollision.detectCollision,
};

const SCENARIOS: BenchmarkScenario[] = [
    {
        name: 'circle-circle',
        createBodies: ({ CircleShape, RigidBody }) => ({
            bodyA: new RigidBody(new CircleShape(30), 100, 120, 5),
            bodyB: new RigidBody(new CircleShape(30), 100, 100, 5),
        }),
    },
    {
        name: 'box-box',
        createBodies: ({ BoxShape, RigidBody }) => ({
            bodyA: new RigidBody(new BoxShape(60, 60), 100, 120, 5),
            bodyB: new RigidBody(new BoxShape(60, 60), 100, 100, 5),
        }),
    },
    {
        name: 'capsule-capsule',
        createBodies: ({ CapsuleShape, RigidBody }) => ({
            bodyA: new RigidBody(new CapsuleShape(30, 30), 100, 120, 5),
            bodyB: new RigidBody(new CapsuleShape(30, 30), 100, 100, 5),
        }),
    },
    {
        name: 'segment-segment',
        createBodies: ({ RigidBody, SegmentShape, Vec2 }) => ({
            bodyA: new RigidBody(new SegmentShape(new Vec2(-50, 0), new Vec2(50, 0)), 120, 100, 5),
            bodyB: new RigidBody(new SegmentShape(new Vec2(0, -50), new Vec2(0, 50)), 100, 100, 5),
        }),
    },
    {
        name: 'circle-box',
        createBodies: ({ BoxShape, CircleShape, RigidBody }) => ({
            bodyA: new RigidBody(new CircleShape(30), 100, 120, 5),
            bodyB: new RigidBody(new BoxShape(60, 60), 100, 100, 5),
        }),
    },
    {
        name: 'circle-capsule',
        createBodies: ({ CapsuleShape, CircleShape, RigidBody }) => ({
            bodyA: new RigidBody(new CircleShape(30), 100, 120, 5),
            bodyB: new RigidBody(new CapsuleShape(30, 30), 100, 100, 5),
        }),
    },
    {
        name: 'circle-segment',
        createBodies: ({ CircleShape, RigidBody, SegmentShape, Vec2 }) => ({
            bodyA: new RigidBody(new CircleShape(30), 100, 120, 5),
            bodyB: new RigidBody(new SegmentShape(new Vec2(0, -50), new Vec2(0, 50)), 100, 100, 5),
        }),
    },
    {
        name: 'box-segment',
        createBodies: ({ BoxShape, RigidBody, SegmentShape, Vec2 }) => ({
            bodyA: new RigidBody(new SegmentShape(new Vec2(0, -50), new Vec2(0, 50)), 100, 100, 5),
            bodyB: new RigidBody(new BoxShape(60, 60), 100, 100, 5),
        }),
    },
    {
        name: 'box-capsule',
        createBodies: ({ BoxShape, CapsuleShape, RigidBody }) => ({
            bodyA: new RigidBody(new BoxShape(60, 60), 100, 100, 5),
            bodyB: new RigidBody(new CapsuleShape(30, 30), 100, 100, 5),
        }),
    },
    {
        name: 'capsule-segment',
        createBodies: ({ CapsuleShape, RigidBody, SegmentShape, Vec2 }) => ({
            bodyA: new RigidBody(new SegmentShape(new Vec2(0, -50), new Vec2(0, 50)), 100, 100, 5),
            bodyB: new RigidBody(new CapsuleShape(30, 30), 100, 100, 5),
        }),
    },
];

describePerformance('Performance comparison: current vs new collision pairs', () => {
    test(
        'prints one table with collision and resolution timings for every pair',
        () => {
            const results = SCENARIOS.map((scenario) => runComparison(scenario));
            const missingCollisionPairs = results.filter(
                ({ current, next }) => !current.inspection.hasCollision || !next.inspection.hasCollision,
            );

            if (missingCollisionPairs.length > 0) {
                const labels = missingCollisionPairs.map(({ name }) => name).join(', ');
                throw new Error(`Expected all benchmark scenarios to collide in both implementations: ${labels}`);
            }

            console.log(
                [
                    [
                        'Collision benchmark comparison',
                        `samples=${PERF_SAMPLES}`,
                        `warmup=${PERF_WARMUP_SAMPLES}`,
                        `collisionIterations=${PERF_COLLISION_ITERATIONS}`,
                        `resolutionIterations=${PERF_RESOLUTION_ITERATIONS}`,
                        `solverIterations=${PERF_SOLVER_ITERATIONS}`,
                    ].join(' | '),
                    formatTable(results),
                    formatSummary(results),
                ].join('\n'),
            );

            expect(results).toHaveLength(SCENARIOS.length);
        },
        PERF_TIMEOUT_MS,
    );
});

function runComparison(scenario: BenchmarkScenario): ComparisonResult {
    return {
        name: scenario.name,
        current: runImplementation(CURRENT_API, scenario),
        next: runImplementation(NEW_API, scenario),
    };
}

function runImplementation(api: BenchmarkApi, scenario: BenchmarkScenario): ImplementationResult {
    const inspection = inspectScenario(api, scenario);

    return {
        inspection,
        collision: benchmark(PERF_COLLISION_ITERATIONS, () => {
            const { bodyA, bodyB } = scenario.createBodies(api);

            return () => {
                api.detectCollision(bodyA, bodyB);
            };
        }),
        resolution: inspection.hasCollision
            ? benchmark(PERF_RESOLUTION_ITERATIONS, () => {
                  const { bodyA, bodyB } = scenario.createBodies(api);
                  const manifold = api.detectCollision(bodyA, bodyB);

                  if (manifold === null) {
                      throw new Error(`Cannot benchmark resolution for ${scenario.name}; no manifold was created.`);
                  }

                  return () => {
                      manifold.preSolve(INVERSE_DELTA_TIME);

                      for (let i = 0; i < PERF_SOLVER_ITERATIONS; i++) {
                          manifold.solve();
                      }
                  };
              })
            : null,
    };
}

function inspectScenario(api: BenchmarkApi, scenario: BenchmarkScenario): InspectionResult {
    const { bodyA, bodyB } = scenario.createBodies(api);
    const manifold = api.detectCollision(bodyA, bodyB);

    return {
        hasCollision: manifold !== null,
        contactCount: manifold?.numContacts ?? 0,
    };
}

function benchmark(iterations: number, createOperation: () => () => void): BenchmarkStats {
    for (let i = 0; i < PERF_WARMUP_SAMPLES; i++) {
        runTimedLoop(iterations, createOperation());
    }

    const samples: number[] = [];

    for (let i = 0; i < PERF_SAMPLES; i++) {
        samples.push(runTimedLoop(iterations, createOperation()));
    }

    const averageMilliseconds = samples.reduce((sum, sample) => sum + sample, 0) / samples.length;

    return {
        averageMilliseconds,
        microsecondsPerIteration: (averageMilliseconds * 1000) / iterations,
    };
}

function runTimedLoop(iterations: number, operation: () => void): number {
    const start = process.hrtime.bigint();

    for (let i = 0; i < iterations; i++) {
        operation();
    }

    const durationNanoseconds = Number(process.hrtime.bigint() - start);
    return durationNanoseconds / 1_000_000;
}

function formatTable(results: ComparisonResult[]): string {
    const headers = [
        'Pair',
        'Contacts C/N',
        'Coll C us',
        'Coll N us',
        'Coll Win',
        'Res C us',
        'Res N us',
        'Res Win',
    ];

    const rows = results.map((result) => [
        result.name,
        `${result.current.inspection.contactCount}/${result.next.inspection.contactCount}`,
        formatMicroseconds(result.current.collision),
        formatMicroseconds(result.next.collision),
        formatWinner(result.current.collision, result.next.collision),
        formatMicroseconds(result.current.resolution),
        formatMicroseconds(result.next.resolution),
        formatWinner(result.current.resolution, result.next.resolution),
    ]);

    const widths = headers.map((header, index) =>
        Math.max(header.length, ...rows.map((row) => row[index].length)),
    );

    const border = widths.map((width) => '-'.repeat(width)).join('-+-');
    const lines = [formatRow(headers, widths), border];

    for (const row of rows) {
        lines.push(formatRow(row, widths));
    }

    return lines.join('\n');
}

function formatSummary(results: ComparisonResult[]): string {
    const collisionWins = countWinners(results, 'collision');
    const resolutionWins = countWinners(results, 'resolution');
    const mismatchedContacts = results.filter(
        ({ current, next }) => current.inspection.contactCount !== next.inspection.contactCount,
    );

    const summaryLines = [
        `Collision wins  | current=${collisionWins.current} | new=${collisionWins.next} | ties=${collisionWins.tie}`,
        `Resolution wins | current=${resolutionWins.current} | new=${resolutionWins.next} | ties=${resolutionWins.tie}`,
    ];

    if (mismatchedContacts.length === 0) {
        summaryLines.push('Contact counts | all pairs matched');
    } else {
        summaryLines.push(
            `Contact counts | mismatches=${mismatchedContacts
                .map(({ name, current, next }) => `${name} (${current.inspection.contactCount}/${next.inspection.contactCount})`)
                .join(', ')}`,
        );
    }

    return summaryLines.join('\n');
}

function countWinners(
    results: ComparisonResult[],
    metric: 'collision' | 'resolution',
): { current: number; next: number; tie: number } {
    let current = 0;
    let next = 0;
    let tie = 0;

    for (const result of results) {
        const left = result.current[metric];
        const right = result.next[metric];
        const winner = getWinner(left, right);

        if (winner === 'current') current++;
        else if (winner === 'new') next++;
        else tie++;
    }

    return { current, next, tie };
}

function formatMicroseconds(stats: BenchmarkStats | null): string {
    if (stats === null) {
        return 'n/a';
    }

    return stats.microsecondsPerIteration.toFixed(2);
}

function formatWinner(current: BenchmarkStats | null, next: BenchmarkStats | null): string {
    const winner = getWinner(current, next);

    if (winner === 'tie') {
        return 'tie';
    }

    if (current === null || next === null) {
        return 'n/a';
    }

    const ratio =
        winner === 'current'
            ? next.microsecondsPerIteration / current.microsecondsPerIteration
            : current.microsecondsPerIteration / next.microsecondsPerIteration;

    return `${winner} ${ratio.toFixed(2)}x`;
}

function getWinner(current: BenchmarkStats | null, next: BenchmarkStats | null): 'current' | 'new' | 'tie' {
    if (current === null || next === null) {
        return 'tie';
    }

    const difference = Math.abs(current.microsecondsPerIteration - next.microsecondsPerIteration);
    const baseline = Math.min(current.microsecondsPerIteration, next.microsecondsPerIteration);

    if (baseline === 0 || difference / baseline < PERF_TIE_THRESHOLD) {
        return 'tie';
    }

    return current.microsecondsPerIteration < next.microsecondsPerIteration ? 'current' : 'new';
}

function formatRow(columns: string[], widths: number[]): string {
    return columns.map((column, index) => column.padEnd(widths[index])).join(' | ');
}

function readPositiveInteger(name: string, fallback: number): number {
    const rawValue = process.env[name];

    if (rawValue === undefined) {
        return fallback;
    }

    const parsedValue = Number(rawValue);

    if (!Number.isFinite(parsedValue) || parsedValue <= 0) {
        return fallback;
    }

    return Math.floor(parsedValue);
}

function readNonNegativeInteger(name: string, fallback: number): number {
    const rawValue = process.env[name];

    if (rawValue === undefined) {
        return fallback;
    }

    const parsedValue = Number(rawValue);

    if (!Number.isFinite(parsedValue) || parsedValue < 0) {
        return fallback;
    }

    return Math.floor(parsedValue);
}
