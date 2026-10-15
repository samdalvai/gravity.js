/**
 * Scaling benchmark: measures how the cost of a world step grows with the number of bodies.
 *
 * Usage (see `npm run benchmark:scaling`):
 *   --label <name>            name of the result file written to benchmark/results/<name>.json
 *   --compare <file.json>     compare against a previous result file
 *   --scenes a,b              subset of scenes to run (default: all)
 *   --sizes 1000,2000         body counts to run (default: 1k..32k)
 *   --warmup <steps>          steps simulated before measuring (default: 120)
 *   --steps <steps>           measured steps (default: 300)
 *
 * Every (scene, size) pair runs in a fresh child process, so body ids, the manifold pool
 * and JIT state never leak from one measurement into the next.
 */
import { execFileSync } from 'child_process';
import * as fs from 'fs';
import * as path from 'path';

import { RigidBody, World } from '../../src';
import { SCENES } from './scenes';

const FRAME_BUDGET_MS = 1000 / 60;
/** Larger sizes of a scene are skipped once a step gets slower than this */
const MAX_STEP_MS = 150;
const PHASES = ['broadPhase', 'narrowPhase', 'solveConstraints', 'ccd'] as const;

type Phase = (typeof PHASES)[number];

type Result = {
    scene: string;
    n: number;
    medianMs: number;
    p95Ms: number;
    meanMs: number;
    phasesMs: Record<Phase | 'other', number>;
    contacts: number;
    maxPenetration: number;
    kineticEnergy: number;
    escaped: number;
};

type ResultFile = {
    label: string;
    date: string;
    node: string;
    results: Result[];
};

function parseArgs() {
    const args = process.argv.slice(2);
    const get = (name: string) => {
        const index = args.indexOf(`--${name}`);
        return index >= 0 ? args[index + 1] : undefined;
    };

    return {
        child: get('child'),
        label: get('label') ?? 'latest',
        compare: get('compare'),
        scenes: (get('scenes') ?? Object.keys(SCENES).join(',')).split(','),
        sizes: (get('sizes') ?? '1000,2000,4000,8000,16000,32000').split(',').map(Number),
        warmup: Number(get('warmup') ?? 120),
        steps: Number(get('steps') ?? 300),
    };
}

/** Wraps private World phases to accumulate the time spent in each one */
function instrumentPhases(): Record<Phase, number> {
    const totals = {} as Record<Phase, number>;
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    const proto = World.prototype as any;

    for (const phase of PHASES) {
        totals[phase] = 0;
        const original = proto[phase];

        proto[phase] = function (...args: unknown[]) {
            const start = performance.now();
            const result = original.apply(this, args);
            totals[phase] += performance.now() - start;
            return result;
        };
    }

    return totals;
}

function runChild(scene: string, n: number, warmup: number, steps: number): Result {
    const phaseTotals = instrumentPhases();
    const { world, tick, bounds } = SCENES[scene](n);

    let step = 0;
    for (; step < warmup; step++) {
        tick?.(step);
        world.update();
    }

    for (const phase of PHASES) phaseTotals[phase] = 0;

    const times: number[] = [];
    let total = 0;

    for (let i = 0; i < steps; i++, step++) {
        tick?.(step);
        const start = performance.now();
        world.update();
        const elapsed = performance.now() - start;
        times.push(elapsed);
        total += elapsed;
    }

    times.sort((a, b) => a - b);

    const phasesMs = {} as Record<Phase | 'other', number>;
    let phaseSum = 0;
    for (const phase of PHASES) {
        phasesMs[phase] = phaseTotals[phase] / steps;
        phaseSum += phaseTotals[phase];
    }
    phasesMs.other = (total - phaseSum) / steps;

    let maxPenetration = 0;
    for (const manifold of world.getManifolds()) {
        maxPenetration = Math.max(maxPenetration, manifold.penetrationDepth);
    }

    let kineticEnergy = 0;
    let escaped = 0;
    for (const body of world.getBodies() as RigidBody[]) {
        const { x, y } = body.position;
        if (x < bounds.minX || x > bounds.maxX || y < bounds.minY || y > bounds.maxY) escaped++;
        if (body.isStatic() || body.isBullet) continue;
        kineticEnergy += 0.5 * body.mass * body.velocity.magnitudeSquared() + 0.5 * body.I * body.angularVelocity ** 2;
    }

    return {
        scene,
        n,
        medianMs: times[Math.floor(times.length / 2)],
        p95Ms: times[Math.min(times.length - 1, Math.floor(times.length * 0.95))],
        meanMs: total / steps,
        phasesMs,
        contacts: world.getManifolds().length,
        maxPenetration,
        kineticEnergy,
        escaped,
    };
}

/** Largest body count that fits the 60 Hz frame budget, interpolated in log-log space */
function maxBodiesAt60Hz(rows: Result[]): string {
    if (rows.length === 0) return '-';
    if (rows[0].medianMs > FRAME_BUDGET_MS) return `< ${rows[0].n}`;

    for (let i = 1; i < rows.length; i++) {
        const a = rows[i - 1];
        const b = rows[i];

        if (b.medianMs > FRAME_BUDGET_MS) {
            const t = Math.log(FRAME_BUDGET_MS / a.medianMs) / Math.log(b.medianMs / a.medianMs);
            return `~${Math.round(Math.exp(Math.log(a.n) + t * Math.log(b.n / a.n)))}`;
        }
    }

    return `> ${rows[rows.length - 1].n}`;
}

function fmt(value: number, digits = 2) {
    return value.toFixed(digits);
}

function printResults(results: Result[], baseline?: ResultFile) {
    const scenes = [...new Set(results.map(r => r.scene))];

    for (const scene of scenes) {
        const rows = results.filter(r => r.scene === scene);
        const baseRows = baseline?.results.filter(r => r.scene === scene) ?? [];

        console.log('');
        console.log(
            `== ${scene} == max bodies @60Hz: ${maxBodiesAt60Hz(rows)}` +
                (baseline ? ` (baseline ${maxBodiesAt60Hz(baseRows)})` : ''),
        );

        const table = rows.map(r => {
            const base = baseRows.find(b => b.n === r.n);
            const row: Record<string, string | number> = {
                n: r.n,
                'median ms': fmt(r.medianMs),
                'p95 ms': fmt(r.p95Ms),
                broad: fmt(r.phasesMs.broadPhase),
                narrow: fmt(r.phasesMs.narrowPhase),
                solve: fmt(r.phasesMs.solveConstraints),
                ccd: fmt(r.phasesMs.ccd),
                other: fmt(r.phasesMs.other),
                contacts: r.contacts,
                'max pen': fmt(r.maxPenetration),
                KE: Math.round(r.kineticEnergy),
                escaped: r.escaped,
            };

            if (base) {
                row.speedup = `${fmt(base.medianMs / r.medianMs)}x`;
                row['base pen'] = fmt(base.maxPenetration);
                row['base KE'] = Math.round(base.kineticEnergy);
                row['base esc'] = base.escaped;
            }

            return row;
        });

        console.table(table);
    }
}

function main() {
    const options = parseArgs();

    if (options.child) {
        const [scene, n] = options.child.split(':');
        process.stdout.write(JSON.stringify(runChild(scene, Number(n), options.warmup, options.steps)));
        return;
    }

    const results: Result[] = [];

    for (const scene of options.scenes) {
        if (!SCENES[scene]) throw new Error(`Unknown scene ${scene}, available: ${Object.keys(SCENES).join(', ')}`);

        for (const n of options.sizes) {
            process.stdout.write(`running ${scene} n=${n}... `);

            const output = execFileSync(
                process.execPath,
                [
                    __filename,
                    '--child',
                    `${scene}:${n}`,
                    '--warmup',
                    String(options.warmup),
                    '--steps',
                    String(options.steps),
                ],
                { encoding: 'utf8', maxBuffer: 1 << 24 },
            );

            const result = JSON.parse(output) as Result;
            results.push(result);
            console.log(`${fmt(result.medianMs)} ms/step`);

            if (result.medianMs > MAX_STEP_MS) {
                console.log(`  skipping larger sizes of ${scene} (step > ${MAX_STEP_MS} ms)`);
                break;
            }
        }
    }

    const baseline = options.compare ? (JSON.parse(fs.readFileSync(options.compare, 'utf8')) as ResultFile) : undefined;

    printResults(results, baseline);

    const outDir = path.resolve(__dirname, '../../../benchmark/results');
    fs.mkdirSync(outDir, { recursive: true });

    const file: ResultFile = { label: options.label, date: new Date().toISOString(), node: process.version, results };
    const outFile = path.join(outDir, `${options.label}.json`);
    fs.writeFileSync(outFile, JSON.stringify(file, null, 2));
    console.log(`\nResults written to ${path.relative(process.cwd(), outFile)}`);
}

main();
