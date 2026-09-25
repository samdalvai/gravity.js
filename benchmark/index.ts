import './benchmarks';
import { benchmarks } from './registry';

const WARMUP_MS = 500;
const SAMPLE_MS = 1_000;
const SAMPLES = 5;

if (benchmarks.length === 0) {
    console.log('No benchmarks registered.');
}

const results: {
    name: string;
    averageMs: number;
    medianMs: number;
    p95Ms: number;
    iterations: number;
    totalMs: number;
}[] = [];

console.log('Nr. of benchmarks: ', benchmarks.length);

console.log('Warming up benchmarks...');
{
    const warmupStart = performance.now();

    while (performance.now() - warmupStart < WARMUP_MS) {
        for (const { run } of benchmarks) {
            run();
        }
    }
}

console.log('Running measurements...');

const totals = benchmarks.map(({ name }) => ({
    name,
    totalMs: 0,
    totalIterations: 0,
    sampleAverages: [] as number[],
}));

const lastValues: unknown[] = new Array(benchmarks.length);

for (let sample = 0; sample < SAMPLES; sample++) {
    const sampleStart = performance.now();
    const sampleTotals = benchmarks.map(() => ({ totalMs: 0, iterations: 0 }));

    while (performance.now() - sampleStart < SAMPLE_MS) {
        for (let i = 0; i < benchmarks.length; i++) {
            const { run } = benchmarks[i];

            const start = performance.now();
            lastValues[i] = run();
            const end = performance.now();

            totals[i].totalMs += end - start;
            totals[i].totalIterations++;
            sampleTotals[i].totalMs += end - start;
            sampleTotals[i].iterations++;
        }
    }

    for (let i = 0; i < sampleTotals.length; i++) {
        const sampleTotal = sampleTotals[i];
        if (sampleTotal.iterations > 0) {
            totals[i].sampleAverages.push(sampleTotal.totalMs / sampleTotal.iterations);
        }
    }
}

void lastValues;

for (const total of totals) {
    const samples = [...total.sampleAverages].sort((a, b) => a - b);
    const percentile = (percent: number) => samples[Math.min(samples.length - 1, Math.floor(percent * samples.length))];

    results.push({
        name: total.name,
        averageMs: total.totalMs / total.totalIterations,
        medianMs: percentile(0.5),
        p95Ms: percentile(0.95),
        iterations: total.totalIterations,
        totalMs: total.totalMs,
    });
}

results.sort((a, b) => a.averageMs - b.averageMs);

if (results.length > 0) {
    const fastest = results[0];

    console.log('');
    console.log('Benchmark results');
    console.log('');

    for (const result of results) {
        const speed = result === fastest ? 'fastest' : `${(result.averageMs / fastest.averageMs).toFixed(2)}x slower`;

        console.log(
            `${result.name}: ${(result.averageMs * 1000).toFixed(3)} us/run (${result.iterations.toLocaleString()} runs, ${result.totalMs.toFixed(2)} ms) ${speed}`,
        );
        console.log(
            `  median ${(result.medianMs * 1000).toFixed(3)} us/run, p95 ${(result.p95Ms * 1000).toFixed(3)} us/run`,
        );
    }
}
