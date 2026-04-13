type Benchmark = {
    name: string;
    run: () => unknown;
};

const benchmarks: Benchmark[] = [];

export function benchmark(name: string, run: () => unknown) {
    benchmarks.push({ name, run });
}

import './benchmarks';

const WARMUP_MS = 250;
const SAMPLE_MS = 500;
const SAMPLES = 5;

if (benchmarks.length === 0) {
    console.log('No benchmarks registered.');
}

const results: {
    name: string;
    averageMs: number;
    iterations: number;
    totalMs: number;
}[] = [];

for (const { name, run } of benchmarks) {
    let value: unknown;

    const warmupStart = performance.now();
    while (performance.now() - warmupStart < WARMUP_MS) {
        value = run();
    }

    let totalMs = 0;
    let totalIterations = 0;

    for (let sample = 0; sample < SAMPLES; sample++) {
        let iterations = 0;
        const start = performance.now();

        while (performance.now() - start < SAMPLE_MS) {
            value = run();
            iterations++;
        }

        totalMs += performance.now() - start;
        totalIterations += iterations;
    }

    void value;

    results.push({
        name,
        averageMs: totalMs / totalIterations,
        iterations: totalIterations,
        totalMs,
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
            `${result.name}: ${(result.averageMs * 1000).toFixed(3)} us/run (${result.iterations.toLocaleString()} runs, ${result.totalMs.toFixed(2)} ms) ${speed}`
        );
    }
}
