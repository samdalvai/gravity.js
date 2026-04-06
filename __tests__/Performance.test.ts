import { describe, expect, test } from '@jest/globals';

describe('Performance test', () => {
    test('Implementation 1', () => {
        console.time('perf');

        // ...

        console.timeEnd('perf');
    });

    test('Implementation 2', () => {
        console.time('perf');

        // ...

        console.timeEnd('perf');
    });
});
