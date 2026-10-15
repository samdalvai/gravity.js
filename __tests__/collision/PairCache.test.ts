import { describe, expect, test } from '@jest/globals';

import { PairCache } from '../../src/collision/PairCache';

describe('PairCache', () => {
    test('stores and finds values by id pair', () => {
        const cache = new PairCache<string>();

        cache.set(1, 2, 'a');
        cache.set(1, 3, 'b');
        cache.set(2, 3, 'c');

        expect(cache.get(1, 2)).toBe('a');
        expect(cache.get(1, 3)).toBe('b');
        expect(cache.get(2, 3)).toBe('c');
        expect(cache.get(3, 4)).toBeUndefined();
        expect(cache.count).toBe(3);
    });

    test('overwrites existing pairs', () => {
        const cache = new PairCache<string>();

        cache.set(1, 2, 'a');
        cache.set(1, 2, 'b');

        expect(cache.get(1, 2)).toBe('b');
        expect(cache.count).toBe(1);
    });

    test('supports ids beyond 16 bits without collisions', () => {
        const cache = new PairCache<number>();

        cache.set(0x10000, 0x10001, 1);
        cache.set(0, 0x10001, 2);
        cache.set(1, 0x10000, 3);

        expect(cache.get(0x10000, 0x10001)).toBe(1);
        expect(cache.get(0, 0x10001)).toBe(2);
        expect(cache.get(1, 0x10000)).toBe(3);
        expect(cache.get(0, 1)).toBeUndefined();
    });

    test('keeps every entry when growing', () => {
        const cache = new PairCache<number>();
        const count = 10_000;

        for (let i = 0; i < count; i++) cache.set(i, i + 7, i);

        expect(cache.count).toBe(count);
        for (let i = 0; i < count; i++) expect(cache.get(i, i + 7)).toBe(i);
    });

    test('clear removes all entries', () => {
        const cache = new PairCache<number>();

        for (let i = 0; i < 100; i++) cache.set(i, i + 1, i);
        cache.clear();

        expect(cache.count).toBe(0);
        for (let i = 0; i < 100; i++) expect(cache.get(i, i + 1)).toBeUndefined();

        cache.set(5, 6, 42);
        expect(cache.get(5, 6)).toBe(42);
    });

    test('remove keeps the other entries reachable', () => {
        const cache = new PairCache<number>();
        const count = 2_000;

        for (let i = 0; i < count; i++) cache.set(i, i + 1, i);

        // Remove every third pair, in an order that exercises probe chains
        for (let i = count - 1; i >= 0; i -= 3) expect(cache.remove(i, i + 1)).toBe(true);
        expect(cache.remove(0, 99_999)).toBe(false);

        for (let i = 0; i < count; i++) {
            const removed = (count - 1 - i) % 3 === 0;
            expect(cache.get(i, i + 1)).toBe(removed ? undefined : i);
        }

        expect(cache.count).toBe(count - Math.ceil(count / 3));

        // Removed pairs can be inserted again
        cache.set(count - 1, count, 7);
        expect(cache.get(count - 1, count)).toBe(7);
    });

    test('matches a Map under random inserts and removals', () => {
        const cache = new PairCache<number>();
        const reference = new Map<string, number>();
        let seed = 12345;
        const random = (max: number) => {
            seed = (Math.imul(seed, 1103515245) + 12345) >>> 0;
            return seed % max;
        };

        for (let step = 0; step < 20_000; step++) {
            const a = random(200);
            const b = a + 1 + random(50);
            const key = `${a}:${b}`;

            if (random(3) === 0) {
                expect(cache.remove(a, b)).toBe(reference.delete(key));
            } else {
                cache.set(a, b, step);
                reference.set(key, step);
            }
        }

        expect(cache.count).toBe(reference.size);
        for (const [key, value] of reference) {
            const [a, b] = key.split(':').map(Number);
            expect(cache.get(a, b)).toBe(value);
        }
    });
});
