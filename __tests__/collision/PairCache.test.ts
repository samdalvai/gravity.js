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
});
