import { describe, expect, test } from '@jest/globals';

import { DynamicTree } from '../../src/collision/DynamicTree';

type Box = { minX: number; minY: number; maxX: number; maxY: number };

describe('DynamicTree', () => {
    let seed = 1;
    const random = () => {
        seed = (Math.imul(seed, 1103515245) + 12345) >>> 0;
        return seed / 4294967296;
    };

    function randomBox(): Box {
        const x = random() * 1000;
        const y = random() * 1000;
        const w = 1 + random() * 40;
        const h = 1 + random() * 40;
        return { minX: x, minY: y, maxX: x + w, maxY: y + h };
    }

    function overlaps(a: Box, b: Box) {
        return !(a.minX > b.maxX || a.maxX < b.minX || a.minY > b.maxY || a.maxY < b.minY);
    }

    test('queries match a brute force search through creations, moves and destructions', () => {
        const tree = new DynamicTree<number>();
        const boxes = new Map<number, Box>();
        const results: number[] = [];

        for (let step = 0; step < 3_000; step++) {
            const action = random();
            const ids = [...boxes.keys()];

            if (action < 0.5 || ids.length === 0) {
                const box = randomBox();
                const id = tree.createProxy(box.minX, box.minY, box.maxX, box.maxY, step);
                boxes.set(id, box);
            } else if (action < 0.8) {
                const id = ids[Math.floor(random() * ids.length)];
                const box = randomBox();
                tree.moveProxy(id, box.minX, box.minY, box.maxX, box.maxY);
                boxes.set(id, box);
            } else {
                const id = ids[Math.floor(random() * ids.length)];
                tree.destroyProxy(id);
                boxes.delete(id);
            }

            if (step % 100 === 0) {
                const height = tree.validate();
                // Balanced enough for logarithmic queries
                expect(height).toBeLessThanOrEqual(4 * Math.ceil(Math.log2(boxes.size + 2)));

                const query = randomBox();
                tree.query(query.minX, query.minY, query.maxX, query.maxY, results);

                const expected = [...boxes].filter(([, box]) => overlaps(box, query)).map(([id]) => id);
                expect([...results].sort((a, b) => a - b)).toEqual(expected.sort((a, b) => a - b));
            }
        }

        expect(tree.proxyCount).toBe(boxes.size);
    });

    test('clear empties the tree', () => {
        const tree = new DynamicTree<number>();
        const results: number[] = [];

        for (let i = 0; i < 100; i++) tree.createProxy(i, i, i + 1, i + 1, i);
        tree.clear();

        expect(tree.query(-1e9, -1e9, 1e9, 1e9, results)).toBe(0);
        expect(tree.proxyCount).toBe(0);
    });
});
