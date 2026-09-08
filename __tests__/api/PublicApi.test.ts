import { describe, expect, test } from '@jest/globals';

import { CollisionCategory, World } from 'gravity.js';
import type { WorldOptions } from 'gravity.js';

describe('public package API', () => {
    test('accepts world options through the package entry point', () => {
        const options: WorldOptions = { maxBodies: 20_000 };
        expect(new World(0, options).maxBodies).toBe(20_000);
    });

    test('exports collision categories at runtime', () => {
        expect(CollisionCategory).toBeDefined();
        expect(CollisionCategory.NONE).toBe(0);
        expect(CollisionCategory.DEFAULT).toBe(1);
        expect(CollisionCategory.LAYER3).toBe(1 << 6);
        expect(CollisionCategory.ALL).toBe(0xffffffff);
    });
});
