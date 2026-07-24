import { describe, expect, test } from '@jest/globals';

import { CollisionCategory } from 'gravity.js';

describe('public package API', () => {
    test('exports collision categories at runtime', () => {
        expect(CollisionCategory).toBeDefined();
        expect(CollisionCategory.NONE).toBe(0);
        expect(CollisionCategory.DEFAULT).toBe(1);
        expect(CollisionCategory.LAYER3).toBe(1 << 6);
        expect(CollisionCategory.ALL).toBe(0xffffffff);
    });
});
