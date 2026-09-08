import { describe, expect, test } from '@jest/globals';
import { RigidBody } from '../../src/core/RigidBody';
import { CircleShape } from '../../src/shapes/CircleShape';
import * as Utils from '../../src/utils/Utils';

describe('Utils', () => {
    test('clamp() value below low returns low', () => {
        expect(Utils.clamp(0, 5, 10)).toBe(5);
        expect(Utils.clamp(-100, 0, 50)).toBe(0);
    });

    test('clamp() value above high returns high', () => {
        expect(Utils.clamp(20, 5, 10)).toBe(10);
        expect(Utils.clamp(999, -5, 5)).toBe(5);
    });

    test('clamp() value inside range returns unchanged', () => {
        expect(Utils.clamp(7, 5, 10)).toBe(7);
        expect(Utils.clamp(0, -5, 5)).toBe(0);
        expect(Utils.clamp(-2, -5, 5)).toBe(-2);
    });

    test('clamp() value equal to low returns low', () => {
        expect(Utils.clamp(5, 5, 10)).toBe(5);
    });

    test('clamp() value equal to high returns high', () => {
        expect(Utils.clamp(10, 5, 10)).toBe(10);
    });

    test('clamp() low equal to high returns that value', () => {
        expect(Utils.clamp(0, 3, 3)).toBe(3);
        expect(Utils.clamp(100, 3, 3)).toBe(3);
    });

    test('clamp() handles reversed bounds (low > high) by behaving like Math.min/max do naturally', () => {
        expect(Utils.clamp(5, 10, 3)).toBe(10);
        expect(Utils.clamp(100, 10, 3)).toBe(10);
        expect(Utils.clamp(-50, 10, 3)).toBe(10);
    });

    test('body pair key is independent of ordering', () => {
        const a = new RigidBody(new CircleShape(10), 100, 100, 10);
        const b = new RigidBody(new CircleShape(10), 100, 100, 10);

        expect(Utils.pairKey(a, b)).toBe(Utils.pairKey(b, a));
    });

    test('body pair keys preserve full IDs across integer boundaries', () => {
        const a = new RigidBody(new CircleShape(1), 0, 0, 1);
        const b = new RigidBody(new CircleShape(1), 0, 0, 1);
        const ids = [0, 1, 32768, 65535, 65536, 65537, 2 ** 32, 2 ** 32 + 1, Number.MAX_SAFE_INTEGER];
        const keys = new Set<Utils.PairKey>();

        for (let i = 0; i < ids.length; i++) {
            for (let j = i + 1; j < ids.length; j++) {
                Object.defineProperty(a, 'id', { value: ids[i] });
                Object.defineProperty(b, 'id', { value: ids[j] });
                const key = Utils.pairKey(a, b);

                expect(Utils.pairKey(b, a)).toBe(key);
                expect(keys.has(key)).toBe(false);
                keys.add(key);
            }
        }
    });
});
