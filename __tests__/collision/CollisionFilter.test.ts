import { describe, expect, test } from '@jest/globals';

import { BodiesFactory } from '../../src';
import { CollisionCategory, canCollide } from '../../src/collision/CollisionFilter';

describe('CollisionFilter', () => {
    test('Rigid bodies with DEFAULT collision filter should collide with everything', () => {
        const a = BodiesFactory.circle({
            radius: 30,
            x: 0,
            y: 0,
            mass: 1,
        });

        const b = BodiesFactory.circle({
            radius: 30,
            x: 0,
            y: 0,
            mass: 1,
        });

        expect(canCollide(a.collisionFilter, b.collisionFilter)).toBe(true);
        expect(canCollide(b.collisionFilter, a.collisionFilter)).toBe(true);
    });

    test('Rigid bodies with collision filter set to NONE should not collide with anything', () => {
        const a = BodiesFactory.circle({
            radius: 30,
            x: 0,
            y: 0,
            mass: 1,
            collisionFilter: {
                category: CollisionCategory.DEFAULT,
                mask: CollisionCategory.NONE,
            },
        });

        const b = BodiesFactory.circle({
            radius: 30,
            x: 0,
            y: 0,
            mass: 1,
            collisionFilter: {
                category: CollisionCategory.DEFAULT,
                mask: CollisionCategory.NONE,
            },
        });

        expect(canCollide(a.collisionFilter, b.collisionFilter)).toBe(false);
        expect(canCollide(b.collisionFilter, a.collisionFilter)).toBe(false);
    });

    test('Rigid body with selective collision filter should not collide with DEFAULT category', () => {
        const a = BodiesFactory.circle({
            radius: 30,
            x: 0,
            y: 0,
            mass: 1,
            collisionFilter: {
                category: CollisionCategory.DEFAULT,
                mask: CollisionCategory.ALL,
            },
        });

        const b = BodiesFactory.circle({
            radius: 30,
            x: 0,
            y: 0,
            mass: 1,
            collisionFilter: {
                category: CollisionCategory.PARTICLE,
                mask: CollisionCategory.ALL & ~CollisionCategory.DEFAULT,
            },
        });

        expect(canCollide(a.collisionFilter, b.collisionFilter)).toBe(false);
        expect(canCollide(b.collisionFilter, a.collisionFilter)).toBe(false);
    });

    test('Rigid body with selective collision filter should collide with a his own category', () => {
        const a = BodiesFactory.circle({
            radius: 30,
            x: 0,
            y: 0,
            mass: 1,
            collisionFilter: {
                category: CollisionCategory.PARTICLE,
                mask: CollisionCategory.ALL & ~CollisionCategory.PARTICLE,
            },
        });

        const b = BodiesFactory.circle({
            radius: 30,
            x: 0,
            y: 0,
            mass: 1,
            collisionFilter: {
                category: CollisionCategory.PARTICLE,
                mask: CollisionCategory.PARTICLE,
            },
        });

        expect(canCollide(a.collisionFilter, b.collisionFilter)).toBe(false);
        expect(canCollide(b.collisionFilter, a.collisionFilter)).toBe(false);
    });
});
