import { describe, expect, test } from '@jest/globals';

import { BodiesFactory } from '../../src';
import {
    CollisionCategory,
    CollisionFilter,
    DEFAULT_COLLISION_FILTER,
    canCollide,
} from '../../src/collision/CollisionFilter';

describe('CollisionFilter', () => {
    test('BodiesFactory.circle uses the default collision filter when none is provided', () => {
        const body = BodiesFactory.circle({
            radius: 30,
            x: 0,
            y: 0,
            mass: 1,
        });

        expect(body.collisionFilter).toEqual(DEFAULT_COLLISION_FILTER);
    });

    test('Filters with DEFAULT settings collide with each other', () => {
        const a: CollisionFilter = {
            category: CollisionCategory.DEFAULT,
            mask: CollisionCategory.ALL,
        };

        const b: CollisionFilter = {
            category: CollisionCategory.DEFAULT,
            mask: CollisionCategory.ALL,
        };

        expect(canCollide(a, b)).toBe(true);
        expect(canCollide(b, a)).toBe(true);
    });

    test('A NONE mask blocks collisions even when categories would otherwise match', () => {
        const a: CollisionFilter = {
            category: CollisionCategory.DEFAULT,
            mask: CollisionCategory.NONE,
        };

        const b: CollisionFilter = {
            category: CollisionCategory.PROJECTILE,
            mask: CollisionCategory.ALL,
        };

        expect(canCollide(a, b)).toBe(false);
        expect(canCollide(b, a)).toBe(false);
    });

    test('Both filters must allow each other for a collision to happen', () => {
        const a: CollisionFilter = {
            category: CollisionCategory.DEFAULT,
            mask: CollisionCategory.PROJECTILE,
        };

        const b: CollisionFilter = {
            category: CollisionCategory.PROJECTILE,
            mask: CollisionCategory.PARTICLE,
        };

        expect(canCollide(a, b)).toBe(false);
        expect(canCollide(b, a)).toBe(false);
    });

    test('A filter can allow exactly one category', () => {
        const a: CollisionFilter = {
            category: CollisionCategory.DEFAULT,
            mask: CollisionCategory.PROJECTILE,
        };

        const projectile: CollisionFilter = {
            category: CollisionCategory.PROJECTILE,
            mask: CollisionCategory.DEFAULT,
        };

        const particle: CollisionFilter = {
            category: CollisionCategory.PARTICLE,
            mask: CollisionCategory.DEFAULT,
        };

        expect(canCollide(a, projectile)).toBe(true);
        expect(canCollide(projectile, a)).toBe(true);
        expect(canCollide(a, particle)).toBe(false);
        expect(canCollide(particle, a)).toBe(false);
    });

    test('A mask can allow a union of categories', () => {
        const a: CollisionFilter = {
            category: CollisionCategory.SENSOR,
            mask: CollisionCategory.DEFAULT | CollisionCategory.PROJECTILE,
        };

        const defaultBody: CollisionFilter = {
            category: CollisionCategory.DEFAULT,
            mask: CollisionCategory.SENSOR,
        };

        const projectile: CollisionFilter = {
            category: CollisionCategory.PROJECTILE,
            mask: CollisionCategory.SENSOR,
        };

        const particle: CollisionFilter = {
            category: CollisionCategory.PARTICLE,
            mask: CollisionCategory.SENSOR,
        };

        expect(canCollide(a, defaultBody)).toBe(true);
        expect(canCollide(defaultBody, a)).toBe(true);
        expect(canCollide(a, projectile)).toBe(true);
        expect(canCollide(projectile, a)).toBe(true);
        expect(canCollide(a, particle)).toBe(false);
        expect(canCollide(particle, a)).toBe(false);
    });

    test('Removing DEFAULT from ALL blocks DEFAULT but still allows other categories', () => {
        const particle: CollisionFilter = {
            category: CollisionCategory.PARTICLE,
            mask: CollisionCategory.ALL & ~CollisionCategory.DEFAULT,
        };

        const defaultBody: CollisionFilter = {
            category: CollisionCategory.DEFAULT,
            mask: CollisionCategory.ALL,
        };

        const sensor: CollisionFilter = {
            category: CollisionCategory.SENSOR,
            mask: CollisionCategory.ALL,
        };

        expect(canCollide(particle, defaultBody)).toBe(false);
        expect(canCollide(defaultBody, particle)).toBe(false);
        expect(canCollide(particle, sensor)).toBe(true);
        expect(canCollide(sensor, particle)).toBe(true);
    });

    test('A body does not collide with its own category when its mask excludes it', () => {
        const a: CollisionFilter = {
            category: CollisionCategory.PARTICLE,
            mask: CollisionCategory.ALL & ~CollisionCategory.PARTICLE,
        };

        const b: CollisionFilter = {
            category: CollisionCategory.PARTICLE,
            mask: CollisionCategory.PARTICLE,
        };

        expect(canCollide(a, b)).toBe(false);
        expect(canCollide(b, a)).toBe(false);
    });

    test('A NONE category never matches another body mask', () => {
        const a: CollisionFilter = {
            category: CollisionCategory.NONE,
            mask: CollisionCategory.ALL,
        };

        const b: CollisionFilter = {
            category: CollisionCategory.DEFAULT,
            mask: CollisionCategory.ALL,
        };

        expect(canCollide(a, b)).toBe(false);
        expect(canCollide(b, a)).toBe(false);
    });
});
