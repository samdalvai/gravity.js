import { describe, expect, test } from '@jest/globals';

import * as Collision from '../../src/collision/NarrowPhase';
import { SETTINGS } from '../../src/core/Constants';
import { RigidBody } from '../../src/core/RigidBody';
import { Vec2 } from '../../src/math/Vec2';
import { BoxShape } from '../../src/shapes/BoxShape';
import { CapsuleShape } from '../../src/shapes/CapsuleShape';
import { CircleShape } from '../../src/shapes/CircleShape';
import { PolygonShape } from '../../src/shapes/PolygonShape';

describe('Collision', () => {
    test('detectCollision() detects collision between fully overlapped circles', () => {
        const a = new RigidBody(new CircleShape(30), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 0, 0, 1.0);

        const result = Collision.detectCollision(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(1);
        expect(result.penetrationDepth).toBe(60);
    });

    test('detectCollision() detects collision between half overlapped circles', () => {
        const a = new RigidBody(new CircleShape(30), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 30, 0, 1.0);

        const result = Collision.detectCollision(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(1);
        expect(result.penetrationDepth).toBe(30);
    });

    test('detectCollision() detects collision between circles that overlap by a quarter', () => {
        const a = new RigidBody(new CircleShape(30), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 45, 0, 1.0);

        const result = Collision.detectCollision(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(1);
        expect(result.penetrationDepth).toBe(15);
    });

    test('detectCollision() keeps circle contacts within contact slop', () => {
        const a = new RigidBody(new CircleShape(30), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 60 + SETTINGS.contactSlop * 0.5, 0, 1.0);

        const result = Collision.detectCollision(a, b);

        expect(result).not.toBeNull();
        expect(result?.contactPoints).toHaveLength(1);
        expect(result?.penetrationDepth).toBe(0);
    });

    test('detectCollision() detects collision between fully overlapped boxes', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);

        const result = Collision.detectCollision(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(2);
        expect(result.penetrationDepth).toBe(60);
    });

    test('detectCollision() detects collision between half overlapped boxes', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new BoxShape(60, 60), 30, 0, 1.0);

        const result = Collision.detectCollision(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(2);
        expect(result.penetrationDepth).toBe(30);
    });

    test('detectCollision() detects collision between boxes that overlap by a quarter', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new BoxShape(60, 60), 30, 30, 1.0);

        const result = Collision.detectCollision(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(2);
        expect(result.penetrationDepth).toBe(30);
    });

    test('detectCollision() should not detect collision for not overlapped boxes', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new BoxShape(60, 60), 200, 200, 1.0);

        const result = Collision.detectCollision(a, b);

        expect(result).toBeNull();
    });

    test('detectCollision() should not detect collision for not overlapped and not touching boxes', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new BoxShape(60, 60), 61, 60, 1.0);

        const result = Collision.detectCollision(a, b);

        expect(result).toBeNull();
    });

    test('detectCollision() detects collision between fully overlapped box and circle', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 0, 0, 1.0);

        const result = Collision.detectCollision(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(1);
        expect(result.penetrationDepth).toBe(60);
    });

    test('detectCollision() detects collision between half overlapped box and circle', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 30, 0, 1.0);

        const result = Collision.detectCollision(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(1);
        expect(result.penetrationDepth).toBe(30);
    });

    test('detectCollision() detects collision between box and circle that overlap by a quarter', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 30, 30, 1.0);

        const result = Collision.detectCollision(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(1);
        expect(result.penetrationDepth).toBe(30);
    });

    test('detectCollision() should not detect collision for not overlapped box and circle', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 200, 200, 1.0);

        const result = Collision.detectCollision(a, b)!;

        expect(result).toBeNull();
    });

    test('detectCollision() should not detect collision for not overlapped and touching box and circle', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 60, 60, 1.0);

        const result = Collision.detectCollision(a, b)!;

        expect(result).toBeNull();
    });

    test('detectCollision() keeps box and circle contacts within contact slop', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 60 + SETTINGS.contactSlop * 0.5, 0, 1.0);

        const result = Collision.detectCollision(a, b);

        expect(result).not.toBeNull();
        expect(result?.contactPoints).toHaveLength(1);
        expect(result?.penetrationDepth).toBe(0);
    });

    test('detectCollision() detects collision between fully overlapped triangles', () => {
        const triangleVertices = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const a = new RigidBody(new PolygonShape(triangleVertices), 0, 0, 1.0);
        const b = new RigidBody(new PolygonShape(triangleVertices), 0, 0, 1.0);

        const result = Collision.detectCollision(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(2);
        expect(result.penetrationDepth).toBeCloseTo(53.665);
    });

    test('detectCollision() detects collision between half overlapped triangles', () => {
        const triangleVerticesA = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const triangleVerticesB = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const a = new RigidBody(new PolygonShape(triangleVerticesA), 0, 0, 1.0);
        const b = new RigidBody(new PolygonShape(triangleVerticesB), 0, 15, 1.0);

        const result = Collision.detectCollision(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(1);
        expect(result.penetrationDepth).toBe(45);
    });

    test('detectCollision() detects collision between triangles that overlap by a quarter', () => {
        const triangleVerticesA = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const triangleVerticesB = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const a = new RigidBody(new PolygonShape(triangleVerticesA), 0, 0, 1.0);
        const b = new RigidBody(new PolygonShape(triangleVerticesB), 0, 30, 1.0);

        const result = Collision.detectCollision(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(1);
        expect(result.penetrationDepth).toBe(30);
    });

    test('detectCollision() should not detect collision for not overlapped triangles', () => {
        const triangleVerticesA = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const triangleVerticesB = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const a = new RigidBody(new PolygonShape(triangleVerticesA), 0, 0, 1.0);
        const b = new RigidBody(new PolygonShape(triangleVerticesB), 200, 0, 1.0);

        const result = Collision.detectCollision(a, b);

        expect(result).toBeNull();
    });

    test('detectCollision() should not detect collision for not overlapped and non touching triangles', () => {
        const triangleVerticesA = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const triangleVerticesB = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const a = new RigidBody(new PolygonShape(triangleVerticesA), 0, 0, 1.0);
        const b = new RigidBody(new PolygonShape(triangleVerticesB), 61, 0, 1.0);

        const result = Collision.detectCollision(a, b);

        expect(result).toBeNull();
    });

    test('detectCollision() detects collision for clockwise-wound convex polygons', () => {
        const pentagonVertices = [
            new Vec2(0, 46),
            new Vec2(42, 14),
            new Vec2(26, -38),
            new Vec2(-26, -38),
            new Vec2(-42, 14),
        ];
        const a = new RigidBody(new PolygonShape(pentagonVertices), 0, 0, 1.0);
        const b = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);

        const result = Collision.detectCollision(a, b);

        expect(result).not.toBeNull();
        expect(result?.contactPoints.length).toBeGreaterThan(0);
        expect(result?.penetrationDepth).toBeGreaterThan(0);
    });

    test('detectCollision() detects collision for counter-clockwise-wound convex polygons', () => {
        const pentagonVertices = [
            new Vec2(-42, 14),
            new Vec2(-26, -38),
            new Vec2(26, -38),
            new Vec2(42, 14),
            new Vec2(0, 46),
        ];
        const a = new RigidBody(new PolygonShape(pentagonVertices), 0, 0, 1.0);
        const b = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);

        const result = Collision.detectCollision(a, b);

        expect(result).not.toBeNull();
        expect(result?.contactPoints.length).toBeGreaterThan(0);
        expect(result?.penetrationDepth).toBeGreaterThan(0);
    });

    test('detectCollision() keeps box and capsule contacts within contact slop', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new CapsuleShape(40, 10), 40 + SETTINGS.contactSlop * 0.5, 0, 1.0);

        const result = Collision.detectCollision(a, b);

        expect(result).not.toBeNull();
        expect(result?.contactPoints.length).toBeGreaterThan(0);
        expect(result?.penetrationDepth).toBe(0);
    });

    test('detectCollision() keeps capsule and circle contacts within contact slop', () => {
        const a = new RigidBody(new CapsuleShape(40, 10), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(10), 20 + SETTINGS.contactSlop * 0.5, 0, 1.0);

        const result = Collision.detectCollision(a, b);

        expect(result).not.toBeNull();
        expect(result?.contactPoints).toHaveLength(1);
        expect(result?.penetrationDepth).toBe(0);
    });

    test('detectCollision() keeps capsule contacts within contact slop', () => {
        const a = new RigidBody(new CapsuleShape(40, 10), 0, 0, 1.0);
        const b = new RigidBody(new CapsuleShape(40, 10), 20 + SETTINGS.contactSlop * 0.5, 0, 1.0);

        const result = Collision.detectCollision(a, b);

        expect(result).not.toBeNull();
        expect(result?.contactPoints.length).toBeGreaterThan(0);
        expect(result?.penetrationDepth).toBe(0);
    });
});
