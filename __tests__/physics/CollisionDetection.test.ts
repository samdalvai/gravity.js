import Vec2 from '../../src/math/Vec2';
import * as Collision from '../../src/physics/Collision';
import RigidBody from '../../src/physics/RigidBody';
import { BoxShape } from '../../src/shapes/BoxShape';
import { CircleShape } from '../../src/shapes/CircleShape';
import { PolygonShape } from '../../src/shapes/PolygonShape';

describe('Collision', () => {
    test('detectCollisionCircleCircle() detects collision between fully overlapped circles', () => {
        const a = new RigidBody(new CircleShape(30), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 0, 0, 1.0);

        const result = Collision.detectCollisionCircleCircle(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(1);
        expect(result.penetrationDepth).toBe(60);
    });

    test('detectCollisionCircleCircle() detects collision between half overlapped circles', () => {
        const a = new RigidBody(new CircleShape(30), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 30, 0, 1.0);

        const result = Collision.detectCollisionCircleCircle(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(1);
        expect(result.penetrationDepth).toBe(30);
    });

    test('detectCollisionCircleCircle() detects collision between circles that overlap by a quarter', () => {
        const a = new RigidBody(new CircleShape(30), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 45, 0, 1.0);

        const result = Collision.detectCollisionCircleCircle(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(1);
        expect(result.penetrationDepth).toBe(15);
    });

    test('detectCollisionPolygonPolygon() detects collision between fully overlapped boxes', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);

        const result = Collision.detectCollisionPolygonPolygon(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(2);
        expect(result.penetrationDepth).toBe(60);
    });

    test('detectCollisionPolygonPolygon() detects collision between half overlapped boxes', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new BoxShape(60, 60), 30, 0, 1.0);

        const result = Collision.detectCollisionPolygonPolygon(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(2);
        expect(result.penetrationDepth).toBe(30);
    });

    test('detectCollisionPolygonPolygon() detects collision between boxes that overlap by a quarter', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new BoxShape(60, 60), 30, 30, 1.0);

        const result = Collision.detectCollisionPolygonPolygon(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(2);
        expect(result.penetrationDepth).toBe(30);
    });

    test('detectCollisionPolygonPolygon() should not detect collision for not overlapped boxes', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new BoxShape(60, 60), 200, 200, 1.0);

        const result = Collision.detectCollisionPolygonPolygon(a, b);

        expect(result).toBeNull();
    });

    test('detectCollisionPolygonPolygon() should not detect collision for not overlapped and touching boxes', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new BoxShape(60, 60), 60, 60, 1.0);

        const result = Collision.detectCollisionPolygonPolygon(a, b);

        expect(result).toBeNull();
    });

    test('detectCollisionPolygonPolygon() detects collision between fully overlapped box and circle', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 0, 0, 1.0);

        const result = Collision.detectCollisionPolygonCircle(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(1);
        expect(result.penetrationDepth).toBe(60);
    });

    test('detectCollisionPolygonPolygon() detects collision between half overlapped box and circle', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 30, 0, 1.0);

        const result = Collision.detectCollisionPolygonCircle(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(1);
        expect(result.penetrationDepth).toBe(30);
    });

    test('detectCollisionPolygonPolygon() detects collision between box and circle that overlap by a quarter', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 30, 30, 1.0);

        const result = Collision.detectCollisionPolygonCircle(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(1);
        expect(result.penetrationDepth).toBe(30);
    });

    test('detectCollisionPolygonPolygon() should not detect collision for not overlapped box and circle', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 200, 200, 1.0);

        const result = Collision.detectCollisionPolygonCircle(a, b)!;

        expect(result).toBeNull();
    });

    test('detectCollisionPolygonPolygon() should not detect collision for not overlapped and touching box and circle', () => {
        const a = new RigidBody(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new RigidBody(new CircleShape(30), 60, 60, 1.0);

        const result = Collision.detectCollisionPolygonCircle(a, b)!;

        expect(result).toBeNull();
    });

    test('detectCollisionPolygonPolygon() detects collision between fully overlapped triangles', () => {
        const triangleVertices = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const a = new RigidBody(new PolygonShape(triangleVertices), 0, 0, 1.0);
        const b = new RigidBody(new PolygonShape(triangleVertices), 0, 0, 1.0);

        const result = Collision.detectCollisionPolygonPolygon(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(2);
        expect(result.penetrationDepth).toBeCloseTo(53.665);
    });

    test('detectCollisionPolygonPolygon() detects collision between half overlapped triangles', () => {
        const triangleVerticesA = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const triangleVerticesB = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const a = new RigidBody(new PolygonShape(triangleVerticesA), 0, 0, 1.0);
        const b = new RigidBody(new PolygonShape(triangleVerticesB), 0, 15, 1.0);

        const result = Collision.detectCollisionPolygonPolygon(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(1);
        expect(result.penetrationDepth).toBe(45);
    });

    test('detectCollisionPolygonPolygon() detects collision between triangles that overlap by a quarter', () => {
        const triangleVerticesA = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const triangleVerticesB = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const a = new RigidBody(new PolygonShape(triangleVerticesA), 0, 0, 1.0);
        const b = new RigidBody(new PolygonShape(triangleVerticesB), 0, 30, 1.0);

        const result = Collision.detectCollisionPolygonPolygon(a, b)!;

        expect(result).not.toBeNull();
        expect(result.contactPoints).toHaveLength(1);
        expect(result.penetrationDepth).toBe(30);
    });

    test('detectCollisionPolygonPolygon() should not detect collision for not overlapped triangles', () => {
        const triangleVerticesA = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const triangleVerticesB = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const a = new RigidBody(new PolygonShape(triangleVerticesA), 0, 0, 1.0);
        const b = new RigidBody(new PolygonShape(triangleVerticesB), 200, 0, 1.0);

        const result = Collision.detectCollisionPolygonPolygon(a, b);

        expect(result).toBeNull();
    });

    test('detectCollisionPolygonPolygon() should not detect collision for not overlapped and touching triangles', () => {
        const triangleVerticesA = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const triangleVerticesB = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const a = new RigidBody(new PolygonShape(triangleVerticesA), 0, 0, 1.0);
        const b = new RigidBody(new PolygonShape(triangleVerticesB), 60, 0, 1.0);

        const result = Collision.detectCollisionPolygonPolygon(a, b);

        expect(result).toBeNull();
    });
});
