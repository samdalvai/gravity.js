import Body from '../../src/physics/body/Body';
import CollisionDetection from '../../src/physics/CollisionDetection';
import { BoxShape, CircleShape, PolygonShape } from '../../src/physics/body/Shape';
import Vec2 from '../../src/math/Vec2';

describe('CollisionDetection', () => {
    test('detectCollisionCircleCircle() detects collision between fully overlapped circles', () => {
        const a = new Body(new CircleShape(30), 0, 0, 1.0);
        const b = new Body(new CircleShape(30), 0, 0, 1.0);

        const collisionResult = CollisionDetection.detectCollisionCircleCircle(a, b);

        expect(collisionResult.isColliding).toBe(true);
        expect(collisionResult.contacts).not.toBeUndefined();
        expect(collisionResult.contacts).toHaveLength(1);
        expect(collisionResult.contacts![0].depth).toBe(0);
    });

    test('detectCollisionCircleCircle() detects collision between half overlapped circles', () => {
        const a = new Body(new CircleShape(30), 0, 0, 1.0);
        const b = new Body(new CircleShape(30), 30, 0, 1.0);

        const collisionResult = CollisionDetection.detectCollisionCircleCircle(a, b);

        expect(collisionResult.isColliding).toBe(true);
        expect(collisionResult.contacts).not.toBeUndefined();
        expect(collisionResult.contacts).toHaveLength(1);
        expect(collisionResult.contacts![0].depth).toBe(30);
    });

    test('detectCollisionCircleCircle() detects collision between circles that overlap by a quarter', () => {
        const a = new Body(new CircleShape(30), 0, 0, 1.0);
        const b = new Body(new CircleShape(30), 45, 0, 1.0);

        const collisionResult = CollisionDetection.detectCollisionCircleCircle(a, b);

        expect(collisionResult.isColliding).toBe(true);
        expect(collisionResult.contacts).not.toBeUndefined();
        expect(collisionResult.contacts).toHaveLength(1);
        expect(collisionResult.contacts![0].depth).toBe(15);
    });

    test('detectCollisionPolygonPolygon() detects collision between fully overlapped boxes', () => {
        const a = new Body(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new Body(new BoxShape(60, 60), 0, 0, 1.0);

        const collisionResult = CollisionDetection.detectCollisionPolygonPolygon(a, b);

        expect(collisionResult.isColliding).toBe(true);
        expect(collisionResult.contacts).not.toBeUndefined();
        expect(collisionResult.contacts).toHaveLength(2);
        expect(collisionResult.contacts![0].depth).toBe(60);
        expect(collisionResult.contacts![1].depth).toBe(60);
    });

    test('detectCollisionPolygonPolygon() detects collision between half overlapped boxes', () => {
        const a = new Body(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new Body(new BoxShape(60, 60), 30, 0, 1.0);

        const collisionResult = CollisionDetection.detectCollisionPolygonPolygon(a, b);

        expect(collisionResult.isColliding).toBe(true);
        expect(collisionResult.contacts).not.toBeUndefined();
        expect(collisionResult.contacts).toHaveLength(2);
        expect(collisionResult.contacts![0].depth).toBe(30);
        expect(collisionResult.contacts![1].depth).toBe(30);
    });

    test('detectCollisionPolygonPolygon() detects collision between boxes that overlap by a quarter', () => {
        const a = new Body(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new Body(new BoxShape(60, 60), 30, 30, 1.0);

        const collisionResult = CollisionDetection.detectCollisionPolygonPolygon(a, b);

        expect(collisionResult.isColliding).toBe(true);
        expect(collisionResult.contacts).not.toBeUndefined();
        expect(collisionResult.contacts).toHaveLength(2);
        expect(collisionResult.contacts![0].depth).toBe(30);
        expect(collisionResult.contacts![1].depth).toBe(30);
    });

    test('detectCollisionPolygonPolygon() should not detect collision for not overlapped boxes', () => {
        const a = new Body(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new Body(new BoxShape(60, 60), 200, 200, 1.0);

        const collisionResult = CollisionDetection.detectCollisionPolygonPolygon(a, b);

        expect(collisionResult.isColliding).toBe(false);
        expect(collisionResult.contacts).toBeUndefined();
    });

    test('detectCollisionPolygonPolygon() should not detect collision for not overlapped and touching boxes', () => {
        const a = new Body(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new Body(new BoxShape(60, 60), 60, 60, 1.0);

        const collisionResult = CollisionDetection.detectCollisionPolygonPolygon(a, b);

        expect(collisionResult.isColliding).toBe(false);
        expect(collisionResult.contacts).toBeUndefined();
    });

    test('detectCollisionPolygonPolygon() detects collision between fully overlapped box and circle', () => {
        const a = new Body(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new Body(new CircleShape(30), 0, 0, 1.0);

        const collisionResult = CollisionDetection.detectCollisionPolygonCircle(a, b);

        expect(collisionResult.isColliding).toBe(true);
        expect(collisionResult.contacts).not.toBeUndefined();
        expect(collisionResult.contacts).toHaveLength(1);
        expect(collisionResult.contacts![0].depth).toBe(60);
    });

    test('detectCollisionPolygonPolygon() detects collision between half overlapped box and circle', () => {
        const a = new Body(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new Body(new CircleShape(30), 30, 0, 1.0);

        const collisionResult = CollisionDetection.detectCollisionPolygonCircle(a, b);

        expect(collisionResult.isColliding).toBe(true);
        expect(collisionResult.contacts).not.toBeUndefined();
        expect(collisionResult.contacts).toHaveLength(1);
        expect(collisionResult.contacts![0].depth).toBe(30);
    });

    test('detectCollisionPolygonPolygon() detects collision between box and circle that overlap by a quarter', () => {
        const a = new Body(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new Body(new CircleShape(30), 30, 30, 1.0);

        const collisionResult = CollisionDetection.detectCollisionPolygonCircle(a, b);

        expect(collisionResult.isColliding).toBe(true);
        expect(collisionResult.contacts).not.toBeUndefined();
        expect(collisionResult.contacts).toHaveLength(1);
        expect(collisionResult.contacts![0].depth).toBe(30);
    });

    test('detectCollisionPolygonPolygon() should not detect collision for not overlapped box and circle', () => {
        const a = new Body(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new Body(new CircleShape(30), 200, 200, 1.0);

        const collisionResult = CollisionDetection.detectCollisionPolygonCircle(a, b);

        expect(collisionResult.isColliding).toBe(false);
        expect(collisionResult.contacts).toBeUndefined();
    });

    test('detectCollisionPolygonPolygon() should not detect collision for not overlapped and touching box and circle', () => {
        const a = new Body(new BoxShape(60, 60), 0, 0, 1.0);
        const b = new Body(new CircleShape(30), 60, 60, 1.0);

        const collisionResult = CollisionDetection.detectCollisionPolygonCircle(a, b);

        expect(collisionResult.isColliding).toBe(false);
        expect(collisionResult.contacts).toBeUndefined();
    });

    test('detectCollisionPolygonPolygon() detects collision between fully overlapped triangles', () => {
        const triangleVertices = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const a = new Body(new PolygonShape(triangleVertices), 0, 0, 1.0);
        const b = new Body(new PolygonShape(triangleVertices), 0, 0, 1.0);

        const collisionResult = CollisionDetection.detectCollisionPolygonPolygon(a, b);

        expect(collisionResult.isColliding).toBe(true);
        expect(collisionResult.contacts).not.toBeUndefined();
        expect(collisionResult.contacts).toHaveLength(2);
        expect(collisionResult.contacts![0].depth).toBe(0);
        expect(collisionResult.contacts![1].depth).toBeLessThanOrEqual(60);
    });

    test('detectCollisionPolygonPolygon() detects collision between half overlapped triangles', () => {
        const triangleVerticesA = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const triangleVerticesB = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const a = new Body(new PolygonShape(triangleVerticesA), 0, 0, 1.0);
        const b = new Body(new PolygonShape(triangleVerticesB), 0, 15, 1.0);

        const collisionResult = CollisionDetection.detectCollisionPolygonPolygon(a, b);

        expect(collisionResult.isColliding).toBe(true);
        expect(collisionResult.contacts).not.toBeUndefined();
        expect(collisionResult.contacts).toHaveLength(1);
        expect(collisionResult.contacts![0].depth).toBe(45);
    });

    test('detectCollisionPolygonPolygon() detects collision between triangles that overlap by a quarter', () => {
        const triangleVerticesA = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const triangleVerticesB = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const a = new Body(new PolygonShape(triangleVerticesA), 0, 0, 1.0);
        const b = new Body(new PolygonShape(triangleVerticesB), 0, 30, 1.0);

        const collisionResult = CollisionDetection.detectCollisionPolygonPolygon(a, b);

        expect(collisionResult.isColliding).toBe(true);
        expect(collisionResult.contacts).not.toBeUndefined();
        expect(collisionResult.contacts).toHaveLength(1);
        expect(collisionResult.contacts![0].depth).toBe(30);
    });

    test('detectCollisionPolygonPolygon() should not detect collision for not overlapped triangles', () => {
        const triangleVerticesA = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const triangleVerticesB = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const a = new Body(new PolygonShape(triangleVerticesA), 0, 0, 1.0);
        const b = new Body(new PolygonShape(triangleVerticesB), 200, 0, 1.0);

        const collisionResult = CollisionDetection.detectCollisionPolygonPolygon(a, b);

        expect(collisionResult.isColliding).toBe(false);
        expect(collisionResult.contacts).toBeUndefined();
    });

    test('detectCollisionPolygonPolygon() should not detect collision for not overlapped and touching triangles', () => {
        const triangleVerticesA = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const triangleVerticesB = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const a = new Body(new PolygonShape(triangleVerticesA), 0, 0, 1.0);
        const b = new Body(new PolygonShape(triangleVerticesB), 60, 0, 1.0);

        const collisionResult = CollisionDetection.detectCollisionPolygonPolygon(a, b);

        expect(collisionResult.isColliding).toBe(false);
        expect(collisionResult.contacts).toBeUndefined();
    });
});
