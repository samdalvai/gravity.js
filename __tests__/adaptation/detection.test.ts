import Vec2 from '../../src/math/Vec2';
import { Box } from '../../src/new/box';
import { Circle } from '../../src/new/circle';
import { support } from '../../src/new/detection';
import { support_adapted } from '../../src/new/detection_adapted';
import { Vector2 } from '../../src/new/math/vector2';
import { RigidBody } from '../../src/new/rigidbody';
import Body from '../../src/physics/Body';
import { BoxShape, CircleShape, PolygonShape } from '../../src/physics/Shape';

describe('Performance', () => {
    test('Test convertion from local to world space for vertices', () => {
        const b1 = new Box(50);
        b1.mass = 2.0;
        b1.position = new Vector2(100, 100);
        b1.restitution = 0.2;

        console.log(b1.vertices);
        const worldVertices = [];
        for (const v of b1.vertices) {
            const worldV = b1.localToGlobal.mulVector2(v, 1);
            worldVertices.push(worldV);
        }
        console.log(worldVertices);

        const b2 = new Body(new BoxShape(50, 50), 100, 100, 2.0);
        const shape = b2.shape as PolygonShape;

        console.log(shape.localVertices);
        console.log(shape.worldVertices);
    });

    test('Support point', () => {
        console.log('**** SUPPORT POINT ***');
        // OLD
        const b1 = new Box(50);
        b1.position = new Vector2(100, 100);
        b1.rotation = 1;

        const dir1 = new Vector2(1, 0);
        const result1 = support(b1, dir1);
        console.log(result1);

        const c1 = new Circle(25);
        c1.position = new Vector2(100, 100);

        const result2 = support(c1, dir1);
        console.log(result2);

        // NEW
        const dir2 = new Vec2(1, 0);
        const b2 = new Body(new BoxShape(50, 50), 100, 100, 1);
        const result12 = support_adapted(b2, dir2);
        console.log(result12);
        expect(result1.vertex.x).toEqual(result12.vertex.x);
        expect(result1.vertex.y).toEqual(result12.vertex.y);
        expect(result1.index).toBe(result12.index);

        const c2 = new Body(new CircleShape(25), 100, 100, 1);
        const result22 = support_adapted(c2, dir2);
        console.log(result22);

        expect(result2.vertex.x).toEqual(result22.vertex.x);
        expect(result2.vertex.y).toEqual(result22.vertex.y);
        expect(result2.index).toBe(result22.index);
    });
});
