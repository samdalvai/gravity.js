import Vec2 from '../../src/math/Vec2';
import { Box } from '../../src/new/box';
import { Circle } from '../../src/new/circle';
import { csoSupport, support } from '../../src/new/detection';
import { csoSupport_adapted, support_adapted } from '../../src/new/detection_adapted';
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

        const worldVerticesOld = [];
        for (const v of b1.vertices) {
            const worldV = b1.localToGlobal.mulVector2(v, 1);
            worldVerticesOld.push(worldV);
        }

        const b2 = new Body(new BoxShape(50, 50), 100, 100, 2.0);
        const shape = b2.shape as PolygonShape;

        const wordlVertexNew = [];
        for (const v of shape.localVertices) {
            const local = b2.localPointToWorld(v);
            wordlVertexNew.push(local);
        }

        for (let i = 0; i < worldVerticesOld.length; i++) {
            const vOld = worldVerticesOld[i];
            const vNew = wordlVertexNew[i];
            expect(vOld.x).toBe(vNew.x);
            expect(vOld.y).toBe(vNew.y);
        }
    });

    test('Support point', () => {
        // OLD
        const b1 = new Box(50);
        b1.position = new Vector2(100, 100);
        b1.rotation = 1;

        const dir1 = new Vector2(1, 0);
        const result1 = support(b1, dir1);

        const c1 = new Circle(25);
        c1.position = new Vector2(100, 100);

        const result2 = support(c1, dir1);

        // NEW
        const dir2 = new Vec2(1, 0);
        const b2 = new Body(new BoxShape(50, 50), 100, 100, 1);
        const result12 = support_adapted(b2, dir2);
        expect(result1.vertex.x).toEqual(result12.vertex.x);
        expect(result1.vertex.y).toEqual(result12.vertex.y);
        expect(result1.index).toBe(result12.index);

        const c2 = new Body(new CircleShape(25), 100, 100, 1);
        const result22 = support_adapted(c2, dir2);

        expect(result2.vertex.x).toEqual(result22.vertex.x);
        expect(result2.vertex.y).toEqual(result22.vertex.y);
        expect(result2.index).toBe(result22.index);
    });

    test('csoSupport', () => {
        const b1 = new Box(50);
        b1.position = new Vector2(100, 100);

        const b2 = new Box(50);
        b2.position = new Vector2(125, 100);

        const dir1 = new Vector2(1, 0);
        const cso1 = csoSupport(b1, b2, dir1);

        const b3 = new Body(new BoxShape(50, 50), 100, 100, 1);
        const b4 = new Body(new BoxShape(50, 50), 125, 100, 1);
        const dir2 = new Vec2(1, 0);
        const cso2 = csoSupport_adapted(b3, b4, dir2);

        expect(cso1.x).toBe(cso2.x);
        expect(cso1.y).toBe(cso2.y);
    });
});
