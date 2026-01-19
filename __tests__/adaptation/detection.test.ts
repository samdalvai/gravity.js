import Vec2 from '../../src/math/Vec2';
import { Box } from '../../src/new/box';
import { Circle } from '../../src/new/circle';
import { csoSupport, epa, findContactPoints, findFarthestEdge, gjk, support } from '../../src/new/detection';
import {
    csoSupport_adapted,
    epa_adapted,
    findContactPoints_adapted,
    findFarthestEdge_adapted,
    gjk_adapted,
    support_adapted,
} from '../../src/new/detection_adapted';
import { RigidBody } from '../../src/new/rigidbody';
import { Vector2 } from '../../src/new/vector2';
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

    test('gjk', () => {
        const b1 = new Box(50);
        b1.position = new Vector2(100, 100);
        const b2 = new Box(50);
        b2.position = new Vector2(125, 100);
        const result1 = gjk(b1, b2);

        const b3 = new Body(new BoxShape(50, 50), 100, 100, 1);
        const b4 = new Body(new BoxShape(50, 50), 125, 100, 1);
        const result2 = gjk_adapted(b3, b4);

        expect(result1.collide).toBe(result2.collide);

        const vertices1 = result1.simplex.vertices;
        const vertices2 = result2.simplex.vertices;
        for (let i = 0; i < vertices1.length; i++) {
            const vertex1 = vertices1[i];
            const vertex2 = vertices2[i];
            expect(vertex1.x).toBe(vertex2.x);
            expect(vertex1.y).toBe(vertex2.y);
        }
    });

    test('epa', () => {
        const b1 = new Box(50);
        b1.position = new Vector2(100, 100);
        const b2 = new Box(50);
        b2.position = new Vector2(125, 125);
        const result1 = gjk(b1, b2);
        const epa1 = epa(b1, b2, result1.simplex);

        const b3 = new Body(new BoxShape(50, 50), 100, 100, 1);
        const b4 = new Body(new BoxShape(50, 50), 125, 125, 1);
        const result2 = gjk_adapted(b3, b4);
        const epa2 = epa_adapted(b3, b4, result2.simplex);

        expect(epa1.penetrationDepth).toBe(epa2.penetrationDepth);
        expect(epa1.contactNormal.x).toBe(epa2.contactNormal.x);
        expect(epa1.contactNormal.y).toBe(epa2.contactNormal.y);
    });

    test('findFarthestEdge', () => {
        const b1 = new Box(50);
        b1.position = new Vector2(100, 100);

        const c1 = new Circle(25);
        c1.position = new Vector2(100, 100);
        const dir1 = new Vector2(1, 0);

        const edgeBox1 = findFarthestEdge(b1, dir1);
        const edgeCircle1 = findFarthestEdge(c1, dir1);

        const b2 = new Body(new BoxShape(50, 50), 100, 100, 1);
        const c2 = new Body(new CircleShape(25), 100, 100, 1);
        const dir2 = new Vec2(1, 0);

        const edgeBox2 = findFarthestEdge_adapted(b2, dir2);
        const edgeCircle2 = findFarthestEdge_adapted(c2, dir2);

        expect(edgeBox1.p1.x).toBe(edgeBox2.p1.x);
        expect(edgeBox1.p1.y).toBe(edgeBox2.p1.y);
        expect(edgeBox1.p2.x).toBe(edgeBox2.p2.x);
        expect(edgeBox1.p2.y).toBe(edgeBox2.p2.y);
        expect(edgeBox1.dir.x).toBe(edgeBox2.dir.x);
        expect(edgeBox1.dir.y).toBe(edgeBox2.dir.y);
        expect(edgeBox1.id1).toBe(edgeBox2.id1);
        expect(edgeBox1.id2).toBe(edgeBox2.id2);

        expect(edgeCircle1.p1.x).toBe(edgeCircle2.p1.x);
        expect(edgeCircle1.p1.y).toBe(edgeCircle2.p1.y);
        expect(edgeCircle1.p2.x).toBe(edgeCircle2.p2.x);
        expect(edgeCircle1.p2.y).toBe(edgeCircle2.p2.y);
        expect(edgeCircle1.dir.x).toBe(edgeCircle2.dir.x);
        expect(edgeCircle1.dir.y).toBe(edgeCircle2.dir.y);
        expect(edgeCircle1.id1).toBe(edgeCircle2.id1);
        expect(edgeCircle1.id2).toBe(edgeCircle2.id2);
    });

    test('findContactPoints', () => {
        const b1 = new Box(50);
        b1.position = new Vector2(100, 100);
        const b2 = new Box(50);
        b2.position = new Vector2(125, 125);
        const result1 = gjk(b1, b2);
        const epa1 = epa(b1, b2, result1.simplex);
        const contacts1 = findContactPoints(epa1.contactNormal, b1, b2);

        const b3 = new Body(new BoxShape(50, 50), 100, 100, 1);
        const b4 = new Body(new BoxShape(50, 50), 125, 125, 1);
        const result2 = gjk_adapted(b3, b4);
        const epa2 = epa_adapted(b3, b4, result2.simplex);
        const contacts2 = findContactPoints_adapted(epa2.contactNormal, b3, b4);

        expect(contacts1.length).toBe(contacts2.length);
        for (let i = 0; i < contacts1.length; i++) {
            const contact1 = contacts1[i];
            const contact2 = contacts2[i];
            expect(contact1.point.x).toBe(contact2.point.x);
            expect(contact1.point.y).toBe(contact2.point.y);
            expect(contact1.id).toBe(contact2.id);
        }
    });
});
