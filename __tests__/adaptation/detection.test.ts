import { Box } from '../../src/new/box';
import { Circle } from '../../src/new/circle';
import { support } from '../../src/new/detection';
import { support_adapted } from '../../src/new/detection_adapted';
import { Vector2 } from '../../src/new/math/vector2';
import { RigidBody } from '../../src/new/rigidbody';
import Body from '../../src/physics/Body';
import { BoxShape, PolygonShape } from '../../src/physics/Shape';

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
        // OLD
        const b1 = new Box(50);
        b1.position = new Vector2(100, 100);
        b1.rotation = 1;

        const dir = new Vector2(1, 0);
        const result1 = support(b1, dir);
        console.log(result1);

        const c1 = new Circle(50);
        c1.position = new Vector2(100, 100);

        const result2 = support(c1, dir);
        console.log(result2);

        // NEW
    });
});
