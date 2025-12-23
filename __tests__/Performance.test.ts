import Vec2 from '../src/math/Vec2';
import Body from '../src/physics/Body';
import { BoxShape, CircleShape } from '../src/physics/Shape';

describe('Performance', () => {
    test('Performance test', () => {
        console.time('test');
        const a = new Body(new BoxShape(20, 20), 100, 100, 5);

        const dt = 1 / 60;

        for (let i = 0; i < 1000000; i++) {
            a.shape.updateVertices(45, new Vec2(i, i));
        }

        console.timeEnd('test');
    });
});
