import Vec2 from '../src/math/Vec2';
import Body from '../src/physics/Body';
import { CircleShape } from '../src/physics/Shape';

describe('Performance', () => {
    test('Performance test', () => {
        console.time('test');
        const a = new Body(new CircleShape(20), 100, 100, 5);

        const impulse = new Vec2(50, 50);

        for (let i = 0; i < 10000000; i++) {
            a.applyImpulseLinear(impulse);
        }

        console.timeEnd('test');
    });
});
