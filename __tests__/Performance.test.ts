import * as Collision from '../src/collision/Collision';
import { RigidBody } from '../src/core/RigidBody';
import { CircleShape } from '../src/shapes/CircleShape';

describe('Performance', () => {
    test('', () => {
        console.time('time');
        const a = new RigidBody(new CircleShape(30), 100, 120, 5);
        const b = new RigidBody(new CircleShape(30), 100, 100, 5);

        for (let i = 0; i < 100000; i++) {
            Collision.detectCollision(a, b);
        }

        console.timeEnd('time');
    });
});
