import * as Collision from '../src/collision/Collision';
import { RigidBody } from '../src/core/RigidBody';
import { CircleShape } from '../src/shapes/CircleShape';
import { BoxShape } from '../src/shapes/BoxShape';

describe('Performance circle vs circle', () => {
    test('collision circle vs circle', () => {
        console.time('collision');
        const a = new RigidBody(new CircleShape(30), 100, 120, 5);
        const b = new RigidBody(new CircleShape(30), 100, 100, 5);

        for (let i = 0; i < 100000; i++) {
            Collision.detectCollision(a, b);
        }

        console.timeEnd('collision');
    });

    test('resolution circle vs circle', () => {
        console.time('resolution');
        const dt = 1 / 60;
        const invDt = 1 / dt;
        const a = new RigidBody(new CircleShape(30), 100, 120, 5);
        const b = new RigidBody(new CircleShape(30), 100, 100, 5);
        const manifold = Collision.detectCollision(a, b)!;

        for (let i = 0; i < 100000; i++) {
            manifold.preSolve(invDt);

            for (let j = 0; j < 10; j++) {
                manifold.solve();
            }
        }

        console.timeEnd('resolution');
    });
});

// describe('Performance box vs box', () => {
//     test('collision box vs box', () => {
//         console.time('collision');
//         const a = new RigidBody(new BoxShape(60, 60), 100, 120, 5);
//         const b = new RigidBody(new BoxShape(60, 60), 100, 100, 5);

//         for (let i = 0; i < 100000; i++) {
//             Collision.detectCollision(a, b);
//         }

//         console.timeEnd('collision');
//     });

//     test('resolution box vs box', () => {
//         console.time('resolution');
//         const dt = 1 / 60;
//         const invDt = 1 / dt;
//         const a = new RigidBody(new BoxShape(60, 60), 100, 120, 5);
//         const b = new RigidBody(new BoxShape(60, 60), 100, 100, 5);
//         const manifold = Collision.detectCollision(a, b)!;

//         for (let i = 0; i < 100000; i++) {
//             manifold.preSolve(invDt);

//             for (let j = 0; j < 10; j++) {
//                 manifold.solve();
//             }
//         }

//         console.timeEnd('resolution');
//     });
// });
