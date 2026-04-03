import { describe, expect, test } from '@jest/globals';

import { BoxShape, CapsuleShape, CircleShape, RigidBody, SegmentShape, Vec2 } from '../src_new';
import * as Collision from '../src_new/collision/Collision';

// Old wins
// describe('Performance circle vs circle', () => {
//     test('collision circle vs circle', () => {
//         console.time('collision');
//         const a = new RigidBody(new CircleShape(30), 100, 120, 5);
//         const b = new RigidBody(new CircleShape(30), 100, 100, 5);

//         for (let i = 0; i < 100000; i++) {
//             Collision.detectCollision(a, b);
//         }

//         console.timeEnd('collision');
//     });

//     test('resolution circle vs circle', () => {
//         console.time('resolution');
//         const dt = 1 / 60;
//         const invDt = 1 / dt;
//         const a = new RigidBody(new CircleShape(30), 100, 120, 5);
//         const b = new RigidBody(new CircleShape(30), 100, 100, 5);
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

// New wins
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

// New wins
// describe('Performance capsule vs capsule', () => {
//     test('collision capsule vs capsule', () => {
//         console.time('collision');
//         const a = new RigidBody(new CapsuleShape(30, 30), 100, 120, 5);
//         const b = new RigidBody(new CapsuleShape(30, 30), 100, 100, 5);

//         for (let i = 0; i < 100000; i++) {
//             Collision.detectCollision(a, b);
//         }

//         console.timeEnd('collision');
//     });

//     test('resolution capsule vs capsule', () => {
//         console.time('resolution');
//         const dt = 1 / 60;
//         const invDt = 1 / dt;
//         const a = new RigidBody(new CapsuleShape(30, 30), 100, 120, 5);
//         const b = new RigidBody(new CapsuleShape(30, 30), 100, 100, 5);
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

// New wins
// describe('Performance segment vs segment', () => {
//     test('collision segment vs segment', () => {
//         console.time('collision');
//         const a = new RigidBody(new SegmentShape(new Vec2(-50, 0), new Vec2(50, 0)), 120, 100, 5);
//         const b = new RigidBody(new SegmentShape(new Vec2(0, -50), new Vec2(0, 50)), 100, 100, 5);

//         for (let i = 0; i < 100000; i++) {
//             Collision.detectCollision(a, b);
//         }

//         console.timeEnd('collision');
//     });

//     test('resolution segment vs segment', () => {
//         console.time('resolution');
//         const dt = 1 / 60;
//         const invDt = 1 / dt;
//         const a = new RigidBody(new SegmentShape(new Vec2(-50, 0), new Vec2(50, 0)), 120, 100, 5);
//         const b = new RigidBody(new SegmentShape(new Vec2(0, -50), new Vec2(0, 50)), 100, 100, 5);
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

// Old wins
// describe('Performance circle vs polygon', () => {
//     test('collision circle vs polygon', () => {
//         console.time('collision');
//         const a = new RigidBody(new CircleShape(30), 100, 120, 5);
//         const b = new RigidBody(new BoxShape(60, 60), 100, 100, 5);

//         for (let i = 0; i < 100000; i++) {
//             Collision.detectCollision(a, b);
//         }

//         console.timeEnd('collision');
//     });

//     test('resolution circle vs polygon', () => {
//         console.time('resolution');
//         const dt = 1 / 60;
//         const invDt = 1 / dt;
//         const a = new RigidBody(new CircleShape(30), 100, 120, 5);
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

// Old wins, but not by a lot
// describe('Performance circle vs capsule', () => {
//     test('collision circle vs capsule', () => {
//         console.time('collision');
//         const a = new RigidBody(new CircleShape(30), 100, 120, 5);
//         const b = new RigidBody(new CapsuleShape(30, 30), 100, 100, 5);

//         for (let i = 0; i < 100000; i++) {
//             Collision.detectCollision(a, b);
//         }

//         console.timeEnd('collision');
//     });

//     test('resolution circle vs capsule', () => {
//         console.time('resolution');
//         const dt = 1 / 60;
//         const invDt = 1 / dt;
//         const a = new RigidBody(new CircleShape(30), 100, 120, 5);
//         const b = new RigidBody(new CapsuleShape(30, 30), 100, 100, 5);
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

// Old wins
// describe('Performance circle vs segment', () => {
//     test('collision circle vs segment', () => {
//         console.time('collision');
//         const a = new RigidBody(new CircleShape(30), 100, 120, 5);
//         const b = new RigidBody(new SegmentShape(new Vec2(0, -50), new Vec2(0, 50)), 100, 100, 5);

//         for (let i = 0; i < 100000; i++) {
//             Collision.detectCollision(a, b);
//         }

//         console.timeEnd('collision');
//     });

//     test('resolution circle vs segment', () => {
//         console.time('resolution');
//         const dt = 1 / 60;
//         const invDt = 1 / dt;
//         const a = new RigidBody(new CircleShape(30), 100, 120, 5);
//         const b = new RigidBody(new SegmentShape(new Vec2(0, -50), new Vec2(0, 50)), 100, 100, 5);
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

// New wins
// describe('Performance box vs segment', () => {
//     test('collision box vs segment', () => {
//         console.time('collision');
//         const a = new RigidBody(new SegmentShape(new Vec2(0, -50), new Vec2(0, 50)), 100, 100, 5);
//         const b = new RigidBody(new BoxShape(60, 60), 100, 100, 5);

//         for (let i = 0; i < 100000; i++) {
//             Collision.detectCollision(a, b);
//         }

//         console.timeEnd('collision');
//     });

//     test('resolution box vs segment', () => {
//         console.time('resolution');
//         const dt = 1 / 60;
//         const invDt = 1 / dt;
//         const a = new RigidBody(new SegmentShape(new Vec2(0, -50), new Vec2(0, 50)), 100, 100, 5);
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

// New wins
// describe('Performance box vs capsule', () => {
//     test('collision box vs capsule', () => {
//         console.time('collision');
//         const a = new RigidBody(new BoxShape(60, 60), 100, 100, 5);
//         const b = new RigidBody(new CapsuleShape(30, 30), 100, 100, 5);

//         for (let i = 0; i < 100000; i++) {
//             Collision.detectCollision(a, b);
//         }

//         console.timeEnd('collision');
//     });

//     test('resolution box vs capsule', () => {
//         console.time('resolution');
//         const dt = 1 / 60;
//         const invDt = 1 / dt;
//         const a = new RigidBody(new BoxShape(60, 60), 100, 100, 5);
//         const b = new RigidBody(new CapsuleShape(30, 30), 100, 100, 5);
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

// New wins
describe('Performance capsule vs segment', () => {
    test('collision capsule vs segment', () => {
        console.time('collision');
        const a = new RigidBody(new SegmentShape(new Vec2(0, -50), new Vec2(0, 50)), 100, 100, 5);
        const b = new RigidBody(new CapsuleShape(30, 30), 100, 100, 5);

        for (let i = 0; i < 100000; i++) {
            Collision.detectCollision(a, b);
        }

        console.timeEnd('collision');
    });

    test('resolution capsule vs segment', () => {
        console.time('resolution');
        const dt = 1 / 60;
        const invDt = 1 / dt;
        const a = new RigidBody(new SegmentShape(new Vec2(0, -50), new Vec2(0, 50)), 100, 100, 5);
        const b = new RigidBody(new CapsuleShape(30, 30), 100, 100, 5);
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
