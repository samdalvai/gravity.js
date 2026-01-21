import Vec2 from '../src/math/Vec2';
import Body from '../src/physics/Body';
import CollisionDetection from '../src/physics/CollisionDetection';
import { JointConstraint } from '../src/physics/Constraint';
import { BoxShape, CircleShape } from '../src/physics/Shape';

describe('Performance', () => {
    // test.each(Array.from({ length: 5 }))('Performance test', () => {
    //     // test('Performance test', () => {
    //     console.time('test');
    //     const a = new Body(new BoxShape(20, 20), 100, 100, 5);
    //     const b = new Body(new CircleShape(10), 100, 100, 10);

    //     // const a = new Body(new CircleShape(60), 100, 100, 5);
    //     // const b = new Body(new CircleShape(60), 100, 200, 5);
    //     // const joint = new JointConstraint(a, b, new Vec2(100, 150));
    //     // a.position.x -= 100;
    //     // b.position.x += 100;

    //     // Move bodies apart
    //     const numFrames = 60;
    //     const solverIterations = 10;

    //     const contacts: ContactConstraint[] = [];
    //     CollisionDetection.detectCollisionCircleCircle(a, b, contacts);
    //     const contact = contacts![0];

    //     const deltaTime = 1 / 60;
    //     const inverseDeltaTime = 1 / deltaTime;

    //     // Baseline = 220 ms
    //     for (let i = 0; i < 1000; i++) {
    //         for (let i = 0; i < numFrames; i++) {
    //             // joint.preSolve(inverseDeltaTime);
    //             contact.preSolve(inverseDeltaTime);

    //             for (let j = 0; j < solverIterations; j++) {
    //                 contact.solve();
    //                 // joint.solve();
    //             }
    //             contact.postSolve();
    //             // joint.postSolve();
    //         }
    //     }

    //     a.integrateVelocities(deltaTime);
    //     b.integrateVelocities(deltaTime);

    //     console.timeEnd('test');
    // });

    // test.each(Array.from({ length: 5 }))('Performance test', () => {
    //     const boxA = new Body(new BoxShape(20, 20), 100, 100, 5);
    //     const boxB = new Body(new BoxShape(20, 20), 110, 100, 5);
    //     const circleA = new Body(new CircleShape(10), 100, 100, 5);
    //     const circleB = new Body(new CircleShape(10), 105, 100, 5);

    //     const contacts: ContactConstraint[] = [];

    //     // circle-circle: 121 ms
    //     // console.time('circle-circle');
    //     // for (let i = 0; i < 10000; i++) {
    //     //     CollisionDetection.detectCollisionCircleCircle(circleA, circleB, contacts);
    //     // }
    //     // console.timeEnd('circle-circle');

    //     // // polygon-polygon: 287 ms
    //     // contacts.length = 0;
    //     // console.time('polygon-polygon');
    //     // for (let i = 0; i < 10000; i++) {
    //     //     CollisionDetection.detectCollisionPolygonPolygon(boxA, boxB, contacts);
    //     // }
    //     // console.timeEnd('polygon-polygon');

    //     // // polygon-circle: 136 ms
    //     // contacts.length = 0;
    //     // console.time('polygon-circle');
    //     // for (let i = 0; i < 10000; i++) {
    //     //     CollisionDetection.detectCollisionPolygonCircle(boxA, circleA, contacts);
    //     // }
    //     // console.timeEnd('polygon-circle');

    //     // polygon-circle: 537 ms
    //     contacts.length = 0;
    //     console.time('mixed-shape');
    //     for (let i = 0; i < 10000; i++) {
    //         CollisionDetection.detectCollision(boxA, boxB, contacts);
    //         CollisionDetection.detectCollision(circleA, circleB, contacts);
    //         CollisionDetection.detectCollision(boxA, circleA, contacts);
    //     }
    //     console.timeEnd('mixed-shape');
    // });

    // test('Performance test', () => {
    //     const a = new Body(new BoxShape(20, 20), 100, 100, 5);
    //     a.rotation = 0.5;
    //     const point = new Vec2(200, 200);

    //     // baseline: 333 ms
    //     console.time('time');
    //     for (let i = 0; i < 1000000; i++) {
    //         a.worldSpaceToLocalSpace(point);
    //     }
    //     console.timeEnd('time');
    // });

    test('', () => {
        expect(true).toBe(true);
    });
});
