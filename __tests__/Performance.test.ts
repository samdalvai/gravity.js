import Vec2 from '../src/math/Vec2';
import Body from '../src/physics/Body';
import CollisionDetection from '../src/physics/CollisionDetection';
import { ContactConstraint, JointConstraint } from '../src/physics/Constraint';
import { BoxShape, CircleShape } from '../src/physics/Shape';

describe('Performance', () => {
    test.each(Array.from({ length: 5 }))('Performance test', () => {
        // test('Performance test', () => {
        console.time('test');
        // const a = new Body(new BoxShape(20, 20), 100, 100, 5);
        // const a = new Body(new CircleShape(10), 100, 100, 10);

        const a = new Body(new CircleShape(60), 100, 100, 5);
        const b = new Body(new CircleShape(60), 100, 200, 5);
        const joint = new JointConstraint(a, b, new Vec2(100, 150));
        a.position.x -= 100;
        b.position.x += 100;

        // Move bodies apart
        const numFrames = 60;
        const solverIterations = 10;

        // const contacts: ContactConstraint[] = [];
        // CollisionDetection.detectCollisionCircleCircle(a, b, contacts);
        // const contact = contacts![0];

        const deltaTime = 1 / 60;
        const inverseDeltaTime = 1 / deltaTime;

        // Baseline = 850 ms
        for (let i = 0; i < 1000; i++) {
            for (let i = 0; i < numFrames; i++) {
                joint.preSolve(inverseDeltaTime);
                //contact.preSolve(inverseDeltaTime);

                for (let j = 0; j < solverIterations; j++) {
                    //contact.solve();
                    joint.solve();
                }
            }
        }

        a.integrateVelocities(deltaTime);
        b.integrateVelocities(deltaTime);

        console.timeEnd('test');
    });
});
