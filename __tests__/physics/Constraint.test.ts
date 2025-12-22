import Vec2 from '../../src/math/Vec2';
import Body from '../../src/physics/Body';
import { JointConstraint } from '../../src/physics/Constraint';
import { CircleShape } from '../../src/physics/Shape';

describe('Constraint', () => {
    test('Joint constraint solving should apply impulses to correct position of bodies', () => {
        const a = new Body(new CircleShape(60), 100, 100, 5);
        const b = new Body(new CircleShape(60), 100, 200, 5);
        const joint = new JointConstraint(a, b, new Vec2(100, 150));

        // Move bodies apart
        a.position.y -= 100;
        b.position.y += 100;
        const numFrames = 60;
        const solverIterations = 10;

        const deltaTime = 1 / 60;
        for (let i = 0; i < numFrames; i++) {
            joint.preSolve(1 / deltaTime);

            for (let j = 0; j < solverIterations; j++) {
                joint.solve();
            }

            a.integrateVelocities(deltaTime);
            b.integrateVelocities(deltaTime);
        }

        console.log('a position: ', a.position);
        console.log('b position: ', b.position);

        console.log('a velocity: ', a.velocity);
        console.log('b velocity: ', b.velocity);
    });
});
