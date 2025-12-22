import Vec2 from '../../src/math/Vec2';
import Body from '../../src/physics/Body';
import CollisionDetection from '../../src/physics/CollisionDetection';
import { ContactConstraint, JointConstraint } from '../../src/physics/Constraint';
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

        // Check that the solver approximation is "good enough"
        expect(Math.abs(a.position.y - 100)).toBeLessThan(0.001);
        expect(Math.abs(a.position.x - 100)).toBeLessThan(0.001);
        expect(Math.abs(b.position.y - 200)).toBeLessThan(0.001);
        expect(Math.abs(b.position.x - 100)).toBeLessThan(0.001);

        expect(Math.abs(a.velocity.x)).toBeLessThan(0.005);
        expect(Math.abs(a.velocity.y)).toBeLessThan(0.005);

        expect(Math.abs(b.velocity.x)).toBeLessThan(0.005);
        expect(Math.abs(b.velocity.y)).toBeLessThan(0.005);
    });

    test('Penetration constraint solving should apply impulses to correct position of bodies', () => {
        const a = new Body(new CircleShape(60), 100, 100, 5);
        const b = new Body(new CircleShape(60), 217.5, 100, 5);

        // Move bodies apart
        const numFrames = 60;
        const solverIterations = 10;

        const contacts: ContactConstraint[] = [];
        CollisionDetection.detectCollisionCircleCircle(a, b, contacts);
        const contact = contacts![0];

        const deltaTime = 1 / 60;
        for (let i = 0; i < numFrames; i++) {
            contact.preSolve(1 / deltaTime);

            for (let j = 0; j < solverIterations; j++) {
                contact.solve();
            }

            a.integrateVelocities(deltaTime);
            b.integrateVelocities(deltaTime);
        }

        expect(a.position.y).toBe(100);
        expect(b.position.y).toBe(100);

        // Check that the solver approximation is "good enough"
        expect(Math.abs(a.position.x - b.position.x)).toBeGreaterThan(120);
        expect(a.velocity.x).toBeLessThan(-30);
        expect(b.velocity.x).toBeGreaterThan(30);
    });
});
