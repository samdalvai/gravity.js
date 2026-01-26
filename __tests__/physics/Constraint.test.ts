import Vec2 from '../../src/math/Vec2';
import CollisionDetection from '../../src/physics/CollisionDetection';
import { DistanceJoint } from '../../src/physics/DistanceJoint';
import RigidBody from '../../src/physics/RigidBody';
import { CircleShape } from '../../src/physics/Shape';

describe('Constraint', () => {
    test('Joint constraint solving should apply impulses to correct position of bodies', () => {
        const a = new RigidBody(new CircleShape(60), 100, 100, 5);
        const b = new RigidBody(new CircleShape(60), 100, 200, 5);
        const joint = new DistanceJoint(a, b);

        // Move bodies apart
        a.position.y -= 10;
        b.position.y += 10;
        const numFrames = 60;
        const solverIterations = 10;

        const deltaTime = 1 / 60;
        for (let i = 0; i < numFrames; i++) {
            joint.preSolve(1 / deltaTime);

            for (let j = 0; j < solverIterations; j++) {
                joint.solve();
            }
        }

        a.integrateVelocities(deltaTime);
        b.integrateVelocities(deltaTime);

        // Check that the solver approximation is "good enough"
        expect(Math.abs(a.position.y - 95)).toBe(0.6009915351155684);
        expect(Math.abs(a.position.x - 100)).toBe(0);
        expect(Math.abs(b.position.y - 205)).toBe(0.6009915351155826);
        expect(Math.abs(b.position.x - 100)).toBe(0);

        expect(Math.abs(a.velocity.x)).toBe(0);
        expect(Math.abs(a.velocity.y - 260)).toBe(3.9405078930655577);

        expect(Math.abs(b.velocity.x)).toBe(0);
        expect(Math.abs(b.velocity.y + 260)).toBe(3.9405078930655577);
    });

    test('Penetration constraint solving should apply impulses to correct position of bodies', () => {
        const a = new RigidBody(new CircleShape(60), 100, 100, 5);
        const b = new RigidBody(new CircleShape(60), 200, 100, 5);

        // Move bodies apart
        const numFrames = 60;
        const solverIterations = 20;

        const manifold = CollisionDetection.detectCollisionCircleCircle(a, b)!;

        const deltaTime = 1 / 60;
        for (let i = 0; i < numFrames; i++) {
            manifold.preSolve(1 / deltaTime);

            for (let j = 0; j < solverIterations; j++) {
                manifold.solve();
            }
        }

        a.integrateVelocities(deltaTime);
        b.integrateVelocities(deltaTime);

        expect(a.position.y).toBe(100);
        expect(b.position.y).toBe(100);

        // Check that the solver moved the objects apart
        expect(a.position.x).toBeLessThan(100);
        expect(b.position.x).toBeGreaterThan(200);
        expect(a.velocity.x).toBeLessThan(0);
        expect(b.velocity.x).toBeGreaterThan(0);
    });
});
