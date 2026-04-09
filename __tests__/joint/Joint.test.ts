import { describe, expect, test } from '@jest/globals';
import { RigidBody } from '../../src/core/RigidBody';
import { DistanceJoint } from '../../src/joint/DistanceJoint';
import { DistanceJoint as DistanceJointPerf } from '../../src/joint/DistanceJoint';
import { CircleShape } from '../../src/shapes/CircleShape';

describe('Joint', () => {
    function runJointSimulation(JointCtor: typeof DistanceJoint) {
        const a = new RigidBody(new CircleShape(60), 100, 100, 5);
        const b = new RigidBody(new CircleShape(60), 100, 200, 5);
        const joint = new JointCtor(a, b);

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
        return { a, b };
    }

    test('Joint constraint solving should apply impulses to correct position of bodies', () => {
        const { a, b } = runJointSimulation(DistanceJoint);

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

    test('Perf joint constraint solving should match the original joint behavior', () => {
        const { a, b } = runJointSimulation(DistanceJointPerf);

        expect(Math.abs(a.position.y - 95)).toBe(0.6009915351155684);
        expect(Math.abs(a.position.x - 100)).toBe(0);
        expect(Math.abs(b.position.y - 205)).toBe(0.6009915351155826);
        expect(Math.abs(b.position.x - 100)).toBe(0);

        expect(Math.abs(a.velocity.x)).toBe(0);
        expect(Math.abs(a.velocity.y - 260)).toBe(3.9405078930655577);

        expect(Math.abs(b.velocity.x)).toBe(0);
        expect(Math.abs(b.velocity.y + 260)).toBe(3.9405078930655577);
    });
});
