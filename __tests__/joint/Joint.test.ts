import { describe, expect, test } from '@jest/globals';
import { Bodies } from '../../src/bodies/Bodies';
import { RigidBody } from '../../src/core/RigidBody';
import { DistanceJoint } from '../../src/joint/DistanceJoint';
import { DistanceJoint as DistanceJointPerf } from '../../src/joint/DistanceJoint';
import { WeldJoint } from '../../src/joint/WeldJoint';
import { Vec2 } from '../../src/math/Vec2';
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

    function createWeldedBoxes() {
        const a = Bodies.box({ width: 60, height: 60, x: 30, y: 0, mass: 1 });
        const b = Bodies.box({ width: 60, height: 60, x: -30, y: 0, mass: 1 });
        const joint = new WeldJoint(a, b);

        return { a, b, joint };
    }

    function getWorldAnchor(body: RigidBody, localAnchor: Vec2): Vec2 {
        return body.position.addNew(localAnchor.rotate(body.rotation));
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

    test('Weld joint should remain at rest when the anchor starts aligned', () => {
        const { a, b, joint } = createWeldedBoxes();
        const deltaTime = 1 / 60;
        const solverIterations = 10;

        for (let frame = 0; frame < 10; frame++) {
            joint.preSolve(1 / deltaTime);

            for (let i = 0; i < solverIterations; i++) {
                joint.solve();
            }

            a.integrateVelocities(deltaTime);
            b.integrateVelocities(deltaTime);
        }

        expect(a.position.x).toBeCloseTo(30, 12);
        expect(a.position.y).toBeCloseTo(0, 12);
        expect(b.position.x).toBeCloseTo(-30, 12);
        expect(b.position.y).toBeCloseTo(0, 12);
        expect(a.velocity.x).toBeCloseTo(0, 12);
        expect(a.velocity.y).toBeCloseTo(0, 12);
        expect(b.velocity.x).toBeCloseTo(0, 12);
        expect(b.velocity.y).toBeCloseTo(0, 12);
    });

    test('Weld joint should reduce anchor separation after the bodies are displaced', () => {
        const { a, b, joint } = createWeldedBoxes();
        const deltaTime = 1 / 60;
        const solverIterations = 10;

        a.position.x += 5;
        b.position.x -= 5;

        const initialAnchorSeparation = getWorldAnchor(b, joint.localAnchorB)
            .subNew(getWorldAnchor(a, joint.localAnchorA))
            .magnitude();

        joint.preSolve(1 / deltaTime);

        for (let i = 0; i < solverIterations; i++) {
            joint.solve();
        }

        a.integrateVelocities(deltaTime);
        b.integrateVelocities(deltaTime);

        const finalAnchorSeparation = getWorldAnchor(b, joint.localAnchorB)
            .subNew(getWorldAnchor(a, joint.localAnchorA))
            .magnitude();

        expect(finalAnchorSeparation).toBeLessThan(initialAnchorSeparation);
        expect(a.velocity.x).toBeLessThan(0);
        expect(b.velocity.x).toBeGreaterThan(0);
    });
});
