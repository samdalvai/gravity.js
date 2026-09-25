import { describe, expect, test } from '@jest/globals';

import * as Collision from '../../src/collision/NarrowPhase';
import { ContactManifold } from '../../src/collision/ContactManifold';
import { RigidBody } from '../../src/core/RigidBody';
import { CircleShape } from '../../src/shapes/CircleShape';

describe('Contact', () => {
    test('Contact constraint solving should apply impulses to correct position of bodies', () => {
        const a = new RigidBody(new CircleShape(60), 100, 100, 5);
        const b = new RigidBody(new CircleShape(60), 200, 100, 5);

        // Move bodies apart
        const numFrames = 60;
        const solverIterations = 20;

        const manifold = Collision.detectCollision(a, b)!;

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
        expect(a.position.x).toBe(98.05);
        expect(b.position.x).toBe(201.95);
        expect(a.velocity.x).toBe(-117);
        expect(b.velocity.x).toBe(117);
    });

    test('falls back to scalar normal solves when a two-point block is singular', () => {
        const a = new RigidBody(new CircleShape(10), 0, 0, 1);
        const b = new RigidBody(new CircleShape(10), 15, 0, 1);
        const manifold = new ContactManifold(
            a,
            b,
            2,
            5,
            1,
            0,
            7.5,
            0,
            1,
            7.5,
            0,
            2,
            false,
        );

        manifold.preSolve(60);
        expect(() => manifold.solve()).not.toThrow();
        expect(Number.isFinite(manifold.normalImpulseSum0)).toBe(true);
        expect(Number.isFinite(manifold.normalImpulseSum1)).toBe(true);
    });

    test('uses the current normal impulse for friction on the first solve pass', () => {
        const a = new RigidBody(new CircleShape(10), 0, 0, 0);
        const b = new RigidBody(new CircleShape(10), 15, 0, 1);
        b.velocity.y = 100;

        const manifold = Collision.detectCollision(a, b)!;
        manifold.preSolve(60);
        manifold.solve();

        expect(manifold.normalImpulseSum0).toBeGreaterThan(0);
        expect(Math.abs(manifold.tangentImpulseSum0)).toBeGreaterThan(0);
    });
});
