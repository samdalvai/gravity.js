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
        expect(a.position.x).toBeLessThan(100);
        expect(b.position.x).toBeGreaterThan(200);
        expect(a.velocity.x).toBeLessThan(0);
        expect(b.velocity.x).toBeGreaterThan(0);
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

    test('keeps per-point separations, local anchors, and persistence independently', () => {
        const a = new RigidBody(new CircleShape(10), 10, 20, 1);
        const b = new RigidBody(new CircleShape(10), 30, 20, 1);
        a.rotation = Math.PI / 2;

        const oldManifold = new ContactManifold(
            a,
            b,
            2,
            4,
            1,
            0,
            20,
            15,
            11,
            20,
            25,
            22,
            false,
            -1,
            -4,
        );
        oldManifold.normalImpulseSum0 = 3;
        oldManifold.tangentImpulseSum0 = 2;
        oldManifold.normalImpulseSum1 = 7;
        oldManifold.tangentImpulseSum1 = 5;

        const manifold = new ContactManifold(
            a,
            b,
            2,
            4,
            1,
            0,
            20,
            25,
            22,
            20,
            15,
            99,
            false,
            -4,
            -0.5,
        );
        manifold.tryWarmStart(oldManifold);

        expect(manifold.penetrationDepth).toBe(4);
        expect(manifold.points.map(point => point.separation)).toEqual([-4, -0.5]);
        expect(manifold.points[0].localAnchorA.x).toBeCloseTo(5);
        expect(manifold.points[0].localAnchorA.y).toBeCloseTo(-10);
        expect(manifold.points[0].persisted).toBe(true);
        expect(manifold.points[0].normalImpulse).toBe(7);
        expect(manifold.points[0].tangentImpulse).toBe(5);
        expect(manifold.points[1].persisted).toBe(false);
        expect(manifold.points[1].normalImpulse).toBe(0);
        expect(manifold.points[1].tangentImpulse).toBe(0);
    });

    test('records per-point impact state while preserving the existing solve', () => {
        const a = new RigidBody(new CircleShape(10), 0, 0, 0);
        const b = new RigidBody(new CircleShape(10), 15, 0, 1);
        b.velocity.x = -200;

        const manifold = Collision.detectCollision(a, b)!;
        manifold.preSolve(60);
        manifold.solve();

        const point = manifold.points[0];
        expect(point.normalVelocity).toBe(-200);
        expect(point.restitutionVelocity).toBeGreaterThan(0);
        expect(point.totalNormalImpulse).toBeGreaterThan(0);
    });

    test('applies rolling resistance as a contact impulse', () => {
        const a = new RigidBody(new CircleShape(10), 0, 0, 0);
        const b = new RigidBody(new CircleShape(10), 15, 0, 1);
        b.angularVelocity = 20;

        const manifold = Collision.detectCollision(a, b)!;
        manifold.preSolve(60);
        manifold.solve();

        expect(Math.abs(b.angularVelocity)).toBeLessThan(20);
        expect(Math.abs(manifold.rollingImpulseSum)).toBeGreaterThan(0);
    });

    test('updates separation from persistent local anchors', () => {
        const a = new RigidBody(new CircleShape(10), 0, 0, 0);
        const b = new RigidBody(new CircleShape(10), 15, 0, 1);
        const manifold = Collision.detectCollision(a, b)!;
        const initialSeparation = manifold.points[0].separation;

        b.position.x += 3;
        manifold.updateSeparationFromAnchors();

        expect(manifold.points[0].separation).toBeCloseTo(initialSeparation + 3);
    });
});
