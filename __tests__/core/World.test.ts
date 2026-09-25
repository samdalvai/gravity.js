import { describe, expect, jest, test } from '@jest/globals';

import { CollisionCategory } from '../../src/collision/CollisionFilter';
import { ContactManifold } from '../../src/collision/ContactManifold';
import * as NarrowPhase from '../../src/collision/NarrowPhase';
import { FIXED_DELTA_TIME, MAX_BODIES, SETTINGS } from '../../src/core/Constants';
import { RigidBody } from '../../src/core/RigidBody';
import { World } from '../../src/core/World';
import { Vec2 } from '../../src/math/Vec2';
import { BoxShape } from '../../src/shapes/BoxShape';
import { CapsuleShape } from '../../src/shapes/CapsuleShape';
import { CircleShape } from '../../src/shapes/CircleShape';

describe('World body limits', () => {
    test('keeps the default limit and supports larger worlds', () => {
        const defaultWorld = new World(0);
        const largeWorld = new World(0, { maxBodies: MAX_BODIES + 1 });

        for (let i = 0; i < MAX_BODIES; i++) {
            defaultWorld.addBody(new RigidBody(new CircleShape(1), i * 3, 0, 1));
            largeWorld.addBody(new RigidBody(new CircleShape(1), i * 3, 0, 1));
        }

        const extra = new RigidBody(new CircleShape(1), MAX_BODIES * 3, 0, 1);
        expect(defaultWorld.maxBodies).toBe(MAX_BODIES);
        expect(() => defaultWorld.addBody(extra)).toThrow('Max number of bodies exceeded');
        largeWorld.addBody(extra);
        largeWorld.update();
        expect(largeWorld.getBodies()).toHaveLength(MAX_BODIES + 1);

        defaultWorld.clear();
        largeWorld.clear();
    });

    test('enforces a per-world limit and restores capacity after removal or clear', () => {
        const world = new World(0, { maxBodies: 1 });
        const a = new RigidBody(new CircleShape(1), 0, 0, 1);
        const b = new RigidBody(new CircleShape(1), 3, 0, 1);
        world.addBody(a);
        expect(() => world.addBody(b)).toThrow('Max number of bodies exceeded');
        expect(world.getBodies()).toEqual([a]);

        world.removeBody(a);
        world.addBody(b);
        world.clear();
        world.addBody(a);
        expect(world.maxBodies).toBe(1);
        expect(world.getBodies()).toEqual([a]);
    });

    test('supports an empty world and an explicitly unlimited world', () => {
        const empty = new World(0, { maxBodies: 0 });
        const unlimited = new World(0, { maxBodies: Infinity });
        expect(() => empty.addBody(new RigidBody(new CircleShape(1), 0, 0, 1))).toThrow();
        for (let i = 0; i <= MAX_BODIES; i++) {
            unlimited.addBody(new RigidBody(new CircleShape(1), i * 3, 0, 1));
        }
        expect(unlimited.getBodies()).toHaveLength(MAX_BODIES + 1);
        unlimited.clear();
    });

    test.each([-1, 1.5, NaN, -Infinity, Number.MAX_SAFE_INTEGER + 1])('rejects invalid limit %s', maxBodies => {
        expect(() => new World(0, { maxBodies })).toThrow(RangeError);
    });
});

describe('World contact cache', () => {
    test('uses per-world material callbacks when building a contact', () => {
        const frictionCallback = jest.fn(() => 0.25);
        const restitutionCallback = jest.fn(() => 0.75);
        const world = new World(0, { frictionCallback, restitutionCallback });
        const a = new RigidBody(new CircleShape(10), 0, 0, 1);
        const b = new RigidBody(new CircleShape(10), 15, 0, 1);
        world.addBody(a);
        world.addBody(b);

        world.update();

        expect(frictionCallback).toHaveBeenCalledWith(a.friction, b.friction, a, b);
        expect(restitutionCallback).toHaveBeenCalledWith(a.restitution, b.restitution, a, b);
        world.clear();
    });

    test('warm starts from the matching pair when IDs differ by 65536', () => {
        const world = new World(0);
        const ids = [1, 2, 65537, 65538];
        const positions = [0, 20, 100, 120];
        for (let i = 0; i < ids.length; i++) {
            const body = new RigidBody(new CircleShape(10), positions[i], 0, 1);
            Object.defineProperty(body, 'id', { value: ids[i] });
            world.addBody(body);
        }

        world.update();
        expect(world.getManifolds()).toHaveLength(2);
        const original = ContactManifold.prototype.tryWarmStart;
        const spy = jest.spyOn(ContactManifold.prototype, 'tryWarmStart').mockImplementation(function (
            this: ContactManifold,
            oldManifold: ContactManifold,
        ) {
            expect(oldManifold.bodyA).toBe(this.bodyA);
            expect(oldManifold.bodyB).toBe(this.bodyB);
            original.call(this, oldManifold);
        });

        try {
            world.update();
            expect(spy).toHaveBeenCalledTimes(2);
        } finally {
            spy.mockRestore();
            world.clear();
        }
    });
});

describe('World substeps', () => {
    test('applies continuous external forces during every substep', () => {
        const previousSubSteps = SETTINGS.subSteps;
        SETTINGS.subSteps = 4;

        try {
            const world = new World(0);
            const body = new RigidBody(new CircleShape(10), 0, 0, 1);
            const force = new Vec2(60, 0);
            let callbackCount = 0;

            world.addBody(body);
            world.update(dt => {
                expect(dt).toBe(FIXED_DELTA_TIME / SETTINGS.subSteps);
                callbackCount++;
                body.addForce(force);
            });

            expect(callbackCount).toBe(SETTINGS.subSteps);
            expect(body.velocity.x).toBeCloseTo(force.x * FIXED_DELTA_TIME);
        } finally {
            SETTINGS.subSteps = previousSubSteps;
        }
    });

    test('collects opt-in timing and robustness metrics', () => {
        const previousCollectMetrics = SETTINGS.collectMetrics;
        SETTINGS.collectMetrics = true;

        try {
            const world = new World(0);
            world.addBody(new RigidBody(new CircleShape(10), 0, 0, 1));
            world.addBody(new RigidBody(new CircleShape(10), 15, 0, 1));

            world.update();

            const metrics = world.getMetrics();
            expect(metrics.updateMs).toBeGreaterThanOrEqual(0);
            expect(metrics.broadPhaseCalls).toBeGreaterThan(0);
            expect(metrics.narrowPhaseCalls).toBeGreaterThan(0);
            expect(metrics.potentialPairCount).toBeGreaterThan(0);
            expect(metrics.narrowPhaseTests).toBeGreaterThan(0);
            expect(metrics.manifoldCount).toBeGreaterThan(0);
            expect(metrics.maxPenetrationDepth).toBeGreaterThan(0);
        } finally {
            SETTINGS.collectMetrics = previousCollectMetrics;
        }
    });
});

describe('World grounding', () => {
    test('broad phase skips pairs rejected by collision filters', () => {
        const world = new World(0);

        const a = new RigidBody(new CircleShape(20), 0, 0, 1);
        a.collisionCategory = CollisionCategory.DEFAULT;
        a.collisionMask = CollisionCategory.NONE;

        const b = new RigidBody(new CircleShape(20), 0, 0, 1);
        b.collisionCategory = CollisionCategory.PROJECTILE;
        b.collisionMask = CollisionCategory.ALL;

        const detectCollisionSpy = jest.spyOn(NarrowPhase, 'detectCollision');

        world.addBody(a);
        world.addBody(b);

        world.update();

        expect(detectCollisionSpy).not.toHaveBeenCalled();
        expect(world.getManifolds()).toHaveLength(0);

        detectCollisionSpy.mockRestore();
    });

    test('grounded follows the collision manifold body order, not the broad phase pair order', () => {
        const world = new World(0);

        const circle = new RigidBody(new CircleShape(10), 0, 20, 1);
        const floor = new RigidBody(new BoxShape(100, 20), 0, 0, 0);

        world.addBody(circle);
        world.addBody(floor);

        world.update();

        expect(circle.isGrounded).toBe(true);
        expect(floor.isGrounded).toBe(false);
    });

    test('a vertical capsule on top of a circle is grounded', () => {
        const world = new World(0);

        const support = new RigidBody(new CircleShape(30), 0, 0, 0);
        const capsule = new RigidBody(new CapsuleShape(30, 10), 0, 70, 1);

        world.addBody(support);
        world.addBody(capsule);

        world.update();

        expect(capsule.isGrounded).toBe(true);
        expect(support.isGrounded).toBe(false);
    });

    test('a box on top of a circle is grounded', () => {
        const world = new World(0);

        const support = new RigidBody(new CircleShape(30), 0, 0, 0);
        const box = new RigidBody(new BoxShape(40, 40), 0, 50, 1);

        world.addBody(support);
        world.addBody(box);

        world.update();

        expect(box.isGrounded).toBe(true);
        expect(support.isGrounded).toBe(false);
    });
});
