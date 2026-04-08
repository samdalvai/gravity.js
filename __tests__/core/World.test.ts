import { describe, expect, test } from '@jest/globals';

import { FIXED_DELTA_TIME } from '../../src/core/Constants';
import { RigidBody } from '../../src/core/RigidBody';
import { World } from '../../src/core/World';
import { BoxShape } from '../../src/shapes/BoxShape';
import { CapsuleShape } from '../../src/shapes/CapsuleShape';
import { CircleShape } from '../../src/shapes/CircleShape';

describe('World grounding', () => {
    test('grounded follows the collision manifold body order, not the broad phase pair order', () => {
        const world = new World(0);

        const circle = new RigidBody(new CircleShape(10), 0, 20, 1);
        const floor = new RigidBody(new BoxShape(100, 20), 0, 0, 0);

        world.addBody(circle);
        world.addBody(floor);

        world.update(FIXED_DELTA_TIME);

        expect(circle.isGrounded).toBe(true);
        expect(floor.isGrounded).toBe(false);
    });

    test('a vertical capsule on top of a circle is grounded', () => {
        const world = new World(0);

        const support = new RigidBody(new CircleShape(30), 0, 0, 0);
        const capsule = new RigidBody(new CapsuleShape(30, 10), 0, 70, 1);

        world.addBody(support);
        world.addBody(capsule);

        world.update(FIXED_DELTA_TIME);

        expect(capsule.isGrounded).toBe(true);
        expect(support.isGrounded).toBe(false);
    });

    test('a box on top of a circle is grounded', () => {
        const world = new World(0);

        const support = new RigidBody(new CircleShape(30), 0, 0, 0);
        const box = new RigidBody(new BoxShape(40, 40), 0, 50, 1);

        world.addBody(support);
        world.addBody(box);

        world.update(FIXED_DELTA_TIME);

        expect(box.isGrounded).toBe(true);
        expect(support.isGrounded).toBe(false);
    });
});
