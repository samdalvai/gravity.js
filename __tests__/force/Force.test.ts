import { describe, expect, test } from '@jest/globals';

import { BodiesFactory, FIXED_DELTA_TIME, Force, Vec2 } from '../../src';

describe('Force', () => {
    test('Buoyancy scales with the body shape area when fully submerged', () => {
        const body = BodiesFactory.circle({
            radius: 10,
            x: 0,
            y: 0,
            mass: 1,
        });

        const buoyancy = Force.generateBuoyancyForce(body, 20, 0.5, 10);

        expect(buoyancy.x).toBe(0);
        expect(buoyancy.y).toBeCloseTo(Math.PI * 10 * 10 * 0.5 * 10);
    });

    test('Water drag is clamped so it cannot reverse velocity in one step', () => {
        const body = BodiesFactory.circle({
            radius: 5,
            x: 0,
            y: 0,
            mass: 0.01,
            velocity: new Vec2(0, -100),
        });

        const waterDrag = Force.generateLinearWaterDragForce(body, 10, 0.2, FIXED_DELTA_TIME);
        const maxForce = (body.mass * body.velocity.magnitude()) / FIXED_DELTA_TIME;

        expect(waterDrag.x).toBeCloseTo(0);
        expect(waterDrag.y).toBeCloseTo(maxForce);
    });
});
