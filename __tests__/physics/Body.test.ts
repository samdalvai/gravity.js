import Vec2 from '../../src/math/Vec2';
import Body from '../../src/physics/Body';
import { CircleShape } from '../../src/physics/Shape';

describe('Body', () => {
    test('Two bodies created should have a different id', () => {
        const a = new Body(new CircleShape(10), 100, 100, 10);
        const b = new Body(new CircleShape(10), 100, 100, 10);

        expect(a.id).toBe(0);
        expect(b.id).toBe(1);
    });

    test('Bodies paur key should return the same value regardless of the ordering', () => {
        const a = new Body(new CircleShape(10), 100, 100, 10);
        const b = new Body(new CircleShape(10), 100, 100, 10);

        expect(Body.pairKey(a, b)).toBe(Body.pairKey(b, a));
    });

    test('Should convert local body point to world space', () => {
        const a = new Body(new CircleShape(10), 100, 100, 10);

        const world = a.localPointToWorld(new Vec2(10, 10));
        expect(world.x).toBe(110);
        expect(world.y).toBe(110);
    });

    test('Should convert world point to body local space', () => {
        const a = new Body(new CircleShape(10), 100, 100, 10);

        const local = a.worldPointToLocal(new Vec2(110, 110));
        expect(local.x).toBe(10);
        expect(local.y).toBe(10);
    });
});
