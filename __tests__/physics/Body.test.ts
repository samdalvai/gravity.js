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
});
