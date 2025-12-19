import VecN from '../../src/math/VecN';

describe('VecN', () => {
    test('constructor creates zero-filled vector', () => {
        const v = new VecN(3);
        expect(v.N).toBe(3);
        expect(v.data).toEqual([0, 0, 0]);
    });

    test('copy constructor clones values', () => {
        const a = new VecN(3);
        a.data = [1, 2, 3];
        const b = VecN.from(a);

        expect(b.N).toBe(3);
        expect(b.data).toEqual([1, 2, 3]);
        expect(b.data).not.toBe(a.data); // deep copy
    });

    test('zero() sets all values to 0', () => {
        const v = new VecN(4);
        v.data = [5, -2, 9, 1];
        v.zero();
        expect(v.data).toEqual([0, 0, 0, 0]);
    });

    test('dot() computes dot product', () => {
        const a = new VecN(3);
        const b = new VecN(3);
        a.data = [1, 2, 3];
        b.data = [4, 5, 6];
        expect(a.dot(b)).toBe(1 * 4 + 2 * 5 + 3 * 6);
    });

    test('assign() copies values', () => {
        const a = new VecN(3);
        const b = new VecN(3);
        a.data = [7, 8, 9];
        b.assign(a);
        expect(b.data).toEqual([7, 8, 9]);
        expect(b.data).not.toBe(a.data);
    });

    test('addNew() returns new vector', () => {
        const a = new VecN(3);
        const b = new VecN(3);
        a.data = [1, 1, 1];
        b.data = [2, 2, 2];

        const c = a.addNew(b);
        expect(c.data).toEqual([3, 3, 3]);

        expect(a.data).toEqual([1, 1, 1]); // unchanged
    });

    test('subNew() returns new vector', () => {
        const a = new VecN(3);
        const b = new VecN(3);
        a.data = [5, 4, 3];
        b.data = [1, 1, 1];

        const c = a.subNew(b);
        expect(c.data).toEqual([4, 3, 2]);
    });

    test('scaleNew() returns new vector', () => {
        const a = new VecN(3);
        a.data = [2, 3, 4];

        const b = a.scaleNew(2);
        expect(b.data).toEqual([4, 6, 8]);
    });

    test('addAssign() modifies in place', () => {
        const a = new VecN(3);
        const b = new VecN(3);
        a.data = [1, 2, 3];
        b.data = [2, 2, 2];

        a.addAssign(b);
        expect(a.data).toEqual([3, 4, 5]);
    });

    test('subAssign() modifies in place', () => {
        const a = new VecN(3);
        const b = new VecN(3);
        a.data = [5, 4, 3];
        b.data = [1, 1, 1];

        a.subAssign(b);
        expect(a.data).toEqual([4, 3, 2]);
    });

    test('scaleAssign() modifies in place', () => {
        const a = new VecN(3);
        a.data = [1, 2, 3];

        a.scaleAssign(3);
        expect(a.data).toEqual([3, 6, 9]);
    });

    test('index getter returns value', () => {
        const a = new VecN(3);
        a.data = [4, 5, 6];
        expect(a.get(0)).toBe(4);
        expect(a.get(2)).toBe(6);
    });

    test('index setter sets value', () => {
        const a = new VecN(3);
        a.set(0, 99);
        expect(a.data[0]).toBe(99);
    });
});
