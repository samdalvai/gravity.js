import MatMN from '../../src/physics/MatMN';
import VecN from '../../src/physics/VecN';

describe('MatMN', () => {
    test('constructor creates correct MxN matrix with zeros', () => {
        const mat = new MatMN(2, 3);
        expect(mat.M).toBe(2);
        expect(mat.N).toBe(3);
        expect(mat.rows.length).toBe(2);
        expect(mat.rows[0].data).toEqual([0, 0, 0]);
        expect(mat.rows[1].data).toEqual([0, 0, 0]);
    });

    test('assign copies matrix values', () => {
        const a = new MatMN(2, 2);
        a.rows[0].data = [1, 2];
        a.rows[1].data = [3, 4];

        const b = new MatMN();
        b.assign(a);

        expect(b.M).toBe(2);
        expect(b.N).toBe(2);
        expect(b.rows[0].data).toEqual([1, 2]);
        expect(b.rows[1].data).toEqual([3, 4]);
        expect(b.rows[0]).not.toBe(a.rows[0]); // deep copy
    });

    test('zero() sets all values to zero', () => {
        const mat = new MatMN(2, 2);
        mat.rows[0].data = [5, -2];
        mat.rows[1].data = [7, 3];

        mat.zero();
        expect(mat.rows[0].data).toEqual([0, 0]);
        expect(mat.rows[1].data).toEqual([0, 0]);
    });

    test('transpose() swaps rows and columns', () => {
        const mat = new MatMN(2, 3);
        mat.rows[0].data = [1, 2, 3];
        mat.rows[1].data = [4, 5, 6];

        const t = mat.transpose();
        expect(t.M).toBe(3);
        expect(t.N).toBe(2);
        expect(t.rows[0].data).toEqual([1, 4]);
        expect(t.rows[1].data).toEqual([2, 5]);
        expect(t.rows[2].data).toEqual([3, 6]);
    });

    test('multiplyVec() computes matrix-vector product', () => {
        const mat = new MatMN(2, 2);
        mat.rows[0].data = [1, 2];
        mat.rows[1].data = [3, 4];

        const v = new VecN(2);
        v.data = [5, 6];

        const result = mat.multiplyVec(v);
        expect(result.data).toEqual([
            1 * 5 + 2 * 6, // 17
            3 * 5 + 4 * 6, // 39
        ]);
    });

    test('multiplyMat() computes matrix-matrix product', () => {
        const a = new MatMN(2, 3);
        a.rows[0].data = [1, 2, 3];
        a.rows[1].data = [4, 5, 6];

        const b = new MatMN(3, 2);
        b.rows[0].data = [7, 8];
        b.rows[1].data = [9, 10];
        b.rows[2].data = [11, 12];

        const c = a.multiplyMat(b);
        expect(c.M).toBe(2);
        expect(c.N).toBe(2);

        // Manually computed
        expect(c.rows[0].data).toEqual([1 * 7 + 2 * 9 + 3 * 11, 1 * 8 + 2 * 10 + 3 * 12]); // [58, 64]
        expect(c.rows[1].data).toEqual([4 * 7 + 5 * 9 + 6 * 11, 4 * 8 + 5 * 10 + 6 * 12]); // [139, 154]
    });
});
