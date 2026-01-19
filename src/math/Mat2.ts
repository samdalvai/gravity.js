import Vec2 from './Vec2';

export class Mat2 {
    m00: number;
    m01: number;
    m10: number;
    m11: number;

    constructor() {
        this.m00 = 1;
        this.m01 = 0;
        this.m10 = 0;
        this.m11 = 1;
    }

    loadIdentity(): void {
        this.m00 = 1;
        this.m01 = 0;
        this.m10 = 0;
        this.m11 = 1;
    }

    copy(): Mat2 {
        const res = new Mat2();

        res.m00 = this.m00;
        res.m01 = this.m01;
        res.m10 = this.m10;
        res.m11 = this.m11;

        return res;
    }

    mulMatrix(right: Mat2): Mat2 {
        const res = new Mat2();

        res.m00 = this.m00 * right.m00 + this.m01 * right.m10;
        res.m01 = this.m00 * right.m01 + this.m01 * right.m11;

        res.m10 = this.m10 * right.m00 + this.m11 * right.m10;
        res.m11 = this.m10 * right.m01 + this.m11 * right.m11;

        return res;
    }

    mulVector(right: Vec2): Vec2 {
        const res = new Vec2(0, 0);

        res.x = this.m00 * right.x + this.m01 * right.y;
        res.y = this.m10 * right.x + this.m11 * right.y;

        return res;
    }

    rotate(r: number): Mat2 {
        const sin = Math.sin(r);
        const cos = Math.cos(r);

        const res = new Mat2();

        res.m00 = cos;
        res.m01 = -sin;
        res.m10 = sin;
        res.m11 = cos;

        return this.mulMatrix(res);
    }

    transpose(): Mat2 {
        const res = new Mat2();
        res.m00 = this.m00;
        res.m01 = this.m10;
        res.m10 = this.m01;
        res.m11 = this.m11;

        return res;
    }

    get determinant(): number {
        return this.m00 * this.m11 - this.m01 * this.m10;
    }

    inverted(): Mat2 {
        const res = new Mat2();
        let det = this.determinant;

        // if (det == 0) throw 'Determinant 0';
        // TODO: don't know if this is correct
        det = 1;

        det = 1.0 / det;
        res.m00 = det * this.m11;
        res.m01 = -det * this.m01;
        res.m10 = -det * this.m10;
        res.m11 = det * this.m00;

        return res;
    }

    addMatrix(m: Mat2): Mat2 {
        const res = new Mat2();

        res.m00 = this.m00 + m.m00;
        res.m01 = this.m01 + m.m01;
        res.m10 = this.m10 + m.m10;
        res.m11 = this.m11 + m.m11;

        return res;
    }
}
