import { Vec2 } from './Vec2';
import { Vec3 } from './Vec3';

export class Mat3 {
    m00: number;
    m01: number;
    m02: number;
    m10: number;
    m11: number;
    m12: number;
    m20: number;
    m21: number;
    m22: number;

    constructor() {
        this.m00 = 1;
        this.m01 = 0;
        this.m02 = 0;
        this.m10 = 0;
        this.m11 = 1;
        this.m12 = 0;
        this.m20 = 0;
        this.m21 = 0;
        this.m22 = 1;
    }

    loadIdentity(): void {
        this.m00 = 1;
        this.m01 = 0;
        this.m02 = 0;
        this.m10 = 0;
        this.m11 = 1;
        this.m12 = 0;
        this.m20 = 0;
        this.m21 = 0;
        this.m22 = 1;
    }

    copy(): Mat3 {
        const res = new Mat3();

        res.m00 = this.m00;
        res.m01 = this.m01;
        res.m02 = this.m02;
        res.m10 = this.m10;
        res.m11 = this.m11;
        res.m12 = this.m12;
        res.m20 = this.m20;
        res.m21 = this.m21;
        res.m22 = this.m22;

        return res;
    }

    mulMatrix(right: Mat3): Mat3 {
        const res = new Mat3();

        res.m00 = this.m00 * right.m00 + this.m01 * right.m10 + this.m02 * right.m20;
        res.m01 = this.m00 * right.m01 + this.m01 * right.m11 + this.m02 * right.m21;
        res.m02 = this.m00 * right.m02 + this.m01 * right.m12 + this.m02 * right.m22;

        res.m10 = this.m10 * right.m00 + this.m11 * right.m10 + this.m12 * right.m20;
        res.m11 = this.m10 * right.m01 + this.m11 * right.m11 + this.m12 * right.m21;
        res.m12 = this.m10 * right.m02 + this.m11 * right.m12 + this.m12 * right.m22;

        res.m20 = this.m20 * right.m00 + this.m21 * right.m10 + this.m22 * right.m20;
        res.m21 = this.m20 * right.m01 + this.m21 * right.m11 + this.m22 * right.m21;
        res.m22 = this.m20 * right.m02 + this.m21 * right.m12 + this.m22 * right.m22;

        return res;
    }

    mulVector2(right: Vec2, z: number): Vec2 {
        const res = new Vec2(0, 0);

        res.x = this.m00 * right.x + this.m01 * right.y + this.m02 * z;
        res.y = this.m10 * right.x + this.m11 * right.y + this.m12 * z;

        return res;
    }

    mulVector3(right: Vec3): Vec3 {
        const res = new Vec3(0, 0, 0);

        res.x = this.m00 * right.x + this.m01 * right.y + this.m02 * right.z;
        res.y = this.m10 * right.x + this.m11 * right.y + this.m12 * right.z;
        res.z = this.m20 * right.x + this.m21 * right.y + this.m22 * right.z;

        return res;
    }

    scale(x: number, y: number): Mat3 {
        const scale = new Mat3();
        scale.m00 = x;
        scale.m11 = y;

        return this.mulMatrix(scale);
    }

    rotate(r: number): Mat3 {
        const sin = Math.sin(r);
        const cos = Math.cos(r);

        const res = new Mat3();

        res.m00 = cos;
        res.m01 = -sin;
        res.m10 = sin;
        res.m11 = cos;

        return this.mulMatrix(res);
    }

    translate(x: number, y: number): Mat3 {
        const res = new Mat3();

        res.m02 = x;
        res.m12 = y;

        return this.mulMatrix(res);
    }

    inverted(): Mat3 {
        const res = new Mat3();

        const det =
            this.m00 * (this.m11 * this.m22 - this.m21 * this.m12) -
            this.m01 * (this.m10 * this.m22 - this.m12 * this.m20) +
            this.m02 * (this.m10 * this.m21 - this.m11 * this.m20);

        if (det == 0) throw 'Determinant 0';
        const inv_det = 1.0 / det;

        res.m00 = (this.m11 * this.m22 - this.m21 * this.m12) * inv_det;
        res.m01 = (this.m02 * this.m21 - this.m01 * this.m22) * inv_det;
        res.m02 = (this.m01 * this.m12 - this.m02 * this.m11) * inv_det;
        res.m10 = (this.m12 * this.m20 - this.m10 * this.m22) * inv_det;
        res.m11 = (this.m00 * this.m22 - this.m02 * this.m20) * inv_det;
        res.m12 = (this.m10 * this.m02 - this.m00 * this.m12) * inv_det;
        res.m20 = (this.m10 * this.m21 - this.m20 * this.m11) * inv_det;
        res.m21 = (this.m20 * this.m01 - this.m00 * this.m21) * inv_det;
        res.m22 = (this.m00 * this.m11 - this.m10 * this.m01) * inv_det;

        return res;
    }
}
