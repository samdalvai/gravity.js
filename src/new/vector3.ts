import * as Util from './util';

export class Vector3 {
    x: number;
    y: number;
    z: number;

    constructor(x: number = 0, y: number = 0, z: number = 0) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    clear(): void {
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }

    copy(): Vector3 {
        return new Vector3(this.x, this.y, this.z);
    }

    fix(limit = 1e-13): void {
        this.x = Util.toFixed(this.x, limit);
        this.y = Util.toFixed(this.y, limit);
        this.z = Util.toFixed(this.z, limit);
    }

    fixed(limit = 1e-13): Vector3 {
        return new Vector3(Util.toFixed(this.x, limit), Util.toFixed(this.y, limit), Util.toFixed(this.z, limit));
    }

    normalize(): void {
        const len = this.length;

        this.x /= len;
        this.y /= len;
        this.z /= len;
    }

    normalized(): Vector3 {
        const len = this.length;

        if (len != 0) return this.divNew(len);
        else return this;
    }

    invert(): void {
        this.x *= -1;
        this.y *= -1;
        this.z *= -1;
    }

    inverted(): Vector3 {
        return new Vector3(this.x * -1, this.y * -1, this.z * -1);
    }

    get length(): number {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }

    dot(v: Vector3): number {
        return this.x * v.x + this.y * v.y + this.z * v.z;
    }

    cross(v: Vector3): Vector3 {
        return new Vector3(this.y * v.z - this.z * v.y, this.z * v.x - this.x * v.z, this.x * v.y - this.y * v.x);
    }

    addNew(v: Vector3): Vector3 {
        return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z);
    }

    subNew(v: Vector3): Vector3 {
        return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z);
    }

    divNew(s: number): Vector3 {
        return new Vector3(this.x / s, this.y / s, this.z / s);
    }

    divXYZNew(x: number, y: number, z: number): Vector3 {
        return new Vector3(this.x / x, this.y / y, this.z / z);
    }

    mulNew(s: number): Vector3 {
        return new Vector3(this.x * s, this.y * s, this.z * s);
    }

    mulXYZNew(x: number, y: number, z: number): Vector3 {
        return new Vector3(this.x * x, this.y * y, this.z * z);
    }

    equals(v: Vector3): boolean {
        return this.x == v.x && this.y == v.y && this.z == v.z;
    }

    to(v: Vector3): Vector3 {
        return v.subNew(this);
    }

    unNaN(): void {
        if (isNaN(this.x) || isNaN(this.y) || isNaN(this.z)) {
            this.x = 0;
            this.y = 0;
            this.z = 0;
        }
    }
}
