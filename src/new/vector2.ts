import * as Util from './util';

export class Vector2 {
    x: number;
    y: number;

    constructor(x: number = 0, y: number = 0) {
        this.x = x;
        this.y = y;
    }

    clear(): void {
        this.x = 0;
        this.y = 0;
    }

    copy(): Vector2 {
        return new Vector2(this.x, this.y);
    }

    fix(limit = 1e-13): void {
        this.x = Util.toFixed(this.x, limit);
        this.y = Util.toFixed(this.y, limit);
    }

    fixed(limit = 1e-13): Vector2 {
        return new Vector2(Util.toFixed(this.x, limit), Util.toFixed(this.y, limit));
    }

    invert(): void {
        this.x *= -1;
        this.y *= -1;
    }

    inverted(): Vector2 {
        return new Vector2(this.x * -1, this.y * -1);
    }

    normalize(): void {
        const len = this.length;

        if (len != 0) {
            this.x /= len;
            this.y /= len;
        }
    }

    normalized(): Vector2 {
        const len = this.length;

        if (len != 0) return this.divNew(len);
        else return this;
    }

    get squaredLength(): number {
        return this.x * this.x + this.y * this.y;
    }

    get length(): number {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    dot(v: Vector2): number {
        return this.x * v.x + this.y * v.y;
    }

    cross(v: Vector2): number {
        return this.x * v.y - this.y * v.x;
    }

    addNew(v: Vector2): Vector2 {
        return new Vector2(this.x + v.x, this.y + v.y);
    }

    subNew(v: Vector2): Vector2 {
        return new Vector2(this.x - v.x, this.y - v.y);
    }

    divNew(v: number): Vector2 {
        return new Vector2(this.x / v, this.y / v);
    }

    mulNew(v: number): Vector2 {
        return new Vector2(this.x * v, this.y * v);
    }

    equals(v: Vector2): boolean {
        return this.x == v.x && this.y == v.y;
    }

    to(v: Vector2): Vector2 {
        return v.subNew(this);
    }

    unNaN(): void {
        if (isNaN(this.x) || isNaN(this.y)) {
            this.x = 0;
            this.y = 0;
        }
    }
}
