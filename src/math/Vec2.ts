export default class Vec2 {
    x: number;
    y: number;

    constructor(x = 0.0, y = 0.0) {
        this.x = x;
        this.y = y;
    }

    add(v: Vec2): void {
        this.x += v.x;
        this.y += v.y;
    }

    sub(v: Vec2): void {
        this.x -= v.x;
        this.y -= v.y;
    }

    scale(n: number): void {
        this.x *= n;
        this.y *= n;
    }

    rotate(angle: number): Vec2 {
        const result = new Vec2();
        result.x = this.x * Math.cos(angle) - this.y * Math.sin(angle);
        result.y = this.x * Math.sin(angle) + this.y * Math.cos(angle);
        return result;
    }

    magnitude(): number {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    magnitudeSquared(): number {
        return this.x * this.x + this.y * this.y;
    }

    normalize(): this {
        const length = this.magnitude();
        if (length !== 0.0) {
            this.x /= length;
            this.y /= length;
        }
        return this;
    }

    unitVector(): Vec2 {
        const result = new Vec2(0, 0);
        const length = this.magnitude();
        if (length !== 0.0) {
            result.x = this.x / length;
            result.y = this.y / length;
        }
        return result;
    }

    normal(): Vec2 {
        return new Vec2(this.y, -this.x).normalize();
    }

    dot(v: Vec2): number {
        return this.x * v.x + this.y * v.y;
    }

    cross(v: Vec2): number {
        return this.x * v.y - this.y * v.x;
    }

    /** Vector in the -90° (clockwise) perpendicular direction scaled by n */
    crossScalar(n: number): Vec2 {
        return new Vec2(-n * this.y, n * this.x);
    }

    /** operator = */
    assign(v: Vec2): this {
        this.x = v.x;
        this.y = v.y;
        return this;
    }

    /** operator == */
    equals(v: Vec2): boolean {
        return this.x === v.x && this.y === v.y;
    }

    /** operator != */
    notEquals(v: Vec2): boolean {
        return !this.equals(v);
    }

    /** operator + */
    addNew(v: Vec2): Vec2 {
        const result = new Vec2();
        result.x = this.x + v.x;
        result.y = this.y + v.y;
        return result;
    }

    /** operator - */
    subNew(v: Vec2): Vec2 {
        return new Vec2(this.x - v.x, this.y - v.y);
    }

    /** operator * (scalar) */
    scaleNew(n: number): Vec2 {
        const result = new Vec2();
        result.x = this.x * n;
        result.y = this.y * n;
        return result;
    }

    /** operator / (scalar) */
    divNew(n: number): Vec2 {
        const result = new Vec2();
        result.x = this.x / n;
        result.y = this.y / n;
        return result;
    }

    /** operator += */
    addAssign(v: Vec2): this {
        this.x += v.x;
        this.y += v.y;
        return this;
    }

    /** operator -= */
    subAssign(v: Vec2): this {
        this.x -= v.x;
        this.y -= v.y;
        return this;
    }

    /** operator *= */
    scaleAssign(n: number): this {
        this.x *= n;
        this.y *= n;
        return this;
    }

    /** operator /= */
    divAssign(n: number): this {
        this.x /= n;
        this.y /= n;
        return this;
    }

    /** operator - (unary negation) */
    negate(): Vec2 {
        const result = new Vec2();
        result.x = -this.x;
        result.y = -this.y;
        return result;
    }

    // TODO: reuse the previous methods instead of these ones, from now on methods are the ones
    // of box 2d

    /** Dot product between two vectors */
    static dot = (a: Vec2, b: Vec2): number => {
        return a.x * b.x + a.y * b.y;
    };

    /** Scalar 2D cross product */
    static cross(a: Vec2, b: Vec2): number;

    /** Vector in the +90° (counterclockwise) perpendicular direction scaled by a */
    static cross(a: Vec2, s: number): Vec2;

    /** Vector in the -90° (clockwise) perpendicular direction scaled by a */
    static cross(s: number, a: Vec2): Vec2;

    static cross(a: Vec2 | number, b: Vec2 | number): number | Vec2 {
        if (a instanceof Vec2 && b instanceof Vec2) {
            return a.x * b.y - a.y * b.x;
        }

        if (a instanceof Vec2 && typeof b === 'number') {
            return new Vec2(b * a.y, -b * a.x);
        }

        if (typeof a === 'number' && b instanceof Vec2) {
            return new Vec2(-a * b.y, a * b.x);
        }

        throw new Error('Invalid arguments');
    }

    /** Operator - */
    static sub = (a: Vec2, b: Vec2): Vec2 => {
        return new Vec2(a.x - b.x, a.y - b.y);
    };

    /** Operator * */
    static scale(s: number, v: Vec2): Vec2;
    static scale(v: Vec2, s: number): Vec2;

    static scale(a: number | Vec2, b: Vec2 | number): Vec2 {
        if (typeof a === 'number' && b instanceof Vec2) {
            return new Vec2(a * b.x, a * b.y);
        }

        if (a instanceof Vec2 && typeof b === 'number') {
            return new Vec2(b * a.x, b * a.y);
        }

        throw new Error('Invalid arguments');
    }

    static abs = (a: Vec2): Vec2 => {
        return new Vec2(Math.abs(a.x), Math.abs(a.y));
    };
}
