export default class Vec2 {
    x: number;
    y: number;

    constructor(x = 0, y = 0) {
        this.x = x;
        this.y = y;
    }

    add = (v: Vec2): void => {
        this.x += v.x;
        this.y += v.y;
    };

    sub = (v: Vec2): void => {
        this.x -= v.x;
        this.y -= v.y;
    };

    scale = (n: number): void => {
        this.x *= n;
        this.y *= n;
    };

    rotate = (angle: number): Vec2 => {
        const result = new Vec2();
        result.x = this.x * Math.cos(angle) - this.y * Math.sin(angle);
        result.y = this.x * Math.sin(angle) + this.y * Math.cos(angle);
        return result;
    };

    magnitude = (): number => {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    };

    magnitudeSquared = (): number => {
        return this.x * this.x + this.y * this.y;
    };

    normalize = (): Vec2 => {
        const length = this.magnitude();
        if (length != 0) {
            this.x /= length;
            this.y /= length;
        }

        return this;
    };

    unitVector = (): Vec2 => {
        const result = new Vec2();
        const length = this.magnitude();
        if (length != 0.0) {
            result.x = this.x / length;
            result.y = this.y / length;
        }
        return result;
    };

    normal = (): Vec2 => {
        return new Vec2(this.y, -this.x).normalize();
    };

    dot = (v: Vec2): number => {
        return this.x * v.x + this.y * v.y;
    };

    cross = (v: Vec2): number => {
        return this.x * v.y - this.y * v.x;
    };

    // TODO: add other operators
    // Vec2& Vec2::operator = (const Vec2& v) {
    //     x = v.x;
    //     y = v.y;
    //     return *this;
    // }

    // bool Vec2::operator == (const Vec2& v) const {
    //     return x == v.x && y == v.y;
    // }

    // bool Vec2::operator != (const Vec2& v) const {
    //     return !(*this == v);
    // }

    // Vec2 Vec2::operator + (const Vec2& v) const {
    //     Vec2 result;
    //     result.x = x + v.x;
    //     result.y = y + v.y;
    //     return result;
    // }

    // Vec2 Vec2::operator - (const Vec2& v) const {
    //     Vec2 result;
    //     result.x = x - v.x;
    //     result.y = y - v.y;
    //     return result;
    // }

    // Vec2 Vec2::operator * (const float n) const {
    //     Vec2 result;
    //     result.x = x * n;
    //     result.y = y * n;
    //     return result;
    // }

    // Vec2 Vec2::operator / (const float n) const {
    //     Vec2 result;
    //     result.x = x / n;
    //     result.y = y / n;
    //     return result;
    // }

    // Vec2& Vec2::operator += (const Vec2& v) {
    //     x += v.x;
    //     y += v.y;
    //     return *this;
    // }

    // Vec2& Vec2::operator -= (const Vec2& v) {
    //     x -= v.x;
    //     y -= v.y;
    //     return *this;
    // }

    // Vec2& Vec2::operator *= (const float n) {
    //     x *= n;
    //     y *= n;
    //     return *this;
    // }

    // Vec2& Vec2::operator /= (const float n) {
    //     x /= n;
    //     y /= n;
    //     return *this;
    // }

    // Vec2 Vec2::operator - () {
    //     Vec2 result;
    //     result.x = x * -1;
    //     result.y = y * -1;
    //     return result;
    // }
}
