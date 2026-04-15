export class Vec3 {
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

    copy(): Vec3 {
        return new Vec3(this.x, this.y, this.z);
    }

    normalize(): void {
        const len = this.length;

        this.x /= len;
        this.y /= len;
        this.z /= len;
    }

    normalized(): Vec3 {
        const len = this.length;

        if (len != 0) return this.div(len);
        else return this;
    }

    invert(): void {
        this.x *= -1;
        this.y *= -1;
        this.z *= -1;
    }

    inverted(): Vec3 {
        return new Vec3(this.x * -1, this.y * -1, this.z * -1);
    }

    get length(): number {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }

    dot(v: Vec3): number {
        return this.x * v.x + this.y * v.y + this.z * v.z;
    }

    cross(v: Vec3): Vec3 {
        return new Vec3(this.y * v.z - this.z * v.y, this.z * v.x - this.x * v.z, this.x * v.y - this.y * v.x);
    }

    add(v: Vec3): Vec3 {
        return new Vec3(this.x + v.x, this.y + v.y, this.z + v.z);
    }

    sub(v: Vec3): Vec3 {
        return new Vec3(this.x - v.x, this.y - v.y, this.z - v.z);
    }

    div(s: number): Vec3 {
        return new Vec3(this.x / s, this.y / s, this.z / s);
    }

    divXYZ(x: number, y: number, z: number): Vec3 {
        return new Vec3(this.x / x, this.y / y, this.z / z);
    }

    mul(s: number): Vec3 {
        return new Vec3(this.x * s, this.y * s, this.z * s);
    }

    mulXYZ(x: number, y: number, z: number): Vec3 {
        return new Vec3(this.x * x, this.y * y, this.z * z);
    }

    equals(v: Vec3): boolean {
        return this.x == v.x && this.y == v.y && this.z == v.z;
    }

    to(v: Vec3): Vec3 {
        return v.sub(this);
    }

    unNaN(): void {
        if (isNaN(this.x) || isNaN(this.y) || isNaN(this.z)) {
            this.x = 0;
            this.y = 0;
            this.z = 0;
        }
    }
}
