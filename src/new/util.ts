import Vec2 from '../math/Vec2';

export interface Pair<A, B> {
    p1: A;
    p2: B;
}

export function toFixed(value: number, limit = 1e-13): number {
    return Math.round(value / limit) * limit;
}

export interface UV {
    u: number;
    v: number;
}

// Project point p to edge ab, calculate barycentric weights and return it
export function getUV(a: Vec2, b: Vec2, p: Vec2): UV {
    const dir = b.subNew(a);
    const len = dir.magnitude();
    dir.normalize();

    const region = dir.dot(p.subNew(a)) / len;

    return { u: 1 - region, v: region };
}

// Linearly combine(interpolate) the vector using weights u, v
export function lerpVector(a: Vec2, b: Vec2, uv: UV): Vec2 {
    // return a.mul(uv.u).add(b.mul(uv.v));
    return new Vec2(a.x * uv.u + b.x * uv.v, a.y * uv.u + b.y * uv.v);
}

export function random(left: number = -1, right: number = 1): number {
    if (left > right) {
        const tmp = right;
        right = left;
        left = tmp;
    }

    const range = right - left;
    return Math.random() * range + left;
}

export function clamp(value: number, min: number, max: number): number {
    // return Math.max(min, Math.min(value, max));

    if (value < min) return min;
    else if (value > max) return max;
    else return value;
}

// export function cross(scalar: number, vector: Vector2): Vector2 {
//     return new Vector2(-scalar * vector.y, scalar * vector.x);
// }

export function calculateBoxInertia(width: number, height: number, mass: number): number {
    return ((width * width + height * height) * mass) / 12.0;
}

export function calculateCircleInertia(radius: number, mass: number): number {
    return (mass * radius * radius) / 2.0;
}

// Cantor pairing function, ((N, N) -> N) mapping function
// https://en.wikipedia.org/wiki/Pairing_function#Cantor_pairing_function
export function make_pair_natural(a: number, b: number): number {
    return ((a + b) * (a + b + 1)) / 2 + b;
}

// Reverse version of pairing function
// this guarantees initial pairing order
export function separate_pair(p: number): Pair<number, number> {
    const w = Math.floor((Math.sqrt(8 * p + 1) - 1) / 2.0);
    const t = (w * w + w) / 2.0;

    const y = p - t;
    const x = w - y;

    return { p1: x, p2: y };
}

export function map(v: number, left: number, right: number, min: number, max: number): number {
    const per = (v - left) / (right - left);
    return lerp(min, max, per);
}

export function lerp(left: number, right: number, per: number): number {
    return left + (right - left) * per;
}

export function assert(...test: boolean[]): void {
    for (let i = 0; i < test.length; i++) if (!test[i]) throw new Error('Assertion failed');
}
