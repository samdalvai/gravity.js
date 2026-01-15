import { Box } from './box';
import { Circle } from './circle';
import { Matrix3 } from './math/matrix3';
import { Vector2 } from './math/vector2';
import { Polygon } from './polygon';
import { RigidBody, Type } from './rigidbody';
import { Settings } from './settings';

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
export function getUV(a: Vector2, b: Vector2, p: Vector2): UV {
    const dir = b.subNew(a);
    const len = dir.length;
    dir.normalize();

    const region = dir.dot(p.subNew(a)) / len;

    return { u: 1 - region, v: region };
}

// Linearly combine(interpolate) the vector using weights u, v
export function lerpVector(a: Vector2, b: Vector2, uv: UV): Vector2 {
    // return a.mul(uv.u).add(b.mul(uv.v));
    return new Vector2(a.x * uv.u + b.x * uv.v, a.y * uv.u + b.y * uv.v);
}

export function createRandomConvexBody(
    radius: number,
    numVertices: number = -1,
    density: number = Settings.defaultDensity,
): RigidBody {
    if (numVertices < 0) numVertices = Math.trunc(Math.random() * Settings.randomConvexMaxVertices);

    if (numVertices == 0) return new Circle(radius);

    if (numVertices == Settings.randomConvexMaxVertices - 1) return new Box(radius * 2, radius * 2);

    numVertices += 2;

    const angles: number[] = [];

    for (let i = 0; i < numVertices; i++) angles.push(Math.random() * Math.PI * 2);

    angles.sort();

    const res = new Polygon(
        angles.map(angle => {
            return new Vector2(Math.cos(angle), Math.sin(angle)).mulNew(radius);
        }),
        Type.Dynamic,
        undefined,
        density,
    );

    return res;
}

export function createRegularPolygon(
    radius: number,
    numVertices: number = -1,
    initialAngle?: number,
    density: number = Settings.defaultDensity,
): Polygon {
    if (numVertices < 3) numVertices = Math.trunc(random(3, Settings.regularPolygonMaxVertices));

    let angleStart = initialAngle != undefined ? initialAngle : Math.PI / 2.0;
    const angle = (Math.PI * 2) / numVertices;
    if (numVertices % 2 == 0) angleStart += angle / 2.0;

    const vertices: Vector2[] = [];

    for (let i = 0; i < numVertices; i++) {
        const currentAngle = angleStart + angle * i;
        vertices.push(new Vector2(Math.cos(currentAngle), Math.sin(currentAngle)).mulNew(radius * 1.4142));
    }

    return new Polygon(vertices, Type.Dynamic, true, density);
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

export function cross(scalar: number, vector: Vector2): Vector2 {
    return new Vector2(-scalar * vector.y, scalar * vector.x);
}

export function calculateBoxInertia(width: number, height: number, mass: number): number {
    return ((width * width + height * height) * mass) / 12.0;
}

export function calculateCircleInertia(radius: number, mass: number): number {
    return (mass * radius * radius) / 2.0;
}

// https://en.wikipedia.org/wiki/List_of_moments_of_inertia
// TODO: mass and area are not considered, why? This was the original signature
// export function calculateConvexPolygonInertia(vertices: Vector2[], mass: number, area: number = -1): number
export function calculateConvexPolygonInertia(vertices: Vector2[]): number {
    let inertia = 0;

    const vertexCount = vertices.length;

    let numerator = 0.0;
    let denominator = 0.0;

    let i0 = vertexCount - 1;
    for (let i1 = 0; i1 < vertexCount; ++i1) {
        const v0 = vertices[i0];
        const v1 = vertices[i1];

        const crs = Math.abs(v1.cross(v0));

        numerator += crs * (v1.dot(v1) + v1.dot(v0) + v0.dot(v0));
        denominator += crs;

        i0 = i1;
    }

    inertia = numerator / (denominator * 6.0);

    return inertia;
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

export function squared_distance(a: Vector2, b: Vector2): number {
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

export function map(v: number, left: number, right: number, min: number, max: number): number {
    const per = (v - left) / (right - left);
    return lerp(min, max, per);
}

export function lerp(left: number, right: number, per: number): number {
    return left + (right - left) * per;
}

export function mid(a: Vector2, b: Vector2): Vector2 {
    return new Vector2((a.x + b.x) / 2.0, (a.y + b.y) / 2.0);
}

// Create a 2D orthographic projection matrix
export function orth(left: number, right: number, bottom: number, top: number): Matrix3 {
    const res = new Matrix3();

    // Scale
    res.m00 = 2.0 / (right - left);
    res.m11 = 2.0 / (top - bottom);

    // Translation
    res.m02 = -(right + left) / (right - left);
    res.m12 = -(top + bottom) / (top - bottom);

    return res;
}

// Create a viewport transform matrix
export function viewport(width: number, height: number, xStart = 0, yStart = 0): Matrix3 {
    const res = new Matrix3();

    // Scale
    res.m00 = width / 2.0;
    res.m11 = height / 2.0;

    // Translation
    res.m02 = xStart + width / 2.0;
    res.m12 = yStart + height / 2.0;

    return res;
}

export function assert(...test: boolean[]): void {
    for (let i = 0; i < test.length; i++) if (!test[i]) throw new Error('Assertion failed');
}
