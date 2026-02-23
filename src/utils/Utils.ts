import Vec2 from '../math/Vec2';
import { PolygonShape } from '../shapes/PolygonShape';
import RigidBody from '../core/RigidBody';

export function randomNumber(min: number = 1.0, max: number = 10.0): number {
    return Math.random() * (max - min) + min;
}

// Returns a random color as a hex string, e.g. "#A3F4C2"
export function randomColor(): string {
    const r = Math.floor(Math.random() * 256);
    const g = Math.floor(Math.random() * 256);
    const b = Math.floor(Math.random() * 256);

    // Convert to hex and pad with zeros if needed
    const rHex = r.toString(16).padStart(2, '0');
    const gHex = g.toString(16).padStart(2, '0');
    const bHex = b.toString(16).padStart(2, '0');

    return `#${rHex}${gHex}${bHex}`;
}

export function clamp(value: number, low: number, high: number): number {
    return Math.max(low, Math.min(value, high));
}

export function assert(...test: boolean[]): void {
    for (let i = 0; i < test.length; i++) if (!test[i]) throw new Error('Assertion failed');
}

export function randomConvexBody(x: number, y: number, radius: number, numVertices: number = -1): RigidBody {
    if (numVertices < 3) throw Error('Must have at least 3 vertices');

    const angles: number[] = [];

    for (let i = 0; i < numVertices; i++) angles.push(Math.random() * Math.PI * 2);

    angles.sort();

    const vertices: Vec2[] = [];

    for (const angle of angles) {
        vertices.push(new Vec2(Math.cos(angle), Math.sin(angle)).scaleNew(radius));
    }

    return new RigidBody(new PolygonShape(vertices), x, y, 1);
}
