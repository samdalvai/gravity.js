import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { Shape, ShapeType } from './Shape';

export class PolygonShape extends Shape {
    localVertices: Vec2[] = [];
    worldVertices: Vec2[] = [];

    localNormals: Vec2[] = [];
    worldNormals: Vec2[] = [];

    width: number;
    height: number;

    constructor(vertices: Vec2[]) {
        super();

        let minX = Infinity;
        let minY = Infinity;
        let maxX = -Infinity;
        let maxY = -Infinity;

        // Compute bounds
        for (const v of vertices) {
            minX = Math.min(minX, v.x);
            minY = Math.min(minY, v.y);
            maxX = Math.max(maxX, v.x);
            maxY = Math.max(maxY, v.y);
        }

        this.width = maxX - minX;
        this.height = maxY - minY;

        const centerX = (minX + maxX) * 0.5;
        const centerY = (minY + maxY) * 0.5;

        // Recenter vertices and initialize world vertices
        for (const v of vertices) {
            const centered = new Vec2(v.x - centerX, v.y - centerY);
            this.localVertices.push(centered);
            this.worldVertices.push(centered.copy());
        }

        for (let i = 0; i < this.localVertices.length; i++) {
            this.localNormals.push(this.localEdgeAt(i).normal());
        }

        // Initialize cached world normals
        for (let i = 0; i < this.localNormals.length; i++) {
            this.worldNormals.push(this.localNormals[i].copy());
        }
    }

    getType(): ShapeType {
        return ShapeType.POLYGON;
    }

    getMomentOfInertia(): number {
        let acc0 = 0;
        let acc1 = 0;

        for (let i = 0; i < this.localVertices.length; i++) {
            const a = this.localVertices[i];
            const b = this.localVertices[(i + 1) % this.localVertices.length];

            const cross = Math.abs(a.cross(b));
            acc0 += cross * (a.dot(a) + b.dot(b) + a.dot(b));
            acc1 += cross;
        }

        return acc0 / 6 / acc1;
    }

    updateVertices(angle: number, position: Vec2): void {
        const cos = Math.cos(angle);
        const sin = Math.sin(angle);

        // Update world vertices
        for (let i = 0; i < this.localVertices.length; i++) {
            const localVertex = this.localVertices[i];
            const worldVertex = this.worldVertices[i];

            worldVertex.x = localVertex.x * cos - localVertex.y * sin + position.x;
            worldVertex.y = localVertex.x * sin + localVertex.y * cos + position.y;
        }

        // Update cached world normals
        for (let i = 0; i < this.localNormals.length; i++) {
            const localNormal = this.localNormals[i];
            const worldNormal = this.worldNormals[i];

            worldNormal.x = localNormal.x * cos - localNormal.y * sin;
            worldNormal.y = localNormal.x * sin + localNormal.y * cos;
        }
    }

    updateAABB(body: RigidBody): void {
        let minX = Infinity;
        let minY = Infinity;
        let maxX = -Infinity;
        let maxY = -Infinity;

        for (const v of this.worldVertices) {
            minX = Math.min(minX, v.x);
            minY = Math.min(minY, v.y);
            maxX = Math.max(maxX, v.x);
            maxY = Math.max(maxY, v.y);
        }

        body.minX = minX;
        body.maxX = maxX;
        body.minY = minY;
        body.maxY = maxY;
    }

    private localEdgeAt(index: number): Vec2 {
        const currVertex = index;
        const nextVertex = (index + 1) % this.localVertices.length;
        return this.localVertices[nextVertex].subNew(this.localVertices[currVertex]);
    }
}
