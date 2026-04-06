import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { Shape, ShapeType } from './Shape';

export class PolygonShape extends Shape {
    localVertices: Vec2[] = [];
    worldVertices: Vec2[] = [];
    width: number;
    height: number;

    localNormals: Vec2[] = [];
    radius = 0;

    constructor(vertices: Vec2[]) {
        super();
        let minX = Infinity;
        let minY = Infinity;
        let maxX = -Infinity;
        let maxY = -Infinity;

        // Initialize the vertices of the polygon shape and set width and height
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

        // Recenter vertices
        for (const v of vertices) {
            const centered = new Vec2(v.x - centerX, v.y - centerY);
            this.localVertices.push(centered);
            this.worldVertices.push(centered.copy());
        }

        this.computeNormals();
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
        // Loop all the vertices, transforming from local to world space
        const cos = Math.cos(angle);
        const sin = Math.sin(angle);

        for (let i = 0; i < this.localVertices.length; i++) {
            // First rotate, then we translate
            this.worldVertices[i].x = this.localVertices[i].x * cos - this.localVertices[i].y * sin;
            this.worldVertices[i].y = this.localVertices[i].x * sin + this.localVertices[i].y * cos;
            this.worldVertices[i].x += position.x;
            this.worldVertices[i].y += position.y;
        }
    }

    localEdgeAt(index: number): Vec2 {
        const currVertex = index;
        const nextVertex = (index + 1) % this.localVertices.length;
        return this.localVertices[nextVertex].subNew(this.localVertices[currVertex]);
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

    private computeNormals(): void {
        this.localNormals.length = 0;

        for (let i = 0; i < this.localVertices.length; i++) {
            this.localNormals.push(this.localEdgeAt(i).normal());
        }
    }
}
