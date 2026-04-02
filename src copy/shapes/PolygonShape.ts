import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { Shape, ShapeType } from './Shape';

export class PolygonShape extends Shape {
    localVertices: Vec2[] = [];
    localNormals: Vec2[] = [];
    worldVertices: Vec2[] = [];
    width: number;
    height: number;
    radius = 0;
    protected readonly localCenterOffset: Vec2;

    constructor(vertices: Vec2[]) {
        super();
        const centerOffset = this.computeCenterOffset(vertices);
        this.localCenterOffset = centerOffset;

        let minX = Infinity;
        let minY = Infinity;
        let maxX = -Infinity;
        let maxY = -Infinity;

        // Initialize the vertices of the polygon shape and set width and height
        for (const v of vertices) {
            const centeredVertex = v.subNew(centerOffset);
            this.localVertices.push(centeredVertex);
            // Need to clone this vector, otherwise both arrays are aliasing the same vector
            this.worldVertices.push(centeredVertex.copy());

            // Find min and max X and Y to calculate polygon width and height
            minX = Math.min(minX, centeredVertex.x);
            minY = Math.min(minY, centeredVertex.y);
            maxX = Math.max(maxX, centeredVertex.x);
            maxY = Math.max(maxY, centeredVertex.y);
        }

        this.width = maxX - minX;
        this.height = maxY - minY;
        this.recomputeNormals();
    }

    getType(): ShapeType {
        return ShapeType.POLYGON;
    }

    getLocalCenterOffset(): Vec2 {
        return this.localCenterOffset.copy();
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

    private recomputeNormals(): void {
        this.localNormals.length = 0;

        for (let i = 0; i < this.localVertices.length; i++) {
            this.localNormals.push(this.localEdgeAt(i).normal());
        }
    }

    private computeCenterOffset(vertices: Vec2[]): Vec2 {
        if (vertices.length === 0) {
            return new Vec2(0, 0);
        }

        if (vertices.length === 1) {
            return vertices[0].copy();
        }

        if (vertices.length === 2) {
            return vertices[0].addNew(vertices[1]).scaleNew(0.5);
        }

        let twiceArea = 0;
        let centroidX = 0;
        let centroidY = 0;

        for (let i = 0; i < vertices.length; ++i) {
            const a = vertices[i];
            const b = vertices[(i + 1) % vertices.length];
            const cross = a.cross(b);

            twiceArea += cross;
            centroidX += (a.x + b.x) * cross;
            centroidY += (a.y + b.y) * cross;
        }

        if (Math.abs(twiceArea) <= 1.0e-8) {
            const average = new Vec2(0, 0);

            for (const vertex of vertices) {
                average.addAssign(vertex);
            }

            return average.divNew(vertices.length);
        }

        const scale = 1 / (3 * twiceArea);
        return new Vec2(centroidX * scale, centroidY * scale);
    }
}
