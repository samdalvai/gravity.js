import Vec2 from '../math/Vec2';
import Edge from './Edge';
import RigidBody from '../core/RigidBody';
import { Shape, ShapeType } from './Shape';

interface SupportResult {
    vertex: Vec2;
    index: number;
}
export class PolygonShape extends Shape {
    localVertices: Vec2[] = [];
    worldVertices: Vec2[] = [];
    width: number;
    height: number;

    constructor(vertices: Vec2[]) {
        super();
        let minX = Infinity;
        let minY = Infinity;
        let maxX = -Infinity;
        let maxY = -Infinity;

        // Initialize the vertices of the polygon shape and set width and height
        for (const v of vertices) {
            this.localVertices.push(v);
            // Need to clone this vector, otherwise both arrays are aliasing the same vector
            this.worldVertices.push(v.copy());

            // Find min and max X and Y to calculate polygon width and height
            minX = Math.min(minX, v.x);
            minY = Math.min(minY, v.y);
            maxX = Math.max(maxX, v.x);
            maxY = Math.max(maxY, v.y);
        }

        this.width = maxX - minX;
        this.height = maxY - minY;
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

    edgeAt(index: number): Vec2 {
        const currVertex = index;
        const nextVertex = (index + 1) % this.worldVertices.length;
        return this.worldVertices[nextVertex].subNew(this.worldVertices[currVertex]);
    }

    findMinSeparation(other: PolygonShape): [number, number] {
        let separation = -Infinity;
        let indexReferenceEdge = 0;

        // Loop all the vertices of "this" polygon
        for (let i = 0; i < this.worldVertices.length; i++) {
            const va = this.worldVertices[i];
            const normal = this.edgeAt(i).normal();

            // Loop all the vertices of the "other" polygon
            let minSep = Infinity;

            for (let j = 0; j < other.worldVertices.length; j++) {
                const vb = other.worldVertices[j];
                const proj = vb.subNew(va).dot(normal);
                if (proj < minSep) {
                    minSep = proj;
                }
            }

            if (minSep > separation) {
                separation = minSep;
                indexReferenceEdge = i;
            }
        }
        return [separation, indexReferenceEdge];
    }

    support(dir: Vec2): SupportResult {
        let idx = 0;
        let maxValue = dir.dot(this.localVertices[idx]);

        for (let i = 1; i < this.localVertices.length; i++) {
            const value = dir.dot(this.localVertices[i]);
            if (value > maxValue) {
                idx = i;
                maxValue = value;
            }
        }

        return { vertex: this.localVertices[idx], index: idx };
    }

    findFarthestEdge(b: RigidBody, dir: Vec2): Edge {
        const localDir = b.worldDirToLocal(dir);
        const farthest = this.support(localDir);
        let curr = farthest.vertex;
        const idx = farthest.index;

        const count = this.localVertices.length;
        const prev = this.localVertices[(idx - 1 + count) % count];
        const next = this.localVertices[(idx + 1) % count];

        const e1 = curr.subNew(prev).normalizeNew();
        const e2 = curr.subNew(next).normalizeNew();

        const w = Math.abs(e1.dot(localDir)) <= Math.abs(e2.dot(localDir));

        curr = b.localPointToWorld(curr);

        return w
            ? new Edge(b.localPointToWorld(prev), curr, (idx - 1 + count) % count, idx)
            : new Edge(curr, b.localPointToWorld(next), idx, (idx + 1) % count);
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
}
