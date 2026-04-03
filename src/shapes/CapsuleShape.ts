import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import Edge from './Edge';
import { PolygonShape } from './PolygonShape';
import { Shape, ShapeType } from './Shape';

interface SupportResult {
    vertex: Vec2;
    index: number;
}
export class CapsuleShape extends Shape {
    halfHeight: number;
    radius: number;
    width: number;
    height: number;

    center1: Vec2;
    center2: Vec2;
    worldCenter1: Vec2;
    worldCenter2: Vec2;

    constructor(halfHeight: number, radius: number) {
        super();

        this.halfHeight = halfHeight;
        this.radius = radius;
        this.width = radius * 2;
        this.height = halfHeight * 2;

        this.center1 = new Vec2(0, halfHeight);
        this.center2 = new Vec2(0, -halfHeight);
        this.worldCenter1 = this.center1.copy();
        this.worldCenter2 = this.center2.copy();
    }

    getType(): ShapeType {
        return ShapeType.CAPSULE;
    }

    getMomentOfInertia(): number {
        // For solid capsules, the moment of inertia is the sum of the two half circles and box body inertia, accounting fot heir position
        // Still needs to be multiplied by the rigidbody's mass
        const r = this.radius;
        const l = this.halfHeight * 2;

        const areaRect = 2 * r * l;
        const areaCircle = Math.PI * r * r;
        const areaTotal = areaRect + areaCircle;

        if (areaTotal === 0) return 0;

        const mRect = areaRect / areaTotal;
        const mCircle = areaCircle / areaTotal;

        const iRect = (1 / 12) * mRect * (l * l + 4 * r * r);
        const iCircle = 0.5 * mCircle * r * r + (mCircle * (l * l)) / 4;

        return iRect + iCircle;
    }

    getTopCirclePosition(body: RigidBody): Vec2 {
        return new Vec2(0, this.halfHeight).rotate(body.rotation).addNew(body.position);
    }

    getBottomCirclePosition(body: RigidBody): Vec2 {
        return new Vec2(0, -this.halfHeight).rotate(body.rotation).addNew(body.position);
    }

    updateVertices(angle: number, position: Vec2): void {
        this.worldCenter1 = this.center1.rotate(angle).addNew(position);
        this.worldCenter2 = this.center2.rotate(angle).addNew(position);
    }

    updateAABB(body: RigidBody): void {
        const radius = this.radius;

        const topCirclePos = this.getTopCirclePosition(body);
        const bottomCirclePos = this.getBottomCirclePosition(body);

        const topCircleMinX = topCirclePos.x - radius;
        const topCircleMinY = topCirclePos.y - radius;
        const topCircleMaxX = topCirclePos.x + radius;
        const topCircleMaxY = topCirclePos.y + radius;

        const bottomCircleMinX = bottomCirclePos.x - radius;
        const bottomCircleMinY = bottomCirclePos.y - radius;
        const bottomCircleMaxX = bottomCirclePos.x + radius;
        const bottomCircleMaxY = bottomCirclePos.y + radius;

        body.minX = Math.min(topCircleMinX, bottomCircleMinX);
        body.minY = Math.min(topCircleMinY, bottomCircleMinY);
        body.maxX = Math.max(topCircleMaxX, bottomCircleMaxX);
        body.maxY = Math.max(topCircleMaxY, bottomCircleMaxY);
    }

    // TODO: to be deleted
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

    get localVertices() {
        return [this.center1, this.center2];
    }

    get worldVertices() {
        return [this.worldCenter1, this.worldCenter2];
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
}
