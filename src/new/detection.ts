import Vec2 from '../math/Vec2';
import Body from '../physics/Body';
import { PIXELS_PER_METER } from '../physics/Constants';
import { CircleShape, PolygonShape } from '../physics/Shape';
import { Edge } from './edge';

interface SupportResult {
    vertex: Vec2;
    index: number;
}

// Returns the fardest vertex in the 'dir' direction
export function support(b: Body, dir: Vec2): SupportResult {
    const shape = b.shape;
    if (shape instanceof PolygonShape) {
        let idx = 0;
        let maxValue = dir.dot(shape.localVertices[idx]);

        for (let i = 1; i < shape.localVertices.length; i++) {
            const value = dir.dot(shape.localVertices[i]);
            if (value > maxValue) {
                idx = i;
                maxValue = value;
            }
        }

        return { vertex: shape.localVertices[idx], index: idx };
    } else if (shape instanceof CircleShape) {
        return { vertex: dir.normalizeNew().scaleNew(shape.radius), index: -1 };
    } else {
        throw 'Not a supported shape';
    }
}


const TANGENT_MIN_LENGTH = 0.01 * PIXELS_PER_METER;

export function findFarthestEdge(b: Body, dir: Vec2): Edge {
    const localDir = b.worldDirToLocal(dir);
    const farthest = support(b, localDir);
    let curr = farthest.vertex;
    const idx = farthest.index;

    if (b.shape instanceof CircleShape) {
        curr = b.localPointToWorld(curr);
        const tangent = Vec2.cross(1, dir).scaleNew(TANGENT_MIN_LENGTH);

        return new Edge(curr, curr.addNew(tangent), -1);
    } else if (b.shape instanceof PolygonShape) {
        const p = b.shape as PolygonShape;

        const count = p.localVertices.length;
        const prev = p.localVertices[(idx - 1 + count) % count];
        const next = p.localVertices[(idx + 1) % count];

        const e1 = curr.subNew(prev).normalizeNew();
        const e2 = curr.subNew(next).normalizeNew();

        const w = Math.abs(e1.dot(localDir)) <= Math.abs(e2.dot(localDir));

        curr = b.localPointToWorld(curr);

        return w
            ? new Edge(b.localPointToWorld(prev), curr, (idx - 1 + count) % count, idx)
            : new Edge(curr, b.localPointToWorld(next), idx, (idx + 1) % count);
    } else {
        throw 'Not a supported shape';
    }
}

export function clipEdge(edge: Edge, p: Vec2, dir: Vec2, remove: boolean = false) {
    const d1 = edge.p1.subNew(p).dot(dir);
    const d2 = edge.p2.subNew(p).dot(dir);

    if (d1 >= 0 && d2 >= 0) return;

    const per = Math.abs(d1) + Math.abs(d2);

    if (d1 < 0) {
        if (remove) {
            edge.p1 = edge.p2;
            edge.id1 = edge.id2;
        } else {
            edge.p1 = edge.p1.addNew(edge.p2.subNew(edge.p1).scaleNew(-d1 / per));
        }
    } else if (d2 < 0) {
        if (remove) {
            edge.p2 = edge.p1;
            edge.id2 = edge.id1;
        } else {
            edge.p2 = edge.p2.addNew(edge.p1.subNew(edge.p2).scaleNew(-d2 / per));
        }
    }
}

export interface ContactPoint {
    point: Vec2;
    id: number;
}

// Since the findFarthestEdge function returns a edge with a minimum length of 0.01 for circle,
// merging threshold should be greater than sqrt(2) * minimum edge length
const CONTACT_MERGE_THRESHOLD = 1.415 * TANGENT_MIN_LENGTH;

export function findContactPoints(n: Vec2, a: Body, b: Body): ContactPoint[] {
    const edgeA = findFarthestEdge(a, n);
    const edgeB = findFarthestEdge(b, n.negated());

    let ref = edgeA; // Reference edge
    let inc = edgeB; // Incidence edge
    let flip = false;

    const aPerpendicularness = Math.abs(edgeA.dir.dot(n));
    const bPerpendicularness = Math.abs(edgeB.dir.dot(n));

    if (aPerpendicularness >= bPerpendicularness) {
        ref = edgeB;
        inc = edgeA;
        flip = true;
    }

    clipEdge(inc, ref.p1, ref.dir);
    clipEdge(inc, ref.p2, ref.dir.negated());
    clipEdge(inc, ref.p1, flip ? n : n.negated(), true);

    let contactPoints: ContactPoint[];

    // If two points are closer than threshold, merge them into one point
    if (inc.length <= CONTACT_MERGE_THRESHOLD) {
        contactPoints = [{ point: inc.p1, id: inc.id1 }];
    } else {
        contactPoints = [
            { point: inc.p1, id: inc.id1 },
            { point: inc.p2, id: inc.id2 },
        ];
    }

    return contactPoints;
}
