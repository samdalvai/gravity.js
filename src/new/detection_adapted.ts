import Vec2 from '../math/Vec2';
import Body from '../physics/Body';
import { PIXELS_PER_METER } from '../physics/Constants';
import { CircleShape, PolygonShape } from '../physics/Shape';
import { Circle } from './circle';
import { ContactManifold } from './contact_adapted';
import { Edge } from './edge_adapted';
// import { Vector2 } from './vector2';
// import { Polygon } from './polygon';
import { ClosestEdgeInfo, Polytope } from './polytope_adapted';
// import { RigidBody } from './rigidbody';
import { Settings } from './settings';
import { Simplex } from './simplex_adapted';
import * as Util from './util';

interface SupportResult {
    vertex: Vec2;
    index: number;
}

// Returns the fardest vertex in the 'dir' direction
export function support_adapted(b: Body, dir: Vec2): SupportResult {
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

/*
 * Returns support point in 'Minkowski Difference' set
 * Minkowski Sum: A ⊕ B = {Pa + Pb| Pa ∈ A, Pb ∈ B}
 * Minkowski Difference : A ⊖ B = {Pa - Pb| Pa ∈ A, Pb ∈ B}
 * CSO stands for Configuration Space Object
 */
export function csoSupport_adapted(b1: Body, b2: Body, dir: Vec2): Vec2 {
    const localDirP1 = b1.worldDirToLocal(dir);
    const localDirP2 = b2.worldDirToLocal(dir.negated());

    let supportP1 = support_adapted(b1, localDirP1).vertex;
    let supportP2 = support_adapted(b2, localDirP2).vertex;

    supportP1 = b1.localPointToWorld(supportP1);
    supportP2 = b2.localPointToWorld(supportP2);

    return supportP1.subNew(supportP2);
}

interface GJKResult {
    collide: boolean;
    simplex: Simplex;
}

export function gjk_adapted(b1: Body, b2: Body): GJKResult {
    const origin = new Vec2(0, 0);
    const simplex: Simplex = new Simplex();
    let dir = new Vec2(1, 0); // Random initial direction

    const result: GJKResult = { collide: false, simplex: simplex };

    let supportPoint = csoSupport_adapted(b1, b2, dir);
    simplex.addVertex(supportPoint);

    for (let k = 0; k < Settings.GJK_MAX_ITERATION; k++) {
        const closest = simplex.getClosest(origin);

        if (Vec2.squaredDistance(closest.result, origin) < Settings.GJK_TOLERANCE) {
            result.collide = true;
            break;
        }

        if (simplex.count != 1) {
            // Rebuild the simplex with vertices that are used(involved) to calculate closest distance
            simplex.shrink(closest.contributors);
        }

        dir = origin.subNew(closest.result);
        supportPoint = csoSupport_adapted(b1, b2, dir);

        // If the new support point is not further along the search direction than the closest point,
        // two objects are not colliding so you can early return here.
        if (dir.magnitude() > dir.normalizeNew().dot(supportPoint.subNew(closest.result))) {
            result.collide = false;
            break;
        }

        if (simplex.containsVertex(supportPoint)) {
            result.collide = false;
            break;
        } else {
            simplex.addVertex(supportPoint);
        }
    }

    result.simplex = simplex;

    // TODO: added because of triangle problem, simplex is a line if x conincide
    if (result.collide && result.simplex.vertices.length < 3) {
        expandSimplexForEPA(result.simplex, b1, b2);
    }

    return result;
}

export function expandSimplexForEPA(simplex: Simplex, b1: Body, b2: Body): void {
    if (simplex.vertices.length === 0) {
        throw new Error('Cannot expand empty simplex');
    }

    const EPS = Settings.GJK_TOLERANCE * 10;

    /* ------------------------------
       Case 1: Point simplex
       ------------------------------ */
    if (simplex.vertices.length === 1) {
        const p = simplex.vertices[0];

        // Choose two perpendicular directions
        const dirs = [new Vec2(1, 0), new Vec2(0, 1)];

        for (const d of dirs) {
            const sp = csoSupport_adapted(b1, b2, d);
            if (!simplex.containsVertex(sp)) {
                simplex.addVertex(sp);
            }
        }
    }

    /* ------------------------------
       Case 2: Line simplex
       ------------------------------ */
    if (simplex.vertices.length === 2) {
        const a = simplex.vertices[0];
        const b = simplex.vertices[1];

        const ab = b.subNew(a);

        // Perpendicular normal
        let normal = new Vec2(-ab.y, ab.x);

        // Ensure normal points toward origin
        if (normal.dot(a.negated()) < 0) {
            normal = normal.negated();
        }

        // Try primary normal
        let c = csoSupport_adapted(b1, b2, normal);

        // If degenerate, try opposite direction
        if (Math.abs(ab.cross(c.subNew(a))) < EPS || simplex.containsVertex(c)) {
            c = csoSupport_adapted(b1, b2, normal.negated());
        }

        // Still degenerate? Rotate slightly
        if (Math.abs(ab.cross(c.subNew(a))) < EPS || simplex.containsVertex(c)) {
            const rotated = new Vec2(normal.x * 0.707 - normal.y * 0.707, normal.x * 0.707 + normal.y * 0.707);
            c = csoSupport_adapted(b1, b2, rotated);
        }

        simplex.addVertex(c);
    }

    /* ------------------------------
       Final validation
       ------------------------------ */
    if (simplex.vertices.length !== 3) {
        throw new Error('Failed to expand simplex to triangle');
    }

    // Ensure CCW winding
    const v0 = simplex.vertices[0];
    const v1 = simplex.vertices[1];
    const v2 = simplex.vertices[2];

    if (v1.subNew(v0).cross(v2.subNew(v0)) < 0) {
        // Swap to enforce CCW
        simplex.vertices[1] = v2;
        simplex.vertices[2] = v1;
    }
}

interface EPAResult {
    penetrationDepth: number;
    contactNormal: Vec2;
}

export function epa_adapted(b1: Body, b2: Body, gjkResult: Simplex): EPAResult {
    const polytope: Polytope = new Polytope(gjkResult);

    let closestEdge: ClosestEdgeInfo = { index: 0, distance: Infinity, normal: new Vec2(0, 0) };

    for (let i = 0; i < Settings.EPA_MAX_ITERATION; i++) {
        closestEdge = polytope.getClosestEdge();
        const supportPoint = csoSupport_adapted(b1, b2, closestEdge.normal);
        const newDistance = closestEdge.normal.dot(supportPoint);

        if (Math.abs(closestEdge.distance - newDistance) > Settings.EPA_TOLERANCE) {
            // Insert the support vertex so that it expands our polytope
            polytope.vertices.splice(closestEdge.index + 1, 0, supportPoint);
        } else {
            // If you didn't expand edge, it means you reached the closest outer edge
            break;
        }
    }

    return {
        penetrationDepth: closestEdge.distance,
        contactNormal: closestEdge.normal,
    };
}

const TANGENT_MIN_LENGTH = 0.01// * PIXELS_PER_METER;

export function findFarthestEdge_adapted(b: Body, dir: Vec2): Edge {
    const localDir = b.worldDirToLocal(dir);
    const farthest = support_adapted(b, localDir);
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

export function findContactPoints_adapted(n: Vec2, a: Body, b: Body): ContactPoint[] {
    const edgeA = findFarthestEdge_adapted(a, n);
    const edgeB = findFarthestEdge_adapted(b, n.negated());

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

// Returns contact data if collide, otherwise returns null
export function detectCollision_adapted(a: Body, b: Body): ContactManifold | null {
    // Circle vs. Circle collision
    if (a.shape instanceof CircleShape && b.shape instanceof CircleShape) {
        let d = Vec2.squaredDistance(a.position, b.position);
        const r2 = a.shape.radius + b.shape.radius;

        if (d > r2 * r2) {
            return null;
        } else {
            d = Math.sqrt(d);

            const contactNormal = b.position.subNew(a.position).normalizeNew();
            const contactPoint = a.position.addNew(contactNormal.scaleNew(a.shape.radius));
            const penetrationDepth = r2 - d;

            let flipped = false;
            if (contactNormal.dot(new Vec2(0, -1)) < 0) {
                const tmp = a;
                a = b;
                b = tmp;
                contactNormal.negate();
                flipped = true;
            }

            const contact = new ContactManifold(
                a,
                b,
                [{ point: contactPoint, id: -1 }],
                penetrationDepth,
                contactNormal,
                flipped,
            );

            return contact;
        }
    }

    const gjkResult = gjk_adapted(a, b);

    if (!gjkResult.collide) {
        return null;
    } else {
        // If the gjk termination simplex has vertices less than 3, expand to full simplex
        // Because EPA needs a full n-simplex to get started
        const simplex = gjkResult.simplex;

        switch (simplex.count) {
            case 1:
                {
                    const v = simplex.vertices[0];
                    let randomSupport = csoSupport_adapted(a, b, new Vec2(1, 0));

                    if (randomSupport.equals(v)) randomSupport = csoSupport_adapted(a, b, new Vec2(-1, 0));

                    simplex.addVertex(randomSupport);
                }
                break;
            case 2:
                {
                    const e = new Edge(simplex.vertices[0], simplex.vertices[1]);
                    const normalSupport = csoSupport_adapted(a, b, e.normal);

                    if (simplex.containsVertex(normalSupport))
                        simplex.addVertex(csoSupport_adapted(a, b, e.normal.negated()));
                    else simplex.addVertex(normalSupport);
                }
                break;
        }

        const epaResult: EPAResult = epa_adapted(a, b, gjkResult.simplex);

        let flipped = false;
        // Apply axis weight to improve coherence
        if (epaResult.contactNormal.dot(new Vec2(0, -1)) < 0) {
            const tmp = a;
            a = b;
            b = tmp;
            epaResult.contactNormal.negate();
            flipped = true;
        }

        // Remove floating point error
        epaResult.contactNormal.fix(Settings.EPA_TOLERANCE);

        const contactPoints = findContactPoints_adapted(epaResult.contactNormal, a, b);

        const contact = new ContactManifold(
            a,
            b,
            contactPoints,
            epaResult.penetrationDepth,
            epaResult.contactNormal,
            flipped,
        );

        return contact;
    }
}

// export function testPointInside(body: RigidBody, point: Vector2): boolean {
//     const localP = body.globalToLocal.mulVector2(point, 1);

//     if (body instanceof Circle) {
//         return localP.length <= (body as Circle).radius;
//     } else if (body instanceof Polygon) {
//         const poly = body as Polygon;

//         const dir = poly.vertices[0].subNew(localP).cross(poly.vertices[1].subNew(localP));

//         for (let i = 1; i < poly.vertices.length; i++) {
//             const nDir = poly.vertices[i].subNew(localP).cross(poly.vertices[(i + 1) % poly.count].subNew(localP));
//             if (dir * nDir < 0) return false;
//         }
//         return true;
//     } else {
//         throw 'Not a supported shape';
//     }
// }
