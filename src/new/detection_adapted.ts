import Vec2 from '../math/Vec2';
import Body from '../physics/Body';
import { CircleShape, PolygonShape } from '../physics/Shape';
import { Circle } from './circle';
import { ContactManifold } from './constraint/contact';
import { Edge } from './edge';
import { Vector2 } from './math/vector2';
import { Polygon } from './polygon';
import { ClosestEdgeInfo, Polytope } from './polytope';
import { RigidBody } from './rigidbody';
import { Settings } from './settings';
import { Simplex } from './simplex';
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
    const localDirP2 = b2.worldDirToLocal(dir.negate());

    let supportP1 = support_adapted(b1, localDirP1).vertex;
    let supportP2 = support_adapted(b2, localDirP2).vertex;

    supportP1 = b1.localDirToWorld(supportP1);
    supportP2 = b2.localDirToWorld(supportP2);

    return supportP1.subNew(supportP2);
}

// interface GJKResult {
//     collide: boolean;
//     simplex: Simplex;
// }

// function gjk(b1: RigidBody, b2: RigidBody): GJKResult {
//     const origin = new Vector2(0, 0);
//     const simplex: Simplex = new Simplex();
//     let dir = new Vector2(1, 0); // Random initial direction

//     const result: GJKResult = { collide: false, simplex: simplex };

//     let supportPoint = csoSupport(b1, b2, dir);
//     simplex.addVertex(supportPoint);

//     for (let k = 0; k < Settings.GJK_MAX_ITERATION; k++) {
//         const closest = simplex.getClosest(origin);

//         if (Util.squared_distance(closest.result, origin) < Settings.GJK_TOLERANCE) {
//             result.collide = true;
//             break;
//         }

//         if (simplex.count != 1) {
//             // Rebuild the simplex with vertices that are used(involved) to calculate closest distance
//             simplex.shrink(closest.contributors);
//         }

//         dir = origin.subNew(closest.result);
//         supportPoint = csoSupport(b1, b2, dir);

//         // If the new support point is not further along the search direction than the closest point,
//         // two objects are not colliding so you can early return here.
//         if (dir.length > dir.normalized().dot(supportPoint.subNew(closest.result))) {
//             result.collide = false;
//             break;
//         }

//         if (simplex.containsVertex(supportPoint)) {
//             result.collide = false;
//             break;
//         } else {
//             simplex.addVertex(supportPoint);
//         }
//     }

//     result.simplex = simplex;

//     return result;
// }

// interface EPAResult {
//     penetrationDepth: number;
//     contactNormal: Vector2;
// }

// function epa(b1: RigidBody, b2: RigidBody, gjkResult: Simplex): EPAResult {
//     const polytope: Polytope = new Polytope(gjkResult);

//     let closestEdge: ClosestEdgeInfo = { index: 0, distance: Infinity, normal: new Vector2(0, 0) };

//     for (let i = 0; i < Settings.EPA_MAX_ITERATION; i++) {
//         closestEdge = polytope.getClosestEdge();
//         const supportPoint = csoSupport(b1, b2, closestEdge.normal);
//         const newDistance = closestEdge.normal.dot(supportPoint);

//         if (Math.abs(closestEdge.distance - newDistance) > Settings.EPA_TOLERANCE) {
//             // Insert the support vertex so that it expands our polytope
//             polytope.vertices.splice(closestEdge.index + 1, 0, supportPoint);
//         } else {
//             // If you didn't expand edge, it means you reached the closest outer edge
//             break;
//         }
//     }

//     return {
//         penetrationDepth: closestEdge.distance,
//         contactNormal: closestEdge.normal,
//     };
// }

// const TANGENT_MIN_LENGTH = 0.01;

// function findFarthestEdge(b: RigidBody, dir: Vector2): Edge {
//     const localDir = b.globalToLocal.mulVector2(dir, 0);
//     const farthest = support_adapted(b, localDir);
//     let curr = farthest.vertex;
//     const idx = farthest.index;

//     const localToGlobal = b.localToGlobal;

//     if (b instanceof Circle) {
//         curr = localToGlobal.mulVector2(curr, 1);
//         const tangent = Util.cross(1, dir).mulNew(TANGENT_MIN_LENGTH);

//         return new Edge(curr, curr.addNew(tangent), -1);
//     } else if (b instanceof Polygon) {
//         const p = b as Polygon;

//         const prev = p.vertices[(idx - 1 + p.count) % p.count];
//         const next = p.vertices[(idx + 1) % p.count];

//         const e1 = curr.subNew(prev).normalized();
//         const e2 = curr.subNew(next).normalized();

//         const w = Math.abs(e1.dot(localDir)) <= Math.abs(e2.dot(localDir));

//         curr = localToGlobal.mulVector2(curr, 1);

//         return w
//             ? new Edge(localToGlobal.mulVector2(prev, 1), curr, (idx - 1 + p.count) % p.count, idx)
//             : new Edge(curr, localToGlobal.mulVector2(next, 1), idx, (idx + 1) % p.count);
//     } else {
//         throw 'Not a supported shape';
//     }
// }

// function clipEdge(edge: Edge, p: Vector2, dir: Vector2, remove: boolean = false) {
//     const d1 = edge.p1.subNew(p).dot(dir);
//     const d2 = edge.p2.subNew(p).dot(dir);

//     if (d1 >= 0 && d2 >= 0) return;

//     const per = Math.abs(d1) + Math.abs(d2);

//     if (d1 < 0) {
//         if (remove) {
//             edge.p1 = edge.p2;
//             edge.id1 = edge.id2;
//         } else {
//             edge.p1 = edge.p1.addNew(edge.p2.subNew(edge.p1).mulNew(-d1 / per));
//         }
//     } else if (d2 < 0) {
//         if (remove) {
//             edge.p2 = edge.p1;
//             edge.id2 = edge.id1;
//         } else {
//             edge.p2 = edge.p2.addNew(edge.p1.subNew(edge.p2).mulNew(-d2 / per));
//         }
//     }
// }

// export interface ContactPoint {
//     point: Vector2;
//     id: number;
// }

// // Since the findFarthestEdge function returns a edge with a minimum length of 0.01 for circle,
// // merging threshold should be greater than sqrt(2) * minimum edge length
// const CONTACT_MERGE_THRESHOLD = 1.415 * TANGENT_MIN_LENGTH;

// function findContactPoints(n: Vector2, a: RigidBody, b: RigidBody): ContactPoint[] {
//     const edgeA = findFarthestEdge(a, n);
//     const edgeB = findFarthestEdge(b, n.inverted());

//     let ref = edgeA; // Reference edge
//     let inc = edgeB; // Incidence edge
//     let flip = false;

//     const aPerpendicularness = Math.abs(edgeA.dir.dot(n));
//     const bPerpendicularness = Math.abs(edgeB.dir.dot(n));

//     if (aPerpendicularness >= bPerpendicularness) {
//         ref = edgeB;
//         inc = edgeA;
//         flip = true;
//     }

//     clipEdge(inc, ref.p1, ref.dir);
//     clipEdge(inc, ref.p2, ref.dir.inverted());
//     clipEdge(inc, ref.p1, flip ? n : n.inverted(), true);

//     let contactPoints: ContactPoint[];

//     // If two points are closer than threshold, merge them into one point
//     if (inc.length <= CONTACT_MERGE_THRESHOLD) {
//         contactPoints = [{ point: inc.p1, id: inc.id1 }];
//     } else {
//         contactPoints = [
//             { point: inc.p1, id: inc.id1 },
//             { point: inc.p2, id: inc.id2 },
//         ];
//     }

//     return contactPoints;
// }

// // Returns contact data if collide, otherwise returns null
// export function detectCollision(a: RigidBody, b: RigidBody): ContactManifold | null {
//     // Circle vs. Circle collision
//     if (a instanceof Circle && b instanceof Circle) {
//         let d = Util.squared_distance(a.position, b.position);
//         const r2 = a.radius + b.radius;

//         if (d > r2 * r2) {
//             return null;
//         } else {
//             d = Math.sqrt(d);

//             const contactNormal = b.position.subNew(a.position).normalized();
//             const contactPoint = a.position.addNew(contactNormal.mulNew(a.radius));
//             const penetrationDepth = r2 - d;

//             let flipped = false;
//             if (contactNormal.dot(new Vector2(0, -1)) < 0) {
//                 const tmp = a;
//                 a = b;
//                 b = tmp;
//                 contactNormal.invert();
//                 flipped = true;
//             }

//             const contact = new ContactManifold(
//                 a,
//                 b,
//                 [{ point: contactPoint, id: -1 }],
//                 penetrationDepth,
//                 contactNormal,
//                 flipped,
//             );

//             return contact;
//         }
//     }

//     const gjkResult = gjk(a, b);

//     if (!gjkResult.collide) {
//         return null;
//     } else {
//         // If the gjk termination simplex has vertices less than 3, expand to full simplex
//         // Because EPA needs a full n-simplex to get started
//         const simplex = gjkResult.simplex;

//         switch (simplex.count) {
//             case 1:
//                 {
//                     const v = simplex.vertices[0];
//                     let randomSupport = csoSupport(a, b, new Vector2(1, 0));

//                     if (randomSupport.equals(v)) randomSupport = csoSupport(a, b, new Vector2(-1, 0));

//                     simplex.addVertex(randomSupport);
//                 }
//                 break;
//             case 2:
//                 {
//                     const e = new Edge(simplex.vertices[0], simplex.vertices[1]);
//                     const normalSupport = csoSupport(a, b, e.normal);

//                     if (simplex.containsVertex(normalSupport)) simplex.addVertex(csoSupport(a, b, e.normal.inverted()));
//                     else simplex.addVertex(normalSupport);
//                 }
//                 break;
//         }

//         const epaResult: EPAResult = epa(a, b, gjkResult.simplex);

//         let flipped = false;
//         // Apply axis weight to improve coherence
//         if (epaResult.contactNormal.dot(new Vector2(0, -1)) < 0) {
//             const tmp = a;
//             a = b;
//             b = tmp;
//             epaResult.contactNormal.invert();
//             flipped = true;
//         }

//         // Remove floating point error
//         epaResult.contactNormal.fix(Settings.EPA_TOLERANCE);

//         const contactPoints = findContactPoints(epaResult.contactNormal, a, b);

//         const contact = new ContactManifold(
//             a,
//             b,
//             contactPoints,
//             epaResult.penetrationDepth,
//             epaResult.contactNormal,
//             flipped,
//         );

//         return contact;
//     }
// }

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
