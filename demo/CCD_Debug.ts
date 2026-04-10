import { CapsuleShape, CircleShape, PolygonShape, RigidBody, SegmentShape, ShapeType, Utils, Vec2 } from '../src';
import { collideCircles, detectCollision } from '../src/collision/NarrowPhase';
import Graphics from './graphics/Graphics';

const EPSILON = 1e-8;

export function resolveCCD(bullet: RigidBody, bodies: RigidBody[], dt: number): number | null {
    Utils.assert(bullet.shape instanceof CircleShape);

    const bulletShape = bullet.shape as CircleShape;
    const radius = bulletShape.radius;

    const currentPos = bullet.position.copy();

    const candidateBodies: RigidBody[] = [];
    let lowestFraction = 1;

    for (const other of bodies) {
        const relVel = bullet.velocity.subNew(other.velocity);
        const nextPos = currentPos.addNew(relVel.scaleNew(dt));

        // Compute swept AABB for bullet
        const minX = Math.min(currentPos.x, nextPos.x) - radius;
        const minY = Math.min(currentPos.y, nextPos.y) - radius;
        const maxX = Math.max(currentPos.x, nextPos.x) + radius;
        const maxY = Math.max(currentPos.y, nextPos.y) + radius;

        if (bullet.id === other.id || other.isBullet) continue;

        // If objects don't overlap on X axis they cannot collide
        if (other.minX > maxX || other.maxX < minX) continue;

        // If objects overlap on X axis but don't overlap on Y axis the cannot collide
        if (other.maxY < minY || other.minY > maxY) continue;

        candidateBodies.push(other);
    }

    for (const other of candidateBodies) {
        const relVel = bullet.velocity.subNew(other.velocity);
        const nextPos = currentPos.addNew(relVel.scaleNew(dt));

        switch (other.shapeType) {
            case ShapeType.BOX:
            case ShapeType.POLYGON: {
                const toi = sweepCircleVsPolygonTOI(bullet, other, dt);

                if (toi != null && toi < lowestFraction) {
                    lowestFraction = toi;
                }

                break;
            }
            case ShapeType.SEGMENT: {
                const toi = sweepCircleVsSegmentTOI(bullet, other, dt);

                if (toi != null && toi < lowestFraction) {
                    lowestFraction = toi;
                }

                break;
            }
            case ShapeType.CIRCLE: {
                const toi = sweepCircleVsCircleTOI(bullet, other, dt);

                if (toi != null && toi < lowestFraction) {
                    lowestFraction = toi;
                }

                break;
            }
            case ShapeType.CAPSULE: {
                const capsuleShape = other.shape as CapsuleShape;
                const topCirclePosition = capsuleShape.getTopCirclePosition();
                const bottomCirclePosition = capsuleShape.getBottomCirclePosition();

                const axis = bottomCirclePosition.subNew(topCirclePosition);
                const axisDir = axis.normalizeNew();

                const topCircleIntersections = edgeCircleIntersection(
                    currentPos,
                    nextPos,
                    topCirclePosition,
                    capsuleShape.radius,
                );

                for (const int of topCircleIntersections) {
                    // TODO: can we take advantage of this to improve capsules collision?
                    const v = int.subNew(topCirclePosition);
                    if (v.dot(axisDir) > 0) continue; // Skip bottom half

                    const fraction = getFraction(currentPos, nextPos, int);

                    if (fraction < lowestFraction) {
                        lowestFraction = fraction;
                    }
                }

                const bottomCircleIntersections = edgeCircleIntersection(
                    currentPos,
                    nextPos,
                    bottomCirclePosition,
                    capsuleShape.radius,
                );

                for (const int of bottomCircleIntersections) {
                    // TODO: can we take advantage of this to improve capsules collision?
                    const v = int.subNew(bottomCirclePosition);
                    if (v.dot(axisDir) < 0) continue; // Skip upper half

                    const fraction = getFraction(currentPos, nextPos, int);

                    if (fraction < lowestFraction) {
                        lowestFraction = fraction;
                    }
                }

                const vertices = capsuleShape.worldVertices;
                for (let i = 0; i < vertices.length; i++) {
                    // TODO: can we take advantage of this to improve capsules collision?
                    if (i % 2 === 0) continue; // Skip top and bottom edges
                    const v0 = vertices[i];
                    const v1 = vertices[(i + 1) % vertices.length];

                    const intersection = edgeEdgeIntersection(currentPos, nextPos, v0, v1);

                    if (intersection) {
                        const fraction = getFraction(currentPos, nextPos, intersection);

                        if (fraction < lowestFraction) {
                            lowestFraction = fraction;
                        }
                    }
                }
                break;
            }
        }

        // Debug code to be removed
        if (lowestFraction === 1) continue;

        const otherInitialPos = other.position.copy();
        const otherShapeNextPost = other.position.addNew(other.velocity.scaleNew(dt * lowestFraction));
        other.position = otherShapeNextPost.copy();
        Graphics.drawBody(other, { fillColor: 'rgba(128, 128, 128, 0.5)' });

        const previousPos = bullet.position.copy();
        const bulletNextPos = previousPos.addNew(bullet.velocity);
        const bulletNextPosFraction = previousPos.addNew(bullet.velocity.scaleNew(dt * lowestFraction));
        bullet.position = bulletNextPosFraction.copy();
        Graphics.drawBody(bullet, { fillColor: 'rgba(128, 128, 128, 0.75)' });

        Graphics.drawLine(previousPos.x, previousPos.y, bulletNextPos.x, bulletNextPos.y, 'red', 1);

        // const collide = detectCollision(bullet, other);

        // if (collide) {
        //     console.log('Colliding');
        // } else {
        //     console.log('Fraction: ', lowestFraction);
        //     console.log('dt Fraction: ', dt * lowestFraction);
        //     console.log('pos bullet: ', bullet.position);
        //     console.log('pos other: ', other.position);
        //     const bulletShape = bullet.shape;
        //     const otherShape = other.shape;
        //     const combinedRadius = bulletShape.radius + otherShape.radius;
        //     const distSq = bullet.position.distanceSquared(other.position);
        //     const dist = Math.sqrt(distSq);

        //     console.log('combined radius: ', combinedRadius);
        //     console.log('distance: ', dist);
        // }

        other.position = otherInitialPos.copy();
        bullet.position = previousPos.copy();
    }

    if (lowestFraction === 1) return null;

    return lowestFraction;
}

function edgeEdgeIntersection(A: Vec2, B: Vec2, C: Vec2, D: Vec2): Vec2 | null {
    const r = B.subNew(A); // vector along first segment
    const s = D.subNew(C); // vector along second segment
    const rxs = r.x * s.y - r.y * s.x;
    if (rxs === 0) return null; // parallel or collinear

    const t = (C.subNew(A).x * s.y - C.subNew(A).y * s.x) / rxs;
    const u = (C.subNew(A).x * r.y - C.subNew(A).y * r.x) / rxs;

    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
        return A.addNew(r.scaleNew(t));
    }

    return null; // no intersection on the segments
}

function edgeCircleIntersection(A: Vec2, B: Vec2, C: Vec2, r: number): Vec2[] {
    const d = B.subNew(A);
    const f = A.subNew(C);

    const a = d.dot(d);
    const b = 2 * f.dot(d);
    const c = f.dot(f) - r * r;

    let discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
        return []; // no intersection
    }

    discriminant = Math.sqrt(discriminant);

    const t1 = (-b - discriminant) / (2 * a);
    const t2 = (-b + discriminant) / (2 * a);

    const intersections: Vec2[] = [];

    if (0 <= t1 && t1 <= 1) {
        intersections.push(A.addNew(d.scaleNew(t1)));
    }

    if (0 <= t2 && t2 <= 1 && t2 != t1) {
        intersections.push(A.addNew(d.scaleNew(t2)));
    }

    return intersections;
}

/**
 * Gets the fraction [0, 1] along an axis where the point lies
 * @param position
 * @param nextPosition
 * @param point
 * @returns
 */
function getFraction(a: Vec2, b: Vec2, point: Vec2): number {
    const ab = b.subNew(a);
    const ap = point.subNew(a);

    const abLenSq = ab.x * ab.x + ab.y * ab.y;

    if (abLenSq === 0) return 0;

    return (ap.x * ab.x + ap.y * ab.y) / abLenSq;
}

function sweepCircleVsCircleTOI(bodyA: RigidBody, bodyB: RigidBody, dt: number): number | null {
    const circleA = bodyA.shape as CircleShape;
    const circleB = bodyB.shape as CircleShape;

    const v = bodyA.velocity.subNew(bodyB.velocity);
    const d = v.scaleNew(dt);
    const r = circleA.radius + circleB.radius;
    const m = bodyA.position.subNew(bodyB.position);

    const a = d.dot(d);
    const b = 2 * m.dot(d);
    const c = m.dot(m) - r * r;

    if (c <= 0) {
        return null;
    }

    if (Math.abs(a) <= EPSILON) {
        return null;
    }

    const disc = b * b - 4 * a * c;

    if (disc < -EPSILON) {
        return null;
    }

    const t = (-b - Math.sqrt(Math.max(disc, 0))) / (2 * a);

    if (t < 0 || t > 1) {
        return null;
    }

    return t;
}

function sweepCircleVsSegmentTOI(bodyA: RigidBody, bodyB: RigidBody, dt: number): number | null {
    const circle = bodyA.shape as CircleShape;
    const segment = bodyB.shape as SegmentShape;

    const a = segment.worldVertices[0];
    const b = segment.worldVertices[1];

    const radius = circle.radius;

    // Relative motion: treat segment as static
    const v = bodyA.velocity.subNew(bodyB.velocity).scaleNew(dt);
    const p = bodyA.position;

    const ab = b.subNew(a);
    const abLenSq = ab.dot(ab);

    if (abLenSq <= EPSILON) {
        // Degenerate segment -> point
        return sweepPointVsPointRadiusTOI(p, v, a, radius);
    }

    const abLen = Math.sqrt(abLenSq);
    const tangent = ab.scaleNew(1 / abLen);
    const normal = new Vec2(-tangent.y, tangent.x);

    let lowestT = Infinity;

    // Optional: if already overlapping at t = 0
    if (distancePointToSegmentSquared(p, a, b) <= radius * radius) {
        return 0;
    }

    // 1) Hit against segment interior (infinite line offset by ±radius)
    const signedDist = p.subNew(a).dot(normal);
    const vn = v.dot(normal);

    if (Math.abs(vn) > EPSILON) {
        const t0 = (radius - signedDist) / vn;
        const t1 = (-radius - signedDist) / vn;

        if (t0 >= 0 && t0 <= 1) {
            const hitPoint = p.addNew(v.scaleNew(t0));
            const proj = hitPoint.subNew(a).dot(tangent);

            if (proj >= 0 && proj <= abLen) {
                lowestT = Math.min(lowestT, t0);
            }
        }

        if (t1 >= 0 && t1 <= 1) {
            const hitPoint = p.addNew(v.scaleNew(t1));
            const proj = hitPoint.subNew(a).dot(tangent);

            if (proj >= 0 && proj <= abLen) {
                lowestT = Math.min(lowestT, t1);
            }
        }
    }

    // 2) Hit endpoint A
    const tA = sweepPointVsPointRadiusTOI(p, v, a, radius);
    if (tA != null) lowestT = Math.min(lowestT, tA);

    // 3) Hit endpoint B
    const tB = sweepPointVsPointRadiusTOI(p, v, b, radius);
    if (tB != null) lowestT = Math.min(lowestT, tB);

    return lowestT !== Infinity ? lowestT : null;
}

function sweepPointVsPointRadiusTOI(
    p: Vec2,
    d: Vec2, // full step displacement, not velocity
    center: Vec2,
    radius: number,
): number | null {
    const m = p.subNew(center);

    const a = d.dot(d);
    const b = 2 * m.dot(d);
    const c = m.dot(m) - radius * radius;

    if (c <= 0) {
        return 0;
    }

    if (a <= EPSILON) {
        return null;
    }

    const disc = b * b - 4 * a * c;

    if (disc < -EPSILON) {
        return null;
    }

    const t = (-b - Math.sqrt(Math.max(0, disc))) / (2 * a);

    if (t < 0 || t > 1) {
        return null;
    }

    return t;
}

function distancePointToSegmentSquared(p: Vec2, a: Vec2, b: Vec2): number {
    const ab = b.subNew(a);
    const ap = p.subNew(a);
    const abLenSq = ab.dot(ab);

    if (abLenSq <= EPSILON) {
        return ap.dot(ap);
    }

    let t = ap.dot(ab) / abLenSq;
    t = Math.max(0, Math.min(1, t));

    const closest = a.addNew(ab.scaleNew(t));
    return p.subNew(closest).dot(p.subNew(closest));
}

function sweepCircleVsPolygonTOI(bodyA: RigidBody, bodyB: RigidBody, dt: number): number | null {
    const circle = bodyA.shape as CircleShape;
    const polygon = bodyB.shape as PolygonShape;

    const p = bodyA.position;
    const d = bodyA.velocity.subNew(bodyB.velocity).scaleNew(dt); // full relative displacement
    const radius = circle.radius;

    const vertices = polygon.worldVertices;
    const normals = polygon.worldNormals;

    let lowestT = Infinity;

    // Already overlapping at t = 0
    if (pointInInflatedPolygon(p, vertices, normals, radius)) {
        return 0;
    }

    // 1) Face hits: solve signed distance to each face offset by radius
    for (let i = 0; i < vertices.length; i++) {
        const v0 = vertices[i];
        const normal = normals[i]; // outward normal

        const dist0 = p.subNew(v0).dot(normal);
        const vn = d.dot(normal);

        // Need to move toward the face
        if (vn >= -EPSILON) continue;

        // Hit when center reaches the face expanded outward by radius
        const t = (radius - dist0) / vn;

        if (t < 0 || t > 1) continue;

        const hitPoint = p.addNew(d.scaleNew(t));

        // Check that the projected contact lies on this edge segment, not outside near vertices
        const v1 = vertices[(i + 1) % vertices.length];
        const edge = v1.subNew(v0);
        const edgeLenSq = edge.dot(edge);

        if (edgeLenSq <= EPSILON) continue;

        const contactOnFace = hitPoint.subNew(normal.scaleNew(radius));
        const proj = contactOnFace.subNew(v0).dot(edge) / edgeLenSq;

        if (proj >= 0 && proj <= 1) {
            if (t < lowestT) lowestT = t;
        }
    }

    // 2) Vertex hits: sweep point vs circle(radius) for every polygon vertex
    for (let i = 0; i < vertices.length; i++) {
        const t = sweepPointVsPointRadiusTOI(p, d, vertices[i], radius);
        if (t != null && t < lowestT) {
            lowestT = t;
        }
    }

    return lowestT !== Infinity ? lowestT : null;
}

// Checks whether point p is inside polygon inflated by radius.
// For a convex polygon with outward normals, point is inside inflated polygon
// if it is not farther than radius outside any face.
function pointInInflatedPolygon(p: Vec2, vertices: Vec2[], normals: Vec2[], radius: number): boolean {
    for (let i = 0; i < vertices.length; i++) {
        const dist = p.subNew(vertices[i]).dot(normals[i]);
        if (dist > radius) {
            return false;
        }
    }
    return true;
}
