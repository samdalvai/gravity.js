import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { CapsuleShape } from '../shapes/CapsuleShape';
import { CircleShape } from '../shapes/CircleShape';
import { PolygonShape } from '../shapes/PolygonShape';
import { ShapeType } from '../shapes/Shape';
import * as Utils from '../utils/Utils';

export function resolveCCD(bullet: RigidBody, bodies: RigidBody[], dt: number): number | null {
    Utils.assert(bullet.shape instanceof CircleShape);

    const bulletShape = bullet.shape as CircleShape;
    const radius = bulletShape.radius;

    const currentPos = bullet.position.copy();
    const nextPos = currentPos.addNew(bullet.velocity.scaleNew(dt));

    // Compute swept AABB for bullet
    const minX = Math.min(currentPos.x, nextPos.x) - radius;
    const minY = Math.min(currentPos.y, nextPos.y) - radius;
    const maxX = Math.max(currentPos.x, nextPos.x) + radius;
    const maxY = Math.max(currentPos.y, nextPos.y) + radius;

    let minDistanceSquared = Infinity;
    let closestIntersection: Vec2 | undefined;

    const candidateBodies: RigidBody[] = [];

    for (const other of bodies) {
        if (bullet.id === other.id || other.isBullet) continue;

        // If objects don't overlap on X axis they cannot collide
        if (other.minX > maxX || other.maxX < minX) continue;

        // If objects overlap on X axis but don't overlap on Y axis the cannot collide
        if (other.maxY < minY || other.minY > maxY) continue;

        candidateBodies.push(other);
    }

    for (const other of candidateBodies) {
        if (
            other.shapeType === ShapeType.BOX ||
            other.shapeType === ShapeType.POLYGON ||
            other.shapeType === ShapeType.SEGMENT
        ) {
            const polygonShape = other.shape as PolygonShape;
            const vertices = polygonShape.worldVertices;

            for (let i = 0; i < vertices.length; i++) {
                const v0 = vertices[i];
                const v1 = vertices[(i + 1) % vertices.length];

                const intersection = edgeEdgeIntersection(currentPos, nextPos, v0, v1);

                if (intersection) {
                    const distanceSquared = intersection.subNew(currentPos).magnitudeSquared();

                    if (distanceSquared < minDistanceSquared) {
                        closestIntersection = intersection.copy();
                        minDistanceSquared = distanceSquared;
                    }
                }
            }
        }

        if (other.shapeType === ShapeType.CIRCLE) {
            const circleShape = other.shape as CircleShape;
            const intersections = edgeCircleIntersection(currentPos, nextPos, other.position, circleShape.radius);

            for (const int of intersections) {
                const distanceSquared = int.subNew(currentPos).magnitudeSquared();

                if (distanceSquared < minDistanceSquared) {
                    closestIntersection = int.copy();
                    minDistanceSquared = distanceSquared;
                }
            }
        }

        if (other.shapeType === ShapeType.CAPSULE) {
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

                const distanceSquared = int.subNew(currentPos).magnitudeSquared();

                if (distanceSquared < minDistanceSquared) {
                    closestIntersection = int.copy();
                    minDistanceSquared = distanceSquared;
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

                const distanceSquared = int.subNew(currentPos).magnitudeSquared();

                if (distanceSquared < minDistanceSquared) {
                    closestIntersection = int.copy();
                    minDistanceSquared = distanceSquared;
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
                    const distanceSquared = intersection.subNew(currentPos).magnitudeSquared();

                    if (distanceSquared < minDistanceSquared) {
                        closestIntersection = intersection.copy();
                        minDistanceSquared = distanceSquared;
                    }
                }
            }
        }
    }

    if (closestIntersection) {
        const toBullet = currentPos.subNew(closestIntersection).unitVector();
        const bulletNewPos = closestIntersection.addNew(toBullet.scaleNew(bulletShape.radius))
        const fraction = getFraction(currentPos, nextPos, bulletNewPos);
        console.log('fraction: ', fraction);
        // TODO: if we move bullet to closestIntersection and shoot down the bullet sticks to the floor,
        // probably something wron with polygon/circle collision, also if we move to bulletNewPos
        // the bounce angle is wrong
        // const toBullet = currentPos.subNew(closestIntersection).unitVector();
        // const bulletNewPos = closestIntersection.addNew(toBullet.scaleNew(bulletShape.radius));
        // bullet.position = bulletNewPos.copy();
        // bullet.shape.updateAABB(bullet);
        // bullet.hasCCD = true;
        return fraction;
    }

    return null;
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
