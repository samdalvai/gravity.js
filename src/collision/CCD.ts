import RigidBody from '../core/RigidBody';
import Vec2 from '../math/Vec2';
import { CapsuleShape } from '../shapes/CapsuleShape';
import { CircleShape } from '../shapes/CircleShape';
import { edgeCircleIntersection, edgeIntersection } from '../shapes/Edge';
import { PolygonShape } from '../shapes/PolygonShape';
import { ShapeType } from '../shapes/Shape';
import * as Utils from '../utils/Utils';

export function resolveCCD(bullet: RigidBody, bodies: RigidBody[], dt: number): void {
    Utils.assert(bullet.shape instanceof CircleShape);

    const bulletShape = bullet.shape as CircleShape;
    const currentPos = bullet.position.copy();
    const nextPos = currentPos.addNew(bullet.velocity.scaleNew(dt));

    let minDistanceSquared = Infinity;
    let closestIntersection: Vec2 | undefined;

    // TODO: We could cast two rays instead of one or check intersection by shifting up and down by radius
    for (const other of bodies) {
        if (bullet.id === other.id || other.isBullet) continue;

        if (other.shapeType === ShapeType.BOX || other.shapeType === ShapeType.POLYGON) {
            const polygonShape = other.shape as PolygonShape;
            const vertices = polygonShape.worldVertices;

            for (let i = 0; i < vertices.length; i++) {
                const v0 = vertices[i];
                const v1 = vertices[(i + 1) % vertices.length];

                const intersection = edgeIntersection(currentPos, nextPos, v0, v1);

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
            const topCirclePosition = capsuleShape.getTopCirclePosition(other);
            const bottomCirclePosition = capsuleShape.getBottomCirclePosition(other);

            const axis = bottomCirclePosition.subNew(topCirclePosition);
            const axisDir = axis.normalizeNew();

            const topCircleIntersections = edgeCircleIntersection(
                currentPos,
                nextPos,
                topCirclePosition,
                capsuleShape.radius,
            );

            for (const int of topCircleIntersections) {
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
                if (i % 2 === 0) continue; // Skip top and bottom vertices
                const v0 = vertices[i];
                const v1 = vertices[(i + 1) % vertices.length];

                const intersection = edgeIntersection(currentPos, nextPos, v0, v1);

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
        // TODO: if we move bullet to closestIntersection and shoot down the bullet sticks to the floor,
        // probably something wron with polygon/circle collision, also if we move to bulletNewPos
        // the bounce angle is wrong
        const toBullet = currentPos.subNew(closestIntersection).unitVector();
        const bulletNewPos = closestIntersection.addNew(toBullet.scaleNew(bulletShape.radius));
        bullet.position = bulletNewPos.copy();
        bullet.shape.updateAABB(bullet);
        bullet.hasCCD = true;
    }
}
