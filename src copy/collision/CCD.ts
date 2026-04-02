import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { CapsuleShape } from '../shapes/CapsuleShape';
import { CircleShape } from '../shapes/CircleShape';
import { PolygonShape } from '../shapes/PolygonShape';
import { ShapeType } from '../shapes/Shape';
import * as Utils from '../utils/Utils';

const EPSILON = 1.0e-8;

function rayCastCircle(origin: Vec2, translation: Vec2, center: Vec2, radius: number): number | null {
    const s = origin.subNew(center);
    const { length, normal } = translation.lengthAndNormalize(EPSILON);

    if (length < EPSILON) {
        return null;
    }

    const t = -s.dot(normal);
    const c = s.addNew(normal.scaleNew(t));
    const radiusSquared = radius * radius;

    if (c.dot(c) > radiusSquared) {
        return null;
    }

    const h = Math.sqrt(radiusSquared - c.dot(c));
    const hitDistance = t - h;

    if (hitDistance < 0 || hitDistance > length) {
        return null;
    }

    return hitDistance / length;
}

function rayCastCapsule(origin: Vec2, translation: Vec2, center1: Vec2, center2: Vec2, radius: number): number | null {
    const axis = center2.subNew(center1);
    const { length: capsuleLength, normal: axisUnit } = axis.lengthAndNormalize(EPSILON);

    if (capsuleLength < EPSILON) {
        return rayCastCircle(origin, translation, center1, radius);
    }

    const q = origin.subNew(center1);
    const qa = q.dot(axisUnit);
    const qp = q.subNew(axisUnit.scaleNew(qa));
    const radiusSquared = radius * radius;

    if (qp.dot(qp) < radiusSquared) {
        if (qa < 0) {
            return rayCastCircle(origin, translation, center1, radius);
        }

        if (qa > capsuleLength) {
            return rayCastCircle(origin, translation, center2, radius);
        }

        return null;
    }

    const { length: rayLength, normal: rayUnit } = translation.lengthAndNormalize(EPSILON);

    if (rayLength < EPSILON) {
        return null;
    }

    const sideNormal = axisUnit.rightPerpNew();
    const denominator = rayUnit.cross(axisUnit);

    if (Math.abs(denominator) < EPSILON) {
        return null;
    }

    const nearOffset = q.subNew(sideNormal.scaleNew(radius));
    const farOffset = q.addNew(sideNormal.scaleNew(radius));
    const nearDistance = axisUnit.cross(nearOffset) / denominator;
    const farDistance = axisUnit.cross(farOffset) / denominator;

    let hitDistance = nearDistance;
    let offset = nearOffset;

    if (farDistance < nearDistance) {
        hitDistance = farDistance;
        offset = farOffset;
        sideNormal.negate();
    }

    if (hitDistance < 0 || hitDistance > rayLength) {
        return null;
    }

    const axisDistance = rayUnit.cross(offset) / denominator;

    if (axisDistance < 0) {
        return rayCastCircle(origin, translation, center1, radius);
    }

    if (axisDistance > capsuleLength) {
        return rayCastCircle(origin, translation, center2, radius);
    }

    return hitDistance / rayLength;
}

function getSegmentFraction(origin: Vec2, translation: Vec2, start: Vec2, end: Vec2, radius: number): number | null {
    return rayCastCapsule(origin, translation, start, end, radius);
}

function getPolygonFraction(origin: Vec2, translation: Vec2, shape: PolygonShape, bulletRadius: number): number | null {
    const vertices = shape.worldVertices;
    const edgeCount = vertices.length;
    let bestFraction: number | null = null;

    for (let i = 0; i < edgeCount; ++i) {
        const start = vertices[i];
        const end = vertices[(i + 1) % edgeCount];
        const fraction = rayCastCapsule(origin, translation, start, end, bulletRadius + shape.radius);

        if (fraction === null) {
            continue;
        }

        if (bestFraction === null || fraction < bestFraction) {
            bestFraction = fraction;
        }
    }

    return bestFraction;
}

function getHitFraction(bullet: RigidBody, other: RigidBody, dt: number): number | null {
    const bulletShape = bullet.shape as CircleShape;
    const origin = bullet.position.copy();
    const translation = bullet.velocity.scaleNew(dt);

    if (translation.magnitudeSquared() < EPSILON * EPSILON) {
        return null;
    }

    switch (other.shapeType) {
        case ShapeType.CIRCLE: {
            const circle = other.shape as CircleShape;
            return rayCastCircle(origin, translation, other.position, bulletShape.radius + circle.radius);
        }

        case ShapeType.CAPSULE: {
            const capsule = other.shape as CapsuleShape;
            return rayCastCapsule(
                origin,
                translation,
                capsule.worldCenter1,
                capsule.worldCenter2,
                bulletShape.radius + capsule.radius,
            );
        }

        case ShapeType.SEGMENT: {
            const segment = other.shape as PolygonShape;
            return getSegmentFraction(
                origin,
                translation,
                segment.worldVertices[0],
                segment.worldVertices[1],
                bulletShape.radius,
            );
        }

        case ShapeType.BOX:
        case ShapeType.POLYGON: {
            return getPolygonFraction(origin, translation, other.shape as PolygonShape, bulletShape.radius);
        }

        default:
            return null;
    }
}

export function resolveCCD(bullet: RigidBody, bodies: RigidBody[], dt: number): void {
    Utils.assert(bullet.shape instanceof CircleShape);

    let bestFraction: number | null = null;

    for (const other of bodies) {
        if (bullet.id === other.id || other.isBullet) {
            continue;
        }

        const fraction = getHitFraction(bullet, other, dt);

        if (fraction === null) {
            continue;
        }

        if (bestFraction === null || fraction < bestFraction) {
            bestFraction = fraction;
        }
    }

    if (bestFraction === null) {
        return;
    }

    const travelTime = dt * bestFraction;

    bullet.position.addAssign(bullet.velocity.scaleNew(travelTime));
    bullet.rotation += bullet.angularVelocity * travelTime;
    bullet.shape.updateVertices(bullet.rotation, bullet.position);
    bullet.shape.updateAABB(bullet);
    bullet.hasCCD = true;
}
