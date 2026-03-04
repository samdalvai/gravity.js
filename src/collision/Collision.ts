import { CONTACT_MERGE_THRESHOLD } from '../core/Constants';
import RigidBody from '../core/RigidBody';
import Vec2 from '../math/Vec2';
import { CapsuleShape } from '../shapes/CapsuleShape';
import { CircleShape } from '../shapes/CircleShape';
import { EdgeShape } from '../shapes/EdgeShape';
import { PolygonShape } from '../shapes/PolygonShape';
import { ShapeType } from '../shapes/Shape';
import * as Utils from '../utils/Utils';
import { ContactManifold } from './ContactManifold';

export function detectCollision(a: RigidBody, b: RigidBody): ContactManifold | null {
    const aIsCircle = a.shapeType === ShapeType.CIRCLE;
    const bIsCircle = b.shapeType === ShapeType.CIRCLE;

    if (aIsCircle && bIsCircle) {
        return detectCollisionCircleCircle(a, b);
    }

    const aIsPolygon = a.shapeType === ShapeType.BOX || a.shapeType === ShapeType.POLYGON;
    const bIsPolygon = b.shapeType === ShapeType.BOX || b.shapeType === ShapeType.POLYGON;

    if (aIsPolygon && bIsPolygon) {
        return detectCollisionPolygonPolygon(a, b);
    }

    if (aIsPolygon && bIsCircle) {
        return detectCollisionPolygonCircle(a, b);
    }

    if (aIsCircle && bIsPolygon) {
        return detectCollisionPolygonCircle(b, a);
    }

    const aIsCapsule = a.shapeType === ShapeType.CAPSULE;
    const bIsCapsule = b.shapeType === ShapeType.CAPSULE;

    if (aIsCapsule && bIsCapsule) {
        return detectCollisionCapsuleCapsule(a, b);
    }

    if (aIsCapsule && bIsCircle) {
        return detectCollisionCapsuleCircle(a, b);
    }

    if (aIsCircle && bIsCapsule) {
        return detectCollisionCapsuleCircle(b, a);
    }

    if (aIsCapsule && bIsPolygon) {
        return detectCollisionCapsulePolygon(a, b);
    }

    if (aIsPolygon && bIsCapsule) {
        return detectCollisionCapsulePolygon(b, a);
    }

    const aIsEdge = a.shapeType === ShapeType.EDGE;
    const bIsEdge = b.shapeType === ShapeType.EDGE;

    if (aIsEdge && bIsCircle) {
        return detectCollisionEdgeCircle(a, b);
    }

    if (aIsCircle && bIsEdge) {
        return detectCollisionEdgeCircle(b, a);
    }

    return null;
}

export function detectCollisionCircleCircle(circleA: RigidBody, circleB: RigidBody): ContactManifold | null {
    const aCircleShape = circleA.shape as CircleShape;
    const bCircleShape = circleB.shape as CircleShape;

    return circleCircleTest(
        circleA.position,
        aCircleShape.radius,
        circleA,
        circleB.position,
        bCircleShape.radius,
        circleB,
    );
}

function circleCircleTest(
    aPos: Vec2,
    aRadius: number,
    aBody: RigidBody,
    bPos: Vec2,
    bRadius: number,
    bBody: RigidBody,
): ContactManifold | null {
    const ab = bPos.subNew(aPos);
    const radiusSum = aRadius + bRadius;

    if (ab.magnitudeSquared() > radiusSum * radiusSum) {
        return null;
    }

    const normal = ab.normalizeNew();
    const contactPoint = aPos.addNew(normal.scaleNew(aRadius));
    const penetrationDepth = radiusSum - ab.magnitude();

    return new ContactManifold(aBody, bBody, [{ point: contactPoint, id: -1 }], penetrationDepth, normal, false);
}

export function detectCollisionPolygonPolygon(polygonA: RigidBody, polygonB: RigidBody): ContactManifold | null {
    const aPoly = polygonA.shape as PolygonShape;
    const bPoly = polygonB.shape as PolygonShape;

    const [abSep, aEdge] = aPoly.findMinSeparation(bPoly);
    if (abSep >= 0) return null;

    const [baSep, bEdge] = bPoly.findMinSeparation(aPoly);
    if (baSep >= 0) return null;

    let normal: Vec2;
    let penetrationDepth: number;
    let flipped = false;

    if (abSep > baSep) {
        normal = aPoly.edgeAt(aEdge).normal();
        penetrationDepth = -abSep;
    } else {
        normal = bPoly.edgeAt(bEdge).normal().negateNew();
        penetrationDepth = -baSep;
        flipped = true;
    }

    const contactPoints = findContactPoints(normal, polygonA, polygonB);
    if (contactPoints.length === 0) return null;

    return new ContactManifold(polygonA, polygonB, contactPoints, penetrationDepth, normal, flipped);
}

export interface ContactPoint {
    point: Vec2;
    id: number;
}

function findContactPoints(normal: Vec2, a: RigidBody, b: RigidBody): ContactPoint[] {
    const aPolygon = a.shape as PolygonShape;
    const bPolygon = b.shape as PolygonShape;
    const edgeA = aPolygon.findFarthestEdge(a, normal);
    const edgeB = bPolygon.findFarthestEdge(b, normal.negateNew());

    let ref = edgeA; // Reference edge
    let inc = edgeB; // Incidence edge
    let flip = false;

    const aPerpendicularness = Math.abs(edgeA.dir.dot(normal));
    const bPerpendicularness = Math.abs(edgeB.dir.dot(normal));

    if (aPerpendicularness >= bPerpendicularness) {
        ref = edgeB;
        inc = edgeA;
        flip = true;
    }

    inc.clip(ref.p1, ref.dir);
    inc.clip(ref.p2, ref.dir.negateNew());
    inc.clip(ref.p1, flip ? normal : normal.negateNew(), true);

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

export function detectCollisionPolygonCircle(polygon: RigidBody, circle: RigidBody): ContactManifold | null {
    const bCircleShape = circle.shape as CircleShape;
    return polygonCircleTest(polygon, circle.position, bCircleShape.radius, circle);
}

function polygonCircleTest(
    polygon: RigidBody,
    circlePos: Vec2,
    circleRadius: number,
    circle: RigidBody,
): ContactManifold | null {
    const polygonShape = polygon.shape as PolygonShape;
    const polygonVertices = polygonShape.worldVertices;

    let isOutside = false;
    let minCurrVertex = new Vec2();
    let minNextVertex = new Vec2();
    let distanceCircleEdge = -Infinity;

    // Loop all the edges of the polygon/box finding the nearest edge to the circle center
    for (let i = 0; i < polygonVertices.length; i++) {
        const currVertex = i;
        const nextVertex = (i + 1) % polygonVertices.length;
        const edge = polygonShape.edgeAt(currVertex);
        const edgeNormal = edge.normal();

        // Compare the circle center with the rectangle vertex
        const vertexToCircleCenter = circlePos.subNew(polygonVertices[currVertex]);
        const projection = vertexToCircleCenter.dot(edgeNormal);

        // If we found a dot product projection that is in the positive/outside side of the normal
        if (projection > 0) {
            // Circle center is outside the polygon
            distanceCircleEdge = projection;
            minCurrVertex = polygonVertices[currVertex];
            minNextVertex = polygonVertices[nextVertex];
            isOutside = true;
            break;
        } else {
            // Circle center is inside the rectangle, find the min edge (the one with the least negative projection)
            if (projection > distanceCircleEdge) {
                distanceCircleEdge = projection;
                minCurrVertex = polygonVertices[currVertex];
                minNextVertex = polygonVertices[nextVertex];
            }
        }
    }

    if (isOutside) {
        ///////////////////////////////////////
        // Check if we are inside region A:
        ///////////////////////////////////////
        let v1 = circlePos.subNew(minCurrVertex); // vector from the nearest vertex to the circle center
        let v2 = minNextVertex.subNew(minCurrVertex); // the nearest edge (from curr vertex to next vertex)
        if (v1.dot(v2) < 0) {
            // Distance from vertex to circle center is greater than radius... no collision
            if (v1.magnitude() > circleRadius) {
                return null;
            } else {
                // Detected collision in region A:
                const a = polygon;
                const b = circle;
                const depth = circleRadius - v1.magnitude();
                const normal = v1.normalize();
                const start = circlePos.addNew(normal.scaleNew(-circleRadius));
                const end = start.addNew(normal.scaleNew(depth));

                const contact = new ContactManifold(a, b, [{ point: end, id: -1 }], depth, normal, false);
                return contact;
            }
        } else {
            ///////////////////////////////////////
            // Check if we are inside region B:
            ///////////////////////////////////////
            v1 = circlePos.subNew(minNextVertex); // vector from the next nearest vertex to the circle center
            v2 = minCurrVertex.subNew(minNextVertex); // the nearest edge
            if (v1.dot(v2) < 0) {
                // Distance from vertex to circle center is greater than radius... no collision
                if (v1.magnitude() > circleRadius) {
                    return null;
                } else {
                    // Detected collision in region B:
                    const a = polygon;
                    const b = circle;
                    const depth = circleRadius - v1.magnitude();
                    const normal = v1.normalize();
                    const start = circlePos.addNew(normal.scaleNew(-circleRadius));
                    const end = start.addNew(normal.scaleNew(depth));

                    const contact = new ContactManifold(a, b, [{ point: end, id: -1 }], depth, normal, false);
                    return contact;
                }
            } else {
                ///////////////////////////////////////
                // We are inside region C:
                ///////////////////////////////////////
                if (distanceCircleEdge > circleRadius) {
                    // No collision... Distance between the closest distance and the circle center is greater than the radius.
                    return null;
                } else {
                    // Detected collision in region C:
                    const a = polygon;
                    const b = circle;
                    const depth = circleRadius - distanceCircleEdge;
                    const normal = minNextVertex.subNew(minCurrVertex).normal();
                    const start = circlePos.subNew(normal.scaleNew(circleRadius));
                    const end = start.addNew(normal.scaleNew(depth));

                    const contact = new ContactManifold(a, b, [{ point: end, id: -1 }], depth, normal, false);
                    return contact;
                }
            }
        }
    } else {
        // The center of circle is inside the polygon... it is definitely colliding!
        const a = polygon;
        const b = circle;
        const depth = circleRadius - distanceCircleEdge;
        const normal = minNextVertex.subNew(minCurrVertex).normal();
        const start = circlePos.subNew(normal.scaleNew(circleRadius));
        const end = start.addNew(normal.scaleNew(depth));

        const contact = new ContactManifold(a, b, [{ point: end, id: -1 }], depth, normal, false);
        return contact;
    }
}

export function detectCollisionCapsuleCapsule(capsuleA: RigidBody, capsuleB: RigidBody): ContactManifold | null {
    const capsuleShapeB = capsuleB.shape as CapsuleShape;

    const offsetUpB = new Vec2(0, capsuleShapeB.halfHeight).rotate(capsuleB.rotation);
    const offsetDownB = new Vec2(0, -capsuleShapeB.halfHeight).rotate(capsuleB.rotation);

    const topPosB = capsuleB.position.addNew(offsetUpB);
    const bottomPosB = capsuleB.position.addNew(offsetDownB);

    const bodyHit = detectCollisionCapsulePolygon(capsuleA, capsuleB);
    if (bodyHit) return bodyHit;

    // Test top circle
    const topHit = capsuleCircleTest(capsuleA, topPosB, capsuleShapeB.radius, capsuleB);
    if (topHit) return topHit;

    // Test bottom circle
    return capsuleCircleTest(capsuleA, bottomPosB, capsuleShapeB.radius, capsuleB);
}

export function detectCollisionCapsuleCircle(capsule: RigidBody, circle: RigidBody): ContactManifold | null {
    const bCircleShape = circle.shape as CircleShape;
    return capsuleCircleTest(capsule, circle.position, bCircleShape.radius, circle);
}

function capsuleCircleTest(
    capsule: RigidBody,
    circlePos: Vec2,
    circleRadius: number,
    circle: RigidBody,
): ContactManifold | null {
    const capsuleShape = capsule.shape as CapsuleShape;

    const offsetUp = new Vec2(0, capsuleShape.halfHeight).rotate(capsule.rotation);
    const offsetDown = new Vec2(0, -capsuleShape.halfHeight).rotate(capsule.rotation);

    const topPos = capsule.position.addNew(offsetUp);
    const bottomPos = capsule.position.addNew(offsetDown);

    const bodyHit = polygonCircleTest(capsule, circlePos, circleRadius, circle);
    if (bodyHit) return bodyHit;

    // Test top circle
    const topHit = circleCircleTest(topPos, capsuleShape.radius, capsule, circlePos, circleRadius, circle);
    if (topHit) return topHit;

    // Test bottom circle
    return circleCircleTest(bottomPos, capsuleShape.radius, capsule, circlePos, circleRadius, circle);
}

export function detectCollisionCapsulePolygon(capsule: RigidBody, polygon: RigidBody): ContactManifold | null {
    const capsuleShape = capsule.shape as CapsuleShape;

    const offsetUp = new Vec2(0, capsuleShape.halfHeight).rotate(capsule.rotation);
    const offsetDown = new Vec2(0, -capsuleShape.halfHeight).rotate(capsule.rotation);

    const topPos = capsule.position.addNew(offsetUp);
    const bottomPos = capsule.position.addNew(offsetDown);

    const bodyHit = detectCollisionPolygonPolygon(capsule, polygon);
    if (bodyHit) return bodyHit;

    // Test top circle
    const topHit = polygonCircleTest(polygon, topPos, capsuleShape.radius, capsule);
    if (topHit) return topHit;

    // Test bottom circle
    return polygonCircleTest(polygon, bottomPos, capsuleShape.radius, capsule);
}

export function detectCollisionEdgeCircle(edge: RigidBody, circle: RigidBody): ContactManifold | null {
    const edgeShape = edge.shape as EdgeShape;
    const circleShape = circle.shape as CircleShape;

    const A = edgeShape.worldVertices[0];
    const B = edgeShape.worldVertices[1];
    const C = circle.position.copy();
    const r = circleShape.radius;

    const AB = B.subNew(A);
    const AC = C.subNew(A);

    // Project AC onto AB
    let t = AC.dot(AB) / AB.dot(AB);

    // Clamp t so projection stays on the segment
    t = Utils.clamp(t, 0, 1);

    // Closest point on segment to circle center
    const closestPoint = A.addNew(AB.scaleNew(t));

    // Vector from closest point to circle center
    const diff = C.subNew(closestPoint);
    const distanceSquared = diff.dot(diff);

    if (distanceSquared > r * r) return null;

    const distance = Math.sqrt(distanceSquared);
    let normal: Vec2;
    let penetration: number;

    if (distance === 0) {
        // Circle center exactly on edge
        // Choose edge normal (perpendicular to AB)
        normal = AB.perpNew().normalize();
        penetration = r;
    } else {
        normal = diff.divNew(distance);
        penetration = r - distance;
    }

    const contact = new ContactManifold(edge, circle, [{ point: closestPoint, id: -1 }], penetration, normal, false);
    return contact;
}
