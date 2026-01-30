import Vec2 from '../math/Vec2';
import { CONTACT_MERGE_THRESHOLD } from './Constants';
import { ContactManifold } from './Contact';
import Edge from './Edge';
import RigidBody from './RigidBody';
import { CapsuleShape, CircleShape, PolygonShape, Shape, ShapeType } from './Shape';

const findFarthestEdge = (b: RigidBody, dir: Vec2): Edge => {
    const localDir = b.worldDirToLocal(dir);
    const farthest = b.shape.support(localDir);
    let curr = farthest.vertex;
    const idx = farthest.index;

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
};

export interface ContactPoint {
    point: Vec2;
    id: number;
}

const findContactPoints = (n: Vec2, a: RigidBody, b: RigidBody): ContactPoint[] => {
    const edgeA = findFarthestEdge(a, n);
    const edgeB = findFarthestEdge(b, n.negateNew());

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

    inc.clip(ref.p1, ref.dir);
    inc.clip(ref.p2, ref.dir.negateNew());
    inc.clip(ref.p1, flip ? n : n.negateNew(), true);

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
};

export default class CollisionDetection {
    static detectCollision = (a: RigidBody, b: RigidBody): ContactManifold | null => {
        const aIsCircle = a.shapeType === ShapeType.CIRCLE;
        const bIsCircle = b.shapeType === ShapeType.CIRCLE;

        if (aIsCircle && bIsCircle) {
            return this.detectCollisionCircleCircle(a, b, a.shape, b.shape);
        }

        const aIsPolygon = a.shapeType === ShapeType.POLYGON;
        const bIsPolygon = b.shapeType === ShapeType.POLYGON;

        if (aIsPolygon && bIsPolygon) {
            return this.detectCollisionPolygonPolygon(a, b);
        }

        if (aIsPolygon && bIsCircle) {
            return this.detectCollisionPolygonCircle(a, b);
        }

        if (aIsCircle && bIsPolygon) {
            return this.detectCollisionPolygonCircle(b, a);
        }

        const aIsCapsule = a.shapeType === ShapeType.CAPSULE;
        const bIsCapsule = b.shapeType === ShapeType.CAPSULE;

        if (aIsCapsule && bIsCircle) {
            //return this.detectCollisionCapsuleCircle(a, b);
        }

        return null;
    };

    // static detectCollisionCapsuleCircle = (a: RigidBody, b: RigidBody): ContactManifold | null => {
    //     const aCapsuleShape = a.shape as CapsuleShape;
    //     const positionUp = a.position.subNew(new Vec2(0, aCapsuleShape.halfHeight)).rotate(a.rotation);
    //     const positionDown = a.position.addNew(new Vec2(0, aCapsuleShape.halfHeight)).rotate(a.rotation);

        
    // };

    static detectCollisionCircleCircle = (
        a: RigidBody,
        b: RigidBody,
        shapeA: Shape,
        shapeB: Shape,
    ): ContactManifold | null => {
        const aCircleShape = shapeA as CircleShape;
        const bCircleShape = shapeB as CircleShape;

        const ab = b.position.subNew(a.position);
        const radiusSum = aCircleShape.radius + bCircleShape.radius;

        if (ab.magnitudeSquared() > radiusSum * radiusSum) {
            return null;
        }

        const normal = ab.normalizeNew();

        const contactPoint = a.position.addNew(normal.scaleNew(aCircleShape.radius));

        const penetrationDepth = radiusSum - ab.magnitude();

        return new ContactManifold(a, b, [{ point: contactPoint, id: -1 }], penetrationDepth, normal, false);
    };

    static detectCollisionPolygonPolygon = (a: RigidBody, b: RigidBody): ContactManifold | null => {
        const aPoly = a.shape as PolygonShape;
        const bPoly = b.shape as PolygonShape;

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

        const contactPoints = findContactPoints(normal, a, b);
        if (contactPoints.length === 0) return null;

        return new ContactManifold(a, b, contactPoints, penetrationDepth, normal, flipped);
    };

    static detectCollisionPolygonCircle = (polygon: RigidBody, circle: RigidBody): ContactManifold | null => {
        const polygonShape = polygon.shape as PolygonShape;
        const circleShape = circle.shape as CircleShape;
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
            const vertexToCircleCenter = circle.position.subNew(polygonVertices[currVertex]);
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
            let v1 = circle.position.subNew(minCurrVertex); // vector from the nearest vertex to the circle center
            let v2 = minNextVertex.subNew(minCurrVertex); // the nearest edge (from curr vertex to next vertex)
            if (v1.dot(v2) < 0) {
                // Distance from vertex to circle center is greater than radius... no collision
                if (v1.magnitude() > circleShape.radius) {
                    return null;
                } else {
                    // Detected collision in region A:
                    const a = polygon;
                    const b = circle;
                    const depth = circleShape.radius - v1.magnitude();
                    const normal = v1.normalize();
                    const start = circle.position.addNew(normal.scaleNew(-circleShape.radius));
                    const end = start.addNew(normal.scaleNew(depth));

                    const contact = new ContactManifold(a, b, [{ point: end, id: -1 }], depth, normal, false);
                    return contact;
                }
            } else {
                ///////////////////////////////////////
                // Check if we are inside region B:
                ///////////////////////////////////////
                v1 = circle.position.subNew(minNextVertex); // vector from the next nearest vertex to the circle center
                v2 = minCurrVertex.subNew(minNextVertex); // the nearest edge
                if (v1.dot(v2) < 0) {
                    // Distance from vertex to circle center is greater than radius... no collision
                    if (v1.magnitude() > circleShape.radius) {
                        return null;
                    } else {
                        // Detected collision in region B:
                        const a = polygon;
                        const b = circle;
                        const depth = circleShape.radius - v1.magnitude();
                        const normal = v1.normalize();
                        const start = circle.position.addNew(normal.scaleNew(-circleShape.radius));
                        const end = start.addNew(normal.scaleNew(depth));

                        const contact = new ContactManifold(a, b, [{ point: end, id: -1 }], depth, normal, false);
                        return contact;
                    }
                } else {
                    ///////////////////////////////////////
                    // We are inside region C:
                    ///////////////////////////////////////
                    if (distanceCircleEdge > circleShape.radius) {
                        // No collision... Distance between the closest distance and the circle center is greater than the radius.
                        return null;
                    } else {
                        // Detected collision in region C:
                        const a = polygon;
                        const b = circle;
                        const depth = circleShape.radius - distanceCircleEdge;
                        const normal = minNextVertex.subNew(minCurrVertex).normal();
                        const start = circle.position.subNew(normal.scaleNew(circleShape.radius));
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
            const depth = circleShape.radius - distanceCircleEdge;
            const normal = minNextVertex.subNew(minCurrVertex).normal();
            const start = circle.position.subNew(normal.scaleNew(circleShape.radius));
            const end = start.addNew(normal.scaleNew(depth));

            const contact = new ContactManifold(a, b, [{ point: end, id: -1 }], depth, normal, false);
            return contact;
        }
    };
}
