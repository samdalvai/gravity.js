import Vec2 from '../math/Vec2';
import { ContactManifold } from '../new/contact_adapted';
import { findContactPoints_adapted } from '../new/detection_adapted';
import Body from './Body';
import { CircleShape, PolygonShape, ShapeType } from './Shape';

export default class CollisionDetection {
    static detectCollision = (a: Body, b: Body): ContactManifold | null => {
        const aIsCircle = a.shapeType === ShapeType.CIRCLE;
        const bIsCircle = b.shapeType === ShapeType.CIRCLE;

        if (aIsCircle && bIsCircle) {
            return this.detectCollisionCircleCircle(a, b);
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

        return null;
    };

    static detectCollisionCircleCircle = (a: Body, b: Body): ContactManifold | null => {
        const aCircleShape = a.shape as CircleShape;
        const bCircleShape = b.shape as CircleShape;

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

    static detectCollisionPolygonPolygon = (a: Body, b: Body): ContactManifold | null => {
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
            normal = bPoly.edgeAt(bEdge).normal().negated();
            penetrationDepth = -baSep;
            flipped = true;
        }

        const contactPoints = findContactPoints_adapted(normal, a, b);
        if (contactPoints.length === 0) return null;

        return new ContactManifold(a, b, contactPoints, penetrationDepth, normal, flipped);
    };

    static detectCollisionPolygonCircle = (polygon: Body, circle: Body): ContactManifold | null => {
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
                    const contactPoint = start.addNew(end).scaleNew(0.5);

                    console.log('REGION A');
                    const contactPoints = findContactPoints_adapted(normal, a, b);
                    const contact = new ContactManifold(a, b, [{ point: end, id: -1 }], depth, normal, false);
                    // const contact = new ContactManifold(a, b, contactPoints, depth, normal, false);
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

                        console.log('REGION B');
                        const contactPoints = findContactPoints_adapted(normal, a, b);
                        // const contact = new ContactManifold(a, b, contactPoints, depth, normal, false);
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

                        console.log('REGION C');
                        const contactPoints = findContactPoints_adapted(normal, a, b);
                        // const contact = new ContactManifold(a, b, contactPoints, depth, normal, false);
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

            console.log('INSIDE POLYGON');
            const contactPoints = findContactPoints_adapted(normal, a, b);
            const contact = new ContactManifold(a, b, contactPoints, depth, normal, false);
            return contact;
        }
    };
}
