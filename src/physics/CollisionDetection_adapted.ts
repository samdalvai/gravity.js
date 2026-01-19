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

        const isColliding = ab.magnitudeSquared() <= radiusSum * radiusSum;

        if (!isColliding) {
            return null;
        }

        const normal = ab;
        normal.normalize();

        const start = b.position.subNew(normal.scaleNew(bCircleShape.radius));
        const end = a.position.addNew(normal.scaleNew(aCircleShape.radius));
        const depth = end.subNew(start).magnitude();

        const contact = new ContactManifold(a, b, [{ point: start, id: -1 }], depth, normal, false);

        return contact;
    };

    static detectCollisionPolygonPolygon = (a: Body, b: Body): ContactManifold | null => {
        const aPolygonShape = a.shape as PolygonShape;
        const bPolygonShape = b.shape as PolygonShape;

        const [abSeparation, aIndexReferenceEdge] = aPolygonShape.findMinSeparation(bPolygonShape);
        if (abSeparation >= 0) {
            return null;
        }

        const [baSeparation, bIndexReferenceEdge] = bPolygonShape.findMinSeparation(aPolygonShape);
        if (baSeparation >= 0) {
            return null;
        }

        // Determine reference and incident polygons
        let referenceShape: PolygonShape;
        let incidentShape: PolygonShape;
        let indexReferenceEdge: number;

        if (abSeparation > baSeparation) {
            referenceShape = aPolygonShape;
            incidentShape = bPolygonShape;
            indexReferenceEdge = aIndexReferenceEdge;
        } else {
            referenceShape = bPolygonShape;
            incidentShape = aPolygonShape;
            indexReferenceEdge = bIndexReferenceEdge;
        }

        // Find the reference edge based on the index that returned from the function
        const referenceEdge = referenceShape.edgeAt(indexReferenceEdge);

        /////////////////////////////////////
        // Clipping
        /////////////////////////////////////
        const referenceEdgeNormal = referenceEdge.normal();
        const incidentIndex = incidentShape.findIncidentEdge(referenceEdgeNormal);
        const incidentNextIndex = (incidentIndex + 1) % incidentShape.worldVertices.length;

        let contactPoints = [
            incidentShape.worldVertices[incidentIndex],
            incidentShape.worldVertices[incidentNextIndex],
        ];
        const clippedPoints = [...contactPoints];

        // Loop through reference polygon edges and clip the incident segment
        for (let i = 0; i < referenceShape.worldVertices.length; i++) {
            if (i === indexReferenceEdge) continue;

            const c0 = referenceShape.worldVertices[i];
            const c1 = referenceShape.worldVertices[(i + 1) % referenceShape.worldVertices.length];

            // Clip the segment to this edge
            const numClipped = referenceShape.clipSegmentToLine(contactPoints, clippedPoints, c0, c1);

            // If less than 2 points, exit
            if (numClipped < 2) break;

            // Make the next contact points the ones that were just clipped
            contactPoints = [...clippedPoints];
        }

        const vref = referenceShape.worldVertices[indexReferenceEdge];

        // Loop all clipped points, but only consider those where separation is negative (objects are penetrating each other)
        for (const vclip of clippedPoints) {
            const separation = vclip.subNew(vref).dot(referenceEdgeNormal);
            if (separation <= 0) {
                let start = vclip;
                let end = vclip.addNew(referenceEdgeNormal.scaleNew(-separation));
                const normal = referenceEdgeNormal.clone();

                let flipped = false;
                // Ensure the start-end points are always from "a" to "b"
                if (baSeparation >= abSeparation) {
                    [start, end] = [end, start];
                    // The collision normal is always from "a" to "b"
                    normal.scale(-1);
                    flipped = true;
                }

                const depth = end.subNew(start).magnitude();

                // TODO: check the clipped points, this should be ran once
                const contactPoints = findContactPoints_adapted(normal, a, b);
                const contact = new ContactManifold(a, b, contactPoints, depth, normal, flipped);

                return contact;
            }
        }

        return null;
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

                    const contactPoints = findContactPoints_adapted(normal, a, b);
                    const contact = new ContactManifold(a, b, contactPoints, depth, normal, false);
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

                        const contactPoints = findContactPoints_adapted(normal, a, b);
                        const contact = new ContactManifold(a, b, contactPoints, depth, normal, false);
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

                        const contactPoints = findContactPoints_adapted(normal, a, b);
                        const contact = new ContactManifold(a, b, contactPoints, depth, normal, false);
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

            const contactPoints = findContactPoints_adapted(normal, a, b);
            const contact = new ContactManifold(a, b, contactPoints, depth, normal, false);
            return contact;
        }
    };
}
