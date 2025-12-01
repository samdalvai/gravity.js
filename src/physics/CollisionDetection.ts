import Body from './Body';
import Contact from './Contact';
import { CircleShape, PolygonShape, ShapeType } from './Shape';
import Vec2 from './Vec2';

export type CollisionResult = { isColliding: false; contacts?: undefined } | { isColliding: true; contacts: Contact[] };

export default class CollisionDetection {
    static detectCollision = (a: Body, b: Body): CollisionResult => {
        const aIsCircle = a.shape.getType() === ShapeType.CIRCLE;
        const bIsCircle = b.shape.getType() === ShapeType.CIRCLE;
        const aIsPolygon = a.shape.getType() === ShapeType.POLYGON || a.shape.getType() === ShapeType.BOX;
        const bIsPolygon = b.shape.getType() === ShapeType.POLYGON || b.shape.getType() === ShapeType.BOX;

        if (aIsCircle && bIsCircle) {
            return this.detectCollisionCircleCircle(a, b);
        }

        if (aIsPolygon && bIsPolygon) {
            return this.detectCollisionPolygonPolygon(a, b);
        }

        if (aIsPolygon && bIsCircle) {
            return this.detectCollisionPolygonCircle(a, b);
        }

        if (aIsCircle && bIsPolygon) {
            return this.detectCollisionPolygonCircle(b, a);
        }

        return { isColliding: false };
    };

    static detectCollisionCircleCircle = (a: Body, b: Body): CollisionResult => {
        const aCircleShape = a.shape as CircleShape;
        const bCircleShape = b.shape as CircleShape;

        const ab = b.position.subNew(a.position);
        const radiusSum = aCircleShape.radius + bCircleShape.radius;

        const isColliding = ab.magnitudeSquared() <= radiusSum * radiusSum;

        if (!isColliding) {
            return { isColliding: false };
        }

        const normal = ab;
        normal.normalize();

        const start = b.position.subNew(normal.scaleNew(bCircleShape.radius));
        const end = a.position.addNew(normal.scaleNew(aCircleShape.radius));
        const depth = end.subNew(start).magnitude();

        return { isColliding: true, contacts: [new Contact(a, b, start, end, normal, depth)] };
    };

    static detectCollisionPolygonPolygon = (a: Body, b: Body): CollisionResult => {
        const aPolygonShape = a.shape as PolygonShape;
        const bPolygonShape = b.shape as PolygonShape;

        const aIndexReferenceEdge = { value: 0 };
        const bIndexReferenceEdge = { value: 0 };
        const aSupportPoint = new Vec2();
        const bSupportPoint = new Vec2();

        const abSeparation = aPolygonShape.findMinSeparation(bPolygonShape, aIndexReferenceEdge, aSupportPoint);
        if (abSeparation >= 0) {
            return { isColliding: false };
        }

        const baSeparation = bPolygonShape.findMinSeparation(aPolygonShape, bIndexReferenceEdge, bSupportPoint);
        if (baSeparation >= 0) {
            return { isColliding: false };
        }

        // Determine reference and incident polygons
        let referenceShape: PolygonShape;
        let incidentShape: PolygonShape;
        let indexReferenceEdge: number;

        if (abSeparation > baSeparation) {
            referenceShape = aPolygonShape;
            incidentShape = bPolygonShape;
            indexReferenceEdge = aIndexReferenceEdge.value;
        } else {
            referenceShape = bPolygonShape;
            incidentShape = aPolygonShape;
            indexReferenceEdge = bIndexReferenceEdge.value;
        }

        // Find the reference edge based on the index that returned from the function
        const referenceEdge = referenceShape.edgeAt(indexReferenceEdge);

        /////////////////////////////////////
        // Clipping
        /////////////////////////////////////
        const incidentIndex = incidentShape.findIncidentEdge(referenceEdge.normal());
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
        const contacts: Contact[] = [];

        // Loop all clipped points, but only consider those where separation is negative (objects are penetrating each other)
        for (const vclip of clippedPoints) {
            const separation = vclip.subNew(vref).dot(referenceEdge.normal());
            if (separation <= 0) {
                const contact: Contact = {
                    a,
                    b,
                    normal: referenceEdge.normal(),
                    start: vclip,
                    end: vclip.addNew(referenceEdge.normal().scaleNew(-separation)),
                    depth: 0,
                };

                // Ensure the start-end points are always from "a" to "b"
                if (baSeparation >= abSeparation) {
                    [contact.start, contact.end] = [contact.end, contact.start];
                    // The collision normal is always from "a" to "b"
                    contact.normal = contact.normal.scaleNew(-1);
                }

                contacts.push(contact);
            }
        }

        return { isColliding: true, contacts };
    };

    static detectCollisionPolygonCircle = (polygon: Body, circle: Body): CollisionResult => {
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
                    return { isColliding: false };
                } else {
                    // Detected collision in region A:
                    const a = polygon;
                    const b = circle;
                    const depth = circleShape.radius - v1.magnitude();
                    const normal = v1.normalize();
                    const start = circle.position.addNew(normal.scaleNew(-circleShape.radius));
                    const end = start.addNew(normal.scaleNew(depth));

                    return { isColliding: true, contacts: [new Contact(a, b, start, end, normal, depth)] };
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
                        return { isColliding: false };
                    } else {
                        // Detected collision in region B:
                        const a = polygon;
                        const b = circle;
                        const depth = circleShape.radius - v1.magnitude();
                        const normal = v1.normalize();
                        const start = circle.position.addNew(normal.scaleNew(-circleShape.radius));
                        const end = start.addNew(normal.scaleNew(depth));

                        return { isColliding: true, contacts: [new Contact(a, b, start, end, normal, depth)] };
                    }
                } else {
                    ///////////////////////////////////////
                    // We are inside region C:
                    ///////////////////////////////////////
                    if (distanceCircleEdge > circleShape.radius) {
                        // No collision... Distance between the closest distance and the circle center is greater than the radius.
                        return { isColliding: false };
                    } else {
                        // Detected collision in region C:
                        const a = polygon;
                        const b = circle;
                        const depth = circleShape.radius - distanceCircleEdge;
                        const normal = minNextVertex.subNew(minCurrVertex).normal();
                        const start = circle.position.subNew(normal.scaleNew(circleShape.radius));
                        const end = start.addNew(normal.scaleNew(depth));

                        return { isColliding: true, contacts: [new Contact(a, b, start, end, normal, depth)] };
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

            return { isColliding: true, contacts: [new Contact(a, b, start, end, normal, depth)] };
        }
    };
}
