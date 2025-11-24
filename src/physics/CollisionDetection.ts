import Body from './Body';
import Contact from './Contact';
import { CircleShape, PolygonShape, ShapeType } from './Shape';
import Vec2 from './Vec2';

export default class CollisionDetection {
    static isColliding = (a: Body, b: Body, contact: Contact): boolean => {
        const aIsCircle = a.shape.getType() === ShapeType.CIRCLE;
        const bIsCircle = b.shape.getType() === ShapeType.CIRCLE;
        const aIsPolygon = a.shape.getType() === ShapeType.POLYGON || a.shape.getType() === ShapeType.BOX;
        const bIsPolygon = b.shape.getType() === ShapeType.POLYGON || b.shape.getType() === ShapeType.BOX;

        if (aIsCircle && bIsCircle) {
            return this.isCollidingCircleCircle(a, b, contact);
        }

        if (aIsPolygon && bIsPolygon) {
            return this.isCollidingPolygonPolygon(a, b, contact);
        }

        if (aIsPolygon && bIsCircle) {
            return this.isCollidingPolygonCircle(a, b, contact);
        }

        if (aIsCircle && bIsPolygon) {
            return this.isCollidingPolygonCircle(b, a, contact);
        }

        return false;
    };

    static isCollidingCircleCircle = (a: Body, b: Body, contact: Contact): boolean => {
        const aCircleShape = a.shape as CircleShape;
        const bCircleShape = b.shape as CircleShape;

        const ab = b.position.subNew(a.position);
        const radiusSum = aCircleShape.radius + bCircleShape.radius;

        const isColliding = ab.magnitudeSquared() <= radiusSum * radiusSum;

        if (!isColliding) {
            return false;
        }

        contact.a = a;
        contact.b = b;

        contact.normal = ab;
        contact.normal.normalize();

        contact.start = b.position.subNew(contact.normal.scaleNew(bCircleShape.radius));
        contact.end = a.position.addNew(contact.normal.scaleNew(aCircleShape.radius));
        contact.depth = contact.end.subNew(contact.start).magnitude();

        return true;
    };

    static isCollidingPolygonPolygon = (a: Body, b: Body, contact: Contact): boolean => {
        const aPolygonShape = a.shape as PolygonShape;
        const bPolygonShape = b.shape as PolygonShape;

        const aAxis = new Vec2();
        const bAxis = new Vec2();
        const aPoint = new Vec2();
        const bPoint = new Vec2();

        const abSeparation = aPolygonShape.findMinSeparation(bPolygonShape, aAxis, aPoint);
        if (abSeparation >= 0) {
            return false;
        }

        const baSeparation = bPolygonShape.findMinSeparation(aPolygonShape, bAxis, bPoint);
        if (baSeparation >= 0) {
            return false;
        }

        contact.a = a;
        contact.b = b;
        if (abSeparation > baSeparation) {
            contact.depth = -abSeparation;
            contact.normal = aAxis.normal();
            contact.start = aPoint;
            contact.end = aPoint.addNew(contact.normal.scaleNew(contact.depth));
        } else {
            contact.depth = -baSeparation;
            contact.normal = bAxis.normal().negate();
            contact.start = bPoint.subNew(contact.normal.scaleNew(contact.depth));
            contact.end = bPoint;
        }
        return true;
    };

    static isCollidingPolygonCircle = (polygon: Body, circle: Body, contact: Contact): boolean => {
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
            const normal = edge.normal();

            // Compare the circle center with the rectangle vertex
            const vertexToCircleCenter = circle.position.subNew(polygonVertices[currVertex]);
            const projection = vertexToCircleCenter.dot(normal);

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
                    return false;
                } else {
                    // Detected collision in region A:
                    contact.a = polygon;
                    contact.b = circle;
                    contact.depth = circleShape.radius - v1.magnitude();
                    contact.normal = v1.normalize();
                    contact.start = circle.position.addNew(contact.normal.scaleNew(-circleShape.radius));
                    contact.end = contact.start.addNew(contact.normal.scaleNew(contact.depth));
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
                        return false;
                    } else {
                        // Detected collision in region B:
                        contact.a = polygon;
                        contact.b = circle;
                        contact.depth = circleShape.radius - v1.magnitude();
                        contact.normal = v1.normalize();
                        contact.start = circle.position.addNew(contact.normal.scaleNew(-circleShape.radius));
                        contact.end = contact.start.addNew(contact.normal.scaleNew(contact.depth));
                    }
                } else {
                    ///////////////////////////////////////
                    // We are inside region C:
                    ///////////////////////////////////////
                    if (distanceCircleEdge > circleShape.radius) {
                        // No collision... Distance between the closest distance and the circle center is greater than the radius.
                        return false;
                    } else {
                        // Detected collision in region C:
                        contact.a = polygon;
                        contact.b = circle;
                        contact.depth = circleShape.radius - distanceCircleEdge;
                        contact.normal = minNextVertex.subNew(minCurrVertex).normal();
                        contact.start = circle.position.subNew(contact.normal.scaleNew(circleShape.radius));
                        contact.end = contact.start.addNew(contact.normal.scaleNew(contact.depth));
                    }
                }
            }
        } else {
            // The center of circle is inside the polygon... it is definitely colliding!
            contact.a = polygon;
            contact.b = circle;
            contact.depth = circleShape.radius - distanceCircleEdge;
            contact.normal = minNextVertex.subNew(minCurrVertex).normal();
            contact.start = circle.position.subNew(contact.normal.scaleNew(circleShape.radius));
            contact.end = contact.start.addNew(contact.normal.scaleNew(contact.depth));
        }

        return true;
    };
}
