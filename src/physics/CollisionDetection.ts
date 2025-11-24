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

        // TODO: check circle with polygon collision
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

    // TODO: static isCollidingPolygonCircle(a: Body, b: Body): boolean => {return false};
}
