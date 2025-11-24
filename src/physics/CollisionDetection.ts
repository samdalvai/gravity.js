import Body from './Body';
import Contact from './Contact';
import { CircleShape, ShapeType } from './Shape';

export default class CollisionDetection {
    static isColliding = (a: Body, b: Body, contact: Contact): boolean => {
        const aIsCircle = a.shape.getType() === ShapeType.CIRCLE;
        const bIsCircle = b.shape.getType() === ShapeType.CIRCLE;

        if (aIsCircle && bIsCircle) {
            return this.isCollidingCircleCircle(a, b, contact);
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

    // TODO: static isCollidingPolygonPolygon = (a: Body, b: Body): boolean => {return false};
    // TODO: static isCollidingPolygonCircle(a: Body, b: Body): boolean => {return false};
}
