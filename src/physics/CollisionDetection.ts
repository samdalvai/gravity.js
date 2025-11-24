import Body from './Body';
import Contact from './Contact';

export default class CollisionDetection {
    static isColliding = (a: Body, b: Body, contact: Contact): boolean => {
        // bool aIsCircle = a->shape->GetType() == CIRCLE;
        // bool bIsCircle = b->shape->GetType() == CIRCLE;

        // if (aIsCircle && bIsCircle) {
        //     return IsCollidingCircleCircle(a, b, contact);
        // }

        return false;
    };
    static isCollidingCircleCircle = (a: Body, b: Body, contact: Contact): boolean => {
        // CircleShape* aCircleShape = (CircleShape*) a->shape;
        // CircleShape* bCircleShape = (CircleShape*) b->shape;

        // const Vec2 ab = b->position - a->position;
        // const float radiusSum = aCircleShape->radius + bCircleShape->radius;

        // bool isColliding = ab.MagnitudeSquared() <= (radiusSum * radiusSum);

        // if (!isColliding) {
        //     return false;
        // }

        // contact.a = a;
        // contact.b = b;

        // contact.normal = ab;
        // contact.normal.Normalize();

        // contact.start = b->position - contact.normal * bCircleShape->radius;
        // contact.end = a->position + contact.normal * aCircleShape->radius;

        // contact.depth = (contact.end - contact.start).Magnitude();
        
        return true;
    };

    // TODO: static isCollidingPolygonPolygon = (a: Body, b: Body): boolean => {return false};
    // TODO: static isCollidingPolygonCircle(a: Body, b: Body): boolean => {return false};
}
