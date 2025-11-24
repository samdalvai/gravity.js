import Body from './Body';
import Vec2 from './Vec2';

export default class Contact {
    // TODO: find a way to avoid initializing all to null
    a: Body | null = null;
    b: Body | null = null;

    start: Vec2 | null = null;
    end: Vec2 | null = null;

    normal: Vec2 | null = null;
    depth: number | null = null;

    constructor() {}

    // Resolves the collision using the position change method
    resolvePenetration = (): void => {
        if (!this.a || !this.b || !this.depth || !this.normal) {
            throw new Error('Some Contact variables are not initialized');
        }

        if (this.a.isStatic() && this.b.isStatic()) {
            return;
        }

        const da = (this.depth / (this.a.invMass + this.b.invMass)) * this.a.invMass;
        const db = (this.depth / (this.a.invMass + this.b.invMass)) * this.b.invMass;

        this.a.position.subAssign(this.normal.scaleNew(da));
        this.b.position.addAssign(this.normal.scaleNew(db));
    };

    // Resolves the collision using the impulse method
    resolveCollision = (): void => {
        //     // Apply positional correction using the projection method
        //     ResolvePenetration();

        //     // Define elasticity (coefficient of restitution e)
        //     float e = std::min(a->restitution, b->restitution);

        //     // Calculate the relative velocity between the two objects
        //     const Vec2 vrel = (a->velocity - b->velocity);

        //     // Calculate the relative velocity along the normal collision vector
        //     float vrelDotNormal = vrel.Dot(normal);

        //     // Now we proceed to calculate the collision impulse
        //     const Vec2 impulseDirection = normal;
        //     const float impulseMagnitude = -(1 + e) * vrelDotNormal / (a->invMass + b->invMass);

        //     Vec2 jn = impulseDirection * impulseMagnitude;

        //     // Apply the impulse vector to both objects in opposite direction
        //     a->ApplyImpulse(jn);
        //     b->ApplyImpulse(-jn);
    };
}
