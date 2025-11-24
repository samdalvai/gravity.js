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
        if (!this.a || !this.b || !this.depth || !this.normal) {
            console.error('Some Contact variables are not initialized: ', this);
            throw new Error('Some Contact variables are not initialized');
        }

        // Apply positional correction using the projection method
        this.resolvePenetration();

        // Define elasticity (coefficient of restitution e)
        const e = Math.min(this.a.restitution, this.b.restitution);

        // Calculate the relative velocity between the two objects
        const vrel = this.a.velocity.subNew(this.b.velocity);

        // Calculate the relative velocity along the normal collision vector
        const vrelDotNormal = vrel.dot(this.normal);

        // Now we proceed to calculate the collision impulse
        // TODO: why we assign normal to impulseDirection? Is this needed?
        const impulseDirection = this.normal;
        const impulseMagnitude = (-(1 + e) * vrelDotNormal) / (this.a.invMass + this.b.invMass);

        const jn = impulseDirection.scaleNew(impulseMagnitude);

        // Apply the impulse vector to both objects in opposite direction
        this.a.applyImpulse(jn);
        this.b.applyImpulse(jn.negate());
    };
}
