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
        if (!this.a || !this.b || this.depth === null || !this.normal) {
            console.error('Some Contact variables are not initialized: ', this);
            throw new Error('Some Contact variables are not initialized');
        }

        if (this.a.isStatic() && this.b.isStatic()) {
            return;
        }

        const da = (this.depth / (this.a.invMass + this.b.invMass)) * this.a.invMass;
        const db = (this.depth / (this.a.invMass + this.b.invMass)) * this.b.invMass;

        this.a.position.subAssign(this.normal.scaleNew(da));
        this.b.position.addAssign(this.normal.scaleNew(db));

        this.a.shape.updateVertices(this.a.rotation, this.a.position);
        this.b.shape.updateVertices(this.b.rotation, this.b.position);
    };

    // Resolves the collision using the impulse method
    resolveCollision = (): void => {
        if (!this.a || !this.b || this.depth === null || !this.normal || !this.start || !this.end) {
            console.error('Some Contact variables are not initialized: ', this);
            throw new Error('Some Contact variables are not initialized');
        }

        // Apply positional correction using the projection method
        this.resolvePenetration();

        // Define coefficient of restitution (elasticity) and friction
        const e = Math.min(this.a.restitution, this.b.restitution);
        const f = Math.min(this.a.friction, this.b.friction);

        // Calculate the relative velocity between the two objects
        const ra = this.end.subNew(this.a.position);
        const rb = this.start.subNew(this.b.position);
        const va = this.a.velocity.addNew(new Vec2(-this.a.angularVelocity * ra.y, this.a.angularVelocity * ra.x));
        const vb = this.b.velocity.addNew(new Vec2(-this.b.angularVelocity * rb.y, this.b.angularVelocity * rb.x));
        const vrel = va.subNew(vb);

        // Now we proceed to calculate the collision impulse along the normal
        const vrelDotNormal = vrel.dot(this.normal);
        const impulseDirectionN = this.normal;
        const impulseMagnitudeN =
            (-(1 + e) * vrelDotNormal) /
            (this.a.invMass +
                this.b.invMass +
                ra.cross(this.normal) * ra.cross(this.normal) * this.a.invI +
                rb.cross(this.normal) * rb.cross(this.normal) * this.b.invI);
        const jN = impulseDirectionN.scaleNew(impulseMagnitudeN);

        // Now we proceed to calculate the collision impulse along the tangent
        const tangent = this.normal.normal();
        const vrelDotTangent = vrel.dot(tangent);
        const impulseDirectionT = tangent;
        const impulseMagnitudeT =
            (f * -(1 + e) * vrelDotTangent) /
            (this.a.invMass +
                this.b.invMass +
                ra.cross(tangent) * ra.cross(tangent) * this.a.invI +
                rb.cross(tangent) * rb.cross(tangent) * this.b.invI);
        const jT = impulseDirectionT.scaleNew(impulseMagnitudeT);

        // Calculate the final impulse j combining normal and tangent impulses
        const j = jN.addNew(jT);

        // Apply the impulse vector to both objects in opposite direction
        this.a.applyImpulseAtPoint(j, ra);
        this.b.applyImpulseAtPoint(j.negate(), rb);
    };
}
