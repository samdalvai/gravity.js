import { PolygonShape, Shape, ShapeType } from './Shape';
import Vec2 from './Vec2';

export default class Body {
    isColliding = false;

    // Linear motion
    position: Vec2;
    velocity: Vec2;
    acceleration: Vec2;

    // // Angular motion
    rotation: number;
    angularVelocity: number;
    angularAcceleration: number;

    // Forces and torque
    sumForces: Vec2;
    sumTorque: number;

    // Mass and Moment of Inertia
    mass: number;
    invMass: number;
    I: number;
    invI: number;

    // Coefficient of restitution (elasticity)
    restitution: number;

    // Pointer to the shape/geometry of this rigid body
    shape: Shape;

    // Body(const Shape& shape, float x, float y, float mass);
    constructor(shape: Shape, x: number, y: number, mass: number) {
        this.shape = shape;
        this.position = new Vec2(x, y);
        this.velocity = new Vec2(0, 0);
        this.acceleration = new Vec2(0, 0);
        this.rotation = 0.0;
        this.angularVelocity = 0.0;
        this.angularAcceleration = 0.0;
        this.sumForces = new Vec2(0, 0);
        this.sumTorque = 0.0;
        this.restitution = 1.0;
        this.mass = mass;
        if (mass != 0.0) {
            this.invMass = 1.0 / mass;
        } else {
            this.invMass = 0.0;
        }
        this.I = shape.getMomentOfInertia() * mass;
        if (this.I != 0.0) {
            this.invI = 1.0 / this.I;
        } else {
            this.invI = 0.0;
        }
    }

    isStatic = (): boolean => {
        const epsilon = 0.005;
        return Math.abs(this.invMass - 0.0) < epsilon;
    };

    addForce = (force: Vec2): void => {
        this.sumForces.addAssign(force);
    };

    addTorque = (torque: number): void => {
        this.sumTorque += torque;
    };

    clearForces = (): void => {
        this.sumForces = new Vec2(0.0, 0.0);
    };

    clearTorque = (): void => {
        this.sumTorque = 0.0;
    };

    applyImpulse = (j: Vec2): void => {
        if (this.isStatic()) {
            return;
        }
        this.velocity.addAssign(j.scaleNew(this.invMass));
    };

    integrateLinear = (dt: number): void => {
        if (this.isStatic()) {
            return;
        }

        // Find the acceleration based on the forces that are being applied and the mass
        this.acceleration = this.sumForces.scaleNew(this.invMass);

        // Integrate the acceleration to find the new velocity
        this.velocity.addAssign(this.acceleration.scaleNew(dt));

        // Integrate the velocity to find the new position
        this.position.addAssign(this.velocity.scaleNew(dt));

        // Clear all the forces acting on the object before the next physics step
        this.clearForces();
    };

    integrateAngular = (dt: number): void => {
        if (this.isStatic()) {
            return;
        }

        // Find the angular acceleration based on the torque that is being applied and the moment of inertia
        this.angularAcceleration = this.sumTorque * this.invI;

        // Integrate the angular acceleration to find the new angular velocity
        this.angularVelocity += this.angularAcceleration * dt;

        // Integrate the angular velocity to find the new rotation angle
        this.rotation += this.angularVelocity * dt;

        // Clear all the torque acting on the object before the next physics step
        this.clearTorque();
    };

    update = (dt: number): void => {
        this.integrateLinear(dt);
        this.integrateAngular(dt);
        
        const isPolygon = this.shape.getType() === ShapeType.POLYGON || this.shape.getType() === ShapeType.BOX;
        if (isPolygon) {
            (this.shape as PolygonShape).updateVertices(this.rotation, this.position);
        }
    };
}
