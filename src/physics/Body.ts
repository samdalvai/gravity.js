import { Shape } from './Shape';
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

    // TODO: fill this methods
    // bool IsStatic() const;

    // void AddForce(const Vec2& force);
    // void AddTorque(float torque);
    // void ClearForces();
    // void ClearTorque();

    // void ApplyImpulse(const Vec2& j);

    // void IntegrateLinear(float dt);
    // void IntegrateAngular(float dt);

    // void Update(float dt);
}
