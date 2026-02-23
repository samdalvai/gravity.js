import AssetStore, { TEXTURES } from '../graphics/AssetStore';
import Vec2 from '../math/Vec2';
import { SETTINGS } from './Constants';
import { Shape, ShapeType } from '../shapes/Shape';

export default class RigidBody {
    static nextId = 0;
    id: number;

    // Linear motion
    position: Vec2;
    velocity: Vec2;
    acceleration: Vec2;

    // Angular motion
    rotation: number;
    angularVelocity: number;
    angularAcceleration: number;
    canRotate: boolean;

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
    friction: number;
    surfaceSpeed: number;

    // Grounded variables
    isGrounded = false;
    lastGroundedTime = 0;

    // Pointer to the shape/geometry of this rigid body
    shape: Shape;
    shapeType: ShapeType;
    shapeFillColor = 'gray';

    texture: CanvasImageSource | null = null;

    minX!: number;
    maxX!: number;
    minY!: number;
    maxY!: number;

    constructor(shape: Shape, x: number, y: number, mass: number) {
        this.id = RigidBody.nextId++;

        this.shape = shape;
        this.shapeType = shape.getType();

        this.position = new Vec2(x, y);
        this.velocity = new Vec2(0, 0);
        this.acceleration = new Vec2(0, 0);

        this.rotation = 0.0;
        this.angularVelocity = 0.0;
        this.angularAcceleration = 0.0;
        this.canRotate = true;

        this.sumForces = new Vec2(0, 0);
        this.sumTorque = 0.0;

        this.restitution = 0.2;
        this.friction = 0.7;
        this.surfaceSpeed = 0;

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

        this.shape.updateVertices(this.rotation, this.position);
        this.shape.updateAABB(this);
    }

    // a.id << 16 → shifts a.id into the upper 16 bits of a 32-bit integer
    // b.id & 0xffff → ensures that only the lower 16 bits of b.id are used
    // | -> bitwise OR combines them into a single 32-bit integer
    static pairKey(a: RigidBody, b: RigidBody): number {
        if (a.id < b.id) {
            return (a.id << 16) | (b.id & 0xffff);
        } else {
            return (b.id << 16) | (a.id & 0xffff);
        }
    }

    setTexture(texture: keyof typeof TEXTURES): void {
        this.texture = AssetStore.getTexture(texture);
    }

    isStatic(): boolean {
        const epsilon = 0.005;
        return Math.abs(this.invMass - 0.0) < epsilon;
    }

    addForce(force: Vec2): void {
        this.sumForces.addAssign(force);
    }

    addTorque(torque: number): void {
        this.sumTorque += torque;
    }

    clearForces(): void {
        this.sumForces = new Vec2(0.0, 0.0);
    }

    clearTorque(): void {
        this.sumTorque = 0.0;
    }

    localPointToWorld(point: Vec2): Vec2 {
        const cos = Math.cos(this.rotation);
        const sin = Math.sin(this.rotation);

        const rotated = new Vec2(
            point.x * cos - point.y * sin + this.position.x,
            point.x * sin + point.y * cos + this.position.y,
        );

        return rotated;
    }

    worldPointToLocal(point: Vec2): Vec2 {
        const cos = Math.cos(-this.rotation);
        const sin = Math.sin(-this.rotation);

        const translatedX = point.x - this.position.x;
        const translatedY = point.y - this.position.y;
        const rotatedX = cos * translatedX - sin * translatedY;
        const rotatedY = cos * translatedY + sin * translatedX;

        return new Vec2(rotatedX, rotatedY);
    }

    worldDirToLocal(dir: Vec2): Vec2 {
        const cos = Math.cos(-this.rotation);
        const sin = Math.sin(-this.rotation);

        return new Vec2(cos * dir.x - sin * dir.y, sin * dir.x + cos * dir.y);
    }

    applyImpulseLinear(j: Vec2): void {
        if (this.isStatic()) {
            return;
        }

        this.velocity.x += j.x * this.invMass;
        this.velocity.y += j.y * this.invMass;
    }

    applyImpulseAngular(j: number): void {
        if (this.isStatic()) {
            return;
        }

        this.angularVelocity += j * this.invI;
    }

    applyImpulseAtPoint(j: Vec2, r: Vec2): void {
        if (this.isStatic()) {
            return;
        }
        this.velocity.addAssign(j.scaleNew(this.invMass));
        this.angularVelocity += r.cross(j) * this.invI;
    }

    integrateForces(dt: number): void {
        if (this.isStatic()) {
            return;
        }

        // Find the acceleration based on the forces that are being applied and the mass
        this.acceleration.x = this.sumForces.x * this.invMass;
        this.acceleration.y = this.sumForces.y * this.invMass;

        // Integrate the acceleration to find the new velocity
        this.velocity.x += this.acceleration.x * dt;
        this.velocity.y += this.acceleration.y * dt;

        // Find the angular acceleration based on the torque that is being applied and the moment of inertia
        this.angularAcceleration = this.sumTorque * this.invI;

        // Integrate the angular acceleration to find the new angular velocity
        this.angularVelocity += this.angularAcceleration * dt;

        // Clear all the forces and torque acting on the object before the next physics step
        this.clearForces();
        this.clearTorque();
    }

    integrateVelocities(dt: number): void {
        if (this.isStatic()) {
            // TODO: this is needed because otherwise AABB is not correctly set for static objects with rotation
            this.shape.updateVertices(this.rotation, this.position);
            this.shape.updateAABB(this);
            return;
        }

        // Integrate the velocity to find the new position
        this.position.x += this.velocity.x * dt;
        this.position.y += this.velocity.y * dt;

        // Integrate the angular velocity to find the new rotation angle
        if (this.canRotate) {
            this.rotation += this.angularVelocity * dt;

            // Clamp tiny angular velocity to avoid circles accumulating angular velocity
            if (Math.abs(this.angularVelocity) < SETTINGS.angularVelocitySlop) {
                this.angularVelocity = 0;
            }
        } else {
            this.angularVelocity = 0;
        }

        // Update the vertices to adjust them to the new position/rotation
        this.shape.updateVertices(this.rotation, this.position);

        // Update AABB values based on new position
        this.shape.updateAABB(this);

        // Bleed off angular velocity for grounded bodies
        if (this.isGrounded) {
            const rollingResistance = 0.5;
            this.angularVelocity *= 1 - rollingResistance * dt;
        }
    }
}
