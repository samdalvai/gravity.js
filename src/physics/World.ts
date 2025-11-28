import Body from './Body';
import CollisionDetection from './CollisionDetection';
import { PIXELS_PER_METER } from './Constants';
import Contact from './Contact';
import Vec2 from './Vec2';

export default class World {
    private G = 9.8;
    private bodies: Body[] = [];
    private forces: Vec2[] = [];
    private torques: number[] = [];

    constructor(gravity: number) {
        this.G = -gravity;
    }

    addBody = (body: Body): void => {
        this.bodies.push(body);
    };

    getBodies = (): Body[] => {
        return this.bodies;
    };

    addForce = (force: Vec2): void => {
        this.forces.push(force);
    };

    addTorque = (torque: number): void => {
        this.torques.push(torque);
    };

    update = (dt: number): void => {
        // Loop all bodies of the world applying forces
        for (const body of this.bodies) {
            // Apply the weight force to all bodies
            const weight = new Vec2(0.0, body.mass * this.G * PIXELS_PER_METER);
            body.addForce(weight);

            // Apply forces to all bodies
            for (const force of this.forces) {
                body.addForce(force);
            }
            // Apply torque to all bodies
            for (const torque of this.torques) {
                body.addTorque(torque);
            }
        }

        // Update all the bodies in the world (integrating and transforming vertices)
        for (const body of this.bodies) {
            body.update(dt);
        }
        // Collision detection and resolution for all bodies of the world
        this.checkCollisions();
    };

    checkCollisions = () => {
        // Check all the bodies with all other bodies detecting collisions
        for (let i = 0; i <= this.bodies.length - 1; i++) {
            for (let j = i + 1; j < this.bodies.length; j++) {
                const a = this.bodies[i];
                const b = this.bodies[j];
                a.isColliding = false;
                b.isColliding = false;
                const contact = new Contact();

                if (CollisionDetection.isColliding(a, b, contact)) {
                    // Resolve the collision
                    contact.resolveCollision();
                }
            }
        }
    };
}
