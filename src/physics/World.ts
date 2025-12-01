import Graphics from '../Graphics';
import Body from './Body';
import CollisionDetection from './CollisionDetection';
import { PIXELS_PER_METER } from './Constants';
import { Constraint, PenetrationConstraint } from './Constraint';
import Vec2 from './Vec2';

export default class World {
    private G = 9.8;
    private bodies: Body[] = [];
    private constraints: Constraint[] = [];

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

    addConstraint = (constraint: Constraint): void => {
        this.constraints.push(constraint);
    };

    getConstraints = (): Constraint[] => {
        return this.constraints;
    };

    addForce = (force: Vec2): void => {
        this.forces.push(force);
    };

    addTorque = (torque: number): void => {
        this.torques.push(torque);
    };

    update = (dt: number): void => {
        const penetrations: PenetrationConstraint[] = [];

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

        // Integrate all the forces
        for (const body of this.bodies) {
            body.integrateForces(dt);
        }

        // Check all the bodies with all other bodies detecting collisions
        for (let i = 0; i <= this.bodies.length - 1; i++) {
            for (let j = i + 1; j < this.bodies.length; j++) {
                const a = this.bodies[i];
                const b = this.bodies[j];

                const collisionResult = CollisionDetection.detectCollision(a, b);

                if (collisionResult.isColliding) {
                    for (const contact of collisionResult.contacts) {
                        // Draw collision points
                        // Graphics.drawCircle(contact.start.x, contact.start.y, 5, 0.0, 'red');
                        // Graphics.drawCircle(contact.end.x, contact.end.y, 2, 0.0, 'red');

                        // Create a new penetration constraint
                        const penetration = new PenetrationConstraint(
                            contact.a,
                            contact.b,
                            contact.start,
                            contact.end,
                            contact.normal,
                        );
                        penetrations.push(penetration);
                    }
                }
            }
        }

        // Solve all constraints
        for (const constraint of this.constraints) {
            constraint.preSolve(dt);
        }

        for (const constraint of penetrations) {
            constraint.preSolve(dt);
        }

        for (let i = 0; i < 5; i++) {
            for (const constraint of this.constraints) {
                constraint.solve();
            }

            for (const constraint of penetrations) {
                constraint.solve();
            }
        }

        for (const constraint of this.constraints) {
            constraint.postSolve();
        }

        for (const constraint of penetrations) {
            constraint.postSolve();
        }

        // Integrate all the velocities
        for (const body of this.bodies) {
            body.integrateVelocities(dt);
        }
    };
}
