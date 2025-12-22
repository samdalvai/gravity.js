import Graphics from '../Graphics';
import Vec2 from '../math/Vec2';
import Body from './Body';
import CollisionDetection from './CollisionDetection';
import { JointConstraint, PenetrationConstraint } from './Constraint';
import Contact from './Contact';
import Force from './Force';

export default class World {
    private G: number;
    private iterations: number;

    private bodies: Body[] = [];
    private contacts: Contact[] = [];
    // private constraints: Constraint[] = [];
    private jointConstraints: JointConstraint[] = [];

    private forces: Vec2[] = [];
    private torques: number[] = [];

    private debug = true;

    constructor(gravity: number, iterations = 10) {
        this.G = -gravity;
        this.iterations = iterations;
    }

    addBody = (body: Body): void => {
        this.bodies.push(body);
    };

    getBodies = (): Body[] => {
        return this.bodies;
    };

    getContacts = (): Contact[] => {
        return this.contacts;
    };

    addConstraint = (constraint: JointConstraint): void => {
        // addConstraint = (constraint: Constraint): void => {
        // this.constraints.push(constraint);
        this.jointConstraints.push(constraint);
    };

    // getConstraints = (): Constraint[] => {
    //     return this.constraints;
    // };
    getConstraints = (): JointConstraint[] => {
        return this.jointConstraints;
    };

    addForce = (force: Vec2): void => {
        this.forces.push(force);
    };

    addTorque = (torque: number): void => {
        this.torques.push(torque);
    };

    setDebug = (newValue: boolean): void => {
        this.debug = newValue;
    };

    update = (dt: number): void => {
        const invDt = dt > 0.0 ? 1.0 / dt : 0.0;
        const penetrations: PenetrationConstraint[] = [];

        // Loop all bodies of the world applying forces
        for (const body of this.bodies) {
            // Apply the weight force to all bodies
            // TODO: can we add this as force to the world? instead of recreating it each time? No, because it depends on the bodies mass
            const weightForce = Force.generateWeightForce(body, this.G);
            body.addForce(weightForce);

            // Apply friction to all bodies
            // TODO: counter friction should only be applied if in contact with another surface
            // const frictionForce = Force.generateFrictionForce(body, body.friction);
            // body.addForce(frictionForce);

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
        this.contacts.length = 0;

        for (let i = 0; i <= this.bodies.length - 1; i++) {
            for (let j = i + 1; j < this.bodies.length; j++) {
                const a = this.bodies[i];
                const b = this.bodies[j];

                // Broad phase check
                const ab = b.position.subNew(a.position);
                const radiusSum = a.shape.radius + b.shape.radius;

                if (ab.magnitudeSquared() <= radiusSum * radiusSum) {
                    // TODO: no need to recheck collision if the two shapes are circles, in that case
                    // return the contact info directly
                    CollisionDetection.detectCollision(a, b, this.contacts);
                }
            }
        }

        for (const contact of this.contacts) {
            penetrations.push(
                new PenetrationConstraint(contact.a, contact.b, contact.start, contact.end, contact.normal),
            );
        }

        // Solve all constraints
        for (const constraint of this.jointConstraints) {
            // for (const constraint of this.constraints) {
            constraint.preSolve(invDt);
        }

        for (const constraint of penetrations) {
            constraint.preSolve(invDt);
        }

        for (let i = 0; i < this.iterations; i++) {
            for (const constraint of this.jointConstraints) {
                // for (const constraint of this.constraints) {
                constraint.solve();
            }

            for (const constraint of penetrations) {
                constraint.solve();
            }
        }

        for (const constraint of this.jointConstraints) {
            constraint.postSolve();
        }

        for (const constraint of penetrations) {
            constraint.postSolve();
        }

        // Integrate all the velocities
        for (const body of this.bodies) {
            body.integrateVelocities(dt);
        }

        // Remove objects that went out of the screen
        for (let i = 0; i < this.bodies.length; i++) {
            const body = this.bodies[i];

            // It suffices to look for the position going below the screen
            if (body.position.y > Graphics.height()) {
                this.bodies[i] = this.bodies[this.bodies.length - 1];
                this.bodies.pop();
            }
        }
    };

    clear = () => {
        this.bodies.length = 0;
        this.jointConstraints.length = 0;
        this.forces.length = 0;
        this.torques.length = 0;
    };
}
