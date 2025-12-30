import Graphics from '../Graphics';
import Vec2 from '../math/Vec2';
import Body from './Body';
import CollisionDetection from './CollisionDetection';
import { ContactConstraint, JointConstraint } from './Constraint';
import Force from './Force';

export default class World {
    private G: number;
    private iterations: number;

    private bodies: Body[] = [];
    private contacts: ContactConstraint[] = [];
    private joints: JointConstraint[] = [];

    private forces: Vec2[] = [];
    private torques: number[] = [];

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

    getContacts = (): ContactConstraint[] => {
        return this.contacts;
    };

    addJoint = (constraint: JointConstraint): void => {
        this.joints.push(constraint);
    };

    getJoints = (): JointConstraint[] => {
        return this.joints;
    };

    addForce = (force: Vec2): void => {
        this.forces.push(force);
    };

    addTorque = (torque: number): void => {
        this.torques.push(torque);
    };

    update = (dt: number): void => {
        const invDt = dt > 0.0 ? 1.0 / dt : 0.0;

        // Loop all bodies of the world applying forces
        for (const body of this.bodies) {
            // Apply the weight force to all bodies
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
        // console.time('contacts');

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

        // console.timeEnd('contacts');

        console.time('solver');

        // Solve all constraints
        for (const constraint of this.joints) {
            // for (const constraint of this.constraints) {
            constraint.preSolve(invDt);
        }

        for (const constraint of this.contacts) {
            constraint.preSolve(invDt);
        }

        for (let i = 0; i < this.iterations; i++) {
            for (const constraint of this.joints) {
                // for (const constraint of this.constraints) {
                constraint.solve();
            }

            for (const constraint of this.contacts) {
                constraint.solve();
            }
        }

        for (const constraint of this.joints) {
            constraint.postSolve();
        }

        for (const constraint of this.contacts) {
            constraint.postSolve();
        }

        console.timeEnd('solver');

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
        this.joints.length = 0;
        this.forces.length = 0;
        this.torques.length = 0;
    };
}
