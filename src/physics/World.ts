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
        // console.time('update');
        const invDt = dt > 0.0 ? 1.0 / dt : 0.0;

        // Loop all bodies of the world applying forces
        for (const body of this.bodies) {
            // Apply the weight force to all bodies
            const weightForce = Force.generateWeightForce(body, this.G);
            body.addForce(weightForce);

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

        // console.time('contacts');
        this.bodies.sort((a, b) => a.minX - b.minX);
        const potentialPairs: [Body, Body][] = [];

        // Broad phase check with prune & sweep algorithm
        for (let i = 0; i < this.bodies.length; i++) {
            const a = this.bodies[i];

            for (let j = i + 1; j < this.bodies.length; j++) {
                const b = this.bodies[j];

                // If objects don't overlap on X axis they cannot collide
                if (b.minX > a.maxX) break;

                // If objects overlap on X axis but don't overlap on Y axis the cannot collide
                if (a.maxY < b.minY || a.minY > b.maxY) {
                    continue;
                }

                // Objects may be colliding
                potentialPairs.push([a, b]);
            }
        }

        this.contacts.length = 0;
        // Narrow phase check, potential pairs may still not collide
        for (const [a, b] of potentialPairs) {
            CollisionDetection.detectCollision(a, b, this.contacts);
        }

        // console.timeEnd('contacts');
        // console.time('solver');

        // Solve all constraints
        for (const joint of this.joints) {
            joint.preSolve(invDt);
        }

        for (const contact of this.contacts) {
            contact.preSolve(invDt);
        }

        for (let i = 0; i < this.iterations; i++) {
            for (const joint of this.joints) {
                joint.solve();
            }

            for (const contact of this.contacts) {
                contact.solve();
            }
        }

        for (const joint of this.joints) {
            joint.postSolve();
        }

        for (const contact of this.contacts) {
            contact.postSolve();
        }

        // console.timeEnd('solver');
        // console.time('Integrate');

        // Integrate all the velocities
        for (const body of this.bodies) {
            body.integrateVelocities(dt);
        }
        // console.timeEnd('Integrate');

        // Kill objects that went out of the screen
        for (let i = 0; i < this.bodies.length; i++) {
            const body = this.bodies[i];

            // It suffices to look for the position going below the screen
            if (body.position.y > Graphics.height()) {
                this.bodies[i] = this.bodies[this.bodies.length - 1];
                this.bodies.pop();
            }
        }

        // console.timeEnd('update');
    };

    clear = () => {
        this.bodies.length = 0;
        this.joints.length = 0;
        this.forces.length = 0;
        this.torques.length = 0;
    };
}
