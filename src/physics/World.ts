import Graphics from '../Graphics';
import Vec2 from '../math/Vec2';
import { ContactManifold } from '../new/contact_adapted';
import { Settings } from '../new/settings';
import Body from './Body';
import CollisionDetection from './CollisionDetection';
import { JointConstraint } from './Constraint';
import Force from './Force';

export default class World {
    private G: number;
    private iterations: number;

    private bodies: Body[] = [];

    // Constraints to be solved
    public manifolds: ContactManifold[] = [];
    public joints: JointConstraint[] = [];

    public manifoldMap: Map<number, ContactManifold> = new Map();
    public jointMap: Map<number, JointConstraint> = new Map();

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

    getManifolds = (): ContactManifold[] => {
        return this.manifolds;
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
        const newManifolds: ContactManifold[] = [];
        const newManifoldMap: Map<number, ContactManifold> = new Map();

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

        // Narrow phase check, potential pairs may still not collide
        for (let [a, b] of potentialPairs) {
            // TODO: enable this when testing is over
            // if (a.isStatic() && b.isStatic()) continue;

            // Improve coherence
            if (a.id > b.id) {
                [a, b] = [b, a];
            }

            const newManifold = CollisionDetection.detectCollision(a, b);
            if (newManifold == null) continue;

            const key = Body.pairKey(a, b);
            if (Settings.warmStarting && this.manifoldMap.has(key)) {
                const oldManifold = this.manifoldMap.get(key)!;
                newManifold.tryWarmStart(oldManifold);
            }

            newManifoldMap.set(key, newManifold);
            newManifolds.push(newManifold);
        }
        // console.timeEnd('collision');

        this.manifoldMap = newManifoldMap;
        this.manifolds = newManifolds;

        // Prepare for solving
        for (let i = 0; i < this.manifolds.length; i++) {
            const m = this.manifolds[i];

            // TODO: move this to main loop after tests are finished
            if (m.bodyA.isStatic() && m.bodyB.isStatic()) {
                continue;
            }

            this.manifolds[i].prepare(invDt);
        }

        for (let i = 0; i < this.joints.length; i++) {
            this.joints[i].preSolve(invDt);
        }

        // Iteratively solve the violated velocity constraint
        for (let i = 0; i < this.iterations; i++) {
            for (let j = 0; j < this.manifolds.length; j++) {
                const m = this.manifolds[j];

                // TODO: move this to main loop after tests are finished
                if (m.bodyA.isStatic() && m.bodyB.isStatic()) {
                    continue;
                }
                this.manifolds[j].solve();
            }

            for (let j = 0; j < this.joints.length; j++) this.joints[j].solve();
        }

        // Integrate all the velocities
        for (const body of this.bodies) {
            body.integrateVelocities(dt);
        }

        // Kill objects that went out of the screen
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
