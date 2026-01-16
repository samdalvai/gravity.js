import Graphics from '../Graphics';
import Vec2 from '../math/Vec2';
import { Box } from '../new/box';
import { ContactManifold } from '../new/constraint/contact';
import { Joint } from '../new/constraint/joint';
import { detectCollision } from '../new/detection';
import { Vector2 } from '../new/math/vector2';
import { RigidBody, Type } from '../new/rigidbody';
import { Settings } from '../new/settings';
import * as Util from '../new/util';
import Body from './Body';
import CollisionDetection from './CollisionDetection';
import { PIXELS_PER_METER } from './Constants';
import { ContactConstraint, JointConstraint } from './Constraint';
import Force from './Force';

export default class World {
    private uid = 0;

    private G: number;
    private iterations: number;

    public bodies: RigidBody[] = [];

    // Constraints to be solved
    public manifolds: ContactManifold[] = [];
    public joints: Joint[] = [];

    public manifoldMap: Map<number, ContactManifold> = new Map();
    public jointMap: Map<number, Joint> = new Map();

    constructor(gravity: number, iterations = 10) {
        this.G = gravity;
        this.iterations = iterations;
    }

    register(b: RigidBody) {
        // TODO: let a body increment his own id
        b.id = this.uid++;
        this.bodies.push(b);
    }

    update = (deltaTime: number): void => {
        const inverseDeltaTime = deltaTime > 0.0 ? 1.0 / deltaTime : 0.0;
        const newManifolds: ContactManifold[] = [];
        const newManifoldMap: Map<number, ContactManifold> = new Map();

        // Check collisions with brute force algorithm
        for (let i = 0; i < this.bodies.length - 1; i++) {
            for (let j = i + 1; j < this.bodies.length; j++) {
                let a = this.bodies[i];
                let b = this.bodies[j];

                // Improve coherence
                if (a.id > b.id) {
                    a = this.bodies[j];
                    b = this.bodies[i];
                }

                if (a.type == Type.Static && b.type == Type.Static) continue;

                const key = Util.make_pair_natural(a.id, b.id);
                // if (this.passTestSet.has(key)) continue;

                const newManifold = detectCollision(a, b);
                if (newManifold == null) continue;

                // REVERSE: reset sleeping if a contact is new
                const isNewContact = !this.manifoldMap.has(key);

                if (isNewContact) {
                    a.awake();
                    b.awake();
                }

                a.manifoldIDs.push(key);
                b.manifoldIDs.push(key);

                if (Settings.warmStarting && this.manifoldMap.has(key)) {
                    const oldManifold = this.manifoldMap.get(key)!;
                    newManifold.tryWarmStart(oldManifold);
                }

                newManifoldMap.set(key, newManifold);
                newManifolds.push(newManifold);
            }
        }

        this.manifoldMap = newManifoldMap;
        this.manifolds = newManifolds;

        // Integrate forces, yield tentative velocities that possibly violate the constraint
        for (let i = 0; i < this.bodies.length; i++) {
            const b = this.bodies[i];

            if (b.sleeping) {
                b.linearVelocity.clear();
                b.angularVelocity = 0;
            }

            // if (this.forceIntegration) {
            //     const linear_a = b.force.mulNew(b.inverseMass * deltaTime); // Force / mass * dt
            //     b.linearVelocity.x += linear_a.x;
            //     b.linearVelocity.y += linear_a.y;

            //     const angular_a = b.torque * b.inverseInertia * deltaTime; // Torque / inertia * dt
            //     b.angularVelocity += angular_a;

            //     if (
            //         (this.sleeping && linear_a.squaredLength >= Settings.restLinearTolerance) ||
            //         angular_a * angular_a >= Settings.restAngularTolerance
            //     ) {
            //         this.sleeping = false;
            //         awakeIsland = true;
            //     }
            // }

            if (
                b.linearVelocity.squaredLength < Settings.restLinearTolerance &&
                b.angularVelocity * b.angularVelocity < Settings.restAngularTolerance
            ) {
                b.resting += deltaTime;
            } else {
                b.resting = 0;
            }

            if (b.resting > Settings.sleepingWait) {
                b.sleeping = true;
            }

            // Apply gravity
            if (Settings.applyGravity && !b.sleeping) {
                const gravity = new Vector2(0, this.G * PIXELS_PER_METER * deltaTime);
                b.linearVelocity.x += gravity.x;
                b.linearVelocity.y += gravity.y;
            }
        }

        // Prepare for solving
        for (let i = 0; i < this.manifolds.length; i++) this.manifolds[i].prepare(inverseDeltaTime);

        for (let i = 0; i < this.joints.length; i++) this.joints[i].prepare(inverseDeltaTime);

        // Iteratively solve the violated velocity constraint
        for (let i = 0; i < Settings.numIterations; i++) {
            for (let j = 0; j < this.manifolds.length; j++) this.manifolds[j].solve();

            for (let j = 0; j < this.joints.length; j++) this.joints[j].solve();
        }

        for (let i = 0; i < this.bodies.length; i++) {
            const b = this.bodies[i];

            if (b.sleeping) continue;

            // if (awakeIsland) b.awake();

            b.force.clear();
            b.torque = 0;

            b.position.x += b.linearVelocity.x * deltaTime;
            b.position.y += b.linearVelocity.y * deltaTime;
            b.rotation += b.angularVelocity * deltaTime;

            // if (b.position.y < Settings.deadBottom) this.unregister(b.id);
        }
    };

    clear = () => {
        // this.bodies.length = 0;
        // this.joints.length = 0;
        // this.forces.length = 0;
        // this.torques.length = 0;
    };
}
