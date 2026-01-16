import Graphics from '../Graphics';
import Vec2 from '../math/Vec2';
import { Box } from '../new/box';
import { Vector2 } from '../new/math/vector2';
import { RigidBody } from '../new/rigidbody';
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

        for (let i = 0; i < this.bodies.length; i++) {
            const b = this.bodies[i];

            const gravity = new Vector2(0, b.mass * this.G);
            b.linearVelocity.x += gravity.x;
            b.linearVelocity.y += gravity.y;
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
