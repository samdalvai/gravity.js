import Graphics from '../Graphics';
import Vec2 from '../math/Vec2';
import { Box } from '../new/box';
import { RigidBody } from '../new/rigidbody';
import Body from './Body';
import CollisionDetection from './CollisionDetection';
import { ContactConstraint, JointConstraint } from './Constraint';
import Force from './Force';

export default class World {
    private uid = 0;

    private G: number;
    private iterations: number;

    public bodies: RigidBody[] = [];

    constructor(gravity: number, iterations = 10) {
        this.G = -gravity;
        this.iterations = iterations;
    }

    register(b: RigidBody) {
        // TODO: let a body increment his own id
        b.id = this.uid++;
        this.bodies.push(b);
    }

    update = (dt: number): void => {
        const invDt = dt > 0.0 ? 1.0 / dt : 0.0;
    };

    clear = () => {
        // this.bodies.length = 0;
        // this.joints.length = 0;
        // this.forces.length = 0;
        // this.torques.length = 0;
    };
}
