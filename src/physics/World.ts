import Graphics from '../Graphics';
import Vec2 from '../math/Vec2';
import Body from './Body';
import CollisionDetection from './CollisionDetection';
import { ContactConstraint, JointConstraint } from './Constraint';
import Force from './Force';

export default class World {
    private G: number;
    private iterations: number;

    constructor(gravity: number, iterations = 10) {
        this.G = -gravity;
        this.iterations = iterations;
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
