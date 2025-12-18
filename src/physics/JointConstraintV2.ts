import Body from './Body';
import Vec2 from './Vec2';

export default class JointConstraintV2 {
    a: Body;
    b: Body;
    position: Vec2;

    constructor(a: Body, b: Body, position: Vec2) {
        this.a = a;
        this.b = b;
        this.position = position;
    }

    preSolve = (dt: number): void => {
        //
    };

    solve = (): void => {
        //
    };
}
