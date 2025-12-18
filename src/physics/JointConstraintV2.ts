import Mat22 from '../math/Mat22';
import Body from './Body';
import Vec2 from './Vec2';

export default class JointConstraintV2 {
    body1: Body;
    body2: Body;

    localAnchor1: Vec2;
    localAnchor2: Vec2;

    constructor(body1: Body, body2: Body, anchor: Vec2) {
        this.body1 = body1;
        this.body2 = body2;

        const Rot1 = new Mat22(this.body1.rotation);
        const Rot2 = new Mat22(this.body2.rotation);
        const Rot1T = Rot1.transpose();
        const Rot2T = Rot2.transpose();

        this.localAnchor1 = Mat22.multiply(Rot1T, Vec2.sub(anchor, this.body1.position));
        this.localAnchor2 = Mat22.multiply(Rot2T, Vec2.sub(anchor, this.body2.position));
    }

    preSolve = (dt: number): void => {
        //
    };

    solve = (): void => {
        //
    };
}
