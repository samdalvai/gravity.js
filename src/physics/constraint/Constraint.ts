import MatMN from '../../math/MatMN';
import Vec2 from '../../math/Vec2';
import VecN from '../../math/VecN';
import Body from '../body/Body';

export default abstract class Constraint {
    a: Body;
    b: Body;

    aPoint: Vec2; // The constraint point in A's local space
    bPoint: Vec2; // The constraint point in B's local space

    constructor(a: Body, b: Body, aPointWorld: Vec2, bPointWorld: Vec2) {
        this.a = a;
        this.b = b;

        this.aPoint = a.worldSpaceToLocalSpace(aPointWorld);
        this.bPoint = b.worldSpaceToLocalSpace(bPointWorld);
    }

    ///////////////////////////////////////////////////////////////////////////////
    // Mat6x6 with the all inverse mass and inverse I of bodies "a" and "b"
    ///////////////////////////////////////////////////////////////////////////////
    //  [ 1/ma  0     0     0     0     0    ]
    //  [ 0     1/ma  0     0     0     0    ]
    //  [ 0     0     1/Ia  0     0     0    ]
    //  [ 0     0     0     1/mb  0     0    ]
    //  [ 0     0     0     0     1/mb  0    ]
    //  [ 0     0     0     0     0     1/Ib ]
    ///////////////////////////////////////////////////////////////////////////////
    getInvM = (): MatMN => {
        const invM = new MatMN(6, 6);
        invM.zero();

        invM.rows[0].set(0, this.a.invMass);
        invM.rows[1].set(1, this.a.invMass);
        invM.rows[2].set(2, this.a.invI);

        invM.rows[3].set(3, this.b.invMass);
        invM.rows[4].set(4, this.b.invMass);
        invM.rows[5].set(5, this.b.invI);

        return invM;
    };

    ///////////////////////////////////////////////////////////////////////////////
    // VecN with the all linear and angular velocities of bodies "a" and "b"
    ///////////////////////////////////////////////////////////////////////////////
    //  [ va.x ]
    //  [ va.y ]
    //  [ ωa   ]
    //  [ vb.x ]
    //  [ vb.y ]
    //  [ ωb   ]
    ///////////////////////////////////////////////////////////////////////////////
    getVelocities = (): VecN => {
        const V = new VecN(6);
        V.zero();

        V.set(0, this.a.velocity.x);
        V.set(1, this.a.velocity.y);
        V.set(2, this.a.angularVelocity);
        V.set(3, this.b.velocity.x);
        V.set(4, this.b.velocity.y);
        V.set(5, this.b.angularVelocity);

        return V;
    };

    abstract preSolve(dt: number): void;
    abstract solve(): void;
    abstract postSolve(): void;
}
