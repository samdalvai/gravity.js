import Body from './Body';
import MatMN from './MatMN';
import Vec2 from './Vec2';
import VecN from './VecN';

export abstract class Constraint {
    a: Body;
    b: Body;

    aPoint: Vec2;
    bPoint: Vec2;

    constructor(a: Body, b: Body, anchorPoint: Vec2) {
        this.a = a;
        this.b = b;

        this.aPoint = a.worldSpaceToLocalSpace(anchorPoint);
        this.bPoint = b.worldSpaceToLocalSpace(anchorPoint);
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

export class JointConstraint extends Constraint {
    private jacobian: MatMN;
    private cachedLambda: VecN;
    private bias: number;

    constructor(a: Body, b: Body, anchorPoint: Vec2) {
        super(a, b, anchorPoint);

        this.jacobian = new MatMN(1, 6);
        this.cachedLambda = new VecN(1);
        this.bias = 0;

        this.cachedLambda.zero();
    }

    preSolve(dt: number): void {
        throw new Error('Method not implemented.');
    }
    solve(): void {
        throw new Error('Method not implemented.');
    }
    postSolve(): void {
        throw new Error('Method not implemented.');
    }
}

// export class PenetrationConstraint extends Constraint {
//     //jacobian: MatMN;

//     preSolve(dt: number): void {
//         throw new Error('Method not implemented.');
//     }
//     solve(): void {
//         throw new Error('Method not implemented.');
//     }
//     postSolve(): void {
//         throw new Error('Method not implemented.');
//     }
// }
