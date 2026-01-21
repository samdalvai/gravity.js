import Body from './Body';

// Children: ContactManifold, Joint
export abstract class Constraint {
    public readonly bodyA: Body;
    public readonly bodyB: Body;
    
    constructor(bodyA: Body, bodyB: Body) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
    }

    /*
     * C: Constraint equation
     * C = J·v = 0
     * J is depend on constraint
     *
     * Calculate Jacobian J and effective mass M
     * M = K^-1 = (J · M^-1 · J^t)^-1
     */
    public abstract preSolve(inverseDeltaTime: number): void;

    /*
     * Solve velocity constraint, calculate corrective impulse for current iteration
     * Pc: Corrective impulse
     * λ: lagrangian multiplier
     *
     * Pc = J^t · λ (∵ Pc ∥ J^t)
     * λ = (J · M^-1 · J^t)^-1 ⋅ -(Jv + (β/h)·C(x)) where C(x): positional error
     *
     * with soft constraint,
     * λ = (J · M^-1 · J^t + λ·I)^-1 ⋅ -( Jv + (β/h)·C(x) + (γ/h)·λ' ) where I = identity matrix and λ' = accumulated impulse
     *
     * More reading:
     * https://pybullet.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
     */
    public abstract solve(): void;
}
