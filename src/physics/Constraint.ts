/*
 * Portions of this file are derived from the Sopiro Physics Engine.
 *
 * Copyright (c) 2022 Sopiro
 * Licensed under the MIT License
 *
 * Original project:
 * https://github.com/Sopiro
 */
import RigidBody from './RigidBody';

export abstract class Constraint {
    public readonly bodyA: RigidBody;
    public readonly bodyB: RigidBody;

    protected beta = 0.0; // Coefficient of position correction (Positional error feedback factor)
    protected gamma = 0.0; // Coefficient of Softness (Force feedback factor)

    constructor(bodyA: RigidBody, bodyB: RigidBody) {
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
