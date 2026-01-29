/*
 * Portions of this file are derived from the Sopiro Physics Engine.
 *
 * Copyright (c) 2022 Sopiro
 * Licensed under the MIT License
 *
 * Original project:
 * https://github.com/Sopiro
 */
import Utils from '../math/Utils';
import { DELTA_TIME } from './Constants';
import { Constraint } from './Constraint';
import RigidBody from './RigidBody';

export abstract class Joint extends Constraint {
    public drawAnchor = false;
    public drawConnectionLine = false;

    private _frequency!: number;
    private _dampingRatio!: number;
    private _jointMass!: number;

    // 0 < Frequency
    // 0 <= Damping ratio <= 1
    // 0 < Joint mass
    constructor(bodyA: RigidBody, bodyB: RigidBody, frequency = 15, dampingRatio = 1.0, jointMass = -1) {
        super(bodyA, bodyB);

        this.setFDM(frequency, dampingRatio, jointMass);
    }

    private setFDM(
        frequency: number = this._frequency,
        dampingRatio: number = this._dampingRatio,
        jointMass: number = this._jointMass,
    ): void {
        if (frequency > 0) {
            this._frequency = frequency;
            this._dampingRatio = Utils.clamp(dampingRatio, 0.0, 1.0);

            Utils.assert(this.bodyA.mass > 0 || this.bodyB.mass > 0);
            this._jointMass = jointMass <= 0 ? (this.bodyA.mass > 0 ? this.bodyA.mass : this.bodyB.mass) : jointMass;

            this.calculateBetaAndGamma();
        } else {
            // If the frequency is less than or equal to zero, make this joint solid
            this._frequency = -1;
            this._dampingRatio = 1.0;
            this._jointMass = -1;

            this.beta = 1.0;
            this.gamma = 0.0;
        }
    }

    private calculateBetaAndGamma() {
        const omega = 2 * Math.PI * this._frequency;
        const d = 2 * this._jointMass * this._dampingRatio * omega; // Damping coefficient
        const k = this._jointMass * omega * omega; // Spring constant
        const h = DELTA_TIME;

        this.beta = (h * k) / (d + h * k);
        this.gamma = 1.0 / ((d + h * k) * h);
    }

    get frequency(): number {
        return this._frequency;
    }

    set frequency(frequency: number) {
        this.setFDM(frequency, undefined, undefined);
    }

    get dampingRatio(): number {
        return this._frequency;
    }

    set dampingRatio(dampingRatio: number) {
        this.setFDM(undefined, dampingRatio, undefined);
    }

    get jointMass(): number {
        return this._frequency;
    }

    set jointMass(jointMass: number) {
        this.setFDM(undefined, undefined, jointMass);
    }

    get isSolid(): boolean {
        return this._frequency <= 0;
    }
}
