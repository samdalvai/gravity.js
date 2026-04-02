/*
 * Portions of this file are derived from Box2D and Phaser Box2D.
 *
 * Copyright (c) 2023 Erin Catto
 * Copyright (c) 2024 Phaser Studio Inc
 * Licensed under the MIT License
 */
import { Constraint } from '../constraint/Constraint';
import { Softness, makeSoftness } from '../constraint/Softness';
import { RigidBody } from '../core/RigidBody';
import * as Utils from '../utils/Utils';

export abstract class Joint extends Constraint {
    public drawAnchor = false;
    public drawConnectionLine = false;

    protected readonly hertz: number;
    protected readonly dampingRatio: number;
    // Kept for API compatibility. The Box2D-style solver derives the effective
    // mass from the connected bodies instead of using this as a tuning input.
    protected readonly jointMass: number;

    protected softness: Softness = {
        biasRate: 0,
        massScale: 1,
        impulseScale: 0,
    };

    constructor(bodyA: RigidBody, bodyB: RigidBody, frequency = 15, dampingRatio = 1.0, jointMass = -1) {
        super(bodyA, bodyB);

        this.hertz = Math.max(0, frequency);
        this.dampingRatio = Utils.clamp(dampingRatio, 0.0, 1.0);

        Utils.assert(this.bodyA.mass > 0 || this.bodyB.mass > 0);
        this.jointMass = jointMass <= 0 ? (this.bodyA.mass > 0 ? this.bodyA.mass : this.bodyB.mass) : jointMass;
    }

    protected updateSoftness(inverseDeltaTime: number): void {
        const invDt = Math.abs(inverseDeltaTime);
        const dt = invDt > 0 ? 1 / invDt : 0;
        this.softness = makeSoftness(this.hertz, this.dampingRatio, dt);
    }

    protected isSpringEnabled(): boolean {
        return this.hertz > 0.0;
    }
}
