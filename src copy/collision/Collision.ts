/*
 * Portions of this file are derived from Box2D and Phaser Box2D.
 *
 * Copyright (c) 2023 Erin Catto
 * Copyright (c) 2024 Phaser Studio Inc
 * Licensed under the MIT License
 */
import { RigidBody } from '../core/RigidBody';
import { ContactManifold } from './ContactManifold';
import * as NarrowPhase from './NarrowPhase';

export function detectCollision(a: RigidBody, b: RigidBody): ContactManifold | null {
    const manifold = NarrowPhase.collideBodies(a, b);

    if (manifold === null) {
        return null;
    }

    return new ContactManifold(a, b, manifold.normal, manifold.points);
}
