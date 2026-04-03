import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { ContactManifold } from './Contact';

export interface ContactPoint {
    point: Vec2;
    id: number;
}

export function detectCollision(a: RigidBody, b: RigidBody): ContactManifold | null {
    // TO BE IMPLEMENTED
    return null;
}
