import { ContactManifold, ContactPoint } from '../collision/ContactManifold';
import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';

export class ContactManifoldPool {
    private readonly manifolds: ContactManifold[] = [];

    constructor(initialCapacity = 0) {
        for (let i = 0; i < initialCapacity; i++) {
            this.manifolds.push(new ContactManifold());
        }
    }

    acquire(
        bodyA: RigidBody,
        bodyB: RigidBody,
        contactPoints: ContactPoint[],
        penetrationDepth: number,
        contactNormalX: number,
        contactNormalY: number,
        featureFlipped: boolean,
    ): ContactManifold {
        const manifold = this.manifolds.pop();

        if (manifold != null) {
            manifold.init(
                bodyA,
                bodyB,
                contactPoints,
                penetrationDepth,
                contactNormalX,
                contactNormalY,
                featureFlipped,
            );
            return manifold;
        }

        return new ContactManifold(
            bodyA,
            bodyB,
            contactPoints,
            penetrationDepth,
            contactNormalX,
            contactNormalY,
            featureFlipped,
        );
    }

    release(manifold: ContactManifold): void {
        this.manifolds.push(manifold);
    }
}
