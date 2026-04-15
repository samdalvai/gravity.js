import { ContactManifold, ContactPoint } from '../collision/ContactManifold';
import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';

export class ContactManifoldPool {
    private readonly manifolds: ContactManifold[] = [];

    acquire(
        bodyA: RigidBody,
        bodyB: RigidBody,
        contactPoints: ContactPoint[],
        penetrationDepth: number,
        contactNormal: Vec2,
        featureFlipped: boolean,
    ): ContactManifold {
        const manifold = this.manifolds.pop();

        if (manifold != null) {
            manifold.init(bodyA, bodyB, contactPoints, penetrationDepth, contactNormal, featureFlipped);
            return manifold;
        }

        return new ContactManifold(bodyA, bodyB, contactPoints, penetrationDepth, contactNormal, featureFlipped);
    }

    release(manifold: ContactManifold): void {
        this.manifolds.push(manifold);
    }
}
