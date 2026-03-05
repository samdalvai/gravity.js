import RigidBody from '../core/RigidBody';
import Vec2 from '../math/Vec2';
import { PolygonShape } from './PolygonShape';
import { ShapeType } from './Shape';

/**
 * Edge shapes are very thin and are meant to be used as static objects,
 * using them as dynamic objects makes them prone to collision tunneling
 */
export class EdgeShape extends PolygonShape {
    constructor(vertexA: Vec2, vertexB: Vec2) {
        const verts = [vertexA.copy(), vertexB.copy()];

        super(verts);
    }

    getType(): ShapeType {
        return ShapeType.EDGE;
    }

    getMomentOfInertia(): number {
        const a = this.localVertices[0];
        const b = this.localVertices[1];

        const dx = b.x - a.x;
        const dy = b.y - a.y;

        const lengthSq = dx * dx + dy * dy;

        return lengthSq * 0.083333;
    }
    updateVertices(angle: number, position: Vec2): void {
        super.updateVertices(angle, position);
    }
    updateAABB(body: RigidBody): void {
        super.updateAABB(body);
    }
}
