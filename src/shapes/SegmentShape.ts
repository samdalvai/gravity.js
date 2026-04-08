import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { PolygonShape } from './PolygonShape';
import { ShapeType } from './Shape';

/**
 * Segment shapes are very thin and are meant to be used as static objects,
 * using them as dynamic objects makes them prone to collision tunneling
 */
export class SegmentShape extends PolygonShape {
    constructor(length: number, horizontal: boolean) {
        if (horizontal) {
            const verts = [new Vec2(-length / 2, 0), new Vec2(length / 2, 0)];
            super(verts);
        } else {
            const verts = [new Vec2(0, -length / 2), new Vec2(0, length / 2)];
            super(verts);
        }
    }

    getType(): ShapeType {
        return ShapeType.SEGMENT;
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
