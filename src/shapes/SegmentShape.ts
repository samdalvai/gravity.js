import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { PolygonShape } from './PolygonShape';
import { ShapeType } from './Shape';

/**
 * Segment shapes are very thin and are meant to be used as static objects,
 * using them as dynamic objects makes them prone to collision tunneling
 */
export class SegmentShape extends PolygonShape {
    constructor(length: number, horizontal: boolean);
    constructor(vertexA: Vec2, vertexB: Vec2);

    constructor(arg0: number | Vec2, arg1: boolean | Vec2) {
        if (typeof arg0 === 'number' && typeof arg1 === 'boolean') {
            if (arg1) {
                const verts = [new Vec2(-arg0 / 2, 0), new Vec2(arg0 / 2, 0)];
                super(verts);
            } else {
                const verts = [new Vec2(0, -arg0 / 2), new Vec2(0, arg0 / 2)];
                super(verts);
            }
        } else if (arg0 instanceof Vec2 && arg1 instanceof Vec2) {
            const verts = [arg0.copy(), arg1.copy()];
            super(verts);
        } else {
            throw new Error('Invalid constructor arguments');
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
