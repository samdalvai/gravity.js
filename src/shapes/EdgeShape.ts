import RigidBody from '../core/RigidBody';
import Vec2 from '../math/Vec2';
import { PolygonShape } from './PolygonShape';
import { ShapeType } from './Shape';

export class EdgeShape extends PolygonShape {
    constructor(vertexA: Vec2, vertexB: Vec2) {
        const verts = [vertexA.copy(), vertexB.copy()];

        super(verts);
    }

    getType(): ShapeType {
        return ShapeType.EDGE;
    }
    getMomentOfInertia(): number {
        // TODO: to be implemented
        return 0.5;
    }
    updateVertices(angle: number, position: Vec2): void {
        super.updateVertices(angle, position);
    }
    updateAABB(body: RigidBody): void {
        super.updateAABB(body);
    }
}
