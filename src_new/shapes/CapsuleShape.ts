import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { Shape, ShapeType } from './Shape';

export class CapsuleShape extends Shape {
    halfHeight: number;
    radius: number;
    width: number;
    height: number;

    center1: Vec2;
    center2: Vec2;
    worldCenter1: Vec2;
    worldCenter2: Vec2;

    constructor(halfHeight: number, radius: number) {
        super();

        this.halfHeight = halfHeight;
        this.radius = radius;
        this.width = radius * 2;
        this.height = halfHeight * 2;

        this.center1 = new Vec2(0, halfHeight);
        this.center2 = new Vec2(0, -halfHeight);
        this.worldCenter1 = this.center1.copy();
        this.worldCenter2 = this.center2.copy();
    }

    getType(): ShapeType {
        return ShapeType.CAPSULE;
    }

    getMomentOfInertia(): number {
        // For solid capsules, the moment of inertia is the sum of the two half circles
        // and the rectangular center section, weighted by their normalized area.
        const r = this.radius;
        const l = this.halfHeight * 2;

        const areaRect = 2 * r * l;
        const areaCircle = Math.PI * r * r;
        const areaTotal = areaRect + areaCircle;

        if (areaTotal === 0) {
            return 0;
        }

        const mRect = areaRect / areaTotal;
        const mCircle = areaCircle / areaTotal;

        const iRect = (1 / 12) * mRect * (l * l + 4 * r * r);
        const iCircle = 0.5 * mCircle * r * r + (mCircle * l * l) / 4;

        return iRect + iCircle;
    }

    updateVertices(angle: number, position: Vec2): void {
        this.worldCenter1 = this.center1.rotate(angle).addNew(position);
        this.worldCenter2 = this.center2.rotate(angle).addNew(position);
    }

    updateAABB(body: RigidBody): void {
        body.minX = Math.min(this.worldCenter1.x, this.worldCenter2.x) - this.radius;
        body.minY = Math.min(this.worldCenter1.y, this.worldCenter2.y) - this.radius;
        body.maxX = Math.max(this.worldCenter1.x, this.worldCenter2.x) + this.radius;
        body.maxY = Math.max(this.worldCenter1.y, this.worldCenter2.y) + this.radius;
    }

    getTopCirclePosition(): Vec2 {
        return this.worldCenter1;
    }

    getBottomCirclePosition(): Vec2 {
        return this.worldCenter2;
    }
}
