import RigidBody from '../core/RigidBody';
import Vec2 from '../math/Vec2';
import { BoxShape } from './BoxShape';
import { ShapeType } from './Shape';

export class CapsuleShape extends BoxShape {
    halfHeight: number;
    radius: number;

    constructor(halfHeight: number, radius: number) {
        super(radius * 2, halfHeight * 2);
        this.halfHeight = halfHeight;
        this.radius = radius;
    }

    getType(): ShapeType {
        return ShapeType.CAPSULE;
    }

    getMomentOfInertia(): number {
        // For solid capsules, the moment of inertia is the sum of the two half circles and box body inertia, accounting fot heir position
        // Still needs to be multiplied by the rigidbody's mass
        const r = this.radius;
        const l = this.halfHeight * 2;

        const areaRect = 2 * r * l;
        const areaCircle = Math.PI * r * r;
        const areaTotal = areaRect + areaCircle;

        if (areaTotal === 0) return 0;

        const mRect = areaRect / areaTotal;
        const mCircle = areaCircle / areaTotal;

        const iRect = (1 / 12) * mRect * (l * l + 4 * r * r);
        const iCircle = 0.5 * mCircle * r * r + (mCircle * (l * l)) / 4;

        return iRect + iCircle;
    }

    getTopCirclePosition(body: RigidBody): Vec2 {
        return new Vec2(0, this.halfHeight).rotate(body.rotation).addNew(body.position);
    }

    getBottomCirclePosition(body: RigidBody): Vec2 {
        return new Vec2(0, -this.halfHeight).rotate(body.rotation).addNew(body.position);
    }

    updateAABB(body: RigidBody): void {
        const radius = this.radius;

        const topCirclePos = this.getTopCirclePosition(body);
        const bottomCirclePos = this.getBottomCirclePosition(body);

        const topCircleMinX = topCirclePos.x - radius;
        const topCircleMinY = topCirclePos.y - radius;
        const topCircleMaxX = topCirclePos.x + radius;
        const topCircleMaxY = topCirclePos.y + radius;

        const bottomCircleMinX = bottomCirclePos.x - radius;
        const bottomCircleMinY = bottomCirclePos.y - radius;
        const bottomCircleMaxX = bottomCirclePos.x + radius;
        const bottomCircleMaxY = bottomCirclePos.y + radius;

        body.minX = Math.min(topCircleMinX, bottomCircleMinX);
        body.minY = Math.min(topCircleMinY, bottomCircleMinY);
        body.maxX = Math.max(topCircleMaxX, bottomCircleMaxX);
        body.maxY = Math.max(topCircleMaxY, bottomCircleMaxY);
    }
}
