import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { BoxShape } from '../shapes/BoxShape';
import { CapsuleShape } from '../shapes/CapsuleShape';
import { CircleShape } from '../shapes/CircleShape';
import { PolygonShape } from '../shapes/PolygonShape';
import { SegmentShape } from '../shapes/SegmentShape';
import { Shape } from '../shapes/Shape';

export interface BodyOptions {
    x: number;
    y: number;
    mass: number;
    rotation?: number;
    velocity?: Vec2;
    angularVelocity?: number;
    restitution?: number;
    friction?: number;
    rollingResistance?: number;
    surfaceSpeed?: number;
    canRotate?: boolean;
    isBullet?: boolean;
}

export interface BoxBodyOptions extends BodyOptions {
    width: number;
    height: number;
}

export interface CircleBodyOptions extends BodyOptions {
    radius: number;
}

export interface CapsuleBodyOptions extends BodyOptions {
    halfHeight: number;
    radius: number;
}

export interface PolygonBodyOptions extends BodyOptions {
    vertices: readonly Vec2[];
}

export interface SegmentBodyOptions extends BodyOptions {
    length: number;
    horizontal: boolean;
}

export class BodiesFactory {
    static fromShape(shape: Shape, options: BodyOptions): RigidBody {
        const body = new RigidBody(shape, options.x, options.y, options.mass);
        return applyBodyOptions(body, options);
    }

    static box(options: BoxBodyOptions): RigidBody {
        const { width, height, ...bodyOptions } = options;
        return BodiesFactory.fromShape(new BoxShape(width, height), bodyOptions);
    }

    static circle(options: CircleBodyOptions): RigidBody {
        const { radius, ...bodyOptions } = options;
        return BodiesFactory.fromShape(new CircleShape(radius), bodyOptions);
    }

    static capsule(options: CapsuleBodyOptions): RigidBody {
        const { halfHeight, radius, ...bodyOptions } = options;
        return BodiesFactory.fromShape(new CapsuleShape(halfHeight, radius), bodyOptions);
    }

    static polygon(options: PolygonBodyOptions): RigidBody {
        const { vertices, ...bodyOptions } = options;
        return BodiesFactory.fromShape(new PolygonShape([...vertices]), bodyOptions);
    }

    static segment(options: SegmentBodyOptions): RigidBody {
        const { length, horizontal, ...bodyOptions } = options;
        return BodiesFactory.fromShape(new SegmentShape(length, horizontal), bodyOptions);
    }
}

function applyBodyOptions(body: RigidBody, options: BodyOptions): RigidBody {
    if (options.rotation !== undefined) {
        body.rotation = options.rotation;
    }

    if (options.velocity !== undefined) {
        body.velocity = options.velocity.copy();
    }

    if (options.angularVelocity !== undefined) {
        body.angularVelocity = options.angularVelocity;
    }

    if (options.canRotate !== undefined) {
        body.canRotate = options.canRotate;
    }

    if (options.restitution !== undefined) {
        body.restitution = options.restitution;
    }

    if (options.friction !== undefined) {
        body.friction = options.friction;
    }

    if (options.rollingResistance !== undefined) {
        body.rollingResistance = options.rollingResistance;
    }

    if (options.surfaceSpeed !== undefined) {
        body.surfaceSpeed = options.surfaceSpeed;
    }

    if (options.isBullet !== undefined) {
        body.isBullet = options.isBullet;
    }

    body.shape.updateVertices(body.rotation, body.position);
    body.shape.updateAABB(body);

    return body;
}
