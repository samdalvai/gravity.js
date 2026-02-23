import Vec2 from '../math/Vec2';
import Edge from './Edge';
import RigidBody from '../core/RigidBody';

export enum ShapeType {
    CIRCLE,
    POLYGON,
    BOX,
    CAPSULE,
}


export abstract class Shape {
    abstract getType(): ShapeType;
    abstract getMomentOfInertia(): number;
    abstract updateVertices(angle: number, position: Vec2): void;
    abstract updateAABB(body: RigidBody): void;
}


