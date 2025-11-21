import Vec2 from './Vec2';

export enum ShapeType {
    CIRCLE,
    POLYGON,
    BOX,
}

export abstract class Shape {
    abstract getType(): ShapeType;
    abstract getMomentOfInertia(): number;
}

export class CircleShape extends Shape {
    radius: number;

    constructor(radius: number) {
        super();
        this.radius = radius;
    }

    getType = (): ShapeType => {
        return ShapeType.CIRCLE;
    };

    getMomentOfInertia = (): number => {
        // For solid circles, the moment of inertia is 1/2 * r^2
        // But this still needs to be multiplied by the rigidbody's mass
        return 0.5 * (this.radius * this.radius);
    };
}

export class PolygonShape extends Shape {
    localVertices: Vec2[] = [];
    worldVertices: Vec2[] = [];

    constructor(vertices: Vec2[]) {
        super();
        // TODO: ....
    }

    getType = (): ShapeType => {
        return ShapeType.POLYGON;
    };

    getMomentOfInertia = (): number => {
        // TODO:
        return 0.0;
    };

    updateVertices = (angle: number, position: Vec2): void => {
        for (let i = 0; i < this.localVertices.length; i++) {
            // First rotate, then we translate
            this.worldVertices[i] = this.localVertices[i].rotate(angle);
            this.worldVertices[i].addAssign(position);
        }
    };
}
