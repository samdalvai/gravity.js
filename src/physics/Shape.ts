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

    constructor() {
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

    edgeAt = (index: number): Vec2 => {
        const currVertex = index;
        const nextVertex = (index + 1) % this.worldVertices.length;
        return this.worldVertices[nextVertex].subNew(this.worldVertices[currVertex]);
    };

    findMinSeparation = (other: PolygonShape, axis: Vec2, point: Vec2): number => {
        let separation = -Infinity;

        // Loop all the vertices of "this" polygon
        for (let i = 0; i < this.worldVertices.length; i++) {
            const va = this.worldVertices[i];
            const normal = this.edgeAt(i).normal();

            // Loop all the vertices of the "other" polygon
            let minSep = Infinity;
            let minVertex: Vec2 | null = null;

            for (let j = 0; j < other.worldVertices.length; j++) {
                const vb = other.worldVertices[j];
                const proj = vb.subNew(va).dot(normal);
                if (proj < minSep) {
                    minSep = proj;
                    minVertex = vb;
                }
            }

            if (minSep > separation) {
                separation = minSep;
                const edgeAtI = this.edgeAt(i);
                axis.x = edgeAtI.x;
                axis.y = edgeAtI.y;

                if (minVertex) {
                    point.x = minVertex.x;
                    point.y = minVertex.y;
                }
            }
        }
        return separation;
    };
}

export class BoxShape extends PolygonShape {
    width: number;
    height: number;

    constructor(width: number, height: number) {
        super();
        this.width = width;
        this.height = height;

        this.localVertices.push(new Vec2(-width / 2.0, -height / 2.0));
        this.localVertices.push(new Vec2(+width / 2.0, -height / 2.0));
        this.localVertices.push(new Vec2(+width / 2.0, +height / 2.0));
        this.localVertices.push(new Vec2(-width / 2.0, +height / 2.0));

        this.worldVertices.push(new Vec2(-width / 2.0, -height / 2.0));
        this.worldVertices.push(new Vec2(+width / 2.0, -height / 2.0));
        this.worldVertices.push(new Vec2(+width / 2.0, +height / 2.0));
        this.worldVertices.push(new Vec2(-width / 2.0, +height / 2.0));
    }

    getType = (): ShapeType => {
        return ShapeType.BOX;
    };

    getMomentOfInertia = (): number => {
        // For a rectangle, the moment of inertia is 1/12 * (w^2 + h^2)
        // But this still needs to be multiplied by the rigidbody's mass
        return 0.083333 * (this.width * this.width + this.height * this.height);
    };
}
