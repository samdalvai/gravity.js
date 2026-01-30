import Vec2 from '../math/Vec2';

export enum ShapeType {
    CIRCLE,
    POLYGON,
    CAPSULE,
}

interface SupportResult {
    vertex: Vec2;
    index: number;
}

export abstract class Shape {
    abstract getType(): ShapeType;
    abstract getMomentOfInertia(): number;
    abstract updateVertices(angle: number, position: Vec2): void;
    abstract support(dir: Vec2): SupportResult;
}

export class CircleShape extends Shape {
    radius: number = 0;

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

    // eslint-disable-next-line @typescript-eslint/no-unused-vars
    updateVertices = (angle: number, position: Vec2): void => {
        return; // Circles don't have vertices... nothing to do here
    };

    support(dir: Vec2): SupportResult {
        return { vertex: dir.normalizeNew().scaleNew(this.radius), index: -1 };
    }
}

export class PolygonShape extends Shape {
    localVertices: Vec2[] = [];
    worldVertices: Vec2[] = [];
    width: number;
    height: number;

    constructor(vertices: Vec2[]) {
        super();
        let minX = Infinity;
        let minY = Infinity;
        let maxX = -Infinity;
        let maxY = -Infinity;

        // Initialize the vertices of the polygon shape and set width and height
        for (const v of vertices) {
            this.localVertices.push(v);
            // Need to clone this vector, otherwise both arrays are aliasing the same vector
            this.worldVertices.push(v.copy());

            // Find min and max X and Y to calculate polygon width and height
            minX = Math.min(minX, v.x);
            minY = Math.min(minY, v.y);
            maxX = Math.max(maxX, v.x);
            maxY = Math.max(maxY, v.y);
        }

        this.width = maxX - minX;
        this.height = maxY - minY;
    }

    getType = (): ShapeType => {
        return ShapeType.POLYGON;
    };

    getMomentOfInertia = (): number => {
        let acc0 = 0;
        let acc1 = 0;

        for (let i = 0; i < this.localVertices.length; i++) {
            const a = this.localVertices[i];
            const b = this.localVertices[(i + 1) % this.localVertices.length];

            const cross = Math.abs(a.cross(b));
            acc0 += cross * (a.dot(a) + b.dot(b) + a.dot(b));
            acc1 += cross;
        }

        return acc0 / 6 / acc1;
    };

    /**
     * @param angle The angle in radians, use degrees * Math.PI / 180 to convert to radians
     * @param position The updated position of the shape
     */
    updateVertices = (angle: number, position: Vec2): void => {
        // Loop all the vertices, transforming from local to world space
        const cos = Math.cos(angle);
        const sin = Math.sin(angle);
        // console.log(this.localVertices);
        for (let i = 0; i < this.localVertices.length; i++) {
            // First rotate, then we translate
            this.worldVertices[i].x = this.localVertices[i].x * cos - this.localVertices[i].y * sin;
            this.worldVertices[i].y = this.localVertices[i].x * sin + this.localVertices[i].y * cos;
            this.worldVertices[i].x += position.x;
            this.worldVertices[i].y += position.y;
        }
    };

    edgeAt = (index: number): Vec2 => {
        const currVertex = index;
        const nextVertex = (index + 1) % this.worldVertices.length;
        return this.worldVertices[nextVertex].subNew(this.worldVertices[currVertex]);
    };

    findMinSeparation = (other: PolygonShape): [number, number] => {
        let separation = -Infinity;
        let indexReferenceEdge = 0;

        // Loop all the vertices of "this" polygon
        for (let i = 0; i < this.worldVertices.length; i++) {
            const va = this.worldVertices[i];
            const normal = this.edgeAt(i).normal();

            // Loop all the vertices of the "other" polygon
            let minSep = Infinity;

            for (let j = 0; j < other.worldVertices.length; j++) {
                const vb = other.worldVertices[j];
                const proj = vb.subNew(va).dot(normal);
                if (proj < minSep) {
                    minSep = proj;
                }
            }

            if (minSep > separation) {
                separation = minSep;
                indexReferenceEdge = i;
            }
        }
        return [separation, indexReferenceEdge];
    };

    support(dir: Vec2): SupportResult {
        let idx = 0;
        let maxValue = dir.dot(this.localVertices[idx]);

        for (let i = 1; i < this.localVertices.length; i++) {
            const value = dir.dot(this.localVertices[i]);
            if (value > maxValue) {
                idx = i;
                maxValue = value;
            }
        }

        return { vertex: this.localVertices[idx], index: idx };
    }
}

export class BoxShape extends PolygonShape {
    constructor(width: number, height: number) {
        const verts = [
            new Vec2(-width / 2, -height / 2),
            new Vec2(+width / 2, -height / 2),
            new Vec2(+width / 2, +height / 2),
            new Vec2(-width / 2, +height / 2),
        ];

        super(verts);

        this.width = width;
        this.height = height;
    }

    getType = (): ShapeType => {
        return ShapeType.POLYGON;
    };

    getMomentOfInertia = (): number => {
        // For a rectangle, the moment of inertia is 1/12 * (w^2 + h^2)
        // But this still needs to be multiplied by the rigidbody's mass
        return 0.083333 * (this.width * this.width + this.height * this.height);
    };
}

export class CapsuleShape extends BoxShape {
    halfHeight: number;
    radius: number;

    constructor(halfHeight: number, radius: number) {
        super(radius * 2, halfHeight * 2);
        // super();
        this.halfHeight = halfHeight;
        this.radius = radius;
    }

    getType = (): ShapeType => {
        return ShapeType.CAPSULE;
    };

    getMomentOfInertia = (): number => {
        // TODO: to be updated
        return 0.5 * (this.radius * this.radius);
    };

    support(dir: Vec2): SupportResult {
        throw new Error('Method not implemented.');
    }
}
