import Vec2 from '../math/Vec2';

export enum ShapeType {
    CIRCLE,
    POLYGON,
    BOX,
}

export abstract class Shape {
    radius: number = 0;

    abstract getType(): ShapeType;
    abstract getMomentOfInertia(): number;
    abstract updateVertices(angle: number, position: Vec2): void;
}

export class CircleShape extends Shape {
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
}

export class PolygonShape extends Shape {
    localVertices: Vec2[] = [];
    worldVertices: Vec2[] = [];
    width: number;
    height: number;

    constructor(vertices: Vec2[]) {
        super();
        let minX = Number.POSITIVE_INFINITY;
        let minY = Number.POSITIVE_INFINITY;
        let maxX = Number.NEGATIVE_INFINITY;
        let maxY = Number.NEGATIVE_INFINITY;

        let maxRadiusSq = 0;

        // Initialize the vertices of the polygon shape and set width and height
        for (const v of vertices) {
            this.localVertices.push(v);
            this.worldVertices.push(v);

            // Find min and max X and Y to calculate polygon width and height
            minX = Math.min(minX, v.x);
            minY = Math.min(minY, v.y);
            maxX = Math.max(maxX, v.x);
            maxY = Math.max(maxY, v.y);

            const r2 = v.x * v.x + v.y * v.y;
            if (r2 > maxRadiusSq) {
                maxRadiusSq = r2;
            }
        }

        this.radius = Math.sqrt(maxRadiusSq);

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

    updateVertices = (angle: number, position: Vec2): void => {
        // Loop all the vertices, transforming from local to world space
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

    findMinSeparation = (other: PolygonShape, indexReferenceEdge: { value: number }, supportPoint: Vec2): number => {
        let separation = -Infinity;

        // Loop all the vertices of "this" polygon
        for (let i = 0; i < this.worldVertices.length; i++) {
            const va = this.worldVertices[i];
            const normal = this.edgeAt(i).normal();

            // Loop all the vertices of the "other" polygon
            let minSep = Infinity;
            let minVertex = new Vec2();

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
                indexReferenceEdge.value = i;
                supportPoint.x = minVertex.x;
                supportPoint.y = minVertex.y;
            }
        }
        return separation;
    };

    findIncidentEdge = (normal: Vec2): number => {
        let index = 0;
        let minProj = Number.POSITIVE_INFINITY;

        for (let i = 0; i < this.worldVertices.length; i++) {
            const edgeNormal = this.edgeAt(i).normal();
            const proj = edgeNormal.dot(normal);

            if (proj < minProj) {
                minProj = proj;
                index = i;
            }
        }

        return index;
    };

    clipSegmentToLine = (contactsIn: Vec2[], contactsOut: Vec2[], c0: Vec2, c1: Vec2): number => {
        // Start with no output points
        let numOut = 0;

        // Calculate the distance of end points to the line
        const normal = c1.subNew(c0).normalize();
        const dist0 = contactsIn[0].subNew(c0).cross(normal);
        const dist1 = contactsIn[1].subNew(c0).cross(normal);

        // If the points are behind the plane
        if (dist0 <= 0) contactsOut[numOut++] = contactsIn[0];
        if (dist1 <= 0) contactsOut[numOut++] = contactsIn[1];

        // If the points are on different sides of the plane (one distance is negative and the other is positive)
        if (dist0 * dist1 < 0) {
            const totalDist = dist0 - dist1;

            // Fint the intersection using linear interpolation: lerp(start,end) => start + t*(end-start)
            const t = dist0 / totalDist;
            const contact = contactsIn[0].addNew(contactsIn[1].subNew(contactsIn[0]).scaleNew(t));
            contactsOut[numOut] = contact;
            numOut++;
        }

        return numOut;
    };
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
        return ShapeType.BOX;
    };

    getMomentOfInertia = (): number => {
        // For a rectangle, the moment of inertia is 1/12 * (w^2 + h^2)
        // But this still needs to be multiplied by the rigidbody's mass
        return 0.083333 * (this.width * this.width + this.height * this.height);
    };
}
