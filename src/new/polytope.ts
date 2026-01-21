import Vec2 from '../math/Vec2';
import { Simplex } from './simplex';

export interface ClosestEdgeInfo {
    index: number;
    distance: number;
    normal: Vec2;
}

export class Polytope {
    public readonly vertices: Vec2[];

    constructor(simplex: Simplex) {
        if (simplex.count != 3) throw "Input simplex isn't a triangle";

        this.vertices = [simplex.vertices[0].clone(), simplex.vertices[1].clone(), simplex.vertices[2].clone()];
    }

    public get count(): number {
        return this.vertices.length;
    }

    // Returns the edge closest to the origin
    getClosestEdge(): ClosestEdgeInfo {
        let minIndex = 0;
        let minDistance = Infinity;
        let minNormal = new Vec2();

        for (let i = 0; i < this.count; i++) {
            const j = (i + 1) % this.count;

            const vertexI = this.vertices[i];
            const vertexJ = this.vertices[j];

            const edge = vertexJ.subNew(vertexI);

            const normal = new Vec2(-edge.y, edge.x).normalizeNew();
            let distance = normal.dot(vertexI);

            if (distance < 0) {
                distance *= -1;
                normal.negated();
            }

            if (distance < minDistance) {
                minDistance = distance;
                minNormal = normal;
                minIndex = i;
            }
        }

        return { index: minIndex, distance: minDistance, normal: minNormal };
    }
}
