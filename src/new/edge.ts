import { Vector2 } from './vector2';
import * as Util from './util';

export class Edge {
    public p1: Vector2;
    public p2: Vector2;
    public dir: Vector2;

    public id1: number;
    public id2: number;

    constructor(p1: Vector2, p2: Vector2, id1: number = -1, id2: number = -1) {
        this.p1 = p1.copy();
        this.p2 = p2.copy();

        if (this.p1.equals(this.p2)) this.dir = new Vector2(0, 0);
        else this.dir = p2.subNew(p1).normalized();

        this.id1 = id1;
        this.id2 = id2;
    }

    get length() {
        return this.p2.subNew(this.p1).length;
    }

    get normal() {
        return Util.cross(1, this.dir);
    }
}
