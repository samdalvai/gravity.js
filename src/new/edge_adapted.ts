// import { Vector2 } from './vector2';
// import * as Util from './util';
import Vec2 from '../math/Vec2';

export class Edge {
    public p1: Vec2;
    public p2: Vec2;
    public dir: Vec2;

    public id1: number;
    public id2: number;

    constructor(p1: Vec2, p2: Vec2, id1: number = -1, id2: number = -1) {
        this.p1 = p1.clone();
        this.p2 = p2.clone();

        if (this.p1.equals(this.p2)) this.dir = new Vec2(0, 0);
        else this.dir = p2.subNew(p1).normalizeNew();

        this.id1 = id1;
        this.id2 = id2;
    }

    get length() {
        return this.p2.subNew(this.p1).magnitude();
    }

    get normal() {
        return Vec2.cross(1, this.dir);
    }
}
