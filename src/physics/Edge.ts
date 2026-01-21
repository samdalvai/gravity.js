import Vec2 from '../math/Vec2';

export default class Edge {
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

    clipEdge = (p: Vec2, dir: Vec2, remove: boolean = false): void => {
        const d1 = this.p1.subNew(p).dot(dir);
        const d2 = this.p2.subNew(p).dot(dir);

        if (d1 >= 0 && d2 >= 0) return;

        const per = Math.abs(d1) + Math.abs(d2);

        if (d1 < 0) {
            if (remove) {
                this.p1 = this.p2;
                this.id1 = this.id2;
            } else {
                this.p1 = this.p1.addNew(this.p2.subNew(this.p1).scaleNew(-d1 / per));
            }
        } else if (d2 < 0) {
            if (remove) {
                this.p2 = this.p1;
                this.id2 = this.id1;
            } else {
                this.p2 = this.p2.addNew(this.p1.subNew(this.p2).scaleNew(-d2 / per));
            }
        }
    };
}
