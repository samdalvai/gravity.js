/*
 * Portions of this file are derived from the Sopiro Physics Engine.
 *
 * Copyright (c) 2022 Sopiro
 * Licensed under the MIT License
 *
 * Original project:
 * https://github.com/Sopiro
 */
import Vec2 from '../math/Vec2';

export default class Edge {
    public p1: Vec2;
    public p2: Vec2;
    public dir: Vec2;

    public id1: number;
    public id2: number;

    constructor(p1: Vec2, p2: Vec2, id1: number = -1, id2: number = -1) {
        this.p1 = p1.copy();
        this.p2 = p2.copy();

        if (this.p1.equals(this.p2)) this.dir = new Vec2(0, 0);
        else this.dir = p2.subNew(p1).normalizeNew();

        this.id1 = id1;
        this.id2 = id2;
    }

    get length() {
        return this.p2.subNew(this.p1).magnitude();
    }

    clip(p: Vec2, dir: Vec2, remove: boolean = false): void {
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
    }
}

export function edgeIntersection(A: Vec2, B: Vec2, C: Vec2, D: Vec2): Vec2 | null {
    const r = B.subNew(A); // vector along first segment
    const s = D.subNew(C); // vector along second segment
    const rxs = r.x * s.y - r.y * s.x;
    if (rxs === 0) return null; // parallel or collinear

    const t = (C.subNew(A).x * s.y - C.subNew(A).y * s.x) / rxs;
    const u = (C.subNew(A).x * r.y - C.subNew(A).y * r.x) / rxs;

    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
        return A.addNew(r.scaleNew(t));
    }

    return null; // no intersection on the segments
}

export function edgeCircleIntersection(A: Vec2, B: Vec2, C: Vec2, r: number): Vec2[] {
    const d = B.subNew(A);
    const f = A.subNew(C);

    const a = d.dot(d);
    const b = 2 * f.dot(d);
    const c = f.dot(f) - r * r;

    let discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
        return []; // no intersection
    }

    discriminant = Math.sqrt(discriminant);

    const t1 = (-b - discriminant) / (2 * a);
    const t2 = (-b + discriminant) / (2 * a);

    const intersections: Vec2[] = [];

    if (0 <= t1 && t1 <= 1) {
        intersections.push(A.addNew(d.scaleNew(t1)));
    }

    if (0 <= t2 && t2 <= 1 && t2 != t1) {
        intersections.push(A.addNew(d.scaleNew(t2)));
    }

    return intersections;
}
