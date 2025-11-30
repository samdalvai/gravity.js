import Body from './Body';
import Vec2 from './Vec2';

export default class Contact {
    a: Body;
    b: Body;

    start: Vec2;
    end: Vec2;

    normal: Vec2;
    depth: number;

    constructor(a: Body, b: Body, start: Vec2, end: Vec2, normal: Vec2, depth: number) {
        this.a = a;
        this.b = b;
        this.start = start;
        this.end = end;
        this.normal = normal;
        this.depth = depth;
    }
}
