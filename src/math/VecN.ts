export default class VecN {
    N: number;
    data: number[];

    constructor(N: number = 0) {
        this.N = N;
        this.data = new Array(N).fill(0);
    }

    // Copy constructor
    static from(v: VecN): VecN {
        const result = new VecN(v.N);
        for (let i = 0; i < v.N; i++) {
            result.data[i] = v.data[i];
        }
        return result;
    }

    zero(): void {
        for (let i = 0; i < this.N; i++) {
            this.data[i] = 0.0;
        }
    }

    /** v1.Dot(v2) */
    dot(v: VecN): number {
        let sum = 0.0;
        for (let i = 0; i < this.N; i++) {
            sum += this.data[i] * v.data[i];
        }
        return sum;
    }

    /** operator = */
    assign(v: VecN): this {
        this.N = v.N;
        this.data = [];
        for (let i = 0; i < v.N; i++) {
            this.data[i] = v.data[i];
        }
        return this;
    }

    /** operator + */
    addNew(v: VecN): VecN {
        const result = VecN.from(this);
        for (let i = 0; i < this.N; i++) {
            result.data[i] += v.data[i];
        }
        return result;
    }

    /** operator - */
    subNew(v: VecN): VecN {
        const result = VecN.from(this);
        for (let i = 0; i < this.N; i++) {
            result.data[i] -= v.data[i];
        }
        return result;
    }

    /** operator * (scalar) */
    scaleNew(n: number): VecN {
        const result = VecN.from(this);
        result.scaleAssign(n);
        return result;
    }

    /** operator += */
    addAssign(v: VecN): this {
        for (let i = 0; i < this.N; i++) {
            this.data[i] += v.data[i];
        }
        return this;
    }

    /** operator -= */
    subAssign(v: VecN): this {
        for (let i = 0; i < this.N; i++) {
            this.data[i] -= v.data[i];
        }
        return this;
    }

    /** operator *= */
    scaleAssign(n: number): this {
        for (let i = 0; i < this.N; i++) {
            this.data[i] *= n;
        }
        return this;
    }

    /** operator [] const */
    get(index: number): number {
        return this.data[index];
    }

    /** operator [] non-const */
    set(index: number, value: number): void {
        this.data[index] = value;
    }
}
