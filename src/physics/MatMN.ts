import VecN from './VecN';

export default class MatMN {
    M: number; // rows
    N: number; // cols
    rows: VecN[]; // array of VecN, each with N columns

    constructor(M = 0, N = 0) {
        this.M = M;
        this.N = N;
        this.rows = [];

        if (M > 0 && N > 0) {
            for (let i = 0; i < M; i++) {
                this.rows[i] = new VecN(N);
            }
        }
    }

    // Copy constructor
    static copy(m: MatMN): MatMN {
        return new MatMN().assign(m);
    }

    /** operator = */
    assign(m: MatMN): this {
        this.M = m.M;
        this.N = m.N;
        this.rows = [];

        for (let i = 0; i < this.M; i++) {
            this.rows[i] = new VecN(this.N);
            this.rows[i].assign(m.rows[i]);
        }

        return this;
    }

    zero(): void {
        for (let i = 0; i < this.M; i++) {
            this.rows[i].zero();
        }
    }

    transpose(): MatMN {
        const result = new MatMN(this.N, this.M);

        for (let i = 0; i < this.M; i++) {
            for (let j = 0; j < this.N; j++) {
                result.rows[j].set(i, this.rows[i].get(j));
            }
        }
        return result;
    }

    /** Matrix * Vector */
    multiplyVec(v: VecN): VecN {
        if (v.N !== this.N) {
            return v;
        }

        const result = new VecN(this.M);

        for (let i = 0; i < this.M; i++) {
            result.set(i, this.rows[i].dot(v));
        }

        return result;
    }

    /** Matrix * Matrix */
    multiplyMat(m: MatMN): MatMN {
        if (m.M !== this.N) {
            throw new Error('Matrix dimension mismatch');
        }

        const transposed = m.transpose();
        const result = new MatMN(this.M, m.N);

        for (let i = 0; i < this.M; i++) {
            for (let j = 0; j < m.N; j++) {
                const dot = this.rows[i].dot(transposed.rows[j]);
                result.rows[i].set(j, dot);
            }
        }

        return result;
    }

    static solveGaussSeidel(A: MatMN, b: VecN): VecN {
        const N = b.N;
        const X = new VecN(N);
        X.zero();

        for (let iterations = 0; iterations < N; iterations++) {
            for (let i = 0; i < N; i++) {
                const diag = A.rows[i].get(i);
                const dx = b.get(i) / diag - A.rows[i].dot(X) / diag;

                if (dx === dx) {
                    // NaN check
                    X.set(i, X.get(i) + dx);
                }
            }
        }

        return X;
    }
}
