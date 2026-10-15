const INITIAL_CAPACITY = 1024;

/**
 * Hash map from an unordered body id pair to a value, using open addressing with linear probing.
 *
 * Compared to a `Map` with composed keys it never allocates keys, works for any id range
 * and is cleared in O(1) by bumping a generation stamp instead of touching every slot.
 */
export class PairCache<T> {
    private capacity = INITIAL_CAPACITY;
    private mask = INITIAL_CAPACITY - 1;
    private size = 0;
    private generation = 1;

    private idsA = new Int32Array(INITIAL_CAPACITY);
    private idsB = new Int32Array(INITIAL_CAPACITY);
    /** A slot is occupied only when its stamp matches the current generation */
    private stamps = new Uint32Array(INITIAL_CAPACITY);
    private values: (T | undefined)[] = new Array(INITIAL_CAPACITY).fill(undefined);

    get count(): number {
        return this.size;
    }

    clear(): void {
        this.size = 0;
        this.generation++;

        if (this.generation === 0xffffffff) {
            this.stamps.fill(0);
            this.generation = 1;
        }
    }

    /** Ids must be ordered so that `idA < idB` */
    get(idA: number, idB: number): T | undefined {
        const stamps = this.stamps;
        const generation = this.generation;
        let slot = hash(idA, idB) & this.mask;

        while (stamps[slot] === generation) {
            if (this.idsA[slot] === idA && this.idsB[slot] === idB) {
                return this.values[slot];
            }

            slot = (slot + 1) & this.mask;
        }

        return undefined;
    }

    /** Ids must be ordered so that `idA < idB` */
    set(idA: number, idB: number, value: T): void {
        if ((this.size + 1) * 2 > this.capacity) {
            this.grow();
        }

        const stamps = this.stamps;
        const generation = this.generation;
        let slot = hash(idA, idB) & this.mask;

        while (stamps[slot] === generation) {
            if (this.idsA[slot] === idA && this.idsB[slot] === idB) {
                this.values[slot] = value;
                return;
            }

            slot = (slot + 1) & this.mask;
        }

        stamps[slot] = generation;
        this.idsA[slot] = idA;
        this.idsB[slot] = idB;
        this.values[slot] = value;
        this.size++;
    }

    /** Ids must be ordered so that `idA < idB`. Returns false when the pair was not present */
    remove(idA: number, idB: number): boolean {
        const stamps = this.stamps;
        const generation = this.generation;
        const mask = this.mask;
        let slot = hash(idA, idB) & mask;

        while (stamps[slot] === generation) {
            if (this.idsA[slot] === idA && this.idsB[slot] === idB) break;
            slot = (slot + 1) & mask;
        }

        if (stamps[slot] !== generation) return false;

        // Backward shift deletion: move later entries of the probe sequence into the hole, so that
        // lookups never stop early at an empty slot and no tombstones are needed
        let hole = slot;
        let next = slot;

        for (;;) {
            stamps[hole] = 0;
            this.values[hole] = undefined;

            for (;;) {
                next = (next + 1) & mask;

                if (stamps[next] !== generation) {
                    this.size--;
                    return true;
                }

                const ideal = hash(this.idsA[next], this.idsB[next]) & mask;

                // The entry can stay when its ideal slot lies cyclically in (hole, next]
                const stays = hole <= next ? hole < ideal && ideal <= next : hole < ideal || ideal <= next;
                if (!stays) break;
            }

            stamps[hole] = generation;
            this.idsA[hole] = this.idsA[next];
            this.idsB[hole] = this.idsB[next];
            this.values[hole] = this.values[next];
            hole = next;
        }
    }

    private grow(): void {
        const oldIdsA = this.idsA;
        const oldIdsB = this.idsB;
        const oldStamps = this.stamps;
        const oldValues = this.values;
        const oldGeneration = this.generation;

        this.capacity *= 2;
        this.mask = this.capacity - 1;
        this.idsA = new Int32Array(this.capacity);
        this.idsB = new Int32Array(this.capacity);
        this.stamps = new Uint32Array(this.capacity);
        this.values = new Array(this.capacity).fill(undefined);
        this.generation = 1;
        this.size = 0;

        for (let i = 0; i < oldStamps.length; i++) {
            if (oldStamps[i] === oldGeneration) {
                this.set(oldIdsA[i], oldIdsB[i], oldValues[i] as T);
            }
        }
    }
}

function hash(idA: number, idB: number): number {
    return (Math.imul(idA, 0x9e3779b1) ^ Math.imul(idB, 0x85ebca77)) >>> 0;
}
