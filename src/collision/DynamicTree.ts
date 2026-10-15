/*
 * Portions of this file are derived from Box2D.
 *
 * Copyright (c) 2009 Erin Catto
 * Licensed under the MIT License
 */
const NULL_NODE = -1;
const INITIAL_CAPACITY = 256;

/**
 * Dynamic AABB tree (bounding volume hierarchy) used by the broad phase, a port of Box2D's b2DynamicTree.
 *
 * Leaves store fattened AABBs: a proxy is reinserted only when its body leaves the fattened box,
 * so resting or slowly moving bodies cost nothing to keep up to date. The tree is kept balanced
 * with AVL style rotations. Node data lives in typed arrays indexed by node id.
 */
export class DynamicTree<T> {
    private root = NULL_NODE;
    private capacity = 0;
    private freeList = NULL_NODE;
    private nodeCount = 0;

    /** minX, minY, maxX, maxY for each node */
    private aabbs = new Float64Array(0);
    private parents = new Int32Array(0);
    private children1 = new Int32Array(0);
    private children2 = new Int32Array(0);
    /** Leaves have height 0, free nodes have height -1 */
    private heights = new Int32Array(0);
    private moved = new Uint8Array(0);
    private userData: (T | null)[] = [];

    private stack = new Int32Array(64);

    constructor() {
        this.grow(INITIAL_CAPACITY);
    }

    get proxyCount(): number {
        return (this.nodeCount + 1) >> 1;
    }

    /** Creates a leaf with the given fattened AABB, returns the proxy id */
    createProxy(minX: number, minY: number, maxX: number, maxY: number, data: T): number {
        const proxyId = this.allocateNode();
        this.setAABB(proxyId, minX, minY, maxX, maxY);
        this.userData[proxyId] = data;
        this.heights[proxyId] = 0;
        this.moved[proxyId] = 1;

        this.insertLeaf(proxyId);

        return proxyId;
    }

    destroyProxy(proxyId: number): void {
        this.removeLeaf(proxyId);
        this.freeNode(proxyId);
    }

    /** Replaces the fattened AABB of a proxy and reinserts it, flagging it as moved */
    moveProxy(proxyId: number, minX: number, minY: number, maxX: number, maxY: number): void {
        this.removeLeaf(proxyId);
        this.setAABB(proxyId, minX, minY, maxX, maxY);
        this.insertLeaf(proxyId);
        this.moved[proxyId] = 1;
    }

    /** True when the fattened AABB of the proxy fully contains the given box */
    contains(proxyId: number, minX: number, minY: number, maxX: number, maxY: number): boolean {
        const a = this.aabbs;
        const i = proxyId * 4;
        return a[i] <= minX && a[i + 1] <= minY && a[i + 2] >= maxX && a[i + 3] >= maxY;
    }

    /** True when the fattened AABB of the proxy fully lies inside the given box */
    isInside(proxyId: number, minX: number, minY: number, maxX: number, maxY: number): boolean {
        const a = this.aabbs;
        const i = proxyId * 4;
        return minX <= a[i] && minY <= a[i + 1] && maxX >= a[i + 2] && maxY >= a[i + 3];
    }

    /** True when the fattened AABBs of two proxies overlap */
    overlaps(proxyA: number, proxyB: number): boolean {
        const a = this.aabbs;
        const i = proxyA * 4;
        const j = proxyB * 4;
        return !(a[j] > a[i + 2] || a[j + 2] < a[i] || a[j + 1] > a[i + 3] || a[j + 3] < a[i + 1]);
    }

    wasMoved(proxyId: number): boolean {
        return this.moved[proxyId] !== 0;
    }

    clearMoved(proxyId: number): void {
        this.moved[proxyId] = 0;
    }

    getUserData(proxyId: number): T {
        return this.userData[proxyId] as T;
    }

    getFatMinX(proxyId: number): number {
        return this.aabbs[proxyId * 4];
    }

    getFatMinY(proxyId: number): number {
        return this.aabbs[proxyId * 4 + 1];
    }

    getFatMaxX(proxyId: number): number {
        return this.aabbs[proxyId * 4 + 2];
    }

    getFatMaxY(proxyId: number): number {
        return this.aabbs[proxyId * 4 + 3];
    }

    /** Collects the ids of the proxies whose fattened AABB overlaps the box into `out`, returns their count */
    query(minX: number, minY: number, maxX: number, maxY: number, out: number[]): number {
        out.length = 0;
        if (this.root === NULL_NODE) return 0;

        const a = this.aabbs;
        const children1 = this.children1;
        const children2 = this.children2;
        let stack = this.stack;
        let top = 0;
        stack[top++] = this.root;

        while (top > 0) {
            const node = stack[--top];
            const i = node * 4;

            if (a[i] > maxX || a[i + 2] < minX || a[i + 1] > maxY || a[i + 3] < minY) continue;

            const child1 = children1[node];

            if (child1 === NULL_NODE) {
                out.push(node);
            } else {
                if (top + 2 > stack.length) {
                    const grown = new Int32Array(stack.length * 2);
                    grown.set(stack);
                    this.stack = stack = grown;
                }

                stack[top++] = child1;
                stack[top++] = children2[node];
            }
        }

        return out.length;
    }

    clear(): void {
        this.root = NULL_NODE;
        this.nodeCount = 0;
        this.capacity = 0;
        this.freeList = NULL_NODE;
        this.userData = [];
        this.grow(INITIAL_CAPACITY);
    }

    /** Checks the tree invariants, used by tests. Returns the height of the tree */
    validate(): number {
        if (this.root === NULL_NODE) return 0;

        const check = (node: number, parent: number): number => {
            if (this.parents[node] !== parent) throw new Error(`Bad parent of node ${node}`);

            const child1 = this.children1[node];
            const child2 = this.children2[node];

            if (child1 === NULL_NODE) {
                if (child2 !== NULL_NODE || this.heights[node] !== 0) throw new Error(`Bad leaf ${node}`);
                return 0;
            }

            const height1 = check(child1, node);
            const height2 = check(child2, node);
            const height = 1 + Math.max(height1, height2);

            if (this.heights[node] !== height) throw new Error(`Bad height of node ${node}`);

            for (const child of [child1, child2]) {
                if (!this.isInside(child, ...this.getAABB(node))) {
                    throw new Error(`Node ${node} does not contain child ${child}`);
                }
            }

            return height;
        };

        return check(this.root, NULL_NODE);
    }

    private getAABB(node: number): [number, number, number, number] {
        const i = node * 4;
        return [this.aabbs[i], this.aabbs[i + 1], this.aabbs[i + 2], this.aabbs[i + 3]];
    }

    private setAABB(node: number, minX: number, minY: number, maxX: number, maxY: number): void {
        const i = node * 4;
        this.aabbs[i] = minX;
        this.aabbs[i + 1] = minY;
        this.aabbs[i + 2] = maxX;
        this.aabbs[i + 3] = maxY;
    }

    /** Sets the AABB of `node` to the union of the AABBs of `a` and `b` */
    private setUnion(node: number, a: number, b: number): void {
        const aabbs = this.aabbs;
        const i = a * 4;
        const j = b * 4;
        const k = node * 4;
        aabbs[k] = Math.min(aabbs[i], aabbs[j]);
        aabbs[k + 1] = Math.min(aabbs[i + 1], aabbs[j + 1]);
        aabbs[k + 2] = Math.max(aabbs[i + 2], aabbs[j + 2]);
        aabbs[k + 3] = Math.max(aabbs[i + 3], aabbs[j + 3]);
    }

    private perimeter(node: number): number {
        const a = this.aabbs;
        const i = node * 4;
        return 2 * (a[i + 2] - a[i] + (a[i + 3] - a[i + 1]));
    }

    private unionPerimeter(a: number, b: number): number {
        const aabbs = this.aabbs;
        const i = a * 4;
        const j = b * 4;
        const width = Math.max(aabbs[i + 2], aabbs[j + 2]) - Math.min(aabbs[i], aabbs[j]);
        const height = Math.max(aabbs[i + 3], aabbs[j + 3]) - Math.min(aabbs[i + 1], aabbs[j + 1]);
        return 2 * (width + height);
    }

    private grow(capacity: number): void {
        const aabbs = new Float64Array(capacity * 4);
        const parents = new Int32Array(capacity);
        const children1 = new Int32Array(capacity);
        const children2 = new Int32Array(capacity);
        const heights = new Int32Array(capacity);
        const moved = new Uint8Array(capacity);

        aabbs.set(this.aabbs.subarray(0, this.capacity * 4));
        parents.set(this.parents.subarray(0, this.capacity));
        children1.set(this.children1.subarray(0, this.capacity));
        children2.set(this.children2.subarray(0, this.capacity));
        heights.set(this.heights.subarray(0, this.capacity));
        moved.set(this.moved.subarray(0, this.capacity));

        // Link the new nodes into the free list
        for (let i = this.capacity; i < capacity; i++) {
            parents[i] = i + 1 < capacity ? i + 1 : this.freeList;
            heights[i] = -1;
            this.userData[i] = null;
        }

        this.freeList = this.capacity;
        this.aabbs = aabbs;
        this.parents = parents;
        this.children1 = children1;
        this.children2 = children2;
        this.heights = heights;
        this.moved = moved;
        this.capacity = capacity;
    }

    private allocateNode(): number {
        if (this.freeList === NULL_NODE) {
            this.grow(this.capacity * 2);
        }

        const node = this.freeList;
        this.freeList = this.parents[node];
        this.parents[node] = NULL_NODE;
        this.children1[node] = NULL_NODE;
        this.children2[node] = NULL_NODE;
        this.heights[node] = 0;
        this.moved[node] = 0;
        this.nodeCount++;

        return node;
    }

    private freeNode(node: number): void {
        this.parents[node] = this.freeList;
        this.heights[node] = -1;
        this.moved[node] = 0;
        this.userData[node] = null;
        this.freeList = node;
        this.nodeCount--;
    }

    private insertLeaf(leaf: number): void {
        if (this.root === NULL_NODE) {
            this.root = leaf;
            this.parents[leaf] = NULL_NODE;
            return;
        }

        let children1 = this.children1;
        let children2 = this.children2;

        // Find the best sibling using the surface area (perimeter in 2D) heuristic
        let index = this.root;

        while (children1[index] !== NULL_NODE) {
            const child1 = children1[index];
            const child2 = children2[index];

            const area = this.perimeter(index);
            const combinedArea = this.unionPerimeter(index, leaf);

            // Cost of creating a new parent for this node and the new leaf
            const cost = 2 * combinedArea;

            // Minimum cost of pushing the leaf further down the tree
            const inheritanceCost = 2 * (combinedArea - area);

            const cost1 =
                children1[child1] === NULL_NODE
                    ? this.unionPerimeter(leaf, child1) + inheritanceCost
                    : this.unionPerimeter(leaf, child1) - this.perimeter(child1) + inheritanceCost;

            const cost2 =
                children1[child2] === NULL_NODE
                    ? this.unionPerimeter(leaf, child2) + inheritanceCost
                    : this.unionPerimeter(leaf, child2) - this.perimeter(child2) + inheritanceCost;

            if (cost < cost1 && cost < cost2) break;

            index = cost1 < cost2 ? child1 : child2;
        }

        const sibling = index;

        // Create a new parent
        const oldParent = this.parents[sibling];
        const newParent = this.allocateNode();

        // Allocation may have grown the node arrays
        children1 = this.children1;
        children2 = this.children2;

        this.parents[newParent] = oldParent;
        this.userData[newParent] = null;
        this.setUnion(newParent, leaf, sibling);
        this.heights[newParent] = this.heights[sibling] + 1;

        if (oldParent !== NULL_NODE) {
            if (children1[oldParent] === sibling) {
                children1[oldParent] = newParent;
            } else {
                children2[oldParent] = newParent;
            }
        } else {
            this.root = newParent;
        }

        children1[newParent] = sibling;
        children2[newParent] = leaf;
        this.parents[sibling] = newParent;
        this.parents[leaf] = newParent;

        // Walk back up the tree fixing heights and AABBs
        this.fixUpwards(this.parents[leaf]);
    }

    private removeLeaf(leaf: number): void {
        if (leaf === this.root) {
            this.root = NULL_NODE;
            return;
        }

        const parent = this.parents[leaf];
        const grandParent = this.parents[parent];
        const sibling = this.children1[parent] === leaf ? this.children2[parent] : this.children1[parent];

        if (grandParent !== NULL_NODE) {
            // Destroy parent and connect sibling to grandParent
            if (this.children1[grandParent] === parent) {
                this.children1[grandParent] = sibling;
            } else {
                this.children2[grandParent] = sibling;
            }

            this.parents[sibling] = grandParent;
            this.freeNode(parent);

            this.fixUpwards(grandParent);
        } else {
            this.root = sibling;
            this.parents[sibling] = NULL_NODE;
            this.freeNode(parent);
        }
    }

    private fixUpwards(start: number): void {
        let index = start;

        while (index !== NULL_NODE) {
            index = this.balance(index);

            const child1 = this.children1[index];
            const child2 = this.children2[index];

            this.heights[index] = 1 + Math.max(this.heights[child1], this.heights[child2]);
            this.setUnion(index, child1, child2);

            index = this.parents[index];
        }
    }

    /** Performs a left or right rotation if node A is imbalanced, returns the new root of the subtree */
    private balance(iA: number): number {
        const parents = this.parents;
        const children1 = this.children1;
        const children2 = this.children2;
        const heights = this.heights;

        if (children1[iA] === NULL_NODE || heights[iA] < 2) {
            return iA;
        }

        const iB = children1[iA];
        const iC = children2[iA];
        const balance = heights[iC] - heights[iB];

        // Rotate C up
        if (balance > 1) {
            const iF = children1[iC];
            const iG = children2[iC];

            // Swap A and C
            children1[iC] = iA;
            parents[iC] = parents[iA];
            parents[iA] = iC;

            // A's old parent should point to C
            const parentC = parents[iC];
            if (parentC !== NULL_NODE) {
                if (children1[parentC] === iA) {
                    children1[parentC] = iC;
                } else {
                    children2[parentC] = iC;
                }
            } else {
                this.root = iC;
            }

            // Rotate
            if (heights[iF] > heights[iG]) {
                children2[iC] = iF;
                children2[iA] = iG;
                parents[iG] = iA;
                this.setUnion(iA, iB, iG);
                this.setUnion(iC, iA, iF);

                heights[iA] = 1 + Math.max(heights[iB], heights[iG]);
                heights[iC] = 1 + Math.max(heights[iA], heights[iF]);
            } else {
                children2[iC] = iG;
                children2[iA] = iF;
                parents[iF] = iA;
                this.setUnion(iA, iB, iF);
                this.setUnion(iC, iA, iG);

                heights[iA] = 1 + Math.max(heights[iB], heights[iF]);
                heights[iC] = 1 + Math.max(heights[iA], heights[iG]);
            }

            return iC;
        }

        // Rotate B up
        if (balance < -1) {
            const iD = children1[iB];
            const iE = children2[iB];

            // Swap A and B
            children1[iB] = iA;
            parents[iB] = parents[iA];
            parents[iA] = iB;

            // A's old parent should point to B
            const parentB = parents[iB];
            if (parentB !== NULL_NODE) {
                if (children1[parentB] === iA) {
                    children1[parentB] = iB;
                } else {
                    children2[parentB] = iB;
                }
            } else {
                this.root = iB;
            }

            // Rotate
            if (heights[iD] > heights[iE]) {
                children2[iB] = iD;
                children1[iA] = iE;
                parents[iE] = iA;
                this.setUnion(iA, iC, iE);
                this.setUnion(iB, iA, iD);

                heights[iA] = 1 + Math.max(heights[iC], heights[iE]);
                heights[iB] = 1 + Math.max(heights[iA], heights[iD]);
            } else {
                children2[iB] = iE;
                children1[iA] = iD;
                parents[iD] = iA;
                this.setUnion(iA, iC, iD);
                this.setUnion(iB, iA, iE);

                heights[iA] = 1 + Math.max(heights[iC], heights[iD]);
                heights[iB] = 1 + Math.max(heights[iA], heights[iE]);
            }

            return iB;
        }

        return iA;
    }
}
