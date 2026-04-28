import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';

export type ChargeQuad = {
    centerX: number;
    centerY: number;
    size: number;
};

export type ChargeQuadNode = {
    children: number;
    next: number;
    posX: number;
    posY: number;
    positiveCharge: number;
    positiveChargeX: number;
    positiveChargeY: number;
    negativeCharge: number;
    negativeChargeX: number;
    negativeChargeY: number;
    quad: ChargeQuad;
};

export class ChargeQuadTree {
    static readonly ROOT = 0;

    thetaSquared: number;
    readonly nodes: ChargeQuadNode[] = [];
    private readonly parents: number[] = [];

    constructor(theta = 0.5) {
        this.thetaSquared = theta * theta;
    }

    static newContaining(bodies: readonly RigidBody[]): ChargeQuad | null {
        let minX = Number.POSITIVE_INFINITY;
        let minY = Number.POSITIVE_INFINITY;
        let maxX = Number.NEGATIVE_INFINITY;
        let maxY = Number.NEGATIVE_INFINITY;

        for (let i = 0; i < bodies.length; i++) {
            const body = bodies[i];
            if (body.charge === 0) continue;

            const x = body.position.x;
            const y = body.position.y;

            minX = Math.min(minX, x);
            minY = Math.min(minY, y);
            maxX = Math.max(maxX, x);
            maxY = Math.max(maxY, y);
        }

        if (minX === Number.POSITIVE_INFINITY) {
            return null;
        }

        return {
            centerX: (minX + maxX) * 0.5,
            centerY: (minY + maxY) * 0.5,
            size: Math.max(maxX - minX, maxY - minY),
        };
    }

    build(bodies: readonly RigidBody[]): boolean {
        const quad = ChargeQuadTree.newContaining(bodies);
        if (quad === null) {
            this.nodes.length = 0;
            this.parents.length = 0;
            return false;
        }

        this.clear(quad);

        for (let i = 0; i < bodies.length; i++) {
            const body = bodies[i];
            this.insert(body.position, body.charge);
        }

        this.propagate();
        return true;
    }

    clear(quad: ChargeQuad): void {
        this.nodes.length = 0;
        this.parents.length = 0;
        this.nodes.push(createNode(0, quad));
    }

    insert(pos: Vec2, charge: number): void {
        if (charge === 0) return;

        let node = ChargeQuadTree.ROOT;

        while (this.nodes[node].children !== 0) {
            const quadrant = findQuadrant(this.nodes[node].quad, pos.x, pos.y);
            node = this.nodes[node].children + quadrant;
        }

        if (isEmpty(this.nodes[node])) {
            setLeaf(this.nodes[node], pos.x, pos.y, charge);
            return;
        }

        const x = this.nodes[node].posX;
        const y = this.nodes[node].posY;
        const existingPositiveCharge = this.nodes[node].positiveCharge;
        const existingNegativeCharge = this.nodes[node].negativeCharge;

        if (pos.x === x && pos.y === y) {
            addCharge(this.nodes[node], pos.x, pos.y, charge);
            return;
        }

        for (;;) {
            const children = this.subdivide(node);
            const q1 = findQuadrant(this.nodes[node].quad, x, y);
            const q2 = findQuadrant(this.nodes[node].quad, pos.x, pos.y);

            if (q1 === q2) {
                node = children + q1;
                continue;
            }

            const n1 = this.nodes[children + q1];
            setLeafFromAggregates(n1, x, y, existingPositiveCharge, existingNegativeCharge);

            const n2 = this.nodes[children + q2];
            setLeaf(n2, pos.x, pos.y, charge);
            return;
        }
    }

    propagate(): void {
        for (let p = this.parents.length - 1; p >= 0; p--) {
            const nodeIndex = this.parents[p];
            const firstChild = this.nodes[nodeIndex].children;

            let positiveChargeX = 0;
            let positiveChargeY = 0;
            let positiveCharge = 0;
            let negativeChargeX = 0;
            let negativeChargeY = 0;
            let negativeCharge = 0;

            for (let i = 0; i < 4; i++) {
                const child = this.nodes[firstChild + i];
                positiveChargeX += child.positiveChargeX * child.positiveCharge;
                positiveChargeY += child.positiveChargeY * child.positiveCharge;
                positiveCharge += child.positiveCharge;
                negativeChargeX += child.negativeChargeX * child.negativeCharge;
                negativeChargeY += child.negativeChargeY * child.negativeCharge;
                negativeCharge += child.negativeCharge;
            }

            const node = this.nodes[nodeIndex];
            node.positiveCharge = positiveCharge;
            node.positiveChargeX = positiveCharge === 0 ? 0 : positiveChargeX / positiveCharge;
            node.positiveChargeY = positiveCharge === 0 ? 0 : positiveChargeY / positiveCharge;
            node.negativeCharge = negativeCharge;
            node.negativeChargeX = negativeCharge === 0 ? 0 : negativeChargeX / negativeCharge;
            node.negativeChargeY = negativeCharge === 0 ? 0 : negativeChargeY / negativeCharge;

            const totalChargeMagnitude = positiveCharge + negativeCharge;
            node.posX =
                totalChargeMagnitude === 0
                    ? 0
                    : (node.positiveChargeX * positiveCharge + node.negativeChargeX * negativeCharge) /
                      totalChargeMagnitude;
            node.posY =
                totalChargeMagnitude === 0
                    ? 0
                    : (node.positiveChargeY * positiveCharge + node.negativeChargeY * negativeCharge) /
                      totalChargeMagnitude;
        }
    }

    forceOn(body: RigidBody, k: number, out = new Vec2(), thetaSquared = this.thetaSquared, epsilon = 0.01): Vec2 {
        out.x = 0;
        out.y = 0;

        if (this.nodes.length === 0 || body.charge === 0) {
            return out;
        }

        return this.forceAt(body.position.x, body.position.y, body.charge, k, out, thetaSquared, epsilon);
    }

    forceAt(
        x: number,
        y: number,
        charge: number,
        k: number,
        out = new Vec2(),
        thetaSquared = this.thetaSquared,
        epsilon = 0.01,
    ): Vec2 {
        out.x = 0;
        out.y = 0;

        if (this.nodes.length === 0 || charge === 0) {
            return out;
        }

        let nodeIndex = ChargeQuadTree.ROOT;

        for (;;) {
            const node = this.nodes[nodeIndex];

            if (isEmpty(node)) {
                if (node.next === 0) {
                    break;
                }

                nodeIndex = node.next;
                continue;
            }

            if (node.children === 0 || canApproximateNode(node, x, y, thetaSquared)) {
                addChargeForce(
                    out,
                    charge,
                    node.positiveCharge,
                    node.positiveChargeX,
                    node.positiveChargeY,
                    x,
                    y,
                    k,
                    epsilon,
                );
                addChargeForce(
                    out,
                    charge,
                    -node.negativeCharge,
                    node.negativeChargeX,
                    node.negativeChargeY,
                    x,
                    y,
                    k,
                    epsilon,
                );

                if (node.next === 0) {
                    break;
                }

                nodeIndex = node.next;
            } else {
                nodeIndex = node.children;
            }
        }

        return out;
    }

    private subdivide(nodeIndex: number): number {
        this.parents.push(nodeIndex);

        const node = this.nodes[nodeIndex];
        const children = this.nodes.length;
        node.children = children;

        this.nodes.push(createNode(children + 1, intoQuadrant(node.quad, 0)));
        this.nodes.push(createNode(children + 2, intoQuadrant(node.quad, 1)));
        this.nodes.push(createNode(children + 3, intoQuadrant(node.quad, 2)));
        this.nodes.push(createNode(node.next, intoQuadrant(node.quad, 3)));

        return children;
    }
}

export function buildChargeQuadTree(bodies: readonly RigidBody[], theta = 0.5): ChargeQuadTree | null {
    const tree = new ChargeQuadTree(theta);
    return tree.build(bodies) ? tree : null;
}

export function canApproximate(node: ChargeQuadNode, body: RigidBody, theta: number): boolean {
    return canApproximateNode(node, body.position.x, body.position.y, theta * theta);
}

function createNode(next: number, quad: ChargeQuad): ChargeQuadNode {
    return {
        children: 0,
        next,
        posX: 0,
        posY: 0,
        positiveCharge: 0,
        positiveChargeX: 0,
        positiveChargeY: 0,
        negativeCharge: 0,
        negativeChargeX: 0,
        negativeChargeY: 0,
        quad,
    };
}

function isEmpty(node: ChargeQuadNode): boolean {
    return node.positiveCharge === 0 && node.negativeCharge === 0;
}

function setLeaf(node: ChargeQuadNode, x: number, y: number, charge: number): void {
    node.posX = x;
    node.posY = y;
    node.positiveCharge = charge > 0 ? charge : 0;
    node.positiveChargeX = charge > 0 ? x : 0;
    node.positiveChargeY = charge > 0 ? y : 0;
    node.negativeCharge = charge < 0 ? -charge : 0;
    node.negativeChargeX = charge < 0 ? x : 0;
    node.negativeChargeY = charge < 0 ? y : 0;
}

function setLeafFromAggregates(
    node: ChargeQuadNode,
    x: number,
    y: number,
    positiveCharge: number,
    negativeCharge: number,
): void {
    node.posX = x;
    node.posY = y;
    node.positiveCharge = positiveCharge;
    node.positiveChargeX = positiveCharge === 0 ? 0 : x;
    node.positiveChargeY = positiveCharge === 0 ? 0 : y;
    node.negativeCharge = negativeCharge;
    node.negativeChargeX = negativeCharge === 0 ? 0 : x;
    node.negativeChargeY = negativeCharge === 0 ? 0 : y;
}

function addCharge(node: ChargeQuadNode, x: number, y: number, charge: number): void {
    if (charge > 0) {
        node.positiveCharge += charge;
        node.positiveChargeX = x;
        node.positiveChargeY = y;
        return;
    }

    if (charge < 0) {
        node.negativeCharge += -charge;
        node.negativeChargeX = x;
        node.negativeChargeY = y;
    }
}

function canApproximateNode(node: ChargeQuadNode, x: number, y: number, thetaSquared: number): boolean {
    if (thetaSquared <= 0) {
        return false;
    }

    const sizeSquared = node.quad.size * node.quad.size;

    if (node.positiveCharge !== 0) {
        const dx = node.positiveChargeX - x;
        const dy = node.positiveChargeY - y;
        const distanceSquared = dx * dx + dy * dy;

        if (distanceSquared === 0 || sizeSquared >= distanceSquared * thetaSquared) {
            return false;
        }
    }

    if (node.negativeCharge !== 0) {
        const dx = node.negativeChargeX - x;
        const dy = node.negativeChargeY - y;
        const distanceSquared = dx * dx + dy * dy;

        if (distanceSquared === 0 || sizeSquared >= distanceSquared * thetaSquared) {
            return false;
        }
    }

    return true;
}

function addChargeForce(
    force: Vec2,
    bodyCharge: number,
    clusterCharge: number,
    sourceX: number,
    sourceY: number,
    targetX: number,
    targetY: number,
    k: number,
    epsilon: number,
): void {
    if (clusterCharge === 0) {
        return;
    }

    const dx = sourceX - targetX;
    const dy = sourceY - targetY;
    const distanceSquared = dx * dx + dy * dy;

    if (distanceSquared === 0) {
        return;
    }

    const safeDistanceSquared = distanceSquared + epsilon;
    const inverseDistance = 1 / Math.sqrt(safeDistanceSquared);
    const forceScalar = (-k * (bodyCharge * clusterCharge)) / safeDistanceSquared;

    force.x += dx * inverseDistance * forceScalar;
    force.y += dy * inverseDistance * forceScalar;
}

function findQuadrant(quad: ChargeQuad, x: number, y: number): number {
    return ((y > quad.centerY ? 1 : 0) << 1) | (x > quad.centerX ? 1 : 0);
}

function intoQuadrant(quad: ChargeQuad, quadrant: number): ChargeQuad {
    const size = quad.size * 0.5;

    return {
        centerX: quad.centerX + ((quadrant & 1) - 0.5) * size,
        centerY: quad.centerY + (((quadrant >> 1) & 1) - 0.5) * size,
        size,
    };
}
