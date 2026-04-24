import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { generateGravitationalForce } from './Gravity';

const MIN_NODE_HALF_SIZE = 1;

export type BarnesHutTreeKind = 'gravity' | 'coulomb' | 'combined';

export type BarnesHutQuadTree = {
    centerX: number;
    centerY: number;
    halfSize: number;
    bodyCount: number;
    totalMass: number;
    centerOfMassX: number;
    centerOfMassY: number;
    positiveCharge: number;
    positiveChargeX: number;
    positiveChargeY: number;
    negativeCharge: number;
    negativeChargeX: number;
    negativeChargeY: number;
    bodies: RigidBody[];
    children: BarnesHutQuadTree[] | null;
};

/**
 * Builds a quadtree once and stores the aggregates needed by Barnes-Hut.
 *
 * The same tree can be reused for:
 * - gravity, using the node mass and center of mass
 * - Coulomb forces, using separate positive/negative charge aggregates
 *
 * Use `kind` to keep the tree focused on the force you want to compute:
 * - `gravity`: insert only bodies with mass
 * - `coulomb`: insert only charged bodies
 * - `combined`: insert bodies that matter for either force
 */
export function buildBarnesHutQuadTree(
    bodies: readonly RigidBody[],
    kind: BarnesHutTreeKind = 'combined',
): BarnesHutQuadTree | null {
    let hasContributor = false;
    let minX = 0;
    let maxX = 0;
    let minY = 0;
    let maxY = 0;

    for (let i = 0; i < bodies.length; i++) {
        const body = bodies[i];
        if (!bodyContributesToTree(body, kind)) continue;

        const x = body.position.x;
        const y = body.position.y;

        if (!hasContributor) {
            minX = x;
            maxX = x;
            minY = y;
            maxY = y;
            hasContributor = true;
            continue;
        }

        minX = Math.min(minX, x);
        maxX = Math.max(maxX, x);
        minY = Math.min(minY, y);
        maxY = Math.max(maxY, y);
    }

    if (!hasContributor) {
        return null;
    }

    const centerX = (minX + maxX) * 0.5;
    const centerY = (minY + maxY) * 0.5;
    const halfSize = Math.max((maxX - minX) * 0.5, (maxY - minY) * 0.5, MIN_NODE_HALF_SIZE) + 1;

    const root = createNode(centerX, centerY, halfSize);

    for (let i = 0; i < bodies.length; i++) {
        const body = bodies[i];
        if (!bodyContributesToTree(body, kind)) continue;
        insertBody(root, body);
    }

    return root;
}

/**
 * Computes the gravitational force on one body by traversing a Barnes-Hut quadtree.
 *
 * Use `theta = 0` to disable approximation and recover the exact pairwise sum.
 * Smaller `theta` is more accurate, larger `theta` is faster.
 */
export function generateBarnesHutGravitationalForce(
    body: RigidBody,
    tree: BarnesHutQuadTree | null,
    G: number,
    minDistanceSquared: number,
    maxDistanceSquared: number,
    theta = 0.5,
): Vec2 {
    if (tree === null || body.mass === 0) {
        return new Vec2();
    }

    const force = new Vec2();
    accumulateGravitationalForce(force, body, tree, G, minDistanceSquared, maxDistanceSquared, theta);
    return force;
}

/**
 * Convenience version that builds the tree once and returns one gravitational force per body.
 */
export function generateBarnesHutGravitationalForces(
    bodies: readonly RigidBody[],
    G: number,
    minDistanceSquared: number,
    maxDistanceSquared: number,
    theta = 0.5,
): Vec2[] {
    const tree = buildBarnesHutQuadTree(bodies, 'gravity');
    const forces = new Array<Vec2>(bodies.length);

    for (let i = 0; i < bodies.length; i++) {
        forces[i] = generateBarnesHutGravitationalForce(
            bodies[i],
            tree,
            G,
            minDistanceSquared,
            maxDistanceSquared,
            theta,
        );
    }

    return forces;
}

/**
 * Computes the Coulomb force on one body by traversing a Barnes-Hut quadtree.
 *
 * Positive and negative charges are aggregated separately so mixed-charge cells
 * can still be approximated without collapsing everything into a single net charge.
 */
export function generateBarnesHutCoulombForce(
    body: RigidBody,
    tree: BarnesHutQuadTree | null,
    k: number,
    theta = 0.5,
    epsilon = 0.01,
): Vec2 {
    if (tree === null || body.charge === 0) {
        return new Vec2();
    }

    const force = new Vec2();
    accumulateCoulombForce(force, body, tree, k, theta, epsilon);
    return force;
}

/**
 * Convenience version that builds the tree once and returns one Coulomb force per body.
 */
export function generateBarnesHutCoulombForces(
    bodies: readonly RigidBody[],
    k: number,
    theta = 0.5,
    epsilon = 0.01,
): Vec2[] {
    const tree = buildBarnesHutQuadTree(bodies, 'coulomb');
    const forces = new Array<Vec2>(bodies.length);

    for (let i = 0; i < bodies.length; i++) {
        forces[i] = generateBarnesHutCoulombForce(bodies[i], tree, k, theta, epsilon);
    }

    return forces;
}

function createNode(centerX: number, centerY: number, halfSize: number): BarnesHutQuadTree {
    return {
        centerX,
        centerY,
        halfSize,
        bodyCount: 0,
        totalMass: 0,
        centerOfMassX: 0,
        centerOfMassY: 0,
        positiveCharge: 0,
        positiveChargeX: 0,
        positiveChargeY: 0,
        negativeCharge: 0,
        negativeChargeX: 0,
        negativeChargeY: 0,
        bodies: [],
        children: null,
    };
}

function bodyContributesToTree(body: RigidBody, kind: BarnesHutTreeKind): boolean {
    if (kind === 'gravity') {
        return body.mass !== 0;
    }

    if (kind === 'coulomb') {
        return body.charge !== 0;
    }

    return body.mass !== 0 || body.charge !== 0;
}

function insertBody(node: BarnesHutQuadTree, body: RigidBody): void {
    node.bodyCount++;
    updateAggregates(node, body);

    if (node.children !== null) {
        insertBody(node.children[getChildIndex(node, body)], body);
        return;
    }

    if (node.bodies.length === 0) {
        node.bodies.push(body);
        return;
    }

    // Keep near-identical bodies together once the node becomes very small.
    if (node.halfSize <= MIN_NODE_HALF_SIZE || allBodiesSharePosition(node.bodies, body)) {
        node.bodies.push(body);
        return;
    }

    subdivide(node);
    const children = node.children!;

    const bodiesToReinsert = node.bodies;
    node.bodies = [];

    for (let i = 0; i < bodiesToReinsert.length; i++) {
        const current = bodiesToReinsert[i];
        insertBody(children[getChildIndex(node, current)], current);
    }

    insertBody(children[getChildIndex(node, body)], body);
}

function updateAggregates(node: BarnesHutQuadTree, body: RigidBody): void {
    if (body.mass !== 0) {
        const nextTotalMass = node.totalMass + body.mass;
        node.centerOfMassX =
            nextTotalMass === 0 ? 0 : (node.centerOfMassX * node.totalMass + body.position.x * body.mass) / nextTotalMass;
        node.centerOfMassY =
            nextTotalMass === 0 ? 0 : (node.centerOfMassY * node.totalMass + body.position.y * body.mass) / nextTotalMass;
        node.totalMass = nextTotalMass;
    }

    if (body.charge > 0) {
        const nextPositiveCharge = node.positiveCharge + body.charge;
        node.positiveChargeX =
            nextPositiveCharge === 0
                ? 0
                : (node.positiveChargeX * node.positiveCharge + body.position.x * body.charge) / nextPositiveCharge;
        node.positiveChargeY =
            nextPositiveCharge === 0
                ? 0
                : (node.positiveChargeY * node.positiveCharge + body.position.y * body.charge) / nextPositiveCharge;
        node.positiveCharge = nextPositiveCharge;
        return;
    }

    if (body.charge < 0) {
        const chargeMagnitude = -body.charge;
        const nextNegativeCharge = node.negativeCharge + chargeMagnitude;
        node.negativeChargeX =
            nextNegativeCharge === 0
                ? 0
                : (node.negativeChargeX * node.negativeCharge + body.position.x * chargeMagnitude) / nextNegativeCharge;
        node.negativeChargeY =
            nextNegativeCharge === 0
                ? 0
                : (node.negativeChargeY * node.negativeCharge + body.position.y * chargeMagnitude) / nextNegativeCharge;
        node.negativeCharge = nextNegativeCharge;
    }
}

function allBodiesSharePosition(bodies: readonly RigidBody[], body: RigidBody): boolean {
    for (let i = 0; i < bodies.length; i++) {
        const current = bodies[i];
        if (current.position.x !== body.position.x || current.position.y !== body.position.y) {
            return false;
        }
    }

    return true;
}

function subdivide(node: BarnesHutQuadTree): void {
    const childHalfSize = node.halfSize * 0.5;
    node.children = [
        createNode(node.centerX - childHalfSize, node.centerY - childHalfSize, childHalfSize),
        createNode(node.centerX + childHalfSize, node.centerY - childHalfSize, childHalfSize),
        createNode(node.centerX - childHalfSize, node.centerY + childHalfSize, childHalfSize),
        createNode(node.centerX + childHalfSize, node.centerY + childHalfSize, childHalfSize),
    ];
}

function getChildIndex(node: BarnesHutQuadTree, body: RigidBody): number {
    const east = body.position.x >= node.centerX ? 1 : 0;
    const south = body.position.y >= node.centerY ? 2 : 0;
    return east + south;
}

function canApproximate(node: BarnesHutQuadTree, body: RigidBody, theta: number): boolean {
    if (theta <= 0 || bodyIsInsideNode(node, body)) {
        return false;
    }

    const dx = node.centerX - body.position.x;
    const dy = node.centerY - body.position.y;
    const distanceSquared = dx * dx + dy * dy;

    if (distanceSquared === 0) {
        return false;
    }

    const size = node.halfSize * 2;
    return size * size < theta * theta * distanceSquared;
}

function bodyIsInsideNode(node: BarnesHutQuadTree, body: RigidBody): boolean {
    const x = body.position.x;
    const y = body.position.y;

    return (
        x >= node.centerX - node.halfSize &&
        x <= node.centerX + node.halfSize &&
        y >= node.centerY - node.halfSize &&
        y <= node.centerY + node.halfSize
    );
}

function accumulateGravitationalForce(
    force: Vec2,
    body: RigidBody,
    node: BarnesHutQuadTree,
    G: number,
    minDistanceSquared: number,
    maxDistanceSquared: number,
    theta: number,
): void {
    if (node.bodyCount === 0 || node.totalMass === 0) {
        return;
    }

    if (node.children === null) {
        for (let i = 0; i < node.bodies.length; i++) {
            const other = node.bodies[i];
            if (other.id === body.id) continue;
            force.addAssign(generateGravitationalForce(body, other, G, minDistanceSquared, maxDistanceSquared));
        }
        return;
    }

    if (canApproximate(node, body, theta)) {
        const dx = node.centerOfMassX - body.position.x;
        const dy = node.centerOfMassY - body.position.y;
        const distanceSquared = dx * dx + dy * dy;

        if (distanceSquared !== 0) {
            const clampedDistanceSquared = Math.min(
                Math.max(distanceSquared, minDistanceSquared),
                maxDistanceSquared,
            );
            const inverseDistance = 1 / Math.sqrt(distanceSquared);
            const magnitude = (G * body.mass * node.totalMass) / clampedDistanceSquared;
            force.x += dx * inverseDistance * magnitude;
            force.y += dy * inverseDistance * magnitude;
        }

        return;
    }

    for (let i = 0; i < node.children.length; i++) {
        accumulateGravitationalForce(force, body, node.children[i], G, minDistanceSquared, maxDistanceSquared, theta);
    }
}

function accumulateCoulombForce(
    force: Vec2,
    body: RigidBody,
    node: BarnesHutQuadTree,
    k: number,
    theta: number,
    epsilon: number,
): void {
    if (node.bodyCount === 0 || (node.positiveCharge === 0 && node.negativeCharge === 0)) {
        return;
    }

    if (node.children === null) {
        for (let i = 0; i < node.bodies.length; i++) {
            const other = node.bodies[i];
            if (other.id === body.id) continue;
            addChargeForce(force, body, other.charge, other.position.x, other.position.y, k, epsilon);
        }
        return;
    }

    if (canApproximate(node, body, theta)) {
        if (node.positiveCharge !== 0) {
            addChargeForce(force, body, node.positiveCharge, node.positiveChargeX, node.positiveChargeY, k, epsilon);
        }

        if (node.negativeCharge !== 0) {
            addChargeForce(force, body, -node.negativeCharge, node.negativeChargeX, node.negativeChargeY, k, epsilon);
        }

        return;
    }

    for (let i = 0; i < node.children.length; i++) {
        accumulateCoulombForce(force, body, node.children[i], k, theta, epsilon);
    }
}

function addChargeForce(
    force: Vec2,
    body: RigidBody,
    clusterCharge: number,
    sourceX: number,
    sourceY: number,
    k: number,
    epsilon: number,
): void {
    const dx = sourceX - body.position.x;
    const dy = sourceY - body.position.y;
    const distanceSquared = dx * dx + dy * dy;

    if (distanceSquared === 0) {
        return;
    }

    const safeDistanceSquared = distanceSquared + epsilon;
    const inverseDistance = 1 / Math.sqrt(safeDistanceSquared);
    const forceScalar = (-k * (body.charge * clusterCharge)) / safeDistanceSquared;

    force.x += dx * inverseDistance * forceScalar;
    force.y += dy * inverseDistance * forceScalar;
}
