/*
 * Portions of this file are derived from the Sopiro Physics Engine.
 *
 * Copyright (c) 2022 Sopiro
 * Licensed under the MIT License
 *
 * Original project:
 * https://github.com/Sopiro
 */
import * as CCD from '../collision/CCD';
import { canCollide } from '../collision/CollisionFilter';
import { ContactManifold } from '../collision/ContactManifold';
import { ContactSolver } from '../collision/ContactSolver';
import { DynamicTree } from '../collision/DynamicTree';
import * as NarrowPhase from '../collision/NarrowPhase';
import { PairCache } from '../collision/PairCache';
import { applyWeightForce } from '../force/Gravity';
import { Joint } from '../joint/Joint';
import { Vec2 } from '../math/Vec2';
import { MIN_BULLET_SPEED_SQUARED, PIXELS_PER_METER, SETTINGS } from './Constants';
import { RigidBody } from './RigidBody';
import { SolverBodies } from './SolverBodies';

const DEFAULT_MAX_BODIES = 5_000;

/** Broad phase AABBs are fattened by this margin so that small movements don't require a tree update */
const AABB_MARGIN = 0.05 * PIXELS_PER_METER;

/** Broad phase AABBs are also extended along the displacement predicted for this many steps */
const AABB_DISPLACEMENT_MULTIPLIER = 4;

/** A potentially colliding pair, alive while the fattened AABBs of its bodies overlap */
class Contact {
    /** Null while the bodies are not touching */
    manifold: ContactManifold | null = null;

    constructor(
        readonly bodyA: RigidBody,
        readonly bodyB: RigidBody,
    ) {}
}

export interface WorldOptions {
    /** Maximum live bodies. Defaults to 5,000; Infinity disables the limit. */
    maxBodies?: number;
}

export class World {
    readonly maxBodies: number;
    private readonly up = new Vec2(0, 1);
    private G: number;

    private bodies: RigidBody[] = [];

    private joints: Joint[] = [];

    /** Broad phase tree, its proxies are the bodies of this world */
    private readonly tree = new DynamicTree<RigidBody>();
    /** Proxies whose fattened AABB changed since the last pair update */
    private moveBuffer: number[] = [];
    private readonly queryResults: number[] = [];

    /** Persistent contacts, created when fattened AABBs start overlapping and destroyed when they stop */
    private contacts: Contact[] = [];
    /** Contacts by body id pair */
    private readonly contactMap = new PairCache<Contact>();

    /** Manifolds of the touching contacts of the last step */
    private manifolds: ContactManifold[] = [];

    private readonly manifoldPool = NarrowPhase.manifoldPool;

    private forces: Vec2[] = [];
    private torques: number[] = [];

    private dtFractions: number[] = [];

    private readonly solverBodies = new SolverBodies();
    private readonly contactSolver = new ContactSolver();

    constructor(gravity: number, options: WorldOptions = {}) {
        const maxBodies = options.maxBodies ?? DEFAULT_MAX_BODIES;
        if (maxBodies !== Infinity && (!Number.isSafeInteger(maxBodies) || maxBodies < 0)) {
            throw new RangeError('maxBodies must be a non-negative safe integer or Infinity');
        }

        this.maxBodies = maxBodies;
        this.G = -gravity;
    }

    addBody(body: RigidBody): void {
        if (this.bodies.length >= this.maxBodies) throw new Error('Max number of bodies exceeded');

        this.bodies.push(body);

        body.proxyStatic = body.isStatic();
        body.proxyId = this.tree.createProxy(
            body.minX - AABB_MARGIN,
            body.minY - AABB_MARGIN,
            body.maxX + AABB_MARGIN,
            body.maxY + AABB_MARGIN,
            body,
        );
        this.moveBuffer.push(body.proxyId);
    }

    removeBody(body: RigidBody): void {
        for (let i = 0; i < this.bodies.length; i++) {
            const current = this.bodies[i];

            // It suffices to look for the position going below the screen
            if (body.id === current.id) {
                this.bodies[i] = this.bodies[this.bodies.length - 1];
                this.bodies.pop();

                // Contacts of the body are destroyed by the next contact update
                this.tree.destroyProxy(current.proxyId);
                current.proxyId = -1;
                return;
            }
        }
    }

    getBodies(): readonly RigidBody[] {
        return this.bodies;
    }

    getManifolds(): ContactManifold[] {
        return this.manifolds;
    }

    addJoint(joint: Joint): void {
        this.joints.push(joint);
    }

    removeJoint(joint: Joint): void {
        for (let i = 0; i < this.joints.length; i++) {
            const current = this.joints[i];

            if (joint.id === current.id) {
                this.joints[i] = this.joints[this.joints.length - 1];
                this.joints.pop();
                return;
            }
        }
    }

    getJoints(): Joint[] {
        return this.joints;
    }

    addForce(force: Vec2): void {
        this.forces.push(force);
    }

    addTorque(torque: number): void {
        this.torques.push(torque);
    }

    update(beforeSubStep?: (dt: number) => void): void {
        const settings = SETTINGS;
        const dt = settings.dt;
        const subSteps = settings.subSteps;

        const bodies = this.bodies;

        for (let i = 0; i < subSteps; i++) {
            beforeSubStep?.(dt);

            // Loop all bodies of the world applying forces
            for (let i = 0; i < bodies.length; i++) {
                const body = bodies[i];
                if (SETTINGS.applyGravity) {
                    // Apply the weight force to all bodies
                    applyWeightForce(body, this.G * body.gravityScale);
                }

                // Apply forces to all bodies
                const forces = this.forces;
                for (let j = 0; j < forces.length; j++) {
                    body.addForce(forces[j]);
                }

                // Apply torque to all bodiesx
                const torques = this.torques;
                for (let j = 0; j < torques.length; j++) {
                    body.addTorque(torques[j]);
                }

                // Update last grounded time
                if (body.isGrounded) {
                    body.lastGroundedTime = 0;
                } else {
                    // Since we have fixed dt we can safely assume that the last frame dt is the same as this one
                    body.lastGroundedTime += dt;
                }

                // Reset grounded value at the beginning of each frame
                body.isGrounded = false;
            }

            // Integrate all the forces
            for (let i = 0; i < bodies.length; i++) {
                const body = bodies[i];
                body.integrateForces(dt);
            }

            if (SETTINGS.ccd) {
                this.ccd(dt);

                for (let i = 0; i < this.dtFractions.length; i++) {
                    const dtFraction = this.dtFractions[i];
                    this.step(dtFraction);
                }
            } else {
                this.step(dt);
            }
        }
    }

    private ccd(dt: number) {
        const fractions = this.dtFractions;
        fractions.length = 0;

        const bodies = this.bodies;

        for (let i = 0; i < bodies.length; i++) {
            const body = bodies[i];

            if (!body.isBullet) continue;

            if (body.velocity.magnitudeSquared() <= MIN_BULLET_SPEED_SQUARED) {
                body.isBullet = false;
                continue;
            }

            const fraction = CCD.resolveCCD(body, bodies, dt);

            if (fraction != null) {
                fractions[fractions.length] = fraction;
            }
        }

        fractions[fractions.length] = 1;
        fractions.sort((a, b) => a - b);

        let previous = 0;

        // Convert fractions to dt slices, the sum of the slices will be equal to dt
        for (let i = 0; i < fractions.length; i++) {
            const current = fractions[i] * dt;
            fractions[i] = current - previous;
            previous = current;
        }
    }

    private step(dt: number) {
        const bodies = this.bodies;
        // Bias terms always use the full sub-step: CCD slices can be arbitrarily small and
        // 1 / slice would turn position correction into huge velocities
        const invDt = dt === 0 ? 0 : SETTINGS.invDt;

        this.updatePairs();

        this.narrowPhase();

        this.solveConstraints(invDt);

        // Integrate all the velocities
        if (dt !== 0) {
            for (let i = 0; i < bodies.length; i++) {
                const body = bodies[i];
                body.integrateVelocities(dt);
            }
        }

        this.synchronizeProxies(dt);
    }

    /** Updates the fattened AABBs of the bodies that left them */
    private synchronizeProxies(dt: number) {
        const bodies = this.bodies;
        const tree = this.tree;
        const moveBuffer = this.moveBuffer;
        const predictedTime = dt * AABB_DISPLACEMENT_MULTIPLIER;

        for (let i = 0; i < bodies.length; i++) {
            const body = bodies[i];
            const proxyId = body.proxyId;
            const isStatic = body.isStatic();

            const minX = body.minX;
            const minY = body.minY;
            const maxX = body.maxX;
            const maxY = body.maxY;

            // Static bodies never search for pairs, a body becoming dynamic must search again
            const staticChanged = isStatic !== body.proxyStatic;
            body.proxyStatic = isStatic;

            if (!staticChanged && tree.contains(proxyId, minX, minY, maxX, maxY)) {
                continue;
            }

            // Fatten the AABB and extend it along the predicted displacement
            const displacementX = body.velocity.x * predictedTime;
            const displacementY = body.velocity.y * predictedTime;

            tree.moveProxy(
                proxyId,
                minX - AABB_MARGIN + Math.min(displacementX, 0),
                minY - AABB_MARGIN + Math.min(displacementY, 0),
                maxX + AABB_MARGIN + Math.max(displacementX, 0),
                maxY + AABB_MARGIN + Math.max(displacementY, 0),
            );
            moveBuffer.push(proxyId);
        }
    }

    /** Creates contacts for the new overlaps of the proxies that moved */
    private updatePairs() {
        const tree = this.tree;
        const moveBuffer = this.moveBuffer;
        const results = this.queryResults;
        const contactMap = this.contactMap;

        for (let i = 0; i < moveBuffer.length; i++) {
            const proxyId = moveBuffer[i];

            // Destroyed proxies and duplicates in the buffer are not flagged
            if (!tree.wasMoved(proxyId)) continue;

            const body = tree.getUserData(proxyId);
            const bodyStatic = body.isStatic();
            const count = tree.query(
                tree.getFatMinX(proxyId),
                tree.getFatMinY(proxyId),
                tree.getFatMaxX(proxyId),
                tree.getFatMaxY(proxyId),
                results,
            );

            for (let j = 0; j < count; j++) {
                const otherId = results[j];

                if (otherId === proxyId) continue;

                // When both proxies moved, the pair is created by the query of the lower id
                if (otherId < proxyId && tree.wasMoved(otherId)) continue;

                const other = tree.getUserData(otherId);

                if (bodyStatic && other.isStatic()) continue;

                // Improve coherence
                const a = body.id < other.id ? body : other;
                const b = body.id < other.id ? other : body;

                if (contactMap.get(a.id, b.id) !== undefined) continue;

                const contact = new Contact(a, b);
                contactMap.set(a.id, b.id, contact);
                this.contacts.push(contact);
            }

            tree.clearMoved(proxyId);
        }

        moveBuffer.length = 0;
    }

    /** Destroys contacts that stopped overlapping and computes the manifolds of the touching ones */
    private narrowPhase() {
        const contacts = this.contacts;
        const manifolds = this.manifolds;
        const tree = this.tree;
        const pool = this.manifoldPool;
        const warmStarting = SETTINGS.warmStarting;

        manifolds.length = 0;

        let i = 0;
        while (i < contacts.length) {
            const contact = contacts[i];
            const a = contact.bodyA;
            const b = contact.bodyB;

            // Removed bodies have no proxy
            if (a.proxyId === -1 || b.proxyId === -1 || !tree.overlaps(a.proxyId, b.proxyId)) {
                this.destroyContact(i);
                continue;
            }

            i++;

            const oldManifold = contact.manifold;

            // Only pairs with overlapping AABBs can collide, filters and static flags may change at any time
            const overlapping = !(a.maxX < b.minX || a.minX > b.maxX || a.maxY < b.minY || a.minY > b.maxY);
            const newManifold =
                overlapping && !(a.isStatic() && b.isStatic()) && canCollide(a, b)
                    ? NarrowPhase.detectCollision(a, b)
                    : null;

            if (oldManifold !== null) {
                if (newManifold !== null && warmStarting) {
                    newManifold.tryWarmStart(oldManifold);
                }

                pool.release(oldManifold);
            }

            contact.manifold = newManifold;

            if (newManifold !== null) {
                manifolds.push(newManifold);
                this.setGrounded(newManifold);
            }
        }
    }

    private destroyContact(index: number) {
        const contacts = this.contacts;
        const contact = contacts[index];

        if (contact.manifold !== null) {
            this.manifoldPool.release(contact.manifold);
        }

        this.contactMap.remove(contact.bodyA.id, contact.bodyB.id);

        contacts[index] = contacts[contacts.length - 1];
        contacts.pop();
    }

    private solveConstraints(invDt: number) {
        const manifolds = this.manifolds;
        const joints = this.joints;
        const solverBodies = this.solverBodies;

        // Stage body velocities in a packed array for the solver, joints may also reference bodies
        // that are not part of the world
        solverBodies.begin();

        const bodies = this.bodies;
        for (let i = 0; i < bodies.length; i++) solverBodies.add(bodies[i]);

        for (let i = 0; i < joints.length; i++) {
            solverBodies.add(joints[i].bodyA);
            solverBodies.add(joints[i].bodyB);
        }

        // Read after staging, the array grows while bodies are added
        const v = solverBodies.velocities;

        const contactSolver = this.contactSolver;

        // Presolve constraints
        contactSolver.prepare(manifolds, invDt, v);

        for (let i = 0; i < joints.length; i++) joints[i].preSolve(invDt, v);

        // Solve constraints
        const iterations = SETTINGS.solverIterations;
        for (let i = 0; i < iterations; i++) {
            contactSolver.solve(v);

            for (let j = 0; j < joints.length; j++) joints[j].solve(v);
        }

        contactSolver.storeImpulses(manifolds);
        solverBodies.end();

        // Run contact callbacks
        for (let i = 0; i < this.manifolds.length; i++) {
            const manifold = this.manifolds[i];
            const bodyA = manifold.bodyA;
            const bodyB = manifold.bodyB;

            if (bodyA.onContact) {
                bodyA.onContact(manifold.contactInfo);
            }

            if (bodyB.onContact) {
                bodyB.onContact(manifold.contactInfo);
            }
        }
    }

    private setGrounded(manifold: ContactManifold) {
        const bodyA = manifold.bodyA;
        const bodyB = manifold.bodyB;
        const normalY = manifold.contactNormalY;

        if (!bodyA.isStatic() && normalY < -0.5) bodyA.isGrounded = true;
        if (!bodyB.isStatic() && normalY > 0.5) bodyB.isGrounded = true;
    }

    clear() {
        for (let i = 0; i < this.contacts.length; i++) {
            const manifold = this.contacts[i].manifold;
            if (manifold !== null) this.manifoldPool.release(manifold);
        }

        for (let i = 0; i < this.bodies.length; i++) {
            this.bodies[i].proxyId = -1;
        }

        this.bodies.length = 0;
        this.contacts.length = 0;
        this.contactMap.clear();
        this.tree.clear();
        this.moveBuffer.length = 0;
        this.manifolds.length = 0;
        this.joints.length = 0;
        this.forces.length = 0;
        this.torques.length = 0;
    }
}
