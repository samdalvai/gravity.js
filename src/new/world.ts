import { AABB, containsAABB, createAABB, toRigidBody } from './aabb';
import { AABBTree } from './aabbtree';
import { ContactManifold } from './contact';
import { GrabJoint } from './grab';
import { Joint } from './joint';
import { detectCollision, testPointInside } from './detection';
import { Island } from './island';
import { Vector2 } from './vector2';
import { RigidBody, Type } from './rigidbody';
import { Settings } from './settings';
import * as Util from './util';

type Registrable = RigidBody | Joint;

export class World {
    private uid = 0;

    // Dynamic AABB Tree for broad phase collision detection
    public tree: AABBTree = new AABBTree();
    // All registered rigid bodies
    public bodies: RigidBody[] = [];

    // Constraints to be solved
    public manifolds: ContactManifold[] = [];
    public joints: Joint[] = [];

    public manifoldMap: Map<number, ContactManifold> = new Map();
    public jointMap: Map<number, Joint> = new Map();
    public passTestSet: Set<number> = new Set();

    public numIslands: number = 0;
    public sleepingIslands: number = 0;
    public sleepingBodies: number = 0;

    public forceIntegration: boolean = false;

    update(deltaTime: number): void {
        const inverseDeltaTime = deltaTime > 0.0 ? 1.0 / deltaTime : 0.0;
        const newManifolds: ContactManifold[] = [];
        const newManifoldMap: Map<number, ContactManifold> = new Map();

        // // Update the AABB tree dynamically
        // for (let i = 0; i < this.bodies.length; i++) {
        //     const b = this.bodies[i];
        //     b.manifoldIDs = [];

        //     if (b.sleeping) continue;

        //     const node = b.node!;
        //     const tightAABB = createAABB(b, 0.0);

        //     if (containsAABB(node.aabb, tightAABB)) continue;

        //     this.tree.remove(node);
        //     this.tree.add(b);
        // }

        // REVERSE: disable efficient broad phase
        // // Broad Phase
        // // Retrieve a list of collider pairs that are potentially colliding
        // // let pairs = getCollisionPairsNSquared(this.bodies);
        // const pairs = this.tree.getCollisionPairs();

        // for (let i = 0; i < pairs.length; i++) {
        //     const pair = pairs[i];
        //     let a = pair.p1;
        //     let b = pair.p2;

        //     // Improve coherence
        //     if (a.id > b.id) {
        //         a = pair.p2;
        //         b = pair.p1;
        //     }

        //     if (a.type == Type.Static && b.type == Type.Static) continue;

        //     const key = Util.make_pair_natural(a.id, b.id);
        //     if (this.passTestSet.has(key)) continue;

        //     // Narrow Phase
        //     // Execute more accurate and expensive collision detection
        //     const newManifold = detectCollision(a, b);
        //     if (newManifold == null) continue;

        //     a.manifoldIDs.push(key);
        //     b.manifoldIDs.push(key);

        //     if (Settings.warmStarting && this.manifoldMap.has(key)) {
        //         const oldManifold = this.manifoldMap.get(key)!;
        //         newManifold.tryWarmStart(oldManifold);
        //     }

        //     newManifoldMap.set(key, newManifold);
        //     newManifolds.push(newManifold);
        // }

        // REVERSE: Check collisions with brute force algorithm
        for (let i = 0; i < this.bodies.length - 1; i++) {
            for (let j = i + 1; j < this.bodies.length; j++) {
                let a = this.bodies[i];
                let b = this.bodies[j];

                // Improve coherence
                if (a.id > b.id) {
                    a = this.bodies[j];
                    b = this.bodies[i];
                }

                if (a.type == Type.Static && b.type == Type.Static) continue;

                const key = Util.make_pair_natural(a.id, b.id);
                if (this.passTestSet.has(key)) continue;

                const newManifold = detectCollision(a, b);
                if (newManifold == null) continue;

                // REVERSE: reset sleeping if a contact is new
                const isNewContact = !this.manifoldMap.has(key);

                if (isNewContact) {
                    a.awake();
                    b.awake();
                }

                a.manifoldIDs.push(key);
                b.manifoldIDs.push(key);

                if (Settings.warmStarting && this.manifoldMap.has(key)) {
                    const oldManifold = this.manifoldMap.get(key)!;
                    newManifold.tryWarmStart(oldManifold);
                }

                newManifoldMap.set(key, newManifold);
                newManifolds.push(newManifold);
            }
        }

        this.manifoldMap = newManifoldMap;
        this.manifolds = newManifolds;

        // REVERSE: disable islands
        // // Build the constraint island
        // const island = new Island(this);
        // let restingBodies = 0;
        // let islandID = 0;
        // this.sleepingIslands = 0;
        // this.sleepingBodies = 0;

        // const visited: Set<number> = new Set();
        // let stack: RigidBody[] = [];

        // // Perform a DFS(Depth First Search) on the constraint graph
        // // After building island, each island can be solved in parallel because they are independent of each other
        // for (let i = 0; i < this.bodies.length; i++) {
        //     const b = this.bodies[i];

        //     if (visited.has(b.id) || b.type == Type.Static) continue;

        //     stack = [];
        //     stack.push(b);

        //     islandID++;
        //     while (stack.length > 0) {
        //         const t = stack.pop()!;
        //         if (visited.has(t.id) || t.type == Type.Static) continue;

        //         visited.add(t.id);
        //         t.islandID = islandID;
        //         island.addBody(t);

        //         for (let m = 0; m < t.manifoldIDs.length; m++) {
        //             const key = t.manifoldIDs[m];
        //             const manifold = this.manifoldMap.get(key)!;

        //             const other = manifold.bodyB.id == t.id ? manifold.bodyA : manifold.bodyB;

        //             if (visited.has(other.id)) continue;

        //             island.addManifold(manifold);
        //             stack.push(other);
        //         }

        //         for (let j = 0; j < t.jointIDs.length; j++) {
        //             const key = t.jointIDs[j];
        //             const joint = this.jointMap.get(key)!;

        //             const other = joint.bodyB.id == t.id ? joint.bodyA : joint.bodyB;

        //             if (joint instanceof GrabJoint) {
        //                 island.addJoint(joint);
        //                 t.awake();
        //             }

        //             if (visited.has(other.id)) continue;

        //             island.addJoint(joint);
        //             stack.push(other);
        //         }

        //         if (t.resting > Settings.sleepingWait) restingBodies++;
        //     }

        //     island.sleeping = Settings.sleepEnabled && restingBodies == island.numBodies;

        //     if (island.sleeping) {
        //         this.sleepingBodies += island.numBodies;
        //         this.sleepingIslands++;
        //     }

        //     island.solve(deltaTime, inverseDeltaTime);
        //     island.clear();
        //     restingBodies = 0;
        // }

        // this.numIslands = islandID;

        // REVERSE: use island code directly
        // Integrate forces, yield tentative velocities that possibly violate the constraint
        for (let i = 0; i < this.bodies.length; i++) {
            const b = this.bodies[i];

            if (b.sleeping) {
                b.linearVelocity.clear();
                b.angularVelocity = 0;
            }

            // if (this.forceIntegration) {
            //     const linear_a = b.force.mulNew(b.inverseMass * deltaTime); // Force / mass * dt
            //     b.linearVelocity.x += linear_a.x;
            //     b.linearVelocity.y += linear_a.y;

            //     const angular_a = b.torque * b.inverseInertia * deltaTime; // Torque / inertia * dt
            //     b.angularVelocity += angular_a;

            //     if (
            //         (this.sleeping && linear_a.squaredLength >= Settings.restLinearTolerance) ||
            //         angular_a * angular_a >= Settings.restAngularTolerance
            //     ) {
            //         this.sleeping = false;
            //         awakeIsland = true;
            //     }
            // }

            if (
                b.linearVelocity.squaredLength < Settings.restLinearTolerance &&
                b.angularVelocity * b.angularVelocity < Settings.restAngularTolerance
            ) {
                b.resting += deltaTime;
            } else {
                b.resting = 0;
            }

            if (b.resting > Settings.sleepingWait) {
                b.sleeping = true;
            }

            // Apply gravity
            if (Settings.applyGravity && !b.sleeping) {
                const gravity = new Vector2(0, Settings.gravity * Settings.gravityScale * deltaTime);
                b.linearVelocity.x += gravity.x;
                b.linearVelocity.y += gravity.y;
            }
        }

        // If island is sleeping, skip the extra computation
        // if (this.sleeping) return;

        // Prepare for solving
        for (let i = 0; i < this.manifolds.length; i++) this.manifolds[i].prepare(inverseDeltaTime);

        for (let i = 0; i < this.joints.length; i++) this.joints[i].prepare(inverseDeltaTime);

        // Iteratively solve the violated velocity constraint
        for (let i = 0; i < Settings.numIterations; i++) {
            for (let j = 0; j < this.manifolds.length; j++) this.manifolds[j].solve();

            for (let j = 0; j < this.joints.length; j++) this.joints[j].solve();
        }

        // REVERSE: this additional loop is used for events/callbacks
        // for (let i = 0; i < this.manifolds.length; i++) {
        //     const manifold = this.manifolds[i];
        //     manifold.solve();

        //     // Contact callbacks
        //     // if (manifold.bodyA.onContact != undefined) {
        //     //     const contactInfo = manifold.getContactInfo(false);
        //     //     if (manifold.bodyA.onContact(contactInfo)) manifold.bodyA.onContact = undefined;
        //     // }

        //     // if (manifold.bodyB.onContact != undefined) {
        //     //     const contactInfo = manifold.getContactInfo(true);
        //     //     if (manifold.bodyB.onContact(contactInfo)) manifold.bodyB.onContact = undefined;
        //     // }
        // }

        // for (let i = 0; i < this.joints.length; i++) this.joints[i].solve();

        // Update positions using corrected velocities (Semi-implicit euler integration)
        for (let i = 0; i < this.bodies.length; i++) {
            const b = this.bodies[i];

            // if (awakeIsland) b.awake();

            b.force.clear();
            b.torque = 0;

            b.position.x += b.linearVelocity.x * deltaTime;
            b.position.y += b.linearVelocity.y * deltaTime;
            b.rotation += b.angularVelocity * deltaTime;

            if (b.position.y < Settings.deadBottom) this.unregister(b.id);
        }
    }

    register(r: Registrable, passTest: boolean = false): void {
        r.id = this.uid++;

        if (r instanceof RigidBody) {
            this.bodies.push(r);
            this.tree.add(r);
        } else if (r instanceof Joint) {
            if (r.bodyA.id == -1 || r.bodyB.id == -1)
                throw 'You should register the rigid bodies before registering the joint';

            if (passTest) this.addPassTestPair(r.bodyA, r.bodyB);

            r.bodyA.jointIDs.push(r.id);
            if (r.bodyA.id != r.bodyB.id)
                // Extra handle for grab joint
                r.bodyB.jointIDs.push(r.id);

            this.jointMap.set(r.id, r);
            this.joints = Array.from(this.jointMap.values());
        }
    }

    unregister(id: number, isJoint: boolean = false): boolean {
        if (isJoint) {
            const joint = this.jointMap.get(id);
            if (joint == undefined) return false;

            for (let i = 0; i < joint.bodyA.jointIDs.length; i++) {
                if (id == joint.bodyA.jointIDs[i]) {
                    joint.bodyA.jointIDs.splice(i, 1);
                    break;
                }
            }

            for (let i = 0; i < joint.bodyB.jointIDs.length; i++) {
                if (id == joint.bodyB.jointIDs[i]) {
                    joint.bodyB.jointIDs.splice(i, 1);
                    break;
                }
            }

            this.jointMap.delete(id);
            this.removePassTestPair(joint.bodyA, joint.bodyB);
            this.joints = Array.from(this.jointMap.values());

            return true;
        }

        for (let i = 0; i < this.bodies.length; i++) {
            const b = this.bodies[i];
            if (b.id != id) continue;

            this.bodies.splice(i, 1);
            this.tree.remove(b.node!);

            for (let m = 0; m < b.manifoldIDs.length; m++) {
                const manifold = this.manifoldMap.get(b.manifoldIDs[m])!;

                manifold.bodyA.awake();
                manifold.bodyB.awake();
            }

            for (let j = 0; j < b.jointIDs.length; j++) {
                const jid = b.jointIDs[j];

                const joint = this.jointMap.get(jid)!;
                const other = joint.bodyA.id == id ? joint.bodyB : joint.bodyA;

                other.awake();

                for (let k = 0; k < other.jointIDs.length; k++) {
                    if (other.jointIDs[k] == jid) {
                        other.jointIDs.splice(k, 1);
                        break;
                    }
                }

                this.jointMap.delete(jid);
            }

            this.joints = Array.from(this.jointMap.values());
            return true;
        }

        return false;
    }

    queryPoint(point: Vector2): RigidBody[] {
        const res: RigidBody[] = [];
        const nodes = this.tree.queryPoint(point);

        for (let i = 0; i < nodes.length; i++) {
            const b = nodes[i].body!;

            if (testPointInside(b, point)) {
                res.push(b);
                break;
            }
        }

        return res;
    }

    queryRegion(region: AABB): RigidBody[] {
        const res: RigidBody[] = [];
        const nodes = this.tree.queryRegion(region);

        for (let i = 0; i < nodes.length; i++) {
            const node = nodes[i];

            if (containsAABB(region, node.aabb)) {
                res.push(node.body!);
                continue;
            }

            if (detectCollision(toRigidBody(region), node.body!) != null) {
                res.push(node.body!);
            }
        }

        return res;
    }

    addPassTestPair(bodyA: RigidBody, bodyB: RigidBody) {
        this.passTestSet.add(Util.make_pair_natural(bodyA.id, bodyB.id));
        this.passTestSet.add(Util.make_pair_natural(bodyB.id, bodyA.id));
    }

    removePassTestPair(bodyA: RigidBody, bodyB: RigidBody) {
        this.passTestSet.delete(Util.make_pair_natural(bodyA.id, bodyB.id));
        this.passTestSet.delete(Util.make_pair_natural(bodyB.id, bodyA.id));
    }

    reset(): void {
        this.tree.reset();
        this.bodies = [];
        this.joints = [];
        this.manifolds = [];
        this.passTestSet.clear();
        this.manifoldMap.clear();
        this.jointMap.clear();
        this.uid = 0;
    }

    surprise(): void {
        for (let i = 0; i < this.bodies.length; i++) {
            const b = this.bodies[i];
            b.awake();
        }
    }

    get numBodies(): number {
        return this.bodies.length;
    }

    get numJoints(): number {
        return this.jointMap.size;
    }
}
