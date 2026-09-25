# Physics engine robustness and porting report

## Scope

This report compares the main engine in `src/` with the checked-in reference engine in
`reference/box2d.ts/src/`. It focuses on changes that improve robustness without accepting a
general performance regression.

The main engine is not a primitive solver. It already has useful features worth preserving:

- allocation-conscious scalar contact data and manifold pooling;
- accumulated impulses and warm starting;
- a two-contact normal block solver;
- analytic circle, capsule, segment, and polygon manifolds;
- insertion-sort sweep-and-prune broad phase;
- fixed substeps and limited circle CCD.

The reference engine's biggest advantages are persistent per-point contact state, speculative
contacts, soft position correction using predicted motion, generic shape TOI, sleeping islands,
and an incremental broad phase. The best result is therefore a selective port, not replacement of
the main engine with the reference architecture.

## Executive recommendation

Implement the work in this order:

1. Fix the current numerical hazards and solver ordering.
2. Replace the manifold's shared penetration/persistence state with per-contact-point state.
3. Add speculative contacts and the reference soft-contact equation, initially inside the current
   world loop.
4. Introduce persistent contacts and move collision generation outside solver substeps.
5. Replace global circle-only CCD slicing with candidate-pruned, shape-generic TOI.
6. Add island sleeping, then benchmark whether an incremental dynamic tree beats the current SAP
   for the engine's real workloads.

Do not port graph coloring, solver-set storage, or the reference engine's entire handle/ID system.
Those primarily support parallelism and a broader API; they add complexity without improving this
single-threaded solver's physical result.

## Architecture comparison

| Area | Main engine | Reference engine | Consequence |
|---|---|---|---|
| Contact lifetime | Rebuilds and pools touching manifolds every substep | Keeps contacts while broad-phase proxies overlap | Main engine repeats narrow phase and only carries impulses across frames |
| Manifold point data | World point, feature ID, impulse; one shared penetration and `persistent` flag | Two fixed points with separate anchors, separation, impulses, impact velocity, and persisted flag | Main solver cannot correct each point independently and can misclassify a newly added point |
| Position correction | Baumgarte bias injected into velocity iterations | Soft constraint computed from current predicted separation, with push-speed cap | Main solver can add energy and over-correct deep overlap |
| Solver schedule | Detect, warm start, run 10 velocity iterations, integrate position | Integrate velocity, warm start, biased solve, integrate predicted position, unbiased/restitution solve per substep | Reference separates overlap recovery from restitution and friction |
| Two-point normal solve | 2-by-2 complementarity block solve | Scalar point solve | Main feature is valuable but currently has an unguarded singular matrix |
| Broad phase | Full 1D SAP pass every substep | Separate dynamic trees, moved-proxy queries, persistent pair set | SAP is excellent for small/coherent scenes; trees scale better for large sparse worlds |
| Narrow phase | Mostly world-space SAT/clipping and analytic routines | Relative-transform local manifolds, GJK simplex cache where useful | Reference has better temporal/geometric coherence and large-coordinate behavior |
| CCD | Circle bullets, all-body scan, global timestep slicing | Motion-based fast-body selection, swept tree query, generic TOI | Main CCD cost grows as bullets times bodies and misses rotation/general shapes |
| Sleeping | None | Whole-island sleeping and wake propagation | Main engine continues integrating and solving settled scenes |
| Geometry input | Winding correction and AABB-center recentering | Hull welding/validation and centroid/mass data | Main irregular polygons can have an incorrect center of mass and inertia |

## Priority matrix

Scores are relative: effectiveness is robustness gained, performance is expected steady-state
effect (`++` large gain, `+` gain, `0` neutral, `-` cost), and effort includes tests and migration.

| Priority | Improvement | Effectiveness | Performance | Effort | Recommendation |
|---|---|---:|:---:|:---:|---|
| P0 | Per-point separation, anchors, and persistence | 5/5 | 0 | M | Implement first; it is the foundation for all solver improvements |
| P0 | Guard ill-conditioned block solves and solve normals before friction | 5/5 | 0/+ | S | Immediate hardening with almost no runtime cost |
| P0 | Fix degenerate segment distance and clipping tolerances | 4/5 | 0 | S | Prevent NaN/false-manifold edge cases |
| P0 | Soft speculative contact solver with push-speed cap | 5/5 | 0/+ | M/L | Main stability upgrade; can permit fewer iterations |
| P0 | Correct polygon centroid and mass properties; validate hulls | 4/5 | 0 | M | Fixes physically wrong torque for asymmetric polygons |
| P1 | Persistent contact objects and cached manifolds | 4/5 | + | M/L | Reduces narrow-phase work and makes warm starting reliable |
| P1 | Generic, candidate-pruned TOI/shape cast | 5/5 for fast bodies | + overall | L | Replaces expensive global slicing and supports every convex shape |
| P1 | Island sleeping and wake propagation | 3/5 | ++ | L | Largest performance win in resting scenes |
| P1 | One-sided chain segments with ghost vertices | 4/5 for terrain | 0 | M | Eliminates internal-edge snagging and back-face contacts |
| P2 | Incremental dynamic-tree broad phase | 2/5 robustness | +/++ at scale | L | Port only after workload benchmarks justify it |
| P2 | Material mixing and impulse-based rolling resistance | 3/5 | 0 | S/M | Better friction/bounce consistency and resting rotation |
| P2 | Velocity damping and translation/rotation caps | 3/5 | 0 | S | Cheap protection against numerical explosions |
| P3 | Manifold recycling without rerunning narrow phase | 2/5 | + | M | Add only after persistent contacts are proven correct |
| Do not port | Constraint graph coloring and parallel scheduling | 0 in current runtime | - | L | No benefit until the engine has actual parallel workers |
| Do not port | Full solver-set/opaque-handle architecture | 0 for physics | - | XL | Unrelated to the requested robustness goal |

## P0: immediate numerical and solver hardening

### 1. Make the two-point block solver conditional

Current location: `src/collision/ContactManifold.ts`, `preSolveBlock()`.

The current code asserts only that the determinant is not exactly zero. Nearly coincident contact
points can produce a very small positive determinant, a huge inverse matrix, and explosive
impulses. An exact floating-point comparison does not protect against this.

Add a `blockSolveReady` boolean and use a condition-number guard:

```ts
const determinant = k00 * k11 - k01 * k01;
const maxConditionNumber = 1000;

blockSolveReady =
    determinant > solverEpsilon &&
    k00 * k00 < maxConditionNumber * determinant;
```

If the guard fails, retain both manifold points but solve them with the scalar normal solver. Do not
throw, invert the matrix, or delete a point. This preserves the main engine's useful block solver
while adopting the reference solver's safe scalar behavior as the fallback.

Also validate all generated manifold values in development builds: finite unit normal, finite
points/separations, `pointCount` in `[1, 2]`, and distinct IDs for two-point manifolds.

### 2. Solve normal impulses before friction

Current `ContactManifold.solve()` solves tangents first. On the first iteration, the Coulomb limit
therefore uses the old or zero normal impulse. The reference solver updates normal impulses first
and friction second.

Use this order per iteration:

1. solve normal block, or scalar normal points when the block is unavailable;
2. solve tangent impulses using the newly accumulated normal impulse;
3. solve rolling resistance, once it exists.

This improves friction convergence and may allow a lower iteration count. Keep warm starting as a
separate pre-solve operation.

### 3. Fix segment degeneracy and clipping rejection

Current location: `src/collision/NarrowPhase.ts`, `segmentDistance()` and `clipConvexEdges()`.

`segmentDistance()` currently checks `dd1 < 0 || dd2 < 0`, but squared segment lengths cannot be
negative. A zero-length segment reaches divisions by zero. Port the reference logic from
`reference/box2d.ts/src/distance.ts`:

```ts
const epsilonSquared = GEOMETRY_EPSILON * GEOMETRY_EPSILON;

if (dd1 < epsilonSquared || dd2 < epsilonSquared) {
    if (dd1 >= epsilonSquared) fraction1 = clamp(-rd1 / dd1, 0, 1);
    else if (dd2 >= epsilonSquared) fraction2 = clamp(rd2 / dd2, 0, 1);
} else {
    // Existing non-degenerate closest-segment calculation.
}
```

In `clipConvexEdges()`, port these reference guards:

- return no manifold when the incident and reference tangential intervals do not overlap
  (`upper2 < 0 || upper1 < lower2`);
- compare the clip denominator against an epsilon, not `> 0`;
- use a small reference-face hysteresis (for example `0.1 * linearSlop`) when two axes have almost
  equal separation, so the manifold normal does not alternate each frame;
- compute the clipped manifold and the closest vertex manifold, then choose the vertex result only
  when it is meaningfully closer. The reference uses `vertexSeparation + 0.1 * linearSlop <
  clippedMinSeparation`.

These are small changes with high value for nearly parallel capsules and polygons touching at
corners.

## P0: manifold data model

### Problem in the current structure

`createCollisionManifold()` already receives a separation for each contact, but it reduces them to
one maximum `penetrationDepth`. Both points then receive the same Baumgarte bias. A box resting with
one corner deeper than the other is therefore solved as if both corners had the deepest overlap,
which can inject angular error.

Persistence is also manifold-wide. If point 0 persists but point 1 is new, `persistent` becomes
true and restitution is suppressed for both points. The reference stores this state per point.

### Target representation

Keep the current allocation-conscious layout. Either use two point objects allocated once with the
pooled manifold, or continue with scalar `point0...`/`point1...` fields. Each point needs:

```ts
interface ContactPointState {
    // Geometry
    localAnchorAX: number;
    localAnchorAY: number;
    localAnchorBX: number;
    localAnchorBY: number;
    baseSeparation: number;
    id: number;

    // Persistence and impact state
    persisted: boolean;
    normalVelocity: number;
    restitutionVelocity: number;

    // Solver cache
    normalImpulse: number;
    tangentImpulse: number;
    totalNormalImpulse: number;
}
```

Porting rules:

- Store both input separations in `ContactManifold.init()`; do not reduce them for solving.
- Keep `penetrationDepth` as a compatibility getter returning
  `max(0, -min(active separations))`.
- Convert each generated world contact into an anchor relative to each body's center. For the
  later substep redesign, store anchors in body-local coordinates and rotate them into solver
  space using cached body sine/cosine values.
- In `tryWarmStart()`, match each new point by ID independently. Copy only that point's impulses,
  set only that point's `persisted`, and leave unmatched points at zero.
- Remove the world-distance warm-start threshold once local anchors and stable IDs are reliable.
  Until then, retain it as a secondary guard after the ID match.
- Preserve `normalImpulse`, `tangentImpulse`, and `rollingImpulse` between updates; clear transient
  fields such as `totalNormalImpulse` at the start of an outer step.
- Derive contact callbacks from the active point states, not pooled object identity.

Change `ContactManifoldPool.acquire()` and all narrow-phase constructors to pass the two
separations. No per-frame arrays or `Vec2` objects should be created in the solver hot path.

### Stable IDs

The polygon and capsule clipping code already uses packed feature IDs in many paths. Preserve the
reference/incident feature orientation exactly when a manifold flips. Add tests that rotate and
slide a polygon across an edge and assert that matching physical points retain IDs.

Single-point circle contacts can continue to use ID zero. Their pair identity and single point are
unambiguous. Do not invent coordinate hashes; they are less stable than feature IDs.

## P0: soft speculative contact solver

### Why this is preferable to stronger Baumgarte correction

The current bias is proportional to deepest penetration and inverse timestep. Deep overlap or a
small CCD slice can therefore request a very large separating velocity. The reference solver:

- creates contacts slightly before overlap;
- uses positive separation to prevent crossing during the next substep;
- treats penetration as a damped soft constraint;
- caps depenetration speed;
- applies restitution in a separate unbiased phase.

This gives better stacks and less jitter without requiring more iterations.

### Constants and units

Centralize length-based settings using `PIXELS_PER_METER`:

```ts
linearSlop = 0.005 * PIXELS_PER_METER;       // currently equivalent to 0.5 px
speculativeDistance = 4 * linearSlop;        // 2 px at the current scale
contactPushSpeed = 3 * PIXELS_PER_METER;     // maximum depenetration speed
restitutionThreshold = 1 * PIXELS_PER_METER;
contactHertz = 30;
contactDampingRatio = 10;
```

Expose these as world settings rather than mutable process-wide globals when practical. Keep the
old settings during migration and compare both solvers behind a feature flag.

### Softness calculation

Port `b2MakeSoft()` from the reference contact solver:

```ts
omega = 2 * PI * hertz;
a1 = 2 * dampingRatio + h * omega;
a2 = h * omega * a1;
a3 = 1 / (1 + a2);

biasRate = omega / a1;
massScale = a2 * a3;
impulseScale = a3;
```

Compute this once per solver substep, not per contact.

### Normal solve equation

For each point, calculate its current separation from its base separation and current/predicted
anchors. Then use the reference cases:

```ts
let bias = 0;
let massScale = 1;
let impulseScale = 0;

if (separation > 0) {
    // Speculative contact: permit closing only by the available gap.
    bias = separation * invH;
} else if (useBias) {
    bias = Math.max(
        softness.massScale * softness.biasRate * separation,
        -contactPushSpeed,
    );
    massScale = softness.massScale;
    impulseScale = softness.impulseScale;
}

let deltaImpulse =
    -normalMass * (massScale * normalVelocity + bias) -
    impulseScale * accumulatedNormalImpulse;

const nextImpulse = Math.max(accumulatedNormalImpulse + deltaImpulse, 0);
deltaImpulse = nextImpulse - accumulatedNormalImpulse;
```

Expand narrow-phase acceptance from `contactSlop` to `speculativeDistance`. A positive-separation
contact must use the speculative bias above; applying the current zero-bias impulse would stop the
body too early.

### Restitution and friction phase

Capture each point's approach velocity when the contact is updated. Set a restitution target only
when the point had a real normal impulse and its approach speed exceeded the threshold:

```ts
restitutionVelocity =
    previousTotalNormalImpulse > 0 &&
    previousNormalVelocity < -restitutionThreshold
        ? -combinedRestitution * previousNormalVelocity
        : 0;
```

Run an unbiased normal pass after predicted position integration. In that pass, include the
restitution target, then solve friction. This prevents restitution and penetration recovery from
adding energy through the same bias.

Initially, this can coexist with the current iteration count. After correctness is established,
benchmark fewer solver iterations plus 2-4 temporal substeps. The reference schedule does much
less repeated velocity iteration because each substep updates the geometric error.

## P0: polygon geometry and mass properties

### Current issue

`PolygonShape` recenters vertices around their AABB center. `RigidBody` then treats its `position`
as the center of mass for torque arms. For an asymmetric convex polygon, the AABB center and area
centroid differ. The resulting inertia and angular response are physically inconsistent.

### Porting instructions

At polygon construction time:

1. Reject non-finite coordinates.
2. Weld points closer than a scale-aware tolerance.
3. Build a convex hull, removing nearly collinear points. Port the reference hull algorithm from
   `reference/box2d.ts/src/hull.ts` or use an allocation-conscious monotone chain equivalent.
4. Reject fewer than three valid points and enforce a documented maximum vertex count.
5. Compute the signed-area centroid.
6. Subtract that centroid from local vertices so body position remains the center of mass.
7. Compute rotational inertia about the centroid using the reference polygon mass integration,
   not about an AABB-derived origin.

Port the reference capsule inertia as well; it accounts for the semicircle centroid offset. Circle
and box formulas can remain specialized.

This can change the placement of existing asymmetric polygons. Treat it as an API migration:
either expose the removed local centroid so callers can preserve the old world outline, or add a
temporary `preserveInputOrigin` option and deprecate it.

## P1: persistent contacts

### Target lifecycle

Introduce a persistent `Contact` record distinct from its current manifold:

```ts
interface Contact {
    bodyA: RigidBody;
    bodyB: RigidBody;
    manifold: ContactManifold;
    touching: boolean;
    simplexCache: SimplexCache;
}
```

Use this lifecycle:

1. The broad phase creates a contact when fat AABBs first overlap and the filters accept the pair.
2. Existing contacts are narrow-phase updated every outer step while their fat AABBs overlap.
3. A contact can exist with `pointCount === 0`; this avoids destroy/recreate churn near the
   boundary.
4. Swap old/new fixed manifold storage, match point IDs, and copy impulses in place.
5. Destroy and pool the contact only when fat AABBs separate, filtering changes, or a body is
   removed.
6. Track touching transitions separately for begin/end callbacks.

The main engine currently rebuilds collision pairs and manifolds for every solver substep. Once
contacts are persistent and speculative, perform broad phase and narrow phase once per fixed outer
step, prepare constraints once, then run solver substeps against updated anchor separations. This
is where the reference design improves robustness and recovers the cost of temporal substepping.

Do not immediately port contact recycling based on a small relative-pose change. First make the
persistent contact update correct. Recycling can later skip narrow phase when:

- both bodies rotate by less than the reference cosine threshold (`cos(deltaAngle) > 0.98`); and
- `translationDelta + maxExtent * abs(relativeAngleDelta)` is less than a recycle distance.

When recycling, recompute point separation from stored anchors; never reuse the old separation
unchanged.

## P1: CCD redesign

### Problems in the current CCD path

`src/collision/CCD.ts` scans every body for every bullet. `World.ccd()` collects all initial TOI
fractions, sorts them, and advances the entire world through every fraction. This has several
issues:

- cost is `O(bullets * bodies)` before repeated world collision passes;
- all bodies pay for one bullet's collision time;
- TOIs are computed from the start-of-step state and are not recomputed after earlier impacts;
- only circle bullets are supported;
- other-body translation is approximated but its rotation is ignored;
- collision filters are not applied in `resolveCCD()`;
- duplicate fractions can produce zero-duration solve steps;
- a slow bullet has its `isBullet` property permanently cleared.

The last two items should be fixed even before generic CCD: deduplicate fractions with an epsilon,
skip zero slices, apply `canCollide()`, and never mutate the user's bullet setting because current
speed is low.

### Reference-based replacement

Port these pieces from `distance.ts` and `solver.ts`:

1. A shape proxy for circle, capsule, segment, and polygon.
2. Cached GJK shape distance with witness points.
3. Separation-function TOI with bounded iterations.
4. Sweeps containing start/end center and rotation.
5. Motion-based fast-body selection:

```ts
maxMotion = linearSpeed * dt + abs(angularSpeed) * maxExtent * dt;
needsContinuous = maxMotion > safetyFactor * minExtent;
```

6. Query only the fast body's swept AABB against broad-phase candidates.
7. For ordinary fast bodies, test against static geometry. For explicit bullets, also test
   kinematic/dynamic bodies.
8. Clamp only that body's transform to the earliest safe TOI fraction, update its proxy, and let
   the next outer step create/solve the contact.

Use the reference's fallback for an initial overlap: retry with a small core proxy rather than
freezing the body at fraction zero. Keep strict iteration caps and return a classified result
(`hit`, `separated`, `overlapped`, `failed`) so failure is observable in tests.

This algorithm is individually more expensive than circle ray tests, but swept-AABB pruning and
removal of global timestep slicing should reduce total cost in scenes with few fast bodies.

## P1: sleeping and islands

Sleeping is the clearest way to improve robustness and performance together: settled stacks stop
accumulating numerical noise and stop consuming solver work.

An initial implementation does not need the reference engine's persistent solver sets. Build
awake islands using an iterative DFS or union-find over active contacts and joints:

1. Static bodies terminate traversal and do not join islands.
2. A dynamic body is sleep-eligible only when
   `linearSpeed + abs(angularSpeed) * maxExtent < sleepThreshold`.
3. Accumulate eligible time per body. Sleep an entire island only after every body has remained
   eligible for at least 0.5 seconds.
4. Sleeping sets velocity, angular velocity, force, and torque to zero and skips force
   integration, broad-phase movement, narrow phase, and solving.
5. Wake the full island when a force/torque/impulse is applied, a transform is externally changed,
   a joint changes, or an awake/fast body creates a touching contact.

Avoid recursive graph traversal because large stacks can exceed the JavaScript call stack. Reuse
scratch arrays and integer visit stamps to avoid per-frame `Set` allocation.

## P1/P2: broad phase

The current insertion-sort SAP is a good design for coherent, compact scenes. Do not replace it
based only on asymptotic complexity.

First instrument:

- insertion shifts;
- X-axis candidate count;
- final AABB pair count;
- narrow-phase calls;
- time spent in sort, pair generation, and narrow phase.

Port the reference dynamic tree only if large sparse or adversarial scenes show SAP pair generation
as a bottleneck. Use separate static and dynamic trees, fat AABBs, moved-proxy queries, and a
persistent pair set. Static-static pairs should never be queried. Extend moving proxies by a small
margin and predicted displacement so minor motion does not remove/reinsert leaves.

A practical hybrid is also valid:

- keep SAP below a measured body/candidate threshold;
- use a static tree plus SAP for dynamic bodies;
- or expose broad phase as a world strategy and benchmark both.

If SAP remains, sort a proxy array rather than `World.bodies` so `getBodies()` retains insertion
order. This also makes body storage independent of broad-phase implementation.

## P1: one-sided chain geometry

Independent zero-thickness segments produce ghost contacts where adjacent edges meet and accept
contacts from both sides. Port the reference chain-segment idea rather than trying to tune contact
slop:

- add previous/current/next vertices (`ghost1`, edge endpoints, `ghost2`);
- designate a front face;
- classify candidate normals as admit, skip, or snap based on adjacent edge convexity;
- suppress endpoint contacts owned by the neighboring segment;
- use a GJK simplex cache for segment-versus-polygon feature selection.

Keep the existing two-sided `SegmentShape` for standalone barriers if that behavior is part of the
public API. Add a separate `ChainShape` or explicit `oneSided` option.

## P2: material response and safety caps

### Material combination

The main manifold multiplies friction and restitution. Port the reference defaults:

```ts
combinedFriction = Math.sqrt(frictionA * frictionB);
combinedRestitution = Math.max(restitutionA, restitutionB);
```

Multiplication makes two `0.7` surfaces behave as `0.49`; geometric mean keeps identical materials
at their authored value. Maximum restitution lets a bouncy material remain bouncy against a dull
surface. Because this changes behavior, expose world callbacks for custom combination and document
the default change.

Replace grounded-body angular damping with an equal-and-opposite rolling impulse:

```ts
rollingMass = invIA + invIB > 0 ? 1 / (invIA + invIB) : 0;
limit = combinedRollingResistance * totalNormalImpulse;
rollingImpulse = clamp(
    oldRollingImpulse - rollingMass * (angularVelocityB - angularVelocityA),
    -limit,
    limit,
);
```

This responds to contact load and conserves pair angular momentum better than damping one body's
rotation after integration.

### Motion caps and damping

Use stable implicit damping (`v *= 1 / (1 + h * damping)`) and cap:

- maximum linear speed;
- angular displacement per outer step (the reference uses at most one quarter turn);
- penetration correction speed independently via `contactPushSpeed`.

Record a debug flag when a cap is hit. Silent caps make instability hard to diagnose.

## Detailed file-to-file port map

| Reference source | Main target | What to port |
|---|---|---|
| `contact.ts`: `b2ManifoldPoint`, `b2UpdateContact()` | `collision/ContactManifold.ts`, `core/World.ts` | Per-point state, ID matching, previous impact velocity, touching transitions |
| `contact_solver.ts`: `b2MakeSoft()`, constraint preparation and solve | `collision/ContactManifold.ts`, new `collision/ContactSolver.ts` if needed | Softness, predicted separation, speculative bias, separate biased/unbiased phases |
| `manifold.ts`: polygon clipping and capsule decisions | `collision/NarrowPhase.ts` | Clip rejection, axis hysteresis, closest-feature fallback, chain classification |
| `distance.ts`: segment distance, GJK, shape cast, TOI | `collision/NarrowPhase.ts`, `collision/CCD.ts` | Degenerate segment fix first; GJK/TOI for CCD and chains later |
| `solver.ts`: solve ordering and motion-based continuous selection | `core/World.ts`, `core/RigidBody.ts` | Temporal substep schedule, motion caps, fast-body selection, finalization |
| `hull.ts`, `geometry.ts` | `shapes/PolygonShape.ts`, `shapes/CapsuleShape.ts` | Hull sanitization, centroid and inertia |
| `island.ts` | new `core/Island.ts`, `core/World.ts` | Awake-component construction, sleep/wake propagation |
| `broad_phase.ts`, `dynamic_tree.ts` | new `collision/BroadPhase.ts`, optional `DynamicTree.ts` | Moved proxies, fat AABBs, persistent pair cache |
| `constants.ts` | `core/Constants.ts` | Scale-derived slop/speculation/margins and sleep time |

## Recommended implementation series

### Change 1: safety fixes and measurements — completed

- [x] Fix degenerate segment distance.
- [x] Add clipping interval/epsilon guards.
- [x] Guard block inversion and add scalar fallback.
- [x] Solve normals before tangents.
- [x] Add finite-value assertions in development builds.
- [x] Add benchmark timers and robustness metrics.

This change should be small and independently releasable.

### Change 2: manifold v2 — completed

- [x] Add per-point separation, anchors, persistence, and impact state.
- [x] Preserve the old public manifold getters.
- [x] Update pool initialization and all narrow-phase builders.
- [x] Add stable-ID and two-point asymmetric-penetration tests.

Do not change solver equations in the same change; first prove the new representation reproduces
current behavior.

### Change 3: speculative soft contacts

- Add scale-derived settings and `makeSoft()`.
- Accept near contacts up to speculative distance.
- Port the soft normal equation and push-speed cap.
- Separate restitution from penetration correction.
- Add impulse-based rolling resistance and material callbacks.
- Benchmark iteration counts; only lower the default after evidence.

### Change 4: persistent contacts and temporal substeps

- Separate contact lifetime from touching manifold state.
- Keep a pair cache and update contacts in place.
- Prepare contacts once per outer step.
- Update separation from local anchors during solver substeps.
- Move broad/narrow work out of each substep.
- Add contact begin/end transitions and robust body-removal cleanup.

Adapt joints to the same predicted-position schedule before making it the only solver path.

### Change 5: generic CCD

- Add support proxies and cached GJK distance.
- Add shape cast/TOI with iteration caps.
- Select fast bodies from motion and extent.
- Query swept broad-phase candidates.
- Remove global timestep-fraction slicing after parity tests pass.

### Change 6: sleep and broad-phase scaling

- Add island sleep/wake behavior.
- Benchmark SAP after sleeping, because sleep may remove enough work that a tree is unnecessary.
- Port the dynamic tree only for workloads where it wins.

### Change 7: geometry hardening

- Add hull welding/validation and centroid-correct mass data.
- Add one-sided chain shapes with ghost vertices.
- Ship polygon recentering as a documented API migration.

Geometry hardening can be developed in parallel, but the centroid behavior should be released
deliberately because it can move existing asymmetric shapes.

## Verification plan and performance gates

The current tests mostly check pair detection and one direct contact solve. Add deterministic
long-running scenarios before changing solver behavior:

| Scenario | Robustness metric | Performance metric |
|---|---|---|
| 20- and 50-box vertical stacks | max/RMS penetration, top-body drift, resting RMS speed | step median and p95 |
| Box pyramid and bridge | collapse count, joint error, NaN count | contacts solved per millisecond |
| Sliding box on flat and inclined planes | stopping distance and static creep | solver passes required |
| Restitution drops at several speeds | rebound-height error, no bounce below threshold | neutral |
| Parallel and crossing capsules | stable point count/normal, no impulse spikes | narrow-phase time |
| Polygon corner/edge transitions | stable IDs and angular-velocity continuity | narrow-phase time |
| Fast circle/capsule/thin polygon | tunneling count and TOI failures | candidate count and CCD time |
| 1,000/5,000 sparse and dense bodies | pair correctness | SAP versus tree median/p95 |
| Settled 1,000-body scene | no false sleep, deterministic wake | awake body/contact count and step time |

Suggested release gates:

- no NaN/Infinity in any body or contact state;
- no missed collision in the CCD regression set;
- no steady-state heap growth after pools reach capacity;
- no more than 5% median regression in fully awake dense scenes unless a robustness metric improves
  materially;
- a clear win in settled scenes after sleeping;
- dynamic tree enabled by default only if it beats SAP across the intended large-scene benchmark,
  not only one adversarial test.

For reproducibility, use seeded scene generation and record the settings with every benchmark.
Measure production builds; development assertions and source maps distort hot-loop timing.

## Final prioritization

If only three improvements are implemented, choose:

1. per-point separations/persistence plus guarded block solving;
2. soft speculative contacts with separate restitution;
3. persistent contacts with collision generation outside solver substeps.

Together these address the main engine's largest stability flaw, reduce contact churn, and create a
path to fewer solver iterations. Add island sleeping next for the largest performance gain, and
replace CCD before expanding bullet support beyond circles.
