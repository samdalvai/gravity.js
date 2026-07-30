# Using Gravity.js in another project

Gravity.js is a fixed-timestep 2D rigid-body physics engine. It owns simulation state, collision detection, constraint solving, joints, and force helpers. It does not create a canvas, render bodies, load assets, or process input; those responsibilities stay in the application using the engine.

This guide documents the package API exported by `src/index.ts` and shows how to integrate it into a browser project.

## Contents

- [Install the package](#install-the-package)
- [Quick start](#quick-start)
- [Simulation model and units](#simulation-model-and-units)
- [Run the simulation](#run-the-simulation)
- [Create bodies](#create-bodies)
- [Render bodies](#render-bodies)
- [World API](#world-api)
- [RigidBody API](#rigidbody-api)
- [Collision filtering and contacts](#collision-filtering-and-contacts)
- [Joints](#joints)
- [Forces](#forces)
- [Shapes](#shapes)
- [Vectors and utilities](#vectors-and-utilities)
- [Global settings and constants](#global-settings-and-constants)
- [Common integration pitfalls](#common-integration-pitfalls)
- [Package exports](#package-exports)

## Install the package

The package name is `gravity.js`. The build emits CommonJS JavaScript and TypeScript declarations into `lib/`.

### Install from a local checkout

Build and pack the engine first:

```sh
cd /path/to/gravity.js
npm ci
npm run build:package
npm pack
```

Then install the generated archive in the other project:

```sh
cd /path/to/your-project
npm install /path/to/gravity.js/gravity.js-1.0.0.tgz
```

Rebuild and repack after changing engine source. For local development, installing the built directory also works:

```sh
npm install /path/to/gravity.js
```

The `lib/` directory must exist before installing because it is generated and is the only code directory included in the package.

### Import the API

TypeScript and projects using a bundler can use named imports:

```ts
import { BodiesFactory, GRAVITY, Vec2, World } from 'gravity.js';
import type { ContactInfo, RigidBody } from 'gravity.js';
```

CommonJS projects can use `require`:

```js
const { BodiesFactory, GRAVITY, Vec2, World } = require('gravity.js');
```

No browser global or script-tag build is provided. Use a bundler such as Parcel, Vite, webpack, or another tool capable of consuming CommonJS packages.

## Quick start

This example creates a world with downward gravity, a static floor, and a falling box. `World.update()` advances exactly one fixed simulation tick each time it is called.

```ts
import { BodiesFactory, FIXED_DELTA_TIME, GRAVITY, World } from 'gravity.js';

const world = new World(GRAVITY);

const floor = BodiesFactory.box({
    width: 800,
    height: 40,
    x: 0,
    y: -250,
    mass: 0, // Zero mass makes a body static.
    friction: 0.8,
});

const box = BodiesFactory.box({
    width: 50,
    height: 50,
    x: 0,
    y: 150,
    mass: 1,
    restitution: 0.2,
    friction: 0.7,
});

world.addBody(floor);
world.addBody(box);

let previousTime = performance.now();
let accumulator = 0;

function frame(now: number): void {
    let frameTime = (now - previousTime) / 1000;
    previousTime = now;

    // Do not try to simulate an unbounded delay after a suspended tab or breakpoint.
    frameTime = Math.min(frameTime, 0.25);
    accumulator += frameTime;

    while (accumulator >= FIXED_DELTA_TIME) {
        world.update();
        accumulator -= FIXED_DELTA_TIME;
    }

    render(world.getBodies());
    requestAnimationFrame(frame);
}

requestAnimationFrame(frame);
```

`render` is supplied by the host project. A basic Canvas implementation appears in [Render bodies](#render-bodies).

## Simulation model and units

Gravity.js uses these conventions:

| Quantity | Convention |
| --- | --- |
| Coordinates | Cartesian coordinates with positive Y pointing up |
| Length | Pixels/world units |
| Time | Seconds |
| Linear velocity | Pixels per second |
| Rotation | Radians, positive counter-clockwise |
| Angular velocity | Radians per second |
| Gravity passed to `World` | Positive acceleration magnitude; `new World(GRAVITY)` accelerates downward |
| Scale | `PIXELS_PER_METER` is `100` and is used by world gravity |
| Physics tick | `FIXED_DELTA_TIME`, currently `1 / 60` second |

The engine uses semi-implicit integration and expects a fixed-step loop. Rendering can happen at any display refresh rate, but physics should not be advanced with the display frame duration.

Canvas has positive Y pointing down by default. A typical renderer moves the origin and flips Y once before drawing world-space positions:

```ts
ctx.save();
ctx.translate(canvas.width / 2, canvas.height / 2);
ctx.scale(1, -1);

// Draw bodies here.

ctx.restore();
```

## Run the simulation

### Fixed timestep

Call `world.update()` once for each accumulated `FIXED_DELTA_TIME`. The method intentionally takes no frame-duration argument.

If the page becomes hidden, reset the last frame timestamp when it becomes visible again. This prevents a large catch-up step:

```ts
document.addEventListener('visibilitychange', () => {
    if (!document.hidden) previousTime = performance.now();
});
```

### Continuous forces and substeps

`SETTINGS.subSteps` divides one fixed tick into smaller solver steps. Use the optional callback to apply forces that must be present during every substep:

```ts
import { Force, SETTINGS } from 'gravity.js';

SETTINGS.subSteps = 4;

world.update(dt => {
    for (const body of world.getBodies()) {
        if (body.isStatic()) continue;
        body.addForce(Force.resistance.generateDragForce(body, 0.001, dt));
    }
});
```

The callback runs `SETTINGS.subSteps` times and receives the substep duration, `FIXED_DELTA_TIME / SETTINGS.subSteps`. A body's accumulated forces and torque are cleared after every substep. Applying a force only once before `world.update()` therefore affects only the first substep when substepping is enabled.

Impulses change velocity immediately and should normally be applied once for an event, outside the substep callback:

```ts
box.applyImpulseLinear(new Vec2(0, 300));
box.applyImpulseAngular(20);
```

### Pause and single-step

To pause, stop calling `world.update()` but continue rendering. To single-step, call it once. Avoid calling `world.update()` from both the render loop and another timer.

## Create bodies

`BodiesFactory` is the simplest and safest way to construct bodies. Every body must specify exactly one of `mass` or `density`.

- `mass: 0` or `density: 0` creates a static body.
- Positive mass or density creates a dynamic body.
- If density is supplied, mass is computed from shape area.
- If mass is supplied, density is computed from shape area.

### Factory methods

```ts
const box = BodiesFactory.box({
    width: 60,
    height: 30,
    x: -100,
    y: 100,
    mass: 2,
});

const circle = BodiesFactory.circle({
    radius: 20,
    x: 0,
    y: 100,
    density: 0.002,
});

const capsule = BodiesFactory.capsule({
    halfHeight: 30,
    radius: 12,
    x: 100,
    y: 100,
    mass: 1,
});

const polygon = BodiesFactory.polygon({
    vertices: [new Vec2(-30, -20), new Vec2(35, -15), new Vec2(0, 35)],
    x: 200,
    y: 100,
    mass: 1,
});

const segment = BodiesFactory.segment({
    length: 300,
    horizontal: true,
    x: 0,
    y: -100,
    mass: 0,
});

for (const body of [box, circle, capsule, polygon, segment]) {
    world.addBody(body);
}
```

The available geometry fields are:

| Factory | Required geometry | Notes |
| --- | --- | --- |
| `BodiesFactory.box(options)` | `width`, `height` | Centered rectangle |
| `BodiesFactory.circle(options)` | `radius` | The only shape supported by bullet CCD |
| `BodiesFactory.capsule(options)` | `halfHeight`, `radius` | Cap centers are `2 * halfHeight` apart; total extent along the local Y axis includes both radii |
| `BodiesFactory.polygon(options)` | `vertices` | Supply a valid convex polygon; winding is corrected and vertices are recentered |
| `BodiesFactory.segment(options)` | `length`, `horizontal` | Zero-thickness static segment; rotate the body for other angles |
| `BodiesFactory.fromShape(shape, options)` | A shape instance | Creates a body from any exported shape |

### Common body options

| Option | Default | Meaning |
| --- | ---: | --- |
| `x`, `y` | required | Initial center position in world units |
| `mass` or `density` | required | Choose exactly one; zero creates a static body |
| `rotation` | `0` | Initial angle in radians |
| `velocity` | `(0, 0)` | Initial velocity; the vector is copied |
| `angularVelocity` | `0` | Initial angular velocity in radians/second |
| `canRotate` | `true` | Set false for a body that must remain upright |
| `restitution` | `0.2` | Bounciness from `0` to `1` |
| `friction` | `0.7` | Contact friction from `0` to `1` |
| `rollingResistance` | `0.5` | Angular damping applied while grounded; must be non-negative |
| `gravityScale` | `1` | Multiplier for this body's world gravity |
| `surfaceSpeed` | `0` | Tangential contact surface speed, useful for conveyor behavior |
| `charge` | `0` | Charge used by Coulomb force helpers |
| `temperature` | `0` | Temperature used by heat/convection helpers |
| `isBullet` | `false` | Enables CCD; valid only for circles |
| `collisionCategory` | `CollisionCategory.DEFAULT` | The category this body belongs to |
| `collisionMask` | `CollisionCategory.ALL` | Categories this body accepts |

The factory synchronizes the shape's transformed vertices and AABB after applying initial position and rotation.

### Construct a body directly

Direct construction is useful when the shape already exists:

```ts
import { CircleShape, RigidBody } from 'gravity.js';

// Fourth argument is mass; fifth is density. At least one must be defined.
const bodyByMass = new RigidBody(new CircleShape(20), 0, 100, 1);
const bodyByDensity = new RigidBody(new CircleShape(20), 0, 100, undefined, 0.001);
```

Prefer `BodiesFactory.fromShape` if you also need material and motion options.

### Teleport or rotate a body at runtime

Position, rotation, transformed vertices, and the AABB are separate pieces of state. If an application directly changes `position` or `rotation`, synchronize the shape before the next physics update:

```ts
function setTransform(body: RigidBody, position: Vec2, rotation: number): void {
    body.position.assign(position);
    body.rotation = rotation;
    body.shape.updateVertices(body.rotation, body.position);
    body.shape.updateAABB(body);
}
```

Without this synchronization, broad-phase collision checks and rendering based on transformed vertices can use stale data for a step. Ordinary simulation movement updates these caches automatically.

## Render bodies

Gravity.js is renderer-agnostic. Read body and shape state after stepping and draw it with Canvas, WebGL, a game framework, or a DOM renderer.

This compact Canvas renderer handles all built-in shapes. It assumes the context is already transformed into Y-up world coordinates:

```ts
import { CapsuleShape, CircleShape, PolygonShape, SegmentShape } from 'gravity.js';
import type { RigidBody } from 'gravity.js';

function drawBody(ctx: CanvasRenderingContext2D, body: RigidBody): void {
    const shape = body.shape;

    ctx.beginPath();

    if (shape instanceof CircleShape) {
        ctx.arc(body.position.x, body.position.y, shape.radius, 0, Math.PI * 2);
        ctx.fill();
        return;
    }

    if (shape instanceof CapsuleShape) {
        ctx.save();
        ctx.lineCap = 'round';
        ctx.lineWidth = shape.radius * 2;
        ctx.moveTo(shape.worldCenter1.x, shape.worldCenter1.y);
        ctx.lineTo(shape.worldCenter2.x, shape.worldCenter2.y);
        ctx.stroke();
        ctx.restore();
        return;
    }

    if (shape instanceof PolygonShape) {
        const vertices = shape.worldVertices;
        if (vertices.length === 0) return;

        ctx.moveTo(vertices[0].x, vertices[0].y);
        for (let i = 1; i < vertices.length; i++) {
            ctx.lineTo(vertices[i].x, vertices[i].y);
        }

        if (shape instanceof SegmentShape) {
            ctx.stroke();
        } else {
            ctx.closePath();
            ctx.fill();
        }
    }
}
```

Useful rendering state includes:

- `body.position` and `body.rotation` for sprites.
- `PolygonShape.worldVertices` for boxes, polygons, and segments.
- `CapsuleShape.worldCenter1`, `worldCenter2`, and `radius` for capsules.
- `body.minX`, `minY`, `maxX`, and `maxY` for debug AABBs.
- `world.getManifolds()` for debug contact points and normals.

The `demo/` directory in this repository is a complete Canvas integration and can be used as a larger reference, but it is not part of the published package.

## World API

Create a world with a positive downward gravity magnitude:

```ts
const world = new World(GRAVITY); // 9.8 m/s² downward, scaled by PIXELS_PER_METER
const space = new World(0);       // no automatic weight force
```

| Method | Behavior |
| --- | --- |
| `addBody(body)` | Adds a body. Throws after `MAX_BODIES` bodies. |
| `removeBody(body)` | Removes the body with the matching ID. Does not automatically remove attached joints. |
| `getBodies()` | Returns a readonly view of the current body array. Do not mutate it by casting. |
| `addJoint(joint)` | Adds a distance, weld, or grab joint. |
| `removeJoint(joint)` | Removes the joint with the matching ID. |
| `getJoints()` | Returns current joints. Concrete joints expose `bodyA` and `bodyB`. |
| `getManifolds()` | Returns contacts produced by the most recent substep. Intended primarily for inspection/debug rendering. |
| `addForce(force)` | Registers a persistent uniform force applied to every body during every substep. |
| `addTorque(torque)` | Registers a persistent uniform torque applied to every body during every substep. |
| `update(callback?)` | Advances one `FIXED_DELTA_TIME`, divided into configured substeps. |
| `clear()` | Removes bodies, joints, contacts, persistent world forces, and persistent world torques. |

`World.addForce` and `World.addTorque` are persistent world-level fields, not one-frame operations. There are no individual remove methods; use body forces for changing effects, or `world.clear()` when resetting the entire scene.

When removing a body, remove joints that reference it first:

```ts
for (const joint of [...world.getJoints()]) {
    if (joint.bodyA.id === body.id || joint.bodyB.id === body.id) {
        world.removeJoint(joint);
    }
}
world.removeBody(body);
```

## RigidBody API

### State and material properties

| Property | Access | Description |
| --- | --- | --- |
| `id` | readonly | Unique numeric ID |
| `position`, `velocity` | read/write | Mutable `Vec2` values |
| `rotation`, `angularVelocity`, `angularAcceleration` | read/write | Angular state in radians |
| `canRotate` | read/write | Whether velocity integration may rotate the body |
| `mass`, `density` | read/write | Changing either recomputes inverse mass and inertia |
| `invMass`, `I`, `invI` | readonly | Inverse mass, moment of inertia, and inverse inertia |
| `restitution`, `friction` | read/write | Values are asserted to be in `[0, 1]` |
| `rollingResistance` | read/write | Non-negative grounded angular damping coefficient |
| `charge`, `surfaceSpeed`, `temperature`, `gravityScale` | read/write | Specialized material properties |
| `isGrounded` | read/write | Recomputed from contact normals every substep |
| `lastGroundedTime` | read/write | Seconds since the last grounded substep; useful for jump grace periods |
| `isBullet` | read/write | Circle-only CCD flag |
| `shape`, `shapeType` | readonly | Geometry and its `ShapeType` |
| `collisionCategory`, `collisionMask` | read/write | Collision filtering bit fields |
| `minX`, `maxX`, `minY`, `maxY` | read/write | Cached broad-phase AABB |
| `onContact` | optional callback | Called for every solved contact involving the body |

### Forces, torque, and impulses

| Method | Use |
| --- | --- |
| `addForce(force)` | Accumulate a force for the current substep |
| `addForceY(value)` | Allocation-free Y force helper |
| `addTorque(torque)` | Accumulate torque for the current substep |
| `addForceAtPoint(force, worldPoint)` | Add linear force plus the torque caused by a world-space application point |
| `clearForces()` / `clearTorque()` | Clear accumulated values before integration |
| `applyImpulseLinear(impulse)` | Immediately change linear velocity |
| `applyImpulseAngular(impulse)` | Immediately change angular velocity |
| `applyImpulseAtPoint(impulse, r)` | Apply an impulse with offset `r` from the center; `r` is not a world position |

The public `integrateForces(dt)` and `integrateVelocities(dt)` methods are used internally by `World`. Applications should normally call `world.update()` rather than integrating individual bodies.

### Coordinate and query helpers

```ts
const worldPoint = body.localPointToWorld(new Vec2(10, 0));
const localPoint = body.worldPointToLocal(pointerWorldPosition);
const localDirection = body.worldDirToLocal(new Vec2(1, 0));
const hit = body.isPointInside(pointerWorldPosition);
```

`isPointInside` is useful for picking bodies before creating a `GrabJoint`.

## Collision filtering and contacts

### Filtering

Each body has one category bit field and one mask bit field. A pair collides only when both bodies opt in:

```text
(bodyA.mask includes bodyB.category)
AND
(bodyB.mask includes bodyA.category)
```

Available categories are `NONE`, `DEFAULT`, `PROJECTILE`, `PARTICLE`, `SENSOR`, `LAYER1`, `LAYER2`, `LAYER3`, and `ALL`.

```ts
const player = BodiesFactory.capsule({
    halfHeight: 20,
    radius: 10,
    x: 0,
    y: 0,
    mass: 1,
    collisionCategory: CollisionCategory.LAYER1,
    collisionMask: CollisionCategory.DEFAULT | CollisionCategory.PROJECTILE,
});

const projectile = BodiesFactory.circle({
    radius: 4,
    x: -100,
    y: 0,
    mass: 0.1,
    collisionCategory: CollisionCategory.PROJECTILE,
    collisionMask: CollisionCategory.DEFAULT | CollisionCategory.LAYER1,
});
```

`CollisionCategory.SENSOR` is a category name, not automatic sensor behavior. All accepted pairs receive normal collision response. Setting `collisionMask: CollisionCategory.NONE` disables both response and contact callbacks because the pair is rejected before narrow-phase collision detection.

### Contact callbacks

Assign `onContact` to either body:

```ts
import type { ContactInfo } from 'gravity.js';

box.onContact = (info: ContactInfo) => {
    const other = info.bodyA.id === box.id ? info.bodyB : info.bodyA;
    console.log('Contact with', other.id, 'normal impulse', info.impulseSum);
};
```

`ContactInfo` contains:

| Field | Meaning |
| --- | --- |
| `bodyA`, `bodyB` | The two bodies in the manifold; callback ownership does not determine their order |
| `impulseSum` | Sum of accumulated normal impulses across manifold contact points |

Callbacks run after constraint solving for every active manifold and every substep. They are continuous contact callbacks, not distinct begin/end events. With multiple substeps they can run more than once during a single `world.update()`.

To inspect current contacts, use `world.getManifolds()`. Each manifold exposes `bodyA`, `bodyB`, `points`, `normal`, `numContacts`, `penetrationDepth`, and `persistent`, along with lower-level solver fields. Manifolds are pooled and reused, so do not retain them as long-lived application state; copy the specific values you need.

## Joints

Add constructed joints to the world and explicitly remove them when no longer needed.

Joint tuning parameters share these meanings:

- `frequency > 0` creates a soft, spring-like constraint. Higher values are stiffer.
- `frequency <= 0` creates a solid constraint.
- `dampingRatio` is clamped between `0` and `1`; `1` is critically damped.
- `jointMass <= 0` selects the mass of the available dynamic body automatically.
- At least one connected body must be dynamic.

### DistanceJoint

```ts
const joint = new DistanceJoint(
    bodyA,
    bodyB,
    bodyA.position, // world-space anchor A
    bodyB.position, // world-space anchor B
    -1,             // <= 0 uses the initial anchor distance
    15,             // frequency
    1,              // damping ratio
    -1,             // automatic joint mass
);
world.addJoint(joint);
```

Signature:

```ts
new DistanceJoint(bodyA, bodyB, anchorA?, anchorB?, length?, frequency?, dampingRatio?, jointMass?)
```

The world anchors are converted to `localAnchorA` and `localAnchorB` at construction.

### WeldJoint

`WeldJoint` maintains a shared anchor and the bodies' initial angular offset:

```ts
const anchor = bodyA.position.lerp(bodyB.position, 0.5);
const weld = new WeldJoint(bodyA, bodyB, anchor);
world.addJoint(weld);
```

Signature:

```ts
new WeldJoint(bodyA, bodyB, anchor?, frequency?, dampingRatio?, jointMass?)
```

The default frequency is `-1`, which makes the weld solid.

### GrabJoint

`GrabJoint` pulls a point on one dynamic body toward a moving world-space target:

```ts
const grab = new GrabJoint(body, pointerPosition, pointerPosition);
world.addJoint(grab);

// On pointer movement:
grab.setTarget(pointerPosition);

// On pointer release:
world.removeJoint(grab);
```

Signature:

```ts
new GrabJoint(body, anchor, target, frequency?, dampingRatio?, jointMass?)
```

The default tuning is frequency `0.8` and damping ratio `0.6`. `setTarget` copies coordinates into the joint, so the caller may reuse its vector.

All concrete joints also expose inherited `id`, `bodyA`, `bodyB`, `drawAnchor`, and `drawConnectionLine`. The two draw properties are metadata used by the repository demo; Gravity.js itself does not render them.

## Forces

Force helpers are grouped under the `Force` namespace. Generators return a force or impulse without applying it; helpers whose names begin with `apply` mutate bodies directly.

Apply continuously generated forces inside the `world.update(dt => { ... })` callback so they are regenerated for every substep.

### Gravity

```ts
Force.gravity.generateWeightForce(body, gravity): Vec2
Force.gravity.applyWeightForce(body, gravity): void
Force.gravity.generateGravitationalForce(a, b, G, minDistanceSquared, maxDistanceSquared): Vec2
Force.gravity.applyGravitationalForces(bodies, G, minDistanceSquared, maxDistanceSquared): void
Force.gravity.applyBarnesHutGravitationalForces(bodies, G, theta?, epsilon?): void
```

World gravity already applies weight automatically when `SETTINGS.applyGravity` is true. The other helpers model attraction between bodies.

The low-level weight helpers take a signed Y acceleration, unlike the positive magnitude accepted by the `World` constructor. Use `-GRAVITY` for downward weight when calling them directly.

`applyGravitationalForces` is an exact all-pairs operation and scales quadratically. The Barnes-Hut version uses a quadtree and is better for large body counts, at the cost of approximation. Its defaults are `theta = 0.5` and `epsilon = 1`.

The exact force generator clamps squared distance to the supplied minimum and maximum. This avoids singular forces at short range and negligible forces at very long range:

```ts
const attraction = Force.gravity.generateGravitationalForce(
    satellite,
    planet,
    GRAVITY,
    80 * 80,
    950 * 950,
);
satellite.addForce(attraction);
```

### Drag and friction

```ts
Force.resistance.generateDragForce(body, k, dt): Vec2
Force.resistance.generateFrictionForce(body, k): Vec2
```

Drag is opposite velocity and proportional to speed squared. It is clamped using `dt` so it cannot reverse the body's velocity in one step.

### Springs

```ts
Force.spring.generateSpringForceBodyAnchor(body, anchor, restLength, k): Vec2
Force.spring.generateSpringForceBodyBody(a, b, restLength, k): Vec2
```

For a body-to-body spring, apply equal and opposite forces:

```ts
const spring = Force.spring.generateSpringForceBodyBody(a, b, 100, 20);
a.addForce(spring);
b.addForce(spring.negateNew());
```

Use a `DistanceJoint` when the spring also needs stable constraint solving and damping.

### Explosions and charge

```ts
Force.interactions.generateExplosionForce(body, source, radius, strength): Vec2
Force.interactions.generateCoulombForce(a, b, k): Vec2
Force.interactions.applyCoulombForces(bodies, k): void
Force.interactions.applyBarnesHutCoulombForces(bodies, k, theta?, epsilon?): void
```

The explosion generator returns an impulse-like vector with linear distance falloff. Apply it once with `applyImpulseLinear`, not continuously with `addForce`:

```ts
for (const body of world.getBodies()) {
    const impulse = Force.interactions.generateExplosionForce(body, source, 250, 5000);
    body.applyImpulseLinear(impulse);
}
```

Coulomb helpers use each body's `charge`. Like charges repel and opposite charges attract. The exact helper is quadratic; the Barnes-Hut version is approximate and defaults to `theta = 0.5`, `epsilon = 0.01`.

### Buoyancy

```ts
Force.buoyancy.generateBuoyancyForce(body, liquidSurfaceY, liquidDensity, gravity)
Force.buoyancy.generateLinearWaterDragForce(body, submergedArea, liquidDensity, dragCoefficient, dt)
Force.buoyancy.generateAngularWaterDragTorque(body, submergedArea, liquidDensity, coefficient)
Force.buoyancy.applyBuoyancyForces(body, liquidSurfaceY, liquidDensity, gravity, linearDrag?, angularDrag?): void
```

The high-level helper applies buoyancy at the submerged centroid plus linear and angular water drag:

```ts
world.update(() => {
    for (const body of world.getBodies()) {
        if (!body.isStatic()) {
            Force.buoyancy.applyBuoyancyForces(body, 0, 0.001, GRAVITY, 1, 1);
        }
    }
});
```

`liquidSurfaceY` is the top Y coordinate of the liquid; material below that line is treated as submerged. The low-level buoyancy generator returns `null` when the body is not submerged, otherwise:

```ts
{
    submergedArea: number;
    force: Vec2;
    applicationPoint: Vec2;
}
```

### Temperature and convection

```ts
Force.temperature.generateConvectionForce(body, ambientTemperature, strength, minDifference?): Vec2
Force.temperature.exchangeHeat(a, b, dt, k?, minTemperature?, maxTemperature?): void
Force.temperature.dissipateHeat(body, ambientTemperature, dt, cooling?): void
```

Temperature is application-defined scalar state. `exchangeHeat` modifies both bodies, `dissipateHeat` cools a dynamic body toward ambient temperature, and `generateConvectionForce` returns an upward force when the body is sufficiently warmer than ambient.

## Shapes

Shapes can be created directly or through `BodiesFactory`:

```ts
new BoxShape(width, height)
new CircleShape(radius)
new CapsuleShape(halfHeight, radius)
new PolygonShape(vertices)
new SegmentShape(length, horizontal)
```

Every shape exposes:

```ts
getType(): ShapeType
getMomentOfInertia(): number // shape factor; RigidBody multiplies it by mass
getArea(): number
getPerimeter(): number
updateVertices(angle: number, position: Vec2): void
updateAABB(body: RigidBody): void
isPointInside(body: RigidBody, point: Vec2): boolean
```

`ShapeType` values are `CIRCLE`, `POLYGON`, `BOX`, `CAPSULE`, and `SEGMENT`.

Polygon collision assumes convex geometry. Concave outlines should be decomposed into multiple convex bodies. Do not supply self-intersecting polygons or repeated adjacent vertices. The polygon constructor corrects clockwise winding and recenters vertices around their bounding-box center.

Segments have zero thickness and must be static. Use a thin box when a dynamic, two-sided piece of geometry is required.

## Vectors and utilities

### Vec2

`Vec2` fields are mutable:

```ts
const v = new Vec2(3, 4);
v.x = 10;
v.y = 20;
```

`add`, `sub`, `scale`, `normalize`, `negate`, and the `*Assign` methods mutate the receiver. The `*New` methods, along with `rotate`, `lerp`, the perpendicular/normal helpers, `unitVector`, and `crossScalar`, return vectors without changing the receiver.

| Category | Methods |
| --- | --- |
| Copy/compare | `copy`, `assign`, `equals`, `notEquals` |
| Mutating arithmetic | `add`, `sub`, `scale`, `normalize`, `addAssign`, `subAssign`, `scaleAssign`, `divAssign`, `negate` |
| Allocating arithmetic | `addNew`, `subNew`, `scaleNew`, `divNew`, `negateNew`, `normalizeNew` |
| Geometry | `rotate`, `lerp`, `leftPerpNew`, `perpNew`, `normal`, `unitVector`, `crossScalar` |
| Products/length | `dot`, `cross`, `magnitude`, `magnitudeSquared`, `distanceSquared` |

`normalizeNew()` returns the original vector itself when its length is zero, while nonzero input returns a new vector. Do not mutate its result if aliasing a zero vector would be a problem; `unitVector()` always returns a distinct vector.

### Utils

The `Utils` namespace exports:

```ts
Utils.randomNumber(min?, max?): number
Utils.randomColor(): string
Utils.clamp(value, low, high): number
Utils.assert(...conditionsAndMessages): void
Utils.randomConvexBody(x, y, radius, numVertices?, mass?): RigidBody
Utils.pairKey(a, b): number
Utils.makeId(a, b): number
Utils.temperatureToColor(temperature, minTemperature, maxTemperature): string
```

`pairKey` and `makeId` are mainly useful for engine-style caches. `randomConvexBody` is convenient for prototypes and test scenes.

`randomNumber` defaults to the range `[1, 10)`. When using `randomConvexBody`, pass `numVertices >= 3`; although the declaration marks that argument optional, the current default is rejected by the implementation.

## Global settings and constants

`SETTINGS` is a shared mutable object. A change affects every `World` in the same JavaScript module instance.

### Common settings

| Setting | Default | Effect |
| --- | ---: | --- |
| `applyGravity` | `true` | Apply each world's gravity to its bodies |
| `ccd` | `true` | Run circle bullet continuous collision detection |
| `solverIterations` | `10` | Constraint solver passes per substep; more can improve stability at added cost |
| `subSteps` | `1` | Solver/integration subdivisions per fixed tick |
| `positionCorrection` | `true` | Correct positional constraint error |
| `warmStarting` | `true` | Reuse impulses from persistent contacts |
| `impulseAccumulation` | `true` | Accumulate contact impulses during solving |
| `blockSolve` | `true` | Jointly solve two-point normal contacts |

Set positive integer values for `solverIterations` and `subSteps`; the object itself does not validate application assignments.

### Advanced solver settings

These are public for engine tuning but should normally remain at their defaults:

| Setting | Default |
| --- | ---: |
| `applyWarmStartingThreshold` | `true` |
| `warmStartingThreshold` | `0.25` |
| `contactMergeThreshold` | `0.000025` |
| `penetrationSlop` | `0.5` |
| `restitutionSlop` | `50` |
| `angularVelocitySlop` | `0.05` |
| `positionCorrectionBeta` | `0.2` |
| `contactSlop` | `0.01` |

`SETTINGS.dt` is a readonly computed value equal to `FIXED_DELTA_TIME / subSteps`; `SETTINGS.invDt` is its reciprocal.

### Exported constants

| Constant | Value/meaning |
| --- | --- |
| `FIXED_DELTA_TIME` | `1 / 60` second |
| `PIXELS_PER_METER` | `100` |
| `MAX_BODIES` | `5,000` |
| `GRAVITY` | `9.8` |
| `MIN_BULLET_SPEED_SQUARED` | `1,000,000` `(pixels/second)²` |

Bullet CCD is deliberately limited:

- Only circle bodies can set `isBullet = true`.
- CCD is more expensive than discrete collision detection.
- A bullet flag is automatically cleared when speed squared is at or below `MIN_BULLET_SPEED_SQUARED`.
- CCD must also be enabled globally with `SETTINGS.ccd`.

## Common integration pitfalls

- Calling `world.update()` once per display frame makes simulation speed depend on monitor refresh rate. Use a fixed-step accumulator.
- Passing seconds or meters as positions while rendering in pixels creates an unintended scale mismatch. Positions and dimensions are pixel/world units; only world gravity performs the built-in meter-to-pixel conversion.
- Applying a continuous body force only once before an update misses later substeps. Regenerate it in the `world.update` callback.
- Treating `World.addForce` as a one-frame force causes it to persist. Use `RigidBody.addForce` for changing per-body effects.
- Directly changing `position` or `rotation` without refreshing vertices and the AABB leaves collision caches stale.
- Removing a body does not remove joints attached to it. Remove those joints explicitly.
- Removing a body while iterating the live `getBodies()` array can skip the swapped-in body. Iterate over a copy when removing multiple bodies: `for (const body of [...world.getBodies()])`.
- A category named `SENSOR` does not disable physical response. Sensor-only overlap events are not a separate engine feature.
- Contact callbacks run for active contacts, potentially once per substep, rather than only when contact begins.
- A polygon must be convex even though clockwise winding is repaired automatically.
- Dynamic segments and non-circle bullets are rejected by assertions.
- `SETTINGS` is global, so two worlds cannot have independent solver settings without coordinating changes.
- Forces and impulses are different: a force is integrated over a substep; an impulse changes velocity immediately.

## Package exports

The root `gravity.js` entry point exports:

| Group | Exports |
| --- | --- |
| Simulation | `World`, `RigidBody`, `BodiesFactory` |
| Math | `Vec2`, `Utils` |
| Shapes | `BoxShape`, `CapsuleShape`, `CircleShape`, `PolygonShape`, `SegmentShape`, `ShapeType` |
| Joints | `DistanceJoint`, `WeldJoint`, `GrabJoint` |
| Collision | `CollisionCategory`, TypeScript type `ContactInfo` |
| Forces | `Force` namespace |
| Configuration | `SETTINGS`, `FIXED_DELTA_TIME`, `PIXELS_PER_METER`, `MAX_BODIES`, `GRAVITY`, `MIN_BULLET_SPEED_SQUARED` |

Import from the package root instead of internal `lib/...` paths. Only the root entry point is declared in `package.json` exports.
