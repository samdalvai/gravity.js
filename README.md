# Gravity.js

A 2D physics engine written in JavaScript and rendered in the browser using the HTML5 Canvas API.

The project is inspired by the C++ physics engine developed in the Pikuma Game Physics course, as well as engines such as Box2D-Lite and other open-source physics implementations referenced in the credits.

Learn more at [pikuma.com](https://pikuma.com/).

# Features

- Collision detection between different shapes: Circles, Boxes, Polygons, Segments and Capsules
- Broad Phase using prune & sweep algorithm with AABB partitioning
- Warm starting with contact caching
- Different types of joints
- Substepping to reduce collision tunneling
- Basic CCD for bullets with circle shape
- Texture rendering for shapes
- Set of demos showcasing different scenarios
- Generation of various forces: attraction, explosion, drag, friction, convection, buoyancy

# Documentation

See the [Gravity.js integration and API guide](docs/USAGE.md) for installation in another project, fixed-step simulation setup, body and joint creation, collision handling, forces, rendering integration, and the complete public API.

# Install as a library

Gravity.js can be installed directly from GitHub. Pin a release tag or commit SHA so that installs are reproducible:

```sh
npm install git+https://github.com/samdalvai/gravity.js.git#<tag-or-commit>
```

The `prepare` script compiles the TypeScript source into `lib/` during installation. Import the public API using the package name:

```ts
import { BodiesFactory, World } from 'gravity.js';
```

# Development

## Prerequisites

- Node.js installed

## Install dependencies

```sh
npm install
```

## Build the library

```sh
npm run build
```

This compiles the library and its TypeScript declarations into `lib/`.

## Build the demo

```sh
npm run build:demo
```

This builds the library first and then creates the production demo in `dist/`.

## Run the demo in development mode

```sh
npm start
```

This builds the Gravity.js package once, changes under `src` rebuild the package and refresh the demo automatically.

## Run the demo from the packaged version only

```sh
npm run start:package
```

This builds the package once and starts the Parcel development server without watching the engine source. Restart the command to include later changes under `src`.

For either command, open the URL printed by Parcel (usually http://localhost:1234).

# Example scenarios

The engine features a set of basic example scenarios with sprites based on the angry birds game.

![game](images/game.png)

In addition to the classic physics demos you can play around with some interesting simulations, for example these 5.000 circle particles orbiting around a gravitational field using the attraction force generation feature:

![gravity](images/gravity.png)

# App demo

A desktop live version of the app can be found at this [link](https://samdalvai.github.io/gravity.js/)

# References

- https://pikuma.com/courses/game-physics-engine-programming
- https://github.com/erincatto/box2d-lite
- https://github.com/erincatto/box2d
- https://github.com/Sopiro/Physics
- https://github.com/phaserjs/phaser-box2d
