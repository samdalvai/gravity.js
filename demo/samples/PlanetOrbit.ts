import { BodiesFactory, GRAVITY, SETTINGS, Utils, Vec2 } from '../../src';
import type { RigidBody, World } from '../../src';
import type Application from '../Application';
import Graphics from '../graphics/Graphics';
import { defineDemo } from './shared';

function setupPlanetOrbit(world: World, app: Application): void {
    app.setBackground('transparent');
    Graphics.zoom = 0.1;

    SETTINGS.applyGravity = false;

    const sun = BodiesFactory.circle({
        radius: 100,
        x: 0,
        y: 0,
        density: 1_000,
    });
    app.setBodyFillColor(sun, 'white');
    world.addBody(sun);

    const mercury = BodiesFactory.circle({
        radius: 10,
        x: 500,
        y: 500,
        density: 1,
    });
    mercury.velocity = getOrbitalSpeed(sun, mercury, GRAVITY);
    app.setBodyFillColor(mercury, 'orange');
    world.addBody(mercury);

    const venus = BodiesFactory.circle({
        radius: 15,
        x: 750,
        y: -750,
        density: 1,
    });
    venus.velocity = getOrbitalSpeed(sun, venus, GRAVITY);
    app.setBodyFillColor(venus, 'yellow');
    world.addBody(venus);

    const earth = BodiesFactory.circle({
        radius: 30,
        x: -1000,
        y: 1000,
        density: 1,
    });
    earth.velocity = getOrbitalSpeed(sun, earth, GRAVITY);
    app.setBodyFillColor(earth, 'green');
    world.addBody(earth);

    const mars = BodiesFactory.circle({
        radius: 25,
        x: -1250,
        y: -1250,
        density: 1,
    });
    mars.velocity = getOrbitalSpeed(sun, mars, GRAVITY);
    app.setBodyFillColor(mars, 'red');
    world.addBody(mars);

    const jupiter = BodiesFactory.circle({
        radius: 25,
        x: 1500,
        y: 1500,
        density: 100,
    });
    jupiter.velocity = getOrbitalSpeed(sun, jupiter, GRAVITY);
    app.setBodyFillColor(jupiter, 'orange');
    world.addBody(jupiter);

    app.setGravitationalForce(true);
}

/**
 * Computes the tangential velocity for a circular orbit of `planet` around `sun`.
 *
 * Uses: v = sqrt(G * (M + m) / r)
 * - M = sun mass
 * - m = planet mass
 * - r = distance between bodies
 *
 * The returned vector is perpendicular to the radius (tangential direction).
 */
function getOrbitalSpeed(sun: RigidBody, planet: RigidBody, G: number): Vec2 {
    const rVec = planet.position.subNew(sun.position);
    const r = rVec.magnitude();
    const v = Math.sqrt((G * (sun.mass + planet.mass)) / r);
    const dir = planet.position.subNew(sun.position).unitVector();
    const tangent = new Vec2(-dir.y, dir.x);

    return tangent.scaleNew(v);
}

const planetOrbitDemo = defineDemo('Planets orbit', setupPlanetOrbit);

export default planetOrbitDemo;
