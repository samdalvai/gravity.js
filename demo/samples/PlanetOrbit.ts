import { BodiesFactory, GRAVITY, SETTINGS, Vec2 } from '../../src';
import type { RigidBody, World } from '../../src';
import type Application from '../Application';
import Graphics from '../graphics/Graphics';
import { defineDemo } from './shared';

const AU_IN_KM = 149_597_870.7;
const EARTH_RADIUS_KM = 6_371;
const EARTH_ORBIT_PIXELS = 700;
const EARTH_READABLE_RADIUS_PIXELS = 10;
const USE_PLANET_TEXTURES = true;

/**
 * 0 = compressed, readable demo scale.
 * 1 = real relative orbit distances and physical radii in the same pixel scale.
 *
 * At 1, planets are physically correct but become sub-pixel dots. Values around
 * 0.25-0.45 keep the system pleasant to watch while preserving the real order
 * and relative differences.
 */
const SOLAR_SYSTEM_SCALE_REALISM = 0.25;

const COMPRESSED_ORBIT_EXPONENT = 0.62;
const COMPRESSED_RADIUS_EXPONENT = 0.55;
const MIN_READABLE_RADIUS_PIXELS = 3;
const MASS_SCALE = 1;

type CelestialBodySpec = {
    name: string;
    radiusKm: number;
    massEarths: number;
    color: string;
    texture?: TextureName;
    orbitAu?: number;
    orbitAngleDegrees?: number;
};

type TextureName = Parameters<Application['setBodyTexture']>[1];

const SUN: CelestialBodySpec = {
    name: 'Sun',
    radiusKm: 695_700,
    massEarths: 332_946,
    color: '#fff7b2',
    texture: 'planetSun',
};

const PLANETS: CelestialBodySpec[] = [
    {
        name: 'Mercury',
        radiusKm: 2_439.7,
        massEarths: 0.0553,
        orbitAu: 0.3871,
        orbitAngleDegrees: 15,
        color: '#b7ada5',
        texture: 'planetMercury',
    },
    {
        name: 'Venus',
        radiusKm: 6_051.8,
        massEarths: 0.815,
        orbitAu: 0.7233,
        orbitAngleDegrees: 105,
        color: '#d8b16f',
        texture: 'planetVenus',
    },
    {
        name: 'Earth',
        radiusKm: 6_371,
        massEarths: 1,
        orbitAu: 1,
        orbitAngleDegrees: 190,
        color: '#4a9fe8',
        texture: 'planetEarth',
    },
    {
        name: 'Mars',
        radiusKm: 3_389.5,
        massEarths: 0.107,
        orbitAu: 1.5237,
        orbitAngleDegrees: 280,
        color: '#c76245',
        texture: 'planetMars',
    },
    {
        name: 'Jupiter',
        radiusKm: 69_911,
        massEarths: 317.83,
        orbitAu: 5.2044,
        orbitAngleDegrees: 335,
        color: '#d1a06f',
        texture: 'planetJupiter',
    },
    {
        name: 'Saturn',
        radiusKm: 58_232,
        massEarths: 95.16,
        orbitAu: 9.5826,
        orbitAngleDegrees: 55,
        color: '#d7c28b',
        texture: 'planetSaturn',
    },
    {
        name: 'Uranus',
        radiusKm: 25_362,
        massEarths: 14.54,
        orbitAu: 19.2184,
        orbitAngleDegrees: 145,
        color: '#9fe1df',
        texture: 'planetUranus',
    },
    {
        name: 'Neptune',
        radiusKm: 24_622,
        massEarths: 17.15,
        orbitAu: 30.11,
        orbitAngleDegrees: 245,
        color: '#5279e8',
        texture: 'planetNeptune',
    },
];

function setupPlanetOrbit(world: World, app: Application): void {
    app.setBackground('transparent');
    Graphics.zoom = 0.055;

    SETTINGS.applyGravity = false;

    const sun = createBody(SUN, new Vec2(0, 0));
    world.addBody(sun);
    applyBodyStyle(app, sun, SUN);

    for (const planetSpec of PLANETS) {
        const position = getOrbitPosition(planetSpec);
        const planet = createBody(planetSpec, position);
        planet.velocity = getOrbitalSpeed(sun, planet, GRAVITY);
        world.addBody(planet);
        applyBodyStyle(app, planet, planetSpec);
    }

    app.setGravitationalForce(true);
}

function applyBodyStyle(app: Application, body: RigidBody, spec: CelestialBodySpec): void {
    if (USE_PLANET_TEXTURES && spec.texture) {
        app.setBodyTexture(body, spec.texture);
        app.setBodyTextureScale(body, 1.2);
        return;
    }

    app.setBodyFillColor(body, spec.color);
}

function createBody(spec: CelestialBodySpec, position: Vec2): RigidBody {
    return BodiesFactory.circle({
        radius: getScaledRadius(spec.radiusKm),
        x: position.x,
        y: position.y,
        mass: spec.massEarths * MASS_SCALE,
        friction: 0,
        restitution: 0,
    });
}

function getOrbitPosition(spec: CelestialBodySpec): Vec2 {
    const orbitAu = spec.orbitAu ?? 0;
    const orbitAngleDegrees = spec.orbitAngleDegrees ?? 0;
    const distance = getScaledOrbitDistance(orbitAu);
    const angle = degreesToRadians(orbitAngleDegrees);

    return new Vec2(Math.cos(angle) * distance, Math.sin(angle) * distance);
}

function getScaledOrbitDistance(orbitAu: number): number {
    const compressedDistance = EARTH_ORBIT_PIXELS * Math.pow(orbitAu, COMPRESSED_ORBIT_EXPONENT);
    const realDistance = EARTH_ORBIT_PIXELS * orbitAu;

    return lerp(compressedDistance, realDistance, SOLAR_SYSTEM_SCALE_REALISM);
}

function getScaledRadius(radiusKm: number): number {
    const radiusEarths = radiusKm / EARTH_RADIUS_KM;
    const readableRadius = EARTH_READABLE_RADIUS_PIXELS * Math.pow(radiusEarths, COMPRESSED_RADIUS_EXPONENT);
    const realRadius = radiusKm * (EARTH_ORBIT_PIXELS / AU_IN_KM);
    const shrinkingMinimumRadius = MIN_READABLE_RADIUS_PIXELS * (1 - SOLAR_SYSTEM_SCALE_REALISM);

    return Math.max(shrinkingMinimumRadius, lerp(readableRadius, realRadius, SOLAR_SYSTEM_SCALE_REALISM));
}

function lerp(from: number, to: number, t: number): number {
    return from + (to - from) * t;
}

function degreesToRadians(degrees: number): number {
    return (degrees * Math.PI) / 180;
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
