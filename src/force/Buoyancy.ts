// TODO: To be improved by computing submerged area and applyin the force at that centroid
import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';
import { CircleShape } from '../shapes/CircleShape';
import { PolygonShape } from '../shapes/PolygonShape';
import { ShapeType } from '../shapes/Shape';

type BuoyancyResult = {
    force: Vec2;
    applicationPoint: Vec2;
} | null;

type SubmergedShape = {
    area: number;
    centroid: Vec2;
};

// instead of the body default centroid compute submerged real area and return real centroid
/**
 * Computes buoyancy force based on submerged area.
 * The force should be applied by using {@link RigidBody.addForceAtPoint}.
 */
export function generateBuoyancyForce(
    body: RigidBody,
    waterSurfaceY: number,
    liquidDensity: number,
    gravity: number,
): BuoyancyResult {
    if (body.shapeType === ShapeType.POLYGON || body.shapeType === ShapeType.BOX) {
        const polygon = body.shape as PolygonShape;
        const vertices = polygon.worldVertices;

        const result = getSubmergedPolygon(vertices, waterSurfaceY);

        if (!result) {
            return null;
        }

        const submergedArea = result.area;
        const buoyancyMagnitude = liquidDensity * submergedArea * gravity;
        const force = new Vec2(0, buoyancyMagnitude);

        return {
            force,
            applicationPoint: result.centroid,
        };
    }

    if (body.shapeType === ShapeType.CIRCLE) {
        const circle = body.shape as CircleShape;
        const r = circle.radius;
        const cx = body.position.x;
        const cy = body.position.y;

        const d = waterSurfaceY - cy;

        // Fully above
        if (d <= -r) {
            return null;
        }

        let submergedArea: number;

        // Fully submerged
        if (d >= r) {
            submergedArea = Math.PI * r * r;
            const buoyancyMagnitude = liquidDensity * submergedArea * gravity;
            const force = new Vec2(0, buoyancyMagnitude);

            return {
                force,
                applicationPoint: body.position.copy(),
            };
        }

        submergedArea = r * r * Math.acos(-d / r) + d * Math.sqrt(r * r - d * d);

        const a = Math.sqrt(r * r - d * d);
        const yOffset = -(2 * Math.pow(a, 3)) / submergedArea;
        const centroid = new Vec2(cx, cy + yOffset);
        const buoyancyMagnitude = liquidDensity * submergedArea * gravity;
        const force = new Vec2(0, buoyancyMagnitude);

        return {
            force,
            applicationPoint: centroid,
        };
    }

    // Approximation if there is no algorithm to compute the real buoyancy
    return approximateBuoyancy(body, waterSurfaceY, liquidDensity, gravity);
}

function getSubmergedPolygon(vertices: readonly Vec2[], waterSurfaceY: number): SubmergedShape | null {
    const clipped: Vec2[] = [];

    for (let i = 0; i < vertices.length; i++) {
        const a = vertices[i];
        const b = vertices[(i + 1) % vertices.length];

        const aUnder = a.y <= waterSurfaceY;
        const bUnder = b.y <= waterSurfaceY;

        if (aUnder) {
            clipped.push(a.copy());
        }

        if (aUnder !== bUnder) {
            const t = (waterSurfaceY - a.y) / (b.y - a.y);
            const x = a.x + (b.x - a.x) * t;
            clipped.push(new Vec2(x, waterSurfaceY));
        }
    }

    if (clipped.length < 3) {
        return null;
    }

    let doubleArea = 0;
    let cx = 0;
    let cy = 0;

    for (let i = 0; i < clipped.length; i++) {
        const current = clipped[i];
        const next = clipped[(i + 1) % clipped.length];

        const cross = current.cross(next);
        doubleArea += cross;
        cx += (current.x + next.x) * cross;
        cy += (current.y + next.y) * cross;
    }

    if (doubleArea === 0) {
        return null;
    }

    const factor = 1 / (3 * doubleArea);
    const centroid = new Vec2(cx * factor, cy * factor);

    return {
        area: Math.abs(doubleArea) * 0.5,
        centroid,
    };
}

function approximateBuoyancy(
    body: RigidBody,
    waterSurfaceY: number,
    liquidDensity: number,
    gravity: number,
): BuoyancyResult {
    const maxX = body.maxX;
    const minX = body.minX;
    const maxY = body.maxY;
    const minY = body.minY;

    const width = maxX - minX;
    const height = maxY - minY;

    if (width <= 0 || height <= 0) {
        return null;
    }

    // Amount of the AABB below the water surface
    const submergedHeight = Math.max(0, Math.min(waterSurfaceY - minY, height));

    if (submergedHeight === 0) {
        return null;
    }

    const submergedFraction = submergedHeight / height;
    const submergedArea = body.shape.getArea() * submergedFraction;
    const buoyancyMagnitude = liquidDensity * submergedArea * gravity;
    const force = new Vec2(0, buoyancyMagnitude);
    const applicationPoint = body.position.copy();

    return {
        force,
        applicationPoint,
    };
}

export function generateLinearWaterDragForce(
    body: RigidBody,
    waterSurfaceY: number,
    dragCoefficient: number,
    dt: number,
): Vec2 {
    const maxY = body.maxY;
    const minY = body.minY;

    const height = maxY - minY;

    if (height <= 0) {
        return new Vec2(0, 0);
    }

    // Same submerged logic as buoyancy
    const submergedHeight = Math.max(0, Math.min(waterSurfaceY - minY, height));

    if (submergedHeight === 0) {
        return new Vec2(0, 0);
    }

    const submergedFraction = submergedHeight / height;

    const v = body.velocity;
    const speedSq = v.magnitudeSquared();

    if (speedSq === 0) {
        return new Vec2(0, 0);
    }

    const speed = Math.sqrt(speedSq);

    // Quadratic drag
    let dragMagnitude = dragCoefficient * speedSq * submergedFraction;

    if (dt > 0) {
        // Do not let water drag reverse the velocity in a single step.
        const maxForce = (body.mass * speed) / dt;
        dragMagnitude = Math.min(dragMagnitude, maxForce);
    }

    // Opposite to velocity
    return v.scaleNew(-dragMagnitude / speed);
}

export function generateAngularWaterDragTorque(
    body: RigidBody,
    waterSurfaceY: number,
    angularDragCoefficient: number,
): number {
    const maxY = body.maxY;
    const minY = body.minY;

    const height = maxY - minY;

    if (height <= 0) {
        return 0;
    }

    const submergedHeight = Math.max(0, Math.min(waterSurfaceY - minY, height));

    if (submergedHeight === 0) {
        return 0;
    }

    const submergedFraction = submergedHeight / height;

    // Scale by moment of inertia so the coefficient behaves like an angular damping rate.
    // Otherwise large bodies barely slow down because angular acceleration is torque / I.
    return -body.angularVelocity * angularDragCoefficient * submergedFraction * body.I;
}
