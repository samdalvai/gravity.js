// TODO: To be improved by computing submerged area and applyin the force at that centroid
import { RigidBody } from '../core/RigidBody';
import { Vec2 } from '../math/Vec2';

type BuoyancyResult = {
    force: Vec2;
    applicationPoint: Vec2;
} | null;

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
): Vec2 {
    const maxX = body.maxX;
    const minX = body.minX;
    const maxY = body.maxY;
    const minY = body.minY;

    const width = maxX - minX;
    const height = maxY - minY;

    if (width <= 0 || height <= 0) {
        return new Vec2(0, 0);
    }

    // Amount of the AABB below the water surface
    const submergedHeight = Math.max(0, Math.min(waterSurfaceY - minY, height));

    if (submergedHeight === 0) {
        return new Vec2(0, 0);
    }

    const submergedFraction = submergedHeight / height;
    const submergedArea = body.shape.getArea() * submergedFraction;
    const buoyancyMagnitude = liquidDensity * submergedArea * gravity;

    return new Vec2(0, buoyancyMagnitude);
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
