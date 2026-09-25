export const FIXED_DELTA_TIME = 1 / 60;

export const PIXELS_PER_METER = 100;
/** Default body limit for worlds that do not specify maxBodies. */
export const MAX_BODIES = 5_000;
export const GRAVITY = 9.8;

export const MIN_BULLET_SPEED_SQUARED = 1_000_000;

export interface ContactSoftness {
    biasRate: number;
    massScale: number;
    impulseScale: number;
}

/** Builds the spring-damper coefficients used by the soft contact normal solve. */
export function makeSoft(hertz: number, dampingRatio: number, dt: number): ContactSoftness {
    if (hertz <= 0 || dt <= 0) {
        return { biasRate: 0, massScale: 0, impulseScale: 0 };
    }

    const omega = 2 * Math.PI * hertz;
    const a1 = 2 * dampingRatio + dt * omega;
    const a2 = dt * omega * a1;
    const a3 = 1 / (1 + a2);

    return {
        biasRate: omega / a1,
        massScale: a2 * a3,
        impulseScale: a3,
    };
}

/**
 * Setting are a global configuration tha affect each world created, if
 * you change some property this will affect all simulations at the same time
 */
export const SETTINGS = {
    // Simulation settings
    applyGravity: true,
    positionCorrection: true,
    impulseAccumulation: true,
    warmStarting: true,
    applyWarmStartingThreshold: true,
    blockSolve: true,
    ccd: true,
    /** Collect per-update timing and collision counters in World. Disabled by default. */
    collectMetrics: false,

    // Thresholds and slops
    warmStartingThreshold: 0.5 * 0.5,
    contactMergeThreshold: 0.005 * 0.005,
    penetrationSlop: 0.005 * PIXELS_PER_METER,
    restitutionSlop: 1 * PIXELS_PER_METER,
    angularVelocitySlop: 0.05,
    positionCorrectionBeta: 0.2,
    contactSlop: 0.01, // Legacy roundoff tolerance. Use speculativeDistance for contact generation.
    speculativeDistance: 4 * 0.005 * PIXELS_PER_METER,
    contactPushSpeed: 3 * PIXELS_PER_METER,
    contactHertz: 30,
    contactDampingRatio: 10,

    // Solver iterations fine tuning
    solverIterations: 10,
    subSteps: 1,

    get dt() {
        return FIXED_DELTA_TIME / this.subSteps;
    },

    get invDt() {
        return 1 / this.dt;
    },
};
