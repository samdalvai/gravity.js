export const FIXED_DELTA_TIME = 1 / 60;

export const PIXELS_PER_METER = 100;
export const MAX_BODIES = 5_000;
export const GRAVITY = 9.8;

export const MIN_BULLET_SPEED = 1_000_000;

export const SETTINGS = {
    // Simulation settings
    applyGravity: true,
    positionCorrection: true,
    impulseAccumulation: true,
    warmStarting: true,
    applyWarmStartingThreshold: true,
    blockSolve: true,
    // ccd: true,
    ccd: false,

    // Thresholds and slops
    warmStartingThreshold: 0.5 * 0.5,
    contactMergeThreshold: 0.005 * 0.005,
    penetrationSlop: 0.5,
    restitutionSlop: 50,
    angularVelocitySlop: 0.05,
    positionCorrectionBeta: 0.2,

    // Solver iterations fine tuning
    solverIterations: 10,
    subSteps: 1,

    get dt() {
        return FIXED_DELTA_TIME / this.subSteps;
    },
};
