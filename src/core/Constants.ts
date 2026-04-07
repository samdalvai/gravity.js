export const FIXED_DELTA_TIME = 1 / 60;

export const PIXELS_PER_METER = 100;
export const MAX_BODIES = 10_000;
export const GRAVITY = 9.8;

export const MIN_BULLET_SPEED = 1_000_000;

export const SETTINGS = {
    applyGravity: true,
    positionCorrection: true,
    impulseAccumulation: true,
    warmStarting: true,
    warmStartingThreshold: 0.5 * 0.5,
    applyWarmStartingThreshold: true,
    contactMergeThreshold: 0.005 * 0.005,
    blockSolve: true,
    penetrationSlop: 0.5,
    restitutionSlop: 50,
    positionCorrectionBeta: 0.2,
    angularVelocitySlop: 0.05,
    solverIterations: 10,
    subSteps: 1,

    get dt() {
        return FIXED_DELTA_TIME / this.subSteps;
    },
};
