export const PIXELS_PER_METER = 100;
export const MAX_BODIES = 5000;
export const GRAVITY = 9.8;

export const SETTINGS = {
    positionCorrection: true,
    impulseAccumulation: true,
    warmStarting: true,
    blockSolve: true,
    applyWarmStartingThreshold: true,
    warmStartingThreshold: (0.005 * PIXELS_PER_METER) ** 2,
    penetrationSlop: 0.005 * PIXELS_PER_METER,
    restitutionSlop: 0.5 * PIXELS_PER_METER,
    positionCorrectionBeta: 0.2,
};
