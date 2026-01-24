export const DELTA_TIME = 1 / 60;
export const INVERSE_DELTA_TIME = 1 / DELTA_TIME;

export const PIXELS_PER_METER = 100;
export const MAX_BODIES = 5000;
export const GRAVITY = 9.8;

// Since the findFarthestEdge function returns a edge with a minimum length of 0.01 for circle,
// merging threshold should be greater than sqrt(2) * minimum edge length
export const TANGENT_MIN_LENGTH = 0.01 * PIXELS_PER_METER;
export const CONTACT_MERGE_THRESHOLD = 1.415 * TANGENT_MIN_LENGTH;

export const SETTINGS = {
    positionCorrection: true,
    impulseAccumulation: true,
    warmStarting: true,
    blockSolve: true,
    penetrationSlop: 0.005 * PIXELS_PER_METER,
    restitutionSlop: 0.5 * PIXELS_PER_METER,
    positionCorrectionBeta: 0.2,
};
