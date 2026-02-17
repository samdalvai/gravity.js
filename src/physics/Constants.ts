export const DELTA_TIME = 1 / 60;
export const INVERSE_DELTA_TIME = 1 / DELTA_TIME;

export const PIXELS_PER_METER = 100;
export const MAX_BODIES = 2000;
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
    penetrationSlop: 0.5,
    restitutionSlop: 50,
    positionCorrectionBeta: 0.2,
};

export const PLAYER_MOVE_FORCE = 1500;
export const PLAYER_MAX_SPEED = 350;
export const PLAYER_ACCELERATION = 10;
export const PLAYER_JUMP_IMPULSE = 600;
