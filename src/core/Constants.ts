export const FIXED_DELTA_TIME = 1 / 60;

export const PIXELS_PER_METER = 100;
export const MAX_BODIES = 10_000;
export const GRAVITY = 9.8;

// Since the findFarthestEdge function returns a edge with a minimum length of 0.01 for circle,
// merging threshold should be greater than sqrt(2) * minimum edge length
export const TANGENT_MIN_LENGTH = 0.01 * PIXELS_PER_METER;
export const CONTACT_MERGE_THRESHOLD = 1.415 * TANGENT_MIN_LENGTH;
export const MIN_BULLET_SPEED = 1_000_000;
export const BODY_REMOVAL_THRESHOLD = 25000;

export const PLAYER_MOVE_FORCE = 1500;
export const PLAYER_MAX_SPEED = 350;
export const PLAYER_ACCELERATION = 10;
export const PLAYER_JUMP_IMPULSE = 600;

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
