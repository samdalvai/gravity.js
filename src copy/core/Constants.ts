export const FIXED_DELTA_TIME = 1 / 60;

export const PIXELS_PER_METER = 100;
export const MAX_BODIES = 2500;
export const GRAVITY = 9.8;

export const MIN_BULLET_SPEED = 1_000_000;


export const SETTINGS = {
    applyGravity: true,
    warmStarting: true,
    positionCorrection: true,
    sleepEnabled: true,
    sleepLinearVelocity: 5,
    sleepAngularVelocity: 0.1,
    sleepTimeThreshold: 0.5,
    positionCorrectionBeta: 0.2,
    contactPushoutVelocity: 5 * PIXELS_PER_METER,
    contactHertz: 30,
    contactDampingRatio: 10,
    restitutionThreshold: 10 * PIXELS_PER_METER,
    angularVelocitySlop: 0.05,
    solverIterations: 8,
    subSteps: 2,
};

export const REAL_DELTA_TIME = () => FIXED_DELTA_TIME / SETTINGS.subSteps;
