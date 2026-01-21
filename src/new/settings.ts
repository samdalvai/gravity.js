import { PIXELS_PER_METER } from '../physics/Constants';

// Simulation settings
export const Settings = {
    positionCorrection: true,
    impulseAccumulation: true,
    warmStarting: true,
    penetrationSlop: 0.005 * PIXELS_PER_METER,
    restitutionSlop: 0.5 * PIXELS_PER_METER,
    positionCorrectionBeta: 0.2,
    warmStartingThreshold: (0.005 * PIXELS_PER_METER) ** 2,
    blockSolve: true,
    applyWarmStartingThreshold: true,
};
