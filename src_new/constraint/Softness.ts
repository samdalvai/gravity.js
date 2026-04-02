export interface Softness {
    biasRate: number;
    massScale: number;
    impulseScale: number;
}

export function makeSoftness(hertz: number, dampingRatio: number, dt: number): Softness {
    if (hertz === 0.0 || dt === 0.0) {
        return {
            biasRate: 0.0,
            massScale: 1.0,
            impulseScale: 0.0,
        };
    }

    const omega = 2.0 * Math.PI * hertz;
    const a1 = 2.0 * dampingRatio + dt * omega;
    const a2 = dt * omega * a1;
    const a3 = 1.0 / (1.0 + a2);

    return {
        biasRate: omega / a1,
        massScale: a2 * a3,
        impulseScale: a3,
    };
}
