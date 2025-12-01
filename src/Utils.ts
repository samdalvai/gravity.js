export default class Utils {
    static randomNumber = (min: number = 1.0, max: number = 10.0): number => {
        return Math.random() * (max - min) + min;
    };

    // Returns a random color as a hex string, e.g. "#A3F4C2"
    static randomColor = (): string => {
        const r = Math.floor(Math.random() * 256);
        const g = Math.floor(Math.random() * 256);
        const b = Math.floor(Math.random() * 256);

        // Convert to hex and pad with zeros if needed
        const rHex = r.toString(16).padStart(2, '0');
        const gHex = g.toString(16).padStart(2, '0');
        const bHex = b.toString(16).padStart(2, '0');

        return `#${rHex}${gHex}${bHex}`;
    };

    static clamp = (value: number, min: number, max: number) => Math.min(Math.max(value, min), max);
}
