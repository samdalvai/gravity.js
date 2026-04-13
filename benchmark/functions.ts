const values = new Array<number>(100_000);

for (let i = 0; i < values.length; i++) {
    values[i] = i;
}

export function runOriginal() {
    let total = 0;

    for (const value of values) {
        total += value * 2;
    }

    return total;
}

export function runModified() {
    let total = 0;

    for (let i = 0; i < values.length; i++) {
        total += values[i] * 2;
    }

    return total;
}
