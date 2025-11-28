import Application from './Application';

const run = async () => {
    const app = new Application();
    await app.setup();

    console.log('Setup finished, starting loop');

    let timePreviousFrame = performance.now();

    const loop = (now: number) => {
        const deltaTime = (now - timePreviousFrame) / 1000;
        timePreviousFrame = now;

        app.input();
        app.update(deltaTime);
        app.render();

        requestAnimationFrame(loop);
    };

    requestAnimationFrame(loop);
};

run();
