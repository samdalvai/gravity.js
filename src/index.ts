import Application from './Application';

const app = new Application();
app.setup();

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
