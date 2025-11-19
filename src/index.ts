import Application from './Application';

const app = new Application();

const run = async () => {
    app.setup();
    const loop = async () => {
        while (app.isRunning()) {
            app.input();
            await app.update();
            app.render();
        }
    };
    loop();
};

run();

app.destroy();
