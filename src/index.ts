import Application from './core/Application';
import { FIXED_DELTA_TIME } from './core/Constants';

async function run() {
    const app = new Application();
    await app.setup();

    console.log('Setup finished, starting loop');

    let timePreviousFrame = performance.now();
    let accumulator = 0;

    document.addEventListener('visibilitychange', () => {
        app.setRunning(!document.hidden);
        if (!document.hidden) {
            // Reset previous frame time to avoid a huge deltaTime spike
            timePreviousFrame = performance.now();
        }
    });

    function loop(now: number) {
        let frameTime = (now - timePreviousFrame) / 1000;
        timePreviousFrame = now;

        // Avoid spiral of death
        frameTime = Math.min(frameTime, 0.25);

        accumulator += frameTime;

        if (app.isRunning()) {
            app.input();

            while (accumulator >= FIXED_DELTA_TIME) {
                app.update(frameTime);
                accumulator -= FIXED_DELTA_TIME;
            }

            // Optional: interpolation (for smooth rendering)
            // TODO: to be implemented if appropriate
            // Store previous position and current one and interpolate position and rotation, e.g.
            // function lerp(a, b, t) {
            //     return a + (b - a) * t;
            // }

            // function lerpVec2(a, b, t) {
            //     return {
            //         x: lerp(a.x, b.x, t),
            //         y: lerp(a.y, b.y, t),
            //     };
            // }
            // const alpha = accumulator / DELTA_TIME;
            app.render();
        }

        requestAnimationFrame(loop);
    }

    requestAnimationFrame(loop);
}

run();
