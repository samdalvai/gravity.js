import Graphics from './Graphics';
import { MILLISECS_PER_FRAME } from './physics/Constants';

export default class Application {
    private running: boolean;
    private timePreviousFrame: number = 0;
    // std::vector<Particle*> particles;
    // Vec2 pushForce = Vec2(0, 0);
    // Vec2 mouseCursor = Vec2(0, 0);
    // bool leftMouseButtonDown = false;

    constructor() {
        this.running = false;
    }

    isRunning = (): boolean => {
        return this.running;
    };

    setup = (): void => {
        this.running = Graphics.openWindow();
    };

    input = (): void => {
        // TODO: implement input from user
    };

    update = async (): Promise<void> => {
        const timeToWait = MILLISECS_PER_FRAME - (performance.now() - this.timePreviousFrame);

        if (timeToWait > 0) {
            await this.sleep(timeToWait);
        }

        let deltaTime = (performance.now() - this.timePreviousFrame) / 1000;

        if (deltaTime > 0.016) {
            deltaTime = 0.016;
        }

        this.timePreviousFrame = performance.now();

        // TODO: implement update of entities
    };

    render = (): void => {
        // TODO: implement rendering pipeline
        Graphics.clearScreen();

        Graphics.drawLine(250, 250, 500, 600, 'red');

        Graphics.drawLine(300, 300, 550, 650, 'green');
        Graphics.drawLine(100, 100, 100, 500, 'blue');

        Graphics.drawCircle(700, 700, 50, 4, 'blue');

        Graphics.drawFillCircle(500, 500, 50, 'orange');
        Graphics.drawFillCircle(800, 800, 15, 'white');
        Graphics.drawFillCircle(1000, 1000, 20, 'green');
    };

    destroy = (): void => {
        // TODO: do we need this method
    };

    sleep = (milliseconds: number) => {
        return new Promise(resolve => setTimeout(resolve, milliseconds));
    };
}
