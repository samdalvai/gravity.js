import Graphics from './Graphics';
import { MILLISECS_PER_FRAME } from './physics/Constants';
import Particle from './physics/Particle';

export default class Application {
    private running: boolean;
    private timePreviousFrame: number = 0;
    private particles: Particle[] = [];
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

        const smallPlanet1 = new Particle(Graphics.width() / 2, 300, 6, 1);
        smallPlanet1.velocity.x = 200;
        this.particles.push(smallPlanet1);

        const smallPlanet2 = new Particle(Graphics.width() / 2, Graphics.height() / 2 + 400, 8, 5);
        smallPlanet2.velocity.x = -220;
        this.particles.push(smallPlanet2);

        const bigPlanet = new Particle(Graphics.width() / 2, Graphics.height() / 2, 20, 20);
        this.particles.push(bigPlanet);
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

        Graphics.drawFillCircle(
            this.particles[0].position.x,
            this.particles[0].position.y,
            this.particles[0].radius,
            'blue',
        );

        Graphics.drawFillCircle(
            this.particles[1].position.x,
            this.particles[1].position.y,
            this.particles[1].radius,
            'green',
        );

        Graphics.drawFillCircle(
            this.particles[2].position.x,
            this.particles[2].position.y,
            this.particles[2].radius,
            'yellow',
        );
    };

    destroy = (): void => {
        // TODO: do we need this method
    };

    sleep = (milliseconds: number) => {
        return new Promise(resolve => setTimeout(resolve, milliseconds));
    };
}
