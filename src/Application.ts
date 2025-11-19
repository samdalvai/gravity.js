import Graphics from './Graphics';

export default class Application {
    private running: boolean;
    private previousFrame: number = 0;
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

    update = (): void => {
        // TODO: implement update of entities
    };

    render = (): void => {
        // TODO: implement rendering pipeline
    };

    destroy = (): void => {
        // TODO: do we need this method
    };
}
