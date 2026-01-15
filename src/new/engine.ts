import { Game } from './game';
import * as Input from './input';
import { Renderer } from './renderer';
import { Settings } from './settings';

export class Engine {
    public cvs: HTMLCanvasElement;
    public gfx: CanvasRenderingContext2D;
    public frameCounter: HTMLDivElement;

    public renderer: Renderer;
    public game: Game;

    private running = true;

    private frames = 0;
    private lastFPSUpdate = 0;
    
    public static fps = 0;

    constructor() {
        // Prevent browser interference
        const body = document.body;
        body.addEventListener('contextmenu', e => e.preventDefault());
        body.ondragstart = () => false;
        body.onselectstart = () => false;

        window.addEventListener('keydown', e => {
            if (e.key === ' ' && e.target === document.body) e.preventDefault();
        });

        this.cvs = document.querySelector('#canvas') as HTMLCanvasElement;
        this.cvs.width = Settings.width;
        this.cvs.height = Settings.height;

        this.gfx = this.cvs.getContext('2d')!;
        this.frameCounter = document.querySelector('.frame_counter') as HTMLDivElement;

        this.renderer = new Renderer(this.gfx);
        this.game = new Game(this.renderer);

        Input.init(this);

        // Pause when tab is hidden
        document.addEventListener('visibilitychange', () => {
            this.running = !document.hidden;
        });
    }

    start(): void {
        let previousTime = performance.now();

        const loop = (now: number) => {
            const deltaTime = (now - previousTime) / 1000;
            previousTime = now;

            if (this.running && !Settings.paused) {
                this.update(deltaTime);
                this.render();
            } else {
                this.drawPause();
            }

            this.updateFPS(now);
            requestAnimationFrame(loop);
        };

        requestAnimationFrame(loop);
    }

    private update(deltaTime: number): void {
        this.game.update(deltaTime);
        Input.update();
    }

    private render(): void {
        this.gfx.clearRect(0, 0, Settings.width, Settings.height);
        this.game.render(this.renderer);
    }

    private drawPause(): void {
        this.gfx.font = '48px system-ui';
        this.gfx.fillText('PAUSE', 4, 40);
    }

    private updateFPS(now: number): void {
        this.frames++;

        if (now - this.lastFPSUpdate >= 1000) {
            Engine.fps = this.frames;
            this.frameCounter.textContent = `${Engine.fps}fps`;
            this.frames = 0;
            this.lastFPSUpdate = now;
        }
    }
}
