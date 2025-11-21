import Graphics from './Graphics';
import InputManager, { MouseButton } from './InputManager';
import Utils from './Utils';
import { PIXELS_PER_METER } from './physics/Constants';
import Force from './physics/Force';
import Particle from './physics/Particle';
import Vec2 from './physics/Vec2';

export default class Application {
    private running: boolean;
    private timePreviousFrame: number = 0;
    private particles: Particle[] = [];
    private pushForce = new Vec2(0, 0);
    private mouseCursor = new Vec2(0, 0);
    private leftMouseButtonDown = false;

    constructor() {
        this.running = false;
    }

    isRunning = (): boolean => {
        return this.running;
    };

    setup = (): void => {
        this.running = Graphics.openWindow();
        
        const bigPlanet = new Particle(Graphics.width() / 2, Graphics.height() / 2, 20, 'yellow', 20);
        this.particles.push(bigPlanet);

        // const smallPlanet1 = new Particle(Graphics.width() / 2, 300, 6, 'blue', 1);
        // smallPlanet1.velocity.x = 200;
        // this.particles.push(smallPlanet1);

        // const smallPlanet2 = new Particle(Graphics.width() / 2, Graphics.height() / 2 + 400, 8, 'green', 5);
        // smallPlanet2.velocity.x = -220;
        // this.particles.push(smallPlanet2);

        // Generate random particles
        for (let i = 0; i < 1000; i++) {
            const randomRadius = Utils.randomNumber(1, 5);
            const smallPlanet = new Particle(
                Utils.randomNumber(20, Graphics.width() - 20),
                Utils.randomNumber(20, Graphics.height() - 20),
                randomRadius,
                Utils.randomColor(),
                randomRadius,
            );
            smallPlanet.velocity.x = Utils.randomNumber(-200, 200);
            this.particles.push(smallPlanet);
        }

        InputManager.initialize();
    };

    input = (): void => {
        // Handle keyboard events
        while (InputManager.keyboardInputBuffer.length > 0) {
            const inputEvent = InputManager.keyboardInputBuffer.shift();

            if (!inputEvent) {
                return;
            }

            switch (inputEvent.type) {
                case 'keydown':
                    switch (inputEvent.code) {
                        case 'ArrowUp':
                            this.pushForce.y = -50 * PIXELS_PER_METER;
                            break;
                        case 'ArrowRight':
                            this.pushForce.x = 50 * PIXELS_PER_METER;
                            break;
                        case 'ArrowDown':
                            this.pushForce.y = 50 * PIXELS_PER_METER;
                            break;
                        case 'ArrowLeft':
                            this.pushForce.x = -50 * PIXELS_PER_METER;
                            break;
                    }
                    break;
                case 'keyup':
                    switch (inputEvent.code) {
                        case 'ArrowUp':
                            this.pushForce.y = 0;
                            break;
                        case 'ArrowRight':
                            this.pushForce.x = 0;
                            break;
                        case 'ArrowDown':
                            this.pushForce.y = 0;
                            break;
                        case 'ArrowLeft':
                            this.pushForce.x = 0;
                            break;
                    }
                    break;
            }
        }

        // Handle mouse move events
        while (InputManager.mouseMoveBuffer.length > 0) {
            const inputEvent = InputManager.mouseMoveBuffer.shift();

            if (!inputEvent) {
                return;
            }

            this.mouseCursor.x = inputEvent.x;
            this.mouseCursor.y = inputEvent.y;
        }

        // Handle mouse click events
        while (InputManager.mouseInputBuffer.length > 0) {
            const inputEvent = InputManager.mouseInputBuffer.shift();

            if (!inputEvent) {
                return;
            }

            switch (inputEvent.type) {
                case 'mousedown':
                    if (!this.leftMouseButtonDown && inputEvent.button === MouseButton.LEFT) {
                        this.leftMouseButtonDown = true;
                        this.mouseCursor.x = inputEvent.x;
                        this.mouseCursor.y = inputEvent.y;
                    }
                    break;
                case 'mouseup':
                    if (this.leftMouseButtonDown && inputEvent.button === MouseButton.LEFT) {
                        this.leftMouseButtonDown = false;
                        const impulseDirection = this.particles[0].position.subNew(this.mouseCursor).unitVector();
                        const impulseMagnitude = this.particles[0].position.subNew(this.mouseCursor).magnitude() * 2;
                        this.particles[0].velocity.assign(impulseDirection.scaleNew(impulseMagnitude));
                    }
                    break;
            }
        }
    };

    update = (deltaTime: number): void => {
        /////////////////////////////////////////////////////////////////////
        ///// Updating all the entities                                 /////
        /////////////////////////////////////////////////////////////////////

        for (const particle of this.particles) {
            particle.addForce(this.pushForce);

            // Apply a friction force
            // const friction = Force.generateFrictionForce(particle, 20);
            // particle.addForce(friction);
        }

        // Applying a gravitational force to our two particles/planets
        for (let i = 0; i < this.particles.length - 1; i++) {
            for (let j = 1; j < this.particles.length; j++) {
                const attraction = Force.generateGravitationalForce(
                    this.particles[i],
                    this.particles[j],
                    1000.0,
                    5,
                    100,
                );
                this.particles[i].addForce(attraction);
                this.particles[j].addForce(attraction.negate());
            }
        }

        for (const particle of this.particles) {
            particle.integrate(deltaTime);
        }

        for (const particle of this.particles) {
            // Hardcoded flip in velocity if it touches the limits of the screen window
            if (particle.position.x - particle.radius <= 0) {
                particle.position.x = particle.radius;
                particle.velocity.x *= -0.9;
            } else if (particle.position.x + particle.radius >= Graphics.width()) {
                particle.position.x = Graphics.width() - particle.radius;
                particle.velocity.x *= -0.9;
            }
            if (particle.position.y - particle.radius <= 0) {
                particle.position.y = particle.radius;
                particle.velocity.y *= -0.9;
            } else if (particle.position.y + particle.radius >= Graphics.height()) {
                particle.position.y = Graphics.height() - particle.radius;
                particle.velocity.y *= -0.9;
            }
        }
    };

    render = (): void => {
        Graphics.clearScreen();

        if (this.leftMouseButtonDown) {
            Graphics.drawLine(
                this.particles[0].position.x,
                this.particles[0].position.y,
                this.mouseCursor.x,
                this.mouseCursor.y,
                'red',
            );
        }

        for (const particle of this.particles) {
            Graphics.drawFillCircle(particle.position.x, particle.position.y, particle.radius, particle.color);
        }
    };

    sleep = (milliseconds: number) => {
        return new Promise(resolve => setTimeout(resolve, milliseconds));
    };
}
