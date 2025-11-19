import Graphics from './Graphics';
import { MILLISECS_PER_FRAME } from './physics/Constants';
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
            // Nasty hardcoded flip in velocity if it touches the limits of the screen window
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
        // TODO: do we need this method ?
    };

    sleep = (milliseconds: number) => {
        return new Promise(resolve => setTimeout(resolve, milliseconds));
    };
}
