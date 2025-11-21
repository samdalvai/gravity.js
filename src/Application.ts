import Graphics from './Graphics';
import InputManager, { MouseButton } from './InputManager';
import Utils from './Utils';
import { PIXELS_PER_METER } from './physics/Constants';
import Force from './physics/Force';
import Particle from './physics/Particle';
import Vec2 from './physics/Vec2';

export default class Application {
    private running: boolean;
    private particles: Particle[] = [];
    private pushForce = new Vec2(0, 0);
    private mouseCursor = new Vec2(0, 0);
    private leftMouseButtonDown = false;

    ///////////////////////////////////////////////////////////////////////////////
    // Soft body with spring force
    ///////////////////////////////////////////////////////////////////////////////
    // private k = 1500;
    // private restLength = 200;
    // private NUM_PARTICLES = 4;

    ///////////////////////////////////////////////////////////////////////////////
    // Particle chain with spring force
    ///////////////////////////////////////////////////////////////////////////////
    private anchor = new Vec2(0, 0);
    private k = 300;
    private restLength = 15;
    private NUM_PARTICLES = 15;

    constructor() {
        this.running = false;
    }

    isRunning = (): boolean => {
        return this.running;
    };

    setup = (): void => {
        this.running = Graphics.openWindow();

        ///////////////////////////////////////////////////////////////////////////////
        // Soft body with spring force
        ///////////////////////////////////////////////////////////////////////////////
        // Particle* a = new Particle(100, 100, 1.0);
        // Particle* b = new Particle(300, 100, 1.0);
        // Particle* c = new Particle(300, 300, 1.0);
        // Particle* d = new Particle(100, 300, 1.0);

        // a->radius = 6;
        // b->radius = 6;
        // c->radius = 6;
        // d->radius = 6;

        // particles.push_back(a);
        // particles.push_back(b);
        // particles.push_back(c);
        // particles.push_back(d);

        ///////////////////////////////////////////////////////////////////////////////
        // Particle chain with spring force
        ///////////////////////////////////////////////////////////////////////////////
        this.anchor = new Vec2(Graphics.width() / 2.0, 30);

        for (let i = 0; i < this.NUM_PARTICLES; i++) {
            const bob = new Particle(this.anchor.x, this.anchor.y + i * this.restLength, 6, 'blue', 2.0);
            this.particles.push(bob);
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
            // Apply push force
            particle.addForce(this.pushForce);

            // Apply a friction force
            // const friction = Force.generateFrictionForce(particle, 20);
            // particle.addForce(friction);

            // Apply a drag force
            const drag = Force.generateDragForce(particle, 0.003);
            particle.addForce(drag);

            // Apply weight force
            const weight = new Vec2(0.0, particle.mass * 9.8 * PIXELS_PER_METER);
            particle.addForce(weight);
        }

        ///////////////////////////////////////////////////////////////////////////////
        // Soft body with spring force
        ///////////////////////////////////////////////////////////////////////////////
        // Attach particles with springs
        // Vec2 ab = Force::GenerateSpringForce(*particles[0], *particles[1], restLength, k); // a <-> b
        // particles[0]->AddForce(ab);
        // particles[1]->AddForce(-ab);

        // Vec2 bc = Force::GenerateSpringForce(*particles[1], *particles[2], restLength, k); // b <-> c
        // particles[1]->AddForce(bc);
        // particles[2]->AddForce(-bc);

        // Vec2 cd = Force::GenerateSpringForce(*particles[2], *particles[3], restLength, k); // c <-> d
        // particles[2]->AddForce(cd);
        // particles[3]->AddForce(-cd);

        // Vec2 da = Force::GenerateSpringForce(*particles[3], *particles[0], restLength, k); // d <-> a
        // particles[3]->AddForce(da);
        // particles[0]->AddForce(-da);

        // Vec2 ac = Force::GenerateSpringForce(*particles[0], *particles[2], restLength, k); // a <-> c
        // particles[0]->AddForce(ac);
        // particles[2]->AddForce(-ac);

        // Vec2 bd = Force::GenerateSpringForce(*particles[1], *particles[3], restLength, k); // b <-> d
        // particles[1]->AddForce(bd);
        // particles[3]->AddForce(-bd);

        ///////////////////////////////////////////////////////////////////////////////
        // Particle chain with spring force
        ///////////////////////////////////////////////////////////////////////////////
        // Attach the head to the anchor with a spring
        // Vec2 springForce = Force::GenerateSpringForce(*particles[0], anchor, restLength, k);
        // particles[0]->AddForce(springForce);
        const springForce = Force.generateSpringForceParticleAnchor(
            this.particles[0],
            this.anchor,
            this.restLength,
            this.k,
        );
        this.particles[0].addForce(springForce);

        // Connect the particles with the one before in a chain of springs
        for (let i = 1; i < this.NUM_PARTICLES; i++) {
            const currParticle = i;
            const prevParticle = i - 1;
            const springForce = Force.generateSpringForceParticleParticle(
                this.particles[currParticle],
                this.particles[prevParticle],
                this.restLength,
                this.k,
            );
            this.particles[currParticle].addForce(springForce);
            this.particles[prevParticle].addForce(springForce.negate());
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
