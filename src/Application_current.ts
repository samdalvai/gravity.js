import AssetStore from './AssetStore';
import Graphics from './Graphics_old';
import InputManager, { MouseButton } from './InputManager';
import Utils from './math/Utils';
import Vec2 from './math/Vec2';
import Body from './physics/Body';
import { GRAVITY, MAX_BODIES } from './physics/Constants';
import Force from './physics/Force';
import { BoxShape, CircleShape } from './physics/Shape';
import World from './physics/World_current';
import Demo from './samples/Demo_current';

export default class Application {
    private running = false;
    private world: World;
    private bgTexture: ImageBitmap | null = null;
    private generateParticle = false;
    private generateCircles = true;
    private showContacts = true;
    private demoIndex = 1;
    private bomb: Body | null = null;
    private testBody: Body | null = null;

    // Debug related properties
    private debug = true;
    private FPS = 0;
    private lastFPSUpdate = 0;

    constructor() {
        this.world = new World(-GRAVITY);
    }

    isRunning = (): boolean => {
        return this.running;
    };

    setRunning = (newValue: boolean): void => {
        this.running = newValue;
    };

    setup = async (): Promise<void> => {
        InputManager.initialize();

        await AssetStore.loadTextures();

        this.running = Graphics.openWindow();
        Demo.demo1(this.world);

        // const b = new Body(new BoxShape(100, 100), Graphics.width() / 2, 500, 0);
        const b = new Body(new CircleShape(50), Graphics.width() / 2, 500, 0);
        b.rotation = 0.5;
        this.world.addBody(b);

        this.testBody = new Body(new CircleShape(25), Graphics.width() / 2, 500, 0);
        // this.testBody = new Body(new BoxShape(50, 50), Graphics.width() / 2, 500, 0);
        this.world.addBody(this.testBody);

        this.bgTexture = AssetStore.getTexture('background');
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
                    if (inputEvent.key === 'd') {
                        this.debug = !this.debug;
                    }

                    if (inputEvent.key === 'e') {
                        {
                            const explosionPos = InputManager.mousePosition;
                            const radius = 250; // pixels
                            const strength = 10000; // you can tune this

                            for (const body of this.world.getBodies()) {
                                const explosionImpulse = Force.generateExplosionForce(
                                    body,
                                    explosionPos,
                                    radius,
                                    strength,
                                );
                                body.applyImpulseLinear(explosionImpulse);
                            }
                        }
                    }

                    if (inputEvent.code === 'Space') {
                        // Drop bomb
                        if (!this.bomb) {
                            const bomb = new Body(new CircleShape(30), Graphics.width() / 2, 0, 10);
                            bomb.friction = 0.2;
                            this.bomb = bomb;
                            this.bomb.setTexture('rockRound');

                            this.world.addBody(bomb);
                        }

                        this.bomb.position = new Vec2(Graphics.width() / 2 + Utils.randomNumber(-500, 500), -50);
                        const middleFloor = new Vec2(
                            Graphics.width() / 2 + Utils.randomNumber(-100, 100),
                            Graphics.height() - 300,
                        );
                        const vectorFromBombToFloor = middleFloor.subNew(this.bomb.position);
                        this.bomb.velocity = vectorFromBombToFloor;
                    }

                    if (inputEvent.key === 'g') {
                        this.generateParticle = true;
                    }

                    if (inputEvent.key === 'c') {
                        this.generateCircles = !this.generateCircles;
                    }

                    if (inputEvent.key === 's') {
                        this.showContacts = !this.showContacts;
                    }

                    if (!Number.isNaN(Number.parseInt(inputEvent.key))) {
                        const index = Number.parseInt(inputEvent.key);
                        this.demoIndex = index;
                        const demo = Demo.demoFunctions[this.demoIndex];

                        if (!demo) {
                            throw new Error(`Demo ${index} does not exist`);
                        }

                        this.world.clear();
                        this.bomb = null;
                        demo(this.world);
                    }

                    break;
                case 'keyup':
                    if (inputEvent.key === 'g') {
                        this.generateParticle = false;
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

            // Test for body collision
            if (this.testBody) {
                this.testBody.position.x = inputEvent.x;
                this.testBody.position.y = inputEvent.y;
            }
        }

        // Handle mouse click events
        while (InputManager.mouseInputBuffer.length > 0) {
            const inputEvent = InputManager.mouseInputBuffer.shift();

            if (!inputEvent) {
                return;
            }

            switch (inputEvent.type) {
                case 'mousedown':
                    if (this.world.getBodies().length >= MAX_BODIES) {
                        continue;
                    }

                    switch (inputEvent.button) {
                        case MouseButton.LEFT:
                            {
                                const ball = new Body(new CircleShape(30), inputEvent.x, inputEvent.y, 4.0);
                                ball.restitution = 0.6;
                                ball.friction = 0.5;
                                ball.setTexture('basketball');
                                this.world.addBody(ball);
                            }
                            break;
                        case MouseButton.RIGHT:
                            {
                                const box = new Body(new BoxShape(60, 60), inputEvent.x, inputEvent.y, 6.0);
                                box.restitution = 0.2;
                                box.friction = 0.7;
                                box.setTexture('crate');
                                this.world.addBody(box);
                            }
                            break;
                    }
                    break;
                case 'mouseup':
                    break;
            }
        }
    };

    update = (deltaTime: number): void => {
        if (this.debug) {
            if (!this.lastFPSUpdate || performance.now() - this.lastFPSUpdate > 1000) {
                this.lastFPSUpdate = performance.now();
                this.FPS = 1 / deltaTime;
            }
        }

        this.world.update(deltaTime);

        if (this.generateParticle) {
            for (let i = 0; i < 10; i++) {
                if (this.world.getBodies().length >= MAX_BODIES) {
                    continue;
                }

                const shape = this.generateCircles ? new CircleShape(5) : new BoxShape(10, 10);
                const particle = new Body(shape, InputManager.mousePosition.x, InputManager.mousePosition.y, 1.0);
                particle.restitution = 0.0;
                particle.friction = 10;
                particle.setTexture(this.generateCircles ? 'rockRound' : 'rockBox');
                this.world.addBody(particle);
            }
        }
    };

    render = (): void => {
        Graphics.clearScreen();

        // Draw background texture
        if (this.bgTexture && !this.debug) {
            Graphics.drawTexture(
                Graphics.width() / 2.0,
                Graphics.height() / 2.0,
                Graphics.width(),
                Graphics.height(),
                0.0,
                this.bgTexture,
            );
        }

        // Draw all bodies
        for (const body of this.world.getBodies()) {
            Graphics.drawBody(body, this.debug);
        }

        // Draw all joints anchor points and debug properties
        if (this.debug) {
            if (this.showContacts) {
                // for (const joint of this.world.getJoints()) {
                //     // TODO: this is just a simple draw method, we need to consider local anchors rather than just
                //     // bodies position
                //     Graphics.drawLine(
                //         joint.bodyA.position.x,
                //         joint.bodyA.position.y,
                //         joint.bodyB.position.x,
                //         joint.bodyB.position.y,
                //         'blue',
                //     );
                // }

                for (const contact of this.world.getContacts()) {
                    const aW = contact.bodyA.localPointToWorld(contact.aPointLocal);
                    const bW = contact.bodyB.localPointToWorld(contact.bPointLocal);
                    // const aW = contact.start;
                    // const bW = contact.end;
                    Graphics.drawFillCircle(aW.x, aW.y, 5, 'red');
                    Graphics.drawFillCircle(bW.x, bW.y, 2, 'red');
                    Graphics.drawLine(aW.x, aW.y, bW.x, bW.y, 'red');
                }
            }
        }

        const debugText = [
            // General info
            'Keys: 1-9 Demos, Left Mouse to generate circles, Right Mouse to generate boxes, Space to drop bomb',
            `${Demo.demoStrings[this.demoIndex]}`,
            `(D)ebug mode: ${this.debug ? 'ON' : 'OFF'}`,
            `(C)hosen particle: ${this.generateCircles ? 'Circle' : 'Box'}`,
            `(S)how contacts and joints: ${this.showContacts ? 'ON' : 'OFF'}`,
            // Debut related info
            `FPS: ${this.FPS.toFixed(2)}`,
            `Num objects: ${this.world.getBodies().length}`,
            `Num contacts: ${this.world.getContacts().length}`,
        ];

        for (let i = 0; i < debugText.length - (this.debug ? 0 : 4); i++) {
            Graphics.drawText(debugText[i], 50, 50 + i * 25, 18, 'arial', this.debug ? 'orange' : 'black');
        }
    };
}
