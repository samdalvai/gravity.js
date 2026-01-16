import AssetStore from './AssetStore';
import Graphics from './Graphics';
import InputManager, { MouseButton } from './InputManager';
import Vec2 from './math/Vec2';
import { Box } from './new/box';
import { Circle } from './new/circle';
import { Vector2 } from './new/math/vector2';
import { Type } from './new/rigidbody';
import Body from './physics/Body';
import { GRAVITY, MAX_BODIES } from './physics/Constants';
import Force from './physics/Force';
import { BoxShape, CircleShape } from './physics/Shape';
import World from './physics/World';
import Demo from './samples/Demo';

export default class Application {
    private running = false;
    private world: World;
    private bgTexture: ImageBitmap | null = null;
    // private generateParticle = false;
    // private generateCircles = true;
    // private showContacts = true;
    // private demoIndex = 1;
    // private bomb: Body | null = null;

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
        // Demo.demo1(this.world);

        this.bgTexture = AssetStore.getTexture('background');

        const ground = new Box(Graphics.width(), 50, Type.Static);
        ground.position.y = -350;
        ground.restitution = 0.45;

        // const b = new Box(50);
        // b.mass = 2.0;
        // b.position = new Vector2(100, 0);
        // b.restitution = 0.7;
        // b.rotation = 1;
        // // b.angularVelocity = random(-8, 8);
        // b.angularVelocity = 0;

        // const c = new Circle(25);
        // c.mass = 10;
        // c.position = new Vector2(200, 0);
        // c.restitution = 0.5;
        // c.rotation = 2;
        // c.angularVelocity = 5;

        this.world.register(ground);
        // this.world.register(b);
        // this.world.register(c);

        const rows = 14;
        const boxSize = 50;
        const xGap = 0.5;
        const yGap = 1.5;
        const xStart = (-(rows - 1) * (boxSize + xGap)) / 2.0;
        const yStart = -250;

        for (let y = 0; y < rows; y++) {
            for (let x = 0; x < rows - y; x++) {
                const b = new Box(boxSize);
                b.mass = 0.1;
                b.position = new Vector2(
                    xStart + (y * (boxSize + xGap)) / 2 + x * (boxSize + xGap),
                    yStart + y * (boxSize + yGap),
                );
                b.restitution = 0.0;
                this.world.register(b);
            }
        }
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

                            // for (const body of this.world.getBodies()) {
                            //     const explosionImpulse = Force.generateExplosionForce(
                            //         body,
                            //         explosionPos,
                            //         radius,
                            //         strength,
                            //     );
                            //     body.applyImpulseLinear(explosionImpulse);
                            // }
                        }
                    }

                    if (inputEvent.code === 'Space') {
                        // Drop bomb
                        // if (!this.bomb) {
                        //     const bomb = new Body(new CircleShape(30), Graphics.width() / 2, 0, 10);
                        //     bomb.friction = 0.2;
                        //     this.bomb = bomb;
                        //     this.bomb.setTexture('rockRound');
                        //     this.world.addBody(bomb);
                        // }
                        // this.bomb.position = new Vec2(Graphics.width() / 2 + Utils.randomNumber(-500, 500), -50);
                        // const middleFloor = new Vec2(
                        //     Graphics.width() / 2 + Utils.randomNumber(-100, 100),
                        //     Graphics.height() - 300,
                        // );
                        // const vectorFromBombToFloor = middleFloor.subNew(this.bomb.position);
                        // this.bomb.velocity = vectorFromBombToFloor;
                    }

                    // if (inputEvent.key === 'g') {
                    //     this.generateParticle = true;
                    // }

                    // if (inputEvent.key === 'c') {
                    //     this.generateCircles = !this.generateCircles;
                    // }

                    // if (inputEvent.key === 's') {
                    //     this.showContacts = !this.showContacts;
                    // }

                    // if (!Number.isNaN(Number.parseInt(inputEvent.key))) {
                    //     const index = Number.parseInt(inputEvent.key);
                    //     this.demoIndex = index;
                    //     const demo = Demo.demoFunctions[this.demoIndex];

                    //     if (!demo) {
                    //         throw new Error(`Demo ${index} does not exist`);
                    //     }

                    //     this.world.clear();
                    //     this.bomb = null;
                    //     demo(this.world);
                    // }

                    break;
                case 'keyup':
                    // if (inputEvent.key === 'g') {
                    //     this.generateParticle = false;
                    // }

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
            // this.world.getBodies()[4].position.x = inputEvent.x;
            // this.world.getBodies()[4].position.y = inputEvent.y;
            // this.world.getBodies()[4].shape.updateVertices(0, this.world.getBodies()[4].position);
        }

        // Handle mouse click events
        while (InputManager.mouseInputBuffer.length > 0) {
            const inputEvent = InputManager.mouseInputBuffer.shift();

            if (!inputEvent) {
                return;
            }

            switch (inputEvent.type) {
                case 'mousedown': {
                    // if (this.world.getBodies().length >= MAX_BODIES) {
                    //     continue;
                    // }
                    const x = inputEvent.x - Graphics.width() / 2;
                    const y = -(inputEvent.y - Graphics.height() / 2);

                    switch (inputEvent.button) {
                        case MouseButton.LEFT:
                            {
                                const ball = new Circle(25);
                                ball.position = new Vector2(x, y);
                                ball.restitution = 0.6;
                                ball.mass = 2.0;
                                this.world.register(ball);
                            }
                            break;
                        case MouseButton.RIGHT:
                            {
                                const box = new Box(50);
                                box.position = new Vector2(x, y);
                                box.mass = 2.0;
                                box.restitution = 0.7;
                                this.world.register(box);
                            }
                            break;
                    }
                    break;
                }
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

        // if (this.generateParticle) {
        //     for (let i = 0; i < 10; i++) {
        //         if (this.world.getBodies().length >= MAX_BODIES) {
        //             continue;
        //         }

        //         const shape = this.generateCircles ? new CircleShape(5) : new BoxShape(10, 10);
        //         const particle = new Body(shape, InputManager.mousePosition.x, InputManager.mousePosition.y, 1.0);
        //         particle.restitution = 0.0;
        //         particle.friction = 10;
        //         particle.setTexture(this.generateCircles ? 'rockRound' : 'rockBox');
        //         this.world.addBody(particle);
        //     }
        // }
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

        Graphics.beginWorld();

        // Draw all bodies
        // for (const body of this.world.getBodies()) {
        //     Graphics.drawBody(body, this.debug);
        // }
        for (const body of this.world.bodies) {
            // console.log("Body: ", body);
            Graphics.drawBody(body);
        }

        // Draw all joints anchor points and debug properties
        if (this.debug) {
            // if (this.showContacts) {
            //     for (const joint of this.world.getJoints()) {
            //         // TODO: this is just a simple draw method, we need to consider local anchors rather than just
            //         // bodies position
            //         Graphics.drawLine(
            //             joint.bodyA.position.x,
            //             joint.bodyA.position.y,
            //             joint.bodyB.position.x,
            //             joint.bodyB.position.y,
            //             'blue',
            //         );
            //     }
            //     for (const contact of this.world.getContacts()) {
            //         const aW = contact.bodyA.localSpaceToWorldSpace(contact.aPointLocal);
            //         const bW = contact.bodyB.localSpaceToWorldSpace(contact.bPointLocal);
            //         // const aW = contact.start;
            //         // const bW = contact.end;
            //         Graphics.drawFillCircle(aW.x, aW.y, 5, 'red');
            //         Graphics.drawFillCircle(bW.x, bW.y, 2, 'red');
            //         Graphics.drawLine(aW.x, aW.y, bW.x, bW.y, 'red');
            //     }
            // }
        }

        Graphics.endWorld();

        const debugText = [
            // General info
            'Keys: 1-9 Demos, Left Mouse to generate circles, Right Mouse to generate boxes, Space to drop bomb',
            // `${Demo.demoStrings[this.demoIndex]}`,
            // `(D)ebug mode: ${this.debug ? 'ON' : 'OFF'}`,
            // `(C)hosen particle: ${this.generateCircles ? 'Circle' : 'Box'}`,
            // `(S)how contacts and joints: ${this.showContacts ? 'ON' : 'OFF'}`,
            // // Debut related info
            `FPS: ${this.FPS.toFixed(2)}`,
            // `Num objects: ${this.world.getBodies().length}`,
            // `Num contacts: ${this.world.getContacts().length}`,
        ];

        // for (let i = 0; i < debugText.length - (this.debug ? 0 : 4); i++) {
        for (let i = 0; i < debugText.length; i++) {
            Graphics.drawText(debugText[i], 50, 50 + i * 25, 18, 'arial', this.debug ? 'orange' : 'black');
        }
    };
}
