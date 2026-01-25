import AssetStore from './AssetStore';
import Graphics from './Graphics';
import InputManager, { MouseButton } from './InputManager';
import Vec2 from './math/Vec2';
import { GRAVITY, MAX_BODIES } from './physics/Constants';
import { DistanceJoint } from './physics/DistanceJoint';
import Force from './physics/Force';
import RigidBody from './physics/RigidBody';
import { BoxShape, CircleShape } from './physics/Shape';
import World from './physics/World';
import Demo from './samples/Demo';

export default class Application {
    private running = false;
    private world: World;
    private bgTexture: ImageBitmap | null = null;
    private generateParticle = false;
    private generateCircles = true;
    private showContacts = true;
    private showAABB = false;
    private demoIndex = 1;
    private testBody: RigidBody | null = null;

    // Debug related properties
    private debug = true;
    private FPS = 0;
    private lastFPSUpdate = 0;

    constructor() {
        this.world = new World(GRAVITY);
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
        const demo = Demo.demoFunctions[this.demoIndex];
        this.world.clear();
        demo(this.world);

        // Test for collision, if you enable this you need to skip static objects from constraint solve
        // otherwise determinant becomes 0
        // const b = new RigidBody(new BoxShape(100, 100), 0, 0, 0);
        // const b = new RigidBody(new CircleShape(50), 0, 0, 0);
        // b.rotation = 0.5;
        // this.world.addBody(b);

        // this.testBody = new RigidBody(new CircleShape(25), 0, 0, 0);
        // this.testBody = new RigidBody(new BoxShape(50, 50), 0, 0, 0);
        // this.world.addBody(this.testBody);

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
                            const x = InputManager.mousePosition.x - Graphics.width() / 2;
                            const y = -(InputManager.mousePosition.y - Graphics.height() / 2);
                            const explosionPos = new Vec2(x, y);
                            const radius = 250;
                            const strength = 10000;

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

                    if (inputEvent.key === 'g') {
                        this.generateParticle = true;
                    }

                    if (inputEvent.key === 'c') {
                        this.generateCircles = !this.generateCircles;
                    }

                    if (inputEvent.key === 'a') {
                        this.showAABB = !this.showAABB;
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
                const x = InputManager.mousePosition.x - Graphics.width() / 2;
                const y = -(InputManager.mousePosition.y - Graphics.height() / 2);
                this.testBody.position.x = x;
                this.testBody.position.y = y;
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
                    {
                        if (this.world.getBodies().length >= MAX_BODIES) {
                            continue;
                        }

                        const x = InputManager.mousePosition.x - Graphics.width() / 2;
                        const y = -(InputManager.mousePosition.y - Graphics.height() / 2);

                        switch (inputEvent.button) {
                            case MouseButton.LEFT:
                                {
                                    const ball = new RigidBody(new CircleShape(30), x, y, 4.0);
                                    ball.restitution = 0.2;
                                    ball.friction = 0.5;
                                    ball.setTexture('basketball');
                                    this.world.addBody(ball);
                                }
                                break;
                            case MouseButton.RIGHT:
                                {
                                    const box = new RigidBody(new BoxShape(60, 60), x, y, 6.0);
                                    box.restitution = 0.3;
                                    box.friction = 0.7;
                                    box.setTexture('crate');
                                    this.world.addBody(box);
                                }
                                break;
                        }
                    }
                    break;
                case 'mouseup':
                    break;
            }
        }
    };

    update = (frameTime: number): void => {
        if (this.debug) {
            if (!this.lastFPSUpdate || performance.now() - this.lastFPSUpdate > 1000) {
                this.lastFPSUpdate = performance.now();
                this.FPS = 1 / frameTime;
            }
        }

        this.world.update();

        if (this.generateParticle) {
            const x = InputManager.mousePosition.x - Graphics.width() / 2;
            const y = -(InputManager.mousePosition.y - Graphics.height() / 2);
            for (let i = 0; i < 10; i++) {
                if (this.world.getBodies().length >= MAX_BODIES) {
                    continue;
                }

                const shape = this.generateCircles ? new CircleShape(5) : new BoxShape(10, 10);
                const particle = new RigidBody(shape, x, y, 0.01);
                particle.restitution = 0.0;
                particle.friction = 0.5;
                particle.setTexture(this.generateCircles ? 'rockRound' : 'rockBox');
                this.world.addBody(particle);
            }
        }
    };

    render = (): void => {
        Graphics.clearScreen();
        Graphics.beginWorld();

        // Draw background texture
        if (this.bgTexture && !this.debug) {
            Graphics.drawTexture(0, 0, Graphics.width(), Graphics.height(), 0.0, this.bgTexture);
        }

        // Draw all bodies
        for (const body of this.world.getBodies()) {
            Graphics.drawBody(body, this.debug);
        }

        // Draw all joints
        for (const joint of this.world.getJoints()) {
            if (joint instanceof DistanceJoint) {
                const anchorA = joint.localAnchorA;
                const anchorB = joint.localAnchorB;
                const worldA = joint.bodyA.localPointToWorld(anchorA);
                const worldB = joint.bodyB.localPointToWorld(anchorB);

                if (joint.drawAnchor) {
                    Graphics.drawFillCircle(worldA.x, worldA.y, 5, 'white');
                    Graphics.drawFillCircle(worldB.x, worldB.y, 5, 'white');
                }

                if (joint.drawConnectionLine) {
                    Graphics.drawLine(worldA.x, worldA.y, worldB.x, worldB.y, 'white');
                }
            }
        }

        // Draw all joints anchor points and debug properties
        if (this.debug) {
            if (this.showAABB) {
                for (const body of this.world.getBodies()) {
                    const centerX = body.minX + (body.maxX - body.minX) / 2;
                    const centerY = body.minY + (body.maxY - body.minY) / 2;
                    const width = body.maxX - body.minX;
                    const height = body.maxY - body.minY;
                    Graphics.drawRect(centerX, centerY, width, height, 'gray');
                }
            }

            if (this.showContacts) {
                for (const joint of this.world.getJoints()) {
                    if (joint instanceof DistanceJoint) {
                        const anchorA = joint.localAnchorA;
                        const anchorB = joint.localAnchorB;
                        const worldA = joint.bodyA.localPointToWorld(anchorA);
                        const worldB = joint.bodyB.localPointToWorld(anchorB);
                        Graphics.drawFillCircle(worldA.x, worldA.y, 5, 'blue');
                        Graphics.drawFillCircle(worldB.x, worldB.y, 5, 'blue');

                        Graphics.drawLine(worldA.x, worldA.y, worldB.x, worldB.y, 'blue');
                    }
                }
                for (const manifold of this.world.getManifolds()) {
                    for (const contact of manifold.contactPoints) {
                        const startPoint = contact.point;
                        const endPoint = contact.point.subNew(
                            manifold.contactNormal.scaleNew(manifold.penetrationDepth),
                        );

                        Graphics.drawFillCircle(startPoint.x, startPoint.y, 5, 'red');
                        Graphics.drawFillCircle(endPoint.x, endPoint.y, 3, 'red');
                        Graphics.drawLine(startPoint.x, startPoint.y, endPoint.x, endPoint.y, 'red');
                    }
                }
            }
        }

        Graphics.endWorld();

        let numContacts = 0;

        for (const manifold of this.world.getManifolds()) {
            numContacts += manifold.numContacts;
        }

        const defaultText = [
            // General info
            'Keys: 1-9 Demos, Left Mouse to generate circles, Right Mouse to generate boxes, Space to drop bomb',
            `${Demo.demoStrings[this.demoIndex]}`,
            `(D) debug mode: ${this.debug ? 'ON' : 'OFF'}`,
            `(C) chosen particle: ${this.generateCircles ? 'Circle' : 'Box'}`,
        ];

        const x = InputManager.mousePosition.x - Graphics.width() / 2;
        const y = -(InputManager.mousePosition.y - Graphics.height() / 2);

        const debugText = [
            // Debug related info
            `(A) show AABB: ${this.showAABB ? 'ON' : 'OFF'}`,
            `(S) show contacts and joints: ${this.showContacts ? 'ON' : 'OFF'}`,
            `FPS: ${this.FPS.toFixed(2)}`,
            `Mouse position: {${x}, ${y}}`,
            `Num objects: ${this.world.getBodies().length}`,
            `Num contacts: ${numContacts}`,
        ];

        const text = [...defaultText, ...(this.debug ? debugText : [])];

        for (let i = 0; i < text.length; i++) {
            Graphics.drawText(text[i], 50, 50 + i * 25, 18, 'arial', this.debug ? 'orange' : 'black');
        }
    };
}
