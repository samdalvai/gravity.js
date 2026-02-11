import AssetStore from './AssetStore';
import Graphics from './Graphics';
import InputManager, { MouseButton } from './InputManager';
import Utils from './math/Utils';
import Vec2 from './math/Vec2';
import { GRAVITY, MAX_BODIES } from './physics/Constants';
import { DistanceJoint } from './physics/DistanceJoint';
import Force from './physics/Force';
import RigidBody from './physics/RigidBody';
import { BoxShape, CapsuleShape, CircleShape } from './physics/Shape';
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
    private middleMousePressed = false;

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
            if (!inputEvent) return;

            switch (inputEvent.type) {
                case 'keydown':
                    if (inputEvent.key === 'd') {
                        this.debug = !this.debug;
                    }

                    if (inputEvent.key === 'e') {
                        {
                            const x = InputManager.mousePosition.x;
                            const y = InputManager.mousePosition.y;
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

                    if (inputEvent.key === 'x') {
                        if (this.world.getBodies().length >= MAX_BODIES) {
                            continue;
                        }

                        const x = InputManager.mousePosition.x;
                        const y = InputManager.mousePosition.y;

                        const capsule = new RigidBody(new CapsuleShape(40, 20), x, y, 1);
                        capsule.restitution = 0.2;
                        capsule.friction = 0.1;
                        // capsule.rotation = Utils.randomNumber(0, 1);
                        this.world.addBody(capsule);
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
            if (!inputEvent) return;

            if (this.middleMousePressed) {
                // Drag the camera opposite to mouse movement
                Graphics.pan.x -= inputEvent.movementX / Graphics.zoom;
                Graphics.pan.y += inputEvent.movementY / Graphics.zoom;
            }

            // Convert screen -> world coordinates
            const screenX = inputEvent.x - Graphics.width() / 2;
            const screenY = -(inputEvent.y - Graphics.height() / 2);

            // Adjust mouse world position with pan and zoom
            InputManager.mousePosition.x = screenX / Graphics.zoom + Graphics.pan.x;
            InputManager.mousePosition.y = screenY / Graphics.zoom + Graphics.pan.y;
        }

        // Handle mouse click events
        while (InputManager.mouseInputBuffer.length > 0) {
            const inputEvent = InputManager.mouseInputBuffer.shift();
            if (!inputEvent) return;

            switch (inputEvent.type) {
                case 'mousedown':
                    {
                        const x = InputManager.mousePosition.x;
                        const y = InputManager.mousePosition.y;

                        switch (inputEvent.button) {
                            case MouseButton.LEFT:
                                {
                                    if (this.world.getBodies().length >= MAX_BODIES) {
                                        continue;
                                    }
                                    const ball = new RigidBody(new CircleShape(30), x, y, 1.0);
                                    ball.restitution = 0.2;
                                    ball.friction = 0.1;
                                    ball.setTexture('basketball');
                                    this.world.addBody(ball);
                                }
                                break;
                            case MouseButton.RIGHT:
                                {
                                    if (this.world.getBodies().length >= MAX_BODIES) {
                                        continue;
                                    }
                                    const box = new RigidBody(new BoxShape(60, 60), x, y, 1.0);
                                    box.restitution = 0.3;
                                    box.friction = 0.7;
                                    box.setTexture('crate');
                                    this.world.addBody(box);
                                }
                                break;
                            case MouseButton.MIDDLE:
                                this.middleMousePressed = true;
                                {
                                    console.log('Mouse middle pressed');
                                }
                                break;
                        }
                    }
                    break;
                case 'mouseup':
                    switch (inputEvent.button) {
                        case MouseButton.MIDDLE:
                            this.middleMousePressed = false;
                            {
                                console.log('Mouse middle released');
                            }
                            break;
                    }
                    break;
            }
        }

        // Handle wheel events
        while (InputManager.mouseWheelBuffer.length > 0) {
            const inputEvent = InputManager.mouseWheelBuffer.shift() as WheelEvent;
            if (!inputEvent) return;

            if (inputEvent.deltaY > 0) {
                Graphics.decreaseZoom();
            } else {
                Graphics.increaseZoom();
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
            const x = InputManager.mousePosition.x;
            const y = InputManager.mousePosition.y;
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

                const color = this.debug ? 'white' : 'black';

                if (joint.drawAnchor) {
                    Graphics.drawFillCircle(worldA.x, worldA.y, 5, color);
                    Graphics.drawFillCircle(worldB.x, worldB.y, 5, color);
                }

                if (joint.drawConnectionLine) {
                    Graphics.drawLine(worldA.x, worldA.y, worldB.x, worldB.y, color);
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
                    Graphics.drawRect(centerX, centerY, width, height, 'pink');
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
            `${Demo.demoStrings[this.demoIndex]}`,
            '(1-9) select demo, Left Mouse to generate circles, Right Mouse to generate boxes',
            '(E) to generate explosion, (G) to generate particles, (X) to generate capsules',
            `(D) debug mode: ${this.debug ? 'ON' : 'OFF'}`,
            `(C) chosen particle: ${this.generateCircles ? 'Circle' : 'Box'}`,
        ];

        const x = InputManager.mousePosition.x;
        const y = InputManager.mousePosition.y;

        const debugText = [
            // Debug related info
            `(A) show AABB: ${this.showAABB ? 'ON' : 'OFF'}`,
            `(S) show contacts and joints: ${this.showContacts ? 'ON' : 'OFF'}`,
            `FPS: ${this.FPS.toFixed(2)}`,
            `Mouse position: {${x.toFixed(2)}, ${y.toFixed(2)}}`,
            `Zoom: ${Graphics.zoom.toFixed(2)}`,
            `Num objects: ${this.world.getBodies().length}`,
            `Num contacts: ${numContacts}`,
        ];

        const text = [...defaultText, ...(this.debug ? debugText : [])];

        for (let i = 0; i < text.length; i++) {
            Graphics.drawText(text[i], 50, 50 + i * 25, 18, 'arial', this.debug ? 'orange' : 'black');
        }
    };
}
