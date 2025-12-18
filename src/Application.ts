import AssetStore from './AssetStore';
import Graphics from './Graphics';
import InputManager, { MouseButton } from './InputManager';
import Body from './physics/Body';
import { JointConstraint } from './physics/Constraint';
import Force from './physics/Force';
import { BoxShape, CircleShape, PolygonShape, ShapeType } from './physics/Shape';
import Vec2 from './physics/Vec2';
import World from './physics/World';

export default class Application {
    private running = false;
    private world: World;
    private bgTexture: ImageBitmap | null = null;
    private generateParticle = false;

    // Debug related properties
    private debug = true;
    private FPS = 0;
    private lastFPSUpdate = 0;

    constructor() {
        this.world = new World(-9.8);
    }

    isRunning = (): boolean => {
        return this.running;
    };

    setRunning = (newValue: boolean): void => {
        this.running = newValue;
    };

    setup = async (): Promise<void> => {
        InputManager.initialize();

        const textures = {
            basketball: 'assets/basketball.png',
            bowlingball: 'assets/bowlingball.png',
            crate: 'assets/crate.png',
            metal: 'assets/metal.png',
            bob: 'assets/ragdoll/bob.png',
            head: 'assets/ragdoll/head.png',
            leftArm: 'assets/ragdoll/leftArm.png',
            leftLeg: 'assets/ragdoll/leftLeg.png',
            rightArm: 'assets/ragdoll/rightArm.png',
            rightLeg: 'assets/ragdoll/rightLeg.png',
            torso: 'assets/ragdoll/torso.png',
            background: 'assets/angrybirds/background.png',
            birdRed: 'assets/angrybirds/bird-red.png',
            pig1: 'assets/angrybirds/pig-1.png',
            pig2: 'assets/angrybirds/pig-2.png',
            rockBox: 'assets/angrybirds/rock-box.png',
            rockBridgeAnchor: 'assets/angrybirds/rock-bridge-anchor.png',
            rockRound: 'assets/angrybirds/rock-round.png',
            woodBox: 'assets/angrybirds/wood-box.png',
            woodBridgeStep: 'assets/angrybirds/wood-bridge-step.png',
            woodPlankCracked: 'assets/angrybirds/wood-plank-cracked.png',
            woodPlankSolid: 'assets/angrybirds/wood-plank-solid.png',
            woodTriangle: 'assets/angrybirds/wood-triangle.png',
        };

        await AssetStore.loadTextures(textures);

        this.running = Graphics.openWindow();

        // this.bgTexture = AssetStore.getTexture('background');

        // // Add bird
        // const bird = new Body(new CircleShape(45), 100, Graphics.height() / 2.0 + 220, 3.0);
        // bird.setTexture('birdRed');
        // this.world.addBody(bird);

        // Add a floor and walls to contain objects objects
        const floor = new Body(
            new BoxShape(Graphics.width() - 50, 50),
            Graphics.width() / 2.0,
            Graphics.height() / 2.0 + 340,
            0.0,
        );
        const leftFence = new Body(new BoxShape(50, Graphics.height() - 75), 0, Graphics.height() / 2.0 - 35, 0.0);
        const rightFence = new Body(
            new BoxShape(50, Graphics.height() - 75),
            Graphics.width(),
            Graphics.height() / 2.0 - 35,
            0.0,
        );
        this.world.addBody(floor);
        this.world.addBody(leftFence);
        this.world.addBody(rightFence);

        // // Add a stack of boxes
        // for (let i = 1; i <= 4; i++) {
        //     const mass = 10.0 / i;
        //     const box = new Body(new BoxShape(50, 50), 600, floor.position.y - i * 55, mass);
        //     box.setTexture('woodBox');
        //     box.friction = 0.9;
        //     box.restitution = 0.1;
        //     this.world.addBody(box);
        // }

        // // Add structure with blocks
        // const plank1 = new Body(new BoxShape(50, 150), Graphics.width() / 2.0 + 20, floor.position.y - 100, 5.0);
        // const plank2 = new Body(new BoxShape(50, 150), Graphics.width() / 2.0 + 180, floor.position.y - 100, 5.0);
        // const plank3 = new Body(new BoxShape(250, 25), Graphics.width() / 2.0 + 100, floor.position.y - 200, 2.0);
        // plank1.setTexture('woodPlankSolid');
        // plank2.setTexture('woodPlankSolid');
        // plank3.setTexture('woodPlankCracked');
        // this.world.addBody(plank1);
        // this.world.addBody(plank2);
        // this.world.addBody(plank3);

        // // Add a triangle polygon
        // const triangleVertices = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        // const triangle = new Body(new PolygonShape(triangleVertices), plank3.position.x, plank3.position.y - 50, 0.5);
        // triangle.setTexture('woodTriangle');
        // this.world.addBody(triangle);

        // // Add a pyramid of boxes
        // const numRows = 5;
        // for (let col = 0; col < numRows; col++) {
        //     for (let row = 0; row < col; row++) {
        //         const x = plank3.position.x + 200 + col * 50 - row * 25;
        //         const y = floor.position.y - 50 - row * 52;
        //         const mass = 5 / (row + 1);
        //         const box = new Body(new BoxShape(50, 50), x, y, mass);
        //         box.friction = 0.9;
        //         box.restitution = 0.0;
        //         box.setTexture('woodBox');
        //         this.world.addBody(box);
        //     }
        // }

        // Add a bridge of connected steps and joints
        const numSteps = 10;
        const spacing = 33;

        // Start anchor (static)
        const startStep = new Body(new BoxShape(80, 20), 200, 200, 0.0);
        startStep.setTexture('rockBridgeAnchor');
        this.world.addBody(startStep);

        // The first connection should be from the anchor, not the floor
        let last = startStep;

        for (let i = 1; i <= numSteps; i++) {
            const x = startStep.position.x + 30 + i * spacing;
            const y = startStep.position.y + 20;
            const mass = i == numSteps ? 0.0 : 3.0;

            const step = new Body(new CircleShape(15), x, y, mass);
            step.setTexture('woodBridgeStep');
            this.world.addBody(step);

            // Connect previous link to this link
            const joint = new JointConstraint(last, step, step.position);
            this.world.addConstraint(joint);

            last = step;
        }

        // Final anchor
        const endStep = new Body(new BoxShape(80, 20), last.position.x + 60, last.position.y - 20, 0.0);
        endStep.setTexture('rockBridgeAnchor');
        this.world.addBody(endStep);

        // // Connect last step to final anchor
        // const finalJoint = new JointConstraint(last, endStep, endStep.position);
        // this.world.addConstraint(finalJoint);

        // // Add pigs
        // const pig1 = new Body(new CircleShape(30), plank1.position.x + 80, floor.position.y - 50, 3.0);
        // const pig2 = new Body(new CircleShape(30), plank2.position.x + 400, floor.position.y - 50, 3.0);
        // const pig3 = new Body(new CircleShape(30), plank2.position.x + 460, floor.position.y - 50, 3.0);
        // const pig4 = new Body(new CircleShape(30), 220, 130, 1.0);
        // pig1.setTexture('pig1');
        // pig2.setTexture('pig2');
        // pig3.setTexture('pig1');
        // pig4.setTexture('pig2');
        // this.world.addBody(pig1);
        // this.world.addBody(pig2);
        // this.world.addBody(pig3);
        // this.world.addBody(pig4);

        // const triangleVertices = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        // const triangle1 = new Body(new PolygonShape(triangleVertices), Graphics.width() / 2, Graphics.height() / 2, 0);
        // const triangle2 = new Body(new PolygonShape(triangleVertices), Graphics.width() / 2, Graphics.height() / 2, 0);
        // this.world.addBody(triangle1);
        // this.world.addBody(triangle2);

        // const boxA = new Body(new BoxShape(60, 60), Graphics.width() / 2, Graphics.height() / 2, 0);
        // const boxB = new Body(new BoxShape(60, 60), Graphics.width() / 2, Graphics.height() / 2, 0);
        // this.world.addBody(boxA);
        // this.world.addBody(boxB);

        // const circleA = new Body(new CircleShape(30), Graphics.width() / 2, Graphics.height() / 2, 0);
        // const circleB = new Body(new CircleShape(30), Graphics.width() / 2, Graphics.height() / 2, 0);
        // this.world.addBody(circleA);
        // this.world.addBody(circleB);
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
                        this.world.setDebug(this.debug);
                    }

                    if (inputEvent.key === 'e') {
                        {
                            const explosionPos = InputManager.mousePosition;
                            const radius = 250; // pixels
                            const strength = 5000; // you can tune this

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

                    if (inputEvent.key === 'ArrowUp') {
                        this.world.getBodies()[0].applyImpulseLinear(new Vec2(0, -600));
                    }

                    if (inputEvent.key === 'ArrowLeft') {
                        this.world.getBodies()[0].applyImpulseLinear(new Vec2(-400, 0));
                    }

                    if (inputEvent.key === 'ArrowRight') {
                        this.world.getBodies()[0].applyImpulseLinear(new Vec2(400, 0));
                    }

                    if (inputEvent.key === 'ArrowDown') {
                        this.world.getBodies()[0].applyImpulseLinear(new Vec2(0, 600));
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
                case 'mousedown':
                    switch (inputEvent.button) {
                        case MouseButton.LEFT:
                            {
                                const ball = new Body(new CircleShape(30), inputEvent.x, inputEvent.y, 4.0);
                                ball.restitution = 0.2;
                                ball.friction = 10;
                                ball.setTexture('rockRound');
                                this.world.addBody(ball);
                            }
                            break;
                        case MouseButton.RIGHT:
                            {
                                const box = new Body(new BoxShape(60, 60), inputEvent.x, inputEvent.y, 6.0);
                                box.restitution = 0.5;
                                box.friction = 0.7;
                                box.setTexture('woodBox');
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
        Graphics.clearScreen();

        if (this.debug) {
            Graphics.drawText(`FPS: ${this.FPS.toFixed(2)}`, Graphics.width() - 100, 50, 25, 'arial', 'red');
            Graphics.drawText(
                `Num objects: ${this.world.getBodies().length}`,
                Graphics.width() - 120,
                75,
                25,
                'arial',
                'red',
            );

            if (!this.lastFPSUpdate || performance.now() - this.lastFPSUpdate > 1000) {
                this.lastFPSUpdate = performance.now();
                this.FPS = 1 / deltaTime;
            }
        }

        this.world.update(deltaTime);

        if (this.generateParticle) {
            const ball = new Body(new CircleShape(5), InputManager.mousePosition.x, InputManager.mousePosition.y, 1.0);
            ball.restitution = 0.2;
            ball.friction = 10;
            ball.setTexture('rockRound');
            this.world.addBody(ball);
        }
    };

    render = (): void => {
        // Draw all joints anchor points
        if (this.debug) {
            for (const joint of this.world.getConstraints()) {
                if (this.debug) {
                    const anchorPoint = joint.a.localSpaceToWorldSpace(joint.aPoint);
                    Graphics.drawFillCircle(anchorPoint.x, anchorPoint.y, 3, 'red');
                }
            }
        }

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
            switch (body.shape.getType()) {
                case ShapeType.CIRCLE:
                    {
                        const circleShape = body.shape as CircleShape;

                        if (!this.debug && body.texture) {
                            Graphics.drawTexture(
                                body.position.x,
                                body.position.y,
                                circleShape.radius * 2,
                                circleShape.radius * 2,
                                body.rotation,
                                body.texture,
                            );
                        } else if (this.debug) {
                            Graphics.drawCircle(
                                body.position.x,
                                body.position.y,
                                circleShape.radius,
                                body.rotation,
                                'white',
                            );
                        }
                    }
                    break;
                case ShapeType.POLYGON:
                    {
                        const polygonShape = body.shape as PolygonShape;
                        if (!this.debug && body.texture) {
                            Graphics.drawTexture(
                                body.position.x,
                                body.position.y,
                                polygonShape.width,
                                polygonShape.height,
                                body.rotation,
                                body.texture,
                            );
                        } else if (this.debug) {
                            Graphics.drawPolygon(body.position.x, body.position.y, polygonShape.worldVertices, 'white');
                        }
                    }
                    break;
                case ShapeType.BOX:
                    {
                        const boxShape = body.shape as BoxShape;

                        if (!this.debug && body.texture) {
                            Graphics.drawTexture(
                                body.position.x,
                                body.position.y,
                                boxShape.width,
                                boxShape.height,
                                body.rotation,
                                body.texture,
                            );
                        } else if (this.debug) {
                            Graphics.drawPolygon(body.position.x, body.position.y, boxShape.worldVertices, 'white');
                        }
                    }
                    break;
            }
        }
    };
}
