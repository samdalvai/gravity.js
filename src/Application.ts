import AssetStore from './AssetStore';
import Graphics from './Graphics';
import InputManager, { MouseButton } from './InputManager';
import Body from './physics/Body';
import { BoxShape, CircleShape, PolygonShape, ShapeType } from './physics/Shape';
import World from './physics/World';

export default class Application {
    private running = false;
    private debug = false;
    private world: World;

    constructor() {
        this.world = new World(-9.8);
    }

    isRunning = (): boolean => {
        return this.running;
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

        // Add a big static circle in the middle of the screen
        const bigBall = new Body(new CircleShape(64), Graphics.width() / 2.0, Graphics.height() / 2.0, 0.0);
        bigBall.setTexture('bowlingball');
        this.world.addBody(bigBall);

        // Add a floor and walls to contain objects
        const floor = new Body(
            new BoxShape(Graphics.width() - 50, 50),
            Graphics.width() / 2.0,
            Graphics.height() - 50,
            0.0,
        );
        const leftWall = new Body(new BoxShape(50, Graphics.height() - 100), 50, Graphics.height() / 2.0 - 25, 0.0);
        const rightWall = new Body(
            new BoxShape(50, Graphics.height() - 100),
            Graphics.width() - 50,
            Graphics.height() / 2.0 - 25,
            0.0,
        );
        floor.restitution = 0.7;
        leftWall.restitution = 0.2;
        rightWall.restitution = 0.2;
        this.world.addBody(floor);
        this.world.addBody(leftWall);
        this.world.addBody(rightWall);
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

                    break;
                case 'keyup':
                    break;
            }
        }

        // Handle mouse move events
        while (InputManager.mouseMoveBuffer.length > 0) {
            const inputEvent = InputManager.mouseMoveBuffer.shift();

            if (!inputEvent) {
                return;
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
                    switch (inputEvent.button) {
                        case MouseButton.LEFT:
                            {
                                const ball = new Body(new CircleShape(30), inputEvent.x, inputEvent.y, 1.0);
                                ball.restitution = 0.7;
                                ball.friction = 0.4;
                                ball.setTexture('basketball');
                                this.world.addBody(ball);
                            }
                            break;
                        case MouseButton.RIGHT:
                            {
                                const box = new Body(new BoxShape(60, 60), inputEvent.x, inputEvent.y, 1.0);
                                box.restitution = 0.2;
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
        // TODO: not the correct place to clear screen
        Graphics.clearScreen();

        this.world.update(deltaTime);
    };

    render = (): void => {
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
                        } else {
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
                            Graphics.drawFillPolygon(
                                body.position.x,
                                body.position.y,
                                polygonShape.worldVertices,
                                'white',
                            );
                        } else {
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
                        } else {
                            Graphics.drawPolygon(body.position.x, body.position.y, boxShape.worldVertices, 'white');
                        }
                    }
                    break;
            }
        }
    };

    sleep = (milliseconds: number) => {
        return new Promise(resolve => setTimeout(resolve, milliseconds));
    };
}
