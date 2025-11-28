import AssetStore from './AssetStore';
import Graphics from './Graphics';
import InputManager, { MouseButton } from './InputManager';
import Body from './physics/Body';
import CollisionDetection from './physics/CollisionDetection';
import { PIXELS_PER_METER } from './physics/Constants';
import Contact from './physics/Contact';
import { BoxShape, CircleShape, ShapeType } from './physics/Shape';
import Vec2 from './physics/Vec2';
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
        };

        await AssetStore.loadTextures(textures);

        this.running = Graphics.openWindow();

        // Add a floor and walls to contain objects objects
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
        floor.restitution = 0.5;
        leftWall.restitution = 0.2;
        rightWall.restitution = 0.2;
        this.world.addBody(floor);
        this.world.addBody(leftWall);
        this.world.addBody(rightWall);

        // Add a static box so other boxes can collide
        const bigBox = new Body(new BoxShape(200, 200), Graphics.width() / 2.0, Graphics.height() / 2.0, 0.0);
        bigBox.restitution = 0.7;
        bigBox.rotation = 1.4;
        this.world.addBody(bigBox);

        // Add a force to all world objects
        const wind = new Vec2(0.5 * PIXELS_PER_METER, 0.0);
        this.world.addForce(wind);
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
                                ball.restitution = 0.5;
                                ball.friction = 0.4;
                                this.world.addBody(ball);
                            }
                            break;
                        case MouseButton.RIGHT:
                            {
                                const box = new Body(new BoxShape(60, 60), inputEvent.x, inputEvent.y, 1.0);
                                box.restitution = 0.2;
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
            // const color = body.isColliding ? 'red' : 'white';
            if (body.shape.getType() === ShapeType.CIRCLE) {
                const circleShape = body.shape as CircleShape;
                // Graphics.drawCircle(body.position.x, body.position.y, circleShape.radius, body.rotation, color);
                // Graphics.drawFillCircle(body.position.x, body.position.y, circleShape.radius, color);
                Graphics.drawTexture(
                    body.position.x,
                    body.position.y,
                    circleShape.radius * 2,
                    circleShape.radius * 2,
                    body.rotation,
                    AssetStore.getTexture('basketball'),
                );
            }
            if (body.shape.getType() === ShapeType.BOX) {
                const boxShape = body.shape as BoxShape;
                // Graphics.drawPolygon(body.position.x, body.position.y, boxShape.worldVertices, color);
                // Graphics.drawFillPolygon(body.position.x, body.position.y, boxShape.worldVertices, color);
                Graphics.drawTexture(
                    body.position.x,
                    body.position.y,
                    boxShape.width,
                    boxShape.height,
                    body.rotation,
                    AssetStore.getTexture('crate'),
                );
            }
        }
    };

    sleep = (milliseconds: number) => {
        return new Promise(resolve => setTimeout(resolve, milliseconds));
    };
}
