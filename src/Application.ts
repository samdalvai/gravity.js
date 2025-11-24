import Graphics from './Graphics';
import InputManager from './InputManager';
import Body from './physics/Body';
import CollisionDetection from './physics/CollisionDetection';
import { PIXELS_PER_METER } from './physics/Constants';
import Contact from './physics/Contact';
import Force from './physics/Force';
import { BoxShape, CircleShape, ShapeType } from './physics/Shape';
import Vec2 from './physics/Vec2';

export default class Application {
    private running: boolean;
    private bodies: Body[];

    constructor() {
        this.running = false;
        this.bodies = [];
    }

    isRunning = (): boolean => {
        return this.running;
    };

    setup = (): void => {
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
        floor.restitution = 0.2;
        leftWall.restitution = 0.2;
        rightWall.restitution = 0.2;
        this.bodies.push(floor);
        this.bodies.push(leftWall);
        this.bodies.push(rightWall);

        // Add a static box so other boxes can collide
        const bigBox = new Body(new BoxShape(200, 200), Graphics.width() / 2.0, Graphics.height() / 2.0, 0.0);
        bigBox.restitution = 0.7;
        bigBox.rotation = 1.4;
        this.bodies.push(bigBox);

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
                    {
                        // const box = new Body(new BoxShape(50, 50), inputEvent.x, inputEvent.y, 1.0);
                        // box.restitution = 0.2;
                        // this.bodies.push(box);

                        const ball = new Body(new CircleShape(30), inputEvent.x, inputEvent.y, 1.0);
                        ball.restitution = 0.5;
                        ball.friction = 0.4;
                        this.bodies.push(ball);
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

        // Apply forces to the bodies
        for (const body of this.bodies) {
            // Apply a drag force
            // const drag = Force.generateDragForce(body, 0.003);
            // body.addForce(drag);

            // Apply the weight force
            const weight = new Vec2(0.0, body.mass * 9.8 * PIXELS_PER_METER);
            body.addForce(weight);

            // Apply the wind force
            // const wind = new Vec2(2.0 * PIXELS_PER_METER, 0.0);
            // body.addForce(wind);
        }

        // // Integrate the acceleration and velocity to estimate the new position
        for (const body of this.bodies) {
            body.update(deltaTime);
        }

        // Check all the rigidbodies with the other rigidbodies for collision
        for (let i = 0; i <= this.bodies.length - 1; i++) {
            for (let j = i + 1; j < this.bodies.length; j++) {
                const a = this.bodies[i];
                const b = this.bodies[j];
                a.isColliding = false;
                b.isColliding = false;
                const contact = new Contact();
                if (CollisionDetection.isColliding(a, b, contact)) {
                    // Resolve the collision using the impulse method
                    contact.resolveCollision();

                    if (!contact.start || !contact.end || !contact.normal) {
                        console.error('Could not determine Contact information: ', contact);
                        throw new Error('Could not determine Contact information');
                    }

                    // TODO: this debug rendering is wrong, screen is cleared afterwards,
                    // we are mixing rendering with update
                    // Draw debug contact information
                    Graphics.drawFillCircle(contact.start.x, contact.start.y, 3, 'red');
                    Graphics.drawFillCircle(contact.end.x, contact.end.y, 3, 'red');
                    Graphics.drawLine(
                        contact.start.x,
                        contact.start.y,
                        contact.start.x + contact.normal.x * 15,
                        contact.start.y + contact.normal.y * 15,
                        'red',
                    );
                    a.isColliding = true;
                    b.isColliding = true;
                }
            }
        }
    };

    render = (): void => {
        // Draw all bodies
        for (const body of this.bodies) {
            const color = body.isColliding ? 'red' : 'white';

            if (body.shape.getType() === ShapeType.CIRCLE) {
                const circleShape = body.shape as CircleShape;
                Graphics.drawCircle(body.position.x, body.position.y, circleShape.radius, body.rotation, color);
            }

            if (body.shape.getType() === ShapeType.BOX) {
                const boxShape = body.shape as BoxShape;
                Graphics.drawPolygon(body.position.x, body.position.y, boxShape.worldVertices, color);
            }
        }
    };

    sleep = (milliseconds: number) => {
        return new Promise(resolve => setTimeout(resolve, milliseconds));
    };
}
