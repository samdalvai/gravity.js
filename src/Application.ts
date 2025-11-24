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

        const bigBall = new Body(new CircleShape(200), Graphics.width() / 2, Graphics.height() / 2, 0);
        this.bodies.push(bigBall);

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
                        const smallBall = new Body(new CircleShape(40), inputEvent.x, inputEvent.y, 1);
                        smallBall.restitution = 0.2;
                        this.bodies.push(smallBall);
                    }
                    break;
                case 'mouseup':
                    break;
            }
        }
    };

    update = (deltaTime: number): void => {
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

        // Check the boundaries of the window applying a hardcoded bounce flip in velocity
        for (const body of this.bodies) {
            if (body.shape.getType() === ShapeType.CIRCLE) {
                const circleShape = body.shape as CircleShape;
                if (body.position.x - circleShape.radius <= 0) {
                    body.position.x = circleShape.radius;
                    body.velocity.x *= -0.9;
                } else if (body.position.x + circleShape.radius >= Graphics.width()) {
                    body.position.x = Graphics.width() - circleShape.radius;
                    body.velocity.x *= -0.9;
                }
                if (body.position.y - circleShape.radius <= 0) {
                    body.position.y = circleShape.radius;
                    body.velocity.y *= -0.9;
                } else if (body.position.y + circleShape.radius >= Graphics.height()) {
                    body.position.y = Graphics.height() - circleShape.radius;
                    body.velocity.y *= -0.9;
                }
            }
        }
    };

    render = (): void => {
        Graphics.clearScreen();

        // Draw all bodies
        for (const body of this.bodies) {
            // const color = body.isColliding ? 'red' : 'white';

            if (body.shape.getType() === ShapeType.CIRCLE) {
                const circleShape = body.shape as CircleShape;
                Graphics.drawFillCircle(body.position.x, body.position.y, circleShape.radius, 'white');
            }

            if (body.shape.getType() === ShapeType.BOX) {
                const boxShape = body.shape as BoxShape;
                // TODO: to be implemented
                // Graphics.drawPolygon(body.position.x, body.position.y, boxShape.worldVertices, "white");
            }
        }
    };

    sleep = (milliseconds: number) => {
        return new Promise(resolve => setTimeout(resolve, milliseconds));
    };
}
