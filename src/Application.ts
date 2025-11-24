import Graphics from './Graphics';
import InputManager from './InputManager';
import Body from './physics/Body';
import { CircleShape } from './physics/Shape';

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
                    // int x, y;
                    // SDL_GetMouseState(&x, &y);
                    // Body* smallBall = new Body(CircleShape(40), x, y, 1.0);
                    // smallBall->restitution = 0.2;
                    // bodies.push_back(smallBall);
                    break;
                case 'mouseup':
                    break;
            }
        }
    };

    update = (deltaTime: number): void => {
        // Apply forces to the bodies
        // for (auto body: bodies) {
        // Apply a drag force
        // const drag = Force.generateDragForce(particle, 0.003);
        // particle.addForce(drag);
        
        //     // Apply the weight force
        //     Vec2 weight = Vec2(0.0, body->mass * 9.8 * PIXELS_PER_METER);
        //     body->AddForce(weight);

        //     // Apply the wind force
        //     Vec2 wind = Vec2(2.0 * PIXELS_PER_METER, 0.0);
        //     body->AddForce(wind);
        // }

        // // Integrate the acceleration and velocity to estimate the new position
        // for (auto body: bodies) {
        //     body->Update(deltaTime);
        // }

        // // Check all the rigidbodies with the other rigidbodies for collision
        // for (int i = 0; i <= bodies.size() - 1; i++) {
        //     for (int j = i + 1; j < bodies.size(); j++) {
        //         Body* a = bodies[i];
        //         Body* b = bodies[j];
        //         a->isColliding = false;
        //         b->isColliding = false;
                
        //         Contact contact;

        //         if (CollisionDetection::IsColliding(a, b, contact)) {
        //             // Resolve the collision using the impulse method
        //             contact.ResolveCollision();

        //             // Draw debug contact information
        //             Graphics::DrawFillCircle(contact.start.x, contact.start.y, 3, 0xFFFF00FF);
        //             Graphics::DrawFillCircle(contact.end.x, contact.end.y, 3, 0xFFFF00FF);
        //             Graphics::DrawLine(contact.start.x, contact.start.y, contact.start.x + contact.normal.x * 15, contact.start.y + contact.normal.y * 15, 0xFFFF00FF);
        //             a->isColliding = true;
        //             b->isColliding = true;
        //         }
        //     }
        // }

        // // Check the boundaries of the window applying a hardcoded bounce flip in velocity
        // for (auto body: bodies) {
        //     if (body->shape->GetType() == CIRCLE) {
        //         CircleShape* circleShape = (CircleShape*) body->shape;
        //         if (body->position.x - circleShape->radius <= 0) {
        //             body->position.x = circleShape->radius;
        //             body->velocity.x *= -0.9;
        //         } else if (body->position.x + circleShape->radius >= Graphics::Width()) {
        //             body->position.x = Graphics::Width() - circleShape->radius;
        //             body->velocity.x *= -0.9;
        //         }
        //         if (body->position.y - circleShape->radius <= 0) {
        //             body->position.y = circleShape->radius;
        //             body->velocity.y *= -0.9;
        //         } else if (body->position.y + circleShape->radius >= Graphics::Height()) {
        //             body->position.y = Graphics::Height() - circleShape->radius;
        //             body->velocity.y *= -0.9;
        //         }
        //     }
        // }
    };

    render = (): void => {
        // Draw all bodies
        // for (auto body: bodies) {
        //     Uint32 color = body->isColliding ? 0xFF0000FF : 0xFFFFFFFF;

        //     if (body->shape->GetType() == CIRCLE) {
        //         CircleShape* circleShape = (CircleShape*) body->shape;
        //         Graphics::DrawFillCircle(body->position.x, body->position.y, circleShape->radius, 0xFFFFFFFF);
        //     }
        //     if (body->shape->GetType() == BOX) {
        //         BoxShape* boxShape = (BoxShape*) body->shape;
        //         Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, 0xFFFFFFFF);
        //     }
        // }

        Graphics.clearScreen();
    };

    sleep = (milliseconds: number) => {
        return new Promise(resolve => setTimeout(resolve, milliseconds));
    };
}
