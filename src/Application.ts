import AssetStore from './AssetStore';
import Graphics from './Graphics';
import InputManager, { MouseButton } from './InputManager';
import Body from './physics/Body';
import { PIXELS_PER_METER } from './physics/Constants';
import { JointConstraint } from './physics/Constraint';
import { BoxShape, CircleShape, PolygonShape, ShapeType } from './physics/Shape';
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
            bob: 'assets/ragdoll/bob.png',
            head: 'assets/ragdoll/head.png',
            leftArm: 'assets/ragdoll/leftArm.png',
            leftLeg: 'assets/ragdoll/leftLeg.png',
            rightArm: 'assets/ragdoll/rightArm.png',
            rightLeg: 'assets/ragdoll/rightLeg.png',
            torso: 'assets/ragdoll/torso.png',
        };

        await AssetStore.loadTextures(textures);

        this.running = Graphics.openWindow();

        // Add ragdoll parts (rigid bodies)
        const bob = new Body(new CircleShape(5), Graphics.width() / 2.0, Graphics.height() / 2.0 - 200, 0.0);
        const head = new Body(new CircleShape(25), bob.position.x, bob.position.y + 70, 5.0);
        const torso = new Body(new BoxShape(50, 100), head.position.x, head.position.y + 80, 3.0);
        const leftArm = new Body(new BoxShape(15, 70), torso.position.x - 32, torso.position.y - 10, 1.0);
        const rightArm = new Body(new BoxShape(15, 70), torso.position.x + 32, torso.position.y - 10, 1.0);
        const leftLeg = new Body(new BoxShape(20, 90), torso.position.x - 20, torso.position.y + 97, 1.0);
        const rightLeg = new Body(new BoxShape(20, 90), torso.position.x + 20, torso.position.y + 97, 1.0);
        bob.setTexture('bob');
        head.setTexture('head');
        torso.setTexture('torso');
        leftArm.setTexture('leftArm');
        rightArm.setTexture('rightArm');
        leftLeg.setTexture('leftLeg');
        rightLeg.setTexture('rightLeg');
        this.world.addBody(bob);
        this.world.addBody(head);
        this.world.addBody(torso);
        this.world.addBody(leftArm);
        this.world.addBody(rightArm);
        this.world.addBody(leftLeg);
        this.world.addBody(rightLeg);

        // Add joints between ragdoll parts (distance constraints with one anchor point)
        const string = new JointConstraint(bob, head, bob.position);
        const neck = new JointConstraint(head, torso, head.position.addNew(new Vec2(0, 25)));
        const leftShoulder = new JointConstraint(torso, leftArm, torso.position.addNew(new Vec2(-28, -45)));
        const rightShoulder = new JointConstraint(torso, rightArm, torso.position.addNew(new Vec2(+28, -45)));
        const leftHip = new JointConstraint(torso, leftLeg, torso.position.addNew(new Vec2(-20, +50)));
        const rightHip = new JointConstraint(torso, rightLeg, torso.position.addNew(new Vec2(+20, +50)));

        this.world.addConstraint(string);
        this.world.addConstraint(neck);
        this.world.addConstraint(leftShoulder);
        this.world.addConstraint(rightShoulder);
        this.world.addConstraint(leftHip);
        this.world.addConstraint(rightHip);

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

                    if (inputEvent.key === 's') {
                        console.log( this.world.getConstraints());
                        this.world.getConstraints().shift();
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

            const mouse = new Vec2(inputEvent.x, inputEvent.y);
            const bob = this.world.getBodies()[0];
            const direction = mouse.subNew(bob.position).normalize();
            const speed = 5;
            bob.position.addAssign(direction.scaleNew(speed));
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
        // Draw a line between the bob and the ragdoll head
        // const bob = this.world.getBodies()[0];
        // const head = this.world.getBodies()[1];
        // Graphics.drawLine(bob.position.x, bob.position.y, head.position.x, head.position.y, 'white');

        // Draw all joints anchor points
        for (const joint of this.world.getConstraints()) {
            if (this.debug) {
                const anchorPoint = joint.a.localSpaceToWorldSpace(joint.aPoint);
                Graphics.drawFillCircle(anchorPoint.x, anchorPoint.y, 3, 'red');
            }
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
