import AssetStore, { TEXTURES } from './AssetStore';
import Graphics from './Graphics';
import InputManager, { MouseButton } from './InputManager';
import Vec2 from './math/Vec2';
import {
    DELTA_TIME,
    GRAVITY,
    MAX_BODIES,
    PIXELS_PER_METER,
    PLAYER_ACCELERATION,
    PLAYER_JUMP_IMPULSE,
    PLAYER_MAX_SPEED,
    SETTINGS,
} from './physics/Constants';
import { DistanceJoint } from './physics/DistanceJoint';
import Force from './physics/Force';
import RigidBody from './physics/RigidBody';
import { BoxShape, CapsuleShape, CircleShape } from './physics/Shape';
import Utils from './physics/Utils';
import World from './physics/World';
import Demo from './samples/Demo';

export default class Application {
    private running = false;
    private world: World;
    private bgTexture: ImageBitmap | null = null;
    private generateParticle = false;
    private generateAttraction = false;
    private showContacts = true;
    private showAABB = false;
    private demoIndex = 1;
    private middleMousePressed = false;

    private player: RigidBody | null = null;
    private leftButtonPressed: boolean = false;
    private rightButtonPressed: boolean = false;

    // Debug related properties
    private debug = true;
    private FPS = 0;
    private lastFPSUpdate = 0;

    constructor() {
        this.world = new World(GRAVITY);
    }

    isRunning(): boolean {
        return this.running;
    }

    setRunning(newValue: boolean): void {
        this.running = newValue;
    }

    setBackground(texture: keyof typeof TEXTURES) {
        this.bgTexture = AssetStore.getTexture(texture);
    }

    async setup(): Promise<void> {
        InputManager.initialize();

        await AssetStore.loadTextures();

        this.running = Graphics.openWindow();
        const demo = Demo.demoFunctions[this.demoIndex];
        this.world.clear();
        demo(this.world, this);
    }

    input(): void {
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

                            const debrisRadius = 50;
                            const debrisWidth = 10;
                            const radius = 250;
                            const strength = 5000;

                            const angles: number[] = [];
                            for (let i = 0; i < 10; i++) angles.push(Math.random() * Math.PI * 2);
                            const positions: Vec2[] = [];

                            for (const angle of angles) {
                                positions.push(new Vec2(Math.cos(angle), Math.sin(angle)).scaleNew(debrisRadius));
                            }

                            for (const pos of positions) {
                                const bodyPosition = new Vec2(x, y).addAssign(pos);
                                const body = Utils.randomConvexBody(bodyPosition.x, bodyPosition.y, debrisWidth, 5);
                                body.angularVelocity = Utils.randomNumber(0, 100);
                                this.world.addBody(body);
                            }

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

                    if (inputEvent.key === 'f') {
                        this.generateAttraction = true;
                    }

                    if (inputEvent.key === 'c') {
                        this.generateParticle = true;
                    }

                    if (inputEvent.key === 'g') {
                        SETTINGS.applyGravity = !SETTINGS.applyGravity;
                    }

                    if (inputEvent.key === 'a') {
                        this.showAABB = !this.showAABB;
                    }

                    if (inputEvent.key === 's') {
                        this.showContacts = !this.showContacts;
                    }

                    if (inputEvent.key === '+') {
                        SETTINGS.solverIterations++;
                    }

                    if (inputEvent.key === '-') {
                        SETTINGS.solverIterations = Math.max(1, SETTINGS.solverIterations - 1);
                    }

                    if (inputEvent.key === '*') {
                        SETTINGS.subSteps++;
                    }

                    if (inputEvent.key === '/') {
                        SETTINGS.subSteps = Math.max(1, SETTINGS.subSteps - 1);
                    }

                    if (inputEvent.key === 'q') {
                        const x = InputManager.mousePosition.x;
                        const y = InputManager.mousePosition.y;

                        if (this.player) {
                            this.world.removeBody(this.player);
                            this.player = null;
                        }

                        this.player = new RigidBody(new CapsuleShape(40, 20), x, y, 1);
                        // this.player = new RigidBody(new CircleShape(40), x, y, 1);
                        // this.player = new RigidBody(new BoxShape(40, 40), x, y, 1);
                        this.player.canRotate = false;
                        this.player.restitution = 0.2;
                        this.player.friction = 0.8;
                        this.player.shapeFillColor = 'orange';
                        this.world.addBody(this.player);
                    }

                    if (inputEvent.key === 'x') {
                        if (this.world.getBodies().length >= MAX_BODIES) {
                            continue;
                        }

                        const x = InputManager.mousePosition.x;
                        const y = InputManager.mousePosition.y;

                        const capsule = new RigidBody(new CapsuleShape(40, 20), x, y, 1);
                        capsule.restitution = 0.2;
                        capsule.friction = 0.7;
                        this.world.addBody(capsule);
                    }

                    if (inputEvent.key === 'r') {
                        if (this.world.getBodies().length >= MAX_BODIES) {
                            continue;
                        }

                        const x = InputManager.mousePosition.x;
                        const y = InputManager.mousePosition.y;

                        const radius = Utils.randomNumber(20, 50);
                        const vertices = Utils.randomNumber(3, 10);

                        const body = Utils.randomConvexBody(x, y, radius, vertices);
                        this.world.addBody(body);
                    }

                    if (!Number.isNaN(Number.parseInt(inputEvent.key))) {
                        const index = Number.parseInt(inputEvent.key);
                        this.demoIndex = index;
                        const demo = Demo.demoFunctions[this.demoIndex];

                        if (!demo) {
                            throw new Error(`Demo ${index} does not exist`);
                        }

                        this.world.clear();
                        demo(this.world, this);
                    }

                    if (inputEvent.code === 'Space') {
                        if (this.player && this.player.isGrounded) {
                            this.player.applyImpulseLinear(new Vec2(0, PLAYER_JUMP_IMPULSE));
                        }
                    }

                    if (inputEvent.code === 'ArrowLeft') {
                        this.leftButtonPressed = true;
                    }

                    if (inputEvent.code === 'ArrowRight') {
                        this.rightButtonPressed = true;
                    }

                    if (inputEvent.code === 'MetaLeft') {
                        this.middleMousePressed = true;
                    }

                    break;
                case 'keyup':
                    if (inputEvent.key === 'c') {
                        this.generateParticle = false;
                    }

                    if (inputEvent.key === 'f') {
                        this.generateAttraction = false;
                    }

                    if (inputEvent.code === 'ArrowLeft') {
                        this.leftButtonPressed = false;
                    }

                    if (inputEvent.code === 'ArrowRight') {
                        this.rightButtonPressed = false;
                    }

                    if (inputEvent.code === 'MetaLeft') {
                        this.middleMousePressed = false;
                    }

                    break;
            }
        }

        // Handle mouse move events
        while (InputManager.mouseMoveBuffer.length > 0) {
            const inputEvent = InputManager.mouseMoveBuffer.shift();
            if (!inputEvent) return;

            if (this.middleMousePressed) {
                document.body.style.cursor = 'pointer';
                // Drag the camera opposite to mouse movement
                Graphics.pan.x -= inputEvent.movementX / Graphics.zoom;
                Graphics.pan.y += inputEvent.movementY / Graphics.zoom;
            } else {
                document.body.style.cursor = 'default';
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
                                    ball.restitution = 0.8;
                                    ball.friction = 0.7;
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
                                break;
                        }
                    }
                    break;
                case 'mouseup':
                    switch (inputEvent.button) {
                        case MouseButton.MIDDLE:
                            this.middleMousePressed = false;
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
    }

    update(frameTime: number): void {
        if (this.debug) {
            if (!this.lastFPSUpdate || performance.now() - this.lastFPSUpdate > 1000) {
                this.lastFPSUpdate = performance.now();
                this.FPS = 1 / frameTime;
            }
        }

        if (this.player) {
            const acceleration = PLAYER_ACCELERATION;

            if (this.leftButtonPressed) {
                const impulse = -acceleration * this.player.mass * DELTA_TIME * PIXELS_PER_METER;
                this.player.applyImpulseLinear(new Vec2(impulse, 0));
            }

            if (this.rightButtonPressed) {
                const impulse = acceleration * this.player.mass * DELTA_TIME * PIXELS_PER_METER;
                this.player.applyImpulseLinear(new Vec2(impulse, 0));
            }

            // Clamp velocity so you don't exceed max speed
            this.player.velocity.x = Utils.clamp(this.player.velocity.x, -PLAYER_MAX_SPEED, PLAYER_MAX_SPEED);
        }

        for (let i = 0; i < SETTINGS.subSteps; i++) {
            this.world.update(DELTA_TIME / SETTINGS.subSteps);
        }

        if (this.generateParticle) {
            const x = InputManager.mousePosition.x;
            const y = InputManager.mousePosition.y;
            const radius = 10;
            for (let i = 0; i < 10; i++) {
                if (this.world.getBodies().length >= MAX_BODIES) {
                    continue;
                }

                const angle = Math.random() * Math.PI * 2;
                const positionOffset = new Vec2(Math.cos(angle), Math.sin(angle)).scaleNew(radius);

                const particle = new RigidBody(new CircleShape(5), x + positionOffset.x, y + positionOffset.y, 0.01);
                particle.restitution = 0.0;
                particle.friction = 0.5;
                particle.setTexture('rockRound');
                this.world.addBody(particle);
            }
        }

        if (this.generateAttraction) {
            const x = InputManager.mousePosition.x;
            const y = InputManager.mousePosition.y;
            const blackHole = new RigidBody(new CircleShape(1), x, y, 50_000);

            for (const body of this.world.getBodies()) {
                const attraction = Force.generateGravitationalForce(body, blackHole, GRAVITY, 1, 200);
                body.addForce(attraction);
            }
        }
    }

    render(): void {
        Graphics.clearScreen();
        Graphics.beginWorld();

        // Draw background texture
        if (this.bgTexture && !this.debug) {
            Graphics.drawTexture(this.bgTexture.width, this.bgTexture.height, this.bgTexture, 0, 100);
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
            '(C) to generate particles, (X) to generate capsules, (R) to generate random convex polygon',
            '(E) to generate explosion, (F) to generate attraction force, (Q) to spawn a player object',
            `(G) apply gravity: ${SETTINGS.applyGravity ? 'ON' : 'OFF'}`,
            `(D) debug mode: ${this.debug ? 'ON' : 'OFF'}`,
        ];

        const x = InputManager.mousePosition.x;
        const y = InputManager.mousePosition.y;

        const debugText = [
            // Debug related info
            `(A) show AABB: ${this.showAABB ? 'ON' : 'OFF'}`,
            `(S) show contacts and joints: ${this.showContacts ? 'ON' : 'OFF'}`,
            `Num objects: ${this.world.getBodies().length} / ${MAX_BODIES} (max)`,
            `Num contacts: ${numContacts}`,
            `(+ -) solver iterations: ${SETTINGS.solverIterations}`,
            `(* /) substeps: ${SETTINGS.subSteps}`,
            '***********************************************',
            `FPS: ${this.FPS.toFixed(2)}`,
            `Mouse position: {${x.toFixed(2)}, ${y.toFixed(2)}}`,
            `Zoom: ${Graphics.zoom.toFixed(2)}`,
        ];

        const text = [...defaultText, ...(this.debug ? debugText : [])];

        for (let i = 0; i < text.length; i++) {
            Graphics.drawText(text[i], 50, 50 + i * 25, 18, 'arial', this.debug ? 'orange' : 'black');
        }
    }
}
