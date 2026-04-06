import {
    BoxShape,
    CapsuleShape,
    CircleShape,
    DistanceJoint,
    FIXED_DELTA_TIME,
    Force,
    GRAVITY,
    MAX_BODIES,
    PIXELS_PER_METER,
    REAL_DELTA_TIME,
    RigidBody,
    SETTINGS,
    SegmentShape,
    Vec2,
    World,
} from '../src';
import { Utils } from '../src';
import AssetStore, { TEXTURES } from './graphics/AssetStore';
import Graphics from './graphics/Graphics';
import InputManager, { MouseButton } from './input/InputManager';
import BodyRenderRegistry from './render/BodyRenderRegistry';
import Demo from './samples/Demo';

const BODY_REMOVAL_THRESHOLD = 25_000;
const PLAYER_MAX_SPEED = 350;
const PLAYER_ACCELERATION = 10;
const PLAYER_JUMP_IMPULSE = 600;

export default class Application {
    private running = false;
    private paused = false;
    private world: World;
    private bgTexture: ImageBitmap | null = null;
    private readonly bodyRenderRegistry = new BodyRenderRegistry();
    private generateParticle = false;
    private demoIndex = 1;

    private player: RigidBody | null = null;

    private testBody: RigidBody | null = null;

    // Inputs
    private leftButtonPressed: boolean = false;
    private rightButtonPressed: boolean = false;
    private middleMousePressed = false;
    private controlPressed = false;

    // Debug related properties
    private debug = true;
    private FPS = 0;
    private lastFPSUpdate = 0;
    private showContacts = true;
    private showAABB = false;

    constructor() {
        this.world = new World(GRAVITY);
    }

    isRunning(): boolean {
        return this.running;
    }

    setRunning(newValue: boolean): void {
        this.running = newValue;
    }

    setBackground(texture: keyof typeof TEXTURES): void {
        this.bgTexture = AssetStore.getTexture(texture);
    }

    setBodyTexture(body: RigidBody, texture: keyof typeof TEXTURES): void {
        this.bodyRenderRegistry.setTexture(body, texture);
    }

    setBodyFillColor(body: RigidBody, fillColor: string): void {
        this.bodyRenderRegistry.setFillColor(body, fillColor);
    }

    setTestBody(body: RigidBody) {
        this.testBody = body;
    }

    async setup(): Promise<void> {
        InputManager.initialize();

        await AssetStore.loadTextures();

        this.running = Graphics.openWindow();
        const demo = Demo.demoFunctions[this.demoIndex];
        this.world.clear();
        this.bgTexture = null;
        this.bodyRenderRegistry.clear();
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

                            const radius = 250;
                            const strength = 5000;

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
                        const x = InputManager.mousePosition.x;
                        const y = InputManager.mousePosition.y;
                        const blackHole = new RigidBody(new CircleShape(0.0001), x, y, 50_000);
                        this.world.blackHole = blackHole;
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

                    if (inputEvent.key === 'b') {
                        // Emit bullet
                        const x = InputManager.mousePosition.x;
                        const y = InputManager.mousePosition.y;

                        const center = new Vec2(0, 0);
                        const target = new Vec2(x, y);
                        const direction = target.subNew(center).normalizeNew();
                        const bulletForce = 50_000;

                        const bullet = new RigidBody(new CircleShape(5), 0, 0, 0.1);
                        bullet.velocity = direction.scaleNew(bulletForce);
                        bullet.isBullet = true;
                        this.setBodyTexture(bullet, 'rockRound');
                        this.world.addBody(bullet);
                    }

                    if (inputEvent.key === 'p') {
                        this.paused = !this.paused;
                    }

                    if (inputEvent.key === '.') {
                        this.world.update(REAL_DELTA_TIME());
                    }

                    if (inputEvent.key === ',') {
                        // Note: this is not physically accurate, as contacts cannot work correctly with
                        // negative delta time, this is just used for testing purposes
                        this.world.update(-REAL_DELTA_TIME());
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
                            this.bodyRenderRegistry.delete(this.player);
                            this.world.removeBody(this.player);
                            this.player = null;
                        }

                        this.player = new RigidBody(new CapsuleShape(30, 25), x, y, 1);
                        this.player.canRotate = false;
                        this.player.restitution = 0.0;
                        this.player.friction = 0.8;
                        this.setBodyFillColor(this.player, 'orange');
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

                    if (inputEvent.key === 'z') {
                        if (this.world.getBodies().length >= MAX_BODIES) {
                            continue;
                        }

                        const x = InputManager.mousePosition.x;
                        const y = InputManager.mousePosition.y;

                        const segment = new RigidBody(new SegmentShape(new Vec2(-100, 0), new Vec2(100, 0)), x, y, 0);
                        segment.restitution = 0.2;
                        segment.friction = 0.7;
                        this.world.addBody(segment);
                    }

                    if (inputEvent.key === 'r' && !inputEvent.ctrlKey && !inputEvent.metaKey) {
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
                        this.bgTexture = null;
                        this.bodyRenderRegistry.clear();
                        this.player = null;
                        Graphics.resetView();
                        demo(this.world, this);
                    }

                    if (inputEvent.code === 'Space') {
                        const JUMP_TIME_TOLERANCE = REAL_DELTA_TIME() * 6;
                        if (
                            this.player &&
                            (this.player.isGrounded || this.player.lastGroundedTime <= JUMP_TIME_TOLERANCE)
                        ) {
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
                        this.controlPressed = true;
                    }

                    break;
                case 'keyup':
                    if (inputEvent.key === 'c') {
                        this.generateParticle = false;
                    }

                    if (inputEvent.code === 'ArrowLeft') {
                        this.leftButtonPressed = false;
                    }

                    if (inputEvent.code === 'ArrowRight') {
                        this.rightButtonPressed = false;
                    }

                    if (inputEvent.code === 'MetaLeft') {
                        this.controlPressed = false;
                    }

                    break;
            }
        }

        // Handle mouse move events
        while (InputManager.mouseMoveBuffer.length > 0) {
            const inputEvent = InputManager.mouseMoveBuffer.shift();
            if (!inputEvent) return;

            if (this.middleMousePressed || this.controlPressed) {
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
                                    this.setBodyTexture(ball, 'basketball');
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
                                    this.setBodyTexture(box, 'crate');
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

        if (this.paused) return;

        if (this.player) {
            const acceleration = PLAYER_ACCELERATION;

            if (this.leftButtonPressed) {
                const impulse = -acceleration * this.player.mass * FIXED_DELTA_TIME * PIXELS_PER_METER;
                this.player.applyImpulseLinear(new Vec2(impulse, 0));
            }

            if (this.rightButtonPressed) {
                const impulse = acceleration * this.player.mass * FIXED_DELTA_TIME * PIXELS_PER_METER;
                this.player.applyImpulseLinear(new Vec2(impulse, 0));
            }

            // Clamp velocity so you don't exceed max speed
            this.player.velocity.x = Utils.clamp(this.player.velocity.x, -PLAYER_MAX_SPEED, PLAYER_MAX_SPEED);
        }

        for (let i = 0; i < SETTINGS.subSteps; i++) {
            this.world.update(REAL_DELTA_TIME());
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
                this.setBodyTexture(particle, 'rockRound');
                this.world.addBody(particle);
            }
        }

        // Test body for collision testing
        if (this.testBody) {
            const x = InputManager.mousePosition.x;
            const y = InputManager.mousePosition.y;
            this.testBody.position.x = x;
            this.testBody.position.y = y;
        }

        this.removeOutOfBoundsBodies();
    }

    render(): void {
        Graphics.clearScreen();
        Graphics.beginWorld();

        // Draw background texture
        if (this.bgTexture && !this.debug) {
            Graphics.drawTexture(this.bgTexture.width, this.bgTexture.height, this.bgTexture, 0, 100);
        }

        if (this.world.blackHole) {
            const blackHole = this.world.blackHole;
            if (!blackHole) {
                return;
            }

            const { x, y } = blackHole.position;
            const radius = 100;

            Graphics.drawFillCircle(x, y, radius, 'rgba(86, 185, 255, 0.05)');
            Graphics.drawFillCircle(x, y, radius * 0.66, 'rgba(86, 185, 255, 0.05)');
            Graphics.drawFillCircle(x, y, radius * 0.33, 'rgba(86, 185, 255, 0.05)');
            Graphics.drawFillCircle(x, y, 10, 'rgba(149, 224, 255, 0.95)');
            Graphics.drawFillCircle(x, y, 4, 'white');
        }

        // Draw all bodies
        for (const body of this.world.getBodies()) {
            Graphics.drawBody(body, this.bodyRenderRegistry.getStyle(body), this.debug);
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
                    }
                }
            }

            Graphics.drawLine(-50, 0, 50, 0, 'gray');
            Graphics.drawLine(0, -50, 0, 50, 'gray');
        }

        Graphics.endWorld();

        let numContacts = 0;

        for (const manifold of this.world.getManifolds()) {
            numContacts += manifold.numContacts;
        }

        const defaultText = [
            // General info
            `${Demo.demoStrings[this.demoIndex]}`,
            '[ 1-9 ] select demo, [ Left Mouse ] to generate circles, [ Right Mouse ] to generate boxes',
            '[ C ] to generate particles, [ X ] to generate capsules, [ R ] to generate random convex polygon',
            '[ E ] to generate explosion, [ F ] to generate gravitational field, [ B ] to shoot bullet',
            '[ Q ] to spawn player object, [ Space ] to jump, [Left arrow / Right arrow] to move',
            `[ G ] apply gravity: ${SETTINGS.applyGravity ? 'ON' : 'OFF'}`,
            `[ D ] debug mode: ${this.debug ? 'ON' : 'OFF'}`,
            `[ P ] pause simulation: ${this.paused ? 'ON' : 'OFF'}, [ . ] step simulation`,
        ];

        const x = InputManager.mousePosition.x;
        const y = InputManager.mousePosition.y;

        const debugText = [
            // Debug related info
            `[ A ] show AABB: ${this.showAABB ? 'ON' : 'OFF'}`,
            `[ S ] show contacts and joints: ${this.showContacts ? 'ON' : 'OFF'}`,
            `Solver Iterations  [ - ] decrease [ + ] increase : ${SETTINGS.solverIterations}`,
            `Substeps [ / ] decrease [ * ] increase : ${SETTINGS.subSteps}`,
            `Num objects: ${this.world.getBodies().length} / ${MAX_BODIES} (max)`,
            `Num contacts: ${numContacts}`,
            '-------------------------------------------------------------------------------',
            `FPS: ${this.FPS.toFixed(2)}`,
            `Mouse position: {${x.toFixed(2)}, ${y.toFixed(2)}}`,
            `Zoom: ${Graphics.zoom.toFixed(2)}`,
        ];

        const text = [...defaultText, ...(this.debug ? debugText : [])];

        for (let i = 0; i < text.length; i++) {
            Graphics.drawText(text[i], 50, 50 + i * 25, 18, 'arial', this.debug ? 'orange' : 'black');
        }
    }

    private removeOutOfBoundsBodies(): void {
        const bodies = this.world.getBodies().slice();

        for (const body of bodies) {
            if (
                Math.abs(body.position.x) <= BODY_REMOVAL_THRESHOLD &&
                Math.abs(body.position.y) <= BODY_REMOVAL_THRESHOLD
            ) {
                continue;
            }

            this.bodyRenderRegistry.delete(body);
            this.world.removeBody(body);

            if (this.player?.id === body.id) {
                this.player = null;
            }
        }
    }
}
