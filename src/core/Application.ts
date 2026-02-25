import AssetStore, { TEXTURES } from '../graphics/AssetStore';
import Graphics from '../graphics/Graphics';
import InputManager, { MouseButton } from '../input/InputManager';
import { DistanceJoint } from '../joint/DistanceJoint';
import Vec2 from '../math/Vec2';
import Force from '../physics/Force';
import Demo from '../samples/Demo';
import { BoxShape } from '../shapes/BoxShape';
import { CapsuleShape } from '../shapes/CapsuleShape';
import { CircleShape } from '../shapes/CircleShape';
import { edgeCircleIntersection, edgeIntersection } from '../shapes/Edge';
import { PolygonShape } from '../shapes/PolygonShape';
import { ShapeType } from '../shapes/Shape';
import * as Utils from '../utils/Utils';
import {
    FIXED_DELTA_TIME,
    GRAVITY,
    MAX_BODIES,
    MIN_BULLET_SPEED,
    PIXELS_PER_METER,
    PLAYER_ACCELERATION,
    PLAYER_JUMP_IMPULSE,
    PLAYER_MAX_SPEED,
    REAL_DELTA_TIME,
    SETTINGS,
} from './Constants';
import RigidBody from './RigidBody';
import World from './World';

export default class Application {
    private running = false;
    private paused = false;
    private world: World;
    private bgTexture: ImageBitmap | null = null;
    private generateParticle = false;
    private demoIndex = 1;

    private player: RigidBody | null = null;

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
                        if (this.world.blackHole) {
                            this.world.blackHole = null;
                        }

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
                        bullet.setTexture('rockRound');
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
                            this.world.removeBody(this.player);
                            this.player = null;
                        }

                        this.player = new RigidBody(new CapsuleShape(40, 20), x, y, 1);
                        // this.player = new RigidBody(new CircleShape(40), x, y, 1);
                        // this.player = new RigidBody(new BoxShape(40, 40), x, y, 1);
                        this.player.canRotate = false;
                        this.player.restitution = 0.0;
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

                    if (inputEvent.key === 'r' && !this.controlPressed) {
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
                        this.player = null;
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
                particle.setTexture('rockRound');
                this.world.addBody(particle);
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

        const bodies = this.world.getBodies();
        for (const body of bodies) {
            if (body.isBullet && body.velocity.magnitudeSquared() > MIN_BULLET_SPEED) {
                const bulletShape = body.shape as CircleShape;
                const currentPos = body.position.copy();
                const nextPos = currentPos.addNew(body.velocity.scaleNew(REAL_DELTA_TIME()));

                Graphics.drawLine(currentPos.x, currentPos.y, nextPos.x, nextPos.y, 'red');

                let minDistanceSquared = Infinity;
                let closestIntersection: Vec2 | undefined;

                // TODO: We could cast two rays instead of one or check intersection by shifting up and down by radius
                for (const other of bodies) {
                    if (body.id === other.id || other.isBullet) continue;

                    if (other.shapeType === ShapeType.BOX || other.shapeType === ShapeType.POLYGON) {
                        const polygonShape = other.shape as PolygonShape;
                        const vertices = polygonShape.worldVertices;

                        for (let i = 0; i < vertices.length; i++) {
                            const v0 = vertices[i];
                            const v1 = vertices[(i + 1) % vertices.length];

                            const intersection = edgeIntersection(currentPos, nextPos, v0, v1);

                            if (intersection) {
                                const distanceSquared = intersection.subNew(currentPos).magnitudeSquared();

                                if (distanceSquared < minDistanceSquared) {
                                    closestIntersection = intersection.copy();
                                    minDistanceSquared = distanceSquared;
                                }
                            }
                        }
                    }

                    if (other.shapeType === ShapeType.CIRCLE) {
                        const circleShape = other.shape as CircleShape;
                        const intersections = edgeCircleIntersection(
                            currentPos,
                            nextPos,
                            other.position,
                            circleShape.radius,
                        );

                        for (const int of intersections) {
                            Graphics.drawFillCircle(int.x, int.y, 2, 'yellow');

                            const distanceSquared = int.subNew(currentPos).magnitudeSquared();

                            if (distanceSquared < minDistanceSquared) {
                                closestIntersection = int.copy();
                                minDistanceSquared = distanceSquared;
                            }
                        }
                    }
                }

                if (closestIntersection) {
                    Graphics.drawFillCircle(closestIntersection.x, closestIntersection.y, 5, 'yellow');


                    const toBullet = currentPos.subNew(closestIntersection).unitVector();
                    const bulletNewPos = closestIntersection.addNew(toBullet.scaleNew(bulletShape.radius));
                    Graphics.drawFillCircle(bulletNewPos.x, bulletNewPos.y, bulletShape.radius, 'green');
                    // body.shape.updateAABB(body);
                    // body.hasCCD = true;
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
            '[ 1-9 ] select demo, [ Left Mouse ] to generate circles, [ Right Mouse ] to generate boxes',
            '[ C ] to generate particles, [ X ] to generate capsules, [ R ] to generate random convex polygon',
            '[ E ] to generate explosion, [ F ] to generate attraction force, [ B ] to shoot bullet',
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
}
