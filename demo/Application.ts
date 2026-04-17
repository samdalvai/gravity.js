import {
    Bodies,
    DistanceJoint,
    FIXED_DELTA_TIME,
    Force,
    GRAVITY,
    GrabJoint,
    MAX_BODIES,
    PIXELS_PER_METER,
    RigidBody,
    SETTINGS,
    Vec2,
    WeldJoint,
    World,
} from '../src';
import { Utils } from '../src';
import AssetStore, { TEXTURES } from './graphics/AssetStore';
import Graphics from './graphics/Graphics';
import InputManager, { MouseButton } from './input/InputManager';
import BodyRenderRegistry from './render/BodyRenderRegistry';
import Demo from './samples/Demo';
import UIManager, { UIState } from './ui/UIManager';

const BODY_REMOVAL_THRESHOLD = 25_000;
const DEMO_SHORTCUT_DELAY_MS = 350;
const MAX_DEMO_SHORTCUT_DIGITS = 2;
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
    private demoShortcutBuffer = '';
    private demoShortcutTimer: number | null = null;

    private player: RigidBody | null = null;

    private testBody: RigidBody | null = null;
    private blackHole: RigidBody | null = null;

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
    private showRuntimeStatsHud = true;
    private readonly uiManager = new UIManager();

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

    setBlackHole(body: RigidBody | null): void {
        this.blackHole = body;
    }

    async setup(): Promise<void> {
        InputManager.initialize();

        await AssetStore.loadTextures();

        this.running = Graphics.openWindow();
        this.uiManager.initialize(this.getUIState(), {
            onSelectDemo: index => this.loadDemo(index),
            onRestartDemo: () => this.loadDemo(this.demoIndex),
            onSetDebug: value => this.setDebug(value),
            onSetShowAABB: value => this.setShowAABB(value),
            onSetShowContacts: value => this.setShowContacts(value),
            onSetShowRuntimeStatsHud: value => this.setShowRuntimeStatsHud(value),
            onSetCCD: value => this.setCCD(value),
            onSetApplyGravity: value => this.setApplyGravity(value),
            onSetPaused: value => this.setPaused(value),
            onSetSolverIterations: value => this.setSolverIterations(value),
            onSetSubSteps: value => this.setSubSteps(value),
            onStep: () => this.stepSimulation(),
        });
        this.loadDemo(this.demoIndex);
    }

    input(): void {
        // Handle keyboard events
        while (InputManager.keyboardInputBuffer.length > 0) {
            const inputEvent = InputManager.keyboardInputBuffer.shift();
            if (!inputEvent) return;

            switch (inputEvent.type) {
                case 'keydown': {
                    const key = inputEvent.key.toLowerCase();

                    if (/^\d$/.test(inputEvent.key)) {
                        this.handleDemoShortcutDigit(inputEvent.key);
                        break;
                    }

                    this.flushDemoShortcut();

                    if (inputEvent.key === 'd') {
                        this.setDebug(!this.debug);
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

                    if (key === 'f' && inputEvent.shiftKey) {
                        this.setBlackHole(null);
                    } else if (key === 'f') {
                        const x = InputManager.mousePosition.x;
                        const y = InputManager.mousePosition.y;
                        this.setBlackHole(Bodies.circle({ radius: 0.0001, x, y, mass: 1_000_000 }));
                    }

                    if (key === 'r' && inputEvent.shiftKey) {
                        this.loadDemo(this.demoIndex);
                    }

                    if (inputEvent.key === 'c') {
                        this.generateParticle = true;
                    }

                    if (inputEvent.key === 'g') {
                        this.setApplyGravity(!SETTINGS.applyGravity);
                    }

                    if (inputEvent.key === 'a') {
                        this.setShowAABB(!this.showAABB);
                    }

                    if (inputEvent.key === 's') {
                        this.setShowContacts(!this.showContacts);
                    }

                    if (inputEvent.key === 'b') {
                        // Emit bullet
                        const x = InputManager.mousePosition.x;
                        const y = InputManager.mousePosition.y;

                        const center = new Vec2(0, 0);
                        const target = new Vec2(x, y);
                        const direction = target.subNew(center).normalizeNew();
                        const bulletForce = 50_000;

                        const bullet = Bodies.circle({
                            radius: 5,
                            x: 0,
                            y: 0,
                            mass: 0.1,
                            velocity: direction.scaleNew(bulletForce),
                            isBullet: true,
                        });
                        this.setBodyTexture(bullet, 'rockRound');
                        this.world.addBody(bullet);
                    }

                    if (inputEvent.key === 'p') {
                        this.setPaused(!this.paused);
                    }

                    if (inputEvent.key === '.') {
                        this.applyBlackHoleForce();
                        this.world.update(SETTINGS.dt);
                    }

                    if (inputEvent.key === ',') {
                        // Note: this is not physically accurate, as contacts cannot work correctly with
                        // negative delta time, this is just used for testing purposes
                        this.world.update(-SETTINGS.dt);
                    }

                    if (inputEvent.key === '+') {
                        this.setSolverIterations(SETTINGS.solverIterations + 1);
                    }

                    if (inputEvent.key === '-') {
                        this.setSolverIterations(SETTINGS.solverIterations - 1);
                    }

                    if (inputEvent.key === '*') {
                        this.setSubSteps(SETTINGS.subSteps + 1);
                    }

                    if (inputEvent.key === '/') {
                        this.setSubSteps(SETTINGS.subSteps - 1);
                    }

                    if (inputEvent.key === 'q') {
                        const x = InputManager.mousePosition.x;
                        const y = InputManager.mousePosition.y;

                        if (this.player) {
                            this.bodyRenderRegistry.delete(this.player);
                            this.world.removeBody(this.player);
                            this.player = null;
                        }

                        this.player = Bodies.capsule({
                            halfHeight: 30,
                            radius: 25,
                            x,
                            y,
                            mass: 1,
                            canRotate: false,
                            restitution: 0.0,
                            friction: 0.8,
                        });
                        this.setBodyFillColor(this.player, 'orange');
                        this.world.addBody(this.player);
                    }

                    if (inputEvent.key === 'x') {
                        if (this.world.getBodies().length >= MAX_BODIES) {
                            continue;
                        }

                        const x = InputManager.mousePosition.x;
                        const y = InputManager.mousePosition.y;

                        const capsule = Bodies.capsule({
                            halfHeight: 40,
                            radius: 20,
                            x,
                            y,
                            mass: 1,
                            restitution: 0.2,
                            friction: 0.7,
                        });
                        this.world.addBody(capsule);
                    }

                    if (inputEvent.key === 'z') {
                        if (this.world.getBodies().length >= MAX_BODIES) {
                            continue;
                        }

                        const x = InputManager.mousePosition.x;
                        const y = InputManager.mousePosition.y;

                        const segment = Bodies.segment({
                            length: 200,
                            horizontal: true,
                            x,
                            y,
                            mass: 0,
                            restitution: 0.2,
                            friction: 0.7,
                        });
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

                    if (inputEvent.code === 'Space') {
                        const JUMP_TIME_TOLERANCE = SETTINGS.dt * 6;
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
                }
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
                                    const ball = Bodies.circle({
                                        radius: 30,
                                        x,
                                        y,
                                        mass: 1.0,
                                        restitution: 0.5,
                                        friction: 0.7,
                                    });
                                    this.setBodyTexture(ball, 'basketball');
                                    this.world.addBody(ball);
                                }
                                break;
                            case MouseButton.RIGHT:
                                {
                                    if (this.world.getBodies().length >= MAX_BODIES) {
                                        continue;
                                    }
                                    const box = Bodies.box({
                                        width: 60,
                                        height: 60,
                                        x,
                                        y,
                                        mass: 1.0,
                                        restitution: 0.3,
                                        friction: 0.7,
                                    });
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

        this.advanceSimulation();

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

                const particle = Bodies.circle({
                    radius: 5,
                    x: x + positionOffset.x,
                    y: y + positionOffset.y,
                    mass: 0.01,
                    restitution: 0.0,
                    friction: 0.5,
                });
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

        if (this.blackHole) {
            const { x, y } = this.blackHole.position;

            Graphics.drawFillCircle(x, y, 150, 'rgba(122, 190, 255, 0.045)');
            Graphics.drawFillCircle(x, y, 110, 'rgba(122, 190, 255, 0.045)');
            Graphics.drawFillCircle(x, y, 78, 'rgba(122, 190, 255, 0.045)');
            Graphics.drawFillCircle(x, y, 52, '#07090f');
            Graphics.drawFillCircle(x, y, 36, '#000000');
            Graphics.drawFillCircle(x, y, 10, 'rgba(149, 224, 255, 0.95)');
            Graphics.drawFillCircle(x, y, 4, 'white');
        }

        // Draw all bodies
        for (const body of this.world.getBodies()) {
            Graphics.drawBody(body, this.bodyRenderRegistry.getStyle(body), this.debug);
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
                    if (joint instanceof DistanceJoint || joint instanceof WeldJoint) {
                        const anchorA = joint.localAnchorA;
                        const anchorB = joint.localAnchorB;
                        const worldA = joint.bodyA.localPointToWorld(anchorA);
                        const worldB = joint.bodyB.localPointToWorld(anchorB);
                        Graphics.drawFillCircle(worldA.x, worldA.y, 3, 'blue');
                        Graphics.drawFillCircle(worldB.x, worldB.y, 3, 'blue');

                        Graphics.drawLine(worldA.x, worldA.y, worldB.x, worldB.y, 'blue');
                    }

                    if (joint instanceof GrabJoint) {
                        const anchor = joint.localAnchor;
                        const world = joint.bodyA.localPointToWorld(anchor);
                        const target = joint.target;
                        Graphics.drawFillCircle(world.x, world.y, 3, 'blue');
                        Graphics.drawFillCircle(target.x, target.y, 3, 'blue');

                        Graphics.drawLine(world.x, world.y, target.x, target.y, 'blue');
                    }
                }

                for (const manifold of this.world.getManifolds()) {
                    for (const contact of manifold.points) {
                        const startPoint = contact.point;
                        const normalEndPoint = contact.point.addNew(manifold.normal.scaleNew(PIXELS_PER_METER * 0.15));

                        Graphics.drawFillCircle(startPoint.x, startPoint.y, 3, 'red');
                        Graphics.drawArrow(startPoint.x, startPoint.y, normalEndPoint.x, normalEndPoint.y, 'red', 1);
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

        if (!this.debug || !this.showRuntimeStatsHud) {
            return;
        }

        const x = InputManager.mousePosition.x;
        const y = InputManager.mousePosition.y;

        const stats: Array<[string, string]> = [
            ['Demo', Demo.demoStrings[this.demoIndex].replace(/^Demo \d+:\s*/, '')],
            ['Gravity', SETTINGS.applyGravity ? 'ON' : 'OFF'],
            ['Paused', this.paused ? 'ON' : 'OFF'],
            ['AABB', this.showAABB ? 'ON' : 'OFF'],
            ['Contacts', this.showContacts ? 'ON' : 'OFF'],
            ['Bodies', `${this.world.getBodies().length}/${MAX_BODIES}`],
            ['Collisions', `${numContacts}`],
            ['FPS', this.FPS.toFixed(2)],
            ['Zoom', Graphics.zoom.toFixed(2)],
            ['Mouse', `${x.toFixed(1)}, ${y.toFixed(1)}`],
        ];

        const panelX = 20;
        const panelY = this.uiManager.headerHeight + 16;
        const panelWidth = 320;
        const panelPaddingX = 14;
        const panelPaddingY = 14;
        const titleHeight = 22;
        const subtitleHeight = 20;
        const rowHeight = 22;
        const panelHeight = panelPaddingY * 2 + titleHeight + subtitleHeight + stats.length * rowHeight;

        Graphics.drawFillRect(panelX, panelY, panelWidth, panelHeight, 'rgba(10, 12, 16, 0.78)');
        Graphics.drawStrokeRect(panelX, panelY, panelWidth, panelHeight, 'rgba(255, 255, 255, 0.14)');
        Graphics.drawFillRect(panelX, panelY, panelWidth, 3, '#ff9d2e');

        Graphics.drawText('DEBUG', panelX + panelPaddingX, panelY + 18, 15, 'Arial', '#ffb15c', 'left', 'middle');
        Graphics.drawText(
            'Runtime Stats',
            panelX + panelPaddingX,
            panelY + 40,
            13,
            'Arial',
            'rgba(255, 255, 255, 0.7)',
            'left',
            'middle',
        );

        const labelX = panelX + panelPaddingX;
        const valueX = panelX + panelWidth - panelPaddingX;
        const rowsTop = panelY + panelPaddingY + titleHeight + subtitleHeight + 10;

        for (let i = 0; i < stats.length; i++) {
            const [label, value] = stats[i];
            const rowY = rowsTop + i * rowHeight;

            Graphics.drawText(label, labelX, rowY, 14, 'Arial', 'rgba(255, 255, 255, 0.72)', 'left', 'middle');
            Graphics.drawText(value, valueX, rowY, 14, 'Arial', '#ffffff', 'right', 'middle');
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

    private applyBlackHoleForce(): void {
        if (!this.blackHole) {
            return;
        }

        const blackHole = this.blackHole;

        for (const body of this.world.getBodies()) {
            if (body.id === blackHole.id) {
                continue;
            }

            const attraction = Force.generateGravitationalForce(body, blackHole, GRAVITY, 80 * 80, 950 * 950);
            body.addForce(attraction);
        }
    }

    private advanceSimulation(): void {
        for (let i = 0; i < SETTINGS.subSteps; i++) {
            this.stepSimulation();
        }
    }

    private handleDemoShortcutDigit(digit: string): void {
        const nextBuffer = `${this.demoShortcutBuffer}${digit}`;
        const matchingDemoIndexes = Demo.demoFunctions
            .map((_demo, index) => `${index}`)
            .filter(index => index.startsWith(nextBuffer));

        if (matchingDemoIndexes.length === 0) {
            this.flushDemoShortcut();
            return;
        }

        this.demoShortcutBuffer = nextBuffer;
        const hasExactMatch = matchingDemoIndexes.includes(nextBuffer);
        const hasLongerMatch = matchingDemoIndexes.some(index => index.length > nextBuffer.length);

        this.clearDemoShortcutTimer();

        if (hasExactMatch && (!hasLongerMatch || nextBuffer.length >= MAX_DEMO_SHORTCUT_DIGITS)) {
            this.flushDemoShortcut();
            return;
        }

        this.demoShortcutTimer = window.setTimeout(() => this.flushDemoShortcut(), DEMO_SHORTCUT_DELAY_MS);
    }

    private clearDemoShortcutTimer(): void {
        if (this.demoShortcutTimer === null) {
            return;
        }

        window.clearTimeout(this.demoShortcutTimer);
        this.demoShortcutTimer = null;
    }

    private flushDemoShortcut(): void {
        if (this.demoShortcutBuffer === '') {
            this.clearDemoShortcutTimer();
            return;
        }

        const index = Number.parseInt(this.demoShortcutBuffer, 10);
        this.demoShortcutBuffer = '';
        this.clearDemoShortcutTimer();

        if (!Demo.demoFunctions[index]) {
            return;
        }

        this.loadDemo(index);
    }

    private loadDemo(index: number): void {
        const demo = Demo.demoFunctions[index];

        if (!demo) {
            this.demoShortcutBuffer = '';
            this.clearDemoShortcutTimer();
            this.syncUI();
            return;
        }

        this.demoShortcutBuffer = '';
        this.clearDemoShortcutTimer();
        this.demoIndex = index;
        this.world.clear();
        this.bgTexture = null;
        this.bodyRenderRegistry.clear();
        this.player = null;
        this.testBody = null;
        this.blackHole = null;
        this.generateParticle = false;
        this.leftButtonPressed = false;
        this.rightButtonPressed = false;
        this.middleMousePressed = false;
        this.controlPressed = false;
        Graphics.resetView();
        demo(this.world, this);
        this.syncUI();
    }

    private setDebug(value: boolean): void {
        this.debug = value;
        this.syncUI();
    }

    private setShowAABB(value: boolean): void {
        this.showAABB = value;
        this.syncUI();
    }

    private setShowContacts(value: boolean): void {
        this.showContacts = value;
        this.syncUI();
    }

    private setPaused(value: boolean): void {
        this.paused = value;
        this.syncUI();
    }

    private setShowRuntimeStatsHud(value: boolean): void {
        this.showRuntimeStatsHud = value;
        this.syncUI();
    }

    private setCCD(value: boolean): void {
        SETTINGS.ccd = value;
        this.syncUI();
    }

    private setApplyGravity(value: boolean): void {
        SETTINGS.applyGravity = value;
        this.syncUI();
    }

    private setSolverIterations(value: number): void {
        const clampedValue = Utils.clamp(value, 1, 30);

        if (!Number.isFinite(clampedValue)) {
            this.syncUI();
            return;
        }

        SETTINGS.solverIterations = Math.max(1, Math.round(clampedValue));
        this.syncUI();
    }

    private setSubSteps(value: number): void {
        const clampedValue = Utils.clamp(value, 1, 10);

        if (!Number.isFinite(clampedValue)) {
            this.syncUI();
            return;
        }

        SETTINGS.subSteps = Math.max(1, Math.round(clampedValue));
        this.syncUI();
    }

    private stepSimulation(): void {
        this.applyBlackHoleForce();
        this.world.update(SETTINGS.dt);
    }

    private getUIState(): UIState {
        return {
            demoIndex: this.demoIndex,
            demoLabels: Demo.demoStrings,
            debug: this.debug,
            showAABB: this.showAABB,
            showContacts: this.showContacts,
            showRuntimeStatsHud: this.showRuntimeStatsHud,
            ccd: SETTINGS.ccd,
            applyGravity: SETTINGS.applyGravity,
            paused: this.paused,
            solverIterations: SETTINGS.solverIterations,
            subSteps: SETTINGS.subSteps,
        };
    }

    private syncUI(): void {
        this.uiManager.sync(this.getUIState());
    }
}
