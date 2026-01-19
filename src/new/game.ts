import { AABB, createAABB, fix } from './aabb';
import { Box } from './box';
import { Camera } from './camera';
import { Circle } from './circle';
import { demos } from './demo';
import { GrabJoint } from './grab';
import * as Input from './input';
import { Vector2 } from './vector2';
import { Polygon } from './polygon';
import { Renderer } from './renderer';
import { RigidBody, Type } from './rigidbody';
import { GenerationShape, MouseMode, Settings, updateSetting } from './settings';
import * as Util from './util';
import { World } from './world';

export let gWorld: World;

export class Game {
    private renderer: Renderer;
    public camera: Camera;
    private world: World;

    public cursorPos: Vector2 = new Vector2(0, 0);
    public deltaTime: number = 0.0;
    public time: number = 0.0;
    public frame: number = 0;

    private cameraPosStart!: Vector2;
    private cursorStart!: Vector2;
    private dragging = false;
    private cameraMove = false;
    private grabbing = false;
    private bindPosition!: Vector2;
    private target!: RigidBody;
    private grabJoint!: GrabJoint;

    private currentDemo = 0;
    public callback = () => {};

    constructor(renderer: Renderer) {
        this.renderer = renderer;
        this.camera = new Camera();
        this.camera.position = new Vector2(0, Settings.clipHeight / 2.0);

        const projectionTransform = Util.orth(
            -Settings.clipWidth / 2.0,
            Settings.clipWidth / 2.0,
            -Settings.clipHeight / 2.0,
            Settings.clipHeight / 2.0,
        );
        const viewportTransform = Util.viewport(Settings.width, Settings.height);

        this.renderer.init(viewportTransform, projectionTransform, this.camera.cameraTransform);

        this.world = new World();
        gWorld = this.world;

        const restartBtn = document.querySelector('#restart') as HTMLButtonElement;
        restartBtn.addEventListener('click', () => {
            this.initDemo();
        });

        const demoSelect = document.querySelector('#demo_select') as HTMLSelectElement;
        demos.forEach(demo => {
            const option = document.createElement('option');
            option.innerHTML = Reflect.get(demo, 'SimulationName');
            demoSelect.appendChild(option);
        });
        demoSelect.addEventListener('input', () => {
            this.currentDemo = demoSelect.selectedIndex;
            this.initDemo();
        });
        demoSelect.selectedIndex = this.currentDemo;
        this.initDemo();
    }

    initDemo(): void {
        this.frame = 0;
        this.time = 0.0;
        if (Settings.resetCamera) {
            this.camera.position = new Vector2(0, Settings.clipHeight / 2.0);
            this.camera.scale = new Vector2(1, 1);
        }
        this.world.reset();
        this.callback = () => {};
        demos[this.currentDemo](this, this.world);
    }

    update(deltaTime: number): void {
        this.deltaTime = deltaTime;
        this.frame++;
        this.time += deltaTime;
        this.handleInput(deltaTime);
        this.callback();
        this.world.update(deltaTime);
    }

    private handleInput(deltaTime: number) {
        const mx = Input.isKeyDown('ArrowLeft') ? -1 : Input.isKeyDown('ArrowRight') ? 1 : 0;
        const my = Input.isKeyDown('ArrowDown') ? -1 : Input.isKeyDown('ArrowUp') ? 1 : 0;

        this.camera.translate(new Vector2(mx, my).mulNew(deltaTime * 10 * this.camera.scale.x));
        const tmpCursorPos = this.renderer.pick(Input.mousePosition);

        this.cursorPos.x = tmpCursorPos.x;
        this.cursorPos.y = tmpCursorPos.y;

        if (!this.dragging && Input.isMousePressed(Input.Button.Middle)) {
            this.cursorStart = this.cursorPos.copy();
            this.dragging = true;
        }

        if (this.dragging && Input.isMouseReleased(Input.Button.Middle)) {
            this.dragging = false;
            const aabb = new AABB(this.cursorStart.copy(), this.cursorPos.copy());
            fix(aabb);

            const bodies = this.world.queryRegion(aabb);
            for (let i = 0; i < bodies.length; i++) {
                const b = bodies[i];

                this.world.unregister(b.id);
            }
        }

        if (Input.isScrolling()) {
            this.camera.scale.x += Input.mouseScroll.y * 0.1;
            this.camera.scale.y += Input.mouseScroll.y * 0.1;

            if (this.camera.scale.x < 0.1) {
                this.camera.scale.x = 0.1;
                this.camera.scale.y = 0.1;
            }
        }

        if (!this.cameraMove && Input.isMousePressed(Input.Button.Right)) {
            this.cameraMove = true;
            this.cursorStart = Input.mousePosition.copy();
            this.cameraPosStart = this.camera.position.copy();
        } else if (Input.isMouseReleased(Input.Button.Right)) {
            this.cameraMove = false;
        }

        if (this.cameraMove) {
            const dist = Input.mousePosition.subNew(this.cursorStart);
            dist.x *= -(Settings.clipWidth / Settings.width) * this.camera.scale.x;
            dist.y *= -(Settings.clipHeight / Settings.height) * this.camera.scale.y;
            this.camera.position = this.cameraPosStart.addNew(dist);
        }

        if (this.grabbing && !this.cameraMove) {
            if (Input.isMouseReleased(Input.Button.Left)) {
                if (Settings.mode == MouseMode.Force) {
                    this.world.forceIntegration = true;

                    const bindInGlobal = this.target.localToGlobal.mulVector2(this.bindPosition, 1);
                    const force = this.cursorPos
                        .subNew(bindInGlobal)
                        .mulNew(this.target.mass)
                        .mulNew(Settings.frequency * (0.8 + Settings.mouseStrength / 3.0));
                    const torque = bindInGlobal
                        .subNew(this.target.localToGlobal.mulVector2(new Vector2(0, 0), 1))
                        .cross(force);

                    this.target.force.x += force.x;
                    this.target.force.y += force.y;
                    this.target.torque += torque;

                    this.target.awake();
                } else if (Settings.mode == MouseMode.Grab) {
                    this.world.forceIntegration = false;

                    this.world.unregister(this.grabJoint.id, true);
                }

                this.grabbing = false;
            }
        }

        if (Input.isMousePressed(Input.Button.Left)) {
            let skipGeneration = false;

            const bodies = this.world.queryPoint(this.cursorPos);

            for (let i = 0; i < bodies.length; i++) {
                const b = bodies[i];
                if (b.type != Type.Static) {
                    this.grabbing = true;
                    if (Settings.grabCenter) this.bindPosition = new Vector2(0, 0);
                    else this.bindPosition = b.globalToLocal.mulVector2(this.cursorPos, 1);
                    this.target = b;
                    skipGeneration = true;
                    break;
                }
            }

            if (skipGeneration && Settings.mode == MouseMode.Grab) {
                const bind = Settings.grabCenter ? this.target.position : this.cursorPos.copy();
                this.grabJoint = new GrabJoint(this.target, bind, this.cursorPos, Settings.mouseStrength, undefined);
                this.world.register(this.grabJoint);
            }

            if (!skipGeneration && !this.cameraMove) {
                let nb!: RigidBody;

                const nbs = Settings.newBodySettings;

                switch (nbs.shape) {
                    case GenerationShape.Box: {
                        nb = new Box(nbs.size, nbs.size, Type.Dynamic, nbs.density);
                        break;
                    }
                    case GenerationShape.Circle: {
                        nb = new Circle(nbs.size / 2.0, Type.Dynamic, nbs.density);
                        break;
                    }
                    case GenerationShape.Regular: {
                        nb = Util.createRegularPolygon(nbs.size / 2.0, nbs.numVertices, undefined, nbs.density);
                        break;
                    }
                    case GenerationShape.Random: {
                        nb = Util.createRandomConvexBody(
                            (Math.random() * nbs.size) / 3 + nbs.size / 2,
                            undefined,
                            nbs.density,
                        );
                        break;
                    }
                }

                nb.position = this.cursorPos;
                nb.friction = nbs.friction;
                nb.restitution = nbs.restitution;

                this.world.register(nb);
            }
        }

        if (Input.isMousePressed(Input.Button.Right)) {
            const bodies = this.world.queryPoint(this.cursorPos);

            if (bodies.length != 0) {
                this.world.unregister(bodies[0].id);
            }
        }

        if (Input.isKeyPressed('r')) this.initDemo();
        if (Input.isKeyPressed('m')) updateSetting('m');
        if (Input.isKeyPressed('p')) updateSetting('p');
        if (Input.isKeyPressed('g')) {
            this.world.surprise();
            updateSetting('g');
        }
        if (Input.isKeyPressed('b')) updateSetting('b');
        if (Input.isKeyPressed('i')) updateSetting('i');
        if (Input.isKeyPressed('f')) updateSetting('f');
        if (Input.isKeyPressed('v')) updateSetting('v');
    }

    render(r: Renderer): void {
        r.setCameraTransform(this.camera.cameraTransform);

        // Body, Bounding box, Center of Mass rendering
        for (let i = 0; i < this.world.bodies.length; i++) {
            const b = this.world.bodies[i];

            const outlineWidth = 1.0;

            if (Settings.colorizeBody || Settings.colorizeIsland || (Settings.colorizeActiveBody && !b.sleeping)) {
                let id = Settings.colorizeIsland ? b.islandID : b.id;

                if (!(Settings.colorizeBody || Settings.colorizeIsland)) id = b.islandID;

                const hStride = 17;
                const sStride = 5;
                const lStride = 3;
                const period = Math.trunc(360 / hStride);
                const cycle = Math.trunc(id / period);
                // let dir = (cycle & 1) == 1 ? -1 : 1;

                const h = (id - 1) * hStride;
                const s = 100 - ((cycle * sStride) % 21);
                let l = 75 - ((cycle * lStride) % 17);

                if (!(Settings.colorizeBody || Settings.colorizeIsland)) l = Util.clamp(l + 10, 0, 100);

                const color = b.type == Type.Static ? '#f0f0f0' : `hsl(${h}, ${s}%, ${l}%)`;
                r.drawBody(b, Settings.indicateCoM, outlineWidth, true, true, '#000000', color);
            } else {
                r.drawBody(b, Settings.indicateCoM, outlineWidth);
            }

            if (Settings.showBoundingBox) {
                const aabb = createAABB(b);
                r.drawAABB(aabb);
            }

            if (Settings.showContactLink) {
                for (let m = 0; m < b.manifoldIDs.length; m++) {
                    const id = b.manifoldIDs[m];
                    const manifold = this.world.manifoldMap.get(id)!;

                    if (manifold.bodyA.type == Type.Static || manifold.bodyB.type == Type.Static) continue;

                    if (manifold.bodyA.id == b.id) r.drawLineV(manifold.bodyB.position, manifold.bodyA.position, 1.0);
                }
            }
        }

        // Rendering for mouse forcing
        if (this.grabbing && Settings.mode == MouseMode.Force) {
            const bindInGlobal = this.target.localToGlobal.mulVector2(this.bindPosition, 1);
            r.drawCircleV(bindInGlobal, 0.03);
            r.drawVectorP(bindInGlobal, this.cursorPos);
        }

        // Rendering contact manifold
        if (Settings.indicateCP) {
            for (let i = 0; i < this.world.manifolds.length; i++) {
                const m = this.world.manifolds[i];

                let j = 0;
                let mid = new Vector2();
                for (; j < m.numContacts; j++) {
                    mid = mid.addNew(m.contactPoints[j].point);
                    r.drawCircleV(m.contactPoints[j].point, 0.04);
                }
                mid = mid.divNew(j);
                r.drawVectorP(mid, mid.addNew(m.contactNormal.mulNew(0.2)), 0.015);
            }
        }

        // Joint rendering
        for (let i = 0; i < this.world.joints.length; i++) {
            const j = this.world.joints[i];
            r.drawJoint(j);
        }

        // Log rigid body information
        let line = 0;
        if (Settings.showProfile) {
            r.log('Bodies: ' + String(this.world.numBodies), line++);
            r.log('Joints: ' + String(this.world.numJoints), line++);
            r.log('Contacts: ' + String(this.world.manifolds.length), line++);
            r.log('Islands: ' + String(this.world.numIslands), line++);
            r.log('Sleeping dynamic bodies: ' + String(this.world.sleepingBodies), line++);
            r.log('Sleeping islands: ' + String(this.world.sleepingIslands), line++);
            line++;
        }

        if (Settings.showInfo) {
            let found = false;
            const bodies = this.world.queryPoint(this.cursorPos);

            if (bodies.length != 0) {
                this.target = bodies[0];
                found = true;
            }

            if (found) {
                r.log('Type: ' + String(Type[this.target.type]), line++);
                r.log('Mass: ' + String(this.target.mass) + 'kg', line++);
                r.log('Moment of inertia: ' + String(this.target.inertia.toFixed(4)) + 'kg⋅m²', line++);

                if (this.target instanceof Polygon) {
                    if (this.target instanceof Box) {
                        r.log('Density: ' + String((this.target as Box).density.toFixed(4)) + 'kg/m²', line++);
                        r.log('Area: ' + String((this.target as Box).area.toFixed(4)) + 'm²', line++);
                    } else {
                        r.log('Density: ' + String((this.target as Polygon).density.toFixed(4)) + 'kg/m²', line++);
                        r.log('Area: ' + String((this.target as Polygon).area.toFixed(4)) + 'm²', line++);
                    }
                } else if (this.target instanceof Circle) {
                    r.log('Density: ' + String((this.target as Circle).density.toFixed(4)) + 'kg/m²', line++);
                    r.log('Area: ' + String((this.target as Circle).area.toFixed(4)) + 'm²', line++);
                }

                r.log('Friction: ' + String(this.target.friction), line++);
                r.log('Restitution: ' + String(this.target.restitution), line++);
                r.log(
                    'Position: [' +
                        String(this.target.position.x.toFixed(4)) +
                        ', ' +
                        String(this.target.position.y.toFixed(4)) +
                        ']',
                    line++,
                );
                r.log('Rotation: ' + String(this.target.rotation.toFixed(4)) + 'rad', line++);
                r.log(
                    'Linear velocity: [' +
                        String((this.target.linearVelocity.x / 100).toFixed(4)) +
                        ', ' +
                        String((this.target.linearVelocity.y / 100).toFixed(4)) +
                        ']m/s',
                    line++,
                );
                r.log('Angular velocity: ' + String(this.target.angularVelocity.toFixed(4)) + 'rad/s', line++);
                r.log('Surface velocity: ' + String(this.target.surfaceSpeed.toFixed(4)) + 'm/s', line++);
                r.log('Contacts: ' + this.target.manifoldIDs.length, line++);
                r.log('Joints: ' + this.target.jointIDs.length, line++);
                r.log('Island: ' + this.target.islandID, line++);
                r.log('Sleeping: ' + this.target.sleeping, line++);
            }
        }

        if (Settings.visualizeAABBTree) {
            this.world.tree.traverse(node => {
                r.drawAABB(node!.aabb, 1.0, !node.isLeaf ? '#00000055' : '#000000');
            });
        }

        if (this.dragging && Input.isMouseDown(Input.Button.Middle)) {
            const aabb = new AABB(this.cursorStart.copy(), this.cursorPos.copy());
            fix(aabb);

            r.drawAABB(aabb);
        }

        // TODO: handle generating a lot of bodies
        // if (Input.isMouseDown(Input.Button.Left)) {
        //     for (let i = 0; i < 10; i++) {
        //         const nbs = Settings.newBodySettings;
        //         const nb = new Circle(0.1, Type.Dynamic, nbs.density);
        //         // const nb = new Box(0.1, 0.1, Type.Dynamic, nbs.density);
        //         nb.position = this.cursorPos;
        //         nb.friction = nbs.friction;
        //         nb.restitution = nbs.restitution;

        //         this.world.register(nb);
        //     }
        // }
    }
}
