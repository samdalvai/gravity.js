/*
 * This implementation is inspired by techniques used in Box2D
 * by Erin Catto, licensed under the MIT License.
 *
 * https://github.com/erincatto/box2d-lite
 */
import {
    Bodies,
    BoxShape,
    CircleShape,
    DistanceJoint,
    PolygonShape,
    RigidBody,
    Vec2,
    World,
} from '../../src';
import Application from '../Application';
import Graphics from '../graphics/Graphics';

const FLOOR_WIDTH = 3200;
const FLOOR_HEIGHT = 50;
const FLOOR_POSITION_Y = -350;

interface JointTuning {
    frequency: number;
    dampingRatio: number;
}

type JointTuningMap = Record<string, JointTuning>;

const JOINT_TUNING: JointTuningMap = {
    bridge: { frequency: 14, dampingRatio: 0.9 },
    whip: { frequency: 11, dampingRatio: 0.7 },
    ragdoll: { frequency: 18, dampingRatio: 0.9 },
    plank: { frequency: 28, dampingRatio: 1.0 },
    cloth: { frequency: 16, dampingRatio: 0.95 },
    stressBridge: { frequency: 16, dampingRatio: 0.92 },
};

export default class Demo {
    static demoStrings = [
        'Demo 0: A complex scene',
        'Demo 1: A single box',
        'Demo 2: A stack of boxes',
        'Demo 3: A pyramid of boxes',
        'Demo 4: A suspension bridge',
        'Demo 5: As simple whip',
        'Demo 6: A skeleton ragdoll',
        'Demo 7: A plank',
        'Demo 8: Cloth simulation',
        'Demo 9: Stress test',
    ];

    static generateFloor(world: World, app: Application): RigidBody {
        const floor = Bodies.box({ width: FLOOR_WIDTH, height: FLOOR_HEIGHT, x: 0, y: FLOOR_POSITION_Y, mass: 0.0 });
        app.setBodyTexture(floor, 'transparent');
        world.addBody(floor);
        return floor;
    }

    static generateFences(world: World, app: Application): void {
        const fenceWidth = 50;
        const fenceHeight = 900 + FLOOR_HEIGHT;
        const leftFence = Bodies.box({
            width: fenceWidth,
            height: fenceHeight,
            x: -(FLOOR_WIDTH / 2 + fenceWidth / 2),
            y: FLOOR_POSITION_Y + FLOOR_HEIGHT / 2 + fenceHeight / 2 - FLOOR_HEIGHT,
            mass: 0.0,
        });

        const rightFence = Bodies.box({
            width: fenceWidth,
            height: fenceHeight,
            x: FLOOR_WIDTH / 2 + fenceWidth / 2,
            y: FLOOR_POSITION_Y + FLOOR_HEIGHT / 2 + fenceHeight / 2 - FLOOR_HEIGHT,
            mass: 0.0,
        });

        app.setBodyTexture(leftFence, 'transparent');
        app.setBodyTexture(rightFence, 'transparent');
        world.addBody(leftFence);
        world.addBody(rightFence);
    }

    static createDistanceJoint(
        bodyA: RigidBody,
        bodyB: RigidBody,
        tuning: JointTuning,
        anchorA: Vec2 = bodyA.position,
        anchorB: Vec2 = bodyB.position,
        length = -1,
    ): DistanceJoint {
        return new DistanceJoint(bodyA, bodyB, anchorA, anchorB, length, tuning.frequency, tuning.dampingRatio);
    }

    static demo1 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 1: Single box demo
        this.generateFloor(world, app);
        this.generateFences(world, app);

        const body = new RigidBody(new BoxShape(60, 60), 0, 0, 1);
        body.angularVelocity = 2;
        body.rotation = 0.7;
        app.setBodyTexture(body, 'crate');
        world.addBody(body);

        // Bodies for collision testing
        // const body = new RigidBody(new CircleShape(30), 0, 0, 0);
        // const body = new RigidBody(new CapsuleShape(30, 15), 0, 0, 0);
        // const body = new RigidBody(new SegmentShape(200, true), 0, 0, 0);

        // const testBody = new RigidBody(new BoxShape(60, 60), 100, 0, 0);
        // const testBody = new RigidBody(new CircleShape(30), 0, 0, 0);
        // const testBody = new RigidBody(new CapsuleShape(30, 15), 100, 0, 0);
        // testBody.angularVelocity = 5;
        // testBody.rotation = 0.5
        // world.addBody(testBody);
        // app.setTestBody(testBody);
    };

    static demo2 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 2: Stack of boxes
        this.generateFloor(world, app);
        this.generateFences(world, app);

        const numOfBoxes = 10;
        const boxSize = 60;
        const boxSpacing = 10;

        for (let i = 0; i < numOfBoxes; i++) {
            const box = new RigidBody(new BoxShape(boxSize, boxSize), 0, -200 + (boxSize + boxSpacing) * i, 1);
            box.restitution = 0;
            app.setBodyTexture(box, 'crate');
            world.addBody(box);
        }
    };

    static demo3 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 3: Pyramid of boxes
        this.generateFloor(world, app);
        this.generateFences(world, app);

        const boxSize = 60;
        const boxSpacing = 10;
        const rows = 10;

        const centerX = 0;
        const baseY = 250;

        for (let row = 0; row < rows; row++) {
            const boxesInRow = rows - row;
            const rowWidth = boxesInRow * boxSize;

            for (let col = 0; col < boxesInRow; col++) {
                const x = centerX - rowWidth / 2 + boxSize / 2 + col * boxSize;
                const y = baseY - row * (boxSize + boxSpacing);

                const box = new RigidBody(new BoxShape(boxSize, boxSize), x, -y, 1);
                box.restitution = 0.1;
                app.setBodyTexture(box, 'crate');
                box.restitution = 0.001;
                world.addBody(box);
            }
        }
    };

    static demo4 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 4: A suspension bridge
        this.generateFloor(world, app);
        this.generateFences(world, app);

        const stepCount = 8;
        const stepWidth = 90;
        const stepHeight = stepWidth / 4;
        const stepSpacing = 100;

        const totalSpan = (stepCount - 1) * stepSpacing + stepWidth;
        const pillarOffsetX = totalSpan / 2 + stepWidth / 2;

        const pillarWidth = 50;
        const pillarHeight = 400;
        const pillarPositionY = FLOOR_POSITION_Y + pillarHeight / 2 + FLOOR_HEIGHT / 2; //-pillarHeight / 2 + FLOOR_HEIGHT / 2;

        const pillarLeft = new RigidBody(new BoxShape(pillarWidth, pillarHeight), -pillarOffsetX, pillarPositionY, 0);
        const pillarRight = new RigidBody(new BoxShape(pillarWidth, pillarHeight), pillarOffsetX, pillarPositionY, 0);

        app.setBodyTexture(pillarLeft, 'metal');
        app.setBodyTexture(pillarRight, 'metal');

        world.addBody(pillarLeft);
        world.addBody(pillarRight);

        const stepPositionY = pillarPositionY + pillarHeight / 2;
        const steps: RigidBody[] = [];

        for (let i = 0; i < stepCount; i++) {
            const x = (i - (stepCount - 1) / 2) * stepSpacing;

            const step = new RigidBody(new BoxShape(stepWidth, stepHeight), x, stepPositionY, 5);

            app.setBodyTexture(step, 'woodBox');
            world.addBody(step);
            steps.push(step);
        }

        const tuning = JOINT_TUNING.bridge;
        const distance = -1;

        // joints between steps
        for (let i = 0; i < steps.length - 1; i++) {
            const a = steps[i];
            const b = steps[i + 1];

            const joint = this.createDistanceJoint(
                a,
                b,
                tuning,
                a.position.addNew(new Vec2(stepWidth / 2, 0)),
                b.position.subNew(new Vec2(stepWidth / 2, 0)),
                distance,
            );

            world.addJoint(joint);
        }

        // left pillar → first step
        const leftJoint = this.createDistanceJoint(
            pillarLeft,
            steps[0],
            tuning,
            pillarLeft.position.addNew(new Vec2(25, 0)).addNew(new Vec2(0, pillarHeight / 2)),
            steps[0].position.subNew(new Vec2(stepWidth / 2, 0)),
            distance,
        );

        // right pillar → last step
        const rightJoint = this.createDistanceJoint(
            pillarRight,
            steps[steps.length - 1],
            tuning,
            pillarRight.position.subNew(new Vec2(25, 0)).addNew(new Vec2(0, pillarHeight / 2)),
            steps[steps.length - 1].position.addNew(new Vec2(stepWidth / 2, 0)),
            distance,
        );

        world.addJoint(leftJoint);
        world.addJoint(rightJoint);
    };

    static demo5 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 5: A simple whip
        this.generateFloor(world, app);
        this.generateFences(world, app);

        const whipAnchor = new RigidBody(new BoxShape(60, 25), 0, 350, 0);
        app.setBodyTexture(whipAnchor, 'rockBridgeAnchor');
        world.addBody(whipAnchor);

        let last = whipAnchor;
        const whipElementHeight = 50;

        const tuning = JOINT_TUNING.whip;
        const distance = -1; // -1 = the initial distance

        for (let i = 0; i < 10; i++) {
            const x = whipAnchor.position.x;
            const y =
                i === 0
                    ? whipAnchor.position.y - whipElementHeight
                    : whipAnchor.position.y - (whipElementHeight + 60 * i);
            const whipElement = new RigidBody(new BoxShape(10, 50), x, y, 1);
            app.setBodyTexture(whipElement, 'crate');
            world.addBody(whipElement);

            const j = this.createDistanceJoint(
                last,
                whipElement,
                tuning,
                last.position.subNew(new Vec2(0, whipElementHeight / 2)),
                whipElement.position.addNew(new Vec2(0, whipElementHeight / 2)),
                distance,
            );
            world.addJoint(j);

            last = whipElement;
        }
    };

    static demo6 = (world: World, app: Application) => {
        app.setBackground('darkBackground');

        // Demo 6: A skeleton ragdoll
        this.generateFloor(world, app);
        this.generateFences(world, app);

        // Add ragdoll parts (rigid bodies)
        const bob = new RigidBody(new CircleShape(5), 0, 100, 0.0);
        const head = new RigidBody(new CircleShape(25), bob.position.x, bob.position.y - 70, 5.0);
        const torso = new RigidBody(new BoxShape(50, 100), head.position.x, head.position.y - 80, 3.0);
        const leftArm = new RigidBody(new BoxShape(15, 70), torso.position.x - 32, torso.position.y + 10, 1.0);
        const rightArm = new RigidBody(new BoxShape(15, 70), torso.position.x + 32, torso.position.y + 10, 1.0);
        const leftLeg = new RigidBody(new BoxShape(20, 90), torso.position.x - 20, torso.position.y - 97, 1.0);
        const rightLeg = new RigidBody(new BoxShape(20, 90), torso.position.x + 20, torso.position.y - 97, 1.0);
        app.setBodyTexture(bob, 'bob');
        app.setBodyTexture(head, 'head');
        app.setBodyTexture(torso, 'torso');
        app.setBodyTexture(leftArm, 'leftArm');
        app.setBodyTexture(rightArm, 'rightArm');
        app.setBodyTexture(leftLeg, 'leftLeg');
        app.setBodyTexture(rightLeg, 'rightLeg');
        world.addBody(bob);
        world.addBody(head);
        world.addBody(torso);
        world.addBody(leftArm);
        world.addBody(rightArm);
        world.addBody(leftLeg);
        world.addBody(rightLeg);

        const tuning = JOINT_TUNING.ragdoll;

        // Add joints between ragdoll parts (distance constraints with one anchor point)
        const string = this.createDistanceJoint(bob, head, tuning, bob.position, head.position);
        const neck = this.createDistanceJoint(
            head,
            torso,
            tuning,
            head.position.subNew(new Vec2(0, 25)),
            torso.position.addNew(new Vec2(0, 50)),
        );
        const leftShoulder = this.createDistanceJoint(
            torso,
            leftArm,
            tuning,
            torso.position.addNew(new Vec2(-28, 45)),
            leftArm.position.addNew(new Vec2(5, 35)),
        );
        const rightShoulder = this.createDistanceJoint(
            torso,
            rightArm,
            tuning,
            torso.position.addNew(new Vec2(28, 45)),
            rightArm.position.addNew(new Vec2(-5, 35)),
        );
        const leftHip = this.createDistanceJoint(
            torso,
            leftLeg,
            tuning,
            torso.position.addNew(new Vec2(-20, -50)),
            leftLeg.position.addNew(new Vec2(0, 45)),
        );
        const rightHip = this.createDistanceJoint(
            torso,
            rightLeg,
            tuning,
            torso.position.addNew(new Vec2(+20, -50)),
            rightLeg.position.addNew(new Vec2(0, 45)),
        );

        world.addJoint(string);
        world.addJoint(neck);
        world.addJoint(leftShoulder);
        world.addJoint(rightShoulder);
        world.addJoint(leftHip);
        world.addJoint(rightHip);
    };

    static demo7 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 7: A plank
        const floor = this.generateFloor(world, app);
        this.generateFences(world, app);

        const plank = new RigidBody(new BoxShape(750, 20), 0, floor.position.y + 100, 10);
        app.setBodyTexture(plank, 'woodPlankCracked');
        world.addBody(plank);

        const joint = this.createDistanceJoint(floor, plank, JOINT_TUNING.plank, plank.position, plank.position);
        world.addJoint(joint);

        const triangleVertices = [new Vec2(-30, -30), new Vec2(30, -30), new Vec2(0, 33.5)];
        const triangle = new RigidBody(new PolygonShape(triangleVertices), 0, floor.position.y + 55, 0);
        app.setBodyTexture(triangle, 'woodTriangle');
        world.addBody(triangle);

        const box1 = new RigidBody(new BoxShape(25, 25), plank.position.x - 350, plank.position.y + 25, 1);
        const box2 = new RigidBody(new BoxShape(25, 25), plank.position.x - 325, plank.position.y + 25, 1);
        const box3 = new RigidBody(new BoxShape(25, 25), plank.position.x - 337.5, plank.position.y + 50, 1);
        app.setBodyTexture(box1, 'crate');
        app.setBodyTexture(box2, 'crate');
        app.setBodyTexture(box3, 'crate');
        world.addBody(box1);
        world.addBody(box2);
        world.addBody(box3);

        const heavyBox = new RigidBody(new BoxShape(50, 50), plank.position.x + 350, Graphics.height() - 750, 10);
        app.setBodyTexture(heavyBox, 'metal');
        world.addBody(heavyBox);
    };

    static demo8 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 8: Cloth simulation
        this.generateFloor(world, app);
        this.generateFences(world, app);

        const rows = 25;
        const cols = 30;
        const spacing = 25;
        const particleRadius = 1;
        const startX = -((cols * spacing) / 2);
        const topY = 100 + (rows * spacing) / 2;

        const particles: RigidBody[][] = [];

        // Generate particles
        for (let row = 0; row < rows; row++) {
            const rowParticles: RigidBody[] = [];
            for (let col = 0; col < cols; col++) {
                const x = startX + col * spacing;
                const y = topY - row * spacing;
                const mass = row === 0 ? 0 : 1; // top row can be anchors (mass=0)
                const particle = new RigidBody(new CircleShape(particleRadius), x, y, mass);
                world.addBody(particle);
                rowParticles.push(particle);
            }
            particles.push(rowParticles);
        }

        const tuning = JOINT_TUNING.cloth;

        // Create joints
        for (let row = 0; row < rows; row++) {
            for (let col = 0; col < cols; col++) {
                const p = particles[row][col];

                // Connect to particle above
                if (row > 0) {
                    const above = particles[row - 1][col];
                    const joint = this.createDistanceJoint(above, p, tuning);
                    joint.drawConnectionLine = true;
                    world.addJoint(joint);
                }

                // Connect to particle to the right
                if (col < cols - 1 && row > 0) {
                    const right = particles[row][col + 1];
                    const joint = this.createDistanceJoint(p, right, tuning);
                    joint.drawConnectionLine = true;
                    world.addJoint(joint);
                }
            }
        }
    };

    static demo9 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 9: stress test
        this.generateFloor(world, app);
        this.generateFences(world, app);

        const tuning = JOINT_TUNING.stressBridge;

        // Suspension Bridge Creation
        const numSteps = 10;
        const stepWidth = 40;
        const spacing = stepWidth + 2.5; // distance between centers
        const startX = -(numSteps * spacing) / 2 - stepWidth / 2;
        const startY = 0;

        // Start anchor (static)
        const startAnchor = new RigidBody(
            new BoxShape(stepWidth * 2, stepWidth * 0.5),
            startX - stepWidth / 2,
            startY,
            0.0,
        );
        app.setBodyTexture(startAnchor, 'rockBridgeAnchor');
        world.addBody(startAnchor);

        // First connection uses the start anchor
        let lastStep = startAnchor;

        // Create steps
        for (let i = 1; i <= numSteps; i++) {
            const x = startX + i * spacing;

            // Optional sag: small vertical sinusoidal displacement
            const y = startY - Math.sin((i / numSteps) * Math.PI) * 10;

            const step = new RigidBody(new CircleShape(stepWidth * 0.5), x, y, 3);
            app.setBodyTexture(step, 'woodBridgeStep');
            world.addBody(step);

            // Joint anchor at left edge of this step
            const joint = this.createDistanceJoint(lastStep, step, tuning);
            world.addJoint(joint);

            lastStep = step;
        }

        // End anchor (static)
        const endAnchor = new RigidBody(
            new BoxShape(stepWidth * 2, stepWidth * 0.5),
            lastStep.position.x + spacing + stepWidth / 2,
            startY,
            0.0,
        );
        app.setBodyTexture(endAnchor, 'rockBridgeAnchor');
        world.addBody(endAnchor);

        // Final joint anchor at right edge of last step
        const lastJoint = this.createDistanceJoint(lastStep, endAnchor, tuning);
        world.addJoint(lastJoint);

        const boxSizeLarge = 40;
        const numBoxLargeHorizontal = 10;

        for (let i = 0; i < numBoxLargeHorizontal; i++) {
            for (let j = 0; j < 10; j++) {
                const box = new RigidBody(
                    new BoxShape(boxSizeLarge, boxSizeLarge),
                    -(numBoxLargeHorizontal * boxSizeLarge) / 2 + boxSizeLarge / 2 + i * boxSizeLarge,
                    500 + j * boxSizeLarge,
                    1,
                );
                app.setBodyTexture(box, 'woodBox');
                world.addBody(box);
            }
        }

        const boxSizeSmall = 20;
        const numBoxSmallHorizontal = 20;

        for (let i = 0; i < numBoxSmallHorizontal; i++) {
            for (let j = 0; j < 10; j++) {
                const box = new RigidBody(
                    new BoxShape(boxSizeSmall, boxSizeSmall),
                    -(numBoxSmallHorizontal * boxSizeSmall) / 2 + boxSizeSmall / 2 + i * boxSizeSmall,
                    2000 + j * boxSizeSmall,
                    1,
                );
                app.setBodyTexture(box, 'metal');
                world.addBody(box);
            }
        }
    };

    static demo0 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 0: a complex scene
        const floor = this.generateFloor(world, app);
        this.generateFences(world, app);

        // Add bird
        const bird = Bodies.circle({ radius: 45, x: -550, y: -200, mass: 3.0 });
        app.setBodyTexture(bird, 'birdRed');
        world.addBody(bird);

        // Add a stack of boxes
        for (let i = 1; i <= 4; i++) {
            const mass = 10.0 / i;
            const box = Bodies.box({
                width: 50,
                height: 50,
                x: -300,
                y: floor.position.y + FLOOR_HEIGHT / 2 + i * 55,
                mass,
                friction: 0.9,
                restitution: 0.1,
            });
            app.setBodyTexture(box, 'woodBox');
            world.addBody(box);
        }

        // Add structure with blocks
        const plank1 = Bodies.box({
            width: 50,
            height: 150,
            x: -30,
            y: floor.position.y + FLOOR_HEIGHT / 2 + 100,
            mass: 5.0,
        });
        const plank2 = Bodies.box({
            width: 50,
            height: 150,
            x: 130,
            y: floor.position.y + FLOOR_HEIGHT / 2 + 100,
            mass: 5.0,
        });
        const plank3 = Bodies.box({
            width: 250,
            height: 25,
            x: 50,
            y: floor.position.y + FLOOR_HEIGHT / 2 + 200,
            mass: 2.0,
        });
        app.setBodyTexture(plank1, 'woodPlankSolid');
        app.setBodyTexture(plank2, 'woodPlankSolid');
        app.setBodyTexture(plank3, 'woodPlankCracked');
        world.addBody(plank1);
        world.addBody(plank2);
        world.addBody(plank3);

        // Add a triangle polygon
        const triangleVertices = [new Vec2(-30, -30), new Vec2(30, -30), new Vec2(0, 30)];
        const triangle = Bodies.polygon({
            vertices: triangleVertices,
            x: plank3.position.x,
            y: plank3.position.y + 50,
            mass: 0.5,
        });
        app.setBodyTexture(triangle, 'woodTriangle');
        world.addBody(triangle);

        // Add a pyramid of boxes
        const numRows = 5;
        for (let col = 0; col < numRows; col++) {
            for (let row = 0; row < col; row++) {
                const x = plank3.position.x + 200 + col * 50 - row * 25;
                const y = floor.position.y + FLOOR_HEIGHT / 2 + 50 + row * 52;
                const mass = 5 / (row + 1);
                const box = new RigidBody(new BoxShape(50, 50), x, y, mass);
                box.friction = 0.9;
                box.restitution = 0.0;
                app.setBodyTexture(box, 'woodBox');
                world.addBody(box);
            }
        }

        const bridgeTuning = JOINT_TUNING.stressBridge;

        // Add a bridge of connected steps and joints
        const numSteps = 10;
        const spacing = 33;

        // Start anchor (static)
        const stepHeight = 20;
        const startStep = new RigidBody(new BoxShape(80, stepHeight), -500, 200, 0.0);
        app.setBodyTexture(startStep, 'rockBridgeAnchor');
        world.addBody(startStep);

        // The first connection should be from the anchor, not the floor
        let last = startStep;

        for (let i = 1; i <= numSteps; i++) {
            const x = startStep.position.x + 30 + i * spacing;
            const y = startStep.position.y - Math.sin((i / numSteps) * Math.PI) * 10;
            const mass = 3;

            const step = new RigidBody(new CircleShape(15), x, y, mass);
            app.setBodyTexture(step, 'woodBridgeStep');
            world.addBody(step);

            // Connect previous link to this link
            const joint = this.createDistanceJoint(last, step, bridgeTuning);
            world.addJoint(joint);

            last = step;
        }

        // Final anchor
        const endStep = new RigidBody(new BoxShape(80, stepHeight), last.position.x + 60, startStep.position.y, 0.0);
        app.setBodyTexture(endStep, 'rockBridgeAnchor');
        world.addBody(endStep);

        const lastJoint = this.createDistanceJoint(last, endStep, bridgeTuning);
        world.addJoint(lastJoint);

        // Add pigs
        const pigRadius = 30;
        const pig1 = new RigidBody(
            new CircleShape(pigRadius),
            plank1.position.x + 80,
            floor.position.y + FLOOR_HEIGHT / 2 + pigRadius,
            3.0,
        );
        const pig2 = new RigidBody(
            new CircleShape(pigRadius),
            plank2.position.x + 400,
            floor.position.y + FLOOR_HEIGHT / 2 + pigRadius,
            3.0,
        );
        const pig3 = new RigidBody(
            new CircleShape(pigRadius),
            plank2.position.x + 460,
            floor.position.y + FLOOR_HEIGHT / 2 + pigRadius,
            3.0,
        );
        const pig4 = new RigidBody(
            new CircleShape(pigRadius),
            startStep.position.x,
            startStep.position.y + stepHeight / 2 + pigRadius,
            1.0,
        );
        app.setBodyTexture(pig1, 'pig1');
        app.setBodyTexture(pig2, 'pig2');
        app.setBodyTexture(pig3, 'pig1');
        app.setBodyTexture(pig4, 'pig2');
        world.addBody(pig1);
        world.addBody(pig2);
        world.addBody(pig3);
        world.addBody(pig4);
    };

    static demoFunctions = [
        this.demo0,
        this.demo1,
        this.demo2,
        this.demo3,
        this.demo4,
        this.demo5,
        this.demo6,
        this.demo7,
        this.demo8,
        this.demo9,
    ];
}
