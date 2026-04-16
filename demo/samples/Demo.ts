/*
 * This implementation is inspired by techniques used in Box2D
 * by Erin Catto, licensed under the MIT License.
 *
 * https://github.com/erincatto/box2d-lite
 */
import { Bodies, DistanceJoint, GRAVITY, RigidBody, SETTINGS, Utils, Vec2, World } from '../../src';
import { WeldJoint } from '../../src/joint/WeldJoint';
import Application from '../Application';
import Graphics from '../graphics/Graphics';

const FLOOR_WIDTH = 3200;
const FLOOR_HEIGHT = 50;
const FLOOR_POSITION_Y = -350;
const STRESS_DEMO_COLUMNS = 25;
const STRESS_DEMO_ROWS = 40;
const SQUARE_CAGE_INNER_SIZE = 1000;

interface CageBounds {
    innerBottom: number;
    innerTop: number;
}

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
        'Demo 10: Continuous collision detection',
        'Demo 11: 1000 Circles',
        'Demo 12: 1000 Boxes',
        'Demo 13: 1000 Capsules',
        'Demo 14: 1000 Random convex shapes',
        'Demo 15: Black hole orbit',
        'Demo 16: Welded boxes',
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

    static generateSquareCage(world: World, app: Application): CageBounds {
        const wallThickness = FLOOR_HEIGHT;
        const innerBottom = FLOOR_POSITION_Y + wallThickness / 2;
        const innerTop = innerBottom + SQUARE_CAGE_INNER_SIZE;
        const wallSpanHorizontal = SQUARE_CAGE_INNER_SIZE;
        const wallSpanVertical = SQUARE_CAGE_INNER_SIZE + wallThickness * 2;
        const wallColor = '#4d5a72';
        const walls = [
            Bodies.box({
                width: wallSpanHorizontal,
                height: wallThickness,
                x: 0,
                y: innerBottom - wallThickness / 2,
                mass: 0,
            }),
            Bodies.box({
                width: wallSpanHorizontal,
                height: wallThickness,
                x: 0,
                y: innerTop + wallThickness / 2,
                mass: 0,
            }),
            Bodies.box({
                width: wallThickness,
                height: wallSpanVertical,
                x: -(SQUARE_CAGE_INNER_SIZE / 2 + wallThickness / 2),
                y: (innerBottom + innerTop) / 2,
                mass: 0,
            }),
            Bodies.box({
                width: wallThickness,
                height: wallSpanVertical,
                x: SQUARE_CAGE_INNER_SIZE / 2 + wallThickness / 2,
                y: (innerBottom + innerTop) / 2,
                mass: 0,
            }),
        ];

        for (const wall of walls) {
            app.setBodyFillColor(wall, wallColor);
            world.addBody(wall);
        }

        return { innerBottom, innerTop };
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

    static demo0 = (world: World, app: Application) => {
        // Demo 0: a complex scene
        app.setBackground('background');
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
                const box = Bodies.box({ width: 50, height: 50, x, y, mass, friction: 0.9, restitution: 0.0 });
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
        const startStep = Bodies.box({ width: 80, height: stepHeight, x: -500, y: 200, mass: 0.0 });
        app.setBodyTexture(startStep, 'rockBridgeAnchor');
        world.addBody(startStep);

        // The first connection should be from the anchor, not the floor
        let last = startStep;

        for (let i = 1; i <= numSteps; i++) {
            const x = startStep.position.x + 30 + i * spacing;
            const y = startStep.position.y - Math.sin((i / numSteps) * Math.PI) * 10;
            const mass = 3;

            const step = Bodies.circle({ radius: 15, x, y, mass });
            app.setBodyTexture(step, 'woodBridgeStep');
            world.addBody(step);

            // Connect previous link to this link
            const joint = this.createDistanceJoint(last, step, bridgeTuning);
            world.addJoint(joint);

            last = step;
        }

        // Final anchor
        const endStep = Bodies.box({
            width: 80,
            height: stepHeight,
            x: last.position.x + 60,
            y: startStep.position.y,
            mass: 0.0,
        });
        app.setBodyTexture(endStep, 'rockBridgeAnchor');
        world.addBody(endStep);

        const lastJoint = this.createDistanceJoint(last, endStep, bridgeTuning);
        world.addJoint(lastJoint);

        // Add pigs
        const pigRadius = 30;
        const pig1 = Bodies.circle({
            radius: pigRadius,
            x: plank1.position.x + 80,
            y: floor.position.y + FLOOR_HEIGHT / 2 + pigRadius,
            mass: 3.0,
        });
        const pig2 = Bodies.circle({
            radius: pigRadius,
            x: plank2.position.x + 400,
            y: floor.position.y + FLOOR_HEIGHT / 2 + pigRadius,
            mass: 3.0,
        });
        const pig3 = Bodies.circle({
            radius: pigRadius,
            x: plank2.position.x + 460,
            y: floor.position.y + FLOOR_HEIGHT / 2 + pigRadius,
            mass: 3.0,
        });
        const pig4 = Bodies.circle({
            radius: pigRadius,
            x: startStep.position.x,
            y: startStep.position.y + stepHeight / 2 + pigRadius,
            mass: 1.0,
        });
        app.setBodyTexture(pig1, 'pig1');
        app.setBodyTexture(pig2, 'pig2');
        app.setBodyTexture(pig3, 'pig1');
        app.setBodyTexture(pig4, 'pig2');
        world.addBody(pig1);
        world.addBody(pig2);
        world.addBody(pig3);
        world.addBody(pig4);
    };

    static demo1 = (world: World, app: Application) => {
        // Demo 1: Single box demo
        app.setBackground('background');
        this.generateFloor(world, app);
        this.generateFences(world, app);

        const body = Bodies.box({ width: 60, height: 60, x: 0, y: 0, mass: 1, angularVelocity: 2, rotation: 0.7 });
        app.setBodyTexture(body, 'crate');
        world.addBody(body);
    };

    static demo2 = (world: World, app: Application) => {
        // Demo 2: Stack of boxes
        app.setBackground('background');
        this.generateFloor(world, app);
        this.generateFences(world, app);

        const numOfBoxes = 10;
        const boxSize = 60;
        const boxSpacing = 10;

        for (let i = 0; i < numOfBoxes; i++) {
            const box = Bodies.box({
                width: boxSize,
                height: boxSize,
                x: 0,
                y: -200 + (boxSize + boxSpacing) * i,
                mass: 1,
                restitution: 0,
            });
            app.setBodyTexture(box, 'crate');
            world.addBody(box);
        }
    };

    static demo3 = (world: World, app: Application) => {
        // Demo 3: Pyramid of boxes
        app.setBackground('background');
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

                const box = Bodies.box({ width: boxSize, height: boxSize, x, y: -y, mass: 1, restitution: 0.001 });
                app.setBodyTexture(box, 'crate');
                world.addBody(box);
            }
        }
    };

    static demo4 = (world: World, app: Application) => {
        // Demo 4: A suspension bridge
        app.setBackground('background');
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

        const pillarLeft = Bodies.box({
            width: pillarWidth,
            height: pillarHeight,
            x: -pillarOffsetX,
            y: pillarPositionY,
            mass: 0,
        });
        const pillarRight = Bodies.box({
            width: pillarWidth,
            height: pillarHeight,
            x: pillarOffsetX,
            y: pillarPositionY,
            mass: 0,
        });

        app.setBodyTexture(pillarLeft, 'metal');
        app.setBodyTexture(pillarRight, 'metal');

        world.addBody(pillarLeft);
        world.addBody(pillarRight);

        const stepPositionY = pillarPositionY + pillarHeight / 2;
        const steps: RigidBody[] = [];

        for (let i = 0; i < stepCount; i++) {
            const x = (i - (stepCount - 1) / 2) * stepSpacing;

            const step = Bodies.box({ width: stepWidth, height: stepHeight, x, y: stepPositionY, mass: 5 });

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
        // Demo 5: A simple whip
        app.setBackground('background');
        this.generateFloor(world, app);
        this.generateFences(world, app);

        const whipAnchor = Bodies.box({ width: 60, height: 25, x: 0, y: 350, mass: 0 });
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
            const whipElement = Bodies.box({ width: 10, height: 50, x, y, mass: 1 });
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
        // Demo 6: A skeleton ragdoll
        app.setBackground('darkBackground');
        this.generateFloor(world, app);
        this.generateFences(world, app);

        // Add ragdoll parts (rigid bodies)
        const torso = Bodies.box({ width: 50, height: 100, x: 0, y: -100, mass: 3.0 });
        const head = Bodies.circle({ radius: 25, x: torso.position.x, y: torso.position.y + 50 + 25, mass: 5.0 });
        const leftArm = Bodies.box({
            width: 15,
            height: 70,
            x: torso.position.x - 32,
            y: torso.position.y + 10,
            mass: 1.0,
        });
        const rightArm = Bodies.box({
            width: 15,
            height: 70,
            x: torso.position.x + 32,
            y: torso.position.y + 10,
            mass: 1.0,
        });
        const leftLeg = Bodies.box({
            width: 20,
            height: 90,
            x: torso.position.x - 20,
            y: torso.position.y - 97,
            mass: 1.0,
        });
        const rightLeg = Bodies.box({
            width: 20,
            height: 90,
            x: torso.position.x + 20,
            y: torso.position.y - 97,
            mass: 1.0,
        });
        app.setBodyTexture(head, 'head');
        app.setBodyTexture(torso, 'torso');
        app.setBodyTexture(leftArm, 'leftArm');
        app.setBodyTexture(rightArm, 'rightArm');
        app.setBodyTexture(leftLeg, 'leftLeg');
        app.setBodyTexture(rightLeg, 'rightLeg');
        world.addBody(head);
        world.addBody(torso);
        world.addBody(leftArm);
        world.addBody(rightArm);
        world.addBody(leftLeg);
        world.addBody(rightLeg);

        const bodyTuning = JOINT_TUNING.ragdoll;

        // Add joints between ragdoll parts (distance constraints with one anchor point)
        const neck = this.createDistanceJoint(
            torso,
            head,
            bodyTuning,
            torso.position.addNew(new Vec2(0, 50)),
            head.position.addNew(new Vec2(0, -25)),
            0,
        );
        const leftShoulder = this.createDistanceJoint(
            torso,
            leftArm,
            bodyTuning,
            torso.position.addNew(new Vec2(-28, 45)),
            leftArm.position.addNew(new Vec2(5, 35)),
        );
        const rightShoulder = this.createDistanceJoint(
            torso,
            rightArm,
            bodyTuning,
            torso.position.addNew(new Vec2(28, 45)),
            rightArm.position.addNew(new Vec2(-5, 35)),
        );
        const leftHip = this.createDistanceJoint(
            torso,
            leftLeg,
            bodyTuning,
            torso.position.addNew(new Vec2(-20, -50)),
            leftLeg.position.addNew(new Vec2(0, 45)),
        );
        const rightHip = this.createDistanceJoint(
            torso,
            rightLeg,
            bodyTuning,
            torso.position.addNew(new Vec2(+20, -50)),
            rightLeg.position.addNew(new Vec2(0, 45)),
        );

        world.addJoint(neck);
        world.addJoint(leftShoulder);
        world.addJoint(rightShoulder);
        world.addJoint(leftHip);
        world.addJoint(rightHip);
    };

    static demo7 = (world: World, app: Application) => {
        // Demo 7: A plank
        app.setBackground('background');
        const floor = this.generateFloor(world, app);
        this.generateFences(world, app);

        const plank = Bodies.box({ width: 750, height: 20, x: 0, y: floor.position.y + 100, mass: 10 });
        app.setBodyTexture(plank, 'woodPlankCracked');
        world.addBody(plank);

        const joint = this.createDistanceJoint(floor, plank, JOINT_TUNING.plank, plank.position, plank.position);
        world.addJoint(joint);

        const triangleVertices = [new Vec2(-30, -30), new Vec2(30, -30), new Vec2(0, 33.5)];
        const triangle = Bodies.polygon({ vertices: triangleVertices, x: 0, y: floor.position.y + 55, mass: 0 });
        app.setBodyTexture(triangle, 'woodTriangle');
        world.addBody(triangle);

        const box1 = Bodies.box({
            width: 25,
            height: 25,
            x: plank.position.x - 350,
            y: plank.position.y + 25,
            mass: 1,
        });
        const box2 = Bodies.box({
            width: 25,
            height: 25,
            x: plank.position.x - 325,
            y: plank.position.y + 25,
            mass: 1,
        });
        const box3 = Bodies.box({
            width: 25,
            height: 25,
            x: plank.position.x - 337.5,
            y: plank.position.y + 50,
            mass: 1,
        });
        app.setBodyTexture(box1, 'crate');
        app.setBodyTexture(box2, 'crate');
        app.setBodyTexture(box3, 'crate');
        world.addBody(box1);
        world.addBody(box2);
        world.addBody(box3);

        const heavyBox = Bodies.box({
            width: 50,
            height: 50,
            x: plank.position.x + 350,
            y: Graphics.height() - 750,
            mass: 10,
        });
        app.setBodyTexture(heavyBox, 'metal');
        world.addBody(heavyBox);
    };

    static demo8 = (world: World, app: Application) => {
        // Demo 8: Cloth simulation
        app.setBackground('background');
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
                const particle = Bodies.circle({ radius: particleRadius, x, y, mass });
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
        // Demo 9: stress test
        app.setBackground('background');
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
        const startAnchor = Bodies.box({
            width: stepWidth * 2,
            height: stepWidth * 0.5,
            x: startX - stepWidth / 2,
            y: startY,
            mass: 0.0,
        });
        app.setBodyTexture(startAnchor, 'rockBridgeAnchor');
        world.addBody(startAnchor);

        // First connection uses the start anchor
        let lastStep = startAnchor;

        // Create steps
        for (let i = 1; i <= numSteps; i++) {
            const x = startX + i * spacing;

            // Optional sag: small vertical sinusoidal displacement
            const y = startY - Math.sin((i / numSteps) * Math.PI) * 10;

            const step = Bodies.circle({ radius: stepWidth * 0.5, x, y, mass: 3 });
            app.setBodyTexture(step, 'woodBridgeStep');
            world.addBody(step);

            // Joint anchor at left edge of this step
            const joint = this.createDistanceJoint(lastStep, step, tuning);
            world.addJoint(joint);

            lastStep = step;
        }

        // End anchor (static)
        const endAnchor = Bodies.box({
            width: stepWidth * 2,
            height: stepWidth * 0.5,
            x: lastStep.position.x + spacing + stepWidth / 2,
            y: startY,
            mass: 0.0,
        });
        app.setBodyTexture(endAnchor, 'rockBridgeAnchor');
        world.addBody(endAnchor);

        // Final joint anchor at right edge of last step
        const lastJoint = this.createDistanceJoint(lastStep, endAnchor, tuning);
        world.addJoint(lastJoint);

        const boxSizeLarge = 40;
        const numBoxLargeHorizontal = 10;

        for (let i = 0; i < numBoxLargeHorizontal; i++) {
            for (let j = 0; j < 10; j++) {
                const box = Bodies.box({
                    width: boxSizeLarge,
                    height: boxSizeLarge,
                    x: -(numBoxLargeHorizontal * boxSizeLarge) / 2 + boxSizeLarge / 2 + i * boxSizeLarge,
                    y: 500 + j * boxSizeLarge,
                    mass: 1,
                });
                app.setBodyTexture(box, 'woodBox');
                world.addBody(box);
            }
        }

        const boxSizeSmall = 20;
        const numBoxSmallHorizontal = 20;

        for (let i = 0; i < numBoxSmallHorizontal; i++) {
            for (let j = 0; j < 10; j++) {
                const box = Bodies.box({
                    width: boxSizeSmall,
                    height: boxSizeSmall,
                    x: -(numBoxSmallHorizontal * boxSizeSmall) / 2 + boxSizeSmall / 2 + i * boxSizeSmall,
                    y: 2000 + j * boxSizeSmall,
                    mass: 1,
                });
                app.setBodyTexture(box, 'metal');
                world.addBody(box);
            }
        }
    };

    static demo10 = (world: World, app: Application) => {
        // Demo 10: Continuous collision detection
        app.setBackground('background');
        this.generateFloor(world, app);
        this.generateFences(world, app);

        const fallingBox = Bodies.box({
            width: 50,
            height: 50,
            x: 500,
            y: 200,
            mass: 1,
            velocity: new Vec2(0, -2_350),
        });

        world.addBody(fallingBox);

        const bullet = Bodies.circle({
            radius: 5,
            x: -1000,
            y: 0,
            mass: 1,
            isBullet: true,
            velocity: new Vec2(20_000, 0),
        });

        world.addBody(bullet);

        Graphics.zoom = 0.35;
    };

    static populateStressDemo(
        world: World,
        bounds: CageBounds,
        createBody: (x: number, y: number, index: number) => RigidBody,
    ): void {
        const stepX = 34;
        const stepY = 23;
        const totalWidth = (STRESS_DEMO_COLUMNS - 1) * stepX;
        const totalHeight = (STRESS_DEMO_ROWS - 1) * stepY;
        const startX = -totalWidth / 2;
        const startY = bounds.innerBottom + (bounds.innerTop - bounds.innerBottom - totalHeight) / 2;

        for (let row = 0; row < STRESS_DEMO_ROWS; row++) {
            for (let col = 0; col < STRESS_DEMO_COLUMNS; col++) {
                const index = row * STRESS_DEMO_COLUMNS + col;
                const body = createBody(startX + col * stepX, startY + row * stepY, index);
                body.restitution = 0.05;
                body.friction = 0.6;
                world.addBody(body);
            }
        }
    }

    static demo11 = (world: World, app: Application) => {
        // Demo 11: 1000 circles inside a square cage
        app.setBackground('darkBackground');
        const cage = this.generateSquareCage(world, app);

        this.populateStressDemo(world, cage, (x, y) => {
            const body = Bodies.circle({ radius: 10, x, y, mass: 1 });
            app.setBodyFillColor(body, '#7bdff2');
            return body;
        });
    };

    static demo12 = (world: World, app: Application) => {
        // Demo 12: 1000 boxes inside a square cage
        app.setBackground('darkBackground');
        const cage = this.generateSquareCage(world, app);

        this.populateStressDemo(world, cage, (x, y) => {
            const body = Bodies.box({ width: 18, height: 18, x, y, mass: 1 });
            app.setBodyFillColor(body, '#f4a261');
            return body;
        });
    };

    static demo13 = (world: World, app: Application) => {
        // Demo 13: 1000 capsules inside a square cage
        app.setBackground('darkBackground');
        const cage = this.generateSquareCage(world, app);

        this.populateStressDemo(world, cage, (x, y) => {
            const body = Bodies.capsule({ halfHeight: 6, radius: 6, x, y, mass: 1 });
            app.setBodyFillColor(body, '#b8f2e6');
            return body;
        });
    };

    static demo14 = (world: World, app: Application) => {
        // Demo 14: 1000 random convex shapes inside a square cage
        app.setBackground('darkBackground');
        const cage = this.generateSquareCage(world, app);
        const palette = ['#f94144', '#f8961e', '#f9c74f', '#43aa8b', '#577590'];

        this.populateStressDemo(world, cage, (x, y, index) => {
            const radius = Utils.randomNumber(8, 12);
            const vertices = Math.round(Utils.randomNumber(3, 8));
            const body = Utils.randomConvexBody(x, y, radius, vertices);
            body.rotation = Utils.randomNumber(0, Math.PI * 2);
            body.shape.updateVertices(body.rotation, body.position);
            body.shape.updateAABB(body);
            app.setBodyFillColor(body, palette[index % palette.length]);
            return body;
        });
    };

    static demo15 = (world: World, app: Application) => {
        // Demo 15: orbiting particles around a central black hole
        app.setBackground('darkBackground');
        Graphics.zoom = 0.4;

        const BLACK_HOLE_ORBIT_PARTICLE_COUNT = 2_500;
        SETTINGS.applyGravity = false;

        const blackHole = Bodies.circle({ radius: 0.0001, x: 0, y: 0, mass: 600_000 });
        app.setBlackHole(blackHole);

        const particleRadius = 2.5;
        const particleMass = 0.02;
        const minOrbitRadius = 180;
        const maxOrbitRadius = 1000;
        const minGravityDistanceSquared = 80 * 80;
        const maxGravityDistanceSquared = 950 * 950;
        const particlePalette = ['#f8fbff', '#cdeeff', '#93dcff', '#ffe7a3'];

        for (let i = 0; i < BLACK_HOLE_ORBIT_PARTICLE_COUNT; i++) {
            const orbitRadius = Math.sqrt(
                Utils.randomNumber(minOrbitRadius * minOrbitRadius, maxOrbitRadius * maxOrbitRadius),
            );
            const angle = Utils.randomNumber(0, Math.PI * 2);
            const radialDirection = new Vec2(Math.cos(angle), Math.sin(angle));
            const position = radialDirection.scaleNew(orbitRadius);
            const effectiveDistanceSquared = Utils.clamp(
                orbitRadius * orbitRadius,
                minGravityDistanceSquared,
                maxGravityDistanceSquared,
            );
            const orbitSpeed =
                Math.sqrt((GRAVITY * blackHole.mass * orbitRadius) / effectiveDistanceSquared) *
                Utils.randomNumber(0.9, 1.12);
            const tangentialVelocity = radialDirection.leftPerpNew().scaleNew(orbitSpeed);
            const radialVelocity = radialDirection.scaleNew(orbitSpeed * Utils.randomNumber(-0.18, 0.12));
            const velocity = tangentialVelocity.addNew(radialVelocity);
            const particle = Bodies.circle({
                radius: particleRadius,
                x: position.x,
                y: position.y,
                mass: particleMass,
                velocity,
                restitution: 0,
                friction: 0,
            });

            app.setBodyFillColor(particle, particlePalette[Math.floor(Utils.randomNumber(0, particlePalette.length))]);
            world.addBody(particle);
        }
    };

    static demo16 = (world: World, app: Application) => {
        // Demo 16: Welded boxes
        app.setBackground('background');
        const floor = this.generateFloor(world, app);
        floor.rotation = -0.1;
        floor.position.y += 100;
        floor.shape.updateAABB(floor);
        floor.shape.updateVertices(floor.rotation, floor.position);

        const boxWidth = 20;
        const boxRows = 10;
        const xOffset = 250;
        const yOffset = 250;
        const spacing = 1;

        for (let i = 0; i < boxRows; i++) {
            for (let j = 0; j < boxRows; j++) {
                const box = Bodies.box({
                    width: boxWidth,
                    height: boxWidth,
                    x: boxWidth * i + spacing * i + xOffset,
                    y: boxWidth * j + spacing * j + yOffset,
                    mass: 1,
                    restitution: 0,
                });
                world.addBody(box);
            }
        }

        const boxes: RigidBody[][] = [];

        for (let i = 0; i < boxRows; i++) {
            boxes[i] = [];

            for (let j = 0; j < boxRows; j++) {
                const box = Bodies.box({
                    width: boxWidth,
                    height: boxWidth,
                    x: boxWidth * i + spacing * i - xOffset - boxWidth * boxRows,
                    y: boxWidth * j + spacing * j + yOffset,
                    mass: 1,
                    restitution: 0,
                });

                world.addBody(box);
                boxes[i][j] = box;
            }
        }

        // Create weld joints between adjacent boxes
        const jointFrequency = 30;
        const jointDamping = 0.5;

        for (let i = 0; i < boxRows; i++) {
            for (let j = 0; j < boxRows; j++) {
                const current = boxes[i][j];

                // weld to the box on the right
                if (i + 1 < boxRows) {
                    const right = boxes[i + 1][j];
                    const weld = new WeldJoint(current, right, undefined, jointFrequency, jointDamping);
                    weld.drawAnchor = true;
                    weld.drawConnectionLine = true;
                    world.addJoint(weld);
                }

                // weld to the box below
                if (j + 1 < boxRows) {
                    const below = boxes[i][j + 1];
                    const weld = new WeldJoint(current, below, undefined, jointFrequency, jointDamping);
                    weld.drawAnchor = true;
                    weld.drawConnectionLine = true;
                    world.addJoint(weld);
                }
            }
        }
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
        this.demo10,
        this.demo11,
        this.demo12,
        this.demo13,
        this.demo14,
        this.demo15,
        this.demo16,
    ];
}
