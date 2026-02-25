/*
 * This implementation is inspired by techniques used in Box2D
 * by Erin Catto, licensed under the MIT License.
 *
 * https://github.com/erincatto/box2d-lite
 */
import Application from '../core/Application';
import RigidBody from '../core/RigidBody';
import World from '../core/World';
import Graphics from '../graphics/Graphics';
import { DistanceJoint } from '../joint/DistanceJoint';
import Vec2 from '../math/Vec2';
import { BoxShape } from '../shapes/BoxShape';
import { CircleShape } from '../shapes/CircleShape';
import { PolygonShape } from '../shapes/PolygonShape';

// const FLOOR_WIDTH = 3200;
const FLOOR_WIDTH = 1000;
const FLOOR_HEIGHT = 50;
const FLOOR_POSITION_Y = -350;

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

    static generateFloor(world: World): RigidBody {
        const floor = new RigidBody(new BoxShape(FLOOR_WIDTH, FLOOR_HEIGHT), 0, FLOOR_POSITION_Y, 0.0);
        floor.setTexture('transparent');
        world.addBody(floor);
        return floor;
    }

    static generateFences(world: World): void {
        const fenceWidth = 50;
        const fenceHeight = 900 + FLOOR_HEIGHT;
        const leftFence = new RigidBody(
            new BoxShape(fenceWidth, fenceHeight),
            -(FLOOR_WIDTH / 2 + fenceWidth / 2),
            FLOOR_POSITION_Y + FLOOR_HEIGHT / 2 + fenceHeight / 2 - FLOOR_HEIGHT,
            0.0,
        );

        const rightFence = new RigidBody(
            new BoxShape(fenceWidth, fenceHeight),
            FLOOR_WIDTH / 2 + fenceWidth / 2,
            FLOOR_POSITION_Y + FLOOR_HEIGHT / 2 + fenceHeight / 2 - FLOOR_HEIGHT,
            0.0,
        );

        leftFence.setTexture('transparent');
        rightFence.setTexture('transparent');
        world.addBody(leftFence);
        world.addBody(rightFence);
    }

    static demo1 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 1: Single box demo
        this.generateFloor(world);
        this.generateFences(world);

        const box = new RigidBody(new BoxShape(60, 60), 0, 0, 1);
        box.angularVelocity = 5;
        box.setTexture('crate');
        // world.addBody(box);
    };

    static demo2 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 2: Stack of boxes
        this.generateFloor(world);
        this.generateFences(world);

        const numOfBoxes = 10;
        const boxSize = 60;
        const boxSpacing = 10;

        for (let i = 0; i < numOfBoxes; i++) {
            const box = new RigidBody(new BoxShape(boxSize, boxSize), 0, -200 + (boxSize + boxSpacing) * i, 1);
            box.restitution = 0;
            box.setTexture('crate');
            world.addBody(box);
        }
    };

    static demo3 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 3: Pyramid of boxes
        this.generateFloor(world);
        this.generateFences(world);

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
                box.setTexture('crate');
                box.restitution = 0.001;
                world.addBody(box);
            }
        }
    };

    static demo4 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 4: A suspension bridge
        this.generateFloor(world);
        this.generateFences(world);

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

        pillarLeft.setTexture('metal');
        pillarRight.setTexture('metal');

        world.addBody(pillarLeft);
        world.addBody(pillarRight);

        const stepPositionY = pillarPositionY + pillarHeight / 2;
        const steps: RigidBody[] = [];

        for (let i = 0; i < stepCount; i++) {
            const x = (i - (stepCount - 1) / 2) * stepSpacing;

            const step = new RigidBody(new BoxShape(stepWidth, stepHeight), x, stepPositionY, 5);

            step.setTexture('woodBox');
            world.addBody(step);
            steps.push(step);
        }

        const distance = -1;
        const frequency = 7;
        const dampingRadio = 0.9;
        const jointMass = -1;

        // joints between steps
        for (let i = 0; i < steps.length - 1; i++) {
            const a = steps[i];
            const b = steps[i + 1];

            const joint = new DistanceJoint(
                a,
                b,
                a.position.addNew(new Vec2(stepWidth / 2, 0)),
                b.position.subNew(new Vec2(stepWidth / 2, 0)),
                distance,
                frequency,
                dampingRadio,
                jointMass,
            );

            world.addJoint(joint);
        }

        // left pillar → first step
        const leftJoint = new DistanceJoint(
            pillarLeft,
            steps[0],
            pillarLeft.position.addNew(new Vec2(25, 0)).addNew(new Vec2(0, pillarHeight / 2)),
            steps[0].position.subNew(new Vec2(stepWidth / 2, 0)),
            distance,
            frequency,
            dampingRadio,
            jointMass,
        );

        // right pillar → last step
        const rightJoint = new DistanceJoint(
            pillarRight,
            steps[steps.length - 1],
            pillarRight.position.subNew(new Vec2(25, 0)).addNew(new Vec2(0, pillarHeight / 2)),
            steps[steps.length - 1].position.addNew(new Vec2(stepWidth / 2, 0)),
            distance,
            frequency,
            dampingRadio,
            jointMass,
        );

        world.addJoint(leftJoint);
        world.addJoint(rightJoint);
    };

    static demo5 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 5: A simple whip
        this.generateFloor(world);
        this.generateFences(world);

        const whipAnchor = new RigidBody(new BoxShape(60, 25), 0, 300, 0);
        whipAnchor.setTexture('rockBridgeAnchor');
        world.addBody(whipAnchor);

        let last = whipAnchor;
        const whipElementHeight = 50;

        const distance = -1; // -1 = the initial distance
        const frequency = 7;
        const dampingRadio = 1;
        const jointMass = 5;

        for (let i = 0; i < 10; i++) {
            const x = whipAnchor.position.x;
            const y =
                i === 0
                    ? whipAnchor.position.y - whipElementHeight
                    : whipAnchor.position.y - (whipElementHeight + 60 * i);
            const whipElement = new RigidBody(new BoxShape(10, 50), x, y, 1);
            whipElement.setTexture('crate');
            world.addBody(whipElement);

            const j = new DistanceJoint(
                last,
                whipElement,
                last.position.subNew(new Vec2(0, whipElementHeight / 2)),
                whipElement.position.addNew(new Vec2(0, whipElementHeight / 2)),
                distance,
                frequency,
                dampingRadio,
                jointMass,
            );
            world.addJoint(j);

            last = whipElement;
        }
    };

    static demo6 = (world: World, app: Application) => {
        app.setBackground('darkBackground');

        // Demo 6: A skeleton ragdoll
        this.generateFloor(world);
        this.generateFences(world);

        // Add ragdoll parts (rigid bodies)
        const bob = new RigidBody(new CircleShape(5), 0, 100, 0.0);
        const head = new RigidBody(new CircleShape(25), bob.position.x, bob.position.y - 70, 5.0);
        const torso = new RigidBody(new BoxShape(50, 100), head.position.x, head.position.y - 80, 3.0);
        const leftArm = new RigidBody(new BoxShape(15, 70), torso.position.x - 32, torso.position.y + 10, 1.0);
        const rightArm = new RigidBody(new BoxShape(15, 70), torso.position.x + 32, torso.position.y + 10, 1.0);
        const leftLeg = new RigidBody(new BoxShape(20, 90), torso.position.x - 20, torso.position.y - 97, 1.0);
        const rightLeg = new RigidBody(new BoxShape(20, 90), torso.position.x + 20, torso.position.y - 97, 1.0);
        bob.setTexture('bob');
        head.setTexture('head');
        torso.setTexture('torso');
        leftArm.setTexture('leftArm');
        rightArm.setTexture('rightArm');
        leftLeg.setTexture('leftLeg');
        rightLeg.setTexture('rightLeg');
        world.addBody(bob);
        world.addBody(head);
        world.addBody(torso);
        world.addBody(leftArm);
        world.addBody(rightArm);
        world.addBody(leftLeg);
        world.addBody(rightLeg);

        // Add joints between ragdoll parts (distance constraints with one anchor point)
        const string = new DistanceJoint(bob, head, bob.position, head.position);
        const neck = new DistanceJoint(
            head,
            torso,
            head.position.subNew(new Vec2(0, 25)),
            torso.position.addNew(new Vec2(0, 50)),
        );
        const leftShoulder = new DistanceJoint(
            torso,
            leftArm,
            torso.position.addNew(new Vec2(-28, 45)),
            leftArm.position.addNew(new Vec2(5, 35)),
        );
        const rightShoulder = new DistanceJoint(
            torso,
            rightArm,
            torso.position.addNew(new Vec2(28, 45)),
            rightArm.position.addNew(new Vec2(-5, 35)),
        );
        const leftHip = new DistanceJoint(
            torso,
            leftLeg,
            torso.position.addNew(new Vec2(-20, -50)),
            leftLeg.position.addNew(new Vec2(0, 45)),
        );
        const rightHip = new DistanceJoint(
            torso,
            rightLeg,
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
        const floor = this.generateFloor(world);
        this.generateFences(world);

        const plank = new RigidBody(new BoxShape(750, 20), 0, floor.position.y + 100, 10);
        plank.setTexture('woodPlankCracked');
        world.addBody(plank);

        const joint = new DistanceJoint(floor, plank, plank.position, plank.position);
        world.addJoint(joint);

        const triangleVertices = [new Vec2(-30, -30), new Vec2(30, -30), new Vec2(0, 33.5)];
        const triangle = new RigidBody(new PolygonShape(triangleVertices), 0, floor.position.y + 55, 0);
        triangle.setTexture('woodTriangle');
        world.addBody(triangle);

        const box1 = new RigidBody(new BoxShape(25, 25), plank.position.x - 350, plank.position.y + 25, 1);
        const box2 = new RigidBody(new BoxShape(25, 25), plank.position.x - 325, plank.position.y + 25, 1);
        const box3 = new RigidBody(new BoxShape(25, 25), plank.position.x - 337.5, plank.position.y + 50, 1);
        box1.setTexture('crate');
        box2.setTexture('crate');
        box3.setTexture('crate');
        world.addBody(box1);
        world.addBody(box2);
        world.addBody(box3);

        const heavyBox = new RigidBody(new BoxShape(50, 50), plank.position.x + 350, Graphics.height() - 750, 10);
        heavyBox.setTexture('metal');
        world.addBody(heavyBox);
    };

    static demo8 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 8: Cloth simulation
        this.generateFloor(world);
        this.generateFences(world);

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

        // Create joints
        for (let row = 0; row < rows; row++) {
            for (let col = 0; col < cols; col++) {
                const p = particles[row][col];

                // Connect to particle above
                if (row > 0) {
                    const above = particles[row - 1][col];
                    const joint = new DistanceJoint(above, p);
                    joint.drawConnectionLine = true;
                    world.addJoint(joint);
                }

                // Connect to particle to the right
                if (col < cols - 1 && row > 0) {
                    const right = particles[row][col + 1];
                    const joint = new DistanceJoint(p, right);
                    joint.drawConnectionLine = true;
                    world.addJoint(joint);
                }
            }
        }
    };

    static demo9 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 9: stress test
        this.generateFloor(world);
        this.generateFences(world);

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
        startAnchor.setTexture('rockBridgeAnchor');
        world.addBody(startAnchor);

        // First connection uses the start anchor
        let lastStep = startAnchor;

        // Create steps
        for (let i = 1; i <= numSteps; i++) {
            const x = startX + i * spacing;

            // Optional sag: small vertical sinusoidal displacement
            const y = startY - Math.sin((i / numSteps) * Math.PI) * 10;

            const step = new RigidBody(new CircleShape(stepWidth * 0.5), x, y, 3);
            step.setTexture('woodBridgeStep');
            world.addBody(step);

            // Joint anchor at left edge of this step
            const joint = new DistanceJoint(lastStep, step);
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
        endAnchor.setTexture('rockBridgeAnchor');
        world.addBody(endAnchor);

        // Final joint anchor at right edge of last step
        const lastJoint = new DistanceJoint(lastStep, endAnchor);
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
                box.setTexture('woodBox');
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
                box.setTexture('metal');
                world.addBody(box);
            }
        }
    };

    static demo0 = (world: World, app: Application) => {
        app.setBackground('background');

        // Demo 0: a complex scene
        const floor = this.generateFloor(world);
        this.generateFences(world);

        // Add bird
        const bird = new RigidBody(new CircleShape(45), -550, -200, 3.0);
        bird.setTexture('birdRed');
        world.addBody(bird);

        // Add a stack of boxes
        for (let i = 1; i <= 4; i++) {
            const mass = 10.0 / i;
            const box = new RigidBody(new BoxShape(50, 50), -300, floor.position.y + FLOOR_HEIGHT / 2 + i * 55, mass);
            box.setTexture('woodBox');
            box.friction = 0.9;
            box.restitution = 0.1;
            world.addBody(box);
        }

        // Add structure with blocks
        const plank1 = new RigidBody(new BoxShape(50, 150), -30, floor.position.y + FLOOR_HEIGHT / 2 + 100, 5.0);
        const plank2 = new RigidBody(new BoxShape(50, 150), 130, floor.position.y + FLOOR_HEIGHT / 2 + 100, 5.0);
        const plank3 = new RigidBody(new BoxShape(250, 25), 50, floor.position.y + FLOOR_HEIGHT / 2 + 200, 2.0);
        plank1.setTexture('woodPlankSolid');
        plank2.setTexture('woodPlankSolid');
        plank3.setTexture('woodPlankCracked');
        world.addBody(plank1);
        world.addBody(plank2);
        world.addBody(plank3);

        // Add a triangle polygon
        const triangleVertices = [new Vec2(-30, -30), new Vec2(30, -30), new Vec2(0, 30)];
        const triangle = new RigidBody(
            new PolygonShape(triangleVertices),
            plank3.position.x,
            plank3.position.y + 50,
            0.5,
        );
        triangle.setTexture('woodTriangle');
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
                box.setTexture('woodBox');
                world.addBody(box);
            }
        }

        // Add a bridge of connected steps and joints
        const numSteps = 10;
        const spacing = 33;

        // Start anchor (static)
        const stepHeight = 20;
        const startStep = new RigidBody(new BoxShape(80, stepHeight), -500, 200, 0.0);
        startStep.setTexture('rockBridgeAnchor');
        world.addBody(startStep);

        // The first connection should be from the anchor, not the floor
        let last = startStep;

        for (let i = 1; i <= numSteps; i++) {
            const x = startStep.position.x + 30 + i * spacing;
            const y = startStep.position.y - Math.sin((i / numSteps) * Math.PI) * 10;
            const mass = 3;

            const step = new RigidBody(new CircleShape(15), x, y, mass);
            step.setTexture('woodBridgeStep');
            world.addBody(step);

            // Connect previous link to this link
            const joint = new DistanceJoint(last, step);
            world.addJoint(joint);

            last = step;
        }

        // Final anchor
        const endStep = new RigidBody(new BoxShape(80, stepHeight), last.position.x + 60, startStep.position.y, 0.0);
        endStep.setTexture('rockBridgeAnchor');
        world.addBody(endStep);

        const lastJoint = new DistanceJoint(last, endStep);
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
        pig1.setTexture('pig1');
        pig2.setTexture('pig2');
        pig3.setTexture('pig1');
        pig4.setTexture('pig2');
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
