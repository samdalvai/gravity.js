import Graphics from '../Graphics';
import Vec2 from '../math/Vec2';
import { JointConstraint } from '../physics/JointConstraint';
import RigidBody from '../physics/RigidBody';
import { BoxShape, CircleShape, PolygonShape } from '../physics/Shape';
import World from '../physics/World';

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

    static generateFloor = (world: World): RigidBody => {
        const floor = new RigidBody(
            new BoxShape(Graphics.width(), 200),
            Graphics.width() / 2.0,
            Graphics.height() - 100,
            0.0,
        );
        world.addBody(floor);
        return floor;
    };

    static generateFences = (world: World): void => {
        const fenceWidth = 50;
        const leftFence = new RigidBody(
            new BoxShape(fenceWidth, Graphics.height() - 200),
            -(fenceWidth / 2),
            Graphics.height() / 2.0 - 100,
            0.0,
        );
        const rightFence = new RigidBody(
            new BoxShape(fenceWidth, Graphics.height() - 200),
            Graphics.width() + fenceWidth / 2,
            Graphics.height() / 2.0 - 100,
            0.0,
        );
        world.addBody(leftFence);
        world.addBody(rightFence);
    };

    static demo1 = (world: World) => {
        // Demo 1: Single box demo
        this.generateFloor(world);
        this.generateFences(world);

        const box = new RigidBody(new BoxShape(60, 60), Graphics.width() / 2.0, Graphics.height() - 300, 1);
        box.setTexture('crate');
        world.addBody(box);
    };

    static demo2 = (world: World) => {
        // Demo 2: Stack of boxes
        this.generateFloor(world);
        this.generateFences(world);

        const numOfBoxes = 8;
        const boxSize = 60;
        const boxSpacing = 10;

        for (let i = 0; i < numOfBoxes; i++) {
            const box = new RigidBody(
                new BoxShape(boxSize, boxSize),
                Graphics.width() / 2.0,
                Graphics.height() - 300 - (boxSize + boxSpacing) * i,
                1,
            );
            box.restitution = 0.1;
            box.setTexture('crate');
            world.addBody(box);
        }
    };

    static demo3 = (world: World) => {
        // Demo 3: Pyramid of boxes
        const floor = this.generateFloor(world);
        this.generateFences(world);
        const floorHeight = 200;

        const boxSize = 60;
        const boxSpacing = 10;
        const rows = 10;

        const centerX = Graphics.width() / 2;
        const baseY = floor.position.y - floorHeight / 2 - 100;

        for (let row = 0; row < rows; row++) {
            const boxesInRow = rows - row;
            const rowWidth = boxesInRow * boxSize;

            for (let col = 0; col < boxesInRow; col++) {
                const x = centerX - rowWidth / 2 + boxSize / 2 + col * boxSize;
                const y = baseY - row * (boxSize + boxSpacing);

                const box = new RigidBody(new BoxShape(boxSize, boxSize), x, y, 1);
                box.restitution = 0.1;
                box.setTexture('crate');
                box.restitution = 0.001;
                world.addBody(box);
            }
        }
    };

    static demo4 = (world: World) => {
        // Demo 4: As suspension bridge
        this.generateFloor(world);
        this.generateFences(world);

        // Suspension Bridge Creation
        const numSteps = 10;
        const stepWidth = 40;
        const spacing = stepWidth + 2.5; // distance between centers
        const startX = Graphics.width() / 2 - (numSteps * spacing) / 2;
        const startY = Graphics.height() / 2;
        const softness = 0.02;
        const bias = 0.1;

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
            const y = startY + Math.sin((i / numSteps) * Math.PI) * 10;

            const step = new RigidBody(new CircleShape(stepWidth * 0.5), x, y, 3);
            step.setTexture('woodBridgeStep');
            world.addBody(step);

            // Joint anchor at left edge of this step
            const anchor = step.position.subNew(new Vec2(stepWidth / 2, 0));
            const joint = new JointConstraint(lastStep, step, anchor, softness, bias);
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
        const finalAnchor = lastStep.position.addNew(new Vec2(stepWidth / 2, 0));
        const lastJoint = new JointConstraint(lastStep, endAnchor, finalAnchor, softness, bias);
        world.addJoint(lastJoint);
    };

    static demo5 = (world: World) => {
        // Demo 5: A simple whip
        this.generateFloor(world);
        this.generateFences(world);

        const whipAnchor = new RigidBody(new BoxShape(40, 20), Graphics.width() / 2, 100, 0);
        whipAnchor.setTexture('rockBridgeAnchor');
        world.addBody(whipAnchor);

        let last = whipAnchor;

        for (let i = 0; i < 10; i++) {
            const x = whipAnchor.position.x;
            const y = i === 0 ? whipAnchor.position.y + 40 : whipAnchor.position.y + 40 + 60 * i;
            const whipElement = new RigidBody(new BoxShape(10, 50), x, y, 1);
            whipElement.setTexture('crate');
            world.addBody(whipElement);

            const anchor = whipElement.position.subNew(new Vec2(0, 25));
            const j = new JointConstraint(last, whipElement, anchor);
            world.addJoint(j);

            last = whipElement;
        }
    };

    static demo6 = (world: World) => {
        // Demo 6: A skeleton ragdoll
        this.generateFloor(world);
        this.generateFences(world);

        // Add ragdoll parts (rigid bodies)
        const bob = new RigidBody(new CircleShape(5), Graphics.width() / 2.0, Graphics.height() / 2.0 - 200, 0.0);
        const head = new RigidBody(new CircleShape(25), bob.position.x, bob.position.y + 70, 5.0);
        const torso = new RigidBody(new BoxShape(50, 100), head.position.x, head.position.y + 80, 3.0);
        const leftArm = new RigidBody(new BoxShape(15, 70), torso.position.x - 32, torso.position.y - 10, 1.0);
        const rightArm = new RigidBody(new BoxShape(15, 70), torso.position.x + 32, torso.position.y - 10, 1.0);
        const leftLeg = new RigidBody(new BoxShape(20, 90), torso.position.x - 20, torso.position.y + 97, 1.0);
        const rightLeg = new RigidBody(new BoxShape(20, 90), torso.position.x + 20, torso.position.y + 97, 1.0);
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
        const string = new JointConstraint(bob, head, bob.position);
        const neck = new JointConstraint(head, torso, head.position.addNew(new Vec2(0, 25)));
        const leftShoulder = new JointConstraint(torso, leftArm, torso.position.addNew(new Vec2(-28, -45)));
        const rightShoulder = new JointConstraint(torso, rightArm, torso.position.addNew(new Vec2(+28, -45)));
        const leftHip = new JointConstraint(torso, leftLeg, torso.position.addNew(new Vec2(-20, +50)));
        const rightHip = new JointConstraint(torso, rightLeg, torso.position.addNew(new Vec2(+20, +50)));

        world.addJoint(string);
        world.addJoint(neck);
        world.addJoint(leftShoulder);
        world.addJoint(rightShoulder);
        world.addJoint(leftHip);
        world.addJoint(rightHip);
    };

    static demo7 = (world: World) => {
        // Demo 7: A plank
        const floor = this.generateFloor(world);
        this.generateFences(world);

        const plank = new RigidBody(new BoxShape(750, 20), Graphics.width() / 2.0, Graphics.height() - 275, 10);
        plank.setTexture('woodPlankCracked');
        world.addBody(plank);

        const triangleVertices = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -33.5)];
        const triangle = new RigidBody(new PolygonShape(triangleVertices), floor.position.x, floor.position.y - 130, 0);
        triangle.setTexture('woodTriangle');
        world.addBody(triangle);

        const box1 = new RigidBody(new BoxShape(25, 25), plank.position.x - 350, Graphics.height() - 400, 1);
        const box2 = new RigidBody(new BoxShape(25, 25), plank.position.x - 325, Graphics.height() - 400, 1);
        const box3 = new RigidBody(new BoxShape(25, 25), plank.position.x - 337.5, Graphics.height() - 425, 1);
        box1.setTexture('crate');
        box2.setTexture('crate');
        box3.setTexture('crate');
        world.addBody(box1);
        world.addBody(box2);
        world.addBody(box3);

        const heavyBox = new RigidBody(new BoxShape(50, 50), plank.position.x + 350, Graphics.height() - 750, 10);
        heavyBox.setTexture('metal');
        world.addBody(heavyBox);

        const joint = new JointConstraint(floor, plank, plank.position);
        world.addJoint(joint);
    };

    static demo8 = (world: World) => {
        // Demo 8: Cloth simulation
        this.generateFloor(world);
        this.generateFences(world);

        const anchor1 = new RigidBody(new CircleShape(5), 500, Graphics.height() / 2, 0);
        const anchor2 = new RigidBody(new CircleShape(5), 550, Graphics.height() / 2, 0);
        const anchor3 = new RigidBody(new CircleShape(5), 600, Graphics.height() / 2, 0);
        const anchor4 = new RigidBody(new CircleShape(5), 650, Graphics.height() / 2, 0);

        world.addBody(anchor1);
        world.addBody(anchor2);
        world.addBody(anchor3);
        world.addBody(anchor4);

        const particle1 = new RigidBody(new CircleShape(5), 500, Graphics.height() / 2 + 50, 1);
        const particle2 = new RigidBody(new CircleShape(5), 550, Graphics.height() / 2 + 50, 1);
        const particle3 = new RigidBody(new CircleShape(5), 600, Graphics.height() / 2 + 50, 1);
        const particle4 = new RigidBody(new CircleShape(5), 650, Graphics.height() / 2 + 50, 1);

        world.addBody(particle1);
        world.addBody(particle2);
        world.addBody(particle3);
        world.addBody(particle4);

        const jointAnchor1Particle1 = new JointConstraint(anchor1, particle1, anchor1.position);
        const jointAnchor2Particle2 = new JointConstraint(anchor2, particle2, anchor2.position);
        const jointAnchor3Particle3 = new JointConstraint(anchor3, particle3, anchor3.position);
        const jointAnchor4Particle4 = new JointConstraint(anchor4, particle4, anchor4.position);

        world.addJoint(jointAnchor1Particle1);
        world.addJoint(jointAnchor2Particle2);
        world.addJoint(jointAnchor3Particle3);
        world.addJoint(jointAnchor4Particle4);

        const jointParticle1Particle2 = new JointConstraint(particle1, particle2, particle1.position);
        const jointParticle2Particle3 = new JointConstraint(particle2, particle3, particle2.position);
        const jointParticle3Particle4 = new JointConstraint(particle3, particle4, particle3.position);

        world.addJoint(jointParticle1Particle2);
        world.addJoint(jointParticle2Particle3);
        world.addJoint(jointParticle3Particle4);

        // const circle1 = new RigidBody(new CircleShape(20), 500, Graphics.height() / 2 + 50, 1);
        // const circle2 = new RigidBody(new CircleShape(20), 600, Graphics.height() / 2 + 50, 1);
        // world.addBody(circle1);
        // world.addBody(circle2);

        // const joint = new JointConstraint(circle1, circle2, circle1.position);
        // world.addJoint(joint);
    };

    static demo9 = (world: World) => {
        // Demo 9: stress test
        this.generateFloor(world);
        this.generateFences(world);

        // Suspension Bridge Creation
        const numSteps = 10;
        const stepWidth = 40;
        const spacing = stepWidth + 2.5; // distance between centers
        const startX = Graphics.width() / 2 - (numSteps * spacing) / 2 - stepWidth / 2;
        const startY = Graphics.height() / 2;
        const softness = 0.002;
        const bias = 0.1;

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
            const y = startY + Math.sin((i / numSteps) * Math.PI) * 10;

            const step = new RigidBody(new CircleShape(stepWidth * 0.5), x, y, 3);
            step.setTexture('woodBridgeStep');
            world.addBody(step);

            // Joint anchor at left edge of this step
            const anchor = step.position.subNew(new Vec2(stepWidth / 2, 0));
            const joint = new JointConstraint(lastStep, step, anchor, softness, bias);
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
        const finalAnchor = lastStep.position.addNew(new Vec2(stepWidth / 2, 0));
        const lastJoint = new JointConstraint(lastStep, endAnchor, finalAnchor, softness, bias);
        world.addJoint(lastJoint);

        const boxSizeLarge = 40;
        const numBoxLargeHorizontal = 10;

        for (let i = 0; i < numBoxLargeHorizontal; i++) {
            for (let j = 0; j < 10; j++) {
                const box = new RigidBody(
                    new BoxShape(boxSizeLarge, boxSizeLarge),
                    Graphics.width() / 2 -
                        (numBoxLargeHorizontal * boxSizeLarge) / 2 +
                        boxSizeLarge / 2 +
                        i * boxSizeLarge,
                    -500 + j * boxSizeLarge,
                    1,
                );
                world.addBody(box);
            }
        }

        const boxSizeSmall = 20;
        const numBoxSmallHorizontal = 20;

        for (let i = 0; i < numBoxSmallHorizontal; i++) {
            for (let j = 0; j < 10; j++) {
                const box = new RigidBody(
                    new BoxShape(boxSizeSmall, boxSizeSmall),
                    Graphics.width() / 2 -
                        (numBoxSmallHorizontal * boxSizeSmall) / 2 +
                        boxSizeSmall / 2 +
                        i * boxSizeSmall,
                    -2000 + j * boxSizeSmall,
                    1,
                );
                world.addBody(box);
            }
        }
    };

    static demo0 = (world: World) => {
        // Demo 0: a complex scene

        const floor = this.generateFloor(world);
        this.generateFences(world);

        // Add bird
        const bird = new RigidBody(new CircleShape(45), 100, Graphics.height() / 2.0 + 220, 3.0);
        bird.setTexture('birdRed');
        world.addBody(bird);

        // Add a stack of boxes
        for (let i = 1; i <= 4; i++) {
            const mass = 10.0 / i;
            const box = new RigidBody(new BoxShape(50, 50), 600, floor.position.y - 100 - i * 55, mass);
            box.setTexture('woodBox');
            box.friction = 0.9;
            box.restitution = 0.1;
            world.addBody(box);
        }

        // Add structure with blocks
        const plank1 = new RigidBody(
            new BoxShape(50, 150),
            Graphics.width() / 2.0 + 20,
            floor.position.y - 100 - 100,
            5.0,
        );
        const plank2 = new RigidBody(
            new BoxShape(50, 150),
            Graphics.width() / 2.0 + 180,
            floor.position.y - 100 - 100,
            5.0,
        );
        const plank3 = new RigidBody(
            new BoxShape(250, 25),
            Graphics.width() / 2.0 + 100,
            floor.position.y - 100 - 200,
            2.0,
        );
        plank1.setTexture('woodPlankSolid');
        plank2.setTexture('woodPlankSolid');
        plank3.setTexture('woodPlankCracked');
        world.addBody(plank1);
        world.addBody(plank2);
        world.addBody(plank3);

        // Add a triangle polygon
        const triangleVertices = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const triangle = new RigidBody(
            new PolygonShape(triangleVertices),
            plank3.position.x,
            plank3.position.y - 50,
            0.5,
        );
        triangle.setTexture('woodTriangle');
        world.addBody(triangle);

        // Add a pyramid of boxes
        const numRows = 5;
        for (let col = 0; col < numRows; col++) {
            for (let row = 0; row < col; row++) {
                const x = plank3.position.x + 200 + col * 50 - row * 25;
                const y = floor.position.y - 100 - 50 - row * 52;
                const mass = 5 / (row + 1);
                const box = new RigidBody(new BoxShape(50, 50), x, y, mass);
                box.friction = 0.9;
                box.restitution = 0.0;
                box.setTexture('woodBox');
                world.addBody(box);
            }
        }

        // Add a bridge of connected steps and joints
        // TODO: use bdirge from suspension bridge demo
        const numSteps = 10;
        const spacing = 33;

        // Start anchor (static)
        const startStep = new RigidBody(new BoxShape(80, 20), 200, 200, 0.0);
        startStep.setTexture('rockBridgeAnchor');
        world.addBody(startStep);

        // The first connection should be from the anchor, not the floor
        let last = startStep;

        for (let i = 1; i <= numSteps; i++) {
            const x = startStep.position.x + 30 + i * spacing;
            const y = startStep.position.y + 20;
            const mass = 3;

            const step = new RigidBody(new CircleShape(15), x, y, mass);
            step.setTexture('woodBridgeStep');
            world.addBody(step);

            // Connect previous link to this link
            const joint = new JointConstraint(last, step, step.position);
            world.addJoint(joint);

            last = step;
        }

        // Final anchor
        const endStep = new RigidBody(new BoxShape(80, 20), last.position.x + 60, last.position.y - 20, 0.0);
        endStep.setTexture('rockBridgeAnchor');
        world.addBody(endStep);

        const lastJoint = new JointConstraint(last, endStep, endStep.position.addNew(new Vec2(15, 0)));
        world.addJoint(lastJoint);

        // Add pigs
        const pig1 = new RigidBody(new CircleShape(30), plank1.position.x + 80, floor.position.y - 100 - 50, 3.0);
        const pig2 = new RigidBody(new CircleShape(30), plank2.position.x + 400, floor.position.y - 100 - 50, 3.0);
        const pig3 = new RigidBody(new CircleShape(30), plank2.position.x + 460, floor.position.y - 100 - 50, 3.0);
        const pig4 = new RigidBody(new CircleShape(30), 220, 130, 1.0);
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
