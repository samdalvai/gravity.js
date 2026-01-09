import Graphics from '../Graphics';
import Vec2 from '../math/Vec2';
import Body from '../physics/Body';
import { JointConstraint } from '../physics/Constraint';
import { BoxShape, CircleShape, PolygonShape } from '../physics/Shape';
import World from '../physics/World';

export default class Demo {
    static demoStrings = [
        'Demo 0: ....',
        'Demo 1: A single box',
        'Demo 2: A pyramid of boxes',
        'Demo 3: A suspension bridge',
        'Demo 4: As simple whip',
        'Demo 5: A skeleton ragdoll',
        'Demo 6: ....',
        'Demo 7: ....',
        'Demo 8: Stress test',
        'Demo 9: A complex scene',
    ];

    static generateFloor = (world: World): Body => {
        const floor = new Body(
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
        const leftFence = new Body(
            new BoxShape(fenceWidth, Graphics.height() - 200),
            -(fenceWidth / 2),
            Graphics.height() / 2.0 - 100,
            0.0,
        );
        const rightFence = new Body(
            new BoxShape(fenceWidth, Graphics.height() - 200),
            Graphics.width() + fenceWidth / 2,
            Graphics.height() / 2.0 - 100,
            0.0,
        );
        world.addBody(leftFence);
        world.addBody(rightFence);
    };

    static demo0 = (world: World) => {
        // Demo 0: ....
    };

    static demo1 = (world: World) => {
        // Demo 1: Single box demo
        this.generateFloor(world);
        this.generateFences(world);
        const box = new Body(new BoxShape(60, 60), Graphics.width() / 2.0, Graphics.height() - 300, 10);
        box.setTexture('crate');
        world.addBody(box);
    };

    static demo2 = (world: World) => {
        // Demo 2: Pyramid of boxes
        const floor = this.generateFloor(world);
        this.generateFences(world);
        const floorHeight = 200;

        const boxSize = 60;
        const rows = 10;

        const centerX = Graphics.width() / 2;
        const baseY = floor.position.y - floorHeight / 2 - 100;

        for (let row = 0; row < rows; row++) {
            const boxesInRow = rows - row;
            const rowWidth = boxesInRow * boxSize;

            for (let col = 0; col < boxesInRow; col++) {
                const x = centerX - rowWidth / 2 + boxSize / 2 + col * boxSize;
                const y = baseY - row * boxSize;

                const box = new Body(new BoxShape(boxSize, boxSize), x, y, 10);
                box.setTexture('crate');
                box.restitution = 0.001;
                world.addBody(box);
            }
        }
    };

    static demo3 = (world: World) => {
        // Demo 3: As suspension bridge
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
        const startAnchor = new Body(new BoxShape(stepWidth * 2, stepWidth * 0.5), startX - stepWidth / 2, startY, 0.0);
        startAnchor.setTexture('rockBridgeAnchor');
        world.addBody(startAnchor);

        // First connection uses the start anchor
        let lastStep = startAnchor;

        // Create steps
        for (let i = 1; i <= numSteps; i++) {
            const x = startX + i * spacing;

            // Optional sag: small vertical sinusoidal displacement
            const y = startY + Math.sin((i / numSteps) * Math.PI) * 10;

            const step = new Body(new CircleShape(stepWidth * 0.5), x, y, 3);
            step.setTexture('woodBridgeStep');
            world.addBody(step);

            // Joint anchor at left edge of this step
            const anchor = step.position.subNew(new Vec2(stepWidth / 2, 0));
            const joint = new JointConstraint(lastStep, step, anchor, softness, bias);
            world.addJoint(joint);

            lastStep = step;
        }

        // End anchor (static)
        const endAnchor = new Body(
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

    static demo4 = (world: World) => {
        // Demo 4: A simple whip
        this.generateFloor(world);
        this.generateFences(world);

        const whipAnchor = new Body(new BoxShape(40, 20), Graphics.width() / 2, 100, 0);
        whipAnchor.setTexture('rockBridgeAnchor');
        world.addBody(whipAnchor);

        let last = whipAnchor;

        for (let i = 0; i < 10; i++) {
            const x = whipAnchor.position.x;
            const y = i === 0 ? whipAnchor.position.y + 40 : whipAnchor.position.y + 40 + 60 * i;
            const whipElement = new Body(new BoxShape(10, 50), x, y, 1);
            whipElement.setTexture('crate');
            world.addBody(whipElement);

            const anchor = whipElement.position.subNew(new Vec2(0, 25));
            const j = new JointConstraint(last, whipElement, anchor);
            world.addJoint(j);

            last = whipElement;
        }
    };

    static demo5 = (world: World) => {
        // Demo 5: A skeleton ragdoll
        this.generateFloor(world);
        this.generateFences(world);

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

    static demo6 = (world: World) => {
        // Demo 6: Debug demo
        this.generateFloor(world);
        this.generateFences(world);
    };

    static demo7 = (world: World) => {
        // Demo 7: ....
    };

    static demo8 = (world: World) => {
        // Demo 8: stress test
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
        const startAnchor = new Body(new BoxShape(stepWidth * 2, stepWidth * 0.5), startX - stepWidth / 2, startY, 0.0);
        startAnchor.setTexture('rockBridgeAnchor');
        world.addBody(startAnchor);

        // First connection uses the start anchor
        let lastStep = startAnchor;

        // Create steps
        for (let i = 1; i <= numSteps; i++) {
            const x = startX + i * spacing;

            // Optional sag: small vertical sinusoidal displacement
            const y = startY + Math.sin((i / numSteps) * Math.PI) * 10;

            const step = new Body(new CircleShape(stepWidth * 0.5), x, y, 3);
            step.setTexture('woodBridgeStep');
            world.addBody(step);

            // Joint anchor at left edge of this step
            const anchor = step.position.subNew(new Vec2(stepWidth / 2, 0));
            const joint = new JointConstraint(lastStep, step, anchor, softness, bias);
            world.addJoint(joint);

            lastStep = step;
        }

        // End anchor (static)
        const endAnchor = new Body(
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
                const box = new Body(
                    new BoxShape(boxSizeLarge, boxSizeLarge),
                    Graphics.width() / 2 - (numBoxLargeHorizontal * boxSizeLarge) / 2 + boxSizeLarge / 2 + i * boxSizeLarge,
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
                const box = new Body(
                    new BoxShape(boxSizeSmall, boxSizeSmall),
                    Graphics.width() / 2 - (numBoxSmallHorizontal * boxSizeSmall) / 2 + boxSizeSmall / 2 + i * boxSizeSmall,
                    -2000 + j * boxSizeSmall,
                    1,
                );
                world.addBody(box);
            }
        }
    };

    static demo9 = (world: World) => {
        // Demo 9: a complex scene

        const floor = this.generateFloor(world);
        this.generateFences(world);

        // Add bird
        const bird = new Body(new CircleShape(45), 100, Graphics.height() / 2.0 + 220, 3.0);
        bird.setTexture('birdRed');
        world.addBody(bird);

        // Add a stack of boxes
        for (let i = 1; i <= 4; i++) {
            const mass = 10.0 / i;
            const box = new Body(new BoxShape(50, 50), 600, floor.position.y - 100 - i * 55, mass);
            box.setTexture('woodBox');
            box.friction = 0.9;
            box.restitution = 0.1;
            world.addBody(box);
        }

        // Add structure with blocks
        const plank1 = new Body(new BoxShape(50, 150), Graphics.width() / 2.0 + 20, floor.position.y - 100 - 100, 5.0);
        const plank2 = new Body(new BoxShape(50, 150), Graphics.width() / 2.0 + 180, floor.position.y - 100 - 100, 5.0);
        const plank3 = new Body(new BoxShape(250, 25), Graphics.width() / 2.0 + 100, floor.position.y - 100 - 200, 2.0);
        plank1.setTexture('woodPlankSolid');
        plank2.setTexture('woodPlankSolid');
        plank3.setTexture('woodPlankCracked');
        world.addBody(plank1);
        world.addBody(plank2);
        world.addBody(plank3);

        // Add a triangle polygon
        const triangleVertices = [new Vec2(30, 30), new Vec2(-30, 30), new Vec2(0, -30)];
        const triangle = new Body(new PolygonShape(triangleVertices), plank3.position.x, plank3.position.y - 50, 0.5);
        triangle.setTexture('woodTriangle');
        world.addBody(triangle);

        // Add a pyramid of boxes
        const numRows = 5;
        for (let col = 0; col < numRows; col++) {
            for (let row = 0; row < col; row++) {
                const x = plank3.position.x + 200 + col * 50 - row * 25;
                const y = floor.position.y - 100 - 50 - row * 52;
                const mass = 5 / (row + 1);
                const box = new Body(new BoxShape(50, 50), x, y, mass);
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
        const startStep = new Body(new BoxShape(80, 20), 200, 200, 0.0);
        startStep.setTexture('rockBridgeAnchor');
        world.addBody(startStep);

        // The first connection should be from the anchor, not the floor
        let last = startStep;

        for (let i = 1; i <= numSteps; i++) {
            const x = startStep.position.x + 30 + i * spacing;
            const y = startStep.position.y + 20;
            const mass = 3;

            const step = new Body(new CircleShape(15), x, y, mass);
            step.setTexture('woodBridgeStep');
            world.addBody(step);

            // Connect previous link to this link
            const joint = new JointConstraint(last, step, step.position);
            world.addJoint(joint);

            last = step;
        }

        // Final anchor
        const endStep = new Body(new BoxShape(80, 20), last.position.x + 60, last.position.y - 20, 0.0);
        endStep.setTexture('rockBridgeAnchor');
        world.addBody(endStep);

        const lastJoint = new JointConstraint(last, endStep, endStep.position.addNew(new Vec2(15, 0)));
        world.addJoint(lastJoint);

        // Add pigs
        const pig1 = new Body(new CircleShape(30), plank1.position.x + 80, floor.position.y - 100 - 50, 3.0);
        const pig2 = new Body(new CircleShape(30), plank2.position.x + 400, floor.position.y - 100 - 50, 3.0);
        const pig3 = new Body(new CircleShape(30), plank2.position.x + 460, floor.position.y - 100 - 50, 3.0);
        const pig4 = new Body(new CircleShape(30), 220, 130, 1.0);
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
