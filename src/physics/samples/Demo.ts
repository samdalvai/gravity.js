import Graphics from '../../Graphics';
import Utils from '../../math/Utils';
import Vec2 from '../../math/Vec2';
import World from '../World';
import Body from '../body/Body';
import { BoxShape, CircleShape } from '../body/Shape';
import JointConstraint from '../constraint/JointConstraint';

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
        'Demo 8: ....',
        'Demo 9: ....',
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
        const leftFence = new Body(new BoxShape(50, Graphics.height() - 200), 0, Graphics.height() / 2.0 - 100, 0.0);
        const rightFence = new Body(
            new BoxShape(50, Graphics.height() - 200),
            Graphics.width(),
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
            world.addConstraint(joint);

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
        world.addConstraint(lastJoint);
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
            world.addConstraint(j);

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

        world.addConstraint(string);
        world.addConstraint(neck);
        world.addConstraint(leftShoulder);
        world.addConstraint(rightShoulder);
        world.addConstraint(leftHip);
        world.addConstraint(rightHip);
    };

    static demo6 = (world: World) => {
        // Demo 6: ....
    };

    static demo7 = (world: World) => {
        // Demo 7: ....
    };

    static demo8 = (world: World) => {
        // Demo 8: ....
    };

    static demo9 = (world: World) => {
        // Demo 9: ....
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
