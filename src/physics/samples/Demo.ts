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
        'Demo 5: ....',
        'Demo 6: ....',
        'Demo 7: ....',
        'Demo 8: ....',
        'Demo 9: ....',
    ];

    static demo0 = (world: World) => {
        // Demo 0: ....
    };

    static demo1 = (world: World) => {
        // Demo 1: Single box demo
        const floor = new Body(
            new BoxShape(Graphics.width(), 50),
            Graphics.width() / 2.0,
            Graphics.height() - 150,
            0.0,
        );
        world.addBody(floor);

        // const leftFence = new Body(new BoxShape(50, Graphics.height() - 75), 0, Graphics.height() / 2.0 - 35, 0.0);
        // const rightFence = new Body(
        //     new BoxShape(50, Graphics.height() - 75),
        //     Graphics.width(),
        //     Graphics.height() / 2.0 - 35,
        //     0.0,
        // );
        // world.addBody(leftFence);
        // world.addBody(rightFence);
        const box = new Body(new BoxShape(60, 60), Graphics.width() / 2.0, Graphics.height() - 300, 10);
        box.setTexture('crate');
        world.addBody(box);
    };

    static demo2 = (world: World) => {
        // Demo 2: Pyramid of boxes
        const floorHeight = 50;
        const floor = new Body(
            new BoxShape(Graphics.width(), floorHeight),
            Graphics.width() / 2.0,
            Graphics.height() - 150,
            0.0,
        );
        world.addBody(floor);

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
        const floor = new Body(
            new BoxShape(Graphics.width(), 50),
            Graphics.width() / 2.0,
            Graphics.height() - 150,
            0.0,
        );
        world.addBody(floor);

        // Suspension Bridge Creation
        const numSteps = 10;
        const stepWidth = 40;
        const stepHeight = 15;
        const spacing = stepWidth + 10; // distance between centers
        const startX = Graphics.width() / 2 - (numSteps * spacing) / 2;
        const startY = Graphics.height() / 2;

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
            const joint = new JointConstraint(lastStep, step, anchor, 0.02, 0.1); // softness, biasFactor
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
        const lastJoint = new JointConstraint(lastStep, endAnchor, finalAnchor, 0.02, 0.1);
        world.addConstraint(lastJoint);
    };

    static demo4 = (world: World) => {
        // Demo 4: A simple whip
        const floor = new Body(
            new BoxShape(Graphics.width(), 50),
            Graphics.width() / 2.0,
            Graphics.height() - 150,
            0.0,
        );
        world.addBody(floor);

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

        // Add a lateral wind force to the world
        // world.addForce(new Vec2(100, 0));
    };

    static demo5 = (world: World) => {
        // Demo 5: ....
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
