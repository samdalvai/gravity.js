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
            Graphics.height() - 100,
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
            Graphics.height() - 100,
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
            Graphics.height() - 100,
            0.0,
        );
        world.addBody(floor);

        // Add a bridge of connected steps and joints
        const numSteps = 10;
        const spacing = 33;

        // Start anchor (static)
        const startStep = new Body(new BoxShape(80, 20), Graphics.width() / 2 - 200, Graphics.height() / 2, 0.0);
        startStep.setTexture('rockBridgeAnchor');
        world.addBody(startStep);

        // The first connection should be from the anchor, not the floor
        let last = startStep;

        for (let i = 1; i <= numSteps; i++) {
            const x = startStep.position.x + 30 + i * spacing;
            const y = startStep.position.y + 20;
            const mass = i < numSteps ? 3 : 0;

            const step = new Body(new CircleShape(15), x, y, mass);
            step.setTexture('woodBridgeStep');
            world.addBody(step);

            // Connect previous link to this link
            const joint = new JointConstraint(last, step, step.position);
            world.addConstraint(joint);

            last = step;
        }

        // Final anchor
        const endStep = new Body(new BoxShape(80, 20), last.position.x + 60, last.position.y - 20, 0.0);
        endStep.setTexture('rockBridgeAnchor');
        world.addBody(endStep);

        const lastJoint = new JointConstraint(last, endStep, endStep.position);
        world.addConstraint(lastJoint);
    };

    static demo4 = (world: World) => {
        // Demo 4: A simple whip
        const floor = new Body(
            new BoxShape(Graphics.width(), 50),
            Graphics.width() / 2.0,
            Graphics.height() - 100,
            0.0,
        );
        world.addBody(floor);

        const whipAnchor = new Body(new BoxShape(10, 10), Graphics.width() / 2, 200, 0);
        whipAnchor.setTexture('rockBridgeAnchor');
        world.addBody(whipAnchor);

        let last = whipAnchor;

        for (let i = 0; i < 10; i++) {
            const x = whipAnchor.position.x;
            const y = i === 0 ? 240 : 240 + 60 * i;
            const whipElement = new Body(new BoxShape(10, 50), x, y, 10);
            whipElement.setTexture('crate');
            world.addBody(whipElement);

            const anchor = whipElement.position.subNew(new Vec2(0, 25));
            const j = new JointConstraint(last, whipElement, anchor);
            world.addConstraint(j);

            last = whipElement;
        }

        // // Connect last step to final anchor
        // const finalJoint = new JointConstraint(last, endStep, endStep.position);
        // world.addConstraint(finalJoint);
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
