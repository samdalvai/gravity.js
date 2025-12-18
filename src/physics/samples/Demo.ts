import Graphics from '../../Graphics';
import Utils from '../../math/Utils';
import Vec2 from '../../math/Vec2';
import World from '../World';
import Body from '../body/Body';
import { BoxShape } from '../body/Shape';

export default class Demo {
    static demoStrings = [
        'Demo 0: ....',
        'Demo 1: A single box',
        'Demo 2: A pyramid of boxes',
        'Demo 3: ....',
        'Demo 4: ....',
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
        const baseY = floor.position.y - floorHeight / 2 - boxSize / 2;

        for (let row = 0; row < rows; row++) {
            const boxesInRow = rows - row;
            const rowWidth = boxesInRow * boxSize;

            for (let col = 0; col < boxesInRow; col++) {
                const x = centerX - rowWidth / 2 + boxSize / 2 + col * boxSize;
                const y = baseY - row * boxSize;

                const box = new Body(new BoxShape(boxSize, boxSize), x, y, 10);
                box.setTexture('crate');
                world.addBody(box);
            }
        }
    };

    static demo3 = (world: World) => {
        // Demo 3: ....
    };

    static demo4 = (world: World) => {
        // Demo 4: ....
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
