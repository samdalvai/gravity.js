import { RigidBody } from '../src/core/RigidBody';
import { BoxShape } from '../src/shapes/BoxShape';
import * as Utils from '../src/utils/Utils';

describe('Performance', () => {
    test('', () => {
        console.time('time');
        const a = new RigidBody(new BoxShape(20, 20), 100, 100, 5);
        const b = new RigidBody(new BoxShape(20, 20), 100, 100, 5);

        for (let i = 0; i < 100000000; i++) {
            Utils.pairKey(a, b);
        }

        console.timeEnd('time');
    });
});
