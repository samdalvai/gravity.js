import { Entity } from './entity';
import { Matrix3 } from './math/matrix3';

export class Camera extends Entity {
    constructor() {
        super();
    }

    get transform(): Matrix3 {
        return super.localToGlobal;
    }

    get cameraTransform(): Matrix3 {
        return super.globalToLocal;
    }

    reset(): void {
        this.resetTransform();
    }
}
