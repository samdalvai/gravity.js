import Vec2 from './Vec2';

export default class Particle {
    radius: number;

    position: Vec2;
    velocity: Vec2;
    acceleration: Vec2;

    sumForces: Vec2;

    mass: number;
    invMass: number;

    constructor(x: number, y: number, radius: number, mass: number) {
        this.radius = radius;
        this.position = new Vec2(x, y);

        this.velocity = new Vec2();
        this.acceleration = new Vec2();
        this.acceleration = new Vec2();
        this.sumForces = new Vec2();

        this.mass = mass;
        if (mass != 0.0) {
            this.invMass = 1.0 / mass;
        } else {
            this.invMass = 0.0;
        }
    }

    addForce = (force: Vec2): void => {
        this.sumForces.addAssign(force);
    };

    clearForces = (): void => {
        this.sumForces = new Vec2(0, 0);
    };

    integrate = (dt: number) => {
        this.acceleration.assign(this.sumForces.scaleNew(this.invMass));

        this.velocity.addAssign(this.acceleration.scaleNew(dt));

        this.position.addAssign(this.velocity.scaleNew(dt));

        this.clearForces();
    };
}
