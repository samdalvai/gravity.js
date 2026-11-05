import { RigidBody } from './RigidBody';

/**
 * Number of floats per body: vx, vy, w and one padding slot,
 * so that a body never straddles two 64 byte cache lines.
 */
export const SOLVER_BODY_STRIDE = 4;

/**
 * Packed copy of body velocities used while solving constraints.
 *
 * The velocity solver touches body velocities many times per step. Reading them from
 * `RigidBody` objects scattered in the heap (constraint -> body -> velocity Vec2) is dominated by
 * cache misses, so the bodies are staged into a contiguous typed array for the duration of the solve,
 * constraints address them through `RigidBody.solverOffset`, and results are written back at the end.
 */
export class SolverBodies {
    velocities = new Float64Array(SOLVER_BODY_STRIDE * 256);

    /** Shared by all instances, so a stamp left on a body by another world never matches */
    private static nextStamp = 1;

    private readonly staged: RigidBody[] = [];
    private stamp = 0;

    /** Starts a new staging pass, previously staged offsets become invalid */
    begin(): void {
        this.staged.length = 0;
        this.stamp = SolverBodies.nextStamp++;
    }

    /** Stages the body if it was not staged yet in this pass */
    add(body: RigidBody): void {
        if (body.solverStamp === this.stamp) return;

        const offset = this.staged.length * SOLVER_BODY_STRIDE;

        if (offset + SOLVER_BODY_STRIDE > this.velocities.length) {
            const grown = new Float64Array(this.velocities.length * 2);
            grown.set(this.velocities);
            this.velocities = grown;
        }

        const v = this.velocities;
        v[offset] = body.velocity.x;
        v[offset + 1] = body.velocity.y;
        v[offset + 2] = body.angularVelocity;

        body.solverStamp = this.stamp;
        body.solverOffset = offset;
        this.staged.push(body);
    }

    /** Copies the solved velocities back into the staged bodies */
    end(): void {
        const v = this.velocities;
        const staged = this.staged;

        for (let i = 0; i < staged.length; i++) {
            const body = staged[i];
            const offset = body.solverOffset;
            body.velocity.x = v[offset];
            body.velocity.y = v[offset + 1];
            body.angularVelocity = v[offset + 2];
        }

        staged.length = 0;
    }
}
