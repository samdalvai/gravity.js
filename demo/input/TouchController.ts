import Graphics from '../graphics/Graphics';
import type { MobileTool } from '../ui/MobileTools';

/** Movement (in CSS pixels) after which a touch is no longer considered a tap */
const TAP_MOVE_TOLERANCE = 10;

export interface TouchActions {
    getTool(): MobileTool;
    /** Updates the shared pointer world position (and any grabbed body target) */
    setPointerScreenPosition(screenX: number, screenY: number): void;
    /** Grabs the dynamic body under the pointer, returns true if the pointer is over a body */
    tryGrabAtPointer(): boolean;
    releaseGrab(): void;
    setEmittingParticles(value: boolean): void;
    /** Runs a one-shot tool (spawn, explosion, ...) at the pointer position */
    applyToolAtPointer(tool: MobileTool): void;
}

interface TrackedPointer {
    x: number;
    y: number;
    startX: number;
    startY: number;
}

type GestureState = 'idle' | 'single' | 'multi';

/**
 * Translates raw pointer events into demo actions:
 * - one finger: uses the selected tool (tap to spawn, drag to grab / pan, hold to emit)
 * - two fingers: pinch to zoom and drag to pan the camera
 */
export default class TouchController {
    private readonly pointers = new Map<number, TrackedPointer>();
    private state: GestureState = 'idle';
    private primaryId: number | null = null;
    private moved = false;
    private grabbing = false;
    private emitting = false;
    private lastDistance = 0;
    private lastMidX = 0;
    private lastMidY = 0;

    constructor(private readonly actions: TouchActions) {}

    handle(event: PointerEvent): void {
        switch (event.type) {
            case 'pointerdown':
                this.onDown(event);
                break;
            case 'pointermove':
                this.onMove(event);
                break;
            case 'pointerup':
                this.onUp(event, false);
                break;
            case 'pointercancel':
                this.onUp(event, true);
                break;
        }
    }

    /** Stops any running gesture, e.g. when a demo is reloaded */
    reset(): void {
        this.pointers.clear();
        this.state = 'idle';
        this.primaryId = null;
        this.grabbing = false;
        this.emitting = false;
    }

    private onDown(event: PointerEvent): void {
        const x = event.clientX;
        const y = event.clientY;
        this.pointers.set(event.pointerId, { x, y, startX: x, startY: y });

        if (this.state === 'idle') {
            this.state = 'single';
            this.primaryId = event.pointerId;
            this.moved = false;
            this.actions.setPointerScreenPosition(x, y);

            const tool = this.actions.getTool();

            if (tool === 'grab') {
                this.grabbing = this.actions.tryGrabAtPointer();
            } else if (tool === 'particles') {
                this.emitting = true;
                this.actions.setEmittingParticles(true);
            }

            return;
        }

        if (this.pointers.size >= 2) {
            // A second finger turns the interaction into a camera gesture
            this.cancelPrimaryAction();
            this.state = 'multi';
            this.updateGestureReference();
        }
    }

    private onMove(event: PointerEvent): void {
        const pointer = this.pointers.get(event.pointerId);
        if (!pointer) {
            return;
        }

        const dx = event.clientX - pointer.x;
        const dy = event.clientY - pointer.y;
        pointer.x = event.clientX;
        pointer.y = event.clientY;

        if (this.state === 'single' && event.pointerId === this.primaryId) {
            const distanceFromStart = Math.hypot(pointer.x - pointer.startX, pointer.y - pointer.startY);
            if (distanceFromStart > TAP_MOVE_TOLERANCE) {
                this.moved = true;
            }

            if (this.actions.getTool() === 'pan') {
                Graphics.panByScreen(dx, dy);
            }

            this.actions.setPointerScreenPosition(pointer.x, pointer.y);
            return;
        }

        if (this.state === 'multi' && this.pointers.size >= 2) {
            const [a, b] = this.firstTwoPointers();
            const distance = Math.hypot(a.x - b.x, a.y - b.y);
            const midX = (a.x + b.x) / 2;
            const midY = (a.y + b.y) / 2;

            if (this.lastDistance > 0 && distance > 0) {
                Graphics.zoomAt(midX, midY, distance / this.lastDistance);
            }

            Graphics.panByScreen(midX - this.lastMidX, midY - this.lastMidY);

            this.lastDistance = distance;
            this.lastMidX = midX;
            this.lastMidY = midY;
        }
    }

    private onUp(event: PointerEvent, cancelled: boolean): void {
        if (!this.pointers.has(event.pointerId)) {
            return;
        }

        this.pointers.delete(event.pointerId);

        if (this.state === 'single' && event.pointerId === this.primaryId) {
            const wasTap = !cancelled && !this.moved && !this.grabbing && !this.emitting;
            this.cancelPrimaryAction();

            if (wasTap) {
                this.actions.setPointerScreenPosition(event.clientX, event.clientY);
                this.actions.applyToolAtPointer(this.actions.getTool());
            }
        }

        if (this.pointers.size === 0) {
            this.state = 'idle';
            this.primaryId = null;
        } else if (this.state === 'multi') {
            this.updateGestureReference();
        }
    }

    private cancelPrimaryAction(): void {
        if (this.grabbing) {
            this.actions.releaseGrab();
            this.grabbing = false;
        }

        if (this.emitting) {
            this.actions.setEmittingParticles(false);
            this.emitting = false;
        }

        this.primaryId = null;
    }

    private updateGestureReference(): void {
        if (this.pointers.size < 2) {
            this.lastDistance = 0;
            return;
        }

        const [a, b] = this.firstTwoPointers();
        this.lastDistance = Math.hypot(a.x - b.x, a.y - b.y);
        this.lastMidX = (a.x + b.x) / 2;
        this.lastMidY = (a.y + b.y) / 2;
    }

    private firstTwoPointers(): [TrackedPointer, TrackedPointer] {
        const iterator = this.pointers.values();
        return [iterator.next().value as TrackedPointer, iterator.next().value as TrackedPointer];
    }
}
