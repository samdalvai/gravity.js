import { Vec2 } from 'gravity.js';

export enum MouseButton {
    LEFT = 0,
    MIDDLE = 1,
    RIGHT = 2,
}

export default class InputManager {
    static keyboardInputBuffer: KeyboardEvent[];
    static mouseInputBuffer: MouseEvent[];
    static mouseMoveBuffer: MouseEvent[];
    static mouseWheelBuffer: MouseEvent[];
    static pointerBuffer: PointerEvent[];
    static mousePosition: Vec2;

    /**
     * When true (mobile view) the primary mouse button is handled by the touch
     * pipeline, so the mobile view can also be tried with a mouse on desktop.
     */
    static routeMouseToTouch = false;

    static lastWheelEventTime = 0;

    static initialize = () => {
        this.keyboardInputBuffer = [];
        this.mouseInputBuffer = [];
        this.mouseMoveBuffer = [];
        this.mouseWheelBuffer = [];
        this.pointerBuffer = [];
        this.mousePosition = new Vec2();

        window.addEventListener('keydown', this.handleKeyboardEvent);
        window.addEventListener('keyup', this.handleKeyboardEvent);

        window.addEventListener('mousemove', this.handleMouseMove);
        window.addEventListener('mousedown', this.handleMouseClick);
        window.addEventListener('mouseup', this.handleMouseClick);

        window.addEventListener('wheel', this.handleWheelEvent);

        const canvas = document.getElementById('gamePhysicsCanvas');
        if (canvas) {
            canvas.addEventListener('pointerdown', this.handlePointerEvent);
            canvas.addEventListener('pointermove', this.handlePointerEvent);
            canvas.addEventListener('pointerup', this.handlePointerEvent);
            canvas.addEventListener('pointercancel', this.handlePointerEvent);
        }

        window.addEventListener('contextmenu', e => {
            e.preventDefault();
        });
    };

    static isUiEventTarget = (target: EventTarget | null) => {
        return target instanceof Element && target.closest('#demo-toolbar, #demo-shortcuts-modal, .mobile-ui') !== null;
    };

    static handleKeyboardEvent = (event: KeyboardEvent) => {
        if (event.type === 'keydown' && this.isUiEventTarget(event.target)) {
            return;
        }

        this.keyboardInputBuffer.push(event);
    };

    static handleMouseMove = (event: MouseEvent) => {
        if (this.isUiEventTarget(event.target)) {
            return;
        }

        this.mouseMoveBuffer.push(event);
    };

    static handleMouseClick = (event: MouseEvent) => {
        if (event.type === 'mousedown' && this.isUiEventTarget(event.target)) {
            return;
        }

        if (this.routeMouseToTouch && event.button === MouseButton.LEFT) {
            return;
        }

        this.mouseInputBuffer.push(event);
    };

    static handleWheelEvent = (event: MouseEvent) => {
        if (this.isUiEventTarget(event.target)) {
            return;
        }

        // Wheel events for mousepads are triggered much faster compared to mouse wheels
        // whith this logic we prevent scrolling too fast on mouse pads
        if (performance.now() - this.lastWheelEventTime < 25) {
            return;
        }

        this.mouseWheelBuffer.push(event);
        this.lastWheelEventTime = performance.now();
    };

    /**
     * Touch and pen input is always handled by the touch pipeline. Mouse input is
     * handled there only for the primary button while the mobile view is active.
     */
    static handlePointerEvent = (event: PointerEvent) => {
        const isMouse = event.pointerType === 'mouse';

        if (isMouse && !this.routeMouseToTouch) {
            return;
        }

        if (isMouse && event.type === 'pointerdown' && event.button !== MouseButton.LEFT) {
            return;
        }

        if (isMouse && event.type === 'pointermove' && (event.buttons & 1) === 0) {
            return;
        }

        if (event.type === 'pointerdown') {
            // Avoid compatibility mouse events and keep receiving moves outside the canvas
            event.preventDefault();
            (event.target as Element | null)?.setPointerCapture?.(event.pointerId);
        }

        this.pointerBuffer.push(event);
    };
}
