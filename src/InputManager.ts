export default class InputManager {
    static keyboardInputBuffer: KeyboardEvent[];
    static mouseInputBuffer: MouseEvent[];
    static mousePosition: { x: number; y: number };

    static initialize = () => {
        this.keyboardInputBuffer = [];
        this.mouseInputBuffer = [];
        this.mousePosition = { x: 0, y: 0 };

        window.addEventListener('keydown', this.handleKeyboardEvent);
        window.addEventListener('keyup', this.handleKeyboardEvent);
        window.addEventListener('mousemove', this.handleMouseMove);
        window.addEventListener('mousedown', this.handleMouseClick);
        window.addEventListener('mouseup', this.handleMouseClick);
    };

    static handleKeyboardEvent = (event: KeyboardEvent) => {
        this.keyboardInputBuffer.push(event);
    };

    static handleMouseMove = (event: MouseEvent) => {
        this.mousePosition = { x: event.clientX, y: event.clientY };
    };

    static handleMouseClick = (event: MouseEvent) => {
        this.mouseInputBuffer.push(event);
    };
}
