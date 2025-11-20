export default class Graphics {
    static windowWidth: number;
    static windowHeight: number;
    static canvas: HTMLCanvasElement;
    static ctx: CanvasRenderingContext2D;

    static width = (): number => {
        return this.windowWidth;
    };

    static height = (): number => {
        return this.windowHeight;
    };

    static openWindow = (): boolean => {
        const canvas = document.getElementById('gamePhysicsCanvas') as HTMLCanvasElement;
        const ctx = canvas.getContext('2d');

        if (!ctx) {
            console.error('Failed to get 2D context for the canvas.');
            return false;
        }

        canvas.width = window.innerWidth;
        canvas.height = window.innerHeight;

        this.canvas = canvas;
        this.ctx = ctx;
        this.windowWidth = window.innerWidth;
        this.windowHeight = window.innerHeight;

        return true;
    };

    static clearScreen = (): void => {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    };

    static drawLine = (x0: number, y0: number, x1: number, y1: number, color: string): void => {
        this.ctx.strokeStyle = color;
        this.ctx.beginPath();
        this.ctx.moveTo(x0, y0);
        this.ctx.lineTo(x1, y1);
        this.ctx.stroke();
    };

    static drawCircle = (x: number, y: number, radius: number, angle: number, color: string): void => {
        // Draw the circle
        this.ctx.beginPath();
        this.ctx.arc(x, y, radius, 0, Math.PI * 2);
        this.ctx.strokeStyle = color;
        this.ctx.stroke();

        // Draw the line from center to circle edge at given angle
        const endX = x + Math.cos(angle) * radius;
        const endY = y + Math.sin(angle) * radius;

        this.ctx.beginPath();
        this.ctx.moveTo(x, y);
        this.ctx.lineTo(endX, endY);
        this.ctx.strokeStyle = color;
        this.ctx.stroke();
    };

    static drawFillCircle = (x: number, y: number, radius: number, color: string): void => {
        this.ctx.beginPath();
        this.ctx.arc(x, y, radius, 0, Math.PI * 2);
        this.ctx.fillStyle = color;
        this.ctx.fill();
    };

    // TODO: implement these draw methods
    // static void DrawRect(int x, int y, int width, int height, Uint32 color);
    // static void DrawFillRect(int x, int y, int width, int height, Uint32 color);
    // static void DrawPolygon(int x, int y, const std::vector<Vec2>& vertices, Uint32 color);
    // static void DrawFillPolygon(int x, int y, const std::vector<Vec2>& vertices, Uint32 color);
    // static void DrawTexture(int x, int y, int width, int height, float rotation, SDL_Texture* texture);
}
