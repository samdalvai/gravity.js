import Vec2 from './physics/Vec2';

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

    static drawRect = (x: number, y: number, width: number, height: number, color: string): void => {
        const hw = width / 2;
        const hh = height / 2;

        this.ctx.strokeStyle = color;
        this.ctx.beginPath();
        this.ctx.moveTo(x - hw, y - hh);
        this.ctx.lineTo(x + hw, y - hh);
        this.ctx.lineTo(x + hw, y + hh);
        this.ctx.lineTo(x - hw, y + hh);
        this.ctx.closePath();
        this.ctx.stroke();
    };

    static drawFillRect = (x: number, y: number, width: number, height: number, color: string): void => {
        const hw = width / 2;
        const hh = height / 2;

        this.ctx.fillStyle = color;
        this.ctx.fillRect(x - hw, y - hh, width, height);
    };

    static drawPolygon = (x: number, y: number, vertices: Vec2[], color: string): void => {
        this.ctx.strokeStyle = color;
        this.ctx.beginPath();

        if (vertices.length > 0) {
            this.ctx.moveTo(vertices[0].x, vertices[0].y);
            for (let i = 1; i < vertices.length; i++) {
                this.ctx.lineTo(vertices[i].x, vertices[i].y);
            }
            this.ctx.closePath();
        }

        this.ctx.stroke();

        // draw the 1px center point like filledCircleColor(..., radius=1)
        this.ctx.fillStyle = color;
        this.ctx.beginPath();
        this.ctx.arc(x, y, 1, 0, Math.PI * 2);
        this.ctx.fill();
    };

    // TODO: implement these draw methods
    // static void DrawFillPolygon(int x, int y, const std::vector<Vec2>& vertices, Uint32 color);
    // static void DrawTexture(int x, int y, int width, int height, float rotation, SDL_Texture* texture);
}
