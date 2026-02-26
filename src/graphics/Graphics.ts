import RigidBody from '../core/RigidBody';
import Vec2 from '../math/Vec2';
import { BoxShape } from '../shapes/BoxShape';
import { CapsuleShape } from '../shapes/CapsuleShape';
import { CircleShape } from '../shapes/CircleShape';
import { PolygonShape } from '../shapes/PolygonShape';
import { ShapeType } from '../shapes/Shape';

export default class Graphics {
    static windowWidth: number;
    static windowHeight: number;
    static canvas: HTMLCanvasElement;
    static ctx: CanvasRenderingContext2D;

    static zoom = 1;
    static pan = new Vec2(0, 0);

    static width(): number {
        return this.windowWidth;
    }

    static height(): number {
        return this.windowHeight;
    }

    static openWindow(): boolean {
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
    }

    static increaseZoom(): void {
        this.zoom += 0.05;
    }

    static decreaseZoom(): void {
        this.zoom -= 0.05;

        if (this.zoom < 0.05) {
            this.zoom = 0.05;
        }
    }

    static resetView(): void {
        this.zoom = 1;
        this.pan.x = 0;
        this.pan.y = 0;
    }

    static clearScreen(): void {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    }

    /**
     * Start world coordinates to screen conversion.
     *
     * This is used because box 2d uses a standard coordinate system for objects
     * positions and dimensions
     */
    static beginWorld(): void {
        const ctx = this.ctx;

        ctx.save();

        // Move origin to screen center
        ctx.translate(this.windowWidth / 2, this.windowHeight / 2);

        // Flip Y axis (world Y up, canvas Y down)
        ctx.scale(this.zoom, -this.zoom);

        // Apply camera pan
        ctx.translate(-this.pan.x, -this.pan.y);

        ctx.lineWidth = 1 / this.zoom;
    }

    /** Restore coordinates to screen conversion */
    static endWorld(): void {
        this.ctx.restore();
    }

    static drawLine(x0: number, y0: number, x1: number, y1: number, color = 'white'): void {
        this.ctx.strokeStyle = color;
        this.ctx.beginPath();
        this.ctx.moveTo(x0, y0);
        this.ctx.lineTo(x1, y1);
        this.ctx.stroke();
    }

    static drawCircle(radius: number, color = 'white'): void {
        // Draw the circle
        this.ctx.beginPath();
        this.ctx.arc(0, 0, radius, 0, Math.PI * 2);
        this.ctx.strokeStyle = color;
        this.ctx.stroke();

        // Draw the line from center to circle edge at given angle
        const endX = Math.cos(0) * radius;
        const endY = Math.sin(0) * radius;

        this.ctx.beginPath();
        this.ctx.moveTo(0, 0);
        this.ctx.lineTo(endX, endY);
        this.ctx.strokeStyle = color;
        this.ctx.stroke();
    }

    static drawHalfCircle(x: number, y: number, radius: number, half: 'top' | 'bottom', color = 'white'): void {
        let localStart: number;
        let localEnd: number;

        if (half === 'top') {
            localStart = 0;
            localEnd = Math.PI;
        } else {
            localStart = Math.PI;
            localEnd = Math.PI * 2;
        }

        // Draw rotated arc
        this.ctx.beginPath();
        this.ctx.arc(x, y, radius, localStart, localEnd);
        this.ctx.strokeStyle = color;
        this.ctx.stroke();
    }

    static drawFillCircle(x: number, y: number, radius: number, color = 'white'): void {
        this.ctx.beginPath();
        this.ctx.arc(x, y, radius, 0, Math.PI * 2);
        this.ctx.fillStyle = color;
        this.ctx.fill();
    }

    static drawPolygon(vertices: Vec2[], color = 'white'): void {
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
        this.ctx.arc(0, 0, 1, 0, Math.PI * 2);
        this.ctx.fill();
    }

    static drawFillPolygon(x: number, y: number, vertices: Vec2[], color = 'white'): void {
        if (vertices.length === 0) return;

        // Fill polygon
        this.ctx.fillStyle = color;
        this.ctx.beginPath();
        this.ctx.moveTo(vertices[0].x, vertices[0].y);
        for (let i = 1; i < vertices.length; i++) {
            this.ctx.lineTo(vertices[i].x, vertices[i].y);
        }
        this.ctx.closePath();
        this.ctx.fill();
    }

    static drawRect(x: number, y: number, width: number, height: number, color = 'white'): void {
        const halfWidth = width / 2;
        const halfHeight = height / 2;

        this.ctx.strokeStyle = color;
        this.ctx.beginPath();
        this.ctx.moveTo(x - halfWidth, y - halfHeight);
        this.ctx.lineTo(x + halfWidth, y - halfHeight);
        this.ctx.lineTo(x + halfWidth, y + halfHeight);
        this.ctx.lineTo(x - halfWidth, y + halfHeight);
        this.ctx.closePath();
        this.ctx.stroke();
    }

    static drawBox(width: number, height: number, color = 'white') {
        const halfWidth = width / 2;
        const halfHeight = height / 2;

        this.ctx.beginPath();
        this.ctx.rect(-halfWidth, -halfHeight, halfWidth * 2, halfHeight * 2);
        this.ctx.strokeStyle = color;
        this.ctx.stroke();

        // draw the 1px center point like filledCircleColor(..., radius=1)
        this.ctx.fillStyle = color;
        this.ctx.beginPath();
        this.ctx.arc(0, 0, 1, 0, Math.PI * 2);
        this.ctx.fill();
    }

    static drawFillBox(width: number, height: number, color = 'white') {
        const halfWidth = width / 2;
        const halfHeight = height / 2;

        this.ctx.fillStyle = color;
        this.ctx.fillRect(-halfWidth, -halfHeight, halfWidth * 2, halfHeight * 2);
    }

    static drawCapsule(capsuleShape: CapsuleShape, color = 'white'): void {
        const r = capsuleShape.radius;
        const hh = capsuleShape.halfHeight;

        this.ctx.beginPath();

        this.ctx.moveTo(-r, hh);
        this.ctx.arc(0, -hh, r, Math.PI, 0);
        this.ctx.lineTo(r, -hh);
        this.ctx.arc(0, hh, r, 0, Math.PI);
        this.ctx.lineTo(-r, hh);

        this.ctx.closePath();

        this.ctx.strokeStyle = color;
        this.ctx.stroke();

        // Draw body position
        this.ctx.fillStyle = color;
        this.ctx.beginPath();
        this.ctx.arc(0, 0, 1, 0, Math.PI * 2);
        this.ctx.fill();
    }

    static drawFillCapsule(capsuleShape: CapsuleShape, color = 'white'): void {
        const r = capsuleShape.radius;
        const hh = capsuleShape.halfHeight;

        this.ctx.beginPath();

        this.ctx.moveTo(-r, hh);
        this.ctx.arc(0, -hh, r, Math.PI, 0);
        this.ctx.lineTo(r, -hh);
        this.ctx.arc(0, hh, r, 0, Math.PI);
        this.ctx.lineTo(-r, hh);

        this.ctx.closePath();

        this.ctx.fillStyle = color;
        this.ctx.fill();
    }

    static drawTexture(width: number, height: number, texture: CanvasImageSource, offsetX = 0, offsetY = 0): void {
        this.ctx.save();

        this.ctx.translate(offsetX, offsetY);

        // This is needed because we flip the canvas with beginWorld()
        this.ctx.scale(1, -1);
        this.ctx.drawImage(texture, -width / 2, -height / 2, width, height);
        this.ctx.restore();
    }

    static drawText(
        text: string,
        x: number,
        y: number,
        fontSize: number = 20,
        fontFamily: string = 'Arial',
        color = 'white',
        align: CanvasTextAlign = 'left',
        baseline: CanvasTextBaseline = 'middle',
    ): void {
        this.ctx.save();
        this.ctx.fillStyle = color;
        this.ctx.font = `${fontSize}px ${fontFamily}`;
        this.ctx.textAlign = align;
        this.ctx.textBaseline = baseline;
        this.ctx.fillText(text, x, y);
        this.ctx.restore();
    }

    static drawBody(body: RigidBody, debug = false): void {
        const x = body.position.x;
        const y = body.position.y;
        const rotation = body.rotation;

        this.ctx.save();
        this.ctx.translate(x, y);
        this.ctx.rotate(rotation);

        // TODO: Draw filled shape if texture is not available
        const color = 'white';

        switch (body.shape.getType()) {
            case ShapeType.CIRCLE:
                {
                    const circleShape = body.shape as CircleShape;
                    if (debug) {
                        this.drawCircle(circleShape.radius, color);
                    } else if (body.texture) {
                        this.drawTexture(circleShape.radius * 2, circleShape.radius * 2, body.texture);
                    } else {
                        this.drawFillCircle(0, 0, circleShape.radius, 'gray');
                    }
                }
                break;
            case ShapeType.POLYGON:
                {
                    const polygonShape = body.shape as PolygonShape;

                    if (debug) {
                        this.drawPolygon(polygonShape.localVertices, color);
                    } else if (body.texture) {
                        this.drawTexture(polygonShape.width, polygonShape.height, body.texture);
                    } else {
                        this.drawFillPolygon(0, 0, polygonShape.localVertices, body.shapeFillColor);
                    }
                }
                break;
            case ShapeType.BOX:
                {
                    const boxShape = body.shape as BoxShape;

                    if (debug) {
                        this.drawBox(boxShape.width, boxShape.height, color);
                    } else if (body.texture) {
                        this.drawTexture(boxShape.width, boxShape.height, body.texture);
                    } else {
                        this.drawFillBox(boxShape.width, boxShape.height, body.shapeFillColor);
                    }
                }
                break;
            case ShapeType.CAPSULE:
                {
                    const capsuleShape = body.shape as CapsuleShape;

                    if (debug) {
                        this.drawCapsule(capsuleShape, color);
                    } else if (body.texture) {
                        const polygonShape = body.shape as PolygonShape;
                        // TODO: draw texture without stretching it
                        this.drawTexture(
                            polygonShape.width,
                            polygonShape.height + capsuleShape.radius * 2,
                            body.texture,
                        );
                    } else {
                        this.drawFillCapsule(capsuleShape, body.shapeFillColor);
                    }
                }
                break;
        }

        this.ctx.restore();
    }
}
