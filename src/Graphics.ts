import Vec2 from './math/Vec2';
import RigidBody from './physics/RigidBody';
import { BoxShape, CapsuleShape, CircleShape, PolygonShape, ShapeType } from './physics/Shape';

export default class Graphics {
    static windowWidth: number;
    static windowHeight: number;
    static canvas: HTMLCanvasElement;
    static ctx: CanvasRenderingContext2D;

    static zoom = 1;
    static pan = new Vec2(0, 0);

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

    static increaseZoom = (): void => {
        this.zoom += 0.05;
    };

    static decreaseZoom = (): void => {
        this.zoom -= 0.05;
    };

    static clearScreen = (): void => {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    };

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

    static drawLine = (x0: number, y0: number, x1: number, y1: number, color = 'white'): void => {
        this.ctx.strokeStyle = color;
        this.ctx.beginPath();
        this.ctx.moveTo(x0, y0);
        this.ctx.lineTo(x1, y1);
        this.ctx.stroke();
    };

    static drawCircle = (radius: number, color = 'white'): void => {
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
    };

    static drawHalfCircle = (x: number, y: number, radius: number, half: 'top' | 'bottom', color = 'white'): void => {
        let localStart: number;
        let localEnd: number;

        if (half === 'bottom') {
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
    };

    static drawFillCircle = (x: number, y: number, radius: number, color = 'white'): void => {
        this.ctx.beginPath();
        this.ctx.arc(x, y, radius, 0, Math.PI * 2);
        this.ctx.fillStyle = color;
        this.ctx.fill();
    };

    static drawPolygon = (vertices: Vec2[], color = 'white'): void => {
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
    };

    static drawFillPolygon = (x: number, y: number, vertices: Vec2[], color = 'white'): void => {
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
    };

    static drawRect = (x: number, y: number, width: number, height: number, color = 'white'): void => {
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
    };

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

    static drawCapsule = (capsuleShape: CapsuleShape, color = 'white'): void => {
        const vertices = capsuleShape.localVertices;
        Graphics.drawLine(
            vertices[vertices.length - 1].x,
            vertices[vertices.length - 1].y,
            vertices[0].x,
            vertices[0].y,
            color,
        );
        Graphics.drawLine(vertices[1].x, vertices[1].y, vertices[2].x, vertices[2].y, color);

        const topCirclePosition = new Vec2(0, capsuleShape.halfHeight);
        const bottomCirclePosition = new Vec2(0, -capsuleShape.halfHeight);

        Graphics.drawHalfCircle(topCirclePosition.x, topCirclePosition.y, capsuleShape.radius, 'bottom', color);
        Graphics.drawHalfCircle(bottomCirclePosition.x, bottomCirclePosition.y, capsuleShape.radius, 'top', color);

        this.ctx.fillStyle = color;
        this.ctx.beginPath();
        this.ctx.arc(0, 0, 1, 0, Math.PI * 2);
        this.ctx.fill();
    };

    static drawTexture = (width: number, height: number, texture: CanvasImageSource): void => {
        this.ctx.save();

        // This is needed because we flip the canvas with beginWorld()
        this.ctx.scale(1, -1);
        this.ctx.drawImage(texture, -width / 2, -height / 2, width, height);
        this.ctx.restore();
    };

    static drawText = (
        text: string,
        x: number,
        y: number,
        fontSize: number = 20,
        fontFamily: string = 'Arial',
        color = 'white',
        align: CanvasTextAlign = 'left',
        baseline: CanvasTextBaseline = 'middle',
    ): void => {
        this.ctx.save();
        this.ctx.fillStyle = color;
        this.ctx.font = `${fontSize}px ${fontFamily}`;
        this.ctx.textAlign = align;
        this.ctx.textBaseline = baseline;
        this.ctx.fillText(text, x, y);
        this.ctx.restore();
    };

    static drawBody = (body: RigidBody, debug = false): void => {
        const x = body.position.x;
        const y = body.position.y;
        const rotation = body.rotation;

        this.ctx.save();
        this.ctx.translate(x, y);
        this.ctx.rotate(rotation);

        // TODO: Draw filled shape if texture is not available

        switch (body.shape.getType()) {
            case ShapeType.CIRCLE:
                {
                    const circleShape = body.shape as CircleShape;
                    if (debug) {
                        this.drawCircle(circleShape.radius, 'white');
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
                        this.drawPolygon(polygonShape.localVertices, 'white');
                    } else if (body.texture) {
                        this.drawTexture(polygonShape.width, polygonShape.height, body.texture);
                    } else {
                        this.drawFillPolygon(0, 0, polygonShape.localVertices, 'gray');
                    }
                }
                break;
            case ShapeType.BOX:
                {
                    const boxShape = body.shape as BoxShape;

                    if (debug) {
                        this.drawBox(boxShape.width, boxShape.height, 'white');
                    } else if (body.texture) {
                        this.drawTexture(boxShape.width, boxShape.height, body.texture);
                    } else {
                        this.drawFillBox(boxShape.width, boxShape.height, 'gray');
                    }
                }
                break;
            case ShapeType.CAPSULE:
                {
                    const capsuleShape = body.shape as CapsuleShape;

                    if (debug) {
                        this.drawCapsule(capsuleShape, 'white');
                    } else if (body.texture) {
                        const polygonShape = body.shape as PolygonShape;
                        // TODO: draw texture without stretching it
                        this.drawTexture(
                            polygonShape.width,
                            polygonShape.height + capsuleShape.radius * 2,
                            body.texture,
                        );
                    } else {
                        // Method missing
                    }
                }
                break;
        }

        this.ctx.restore();
    };
}
