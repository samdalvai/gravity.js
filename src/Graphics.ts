import Vec2 from './math/Vec2';
import RigidBody from './physics/RigidBody';
import { CapsuleShape, CircleShape, PolygonShape, ShapeType } from './physics/Shape';

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

    static drawHalfCircle = (
        x: number,
        y: number,
        radius: number,
        bodyAngle: number,
        half: 'top' | 'bottom',
        color: string,
    ): void => {
        let localStart: number;
        let localEnd: number;

        if (half === 'bottom') {
            localStart = 0;
            localEnd = Math.PI;
        } else {
            localStart = Math.PI;
            localEnd = Math.PI * 2;
        }

        const startAngle = bodyAngle + localStart;
        const endAngle = bodyAngle + localEnd;

        // Draw rotated arc
        this.ctx.beginPath();
        this.ctx.arc(x, y, radius, startAngle, endAngle);
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

    static drawFillPolygon = (x: number, y: number, vertices: Vec2[], color: string): void => {
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

        // Draw the 1px center point
        this.ctx.fillStyle = '#000000'; // black like 0xFF000000 in SDL
        this.ctx.beginPath();
        this.ctx.arc(x, y, 1, 0, Math.PI * 2);
        this.ctx.fill();
    };

    static drawCapsule = (
        x: number,
        y: number,
        halfHeight: number,
        radius: number,
        angle: number,
        color: string,
    ): void => {
        this.ctx.strokeStyle = color;

        const topCenterX = x + Math.cos(angle) * halfHeight;
        const topCenterY = y + Math.sin(angle) * halfHeight;

        this.ctx.beginPath();
        this.ctx.arc(topCenterX, topCenterY, radius, 0, Math.PI * 2);
        this.ctx.strokeStyle = color;
        this.ctx.stroke();

        const bottomCenterX = x - Math.cos(angle) * halfHeight;
        const bottomCenterY = y - Math.sin(angle) * halfHeight;

        this.ctx.beginPath();
        this.ctx.arc(bottomCenterX, bottomCenterY, radius, 0, Math.PI * 2);
        this.ctx.strokeStyle = color;
        this.ctx.stroke();

        const rectUpLeftX = topCenterX - radius;
        const rectUpLeftY = topCenterY - radius;
        const rectUpRightX = topCenterX + radius;
        const rectUpRightY = topCenterY + radius;

        const rectDownLeftX = bottomCenterX - radius;
        const rectDownLeftY = bottomCenterY - radius;
        const rectDownRightX = bottomCenterX + radius;
        const rectDownRightY = bottomCenterY + radius;

        this.ctx.beginPath();
        this.ctx.moveTo(rectUpLeftX, rectUpLeftY);
        this.ctx.lineTo(rectUpRightX, rectUpRightY);
        this.ctx.lineTo(rectDownRightX, rectDownRightY);
        this.ctx.lineTo(rectDownLeftX, rectDownLeftY);
        this.ctx.closePath();
        this.ctx.stroke();
    };

    static drawTexture = (
        x: number,
        y: number,
        width: number,
        height: number,
        rotation: number, // in radians
        texture: CanvasImageSource,
    ): void => {
        this.ctx.save();
        this.ctx.translate(x, y);
        this.ctx.rotate(rotation);

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
        color: string = 'white',
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
        switch (body.shape.getType()) {
            case ShapeType.CIRCLE:
                {
                    const circleShape = body.shape as CircleShape;

                    if (!debug && body.texture) {
                        Graphics.drawTexture(
                            body.position.x,
                            body.position.y,
                            circleShape.radius * 2,
                            circleShape.radius * 2,
                            body.rotation,
                            body.texture,
                        );
                    } else if (debug) {
                        Graphics.drawCircle(
                            body.position.x,
                            body.position.y,
                            circleShape.radius,
                            body.rotation,
                            'white',
                        );
                    }
                }
                break;
            case ShapeType.POLYGON:
                {
                    const polygonShape = body.shape as PolygonShape;
                    if (!debug && body.texture) {
                        Graphics.drawTexture(
                            body.position.x,
                            body.position.y,
                            polygonShape.width,
                            polygonShape.height,
                            body.rotation,
                            body.texture,
                        );
                    } else if (debug) {
                        Graphics.drawPolygon(body.position.x, body.position.y, polygonShape.worldVertices, 'white');
                    }
                }
                break;
            case ShapeType.CAPSULE:
                {
                    const capsuleShape = body.shape as CapsuleShape;
                    if (!debug && body.texture) {
                        const polygonShape = body.shape as PolygonShape;
                        // TODO: draw texture without stretching it
                        Graphics.drawTexture(
                            body.position.x,
                            body.position.y,
                            polygonShape.width,
                            polygonShape.height + capsuleShape.radius * 2,
                            body.rotation,
                            body.texture,
                        );
                    } else if (debug) {
                        const vertices = capsuleShape.worldVertices;
                        Graphics.drawLine(
                            vertices[vertices.length - 1].x,
                            vertices[vertices.length - 1].y,
                            vertices[0].x,
                            vertices[0].y,
                            'white',
                        );
                        Graphics.drawLine(vertices[1].x, vertices[1].y, vertices[2].x, vertices[2].y, 'white');
                        const offsetUp = new Vec2(0, capsuleShape.halfHeight).rotate(body.rotation);
                        const offsetDown = new Vec2(0, -capsuleShape.halfHeight).rotate(body.rotation);

                        const topCirclePosition = body.position.addNew(offsetUp);
                        const bottomCirclePosition = body.position.addNew(offsetDown);

                        Graphics.drawHalfCircle(
                            topCirclePosition.x,
                            topCirclePosition.y,
                            capsuleShape.radius,
                            body.rotation,
                            'bottom',
                            'white',
                        );

                        Graphics.drawHalfCircle(
                            bottomCirclePosition.x,
                            bottomCirclePosition.y,
                            capsuleShape.radius,
                            body.rotation,
                            'top',
                            'white',
                        );

                        Graphics.drawFillCircle(body.position.x, body.position.y, 5, 'blue');
                    }
                }
                break;
        }
    };
}
