import { RigidBody } from '../../src';
import AssetStore, { TEXTURES } from '../graphics/AssetStore';

export interface BodyRenderStyle {
    fillColor?: string;
    strokeColor?: string;
    texture?: ImageBitmap | null;
}

export default class BodyRenderRegistry {
    private readonly styles = new Map<number, BodyRenderStyle>();

    clear(): void {
        this.styles.clear();
    }

    delete(body: RigidBody): void {
        this.styles.delete(body.id);
    }

    getStyle(body: RigidBody): BodyRenderStyle | undefined {
        return this.styles.get(body.id);
    }

    setFillColor(body: RigidBody, fillColor: string): void {
        const style = this.ensureStyle(body);
        style.fillColor = fillColor;
    }

    setTexture(body: RigidBody, texture: keyof typeof TEXTURES): void {
        const style = this.ensureStyle(body);
        style.texture = AssetStore.getTexture(texture);
    }

    private ensureStyle(body: RigidBody): BodyRenderStyle {
        let style = this.styles.get(body.id);

        if (!style) {
            style = {};
            this.styles.set(body.id, style);
        }

        return style;
    }
}
