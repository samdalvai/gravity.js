type TextureMap = Record<string, ImageBitmap>;

export default class AssetStore {
    private static textures: TextureMap = {};

    /**
     * Loads a PNG texture from the assets folder and stores it by name.
     * @param name The key to reference the texture
     * @param src Path to the PNG file (relative to your project)
     */
    static async loadTexture(name: string, src: string): Promise<void> {
        const img = new Image();
        img.src = src;
        await img.decode();

        console.log(`Loaded texture: ${src}`);

        const bitmap = await createImageBitmap(img);
        this.textures[name] = bitmap;
    }

    /** Retrieves a texture by name */
    static getTexture(name: string): ImageBitmap {
        const texture = this.textures[name];

        if (!texture) {
            throw new Error(`Texture "${name}" not found!`);
        }

        return texture;
    }

    /** Preload multiple textures at once */
    static async loadTextures(textureList: Record<string, string>): Promise<void> {
        const promises = Object.entries(textureList).map(([name, path]) => this.loadTexture(name, path));
        await Promise.all(promises);
    }
}
