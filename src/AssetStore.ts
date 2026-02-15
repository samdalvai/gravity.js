export const TEXTURES = {
    // Base sprites
    basketball: 'assets/basketball.png',
    bowlingball: 'assets/bowlingball.png',
    crate: 'assets/crate.png',
    metal: 'assets/metal.png',
    transparent: 'assets/transparent.png',

    // Skeleton ragdoll
    bob: 'assets/ragdoll/bob.png',
    head: 'assets/ragdoll/head.png',
    leftArm: 'assets/ragdoll/leftArm.png',
    leftLeg: 'assets/ragdoll/leftLeg.png',
    rightArm: 'assets/ragdoll/rightArm.png',
    rightLeg: 'assets/ragdoll/rightLeg.png',
    torso: 'assets/ragdoll/torso.png',

    // Angry birds sprites
    background: 'assets/angrybirds/background.png',
    birdRed: 'assets/angrybirds/bird-red.png',
    pig1: 'assets/angrybirds/pig-1.png',
    pig2: 'assets/angrybirds/pig-2.png',
    rockBox: 'assets/angrybirds/rock-box.png',
    rockBridgeAnchor: 'assets/angrybirds/rock-bridge-anchor.png',
    rockRound: 'assets/angrybirds/rock-round.png',
    woodBox: 'assets/angrybirds/wood-box.png',
    woodBridgeStep: 'assets/angrybirds/wood-bridge-step.png',
    woodPlankCracked: 'assets/angrybirds/wood-plank-cracked.png',
    woodPlankSolid: 'assets/angrybirds/wood-plank-solid.png',
    woodTriangle: 'assets/angrybirds/wood-triangle.png',
};

type TextureMap = Record<string, ImageBitmap>;

export default class AssetStore {
    private static textures: TextureMap = {};

    /**
     * Loads a PNG texture from the assets folder and stores it by name.
     * @param name The key to reference the texture
     * @param src Path to the PNG file (relative to your project)
     */
    static async loadTexture(name: keyof typeof TEXTURES, src: string): Promise<void> {
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
    static async loadTextures(): Promise<void> {
        const promises = Object.entries(TEXTURES).map(([name, path]) =>
            this.loadTexture(name as keyof typeof TEXTURES, path),
        );
        await Promise.all(promises);
    }
}
