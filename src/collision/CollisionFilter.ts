export const enum CollisionCategory {
    NONE = 0,
    DEFAULT = 1 << 0,
    PROJECTILE = 1 << 2,
    PARTICLE = 1 << 3,
    SENSOR = 1 << 4,
    // Add other collision filters here
    ALL = 0xffffffff,
}

export type CollisionFilter = {
    category: number;
    mask: number;
};

export const DEFAULT_COLLISION_FILTER: CollisionFilter = {
    category: CollisionCategory.DEFAULT,
    mask: CollisionCategory.ALL,
};

export function canCollide(a: CollisionFilter, b: CollisionFilter): boolean {
    return (a.mask & b.category) !== 0 && (b.mask & a.category) !== 0;
}
