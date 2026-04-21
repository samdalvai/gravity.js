export const enum CollisionCategory {
    NONE = 0,
    DEFAULT = 1 << 0,
    PROJECTILE = 1 << 2,
    PARTICLE = 1 << 3,
    SENSOR = 1 << 4,
    LAYER1 = 1 << 5,
    LAYER2 = 1 << 6,
    LAYER3 = 1 << 7,
    ALL = 0xffffffff,
}

/**
 * Describes how a body participates in broad-phase collision filtering.
 *
 * `category` is the single category bit assigned to the body.
 * `mask` is the set of categories that the body agrees to collide with.
 *
 * Simple cases:
 * - `{ category: CollisionCategory.DEFAULT, mask: CollisionCategory.ALL }`
 *   collides with every category.
 * - `{ category: CollisionCategory.SENSOR, mask: CollisionCategory.NONE }`
 *   collides with nothing.
 * - `{ category: CollisionCategory.PROJECTILE, mask: CollisionCategory.DEFAULT }`
 *   collides only with bodies in the `DEFAULT` category.
 *
 * ```ts
 * const player: CollisionFilter = {
 *   category: CollisionCategory.DEFAULT,
 *   mask: CollisionCategory.ALL,
 * };
 *
 * const projectile: CollisionFilter = {
 *   category: CollisionCategory.PROJECTILE,
 *   mask: CollisionCategory.DEFAULT,
 * };
 *
 * canCollide(player, projectile); // true
 *
 * const particle: CollisionFilter = {
 *   category: CollisionCategory.PARTICLE,
 *   mask: CollisionCategory.ALL & ~CollisionCategory.DEFAULT,
 * };
 *
 * canCollide(player, particle); // false
 * ```
 */
export type CollisionFilter = {
    category: CollisionCategory;
    mask: CollisionCategory;
};

export const DEFAULT_COLLISION_FILTER: CollisionFilter = {
    category: CollisionCategory.DEFAULT,
    mask: CollisionCategory.ALL,
};

/**
 * Returns `true` only when both filters opt into the collision.
 *
 * In other words:
 * - `a.mask` must include `b.category`
 * - `b.mask` must include `a.category`
 */
export function canCollide(a: CollisionFilter, b: CollisionFilter): boolean {
    return (a.mask & b.category) !== 0 && (b.mask & a.category) !== 0;
}
