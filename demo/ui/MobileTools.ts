export type MobileTool =
    | 'grab'
    | 'box'
    | 'capsule'
    | 'polygon'
    | 'segment'
    | 'particles'
    | 'bullet'
    | 'explosion'
    | 'blackHole'
    | 'player'
    | 'pan';

export interface MobileToolDefinition {
    id: MobileTool;
    icon: string;
    label: string;
    /** Short explanation shown in the touch controls help */
    help: string;
    /** Desktop shortcut equivalent */
    desktop: string;
}

export const MOBILE_TOOLS: MobileToolDefinition[] = [
    {
        id: 'grab',
        icon: '✋',
        label: 'Grab',
        help: 'Drag a body to move it, tap on empty space to spawn a ball',
        desktop: 'Left Mouse',
    },
    { id: 'box', icon: '📦', label: 'Box', help: 'Tap to spawn a box', desktop: 'Right Mouse' },
    { id: 'capsule', icon: '💊', label: 'Capsule', help: 'Tap to spawn a capsule', desktop: 'X' },
    { id: 'polygon', icon: '🔷', label: 'Polygon', help: 'Tap to spawn a random convex polygon', desktop: 'R' },
    { id: 'segment', icon: '➖', label: 'Segment', help: 'Tap to spawn a static segment', desktop: 'Z' },
    {
        id: 'particles',
        icon: '✨',
        label: 'Particles',
        help: 'Hold and drag to emit particles',
        desktop: 'C (hold)',
    },
    { id: 'bullet', icon: '🎯', label: 'Bullet', help: 'Tap to shoot a bullet from the origin', desktop: 'B' },
    { id: 'explosion', icon: '💥', label: 'Explode', help: 'Tap to trigger an explosion', desktop: 'E' },
    {
        id: 'blackHole',
        icon: '🕳️',
        label: 'Black hole',
        help: 'Tap to place a black hole (use “Clear hole” to remove it)',
        desktop: 'F / Shift + F',
    },
    {
        id: 'player',
        icon: '🏃',
        label: 'Player',
        help: 'Tap to spawn the player, then use the ◀ ▶ ⤒ buttons',
        desktop: 'Q, Arrows, Space',
    },
    { id: 'pan', icon: '🧭', label: 'Move', help: 'Drag with one finger to move the camera', desktop: 'Middle Mouse' },
];

export const MOBILE_GESTURES: string[] = [
    'Pinch with two fingers to zoom',
    'Drag with two fingers to move the camera',
    '“Fit” resets the camera to the demo view',
    '⚙ opens all the simulation settings',
    '‹ › switch to the previous / next demo',
];
