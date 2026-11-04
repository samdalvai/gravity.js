import type { IconName } from './Icons';

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
    icon: IconName;
    label: string;
    /** Short explanation shown in the touch controls help */
    help: string;
    /** Desktop shortcut equivalent */
    desktop: string;
}

export const MOBILE_TOOLS: MobileToolDefinition[] = [
    {
        id: 'grab',
        icon: 'grab',
        label: 'Grab',
        help: 'Drag a body to move it, tap on empty space to spawn a ball',
        desktop: 'Left Mouse',
    },
    { id: 'box', icon: 'box', label: 'Box', help: 'Tap to spawn a box', desktop: 'Right Mouse' },
    { id: 'capsule', icon: 'capsule', label: 'Capsule', help: 'Tap to spawn a capsule', desktop: 'X' },
    { id: 'polygon', icon: 'polygon', label: 'Polygon', help: 'Tap to spawn a random convex polygon', desktop: 'R' },
    { id: 'segment', icon: 'segment', label: 'Segment', help: 'Tap to spawn a static segment', desktop: 'Z' },
    {
        id: 'particles',
        icon: 'particles',
        label: 'Particles',
        help: 'Hold and drag to emit particles',
        desktop: 'C (hold)',
    },
    { id: 'bullet', icon: 'bullet', label: 'Bullet', help: 'Tap to shoot a bullet from the origin', desktop: 'B' },
    { id: 'explosion', icon: 'explosion', label: 'Explode', help: 'Tap to trigger an explosion', desktop: 'E' },
    {
        id: 'blackHole',
        icon: 'blackHole',
        label: 'Black hole',
        help: 'Tap to place a black hole (use “Clear hole” to remove it)',
        desktop: 'F / Shift + F',
    },
    {
        id: 'player',
        icon: 'player',
        label: 'Player',
        help: 'Tap to spawn the player, then use the move and jump buttons',
        desktop: 'Q, Arrows, Space',
    },
    { id: 'pan', icon: 'pan', label: 'Move', help: 'Drag with one finger to move the camera', desktop: 'Middle Mouse' },
];

export const MOBILE_GESTURES: string[] = [
    'Pinch with two fingers to zoom',
    'Drag with two fingers to move the camera',
    '“Fit” resets the camera to the demo view',
    'The settings button (top left) opens all the simulation settings',
    'The arrows next to the demo name switch to the previous / next demo',
];
