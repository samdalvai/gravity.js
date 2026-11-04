/**
 * Minimal monochrome line icons (24x24, drawn with currentColor) used by the
 * mobile UI. Kept as SVG instead of unicode glyphs because some platforms render
 * characters such as ▶ or ⚙ as colored emoji.
 */
const ICON_PATHS = {
    // Tools
    grab: '<path d="M6 3.5l11.5 8.2-5.2 1.1 3 6.2-2.4 1.1-3-6.2L6 17.6z"/>',
    box: '<rect x="5" y="5" width="14" height="14" rx="1"/>',
    capsule: '<rect x="8" y="3" width="8" height="18" rx="4"/>',
    polygon: '<path d="M12 3.5l8 5.8-3 9.2H7l-3-9.2z"/>',
    segment:
        '<path d="M5 12h14"/><circle cx="5" cy="12" r="1.6" fill="currentColor"/><circle cx="19" cy="12" r="1.6" fill="currentColor"/>',
    particles:
        '<g fill="currentColor" stroke="none"><circle cx="6" cy="7" r="1.4"/><circle cx="12" cy="5" r="1.4"/><circle cx="17.5" cy="8" r="1.4"/><circle cx="8.5" cy="12" r="1.4"/><circle cx="15" cy="13" r="1.4"/><circle cx="5.5" cy="17" r="1.4"/><circle cx="11.5" cy="18.5" r="1.4"/><circle cx="18" cy="17.5" r="1.4"/></g>',
    bullet: '<path d="M3 15h7M5 11h6M3 7h5"/><circle cx="16" cy="11" r="3.5"/>',
    explosion:
        '<circle cx="12" cy="12" r="2.5"/><path d="M12 3v3.5M12 17.5V21M3 12h3.5M17.5 12H21M5.6 5.6l2.5 2.5M15.9 15.9l2.5 2.5M5.6 18.4l2.5-2.5M15.9 8.1l2.5-2.5"/>',
    blackHole:
        '<circle cx="12" cy="12" r="8.5" stroke-dasharray="2.5 2.5"/><circle cx="12" cy="12" r="5"/><circle cx="12" cy="12" r="1.8" fill="currentColor"/>',
    player: '<circle cx="12" cy="5.5" r="2.5"/><rect x="8.5" y="9.5" width="7" height="11" rx="3.5"/>',
    pan: '<path d="M12 3v18M3 12h18M12 3l-2.5 2.5M12 3l2.5 2.5M12 21l-2.5-2.5M12 21l2.5-2.5M3 12l2.5-2.5M3 12l2.5 2.5M21 12l-2.5-2.5M21 12l-2.5 2.5"/>',

    // Bar and pad
    settings: '<path d="M4 7h16M4 12h16M4 17h16M9 4.5v5M15 9.5v5M8 14.5v5"/>',
    prev: '<path d="M14.5 6l-6 6 6 6"/>',
    next: '<path d="M9.5 6l6 6-6 6"/>',
    restart: '<path d="M19 12a7 7 0 1 1-2.05-4.95"/><path d="M19 4.5V8h-3.5"/>',
    pause: '<path d="M9 6v12M15 6v12"/>',
    play: '<path d="M8 5.5v13l10-6.5z"/>',
    left: '<path d="M15 5.5v13L5 12z"/>',
    right: '<path d="M9 5.5v13L19 12z"/>',
    jump: '<path d="M12 20V6M6.5 11.5L12 6l5.5 5.5M6 3.5h12"/>',
} as const;

export type IconName = keyof typeof ICON_PATHS;

export function iconSvg(name: IconName): string {
    return (
        '<svg class="ui-icon" viewBox="0 0 24 24" width="24" height="24" fill="none" stroke="currentColor" ' +
        `stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${ICON_PATHS[name]}</svg>`
    );
}

export function setIcon(element: HTMLElement, name: IconName): void {
    element.innerHTML = iconSvg(name);
}
