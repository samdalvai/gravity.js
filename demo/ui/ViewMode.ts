export type ViewMode = 'desktop' | 'mobile';

const STORAGE_KEY = 'gravity.viewMode';
const QUERY_PARAM = 'view';

function isViewMode(value: unknown): value is ViewMode {
    return value === 'desktop' || value === 'mobile';
}

/**
 * Initial view mode, by priority:
 * 1. `?view=mobile` / `?view=desktop` query parameter
 * 2. the last mode chosen by the user
 * 3. auto detection (touch-first device with a small screen)
 */
export function getInitialViewMode(): ViewMode {
    const fromQuery = new URLSearchParams(window.location.search).get(QUERY_PARAM);
    if (isViewMode(fromQuery)) {
        return fromQuery;
    }

    try {
        const stored = window.localStorage.getItem(STORAGE_KEY);
        if (isViewMode(stored)) {
            return stored;
        }
    } catch {
        // Storage may be unavailable (private mode, sandboxed frames)
    }

    return detectViewMode();
}

export function detectViewMode(): ViewMode {
    const coarsePointer = window.matchMedia?.('(pointer: coarse)').matches ?? false;
    const smallScreen = Math.min(window.innerWidth, window.innerHeight) < 820;
    return coarsePointer && smallScreen ? 'mobile' : 'desktop';
}

export function saveViewMode(mode: ViewMode): void {
    try {
        window.localStorage.setItem(STORAGE_KEY, mode);
    } catch {
        // Ignore, the choice will simply not be remembered
    }
}
