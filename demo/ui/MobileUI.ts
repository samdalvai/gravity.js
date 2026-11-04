import { MOBILE_TOOLS, type MobileTool } from './MobileTools';
import type { UIActions, UIState } from './UIManager';

const SETTINGS_OPEN_CLASS = 'mobile-settings-open';

interface MobileUICallbacks {
    onOpenHelp(): void;
}

function createButton(className: string, content: string, title: string): HTMLButtonElement {
    const button = document.createElement('button');
    button.type = 'button';
    button.className = className;
    button.textContent = content;
    button.title = title;
    button.setAttribute('aria-label', title);
    return button;
}

/**
 * Touch-first controls: a compact top bar, a tool palette at the bottom and an
 * on-screen pad for the player. All elements carry the `mobile-ui` class so that
 * InputManager does not treat touches on them as canvas input.
 */
export default class MobileUI {
    private bar: HTMLElement | null = null;
    private demoTitle: HTMLButtonElement | null = null;
    private pauseButton: HTMLButtonElement | null = null;
    private clearHoleButton: HTMLButtonElement | null = null;
    private playerPad: HTMLElement | null = null;
    private readonly toolButtons = new Map<MobileTool, HTMLButtonElement>();
    private state: UIState | null = null;

    initialize(state: UIState, actions: UIActions, callbacks: MobileUICallbacks): void {
        this.state = state;

        // Top bar
        const bar = document.createElement('header');
        bar.id = 'mobile-bar';
        bar.className = 'mobile-ui';

        const settingsButton = createButton('mobile-icon-button', '⚙', 'Settings');
        settingsButton.addEventListener('click', () => this.toggleSettings());

        const prevButton = createButton('mobile-icon-button', '‹', 'Previous demo');
        prevButton.addEventListener('click', () => this.selectRelativeDemo(actions, -1));

        this.demoTitle = createButton('mobile-demo-title', '', 'Choose demo');
        this.demoTitle.addEventListener('click', () => this.toggleSettings(true));

        const nextButton = createButton('mobile-icon-button', '›', 'Next demo');
        nextButton.addEventListener('click', () => this.selectRelativeDemo(actions, 1));

        const restartButton = createButton('mobile-icon-button', '↻', 'Restart demo');
        restartButton.addEventListener('click', () => actions.onRestartDemo());

        this.pauseButton = createButton('mobile-icon-button', '⏸', 'Pause');
        this.pauseButton.addEventListener('click', () => actions.onSetPaused(!this.state?.paused));

        bar.append(settingsButton, prevButton, this.demoTitle, nextButton, restartButton, this.pauseButton);

        // Backdrop closing the settings sheet
        const backdrop = document.createElement('div');
        backdrop.id = 'mobile-settings-backdrop';
        backdrop.className = 'mobile-ui';
        backdrop.addEventListener('click', () => this.toggleSettings(false));

        // Tool palette
        const palette = document.createElement('nav');
        palette.id = 'mobile-palette';
        palette.className = 'mobile-ui';

        const tools = document.createElement('div');
        tools.className = 'mobile-tools';
        for (const tool of MOBILE_TOOLS) {
            const button = createButton('mobile-tool', '', tool.label);
            const icon = document.createElement('span');
            icon.className = 'mobile-tool-icon';
            icon.textContent = tool.icon;
            const label = document.createElement('span');
            label.className = 'mobile-tool-label';
            label.textContent = tool.label;
            button.append(icon, label);
            button.addEventListener('click', () => actions.onSelectTool(tool.id));
            this.toolButtons.set(tool.id, button);
            tools.appendChild(button);
        }

        const quickActions = document.createElement('div');
        quickActions.className = 'mobile-quick-actions';

        const fitButton = createButton('mobile-chip', 'Fit', 'Reset camera');
        fitButton.addEventListener('click', () => actions.onFitView());

        this.clearHoleButton = createButton('mobile-chip', 'Clear hole', 'Remove black hole');
        this.clearHoleButton.addEventListener('click', () => actions.onClearBlackHole());

        const helpButton = createButton('mobile-chip', '?', 'Touch controls');
        helpButton.addEventListener('click', () => callbacks.onOpenHelp());

        quickActions.append(fitButton, this.clearHoleButton, helpButton);

        // Player pad (anchored right above the palette)
        this.playerPad = document.createElement('div');
        this.playerPad.id = 'mobile-player-pad';
        this.playerPad.className = 'mobile-ui';

        const moveGroup = document.createElement('div');
        moveGroup.className = 'mobile-pad-group';
        moveGroup.append(
            this.createHoldButton('◀', 'Move left', pressed => actions.onPlayerMove('left', pressed)),
            this.createHoldButton('▶', 'Move right', pressed => actions.onPlayerMove('right', pressed)),
        );

        const jumpButton = createButton('mobile-pad-button', '⤒', 'Jump');
        jumpButton.addEventListener('pointerdown', event => {
            event.preventDefault();
            actions.onPlayerJump();
        });

        this.playerPad.append(moveGroup, jumpButton);
        palette.append(this.playerPad, tools, quickActions);

        document.body.append(bar, backdrop, palette);
        this.bar = bar;

        this.sync(state);
    }

    sync(state: UIState): void {
        this.state = state;

        if (this.demoTitle) {
            const label = state.demoLabels[state.demoIndex] ?? '';
            this.demoTitle.textContent = label.replace(/^Demo (\d+): /, '$1 · ');
        }

        if (this.pauseButton) {
            this.pauseButton.textContent = state.paused ? '▶' : '⏸';
            this.pauseButton.title = state.paused ? 'Resume' : 'Pause';
            this.pauseButton.classList.toggle('is-active', state.paused);
        }

        for (const [tool, button] of this.toolButtons) {
            const selected = tool === state.tool;
            button.classList.toggle('is-selected', selected);
            button.setAttribute('aria-pressed', `${selected}`);
        }

        if (this.clearHoleButton) {
            this.clearHoleButton.disabled = !state.hasBlackHole;
        }

        if (this.playerPad) {
            this.playerPad.classList.toggle('is-visible', state.hasPlayer);
        }

        if (state.viewMode !== 'mobile') {
            this.toggleSettings(false);
        }
    }

    get height(): number {
        return this.bar?.offsetHeight ?? 0;
    }

    toggleSettings(open?: boolean): void {
        const shouldOpen = open ?? !document.body.classList.contains(SETTINGS_OPEN_CLASS);
        document.body.classList.toggle(SETTINGS_OPEN_CLASS, shouldOpen);
    }

    private selectRelativeDemo(actions: UIActions, offset: number): void {
        if (!this.state) {
            return;
        }

        const count = this.state.demoLabels.length;
        actions.onSelectDemo((this.state.demoIndex + offset + count) % count);
    }

    private createHoldButton(content: string, title: string, onChange: (pressed: boolean) => void): HTMLButtonElement {
        const button = createButton('mobile-pad-button', content, title);
        let pressed = false;

        const set = (value: boolean) => {
            if (pressed === value) {
                return;
            }

            pressed = value;
            button.classList.toggle('is-active', value);
            onChange(value);
        };

        button.addEventListener('pointerdown', event => {
            event.preventDefault();
            button.setPointerCapture?.(event.pointerId);
            set(true);
        });
        button.addEventListener('pointerup', () => set(false));
        button.addEventListener('pointercancel', () => set(false));
        button.addEventListener('lostpointercapture', () => set(false));
        button.addEventListener('contextmenu', event => event.preventDefault());

        return button;
    }
}
