import { MOBILE_GESTURES, MOBILE_TOOLS, type MobileTool } from './MobileTools';
import MobileUI from './MobileUI';
import type { ViewMode } from './ViewMode';

const SHORTCUT_SECTIONS = [
    {
        title: 'Scene',
        items: [
            '[ 0-99 ] Select demo (type 2 digits quickly)',
            '[ Shift + R ] Restart demo',
            '[ G ] Toggle gravity',
            '[ D ] Toggle debug rendering',
            '[ A ] Toggle AABB debug',
            '[ S ] Toggle contact and joint debug',
            '[ P ] Pause simulation',
            '[ . ] Step simulation',
            '[ + ] [ - ] Increase or decrease solver iterations',
            '[ * ] [ / ] Increase or decrease substeps',
        ],
    },
    {
        title: 'Spawn',
        items: [
            '[ Left Mouse ] Spawn circle / Grab body',
            '[ Right Mouse ] Spawn box',
            '[ C ] Spawn particles while held',
            '[ X ] Spawn capsule',
            '[ Z ] Spawn segment',
            '[ R ] Spawn random convex polygon',
            '[ B ] Shoot bullet',
            '[ E ] Trigger explosion at mouse',
            '[ F ] Spawn black hole at mouse',
            '[ Shift + F ] Clear black hole',
        ],
    },
    {
        title: 'Player And Camera',
        items: [
            '[ Q ] Spawn player at mouse',
            '[ Space ] Jump',
            '[ Left Arrow ] [ Right Arrow ] Move player',
            '[ Middle Mouse ] Drag camera',
            '[ Mouse Wheel ] Zoom camera',
        ],
    },
];

const TOUCH_SECTIONS = [
    {
        title: 'Tools (bottom bar)',
        items: MOBILE_TOOLS.map(tool => `${tool.label}: ${tool.help}`),
    },
    {
        title: 'Gestures',
        items: MOBILE_GESTURES,
    },
];

export interface UIState {
    demoIndex: number;
    demoLabels: string[];
    debug: boolean;
    showLabels: boolean;
    showAABB: boolean;
    showContacts: boolean;
    showRuntimeStatsHud: boolean;
    ccd: boolean;
    applyGravity: boolean;
    grab: boolean;
    paused: boolean;
    frequency: number;
    solverIterations: number;
    subSteps: number;
    viewMode: ViewMode;
    tool: MobileTool;
    hasPlayer: boolean;
    hasBlackHole: boolean;
}

export interface UIActions {
    onSelectDemo(index: number): void;
    onRestartDemo(): void;
    onSetDebug(value: boolean): void;
    onSetShowLabels(value: boolean): void;
    onSetShowAABB(value: boolean): void;
    onSetShowContacts(value: boolean): void;
    onSetShowRuntimeStatsHud(value: boolean): void;
    onSetCCD(checked: boolean): void;
    onSetApplyGravity(value: boolean): void;
    onSetGrab(checked: boolean): void;
    onSetPaused(value: boolean): void;
    onSetFrequency(value: number): void;
    onSetSolverIterations(value: number): void;
    onSetSubSteps(value: number): void;
    onStep(): void;
    onSetViewMode(mode: ViewMode): void;
    onSelectTool(tool: MobileTool): void;
    onClearBlackHole(): void;
    onFitView(): void;
    onPlayerMove(direction: 'left' | 'right', pressed: boolean): void;
    onPlayerJump(): void;
}

export default class UIManager {
    private currentState: UIState | null = null;
    private toolbar: HTMLElement | null = null;

    // Select
    private demoSelect: HTMLSelectElement | null = null;

    // Checkboxes
    private debugCheckbox: HTMLInputElement | null = null;
    private showLabelsCheckbox: HTMLInputElement | null = null;
    private showAABBCheckbox: HTMLInputElement | null = null;
    private ccdCheckbox: HTMLInputElement | null = null;
    private showContactsCheckbox: HTMLInputElement | null = null;
    private gravityCheckbox: HTMLInputElement | null = null;
    private pausedCheckbox: HTMLInputElement | null = null;

    // Inputs
    private frequencyInput: HTMLInputElement | null = null;
    private solverIterationsInput: HTMLInputElement | null = null;
    private subStepsInput: HTMLInputElement | null = null;

    // Buttons
    private stepButton: HTMLButtonElement | null = null;
    private restartButton: HTMLButtonElement | null = null;
    private runtimeStatsButton: HTMLButtonElement | null = null;
    private shortcutsButton: HTMLButtonElement | null = null;
    private shortcutsDialog: HTMLDialogElement | null = null;
    private grabCheckbox: HTMLInputElement | null = null;
    private viewModeButton: HTMLButtonElement | null = null;
    private shortcutsTitle: HTMLHeadingElement | null = null;
    private shortcutsContent: HTMLElement | null = null;
    private renderedHelpMode: ViewMode | null = null;
    private readonly mobileUI = new MobileUI();

    initialize(state: UIState, actions: UIActions): void {
        this.currentState = state;
        const toolbar = document.getElementById('demo-toolbar');
        if (!(toolbar instanceof HTMLElement)) {
            return;
        }

        this.toolbar = toolbar;
        toolbar.replaceChildren();

        const createGroup = () => {
            const group = document.createElement('div');
            group.className = 'toolbar-group';
            toolbar.appendChild(group);
            return group;
        };

        const createLabeledControl = (group: HTMLElement, labelText: string, control: HTMLElement) => {
            const label = document.createElement('label');
            label.className = 'toolbar-control';

            const text = document.createElement('span');
            text.textContent = labelText;

            label.append(text, control);
            group.appendChild(label);
        };

        const createCheckbox = (group: HTMLElement, labelText: string, onChange: (checked: boolean) => void) => {
            const label = document.createElement('label');
            label.className = 'toolbar-control';

            const input = document.createElement('input');
            input.type = 'checkbox';
            input.addEventListener('change', () => onChange(input.checked));

            label.append(input, document.createTextNode(labelText));
            group.appendChild(label);

            return input;
        };

        const createNumberInput = (group: HTMLElement, labelText: string, onChange: (value: number) => void) => {
            const input = document.createElement('input');
            input.type = 'number';
            input.min = '1';
            input.step = '1';
            input.addEventListener('change', () => onChange(Number.parseInt(input.value)));
            createLabeledControl(group, labelText, input);
            return input;
        };

        const demoGroup = createGroup();
        this.demoSelect = document.createElement('select');
        state.demoLabels.forEach((label, index) => {
            const option = document.createElement('option');
            option.value = `${index}`;
            option.textContent = label;
            this.demoSelect!.appendChild(option);
        });
        this.demoSelect.addEventListener('change', () => {
            const index = Number.parseInt(this.demoSelect!.value);
            this.demoSelect!.blur();
            actions.onSelectDemo(index);
            this.mobileUI.toggleSettings(false);
        });
        createLabeledControl(demoGroup, 'Demo', this.demoSelect);

        this.restartButton = document.createElement('button');
        this.restartButton.type = 'button';
        this.restartButton.className = 'toolbar-button';
        this.restartButton.textContent = 'Restart';
        this.restartButton.addEventListener('click', () => actions.onRestartDemo());
        demoGroup.appendChild(this.restartButton);

        const toggleGroup = createGroup();
        this.debugCheckbox = createCheckbox(toggleGroup, 'Debug', checked => actions.onSetDebug(checked));
        this.showLabelsCheckbox = createCheckbox(toggleGroup, 'Labels', checked => actions.onSetShowLabels(checked));
        this.showAABBCheckbox = createCheckbox(toggleGroup, 'Show AABB', checked => actions.onSetShowAABB(checked));
        this.showContactsCheckbox = createCheckbox(toggleGroup, 'Show Contacts', checked =>
            actions.onSetShowContacts(checked),
        );
        this.ccdCheckbox = createCheckbox(toggleGroup, 'CCD', checked => actions.onSetCCD(checked));
        this.gravityCheckbox = createCheckbox(toggleGroup, 'Gravity', checked => actions.onSetApplyGravity(checked));
        this.grabCheckbox = createCheckbox(toggleGroup, 'Grab', checked => actions.onSetGrab(checked));
        this.pausedCheckbox = createCheckbox(toggleGroup, 'Paused', checked => actions.onSetPaused(checked));

        const numericGroup = createGroup();
        this.frequencyInput = createNumberInput(numericGroup, 'Frequency (Hz)', value => actions.onSetFrequency(value));
        this.solverIterationsInput = createNumberInput(numericGroup, 'Iterations', value =>
            actions.onSetSolverIterations(value),
        );
        this.subStepsInput = createNumberInput(numericGroup, 'Substeps', value => actions.onSetSubSteps(value));

        this.stepButton = document.createElement('button');
        this.stepButton.type = 'button';
        this.stepButton.className = 'toolbar-button';
        this.stepButton.textContent = 'Step';
        this.stepButton.addEventListener('click', () => actions.onStep());
        numericGroup.appendChild(this.stepButton);

        const actionsGroup = createGroup();
        actionsGroup.classList.add('toolbar-group-actions');
        this.runtimeStatsButton = document.createElement('button');
        this.runtimeStatsButton.type = 'button';
        this.runtimeStatsButton.className = 'toolbar-button';
        this.runtimeStatsButton.addEventListener('click', () =>
            actions.onSetShowRuntimeStatsHud(!this.currentState?.showRuntimeStatsHud),
        );
        actionsGroup.appendChild(this.runtimeStatsButton);

        this.shortcutsButton = document.createElement('button');
        this.shortcutsButton.type = 'button';
        this.shortcutsButton.className = 'toolbar-button';
        this.shortcutsButton.textContent = 'Shortcuts';
        this.shortcutsButton.addEventListener('click', () => this.shortcutsDialog?.showModal());
        actionsGroup.appendChild(this.shortcutsButton);

        this.viewModeButton = document.createElement('button');
        this.viewModeButton.type = 'button';
        this.viewModeButton.className = 'toolbar-button toolbar-button-accent';
        this.viewModeButton.addEventListener('click', () =>
            actions.onSetViewMode(this.currentState?.viewMode === 'mobile' ? 'desktop' : 'mobile'),
        );
        actionsGroup.appendChild(this.viewModeButton);

        this.shortcutsDialog = document.createElement('dialog');
        this.shortcutsDialog.id = 'demo-shortcuts-modal';
        this.shortcutsDialog.addEventListener('click', event => {
            if (event.target === this.shortcutsDialog) {
                this.shortcutsDialog?.close();
            }
        });

        const dialogHeader = document.createElement('div');
        dialogHeader.className = 'shortcuts-modal-header';

        const dialogTitle = document.createElement('h2');
        this.shortcutsTitle = dialogTitle;

        const closeButton = document.createElement('button');
        closeButton.type = 'button';
        closeButton.className = 'toolbar-button';
        closeButton.textContent = 'Close';
        closeButton.addEventListener('click', () => this.shortcutsDialog?.close());

        dialogHeader.append(dialogTitle, closeButton);
        this.shortcutsDialog.appendChild(dialogHeader);

        const dialogContent = document.createElement('div');
        dialogContent.className = 'shortcuts-modal-content';
        this.shortcutsContent = dialogContent;

        this.shortcutsDialog.appendChild(dialogContent);
        document.body.appendChild(this.shortcutsDialog);

        this.mobileUI.initialize(state, actions, {
            onOpenHelp: () => this.shortcutsDialog?.showModal(),
        });

        this.sync(state);
    }

    sync(state: UIState): void {
        this.currentState = state;
        this.mobileUI.sync(state);
        this.renderHelp(state.viewMode);

        if (this.viewModeButton) {
            this.viewModeButton.textContent = state.viewMode === 'mobile' ? 'Switch to desktop view' : 'Mobile view';
        }

        if (this.shortcutsButton) {
            this.shortcutsButton.textContent = state.viewMode === 'mobile' ? 'Touch controls' : 'Shortcuts';
        }
        if (
            !this.demoSelect ||
            !this.debugCheckbox ||
            !this.showLabelsCheckbox ||
            !this.showAABBCheckbox ||
            !this.showContactsCheckbox ||
            !this.ccdCheckbox ||
            !this.gravityCheckbox ||
            !this.grabCheckbox ||
            !this.pausedCheckbox ||
            !this.frequencyInput ||
            !this.solverIterationsInput ||
            !this.subStepsInput
        ) {
            return;
        }

        this.demoSelect.value = `${state.demoIndex}`;

        this.debugCheckbox.checked = state.debug;
        this.showLabelsCheckbox.checked = state.showLabels;
        this.showAABBCheckbox.checked = state.showAABB;
        this.showContactsCheckbox.checked = state.showContacts;

        this.ccdCheckbox.checked = state.ccd;
        this.gravityCheckbox.checked = state.applyGravity;
        this.grabCheckbox.checked = state.grab;
        this.pausedCheckbox.checked = state.paused;
        this.frequencyInput.value = `${state.frequency}`;
        this.solverIterationsInput.value = `${state.solverIterations}`;
        this.subStepsInput.value = `${state.subSteps}`;
        this.showAABBCheckbox.disabled = !state.debug;
        this.showContactsCheckbox.disabled = !state.debug;

        if (this.stepButton) {
            this.stepButton.disabled = !state.paused;
        }

        if (this.runtimeStatsButton) {
            this.runtimeStatsButton.textContent = state.showRuntimeStatsHud ? 'Collapse HUD' : 'Expand HUD';
            this.runtimeStatsButton.disabled = !state.debug;
        }
    }

    get headerHeight(): number {
        if (this.currentState?.viewMode === 'mobile') {
            return this.mobileUI.height;
        }

        return this.toolbar?.offsetHeight ?? 0;
    }

    private renderHelp(mode: ViewMode): void {
        if (!this.shortcutsContent || this.renderedHelpMode === mode) {
            return;
        }

        this.renderedHelpMode = mode;

        if (this.shortcutsTitle) {
            this.shortcutsTitle.textContent = mode === 'mobile' ? 'Touch Controls' : 'Keyboard And Mouse Shortcuts';
        }

        const sections = mode === 'mobile' ? TOUCH_SECTIONS : SHORTCUT_SECTIONS;
        this.shortcutsContent.replaceChildren();

        sections.forEach(section => {
            const sectionEl = document.createElement('section');
            sectionEl.className = 'shortcuts-section';

            const title = document.createElement('h3');
            title.textContent = section.title;
            sectionEl.appendChild(title);

            const list = document.createElement('ul');
            section.items.forEach(item => {
                const listItem = document.createElement('li');
                listItem.textContent = item;
                list.appendChild(listItem);
            });

            sectionEl.appendChild(list);
            this.shortcutsContent?.appendChild(sectionEl);
        });
    }
}
