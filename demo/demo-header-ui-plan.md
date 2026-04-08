# Demo Header UI Plan

## Goal

Add a simple HTML header above the demo canvas while keeping the current keyboard controls as shortcuts.

The header should allow:

- Selecting a demo from a dropdown
- Toggling debug options such as `debug`, `show AABB`, `show contacts`, `apply gravity`
- Adjusting numeric settings such as `solverIterations` and `subSteps`

The implementation should stay as small and direct as possible.

## Current Structure

### `demo/Application.ts`

This is already the main place where the demo state lives.

It currently owns:

- `demoIndex`
- `debug`
- `showAABB`
- `showContacts`
- `paused`
- Mutations of `SETTINGS.applyGravity`
- Mutations of `SETTINGS.solverIterations`
- Mutations of `SETTINGS.subSteps`

It also already handles:

- Switching demos
- Rendering the debug overlay
- Keyboard shortcuts for all relevant settings

Because of that, `Application` should remain the single source of truth for the new header.

### `demo/samples/Demo.ts`

This already exposes:

- `Demo.demoStrings`
- `Demo.demoFunctions`

That is enough to populate a demo selector without introducing a new registry.

### `demo/input/InputManager.ts`

This file currently listens to all keyboard and mouse events on `window` and forwards them into buffers.

This matters because adding native HTML controls means:

- keyboard events from a `<select>` or `<input>` could still trigger game shortcuts
- mouse wheel on a number input could still zoom the camera
- mouse interaction with the header could interfere with world input

This is the main integration detail to handle carefully.

### `index.html` and `styles.css`

The page is currently just a fullscreen canvas.

The layout is simple, but the current flex-centered body is not ideal for a fixed top header. The page structure should be slightly simplified so a toolbar can sit above the canvas cleanly.

## Recommended Approach

Keep things plain and local:

- Add one fixed HTML toolbar above the canvas
- Build it with direct DOM APIs
- Keep keyboard shortcuts exactly as they are
- Make both keyboard and HTML controls call the same `Application` methods
- Do not introduce any framework or new state layer

## Implementation Steps

### 1. Add a toolbar container in `index.html`

Add a top-level element before the canvas, for example:

```html
<header id="demo-toolbar"></header>
<canvas id="gamePhysicsCanvas"></canvas>
```

This gives the app a stable mount point for the controls.

### 2. Adjust page layout in `styles.css`

Replace the current body centering with a more natural fullscreen layout.

Recommended direction:

- `body` should still fill the viewport
- the toolbar should be `position: fixed` at the top
- the canvas should remain fullscreen underneath
- the toolbar should have a background, padding, and a higher `z-index`

The simplest approach is to keep the canvas fullscreen and overlay the toolbar, instead of shrinking the canvas area. That avoids any changes to camera math or world rendering.

### 3. Extract shared state-changing methods in `Application.ts`

Right now the keyboard logic directly mutates fields and settings.

Create small methods such as:

- `loadDemo(index: number)`
- `setDebug(value: boolean)`
- `setShowAABB(value: boolean)`
- `setShowContacts(value: boolean)`
- `setPaused(value: boolean)`
- `setApplyGravity(value: boolean)`
- `setSolverIterations(value: number)`
- `setSubSteps(value: number)`

These methods should:

- update the relevant property or `SETTINGS` value
- clamp numeric values to a minimum of `1` where needed
- call a UI sync method after state changes

This avoids duplicating logic between keyboard input and header controls.

### 4. Extract the existing demo-reset logic into `loadDemo`

`setup()` and the number-key handler both perform very similar work:

- clear world
- clear background
- clear render registry
- reset player
- reset view
- run the selected demo

Put this into a single `loadDemo(index)` method and reuse it from:

- initial setup
- keyboard number shortcuts
- demo selector change event

That makes the new UI much easier to add and lowers the risk of state getting out of sync.

### 5. Add toolbar creation in `Application.ts`

Add a method such as `setupToolbar()` that:

- locates `#demo-toolbar`
- creates the controls once
- attaches event listeners

Also add `syncToolbar()` that updates the DOM controls from the current app state.

This split keeps responsibilities simple:

- `setupToolbar()` builds the UI
- `syncToolbar()` keeps it accurate

### 6. Keep the toolbar controls minimal

For the first pass, only expose persistent settings that already exist in the app.

Recommended controls:

- Demo `<select>`
- `Debug` checkbox
- `Show AABB` checkbox
- `Show Contacts` checkbox
- `Apply Gravity` checkbox
- `Paused` checkbox or toggle
- `Solver Iterations` number input
- `SubSteps` number input
- Optional `Step` button to run one frame while paused

Avoid putting one-off actions into the header for now, such as:

- explosion
- spawn player
- shoot bullet
- random body generation

Those actions are better left on keyboard/mouse for now.

### 7. Make keyboard and header use the same methods

Update the keyboard handler so it no longer changes state directly where avoidable.

Examples:

- `d` should call `setDebug(!this.debug)`
- `g` should call `setApplyGravity(!SETTINGS.applyGravity)`
- `a` should call `setShowAABB(!this.showAABB)`
- `s` should call `setShowContacts(!this.showContacts)`
- `+` and `-` should call `setSolverIterations(...)`
- `*` and `/` should call `setSubSteps(...)`
- number keys should call `loadDemo(index)`

This ensures the header always stays in sync with keyboard changes.

### 8. Prevent toolbar interaction from triggering game input

This should be handled in `demo/input/InputManager.ts`, not scattered across the app.

Add a simple helper to detect whether an event target is inside the toolbar, for example by checking:

- `event.target instanceof HTMLElement`
- `event.target.closest('#demo-toolbar')`

If the event came from inside the toolbar:

- do not enqueue keyboard events
- do not enqueue mouse click events
- do not enqueue mouse move events
- do not enqueue wheel events

This prevents:

- typing in inputs from triggering shortcuts
- scrolling number inputs from zooming the camera
- header clicks from acting like world clicks

This is the cleanest minimal solution.

### 9. Decide how to coexist with the in-canvas text HUD

`Application.render()` currently draws textual instructions and debug info directly on the canvas.

For a simple implementation, keep it.

Possible small adjustment:

- move the text start position slightly lower so it does not sit visually under the new header

Do not try to redesign the whole HUD in the same change.

### 10. Keep demo options generated from `Demo.demoStrings`

The dropdown should be populated from `Demo.demoStrings` so there is no second source of demo labels.

The option value should be the numeric demo index.

That keeps the selector aligned with the existing `demoFunctions` array.

### 11. Manual verification checklist

After implementation, verify the following:

- Changing the demo from the selector loads the correct demo
- Keyboard number shortcuts still switch demos
- Toggling debug options in the header updates the scene immediately
- Keyboard shortcuts also update the header controls immediately
- Solver iterations and substeps cannot go below `1`
- Pausing and stepping still work correctly
- Clicking or scrolling inside the toolbar does not spawn objects, pan the camera, or zoom the world
- The canvas still resizes correctly

## Suggested File Changes

Smallest clean set of files:

- `index.html`
- `styles.css`
- `demo/Application.ts`
- `demo/input/InputManager.ts`

No other files should be required for a first pass.

## Notes on Simplicity

To keep the implementation simple:

- use plain DOM APIs
- avoid introducing new abstractions unless they remove clear duplication
- keep the toolbar state driven by `Application`
- do not move rendering or physics logic around
- do not try to turn the demo into a full UI app

## Recommended First Version

If implementing this incrementally, the cleanest first version would be:

1. Add the toolbar container and CSS
2. Extract `loadDemo()` and setting setters
3. Build the demo selector and a few checkboxes
4. Add number inputs for solver iterations and substeps
5. Filter toolbar-originated input in `InputManager`
6. Do a manual verification pass

That gives the feature you want without overcomplicating the demo codebase.
