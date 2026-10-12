# CE5515 Braccio Dashboard - UI Refinement

## Included
- Persistent light/dark mode toggle with coordinated styling for ttk widgets, classic Tk controls, the simulator canvas, slider tracks, entries, the step table, and CSV paste area.
- Refined surface hierarchy, keyboard focus treatment, clearer table row styling, and theme-appropriate semantic connection/playback status badges.
- More reliable numeric-angle entry: slider updates no longer replace text while an angle field is actively focused.
- Pause button clearly switches between **Pause playback** and **Resume playback**.
- Kept the 2D arm simulator and its trajectory overlay; no 3D engine or extra runtime dependency added.
- Slightly narrower preview and greater guaranteed width for slider, +/- nudge, value-chip, and numeric-entry controls.
- Recorded-step horizontal scrollbar appears when needed and is hidden along with the vertical scrollbar when the table is empty.
- Updated README plus persistent regression tests.

## Core compatibility preserved
- Serial frame remains `base,shoulder,elbow,wristV,wristR,gripper,speed\n` (seven integers, newline terminated) at 115200 baud.
- `LIMITS`, `HOME_POSE`, relative joint slider semantics, gripper range (-30, +33), low=open/high=closed, simulation mode, record/playback, CSV import/export, and optional pyserial behavior are unchanged.

## Local verification
- `python -m py_compile CE5515_Braccio_Final_UI.py test_braccio_dashboard.py`: passed.
- `xvfb-run ... python -m unittest -v test_braccio_dashboard.py`: 15 tests passed.
- Light and dark UI screenshots captured and visually inspected at 1650x1000.
- Layout probe confirmed the arm panel and all plus/numeric controls fit at 1380x860 in the tested Xvfb environment.
- Native Windows scaling and physical hardware remain unverified.
