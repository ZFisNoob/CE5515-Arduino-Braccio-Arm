# Braccio Control Dashboard (`CE5515_Braccio_Final_UI.py`)

A Tkinter GUI for the Arduino Braccio robotic arm. It keeps the original serial
protocol intact and adds a live 2D simulated arm preview, a simulation-only
mode, a deliberate hardware-send gate, recording/playback with progress, and
CSV save/load — all of which work with or without the physical arm connected.

The serial command format sent to the Arduino is unchanged:

```
base,shoulder,elbow,wristV,wristR,gripper,speed\n
```

(7 integers, comma-separated, newline-terminated, at 115200 baud.)

---

## 1. Requirements

* **Python 3.8 or newer** (uses `tkinter`, which ships with standard Python).
* **`pyserial`** — only needed to talk to the real arm. The GUI still opens and
  runs in simulation mode without it.

Tkinter is included in the standard python.org installers for Windows and
macOS. On some Linux distributions it is a separate package
(`sudo apt install python3-tk`).

Install the one external dependency:

```bash
pip install pyserial
```

> Pillow is **not** required. The earlier static preview image has been
> replaced by a live canvas simulator, so there are no image dependencies.

---

## 2. Running the program

```bash
python CE5515_Braccio_Final_UI.py
```

If `pyserial` is missing, the program prints a note in the console panel and
opens in simulation-only mode (you can still move sliders, record steps, and
preview playback — nothing is sent to hardware).

---

## 3. Connecting to the Braccio

1. Plug in the Braccio / Arduino over USB.
2. Click **Refresh** next to the **Port** dropdown and pick the COM port
   (e.g. `COM5` on Windows, `/dev/ttyACM0` or `/dev/ttyUSB0` on Linux,
   `/dev/cu.usbmodem*` on macOS).
3. Click **Connect**. The status line turns green.
4. If **Auto-home on connect** is ticked (default), the arm is sent the home
   pose `90,90,90,90,90,40,20` immediately so it starts from a known position.
5. Use **Home Now** at any time to recenter the sliders/simulator (and the arm,
   if hardware send is enabled).

The connection uses **115200 baud**, matching the professor's existing sketch.

---

## 4. Simulation mode vs. Hardware-send mode

These two ideas are deliberately separated so the GUI is safe to use without the
arm, and so the real arm only moves when you mean it to.

**Live 2D simulator (always on).**
The canvas always reflects the *commanded* pose from the sliders, the selected
recorded step, or the current playback step. It updates whether or not you are
connected, so the interface can be developed and demonstrated with no hardware
present.

**Drag to scrub, release to move (preview behavior).**
When you press and drag a joint slider, the canvas shows a **dashed "ghost" arm**
at the target you are scrubbing toward, while the **solid arm stays put** — so you
can judge how far the arm will travel before committing. When you **release the
mouse**, the solid arm animates to the new pose, and the speed of that animation
scales with the **Move speed** value (higher speed → faster travel). Nudging with
the **+/–** buttons, loading a step, or homing commits immediately (no release
needed). This affects only the on-screen preview; **hardware is still sent on
release / via _Send Current Pose_ exactly as before** — the serial behavior is
unchanged.

**Reading base and wrist rotation.**
Base yaw (M1) is drawn as a **turntable**: the arm's mounting point swings across
the disc and an arrow on the disc points in the current base direction, with the
exact angle labelled below. Wrist rotation (M4/M5) is drawn as a small **roll
dial** on the wrist whose hand follows the roll angle, and the gripper jaws rotate
in-plane with it. The gripper's **open/closed bar** under the canvas shows how far
the jaws are closed independently of rotation.

**Hover a joint to inspect it.**
Moving the mouse over any joint — or the base turntable — pops a small tooltip
with that joint's exact commanded angle and its offset from home (the wrist shows
both up/down and rotate; the gripper shows its servo value and % closed). It is a
read-only inspector and does not change anything.

**Smooth, speed-scaled motion.**
After you release a slider (or nudge, load a step, or home), the solid arm makes
one coordinated, eased move to the new pose — all joints start and finish together
and decelerate into place — at a velocity that scales with the *Move speed* value.
This is a feel/approximation of the move, not calibrated hardware timing.

**Hardware send enabled (checkbox).**
This is the gate that decides whether commands actually reach the Arduino.

* **Ticked + connected → "LIVE READY".** Sending a pose moves the real arm.
* **Unticked → simulation only.** Sliders, recording, and playback all work and
  the preview animates, but **nothing is written to the serial port.** Use this
  whenever you do not have the arm, or want to rehearse a sequence safely.

**Send Current Pose (button).**
The deliberate, one-shot way to push the sliders' current pose to the arm. By
default the sliders do *not* stream to hardware while you drag.

**Send while dragging (optional, throttled).**
If you tick this, slider drags stream to the arm, but writes are throttled
(about every 90 ms) so the Arduino is never flooded. It still only sends when
*Hardware send enabled* is on and a port is connected.

**During playback**, manual controls are locked to prevent conflicting commands;
only **Stop** stays active. The status badge shows the current system state:
`SIMULATION ONLY`, `SIM READY - DISCONNECTED`, `SIM ONLY - CONNECTED`,
`LIVE READY`, or `PLAYBACK RUNNING`.

---

## 5. Recording, playback, and the CSV format

Recorded steps appear in a sortable **table** (columns: #, Base, Shld, Elbw,
WrV, WrR, Grip, Spd) so a sequence can be scanned at a glance. The panel header
shows the current step count.

* **Add Step (Record)** captures the current pose to the table.
* **Click a step** (or arrow-key to it) to load/preview it into the sliders. This
  does **not** send to hardware — it only sets the sliders and the simulator.
  *Reordering or duplicating does not change the live pose; only a deliberate
  click/arrow does.*
* **Move Up / Move Down** (or **Ctrl+↑ / Ctrl+↓**) reorder the selected step;
  **Duplicate** inserts a copy after it; **Delete** (or the **Delete** key)
  removes it.
* **Play Loop** runs through the recorded steps repeatedly, highlighting the
  current step as a playhead. Progress is shown as `Playing step 3/12: ...`.
  Stop with **Stop**.
* Playback wait time per step is computed dynamically:
  `wait = max(base delay, largest joint movement × ms-per-degree) + settle`.
* If *Hardware send enabled* is off, playback is a pure simulation (no serial
  output); if on and connected, each step is transmitted once.

**Diagnostics console.** Log lines are colour-coded by level — hardware sends
(`TX`) in green, simulation-only lines muted, notices/skips in amber, and errors
in red — so the log can be scanned quickly. It trims to the last 500 lines and
has a **Clear console** button.

**CSV format** — one pose per line, 7 integers in joint order:

```
base,shoulder,elbow,wristV,wristR,gripper,speed
90,90,90,90,90,40,20
45,120,90,90,90,73,30
```

* **Save Steps (CSV)** / **Load Steps (CSV)** use this exact format, which is
  the same line the program sends over serial.
* **Paste & Add Steps** accepts the same format pasted in directly. Malformed
  lines are skipped and reported in the console; valid values are clamped to the
  servo limits below.

**Servo limits (clamped on input and output):**

| Joint   | Range    | Home |
|---------|----------|------|
| base    | 0–180    | 90   |
| shoulder| 15–165   | 90   |
| elbow   | 0–180    | 90   |
| wristV  | 0–180    | 90   |
| wristR  | 0–180    | 90   |
| gripper | 10–73    | 40   |
| speed   | 10–30    | 20   |

Gripper: lower value = more open, higher = more closed (10 ≈ open, 73 ≈ closed).
The sliders are relative to the home pose (negative ↔ 0 ↔ positive), except
speed, which is absolute.

---

## 6. Limitations (please read)

* **The simulator shows the *commanded* pose, not verified physical feedback.**
  It draws where the arm has been *told* to go, based on the values you set.
  The standard Braccio does not report joint positions back to the PC, so the
  preview cannot confirm the arm actually reached that pose (e.g. if a servo
  stalls or is obstructed, the drawing will not know). Treat it as an intent/
  teaching view, not ground truth.
* The 2D drawing is a lightweight schematic, **not** a kinematically exact or
  scaled model of the real arm. It is intended to make slider/step/playback
  motion easy to read, not to measure reach or check for collisions.
* **The preview animation speed is a visual approximation, not calibrated
  hardware timing.** When the solid arm travels to a pose after you release a
  slider, its on-screen velocity scales with the *Move speed* value to give a
  feel for fast vs. slow moves; it is not a prediction of how long the real
  servos take. Actual hardware timing is governed by the dynamic-wait formula in
  the Dynamic Timing panel.
* There is no return/acknowledgement parsing from the Arduino; the program only
  writes commands. Timing between steps is governed by the dynamic-wait formula,
  not by hardware confirmation.
* Available COM ports depend on your OS and drivers; if the port does not
  appear, check the USB cable, drivers, and that no other program (e.g. the
  Arduino Serial Monitor) is holding the port open.

---

## 7. What changed from the previous revision (for reference)

The core program and serial protocol are preserved. The notable correctness fix
carried over from the prior revision: playback runs on a background thread, and
that thread no longer calls Tkinter directly (which is not thread-safe and could
intermittently raise `RuntimeError: main thread is not in main loop`, leaving the
UI locked). Playback now posts plain-data updates to the main thread through a
queue — the same pattern already used for console logging — so playback and
window-close are robust. The gripper slider's range was also restored so it can
reach the fully closed value (73).

This revision adds a UI/UX refinement pass on top of that:

* **Drag-to-scrub / commit-on-release preview.** Dragging a slider now moves only
  the dashed target ghost; the solid arm travels to the new pose on release, at a
  velocity that scales with the selected speed. (Hardware send timing is
  unchanged — still on release / via *Send Current Pose*.)
* **Clearer base & wrist rotation.** The base is shown as a rotating turntable
  with a direction arrow; wrist roll is shown as a roll dial on the wrist with the
  jaws turning in-plane, plus an open/closed gripper bar.
* **Fixed a minor layout glitch** where dragging the *Gripper open/close* slider
  could nudge the whole control column sideways. The preview column is now a fixed
  width and the readout uses fixed-width fields, so nothing reflows while dragging.
* **Visual polish:** a uniform warm-neutral theme, flat sliders with higher-
  contrast handles, an accent for primary actions, a dark diagnostics console, and
  tidier spacing and typography throughout.

A further round of usability work added:

* **Joint inspect tooltips** — hover any joint or the base turntable to read its
  exact angle and offset from home.
* **Eased, coordinated motion** — the solid arm now eases in and out of a move
  (all joints together) rather than crawling at constant speed.
* **A proper step table** — recorded steps are shown in a scannable, columnar
  table with a live count, zebra rows, a playback playhead, an empty-state hint,
  and **Move Up / Move Down / Duplicate / Delete** controls (plus **Ctrl+↑/↓** and
  **Delete** shortcuts). Loading a step into the sliders happens only on a genuine
  click or arrow-key, so reordering never disturbs the live pose.
* **Colour-coded console** — log lines are tinted by level (TX/sim/notice/error),
  trimmed to the last 500 lines, with a **Clear console** button.
* **Refined arm rendering** — the simulated arm was restyled to read like a real
  robot: links are shaded like lit cylinders with a sheen, joints look like servo
  hubs, the gripper is a machined two-jaw hand on a wrist plate, and the base is a
  3D rotary cylinder with a soft ground shadow and a pointer that swings with the
  yaw. This is purely cosmetic — kinematics, the hover tooltips, and all behaviour
  are unchanged.

No changes were made to the command format, baud rate, servo limits, home pose,
or CSV format.


## 8. Interface themes and recent UI refinements

The dashboard includes a **Switch to dark mode / Switch to light mode** control in
the header. The selection is saved in `~/.braccio_dashboard_theme` on the current
user account and restored on the next launch. The theme updates the surrounding
Tk/ttk interface, slider tracks, numeric controls, simulator canvas, trajectory
grid, step table, and paste area. The diagnostics console intentionally remains
dark in both themes so transmit, simulation, warning, and error messages retain
consistent contrast.

The simulator remains a lightweight **2D** view. No 3D rendering dependency is
required. The preview was narrowed slightly to protect the complete joint-control
strip, while the left column now reserves width for the minus/plus nudges,
sliders, value chips, and absolute-angle entries. On narrower displays, the
recorded-step table can scroll horizontally; its horizontal scrollbar is hidden
when there are no recorded steps.

Numeric-angle fields are no longer overwritten while a user is actively typing.
Playback's pause button reflects its current action (Pause or Resume). These
interface changes do not alter the seven-integer newline-terminated serial
frame, the 115200 baud rate, the servo limits, or the home pose.

### Theme and layout verification

Run the GUI regression tests in a graphical session or under Xvfb:

```bash
python -m unittest -v test_braccio_dashboard.py
```

The tests cover serial framing, servo limits, CSV parsing, recording-table
operations, light/dark theme switching and persistence, focused numeric entry,
trajectory drawing, Play Once simulation playback, side-by-side arm controls,
and slider/numeric-input layout bounds. The screenshots in the refinement ZIP
show the light and dark themes. Native Windows DPI behavior and physical Arduino
motion should still be checked on the target computer and hardware.
