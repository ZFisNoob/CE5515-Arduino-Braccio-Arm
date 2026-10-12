"""
Braccio GUI (Tkinter) - Refined Final UI

Core serial protocol kept unchanged:
  base,shoulder,elbow,wristV,wristR,gripper,speed\n
Main refinements:
- Live 2D simulated arm preview driven by slider/step/playback state
- Simulation-only mode for testing without the physical Braccio arm
- Hardware-send gate to prevent accidental robot movement
- Optional throttled live send while dragging
- Click recorded step to preview/load it into sliders
- Playback progress and current pose readout
- UI locking during playback to avoid conflicting commands
- Original record, save/load CSV, paste CSV, connect, home, and dynamic wait retained
"""

from __future__ import annotations

import math
import os
import time
import threading
import queue
from dataclasses import dataclass
from typing import Dict, List, Optional
from pathlib import Path

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError:
    serial = None  # type: ignore
    list_ports = None  # type: ignore


# ----------------------------
# CONFIG
# ----------------------------
ARM_PREVIEW_WIDTH = 400
ARM_PREVIEW_HEIGHT = 340
PREVIEW_FRAME_MS = 24              # ~40 FPS for a smoother, more refined animation.
DEFAULT_SEND_INTERVAL_MS = 90      # Throttle for optional live send while dragging.
CONSOLE_MAX_LINES = 500            # Trim the diagnostics console to stay responsive.

# The committed (solid) arm animates at a constant angular velocity that scales
# with the chosen "speed" value, so a higher speed visibly moves the arm faster.
# These bounds map speed 10..30 onto degrees-per-second. It is a readable visual
# approximation of feel, not a calibrated model of the Braccio's real timing.
SIM_DEG_PER_SEC_AT_MIN_SPEED = 55.0
SIM_DEG_PER_SEC_AT_MAX_SPEED = 280.0

# ---- Refined palette (single warm accent + neutral chrome + semantic states) ----
COLORS = {
    "bg": "#f4f5f7", "panel": "#ffffff", "panel_alt": "#fafbfc",
    "border": "#dfe3e8", "text": "#1f2933", "muted": "#6b7280",
    "faint": "#9aa3af", "accent": "#e8732a", "accent_dark": "#c75d1c",
    "accent_soft": "#fbe7d6", "track": "#e6e8eb", "canvas_bg": "#fcfcfd",
    "floor": "#eceef1", "grid": "#eef0f3",
}
LIGHT_COLORS = dict(COLORS)
DARK_COLORS = {
    "bg": "#171b22", "panel": "#202630", "panel_alt": "#262d38",
    "border": "#394250", "text": "#e6ebf2", "muted": "#a0acbb",
    "faint": "#758295", "accent": "#f08a45", "accent_dark": "#e07832",
    "accent_soft": "#493021", "track": "#343d49", "canvas_bg": "#1b222b",
    "floor": "#252d38", "grid": "#2b3440",
}
THEME_PALETTES = {"light": LIGHT_COLORS, "dark": DARK_COLORS}
THEME_PREFERENCE_PATH = os.path.join(os.path.expanduser("~"), ".braccio_dashboard_theme")

# Per-link colors for the simulated arm (base->tip), tuned to read as one limb.
ARM_LINK_COLORS = ("#d9641f", "#e8732a", "#f2913f")

# Refined "robot skin" palette. Links are drawn as cylinders via concentric
# strokes (dark rim -> light core) with a sheen; joints look like servos; the
# gripper and base read as machined metal / a rotary base.
ARM_SKIN = {
    "link_outline": "#9c460f",
    "link_dark":    "#c45a18",
    "link_mid":     "#e8732a",
    "link_light":   "#f6a45f",
    "link_sheen":   "#ffd9b3",
    "joint_ring":   "#9c460f",
    "joint_face":   "#f4f6f8",
    "joint_hub":    "#d9641f",
    "joint_hub_hi": "#ffe8d4",
    "grip_dark":    "#4d535c",
    "grip_mid":     "#767e8a",
    "grip_light":   "#a7afba",
    "base_shadow":  "#d7dade",
    "base_bottom":  "#d2b88f",
    "base_side":    "#e0c79f",
    "base_side_dk": "#cdb084",
    "base_top":     "#f1dcbf",
    "base_rim_dk":  "#c9ad82",
    "base_rim_hi":  "#fbeed7",
}

JOINT_COLORS = {
    "gripper": "#7a8088",
    "wristR": "#f2913f",
    "wristV": "#e8732a",
    "elbow": "#e06a22",
    "shoulder": "#d9641f",
    "base": "#c75d1c",
    "speed": "#2f6fd0",
}

POSE_FIELDS = ("base", "shoulder", "elbow", "wristV", "wristR", "gripper", "speed")
ARM_JOINTS = ("base", "shoulder", "elbow", "wristV", "wristR", "gripper")


# ----------------------------
# Data model
# ----------------------------
@dataclass
class Pose:
    base: int
    shoulder: int
    elbow: int
    wristV: int
    wristR: int
    gripper: int
    speed: int

    def to_line(self) -> str:
        return f"{self.base},{self.shoulder},{self.elbow},{self.wristV},{self.wristR},{self.gripper},{self.speed}\n"

    def compact(self) -> str:
        return (
            f"B{self.base:03d}  SH{self.shoulder:03d}  EL{self.elbow:03d}  "
            f"WV{self.wristV:03d}  WR{self.wristR:03d}  G{self.gripper:03d}  S{self.speed:02d}"
        )

    def __str__(self) -> str:
        return (
            f"B{self.base} SH{self.shoulder} EL{self.elbow} "
            f"WV{self.wristV} WR{self.wristR} G{self.gripper} | Sp{self.speed}"
        )


LIMITS = {
    "base": (0, 180),
    "shoulder": (15, 165),
    "elbow": (0, 180),
    "wristV": (0, 180),
    "wristR": (0, 180),
    "gripper": (10, 73),
    "speed": (10, 30),
}

HOME_POSE = Pose(base=90, shoulder=90, elbow=90, wristV=90, wristR=90, gripper=40, speed=20)


def _build_ui_limits() -> dict[str, tuple[int, int]]:
    out: dict[str, tuple[int, int]] = {}
    for joint in ARM_JOINTS:
        lo, hi = LIMITS[joint]
        home = int(getattr(HOME_POSE, joint))
        out[joint] = (lo - home, hi - home)
    # Gripper UI is centered near zero for intuitive control, but the positive
    # end is kept at +33 so the slider can still reach the true closed value
    # (home 40 + 33 = 73), preserving the original hardware range.
    out["gripper"] = (-30, 33)
    out["speed"] = LIMITS["speed"]
    return out


UI_LIMITS = _build_ui_limits()


def clamp(name: str, v: int) -> int:
    lo, hi = LIMITS[name]
    return lo if v < lo else hi if v > hi else v


def pose_to_values(p: Pose) -> Dict[str, float]:
    return {field: float(getattr(p, field)) for field in POSE_FIELDS}


def values_to_pose(values: Dict[str, float]) -> Pose:
    return Pose(
        base=clamp("base", int(round(values["base"]))),
        shoulder=clamp("shoulder", int(round(values["shoulder"]))),
        elbow=clamp("elbow", int(round(values["elbow"]))),
        wristV=clamp("wristV", int(round(values["wristV"]))),
        wristR=clamp("wristR", int(round(values["wristR"]))),
        gripper=clamp("gripper", int(round(values["gripper"]))),
        speed=clamp("speed", int(round(values["speed"]))),
    )


def list_serial_ports() -> List[str]:
    if list_ports is None:
        return []
    ports: List[str] = []
    try:
        for p in list_ports.comports():
            if p.device:
                ports.append(p.device)
    except Exception:
        pass
    return sorted(set(ports), key=lambda s: (len(s), s))


# ----------------------------
# Serial manager
# ----------------------------
class SerialManager:
    def __init__(self, log_fn):
        self.log = log_fn
        self.ser: Optional["serial.Serial"] = None
        self.lock = threading.Lock()

    def is_available(self) -> bool:
        return serial is not None

    def is_connected(self) -> bool:
        return self.ser is not None and getattr(self.ser, "is_open", False)

    def connect(self, port: str, baud: int = 115200) -> bool:
        if serial is None:
            self.log("ERR connect: pyserial is not installed. Run: pip install pyserial")
            return False

        with self.lock:
            if self.is_connected():
                return True
            try:
                self.ser = serial.Serial(  # type: ignore[union-attr]
                    port=port,
                    baudrate=baud,
                    timeout=0.1,
                    write_timeout=1.0,
                )
            except Exception as e:
                self.ser = None
                self.log(f"ERR connect: {e}")
                return False
        self.log(f"Connected {port} @ {baud}")
        return True

    def disconnect(self):
        with self.lock:
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.log("Disconnected")

    def send_pose(self, p: Pose) -> bool:
        line = p.to_line().encode("ascii", errors="ignore")
        with self.lock:
            if not self.is_connected():
                self.log("ERR: Not connected")
                return False
            try:
                self.ser.write(line)  # type: ignore[union-attr]
                self.ser.flush()      # type: ignore[union-attr]
                return True
            except Exception as e:
                self.log(f"ERR send: {e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                return False


# ----------------------------
# App
# ----------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.theme_name = self._read_theme_preference()
        COLORS.clear()
        COLORS.update(THEME_PALETTES[self.theme_name])
        self._value_labels: List[tk.Label] = []
        self._angle_entries: List[ttk.Entry] = []
        self._plus_buttons: List[ttk.Button] = []
        self.title("Braccio Control Dashboard")
        self.geometry("1380x860")
        self.minsize(1120, 720)
        try:
            self.state("zoomed")
        except tk.TclError:
            # Some platforms do not support the Windows-style zoomed state.
            pass

        self.log_q = queue.Queue()
        # Playback runs on a background thread, which must never call Tkinter
        # directly (Tk is not thread-safe). The worker posts plain-data UI
        # update messages here, and the main thread drains them in
        # _poll_ui_queue. This mirrors the existing log_q/_poll_log pattern.
        self.ui_q: "queue.Queue" = queue.Queue()
        self._closing = False
        self.serial_mgr = SerialManager(self._log)

        self.recorded: List[Pose] = []
        self.play_thread: Optional[threading.Thread] = None
        self.play_stop = threading.Event()
        self.play_resume = threading.Event()
        self.play_once = False
        self.trajectory_var = tk.BooleanVar(value=False)
        self.duration_var = tk.StringVar(value="Estimated cycle: 0.0 s")

        self.vars: dict[str, tk.IntVar] = {}
        self.scales: dict[str, tk.Scale] = {}
        self._scale_guard: dict[str, bool] = {}
        self.lock_during_playback_widgets: List[tk.Widget] = []

        # Core pose states.
        #   target_pose  : current slider pose (what is sent/recorded + the readout)
        #   commit_pose  : pose the SOLID simulated arm animates toward
        #   sim_values   : the animated solid arm position (chases commit_pose)
        # While a slider handle is being dragged we update target_pose live (so the
        # ghost/target preview tracks the drag) but hold commit_pose, so the solid
        # arm only travels to the new pose once the mouse is released.
        self.target_pose: Pose = HOME_POSE
        self.commit_pose: Pose = HOME_POSE
        self.sim_values: Dict[str, float] = pose_to_values(HOME_POSE)
        self.last_commanded: Pose = HOME_POSE
        self._active_drag: Optional[str] = None   # name of joint whose handle is held
        self._last_anim_time: float = time.time()

        # Eased coordinated motion: when commit_pose changes we snapshot where the
        # solid arm starts and drive a single progress value 0->1 with smoothstep,
        # so every joint starts and finishes together and decelerates into place
        # (much closer to how the real arm executes a coordinated move).
        self._anim_start: Dict[str, float] = pose_to_values(HOME_POSE)
        self._anim_progress: float = 1.0

        # Canvas hover state (joint inspect tooltips).
        self._solid_joints: List[dict] = []   # hit targets from the last solid draw
        self._base_hit: Optional[tuple] = None
        self._hover_joint: Optional[str] = None

        # A short-lived guard so a keyboard reorder (Ctrl+Up/Down) does not also
        # trigger a pose-load when the Up/Down key is released afterwards.
        self._block_key_load = False

        # Throttled manual sending state.
        self._last_send_time = 0.0
        self._pending_send_after_id: Optional[str] = None
        self._pending_send_pose: Optional[Pose] = None
        self._pending_send_reason = ""

        # UI state variables.
        self.hardware_send_var = tk.BooleanVar(value=True)
        self.live_drag_var = tk.BooleanVar(value=False)
        self.auto_home_var = tk.BooleanVar(value=True)
        self.step_size_var = tk.IntVar(value=5)
        self.base_delay_var = tk.IntVar(value=500)
        self.settle_var = tk.IntVar(value=200)
        self.ms_per_deg_var = tk.IntVar(value=18)

        self.status_var = tk.StringVar(value="Disconnected")
        self.system_state_var = tk.StringVar(value="Simulator Ready")
        self.pose_summary_var = tk.StringVar(value="")
        self.gripper_state_var = tk.StringVar(value="")
        self.playback_status_var = tk.StringVar(value="Idle")

        self._build_ui()
        self._apply_classic_widget_theme()
        self.theme_toggle_btn.configure(text=self._theme_button_text())
        self._refresh_ports()
        self._sync_target_from_sliders()
        self._draw_live_preview()
        self._update_system_state()

        self.after(PREVIEW_FRAME_MS, self._animate_preview)
        self.after(60, self._poll_log)
        self.after(60, self._poll_ui_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- logging ----------
    @staticmethod
    def _classify_log(msg: str) -> str:
        low = msg.lower()
        if msg.startswith("ERR") or "failure" in low or "could not" in low \
                or "error" in low:
            return "err"
        if "skip send" in low or "ignored" in low or "not connected" in low \
                or "not installed" in low or "disabled" in low or "stopping" in low:
            return "warn"
        if msg.startswith("TX") or " tx " in f" {low} " or low.startswith("tx loop"):
            return "tx"
        if msg.startswith("SIM") or low.startswith("sim loop") or "sim only" in low:
            return "sim"
        return "info"

    def _log(self, msg: str, level: Optional[str] = None):
        ts = time.strftime("%H:%M:%S")
        if level is None:
            level = self._classify_log(msg)
        self.log_q.put((f"[{ts}] {msg}", level))

    def _poll_log(self):
        try:
            while True:
                try:
                    item = self.log_q.get_nowait()
                except queue.Empty:
                    break
                if isinstance(item, tuple):
                    s, level = item
                else:
                    s, level = item, "info"
                self.console.configure(state="normal")
                self.console.insert("end", s + "\n", (level,))
                # Keep the console bounded so long sessions stay responsive.
                line_count = int(self.console.index("end-1c").split(".")[0])
                if line_count > CONSOLE_MAX_LINES:
                    self.console.delete("1.0", f"{line_count - CONSOLE_MAX_LINES}.0")
                self.console.see("end")
                self.console.configure(state="disabled")
        finally:
            if not self._closing:
                self.after(60, self._poll_log)

    def _clear_console(self):
        self.console.configure(state="normal")
        self.console.delete("1.0", "end")
        self.console.configure(state="disabled")

    def _poll_ui_queue(self):
        """Drain UI update messages posted by the playback worker thread.

        Runs on the main thread, so it is the only place playback is allowed to
        touch widgets. Messages are simple tuples of plain data:
            ("step", pose, index, total)  -> show/preview the step being played
            ("done", message)             -> playback finished; unlock the UI
        """
        try:
            while True:
                try:
                    msg = self.ui_q.get_nowait()
                except queue.Empty:
                    break
                kind = msg[0]
                if kind == "step":
                    _, target, index, total = msg
                    self._apply_playback_target(target, index, total)
                elif kind == "done":
                    _, message = msg
                    self._on_playback_finished_ui(message)
        finally:
            if not self._closing:
                self.after(60, self._poll_ui_queue)

    # ---------- Theme and UI ----------
    @staticmethod
    def _read_theme_preference() -> str:
        try:
            value = Path(THEME_PREFERENCE_PATH).read_text(encoding="utf-8").strip().lower()
            return value if value in THEME_PALETTES else "light"
        except (OSError, ValueError):
            return "light"

    def _write_theme_preference(self):
        try:
            Path(THEME_PREFERENCE_PATH).write_text(self.theme_name + "\n", encoding="utf-8")
        except OSError as exc:
            self._log(f"WARN: Could not save theme preference: {exc}")

    def _theme_button_text(self) -> str:
        return "Switch to dark mode" if getattr(self, "theme_name", "light") == "light" else "Switch to light mode"

    def _toggle_theme(self):
        self.theme_name = "dark" if self.theme_name == "light" else "light"
        COLORS.clear()
        COLORS.update(THEME_PALETTES[self.theme_name])
        self._init_style()
        self._apply_classic_widget_theme()
        self.theme_toggle_btn.configure(text=self._theme_button_text())
        self._write_theme_preference()
        self._draw_live_preview()
        self._draw_gripper_bar_from_pose()
        self._update_system_state()
        self._log(f"Theme changed to {self.theme_name} mode.")

    def _draw_gripper_bar_from_pose(self):
        p = self.target_pose
        span = LIMITS["gripper"][1] - LIMITS["gripper"][0]
        pct = int(round((p.gripper - LIMITS["gripper"][0]) / span * 100))
        pct = max(0, min(100, pct))
        label = "open" if pct <= 25 else "closed" if pct >= 75 else "mid"
        self._draw_gripper_bar(pct, label)

    def _apply_classic_widget_theme(self):
        c = COLORS
        self.configure(bg=c["bg"])
        self.controls_canvas.configure(bg=c["bg"])
        self.preview_canvas.configure(bg=c["canvas_bg"], highlightbackground=c["border"])
        self.gripper_bar.configure(bg=c["panel"])
        self.paste_text.configure(bg=c["panel"], fg=c["text"], insertbackground=c["text"],
                                  highlightbackground=c["border"])
        self.console.configure(bg="#151a21", fg="#d7dee8", insertbackground="#d7dee8")
        for label in getattr(self, "_value_labels", []):
            label.configure(bg=c["panel"], fg=c["text"])
        for name, scale in self.scales.items():
            scale.configure(bg=c["panel"], troughcolor=c["track"],
                            activebackground=JOINT_COLORS.get(name, c["accent"]),
                            highlightbackground=c["border"])
        for rb in getattr(self, "_step_radios", []):
            rb.configure(bg=c["panel"], fg=c["muted"], selectcolor=c["accent_soft"],
                         activebackground=c["accent_soft"], activeforeground=c["text"],
                         highlightbackground=c["border"])
        for w in self._iter_widgets(self):
            if isinstance(w, tk.Canvas) and w not in (self.preview_canvas, self.gripper_bar, self.controls_canvas):
                w.configure(bg=c["panel"])
        self.steps_tree.tag_configure("odd", background=c["panel"])
        self.steps_tree.tag_configure("even", background=c["panel_alt"])

    @staticmethod
    def _iter_widgets(root):
        for child in root.winfo_children():
            yield child
            yield from App._iter_widgets(child)

    def _init_style(self):
        """Apply a refined, flat theme on top of ttk 'clam'."""
        style = ttk.Style(self)
        try:
            if "clam" in style.theme_names():
                style.theme_use("clam")
        except Exception:
            pass

        c = COLORS
        self.configure(bg=c["bg"])
        base_font = ("Segoe UI", 10)

        style.configure(".", background=c["bg"], foreground=c["text"], font=base_font)
        style.configure("TFrame", background=c["bg"])
        style.configure("Card.TFrame", background=c["panel"])
        style.configure("TLabel", background=c["bg"], foreground=c["text"])
        style.configure("Muted.TLabel", background=c["bg"], foreground=c["muted"])
        style.configure("CardMuted.TLabel", background=c["panel"], foreground=c["muted"])
        style.configure("Card.TLabel", background=c["panel"], foreground=c["text"])
        style.configure("Title.TLabel", background=c["bg"], foreground=c["text"],
                        font=("Segoe UI Semibold", 17))
        style.configure("Subtitle.TLabel", background=c["bg"], foreground=c["muted"],
                        font=("Segoe UI", 10))
        style.configure("Section.TLabel", background=c["bg"], foreground=c["accent_dark"],
                        font=("Segoe UI Semibold", 10))
        style.configure("Pose.TLabel", background=c["bg"], foreground=c["text"],
                        font=("Consolas", 11))

        # Bordered "card" frames over the uniform background.
        style.configure("Card.TLabelframe", background=c["panel"], bordercolor=c["border"],
                        relief="solid", borderwidth=1)
        style.configure("Card.TLabelframe.Label", background=c["panel"],
                        foreground=c["accent_dark"], font=("Segoe UI Semibold", 10))

        # Buttons: neutral default + an accent "primary" variant.
        style.configure("TButton", background=c["panel"], foreground=c["text"],
                        bordercolor=c["border"], focusthickness=2, focuscolor=c["accent"],
                        padding=(10, 6))
        style.map("TButton",
                  background=[("active", c["panel_alt"]), ("pressed", c["track"]),
                              ("disabled", c["panel_alt"])],
                  foreground=[("disabled", c["faint"])])
        style.configure("Primary.TButton", background=c["accent"], foreground="#ffffff",
                        bordercolor=c["accent_dark"], font=("Segoe UI Semibold", 10),
                        padding=(10, 7))
        style.map("Primary.TButton",
                  background=[("active", c["accent_dark"]), ("pressed", c["accent_dark"]),
                              ("disabled", "#f0d8c6")],
                  foreground=[("disabled", "#ffffff")])

        style.configure("TCheckbutton", background=c["bg"], foreground=c["text"])
        style.map("TCheckbutton", background=[("active", c["bg"])])
        style.configure("TCombobox", fieldbackground=c["panel"], background=c["panel"],
                        foreground=c["text"])
        style.configure("TEntry", fieldbackground=c["panel"], foreground=c["text"])
        style.configure("TSeparator", background=c["border"])

        # Recorded-steps table.
        style.configure("Steps.Treeview", background=c["panel"], fieldbackground=c["panel"],
                        foreground=c["text"], rowheight=27, borderwidth=0,
                        font=("Consolas", 9))
        style.configure("Steps.Treeview.Heading", background=c["panel_alt"], foreground=c["muted"],
                        font=("Segoe UI Semibold", 8), relief="flat")
        style.map("Steps.Treeview.Heading", background=[("active", c["track"])])
        style.map("Steps.Treeview",
                  background=[("selected", c["accent_soft"])],
                  foreground=[("selected", c["accent_dark"])])
        return style

    def _on_controls_content_resize(self, _event=None):
        self.controls_canvas.configure(scrollregion=self.controls_canvas.bbox("all"))

    def _on_controls_viewport_resize(self, event):
        # Keep the simulator and joint sliders side by side at every DPI.
        # When space is insufficient, allow horizontal scrolling rather than
        # stacking the controls or clipping the right-hand +/angle inputs.
        if not hasattr(self, "arm_controls_panel"):
            return
        required = self.arm_panel.winfo_reqwidth() + 24
        self.controls_canvas.itemconfigure(
            self._controls_window, width=max(event.width, required))
        self.controls_canvas.after_idle(self._on_controls_content_resize)

    def _bind_controls_wheel(self, _event=None):
        self.bind_all("<MouseWheel>", self._scroll_controls_wheel)
        self.bind_all("<Button-4>", self._scroll_controls_wheel)
        self.bind_all("<Button-5>", self._scroll_controls_wheel)

    def _unbind_controls_wheel(self, _event=None):
        self.unbind_all("<MouseWheel>")
        self.unbind_all("<Button-4>")
        self.unbind_all("<Button-5>")

    def _scroll_controls_wheel(self, event):
        direction = -1 if getattr(event, "num", None) == 4 or getattr(event, "delta", 0) > 0 else 1
        self.controls_canvas.yview_scroll(direction * 3, "units")

    def _build_ui(self):
        self._init_style()

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        content = ttk.Frame(self, padding=14)
        content.grid(row=0, column=0, sticky="nsew")
        content.columnconfigure(0, weight=1)
        content.rowconfigure(1, weight=1)

        # Header gives the operator the important state without reading the console.
        header = ttk.Frame(content)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        header.columnconfigure(1, weight=1)

        title_box = ttk.Frame(header)
        title_box.grid(row=0, column=0, sticky="w")
        ttk.Label(title_box, text="Braccio Control Dashboard", style="Title.TLabel").grid(
            row=0, column=0, sticky="w")
        ttk.Label(title_box, text="Manual control + live 2D simulation",
                  style="Subtitle.TLabel").grid(row=1, column=0, sticky="w")

        ttk.Label(header, textvariable=self.pose_summary_var, style="Pose.TLabel").grid(
            row=0, column=1, sticky="e", padx=(12, 12))
        self.system_badge = tk.Label(
            header,
            textvariable=self.system_state_var,
            font=("Segoe UI Semibold", 10),
            padx=14,
            pady=7,
            bg="#e5e7eb",
            fg="#111827",
            relief="flat",
            bd=0,
        )
        self.system_badge.grid(row=0, column=2, sticky="e", padx=(0, 8))
        self.theme_toggle_btn = ttk.Button(
            header, text=self._theme_button_text(), command=self._toggle_theme,
            style="TButton")
        self.theme_toggle_btn.grid(row=0, column=3, sticky="e")
        header.columnconfigure(1, weight=1)

        body = ttk.Frame(content)
        body.grid(row=1, column=0, sticky="nsew")
        # Reserve enough width for the complete arm-control strip, including
        # nudge buttons and numeric-angle inputs. The right diagnostics column
        # can narrow and scroll its step table rather than clipping arm controls.
        body.columnconfigure(0, weight=7, minsize=1000)
        body.columnconfigure(1, weight=3, minsize=320)
        body.rowconfigure(0, weight=1)

        # The control column may exceed the available screen height at 125-200% DPI.
        # Scroll only that column, preserving the fixed-width simulator geometry.
        left_host = ttk.Frame(body)
        left_host.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        left_host.columnconfigure(0, weight=1)
        left_host.rowconfigure(0, weight=1)
        self.controls_canvas = tk.Canvas(left_host, highlightthickness=0, bd=0,
                                         bg=COLORS["bg"])
        self.controls_canvas.grid(row=0, column=0, sticky="nsew")
        self.controls_scrollbar = ttk.Scrollbar(left_host, orient="vertical",
                                               command=self.controls_canvas.yview)
        self.controls_scrollbar.grid(row=0, column=1, sticky="ns")
        self.controls_canvas.configure(yscrollcommand=self.controls_scrollbar.set)
        self.controls_hscrollbar = ttk.Scrollbar(left_host, orient="horizontal",
                                                 command=self.controls_canvas.xview)
        self.controls_hscrollbar.grid(row=1, column=0, sticky="ew")
        self.controls_canvas.configure(xscrollcommand=self.controls_hscrollbar.set)
        left = ttk.Frame(self.controls_canvas)
        self._controls_window = self.controls_canvas.create_window((0, 0), window=left, anchor="nw")
        left.bind("<Configure>", self._on_controls_content_resize)
        self.controls_canvas.bind("<Configure>", self._on_controls_viewport_resize)
        self.controls_canvas.bind("<Enter>", self._bind_controls_wheel)
        self.controls_canvas.bind("<Leave>", self._unbind_controls_wheel)
        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky="nsew")

        left.columnconfigure(0, weight=1)
        right.columnconfigure(0, weight=1)
        right.rowconfigure(2, weight=1)

        # Arm control panel
        arm = ttk.LabelFrame(left, text="  Manual Arm Control + Live Simulation  ",
                             style="Card.TLabelframe", padding=12)
        arm.grid(row=0, column=0, sticky="nsew")
        self.arm_panel = arm
        # Column 0 (the preview) is pinned to a fixed width so that text whose
        # width changes during a drag (e.g. the gripper "open/closed" readout)
        # can never reflow the layout and shift the sliders sideways.
        PREVIEW_COL_W = ARM_PREVIEW_WIDTH + 16
        arm.columnconfigure(0, weight=0, minsize=PREVIEW_COL_W)
        arm.columnconfigure(1, weight=1)
        arm.rowconfigure(0, weight=1)

        preview = ttk.Frame(arm, style="Card.TFrame")
        preview.grid(row=0, column=0, sticky="nw", padx=(0, 16))
        self.preview_panel = preview
        preview.columnconfigure(0, weight=1)

        ttk.Label(preview, text="Live 2D arm simulator", style="Section.TLabel").grid(
            row=0, column=0, sticky="w")
        ttk.Label(
            preview,
            text="Commanded-pose preview. Drag a slider to scrub the target (dashed); "
                 "release to move the arm at the selected speed. Updates even when disconnected.",
            wraplength=ARM_PREVIEW_WIDTH,
            justify="left",
            style="CardMuted.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(2, 8))

        self.preview_canvas = tk.Canvas(
            preview,
            width=ARM_PREVIEW_WIDTH,
            height=ARM_PREVIEW_HEIGHT,
            bg=COLORS["canvas_bg"],
            highlightthickness=1,
            highlightbackground=COLORS["border"],
        )
        self.preview_canvas.grid(row=2, column=0, sticky="n")
        self.preview_canvas.bind("<Motion>", self._on_canvas_motion)
        self.preview_canvas.bind("<Leave>", self._on_canvas_leave)
        ttk.Checkbutton(preview, text="Show recorded trajectory", variable=self.trajectory_var, command=self._draw_live_preview).grid(row=6, column=0, sticky="w")

        # Fixed-width readout strip so its content never resizes the column.
        self.pose_readout_lbl = ttk.Label(
            preview, textvariable=self.gripper_state_var, style="Card.TLabel",
            font=("Consolas", 9), wraplength=ARM_PREVIEW_WIDTH, justify="left")
        self.pose_readout_lbl.grid(row=3, column=0, sticky="w", pady=(8, 0))

        self.gripper_bar = tk.Canvas(preview, width=ARM_PREVIEW_WIDTH, height=14,
                                     bg=COLORS["bg"], highlightthickness=0)
        self.gripper_bar.grid(row=4, column=0, sticky="w", pady=(4, 0))

        ttk.Label(
            preview,
            text="Note: this shows the commanded pose, not verified physical feedback.",
            wraplength=ARM_PREVIEW_WIDTH,
            justify="left",
            style="CardMuted.TLabel",
        ).grid(row=5, column=0, sticky="w", pady=(6, 0))

        arm_controls = ttk.Frame(arm, style="Card.TFrame")
        arm_controls.grid(row=0, column=1, sticky="nsew")
        self.arm_controls_panel = arm_controls
        arm_controls.columnconfigure(0, minsize=20)
        arm_controls.columnconfigure(1, minsize=128)
        arm_controls.columnconfigure(2, minsize=34)
        arm_controls.columnconfigure(3, weight=1, minsize=150)
        arm_controls.columnconfigure(4, minsize=34)
        arm_controls.columnconfigure(5, minsize=45)
        arm_controls.columnconfigure(6, minsize=50)

        ttk.Label(
            arm_controls,
            text="Drag a handle to scrub the target; release to commit. "
                 "Angles are relative to home:  – ◄  0  ► +.",
            wraplength=390,
            justify="left",
            style="CardMuted.TLabel",
        ).grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 10))

        ttk.Label(arm_controls, text="Nudge step", style="Card.TLabel").grid(
            row=1, column=0, columnspan=2, sticky="w")
        step_row = ttk.Frame(arm_controls, style="Card.TFrame")
        step_row.grid(row=1, column=2, columnspan=4, sticky="w", padx=(8, 0))
        self._step_radios = []
        for step in (1, 2, 5, 10, 15, 30):
            rb = tk.Radiobutton(
                step_row,
                text=str(step),
                value=step,
                variable=self.step_size_var,
                indicatoron=0,
                width=4,
                padx=2,
                pady=3,
                relief="flat",
                bd=1,
                bg=COLORS["bg"],
                fg=COLORS["muted"],
                selectcolor=COLORS["accent_soft"],
                activebackground=COLORS["accent_soft"],
                highlightthickness=1,
                highlightbackground=COLORS["border"],
            )
            rb.pack(side="left", padx=(0, 5))
            self._step_radios.append(rb)
            self.lock_during_playback_widgets.append(rb)

        # Sliders ordered to match the physical arm from top (gripper) to bottom (base).
        self._make_slider(arm_controls, 2, "gripper", "Gripper open/close", 0)
        self._make_slider(arm_controls, 4, "wristR", "Wrist rotate", 0)
        self._make_slider(arm_controls, 6, "wristV", "Wrist up/down", 0)
        self._make_slider(arm_controls, 8, "elbow", "Elbow", 0)
        self._make_slider(arm_controls, 10, "shoulder", "Shoulder", 0)
        self._make_slider(arm_controls, 12, "base", "Base rotation", 0)
        ttk.Separator(arm_controls, orient="horizontal").grid(row=14, column=0, columnspan=6, sticky="ew", pady=(10, 8))
        self._make_slider(arm_controls, 15, "speed", "Move speed", HOME_POSE.speed)

        # Connection
        conn = ttk.LabelFrame(left, text="  Connection + Safety  ", style="Card.TLabelframe", padding=12)
        conn.grid(row=1, column=0, sticky="ew", pady=(12, 0))
        conn.columnconfigure(1, weight=1)

        ttk.Label(conn, text="Port", style="Card.TLabel").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar(value="")
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, state="readonly", values=[])
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=8)

        ttk.Button(conn, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=(0, 8))
        self.conn_btn = ttk.Button(conn, text="Connect", style="Primary.TButton", command=self._toggle_connect)
        self.conn_btn.grid(row=0, column=3)

        self.status_lbl = ttk.Label(conn, textvariable=self.status_var, style="Card.TLabel",
                                    foreground="#b42318")
        self.status_lbl.grid(row=1, column=0, columnspan=4, sticky="w", pady=(8, 2))

        self.hardware_chk = ttk.Checkbutton(
            conn,
            text="Hardware send enabled",
            variable=self.hardware_send_var,
            command=self._on_hardware_send_toggle,
        )
        self.hardware_chk.grid(row=2, column=0, columnspan=2, sticky="w", pady=(4, 0))

        self.live_drag_chk = ttk.Checkbutton(
            conn,
            text=f"Live send while dragging (~{DEFAULT_SEND_INTERVAL_MS} ms)",
            variable=self.live_drag_var,
            command=self._on_live_drag_toggle,
        )
        self.live_drag_chk.grid(row=2, column=2, columnspan=2, sticky="w", pady=(4, 0))

        ttk.Checkbutton(conn, text="Auto-home on connect", variable=self.auto_home_var).grid(
            row=3, column=0, columnspan=2, sticky="w", pady=(4, 0)
        )
        self.home_btn = ttk.Button(conn, text="Home Now", command=self._home_to_start_pose)
        self.home_btn.grid(row=3, column=3, sticky="e", pady=(4, 0))
        self.lock_during_playback_widgets.append(self.home_btn)

        # Controls
        controls = ttk.LabelFrame(left, text="  Record / Save / Load / Playback  ",
                                  style="Card.TLabelframe", padding=12)
        controls.grid(row=2, column=0, sticky="ew", pady=(12, 0))
        for col in range(4):
            controls.columnconfigure(col, weight=1)

        self.add_btn = ttk.Button(controls, text="Add Step (Record)", command=self._add_step)
        self.add_btn.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.delete_btn = ttk.Button(controls, text="Delete Selected", command=self._delete_selected)
        self.delete_btn.grid(row=0, column=1, sticky="ew", padx=(0, 6))
        self.clear_btn = ttk.Button(controls, text="Clear Steps", command=self._clear_steps)
        self.clear_btn.grid(row=0, column=2, sticky="ew", padx=(0, 6))
        self.stop_btn = ttk.Button(controls, text="Stop", command=self._stop_play)
        self.stop_btn.grid(row=0, column=3, sticky="ew")

        self.save_btn = ttk.Button(controls, text="Save Steps (CSV)", command=self._save_steps)
        self.save_btn.grid(row=1, column=0, columnspan=2, sticky="ew", padx=(0, 6), pady=(8, 0))
        self.load_btn = ttk.Button(controls, text="Load Steps (CSV)", command=self._load_steps)
        self.load_btn.grid(row=1, column=2, columnspan=2, sticky="ew", pady=(8, 0))

        self.send_current_btn = ttk.Button(controls, text="Send Current Pose", style="Primary.TButton",
                                            command=lambda: self._send_current("manual-send", force=True))
        self.send_current_btn.grid(row=2, column=0, columnspan=2, sticky="ew", padx=(0, 6), pady=(8, 0))
        self.play_btn = ttk.Button(controls, text="Play Loop", style="Primary.TButton",
                                   command=self._start_play_loop)
        self.play_btn.grid(row=2, column=2, sticky="ew", pady=(8, 0))
        self.once_btn = ttk.Button(controls, text="Play Once", command=lambda: self._start_play_loop(once=True))
        self.once_btn.grid(row=2, column=3, sticky="ew", pady=(8, 0))
        self.pause_btn = ttk.Button(controls, text="Pause playback", command=self._toggle_pause)
        self.pause_btn.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Label(controls, textvariable=self.duration_var, style="CardMuted.TLabel").grid(row=4, column=2, columnspan=2, sticky="w")

        ttk.Label(controls, textvariable=self.playback_status_var, style="Card.TLabel",
                  foreground=COLORS["muted"]).grid(
            row=3, column=0, columnspan=4, sticky="w", pady=(10, 0)
        )

        self.lock_during_playback_widgets.extend([
            self.add_btn,
            self.delete_btn,
            self.clear_btn,
            self.save_btn,
            self.load_btn,
            self.send_current_btn,
            self.play_btn, self.once_btn,
        ])

        timing = ttk.LabelFrame(left, text="  Dynamic Timing  ", style="Card.TLabelframe", padding=12)
        timing.grid(row=3, column=0, sticky="ew", pady=(12, 0))
        for col in range(6):
            timing.columnconfigure(col, weight=1)

        ttk.Label(timing, text="Base delay (ms)", style="Card.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Entry(timing, textvariable=self.base_delay_var, width=8).grid(row=0, column=1, sticky="w")

        ttk.Label(timing, text="Settle (ms)", style="Card.TLabel").grid(row=0, column=2, sticky="w")
        ttk.Entry(timing, textvariable=self.settle_var, width=8).grid(row=0, column=3, sticky="w")

        ttk.Label(timing, text="ms per degree", style="Card.TLabel").grid(row=0, column=4, sticky="w")
        ttk.Entry(timing, textvariable=self.ms_per_deg_var, width=8).grid(row=0, column=5, sticky="w")

        ttk.Label(
            timing,
            text="Playback wait = max(base delay, largest joint movement × ms/degree) + settle.",
            style="CardMuted.TLabel",
        ).grid(row=1, column=0, columnspan=6, sticky="w", pady=(8, 0))

        # Right: recorded list
        self.rec_frame = ttk.LabelFrame(right, text="  Recorded Steps  ",
                                        style="Card.TLabelframe", padding=12)
        self.rec_frame.grid(row=0, column=0, sticky="nsew")
        self.rec_frame.columnconfigure(0, weight=1)
        self.rec_frame.rowconfigure(1, weight=1)

        ttk.Label(self.rec_frame,
                  text="Click a step to load it into the sliders (preview only). "
                       "Reorder with the arrows or Ctrl+↑/↓; Delete removes the selected step.",
                  style="CardMuted.TLabel", wraplength=360, justify="left").grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 8)
        )

        cols = ("idx", "base", "sh", "el", "wv", "wr", "grip", "spd")
        headings = {"idx": "#", "base": "Base", "sh": "Shld", "el": "Elbw",
                    "wv": "WrV", "wr": "WrR", "grip": "Grip", "spd": "Spd"}
        widths = {"idx": 30, "base": 42, "sh": 42, "el": 42, "wv": 42,
                  "wr": 42, "grip": 42, "spd": 36}
        self.steps_tree = ttk.Treeview(self.rec_frame, columns=cols, show="headings",
                                       height=12, style="Steps.Treeview",
                                       selectmode="browse")
        for cdef in cols:
            anchor = "w" if cdef == "idx" else "center"
            self.steps_tree.heading(cdef, text=headings[cdef], anchor=anchor)
            self.steps_tree.column(cdef, width=widths[cdef], anchor=anchor, stretch=False)
        self.steps_tree.column("idx", anchor="center")
        # Zebra striping for fast scanning.
        self.steps_tree.tag_configure("odd", background=COLORS["panel"])
        self.steps_tree.tag_configure("even", background=COLORS["panel_alt"])
        self.steps_tree.grid(row=1, column=0, sticky="nsew")

        tree_sb = ttk.Scrollbar(self.rec_frame, command=self.steps_tree.yview)
        self.tree_sb = tree_sb
        tree_sb.grid(row=1, column=1, sticky="ns")
        tree_hsb = ttk.Scrollbar(self.rec_frame, orient="horizontal", command=self.steps_tree.xview)
        self.tree_hsb = tree_hsb
        tree_hsb.grid(row=2, column=0, sticky="ew")
        self.steps_tree.configure(yscrollcommand=tree_sb.set, xscrollcommand=tree_hsb.set)

        # Load a step into the sliders only on genuine user input — a mouse click
        # or arrow-key navigation. Programmatic selection (reorder, duplicate,
        # delete-reselect, playback playhead) never loads, so it can't disturb
        # the live commanded pose.
        self.steps_tree.bind("<ButtonRelease-1>", self._on_step_activate)
        self.steps_tree.bind("<KeyRelease-Up>", self._on_step_activate)
        self.steps_tree.bind("<KeyRelease-Down>", self._on_step_activate)
        self.steps_tree.bind("<Control-Up>", lambda e: (self._move_step(-1), "break")[1])
        self.steps_tree.bind("<Control-Down>", lambda e: (self._move_step(1), "break")[1])
        self.steps_tree.bind("<Delete>", lambda e: (self._delete_selected(), "break")[1])

        # Empty-state hint shown over the (empty) table.
        self.steps_empty_lbl = ttk.Label(
            self.rec_frame, text="No steps yet — set a pose and click “Add Step (Record)”.",
            style="CardMuted.TLabel")

        rec_btns = ttk.Frame(self.rec_frame, style="Card.TFrame")
        rec_btns.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        for i in range(4):
            rec_btns.columnconfigure(i, weight=1)
        self.move_up_btn = ttk.Button(rec_btns, text="↑ Move Up",
                                      command=lambda: self._move_step(-1))
        self.move_up_btn.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.move_down_btn = ttk.Button(rec_btns, text="↓ Move Down",
                                        command=lambda: self._move_step(1))
        self.move_down_btn.grid(row=0, column=1, sticky="ew", padx=(0, 6))
        self.dup_btn = ttk.Button(rec_btns, text="Duplicate",
                                  command=self._duplicate_selected)
        self.dup_btn.grid(row=0, column=2, sticky="ew", padx=(0, 6))
        self.del_step_btn = ttk.Button(rec_btns, text="Delete",
                                       command=self._delete_selected)
        self.del_step_btn.grid(row=0, column=3, sticky="ew")
        self.lock_during_playback_widgets.extend(
            [self.move_up_btn, self.move_down_btn, self.dup_btn, self.del_step_btn])

        self._refresh_steps_view()

        # Right: paste box
        paste = ttk.LabelFrame(right, text="  Advanced Import: Paste CSV Steps  ",
                               style="Card.TLabelframe", padding=12)
        paste.grid(row=1, column=0, sticky="ew", pady=(12, 0))
        paste.columnconfigure(0, weight=1)

        ttk.Label(paste, text="Format per line:  base,shoulder,elbow,wristV,wristR,gripper,speed",
                  style="CardMuted.TLabel", font=("Consolas", 9)).grid(
            row=0, column=0, sticky="w", pady=(0, 4)
        )
        self.paste_text = tk.Text(paste, height=5, wrap="none", bg="#ffffff", fg=COLORS["text"],
                                  borderwidth=1, relief="solid", highlightthickness=0,
                                  font=("Consolas", 9))
        self.paste_text.grid(row=1, column=0, sticky="ew")

        self.paste_btn = ttk.Button(paste, text="Paste & Add Steps", command=self._paste_and_add_steps)
        self.paste_btn.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        self.lock_during_playback_widgets.append(self.paste_btn)

        # Right: console
        console_box = ttk.LabelFrame(right, text="  Console / Diagnostics  ",
                                     style="Card.TLabelframe", padding=12)
        console_box.grid(row=2, column=0, sticky="nsew", pady=(12, 0))
        console_box.columnconfigure(0, weight=1)
        console_box.rowconfigure(0, weight=1)

        self.console = tk.Text(console_box, wrap="word", state="disabled",
                               bg="#1f242b", fg="#cdd5df", insertbackground="#cdd5df",
                               borderwidth=0, relief="flat", highlightthickness=0,
                               font=("Consolas", 9), padx=8, pady=6)
        self.console.grid(row=0, column=0, sticky="nsew")
        sb = ttk.Scrollbar(console_box, command=self.console.yview)
        sb.grid(row=0, column=1, sticky="ns")
        self.console.configure(yscrollcommand=sb.set)

        # Colour the console by log level so it can be scanned at a glance.
        self.console.tag_configure("info", foreground="#cdd5df")
        self.console.tag_configure("tx", foreground="#6fcf97")     # sent to hardware
        self.console.tag_configure("sim", foreground="#8b94a3")    # simulation only
        self.console.tag_configure("warn", foreground="#e3b341")   # skipped / notices
        self.console.tag_configure("err", foreground="#ff7b72")    # errors / failures

        console_box.rowconfigure(1, weight=0)
        clear_console = ttk.Button(console_box, text="Clear console",
                                   command=self._clear_console)
        clear_console.grid(row=1, column=0, columnspan=2, sticky="e", pady=(8, 0))

    def _make_slider(self, parent, row, name, label, default):
        c = COLORS
        lo, hi = UI_LIMITS[name]
        tag = tk.Canvas(parent, width=12, height=12, highlightthickness=0, bd=0, bg=c["bg"])
        tag.grid(row=row, column=0, sticky="nw", padx=(0, 6), pady=(4, 0), rowspan=2)
        tag.create_oval(1, 1, 11, 11, fill=JOINT_COLORS.get(name, "#888"), outline="")

        ttk.Label(parent, text=label, style="Card.TLabel").grid(
            row=row, column=1, sticky="nw", rowspan=2)

        v = tk.IntVar(value=int(default))
        self.vars[name] = v
        self._scale_guard[name] = False

        accent = JOINT_COLORS.get(name, c["accent"])
        scale = tk.Scale(
            parent,
            from_=lo,
            to=hi,
            orient="horizontal",
            variable=v,
            showvalue=False,
            resolution=1,
            sliderlength=20,
            width=16,
            length=170,
            command=lambda x, nm=name: self._on_scale_move(nm, x),
            highlightthickness=0,
            borderwidth=1,
            bg=c["bg"],
            troughcolor="#cdd2da",
            sliderrelief="raised",
            activebackground=accent,
        )
        scale.set(default)
        scale.grid(row=row, column=3, sticky="ew", padx=8)        self.scales[name] = scale
        scale.bind("<FocusIn>", lambda _e, nm=name: self._log(f"Focused {nm}: arrow keys adjust the slider"))

        minus_btn = ttk.Button(parent, text="–", width=2,
                               command=lambda nm=name: self._nudge_joint(nm, -self.step_size_var.get()))
        minus_btn.grid(row=row, column=2, sticky="w", rowspan=2)
        plus_btn = ttk.Button(parent, text="+", width=2,
                              command=lambda nm=name: self._nudge_joint(nm, self.step_size_var.get()))
        plus_btn.grid(row=row, column=4, sticky="w", padx=(6, 0), rowspan=2)
        self._plus_buttons.append(plus_btn)
        self.lock_during_playback_widgets.extend([scale, minus_btn, plus_btn])
        # Absolute servo angle entry, committed on Return or focus loss.
        entry = ttk.Entry(parent, width=5)
        entry.insert(0, str(HOME_POSE.__dict__[name] if name != "speed" else default))
        entry.grid(row=row, column=6, padx=(5, 0), sticky="w", rowspan=2)
        self._angle_entries.append(entry)
        def commit_entry(_event=None):
            try:
                absolute = int(entry.get().strip())
                if not LIMITS[name][0] <= absolute <= LIMITS[name][1]:
                    raise ValueError("outside servo limits")
                relative = absolute if name == "speed" else absolute - getattr(HOME_POSE, name)
                changed = relative != self.vars[name].get()
                self._scale_guard[name] = True
                try:
                    self.scales[name].set(relative)
                finally:
                    self._scale_guard[name] = False
                if changed:
                    self._sync_target_from_sliders()
                    self._commit_preview()
                    self._send_current("numeric-entry", force=True)
            except (ValueError, tk.TclError):
                self._log(f"WARN: Invalid {name} angle. Allowed: {LIMITS[name][0]}..{LIMITS[name][1]}")
            entry.delete(0, "end")
            entry.insert(0, str(getattr(self._current_pose(), name)))
        entry.bind("<Return>", commit_entry)
        entry.bind("<FocusOut>", commit_entry)
        self.lock_during_playback_widgets.append(entry)

        def sync_angle_entry(*_):
            # Update the numeric display from the slider, but never interrupt
            # text that the operator is actively entering.
            try:
                if entry.focus_get() == entry:
                    return
            except tk.TclError:
                return
            absolute_value = v.get() if name == "speed" else v.get() + getattr(HOME_POSE, name)
            if entry.get() != str(absolute_value):
                entry.delete(0, "end")
                entry.insert(0, str(absolute_value))

        v.trace_add("write", sync_angle_entry)

        # Clear semantic direction under each slider.
        if name in ARM_JOINTS:
            min_txt = f"{lo:+d}°"
            mid_txt = "0°"
            max_txt = f"{hi:+d}°"
        else:
            min_txt = f"{lo}"
            mid_txt = ""
            max_txt = f"{hi}"

        under = ttk.Frame(parent, style="Card.TFrame")
        under.grid(row=row + 1, column=3, sticky="ew", padx=8)
        under.columnconfigure(0, weight=1)
        under.columnconfigure(1, weight=1)
        under.columnconfigure(2, weight=1)
        small = ("Segoe UI", 8)
        ttk.Label(under, text=min_txt, style="CardMuted.TLabel", font=small).grid(row=0, column=0, sticky="w")
        ttk.Label(under, text=mid_txt, style="CardMuted.TLabel", font=small).grid(row=0, column=1)
        ttk.Label(under, text=max_txt, style="CardMuted.TLabel", font=small).grid(row=0, column=2, sticky="e")

        # Fixed-width value chip so its width never changes (no layout shift).
        val_lbl = tk.Label(parent, width=4, anchor="e", text=str(default),
                           bg=c["bg"], fg=c["text"], font=("Consolas", 10, "bold"))
        val_lbl.grid(row=row, column=5, sticky="ne", rowspan=2)
        self._value_labels.append(val_lbl)

        def update_lbl(*_):
            val_lbl.configure(text=str(v.get()))

        v.trace_add("write", update_lbl)

        # Track the handle being held so we can defer committing the solid arm
        # until release (drag = scrub the dashed target; release = move the arm).
        scale.bind("<Button-1>", lambda _e, nm=name: self._on_slider_press(nm))
        scale.bind("<ButtonRelease-1>", lambda _e, nm=name: self._on_slider_release(nm))

    # ---------- Live preview renderer ----------
    def _commit_speed_deg_per_frame(self) -> float:
        """Degrees the fastest joint may move this frame, scaled by speed."""
        lo, hi = LIMITS["speed"]
        sp = max(lo, min(hi, self.commit_pose.speed))
        frac = (sp - lo) / (hi - lo) if hi > lo else 0.0
        dps = SIM_DEG_PER_SEC_AT_MIN_SPEED + frac * (
            SIM_DEG_PER_SEC_AT_MAX_SPEED - SIM_DEG_PER_SEC_AT_MIN_SPEED)
        now = time.time()
        dt = now - self._last_anim_time
        self._last_anim_time = now
        dt = max(0.0, min(0.1, dt))      # guard against long stalls
        return dps * dt

    @staticmethod
    def _smoothstep(t: float) -> float:
        """Ease-in / ease-out (3t^2 - 2t^3); accelerates then decelerates."""
        t = max(0.0, min(1.0, t))
        return t * t * (3.0 - 2.0 * t)

    def _animate_preview(self):
        # The solid arm executes one coordinated, eased move from _anim_start to
        # commit_pose. A single progress value advances at a speed-scaled rate
        # (normalised by the largest joint travel), so all joints start and stop
        # together and ease into place rather than crawling at constant speed.
        changed = False

        # Speed is a parameter, not a moving part: snap it without animation.
        if self.sim_values["speed"] != self.commit_pose.speed:
            self.sim_values["speed"] = float(self.commit_pose.speed)
            changed = True

        # Largest angular distance of this move (arm joints only).
        span = max(abs(float(getattr(self.commit_pose, f)) - self._anim_start[f])
                   for f in ARM_JOINTS)

        if self._anim_progress < 1.0 and span > 1e-6:
            d_deg = self._commit_speed_deg_per_frame()
            self._anim_progress = min(1.0, self._anim_progress + d_deg / span)
            eased = self._smoothstep(self._anim_progress)
            for f in ARM_JOINTS:
                start = self._anim_start[f]
                goal = float(getattr(self.commit_pose, f))
                self.sim_values[f] = start + (goal - start) * eased
            changed = True
        elif self._anim_progress < 1.0:
            # Nothing meaningful to travel; finish immediately.
            self._anim_progress = 1.0
            for f in ARM_JOINTS:
                self.sim_values[f] = float(getattr(self.commit_pose, f))
            changed = True

        if changed or self._active_drag is not None or self._hover_joint is not None:
            self._draw_live_preview()
        if not self._closing:
            self.after(PREVIEW_FRAME_MS, self._animate_preview)

    # -- geometry --------------------------------------------------------------
    def _base_geom(self, width: int, height: int):
        """Turntable centre + the projected azimuth of the base joint."""
        cx = width * 0.40
        disc_cy = height - 40
        rx, ry = 64, 24                       # turntable ellipse radii (perspective)
        return cx, disc_cy, rx, ry

    def _forward_kinematics(self, values: Dict[str, float]) -> dict:
        width = int(self.preview_canvas.cget("width"))
        height = int(self.preview_canvas.cget("height"))

        base_rel = values["base"] - HOME_POSE.base          # -90..+90
        shoulder_rel = values["shoulder"] - HOME_POSE.shoulder
        elbow_rel = values["elbow"] - HOME_POSE.elbow
        wrist_rel = values["wristV"] - HOME_POSE.wristV

        cx, disc_cy, rx, ry = self._base_geom(width, height)

        # Base rotation is yaw about the vertical axis. In this side view we show
        # it two ways: the arm's mounting point swings along the turntable rim,
        # and the arm's horizontal reach is gently foreshortened as it turns away
        # from the viewing plane (never collapsing, so elevation stays readable).
        az = math.radians(base_rel * 0.95)
        mount_x = cx + math.sin(az) * rx * 0.66
        mount_y = disc_cy - ry * 0.30 - 10
        fore = 0.72 + 0.28 * math.cos(az)                    # 0.44..1.0 horizontal scale

        l1, l2, l3 = 84, 74, 50
        theta1 = math.radians(-92 + shoulder_rel * 0.82)
        theta2 = theta1 + math.radians(66 - elbow_rel * 0.72)
        theta3 = theta2 + math.radians(20 + wrist_rel * 0.78)

        def step(p, theta, length):
            return (p[0] + fore * length * math.cos(theta), p[1] + length * math.sin(theta))

        shoulder = (mount_x, mount_y)
        elbow = step(shoulder, theta1, l1)
        wrist = step(elbow, theta2, l2)
        tip = step(wrist, theta3, l3)

        return {
            "shoulder": shoulder, "elbow": elbow, "wrist": wrist, "tip": tip,
            "theta_tip": theta3, "az": az,
        }

    # -- refined arm pieces -----------------------------------------------------
    def _draw_link(self, canvas, a, b, tag, width=18):
        """A link drawn as a lit cylinder: concentric centered strokes from a dark
        rim to a light core, plus a thin sheen near the top edge."""
        s = ARM_SKIN
        for w, col in ((width + 2, s["link_outline"]),
                       (width, s["link_dark"]),
                       (width - 5, s["link_mid"]),
                       (max(2, width - 11), s["link_light"])):
            canvas.create_line(*a, *b, fill=col, width=w, capstyle="round", tags=tag)
        # Sheen: a short thin highlight offset toward the screen-up edge.
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy) or 1.0
        ux, uy = dx / L, dy / L
        px, py = -uy, ux
        if py > 0:                       # ensure the offset points "up" (-y)
            px, py = -px, -py
        off = width * 0.24
        canvas.create_line(a[0] + ux * width * 0.55 + px * off,
                           a[1] + uy * width * 0.55 + py * off,
                           b[0] - ux * width * 0.55 + px * off,
                           b[1] - uy * width * 0.55 + py * off,
                           fill=s["link_sheen"], width=max(1, int(width * 0.16)),
                           capstyle="round", tags=tag)

    def _draw_servo_joint(self, canvas, x, y, tag, r=11):
        """A joint that reads as a servo hub: dark ring, light face, accent bolt."""
        s = ARM_SKIN
        canvas.create_oval(x - r - 1, y - r - 1, x + r + 1, y + r + 1,
                           fill=s["joint_ring"], outline="", tags=tag)
        canvas.create_oval(x - r + 2, y - r + 2, x + r - 2, y + r - 2,
                           fill=s["joint_face"], outline="", tags=tag)
        canvas.create_arc(x - r + 3, y - r + 3, x + r - 3, y + r - 3,
                          start=35, extent=130, style="arc", outline="#ffffff",
                          width=2, tags=tag)
        hub = 4
        canvas.create_oval(x - hub, y - hub, x + hub, y + hub,
                           fill=s["joint_hub"], outline=s["joint_ring"], tags=tag)
        canvas.create_oval(x - 1.6, y - 2.4, x + 1.2, y + 0.2,
                           fill=s["joint_hub_hi"], outline="", tags=tag)

    def _draw_chain(self, values: Dict[str, float], ghost: bool = False):
        canvas = self.preview_canvas
        pts = self._forward_kinematics(values)
        shoulder, elbow, wrist, tip = pts["shoulder"], pts["elbow"], pts["wrist"], pts["tip"]
        theta = pts["theta_tip"]
        s = ARM_SKIN

        # Gripper opening (servo value only) + wrist roll (orientation) — needed
        # for both the solid and ghost renders.
        open_frac = 1.0 - ((values["gripper"] - LIMITS["gripper"][0]) /
                           (LIMITS["gripper"][1] - LIMITS["gripper"][0]))
        open_frac = max(0.0, min(1.0, open_frac))
        roll = math.radians(values["wristR"] - 90)
        dx, dy = math.cos(theta), math.sin(theta)        # forearm direction
        nx, ny = -math.sin(theta), math.cos(theta)       # in-plane perpendicular
        ax = nx * math.cos(roll) + dx * math.sin(roll)   # jaw-opening axis (rolls)
        ay = ny * math.cos(roll) + dy * math.sin(roll)
        spread = 5 + open_frac * 15
        finger_len = 24
        tx, ty = tip

        # ---- Ghost: a clean, light dashed skeleton of the target pose ----
        if ghost:
            tag = "ghost"
            for a, b in ((shoulder, elbow), (elbow, wrist), (wrist, tip)):
                canvas.create_line(*a, *b, fill="#c4cad3", width=6, dash=(5, 4),
                                   capstyle="round", tags=tag)
            for (jx, jy) in (shoulder, elbow, wrist):
                canvas.create_oval(jx - 5, jy - 5, jx + 5, jy + 5,
                                   fill="#eef0f3", outline="#aab1bb", width=1, tags=tag)
            for sgn in (1, -1):
                bx, by = tx + ax * spread * sgn, ty + ay * spread * sgn
                ex = bx + dx * finger_len + ax * sgn * open_frac * 6
                ey = by + dy * finger_len + ay * sgn * open_frac * 6
                canvas.create_line(tx, ty, bx, by, fill="#c4cad3", width=2,
                                   dash=(4, 3), capstyle="round", tags=tag)
                canvas.create_line(bx, by, ex, ey, fill="#c4cad3", width=2,
                                   dash=(4, 3), capstyle="round", tags=tag)
            return

        # ---- Solid arm ----
        tag = "sim"
        # Tapered link widths from the heavy upper arm down to the slim hand.
        self._draw_link(canvas, shoulder, elbow, tag, width=19)
        self._draw_link(canvas, elbow, wrist, tag, width=16)
        self._draw_link(canvas, wrist, tip, tag, width=12)

        # Shoulder and elbow as servo hubs (the wrist gets the roll dial below).
        self._draw_servo_joint(canvas, shoulder[0], shoulder[1], tag, r=12)
        self._draw_servo_joint(canvas, elbow[0], elbow[1], tag, r=10)

        # ---- Gripper: a wrist plate + two machined jaws with inward hooks ----
        plate_half = max(spread + 4, 9)
        p1 = (tx - ax * plate_half, ty - ay * plate_half)
        p2 = (tx + ax * plate_half, ty + ay * plate_half)
        canvas.create_line(*p1, *p2, fill=s["grip_dark"], width=10, capstyle="round", tags=tag)
        canvas.create_line(*p1, *p2, fill=s["grip_mid"], width=6, capstyle="round", tags=tag)
        canvas.create_line(*p1, *p2, fill=s["grip_light"], width=2, capstyle="round", tags=tag)
        for sgn in (1, -1):
            bx, by = tx + ax * spread * sgn, ty + ay * spread * sgn
            mx, my = bx + dx * finger_len, by + dy * finger_len
            hx, hy = mx - ax * sgn * 7, my - ay * sgn * 7        # inward hook
            canvas.create_line(bx, by, mx, my, hx, hy, fill=s["grip_dark"], width=6.5,
                               capstyle="round", joinstyle="round", tags=tag)
            canvas.create_line(bx, by, mx, my, hx, hy, fill=s["grip_mid"], width=3,
                               capstyle="round", joinstyle="round", tags=tag)

        # ---- Wrist-roll dial at M4/M5 (servo-style face + hand) ----
        wx, wy = wrist
        rr = 12
        canvas.create_oval(wx - rr - 1, wy - rr - 1, wx + rr + 1, wy + rr + 1,
                           fill=s["joint_ring"], outline="", tags=tag)
        canvas.create_oval(wx - rr + 2, wy - rr + 2, wx + rr - 2, wy + rr - 2,
                           fill="#ffffff", outline="", tags=tag)
        canvas.create_line(wx - ax * (rr - 3), wy - ay * (rr - 3),
                           wx + ax * (rr - 3), wy + ay * (rr - 3),
                           fill="#c9ced6", width=1, tags=tag)
        canvas.create_line(wx, wy, wx + ax * (rr - 3), wy + ay * (rr - 3),
                           fill=COLORS["accent_dark"], width=2, arrow="last",
                           arrowshape=(7, 8, 3), tags=tag)
        canvas.create_oval(wx - 2, wy - 2, wx + 2, wy + 2, fill=COLORS["accent_dark"],
                           outline="", tags=tag)

        # Compact motor labels. M6 sits just past the gripper along the forearm so
        # it never collides with the jaws whichever way the hand points.
        lab = [("M2", shoulder, (-15, 15)), ("M3", elbow, (13, -4)),
               ("M4/M5", wrist, (17, -14))]
        for txt, (px, py), (ox, oy) in lab:
            canvas.create_text(px + ox, py + oy, text=txt, fill=COLORS["muted"],
                               font=("Segoe UI", 8, "bold"), tags=tag)
        m6x = tip[0] + dx * (finger_len + 15)
        m6y = tip[1] + dy * (finger_len + 15) + 4
        canvas.create_text(m6x, m6y, text="M6", fill=COLORS["muted"],
                           font=("Segoe UI", 8, "bold"), tags=tag)

        # Record hover hit-targets (absolute servo angles for the tooltip).
        sh = int(round(values["shoulder"]))
        el = int(round(values["elbow"]))
        wv = int(round(values["wristV"]))
        wrr = int(round(values["wristR"]))
        gp = int(round(values["gripper"]))
        gspan = LIMITS["gripper"][1] - LIMITS["gripper"][0]
        gpct = max(0, min(100, int(round((gp - LIMITS["gripper"][0]) / gspan * 100))))
        self._solid_joints = [
            {"name": "shoulder", "title": "Shoulder · M2", "x": shoulder[0],
             "y": shoulder[1], "r": 16,
             "lines": [f"{sh}°", f"{sh - HOME_POSE.shoulder:+d}° from home"]},
            {"name": "elbow", "title": "Elbow · M3", "x": elbow[0], "y": elbow[1],
             "r": 15,
             "lines": [f"{el}°", f"{el - HOME_POSE.elbow:+d}° from home"]},
            {"name": "wrist", "title": "Wrist · M4 / M5", "x": wrist[0], "y": wrist[1],
             "r": 15,
             "lines": [f"up/down  {wv}°", f"rotate  {wrr}°"]},
            {"name": "gripper", "title": "Gripper · M6", "x": tip[0], "y": tip[1],
             "r": 16,
             "lines": [f"servo  {gp}", f"{gpct}% closed"]},
        ]

    def _draw_turntable(self, canvas, width, height):
        """Base motor (M1) as a short rotary cylinder with a soft ground shadow
        and a pointer on the top face that swings with the base yaw."""
        cx, cy, rx, ry = self._base_geom(width, height)
        az = math.radians(self.sim_values["base"] - HOME_POSE.base)
        s = ARM_SKIN
        ch = 15                                   # cylinder height

        # Soft ground shadow.
        canvas.create_oval(cx - rx - 8, cy + ch - ry * 0.35,
                           cx + rx + 8, cy + ch + ry * 0.75,
                           fill=s["base_shadow"], outline="", tags="sim")
        # Cylinder: bottom ellipse, side band (with shaded edges), top face.
        canvas.create_oval(cx - rx, cy + ch - ry, cx + rx, cy + ch + ry,
                           fill=s["base_bottom"], outline=s["base_rim_dk"], width=1, tags="sim")
        canvas.create_rectangle(cx - rx, cy, cx + rx, cy + ch,
                                fill=s["base_side"], outline="", tags="sim")
        canvas.create_rectangle(cx - rx, cy, cx - rx + 7, cy + ch,
                                fill=s["base_side_dk"], outline="", tags="sim")
        canvas.create_rectangle(cx + rx - 7, cy, cx + rx, cy + ch,
                                fill=s["base_side_dk"], outline="", tags="sim")
        canvas.create_oval(cx - rx, cy - ry, cx + rx, cy + ry,
                           fill=s["base_top"], outline=s["base_rim_dk"], width=1, tags="sim")
        canvas.create_oval(cx - rx + 10, cy - ry + 4, cx + rx - 10, cy + ry - 4,
                           outline=s["base_rim_dk"], tags="sim")
        canvas.create_arc(cx - rx, cy - ry, cx + rx, cy + ry,
                          start=25, extent=130, style="arc",
                          outline=s["base_rim_hi"], width=2, tags="sim")

        # Direction pointer on the top face (filled wedge) + bolt hub.
        def face(ang, k=1.0):
            return (cx + math.sin(ang) * (rx - 7) * k,
                    cy - math.cos(ang) * (ry - 4) * k)
        tipp = face(az, 1.0)
        basel = face(az - 2.3, 0.34)
        baser = face(az + 2.3, 0.34)
        canvas.create_polygon(*basel, *tipp, *baser,
                              fill=COLORS["accent"], outline=COLORS["accent_dark"],
                              width=1, tags="sim")
        canvas.create_oval(cx - 6, cy - 4, cx + 6, cy + 4, fill="#ffffff",
                           outline=COLORS["accent_dark"], width=2, tags="sim")
        canvas.create_oval(cx - 2.4, cy - 1.6, cx + 2.4, cy + 1.6,
                           fill=COLORS["accent_dark"], outline="", tags="sim")
        canvas.create_text(cx, cy + ch + 22,
                           text=f"M1 base  {int(round(self.sim_values['base']))}°",
                           fill=COLORS["muted"], font=("Segoe UI", 8, "bold"), tags="sim")

        b = int(round(self.sim_values["base"]))
        self._base_hit = {"name": "base", "title": "Base · M1", "x": cx, "y": cy,
                          "rx": rx, "ry": ry + 6,
                          "lines": [f"{b}° yaw", f"{b - HOME_POSE.base:+d}° from home"]}

    def _draw_live_preview(self):
        canvas = self.preview_canvas
        canvas.delete("all")
        width = int(canvas.cget("width"))
        height = int(canvas.cget("height"))
        c = COLORS

        # Background, subtle grid, floor band.
        canvas.create_rectangle(0, 0, width, height, fill=c["canvas_bg"], outline="")
        for gy in range(40, height - 30, 34):
            canvas.create_line(12, gy, width - 12, gy, fill=c["grid"])
        for gx in range(40, width - 12, 48):
            canvas.create_line(gx, 30, gx, height - 30, fill=c["grid"])
        canvas.create_rectangle(2, height - 30, width - 2, height - 2,
                                fill=c["floor"], outline="")
        canvas.create_line(2, height - 30, width - 2, height - 30, fill="#dde1e6")
        canvas.create_text(12, 12, text="Live commanded-pose preview", anchor="nw",
                           fill=c["accent_dark"], font=("Segoe UI Semibold", 9))

        # Base turntable.
        self._draw_turntable(canvas, width, height)

        # Dashed target ghost: the live slider target while dragging, otherwise
        # the committed destination the solid arm is still travelling toward.
        ghost_pose = self.target_pose if self._active_drag else self.commit_pose
        ghost_values = pose_to_values(ghost_pose)
        delta = max(abs(ghost_values[f] - self.sim_values[f]) for f in ARM_JOINTS)
        if delta > 1.0:
            self._draw_chain(ghost_values, ghost=True)
        self._draw_chain(self.sim_values, ghost=False)
        if self.trajectory_var.get():
            points = []
            for pose in self.recorded:
                joints = self._forward_kinematics(pose_to_values(pose))
                # End effector is the final joint in the 2D kinematic chain.
                if isinstance(joints, dict):
                    pt = joints.get("tip") or joints.get("gripper")
                    if isinstance(pt, (tuple, list)) and len(pt) >= 2:
                        points.extend(pt[:2])
            if len(points) >= 4:
                canvas.create_line(*points, fill="#e08b39", width=2, dash=(3, 5))

        # Legend chip.
        if self._active_drag:
            legend = "Solid = arm now   •   dashed = where it will go on release"
        else:
            legend = "Solid = simulated arm   •   dashed = target"
        canvas.create_text(12, height - 14, text=legend, anchor="sw",
                           fill=c["muted"], font=("Segoe UI", 8))

        # Hover tooltip (drawn last so it sits on top).
        if self._hover_joint is not None:
            self._draw_joint_tooltip(canvas, width, height)

    def _joint_at(self, mx: float, my: float) -> Optional[str]:
        """Return the name of the joint/base under the cursor, or None."""
        for j in self._solid_joints:
            if (mx - j["x"]) ** 2 + (my - j["y"]) ** 2 <= j["r"] ** 2:
                return j["name"]
        b = self._base_hit
        if b is not None and b["rx"] > 0 and b["ry"] > 0:
            if ((mx - b["x"]) / b["rx"]) ** 2 + ((my - b["y"]) / b["ry"]) ** 2 <= 1.0:
                return b["name"]
        return None

    def _hover_record(self, name: str) -> Optional[dict]:
        for j in self._solid_joints:
            if j["name"] == name:
                return j
        if self._base_hit is not None and self._base_hit["name"] == name:
            return self._base_hit
        return None

    def _draw_joint_tooltip(self, canvas, width, height):
        rec = self._hover_record(self._hover_joint)
        if rec is None:
            return
        title = rec["title"]
        lines = rec["lines"]
        pad = 8
        title_f = ("Segoe UI Semibold", 9)
        body_f = ("Consolas", 9)
        # Measure with hidden text items so the box fits exactly.
        widths = []
        tmp = canvas.create_text(-100, -100, text=title, font=title_f, anchor="nw")
        widths.append(canvas.bbox(tmp)[2] - canvas.bbox(tmp)[0]); canvas.delete(tmp)
        for ln in lines:
            tmp = canvas.create_text(-100, -100, text=ln, font=body_f, anchor="nw")
            widths.append(canvas.bbox(tmp)[2] - canvas.bbox(tmp)[0]); canvas.delete(tmp)
        box_w = max(widths) + pad * 2
        box_h = pad * 2 + 16 + 14 * len(lines)

        # Anchor above-right of the joint, then clamp inside the canvas.
        ax, ay = rec["x"] + 14, rec["y"] - box_h - 10
        ax = max(6, min(ax, width - box_w - 6))
        ay = max(6, min(ay, height - box_h - 6))

        # Connector dot on the joint.
        canvas.create_oval(rec["x"] - 3, rec["y"] - 3, rec["x"] + 3, rec["y"] + 3,
                           fill=COLORS["accent"], outline="")
        # Card with a soft shadow.
        canvas.create_rectangle(ax + 2, ay + 3, ax + box_w + 2, ay + box_h + 3,
                                fill="#d7dbe0", outline="")
        canvas.create_rectangle(ax, ay, ax + box_w, ay + box_h,
                                fill="#272d36", outline=COLORS["accent"], width=1)
        canvas.create_text(ax + pad, ay + pad, text=title, anchor="nw",
                           fill="#ffd9bf", font=title_f)
        for i, ln in enumerate(lines):
            canvas.create_text(ax + pad, ay + pad + 17 + i * 14, text=ln, anchor="nw",
                               fill="#e6ebf2", font=body_f)

    def _on_canvas_motion(self, event):
        name = self._joint_at(event.x, event.y)
        if name != self._hover_joint:
            self._hover_joint = name
            try:
                self.preview_canvas.configure(cursor="hand2" if name else "")
            except Exception:
                pass
            self._draw_live_preview()

    def _on_canvas_leave(self, _event=None):
        if self._hover_joint is not None:
            self._hover_joint = None
            try:
                self.preview_canvas.configure(cursor="")
            except Exception:
                pass
            self._draw_live_preview()

    # ---------- Sliders and pose state ----------
    def _commit_preview(self):
        """Let the solid simulated arm travel to the current target pose.

        Snapshots the current animated position as the start of a new eased move
        and resets progress, so re-targeting mid-travel stays smooth.
        """
        self.commit_pose = self.target_pose
        self._anim_start = dict(self.sim_values)
        self._anim_progress = 0.0
        self._last_anim_time = time.time()

    def _nudge_joint(self, name: str, delta: int):
        lo, hi = UI_LIMITS[name]
        cur = int(self.vars[name].get())
        nxt = max(lo, min(hi, cur + int(delta)))

        self._scale_guard[name] = True
        try:
            self.vars[name].set(nxt)
            self.scales[name].set(nxt)
        finally:
            self._scale_guard[name] = False

        self._sync_target_from_sliders()
        self._commit_preview()          # a nudge is a committed move: animate the arm
        self._send_current(reason=f"nudge({name},{delta:+d})", force=False)

    def _on_slider_press(self, name: str):
        # Begin a scrub: from here until release we only move the dashed target
        # ghost, holding the solid arm where it is.
        self._active_drag = name

    def _on_scale_move(self, name: str, x: str):
        if self._scale_guard.get(name, False):
            return
        try:
            val = int(round(float(x)))
        except Exception:
            return
        lo, hi = UI_LIMITS[name]
        val = max(lo, min(hi, val))
        self.vars[name].set(val)
        self._sync_target_from_sliders()

        # If this move did not come from a held handle (e.g. keyboard arrows),
        # there is no release coming, so commit immediately.
        if self._active_drag is None:
            self._commit_preview()

        if self.live_drag_var.get():
            self._send_current(reason=f"drag({name})", force=False)

    def _on_slider_release(self, name: str):
        scale = self.scales[name]
        lo, hi = UI_LIMITS[name]
        val = int(self.vars[name].get())
        val = max(lo, min(hi, val))

        self._scale_guard[name] = True
        try:
            self.vars[name].set(val)
            scale.set(val)
        finally:
            self._scale_guard[name] = False

        self._active_drag = None
        self._sync_target_from_sliders()
        self._commit_preview()          # release: solid arm now travels to target
        self._send_current(reason=f"release({name})", force=True)

    def _current_pose(self) -> Pose:
        rel = {k: int(self.vars[k].get()) for k in ARM_JOINTS}
        return Pose(
            base=clamp("base", HOME_POSE.base + rel["base"]),
            shoulder=clamp("shoulder", HOME_POSE.shoulder + rel["shoulder"]),
            elbow=clamp("elbow", HOME_POSE.elbow + rel["elbow"]),
            wristV=clamp("wristV", HOME_POSE.wristV + rel["wristV"]),
            wristR=clamp("wristR", HOME_POSE.wristR + rel["wristR"]),
            gripper=clamp("gripper", HOME_POSE.gripper + rel["gripper"]),
            speed=clamp("speed", int(self.vars["speed"].get())),
        )

    def _sync_target_from_sliders(self):
        self.target_pose = self._current_pose()
        self._update_pose_readout()

    def _update_pose_readout(self):
        p = self.target_pose
        self.pose_summary_var.set("Target  " + p.compact())

        gripper_span = LIMITS["gripper"][1] - LIMITS["gripper"][0]
        closed_pct = int(round((p.gripper - LIMITS["gripper"][0]) / gripper_span * 100))
        closed_pct = max(0, min(100, closed_pct))
        if closed_pct <= 25:
            grip_label = "open"
        elif closed_pct >= 75:
            grip_label = "closed"
        else:
            grip_label = "mid"
        # Fixed-width fields (<6 and <4) so the string length never changes ->
        # the readout label can never resize the preview column.
        self.gripper_state_var.set(
            f"Absolute pose   {p.compact()}\n"
            f"Gripper   {grip_label:<6} {closed_pct:>3}% closed"
        )
        self._draw_gripper_bar(closed_pct, grip_label)

    def _draw_gripper_bar(self, closed_pct: int, grip_label: str):
        bar = getattr(self, "gripper_bar", None)
        if bar is None:
            return
        bar.delete("all")
        w = int(bar.cget("width"))
        h = int(bar.cget("height"))
        pad = 1
        bar.create_rectangle(pad, pad, w - pad, h - pad, fill=COLORS["track"],
                             outline=COLORS["border"])
        fill_w = (w - 2 * pad) * (closed_pct / 100.0)
        if fill_w > 1:
            bar.create_rectangle(pad, pad, pad + fill_w, h - pad,
                                 fill=COLORS["accent"], outline="")
        bar.create_text(8, h / 2, text="open", anchor="w", fill=COLORS["muted"],
                        font=("Segoe UI", 7))
        bar.create_text(w - 8, h / 2, text="closed", anchor="e", fill=COLORS["muted"],
                        font=("Segoe UI", 7))

    def _set_pose_on_sliders(self, p: Pose, update_target: bool = True):
        # Sync UI slider positions from absolute pose.
        rel_targets = {
            "base": p.base - HOME_POSE.base,
            "shoulder": p.shoulder - HOME_POSE.shoulder,
            "elbow": p.elbow - HOME_POSE.elbow,
            "wristV": p.wristV - HOME_POSE.wristV,
            "wristR": p.wristR - HOME_POSE.wristR,
            "gripper": p.gripper - HOME_POSE.gripper,
        }

        for name in ARM_JOINTS:
            self._scale_guard[name] = True
        self._scale_guard["speed"] = True
        try:
            for name in ARM_JOINTS:
                lo, hi = UI_LIMITS[name]
                val = max(lo, min(hi, int(rel_targets[name])))
                self.vars[name].set(val)
                self.scales[name].set(val)

            speed = clamp("speed", p.speed)
            self.vars["speed"].set(speed)
            self.scales["speed"].set(speed)
        finally:
            for name in self._scale_guard:
                self._scale_guard[name] = False

        if update_target:
            self._active_drag = None
            self._sync_target_from_sliders()
            self._commit_preview()

    # ---------- Connection and hardware send ----------
    def _refresh_ports(self):
        ports = list_serial_ports()
        self.port_combo["values"] = ports
        cur = self.port_var.get()
        if cur not in ports:
            self.port_var.set(ports[0] if ports else "")
        if serial is None:
            self._log("Ports: pyserial not installed. Install with: pip install pyserial")
        else:
            self._log(f"Ports: {ports if ports else '(none)'}")

    def _toggle_connect(self):
        if self.serial_mgr.is_connected():
            self._stop_play()
            self.serial_mgr.disconnect()
            self.status_var.set("Disconnected")
            self.status_lbl.configure(foreground="red")
            self.conn_btn.configure(text="Connect")
            self._update_system_state()
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("No port", "Select a COM port first.")
            return

        if self.serial_mgr.connect(port, 115200):
            self.status_var.set(f"Connected: {port}")
            self.status_lbl.configure(foreground="green")
            self.conn_btn.configure(text="Disconnect")
            self._update_system_state()
            if self.auto_home_var.get():
                self._home_to_start_pose(reason="home-on-connect")
            else:
                self._send_current(reason="connect-sync", force=True)
                self._log("Connected without auto-home (checkbox off).")
        else:
            messagebox.showerror("Connect failed", "Could not connect. Check port, pyserial installation, and permissions.")
            self._update_system_state()

    def _on_hardware_send_toggle(self):
        enabled = self.hardware_send_var.get()
        if not enabled:
            self._cancel_pending_send()
        self._log(f"Hardware send {'enabled' if enabled else 'disabled'}.")
        self._update_system_state()

    def _on_live_drag_toggle(self):
        if self.live_drag_var.get() and not self.hardware_send_var.get():
            self._log("Live drag is on, but hardware send is disabled. Preview will still update; hardware will not move.")
        elif self.live_drag_var.get():
            self._log(f"Live drag send enabled with ~{DEFAULT_SEND_INTERVAL_MS} ms throttle.")
        else:
            self._log("Live drag send disabled. Sliders send on release/nudge/manual send only.")

    def _home_to_start_pose(self, reason: str = "home-manual"):
        self._set_pose_on_sliders(HOME_POSE)
        self._send_current(reason=reason, force=True)
        if self.serial_mgr.is_connected() and self.hardware_send_var.get():
            self._log("Homed to slider start pose (0 deg for all joints).")
        else:
            self._log("Home requested: sliders/simulator reset to home pose. Hardware was not moved.")

    def _cancel_pending_send(self):
        if self._pending_send_after_id is not None:
            try:
                self.after_cancel(self._pending_send_after_id)
            except Exception:
                pass
        self._pending_send_after_id = None
        self._pending_send_pose = None
        self._pending_send_reason = ""

    def _send_current(self, reason: str, force: bool = False):
        p = self._current_pose()
        self.target_pose = p
        self._update_pose_readout()
        self._request_send(p, reason=reason, force=force)

    def _request_send(self, p: Pose, reason: str, force: bool = False):
        if self.play_thread and self.play_thread.is_alive():
            self._log(f"Ignored manual send ({reason}): playback running")
            return

        if not self.hardware_send_var.get():
            self.last_commanded = p
            self._log(f"SIM ONLY {reason}: {p.to_line().strip()}")
            return

        if not self.serial_mgr.is_connected():
            self._log(f"Skip send ({reason}): not connected")
            return

        if force:
            self._cancel_pending_send()
            self._send_pose_now(p, reason)
            return

        now = time.time()
        elapsed_ms = (now - self._last_send_time) * 1000.0
        if elapsed_ms >= DEFAULT_SEND_INTERVAL_MS:
            self._cancel_pending_send()
            self._send_pose_now(p, reason)
        else:
            self._pending_send_pose = p
            self._pending_send_reason = reason
            if self._pending_send_after_id is None:
                delay = max(1, int(DEFAULT_SEND_INTERVAL_MS - elapsed_ms))
                self._pending_send_after_id = self.after(delay, self._flush_pending_send)

    def _flush_pending_send(self):
        self._pending_send_after_id = None
        p = self._pending_send_pose
        reason = self._pending_send_reason or "throttled"
        self._pending_send_pose = None
        self._pending_send_reason = ""
        if p is not None:
            self._send_pose_now(p, reason)

    def _send_pose_now(self, p: Pose, reason: str) -> bool:
        if not self.hardware_send_var.get():
            self.last_commanded = p
            self._log(f"SIM ONLY {reason}: {p.to_line().strip()}")
            return True

        if not self.serial_mgr.is_connected():
            self._log(f"Skip send ({reason}): not connected")
            return False

        if self.serial_mgr.send_pose(p):
            self._last_send_time = time.time()
            self.last_commanded = p
            self._log(f"TX {reason}: {p.to_line().strip()}")
            return True
        return False

    def _update_system_state(self):
        connected = self.serial_mgr.is_connected()
        playing = self.play_thread is not None and self.play_thread.is_alive() and not self.play_stop.is_set()
        hw_enabled = self.hardware_send_var.get()

        dark = getattr(self, "theme_name", "light") == "dark"
        if playing:
            text = "PLAYBACK RUNNING"
            bg, fg = ("#4b3714", "#f7d48a") if dark else ("#fef3c7", "#92400e")
        elif connected and hw_enabled:
            text = "LIVE READY"
            bg, fg = ("#183a2b", "#85e0ad") if dark else ("#dcfce7", "#166534")
        elif connected and not hw_enabled:
            text = "SIM ONLY - CONNECTED"
            bg, fg = ("#17364b", "#8bd4f4") if dark else ("#e0f2fe", "#075985")
        elif not connected and hw_enabled:
            # Disconnected is expected during simulation; use an amber warning,
            # not an error-red alarm that implies the app itself has failed.
            text = "SIM READY - DISCONNECTED"
            bg, fg = ("#3f3020", "#f6c987") if dark else ("#fff7ed", "#9a3412")
        else:
            text = "SIMULATION ONLY"
            bg, fg = ("#303846", "#cbd5e1") if dark else ("#e5e7eb", "#374151")

        self.system_state_var.set(text)
        try:
            self.system_badge.configure(bg=bg, fg=fg)
        except Exception:
            pass

    # ---------- Recording ----------
    @staticmethod
    def _step_row_values(p: Pose) -> tuple:
        return (p.base, p.shoulder, p.elbow, p.wristV, p.wristR, p.gripper, p.speed)

    def _update_step_buttons(self):
        n = len(self.recorded)
        try:
            self.rec_frame.configure(text=f"  Recorded Steps ({n})  ")
        except Exception:
            pass
        if n > 0:
            self.steps_empty_lbl.grid_remove()
            self.steps_tree.grid(row=1, column=0, sticky="nsew")
        else:
            self.steps_tree.grid_remove()
            self.steps_empty_lbl.grid(row=1, column=0, sticky="nw", pady=(4, 0))
        playing = self.play_thread is not None and self.play_thread.is_alive()
        state = "normal" if (n > 0 and not playing) else "disabled"
        for b in (self.move_up_btn, self.move_down_btn, self.dup_btn, self.del_step_btn):
            try:
                b.configure(state=state)
            except Exception:
                pass

    def _refresh_steps_view(self, select_pos: Optional[int] = None):
        self.steps_tree.delete(*self.steps_tree.get_children())
        for i, p in enumerate(self.recorded):
            tag = "even" if i % 2 else "odd"
            self.steps_tree.insert("", "end", iid=str(i),
                                   values=(f"{i + 1:02d}",) + self._step_row_values(p),
                                   tags=(tag,))
        self._update_step_buttons()
        if self.recorded:
            self.tree_sb.grid()
            self.tree_hsb.grid()
        else:
            self.tree_sb.grid_remove()
            self.tree_hsb.grid_remove()
        self._update_duration()
        if select_pos is not None and 0 <= select_pos < len(self.recorded):
            self._select_step_at(select_pos)

    def _select_step_at(self, pos: int):
        """Move the selection programmatically (never loads the pose)."""
        iid = str(pos)
        if not self.steps_tree.exists(iid):
            return
        self.steps_tree.selection_set(iid)
        self.steps_tree.focus(iid)
        self.steps_tree.see(iid)

    def _add_step(self):
        p = self._current_pose()
        self.recorded.append(p)
        self._refresh_steps_view(select_pos=len(self.recorded) - 1)
        self._log(f"Recorded step #{len(self.recorded)}: {p.to_line().strip()}")

    def _selected_index(self) -> Optional[int]:
        sel = self.steps_tree.selection()
        if not sel:
            return None
        try:
            return self.steps_tree.index(sel[0])
        except Exception:
            return None

    def _on_step_activate(self, event=None):
        """User clicked or arrowed onto a step: load it into the sliders."""
        if self._block_key_load:
            return
        if self.play_thread and self.play_thread.is_alive():
            return
        idx = self._selected_index()
        if idx is None or idx >= len(self.recorded):
            return
        p = self.recorded[idx]
        self._set_pose_on_sliders(p)
        self.playback_status_var.set(
            f"Loaded step {idx + 1}/{len(self.recorded)} into sliders (not sent).")
        self._log(f"Preview loaded step #{idx + 1}: {p.to_line().strip()}")

    def _move_step(self, delta: int):
        if self.play_thread and self.play_thread.is_alive():
            return
        idx = self._selected_index()
        if idx is None:
            return
        j = idx + delta
        if j < 0 or j >= len(self.recorded):
            return
        self.recorded[idx], self.recorded[j] = self.recorded[j], self.recorded[idx]
        self._refresh_steps_view(select_pos=j)
        # Suppress the key-release pose-load that follows a Ctrl+Up/Down reorder.
        self._block_key_load = True
        self.after(250, lambda: setattr(self, "_block_key_load", False))
        self._log(f"Reordered step {idx + 1} → {j + 1}")

    def _duplicate_selected(self):
        if self.play_thread and self.play_thread.is_alive():
            return
        idx = self._selected_index()
        if idx is None:
            self._log("No step selected to duplicate.")
            return
        p = self.recorded[idx]
        dup = Pose(p.base, p.shoulder, p.elbow, p.wristV, p.wristR, p.gripper, p.speed)
        self.recorded.insert(idx + 1, dup)
        self._refresh_steps_view(select_pos=idx + 1)
        self._log(f"Duplicated step #{idx + 1}")

    def _delete_selected(self):
        if self.play_thread and self.play_thread.is_alive():
            return
        idx = self._selected_index()
        if idx is None:
            self._log("No step selected to delete.")
            return
        removed = self.recorded.pop(idx)
        new_sel = min(idx, len(self.recorded) - 1) if self.recorded else None
        self._refresh_steps_view(select_pos=new_sel)
        self._log(f"Deleted step: {removed}")

    def _clear_steps(self):
        self._stop_play()
        self.recorded.clear()
        self._refresh_steps_view()
        self.playback_status_var.set("Idle - no recorded steps.")
        self._log("Cleared steps")

    # Backwards-compatible alias for the few call sites that rebuild the list.
    def _rebuild_listbox(self):
        self._refresh_steps_view()

    # ---------- Save / Load ----------
    def _save_steps(self):
        if not self.recorded:
            self._log("No steps to save.")
            return

        file_path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            title="Save Braccio steps (CSV)",
        )
        if not file_path:
            return

        try:
            with open(file_path, "w", encoding="utf-8") as f:
                for p in self.recorded:
                    f.write(p.to_line())
            self._log(f"Saved {len(self.recorded)} step(s) to {os.path.basename(file_path)}")
        except Exception as e:
            self._log(f"ERR saving: {e}")

    def _parse_pose_line(self, line: str) -> Optional[Pose]:
        parts = [p.strip() for p in line.split(",")]
        if len(parts) != 7:
            return None
        try:
            nums = [int(x) for x in parts]
        except ValueError:
            return None
        return Pose(            base=clamp("base", nums[0]),
            shoulder=clamp("shoulder", nums[1]),
            elbow=clamp("elbow", nums[2]),
            wristV=clamp("wristV", nums[3]),
            wristR=clamp("wristR", nums[4]),
            gripper=clamp("gripper", nums[5]),
            speed=clamp("speed", nums[6]),
        )

    def _load_steps(self):
        file_path = filedialog.askopenfilename(
            filetypes=[("CSV files", "*.csv")],
            title="Load Braccio steps (CSV)",
        )
        if not file_path:
            return

        try:
            with open(file_path, "r", encoding="utf-8") as f:
                lines = [ln.strip() for ln in f.readlines() if ln.strip()]

            new_list: List[Pose] = []
            skipped = 0
            for line in lines:
                pose = self._parse_pose_line(line)
                if pose is None:
                    skipped += 1
                else:
                    new_list.append(pose)

            self._stop_play()
            self.recorded = new_list
            self._rebuild_listbox()
            self.playback_status_var.set(f"Loaded {len(new_list)} step(s).")
            self._log(f"Loaded {len(new_list)} step(s) from {os.path.basename(file_path)} (skipped {skipped})")
        except Exception as e:
            self._log(f"ERR loading: {e}")

    # ---------- Paste CSV steps ----------
    def _paste_and_add_steps(self):
        raw = self.paste_text.get("1.0", "end").strip()
        if not raw:
            self._log("No text to paste.")
            return

        lines = [ln.strip() for ln in raw.splitlines() if ln.strip()]
        added, skipped = 0, 0

        for line in lines:
            pose = self._parse_pose_line(line)
            if pose is None:
                skipped += 1
                continue
            self.recorded.append(pose)
            added += 1

        if added:
            self._rebuild_listbox()
            self.playback_status_var.set(f"Added {added} pasted step(s).")
        self._log(f"Paste result: added={added}, skipped={skipped}")

    # ---------- Dynamic playback ----------
    def _sleep_responsive(self, ms: int):
        end = time.time() + (ms / 1000.0)
        while time.time() < end:
            if self.play_stop.is_set():
                break
            if self.play_resume.is_set():
                time.sleep(0.01)
                end += 0.01
                continue
            time.sleep(0.01)

    def _max_delta_deg(self, a: Pose, b: Pose) -> int:
        return max(
            abs(b.base - a.base),
            abs(b.shoulder - a.shoulder),
            abs(b.elbow - a.elbow),
            abs(b.wristV - a.wristV),
            abs(b.wristR - a.wristR),
            abs(b.gripper - a.gripper),
        )

    def _get_int(self, var: tk.IntVar, default: int) -> int:
        try:
            return int(var.get())
        except Exception:
            var.set(default)
            return default

    def _set_playback_controls_enabled(self, enabled: bool):
        state = "normal" if enabled else "disabled"
        for widget in self.lock_during_playback_widgets:
            try:
                widget.configure(state=state)
            except Exception:
                pass
        try:
            self.stop_btn.configure(state="normal")
        except Exception:
            pass

    def _apply_playback_target(self, target: Pose, index: int, total: int):
        self._set_pose_on_sliders(target)
        self.playback_status_var.set(f"Playing step {index}/{total}: {target.compact()}")
        # Move the selection to act as a playhead (selecting during playback does
        # not load/preview because _on_step_activate returns early while playing,
        # and programmatic selection never triggers a load anyway).
        self._select_step_at(index - 1)
        self._update_system_state()

    def _on_playback_finished_ui(self, message: str = "Playback stopped"):
        self.playback_status_var.set(message)
        self._set_playback_controls_enabled(True)
        self.play_resume.clear()
        if hasattr(self, "pause_btn"):
            self.pause_btn.configure(text="Pause playback")
        self._update_step_buttons()
        self._update_system_state()

    def _update_duration(self):
        current = self.last_commanded
        total = 0
        for target in self.recorded:
            total += max(max(0, self._get_int(self.base_delay_var, 500)), self._max_delta_deg(current, target) * max(1, self._get_int(self.ms_per_deg_var, 18))) + max(0, self._get_int(self.settle_var, 200))
            current = target
        self.duration_var.set(f"Estimated cycle: {total / 1000:.1f} s")

    def _toggle_pause(self):
        if not self.play_thread or not self.play_thread.is_alive():
            return
        if self.play_resume.is_set():
            self.play_resume.clear()
            self.pause_btn.configure(text="Pause playback")
            self.playback_status_var.set("Playback resumed")
        else:
            self.play_resume.set()
            self.pause_btn.configure(text="Resume playback")
            self.playback_status_var.set("Playback paused")

    def _start_play_loop(self, once=False):
        if self.play_thread and self.play_thread.is_alive():
            self._log("Playback already running")
            return

        hardware_enabled = self.hardware_send_var.get()
        if hardware_enabled and not self.serial_mgr.is_connected():
            self._log("ERR: Not connected. Disable 'Hardware send enabled' to run simulation-only playback.")
            messagebox.showwarning(
                "Not connected",
                "Connect to the Braccio first, or untick 'Hardware send enabled' to run simulation-only playback.",
            )
            return

        if not self.recorded:
            self._log("ERR: No steps recorded")
            return

        base_delay = max(0, self._get_int(self.base_delay_var, 500))
        settle = max(0, self._get_int(self.settle_var, 200))
        ms_per_deg = max(1, self._get_int(self.ms_per_deg_var, 18))
        steps = list(self.recorded)
        for index, pose in enumerate(steps, 1):
            for field, (lo, hi) in LIMITS.items():
                if not lo <= getattr(pose, field) <= hi:
                    self._log(f"ERR preflight: step {index} {field} out of range ({lo}..{hi})")
                    messagebox.showerror("Invalid sequence", f"Step {index}: {field} outside {lo}..{hi}")
                    return
        self.play_once = once
        self.play_resume.clear()
        self._update_duration()

        self._cancel_pending_send()
        self.play_stop.clear()
        self._set_playback_controls_enabled(False)
        self.playback_status_var.set(f"Starting playback... 0/{len(steps)}")
        self._update_system_state()

        def worker():
            mode = "hardware" if hardware_enabled else "simulation-only"
            self._log(
                f"Playback start ({mode}) | steps={len(steps)} | "
                f"base={base_delay}ms settle={settle}ms ms/deg={ms_per_deg}"
            )

            current = self.last_commanded
            stopped_message = "Playback stopped"

            while not self.play_stop.is_set():
                for i, target in enumerate(steps, start=1):
                    if self.play_stop.is_set():
                        break
                    while self.play_resume.is_set() and not self.play_stop.is_set():
                        time.sleep(0.01)
                    if self.play_stop.is_set():
                        break

                    # Post the UI update to the main thread instead of calling
                    # Tkinter from this worker thread (Tk is not thread-safe).
                    self.ui_q.put(("step", target, i, len(steps)))

                    ok = True
                    if hardware_enabled:
                        ok = self.serial_mgr.send_pose(target)
                    if not ok:
                        self._log("Playback stopped (send failure)")
                        self.play_stop.set()
                        stopped_message = "Playback stopped - send failure"
                        break

                    maxd = self._max_delta_deg(current, target)
                    wait_ms = max(base_delay, maxd * ms_per_deg) + settle

                    if hardware_enabled:
                        self._log(f"TX loop#{i}: {target.to_line().strip()} | WAIT {wait_ms}ms (maxΔ={maxd})")
                    else:
                        self._log(f"SIM loop#{i}: {target.to_line().strip()} | WAIT {wait_ms}ms (maxΔ={maxd})")

                    self.last_commanded = target
                    self._sleep_responsive(wait_ms)
                    current = target
                if self.play_once and not self.play_stop.is_set():
                    stopped_message = "Playback completed (once)"
                    break

            self._log(stopped_message)
            # Hand the "finished" cleanup back to the main thread so the UI is
            # always unlocked even if a step send failed mid-loop.
            self.ui_q.put(("done", stopped_message))

        self.play_thread = threading.Thread(target=worker, daemon=True)
        self.play_thread.start()
        self.after(80, self._update_system_state)

    def _stop_play(self):
        self.play_stop.set()
        self.play_resume.clear()
        if self.play_thread and self.play_thread.is_alive():
            self.playback_status_var.set("Stopping playback...")

    # ---------- Close ----------
    def _on_close(self):
        self._closing = True
        try:
            self._stop_play()
        except Exception:
            pass
        try:
            self._cancel_pending_send()
        except Exception:
            pass
        try:
            self.serial_mgr.disconnect()
        except Exception:
            pass
        for timer_id in self.tk.call("after", "info"):
            try:
                self.after_cancel(timer_id)
            except tk.TclError:
                pass
        self.destroy()


if __name__ == "__main__":
    App().mainloop()