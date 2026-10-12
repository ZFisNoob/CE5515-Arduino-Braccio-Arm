"""Regression tests for the Braccio Tkinter dashboard; run under a display/Xvfb."""
import os
import tempfile
import time
import unittest
from pathlib import Path

import tkinter as tk
import CE5515_Braccio_Final_UI as appmod


class BraccioDashboardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="braccio-tests-")
        appmod.THEME_PREFERENCE_PATH = os.path.join(cls.tmp.name, "theme")

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def setUp(self):
        try:
            self.app = appmod.App()
            self.app.state("normal")
            self.app.geometry("1380x860+0+0")
            self.app.update()
        except tk.TclError as exc:
            self.fail(f"Tk app did not initialize: {exc}")

    def tearDown(self):
        try:
            self.app._on_close()
        except Exception:
            try:
                self.app.destroy()
            except Exception:
                pass

    def test_serial_frame_exactly_seven_integers_with_newline(self):
        p = appmod.Pose(90, 90, 90, 90, 90, 40, 20)
        self.assertEqual(p.to_line(), "90,90,90,90,90,40,20\\n")
        self.assertEqual(len(p.to_line().strip().split(",")), 7)

    def test_serial_write_failure_is_reported_without_protocol_changes(self):
        messages = []
        manager = appmod.SerialManager(messages.append)

        class BrokenSerial:
            is_open = True
            def write(self, _payload):
                raise OSError("mock cable removed")
            def flush(self):
                pass
            def close(self):
                self.is_open = False

        manager.ser = BrokenSerial()
        self.assertFalse(manager.send_pose(appmod.HOME_POSE))
        self.assertTrue(any("mock cable removed" in message for message in messages))
        self.assertEqual(appmod.HOME_POSE.to_line(), "90,90,90,90,90,40,20\\n")

    def test_limits_and_home_pose_remain_compatible(self):
        self.assertEqual(
            (appmod.HOME_POSE.base, appmod.HOME_POSE.gripper, appmod.HOME_POSE.speed),
            (90, 40, 20),
        )
        self.assertEqual(appmod.clamp("shoulder", 0), 15)
        self.assertEqual(appmod.clamp("gripper", 100), 73)
        self.assertEqual(appmod.UI_LIMITS["gripper"], (-30, 33))

    def test_csv_parser_validates_shape_and_clamps_values(self):
        p = self.app._parse_pose_line("200,90,90,90,90,99,99")
        self.assertIsNotNone(p)
        self.assertEqual((p.base, p.gripper, p.speed), (180, 73, 30))
        self.assertIsNone(self.app._parse_pose_line("1,2,3"))
        self.assertIsNone(self.app._parse_pose_line("a,b,c,d,e,f,g"))

    def test_record_duplicate_reorder_delete_and_empty_state(self):
        self.app.hardware_send_var.set(False)
        self.app._add_step()
        self.app._nudge_joint("base", 5)
        self.app._add_step()
        self.assertEqual(len(self.app.recorded), 2)
        self.app._select_step_at(0)
        self.app._duplicate_selected()
        self.assertEqual(len(self.app.recorded), 3)
        self.app._move_step(1)
        self.app._delete_selected()
        self.app._clear_steps()
        self.app.update()
        self.assertEqual(len(self.app.recorded), 0)
        self.assertFalse(self.app.tree_sb.winfo_ismapped())
        self.assertFalse(self.app.tree_hsb.winfo_ismapped())
        self.assertTrue(self.app.steps_empty_lbl.winfo_ismapped())

    def test_light_dark_theme_toggle_recolors_and_persists(self):
        self.assertEqual(self.app.theme_name, "light")
        self.assertEqual(self.app.cget("bg"), appmod.LIGHT_COLORS["bg"])
        self.app._toggle_theme()
        self.app.update()
        self.assertEqual(self.app.theme_name, "dark")
        self.assertEqual(self.app.cget("bg"), appmod.DARK_COLORS["bg"])
        self.assertEqual(Path(appmod.THEME_PREFERENCE_PATH).read_text().strip(), "dark")
        self.assertEqual(self.app.preview_canvas.cget("bg"), appmod.DARK_COLORS["canvas_bg"])
        self.app._toggle_theme()
        self.app.update()
        self.assertEqual(self.app.theme_name, "light")
        self.assertEqual(self.app.theme_toggle_btn.cget("text"), "Switch to dark mode")

    def test_theme_can_be_loaded_from_saved_preference(self):
        Path(appmod.THEME_PREFERENCE_PATH).write_text("dark\\n")
        self.assertEqual(self.app._read_theme_preference(), "dark")

    def test_numeric_entry_not_overwritten_while_focused(self):
        entry = self.app._angle_entries[0]
        entry.focus_force()
        self.app.update()
        entry.delete(0, "end")
        entry.insert(0, "123")
        self.app.vars["gripper"].set(2)
        self.app.update()
        self.assertEqual(entry.get(), "123")

    def test_side_by_side_panels_and_slider_buttons_exist(self):
        self.app.update()
        self.assertGreater(
            self.app.arm_controls_panel.winfo_rootx(), self.app.preview_panel.winfo_rootx()
        )
        self.assertEqual(set(self.app.scales), set(appmod.POSE_FIELDS))
        self.assertTrue(all(w.winfo_exists() for w in self.app._angle_entries))

    def test_slider_nudge_and_numeric_controls_fit_arm_panel(self):
        self.app.geometry("1380x860+0+0")
        self.app.update()
        right_edge = self.app.arm_panel.winfo_rootx() + self.app.arm_panel.winfo_width()
        controls = self.app._plus_buttons + self.app._angle_entries
        self.assertEqual(len(self.app._plus_buttons), 7)
        self.assertEqual(len(self.app._angle_entries), 7)
        self.assertTrue(
            all(w.winfo_rootx() + w.winfo_width() <= right_edge + 2 for w in controls),
            "A + button or numeric-angle entry is clipped past the arm panel",
        )

    def test_preview_draw_and_trajectory_toggle_are_safe(self):
        self.app.hardware_send_var.set(False)
        self.app._add_step()
        self.app._nudge_joint("shoulder", 10)
        self.app._add_step()
        self.app.trajectory_var.set(True)
        self.app._draw_live_preview()
        self.assertGreater(len(self.app.preview_canvas.find_all()), 15)

    def test_simulation_play_once_completes_and_unlocks_controls(self):
        self.app.hardware_send_var.set(False)
        self.app.base_delay_var.set(0)
        self.app.settle_var.set(0)
        self.app.ms_per_deg_var.set(1)
        self.app._add_step()
        self.app._start_play_loop(once=True)
        deadline = time.time() + 2.5
        while time.time() < deadline:
            self.app.update()
            if self.app.playback_status_var.get().startswith("Playback completed"):
                break
            time.sleep(0.01)
        self.app.update()
        self.assertIn("Playback completed", self.app.playback_status_var.get())
        self.assertEqual(str(self.app.add_btn.cget("state")), "normal")

    def test_status_badge_uses_theme_appropriate_semantic_colors(self):
        self.app._update_system_state()
        self.assertEqual(self.app.system_badge.cget("bg"), "#fff7ed")
        self.app._toggle_theme()
        self.app._update_system_state()
        self.assertEqual(self.app.system_badge.cget("bg"), "#3f3020")

    def test_pause_button_label_tracks_pause_and_resume(self):
        self.app.hardware_send_var.set(False)
        self.app.base_delay_var.set(1000)
        self.app.settle_var.set(0)
        self.app._add_step()
        self.app._start_play_loop(once=True)
        deadline = time.time() + 0.5
        while time.time() < deadline and not (self.app.play_thread and self.app.play_thread.is_alive()):
            self.app.update()
            time.sleep(0.01)
        self.app._toggle_pause()
        self.assertEqual(self.app.pause_btn.cget("text"), "Resume playback")
        self.assertTrue(self.app.play_resume.is_set())
        self.app._toggle_pause()
        self.assertEqual(self.app.pause_btn.cget("text"), "Pause playback")
        self.app._stop_play()
        deadline = time.time() + 1.0
        while time.time() < deadline and self.app.play_thread and self.app.play_thread.is_alive():
            self.app.update()
            time.sleep(0.01)
        self.app.update()

    def test_theme_toggle_preserves_pose_and_serial_protocol(self):
        before = self.app._current_pose()
        expected_line = before.to_line()
        self.app._toggle_theme()
        self.app.update()
        self.assertEqual(self.app._current_pose().to_line(), expected_line)
        self.assertEqual(len(expected_line.strip().split(",")), 7)


if __name__ == "__main__":
    unittest.main(verbosity=2)
