# kmotion-panel.py: gladevcp handler for kmotion-panel.ui (the -u option of the config's
# EMBED_TAB_COMMAND). Hides the Geo reload button when the config has no [KMOTION] GEO_FILE,
# shows the Measure button only with [KMOTION] MEASURE_BUTTON = 1, and keeps Back/Forward
# grey unless kmotion-motion says a hold jog is possible (its pin kmotion.hold-allowed, netted
# to this panel's hold-allowed in kmotion-panel.hal: the program paused, board mode, machine on)
import os
import re
import hal
import linuxcnc
from hal_glib import GPin
from gi.repository import Gtk, GLib


class KMotionPanel:
    def __init__(self, halcomp, builder, useropts):
        self.builder = builder
        self.back = builder.get_object("hold-reverse")
        self.fwd = builder.get_object("hold-forward")
        self.back.set_sensitive(False)
        self.fwd.set_sensitive(False)
        self.hold_allowed = GPin(halcomp.newpin("hold-allowed", hal.HAL_BIT, hal.HAL_IN))
        self.hold_allowed.connect("value-changed", self._on_hold_allowed)
        geo = measure = ""
        try:
            ini = linuxcnc.ini(os.environ["INI_FILE_NAME"])
            geo = (ini.find("KMOTION", "GEO_FILE") or "").strip()
            measure = (ini.find("KMOTION", "MEASURE_BUTTON") or "0").strip()
            self._follow_gmoccapy_theme(ini)
        except Exception:
            pass
        show_geo = geo != ""
        show_measure = measure not in ("", "0")
        builder.get_object("geo-reload").set_visible(show_geo)
        builder.get_object("measure").set_visible(show_measure)
        builder.get_object("geo_row").set_visible(True)
        # Continue (KMotionCNC's Go after a halt): LinuxCNC cannot jog a paused program, so
        # the geo measurement goes halt, jog, Measure, Continue - the program runs again from
        # the line after the one it stopped on (LinuxCNC's run-from-line, without the dialog),
        # from wherever the pen stands. The stop line is watched while the program runs
        self.cont = builder.get_object("continue")
        self.stat = linuxcnc.stat()
        self.cmd = linuxcnc.command()
        self.stop_line = 0
        self.file_lines = 0
        self.stop_file = ""
        GLib.timeout_add(100, self._watch_program)

    def _follow_gmoccapy_theme(self, ini):
        """the panel is its own process (gladevcp, embedded by window id) and would take the
        desktop's GTK theme: use the one gmoccapy chose, from its preference file
        ([DISPLAY] PREFERENCE_FILE_PATH, relative to the ini; gtk_theme = <name>, or
        "Follow System Theme"). Read at start: a theme changed on gmoccapy's settings page
        reaches the panel at the next start"""
        pref = (ini.find("DISPLAY", "PREFERENCE_FILE_PATH") or "").strip()
        if not pref:
            return
        if not os.path.isabs(pref):
            pref = os.path.join(os.path.dirname(os.environ["INI_FILE_NAME"]), pref)
        with open(pref) as f:
            m = re.search(r"^\s*gtk_theme\s*=\s*(.+?)\s*$", f.read(), re.M)
        if m and m.group(1) != "Follow System Theme":
            Gtk.Settings.get_default().set_property("gtk-theme-name", m.group(1))

    def _watch_program(self):
        try:
            self.stat.poll()
        except Exception:
            return True
        line = max(self.stat.current_line, self.stat.motion_line)
        if self.stat.interp_state != linuxcnc.INTERP_IDLE:
            if line > 0 and self.stat.file:
                self.stop_line = line
                if self.stat.file != self.stop_file:
                    self.stop_file = self.stat.file
                    try:
                        with open(self.stat.file) as f:
                            self.file_lines = sum(1 for _ in f)
                    except Exception:
                        self.file_lines = 0
            self.cont.set_sensitive(False)
        else:
            self.cont.set_sensitive(self.stop_line > 0 and self.stop_line < self.file_lines and self.stat.file == self.stop_file
                                    and self.stat.task_state == linuxcnc.STATE_ON)
        return True

    def on_continue_clicked(self, widget):
        if self.stop_line <= 0:
            return
        start = self.stop_line + 1
        self.stop_line = 0
        try:
            self.cmd.mode(linuxcnc.MODE_AUTO)
            self.cmd.wait_complete()
            self.cmd.auto(linuxcnc.AUTO_RUN, start)
        except Exception:
            pass

    def _on_hold_allowed(self, pin):
        on = bool(pin.get())
        self.back.set_sensitive(on)
        self.fwd.set_sensitive(on)


def get_handlers(halcomp, builder, useropts):
    return [KMotionPanel(halcomp, builder, useropts)]
