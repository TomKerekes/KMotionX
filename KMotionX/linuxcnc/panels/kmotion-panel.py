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
        self.stop_line = 0              # the last line LinuxCNC reported while the program ran
        self.stop_paused = False        # ... and whether the program was paused (M0) then
        self.file_lines = []            # the program's lines, for the M0 after the stop line
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
        # LinuxCNC's current line is the last line that EXECUTED: at an M0 pause it is the
        # motion before the M0 (the interpreter has read ahead to the end of the file by
        # then), after an abort it is meaningless. So: the last line seen while the program
        # ran, and whether it was paused there
        line = max(self.stat.current_line, self.stat.motion_line)
        if self.stat.interp_state != linuxcnc.INTERP_IDLE:
            if line > 0 and self.stat.file:
                self.stop_line = line
                self.stop_paused = bool(self.stat.paused)
                if self.stat.file != self.stop_file:
                    self.stop_file = self.stat.file
                    path = self.stat.file
                    if not os.path.isabs(path):
                        path = os.path.join(os.path.dirname(os.environ.get("INI_FILE_NAME", "")), path)
                    try:
                        with open(path) as f:
                            self.file_lines = f.read().splitlines()
                    except Exception:
                        self.file_lines = []
            self.cont.set_sensitive(False)
        else:
            self.cont.set_sensitive(self._continue_line() > 0 and self.stat.file == self.stop_file
                                    and self.stat.task_state == linuxcnc.STATE_ON)
        return True

    def _continue_line(self):
        """the line to run from: after a pause, the line after the M0 (or M1) that follows the
        last executed line; after a halt in a move, that move's line again. 0: nothing to continue"""
        if self.stop_line <= 0 or not self.file_lines:
            return 0
        start = self.stop_line
        if self.stop_paused:
            start = self.stop_line + 1
            for n in range(self.stop_line, len(self.file_lines)):          # lines are 1-based: index n is line n+1
                if re.match(r"\s*[Mm]0*[01]\b", self.file_lines[n]):
                    start = n + 2
                    break
        return start if start <= len(self.file_lines) else 0

    def on_continue_clicked(self, widget):
        start = self._continue_line()
        if start <= 0:
            return
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
