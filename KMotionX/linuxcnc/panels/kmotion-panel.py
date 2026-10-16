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
from gi.repository import Gtk


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
        builder.get_object("geo_row").set_visible(show_geo or show_measure)

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

    def _on_hold_allowed(self, pin):
        on = bool(pin.get())
        self.back.set_sensitive(on)
        self.fwd.set_sensitive(on)


def get_handlers(halcomp, builder, useropts):
    return [KMotionPanel(halcomp, builder, useropts)]
