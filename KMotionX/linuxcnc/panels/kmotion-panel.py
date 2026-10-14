# kmotion-panel.py: gladevcp handler for kmotion-panel.ui (the -u option of the config's
# EMBED_TAB_COMMAND). Hides the Geo reload button when the config has no [KMOTION] GEO_FILE,
# shows the Measure button only with [KMOTION] MEASURE_BUTTON = 1, and keeps Back/Forward
# grey unless kmotion-motion says a hold jog is possible (its pin kmotion.hold-allowed, netted
# to this panel's hold-allowed in kmotion-panel.hal: the program paused, board mode, machine on)
import os
import hal
import linuxcnc
from hal_glib import GPin


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
        except Exception:
            pass
        show_geo = geo != ""
        show_measure = measure not in ("", "0")
        builder.get_object("geo-reload").set_visible(show_geo)
        builder.get_object("measure").set_visible(show_measure)
        builder.get_object("geo_row").set_visible(show_geo or show_measure)

    def _on_hold_allowed(self, pin):
        on = bool(pin.get())
        self.back.set_sensitive(on)
        self.fwd.set_sensitive(on)


def get_handlers(halcomp, builder, useropts):
    return [KMotionPanel(halcomp, builder, useropts)]
