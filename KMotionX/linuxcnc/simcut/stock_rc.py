# stock_rc.py : the Stock in a LinuxCNC GUI, from the ini file's [DISPLAY] USER_COMMAND_FILE
# Copyright (c) 2026 DynoMotion Incorporated.  SPDX-License-Identifier: MIT
#
# gmoccapy and AXIS run the file named by [DISPLAY] USER_COMMAND_FILE when they start (gmoccapy
# with its own object as `self`, AXIS with its canvas as `o`; both with the file's path as
# `rcfile`).  This one puts kmotion_stock's StockPart into the preview.  A GUI that has a user
# command file of its own does the same with these two lines:
#
#     import sys; sys.path.insert(0, "<KMotionX>/KMotionX/linuxcnc/simcut"); import kmotion_stock
#     kmotion_stock.install(<the gremlin widget or AXIS's o>, <the linuxcnc.ini object>)
import logging
import os
import sys

_here = os.path.dirname(os.path.abspath(rcfile))                 # noqa: F821 - set by the GUI
if _here not in sys.path:
    sys.path.insert(0, _here)
try:
    import kmotion_stock
    if "self" in dir() and hasattr(self, "widgets") and hasattr(self.widgets, "gremlin"):   # noqa: F821 - gmoccapy
        _widget = self.widgets.gremlin                                                        # noqa: F821
        _ini = getattr(_widget, "inifile", None) or self.get_ini_info.inifile                 # noqa: F821
    elif "o" in dir():                                                                        # AXIS
        _widget = o                                                                           # noqa: F821
        _ini = inifile                                                                        # noqa: F821
    else:
        raise RuntimeError("stock_rc.py: neither gmoccapy (self.widgets.gremlin) nor AXIS (o) is here")
    _part = kmotion_stock.install(_widget, _ini, os.getcwd())
    if "o" in dir() and "root_window" in dir():                                              # AXIS: a Stock menu
        try:
            import axis_stock
            axis_stock.install(root_window, o, s, c, _part)                                   # noqa: F821
        except Exception:
            kmotion_stock.log.exception("AXIS's Stock menu")
    if "self" in dir() and hasattr(self, "widgets") and hasattr(self.widgets, "tooledit1"):      # noqa: F821 - gmoccapy
        import gmoccapy_stock
        gmoccapy_stock.install(self, _part)                                                       # noqa: F821
except Exception:
    logging.getLogger("kmotion_stock").exception("the Stock could not be installed")
