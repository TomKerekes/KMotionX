#!/usr/bin/env python3
"""KMotion tool for Linux: the setup and troubleshooting screens of KMotion.exe (Digital
I/O, Axis, C Programs) over KMotionX's library, sharing KMotionServer and the board with
whatever else runs (LinuxCNC's kmotion-motion, kmxWeb).

    python3 kmotion_tool.py            # needs python3-pyside6.qtwidgets and libkmx_c.so (shim/)
"""
import signal
import sys
import time
from pathlib import Path
from PySide6.QtCore import Qt, QThread, Signal, QTimer, QSize
from PySide6.QtGui import QIcon
from PySide6.QtWidgets import (QApplication, QMainWindow, QTabWidget, QLabel, QWidget, QVBoxLayout, QPlainTextEdit,
                               QDockWidget, QCheckBox, QHBoxLayout, QPushButton)
import kmx
from io_screen import DigitalIOScreen
from axis_screen import AxisScreen
from program_screen import ProgramScreen
from config_screen import ConfigScreen
from console_screen import ConsoleScreen


class StatusPoller(QThread):
    """reads the status block ten times a second and hands it to the GUI thread"""
    status = Signal(object)              # a MAIN_STATUS copy
    lost = Signal(str)
    console = Signal(list)
    errors = Signal(list)
    title = Signal(str)                  # the window title, when it changes

    def __init__(self, km, period=0.1):
        super().__init__()
        self.km = km
        self.period = period
        self._stop = False
        self._pause = False
        self._loc = None                 # the board location the title was made from
        self._locs = []                  # the server's board list (a Kogna's serial number)
        self._locs_t = 0.0
        self._title = ""

    def pause(self, on):
        """hold off polling while another thread owns the board for a long operation"""
        self._pause = on

    def stop(self):
        self._stop = True
        self.wait(2000)

    def run(self):
        failures = 0
        while not self._stop:
            if self._pause:
                time.sleep(0.1)
                continue
            t0 = time.time()
            try:
                st = self.km.status()
                failures = 0
                self.status.emit(st)
            except kmx.KMotionError as e:
                failures += 1
                if failures == 3:
                    self.lost.emit(str(e))
                time.sleep(0.5)
            self._update_title()
            lines = self.km.console_lines()
            if lines:
                self.console.emit(lines)
            errs = self.km.errors()
            if errs:
                self.errors.emit(errs)
            dt = self.period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


    def _update_title(self):
        """KMotion.exe's title: "KMotion - Connected - <board>" or "KMotion - Disconnected",
        from the server's own view of the connection (asked every poll; it never reaches the
        board). The board list, for a Kogna's serial number, is read again when the board
        changes, and every 2 s while a Kogna's number is missing from it"""
        loc = self.km.usb_location()
        if -15 <= loc <= 0:
            title = "KMotion - Disconnected"
        else:
            now = time.time()
            kogna = (loc & 0xFFFFFFFF) >= 0x00FFFFFF
            if loc != self._loc or (kogna and " - SN" not in self._title and now - self._locs_t > 2):
                self._locs = self.km.locations()
                self._locs_t = now
            title = "KMotion - Connected - " + kmx.board_string(loc, self._locs)
        self._loc = loc
        if title != self._title:
            self._title = title
            self.title.emit(title)


class MainWindow(QMainWindow):
    def __init__(self, km):
        super().__init__()
        self.km = km
        self.setWindowTitle("KMotion")
        self.board_type = km.board_type()
        board = kmx.BOARD_NAMES.get(self.board_type, "no board")
        self.tabs = QTabWidget()
        self.io = DigitalIOScreen(km, self.board_type)
        self.tabs.addTab(self.io, "Digital I/O")
        self.axis = AxisScreen(km, self.board_type)
        self.tabs.addTab(self.axis, "Axis")
        self.program = ProgramScreen(km, self.board_type, self.log)
        self.tabs.addTab(self.program, "C Programs")
        self.config = ConfigScreen(km, self.board_type, self.log)
        self.tabs.addTab(self.config, "Config && Flash")
        self.config.busy.connect(self.on_busy)
        self.console_screen = ConsoleScreen(km)
        self.tabs.addTab(self.console_screen, "Console")
        self.setCentralWidget(self.tabs)
        # console dock
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(5000)
        box = QWidget()
        v = QVBoxLayout(box)
        v.setContentsMargins(2, 2, 2, 2)
        bar = QHBoxLayout()
        self.capture = QCheckBox("Capture the board's console here (takes it from any other program, e.g. LinuxCNC)")
        self.capture.toggled.connect(self.on_capture)
        clear = QPushButton("Clear")
        clear.clicked.connect(self.console.clear)
        bar.addWidget(self.capture)
        bar.addStretch(1)
        bar.addWidget(clear)
        v.addLayout(bar)
        v.addWidget(self.console)
        dock = QDockWidget("Console", self)
        dock.setWidget(box)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)
        # status bar
        self.sb_board = QLabel(board)
        self.sb_link = QLabel("waiting for status")
        self.statusBar().addWidget(self.sb_board)
        self.statusBar().addPermanentWidget(self.sb_link)
        self.last_status_t = 0
        # polling
        self.poller = StatusPoller(km)
        self.poller.status.connect(self.on_status)
        self.poller.lost.connect(self.on_lost)
        self.poller.console.connect(self.on_console)
        self.poller.errors.connect(self.on_errors)
        self.poller.title.connect(self.setWindowTitle)
        self.poller.start()
        self.age_timer = QTimer(self)
        self.age_timer.timeout.connect(self.on_age)
        self.age_timer.start(500)
        self.resize(1100, 750)

    def on_status(self, st):
        self.last_status_t = time.time()
        if self.board_type <= 0:
            self.board_type = self.km.board_type()
            self.sb_board.setText(kmx.BOARD_NAMES.get(self.board_type, "no board"))
        self.io.update_status(st)
        self.axis.update_status(st)
        self.program.update_status(st)
        self.config.update_status(st)
        self.sb_link.setText(f"status {st.TimeStamp:.1f} s  threads {st.ThreadActive & 0xfe:#04x}  enables {st.Enables & 0xffff:#06x}")

    def on_busy(self, on):
        self.poller.pause(on)
        if on:
            self.last_status_t = 0           # no "no status for N s" while we are deliberately quiet
            self.sb_link.setText("status paused: flash operation in progress")

    def on_lost(self, msg):
        self.sb_link.setText(f"no status: {msg}")

    def on_age(self):
        if self.last_status_t and time.time() - self.last_status_t > 2:
            self.sb_link.setText("no status for %.0f s" % (time.time() - self.last_status_t))

    def log(self, text):
        self.console.appendPlainText(text)

    def on_console(self, lines):
        for l in lines:
            self.console.appendPlainText(l)
        self.console_screen.append_lines(lines)

    def on_errors(self, errs):
        for e in errs:
            self.console.appendPlainText("error: " + e)

    def on_capture(self, on):
        if on:
            self.km.capture_console()

    def closeEvent(self, ev):
        self.poller.stop()
        super().closeEvent(ev)


ICON_SIZES = (16, 32, 48, 64, 128, 256)


def app_icon():
    """KMotion.exe's icon at every size it was drawn at. The small sizes are their own art (a
    bold "DM" at 16), not a downscale of the big one, which turns to mush at taskbar size."""
    icon = QIcon()
    folder = Path(__file__).resolve().parent / "icons"
    for n in ICON_SIZES:
        f = folder / f"kmotion-{n}.png"
        if f.exists():
            icon.addFile(str(f), QSize(n, n))
    return icon


def setup_application(app):
    """the icon for the window, the taskbar and the window switcher. The name becomes the
    window's WM_CLASS, which lets the taskbar tie the window to the kmotion-tool.desktop
    launcher (its StartupWMClass); QSettings keep their own explicit names."""
    app.setApplicationName("kmotion-tool")
    app.setDesktopFileName("kmotion-tool")
    app.setWindowIcon(app_icon())


def main():
    app = QApplication(sys.argv)
    setup_application(app)
    # Ctrl-C in the terminal quits the tool cleanly: Qt's event loop would otherwise only
    # print the KeyboardInterrupt from whatever slot it hit and carry on. The window's
    # half-second timer keeps the interpreter running so the handler gets its turn.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    try:
        km = kmx.KMotion()
    except kmx.KMotionError as e:
        print("kmotion_tool:", e, file=sys.stderr)
        return 1
    w = MainWindow(km)
    w.show()
    rc = app.exec()
    w.poller.stop()
    km.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
