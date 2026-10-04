"""The Axis screen: per axis the commanded destination, the measured position, Enable,
the input and output modes and Done, as KMotion.exe's Axis status screen shows them.
Enable toggles the axis (EnableAxis / DisableAxis) without touching its parameters."""
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QWidget, QGridLayout, QLabel, QCheckBox, QVBoxLayout, QHBoxLayout
import kmx

# the names KMotion.exe shows (TranslateInputMode / TranslateOutputMode in PC-DSP.h)
INPUT_MODES = {0: "No Input", 1: "Encoder", 2: "ADC", 3: "Resolver", 4: "User Input", 5: "Serial Servo"}
OUTPUT_MODES = {0: "No Output", 1: "Microstep", 2: "DC Servo", 3: "3PH Servo", 4: "4PH Servo", 5: "DAC Servo",
                6: "Step Dir", 7: "CL Step", 8: "CL Micro", 9: "Serial Servo", 10: "CL Serial Servo"}
RESOLVER_MODE = 3


def input_mode(st, i):
    words = (st.InputModes, st.InputModes2, st.InputModes3, st.InputModes4)
    return (words[i // 8] >> (4 * (i % 8))) & 0xf


def output_mode(st, i):
    words = (st.OutputModes, st.OutputModes2, st.OutputModes3, st.OutputModes4)
    return (words[i // 8] >> (4 * (i % 8))) & 0xf


def dest_text(d):
    if abs(d) < 1e-4:
        d = 0.0                                   # no -0.00
    if abs(d) < 1e10:
        return f"{d:14.2f}"
    if abs(d) < 1e11:
        return f"{d:14.1f}"
    if abs(d) < 1e13:
        return f"{d:14.0f}"
    return f"{d:14e}"


def position_text(d, resolver):
    if resolver:
        return f"{d:13.4f}"
    if abs(d) < 1e12:
        return f"{d:13.0f}"
    return f"{d:13g}"


class AxisScreen(QWidget):
    def __init__(self, km, board_type):
        super().__init__()
        self.km = km
        self.n = 16 if board_type == kmx.BOARD_TYPE_KOGNA else 8
        self.rows = []
        self.last_text = {}
        outer = QVBoxLayout(self)
        g = QGridLayout()
        g.setHorizontalSpacing(18)
        g.setVerticalSpacing(2)
        mono = QFont("monospace")
        mono.setStyleHint(QFont.TypeWriter)
        for c, title in enumerate(("Axis", "Dest", "Position", "Enable", "Input", "Output", "Done")):
            l = QLabel(title)
            l.setStyleSheet("font-weight: bold")
            g.addWidget(l, 0, c)
        for i in range(self.n):
            dest = QLabel("0"); dest.setFont(mono)
            pos = QLabel("0"); pos.setFont(mono)
            en = QCheckBox()
            en.setToolTip("enable or disable the axis without downloading any parameters")
            en.clicked.connect(lambda checked, ch=i: self.on_enable(ch, checked))
            imode = QLabel("")
            omode = QLabel("")
            done = QCheckBox()
            done.setAttribute(Qt.WA_TransparentForMouseEvents)
            done.setFocusPolicy(Qt.NoFocus)
            g.addWidget(QLabel(f"{i}"), i + 1, 0)
            g.addWidget(dest, i + 1, 1)
            g.addWidget(pos, i + 1, 2)
            g.addWidget(en, i + 1, 3, alignment=Qt.AlignHCenter)
            g.addWidget(imode, i + 1, 4)
            g.addWidget(omode, i + 1, 5)
            g.addWidget(done, i + 1, 6, alignment=Qt.AlignHCenter)
            self.rows.append((dest, pos, en, imode, omode, done))
        g.setRowStretch(self.n + 1, 1)
        g.setColumnStretch(7, 1)
        outer.addLayout(g)
        outer.addStretch(1)

    def on_enable(self, ch, checked):
        try:
            self.km.write(f"{'EnableAxis' if checked else 'DisableAxis'}{ch}")
        except kmx.KMotionError:
            pass

    def _set(self, label, key, text):
        if self.last_text.get(key) != text:
            self.last_text[key] = text
            label.setText(text)

    def update_status(self, st):
        for i, (dest, pos, en, imode, omode, done) in enumerate(self.rows):
            im = input_mode(st, i)
            self._set(dest, ("d", i), dest_text(st.Dest[i]))
            self._set(pos, ("p", i), position_text(st.Position[i], im == RESOLVER_MODE))
            self._set(imode, ("i", i), INPUT_MODES.get(im, str(im)))
            self._set(omode, ("o", i), OUTPUT_MODES.get(output_mode(st, i), "?"))
            enabled = bool((st.Enables >> i) & 1)
            if en.isChecked() != enabled:
                en.blockSignals(True)
                en.setChecked(enabled)
                en.blockSignals(False)
            d = bool((st.AxisDone >> i) & 1)
            if done.isChecked() != d:
                done.setChecked(d)
