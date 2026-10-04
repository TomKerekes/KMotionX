"""The Digital I/O screen: every I/O bit's state and direction from the status block, a
click on State toggles an output (SetBit / ClearBit), a click on Output flips the
direction (SetBitDirection), the Kogna's HRPWM and SPI pins have their function/GPIO
mux. Laid out as KMotion.exe does it: one tab per hardware group."""
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QWidget, QTabWidget, QScrollArea, QGroupBox, QGridLayout, QHBoxLayout, QVBoxLayout,
                               QLabel, QCheckBox, QRadioButton, QButtonGroup, QSizePolicy)
import kmx

# bits 0-47 as the board labels them
KOGNA_LABELS = {i: "JP7" for i in range(0, 16)}
KOGNA_LABELS.update({i: "Aux0" for i in range(16, 26)})
KOGNA_LABELS.update({i: "Aux1" for i in range(26, 36)})
KOGNA_LABELS.update({i: "Com" for i in range(36, 44)})
KOGNA_LABELS.update({46: "LED0", 47: "LED1"})
KFLOP_LABELS = {0: "Enc 0 A", 1: "Enc 0 B", 2: "Enc 1 A", 3: "Enc 1 B", 4: "Enc 2 A", 5: "Enc 2 B", 6: "Enc 3 A", 7: "Enc 3 B",
                8: "Home 0", 9: "Home 1", 10: "Home 2", 11: "Home 3", 12: "LIM + 0", 13: "LIM - 0", 14: "LIM + 1", 15: "LIM - 1"}
KFLOP_LABELS.update({i: "Aux0" for i in range(16, 26)})
KFLOP_LABELS.update({i: "Aux1" for i in range(26, 36)})
KFLOP_LABELS.update({i: "Com" for i in range(36, 44)})
KFLOP_LABELS.update({46: "LED0", 47: "LED1"})
SPI_NAMES = ["SPI SIMO", "SPI SOMI", "SPI_ENA", "SPI CLK", "SPI SC6", "SPI SC7"]


class BitRow:
    """one I/O bit in a group: number, name, optional Output box, State box"""

    def __init__(self, bit, name, has_dir, settable):
        self.bit, self.name, self.has_dir, self.settable = bit, name, has_dir, settable
        self.dir_box = QCheckBox() if has_dir else None
        self.state_box = QCheckBox()
        if not settable:
            self.state_box.setAttribute(Qt.WA_TransparentForMouseEvents)
            self.state_box.setFocusPolicy(Qt.NoFocus)
        self.mux_group = None            # Kogna HRPWM/SPI pins

    def update(self, st):
        s = kmx.bit_state(st, self.bit)
        if s is not None:
            self.state_box.blockSignals(True)
            self.state_box.setChecked(bool(s))
            self.state_box.blockSignals(False)
        if self.dir_box is not None:
            d = kmx.bit_direction(st, self.bit)
            if d is not None:
                self.dir_box.blockSignals(True)
                self.dir_box.setChecked(bool(d))
                self.dir_box.blockSignals(False)


class BitGroup(QGroupBox):
    def __init__(self, title, rows, km, mux=None):
        super().__init__(title)
        self.km = km
        self.rows = rows
        self.mux = mux                   # None, "hrpwm" or "spi"
        g = QGridLayout(self)
        g.setHorizontalSpacing(10)
        g.setVerticalSpacing(1)
        col = 0
        if mux:
            g.addWidget(QLabel("HRPWM" if mux == "hrpwm" else "SPI"), 0, col); col += 1
            g.addWidget(QLabel("GPIO"), 0, col); col += 1
            if mux == "spi":
                g.addWidget(QLabel("I2C"), 0, col); col += 1
        g.addWidget(QLabel("Bit"), 0, col); col += 1
        g.addWidget(QLabel(""), 0, col); col += 1
        has_dir = any(r.has_dir for r in rows)
        if has_dir:
            g.addWidget(QLabel("Output"), 0, col); col += 1
        g.addWidget(QLabel("State"), 0, col)
        for i, r in enumerate(rows, start=1):
            col = 0
            if mux:
                r.mux_group = QButtonGroup(self)
                r.mux_group.setExclusive(True)
                n_opts = 3 if (mux == "spi" and r.bit >= kmx.KOGNA_SPI0 + 4) else 2
                for opt in range(3 if mux == "spi" else 2):
                    rb = QRadioButton()
                    if opt < n_opts:
                        r.mux_group.addButton(rb, opt)
                        rb.toggled.connect(lambda on, row=r, m=opt: on and self.on_mux(row, m))
                    else:
                        rb.setEnabled(False)
                    g.addWidget(rb, i, col); col += 1
            g.addWidget(QLabel(f"{r.bit:4d}"), i, col); col += 1
            g.addWidget(QLabel(r.name), i, col); col += 1
            if has_dir:
                if r.dir_box is not None:
                    r.dir_box.clicked.connect(lambda checked, row=r: self.on_dir(row, checked))
                    g.addWidget(r.dir_box, i, col)
                col += 1
            r.state_box.clicked.connect(lambda checked, row=r: self.on_state(row, checked))
            g.addWidget(r.state_box, i, col)
        g.setRowStretch(len(rows) + 1, 1)

    def on_state(self, row, checked):
        try:
            self.km.set_bit(row.bit, checked)
        except kmx.KMotionError:
            pass

    def on_dir(self, row, checked):
        try:
            self.km.set_bit_direction(row.bit, checked)
        except kmx.KMotionError:
            pass

    def on_mux(self, row, mode):
        try:
            if self.mux == "hrpwm":
                self.km.set_hrpwm_mode(row.bit - kmx.KOGNA_HRPWM0, mode)
            else:
                self.km.set_spi_mode(row.bit - kmx.KOGNA_SPI0, mode)
        except kmx.KMotionError:
            pass

    def update(self, st):
        for r in self.rows:
            r.update(st)
            if r.mux_group is not None:
                pin = r.bit - kmx.KOGNA_HRPWM0            # HRPWM 0-3, SPI 4-9
                mode = kmx.pin_mux_mode(st, pin)
                b = r.mux_group.button(mode)
                if b is not None and not b.isChecked():
                    r.mux_group.blockSignals(True)
                    for btn in r.mux_group.buttons():
                        btn.blockSignals(True)
                    b.setChecked(True)
                    for btn in r.mux_group.buttons():
                        btn.blockSignals(False)
                    r.mux_group.blockSignals(False)
                gpio = mode == 1
                if r.dir_box is not None:
                    r.dir_box.setEnabled(gpio)
                r.state_box.setEnabled(gpio)


def _columns(groups):
    """groups side by side, each scrolling vertically if needed"""
    w = QWidget()
    h = QHBoxLayout(w)
    for grp in groups:
        h.addWidget(grp, alignment=Qt.AlignTop)
    h.addStretch(1)
    scroll = QScrollArea()
    scroll.setWidgetResizable(True)
    scroll.setWidget(w)
    return scroll


class DigitalIOScreen(QWidget):
    def __init__(self, km, board_type):
        super().__init__()
        self.km = km
        self.groups = []
        tabs = QTabWidget()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(tabs)
        kogna = board_type == kmx.BOARD_TYPE_KOGNA
        labels = KOGNA_LABELS if kogna else KFLOP_LABELS
        # bits 0-47 in two columns, as the KFLOP page shows them
        rows_a = [BitRow(b, labels.get(b, ""), True, True) for b in range(0, 24)]
        rows_b = [BitRow(b, labels.get(b, ""), True, True) for b in range(24, 48)]
        tabs.addTab(_columns([self._group("Bits 0-23", rows_a), self._group("Bits 24-47", rows_b)]), "Kogna" if kogna else "KFLOP")
        if kogna:
            aux = [BitRow(b, f"Aux2-{b - 200}", True, True) for b in range(200, 210)] + \
                  [BitRow(b, f"EX_IO{b - 210}", True, True) for b in range(210, 224)]
            din = [BitRow(b, f"Diff{b - 225}", False, False) for b in range(225, 249)]
            dout = [BitRow(b, f"Diff{b - 250}", False, True) for b in range(250, 274)]
            misc = [BitRow(b, f"Opto{b - 274}", False, True) for b in range(274, 278)] + \
                   [BitRow(278, "RS485 Mode", False, True), BitRow(279, "EN +/-15V", False, True)]
            hr = [BitRow(b, f"HRPWM{b - 280}", True, True) for b in range(280, 284)]
            spi = [BitRow(b, SPI_NAMES[b - 284], True, True) for b in range(284, 290)]
            right = QWidget()
            rv = QVBoxLayout(right)
            rv.setContentsMargins(0, 0, 0, 0)
            rv.addWidget(self._group("Opto / misc outputs", misc))
            rv.addWidget(self._group("HRPWM pins", hr, mux="hrpwm"))
            rv.addWidget(self._group("SPI pins", spi, mux="spi"))
            rv.addStretch(1)
            w = QWidget()
            h = QHBoxLayout(w)
            h.addWidget(self._group("Aux2 / EX_IO", aux), alignment=Qt.AlignTop)
            h.addWidget(self._group("Differential inputs", din), alignment=Qt.AlignTop)
            h.addWidget(self._group("Differential outputs", dout), alignment=Qt.AlignTop)
            h.addWidget(right, alignment=Qt.AlignTop)
            h.addStretch(1)
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setWidget(w)
            tabs.addTab(scroll, "Kogna 200-289")
        virt = [BitRow(b, "", False, True) for b in range(48, 64)] + [BitRow(b, "", False, True) for b in range(168, 184)]
        virt_ex = [BitRow(b, "", False, True) for b in range(1024, 1056)]
        tabs.addTab(_columns([self._group("Virtual bits 48-63, 168-183", virt), self._group("Virtual bits 1024-1055", virt_ex)]), "Virtual")
        kan = [BitRow(b, f"In{b - 128}", False, False) for b in range(128, 144)]
        kan_out = [BitRow(b, f"Out{b - 144}", False, True) for b in range(144, 168)]
        tabs.addTab(_columns([self._group("Kanalog inputs 128-143", kan), self._group("Kanalog outputs 144-167", kan_out)]), "Kanalog")
        snap0 = [BitRow(b, "", True, True) for b in range(64, 96)]
        snap1 = [BitRow(b, "", True, True) for b in range(96, 128)]
        tabs.addTab(_columns([self._group("SnapAmp 0, bits 64-95", snap0), self._group("SnapAmp 1, bits 96-127", snap1)]), "SnapAmp")

    def _group(self, title, rows, mux=None):
        g = BitGroup(title, rows, self.km, mux)
        self.groups.append(g)
        return g

    def update_status(self, st):
        for g in self.groups:
            g.update(st)
