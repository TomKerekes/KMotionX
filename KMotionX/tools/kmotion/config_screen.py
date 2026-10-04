"""The Config & Flash screen: one channel's settings as KMotion.exe's Config, Step Response
and Filter screens hold them, uploaded from and downloaded to the board as one block,
exported as C code; the user-memory flash and the threads to launch on power-up."""
import shutil
import time
from PySide6.QtCore import Qt, QThread, Signal, QProcess
from PySide6.QtGui import QGuiApplication, QFont
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QFormLayout, QGroupBox, QLabel, QLineEdit,
                               QComboBox, QSpinBox, QCheckBox, QPushButton, QMessageBox, QScrollArea, QProgressDialog,
                               QFileDialog, QDialog, QPlainTextEdit)
import kmx

INPUT_MODES = ["No Input", "Encoder", "ADC", "Resolver", "User Input", "Serial Servo"]
OUTPUT_MODES = ["No Output", "Microstep", "DC Servo", "3PH Servo", "4PH Servo", "DAC Servo", "Step Dir", "CL Step",
                "CL Micro", "Serial Servo", "CL Serial Servo"]
STEP_DIR_MODES = ["Step/Dir", "Quadrature", "CW/CCW"]
DRIVES = ["Open Collector", "LVTTL", "Differential"]
BACKLASH_MODES = ["off", "Linear"]
LIMIT_ACTIONS = ["Kill Motor Drive", "Disallow drive in direction of limit", "Stop movement"]
MICROSTEP_MODE, BRUSHLESS_4PH_MODE, STEP_DIR_MODE, CL_STEP_DIR_MODE, CL_MICROSTEP_MODE = 1, 4, 6, 7, 8
SERIAL_SERVO_MODE, CL_SERIAL_SERVO_MODE = 9, 10
TUNING = ["Vel", "Accel", "Jerk", "P", "I", "D", "FFVel", "FFAccel", "MaxI", "MaxErr", "MaxOutput", "DeadBandGain", "DeadBandRange"]
MISC = [("InvDistPerCycle", "Inv Dist Per Cycle"), ("Lead", "Lead Compensation"), ("MaxFollowingError", "Max Following Error"),
        ("StepperAmplitude", "Microstepper Amplitude")]


# the Step/Dir generator channel, pulse mode and drive type packed into OutputChan0 (ConfigDlg)
def encode_output_chan_sd(chan, op_mode, drive):
    if chan > 7:
        chan += 56
    if drive == 1:
        chan += 8
    elif drive == 2:
        chan += 128
    if op_mode == 1:
        chan += 16
    elif op_mode == 2:
        chan += 48
    return chan


def sd_chan(code):
    return ((code & 0x40) >> 3) | (code & 7)


def sd_mode(code):
    i = (code >> 4) & 3
    return 2 if i == 3 else i


def sd_drive(code):
    if code >> 7:
        return 2
    return 1 if code & 8 else 0


class FlashWorker(QThread):
    done = Signal(bool, str)

    def __init__(self, km):
        super().__init__()
        self.km = km

    def run(self):
        try:
            ok = self.km.flash_user_memory()
            self.done.emit(ok, "" if ok else "no Ready from the board within 80 s")
        except kmx.KMotionError as e:
            self.done.emit(False, str(e))


class NewVersionWorker(QThread):
    done = Signal(int)

    def __init__(self, km, image):
        super().__init__()
        self.km, self.image = km, image

    def run(self):
        self.done.emit(self.km.flash_new_version(self.image, 80))


class RecoveryWorker(QThread):
    done = Signal(str)
    connected = Signal()

    def __init__(self, km, image):
        super().__init__()
        self.km, self.image = km, image
        self.deadline = time.time() + 20
        self.cancel = False

    def keep_waiting(self):
        return not self.cancel and time.time() < self.deadline

    def run(self):
        self.done.emit(self.km.recover_kflop(self.image, self.keep_waiting))


class FlashComDialog(QDialog):
    """TI's serial flash writer for the OMAP-L138 (a .NET program, run with Mono), with its
    output live; the Kogna must be in its UART boot mode (the jumper procedure)"""

    def __init__(self, parent, port, image):
        super().__init__(parent)
        self.setWindowTitle("Flash Kogna over " + port)
        self.resize(700, 400)
        v = QVBoxLayout(self)
        self.out = QPlainTextEdit()
        self.out.setReadOnly(True)
        v.addWidget(self.out)
        self.btn = QPushButton("Close")
        self.btn.setEnabled(False)
        self.btn.clicked.connect(self.accept)
        v.addWidget(self.btn)
        self.proc = QProcess(self)
        self.proc.setProcessChannelMode(QProcess.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self.on_output)
        self.proc.finished.connect(self.on_finished)
        tool = kmx.firmware_dir(kmx.BOARD_TYPE_KOGNA) / "ti_tools" / "flash_writer" / "sfh_OMAP-L138.exe"
        args = [str(tool), "-targettype", "C6748_LCDK", "-flashtype", "NAND", "-v", "-p", port, "-flash_noubl", str(image)]
        self.out.appendPlainText("mono " + " ".join(args) + "\n")
        self.proc.start("mono", args)

    def on_output(self):
        self.out.appendPlainText(bytes(self.proc.readAllStandardOutput()).decode(errors="replace").rstrip("\n"))

    def on_finished(self, code, status):
        self.exit_code = code
        self.out.appendPlainText(f"\n[flash writer exited with {code}]")
        self.btn.setEnabled(True)


class ConfigScreen(QWidget):
    busy = Signal(bool)                  # True while a flash operation owns the board: the window pauses polling

    def __init__(self, km, board_type, log):
        super().__init__()
        self.km = km
        self.log = log
        self.n_channels = 16 if board_type == kmx.BOARD_TYPE_KOGNA else 8
        self.fields = {}                 # name -> QLineEdit
        self.params = kmx.ChannelParams()
        self.flash_worker = None
        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        top = QHBoxLayout()
        top.addWidget(QLabel("Channel"))
        self.chan = QSpinBox()
        self.chan.setRange(0, self.n_channels - 1)
        self.chan.setToolTip("the axis channel the settings below belong to; Upload reads it from the board")
        top.addWidget(self.chan)
        for text, slot, tip in (("Upload Channel", self.on_upload, "read every setting of this channel from the board"),
                                ("Download Channel", self.on_download, "write the settings below to the board (an enabled axis is disabled and re-enabled)"),
                                ("C Code -> Clipboard", self.on_c_code, "the channel's settings as C statements for an init program")):
            b = QPushButton(text)
            b.clicked.connect(slot)
            b.setToolTip(tip)
            top.addWidget(b)
        top.addStretch(1)
        self.status = QLabel("")
        top.addWidget(self.status)
        outer.addLayout(top)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        body = QWidget()
        grid = QGridLayout(body)
        grid.setAlignment(Qt.AlignTop)
        # column 1: modes, channels
        modes = QGroupBox("Axis Modes")
        f = QFormLayout(modes)
        self.input_mode = QComboBox(); self.input_mode.addItems(INPUT_MODES)
        self.output_mode = QComboBox(); self.output_mode.addItems(OUTPUT_MODES)
        self.output_mode.currentIndexChanged.connect(self.apply_mode_rules)
        self.sd_mode = QComboBox(); self.sd_mode.addItems(STEP_DIR_MODES)
        self.drive = QComboBox(); self.drive.addItems(DRIVES)
        f.addRow("input", self.input_mode)
        f.addRow("output", self.output_mode)
        f.addRow("Step/Dir mode", self.sd_mode)
        f.addRow("drive", self.drive)
        grid.addWidget(modes, 0, 0)
        inputs = QGroupBox("Input Channels")
        f = QFormLayout(inputs)
        self.in0 = QSpinBox(); self.in0.setRange(-1, 255)
        self.in1 = QSpinBox(); self.in1.setRange(-1, 255)
        f.addRow("channel 0", self.in0)
        f.addRow("gain 0", self._field("InputGain0"))
        f.addRow("offset 0", self._field("InputOffset0"))
        f.addRow("channel 1", self.in1)
        f.addRow("gain 1", self._field("InputGain1"))
        f.addRow("offset 1", self._field("InputOffset1"))
        grid.addWidget(inputs, 1, 0)
        outputs = QGroupBox("Output Channels")
        f = QFormLayout(outputs)
        self.out0 = QSpinBox(); self.out0.setRange(-1, 255)
        self.out1 = QSpinBox(); self.out1.setRange(-1, 255)
        f.addRow("channel 0", self.out0)
        f.addRow("channel 1", self.out1)
        f.addRow("gain", self._field("OutputGain"))
        f.addRow("offset", self._field("OutputOffset"))
        grid.addWidget(outputs, 2, 0)
        # column 2: tuning, misc
        tuning = QGroupBox("Tuning (Step Response screen)")
        f = QFormLayout(tuning)
        for name in TUNING:
            f.addRow(name, self._field(name))
        grid.addWidget(tuning, 0, 1, 2, 1)
        misc = QGroupBox("Motor")
        f = QFormLayout(misc)
        for name, label in MISC:
            f.addRow(label, self._field(name))
        grid.addWidget(misc, 2, 1)
        # column 3: limits, soft limits, master/slave, backlash
        limits = QGroupBox("Limit Switch Options")
        f = QGridLayout(limits)
        f.addWidget(QLabel("Negative"), 0, 1); f.addWidget(QLabel("Positive"), 0, 2)
        self.watch_neg = QCheckBox(); self.watch_pos = QCheckBox()
        self.pol_neg = QCheckBox(); self.pol_pos = QCheckBox()
        self.bit_neg = QSpinBox(); self.bit_neg.setRange(0, 2047)
        self.bit_pos = QSpinBox(); self.bit_pos.setRange(0, 2047)
        f.addWidget(QLabel("Watch Limit"), 1, 0); f.addWidget(self.watch_neg, 1, 1); f.addWidget(self.watch_pos, 1, 2)
        f.addWidget(QLabel("Stop when low"), 2, 0); f.addWidget(self.pol_neg, 2, 1); f.addWidget(self.pol_pos, 2, 2)
        f.addWidget(QLabel("bit no."), 3, 0); f.addWidget(self.bit_neg, 3, 1); f.addWidget(self.bit_pos, 3, 2)
        self.action = QComboBox(); self.action.addItems(LIMIT_ACTIONS)
        f.addWidget(QLabel("Action"), 4, 0); f.addWidget(self.action, 4, 1, 1, 2)
        grid.addWidget(limits, 0, 2)
        soft = QGroupBox("Soft Limits")
        f = QFormLayout(soft)
        f.addRow("Soft Limit +", self._field("SoftLimitPos"))
        f.addRow("Soft Limit -", self._field("SoftLimitNeg"))
        grid.addWidget(soft, 1, 2)
        ms = QGroupBox("Master/Slave and Backlash")
        f = QFormLayout(ms)
        self.master = QSpinBox(); self.master.setRange(-1, 15)
        f.addRow("master axis (-1 none)", self.master)
        f.addRow("slave gain", self._field("SlaveGain"))
        self.backlash_mode = QComboBox(); self.backlash_mode.addItems(BACKLASH_MODES)
        f.addRow("backlash mode", self.backlash_mode)
        f.addRow("amount", self._field("BacklashAmount"))
        f.addRow("rate", self._field("BacklashRate"))
        grid.addWidget(ms, 2, 2)
        # row below: IIR filters and flash
        iir = QGroupBox("IIR Filters (Filter screen)")
        g = QGridLayout(iir)
        for c, name in enumerate(("B0", "B1", "B2", "A1", "A2")):
            g.addWidget(QLabel(name), 0, c + 1)
        self.iir = []
        for r in range(kmx.N_IIR_FILTERS):
            g.addWidget(QLabel(f"filter {r}"), r + 1, 0)
            row = []
            for c in range(5):
                e = QLineEdit("0")
                e.setMaximumWidth(110)
                g.addWidget(e, r + 1, c + 1)
                row.append(e)
            self.iir.append(row)
        grid.addWidget(iir, 3, 0, 1, 2)
        flash = QGroupBox("Flash")
        v = QVBoxLayout(flash)
        b = QPushButton("User Memory")
        b.setToolTip("copy the user programs and all settings into the board's flash (FLASH)")
        b.clicked.connect(self.on_flash)
        v.addWidget(b)
        v.addWidget(QLabel("Launch on Power Up:"))
        self.launch = []
        for t in range(1, 8):
            cb = QCheckBox(f"Thread {t}")
            cb.clicked.connect(lambda checked, th=t: self.on_launch(th, checked))
            v.addWidget(cb)
            self.launch.append(cb)
        b = QPushButton("New Version")
        b.setToolTip("re-flash the board's firmware from a .out image (ProgFlashImage); the board keeps running until reset")
        b.clicked.connect(self.on_new_version)
        v.addWidget(b)
        b = QPushButton("Recovery")
        b.setToolTip("KFLOP: load the firmware into RAM through the USB boot loader; Kogna: the boot jumper procedure")
        b.clicked.connect(self.on_recovery)
        v.addWidget(b)
        row = QHBoxLayout()
        self.port = QComboBox()
        self.port.setToolTip("serial port of the Kogna's console (FT232R)")
        for p in kmx.serial_ports():
            self.port.addItem(p)
        row.addWidget(self.port)
        b = QPushButton("Flash COM")
        b.setToolTip("Kogna primary boot loader over the serial port, with TI's flash writer (needs mono-runtime)")
        b.clicked.connect(self.on_flash_com)
        row.addWidget(b)
        v.addLayout(row)
        v.addStretch(1)
        grid.addWidget(flash, 3, 2)
        scroll.setWidget(body)
        outer.addWidget(scroll, 1)
        self.apply_mode_rules()
        self.chan.valueChanged.connect(lambda _: self.status.setText("channel changed: Upload to read it"))

    def _field(self, name):
        e = QLineEdit("0")
        e.setMaximumWidth(130)
        self.fields[name] = e
        return e

    # ---- mode rules, as DoGrayoutControls ----
    def apply_mode_rules(self):
        om = self.output_mode.currentIndex()
        step_dir = om in (STEP_DIR_MODE, CL_STEP_DIR_MODE)
        self.sd_mode.setEnabled(step_dir)
        self.drive.setEnabled(step_dir)
        self.fields["StepperAmplitude"].setEnabled(om in (MICROSTEP_MODE, CL_MICROSTEP_MODE, BRUSHLESS_4PH_MODE))
        open_loop = om in (STEP_DIR_MODE, MICROSTEP_MODE)
        serial = om in (SERIAL_SERVO_MODE, CL_SERIAL_SERVO_MODE)
        for name in ("MaxFollowingError",):
            self.fields[name].setEnabled(not open_loop)
        for name in ("InvDistPerCycle", "Lead"):
            self.fields[name].setEnabled(not open_loop and not serial)

    # ---- form <-> params ----
    def show_params(self, p):
        v = p.values
        for name, e in self.fields.items():
            e.setText(f"{v[name]:.8g}")
        self.input_mode.setCurrentIndex(max(0, min(v["InputMode"], len(INPUT_MODES) - 1)))
        self.output_mode.setCurrentIndex(max(0, min(v["OutputMode"], len(OUTPUT_MODES) - 1)))
        if v["OutputMode"] in (STEP_DIR_MODE, CL_STEP_DIR_MODE):
            self.out0.setValue(sd_chan(v["OutputChan0"]))
            self.sd_mode.setCurrentIndex(sd_mode(v["OutputChan0"]))
            self.drive.setCurrentIndex(sd_drive(v["OutputChan0"]))
        else:
            self.out0.setValue(v["OutputChan0"])
        self.out1.setValue(v["OutputChan1"])
        self.in0.setValue(v["InputChan0"])
        self.in1.setValue(v["InputChan1"])
        self.master.setValue(v["MasterAxis"])
        self.backlash_mode.setCurrentIndex(max(0, min(v["BacklashMode"], 1)))
        self.watch_neg.setChecked(p.watch_neg); self.watch_pos.setChecked(p.watch_pos)
        self.pol_neg.setChecked(p.polarity_neg); self.pol_pos.setChecked(p.polarity_pos)
        self.bit_neg.setValue(v["LimitSwitchNegBit"]); self.bit_pos.setValue(v["LimitSwitchPosBit"])
        self.action.setCurrentIndex(max(0, min(p.action, len(LIMIT_ACTIONS) - 1)))
        for r in range(kmx.N_IIR_FILTERS):
            for c in range(5):
                self.iir[r][c].setText(f"{p.iir[r][c]:.7g}")
        self.apply_mode_rules()

    def read_params(self):
        p = kmx.ChannelParams()
        v = p.values
        for name, e in self.fields.items():
            try:
                v[name] = float(e.text())
            except ValueError:
                raise ValueError(f"{name}: '{e.text()}' is not a number")
        v["InputMode"] = self.input_mode.currentIndex()
        v["OutputMode"] = self.output_mode.currentIndex()
        if v["OutputMode"] in (STEP_DIR_MODE, CL_STEP_DIR_MODE):
            v["OutputChan0"] = encode_output_chan_sd(self.out0.value(), self.sd_mode.currentIndex(), self.drive.currentIndex())
        else:
            v["OutputChan0"] = self.out0.value()
        v["OutputChan1"] = self.out1.value()
        v["InputChan0"] = self.in0.value()
        v["InputChan1"] = self.in1.value()
        v["MasterAxis"] = self.master.value()
        v["BacklashMode"] = self.backlash_mode.currentIndex()
        p.watch_neg, p.watch_pos = self.watch_neg.isChecked(), self.watch_pos.isChecked()
        p.polarity_neg, p.polarity_pos = self.pol_neg.isChecked(), self.pol_pos.isChecked()
        v["LimitSwitchNegBit"], v["LimitSwitchPosBit"] = self.bit_neg.value(), self.bit_pos.value()
        p.action = self.action.currentIndex()
        for r in range(kmx.N_IIR_FILTERS):
            try:
                p.iir[r] = tuple(float(self.iir[r][c].text()) for c in range(5))
            except ValueError:
                raise ValueError(f"IIR filter {r}: not a number")
        return p

    # ---- buttons ----
    def on_upload(self):
        ch = self.chan.value()
        try:
            self.params = self.km.upload_channel(ch)
        except (kmx.KMotionError, ValueError) as e:
            self.status.setText(f"upload failed: {e}")
            return
        self.show_params(self.params)
        self.status.setText(f"channel {ch} uploaded")

    def on_download(self):
        ch = self.chan.value()
        try:
            p = self.read_params()
        except ValueError as e:
            QMessageBox.warning(self, "Download", str(e))
            return
        try:
            self.km.download_channel(ch, p)
        except (kmx.KMotionError, ValueError) as e:
            self.status.setText(f"download failed: {e}")
            return
        self.params = p
        self.status.setText(f"channel {ch} downloaded")

    def on_c_code(self):
        try:
            p = self.read_params()
        except ValueError as e:
            QMessageBox.warning(self, "C Code", str(e))
            return
        code = p.to_c(self.chan.value())
        QGuiApplication.clipboard().setText(code)
        self.status.setText("C code copied to the clipboard")
        self.log(code)

    def on_flash(self):
        if QMessageBox.question(self, "Flash", "Copy User Programs and settings to FLASH memory?") != QMessageBox.Yes:
            return
        self.progress = QProgressDialog("Programming FLASH memory, please wait...", None, 0, 0, self)
        self.progress.setWindowModality(Qt.WindowModal)
        self.progress.show()
        self.flash_worker = FlashWorker(self.km)
        self.flash_worker.done.connect(self.on_flash_done)
        self.flash_worker.start()

    def on_flash_done(self, ok, msg):
        self.progress.close()
        if ok:
            QMessageBox.information(self, "Flash", "Programming FLASH complete")
        else:
            QMessageBox.warning(self, "Flash", f"Flash failed: {msg}")

    # ---- firmware ----
    def board_name(self):
        t = self.km.board_type()
        return kmx.BOARD_NAMES.get(t, "board"), t

    def on_new_version(self):
        name, t = self.board_name()
        if t <= 0:
            QMessageBox.warning(self, "New Version", "Unable to verify the board type.")
            return
        default = kmx.firmware_dir(t) / ("DSPKOGNA.out" if t == kmx.BOARD_TYPE_KOGNA else "DSPKFLOP.out")
        if QMessageBox.question(self, "New Version", f"This command will completely clear and re-load {name}'s internal FLASH memory. "
                                f"Loading an invalid file will render the {name} board inoperable. Are you sure you wish to proceed?") != QMessageBox.Yes:
            return
        image, _ = QFileDialog.getOpenFileName(self, "Select the firmware image to FLASH", str(default), "Program files (*.out);;All files (*)")
        if not image:
            return
        self.progress = QProgressDialog(f"Downloading the flash image to {name}, then programming the flash...\nplease wait, up to two minutes", None, 0, 0, self)
        self.progress.setWindowModality(Qt.WindowModal)
        self.progress.show()
        self.nv_worker = NewVersionWorker(self.km, image)
        self.nv_worker.done.connect(lambda rc, n=name: self.on_new_version_done(rc, n))
        self.busy.emit(True)
        self.nv_worker.start()

    def on_new_version_done(self, rc, name):
        self.progress.close()
        self.busy.emit(False)
        if rc == 0:
            if name == "Kogna" and QMessageBox.question(self, "New Version", "Programming FLASH with the new version complete.\n\n"
                                                        "Reboot the Kogna now to execute the new version?") == QMessageBox.Yes:
                try:
                    self.km.send_console("reboot!")
                    self.log("reboot! sent: the Kogna is back on the network in about 15 s")
                except kmx.KMotionError as e:
                    QMessageBox.warning(self, "New Version", f"reboot! failed: {e}")
            else:
                QMessageBox.information(self, "New Version", f"Programming FLASH with the new version complete.\n\nReboot/reset the {name} board (or cycle power) to execute the new version.")
        elif rc == 1:
            QMessageBox.critical(self, "New Version", "Timeout programming FLASH with the new version!")
        elif rc == 2:
            QMessageBox.critical(self, "New Version", f"Error programming FLASH with the new version!\n\n{name} is in an undefined state and may not be bootable. Please attempt to re-program before losing power.")
        else:
            QMessageBox.critical(self, "New Version", "Error downloading the flash image! FLASH not altered.\n" + "\n".join(self.km.errors()))

    def on_recovery(self):
        name, t = self.board_name()
        if t == kmx.BOARD_TYPE_KOGNA or t <= 0:
            QMessageBox.information(self, "Flash Recovery", "To perform a Flash Recovery on Kogna:\n\n#1 Insert the Kogna boot jumper\n#2 Apply power to Kogna\n#3 Remove the boot jumper within 1 second")
            return
        if QMessageBox.question(self, "Flash Recovery", "Flash Recovery is used to re-install firmware when KFLOP is un-bootable due to corrupted flash memory.\n\n"
                                "Please terminate any other programs which may attempt to access KFLOP before continuing.",
                                QMessageBox.Ok | QMessageBox.Cancel) != QMessageBox.Ok:
            return
        image, _ = QFileDialog.getOpenFileName(self, "Select the firmware to execute", str(kmx.firmware_dir(t) / "DSPKFLOP.out"), "Program files (*.out);;All files (*)")
        if not image:
            return
        if QMessageBox.question(self, "Flash Recovery", "#1 Power to KFLOP should be off now\n\n#2 Press OK\n\n#3 When prompted turn KFLOP power ON",
                                QMessageBox.Ok | QMessageBox.Cancel) != QMessageBox.Ok:
            return
        self.progress = QProgressDialog("Turn KFLOP power ON now... (20 s)", "Cancel", 0, 0, self)
        self.progress.setWindowModality(Qt.WindowModal)
        self.rc_worker = RecoveryWorker(self.km, image)
        self.progress.canceled.connect(lambda: setattr(self.rc_worker, "cancel", True))
        self.rc_worker.done.connect(self.on_recovery_done)
        self.progress.show()
        self.busy.emit(True)
        self.rc_worker.start()

    def on_recovery_done(self, msg):
        self.progress.close()
        self.busy.emit(False)
        if msg:
            QMessageBox.critical(self, "Flash Recovery", f"Recovery failed: {msg}")
        else:
            QMessageBox.information(self, "Flash Recovery", "Firmware loaded to RAM and executing.\n\nUse the FLASH command or New Version to program the flash memory.")

    def on_flash_com(self):
        port = self.port.currentText()
        if not port:
            QMessageBox.warning(self, "Flash COM", "No serial port found (/dev/ttyUSB*, /dev/ttyACM*).")
            return
        if not shutil.which("mono"):
            QMessageBox.warning(self, "Flash COM", "TI's serial flash writer is a .NET program and needs Mono:\n\n    sudo apt install mono-runtime")
            return
        image = kmx.firmware_dir(kmx.BOARD_TYPE_KOGNA) / "DSPKOGNA.bin"
        if not image.exists():
            QMessageBox.warning(self, "Flash COM", f"{image} not found")
            return
        text = (f"Flashing the Kogna primary boot loader over {port}:\n\n"
                "Close any apps using the port\nUSB J3 should be connected to the PC\nDisconnect any external 5V supply\n"
                "Remove the boot jumper (JP13)\nRemove the USB power jumper (J2)\nClick OK\nWait for \"Waiting for BOOTME message\"\n"
                "Insert the USB power jumper (JP2)")
        if QMessageBox.question(self, "Flash COM", text, QMessageBox.Ok | QMessageBox.Cancel) != QMessageBox.Ok:
            return
        self.busy.emit(True)
        dlg = FlashComDialog(self, port, image)
        dlg.exec()
        self.busy.emit(False)
        if getattr(dlg, "exit_code", 1) == 0:
            QMessageBox.information(self, "Flash COM", "Kogna primary boot loader flashed successfully.\n\nInsert the boot jumper (JP13) to boot from flash\nCycle power to reboot")

    def on_launch(self, thread, checked):
        try:
            self.km.set_startup_thread(thread, checked)
        except kmx.KMotionError as e:
            self.status.setText(str(e))

    def update_status(self, st):
        for t, cb in enumerate(self.launch, start=1):
            on = bool((st.RunOnStartUp >> t) & 1)
            if cb.isChecked() != on:
                cb.blockSignals(True)
                cb.setChecked(on)
                cb.blockSignals(False)
