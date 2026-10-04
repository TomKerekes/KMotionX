"""The Console screen: a command line to the board. A command goes out with echo and the
board's reply lines are read back until its "Ready", as KMotion.exe's console does it;
several commands on one line may be separated by semicolons. Up/Down recall the history,
Tab completes command names from commands.txt. Board printf output appears here as well
once the console is captured (the checkbox in the Console dock)."""
from pathlib import Path
from PySide6.QtCore import Qt, QThread, Signal, QStringListModel
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPlainTextEdit, QLineEdit, QPushButton, QCompleter, QLabel
import kmx


class Sender(QThread):
    reply = Signal(str, list)
    failed = Signal(str, str)

    def __init__(self, km, cmd):
        super().__init__()
        self.km, self.cmd = km, cmd

    def run(self):
        try:
            self.reply.emit(self.cmd, self.km.send_console(self.cmd))
        except kmx.KMotionError as e:
            self.failed.emit(self.cmd, str(e))


class HistoryLine(QLineEdit):
    def __init__(self):
        super().__init__()
        self.history = []
        self.pos = 0

    def remember(self, cmd):
        if cmd and (not self.history or self.history[-1] != cmd):
            self.history.append(cmd)
            self.history = self.history[-200:]
        self.pos = len(self.history)

    def keyPressEvent(self, ev):
        if ev.key() == Qt.Key_Up and self.history:
            self.pos = max(0, self.pos - 1)
            self.setText(self.history[self.pos])
            return
        if ev.key() == Qt.Key_Down and self.history:
            self.pos = min(len(self.history), self.pos + 1)
            self.setText(self.history[self.pos] if self.pos < len(self.history) else "")
            return
        super().keyPressEvent(ev)


class ConsoleScreen(QWidget):
    def __init__(self, km):
        super().__init__()
        self.km = km
        self.sender = None
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setMaximumBlockCount(5000)
        font = QFont("monospace")
        font.setStyleHint(QFont.TypeWriter)
        self.output.setFont(font)
        lay.addWidget(self.output, 1)
        bar = QHBoxLayout()
        bar.addWidget(QLabel("Command:"))
        self.input = HistoryLine()
        self.input.setFont(font)
        self.input.returnPressed.connect(self.on_send)
        names = []
        p = Path(__file__).with_name("commands.txt")
        if p.exists():
            names = [l.strip() for l in p.read_text().splitlines() if l.strip()]
        comp = QCompleter(QStringListModel(names, self))
        comp.setCaseSensitivity(Qt.CaseInsensitive)
        comp.setCompletionMode(QCompleter.PopupCompletion)
        self.input.setCompleter(comp)
        bar.addWidget(self.input, 1)
        self.send_btn = QPushButton("Send")
        self.send_btn.clicked.connect(self.on_send)
        bar.addWidget(self.send_btn)
        clear = QPushButton("Clear")
        clear.clicked.connect(self.output.clear)
        bar.addWidget(clear)
        lay.addLayout(bar)

    def append(self, text):
        self.output.appendPlainText(text)

    def append_lines(self, lines):
        for l in lines:
            self.output.appendPlainText(l)

    def on_send(self):
        cmd = self.input.text().strip()
        if not cmd or self.sender is not None:
            return
        self.input.remember(cmd)
        self.input.clear()
        self.send_btn.setEnabled(False)
        self.sender = Sender(self.km, cmd)
        self.sender.reply.connect(self.on_reply)
        self.sender.failed.connect(self.on_failed)
        self.sender.finished.connect(self.on_finished)
        self.sender.start()

    def on_reply(self, cmd, lines):
        # the board echoes the command itself as the first line
        if not lines or lines[0].strip() != cmd:
            self.append("> " + cmd)
        self.append_lines(lines)

    def on_failed(self, cmd, msg):
        self.append(f"> {cmd}\n{msg}")

    def on_finished(self):
        self.sender = None
        self.send_btn.setEnabled(True)
        self.input.setFocus()
