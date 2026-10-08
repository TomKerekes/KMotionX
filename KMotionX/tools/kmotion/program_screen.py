"""The C Programs screen: an editor per thread (1-7), Compile, Download, Run, All and Halt
as KMotion.exe's Program screen has them, the compiler's output below the editor, the
thread buttons lit while their thread runs (ThreadActive from the status)."""
import fnmatch
import itertools
import os
from pathlib import Path
from PySide6.QtCore import Qt, QSettings, QRegularExpression, QTimer
from PySide6.QtGui import QFont, QSyntaxHighlighter, QTextCharFormat, QColor, QTextCursor
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QRadioButton, QButtonGroup, QLabel,
                               QPlainTextEdit, QFileDialog, QMessageBox, QSplitter, QDialog, QLineEdit, QCheckBox,
                               QComboBox, QTreeWidget, QTreeWidgetItem)
import kmx

N_USER_THREADS = 7
C_KEYWORDS = ("auto break case char const continue default do double else enum extern float for goto if int long "
              "register return short signed sizeof static struct switch typedef union unsigned void volatile while "
              "main TRUE FALSE NULL").split()


class CHighlighter(QSyntaxHighlighter):
    def __init__(self, doc):
        super().__init__(doc)
        kw = QTextCharFormat(); kw.setForeground(QColor("#0000c0")); kw.setFontWeight(QFont.Bold)
        pre = QTextCharFormat(); pre.setForeground(QColor("#806000"))
        num = QTextCharFormat(); num.setForeground(QColor("#a00060"))
        self.rules = [(QRegularExpression(r"\b%s\b" % w), kw) for w in C_KEYWORDS]
        self.rules.append((QRegularExpression(r"^\s*#.*"), pre))
        self.rules.append((QRegularExpression(r"\b[0-9][0-9.eE+-]*[fF]?\b"), num))
        self.str_fmt = QTextCharFormat(); self.str_fmt.setForeground(QColor("#008000"))
        self.cmt_fmt = QTextCharFormat(); self.cmt_fmt.setForeground(QColor("#808080")); self.cmt_fmt.setFontItalic(True)
        self.str_re = QRegularExpression(r'"(\\.|[^"\\])*"')
        self.line_cmt = QRegularExpression(r"//[^\n]*")
        self.cmt_start = QRegularExpression(r"/\*")
        self.cmt_end = QRegularExpression(r"\*/")

    def highlightBlock(self, text):
        for rx, fmt in self.rules:
            it = rx.globalMatch(text)
            while it.hasNext():
                m = it.next()
                self.setFormat(m.capturedStart(), m.capturedLength(), fmt)
        it = self.str_re.globalMatch(text)
        while it.hasNext():
            m = it.next()
            self.setFormat(m.capturedStart(), m.capturedLength(), self.str_fmt)
        m = self.line_cmt.match(text)
        if m.hasMatch():
            self.setFormat(m.capturedStart(), m.capturedLength(), self.cmt_fmt)
        # block comments
        self.setCurrentBlockState(0)
        start = 0
        if self.previousBlockState() != 1:
            m = self.cmt_start.match(text)
            start = m.capturedStart() if m.hasMatch() else -1
        while start >= 0:
            m = self.cmt_end.match(text, start)
            if m.hasMatch():
                length = m.capturedEnd() - start
                self.setFormat(start, length, self.cmt_fmt)
                m2 = self.cmt_start.match(text, m.capturedEnd())
                start = m2.capturedStart() if m2.hasMatch() else -1
            else:
                self.setCurrentBlockState(1)
                self.setFormat(start, len(text) - start, self.cmt_fmt)
                break


class OpenCFileDialog(QDialog):
    """Open a C file with what Windows' Open dialog has and Qt's lacks: a search by name, as you
    type, in a folder and (by default) its subfolders. A plain text matches anywhere in the name,
    either case; * ? [ ] make it a wildcard pattern (*jog*.c). The folder is read in chunks so the
    window stays responsive; hidden folders and build trees are skipped"""
    TYPES = (("C files (*.c)", (".c",)), ("C files and headers (*.c *.h)", (".c", ".h")), ("All files", None))
    SKIP_DIRS = {"node_modules", "__pycache__", "build"}
    MAX_FILES = 50000         # stop reading the folder after this many files
    MAX_SHOWN = 2000          # list at most this many matches
    CHUNK = 2000              # files read per timer tick

    def __init__(self, parent, title, folder):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.resize(780, 500)
        self.selected = ""
        self.files = []                      # (name, folder) as read
        self.walker = iter(())
        self.truncated = False
        lay = QVBoxLayout(self)
        row = QHBoxLayout()
        row.addWidget(QLabel("Look in:"))
        self.folder = QLineEdit(folder)
        self.folder.setToolTip("the folder to search; type a path and press Enter, or Browse")
        self.folder.returnPressed.connect(self.rescan)
        row.addWidget(self.folder, 1)
        b = QPushButton("Browse...")
        b.clicked.connect(self.on_browse)
        row.addWidget(b)
        lay.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(QLabel("Search:"))
        self.search = QLineEdit()
        self.search.setPlaceholderText("part of the name, or a pattern such as *jog*.c")
        self.search.setClearButtonEnabled(True)
        self.search.textChanged.connect(self.refilter)
        self.search.returnPressed.connect(self.on_open)
        row.addWidget(self.search, 1)
        self.subfolders = QCheckBox("Subfolders")
        self.subfolders.setChecked(True)
        self.subfolders.toggled.connect(self.rescan)
        row.addWidget(self.subfolders)
        self.types = QComboBox()
        for text, _ in self.TYPES:
            self.types.addItem(text)
        self.types.currentIndexChanged.connect(self.refilter)
        row.addWidget(self.types)
        lay.addLayout(row)
        self.list = QTreeWidget()
        self.list.setHeaderLabels(["Name", "Folder"])
        self.list.setRootIsDecorated(False)
        self.list.setSortingEnabled(True)
        self.list.sortByColumn(0, Qt.AscendingOrder)
        self.list.setColumnWidth(0, 260)
        self.list.itemDoubleClicked.connect(lambda item, col: self.on_open())
        lay.addWidget(self.list, 1)
        row = QHBoxLayout()
        self.status = QLabel("")
        row.addWidget(self.status, 1)
        b = QPushButton("Standard dialog...")
        b.setToolTip("the usual file dialog, without the search")
        b.clicked.connect(self.on_standard)
        row.addWidget(b)
        self.open_button = QPushButton("Open")
        self.open_button.clicked.connect(self.on_open)
        row.addWidget(self.open_button)
        b = QPushButton("Cancel")
        b.clicked.connect(self.reject)
        row.addWidget(b)
        lay.addLayout(row)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.scan_step)
        self.search.setFocus()
        self.rescan()

    def root(self):
        return os.path.expanduser(self.folder.text().strip())

    def walk(self, root):
        if not self.subfolders.isChecked():
            try:
                with os.scandir(root) as it:
                    for e in it:
                        if e.is_file():
                            yield e.name, root
            except OSError:
                pass
            return
        for d, dirs, names in os.walk(root):
            dirs[:] = [x for x in dirs if not x.startswith(".") and x not in self.SKIP_DIRS]
            for n in names:
                yield n, d

    def rescan(self):
        self.timer.stop()
        self.files = []
        self.truncated = False
        root = self.root()
        self.walker = self.walk(root) if os.path.isdir(root) else iter(())
        if not os.path.isdir(root):
            self.status.setText("no such folder")
        self.list.clear()
        self.timer.start(0)

    def scan_step(self):
        chunk = list(itertools.islice(self.walker, self.CHUNK))
        self.files.extend(chunk)
        if len(self.files) >= self.MAX_FILES:
            self.truncated = True
        if len(chunk) < self.CHUNK or self.truncated:
            self.timer.stop()
        self.refilter()

    def matches(self):
        text = self.search.text().strip().lower()
        exts = self.TYPES[self.types.currentIndex()][1]
        if any(c in text for c in "*?["):
            hit = lambda name: fnmatch.fnmatchcase(name.lower(), text)
        else:
            hit = lambda name: text in name.lower()
        return [(n, d) for n, d in self.files if (exts is None or n.lower().endswith(exts)) and hit(n)]

    def refilter(self):
        found = self.matches()
        root = self.root()
        current = self.list.currentItem().data(0, Qt.UserRole) if self.list.currentItem() else None
        self.list.setSortingEnabled(False)
        self.list.clear()
        for n, d in found[:self.MAX_SHOWN]:
            rel = os.path.relpath(d, root)
            item = QTreeWidgetItem([n, "" if rel == "." else rel])
            item.setData(0, Qt.UserRole, os.path.join(d, n))
            item.setToolTip(0, os.path.join(d, n))
            self.list.addTopLevelItem(item)
            if os.path.join(d, n) == current:
                self.list.setCurrentItem(item)
        self.list.setSortingEnabled(True)
        if self.list.currentItem() is None and self.list.topLevelItemCount():
            self.list.setCurrentItem(self.list.topLevelItem(0))
        scanning = self.timer.isActive()
        text = f"{len(found)} of {len(self.files)} files"
        if len(found) > self.MAX_SHOWN:
            text += f", the first {self.MAX_SHOWN} shown"
        if scanning:
            text += ", still reading the folder..."
        elif self.truncated:
            text += f" (stopped reading after {self.MAX_FILES}: pick a smaller folder)"
        self.status.setText(text)
        self.open_button.setEnabled(self.list.currentItem() is not None)

    def on_browse(self):
        d = QFileDialog.getExistingDirectory(self, "Look in", self.root())
        if d:
            self.folder.setText(d)
            self.rescan()

    def on_open(self):
        item = self.list.currentItem()
        if item is None:
            return
        self.selected = item.data(0, Qt.UserRole)
        self.accept()

    def on_standard(self):
        p, _ = QFileDialog.getOpenFileName(self, self.windowTitle(), self.root(), "C files (*.c);;All files (*)")
        if p:
            self.selected = p
            self.accept()


class ProgramScreen(QWidget):
    def __init__(self, km, board_type, log):
        super().__init__()
        self.km = km
        self.board_type = board_type
        self.log = log                                   # a callable for the console dock
        self.settings = QSettings("Dynomotion", "kmotion-tool")
        self.files = [self.settings.value(f"program/file{t}", "") for t in range(1, N_USER_THREADS + 1)]
        self.thread = int(self.settings.value("program/thread", 1))
        self.thread_active = [False] * N_USER_THREADS
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        top = QHBoxLayout()
        top.addWidget(QLabel("Thread:"))
        self.thread_group = QButtonGroup(self)
        self.thread_buttons = []
        for t in range(1, N_USER_THREADS + 1):
            rb = QRadioButton(str(t))
            rb.setToolTip(f"thread {t}: the editor shows this thread's file; the button is lit while the thread runs")
            self.thread_group.addButton(rb, t)
            self.thread_buttons.append(rb)
            top.addWidget(rb)
        self.thread_group.idToggled.connect(self.on_thread)
        top.addSpacing(20)
        for text, slot, tip in (("New", self.on_new, "start a new file for this thread"),
                                ("Open...", self.on_open, "load a C file into this thread"),
                                ("Save", self.on_save, ""), ("Save As...", self.on_save_as, ""),
                                ("Compile", self.on_compile, "tcc67 for the connected board, no download"),
                                ("Download", self.on_download, "stop the thread and load the compiled program"),
                                ("Run", self.on_run, "Execute the thread"),
                                ("All", self.on_all, "Compile, Download and Run"),
                                ("Halt", self.on_halt, "Kill the thread")):
            b = QPushButton(text)
            b.clicked.connect(slot)
            if tip:
                b.setToolTip(tip)
            top.addWidget(b)
        top.addStretch(1)
        lay.addLayout(top)
        self.title = QLabel("")
        lay.addWidget(self.title)
        split = QSplitter(Qt.Vertical)
        self.editor = QPlainTextEdit()
        font = QFont("monospace")
        font.setStyleHint(QFont.TypeWriter)
        self.editor.setFont(font)
        self.editor.setTabStopDistance(4 * self.editor.fontMetrics().horizontalAdvance(" "))
        self.editor.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.highlighter = CHighlighter(self.editor.document())
        self.editor.textChanged.connect(self.refresh_title)
        split.addWidget(self.editor)
        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setFont(font)
        self.output.setMaximumBlockCount(2000)
        split.addWidget(self.output)
        split.setSizes([600, 150])
        lay.addWidget(split, 1)
        self.thread_buttons[self.thread - 1].setChecked(True)
        self.load_file()

    # ---- files ----
    @property
    def path(self):
        return self.files[self.thread - 1]

    def load_file(self):
        p = self.path
        text = ""
        if p and Path(p).exists():
            try:
                text = Path(p).read_text(errors="replace")
            except OSError as e:
                self.output.appendPlainText(f"cannot read {p}: {e}")
        self.editor.blockSignals(True)
        self.editor.setPlainText(text)
        self.editor.blockSignals(False)
        self.editor.document().setModified(False)
        self.refresh_title()

    def save_file(self, ask=False):
        """returns False if the save did not happen"""
        if not self.path or ask:
            p, _ = QFileDialog.getSaveFileName(self, f"Save the thread {self.thread} program", self.path or str(Path.home()), "C files (*.c);;All files (*)")
            if not p:
                return False
            self.files[self.thread - 1] = p
            self.settings.setValue(f"program/file{self.thread}", p)
        try:
            Path(self.path).write_text(self.editor.toPlainText())
        except OSError as e:
            QMessageBox.warning(self, "Save", f"cannot write {self.path}: {e}")
            return False
        self.editor.document().setModified(False)
        self.refresh_title()
        return True

    def refresh_title(self):
        mod = " (modified)" if self.editor.document().isModified() else ""
        self.title.setText(f"Thread {self.thread}: {self.path or 'no file'}{mod}")

    def on_thread(self, t, checked):
        if not checked or t == self.thread:
            return
        if self.editor.document().isModified() and not self.save_file():
            self.thread_buttons[self.thread - 1].setChecked(True)      # stay
            return
        self.thread = t
        self.settings.setValue("program/thread", t)
        self.load_file()

    def on_new(self):
        if self.editor.document().isModified() and not self.save_file():
            return
        self.files[self.thread - 1] = ""
        self.settings.setValue(f"program/file{self.thread}", "")
        self.editor.setPlainText('#include "KMotionDef.h"\n\nmain()\n{\n\tprintf("Hello World\\n");\n}\n')
        self.editor.document().setModified(True)
        self.refresh_title()

    def on_open(self):
        if self.editor.document().isModified() and not self.save_file():
            return
        # where to look: this thread's file's folder, else the last folder searched, else
        # KMotionX's C Programs (Dynomotion's examples), else home
        examples = Path(__file__).resolve().parents[3] / "C Programs"
        start = (str(Path(self.path).parent) if self.path else self.settings.value("program/open_dir", "")
                 or (str(examples) if examples.is_dir() else str(Path.home())))
        dlg = OpenCFileDialog(self, f"Open a C file into thread {self.thread}", start)
        if dlg.exec() != QDialog.Accepted or not dlg.selected:
            return
        p = dlg.selected
        self.settings.setValue("program/open_dir", dlg.root())
        self.files[self.thread - 1] = p
        self.settings.setValue(f"program/file{self.thread}", p)
        self.load_file()

    def on_save(self):
        self.save_file()

    def on_save_as(self):
        self.save_file(ask=True)

    # ---- the board ----
    def compile(self):
        """as KMotion.exe: save first, then tcc67 for the connected board; True on success"""
        if not self.path and not self.save_file(ask=True):
            return False
        if self.editor.document().isModified() and not self.save_file():
            return False
        bt = self.km.board_type()
        if bt <= 0:
            bt = self.board_type if self.board_type > 0 else kmx.BOARD_TYPE_KOGNA
        out = self.km.compile(self.path, self.thread, bt)
        if out:
            self.output.setPlainText(out)
            self.select_error_line(out)
            return False
        self.output.setPlainText(f"No Errors - {Path(self.path).name} compiled for thread {self.thread} ({kmx.BOARD_NAMES.get(bt, bt)})")
        return True

    def download(self):
        out = self.km.out_name(self.path, self.thread)    # the compiled .out, not the source
        if not Path(out).exists():
            self.output.appendPlainText(f"nothing to download: {out} does not exist, compile first")
            return False
        try:
            self.km.kill(self.thread)                     # the thread must be stopped first
            rc = self.km.load(out, self.thread)
        except kmx.KMotionError as e:
            self.output.appendPlainText(str(e))
            return False
        if rc:
            self.output.appendPlainText(f"Download failed ({rc})")
            for e in self.km.errors():                    # the library's own message about it
                self.output.appendPlainText("  " + e)
            return False
        self.output.appendPlainText(f"Downloaded to thread {self.thread}")
        return True

    def on_compile(self):
        self.compile()

    def on_download(self):
        if self.path:
            self.download()

    def on_run(self):
        try:
            self.km.execute(self.thread)
        except kmx.KMotionError as e:
            self.output.appendPlainText(str(e))

    def on_all(self):
        if self.compile() and self.download():
            self.on_run()

    def on_halt(self):
        try:
            self.km.kill(self.thread)
        except kmx.KMotionError as e:
            self.output.appendPlainText(str(e))

    def select_error_line(self, out):
        """tcc67 reports 'file:line: error ...': put the cursor on the first such line"""
        import re
        m = re.search(r":(\d+): ", out)
        if not m:
            return
        line = int(m.group(1))
        cursor = QTextCursor(self.editor.document().findBlockByNumber(max(line - 1, 0)))
        cursor.select(QTextCursor.LineUnderCursor)
        self.editor.setTextCursor(cursor)
        self.editor.setFocus()

    # ---- status ----
    def update_status(self, st):
        for i, rb in enumerate(self.thread_buttons):
            active = bool((st.ThreadActive >> (i + 1)) & 1)
            if active != self.thread_active[i]:
                self.thread_active[i] = active
                rb.setStyleSheet("QRadioButton { color: #00a000; font-weight: bold; }" if active else "")
