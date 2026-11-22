"""AXIS's Stock menu (the gmoccapy page's buttons, for AXIS): a "Stock" cascade before Help with the
Stock shown or hidden, a tool image picked for a tool, taken away, or made with the Create tool
dialog.  AXIS's own View menu has the program's lines and the live plot.  AXIS is Tk and its tool
table is LinuxCNC's separate tooledit program, so a tool is asked for by number (the one in the
spindle offered), and the Create tool dialog, which is GTK, runs as its own process
(tool_dialog.py); both write the image's name into the tool table file and have LinuxCNC read the
table again, and the Stock, watching the file, cuts with the new image."""
import logging
import os
import subprocess
import sys
import tkinter.filedialog
import tkinter.messagebox
import tkinter.simpledialog

import kmotion_stock

log = logging.getLogger("kmotion-stock")
HERE = os.path.dirname(os.path.abspath(__file__))
MENU = ".menu.stock"
SHOW_VAR = "kmotion_stock_show"


def install(root, o, stat, command, part):
    """The menu into AXIS's menu bar (root: AXIS's Tk root, o: its preview, stat/command: its
    linuxcnc.stat() and linuxcnc.command(), part: the StockPart).  Returns the menu's path."""
    tk = root.tk
    table = part.settings.tool_table
    image_dir = part.images.image_dir

    def redraw():
        try:
            o.tkRedraw()
        except Exception:
            pass

    def ask_tool(title):
        try:
            stat.poll()
            current = int(getattr(stat, "tool_in_spindle", 0) or 0)
        except Exception:
            current = 0
        return tkinter.simpledialog.askinteger(title, "Tool number (the one in the spindle: %d)" % current,
                                               initialvalue=current or 1, minvalue=1, parent=root)

    def assign(tool, name):
        if not table:
            tkinter.messagebox.showerror("Tool image", "No tool table ([EMCIO] TOOL_TABLE).", parent=root)
            return
        try:
            if not kmotion_stock.table_file_with_image(table, tool, name):
                tkinter.messagebox.showerror("Tool image", "Tool %d is not in the tool table." % tool, parent=root)
                return
        except OSError as e:
            tkinter.messagebox.showerror("Tool image", str(e), parent=root)
            return
        try:
            command.load_tool_table()                        # LinuxCNC reads it; the Stock watches the file
        except Exception as e:
            log.warning("load_tool_table: %s", e)
        log.info("AXIS: tool %d's image set to %r in %s", tool, name, table)

    def on_show():
        shown = bool(int(tk.getvar(SHOW_VAR)))
        part.set_visible(shown)
        redraw()

    def on_tool_image():
        tool = ask_tool("Tool image")
        if not tool:
            return
        path = tkinter.filedialog.askopenfilename(title="Tool image for tool %d" % tool, parent=root, initialdir=image_dir,
                                                  filetypes=[("Tool images (VRML)", "*.wrl"), ("All files", "*")])
        if not path:
            return
        name = os.path.basename(path) if os.path.normpath(os.path.dirname(os.path.abspath(path))) == os.path.normpath(image_dir) else path
        assign(tool, name)

    def on_remove_image():
        tool = ask_tool("Remove tool image")
        if tool:
            assign(tool, "")

    def on_create_tool():
        tool = ask_tool("Create tool")
        if not tool:
            return
        args = [sys.executable, os.path.join(HERE, "tool_dialog.py"), "--images", image_dir, "--tool", str(tool)]
        if table:
            args += ["--table", table]
        env = dict(os.environ)                                 # LinuxCNC's python modules, where AXIS has them from
        rs274 = sys.modules.get("rs274")
        if rs274 is not None and getattr(rs274, "__file__", None):
            lib = os.path.dirname(os.path.dirname(os.path.abspath(rs274.__file__)))
            env["PYTHONPATH"] = lib + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
        try:
            subprocess.Popen(args, cwd=os.getcwd(), env=env)
        except OSError as e:
            tkinter.messagebox.showerror("Create tool", str(e), parent=root)

    tk.setvar(SHOW_VAR, "1" if part.visible else "0")
    tk.call("menu", MENU, "-tearoff", "0")
    tk.call(MENU, "add", "checkbutton", "-label", "Show stock", "-variable", SHOW_VAR, "-command", root.register(on_show))
    tk.call(MENU, "add", "separator")
    tk.call(MENU, "add", "command", "-label", "Tool image...", "-command", root.register(on_tool_image))
    tk.call(MENU, "add", "command", "-label", "Remove tool image...", "-command", root.register(on_remove_image))
    tk.call(MENU, "add", "command", "-label", "Create tool...", "-command", root.register(on_create_tool))
    try:
        last = int(tk.call(".menu", "index", "end"))          # Help is the last cascade: the Stock before it
    except Exception:
        last = 0
    tk.call(".menu", "insert", str(last), "cascade", "-label", "Stock", "-menu", MENU, "-underline", "0")
    log.info("AXIS: the Stock menu (show, tool image, create tool)")
    return MENU
