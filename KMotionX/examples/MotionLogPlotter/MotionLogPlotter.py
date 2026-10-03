#!/usr/bin/env python3
"""MotionLogPlotter.py - plot KMotion's Trajectory Planner logs on Linux.

A Python counterpart of Dynomotion's MotionLogPlotter (PC VCS Examples): it
reads the segment log the planner writes while the "Log" option is on
(/tmp/TPSegLog.csv here, c:\\Temp\\TPSegLog.csv on Windows) or a sampled
Dest/Pos capture CSV (CaptureXYZPosDestToFile.c and friends), re-creates the
controller's interpolation tick by tick the way the plotter does, and shows
the XY path next to Position, Velocity, Accel and Jerk over time.

    MotionLogPlotter.py [LOG] [--axis X] [--tick 0.00009] [--keep-seams]
                     [--tail 0.02] [--range T0 T1] [--save FILE] [--stats]

In the window: keys x y z a b c u v (or 1-8) pick the axis, a click marks the
nearest sample on every plot and prints its values, r resets the view, ctrl+o
opens another log, F5 reloads the current one, w watches the file and reloads
it by itself after each run (also --watch); the toolbar zooms and pans (the
time plots share their time axis), and zooming the XY plot limits the time
plots to the samples in view.

Positions are in the planner's units (inches for KMotionCNC/kmxWeb logs),
velocities etc. per second. Needs numpy and matplotlib (Debian:
python3-matplotlib). --stats prints a summary without plotting.
"""
import argparse
import math
import os
import sys
import warnings

import numpy as np

AXES = "XYZABCUV"
TICK = 0.00009          # servo tick: 90 us
SEG_COLUMNS = 60        # a segment row has at least this many columns


def fnum(s):
    try:
        return float(s)
    except ValueError:
        return 0.0


class Seg:
    """One downloaded trajectory segment: 'L' linear/rapid/cubic, 'A' arc,
    'D' dwell, 'K' per-axis cubic knot (TRAJECTORY_CUBIC8)."""

    def __init__(self, f):
        self.kind = f[0][0] if f[0] else 'L'
        self.plane = int(fnum(f[1]))
        self.ccw = int(fnum(f[2]))
        self.seq = int(fnum(f[3]))
        self.dx = fnum(f[4])
        self.dwell = fnum(f[5])
        self.ntrips = int(fnum(f[6]))
        self.p0 = np.array([fnum(x) for x in f[7:15]])
        self.p1 = np.array([fnum(x) for x in f[15:23]])
        self.xc, self.yc = fnum(f[23]), fnum(f[24])
        # 7 trips x (time, a, b, c, d): distance = a t^3 + b t^2 + c t + d
        self.trips = np.array([[fnum(f[25 + 5 * i + j]) for j in range(5)] for i in range(7)])
        self.k8 = None
        if not math.isfinite(self.dx):
            self.dx = 0.0
        if self.kind == 'K':
            # tt0 = duration, then 8 axes x (a, b, c, d) of time
            k8 = np.array([fnum(x) for x in f[26:58]])
            k8[~np.isfinite(k8)] = 0.0
            self.k8 = k8.reshape(8, 4)
            self.ntrips = 1
            tt0 = self.trips[0, 0]
            self.total = tt0 if math.isfinite(tt0) and tt0 > 0 else 0.0
            return
        # zero-length stop segments can carry inf/NaN in unused trips
        tt = self.trips[:, 0]
        tt[~np.isfinite(tt) | (tt < 0)] = 0.0
        co = self.trips[:, 1:]
        co[~np.isfinite(co)] = 0.0
        if not math.isfinite(self.dwell) or self.dwell < 0:
            self.dwell = 0.0
        self.total = float(tt[:min(self.ntrips, 7)].sum())
        if self.total <= 0:
            self.total = self.dwell      # a dwell holds position

    def eval(self, t):
        """8-axis positions (n, 8) at the local times t (array)."""
        t = np.asarray(t, dtype=float)
        if self.kind == 'K':
            tc = np.clip(t, 0.0, self.total)
            o = np.empty((t.size, 8))
            for a in range(8):
                k = self.k8[a]
                o[:, a] = ((k[0] * tc + k[1]) * tc + k[2]) * tc + k[3]
            return o
        return self.point_at(self.dist_at(t))

    def dist_at(self, t):
        """Distance along the segment at local time t: the trip cubics,
        each trip's cubic giving the absolute distance."""
        res = np.full(t.shape, self.dx)
        done = np.zeros(t.shape, dtype=bool)
        tl = t.copy()
        for i in range(self.ntrips):
            tt, a, b, c, d = self.trips[i]
            m = ~done & ((tl <= tt) | (i == self.ntrips - 1))
            tc = np.minimum(tl[m], tt)
            res[m] = ((a * tc + b) * tc + c) * tc + d
            done |= m
            tl = tl - tt
        return res

    def point_at(self, s):
        """8-axis points (n, 8) at path distances s. Arcs are stored
        plane-local (x, y = plane, z = third axis) and un-swapped here."""
        f = np.clip(s / self.dx, 0.0, 1.0) if self.dx > 0 else np.zeros(s.shape)
        o = self.p0 + f[:, None] * (self.p1 - self.p0)
        if self.kind == 'A':
            p0, p1 = self.p0, self.p1
            r0 = math.hypot(p0[0] - self.xc, p0[1] - self.yc)
            r1 = math.hypot(p1[0] - self.xc, p1[1] - self.yc)
            th0 = math.atan2(p0[1] - self.yc, p0[0] - self.xc)
            th1 = math.atan2(p1[1] - self.yc, p1[0] - self.xc)
            dth = th1 - th0
            if self.ccw:
                if dth <= 0:
                    dth += 2 * math.pi
            elif dth >= 0:
                dth -= 2 * math.pi
            th = th0 + f * dth
            r = r0 + f * (r1 - r0)
            lx = self.xc + r * np.cos(th)
            ly = self.yc + r * np.sin(th)
            lz = p0[2] + f * (p1[2] - p0[2])
            if self.plane == 1:
                o[:, 2], o[:, 0], o[:, 1] = lx, ly, lz
            elif self.plane == 2:
                o[:, 1], o[:, 0], o[:, 2] = lx, lz, ly
            else:
                o[:, 0], o[:, 1], o[:, 2] = lx, ly, lz
        return o


class Log:
    def __init__(self):
        self.segs = None            # segment logs only
        self.T = np.zeros(0)
        self.P = np.zeros((8, 0))   # commanded (Dest) positions
        self.P2 = None              # measured positions, capture logs only
        self.dt = TICK
        self.seg_idx = None
        self.bt = self.bx = self.by = np.zeros(0)   # segment starts
        self._der = {}

    @property
    def count(self):
        return self.T.size

    # ---- loading -------------------------------------------------------
    @staticmethod
    def is_seg_row(line):
        f = line.split(',')
        return len(f) >= SEG_COLUMNS and len(f[0]) == 1 and f[0].upper() in "LARDK"

    @classmethod
    def load(cls, path):
        log = cls()
        with open(path, 'r', newline='') as fh:
            lines = [l.rstrip('\r\n') for l in fh]
        if not lines:
            return log
        if lines[0].startswith('kind'):
            log._load_segments(lines[1:])
        elif cls.is_seg_row(lines[0]):
            log._load_segments(lines)
        else:
            log._load_sampled(lines)
        return log

    def _load_segments(self, lines):
        self.segs = []
        for line in lines:
            if not line:
                continue
            f = line.split(',')
            if len(f) < SEG_COLUMNS:
                continue
            self.segs.append(Seg(f))

    def _load_sampled(self, lines):
        """Sampled logs: t + 8 Dest (9 cols), t + Pos x3 + Dest x3 (7 cols)
        or t + 8 Dest + 8 Pos (17 cols). Dest goes to P, Pos to P2."""
        t, p, p2 = [], [], []
        any_p2 = False
        for line in lines:
            if not line:
                continue
            f = line.split(',')
            if len(f) < 7:
                continue
            try:
                tv = float(f[0])
            except ValueError:
                continue            # header or text row
            t.append(tv)
            if len(f) >= 17:
                p.append([fnum(x) for x in f[1:9]])
                p2.append([fnum(x) for x in f[9:17]])
                any_p2 = True
            elif len(f) >= 9:
                p.append([fnum(x) for x in f[1:9]])
                p2.append([0.0] * 8)
            else:
                p2.append([fnum(x) for x in f[1:4]] + [0.0] * 5)
                p.append([fnum(x) for x in f[4:7]] + [0.0] * 5)
                any_p2 = True
        self.T = np.array(t)
        self.P = np.array(p).T if p else np.zeros((8, 0))
        if any_p2:
            self.P2 = np.array(p2).T
        if self.count > 1:
            dt = (self.T[-1] - self.T[0]) / (self.count - 1)
            if dt > 0:
                self.dt = dt

    # ---- re-creating the controller's interpolation --------------------
    def build_samples(self, tick=TICK, deglitch=True, tail=0.02):
        """Step time forward by the servo tick, carrying leftover time
        across segment boundaries like DoTrajectorys does, and evaluate the
        segments at each tick. With deglitch, each segment continues exactly
        from the previous one's analytic end point (the float32 download
        resolution leaves ~1e-6 steps at the seams that would explode into
        accel/jerk spikes). tail seconds of hold-at-end samples follow."""
        segs = self.segs
        if segs is None:
            return
        self.dt = tick
        self._der = {}
        ts, ps, idx = [], [], []
        bt, bx, by = [0.0], [segs[0].p0[0]], [segs[0].p0[1]]
        off = np.zeros(8)
        ct = 0.0            # local time in the current segment
        abs_t = 0.0
        for si, s in enumerate(segs):
            n = 0
            if ct < s.total - 1e-12:
                n = int(math.floor((s.total - 1e-12 - ct) / tick)) + 1
            if si == 0:
                n = max(n, 1)   # the plotter always samples the first segment once
            if n > 0:
                k = np.arange(n)
                ts.append(abs_t + tick * k)
                ps.append(s.eval(ct + tick * k) + off)
                idx.append(np.full(n, si))
                abs_t += tick * n
                ct += tick * n
            ct -= s.total
            if deglitch and si + 1 < len(segs):
                off += s.eval([s.total])[0] - segs[si + 1].eval([0.0])[0]
            if si + 1 < len(segs):
                bt.append(abs_t - ct)
                bx.append(segs[si + 1].p0[0] + off[0])
                by.append(segs[si + 1].p0[1] + off[1])
        if ts and tail > 0:
            bt.append(abs_t - ct)
            bx.append(segs[-1].p1[0] + off[0])
            by.append(segs[-1].p1[1] + off[1])
            n = int(math.ceil(tail / tick))
            ts.append(abs_t + tick * np.arange(n))
            ps.append(np.repeat(ps[-1][-1:], n, axis=0))
            idx.append(np.full(n, len(segs) - 1))
        if ts:
            self.T = np.concatenate(ts)
            self.P = np.concatenate(ps).T
            self.seg_idx = np.concatenate(idx)
        self.bt, self.bx, self.by = np.array(bt), np.array(bx), np.array(by)

    # ---- derived series -------------------------------------------------
    def diff(self, src):
        """Central difference against the real time stamps; the end samples
        have no central difference and are NaN (not a copied neighbour)."""
        d = np.full(src.shape, np.nan)
        n = min(src.size, self.T.size)
        if n >= 3:
            span = self.T[2:n] - self.T[:n - 2]
            with np.errstate(divide='ignore', invalid='ignore'):
                d[1:n - 1] = np.where(span > 0, (src[2:n] - src[:n - 2]) / span, np.nan)
        return d

    def series(self, axis, level):
        """level 0 position, 1 velocity, 2 accel, 3 jerk of an axis."""
        key = (axis, level)
        if key not in self._der:
            self._der[key] = self.P[axis] if level == 0 else self.diff(self.series(axis, level - 1))
        return self._der[key]


def fmt(v):
    if v is None or not math.isfinite(v):
        return "-"
    if v == 0:
        return "0"
    return "%.4e" % v if abs(v) < 1e-3 or abs(v) >= 1e7 else "%.6g" % v


def stats(log, path):
    print("%s: %d samples, %.4f s, tick %.6f s" % (path, log.count, log.T[-1] - log.T[0] if log.count else 0, log.dt))
    if log.segs is not None:
        kinds = {}
        for s in log.segs:
            kinds[s.kind] = kinds.get(s.kind, 0) + 1
        print("%d segments: %s" % (len(log.segs), ", ".join("%s=%d" % kv for kv in sorted(kinds.items()))))
    if log.count:
        print("start X %s Y %s Z %s" % tuple(fmt(v) for v in log.P[:3, 0]))
        print("end   X %s Y %s Z %s" % tuple(fmt(v) for v in log.P[:3, -1]))
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            for a in range(8):
                if np.ptp(log.P[a]) == 0:
                    continue
                print("%s: travel %s..%s  max |vel| %s  |acc| %s  |jerk| %s" % (
                    AXES[a], fmt(log.P[a].min()), fmt(log.P[a].max()),
                    *[fmt(np.nanmax(np.abs(log.series(a, l)))) for l in (1, 2, 3)]))
    if log.P2 is not None:
        err = log.P[:3] - log.P2[:3]
        print("Dest - Pos: max |err| X %s Y %s Z %s" % tuple(fmt(v) for v in np.abs(err).max(axis=1)))


class Plotter:
    LEVELS = ("Position", "Velocity", "Accel", "Jerk")
    LETTERS = "DVAJ"

    def __init__(self, log, path, axis, trange, build):
        """build: the tick/deglitch/tail arguments of build_samples, used again on reload."""
        import matplotlib.pyplot as plt
        from matplotlib import gridspec
        from matplotlib.collections import LineCollection
        from matplotlib.ticker import FuncFormatter
        from matplotlib.transforms import blended_transform_factory
        self.plt = plt
        self.build = build
        self.axis = axis
        self.mark = None                    # marked sample index
        self.watching = False
        self._seen = self._pending = None   # (mtime, size) of the file: as loaded, as last polled
        # keys matplotlib would otherwise act on: c/v (view history), l (log scale)
        plt.rcParams['keymap.back'] = ['left', 'backspace']
        plt.rcParams['keymap.forward'] = ['right']
        plt.rcParams['keymap.yscale'] = []
        self.fig = plt.figure(figsize=(15, 8.5))
        gs = gridspec.GridSpec(4, 2, figure=self.fig, width_ratios=[1, 1.25], hspace=0.35, wspace=0.15,
                               left=0.05, right=0.985, top=0.93, bottom=0.085)
        self.ax_xy = self.fig.add_subplot(gs[:, 0])
        self.strips = []
        for i in range(4):
            self.strips.append(self.fig.add_subplot(gs[i, 1], sharex=self.strips[0] if i else None))
        ax = self.ax_xy
        (self.xyline,) = ax.plot([], [], lw=0.8, color='C0')
        (self.xyline2,) = ax.plot([], [], lw=0.8, color='C2', ls='--')      # measured Pos of capture logs
        (self.xydots,) = ax.plot([], [], '.', color='orange', ms=4)         # segment starts, once they resolve
        (self.xmark,) = ax.plot([], [], 'x', color='red', ms=9, mew=2)
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.grid(True, alpha=0.3)
        ax.set_title("XY path (zoom here to select the time range)", fontsize=10)
        self.lines, self.lines2, self.vmarks, self.ticks = [], [], [], []
        for i, ax in enumerate(self.strips):
            (ln,) = ax.plot([], [], lw=0.8, color='C0')
            self.lines.append(ln)
            (ln2,) = ax.plot([], [], lw=0.8, color='C2', ls='--', visible=False)
            self.lines2.append(ln2)
            # segment starts: short ticks along the bottom, shown once they resolve
            tc = LineCollection([], colors='orange', lw=1,
                                transform=blended_transform_factory(ax.transData, ax.transAxes))
            ax.add_collection(tc, autolim=False)
            self.ticks.append(tc)
            self.vmarks.append(ax.axvline(np.nan, color='red', lw=0.8))
            ax.grid(True, alpha=0.3)
            ax.yaxis.set_major_formatter(FuncFormatter(lambda v, pos: '%g' % v))
            if i < 3:
                plt.setp(ax.get_xticklabels(), visible=False)
        self.strips[-1].set_xlabel("time (s)")
        self.readout = self.fig.text(0.01, 0.985, "", va='top', family='monospace', fontsize=9)
        self.info = self.fig.text(0.99, 0.985, "", va='top', ha='right', fontsize=9, color='gray')
        self.fig.text(0.01, 0.008, "ctrl+o open   F5 reload   w watch the file   r reset view   "
                      "x y z a b c u v or 1-8 axis   click marks a sample   esc clears it",
                      fontsize=8, color='gray')
        self.strips[0].callbacks.connect('xlim_changed', lambda ax: self.relim())
        self.ax_xy.callbacks.connect('xlim_changed', lambda ax: self.on_xy_view())
        self.ax_xy.callbacks.connect('ylim_changed', lambda ax: self.on_xy_view())
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.timer = self.fig.canvas.new_timer(interval=1000)
        self.timer.add_callback(self.poll_file)
        self.set_log(log, path, trange)

    # ---- loading --------------------------------------------------------
    def set_log(self, log, path, trange=None):
        """Show a (re)loaded log: the view resets, the axis choice stays."""
        self.log, self.path = log, path
        self.mark = None
        self._dots_shown = None
        self._seen = self.file_state()
        self.xyline.set_data(log.P[0], log.P[1])
        if log.P2 is not None:
            self.xyline2.set_data(log.P2[0], log.P2[1])
        else:
            self.xyline2.set_data([], [])
        info = "%d samples at %.0f µs" % (log.count, log.dt * 1e6)
        if log.segs is not None:
            info = "%d segments, " % len(log.segs) + info
        self.info.set_text("%s:  %s" % (os.path.basename(path), info))
        self.set_title()
        self.set_axis(self.axis)
        self.ax_xy.relim()
        self.ax_xy.autoscale()
        if trange:
            self.strips[0].set_xlim(*trange)
        elif log.count:
            self.strips[0].set_xlim(log.T[0], log.T[-1])
        self.relim()
        self.update_xy_dots()
        self.fig.canvas.draw_idle()

    def set_title(self):
        manager = getattr(self.fig.canvas, 'manager', None)
        if manager is not None:
            manager.set_window_title("Motion log: %s%s" % (self.path, "  (watching)" if self.watching else ""))

    def load_file(self, path):
        """Load a log the way main() did; the old one stays if that fails."""
        try:
            log = Log.load(path)
            if log.segs is not None:
                log.build_samples(**self.build)
        except OSError as e:
            print("cannot read %s: %s" % (path, e))
            return False
        if not log.count:
            print("%s: no samples" % path)
            return False
        self.set_log(log, path)
        print("loaded", self.info.get_text())
        return True

    def open_dialog(self):
        try:
            import tkinter as tk
            from tkinter import filedialog
        except ImportError:
            print("no tkinter: start the plotter with the file name instead")
            return
        parent = getattr(getattr(self.fig.canvas, 'manager', None), 'window', None)
        root = None
        if not hasattr(parent, 'tk'):          # not the Tk backend: use a hidden root of our own
            root = tk.Tk()
            root.withdraw()
            parent = root
        path = filedialog.askopenfilename(parent=parent, title="Open a motion log",
                                          initialdir=os.path.dirname(os.path.abspath(self.path)),
                                          filetypes=[("CSV logs", "*.csv"), ("All files", "*")])
        if root is not None:
            root.destroy()
        if path:
            self.load_file(path)

    # ---- watching the file ----------------------------------------------
    def file_state(self):
        try:
            st = os.stat(self.path)
            return (st.st_mtime_ns, st.st_size)
        except OSError:
            return None

    def toggle_watch(self, on=None):
        self.watching = (not self.watching) if on is None else on
        self._pending = None
        if self.watching:
            self._seen = self.file_state()
            self.timer.start()
        else:
            self.timer.stop()
        self.set_title()
        print("watching %s" % self.path if self.watching else "not watching")

    def poll_file(self):
        """Reload once the file has changed and then held still for a poll,
        i.e. after the run that wrote it has finished."""
        state = self.file_state()
        if state is None or state == self._seen:
            self._pending = None
        elif state == self._pending:
            self.load_file(self.path)       # set_log records the new state
        else:
            self._pending = state

    def set_axis(self, axis):
        self.axis = axis
        log = self.log
        for i, ax in enumerate(self.strips):
            self.lines[i].set_data(log.T, log.series(axis, i))
            if i == 0 and log.P2 is not None:       # measured position over the commanded one
                self.lines2[i].set_data(log.T, log.P2[axis])
                self.lines2[i].set_visible(True)
            else:
                self.lines2[i].set_data([], [])
                self.lines2[i].set_visible(False)
            ax.set_title("%s %s" % (AXES[axis], self.LEVELS[i]), fontsize=10, loc='left')
        self.relim()
        self.show_mark()

    def relim(self):
        """Autoscale each strip's y range to the samples in the time range."""
        log = self.log
        if not log.count:
            return
        t0, t1 = self.strips[0].get_xlim()
        i0, i1 = np.searchsorted(log.T, [t0, t1])
        i0, i1 = max(i0 - 1, 0), min(i1 + 1, log.count)
        if i1 - i0 < 2:
            return
        for i, ax in enumerate(self.strips):
            ys = [self.lines[i].get_ydata()[i0:i1]]
            if self.lines2[i].get_visible():
                ys.append(self.lines2[i].get_ydata()[i0:i1])
            y = np.concatenate(ys)
            y = y[np.isfinite(y)]
            if y.size:
                lo, hi = y.min(), y.max()
                # never zoom into the float32 noise of a flat stretch: keep the span
                # at least 1e-4 of the series' full range
                span_min = 1e-4 * self.full_range(i)
                if hi - lo < span_min:
                    mid = 0.5 * (lo + hi)
                    lo, hi = mid - span_min / 2, mid + span_min / 2
                pad = (hi - lo) * 0.05
                ax.set_ylim(lo - pad, hi + pad)
        self.update_ticks(t0, t1)
        # no draw here: this runs from limit callbacks, and whoever changed the
        # limits (the toolbar, a key handler) redraws afterwards

    def full_range(self, level):
        y = self.lines[level].get_ydata()
        y = y[np.isfinite(y)]
        r = float(y.max() - y.min()) if y.size else 0.0
        return r if r > 0 else 1.0

    MIN_PX = 3      # marker spacing (pixels) below which segment starts don't resolve

    def update_ticks(self, t0, t1):
        """Segment-start ticks on the strips, only while they resolve."""
        bt = self.log.bt
        vis = bt[(bt >= t0) & (bt <= t1)]
        show = vis.size > 0 and (vis.size < 2 or
                                 self.strips[0].bbox.width * np.median(np.diff(vis)) / (t1 - t0) >= self.MIN_PX)
        segs = [((t, 0.0), (t, 0.06)) for t in vis] if show else []
        for tc in self.ticks:
            tc.set_segments(segs)

    def update_xy_dots(self):
        """Segment-start dots on the XY path, only while they resolve."""
        log = self.log
        if not log.bx.size:
            return
        x0, x1 = sorted(self.ax_xy.get_xlim())
        y0, y1 = sorted(self.ax_xy.get_ylim())
        inside = np.nonzero((log.bx >= x0) & (log.bx <= x1) & (log.by >= y0) & (log.by <= y1))[0]
        show = inside.size > 0
        if inside.size > 1:
            px = self.ax_xy.transData.transform(np.column_stack([log.bx[inside], log.by[inside]]))
            d = np.hypot(*np.diff(px, axis=0).T)
            d = d[d > 0.5]      # ignore coincident starts (XY stands still while Z moves)
            show = d.size == 0 or np.median(d) >= self.MIN_PX
        key = (show, inside.size, inside[0] if inside.size else -1)
        if key == self._dots_shown:
            return
        self._dots_shown = key
        if show:
            self.xydots.set_data(log.bx[inside], log.by[inside])
        else:
            self.xydots.set_data([], [])

    def on_xy_view(self):
        """XY view changed: limit the time plots to the samples in view."""
        self.update_xy_dots()
        log = self.log
        if not log.count:
            return
        x0, x1 = sorted(self.ax_xy.get_xlim())
        y0, y1 = sorted(self.ax_xy.get_ylim())
        inside = (log.P[0] >= x0) & (log.P[0] <= x1) & (log.P[1] >= y0) & (log.P[1] <= y1)
        if inside.sum() < 2:
            return
        t = log.T[inside]
        cur = self.strips[0].get_xlim()
        if abs(cur[0] - t.min()) > 1e-9 or abs(cur[1] - t.max()) > 1e-9:
            self.strips[0].set_xlim(t.min(), t.max())

    def show_mark(self):
        i, log = self.mark, self.log
        if i is None or not log.count:
            self.xmark.set_data([], [])
            for v in self.vmarks:
                v.set_xdata([np.nan, np.nan])
            self.readout.set_text("")
        else:
            self.xmark.set_data([log.P[0, i]], [log.P[1, i]])
            for v in self.vmarks:
                v.set_xdata([log.T[i], log.T[i]])
            vals = "  ".join("%s=%s" % (self.LETTERS[l], fmt(log.series(self.axis, l)[i])) for l in range(4))
            pos = "  ".join("%s %s" % (AXES[a], fmt(log.P[a, i])) for a in range(3))
            seg = "  seg %d" % log.seg_idx[i] if log.seg_idx is not None else ""
            if log.P2 is not None:
                vals += "  P=%s" % fmt(log.P2[self.axis, i])
            self.readout.set_text("t=%.6f s  sample %d%s   %s   %s: %s" % (log.T[i], i, seg, pos, AXES[self.axis], vals))
        self.fig.canvas.draw_idle()

    def on_click(self, ev):
        toolbar = getattr(self.fig.canvas, 'toolbar', None)
        if ev.button != 1 or ev.inaxes is None or (toolbar is not None and toolbar.mode):
            return      # zoom/pan drags are the toolbar's
        log = self.log
        if not log.count:
            return
        if ev.inaxes is self.ax_xy:
            # snap to the nearest path sample, in screen pixels
            pts = self.ax_xy.transData.transform(np.column_stack([log.P[0], log.P[1]]))
            d = (pts[:, 0] - ev.x) ** 2 + (pts[:, 1] - ev.y) ** 2
            self.mark = int(np.argmin(d))
        elif ev.inaxes in self.strips:
            self.mark = int(np.clip(np.searchsorted(log.T, ev.xdata), 0, log.count - 1))
        else:
            return
        self.show_mark()
        print(self.readout.get_text())

    def on_key(self, ev):
        k = (ev.key or '').lower()
        if k == 'ctrl+o':
            self.open_dialog()
        elif k in ('f5', 'ctrl+r'):
            self.load_file(self.path)
        elif k == 'w':
            self.toggle_watch()
        elif k in AXES.lower():
            self.set_axis(AXES.lower().index(k))
        elif k in "12345678":
            self.set_axis(int(k) - 1)
        elif k == 'r' and self.log.count:
            self.ax_xy.autoscale()
            self.strips[0].set_xlim(self.log.T[0], self.log.T[-1])
            self.fig.canvas.draw_idle()
        elif k == 'escape':
            self.mark = None
            self.show_mark()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log', nargs='?', default='/tmp/TPSegLog.csv', help="TPSegLog.csv or a sampled capture CSV")
    ap.add_argument('--axis', default='X', help="axis for the time plots: X Y Z A B C U V (default X)")
    ap.add_argument('--tick', type=float, default=TICK, help="servo tick for re-creating the interpolation (default 0.00009)")
    ap.add_argument('--keep-seams', action='store_true', help="don't align segments at their seams (plotter: 'Remove Knot Seams' off)")
    ap.add_argument('--tail', type=float, default=0.02, help="seconds of hold-at-end samples to append (default 0.02)")
    ap.add_argument('--range', nargs=2, type=float, metavar=('T0', 'T1'), help="initial time range")
    ap.add_argument('--save', metavar='FILE', help="write the figure to FILE (png/pdf/svg) instead of opening a window")
    ap.add_argument('--stats', action='store_true', help="print a summary and exit")
    ap.add_argument('--watch', action='store_true', help="reload the file whenever a run has finished writing it (key w)")
    a = ap.parse_args()
    axis = a.axis.upper()
    if axis not in AXES:
        ap.error("--axis must be one of " + " ".join(AXES))
    build = dict(tick=a.tick, deglitch=not a.keep_seams, tail=a.tail)
    log = Log.load(a.log)
    if log.segs is not None:
        log.build_samples(**build)
    if not log.count:
        sys.exit("%s: no samples" % a.log)
    if a.stats:
        stats(log, a.log)
        return
    import matplotlib
    if a.save:
        matplotlib.use('Agg')
    p = Plotter(log, a.log, AXES.index(axis), a.range, build)
    if a.save:
        p.fig.savefig(a.save, dpi=120)
        print("written", a.save)
    else:
        if a.watch:
            p.toggle_watch(True)
        p.plt.show()


if __name__ == '__main__':
    main()
