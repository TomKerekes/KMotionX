/*
 * kmotion-backend.cc - the KMotion side of kmotion-motion (see kmotion-backend.h).
 *
 * Simulate mode: CCoordMotion runs KMotion's real trajectory planner with
 * m_Simulate and m_DoTime set, which plans and times every segment exactly as
 * for a board but downloads nothing. The finalized segments are copied out of the
 * planner's buffer (the same data the TPSegLog holds) and replayed over time here,
 * so LinuxCNC shows the motion the controller would execute, 3rd order knots,
 * blending and all. The only part of CCoordMotion that talks to a board even in
 * simulate mode is FlushSegments, so the flush is composed here from its public
 * pieces instead.
 *
 * Board mode: the same planner downloads to the board through KMotionServer the
 * way KMotionCNC and kmxWeb do; positions, the executing line (ExecTime) and the
 * end of a run (CheckDoneBuf) are polled from the board.
 *
 * Everything that touches CoordMotion runs in the worker thread.
 */
#include "kmotion-backend.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "GCodeInterpreterX.h"
#include "CoordMotion.h"

static std::mutex g_msg_mutex;
static std::string g_message;                   // operator messages from KMotion
static void err_handler(const char *msg)
{
    std::lock_guard<std::mutex> lock(g_msg_mutex);
    if (!g_message.empty()) g_message += " | ";
    g_message += msg;
}
static int console_handler(const char *msg)     // the board's console output (printf from C programs)
{
    fprintf(stderr, "kmotion-motion: board console: %s", msg);
    return 0;
}

static double now_s()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// a planned segment, as the controller would execute it (the segment log's view)
struct Seg {
    char kind;              // 'L' linear/rapid, 'A' arc, 'D' dwell, 'K' per-axis cubic knot
    int seq;                // our sequence number -> LinuxCNC id
    bool act_frame;         // positions in actuator units (TP3 knots, transformed linears)
    int plane;              // arcs: 0 XY, 1 XZ, 2 YZ (plane-local xc, yc)
    bool ccw;
    double dx, dwell;
    int ntrips;
    double P0[8], P1[8], xc, yc;
    double T[7], A[7], B[7], C[7], D[7];
    double K[32];
    double total;           // duration
};

struct Cmd {
    enum Kind { LINE, ARC, DWELL, ABORT, FEED, SET_POS, MACHINE_ON, JOG_TO } kind;
    double end[9], center[3], normal[3];
    double vel, acc, seconds, feed_scale, rapid_scale;
    bool rapid, on;
    int turn, id, axis;
};

struct KmBackend::Impl {
    KmConfig cfg;
    CKMotionDLL *km = nullptr;
    CCoordMotion *cm = nullptr;
    double scale[8];                // counts per actuator unit
    bool used[8];

    // command queue, main thread -> worker
    std::mutex qmx;
    std::condition_variable qcv;
    std::deque<Cmd> q;
    std::thread worker;             // plans and downloads (may block on the board's flow control)
    std::thread poller;             // board mode: polls status so the display never freezes
    std::mutex wmx;                 // the run bookkeeping below, shared by the two threads
    std::atomic<bool> stop{false};
    std::atomic<bool> paused{false};

    // published state
    std::mutex smx;
    KmState st;
    std::string messages;

    // worker state
    int serial = 0;                 // last sequence number handed to the planner
    int done_upto = 0;              // every sequence number <= this has finished
    std::deque<int> ids;            // LinuxCNC id per sequence number, ids[0] is for done_upto + 1
    bool unflushed = false;         // moves handed over since the last flush
    double last_move_t = 0;
    double pos[9];
    bool enabled[8];
    double feed_scale = 1.0, rapid_scale = 1.0;
    double current_vel = 0;
    // simulate: replay
    int harvested = 0;              // planner segments copied so far (index into its buffer)
    std::deque<Seg> replay;
    Seg cur;
    bool have_cur = false;
    double t_in_seg = 0;
    double last_tick = 0;
    // board
    bool run_active = false;        // a run is downloaded/executing on the board
    double last_poll = 0;
    bool connected = false;

    int id_of(int seq) const
    {
        int k = seq - done_upto - 1;
        return (k >= 0 && k < (int) ids.size()) ? ids[k] : 0;
    }
    void finish_upto(int seq)
    {
        while (done_upto < seq && !ids.empty()) { ids.pop_front(); done_upto++; }
    }
    void post(const Cmd &c)
    {
        std::lock_guard<std::mutex> lock(qmx);
        q.push_back(c);
        qcv.notify_one();
    }
    void message(const std::string &m)
    {
        std::lock_guard<std::mutex> lock(smx);
        if (!messages.empty()) messages += " | ";
        messages += m;
    }
};

// ---- geometry of a segment (the same reconstruction MotionLogPlotter does) ------------
static double dist_at(const Seg &s, double t)
{
    for (int i = 0; i < s.ntrips; i++) {
        if (t <= s.T[i] || i == s.ntrips - 1) {
            if (t > s.T[i]) t = s.T[i];
            return ((s.A[i] * t + s.B[i]) * t + s.C[i]) * t + s.D[i];
        }
        t -= s.T[i];
    }
    return s.dx;
}

static void point_at(const Seg &s, double dist, double o[8])
{
    double f = s.dx > 0 ? dist / s.dx : 0;
    if (f < 0) f = 0; else if (f > 1) f = 1;
    for (int i = 0; i < 8; i++) o[i] = s.P0[i] + f * (s.P1[i] - s.P0[i]);
    if (s.kind == 'A') {
        double r0 = hypot(s.P0[0] - s.xc, s.P0[1] - s.yc);
        double r1 = hypot(s.P1[0] - s.xc, s.P1[1] - s.yc);
        double th0 = atan2(s.P0[1] - s.yc, s.P0[0] - s.xc);
        double th1 = atan2(s.P1[1] - s.yc, s.P1[0] - s.xc);
        double dth = th1 - th0;
        if (s.ccw) { if (dth <= 0) dth += 2 * M_PI; }
        else { if (dth >= 0) dth -= 2 * M_PI; }
        // a planned arc longer than the end points suggest: full turns
        if (r0 > 0 && s.dx > fabs(dth) * r0 * 1.5) {
            double turns = floor((s.dx / r0 - fabs(dth)) / (2 * M_PI) + 0.5);
            dth += (s.ccw ? 1 : -1) * turns * 2 * M_PI;
        }
        double r = r0 + f * (r1 - r0);
        double th = th0 + f * dth;
        double lx = s.xc + r * cos(th), ly = s.yc + r * sin(th);
        double lz = s.P0[2] + f * (s.P1[2] - s.P0[2]);
        if (s.plane == 1) { o[2] = lx; o[0] = ly; o[1] = lz; }
        else if (s.plane == 2) { o[1] = lx; o[0] = lz; o[2] = ly; }
        else { o[0] = lx; o[1] = ly; o[2] = lz; }
    }
}

static void eval(const Seg &s, double t, double o[8])
{
    if (s.kind == 'K') {
        if (t < 0) t = 0; else if (t > s.total) t = s.total;
        for (int a = 0; a < 8; a++) {
            const double *k = &s.K[4 * a];
            o[a] = ((k[0] * t + k[1]) * t + k[2]) * t + k[3];
        }
        return;
    }
    if (s.kind == 'D') { memcpy(o, s.P0, sizeof(double) * 8); return; }
    point_at(s, dist_at(s, t), o);
}

// ---- the planner's buffer -> the replay queue (simulate mode) -------------------------
static void copy_segment(KmBackend::Impl *d, SEGMENT *p, Seg &s)
{
    memset(&s, 0, sizeof s);
    s.kind = 'L';
    if (p->type == SEG_ARC) s.kind = 'A';
    else if (p->type == SEG_DWELL || p->nTrips <= 0) s.kind = 'D';
    if (p->Cubic8) s.kind = 'K';
    s.plane = 0;
    if (p->plane == CANON_PLANE_XZ) s.plane = 1;
    else if (p->plane == CANON_PLANE_YZ) s.plane = 2;
    s.ccw = p->DirIsCCW != 0;
    s.seq = p->sequence_number;
    s.dx = std::isfinite(p->dx) ? p->dx : 0;
    s.dwell = (std::isfinite(p->dwell_time) && p->dwell_time > 0) ? p->dwell_time : 0;
    s.ntrips = p->nTrips;
    double P0[8] = {p->x0, p->y0, p->z0, p->a0, p->b0, p->c0, p->u0, p->v0};
    double P1[8] = {p->x1, p->y1, p->z1, p->a1, p->b1, p->c1, p->u1, p->v1};
    s.act_frame = p->ActSpace || p->Cubic8;
    if (!p->ActSpace && !p->Cubic8 && p->type != SEG_ARC) {
        // a legacy segment: the controller interpolates linearly in actuator space
        // between the transformed end points (SegLogSegment does the same)
        double A0[MAX_ACTUATORS], A1[MAX_ACTUATORS];
        if (!d->cm->Kinematics->TransformCADtoActuators(p->x0, p->y0, p->z0, p->a0, p->b0, p->c0, p->u0, p->v0, A0) &&
            !d->cm->Kinematics->TransformCADtoActuators(p->x1, p->y1, p->z1, p->a1, p->b1, p->c1, p->u1, p->v1, A1)) {
            for (int i = 0; i < 8; i++) {
                P0[i] = d->scale[i] != 0 ? A0[i] / d->scale[i] : 0;
                P1[i] = d->scale[i] != 0 ? A1[i] / d->scale[i] : 0;
            }
            s.act_frame = true;
        }
    }
    memcpy(s.P0, P0, sizeof P0);
    memcpy(s.P1, P1, sizeof P1);
    s.xc = p->xc; s.yc = p->yc;
    for (int i = 0; i < 7; i++) {
        s.T[i] = p->C[i].t; s.A[i] = p->C[i].a; s.B[i] = p->C[i].b; s.C[i] = p->C[i].c; s.D[i] = p->C[i].d;
        if (!std::isfinite(s.T[i]) || s.T[i] < 0) s.T[i] = 0;
        if (!std::isfinite(s.A[i])) s.A[i] = 0;
        if (!std::isfinite(s.B[i])) s.B[i] = 0;
        if (!std::isfinite(s.C[i])) s.C[i] = 0;
        if (!std::isfinite(s.D[i])) s.D[i] = 0;
    }
    if (s.kind == 'K') {
        const double *k = SEG_CUBIC8_COEFFS(p);
        for (int j = 0; j < 32; j++) s.K[j] = std::isfinite(k[j]) ? k[j] : 0;
        s.ntrips = 1;
        s.total = s.T[0] > 0 ? s.T[0] : 0;
    } else {
        s.total = 0;
        for (int i = 0; i < s.ntrips && i < 7; i++) s.total += s.T[i];
        if (s.total <= 0) s.total = s.dwell;
    }
}

static void harvest(KmBackend::Impl *d)
{
    if (!d->cfg.simulate) return;
    for (int i = d->harvested; i < d->cm->m_nsegs_downloaded; i++) {
        Seg s;
        copy_segment(d, GetSegPtr(i), s);
        d->replay.push_back(s);
    }
    d->harvested = d->cm->m_nsegs_downloaded;
}

// actuator positions (counts) back to CAD through the kinematics
static void acts_to_cad(KmBackend::Impl *d, const double acts_in[8], double out[9])
{
    double acts[MAX_ACTUATORS];
    for (int i = 0; i < 8; i++) acts[i] = acts_in[i];
    double x, y, z, a, b, c, u, v;
    if (d->cm->Kinematics->TransformActuatorstoCAD(acts, &x, &y, &z, &a, &b, &c, &u, &v) == 0) {
        out[0] = x; out[1] = y; out[2] = z; out[3] = a; out[4] = b; out[5] = c; out[6] = u; out[7] = v;
    }
    out[8] = 0;
}

static void seg_to_cad(KmBackend::Impl *d, const Seg &s, const double in[8], double out[9])
{
    if (s.act_frame) {
        double acts[8];
        for (int i = 0; i < 8; i++) acts[i] = in[i] * d->scale[i];
        acts_to_cad(d, acts, out);
        return;
    }
    for (int i = 0; i < 8; i++) out[i] = in[i];
    out[8] = 0;
}

static void set_cm_position(KmBackend::Impl *d, const double pos[9])
{
    CCoordMotion *cm = d->cm;
    cm->current_x = pos[0]; cm->current_y = pos[1]; cm->current_z = pos[2];
    cm->current_a = pos[3]; cm->current_b = pos[4]; cm->current_c = pos[5];
    cm->current_u = pos[6]; cm->current_v = pos[7];
}

static void fail(KmBackend::Impl *d, const char *what)
{
    std::string m;
    {
        std::lock_guard<std::mutex> lock(g_msg_mutex);
        m = g_message;
        g_message.clear();
    }
    if (m.empty()) m = std::string(what) + " failed in KMotion's planner";
    d->message(m);
    d->cm->ClearAbort();
    d->cm->ClearHalt();
}

// ---- worker: planning ----------------------------------------------------------------
static int begin_move(KmBackend::Impl *d, int id)
{
    std::lock_guard<std::mutex> lk(d->wmx);
    int seq = ++d->serial;
    d->ids.push_back(id);
    d->unflushed = true;
    d->last_move_t = now_s();
    if (!d->cfg.simulate) d->run_active = true;
    return seq;
}

static void w_line(KmBackend::Impl *d, const Cmd &c)
{
    int seq = begin_move(d, c.id);
    int r = d->cm->StraightFeedAccelRapid(c.vel, c.acc, c.rapid, true, c.end[0], c.end[1], c.end[2], c.end[3], c.end[4], c.end[5], c.end[6], c.end[7], seq, c.id);
    harvest(d);
    if (r) fail(d, "StraightFeed");
}

static void w_arc(KmBackend::Impl *d, const Cmd &c)
{
    // LinuxCNC's center/normal/turn back to rs274ngc's canon arc, which KMotion's
    // ArcFeed takes: the plane from the normal's dominant axis, (first, second,
    // helix) axes in the plane's canonical order, rotation = sign * (turns + 1)
    int plane_axis = 2;
    if (fabs(c.normal[1]) > fabs(c.normal[plane_axis])) plane_axis = 1;
    if (fabs(c.normal[0]) > fabs(c.normal[plane_axis])) plane_axis = 0;
    CANON_PLANE plane;
    int i1, i2, i3;
    if (plane_axis == 2) { plane = CANON_PLANE_XY; i1 = 0; i2 = 1; i3 = 2; }
    else if (plane_axis == 1) { plane = CANON_PLANE_XZ; i1 = 2; i2 = 0; i3 = 1; }
    else { plane = CANON_PLANE_YZ; i1 = 1; i2 = 2; i3 = 0; }
    int turns = c.turn < 0 ? -c.turn : c.turn;
    int rotation = (c.normal[plane_axis] >= 0 ? 1 : -1) * (turns + 1);
    int seq = begin_move(d, c.id);
    int r = d->cm->ArcFeedAccel(c.vel, c.acc, plane, c.end[i1], c.end[i2], c.center[i1], c.center[i2], rotation, c.end[i3],
                                c.end[3], c.end[4], c.end[5], c.end[6], c.end[7], seq, c.id);
    harvest(d);
    if (r) fail(d, "ArcFeed");
}

static void w_dwell(KmBackend::Impl *d, const Cmd &c)
{
    int seq = begin_move(d, c.id);
    int r = d->cm->Dwell(c.seconds, seq);
    harvest(d);
    if (r) fail(d, "Dwell");
}

static void w_abort(KmBackend::Impl *d);

static void w_flush(KmBackend::Impl *d)
{
    CCoordMotion *cm = d->cm;
    {
        std::lock_guard<std::mutex> lk(d->wmx);
        if (!d->unflushed) return;
        d->unflushed = false;
    }
    if (!d->cfg.simulate) {
        // the real thing: finishes the plan, downloads the rest, starts the buffer
        if (cm->FlushSegments()) {
            if (cm->m_AxisDisabled) { d->message("an axis of the coordinate system is disabled: motion refused"); cm->m_AxisDisabled = false; }
            fail(d, "FlushSegments");
            w_abort(d);                 // the moves are lost: let LinuxCNC's queue drain
        }
        return;
    }
    // what FlushSegments does, minus the board: finish a 3rd order streaming run,
    // finalize the remaining segments, "download" them (into the planner's buffer only),
    // then reset for the next run
    if (cm->Kinematics->m_MotionParams.ThirdOrderTP && cm->TP3FlushRun("linuxcnc flush")) { fail(d, "TP3FlushRun"); return; }
    MaximizeSegments();
    for (int iseg = cm->m_nsegs_downloaded; iseg < nsegs; iseg++)
        if (cm->OutputSegment(iseg)) { fail(d, "OutputSegment"); return; }
    harvest(d);
    tp_init();
    cm->TP3ClearRun();
    cm->DownloadInit();
    d->harvested = 0;
}

static void w_abort(KmBackend::Impl *d)
{
    CCoordMotion *cm = d->cm;
    cm->SetAbort();
    cm->ClearAbort();                 // re-initializes the planner
    cm->ClearHalt();
    d->replay.clear();
    d->have_cur = false;
    d->t_in_seg = 0;
    d->harvested = 0;
    {
        std::lock_guard<std::mutex> lk(d->wmx);
        d->unflushed = false;
        d->run_active = false;
        d->ids.clear();
        d->done_upto = d->serial;
    }
    d->current_vel = 0;
    if (d->cfg.simulate) {
        set_cm_position(d, d->pos);
    } else {
        cm->RearmCoordLaunch();
        double x, y, z, a, b, c, u, v;
        if (cm->ReadCurAbsPosition(&x, &y, &z, &a, &b, &c, &u, &v) == 0) {
            double p[9] = {x, y, z, a, b, c, u, v, 0};
            memcpy(d->pos, p, sizeof p);
        }
        cm->ClearAbort();
    }
}

static void w_replay(KmBackend::Impl *d, double dt)
{
    double before[9];
    memcpy(before, d->pos, sizeof before);
    d->current_vel = 0;
    if (!d->paused) {
        // the board's feed override scales the whole coordinated motion's clock
        double adv = dt * (d->feed_scale > 0 ? d->feed_scale : 0);
        while (adv > 0) {
            if (!d->have_cur) {
                if (d->replay.empty()) break;
                d->cur = d->replay.front();
                d->replay.pop_front();
                d->have_cur = true;
                d->t_in_seg = 0;
                // a move spans many planned segments (hundreds of knots): it is finished
                // once a segment of a later move starts
                std::lock_guard<std::mutex> lk(d->wmx);
                d->finish_upto(d->cur.seq - 1);
            }
            double left = d->cur.total - d->t_in_seg;
            if (adv >= left) {
                adv -= left;
                double o[8];
                eval(d->cur, d->cur.total, o);
                seg_to_cad(d, d->cur, o, d->pos);
                d->have_cur = false;
            } else {
                d->t_in_seg += adv;
                adv = 0;
                double o[8];
                eval(d->cur, d->t_in_seg, o);
                seg_to_cad(d, d->cur, o, d->pos);
            }
        }
        double dd = 0;
        for (int i = 0; i < 3; i++) dd += (d->pos[i] - before[i]) * (d->pos[i] - before[i]);
        d->current_vel = dt > 0 ? sqrt(dd) / dt : 0;
    }
    std::lock_guard<std::mutex> lk(d->wmx);
    if (!d->have_cur && d->replay.empty() && !d->unflushed) d->finish_upto(d->serial);
}

// ---- worker: the board ------------------------------------------------------------------
static int board_query(KmBackend::Impl *d, const char *cmd, char *reply)
{
    reply[0] = 0;
    return d->km->WriteLineReadLine(cmd, reply);
}

// CoordMotion::UpdateRealTimeState is private: the same walk over the executing
// buffer, from the public planner globals - which downloaded segment the board's
// execution time T falls into, hence the sequence number being executed
static bool realtime_sequence(double T, int &seq)
{
    if (T < 0.0) return false;                      // nothing executing
    SEGMENT *segs = segments_executing;
    if (segs != segments0 && segs != segments1) return false;
    int index = (segs == segments0) ? 0 : 1;
    double BufTime = SegsDoneTime[index];
    if (SegsDone[index] == -1) {
        if (special_cmds_initial_sequence_no[index] >= 0) { seq = special_cmds_initial_sequence_no[index]; return true; }
        return false;
    }
    int i;
    for (i = SegsDone[index]; i >= 0; i--) {
        for (int k = segs[TPMOD(i)].nTrips - 1; k >= 0; k--) {
            if (BufTime <= T) break;
            BufTime -= segs[TPMOD(i)].C[k].t;
        }
        if (BufTime <= T || (T == 0.0 && BufTime < 1e-6)) break;
    }
    if (i < 0) return false;
    seq = segs[TPMOD(i)].sequence_number;
    return true;
}

static void w_poll_board(KmBackend::Impl *d)
{
    CCoordMotion *cm = d->cm;
    MAIN_STATUS status;
    memset(&status, 0, sizeof status);
    if (d->km->WaitToken(false, 100, "kmotion-motion") != KMOTION_LOCKED) {
        d->connected = false;
        return;
    }
    int r = d->km->GetStatus(status, false);
    d->km->ReleaseToken();
    if (r) {
        if (d->connected) d->message("lost the board: GetStatus failed");
        d->connected = false;
        return;
    }
    d->connected = true;
    // positions: the commanded destinations (open-loop machines report no other position)
    double acts[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    double before[9];
    memcpy(before, d->pos, sizeof before);
    for (int i = 0; i < 8; i++) {
        int ch = d->cfg.channel[i];
        if (ch >= 0 && ch < N_CHANNELS_KOGNA) {
            acts[i] = status.Dest[ch];
            d->enabled[i] = (status.Enables >> ch) & 1;
        } else {
            d->enabled[i] = true;
        }
    }
    acts_to_cad(d, acts, d->pos);
    // speed from the board's own clock between two status reads (the host-side interval
    // includes the query latency and reads low)
    static double last_stamp = 0;
    double dt = status.TimeStamp - last_stamp;
    last_stamp = status.TimeStamp;
    double dd = 0;
    for (int i = 0; i < 3; i++) dd += (d->pos[i] - before[i]) * (d->pos[i] - before[i]);
    if (dt > 0.001 && dt < 1.0) d->current_vel = sqrt(dd) / dt;

    bool active, flushed;
    {
        std::lock_guard<std::mutex> lk(d->wmx);
        active = d->run_active;
        flushed = !d->unflushed;
    }
    if (active && cm->CoordLaunched()) {
        char reply[MAX_LINE + 1];
        // where the board is in the downloaded buffer: the executing line
        if (board_query(d, "ExecTime", reply) == 0) {
            double T;
            int seq;
            if (sscanf(reply, "%lf", &T) == 1 && realtime_sequence(T, seq)) {
                std::lock_guard<std::mutex> lk(d->wmx);
                if (seq > d->done_upto && seq <= d->serial) d->finish_upto(seq - 1);
            }
        }
        if (flushed && board_query(d, "CheckDoneBuf", reply) == 0) {
            if (strcmp(reply, "1") == 0) {
                // the buffer ran to its end: the run is complete
                std::lock_guard<std::mutex> lk(d->wmx);
                if (!d->unflushed) {
                    d->finish_upto(d->serial);
                    d->run_active = false;
                    cm->RearmCoordLaunch();
                }
            } else if (strcmp(reply, "-1") == 0) {
                d->message("an axis is disabled: motion aborted");
                cm->SetAbort();
                Cmd c;
                memset(&c, 0, sizeof c);
                c.kind = Cmd::ABORT;
                d->post(c);
            }
        }
    }
}

// what the main thread reads
static void publish(KmBackend::Impl *d)
{
    std::lock_guard<std::mutex> lock(d->smx);
    std::lock_guard<std::mutex> lk(d->wmx);
    KmState &s = d->st;
    memcpy(s.pos, d->pos, sizeof s.pos);
    for (int i = 0; i < 8; i++) s.enabled[i] = d->cfg.simulate ? true : d->enabled[i];
    s.depth = d->serial - d->done_upto;
    s.running = d->unflushed || d->run_active || d->have_cur || !d->replay.empty();
    s.active_id = s.depth > 0 ? d->id_of(d->done_upto + 1) : 0;
    s.paused = d->paused;
    s.connected = d->cfg.simulate ? true : d->connected;
    s.current_vel = d->current_vel;
    s.distance_to_go = (d->cfg.simulate && d->have_cur && d->cur.kind != 'K' && d->cur.total > 0)
                       ? d->cur.dx * (1 - d->t_in_seg / d->cur.total) : 0;
    std::lock_guard<std::mutex> lock2(g_msg_mutex);
    if (!g_message.empty()) {
        if (!d->messages.empty()) d->messages += " | ";
        d->messages += g_message;
        g_message.clear();
    }
}

// board mode: status at its own pace, whatever the planner is waiting on
static void poller_main(KmBackend::Impl *d)
{
    while (!d->stop) {
        double t = now_s();
        w_poll_board(d);
        d->last_poll = t;
        publish(d);
        double sleep_s = d->cfg.status_period - (now_s() - t);
        if (sleep_s > 0) std::this_thread::sleep_for(std::chrono::duration<double>(sleep_s));
    }
}

static int run_program(KmBackend::Impl *d, const char *path, int thread)
{
    char err[512] = "";
    if (d->km->CompileAndLoadCoff(path, thread, err, sizeof err - 1)) {
        d->message(std::string("init program failed: ") + path + ": " + err);
        return 1;
    }
    char cmd[64], reply[MAX_LINE + 1];
    snprintf(cmd, sizeof cmd, "Execute%d", thread);
    if (d->km->WriteLine(cmd)) { d->message("Execute failed for the init program"); return 1; }
    snprintf(cmd, sizeof cmd, "CheckThread%d", thread);
    double t0 = now_s();
    for (;;) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (board_query(d, cmd, reply)) { d->message("CheckThread failed"); return 1; }
        if (strcmp(reply, "0") == 0) return 0;
        if (now_s() - t0 > 30) { d->message("the init program did not finish within 30 s"); return 1; }
    }
}

static void w_machine_on(KmBackend::Impl *d, bool on)
{
    char cmd[64];
    MAIN_STATUS status;
    memset(&status, 0, sizeof status);
    bool have_status = d->km->GetStatus(status, true) == 0;
    for (int i = 0; i < 8; i++) {
        int ch = d->cfg.channel[i];
        if (ch < 0) continue;
        if (on) {
            // enable at the current destination: no jump for open-loop axes
            if (have_status) snprintf(cmd, sizeof cmd, "EnableAxisDest%d %.0f", ch, status.Dest[ch]);   // console form: no "="
            else snprintf(cmd, sizeof cmd, "EnableAxis%d", ch);
        } else {
            snprintf(cmd, sizeof cmd, "DisableAxis%d", ch);
        }
        if (d->km->WriteLine(cmd)) { d->message(std::string("board command failed: ") + cmd); return; }
    }
}

static void w_jog_to(KmBackend::Impl *d, const Cmd &c)
{
    // an incremental or absolute jog: a move of the one actuator to the target counts
    if (c.axis < 0 || c.axis >= 8) return;
    int ch = d->cfg.channel[c.axis];
    if (ch < 0) return;
    double target[9];
    memcpy(target, d->pos, sizeof target);
    target[c.axis] = c.end[c.axis];
    double acts[MAX_ACTUATORS];
    if (d->cm->Kinematics->TransformCADtoActuators(target[0], target[1], target[2], target[3], target[4], target[5], target[6], target[7], acts)) return;
    char cmd[64];
    snprintf(cmd, sizeof cmd, "Move%d=%.3f", ch, acts[c.axis]);
    if (d->km->WriteLine(cmd)) d->message(std::string("board command failed: ") + cmd);
}

// ---- worker loop --------------------------------------------------------------------------
static void worker_main(KmBackend::Impl *d)
{
    d->last_tick = d->last_poll = now_s();
    while (!d->stop) {
        // commands
        for (;;) {
            Cmd c;
            {
                std::unique_lock<std::mutex> lock(d->qmx);
                if (d->q.empty()) break;
                c = d->q.front();
                d->q.pop_front();
            }
            switch (c.kind) {
            case Cmd::LINE: w_line(d, c); break;
            case Cmd::ARC: w_arc(d, c); break;
            case Cmd::DWELL: w_dwell(d, c); break;
            case Cmd::ABORT: w_abort(d); break;
            case Cmd::FEED:
                d->feed_scale = c.feed_scale;
                d->rapid_scale = c.rapid_scale;
                if (!d->cfg.simulate) {
                    d->cm->SetFeedRateOverride(c.feed_scale);
                    d->cm->SetFeedRateRapidOverride(c.rapid_scale);
                }
                break;
            case Cmd::SET_POS:
                if (d->cfg.simulate && !d->have_cur && d->replay.empty() && !d->unflushed) {
                    memcpy(d->pos, c.end, sizeof d->pos);
                    set_cm_position(d, d->pos);
                }
                break;
            case Cmd::MACHINE_ON: if (!d->cfg.simulate) w_machine_on(d, c.on); break;
            case Cmd::JOG_TO: if (!d->cfg.simulate) w_jog_to(d, c); break;
            }
        }
        double t = now_s();
        // the planner finalizes the last moves only when told that no more are coming:
        // flush once task has been quiet for a moment (program end, M0, tool change, dwell)
        bool flush_now;
        {
            std::lock_guard<std::mutex> lk(d->wmx);
            flush_now = d->unflushed && t - d->last_move_t > 0.05;
        }
        if (flush_now) w_flush(d);
        if (d->cfg.simulate) {
            double dt = t - d->last_tick;
            if (dt > 0.1) dt = 0.1;
            w_replay(d, dt);
            d->last_tick = t;
            publish(d);
        }
        std::unique_lock<std::mutex> lock(d->qmx);
        if (d->q.empty()) d->qcv.wait_for(lock, std::chrono::milliseconds(5));
    }
}

// ---- KmBackend ----------------------------------------------------------------------------
KmBackend::KmBackend() : d(new Impl)
{
    memset(&d->st, 0, sizeof d->st);
    memset(d->pos, 0, sizeof d->pos);
    for (int i = 0; i < 8; i++) { d->enabled[i] = true; d->scale[i] = 1; d->used[i] = false; }
}

KmBackend::~KmBackend()
{
    d->stop = true;
    d->qcv.notify_one();
    if (d->worker.joinable()) d->worker.join();
    if (d->poller.joinable()) d->poller.join();
    delete d->cm;
    delete d->km;
    delete d;
}

bool KmBackend::is_board() const { return !d->cfg.simulate; }

const char *KmBackend::mode_name() const
{
    return d->cfg.simulate ? "KMotion planner, simulated execution" : "KMotion planner and board";
}

bool KmBackend::init(const KmConfig &cfg, const double pos[9])
{
    d->cfg = cfg;
    memcpy(d->pos, pos, sizeof d->pos);
    d->km = new CKMotionDLL(0);
    d->km->SetErrMsgCallback(err_handler);
    d->cm = new CCoordMotion(d->km);
    CCoordMotion *cm = d->cm;
    cm->m_Simulate = cfg.simulate;
    cm->m_DoTime = true;            // plan with real timing even without a board

    MOTION_PARAMS *MP = cm->GetMotionParams();
    MP->BreakAngle = cfg.break_angle;
    MP->CollinearTol = cfg.collinear_tol;
    MP->CornerTol = cfg.corner_tol;
    MP->FacetAngle = cfg.facet_angle;
    MP->TPLookahead = cfg.lookahead;
    MP->MaxRapidFRO = 1.0;
    MP->DegreesA = MP->DegreesB = MP->DegreesC = true;
    double *counts[8] = {&MP->CountsPerInchX, &MP->CountsPerInchY, &MP->CountsPerInchZ, &MP->CountsPerInchA,
                         &MP->CountsPerInchB, &MP->CountsPerInchC, &MP->CountsPerInchU, &MP->CountsPerInchV};
    double *vel[8] = {&MP->MaxVelX, &MP->MaxVelY, &MP->MaxVelZ, &MP->MaxVelA, &MP->MaxVelB, &MP->MaxVelC, &MP->MaxVelU, &MP->MaxVelV};
    double *acc[8] = {&MP->MaxAccelX, &MP->MaxAccelY, &MP->MaxAccelZ, &MP->MaxAccelA, &MP->MaxAccelB, &MP->MaxAccelC, &MP->MaxAccelU, &MP->MaxAccelV};
    double *jerk[8] = {&MP->MaxJerkX, &MP->MaxJerkY, &MP->MaxJerkZ, &MP->MaxJerkA, &MP->MaxJerkB, &MP->MaxJerkC, &MP->MaxJerkU, &MP->MaxJerkV};
    double *rvel[8] = {&MP->MaxRapidVelX, &MP->MaxRapidVelY, &MP->MaxRapidVelZ, &MP->MaxRapidVelA, &MP->MaxRapidVelB, &MP->MaxRapidVelC, &MP->MaxRapidVelU, &MP->MaxRapidVelV};
    double *racc[8] = {&MP->MaxRapidAccelX, &MP->MaxRapidAccelY, &MP->MaxRapidAccelZ, &MP->MaxRapidAccelA, &MP->MaxRapidAccelB, &MP->MaxRapidAccelC, &MP->MaxRapidAccelU, &MP->MaxRapidAccelV};
    double *rjerk[8] = {&MP->MaxRapidJerkX, &MP->MaxRapidJerkY, &MP->MaxRapidJerkZ, &MP->MaxRapidJerkA, &MP->MaxRapidJerkB, &MP->MaxRapidJerkC, &MP->MaxRapidJerkU, &MP->MaxRapidJerkV};
    double *spos[8] = {&MP->SoftLimitPosX, &MP->SoftLimitPosY, &MP->SoftLimitPosZ, &MP->SoftLimitPosA, &MP->SoftLimitPosB, &MP->SoftLimitPosC, &MP->SoftLimitPosU, &MP->SoftLimitPosV};
    double *sneg[8] = {&MP->SoftLimitNegX, &MP->SoftLimitNegY, &MP->SoftLimitNegZ, &MP->SoftLimitNegA, &MP->SoftLimitNegB, &MP->SoftLimitNegC, &MP->SoftLimitNegU, &MP->SoftLimitNegV};
    for (int i = 0; i < 8; i++) {
        const KmAxisParams &ax = cfg.axis[i];
        d->used[i] = ax.counts_per_unit > 0;
        // kmxWeb's defaults for an undefined axis: the planner needs non-zero numbers
        double c = ax.counts_per_unit > 0 ? ax.counts_per_unit : 100.0;
        double v = ax.max_vel > 0 ? ax.max_vel : 0.1;
        double a = ax.max_accel > 0 ? ax.max_accel : 0.01;
        double j = ax.max_jerk > 0 ? ax.max_jerk : 10.0 * a;
        *counts[i] = c; *vel[i] = v; *acc[i] = a; *jerk[i] = j;
        *rvel[i] = v; *racc[i] = a; *rjerk[i] = j;
        *spos[i] = 1e9; *sneg[i] = -1e9;          // LinuxCNC enforces its own limits
        d->scale[i] = c;
        // one actuator per axis
        MP->ActScale[i] = d->used[i] ? c : 0;
        MP->MaxActVel[i] = d->used[i] ? v : 0;
        MP->MaxActAccel[i] = d->used[i] ? a : 0;
        MP->MaxActJerk[i] = d->used[i] ? j : 0;
        MP->ActDegrees[i] = (i >= 3 && i <= 5);
    }
    MP->ThirdOrderTP = cfg.third_order;
    MP->CubicKnots = cfg.third_order && cfg.cubic_knots;
    MP->ActuatorLimits = cfg.actuator_limits;
    MP->LogSegments = cfg.log_segments;
    cm->SetTPParams();

    if (cfg.simulate) {
        // the coordinate system, without asking the board
        cm->x_axis = cfg.channel[0]; cm->y_axis = cfg.channel[1]; cm->z_axis = cfg.channel[2];
        cm->a_axis = cfg.channel[3]; cm->b_axis = cfg.channel[4]; cm->c_axis = cfg.channel[5];
        cm->u_axis = cfg.channel[6]; cm->v_axis = cfg.channel[7];
        cm->m_DefineCS_valid = cm->m_DefineCS_known = true;
        // rapids use the board's own axis settings and GetRapidSettings would fetch them
        // (even in simulate mode it tries, which launches KMotionServer): the rapid limits
        // are set above, so mark them fresh
        cm->RapidParamsDirty = false;
        set_cm_position(d, pos);
    } else {
        int type = 0;
        if (d->km->CheckKMotionVersion(&type)) {
            std::string m;
            { std::lock_guard<std::mutex> lock(g_msg_mutex); m = g_message; g_message.clear(); }
            fprintf(stderr, "kmotion-motion: no board, or firmware mismatch: %s\n", m.c_str());
            return false;
        }
        fprintf(stderr, "kmotion-motion: connected to a %s\n", type == BOARD_TYPE_KOGNA ? "Kogna" : "KFLOP");
        d->km->SetConsoleCallback(console_handler);     // registers with the server: board mode only
        if (cfg.init_program[0] && run_program(d, cfg.init_program, cfg.init_thread)) {
            std::string m;
            { std::lock_guard<std::mutex> lock(d->smx); m = d->messages; }
            fprintf(stderr, "kmotion-motion: %s\n", m.c_str());
            return false;
        }
        if (cm->SetAxisDefinitions(cfg.channel[0], cfg.channel[1], cfg.channel[2], cfg.channel[3],
                                   cfg.channel[4], cfg.channel[5], cfg.channel[6], cfg.channel[7])) {
            fprintf(stderr, "kmotion-motion: DefineCS failed\n");
            return false;
        }
        double x, y, z, a, b, c, u, v;
        if (cm->ReadCurAbsPosition(&x, &y, &z, &a, &b, &c, &u, &v)) {
            fprintf(stderr, "kmotion-motion: reading the board position failed\n");
            cm->ClearAbort();
            return false;
        }
        double p[9] = {x, y, z, a, b, c, u, v, 0};
        memcpy(d->pos, p, sizeof p);
        d->connected = true;
    }
    cm->ClearAbort();
    cm->ClearHalt();
    {
        std::lock_guard<std::mutex> lock(d->smx);
        memcpy(d->st.pos, d->pos, sizeof d->st.pos);
        for (int i = 0; i < 8; i++) d->st.enabled[i] = true;
        d->st.connected = true;
    }
    d->worker = std::thread(worker_main, d);
    if (!cfg.simulate) d->poller = std::thread(poller_main, d);
    return true;
}

void KmBackend::set_position(const double pos[9])
{
    Cmd c;
    memset(&c, 0, sizeof c);
    c.kind = Cmd::SET_POS;
    memcpy(c.end, pos, sizeof c.end);
    d->post(c);
}

int KmBackend::line(const double end[9], double vel, double acc, bool rapid, int id)
{
    Cmd c;
    memset(&c, 0, sizeof c);
    c.kind = Cmd::LINE;
    memcpy(c.end, end, sizeof c.end);
    c.vel = vel; c.acc = acc; c.rapid = rapid; c.id = id;
    d->post(c);
    return 0;
}

int KmBackend::arc(const double end[9], const double center[3], const double normal[3], int turn, double vel, double acc, int id)
{
    Cmd c;
    memset(&c, 0, sizeof c);
    c.kind = Cmd::ARC;
    memcpy(c.end, end, sizeof c.end);
    memcpy(c.center, center, sizeof c.center);
    memcpy(c.normal, normal, sizeof c.normal);
    c.turn = turn; c.vel = vel; c.acc = acc; c.id = id;
    d->post(c);
    return 0;
}

int KmBackend::dwell(double seconds, int id)
{
    Cmd c;
    memset(&c, 0, sizeof c);
    c.kind = Cmd::DWELL;
    c.seconds = seconds; c.id = id;
    d->post(c);
    return 0;
}

void KmBackend::abort()
{
    // break the worker out of any wait on the board, and stop the board now
    d->cm->SetAbort();
    if (!d->cfg.simulate) d->km->WriteLine("StopImmediate2");
    d->paused = false;
    Cmd c;
    memset(&c, 0, sizeof c);
    c.kind = Cmd::ABORT;
    d->post(c);
}

void KmBackend::pause(bool on)
{
    d->paused = on;
    if (!d->cfg.simulate) d->km->WriteLine(on ? "StopImmediate0" : "StopImmediate1");   // feed hold / resume
}

void KmBackend::set_feed_override(double feed_scale, double rapid_scale)
{
    Cmd c;
    memset(&c, 0, sizeof c);
    c.kind = Cmd::FEED;
    c.feed_scale = feed_scale; c.rapid_scale = rapid_scale;
    d->post(c);
}

void KmBackend::machine_on(bool on)
{
    Cmd c;
    memset(&c, 0, sizeof c);
    c.kind = Cmd::MACHINE_ON;
    c.on = on;
    d->post(c);
}

void KmBackend::jog(int axis, double vel)
{
    if (d->cfg.simulate || axis < 0 || axis >= 8) return;
    int ch = d->cfg.channel[axis];
    if (ch < 0) return;
    char cmd[64];
    snprintf(cmd, sizeof cmd, "Jog%d=%.3f", ch, vel * d->scale[axis]);   // counts/s, the board's limits apply
    if (d->km->WriteLine(cmd)) d->message(std::string("board command failed: ") + cmd);
}

void KmBackend::jog_to(int axis, double target, double)
{
    if (d->cfg.simulate || axis < 0 || axis >= 8) return;
    Cmd c;
    memset(&c, 0, sizeof c);
    c.kind = Cmd::JOG_TO;
    c.axis = axis;
    c.end[axis] = target;
    d->post(c);
}

void KmBackend::jog_stop(int axis)
{
    if (d->cfg.simulate) return;
    for (int i = 0; i < 8; i++)
        if ((axis < 0 || axis == i) && d->cfg.channel[i] >= 0) jog(i, 0);
}

void KmBackend::state(KmState &out)
{
    std::lock_guard<std::mutex> lock(d->smx);
    out = d->st;
    out.message[0] = 0;
    if (!d->messages.empty()) {
        snprintf(out.message, sizeof out.message, "%s", d->messages.c_str());
        d->messages.clear();
    }
}
