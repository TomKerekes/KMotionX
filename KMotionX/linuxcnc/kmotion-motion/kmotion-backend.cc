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
 */
#include "kmotion-backend.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>

#include "GCodeInterpreterX.h"
#include "CoordMotion.h"

static std::string g_message;                   // operator messages from KMotion
static void err_handler(const char *msg)
{
    if (!g_message.empty()) g_message += " | ";
    g_message += msg;
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
    double start[8];        // set when it starts
};

struct KmBackend::Impl {
    KmConfig cfg;
    CKMotionDLL *km = nullptr;
    CCoordMotion *cm = nullptr;
    int serial = 0;                 // last sequence number handed to the planner
    int done_upto = 0;              // every sequence number <= this has finished
    std::deque<int> ids;            // LinuxCNC id per sequence number, ids[0] is for done_upto + 1
    int harvested = 0;              // planner segments copied so far (index into its buffer)
    bool unflushed = false;         // moves handed over since the last flush
    std::deque<Seg> replay;
    Seg cur;
    bool have_cur = false;
    double t_in_seg = 0;
    double pos[9];
    bool paused = false;
    double feed_scale = 1.0, rapid_scale = 1.0;
    double current_vel = 0;
    double scale[8];                // counts per actuator unit, for the actuator frame
    bool used[8];

    int id_of(int seq) const
    {
        int k = seq - done_upto - 1;
        return (k >= 0 && k < (int) ids.size()) ? ids[k] : 0;
    }
    void finish_upto(int seq)
    {
        while (done_upto < seq && !ids.empty()) { ids.pop_front(); done_upto++; }
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
        // a planned arc longer than the chord suggests: full turns
        double r = r0 + f * (r1 - r0);
        if (r0 > 0 && s.dx > fabs(dth) * r0 * 1.5) {
            double turns = floor((s.dx / r0 - fabs(dth)) / (2 * M_PI) + 0.5);
            dth += (s.ccw ? 1 : -1) * turns * 2 * M_PI;
        }
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

// ---- the planner's buffer -> our replay queue ---------------------------------------
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
    for (int i = d->harvested; i < d->cm->m_nsegs_downloaded; i++) {
        Seg s;
        copy_segment(d, GetSegPtr(i), s);
        d->replay.push_back(s);
    }
    d->harvested = d->cm->m_nsegs_downloaded;
}

// actuator-frame positions back to CAD through the kinematics
static void to_cad(KmBackend::Impl *d, const Seg &s, const double in[8], double out[9])
{
    if (s.act_frame) {
        double acts[MAX_ACTUATORS];
        for (int i = 0; i < 8; i++) acts[i] = in[i] * d->scale[i];
        double x, y, z, a, b, c, u, v;
        if (d->cm->Kinematics->TransformActuatorstoCAD(acts, &x, &y, &z, &a, &b, &c, &u, &v) == 0) {
            out[0] = x; out[1] = y; out[2] = z; out[3] = a; out[4] = b; out[5] = c; out[6] = u; out[7] = v;
            out[8] = 0;
            return;
        }
    }
    for (int i = 0; i < 8; i++) out[i] = in[i];
    out[8] = 0;
}

// ---- KmBackend --------------------------------------------------------------------------
KmBackend::KmBackend() : d(new Impl) {}
KmBackend::~KmBackend()
{
    delete d->cm;
    delete d->km;
    delete d;
}

const char *KmBackend::mode_name() const { return d->cfg.simulate ? "KMotion planner, simulated execution" : "KMotion planner and board"; }

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

    // the coordinate system, without asking the board
    cm->x_axis = cfg.channel[0]; cm->y_axis = cfg.channel[1]; cm->z_axis = cfg.channel[2];
    cm->a_axis = cfg.channel[3]; cm->b_axis = cfg.channel[4]; cm->c_axis = cfg.channel[5];
    cm->u_axis = cfg.channel[6]; cm->v_axis = cfg.channel[7];
    cm->m_DefineCS_valid = cm->m_DefineCS_known = true;

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
    // rapids use the board's own axis settings and GetRapidSettings would fetch them
    // (even in simulate mode it tries, which launches KMotionServer): the rapid limits
    // are set above, so mark them fresh
    if (cfg.simulate) cm->RapidParamsDirty = false;
    cm->ClearAbort();
    cm->ClearHalt();
    set_position(pos);
    return true;
}

void KmBackend::set_position(const double pos[9])
{
    memcpy(d->pos, pos, sizeof d->pos);
    CCoordMotion *cm = d->cm;
    cm->current_x = pos[0]; cm->current_y = pos[1]; cm->current_z = pos[2];
    cm->current_a = pos[3]; cm->current_b = pos[4]; cm->current_c = pos[5];
    cm->current_u = pos[6]; cm->current_v = pos[7];
}

static int fail(KmBackend::Impl *d, const char *what)
{
    if (g_message.empty()) g_message = std::string(what) + " failed in KMotion's planner";
    d->cm->ClearAbort();
    return 1;
}

int KmBackend::line(const double end[9], double vel, double acc, bool rapid, int id)
{
    int seq = ++d->serial;
    d->ids.push_back(id);
    d->unflushed = true;
    int r = d->cm->StraightFeedAccelRapid(vel, acc, rapid, true, end[0], end[1], end[2], end[3], end[4], end[5], end[6], end[7], seq, id);
    harvest(d);
    return r ? fail(d, "StraightFeed") : 0;
}

int KmBackend::arc(const double end[9], const double center[3], const double normal[3], int turn, double vel, double acc, int id)
{
    // LinuxCNC's center/normal/turn back to rs274ngc's canon arc, which KMotion's
    // ArcFeed takes: the plane from the normal's dominant axis, (first, second,
    // helix) axes in the plane's canonical order, rotation = sign * (turns + 1)
    int plane_axis = 2;
    if (fabs(normal[1]) > fabs(normal[plane_axis])) plane_axis = 1;
    if (fabs(normal[0]) > fabs(normal[plane_axis])) plane_axis = 0;
    CANON_PLANE plane;
    int i1, i2, i3;
    if (plane_axis == 2) { plane = CANON_PLANE_XY; i1 = 0; i2 = 1; i3 = 2; }
    else if (plane_axis == 1) { plane = CANON_PLANE_XZ; i1 = 2; i2 = 0; i3 = 1; }
    else { plane = CANON_PLANE_YZ; i1 = 1; i2 = 2; i3 = 0; }
    int turns = turn < 0 ? -turn : turn;
    int rotation = (normal[plane_axis] >= 0 ? 1 : -1) * (turns + 1);
    int seq = ++d->serial;
    d->ids.push_back(id);
    d->unflushed = true;
    int r = d->cm->ArcFeedAccel(vel, acc, plane, end[i1], end[i2], center[i1], center[i2], rotation, end[i3],
                                end[3], end[4], end[5], end[6], end[7], seq, id);
    harvest(d);
    return r ? fail(d, "ArcFeed") : 0;
}

int KmBackend::dwell(double seconds, int id)
{
    int seq = ++d->serial;
    d->ids.push_back(id);
    d->unflushed = true;
    int r = d->cm->Dwell(seconds, seq);
    harvest(d);
    return r ? fail(d, "Dwell") : 0;
}

bool KmBackend::needs_flush() const { return d->unflushed; }

int KmBackend::flush()
{
    if (!d->unflushed) return 0;
    CCoordMotion *cm = d->cm;
    // what FlushSegments does, minus the board: finish a 3rd order streaming run,
    // finalize the remaining segments, "download" them (into the planner's buffer only,
    // in simulate mode), then reset for the next run
    if (cm->Kinematics->m_MotionParams.ThirdOrderTP && cm->TP3FlushRun("linuxcnc flush")) return fail(d, "TP3FlushRun");
    MaximizeSegments();
    for (int iseg = cm->m_nsegs_downloaded; iseg < nsegs; iseg++)
        if (cm->OutputSegment(iseg)) return fail(d, "OutputSegment");
    harvest(d);
    tp_init();
    cm->TP3ClearRun();
    cm->DownloadInit();
    d->harvested = 0;
    d->unflushed = false;
    return 0;
}

void KmBackend::abort()
{
    CCoordMotion *cm = d->cm;
    cm->SetAbort();
    cm->ClearAbort();                 // re-initializes the planner
    cm->ClearHalt();
    d->replay.clear();
    d->have_cur = false;
    d->t_in_seg = 0;
    d->harvested = 0;
    d->unflushed = false;
    d->ids.clear();
    d->done_upto = d->serial;
    d->current_vel = 0;
    set_position(d->pos);
}

void KmBackend::pause(bool on) { d->paused = on; }

void KmBackend::set_feed_override(double feed_scale, double rapid_scale)
{
    d->feed_scale = feed_scale;
    d->rapid_scale = rapid_scale;
}

void KmBackend::step(double dt, KmState &out)
{
    Impl *d = this->d;
    d->current_vel = 0;
    double before[9];
    memcpy(before, d->pos, sizeof before);     // the whole step's displacement gives the speed
    if (!d->paused) {
        // the board's feed override scales the whole coordinated motion's clock
        double adv = dt * (d->feed_scale > 0 ? d->feed_scale : 0);
        while (adv > 0) {
            if (!d->have_cur) {
                if (d->replay.empty()) break;
                d->cur = d->replay.front();
                d->replay.pop_front();
                memcpy(d->cur.start, d->pos, sizeof d->cur.start);
                d->have_cur = true;
                d->t_in_seg = 0;
                // a move spans many planned segments (hundreds of knots): it is finished
                // once a segment of a later move starts
                d->finish_upto(d->cur.seq - 1);
            }
            double left = d->cur.total - d->t_in_seg;
            if (adv >= left) {
                adv -= left;
                double o[8];
                eval(d->cur, d->cur.total, o);
                to_cad(d, d->cur, o, d->pos);
                d->have_cur = false;
            } else {
                d->t_in_seg += adv;
                adv = 0;
                double o[8];
                eval(d->cur, d->t_in_seg, o);
                to_cad(d, d->cur, o, d->pos);
            }
        }
        double dd = 0;
        for (int i = 0; i < 3; i++) dd += (d->pos[i] - before[i]) * (d->pos[i] - before[i]);
        d->current_vel = dt > 0 ? sqrt(dd) / dt : 0;
    }
    if (!d->have_cur && d->replay.empty() && !d->unflushed) d->finish_upto(d->serial);

    memcpy(out.pos, d->pos, sizeof out.pos);
    out.running = d->have_cur || !d->replay.empty() || d->unflushed;
    out.depth = d->serial - d->done_upto;
    out.active_id = d->have_cur ? d->id_of(d->cur.seq) : (out.depth > 0 ? d->id_of(d->done_upto + 1) : 0);
    out.current_vel = d->current_vel;
    out.distance_to_go = d->have_cur && d->cur.kind != 'K' && d->cur.total > 0 ? d->cur.dx * (1 - d->t_in_seg / d->cur.total) : 0;
    out.message[0] = 0;
    if (!g_message.empty()) {
        snprintf(out.message, sizeof out.message, "%s", g_message.c_str());
        g_message.clear();
    }
}
