/* kmotion-joglimit.h: a world (teleop) jog stops where a joint reaches its soft limit.
 *
 * motmod (motion/control.c) aborts a teleop jog as soon as a joint's commanded position is
 * beyond its [JOINT_n] MIN/MAX_LIMIT. Here the decision is made once, when the jog starts,
 * on the jog's own path: the straight line from where the axes stand to where the jog
 * would end (the axis's limit, or the increment), walked through the kinematics. The jog is
 * given the end point where the first active joint would reach its limit heading out, so
 * the board (or the simulate model) stops there by itself: nothing depends on status
 * reports arriving, or on when they arrive (a PC that stalls for a second still stops at
 * the limit). A joint already beyond its limit may come back - a path along which it moves
 * in is not cut - but a path whose first step takes it further out is (fraction 0: refuse).
 *
 * Header-only, so backend-test (joglimit) exercises the same code kmotion-motion.cc runs.
 */
#ifndef KMOTION_JOGLIMIT_H
#define KMOTION_JOGLIMIT_H

#include <cmath>

struct JogLimit {
    double s = 1;        // the fraction of the path the jog may travel: 1 all of it, 0 none
    int joint = -1;      // the joint that cut it (-1: none)
    int dir = 0;         // +1 its max limit, -1 its min limit
    bool reach = true;   // false: the cut is where the kinematics' reach ends, not a joint limit
};

namespace joglimit_detail {
// how far joint j is beyond its limits (0: within)
inline double excess(const double jp[8], int j, const double lo[8], const double hi[8])
{
    if (jp[j] > hi[j]) return jp[j] - hi[j];
    if (jp[j] < lo[j]) return lo[j] - jp[j];
    return 0;
}
}

// joints_at(const double pos[9], double joints[8]) -> bool: the joints at a CAD position
// (false: out of the kinematics' reach). The path from `from` to `to` is walked in steps of
// at most `step` machine units; a crossing into a limit is then found exactly by bisection.
// A joint already beyond its limit that turns outward within a step is cut at the step
// before (conservative by at most one step). Only joints in `active` (bit = joint) count.
template <class F>
JogLimit jog_limit(const double from[9], const double to[9], F joints_at, const double lo[8], const double hi[8],
                   unsigned active, double step = 0.001)
{
    using joglimit_detail::excess;
    JogLimit r;
    double len = 0;
    for (int i = 0; i < 9; i++) len += (to[i] - from[i]) * (to[i] - from[i]);
    len = std::sqrt(len);
    if (len <= 0) return r;
    int n = (int) std::ceil(len / step);
    if (n < 1) n = 1;
    if (n > 100000) n = 100000;
    auto at = [&](double s, double jp[8]) {
        double p[9];
        for (int i = 0; i < 9; i++) p[i] = from[i] + s * (to[i] - from[i]);
        return joints_at(p, jp);
    };
    double prev[8];
    if (!at(0, prev)) { r.s = 0; r.reach = false; return r; }
    double s_prev = 0;
    for (int k = 1; k <= n; k++) {
        double s = (double) k / n, cur[8];
        if (!at(s, cur)) { r.s = s_prev; r.reach = false; return r; }
        double best = 2;
        for (int j = 0; j < 8; j++) {
            if (!(active & (1u << j))) continue;
            double e0 = excess(prev, j, lo, hi), e1 = excess(cur, j, lo, hi);
            if (!(e1 > e0)) continue;                     // within, or coming back in
            double cut;
            if (e0 == 0) {
                // it crosses into its limit within this step: find where
                double a = s_prev, b = s;
                for (int it = 0; it < 60; it++) {
                    double mid = 0.5 * (a + b), jp[8];
                    if (at(mid, jp) && excess(jp, j, lo, hi) == 0) a = mid; else b = mid;
                }
                cut = a;
            } else {
                cut = s_prev;                             // already beyond, now heading further out
            }
            if (cut < best) {
                best = cut;
                r.joint = j;
                r.dir = (e0 == 0 ? cur[j] > hi[j] : prev[j] > hi[j]) ? 1 : -1;
            }
        }
        if (best <= 1) { r.s = best; return r; }
        for (int j = 0; j < 8; j++) prev[j] = cur[j];
        s_prev = s;
    }
    return r;
}

#endif
