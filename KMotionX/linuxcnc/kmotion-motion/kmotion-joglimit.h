/* kmotion-joglimit.h: a world (teleop) jog stops when a joint reaches its soft limit.
 *
 * motmod (motion/control.c) aborts every teleop jog as soon as a joint's commanded position
 * is beyond its [JOINT_n] MIN/MAX_LIMIT. In kmotion-motion a world jog runs on the board (a
 * planner move toward the axis's limit) or in the simulate model, and the joints' positions
 * come from the status through the kinematics, so the decision is made on the position
 * reported, and nothing else: the joint that is beyond a limit and heading further out (its
 * position moved outward since the last report) stops the jog. A joint already past its
 * limit must be able to come back, so moving in, or standing, is no stop. Header-only: the
 * backend test (backend-test.cc joglimit) exercises the decision kmotion-motion.cc acts on.
 */
#ifndef KMOTION_JOGLIMIT_H
#define KMOTION_JOGLIMIT_H

struct JogLimitWatch {
    double last[8];
    bool have;

    JogLimitWatch() : have(false) { for (int j = 0; j < 8; j++) last[j] = 0; }

    // the jog starts here: the direction of travel is measured from this position
    void start(const double act[8])
    {
        for (int j = 0; j < 8; j++) last[j] = act[j];
        have = true;
    }

    // one report of the joints' positions. Returns the joint that is beyond its limit and
    // moving further out (-1: none); dir says which limit (+1 max, -1 min). Only joints in
    // `active` (bit = joint) count
    int step(const double act[8], const double min[8], const double max[8], unsigned active, int *dir)
    {
        int hit = -1;
        if (dir) *dir = 0;
        if (!have) { start(act); return -1; }
        for (int j = 0; j < 8; j++) {
            if (!(active & (1u << j))) continue;
            double moved = act[j] - last[j];
            if (hit < 0 && moved > 0 && act[j] > max[j]) { hit = j; if (dir) *dir = 1; }
            if (hit < 0 && moved < 0 && act[j] < min[j]) { hit = j; if (dir) *dir = -1; }
            last[j] = act[j];
        }
        return hit;
    }
};

#endif
