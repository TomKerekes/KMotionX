/*
 * kins3link-offsets - the 3 Link robot's servo zero offsets from one measured pose.
 *
 * Kinematics3Link.cpp's OFFSET_R/L/Z_DEG align the servos' count zeros with the geometry's
 * angle conventions (Right/Left: 0 = arm along +X, CCW from above; Z: 0 = arm horizontal
 * toward the paper, 90 = up). Setting those zero poses by eye leaves errors of several
 * degrees, and the Z angle at the paper changes by 13-15 degrees per inch of Y, so a Y error
 * from the Left/Right zeros looks like a Z zero error. This solves all three at once from one
 * pose: put the pen somewhere you can measure, read the three joint angles LinuxCNC shows (the
 * servo counts / 11.378, offsets as compiled into the library now), measure the pen's X Y Z in
 * CAD coordinates (origin: the center of travel, 300 mm toward the paper from the midpoint
 * between the Left and Right shafts, at the paper surface; +X toward the Right servo, +Y toward
 * the paper, +Z up; inches), and run
 *
 *     ./kins3link-offsets <x> <y> <z> <right_deg> <left_deg> <z_deg>
 *
 * It prints the OFFSET_R/L/Z_DEG that make the kinematics agree with the measurement (to be
 * added to what is compiled in now), and the pen position the kinematics would report at the
 * measured pose with them.
 */
#include <cstdio>
#include <cstdlib>
#include "GCodeInterpreterX.h"
#include "Kinematics3Link.h"

int main(int argc, char *argv[])
{
    if (argc != 7) {
        fprintf(stderr, "usage: kins3link-offsets <x> <y> <z> <right_deg> <left_deg> <z_deg>  (inches, degrees)\n");
        return 1;
    }
    double x = atof(argv[1]), y = atof(argv[2]), z = atof(argv[3]);
    double read[3] = {atof(argv[4]), atof(argv[5]), atof(argv[6])};
    CKinematics3Link k;
    const double cpd = 1.0;             // 1 count per degree: the actuators come out in degrees
    k.m_MotionParams.CountsPerInchX = k.m_MotionParams.CountsPerInchY = k.m_MotionParams.CountsPerInchZ = cpd;
    double A[8];
    if (k.TransformCADtoActuators(x, y, z, 0, 0, 0, A)) {
        fprintf(stderr, "the pose x %g y %g z %g is out of the kinematics' reach\n", x, y, z);
        return 1;
    }
    printf("the geometry has the servos at   R %8.2f  L %8.2f  Z %8.2f deg for the pen at x %g y %g z %g\n", A[0], A[1], A[2], x, y, z);
    printf("the servos read                  R %8.2f  L %8.2f  Z %8.2f deg\n", read[0], read[1], read[2]);
    printf("add to OFFSET_R/L/Z_DEG:         R %+8.2f  L %+8.2f  Z %+8.2f deg\n", read[0] - A[0], read[1] - A[1], read[2] - A[2]);
    // what the kinematics make of the reading as they are now
    double B[8] = {read[0], read[1], read[2], 0, 0, 0, 0, 0}, xr, yr, zr, a, b, c;
    k.TransformActuatorstoCAD(B, &xr, &yr, &zr, &a, &b, &c);
    printf("as compiled now the reading means x %.3f y %.3f z %.3f (measured x %g y %g z %g)\n", xr, yr, zr, x, y, z);
    return 0;
}
