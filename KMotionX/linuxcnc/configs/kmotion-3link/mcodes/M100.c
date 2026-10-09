// M100 on the 3 Link robot, run by [KMOTION] MCODE_100 = PROGRAM_WAIT 5 10 mcodes/M100.c:
// the demo program DynoMotion3Link.ngc starts with M100, which has nothing assigned yet.
// LinuxCNC's P and Q arrive as floats in persist variables 10 and 11 (-1 for a word not given).
#include "KMotionDef.h"

int main()
{
	float p = *(float *)&persist.UserData[10];
	float q = *(float *)&persist.UserData[11];

	printf("M100 P=%f Q=%f (nothing assigned)\n", p, q);
	return 0;
}
