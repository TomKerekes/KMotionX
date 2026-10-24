// M100 on the kmotion-minirouter bench, run by [KMOTION] MCODE_100 = PROGRAM_WAIT 5 10 mcodes/M100.c:
// LinuxCNC's P and Q arrive as floats in persist variables 10 and 11 (-1 for a word not given).
// Sets the KFLOP's virtual bit 63 when P is not 0, clears it otherwise, and prints both.
#include "KMotionDef.h"

int main()
{
	float p = *(float *)&persist.UserData[10];
	float q = *(float *)&persist.UserData[11];

	SetStateBit(63, p != 0.0f);
	printf("M100 P=%f Q=%f\n", p, q);
	return 0;
}
