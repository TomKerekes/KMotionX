// SimulateStock.c - a probe and a block of stock for a bare board.
//
// Dynomotion's SimulateProbeAndStock.c as used with LinuxCNC (configs/kmotion-kogna: PROBE_BIT
// 1032, PROBE_ACTIVE 1): run in a thread of its own, it sets virtual bit 1032 while the probe
// tip (0.125 in) is inside a 4 x 3 x 1 in block whose top is 4.5 in up (Z 4..5), from the axes'
// Dest at 2540 counts/inch, so G38 moves trip where the stock is. Differences from the
// original: the axis resolutions are fixed here (the original asks KMotionCNC for them through
// DoPCInt, which LinuxCNC does not answer) and the bit means contact, 1 inside the stock,
// where the original's 1032 is the opposite.
#include "KMotionDef.h"
#define TMP 10					// which spare persist to use to transfer data
#include "KflopToKMotionCNCFunctions.c"

#define StockWidth 4.0
#define StockDepth 3.0
#define StockHeight 1.0

#define Length 4.0
#define PROBETIPDIAMETER 0.125f 		// #121		Diameter of probe tip

int main()
{
	double x,y,z,xRes,yRes,zRes;
	printf("Getting Axis Resolution\n");
//	DoPCInt(PC_COMM_GETAXISRES, TMP);

//	xRes = *(float *)&persist.UserData[TMP];
//	yRes = *(float *)&persist.UserData[TMP+1];
//	zRes = *(float *)&persist.UserData[TMP+2];

	xRes = yRes = zRes = 2540;

	printf("Axis Resolutions Found:\n\tx: %f\n\ty: %f\n\tz: %f\n", xRes, yRes, zRes);


	for (;;)
	{
		x = ch0->Dest / xRes;
		y = ch1->Dest / yRes;
		z = ch2->Dest / zRes;

//		printf("XYZ %f %f %f %d %d\n",x,y,z,ReadBit(1032),fast_fabs(z-Length-StockHeight/2) < StockHeight/2);
//		Delay_sec(1);
		SetStateBit(1032,  (fast_fabs(x) < (StockWidth+PROBETIPDIAMETER)/2 && 
                            fast_fabs(y) < (StockDepth+PROBETIPDIAMETER)/2 && 
                            fast_fabs(z-Length-StockHeight/2) < StockHeight/2));
	}
}
