// Simulates home and limit switches on virtual I/O bits, from the axes' commanded
// destinations, for a bare board (no switches wired). Run it in a thread of its own
// (thread 7 on the bench) while testing homing and the limit handling.
//
//   bit 46, 47, 48  home switch of channel 0, 1, 2: made (1) below -1000/-2000/-3000 counts
//   bit 49, 50, 51  negative limit switch of channel 0, 1, 2: made below -1500/-2500/-3500
//                   (500 counts past the home switch: a home search stops before it)
//   bit 52, 53, 54  positive limit switch of channel 0, 1, 2: made above +2000 counts
//
// Home.c reads the home bits; kmotion-motion reads the limit bits when its ini names them
// ([KMOTION] LIMIT_NEG_BITS = 49 50 51, LIMIT_POS_BITS = 52 53 54, LIMIT_ACTIVE = 1).
#include "KMotionDef.h"

int main()
{
	for (;;)
	{
		SetStateBit(46,ch0->Dest < -1000.0);
		SetStateBit(47,ch1->Dest < -2000.0);
		SetStateBit(48,ch2->Dest < -3000.0);

		SetStateBit(49,ch0->Dest < -1500.0);
		SetStateBit(50,ch1->Dest < -2500.0);
		SetStateBit(51,ch2->Dest < -3500.0);

		SetStateBit(52,ch0->Dest > 2000.0);
		SetStateBit(53,ch1->Dest > 2000.0);
		SetStateBit(54,ch2->Dest > 2000.0);
	}
}
