// The kmotion-kflop bench's forever loop. kmotion-motion starts it in thread 1 once the init
// programs have run ([KMOTION] START_PROGRAM = 1 BenchLoop.c). Everything that has to run
// continuously is serviced here, in one thread, because every thread running slows down
// thread 0; each part is #included and does its work only when enabled:
//   ServiceCSS()       Dynomotion's G96 constant surface speed (spindle/CSSJog.c), on while
//                      persist 110 is 2 (kmotion-motion sets it for G96, as KMotionCNC does)
//   ServiceProbe()     the probe watcher for G38.2-G38.5 (probe/ProbeService.c), on while
//                      kmotion-motion has it armed during a probe move
//   SimulateSwitches() the bare board's home and limit switches, on the KFLOP's virtual I/O
//                      bits (48-63), from the axes' commanded positions (Dynomotion's
//                      C Programs/SimulateHomes.c, with the bits this config uses):
//                        48 49 50  home switch of channel 0 1 2: made below -1000/-2000/-3000 counts
//                                  (Home.c reads them; all three home searches go negative)
//                        51 52 53  negative limit switch: made below -1500/-2500/-3500 counts
//                                  (500 past the home switch, so a home search stops before it)
//                        54 55 56  positive limit switch: made above +2000 counts
//                      ([KMOTION] LIMIT_NEG_BITS = 51 52 53, LIMIT_POS_BITS = 54 55 56)
#include "KMotionDef.h"
#include "spindle/MySpindleDefs.h"
#include "spindle/CSSJog.c"
#include "probe/ProbeService.c"

static void SimulateSwitches(void)
{
	SetStateBit(48, ch0->Dest < -1000.0);
	SetStateBit(49, ch1->Dest < -2000.0);
	SetStateBit(50, ch2->Dest < -3000.0);

	SetStateBit(51, ch0->Dest < -1500.0);
	SetStateBit(52, ch1->Dest < -2500.0);
	SetStateBit(53, ch2->Dest < -3500.0);

	SetStateBit(54, ch0->Dest > 2000.0);
	SetStateBit(55, ch1->Dest > 2000.0);
	SetStateBit(56, ch2->Dest > 2000.0);
}

int main()
{
	for (;;)
	{
		WaitNextTimeSlice();
		SimulateSwitches();
		ServiceCSS();
		ServiceProbe();
	}
}
