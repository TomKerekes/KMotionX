// The kmotion-kogna bench's forever loop. kmotion-motion starts it in thread 1 once the init
// programs have run ([KMOTION] START_PROGRAM = 1 BenchLoop.c). Everything that has to run
// continuously is serviced here, in one thread, because every thread running slows down
// thread 0; each part is #included and does its work only when enabled:
//   ServiceCSS()    Dynomotion's G96 constant surface speed (spindle/CSSJog.c), on while
//                   persist 110 is 2 (kmotion-motion sets it for G96, as KMotionCNC does)
//   ServiceProbe()  the probe watcher for G38.2-G38.5 (probe/ProbeService.c), on while
//                   kmotion-motion has it armed during a probe move
#include "KMotionDef.h"
#include "spindle/MySpindleDefs.h"
#include "spindle/CSSJog.c"
#include "probe/ProbeService.c"

int main()
{
	for (;;)
	{
		WaitNextTimeSlice();
		ServiceCSS();
		ServiceProbe();
	}
}
