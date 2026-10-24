// ProbeTrip.c - trips the bench's simulated probe (virtual bit 57, [KMOTION] PROBE_BIT) a
// moment after it is started, so a G38 move can be tested with nothing wired: MDI
//   M102          starts this on thread 7 and goes on at once ([KMOTION] MCODE_102 = PROGRAM 7 ...)
//   G38.2 Z-1 F2  the probe move, within the next 4 s; then the bit goes high for 4 s and
//                 the move stops as tripped (a G38 that starts while it is high is refused:
//                 "Probe is already tripped")
// The bit is cleared again afterwards, so the next probe starts open.
#include "KMotionDef.h"

int main()
{
	ClearBit(57);
	Delay_sec(4.0);
	SetBit(57);
	Delay_sec(4.0);
	ClearBit(57);
	return 0;
}
