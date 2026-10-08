// LinuxCNC's probe moves G38.2-G38.5 on the board ([KMOTION] PROBE_BIT), after the probing in
// Dynomotion's NotifyProbeMach3.c (the Mach3 plugin's) and ProbeMain.c (the probe screen).
//
// #include this into the forever loop that services the board (thread 1, see ../BenchLoop.c)
// and call ServiceProbe() every time slice. It does nothing until kmotion-motion arms it, just
// before it hands a probe move to the planner, which runs the move like any move, so any
// kinematics apply (3Link included). While armed it watches the probe bit and, the moment the
// probe makes contact (G38.2/.3) or loses it (G38.4/.5), records where every coordinate-system
// axis is, stops the coordinated motion (as a feed hold does) and disarms. kmotion-motion then
// abandons the rest of the move and turns the recorded actuator positions into LinuxCNC's
// probed position through the kinematics.
//
// Persist variables (kmotion-motion sets 69 and 71-73 and 70 = -1, reads 70 and 74-89):
//   69  1 = armed (kmotion-motion arms; it and this code disarm)
//   70  status: 0 watching, 1 the probe already was in the state sought (disarmed, the move
//       doesn't run), 2 tripped: the positions are in 74-89 and the motion has been told to stop
//   71  the probe's input bit
//   72  the bit's level that means contact
//   73  1 = stop when contact is lost (G38.4/.5), 0 = when it is made (G38.2/.3)
//   74-89  at the trip: Dest of the x y z a b c u v axes' channels, 8 doubles (counts)

#define PROBE_ARM 69
#define PROBE_STATUS 70
#define PROBE_BIT 71
#define PROBE_LEVEL 72
#define PROBE_AWAY 73
#define PROBE_POS 74

int probe_watching = 0;     // armed and the first look done

void ServiceProbe(void)
{
	int stop_on, i, axis[8];
	double *pos;

	if (persist.UserData[PROBE_ARM] != 1)
	{
		probe_watching = 0;
		return;
	}
	stop_on = persist.UserData[PROBE_AWAY] ? !persist.UserData[PROBE_LEVEL] : persist.UserData[PROBE_LEVEL];

	if (!probe_watching)
	{
		// just armed: the probe must not already be where the move is to end
		if (ReadBit(persist.UserData[PROBE_BIT]) == stop_on)
		{
			persist.UserData[PROBE_ARM] = 0;
			persist.UserData[PROBE_STATUS] = 1;
			return;
		}
		probe_watching = 1;
		persist.UserData[PROBE_STATUS] = 0;
		return;
	}

	if (ReadBit(persist.UserData[PROBE_BIT]) != stop_on)
		return;

	// tripped: where every axis is, then stop
	axis[0] = CS0_axis_x;  axis[1] = CS0_axis_y;  axis[2] = CS0_axis_z;  axis[3] = CS0_axis_a;
	axis[4] = CS0_axis_b;  axis[5] = CS0_axis_c;  axis[6] = CS0_axis_u;  axis[7] = CS0_axis_v;
	pos = (double *)&persist.UserData[PROBE_POS];
	for (i = 0; i < 8; i++)
	{
		// (if/else: TCC67 before 2026-10-07 got a ?: whose result is a double wrong)
		if (axis[i] >= 0)
			pos[i] = chan[axis[i]].Dest;
		else
			pos[i] = 0.0;
	}
	StopCoordinatedMotion();
	persist.UserData[PROBE_ARM] = 0;
	probe_watching = 0;
	persist.UserData[PROBE_STATUS] = 2;
}
