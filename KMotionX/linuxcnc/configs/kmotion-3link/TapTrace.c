#include "KMotionDef.h"

// Diagnosis: the coordinated motion's time flow while a rigid tap runs. Every 10 ms the
// executed path time (CS0_TimeExecuted + CS0_t) should advance by about 10 ms x the sync
// factor; when it advances by less than a quarter of that the flow has stalled, and the
// state is printed once at the stall's start and once at its end (with its length).
// Also a heartbeat every second. Run it on a free thread before the tap (Execute6).
// Stop it with Kill6. Reads only; it never changes anything.

main()
{
	double last = CS0_TimeExecuted + CS0_t, t_last = Time_sec(), beat = t_last, stall_t0 = 0;
	int stalled = 0, n = 0;
	printf("TapTrace: watching the time flow (TIMEBASE %.6f)\n", TIMEBASE);
	for (;;)
	{
		Delay_sec(0.01);
		double now = Time_sec();
		double exec = CS0_TimeExecuted + CS0_t;
		double sf = Spindle.ThreadingActive ? Spindle.SyncFactor : 1.0;
		double expect = (now - t_last) * (CS0_TimeBase / TIMEBASE) * sf;
		int active = CoordSystem0 != 0 && Spindle.ThreadingActive;
		if (active && !stalled && exec - last < 0.25 * expect && expect > 0.001)
		{
			stalled = 1; stall_t0 = now;
			printf("STALL at %.3f s: exec %.4f, TimeBase %.3f (desired %.3f), SyncFactor %.3f, stop state %d, flushed %d, seg mode %d seg t %.4f CS0_t %.4f, spindle %.3f revs %.2f rps, downloaded %.2f\n",
				now, exec, CS0_TimeBase / TIMEBASE, CS0_TimeBaseDesired / TIMEBASE, Spindle.SyncFactor, CS0_StoppingState, CS0_Flushed,
				CoordSystem0->trajectory_mode, CoordSystem0->t, CS0_t, Spindle.Position, Spindle.TrueSpeedRPS, CS0_TimeDownloaded);
		}
		else if (stalled && (exec - last >= 0.25 * expect || !active))
		{
			stalled = 0;
			printf("  flows again at %.3f s after %.3f s: exec %.4f, TimeBase %.3f, SyncFactor %.3f, seg mode %d\n",
				now, now - stall_t0, exec, CS0_TimeBase / TIMEBASE, Spindle.SyncFactor, CoordSystem0 ? CoordSystem0->trajectory_mode : -1);
		}
		if (now - beat >= 1.0)
		{
			beat = now;
			printf("  %.1f s: exec %.3f, TimeBase %.3f, SyncFactor %.3f, threading %d, coord %d\n", now, exec, CS0_TimeBase / TIMEBASE, sf, Spindle.ThreadingActive, CoordSystem0 != 0);
		}
		last = exec; t_last = now;
	}
}
