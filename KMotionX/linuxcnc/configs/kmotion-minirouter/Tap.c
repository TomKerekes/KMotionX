#include "KMotionDef.h"

// Rigid tapping (LinuxCNC's G33.1) on the board, with kmotion-motion: the PC plans the tap
// move through the bottom (plus an overshoot allowance for the spindle's deceleration) and
// launches it in the board's spindle-sync mode (TrigThread), so the coordinated motion's
// time base follows the spindle encoder - every actuator, through any kinematics, and a
// reversal runs the path backwards. This program handles the spindle: it waits for the
// launch, lets the spindle turn the revolutions for the depth, reverses it at the retract
// speed, and stops it once the encoder is back at the start. kmotion-motion then clears the
// motion, sets the planner to where the board stands, and tells the spindle to run again
// as LinuxCNC has it (M3 S). The spindle turns at speed when this starts (M3 S came first).
//
// Parameters from kmotion-motion through persist (all x 1e6 as integers):
//   persist.UserData[192]  state: 0 running (set by the PC), 1 done (set here), 2 failed (the
//                          sync ended before the spindle was back: the path ran out)
//   persist.UserData[193]  revolutions for the depth
//   persist.UserData[194]  spindle speed in rev/s, signed (negative: M4, counterclockwise)
//   persist.UserData[195]  retract speed as a multiple of the spindle speed (G33.1's I)
//   persist.UserData[196]  the overshoot allowance the path has past the bottom, revolutions
//   persist.UserData[197]  the spindle's axis channel ([KMOTION] SPINDLE_SPEED_AXIS): its Position
//                          is the encoder, and here Jog() drives it
//   persist.UserData[198]  the encoder's counts per revolution (SPINDLE_COUNTS_PER_REV)
//
// What this program knows about the machine - EDIT FOR YOURS: how the spindle is driven
// (spindle_run below) and the Spindle Using Jogs state variable it shares with the M3/M4/M5
// programs (MySpindleDefs.h). The spindle's channel and its encoder's counts per revolution
// come from the ini (persist 197, 198). Bench (kmotion-minirouter): the spindle is axis 3, a
// step/dir axis jogged by the Spindle Using Jogs programs, whose quadrature is looped into
// encoder 4 = axis 3's input (spindle/SpindleAxis.c), so axis 3's Position is the encoder
// and Jog(3, ...) drives it.
#include "spindle/MySpindleDefs.h"
#define UPDATE_TIME 0.01        // the firmware's spindle speed measurement: update interval, s
#define TAU 0.02                // ... and its filter time constant, s

static int spindle_axis = 3;             // from persist 197 at the start
static double counts_per_rev = 1000.0;   // from persist 198

#define P_STATE 192
#define P_REVS 193
#define P_RPS 194
#define P_RETRACT 195
#define P_OVERSHOOT 196
#define P_AXIS 197
#define P_CPR 198

static void spindle_run(double rps)       // EDIT FOR YOURS: the spindle at rps (signed), 0 = stop
{
	Jog(spindle_axis, rps * counts_per_rev);
}

int main()
{
	double revs = persist.UserData[P_REVS] * 1e-6;
	double rps = persist.UserData[P_RPS] * 1e-6;
	double retract = persist.UserData[P_RETRACT] * 1e-6;
	double overshoot = persist.UserData[P_OVERSHOOT] * 1e-6;
	spindle_axis = persist.UserData[P_AXIS];
	counts_per_rev = persist.UserData[P_CPR];
	double dir = rps >= 0 ? 1.0 : -1.0;
	double speed = rps >= 0 ? rps : -rps;      // the firmware's library has no fabs
	double start, turned;

	printf("Tap: %.3f revs at %.3f rev/s, retract x%.2f, path %.2f revs past the bottom\n", revs, rps, retract, overshoot);
	persist.UserData[P_STATE] = 0;

	// the firmware's spindle measurement, which the sync mode runs on
	ConfigureSpindle(1, spindle_axis, UPDATE_TIME, TAU, counts_per_rev);

	// the launch (TrigThread from the PC): the firmware begins the motion not at the trigger
	// but when the spindle reaches Spindle.StartPosition, two whole revolutions on (so every
	// pass starts at the same spindle angle, as lathe threading needs); the time base is
	// held at zero until then. Count the revolutions from there, in the firmware's own
	// units (Spindle.Position, revolutions)
	while (!Spindle.ThreadingActive) WaitNextTimeSlice();
	start = Spindle.StartPosition;

	// down: the spindle turns the revolutions for the depth. The sync ending before that
	// means the path ran out (the overshoot allowance too small): the motion is stranded
	for (;;)
	{
		turned = dir * (Spindle.Position - start);
		if (turned >= revs) break;
		if (!Spindle.ThreadingActive) { printf("Tap: the sync ended on the way down after %.3f revs\n", turned); spindle_run(0); persist.UserData[STATEVAR] = 0; persist.UserData[P_STATE] = 2; return 0; }
		WaitNextTimeSlice();
	}
	printf("Tap: at the bottom after %.3f revs, reversing\n", turned);

	// back: the spindle reversed at the retract speed; the motion follows the spindle up,
	// the time base running backwards. The spindle's deceleration past the bottom is the
	// overshoot the path allows for: past the path's end the sync ends and the tool stays
	spindle_run(-dir * speed * retract);
	for (;;)
	{
		turned = dir * (Spindle.Position - start);
		if (turned <= 0.0) break;
		if (!Spindle.ThreadingActive) { printf("Tap: the sync ended on the way back at %.3f revs: the path ran out past the bottom\n", turned); spindle_run(0); persist.UserData[STATEVAR] = 0; persist.UserData[P_STATE] = 2; return 0; }
		WaitNextTimeSlice();
	}

	// at the top: the spindle stopped (its deceleration past the start finds the motion at
	// its beginning, where it stays). The PC restarts the spindle as LinuxCNC has it
	spindle_run(0);
	while (!CheckDone(spindle_axis)) WaitNextTimeSlice();
	persist.UserData[STATEVAR] = 0;   // off, as the Spindle Using Jogs programs keep it: the PC's M3 then starts it again
	printf("Tap: back at the start, the spindle stopped\n");
	persist.UserData[P_STATE] = 1;
	return 0;
}
