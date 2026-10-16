#include "KMotionDef.h"

// MPG pendant service for KMotionCNC: one handwheel, three selector switches
//
//   MPG FUNCTION switch
//     F&S%     handwheel changes Feed Rate Override and Spindle Speed Override together
//     S%       handwheel changes Spindle Speed Override
//     F%       handwheel changes Feed Rate Override
//     RAPID%   handwheel changes Rapid Rate Override
//     JOG      handwheel jogs the axis selected with the AXIS switch (X Z C Q)
//     PRG.STP  "program step": the handwheel moves a point in the running program's
//              TIME forward or backward and the feed rate override is servoed so the
//              program follows that point (details below)
//
//   STEP switch  .0001 / .001 / .01 / HW  = amount per handwheel detent
//     JOG      inches (degrees for C) per detent            JogStep[]
//     PRG.STP  seconds of program time per detent           TimeStep[]
//     F% S% RAPID%  percent per detent                      PctStep[]
//   Every encoder count is applied as 1/MPG_COUNTS_PER_DETENT of the amount, so
//   the motion is smooth within a detent rather than stepping once per click.
//
// PROGRAM STEP - how it works
//
// Coordinated motion is a buffer of time segments that the board executes at a
// rate set by the Feed Rate Override: at FRO = 1 a second of program time takes a
// real second, at FRO = 0 the program stands still, and at a negative FRO it runs
// BACKWARDS along the buffered path, as far back as the buffer still holds.  The
// program's current time is CS0_TimeExecuted + CS0_t.
//
// This mode keeps a target time.  Each handwheel count moves the target forward or
// back.  Every time slice the FRO is set in proportion to the distance from the
// target, limited to +-TS_FRO_MAX and to what the machine's own deceleration can
// still stop within, so the program approaches the target without overshoot and
// stops exactly on it.  The FRO is driven with the "Temp" calls, which steer the
// board's rate of time without disturbing the FRO that KMotionCNC set: leaving
// the mode simply puts that FRO back.  A real feed hold is respected.  Cycle Start
// with this mode selected is caught in the first time slice and the program is
// brought to rest a few milliseconds of program time in, then backed up to its
// start: nothing moves until the wheel is turned.  Turning it back backs up along
// the path (SetBit style M-code outputs are skipped going backwards, not undone).
// Selecting this mode while a program is running brings it to a stop; leaving it
// lets the program continue at its normal override.
//
// Run this program in its own thread (it loops forever) or call ServiceMPGPanel()
// from an Init program's main loop every time slice.

#define TMP 10                   // persist var used by the KMotionCNC command helpers
#include "KflopToKMotionCNCFunctions.c"

// --- hardware --------------------------------------------------------------

#define MPG_INPUT_AXIS        5  // axis channel whose Position reads the handwheel encoder
#define MPG_COUNTS_PER_DETENT 4  // quadrature counts per handwheel click

// digital input bits, one per switch position, active high (change to your wiring)
int StepBits[4]  = {16, 17, 18, 19};               // .0001  .001  .01  HW
int AxisBits[4]  = {20, 21, 22, 23};               // X  Z  C  Q
int FuncBits[6]  = {24, 25, 26, 27, 28, 29};       // F&S%  S%  F%  RAPID%  JOG  PRG.STP

#define FUNC_FS    0
#define FUNC_S     1
#define FUNC_F     2
#define FUNC_RAPID 3
#define FUNC_JOG   4
#define FUNC_STEP  5

// axis channel for each AXIS switch position and its counts per inch (per degree for C)
int    AxisChan[4]      = {0, 1, 2, 3};            // X  Z  C  Q
double CountsPerUnit[4] = {10000.0, 10000.0, 100.0, 10000.0};

// --- amounts per detent for each STEP switch position ----------------------

double JogStep[4]  = {0.0001, 0.001, 0.01, 0.1};   // JOG: inches (degrees) per detent
double TimeStep[4] = {0.005, 0.02, 0.1, 0.5};      // PRG.STP: seconds of program time per detent
double PctStep[4]  = {1.0, 2.0, 5.0, 10.0};        // overrides: percent per detent

// --- override limits ---------------------------------------------------------

#define OVERRIDE_SEND_PERIOD 0.05  // seconds between commands to KMotionCNC
#define RRO_MIN 0.05
#define RRO_MAX 1.0                // KMotionCNC's own Max Rapid FRO setting also applies
#define PC_TIMEOUT 1.0             // give up on a KMotionCNC command after this long

// --- program step (time servo) ------------------------------------------------

#define TS_GAIN        10.0        // FRO per second of time error (approach time constant 0.1 s)
#define TS_FRO_MAX     1.0         // never faster than programmed, forward or backward
#define TS_DEADBAND    0.0002      // seconds: this close to the target commands FRO 0
#define TS_LAUNCH_RAMP 0.005       // seconds to bring FRO 1 -> 0 when a program starts in this mode

// --- smooth jog (from MPGServiceSmoothHardwareEncFilteredRev4.c) ---------------

#define TAU        0.01            // MoveExp time constant, seconds
#define TAU1       0.015           // pre filter 1
#define TAU2       0.015           // pre filter 2
#define FINAL_TIME 1.0             // finalize the move after this long with no change


// Send a command with a float parameter to KMotionCNC, but do not hang the
// service loop if KMotionCNC is not running.  Returns 0 on success.
int DoPCFloatTimeout(int cmd, float f)
{
	double T0 = Time_sec();
	int result;

	persist.UserData[PC_COMM_PERSIST + 1] = *(int *)&f;
	persist.UserData[PC_COMM_PERSIST] = cmd;
	do
	{
		WaitNextTimeSlice();
		result = persist.UserData[PC_COMM_PERSIST];
		if (Time_sec() > T0 + PC_TIMEOUT)
		{
			persist.UserData[PC_COMM_PERSIST] = 0;
			return -1;
		}
	} while (result > 0);
	return result;
}

// index of the switch position whose input is high, or the previous one if none
int ReadSwitch(int *bits, int n, int *last)
{
	int i;
	for (i = 0; i < n; i++)
		if (ReadBit(bits[i])) { *last = i; break; }
	return *last;
}

// program time, read consistently: the interrupt moves time from CS0_t into
// CS0_TimeExecuted at every segment boundary, so re-read if that happened
double ReadProgramTime(void)
{
	double e, t;
	int i;
	for (i = 0; i < 4; i++)
	{
		e = CS0_TimeExecuted;
		t = CS0_t;
		if (e == CS0_TimeExecuted) break;
	}
	return e + t;
}


// F&S%  S%  F%  RAPID%: accumulate counts and send at most every OVERRIDE_SEND_PERIOD
void ServiceOverrides(int func, int counts, int step)
{
	static int    Accum = 0, LastFunc = -1;
	static double LastSendTime = 0, Rapid = 1.0;
	float factor;
	double pct;

	if (func != LastFunc) Accum = 0;             // do not carry counts over from another function
	LastFunc = func;
	Accum += counts;
	if (Accum == 0 || Time_sec() < LastSendTime + OVERRIDE_SEND_PERIOD) return;

	pct = Accum * PctStep[step] / MPG_COUNTS_PER_DETENT;
	Accum = 0;
	LastSendTime = Time_sec();

	factor = 1.0f + (float)(pct / 100.0);      // multiplies KMotionCNC's current value
	if (factor < 0.05f) factor = 0.05f;

	switch (func)
	{
	case FUNC_F:
		DoPCFloatTimeout(PC_COMM_SET_FRO_INC, factor);
		break;
	case FUNC_S:
		DoPCFloatTimeout(PC_COMM_SET_SSO_INC, factor);
		break;
	case FUNC_FS:
		DoPCFloatTimeout(PC_COMM_SET_FRO_INC, factor);
		DoPCFloatTimeout(PC_COMM_SET_SSO_INC, factor);
		break;
	case FUNC_RAPID:                            // no increment command, so track the value
		Rapid += pct / 100.0;
		if (Rapid < RRO_MIN) Rapid = RRO_MIN;
		if (Rapid > RRO_MAX) Rapid = RRO_MAX;
		DoPCFloatTimeout(PC_COMM_SET_RRO, (float)Rapid);
		break;
	}
}


// JOG: smooth exponential motion of the selected axis, one STEP amount per detent
void ServiceJog(int active, int counts, int step, int axissel)
{
	static int    InMotion = FALSE, LastChan = -1, FirstTime = TRUE;
	static double LastChangeTime = 0, Target, Target1, Target2, K1, K2, K1M, K2M;
	int chn = AxisChan[axissel];

	if (FirstTime)
	{
		K1 = exp(-2 * TIMEBASE / TAU1);           // filter coefficients
		K2 = exp(-2 * TIMEBASE / TAU2);
		K1M = 1.0 - K1;
		K2M = 1.0 - K2;
		FirstTime = FALSE;
	}

	if (!active || JOB_ACTIVE) counts = 0;       // other function selected or a job is running

	if (CS0_StoppingState == 4 && InMotion)      // feedhold fully stopped?
	{
		counts = 0;
		Jog(LastChan, 0);
		InMotion = FALSE;
	}

	// axis or function changed, or converged for a while: finalize the motion
	if (chn != LastChan || !active || (InMotion && Time_sec() > LastChangeTime + FINAL_TIME))
	{
		if (InMotion) Move(LastChan, Target1);
		LastChan = chn;
		InMotion = FALSE;
	}

	if (counts)
	{
		if (!InMotion) Target = Target1 = Target2 = chan[chn].Dest;
		Target1 += counts * JogStep[step] * CountsPerUnit[axissel] / MPG_COUNTS_PER_DETENT;
		LastChangeTime = Time_sec();
		InMotion = TRUE;
	}

	if (InMotion)
	{
		if (Target1 > chan[chn].SoftLimitPos) Target1 = chan[chn].SoftLimitPos;
		if (Target1 < chan[chn].SoftLimitNeg) Target1 = chan[chn].SoftLimitNeg;
		Target2 = Target2 * K1 + Target1 * K1M;
		Target  = Target  * K2 + Target2 * K2M;
		MoveExp(chn, Target, TAU);
	}
}


// PRG.STP: servo the rate of program time so it follows the handwheel
void ServiceProgramStep(int active, int counts, int step)
{
	static int    WasActive = FALSE, WasExecuting = FALSE, LaunchCatch = FALSE;
	static double Target = 0, LastNow = 0, RampTime = 0.5, RampCheckTime = 0, LastFRO = 0;
	double now, err, aerr, lo, hi, cap, fro, T;
	int executing;

	if (!active)
	{
		if (WasActive)                           // leaving the mode: put KMotionCNC's FRO back
		{
			if (CS0_StoppingState == 0)
				SetFROTemp(CS0_DoingRapid ? CS0_LastRapidFRO : CS0_LastFRO);
			WasActive = FALSE;
		}
		return;
	}

	T = Time_sec();
	now = ReadProgramTime();
	executing = (CoordSystem0 != NULL);

	// nominal time to ramp FRO from 1 to 0 at the axes' max acceleration; sets both
	// the ramp the board uses and the stopping-distance limit below
	if (!WasActive || T > RampCheckTime)
	{
		RampTime = GetNominalFROChangeTime();
		if (RampTime < 0.01) RampTime = 0.01;
		RampCheckTime = T + 1.0;
	}

	// a program just started: the board set the time base to KMotionCNC's FRO at
	// launch, so stop it quickly (the axes have barely begun to accelerate) and
	// back up to where it started from
	if (WasActive && ((executing && !WasExecuting) || (now < LastNow - 0.02 && LastFRO >= 0.0)))
	{
		LaunchCatch = TRUE;
		Target = now;
	}

	// start from where the program is when entering the mode or nothing executes
	if (!WasActive || !executing)
		Target = now;

	WasActive = TRUE;
	WasExecuting = executing;
	LastNow = now;

	if (CS0_StoppingState != 0)                  // real feed hold: let it hold, ignore the wheel
	{
		Target = now;
		LaunchCatch = FALSE;
		return;
	}

	if (LaunchCatch)
	{
		SetFROwRateTemp(0.0f, TS_LAUNCH_RAMP);
		LastFRO = 0.0;
		if (CS0_TimeBase <= 1e-9) LaunchCatch = FALSE;   // at rest, the servo takes over
		return;
	}

	Target += counts * TimeStep[step] / MPG_COUNTS_PER_DETENT;

	// stay within what the buffer holds: not before segments already overwritten,
	// not past what KMotionCNC has downloaded so far
	lo = ParametricIndexWrapped ? CS0_TimeLost + 0.01 : 0.0;
	hi = CS0_TimeDownloaded;
	if (Target < lo) Target = lo;
	if (Target > hi) Target = hi;

	err = Target - now;
	aerr = err < 0.0 ? -err : err;

	if (aerr < TS_DEADBAND)
	{
		fro = 0.0;
	}
	else
	{
		fro = TS_GAIN * err;                                  // proportional approach
		cap = 0.9 * sqrt(2.0 * aerr / RampTime);              // can still stop at the target
		if (fro >  cap) fro =  cap;
		if (fro < -cap) fro = -cap;
		if (fro >  TS_FRO_MAX) fro =  TS_FRO_MAX;
		if (fro < -TS_FRO_MAX) fro = -TS_FRO_MAX;
	}

	// Temp: steers the rate of time (feed or rapid) without changing the FRO
	// KMotionCNC set.  The board ramps to it at the nominal rate = accel limit.
	SetFROwRateTemp((float)fro, (float)RampTime);
	LastFRO = fro;
}


void ServiceMPGPanel(void)
{
	static int Pos, FirstTime = TRUE;
	static int Step = 0, Axis = 0, Func = -1;     // -1: no function input seen yet, do nothing
	int NewPos, counts;

	if (FirstTime)
	{
		Pos = chan[MPG_INPUT_AXIS].Position;
		FirstTime = FALSE;
	}

	NewPos = chan[MPG_INPUT_AXIS].Position;
	counts = NewPos - Pos;
	Pos = NewPos;

	ReadSwitch(StepBits, 4, &Step);
	ReadSwitch(AxisBits, 4, &Axis);
	ReadSwitch(FuncBits, 6, &Func);

	ServiceJog(Func == FUNC_JOG, counts, Step, Axis);
	ServiceProgramStep(Func == FUNC_STEP, counts, Step);
	if (Func >= 0 && Func <= FUNC_RAPID) ServiceOverrides(Func, counts, Step);
}


main()
{
	for (;;)
	{
		WaitNextTimeSlice();
		ServiceMPGPanel();
	}
}
