// Home.c - homing for the Minirouter, run on the KFLOP/Kogna by kmotion-motion when
// LinuxCNC homes a joint ([KMOTION] HOME_PROGRAM). After Dynomotion's
// SimpleHomeIndexFunction.c: jog to the switch, back off and latch slowly, optionally go on
// to the encoder index, set the position, move to the home position.
//
// What to home and the numbers come from LinuxCNC's ini through the persist array (all in
// counts and counts per second; kmotion-motion writes them before starting this thread):
//   persist.UserData[120]  bit ch set: home board channel ch (the host sets this)
//   persist.UserData[121]  bit ch set: channel ch homed            (this program sets these)
//   persist.UserData[122]  bit ch set: homing of channel ch failed
//   persist.UserData[123]  channel being homed, -1 when none
//   persist.UserData[128 + 8*ch + k]:
//     k=0 search velocity, counts/s, its sign is the direction toward the switch
//                                                           ([JOINT_n] HOME_SEARCH_VEL)
//     k=1 latch velocity, counts/s; same sign as the search: back off and approach the
//         switch again slowly, its leading edge is home; opposite sign: back off until the
//         switch opens, that edge is home                    (HOME_LATCH_VEL)
//     k=2 position the switch edge (or the index) gets, counts (HOME_OFFSET)
//     k=3 position to move to afterwards, counts            (HOME)
//     k=4 velocity for that move, counts/s, 0 = the axis Vel (HOME_FINAL_VEL)
//     k=5 flags: 1 go on to the index after the switch (HOME_USE_INDEX), 4 no final move
//     k=6 how far the search may go before giving up, counts, 0 = unlimited
//         (the joint's MAX_LIMIT - MIN_LIMIT)
//     k=7 homing order, lower first                          (HOME_SEQUENCE)
//
// What this program knows about the machine - EDIT FOR YOURS: the input bit of each
// channel's home switch, the value ReadBit gives when the switch is made (polarity), and
// the index bit (-1 = none). Set here for the Thread7 home-switch simulator on this Kogna:
// channels 0,1,2 read bits 46,47,48, each driven high (polarity 1) when that axis's
// Destination falls below -1000/-2000/-3000 counts - so all three must search negative.
#include "KMotionDef.h"

#define N_HOME_CHANNELS 3
static const int SwitchBit[N_HOME_CHANNELS]      = {46, 47, 48};
static const int SwitchPolarity[N_HOME_CHANNELS] = {1, 1, 1};
static const int IndexBit[N_HOME_CHANNELS]       = {-1, -1, -1};
static const int IndexPolarity[N_HOME_CHANNELS]  = {1, 1, 1};

#define P_MASK    120
#define P_OK      121
#define P_FAILED  122
#define P_CURRENT 123
#define P_JOINT(ch) (&persist.UserData[128 + 8 * (ch)])
#define FLAG_USE_INDEX     1
#define FLAG_NO_FINAL_MOVE 4

static double Abs(double x) { return x < 0 ? -x : x; }

// stop the channel and wait until it stands still; 0 ok, 1 if it got disabled meanwhile
static int StopAndWait(int ch)
{
	Jog(ch, 0);
	while (!CheckDone(ch))
		if (!chan[ch].Enable) return 1;
	return 0;
}

// jog at vel until ReadBit(bit) == value; 0 ok, 1 disabled, 2 travel exceeded
static int JogUntilBit(int ch, double vel, int bit, int value, double start, double travel)
{
	Jog(ch, vel);
	while (ReadBit(bit) != value)
	{
		if (!chan[ch].Enable) return 1;
		if (travel > 0 && Abs(chan[ch].Dest - start) > travel) { StopAndWait(ch); return 2; }
	}
	return 0;
}

// returns 0 when homed, otherwise a failure code
int HomeChannel(int ch)
{
	int *p = P_JOINT(ch);
	double search = p[0], latch = p[1], offset = p[2], home = p[3], finalvel = p[4];
	int flags = p[5];
	double travel = p[6];
	int sw = SwitchBit[ch], pol = SwitchPolarity[ch], idx = IndexBit[ch], idxpol = IndexPolarity[ch];
	int dir = search < 0 ? -1 : 1;
	int latchdir = latch < 0 ? -1 : 1;
	double lv = Abs(latch);
	double start, trig, pos;
	int r, SaveLimits;

	if (search == 0.0) return 10;              // nothing to search for
	if (lv == 0.0) lv = Abs(search) / 4;

	// the limit switch may be the home switch: disable limit handling while homing, as the
	// example does, and restore it afterwards
	SaveLimits = chan[ch].LimitSwitchOptions;
	chan[ch].LimitSwitchOptions = 0;

	// 1. toward the switch at the search velocity until it is made
	start = chan[ch].Dest;
	r = JogUntilBit(ch, search, sw, pol, start, travel);
	if (r) { chan[ch].LimitSwitchOptions = SaveLimits; return r; }
	if (StopAndWait(ch)) return 1;

	// 2. the latch, slowly. Both moves are held to the search travel too: a switch that stays
	//    made, or never makes again, must not send the axis off for ever
	if (latchdir == dir)
	{
		// back off until the switch opens, then approach again: the leading edge is home
		r = JogUntilBit(ch, -lv * dir, sw, !pol, chan[ch].Dest, travel);
		if (r) { chan[ch].LimitSwitchOptions = SaveLimits; return r == 2 ? 3 : r; }
		if (StopAndWait(ch)) return 1;
		r = JogUntilBit(ch, lv * dir, sw, pol, chan[ch].Dest, travel);
		if (r) { chan[ch].LimitSwitchOptions = SaveLimits; return r == 2 ? 3 : r; }
	}
	else
	{
		// back off until the switch opens: that edge is home
		r = JogUntilBit(ch, -lv * dir, sw, !pol, chan[ch].Dest, travel);
		if (r) { chan[ch].LimitSwitchOptions = SaveLimits; return r == 2 ? 3 : r; }
	}
	trig = chan[ch].Dest;                      // where the edge was seen

	// 3. on to the index, if asked and there is one
	if ((flags & FLAG_USE_INDEX) && idx >= 0)
	{
		while (ReadBit(idx) != idxpol)             // still jogging from step 2
			if (!chan[ch].Enable) return 1;
		trig = chan[ch].Dest;
	}
	if (StopAndWait(ch)) return 1;

	// 4. the latched edge gets HOME_OFFSET; we came to rest a little past it, so our current
	//    position is that offset plus how far past the edge we stopped (read before Zero(),
	//    which clears Dest)
	DisableAxis(ch);
	pos = offset + (chan[ch].Dest - trig);
	if (chan[ch].OutputMode == STEP_DIR_MODE || chan[ch].OutputMode == MICROSTEP_MODE)
	{
		// open loop: Position is not feedback - meaningless with no input, or an encoder used
		// for something else such as an MPG - so leave it alone and only redefine the commanded
		// position.  (EnableAxis() would re-enable a step/dir axis at the stale Position.)
		EnableAxisDest(ch, pos);
	}
	else
	{
		// closed loop: Position is the feedback and moves with Dest.  Zero() clears both and
		// keeps commutation (Position + CommutationOffset) continuous; take pos back out of the
		// offset as Position gets it, so a brushless motor's commutation does not jump
		Zero(ch);
		chan[ch].CommutationOffset -= pos;
		chan[ch].Position = chan[ch].Dest = pos;
		EnableAxis(ch);
	}
	chan[ch].LimitSwitchOptions = SaveLimits;

	// 5. to the home position
	if (!(flags & FLAG_NO_FINAL_MOVE))
	{
		if (finalvel > 0) MoveAtVel(ch, home, finalvel);
		else Move(ch, home);
		while (!CheckDone(ch))
			if (!chan[ch].Enable) return 1;
	}
	return 0;
}

main()
{
	int mask = persist.UserData[P_MASK];
	int done = 0, ch, best, r;

	persist.UserData[P_OK] = 0;
	persist.UserData[P_FAILED] = 0;
	// in the order of the sequence numbers, lower first (ties: lower channel first)
	for (;;)
	{
		best = -1;
		for (ch = 0; ch < N_HOME_CHANNELS; ch++)
			if ((mask & (1 << ch)) && !(done & (1 << ch)) &&
			    (best < 0 || P_JOINT(ch)[7] < P_JOINT(best)[7]))
				best = ch;
		if (best < 0) break;
		done |= 1 << best;
		persist.UserData[P_CURRENT] = best;
		r = HomeChannel(best);
		if (r == 0)
		{
			persist.UserData[P_OK] |= 1 << best;
			printf("Home: channel %d homed\n", best);
		}
		else
		{
			persist.UserData[P_FAILED] |= 1 << best;
			printf("Home: channel %d failed (%s)\n", best,
			       r == 1 ? "axis disabled" : r == 2 ? "switch not found within the travel" :
			       r == 3 ? "the switch did not change state during the latch" : "no search velocity");
			break;                             // a failure ends the sequence, as in LinuxCNC
		}
	}
	persist.UserData[P_CURRENT] = -1;
}
