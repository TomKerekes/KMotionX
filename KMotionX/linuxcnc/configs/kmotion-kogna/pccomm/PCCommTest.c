// PCCommTest.c - exercises kmotion-motion's service of the board's commands to the PC
// (PC_COMM, persist 100-107; kmotion-motion/kmotion-pccomm.cc) with KMotionCNC's own helpers
// (C Programs/KflopToKMotionCNCFunctions.c), against LinuxCNC on the kmotion-kogna config.
//
// Run from LinuxCNC: MDI "M101" ([KMOTION] MCODE_101 = PROGRAM 6 12 pccomm/PCCommTest.c starts
// this on thread 6 without waiting, so task is idle again when the commands arrive), or
// pccomm/pccomm-test.py, which does that and checks what LinuxCNC's status shows afterwards.
// Each check prints ok or FAIL on the board's console (kmotion-motion's stderr, the terminal
// LinuxCNC was started from with -v -d); the last line is the failure count. The machine must
// be on and homed (MDI needs it); the test changes the X touch-off and
// puts it back.
#include "KMotionDef.h"
#define TMP 10                       // a spare persist for the helpers' transfers
#include "KflopToKMotionCNCFunctions.c"

static int fails = 0;
static void check(int ok, char *what)
{
	printf("  %s   %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok) fails++;
}
static int near(double a, double b, double tol) { return a - b < tol && b - a < tol; }

int main()
{
	int r, units, t, h, dw;
	double x, y, z, a, b, c, d, d2, x0;
	char s[64];

	Delay_sec(1.0);                  // M101's MDI line is over: task is idle
	printf("PC_COMM test\n");

	// a message: an MB_OK box is answered IDOK (1) and the command succeeds
	r = MsgBox("PC_COMM test from the board", MB_OK | MB_ICONASTERISK);
	check(r == 1, "MSG (MB_OK): shown, IDOK");
	// a question nobody can answer: IDCANCEL (2), and the command fails (-1)
	r = MsgBox("Does nobody answer this?", MB_YESNO);
	check(r == 2 && persist.UserData[PC_COMM_PERSIST] == -1, "MSG (MB_YESNO): IDCANCEL, command failed");

	// feed override: 50%, then x1.5 -> 75%
	check(DoPCFloat(PC_COMM_SET_FRO, 0.5f) == 0, "SET_FRO 0.5");
	check(DoPCFloat(PC_COMM_SET_FRO_INC, 1.5f) == 0, "SET_FRO_INC 1.5");

	// touch off X to 1.25 (G10 L20 P0 X1.25 in LinuxCNC), read the DROs back, then put the
	// touch-off back where it was (the offset lives on in the var file otherwise)
	r = GetDROs(&x0, &y, &z, &a, &b, &c);
	check(r == 0, "GET_DROS before the touch-off");
	check(DoPCFloat(PC_COMM_SET_X, 1.25f) == 0, "SET_X 1.25");
	r = GetDROs(&x, &y, &z, &a, &b, &c);
	printf("    DROs %.4f %.4f %.4f\n", x, y, z);
	check(r == 0 && near(x, 1.25, 0.001), "GET_DROS: X reads 1.25");
	check(DoPCFloat(PC_COMM_SET_X, (float) x0) == 0, "SET_X back to where it was");
	r = GetMachine(&x, &y, &z, &a, &b, &c);
	printf("    machine %.4f %.4f %.4f\n", x, y, z);
	check(r == 0, "GET_MACHINE_COORDS");

	r = GetMiscSettings(&units, &t, &h, &dw);
	printf("    units %d (1 inch, 2 mm) T%d H%d D%d\n", units, t, h, dw);
	check(r == 0 && (units == 1 || units == 2), "GET_MISC_SETTINGS");

	check(MDI("G4 P0.2") == 0, "MDI G4 P0.2");
	check(MDI("G99999") != 0, "MDI of a bad line fails");

	// #variables: SET goes through MDI; GET reads the var file, which holds the persistent
	// ones (#5220, the active coordinate system number, is one)
	SetUserDataDouble(12, 42.5);
	check(SetVars(100, 1, 12) == 0, "SET_VARS #100 = 42.5");
	check(GetVars(5220, 1, 13) == 0, "GET_VARS #5220");
	d = GetUserDataDouble(13);
	printf("    #5220 = %.1f\n", d);
	check(d >= 1 && d <= 9, "#5220 is the coordinate system number");

	// the tool table: tool 1's length, changed and put back
	r = GetToolLength(1, &d);
	printf("    tool 1 length %.4f\n", d);
	check(r == 0, "GET_TOOLTABLE_LENGTH 1");
	check(SetToolLength(1, d + 0.01) == 0, "SET_TOOLTABLE_LENGTH 1");
	r = GetToolLength(1, &d2);
	check(r == 0 && near(d2, d + 0.01, 0.0001), "the new length reads back");
	check(SetToolLength(1, d) == 0, "restored");
	check(GetToolTableIndexFromID(1, &t) == 0 && t == 1, "GET_TOOLTABLE_INDEX 1 -> 1");
	check(GetToolTableIndexFromID(9999, &t) != 0, "GET_TOOLTABLE_INDEX 9999 fails");

	// planner parameters: X's counts per inch
	r = GetTPParameter(2, 0, &d);
	printf("    X counts/inch %.1f\n", d);
	check(r == 0 && d > 0, "GET_TP_PARAM counts/inch");

	GetDateTime(0, s);
	printf("    date/time %s\n", s);
	check(persist.UserData[PC_COMM_PERSIST] == 0 && s[0], "GET_DATE_TIME");

	check(DoPCInt(PC_COMM_USER_BUTTON, 0) == 0, "USER_BUTTON 0 ([KMOTION] USER_BUTTON_0)");
	check(DoPCInt(PC_COMM_USER_BUTTON, 99) == -2, "USER_BUTTON 99: not in the ini, -2");
	check(DoPC(PC_COMM_RECREATE_DIALOG_FACE) == -1, "RECREATE_DIALOG_FACE: not supported, -1");

	check(DoPCFloat(PC_COMM_SET_FRO, 1.0f) == 0, "SET_FRO 1.0 (restored)");

	printf("PC_COMM test: %d failure(s)\n", fails);
	persist.UserData[40] = fails;
	return 0;
}
