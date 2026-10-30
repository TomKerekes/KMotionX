// ToolChange.c - the kmotion-minirouter bench's tool change, run by kmotion-motion for M6
// ([KMOTION] TOOL_CHANGE = PROGRAM_WAIT 5 9 ToolChange.c, through the M6 remap ngc/kmotion_m6.ngc):
// the selected pocket (slot) arrives in persist 9 and the tool ID in persist 10, as ints -
// KMotionCNC's M6 action writes the same two (Dynomotion's C Programs/ToolChange.c reads them).
// The bench has no changer: this prints them to the console. A real changer moves to the change
// position, swaps, and returns; LinuxCNC waits for the program, then marks the tool in the spindle.
#include "KMotionDef.h"

int main()
{
	int slot = persist.UserData[9];     // the pocket the tool is in (the tool table's P)
	int id = persist.UserData[10];      // the tool number (T)
	printf("Tool change: slot %d, tool ID %d\n", slot, id);
}
