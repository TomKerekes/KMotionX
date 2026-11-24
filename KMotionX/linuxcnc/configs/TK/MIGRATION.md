# TK: converted from KMotionCNC

Made by `linuxcnc/tools/migrate-kmotioncnc` on 2026-10-31 from `/mnt/kmotion/KMotion5.5.1`.
Start it with the desktop icon "LinuxCNC (TK)" or
`~/linuxcnc-dev/scripts/linuxcnc /home/tk/KMotionX/KMotionX/linuxcnc/configs/TK/TK.ini`.

## What was taken

- Setup read: `/mnt/kmotion/KMotion5.5.1/KMotion/Data/GCodeConfigCNC.txt`
- Units: inches (`DefaultKeepOffsetsInches.set`: length_units)
- INIT: button 1 "INIT" runs `KStep\InitKStep3Axis.c` in thread 1
- Coordinate system: DefineCoordSystem(0, 1, 2, -1) -> X=ch0, Y=ch1, Z=ch2
- Home: button 2 "Home" runs `SimpleHomeIndexFunctionTest.c`: 3 SimpleHomeIndexFunction call(s), converted to LinuxCNC's homing (Home.c on the bench protocol)
- Program copied: `KStep/InitKStep3Axis.c` -> `programs/KStep/InitKStep3Axis.c` (the INIT button)
- Program copied: `SpindleUsingJogs/CSS/OnCWJog.c` -> `programs/SpindleUsingJogs/CSS/OnCWJog.c` (an action's program)
- Program copied: `SpindleUsingJogs/CSS/MySpindleDefs.h` -> `programs/SpindleUsingJogs/CSS/MySpindleDefs.h` (included by SpindleUsingJogs/CSS/OnCWJog.c)
- SPINDLE_M3 = PROGRAM 3 0 programs/SpindleUsingJogs/CSS/OnCWJog.c
- SPINDLE_M4: no action in the setup
- Program copied: `SpindleUsingJogs/CSS/OffJog.c` -> `programs/SpindleUsingJogs/CSS/OffJog.c` (an action's program)
- SPINDLE_M5 = PROGRAM 3 0 programs/SpindleUsingJogs/CSS/OffJog.c
- Program copied: `SpindleUsingJogs/CSS/SpindleJog.c` -> `programs/SpindleUsingJogs/CSS/SpindleJog.c` (an action's program)
- SPINDLE_S = PROGRAM 3 113 programs/SpindleUsingJogs/CSS/SpindleJog.c
- Program copied: `SpindleUsingJogs/CSS/CSSJog.c` -> `programs/SpindleUsingJogs/CSS/CSSJog.c` (included by Loop.c: G96 constant surface speed (ServiceCSS))
- Spindle: m_SpindleType=1, m_SpindleAxis=3, m_SpindleCntsPerRev=3200, m_SpindleTau=0.1
- Program copied: `ToolChange.c` -> `programs/ToolChange.c` (an action's program)
- M6 (tool change): TOOL_CHANGE = PROGRAM_WAIT_SYNC 4 9 programs/ToolChange.c; the pocket and tool ID go to the program's persist variable and the next one as ints, as in KMotionCNC (REMAP=M6 in [RS274NGC], ngc/kmotion_m6.ngc, mcodes/M198)
- Limit switches: none watched in the INIT program; soft limits only
- Home Z (channel 2): switch bit 138 at 0, search -1000 counts/s -> HOME_SEARCH_VEL -0.1, back off until the switch opens at 100 counts/s (HOME_LATCH_VEL 0.01), no index, then 5000 counts inside -> HOME 0.5; sequence 0
- Home X (channel 0): switch bit 136 at 0, search -1000 counts/s -> HOME_SEARCH_VEL -0.1, back off until the switch opens at 100 counts/s (HOME_LATCH_VEL 0.01), no index, then 5000 counts inside -> HOME 0.5; sequence 1
- Home Y (channel 1): switch bit 137 at 0, search -1000 counts/s -> HOME_SEARCH_VEL -0.1, back off until the switch opens at 100 counts/s (HOME_LATCH_VEL 0.01), no index, then 5000 counts inside -> HOME 0.5; sequence 2
- G96: the S program's folder has CSSJog.c (the Spindle Using Jogs CSS set): SPINDLE_CSS = 1 and Loop.c runs ServiceCSS() in thread 1
- Tool table: `Default.tbl`, 4 tools -> tool.tbl (T = KMotionCNC's ID, P = its slot; the values as they are, in the setup's units)
- Fixture offsets and parameters: `emc.var` copied as `TK.var` (the same numbering as LinuxCNC's)
- G-code files: 7 of the 7 in KMotionCNC's recent list copied into nc_files/ (from `/mnt/kmotion/KMotion5.5.1/GCode Programs`)
- Compile check (tcc67, KFLOP image): 7 programs, 0 with errors

## Not converted by design

- M0, M1 and M2 actions: LinuxCNC runs those codes itself.
- KMotionCNC's 8 special actions (program end and the like).
- Rigid tapping (TAP_PROGRAM): copy the bench's Tap.c if the spindle has an encoder.
- Probing: no probe bit in a KMotionCNC setup; set [KMOTION] PROBE_BIT yourself.
- The screen script, the jog step sizes, the DRO settings: gmoccapy has its own.
