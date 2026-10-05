// Spindle Using Jogs settings for the kmotion-kogna bench: Dynomotion's
// C Programs/SpindleUsingJogs/CSS/MySpindleDefs.h, edited as its help page says
// ("KMotionCNC Spindle Control"). LinuxCNC's M3, M4, M5 and S run OnCWJog.c, OnCCWJog.c,
// OffJog.c and SpindleJog.c ([KMOTION] SPINDLE_* in kmotion-kogna.ini), which jog channel 3.
#define SPINDLEAXIS 3			// Axis Channel to Jog to rotate Spindle (set up by SpindleAxis.c)
#define FACTOR (1000/60.0)  	// to convert RPM to counts/sec (counts/rev / 60.0sec): 1000 counts/rev
#define SPINDLECW_BIT 1024   	// bit to activate to cause CW rotation (a virtual bit on the bench)
#define SPINDLECCW_BIT 1025		// bit to activate to cause CCW rotation (a virtual bit on the bench)
#define SPEEDVAR 99				// global persistant variable to store latest speed
#define STATEVAR 98				// global persistant variable to store latest state (-1=CCW,0=off,1=CW)
#define KMVAR PC_COMM_CSS_S 	// variable KMotionCNC will pass speed parameter (113)
#define USE_POS_NEG_VOLTAGE 1 	// 0 = output Magnitude, 1 = output positive and negative speed
