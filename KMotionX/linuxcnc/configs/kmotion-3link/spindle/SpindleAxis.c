#include "KMotionDef.h"

// A fake spindle for the 3 Link robot's Kogna (kmotion-3link.ini runs this at start, its
// INIT_PROGRAM), the bench's: channel 3, which Dynomotion's Spindle Using Jogs programs in
// this folder jog when LinuxCNC runs M3, M4, M5 and S. Speed in counts/s is RPM x FACTOR
// (MySpindleDefs.h, 1000 counts/rev): 600 RPM = 10000 counts/s. Tom's trick for a spindle
// without an encoder: step/dir generator 4 puts the motion out in quadrature on JP5 pins 1
// and 2, which are encoder 4's inputs, so Position counts it back as if an encoder measured
// the spindle - the Kogna's JP5 and generator numbering are the KFLOP's. The robot's three
// servos are channels 0-2 (DxlAxisInit3.c); this channel is outside the coordinate system.
// Rigid tapping (Tap.c) runs the spindle through this channel and reads its encoder.
main()
{
  ch3->InputMode=ENCODER_MODE;
  ch3->OutputMode=STEP_DIR_MODE;
  ch3->Vel=100000;          // counts/s, 6000 RPM
  ch3->Accel=10000;         // counts/s^2: 0 to 600 RPM in about a second
  ch3->Jerk=100000;         // counts/s^3
  ch3->P=0;
  ch3->I=0;
  ch3->D=0;
  ch3->FFAccel=0;
  ch3->FFVel=0;
  ch3->MaxI=200;
  ch3->MaxErr=1e+006;
  ch3->MaxOutput=200;
  ch3->DeadBandGain=1;
  ch3->DeadBandRange=0;
  ch3->InputChan0=4;         // encoder 4: JP5 pins 1 and 2
  ch3->InputChan1=0;
  ch3->OutputChan0=28;       // step/dir generator 4, LVTTL, quadrature: on JP5 pins 1 and 2
  ch3->OutputChan1=0;
  ch3->MasterAxis=-1;
  ch3->LimitSwitchOptions=0;
  ch3->LimitSwitchNegBit=0;
  ch3->LimitSwitchPosBit=0;
  ch3->SoftLimitPos=1e+030;  // a spindle turns for ever
  ch3->SoftLimitNeg=-1e+030;
  ch3->InputGain0=1;
  ch3->InputGain1=1;
  ch3->InputOffset0=0;
  ch3->InputOffset1=0;
  ch3->OutputGain=1;
  ch3->OutputOffset=0;
  ch3->SlaveGain=1;
  ch3->BacklashMode=BACKLASH_OFF;
  ch3->BacklashAmount=0;
  ch3->BacklashRate=0;
  ch3->invDistPerCycle=1;
  ch3->Lead=0;
  ch3->MaxFollowingError=1e+030;
  ch3->StepperAmplitude=20;

  ch3->iir[0].B0=1;
  ch3->iir[0].B1=0;
  ch3->iir[0].B2=0;
  ch3->iir[0].A1=0;
  ch3->iir[0].A2=0;

  ch3->iir[1].B0=1;
  ch3->iir[1].B1=0;
  ch3->iir[1].B2=0;
  ch3->iir[1].A1=0;
  ch3->iir[1].A2=0;

  ch3->iir[2].B0=1;
  ch3->iir[2].B1=0;
  ch3->iir[2].B2=0;
  ch3->iir[2].A1=0;
  ch3->iir[2].A2=0;

  EnableAxisDest(3,0);
}
