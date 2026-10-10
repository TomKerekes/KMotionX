#include "KMotionDef.h"

int main()
{
	for (;;)
	{
		SetStateBit(46,ch0->Dest < -1000.0);
		SetStateBit(47,ch1->Dest < -2000.0);
		SetStateBit(48,ch2->Dest < -3000.0);
	}
}
