#include "KMotionDef.h"
#include "MySpindleDefs.h"
#include "CSSJog.c"
int main()
{
     for (;;)
     {
            WaitNextTimeSlice();
            ServiceCSS();
     }
}
