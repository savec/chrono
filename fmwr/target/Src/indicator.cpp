#include <indicator.h>
#include "cmsis_os.h"

void Indicator::runtask()
{
    for(;;)
    {
        osDelay(1);
    }
}