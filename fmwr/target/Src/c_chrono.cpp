#include <c_chrono.h>
#include <stdlib.h>

void Chrono::runtask()
{
    for(;;)
    {
        for(int i = 0; i < 1000; i++)
        {
            char counter_str[4];
            indicator.set(itoa(i, counter_str, 10));
            osDelay(1000);
        }

    }
}

void Chrono::start()
{
    adc.start();
}