#include <c_chrono.h>
#include <stdlib.h>
#include <isr_events.h>
#include "c_adc.h"

uint16_t adc[ADC::ADC_CHANNEL_NUM];

void Chrono::runtask()
{
    for(;;)
    {
        for(int i = 0; i < 1000; i++)
        {
            adc[ADC::ADC_CHANNEL_OPT0] = ADC::get_instance().get_filtered(ADC::ADC_CHANNEL_OPT0);
            adc[ADC::ADC_CHANNEL_OPT1] = ADC::get_instance().get_filtered(ADC::ADC_CHANNEL_OPT1);
            adc[ADC::ADC_CHANNEL_BAT] = ADC::get_instance().get_filtered(ADC::ADC_CHANNEL_BAT);

            char counter_str[4];
            Indicator::get_instance().set(itoa(i, counter_str, 10));
            osDelay(1000);
        }
    }
}

