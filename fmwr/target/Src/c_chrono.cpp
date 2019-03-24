#include <c_chrono.h>
#include <stdlib.h>
#include <isr_events.h>
#include "c_adc.h"
#include "c_dac.h"
#include <gpio.h>

uint16_t adc[cADC::ADC_CHANNEL_NUM];

Chrono::Chrono()
: ActiveObject("Chrono")
{
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1|LL_GPIO_PIN_2);
}

void Chrono::runtask()
{
    for(;;)
    {
        const uint16_t adc_opt0 = cADC::get_instance().get_filtered(cADC::ADC_CHANNEL_OPT0);

        char counter_str[4];
        cIndicator::get_instance().set(itoa((adc_opt0 * 900L) / 4095, counter_str, 10));
        cDAC::get_instance().set_channel1( 128 );
        cDAC::get_instance().set_channel2( 0 );
        osDelay(10);
    }
}

