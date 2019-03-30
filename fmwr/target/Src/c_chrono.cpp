#include <c_chrono.h>
#include <stdlib.h>
#include <isr_events.h>
#include "c_adc.h"
#include "c_dac.h"
#include <gpio.h>
#include <tim.h>

uint16_t adc[cADC::ADC_CHANNEL_NUM];

Chrono::Chrono()
: ActiveObject("Chrono")
{
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1|LL_GPIO_PIN_2);

    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_7);
}

void Chrono::adjust_comparators_reference()
{
    const uint16_t adc_opt_0 = cADC::get_instance().get_filtered(cADC::ADC_CHANNEL_OPT0);
    const uint16_t adc_opt_1 = cADC::get_instance().get_filtered(cADC::ADC_CHANNEL_OPT1);

    const uint32_t reference_0 = ((4095ul - adc_opt_0) * 25ul) / 100ul + adc_opt_0;
    const uint32_t reference_1 = ((4095ul - adc_opt_1) * 25ul) / 100ul + adc_opt_1;

    cDAC::get_instance().set_channel1( convert_12_bits(reference_0) );
    cDAC::get_instance().set_channel2( convert_12_bits(reference_1) );
}

uint8_t Chrono::convert_12_bits(const uint32_t value)
{
    return (value * 255ul) / 4095ul;
}

uint32_t counter_shot;

void Chrono::runtask()
{
    uint8_t cntr = 0;
    for(;;)
    {
        adjust_comparators_reference();

        const uint32_t evt = notify_wait(100);

        if(evt & ISREvents::EVT_SHOT_WAS_PERFORMED)
        {
            counter_shot++;
        }

        char counter_str[4];
        cIndicator::get_instance().set(itoa(++cntr, counter_str, 10));
    }
}
