#include <c_adc.h>
#include "adc.h"
#include "cmsis_os.h"
#include "isr_events.h"
#include "cinterface.h"

using namespace ISREvents;

cADC::cADC()
: ActiveObject("ADC")
{
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC_CHANNEL_NUM);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)adc_data);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR);

    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_ADC_Enable(ADC1);
}

void cADC::runtask()
{
    for(;;)
    {
        osDelay(100);
        LL_ADC_REG_StartConversionSWStart(ADC1);

        uint32_t events = notify_wait(portMAX_DELAY);

        if( ( events & ADC_DMA_EVENT_TC ) != 0 )
        {
            filters[ADC_CHANNEL_OPT0].add_sample(adc_data[ADC_CHANNEL_OPT0]);
            filters[ADC_CHANNEL_OPT1].add_sample(adc_data[ADC_CHANNEL_OPT1]);
            filters[ADC_CHANNEL_BAT].add_sample(adc_data[ADC_CHANNEL_BAT]);
        }
        else if( ( events & ADC_DMA_EVENT_TE ) != 0 )
        {
            assert_param(false);
        }
    }
}

void dma_transfer_complete_handler()
{
    cADC::get_instance().notify_from_isr(ADC_DMA_EVENT_TC);
}

void dma_transfer_error_handler()
{
    cADC::get_instance().notify_from_isr(ADC_DMA_EVENT_TE);
}