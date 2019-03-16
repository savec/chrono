#include <c_adc.h>
#include "adc.h"
#include "cmsis_os.h"

uint16_t ADC::adc_data[ADC::ADC_CHANNEL_NUM];
EventGroupHandle_t ADC::adc_events;

ADC::ADC()
: ActiveObject("ADC")
{
    adc_events = xEventGroupCreate();
    assert_param(adc_events);

    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC_CHANNEL_NUM);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)adc_data);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR);

    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_ADC_Enable(ADC1);
}

void ADC::runtask()
{
    for(;;)
    {
        osDelay(100);
        LL_ADC_REG_StartConversionSWStart(ADC1);

        EventBits_t events = xEventGroupWaitBits(
                                                adc_events,
                                                ADC_DMA_EVENT_TC | ADC_DMA_EVENT_TE,
                                                pdTRUE,        /* should be cleared before returning. */
                                                pdFALSE,       /* Don't wait for both bits, either bit will do. */
                                                portMAX_DELAY );

        if( ( events & ADC_DMA_EVENT_TC ) != 0 )
        {
            opt0.add_sample(adc_data[ADC_CHANNEL_OPT0]);
            opt1.add_sample(adc_data[ADC_CHANNEL_OPT1]);
            bat.add_sample(adc_data[ADC_CHANNEL_BAT]);
        }
        else if( ( events & ADC_DMA_EVENT_TE ) != 0 )
        {
            assert_param(false);
        }
    }
}

void dma_transfer_complete_handler()
{
    BaseType_t xHigherPriorityTaskWoken(pdFALSE), xResult;

    xResult = xEventGroupSetBitsFromISR(
                                        ADC::adc_events,   /* The event group being updated. */
                                        ADC::ADC_DMA_EVENT_TC, /* The bits being set. */
                                        &xHigherPriorityTaskWoken );

    if( xResult != pdFAIL ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void dma_transfer_error_handler()
{
    BaseType_t xHigherPriorityTaskWoken(pdFALSE), xResult;

    xResult = xEventGroupSetBitsFromISR(
                                        ADC::adc_events,   /* The event group being updated. */
                                        ADC::ADC_DMA_EVENT_TE, /* The bits being set. */
                                        &xHigherPriorityTaskWoken );

    if( xResult != pdFAIL ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
