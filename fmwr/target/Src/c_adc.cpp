#include <c_adc.h>
#include "adc.h"
#include "cmsis_os.h"

#define SIZE_OF_ARRAY(a)    (sizeof(a)/sizeof(a[0]))

ADC::ADC()
{
}

void ADC::start()
{
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, SIZE_OF_ARRAY(adc_data));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)adc_data);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR);

    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_ADC_Enable(ADC1);
    osDelay(1);

    LL_ADC_REG_StartConversionSWStart(ADC1);
}