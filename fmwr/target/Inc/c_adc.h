#ifndef C_ADC__
#define C_ADC__

#include "FreeRTOS.h"
#include "c_adc_c_interface.h"
#include <cxx_task.h>
#include <event_groups.h>
#include "c_median_filter.h"

class ADC: public ActiveObject
{
private:
    enum
    {
        ADC_DMA_EVENT_HT = (1l << 0),
        ADC_DMA_EVENT_TC = (1l << 1),
        ADC_DMA_EVENT_TE = (1l << 2)
    };

    enum
    {
        ADC_CHANNEL_OPT0 = 0,
        ADC_CHANNEL_OPT1,
        ADC_CHANNEL_BAT,

        ADC_CHANNEL_NUM
    };

    static uint16_t adc_data[];
    static EventGroupHandle_t adc_events;

    MedianFilter<uint16_t, 8> opt0;
    MedianFilter<uint16_t, 8> opt1;
    MedianFilter<uint16_t, 8> bat;

    virtual void runtask();
public:
    ADC();
    friend void dma_transfer_complete_handler();
    friend void dma_transfer_error_handler();
};

#endif /* C_ADC__ */