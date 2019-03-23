#ifndef ISR_EVENTS__
#define ISR_EVENTS__

namespace ISREvents
{
    enum
    {
        ADC_DMA_EVENT_TC = (1l << 0),
        ADC_DMA_EVENT_TE = (1l << 1)
    };
}

#endif /* ISR_EVENTS__ */
