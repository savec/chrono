#ifndef ISR_EVENTS__
#define ISR_EVENTS__

namespace ISREvents
{
    enum
    {
        ADC_DMA_EVENT_TC = (1l << 0),
        ADC_DMA_EVENT_TE = (1l << 1)
    };

    enum
    {
        EVT_SHOT_WAS_PERFORMED =    (1ul << 0),
        EVT_COUNTER_OVERFLOW =      (1ul << 1)
    };
}

#endif /* ISR_EVENTS__ */
