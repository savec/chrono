#ifndef C_DAC__
#define C_DAC__

#include "dac.h"
#include "singleton.h"

class cDAC: public Singleton<cDAC>
{
    friend class Singleton<cDAC>;
public:

    void set_channel1(const uint8_t value)
    {
        LL_DAC_ConvertData8RightAligned(DAC, LL_DAC_CHANNEL_1, (uint32_t)value);
        LL_DAC_TrigSWConversion(DAC, LL_DAC_CHANNEL_1);
    }

    void set_channel2(const uint8_t value)
    {
        LL_DAC_ConvertData8RightAligned(DAC, LL_DAC_CHANNEL_2, (uint32_t)value);
        LL_DAC_TrigSWConversion(DAC, LL_DAC_CHANNEL_2);
    }

private:
    cDAC()
    {
        LL_DAC_ConvertDualData8RightAligned(DAC, 0xff, 0xff);
        LL_DAC_Enable(DAC, LL_DAC_CHANNEL_1);
        LL_DAC_Enable(DAC, LL_DAC_CHANNEL_2);
    }
};

#endif /* C_DAC__ */