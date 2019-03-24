#ifndef C_ADC__
#define C_ADC__

#include "FreeRTOS.h"
#include "cxx_task.h"
#include "singleton.h"
#include "c_median_filter.h"

class cADC: public ActiveObject, public Singleton<cADC>
{
    friend class Singleton<cADC>;
public:
    enum AdcChannel
    {
        ADC_CHANNEL_OPT0 = 0,
        ADC_CHANNEL_OPT1,
        ADC_CHANNEL_BAT,

        ADC_CHANNEL_NUM
    };

    static cADC & get_instance()
    {
        static cADC instance;
        return instance;
    }
    uint16_t get_filtered(AdcChannel channel) {return filters[channel].get_median();}

private:

    uint16_t adc_data[ADC_CHANNEL_NUM];
    MedianFilter<uint16_t, 8> filters[ADC_CHANNEL_NUM];

    virtual void runtask();

    cADC();
    ~cADC()= default;
    cADC(const cADC&)= delete;
    cADC& operator=(const cADC&)= delete;
};

#endif /* C_ADC__ */