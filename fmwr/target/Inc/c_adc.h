#ifndef C_ADC__
#define C_ADC__

#include "FreeRTOS.h"
#include "cxx_task.h"
#include "singleton.h"
#include "c_median_filter.h"

class ADC: public ActiveObject, public Singleton<ADC>
{
    friend class Singleton<ADC>;
public:
    enum AdcChannel
    {
        ADC_CHANNEL_OPT0 = 0,
        ADC_CHANNEL_OPT1,
        ADC_CHANNEL_BAT,

        ADC_CHANNEL_NUM
    };

    static ADC & get_instance()
    {
        static ADC instance;
        return instance;
    }
    uint16_t get_filtered(AdcChannel channel) {return filters[channel].get_median();}

private:

    uint16_t adc_data[ADC_CHANNEL_NUM];
    MedianFilter<uint16_t, 8> filters[ADC_CHANNEL_NUM];

    virtual void runtask();

    ADC();
    ~ADC()= default;
    ADC(const ADC&)= delete;
    ADC& operator=(const ADC&)= delete;
};

#endif /* C_ADC__ */