#include "FreeRTOS.h"

class ADC
{
private:
    static const size_t channels_num = 3;
    static const size_t transfer_length = 32;
    uint16_t adc_data[channels_num * transfer_length];

public:
    ADC();
    void start();

};
