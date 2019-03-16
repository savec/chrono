#ifndef C_CHRONO__
#define C_CHRONO__

#include "c_indicator.h"
#include "c_adc.h"
#include <cxx_task.h>

class Chrono: public ActiveObject
{
private:
    Indicator indicator;
    ADC adc;
    virtual void runtask();

public:
    Chrono()
    : ActiveObject("Chrono")
    {};
};

#endif /* C_CHRONO__ */