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
    void start();
};
