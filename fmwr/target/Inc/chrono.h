#include "indicator.h"
#include <cxx_task.h>

class Chrono: public ActiveObject
{
private:
    Indicator indicator;
    virtual void runtask();

public:
    Chrono()
    : ActiveObject("Chrono")
    {};
};
