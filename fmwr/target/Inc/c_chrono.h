#ifndef C_CHRONO__
#define C_CHRONO__

#include "c_indicator.h"
#include <cxx_task.h>

class Chrono: public ActiveObject
{
private:
    virtual void runtask();

public:
    Chrono();
};

#endif /* C_CHRONO__ */