#ifndef C_CHRONO__
#define C_CHRONO__

#include "c_indicator.h"
#include <cxx_task.h>

class Chrono: public ActiveObject
{
private:
    virtual void runtask();
    void adjust_comparators_reference();
    inline uint8_t convert_12_bits(const uint32_t value);
    uint32_t calculate_velocity();
public:
    Chrono();
};

#endif /* C_CHRONO__ */