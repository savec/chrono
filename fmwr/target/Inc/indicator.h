#include <cxx_task.h>

class Indicator: public ActiveObject
{
private:

public:
    Indicator()
    : ActiveObject("indicator")
    {}

    virtual void runtask();
};

