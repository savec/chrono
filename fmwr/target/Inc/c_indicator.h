#include <cxx_task.h>

class Indicator: public ActiveObject
{
public:
    static const size_t size = 3;

    Indicator()
    : ActiveObject("Indicator")
    {}

    void set(char *str);

private:
    static const uint32_t char_to_segments[];
    uint32_t decode(char ch);
    uint32_t segments[size];
    static const uint32_t digits[size];

    virtual void runtask();

};

