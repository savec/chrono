#ifndef C_INDICATOR__
#define C_INDICATOR__

#include <array>
#include <cxx_task.h>
#include <singleton.h>

class Indicator: public ActiveObject, public Singleton<Indicator>
{
    friend class Singleton<Indicator>;
public:
    static const size_t size = 3;
    void set(char *str);

private:
    Indicator()
    : ActiveObject("Indicator")
    {}

    static const uint32_t char_to_segments[];
    uint32_t decode(char ch);
    std::array<uint32_t, size> segments;
    static const uint32_t digits[size];

    virtual void runtask();

};

#endif /* C_INDICATOR__ */