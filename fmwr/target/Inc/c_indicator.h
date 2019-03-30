#ifndef C_INDICATOR__
#define C_INDICATOR__

#include <array>
#include <cxx_task.h>
#include <singleton.h>

class cIndicator: public ActiveObject, public Singleton<cIndicator>
{
    friend class Singleton<cIndicator>;
public:
    static const size_t size = 3;
    void set(const char *str);

private:
    cIndicator()
    : ActiveObject("Indicator")
    {}

    static const uint32_t digit_to_segments[];
    uint32_t decode(char ch);
    std::array<uint32_t, size> segments;
    static const uint32_t digits[size];

    virtual void runtask();

};

#endif /* C_INDICATOR__ */