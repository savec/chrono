#ifndef C_MEDIAN_FILTER__
#define C_MEDIAN_FILTER__

#include <stddef.h>
#include <array>
#include <algorithm>

template <typename T, int S>
class MedianFilter
{
private:
    std::array<T, S> buf;
    size_t data_ix;
    size_t samples_num;

public:
    MedianFilter()
    : data_ix(0)
    , samples_num(0)
    {}

    void add_sample(const T & sample)
    {
        buf[data_ix++] = sample;
        if(data_ix > S - 1) data_ix = 0;
        if(samples_num < S) samples_num ++;
    }

    T get_median()
    {
        std::array<T, S> to_sort(buf);
        std::sort(to_sort.begin(), to_sort.begin() + samples_num);
        return to_sort[samples_num/2];
    }
};


#endif /* C_MEDIAN_FILTER__ */