#include "FreeRTOS.h"

void* operator new(size_t sz)
{
    return pvPortMalloc(sz);
}

void* operator new[](size_t sz)
{
    return pvPortMalloc(sz);
}

void operator delete(void* p)
{
    vPortFree(p);
}

void operator delete[](void* p)
{
    vPortFree(p);
}

void* operator new(size_t size, void* p)
{
    (void)size;
    return p;
}

void* operator new[](size_t size, void* p)
{
    (void)size;
    return p;
}

void operator delete(void*, void*)
{
}

void operator delete[](void*, void*)
{
}