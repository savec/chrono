#ifndef CXX_TASK__
#define CXX_TASK__

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

class ActiveObject
{
private:
    xTaskHandle handle;

protected:
    virtual void runtask() {}
    static void runtaskstub(void* param) { (static_cast<ActiveObject*>(param))->runtask(); }

public:
    ActiveObject(char const* name, unsigned portBASE_TYPE priority = osPriorityNormal, unsigned portSHORT stackDepth = configMINIMAL_STACK_SIZE)
    {
        xTaskCreate(&runtaskstub, name, stackDepth, this, priority, &handle);
    }

    ~ActiveObject()
    {
#if INCLUDE_vTaskDelete
        vTaskDelete(handle);
#endif
        return;
    }
};

#endif