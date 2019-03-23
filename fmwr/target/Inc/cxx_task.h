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

    uint32_t notify_wait(TickType_t timeout)
    {
        uint32_t mask;
        auto result = xTaskNotifyWait( 0, -1, &mask, timeout );
        return (result == pdFALSE) ? 0 : mask;
    }

public:
    ActiveObject(char const* name, unsigned portBASE_TYPE priority = osPriorityNormal, unsigned portSHORT stackDepth = configMINIMAL_STACK_SIZE)
    {
        xTaskCreate(&runtaskstub, name, stackDepth, this, priority, &handle);
    }

    virtual ~ActiveObject()
    {
#if INCLUDE_vTaskDelete
        vTaskDelete(handle);
#endif
        return;
    }

    void notify_from_isr(uint32_t mask)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xTaskNotifyFromISR( handle,
                            mask,
                            eSetBits,
                            &xHigherPriorityTaskWoken );

        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
};

#endif