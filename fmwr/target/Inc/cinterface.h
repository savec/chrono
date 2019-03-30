#ifndef CINTERFACE__
#define CINTERFACE__

#define EXTI_GATE_IN    LL_EXTI_LINE_7
#define EXTI_GATE_OUT   LL_EXTI_LINE_6

#ifdef __cplusplus
extern "C" {
#endif
    void dma_transfer_complete_handler();
    void dma_transfer_error_handler();
    void gate_out_handler();
    void gate_in_handler();
    void gate_timeout_handler();
#ifdef __cplusplus
}
#endif

#endif /* CINTERFACE__ */
