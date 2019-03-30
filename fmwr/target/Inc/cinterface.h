#ifndef CINTERFACE__
#define CINTERFACE__

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
