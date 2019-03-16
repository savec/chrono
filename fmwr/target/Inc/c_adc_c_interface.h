#ifndef C_ADC_CINTERFACE__
#define C_ADC_CINTERFACE__

#ifdef __cplusplus
extern "C" {
#endif
    void dma_transfer_complete_handler();
    void dma_transfer_error_handler();
#ifdef __cplusplus
}
#endif

#endif /* C_ADC_CINTERFACE__ */
