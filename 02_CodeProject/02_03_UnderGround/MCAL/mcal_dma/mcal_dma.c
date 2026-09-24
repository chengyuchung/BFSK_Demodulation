/* mcal_dma.c */
#include "mcal_dma.h"

void mcal_dma_init(void) {}
void mcal_dma_start(uint32_t stream, uint32_t src, uint32_t dst, uint32_t len) {(void)stream;(void)src;(void)dst;(void)len;}
void mcal_dma_stop(uint32_t stream) {(void)stream;}
__weak void mcal_dma_cplt_callback(uint32_t stream) {(void)stream;}
