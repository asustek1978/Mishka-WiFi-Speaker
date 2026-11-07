/* Stable C ABI bridge between C++ microfrontend and C KissFFT. */
#define FIXED_POINT 16
#include "kiss_fft.h"
#include "kiss_fftr.h"

void mishka_kiss_fftr_c(kiss_fftr_cfg cfg, const kiss_fft_scalar *input,
                       kiss_fft_cpx *output) {
    kiss_fftr(cfg, input, output);
}
kiss_fftr_cfg mishka_kiss_fftr_alloc_c(int nfft, int inverse_fft,
                                       void *mem, size_t *lenmem) {
    return kiss_fftr_alloc(nfft, inverse_fft, mem, lenmem);
}
