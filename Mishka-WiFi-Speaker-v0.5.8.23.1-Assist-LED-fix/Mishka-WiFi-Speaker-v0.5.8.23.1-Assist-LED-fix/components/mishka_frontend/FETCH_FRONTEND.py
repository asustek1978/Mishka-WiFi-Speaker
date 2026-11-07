"""Fetch TensorFlow microfrontend and KissFFT on a machine with internet access.
Run from this directory: python FETCH_FRONTEND.py
"""
from pathlib import Path
from urllib.request import urlopen
import time
root = Path(__file__).resolve().parent
base = 'https://raw.githubusercontent.com/tensorflow/tensorflow/master/tensorflow/lite/experimental/microfrontend/lib/'
files = """bits.h fft.cc fft.h fft_util.cc fft_util.h filterbank.c filterbank.h filterbank_util.c filterbank_util.h
frontend.c frontend.h frontend_util.c frontend_util.h log_lut.c log_lut.h log_scale.c log_scale.h
log_scale_util.c log_scale_util.h noise_reduction.c noise_reduction.h noise_reduction_util.c
noise_reduction_util.h pcan_gain_control.c pcan_gain_control.h pcan_gain_control_util.c
pcan_gain_control_util.h window.c window.h window_util.c window_util.h""".split()
for f in files:
    dest = root/'tensorflow/lite/experimental/microfrontend/lib'/f
    dest.parent.mkdir(parents=True, exist_ok=True)
    if not dest.exists():
        print('Fetching',f)
        dest.write_bytes(urlopen(base+f,timeout=30).read())
for f, url in {
    'kiss_fft.c':'https://raw.githubusercontent.com/espressif/esp-tflite-micro/master/third_party/kissfft/kiss_fft.c',
    'kiss_fft.h':'https://raw.githubusercontent.com/espressif/esp-tflite-micro/master/third_party/kissfft/kiss_fft.h',
    '_kiss_fft_guts.h':'https://raw.githubusercontent.com/espressif/esp-tflite-micro/master/third_party/kissfft/_kiss_fft_guts.h',
    'kiss_fftr.c':'https://raw.githubusercontent.com/espressif/esp-tflite-micro/master/third_party/kissfft/tools/kiss_fftr.c',
    'kiss_fftr.h':'https://raw.githubusercontent.com/espressif/esp-tflite-micro/master/third_party/kissfft/tools/kiss_fftr.h',
}.items():
    dest = root/'kissfft'/f
    dest.parent.mkdir(parents=True,exist_ok=True)
    if not dest.exists():
        print('Fetching',f)
        dest.write_bytes(urlopen(url,timeout=30).read())
print('Frontend sources downloaded')

# KissFFT is compiled as C, while fft.cc and fft_util.cc are C++.
# Ensure the declarations use C linkage, including on repeated fetches.
for header_name in ('kiss_fft.h', 'kiss_fftr.h'):
    header = root / 'kissfft' / header_name
    content = header.read_text(encoding='utf-8')
    marker = '/* MISHKA_KISSFFT_C_LINKAGE */'
    if marker not in content:
        content = (marker + '\n#ifdef __cplusplus\nextern "C" {\n#endif\n'
                   + content
                   + '\n#ifdef __cplusplus\n}\n#endif\n')
        header.write_text(content, encoding='utf-8')
print('KissFFT C linkage patched')

# Enforce C ABI at the call sites, not only in downloaded KissFFT headers.
# Some include search orders pick another kiss_fftr.h before our patched copy.
for source_name in ('fft.cc', 'fft_util.cc'):
    source = root / 'tensorflow/lite/experimental/microfrontend/lib' / source_name
    content = source.read_text(encoding='utf-8')
    original = '#include "kiss_fft.h"\n#include "kiss_fftr.h"'
    replacement = ('/* MISHKA_KISSFFT_EXPLICIT_C_ABI */\n'
                   'extern "C" {\n'
                   '#include "kiss_fft.h"\n'
                   '#include "kiss_fftr.h"\n'
                   '}')
    if 'MISHKA_KISSFFT_EXPLICIT_C_ABI' not in content:
        if original not in content:
            raise RuntimeError(f'Unexpected KissFFT includes in {source}: inspect manually')
        source.write_text(content.replace(original, replacement, 1), encoding='utf-8')
    print('Verified C ABI in', source_name)

# Explicit bridge: some upstream KissFFT headers resolve as C++ declarations.
# Rename only the C++ call sites to the bridge, keeping C implementation unchanged.
for source_name in ('fft.cc', 'fft_util.cc'):
    source = root / 'tensorflow/lite/experimental/microfrontend/lib' / source_name
    content = source.read_text(encoding='utf-8')
    if 'MISHKA_KISSFFT_C_BRIDGE_V5' not in content:
        import re
        content = re.sub(r'\bkiss_fftr_alloc\s*\(', 'mishka_kiss_fftr_alloc_c(', content)
        content = re.sub(r'\bkiss_fftr\s*\(', 'mishka_kiss_fftr_c(', content)
        declaration = ("""/* MISHKA_KISSFFT_C_BRIDGE_V5 */
extern "C" {
void mishka_kiss_fftr_c(kiss_fftr_cfg cfg, const kiss_fft_scalar *input,
                       kiss_fft_cpx *output);
kiss_fftr_cfg mishka_kiss_fftr_alloc_c(int nfft, int inverse_fft,
                                       void *mem, size_t *lenmem);
}
""")
        position = content.index('#include "kiss_fftr.h"') + len('#include "kiss_fftr.h"')
        content = content[:position] + '\n' + declaration + content[position:]
        source.write_text(content, encoding='utf-8')
    print('C bridge verified in', source_name)
