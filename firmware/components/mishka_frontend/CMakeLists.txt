set(FRONTEND "${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite/experimental/microfrontend/lib")
set(KISS "${CMAKE_CURRENT_SOURCE_DIR}/kissfft")
set(FRONTEND_SRCS
  frontend.c frontend_util.c window.c window_util.c filterbank.c filterbank_util.c
  noise_reduction.c noise_reduction_util.c pcan_gain_control.c pcan_gain_control_util.c
  log_scale.c log_scale_util.c log_lut.c fft.cc fft_util.cc)
set(SOURCES "${KISS}/kiss_fft.c" "${KISS}/kiss_fftr.c" "${CMAKE_CURRENT_SOURCE_DIR}/kissfft_c_bridge.c")
foreach(file IN LISTS FRONTEND_SRCS)
  list(APPEND SOURCES "${FRONTEND}/${file}")
endforeach()
idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS "." "kissfft" REQUIRES espressif__esp-tflite-micro)
target_compile_definitions(${COMPONENT_LIB} PRIVATE FIXED_POINT=16)
