#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
bool mishka_wake_ready(void);
void mishka_wake_init(const uint8_t *model, size_t model_size);
void mishka_wake_push(const int16_t *pcm, size_t count);
void mishka_wake_set_muted(bool muted);
#ifdef __cplusplus
}
#endif
