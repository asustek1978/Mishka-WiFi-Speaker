#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
void mishka_assist_init(void);
void mishka_assist_detected(void);
void mishka_assist_pcm(const int16_t *samples, size_t count);
void mishka_assist_mute(bool muted);
bool mishka_assist_busy(void);
void mishka_assist_set_tts_playing(bool playing);
uint32_t mishka_assist_wake_flash_until(void);
const char *mishka_assist_state(void);
const char *mishka_assist_tts_url(void);
bool mishka_assist_resolve_tts_url(char *out, size_t out_size);
bool mishka_assist_authorization(char *out, size_t out_size);
#ifdef __cplusplus
}
#endif
