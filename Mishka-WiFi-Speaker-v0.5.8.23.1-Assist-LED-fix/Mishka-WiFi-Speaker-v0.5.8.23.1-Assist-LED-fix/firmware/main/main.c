#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8311_codec.h"
#include "es7210_adc.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "sdkconfig.h"
#include "freertos/event_groups.h"
#include "esp_heap_caps.h"
#include "nvs.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_app_format.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "esp_audio_simple_dec.h"
#include "esp_audio_dec_default.h"
#include "esp_mac.h"
#include "cJSON.h"
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#define SDA_PIN 11
#define SCL_PIN 10
#define LED_PIN 38
#define LED_COUNT 7
#define KEY_MASK 0x0E  // KEY1..KEY3 on TCA9555 P1 bits 1..3
#define MIC_BOOT_BUTTON GPIO_NUM_0  // BOOT, active LOW; strapping pin
#define PA_BIT 0x01  // EXIO8 = TCA9555 port 1 bit 0
#define ES8311_ADDR 0x18
#define ES7210_ADDR 0x40
#define MIC_DIN GPIO_NUM_15
#define MIC_DIAGNOSTICS_ENABLE 1
#define MIC_VERBOSE_LOGS 0
#define MIC_SUMMARY_PERIOD_MS 30000U
/* Build-time RX-only diagnostic selection: 16 or 32. No live clock switching. */
#ifndef MIC_RX_BITS
#define MIC_RX_BITS 16
#endif
#if MIC_RX_BITS != 16 && MIC_RX_BITS != 32
#error "MIC_RX_BITS must be 16 or 32"
#endif
/* Gain test: change ES7210 MIC1/MIC2 gain only; leave shared clocks and I2S unchanged. */
#define SAMPLE_RATE 16000
static const char *TAG = "MISHKA_V0580";
static i2s_chan_handle_t speaker_tx = NULL;
static i2s_chan_handle_t microphone_rx = NULL;
static esp_codec_dev_handle_t microphone_dev = NULL;
static int speaker_rate = SAMPLE_RATE;
static int speaker_channels = 1;

static esp_err_t reg_read(i2c_master_dev_handle_t d, uint8_t r, uint8_t *v) {
    return i2c_master_transmit_receive(d, &r, 1, v, 1, 100);
}
static esp_err_t reg_write(i2c_master_dev_handle_t d, uint8_t r, uint8_t v) {
    uint8_t b[2] = {r,v};
    return i2c_master_transmit(d,b,2,100);
}
static void led_show(rmt_channel_handle_t ch, rmt_encoder_handle_t enc, uint8_t *buf,
                     uint8_t r, uint8_t g, uint8_t b) {
    for(int i=0;i<LED_COUNT;i++) { buf[3*i]=r; buf[3*i+1]=g; buf[3*i+2]=b; }
    rmt_transmit_config_t cfg={.loop_count=0};
    ESP_ERROR_CHECK(rmt_transmit(ch,enc,buf,LED_COUNT*3,&cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(ch,-1));
}
static bool pa_set(i2c_master_dev_handle_t tca, bool enable) {
    uint8_t before, after, cfg, pins;
    if (reg_read(tca, 0x03, &before) != ESP_OK) return false;
    uint8_t requested = enable ? (before | PA_BIT) : (before & (uint8_t)~PA_BIT);
    if (reg_write(tca, 0x03, requested) != ESP_OK) return false;
    if (reg_read(tca, 0x03, &after) != ESP_OK ||
        reg_read(tca, 0x07, &cfg) != ESP_OK ||
        reg_read(tca, 0x01, &pins) != ESP_OK) return false;
    ESP_LOGI(TAG, "EXIO8 command=%s output_latch=0x%02X config=0x%02X input=0x%02X",
             enable ? "HIGH" : "LOW", after, cfg, pins);
    if ((after & PA_BIT) != (requested & PA_BIT) || (cfg & PA_BIT)) {
        ESP_LOGE(TAG, "EXIO8 register verification FAILED");
        return false;
    }
    return true;
}
static bool pa_prepare(i2c_master_dev_handle_t tca) {
    uint8_t cfg;
    // First latch LOW while EXIO8 is still an input (prevents PA pop).
    uint8_t latch;
    if (reg_read(tca, 0x03, &latch) != ESP_OK) return false;
    if (reg_write(tca, 0x03, latch & (uint8_t)~PA_BIT) != ESP_OK) return false;
    // Configure EXIO8 as output, EXIO9..11 as inputs, then verify.
    if (reg_read(tca, 0x07, &cfg) != ESP_OK) return false;
    cfg = (cfg & (uint8_t)~PA_BIT) | KEY_MASK;
    if (reg_write(tca, 0x07, cfg) != ESP_OK) return false;
    return pa_set(tca, false);
}
/* Microphone bring-up stage 1: read-only I2C presence check.
 * Does not alter ES7210 registers or claim that PCM capture is operational.
 * I2S RX pin and ES7210 register initialization must be verified before enabling capture.
 */
static void microphone_probe(i2c_master_bus_handle_t bus) {
    const uint8_t candidates[] = {0x40, 0x41};
    bool found = false;
    for (size_t i=0; i<sizeof(candidates); i++) {
        esp_err_t e = i2c_master_probe(bus, candidates[i], 100);
        ESP_LOGI(TAG, "MIC_PROBE ES7210 candidate=0x%02X result=%s",
                 candidates[i], esp_err_to_name(e));
        if (e == ESP_OK) found = true;
    }
    ESP_LOGI(TAG, "MIC_PROBE %s; PCM capture NOT enabled; speaker TX unchanged",
             found ? "ADC I2C responder detected" : "no ADC responder detected");
}

/* Five-second diagnostic WAV capture: PSRAM ring, PCM16 mono 16 kHz. */
#define WAV_HZ 16000U
#define WAV_N 80000U
static int16_t *wav_ring;
static SemaphoreHandle_t wav_mutex;
static size_t wav_head, wav_count;
static int wav_input_rate = 16000;
static volatile bool microphone_muted = false;
/* v0.5.8.21: wake word inference is isolated in a separate FreeRTOS task.
 * Producer never runs inference in the I2S read task. */
extern const uint8_t mishka_model_start[] asm("_binary_mishka_tflite_start");
extern const uint8_t mishka_model_end[] asm("_binary_mishka_tflite_end");
#include "mishka_wake.h"
#include "mishka_assist.h"
static void wake_pcm_probe(const int16_t *pcm, size_t count) {
    if (!microphone_muted) { mishka_wake_push(pcm, count); mishka_assist_pcm(pcm,count); }
}

/* Diagnostic voice enhancement, applied only to downloaded WAV.
 * Input MIC_PCM metrics remain RAW for unbiased comparison.
 * No change to playback samples, codec gain, or I2S clocks.
 * First-order DC blocker + slow AGC + smooth peak limiter.
 * This is not acoustic echo cancellation (AEC).
 */
static float mic_hp_x1 = 0.0f, mic_hp_y1 = 0.0f;
static float mic_agc_gain = 1.0f;
static float mic_agc_envelope = 0.0f;
static uint32_t mic_dsp_blocks = 0;
static void mic_dsp_reset(void) {
    mic_hp_x1 = 0.0f;
    mic_hp_y1 = 0.0f;
    mic_agc_gain = 1.0f;
    mic_agc_envelope = 0.0f;
}
static void mic_dsp_process(int16_t *samples, size_t n) {
    if (!n) return;
    /* 16 kHz HPF cutoff approximately 40 Hz (pole 0.9845). */
    float peak = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        float x = (float)samples[i];
        float y = x - mic_hp_x1 + 0.9845f * mic_hp_y1;
        mic_hp_x1 = x;
        mic_hp_y1 = y;
        if (y > 32767.0f) y = 32767.0f;
        if (y < -32768.0f) y = -32768.0f;
        samples[i] = (int16_t)y;
        float a = fabsf(y);
        if (a > peak) peak = a;
    }
    /* Block envelope follower: fast rise, slow decay to avoid pumping. */
    if (peak > mic_agc_envelope)
        mic_agc_envelope = 0.65f * mic_agc_envelope + 0.35f * peak;
    else
        mic_agc_envelope = 0.995f * mic_agc_envelope + 0.005f * peak;
    /* Silence gate prevents amplification of quiet microphone self-noise. */
    float desired = 1.0f;
    if (mic_agc_envelope > 100.0f) {
        desired = 11000.0f / mic_agc_envelope;
        if (desired < 1.0f) desired = 1.0f;
        if (desired > 10.0f) desired = 10.0f;
    }
    /* Reduce gain quickly on loud sound; recover gradually. */
    float alpha = desired < mic_agc_gain ? 0.20f : 0.015f;
    mic_agc_gain += alpha * (desired - mic_agc_gain);
    /* Avoid clipping by limiting gain to available headroom per block. */
    float gain = mic_agc_gain;
    if (peak > 0.0f && peak * gain > 28000.0f)
        gain = 28000.0f / peak;
    for (size_t i = 0; i < n; ++i) {
        float y = (float)samples[i] * gain;
        if (y > 28000.0f) y = 28000.0f;
        if (y < -28000.0f) y = -28000.0f;
        samples[i] = (int16_t)y;
    }
    if ((++mic_dsp_blocks % 250U) == 0U && MIC_VERBOSE_LOGS)
        ESP_LOGI("MIC_DSP", "wav16k AGC=%.2fx envelope=%.0f peak=%.0f (diagnostic, no AEC)",
                 (double)gain, (double)mic_agc_envelope, (double)peak);
}

static void wav_store(const int16_t *buf, size_t n, int rate) {
    if (!wav_ring || !wav_mutex || xSemaphoreTake(wav_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
    if (wav_input_rate != rate) { wav_input_rate = rate; wav_head = wav_count = 0; }
    for (size_t i=0;i<n;i++) { wav_ring[wav_head]=buf[i]; wav_head=(wav_head+1)%WAV_N; if(wav_count<WAV_N)wav_count++; }
    xSemaphoreGive(wav_mutex);
}
static void wav_reset(int rate) {
    if(wav_mutex && xSemaphoreTake(wav_mutex,pdMS_TO_TICKS(20))==pdTRUE) {
        wav_input_rate=rate; wav_head=wav_count=0; xSemaphoreGive(wav_mutex);
    }
}
static void wav_u16(uint8_t *p,uint16_t x){p[0]=x;p[1]=x>>8;}
static void wav_u32(uint8_t *p,uint32_t x){wav_u16(p,x);wav_u16(p+2,x>>16);}
static void wav_header(uint8_t *h,uint32_t bytes){
    memset(h,0,44);memcpy(h,"RIFF",4);wav_u32(h+4,bytes+36);
    memcpy(h+8,"WAVEfmt ",8);wav_u32(h+16,16);wav_u16(h+20,1);wav_u16(h+22,1);
    wav_u32(h+24,WAV_HZ);wav_u32(h+28,WAV_HZ*2);wav_u16(h+32,2);wav_u16(h+34,16);
    memcpy(h+36,"data",4);wav_u32(h+40,bytes);
}
/* Microphone diagnostics: RX uses I2S1 as a SLAVE of the shared TX clocks.
 * Never drive GPIO12/13/14 from RX. The microphone ADC is configured separately.
 * Capture is diagnostic only; no audio is transmitted or stored.
 */
static void microphone_pcm_task(void *arg) {
    (void)arg;
    /* Raw DMA payload. Keep 32-bit words intact to diagnose byte/slot alignment. */
    uint32_t raw[256];
    uint64_t total_frames = 0;
    uint32_t errors = 0;
    uint64_t sum_sq = 0, upper_sq = 0, lower_sq = 0;
    uint32_t upper_nz = 0, lower_nz = 0, even_nz = 0, odd_nz = 0;
    uint32_t positive = 0, negative = 0;
    int peak = 0, upper_peak = 0, lower_peak = 0;
    int32_t first[8] = {0};
    uint32_t reports = 0;
    int last_rate=0, skip_blocks=0;
    uint32_t phase=0;
    int64_t accum=0;
    TickType_t start = xTaskGetTickCount();
    while (true) {
        if (microphone_muted) {
            if (last_rate != 0) {
                mic_dsp_reset();
                phase = 0;
                accum = 0;
                last_rate = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        memset(raw, 0, sizeof(raw));
        int rc = esp_codec_dev_read(microphone_dev, raw, sizeof(raw));
        if (rc != 0) {
            ++errors;
            if (errors % 10 == 1) ESP_LOGW("MIC_PCM", "read error=%d total=%lu", rc, (unsigned long)errors);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        int input_rate=speaker_rate;
        if(input_rate<8000 || input_rate>48000)input_rate=16000;
        if(input_rate!=last_rate){mic_dsp_reset();last_rate=input_rate;skip_blocks=4;phase=0;accum=0;
            wav_reset(input_rate);ESP_LOGI("MIC_WAV","RX rate=%d Hz; buffer reset",input_rate);}
        if(skip_blocks){--skip_blocks;continue;}
        int16_t wav_block[512];size_t wav_len=0;
        const size_t n = sizeof(raw) / (MIC_RX_BITS / 8);
        for (size_t i = 0; i < n; ++i) {
            int32_t v;
            int16_t hi, lo;
            if (MIC_RX_BITS == 32) {
                uint32_t w = raw[i];
                v = (int32_t)w;
                hi = (int16_t)(w >> 16);
                lo = (int16_t)(w & 0xffffU);
            } else {
                uint16_t w = ((const uint16_t *)raw)[i];
                v = (int16_t)w;
                hi = (int16_t)w;
                lo = 0;
            }
            if(MIC_RX_BITS==16){
                /* Area-average resampler: output 16k from 16k/44.1k/48k.
                   Diagnostic quality only, not production ASR filtering. */
                uint32_t remaining=WAV_HZ;
                while(remaining){
                    uint32_t space=(uint32_t)input_rate-phase;
                    uint32_t weight=remaining<space?remaining:space;
                    accum+=(int64_t)hi*weight;phase+=weight;remaining-=weight;
                    if(phase==(uint32_t)input_rate){
                        int32_t sample=(int32_t)(accum/input_rate);
                        if (sample > 32767) {
                            sample = 32767;
                        }
                        if (sample < -32768) {
                            sample = -32768;
                        }
                        if(wav_len<512)wav_block[wav_len++]=(int16_t)sample;
                        accum=0;phase=0;
                    }
                }
            }
            if (reports == 0 && i < 8) first[i] = v;
            /* Use the upper signed 16 bits for comparable amplitude statistics. */
            int32_t a = hi < 0 ? -(int32_t)hi : (int32_t)hi;
            int32_t b = lo < 0 ? -(int32_t)lo : (int32_t)lo;
            if (a > upper_peak) upper_peak = a;
            if (b > lower_peak) lower_peak = b;
            if (a > peak) peak = a;
            sum_sq += (uint64_t)((int64_t)hi * hi);
            upper_sq += (uint64_t)((int64_t)hi * hi);
            lower_sq += (uint64_t)((int64_t)lo * lo);
            if (hi) ++upper_nz;
            if (lo) ++lower_nz;
            if (i & 1U) { if (hi) ++odd_nz; } else { if (hi) ++even_nz; }
            if (hi > 0) ++positive; else if (hi < 0) ++negative;
        }
        if (wav_len && !microphone_muted) {
            /* Preserve the raw 16 kHz stream for future feature extraction. */
            wake_pcm_probe(wav_block, wav_len);
            mic_dsp_process(wav_block, wav_len);
            if (!microphone_muted) wav_store(wav_block, wav_len, input_rate);
        }
        total_frames += n;
        if (++reports >= 32) {
            TickType_t now = xTaskGetTickCount();
            uint32_t elapsed = (uint32_t)((now - start) * portTICK_PERIOD_MS);
            uint32_t rate = elapsed ? (uint32_t)((uint64_t)n * reports * 1000 / elapsed) : 0;
            static uint32_t last_summary_ms=0;
            bool report_summary=(uint32_t)(now*portTICK_PERIOD_MS-last_summary_ms)>=MIC_SUMMARY_PERIOD_MS;
            if (report_summary) last_summary_ms=(uint32_t)(now*portTICK_PERIOD_MS);
            if (MIC_VERBOSE_LOGS || report_summary) ESP_LOGI("MIC_PCM", "bits=%d frames=%llu rms_hi=%d peak_hi=%d errors=%lu rate_est=%luHz tx_rate=%dHz",
                     MIC_RX_BITS, (unsigned long long)total_frames,
                     (int)sqrt((double)sum_sq / (n * reports)), peak, (unsigned long)errors,
                     (unsigned long)rate, speaker_rate);
            if (MIC_VERBOSE_LOGS) ESP_LOGI("MIC_DIAG", "hi_rms=%d lo_rms=%d hi_peak=%d lo_peak=%d hi_nz=%lu lo_nz=%lu even_nz=%lu odd_nz=%lu pos=%lu neg=%lu",
                     (int)sqrt((double)upper_sq / (n * reports)),
                     (int)sqrt((double)lower_sq / (n * reports)), upper_peak, lower_peak,
                     (unsigned long)upper_nz, (unsigned long)lower_nz,
                     (unsigned long)even_nz, (unsigned long)odd_nz,
                     (unsigned long)positive, (unsigned long)negative);
            if (MIC_VERBOSE_LOGS) ESP_LOGI("MIC_RAW", "first8=%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
                     (long)first[0], (long)first[1], (long)first[2], (long)first[3],
                     (long)first[4], (long)first[5], (long)first[6], (long)first[7]);
            reports = 0; sum_sq = upper_sq = lower_sq = 0;
            upper_nz = lower_nz = even_nz = odd_nz = positive = negative = 0;
            peak = upper_peak = lower_peak = 0;
            start = now;
        }
    }
}
/* Read-only snapshot of ES7210 after codec open. */
static void es7210_dump_registers(i2c_master_bus_handle_t bus) {
    static const uint8_t regs[] = {
        0x00,0x01,0x02,0x04,0x05,0x06,0x07,0x08,
        0x11,0x12,0x40,0x41,0x42,0x43,0x44,0x45,0x46,
        0x47,0x48,0x49,0x4A,0x4B,0x4C
    };
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ES7210_ADDR,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &dev);
    if (err != ESP_OK) {
        ESP_LOGE("MIC_REG", "ES7210 I2C handle: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI("MIC_REG", "ES7210 register snapshot (read-only, after codec open):");
    for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); ++i) {
        uint8_t value = 0;
        err = reg_read(dev, regs[i], &value);
        if (err == ESP_OK)
            ESP_LOGI("MIC_REG", "reg 0x%02X = 0x%02X", regs[i], value);
        else
            ESP_LOGW("MIC_REG", "reg 0x%02X read failed: %s", regs[i], esp_err_to_name(err));
    }
    err = i2c_master_bus_rm_device(dev);
    if (err != ESP_OK)
        ESP_LOGW("MIC_REG", "I2C handle cleanup: %s", esp_err_to_name(err));
}

/* ES7210 MIC1/MIC2 gain verification. 0x1A encodes 30 dB in the
 * reference ES7210 driver (gain code 10 plus enable bit 0x10).
 * Only registers 0x43 and 0x44 are written. All other codec settings,
 * including channel mapping, I2S mode and shared clocks, are untouched.
 */
#define MIC_GAIN_REG_VALUE 0x1A
static void es7210_test_mic_gain(i2c_master_bus_handle_t bus) {
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ES7210_ADDR,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &dev);
    if (err != ESP_OK) {
        ESP_LOGE("MIC_GAIN", "Cannot create I2C handle: %s", esp_err_to_name(err));
        return;
    }
    const uint8_t regs[] = {0x43, 0x44};
    uint8_t old[2] = {0};
    bool read_ok = true;
    for (size_t i = 0; i < 2; i++) {
        err = reg_read(dev, regs[i], &old[i]);
        if (err != ESP_OK) {
            ESP_LOGE("MIC_GAIN", "MIC%u initial read failed: %s", (unsigned)(i+1), esp_err_to_name(err));
            read_ok = false;
        } else {
            ESP_LOGI("MIC_GAIN", "MIC%u before: reg 0x%02X=0x%02X", (unsigned)(i+1), regs[i], old[i]);
        }
    }
    if (read_ok) {
        bool verified = true;
        for (size_t i = 0; i < 2; i++) {
            err = reg_write(dev, regs[i], MIC_GAIN_REG_VALUE);
            if (err != ESP_OK) {
                ESP_LOGE("MIC_GAIN", "MIC%u write failed: %s", (unsigned)(i+1), esp_err_to_name(err));
                verified = false;
                continue;
            }
            uint8_t actual = 0;
            err = reg_read(dev, regs[i], &actual);
            if (err != ESP_OK || actual != MIC_GAIN_REG_VALUE) {
                ESP_LOGE("MIC_GAIN", "MIC%u verify failed: err=%s actual=0x%02X expected=0x%02X",
                         (unsigned)(i+1), esp_err_to_name(err), actual, MIC_GAIN_REG_VALUE);
                verified = false;
            } else {
                ESP_LOGI("MIC_GAIN", "MIC%u after: reg 0x%02X=0x%02X (30dB requested)",
                         (unsigned)(i+1), regs[i], actual);
            }
        }
        ESP_LOGI("MIC_GAIN", "30dB MIC1/MIC2 register test: %s", verified ? "VERIFIED" : "FAILED");
    } else {
        ESP_LOGW("MIC_GAIN", "Skipping writes because baseline read failed");
    }
    err = i2c_master_bus_rm_device(dev);
    if (err != ESP_OK) ESP_LOGW("MIC_GAIN", "I2C cleanup: %s", esp_err_to_name(err));
}

static void microphone_pcm_init(i2c_master_bus_handle_t bus) {
#if MIC_DIAGNOSTICS_ENABLE
    if(i2c_master_probe(bus, ES7210_ADDR, 100)!=ESP_OK) {
        ESP_LOGW("MIC_PCM","ES7210 absent; RX disabled"); return;
    }
    i2s_chan_config_t ch=I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1,I2S_ROLE_SLAVE);
    esp_err_t err=i2s_new_channel(&ch,NULL,&microphone_rx);
    if(err!=ESP_OK){ESP_LOGE("MIC_PCM","RX channel: %s",esp_err_to_name(err));return;}
    i2s_std_config_t cfg={
        .clk_cfg=I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg=I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            MIC_RX_BITS == 32 ? I2S_DATA_BIT_WIDTH_32BIT : I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg={.mclk=I2S_GPIO_UNUSED,.bclk=GPIO_NUM_13,.ws=GPIO_NUM_14,
                   .dout=I2S_GPIO_UNUSED,.din=MIC_DIN},
    };
    err=i2s_channel_init_std_mode(microphone_rx,&cfg);
    if(err!=ESP_OK){ESP_LOGE("MIC_PCM","RX init: %s",esp_err_to_name(err));return;}
    audio_codec_i2c_cfg_t ic={.addr=ES7210_ADDR<<1,.bus_handle=bus};
    const audio_codec_ctrl_if_t *ctrl=audio_codec_new_i2c_ctrl(&ic);
    audio_codec_i2s_cfg_t ai={.port=I2S_NUM_1,.rx_handle=microphone_rx};
    const audio_codec_data_if_t *data=audio_codec_new_i2s_data(&ai);
    if(!ctrl || !data){ESP_LOGE("MIC_PCM","codec interfaces failed");return;}
    es7210_codec_cfg_t ac={.ctrl_if=ctrl,.mic_selected=ES7210_SEL_MIC1 | ES7210_SEL_MIC2};
    const audio_codec_if_t *codec=es7210_codec_new(&ac);
    if(!codec){ESP_LOGE("MIC_PCM","ADC codec init failed");return;}
    esp_codec_dev_cfg_t dc={.codec_if=codec,.data_if=data,.dev_type=ESP_CODEC_DEV_TYPE_IN};
    microphone_dev=esp_codec_dev_new(&dc);
    if(!microphone_dev){ESP_LOGE("MIC_PCM","ADC device creation failed");return;}
    esp_codec_dev_sample_info_t fmt={.sample_rate=SAMPLE_RATE,.channel=1,.bits_per_sample=MIC_RX_BITS};
    int rc=esp_codec_dev_open(microphone_dev,&fmt);
    if(rc!=0){ESP_LOGE("MIC_PCM","ADC open failed: %d",rc);microphone_dev=NULL;return;}
    es7210_dump_registers(bus);
    es7210_test_mic_gain(bus);
    wav_mutex=xSemaphoreCreateMutex();
    wav_ring=heap_caps_malloc(WAV_N*sizeof(int16_t),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if(!wav_ring || !wav_mutex)ESP_LOGE("MIC_WAV","PSRAM WAV ring unavailable");
    else ESP_LOGI("MIC_WAV","16k mono rolling 5s buffer ready");
    ESP_LOGI("MIC_PCM","ES7210 RX slave GPIO15 bits=%d MONO LEFT; nominal 16k, actual follows shared BCLK/WS; MIC1/MIC2 gain requested via MIC_GAIN (0x1A)", MIC_RX_BITS);
    mishka_assist_init();
    mishka_wake_init(mishka_model_start, (size_t)(mishka_model_end - mishka_model_start));
    if(xTaskCreate(microphone_pcm_task,"mishka_mic",6144,NULL,3,NULL)!=pdPASS)
        ESP_LOGE("MIC_PCM","task creation failed");
#endif
}

static esp_codec_dev_handle_t audio_init(i2c_master_bus_handle_t bus) {
    if(i2c_master_probe(bus,ES8311_ADDR,100)!=ESP_OK) {
        ESP_LOGE(TAG,"ES8311 not detected at 0x%02X; audio disabled",ES8311_ADDR);
        return NULL;
    }
    i2s_chan_handle_t tx=NULL;
    i2s_chan_config_t ch_cfg=I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0,I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&ch_cfg,&tx,NULL));
    speaker_tx = tx;
    i2s_std_config_t std_cfg={
        .clk_cfg=I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg=I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,I2S_SLOT_MODE_MONO),
        .gpio_cfg={.mclk=GPIO_NUM_12,.bclk=GPIO_NUM_13,.ws=GPIO_NUM_14,
                   .dout=GPIO_NUM_16,.din=I2S_GPIO_UNUSED},
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx,&std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx));
    audio_codec_i2s_cfg_t i2s_cfg={.port=I2S_NUM_0,.tx_handle=tx};
    const audio_codec_data_if_t *data_if=audio_codec_new_i2s_data(&i2s_cfg);
    // esp_codec_dev I2C control interface expects the 8-bit write address.
    // Physical 7-bit address 0x18 corresponds to 8-bit write address 0x30.
    audio_codec_i2c_cfg_t i2c_cfg={.addr=ES8311_CODEC_DEFAULT_ADDR,.bus_handle=bus};
    ESP_LOGI(TAG,"ES8311 physical 7-bit address 0x%02X, codec control address 0x%02X",
             ES8311_ADDR, ES8311_CODEC_DEFAULT_ADDR);
    const audio_codec_ctrl_if_t *ctrl_if=audio_codec_new_i2c_ctrl(&i2c_cfg);
    if(!data_if || !ctrl_if) { ESP_LOGE(TAG,"Codec interfaces unavailable"); return NULL; }
    es8311_codec_cfg_t codec_cfg={
        .ctrl_if=ctrl_if,
        .gpio_if=NULL,
        .codec_mode=ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin=-1, // PA is controlled separately through TCA9555
        .pa_reverted=false,
        .master_mode=false,
        .use_mclk=true,
        .digital_mic=false,
        .invert_mclk=false,
        .invert_sclk=false,
        .mclk_div=256,
    };
    const audio_codec_if_t *codec_if=es8311_codec_new(&codec_cfg);
    if(!codec_if) { ESP_LOGE(TAG,"ES8311 driver creation failed"); return NULL; }
    esp_codec_dev_cfg_t dev_cfg={.codec_if=codec_if,.data_if=data_if,
                                  .dev_type=ESP_CODEC_DEV_TYPE_OUT};
    esp_codec_dev_handle_t dev=esp_codec_dev_new(&dev_cfg);
    if(!dev) { ESP_LOGE(TAG,"Audio device creation failed"); return NULL; }
    esp_codec_dev_sample_info_t fs={.sample_rate=SAMPLE_RATE,.channel=1,.bits_per_sample=16};
    int ret=esp_codec_dev_open(dev,&fs);
    if(ret!=0) { ESP_LOGE(TAG,"Audio open failed: %d",ret); return NULL; }
    ret=esp_codec_dev_set_out_vol(dev,CONFIG_MISHKA_VOLUME);
    if(ret!=0) ESP_LOGW(TAG,"Volume setup returned %d",ret);
    ESP_LOGI(TAG,"ES8311 initialized: 16 kHz MONO 16-bit, volume %d%%; I2S DOUT=GPIO16", CONFIG_MISHKA_VOLUME);
    return dev;
}

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAILED_BIT BIT1
static EventGroupHandle_t wifi_events;
static int retry_count;
static bool wifi_sta_ready;
static char wifi_ssid[33];
static char wifi_password[65];
static char mqtt_host[128];
static char mqtt_user[96];
static char mqtt_password[128];
static int mqtt_port;
static httpd_handle_t config_server;
static esp_err_t portal_favorites(httpd_req_t *req);
static esp_err_t portal_ota_upload(httpd_req_t *req);
static esp_err_t portal_ota_status(httpd_req_t *req);

static void trim_mqtt_user(char *s) {
    char *p=s; while (*p && isspace((unsigned char)*p)) ++p;
    if (p!=s) memmove(s,p,strlen(p)+1);
    size_t n=strlen(s); while(n && isspace((unsigned char)s[n-1])) s[--n]=0;
}
static void settings_load(void) {
    strlcpy(wifi_ssid, CONFIG_MISHKA_WIFI_SSID, sizeof(wifi_ssid));
    strlcpy(wifi_password, CONFIG_MISHKA_WIFI_PASSWORD, sizeof(wifi_password));
    strlcpy(mqtt_host, CONFIG_MISHKA_MQTT_HOST, sizeof(mqtt_host));
    strlcpy(mqtt_user, CONFIG_MISHKA_MQTT_USER, sizeof(mqtt_user));
    strlcpy(mqtt_password, CONFIG_MISHKA_MQTT_PASSWORD, sizeof(mqtt_password));
    mqtt_port = CONFIG_MISHKA_MQTT_PORT;
    nvs_handle_t h;
    if (nvs_open("speaker_cfg", NVS_READONLY, &h) == ESP_OK) {
        size_t n;
#define LOAD_STR(key, dest) do { n=sizeof(dest); nvs_get_str(h,key,dest,&n); } while (0)
        LOAD_STR("ssid", wifi_ssid);
        LOAD_STR("wifi_pass", wifi_password);
        LOAD_STR("mqtt_host", mqtt_host);
        LOAD_STR("mqtt_user", mqtt_user);
        LOAD_STR("mqtt_pass", mqtt_password);
#undef LOAD_STR
        int32_t port;
        if (nvs_get_i32(h,"mqtt_port",&port)==ESP_OK && port>=1 && port<=65535) mqtt_port=port;
        nvs_close(h);
    }
    trim_mqtt_user(mqtt_user);
    // Legacy volume remains in the original 'mishka' namespace; it is never overwritten.
    ESP_LOGI(TAG, "Settings loaded: SSID=%s, MQTT host=%s:%d (credentials hidden)",
             wifi_ssid[0]?wifi_ssid:"<not configured>",mqtt_host,mqtt_port);
    ESP_LOGI(TAG, "MQTT credential diagnostics: username_present=%s username_len=%u password_present=%s password_len=%u",
             mqtt_user[0]?"yes":"no",(unsigned)strlen(mqtt_user),
             mqtt_password[0]?"yes":"no",(unsigned)strlen(mqtt_password));
}

static void wifi_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_sta_ready=false;
        xEventGroupClearBits(wifi_events,WIFI_CONNECTED_BIT);
        if (++retry_count <= 12) esp_wifi_connect();
        else xEventGroupSetBits(wifi_events, WIFI_FAILED_BIT);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        retry_count = 0;
        wifi_sta_ready=true;
        const ip_event_got_ip_t *ev = data;
        ESP_LOGI(TAG, "Wi-Fi IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(wifi_events, WIFI_CONNECTED_BIT);
    }
}

static int hex_digit(char c) {
    if (c>='0'&&c<='9') return c-'0';
    if (c>='a'&&c<='f') return c-'a'+10;
    if (c>='A'&&c<='F') return c-'A'+10;
    return -1;
}
static void form_field(const char *body,const char *key,char *out,size_t cap) {
    size_t kl=strlen(key);
    const char *p=body;
    while (*p) {
        const char *end=strchr(p,'&');
        if (!end) end=p+strlen(p);
        if ((size_t)(end-p)>kl && !memcmp(p,key,kl) && p[kl]=='=') {
            p+=kl+1; size_t n=0;
            while (p<end) {
                char c=*p++;
                if (c=='%' && p+1<end) {
                    int a=hex_digit(p[0]),b=hex_digit(p[1]);
                    if (a>=0&&b>=0) {c=(char)((a<<4)|b);p+=2;}
                } else if (c=='+') c=' ';
                if (n+1<cap) out[n++]=c;
            }
            out[n]=0; return;
        }
        p=(*end=='&')?end+1:end;
    }
    if (cap) out[0]=0;
}
static esp_err_t portal_get(httpd_req_t *req) {
    static const char page[] =
        "<!doctype html><html lang=\"ru\"><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><title>Mishka Speaker</title><style>*{box-sizing:border-box}body{margin:0;background:#10131a;color:#edf2fa;font:15px system-ui,sans-serif}header{padding:20px;background:#1b2230}h1{font-size:22px;margin:0}small,.muted{color:#aab7ca}.wrap{max-width:850px;margin:auto;padding:16px}.tabs{display:flex;gap:6px;overflow:auto;padding:10px 0}.tabs button{width:auto;white-space:nowrap;padding:10px}.tabs button.active{background:#326de1}section{display:none;background:#1b2230;padding:18px;border-radius:12px;margin-bottom:16px}section.active{display:block}label{display:block;margin:14px 0 5px}input,select,button{background:#111827;border:1px solid #47566d;border-radius:8px;color:white;padding:11px;width:100%;font:inherit}input[type=range]{padding:0}button{cursor:pointer;background:#2359b5;margin:8px 0}button:disabled{opacity:.5}button.secondary{background:#344052}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px}.status{padding:10px;border-radius:8px;background:#101b29;margin:10px 0}pre{white-space:pre-wrap;word-break:break-word}hr{border:0;border-top:1px solid #39475a;margin:20px 0}</style><header><div class=\"wrap\"><h1>🐻 Mishka Wi-Fi Speaker</h1><small>v0.5.8.23 · Локальная панель управления</small></div></header><main class=\"wrap\"><nav class=\"tabs\" id=\"tabs\"></nav><section id=\"wifi\"><h2>Wi-Fi</h2><p class=\"muted\">Изменение сети требует перезагрузки.</p><form method=\"post\" action=\"/save\" id=\"network\"><label>Название сети (SSID)</label><input name=\"ssid\" maxlength=\"32\" required><label>Пароль Wi-Fi</label><input name=\"wifi_pass\" type=\"password\" maxlength=\"64\" placeholder=\"Оставьте пустым, чтобы сохранить текущий\"><h2>MQTT</h2><label>Адрес брокера</label><input name=\"mqtt_host\" maxlength=\"127\"><label>Порт</label><input name=\"mqtt_port\" type=\"number\" value=\"1883\" min=\"1\" max=\"65535\"><label>Пользователь</label><input name=\"mqtt_user\" maxlength=\"95\"><label>Пароль MQTT</label><input name=\"mqtt_pass\" type=\"password\" maxlength=\"127\" placeholder=\"Оставьте пустым, чтобы сохранить текущий\"><label>Новый пароль OTA (минимум 12 символов; пустое поле сохраняет прежний)</label><input name=\"ota_token\" type=\"password\" minlength=\"12\" maxlength=\"63\" autocomplete=\"new-password\"><button>Сохранить сеть и перезагрузить</button></form></section><section id=\"audio\"><h2>Звук</h2><p>Громкость: <b id=\"vlabel\">30</b>%</p><input id=\"vol\" type=\"range\" min=\"0\" max=\"100\" value=\"30\"><div class=\"grid\"><button onclick=\"cmd('play')\">▶ Воспроизвести</button><button onclick=\"cmd('pause')\">⏸ Пауза</button><button onclick=\"cmd('stop')\">■ Стоп</button></div><div id=\"playstate\" class=\"status\">Состояние: загрузка...</div></section><section id=\"radio\"><h2>Радиостанции</h2><p class=\"muted\">Прямые URL MP3/AAC/WAV; поддержка зависит от формата потока.</p><label>Адрес потока</label><input id=\"stream\" type=\"url\" placeholder=\"https://.../stream\"><button onclick=\"cmd('url',document.getElementById('stream').value)\">▶ Включить поток</button><div class=\"grid\"><button onclick=\"cmd('previous')\">⏮ Предыдущая</button><button onclick=\"cmd('next')\">⏭ Следующая</button></div><div id=\"favorites\"></div><label>Название избранной станции</label><input id=\"favname\" maxlength=\"63\"><button onclick=\"addFavorite()\">Добавить текущий URL в избранное</button><button class=\"secondary\" onclick=\"if(confirm('Очистить избранное?'))cmd('favorite_clear').then(loadFavorites)\">Очистить избранное</button><p class=\"muted\">Избранные станции сохраняются в NVS. Максимум 8.</p></section><section id=\"lights\"><h2>Светодиоды</h2><label>Цвет</label><input id=\"rgb\" type=\"color\" value=\"#000050\"><label>Яркость: <b id=\"blabel\">90</b></label><input id=\"bright\" type=\"range\" min=\"0\" max=\"255\" value=\"90\" oninput=\"document.getElementById('blabel').textContent=this.value\"><div class=\"grid\"><button onclick=\"light(true)\">Применить цвет</button><button onclick=\"light(false)\">Выключить</button></div><hr><h3>Светомузыка</h3><p class=\"muted\">Режим реагирует на декодированный звук MP3, AAC и WAV. При паузе свет гаснет.</p></section><div class=\"grid\"><button onclick=\"cmd('effect','static')\">Статичный</button><button onclick=\"cmd('effect','breathe')\">Дыхание</button><button onclick=\"cmd('effect','chase')\">Бегущий огонь</button><button onclick=\"cmd('effect','music')\">Светомузыка</button></div><section id=\"buttons\"><h2>Физические кнопки</h2><p>KEY1 — тише, KEY2 — пауза/продолжение, KEY3 — громче, BOOT (GPIO0) — микрофон вкл/выкл.</p><p class=\"muted\">Переназначение кнопок будет доступно после добавления безопасного сохранения конфигурации.</p></section><section id=\"ota\"><h2>OTA</h2><p class=\"muted\">Доступно только после USB-миграции на OTA-разделы. Сначала задайте отдельный пароль OTA в разделе Wi-Fi / MQTT. Загружайте только совместимый firmware.bin. Во время обновления не отключайте питание.</p><label>Пароль OTA</label><input type=\"password\" id=\"otapass\" autocomplete=\"off\" placeholder=\"Отдельный пароль OTA\"><label>Файл прошивки (firmware.bin)</label><input type=\"file\" id=\"otafile\" accept=\".bin,application/octet-stream\"><button onclick=\"uploadOta()\">Загрузить прошивку</button><button class=\"secondary\" onclick=\"confirmOta()\">Подтвердить работающую прошивку</button><pre id=\"otastatus\">Загрузка состояния...</pre></section><section id=\"assist\"><h2>Home Assistant Assist</h2><p>Сервер 192.168.1.67:8123. Токен не отображается и не возвращается в API. Для сохранения требуется пароль OTA.</p><label>Хост</label><input id=\"ha_host\" value=\"192.168.1.67\"><label>Порт</label><input id=\"ha_port\" type=\"number\" value=\"8123\"><label>Pipeline ID (пусто = предпочтительный)</label><input id=\"ha_pipe\"><label>Долгосрочный токен</label><input id=\"ha_token\" type=\"password\" autocomplete=\"new-password\" placeholder=\"Пусто — сохранить прежний\"><label>Пароль OTA (для сохранения настроек)</label><input id=\"ha_ota_pass\" type=\"password\" autocomplete=\"off\" placeholder=\"Пароль OTA\"><button id=\"ha_save_button\" onclick=\"saveAssist()\">Сохранить Assist и перезагрузить</button><p id=\"ha_save_status\" role=\"status\"></p><p>Пароль OTA вводится на вкладке OTA. Тестовый режим Assist: STT и воспроизведение MP3, AAC/ADTS или PCM16 mono WAV через ES8311.</p></section><section id=\"diag\"><h2>Диагностика</h2><button onclick=\"downloadMicWav()\">Скачать WAV 16 кГц (HPF + AGC + ограничитель)</button><button onclick=\"refresh()\">Обновить</button><pre id=\"diagnostics\">Загрузка...</pre><button class=\"secondary\" onclick=\"if(confirm('Перезагрузить колонку?'))cmd('reboot')\">Перезагрузить</button></section><p class=\"muted\" id=\"notice\">Пароли не отображаются. Пустое поле пароля сохраняет прежнее значение. Wi-Fi/MQTT хранятся в NVS.</p></main><script>const pages=[['wifi','Wi-Fi / MQTT'],['audio','Звук'],['radio','Радио'],['lights','Свет'],['buttons','Кнопки'],['ota','OTA'],['assist','Assist'],['diag','Диагностика']];let active='wifi';function show(id){active=id;document.querySelectorAll('section').forEach(x=>x.classList.toggle('active',x.id===id));document.querySelectorAll('.tabs button').forEach(x=>x.classList.toggle('active',x.dataset.page===id))}pages.forEach(([id,name])=>{const b=document.createElement('button');b.textContent=name;b.dataset.page=id;b.onclick=()=>show(id);document.getElementById('tabs').append(b)});show('wifi');async function cmd(action,value=''){try{const r=await fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({action,value})});const t=await r.text();document.getElementById('notice').textContent=t;if(!r.ok)throw Error(t);setTimeout(refresh,350)}catch(e){document.getElementById('notice').textContent='Ошибка: '+e.message}}document.getElementById('vol').onchange=e=>cmd('volume',e.target.value);document.getElementById('vol').oninput=e=>document.getElementById('vlabel').textContent=e.target.value;function light(on){const hex=document.getElementById('rgb').value;cmd('light',on?hex.slice(1)+','+document.getElementById('bright').value:'off')}async function refresh(){try{const r=await fetch('/api/status');const x=await r.json();document.getElementById('diagnostics').textContent=JSON.stringify(x,null,2);document.getElementById('playstate').textContent='Состояние: '+x.state;document.getElementById('vlabel').textContent=x.volume;document.getElementById('vol').value=x.volume;if(!document.getElementById('network').contains(document.activeElement)){if(x.ssid)document.querySelector('[name=ssid]').value=x.ssid;if(x.mqtt_host)document.querySelector('[name=mqtt_host]').value=x.mqtt_host;if(x.mqtt_port)document.querySelector('[name=mqtt_port]').value=x.mqtt_port;if(x.mqtt_user)document.querySelector('[name=mqtt_user]').value=x.mqtt_user}}catch(e){document.getElementById('diagnostics').textContent='Нет связи: '+e.message}}async function loadFavorites(){try{const r=await fetch('/api/favorites');const a=await r.json();const e=document.getElementById('favorites');e.replaceChildren();a.forEach(x=>{const b=document.createElement('button');b.textContent=x.name;b.onclick=()=>cmd('favorite',String(x.index));e.append(b)})}catch(e){}}async function addFavorite(){const name=document.getElementById('favname').value;const url=document.getElementById('stream').value;if(name.includes('|')){alert('Символ | запрещён');return}await cmd('favorite_add',name+'|'+url);loadFavorites()}async function uploadOta(){const f=document.getElementById(\"otafile\").files[0];const pw=document.getElementById(\"otapass\").value;const out=document.getElementById(\"otastatus\");if(!f||pw.length<12){out.textContent=\"Выберите firmware.bin и введите пароль OTA\";return}if(!confirm(\"Обновить прошивку? Не отключайте питание!\"))return;out.textContent=\"Загрузка...\";try{const r=await fetch(\"/api/ota\",{method:\"POST\",headers:{\"X-OTA-Token\":pw,\"Content-Type\":\"application/octet-stream\"},body:f});out.textContent=(r.ok?\"Успешно: \":\"Ошибка: \")+await r.text()}catch(e){out.textContent=\"Сбой соединения: \"+e.message}}async function confirmOta(){const pw=document.getElementById(\"otapass\").value;const out=document.getElementById(\"otastatus\");if(pw.length<12){out.textContent=\"Введите пароль OTA\";return}if(!confirm(\"Подтверждать только после проверки радио, света, Wi-Fi и MQTT. Продолжить?\"))return;try{const r=await fetch(\"/api/ota/confirm\",{method:\"POST\",headers:{\"X-OTA-Token\":pw}});out.textContent=(r.ok?\"Подтверждено: \":\"Ошибка: \")+await r.text()}catch(e){out.textContent=\"Ошибка подтверждения: \"+e.message}}async function downloadMicWav(){const pw=document.getElementById(\"otapass\").value;const note=document.getElementById(\"notice\");if(pw.length<12){note.textContent=\"Введите пароль OTA на вкладке OTA\";return}try{const r=await fetch(\"/api/mic/wav\",{headers:{\"X-OTA-Token\":pw},cache:\"no-store\"});if(!r.ok){note.textContent=\"WAV: \"+await r.text();return}const b=await r.blob();const u=URL.createObjectURL(b);const a=document.createElement(\"a\");a.href=u;a.download=\"mishka-mic-16k.wav\";a.click();setTimeout(()=>URL.revokeObjectURL(u),10000);note.textContent=\"WAV скачан: \"+b.size+\" байт\"}catch(e){note.textContent=\"WAV: \"+e.message}}async function refreshOta(){try{const r=await fetch(\"/api/ota/status\");document.getElementById(\"otastatus\").textContent=JSON.stringify(await r.json(),null,2)}catch(e){}}async function saveAssist(){const status=document.getElementById(\"ha_save_status\");const btn=document.getElementById(\"ha_save_button\");const pass=document.getElementById(\"ha_ota_pass\").value;if(!pass){status.textContent=\"Введите пароль OTA в этом разделе\";return;}const fields={host:document.getElementById(\"ha_host\").value.trim(),port:document.getElementById(\"ha_port\").value,pipeline:document.getElementById(\"ha_pipe\").value.trim(),token:document.getElementById(\"ha_token\").value};btn.disabled=true;status.textContent=\"Сохраняю настройки... В случае успеха Мишка перезагрузится.\";try{const r=await fetch(\"/api/assist/config\",{method:\"POST\",headers:{\"X-OTA-Token\":pass,\"Content-Type\":\"application/x-www-form-urlencoded\"},body:new URLSearchParams(fields)});const msg=await r.text();if(!r.ok)throw new Error(\"HTTP \"+r.status+\": \"+msg);status.textContent=\"Настройки сохранены. Перезагрузка Мишки... Обновите страницу через несколько секунд.\";document.getElementById(\"ha_token\").value=\"\";document.getElementById(\"ha_ota_pass\").value=\"\";}catch(e){status.textContent=\"Ошибка сохранения: \"+e.message;btn.disabled=false;}}refresh();refreshOta();loadFavorites();setInterval(refresh,10000)</script></html>"
        ;
    httpd_resp_set_type(req,"text/html; charset=utf-8");
    httpd_resp_set_hdr(req,"Cache-Control","no-store");
    return httpd_resp_send(req,page,HTTPD_RESP_USE_STRLEN);
}
static void reboot_task(void *unused) {vTaskDelay(pdMS_TO_TICKS(1200));esp_restart();}
static bool ota_authorized(httpd_req_t *req) {
    char provided[64]={0}, saved[64]={0};
    if (httpd_req_get_hdr_value_str(req,"X-OTA-Token",provided,sizeof(provided))!=ESP_OK)
        return false;
    nvs_handle_t h;
    if (nvs_open("speaker_cfg",NVS_READONLY,&h)!=ESP_OK) return false;
    size_t n=sizeof(saved);
    esp_err_t err=nvs_get_str(h,"ota_token",saved,&n);
    nvs_close(h);
    if (err!=ESP_OK || strlen(saved)<12) return false;
    size_t a=strlen(saved), b=strlen(provided);
    unsigned diff=(unsigned)(a^b);
    for(size_t i=0;i<sizeof(saved);i++) diff|=(unsigned)((uint8_t)saved[i]^(uint8_t)provided[i]);
    return diff==0;
}
static esp_err_t portal_mic_wav(httpd_req_t *req){
    if(!ota_authorized(req))return httpd_resp_send_err(req,HTTPD_401_UNAUTHORIZED,"Invalid OTA password");
    if(microphone_muted)return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Microphone muted");
    if(!wav_ring || !wav_mutex)return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"WAV unavailable");
    int16_t *copy=heap_caps_malloc(WAV_N*sizeof(int16_t),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if(!copy)return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"Out of PSRAM");
    if(xSemaphoreTake(wav_mutex,pdMS_TO_TICKS(200))!=pdTRUE){free(copy);return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"WAV busy");}
    if(microphone_muted){xSemaphoreGive(wav_mutex);free(copy);return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Microphone muted");}
    size_t n=wav_count, begin=(wav_head+WAV_N-wav_count)%WAV_N;
    int rate=wav_input_rate;
    for(size_t i=0;i<n;i++)copy[i]=wav_ring[(begin+i)%WAV_N];
    xSemaphoreGive(wav_mutex);
    if(n<16000){free(copy);return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"Wait 1 second");}
    uint8_t h[44];wav_header(h,(uint32_t)(n*2));
    httpd_resp_set_type(req,"audio/wav");
    httpd_resp_set_hdr(req,"Cache-Control","no-store");
    httpd_resp_set_hdr(req,"Content-Disposition","attachment; filename=\"mishka-mic-16k.wav\"");
    esp_err_t e=httpd_resp_send_chunk(req,(const char*)h,44);
    for(size_t i=0;i<n && e==ESP_OK;i+=1024){size_t k=n-i;if(k>1024)k=1024;
        e=httpd_resp_send_chunk(req,(const char*)(copy+i),k*2);}
    if(e==ESP_OK)e=httpd_resp_send_chunk(req,NULL,0);
    ESP_LOGI("MIC_WAV","WAV samples=%lu source_rate=%d result=%s",(unsigned long)n,rate,esp_err_to_name(e));
    free(copy);return e;
}
static esp_err_t portal_ota_status(httpd_req_t *req) {
    const esp_partition_t *running=esp_ota_get_running_partition();
    const esp_partition_t *next=esp_ota_get_next_update_partition(NULL);
    nvs_handle_t h;
    bool configured=false;
    if(nvs_open("speaker_cfg",NVS_READONLY,&h)==ESP_OK) {
        size_t len=0;
        configured=nvs_get_str(h,"ota_token",NULL,&len)==ESP_OK && len>=13;
        nvs_close(h);
    }
    esp_ota_img_states_t ota_state = ESP_OTA_IMG_UNDEFINED;
    bool pending = running && esp_ota_get_state_partition(running, &ota_state) == ESP_OK &&
                   ota_state == ESP_OTA_IMG_PENDING_VERIFY;
    char out[320];
    snprintf(out,sizeof(out),"{\"version\":\"0.5.8.23\",\"active\":\"%s\",\"next\":\"%s\",\"slot_bytes\":%lu,\"password_configured\":%s,\"pending_confirmation\":%s}",
        running?running->label:"unknown",next?next->label:"none",
        (unsigned long)(next?next->size:0),configured?"true":"false",pending?"true":"false");
    httpd_resp_set_type(req,"application/json");
    httpd_resp_set_hdr(req,"Cache-Control","no-store");
    return httpd_resp_sendstr(req,out);
}
static esp_err_t portal_ota_upload(httpd_req_t *req) {
    // No password configured = OTA disabled. No HTTP OTA on untrusted networks.
    if(!ota_authorized(req)) return httpd_resp_send_err(req,HTTPD_401_UNAUTHORIZED,"Invalid OTA password");
    const esp_partition_t *running=esp_ota_get_running_partition();
    const esp_partition_t *target=esp_ota_get_next_update_partition(NULL);
    if(!running || !target || running==target || target->type!=ESP_PARTITION_TYPE_APP)
        return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"No inactive OTA partition; USB migration required");
    if(req->content_len<1024 || (size_t)req->content_len>target->size)
        return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Invalid firmware size");
    uint8_t *buf=malloc(4096);
    if(!buf) return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"Out of memory");
    esp_ota_handle_t handle=0;
    esp_err_t err=esp_ota_begin(target,OTA_SIZE_UNKNOWN,&handle);
    if(err!=ESP_OK) {free(buf);return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"OTA begin failed");}
    int remaining=req->content_len;
    while(remaining>0) {
        int want=remaining>4096?4096:remaining;
        int n=httpd_req_recv(req,(char*)buf,want);
        if(n==HTTPD_SOCK_ERR_TIMEOUT) continue;
        if(n<=0) {err=ESP_FAIL;break;}
        err=esp_ota_write(handle,buf,(size_t)n);
        if(err!=ESP_OK) break;
        remaining-=n;
    }
    free(buf);
    if(err!=ESP_OK || remaining!=0) {
        esp_ota_abort(handle);
        return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"OTA transfer/write failed; previous firmware retained");
    }
    err=esp_ota_end(handle); // validates ESP image structure and checksum
    if(err!=ESP_OK) return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Firmware validation failed");
    err=esp_ota_set_boot_partition(target);
    if(err!=ESP_OK) return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"Cannot select new boot partition");
    ESP_LOGW(TAG,"OTA accepted: new slot=%s, rebooting",target->label);
    httpd_resp_set_type(req,"text/plain; charset=utf-8");
    httpd_resp_sendstr(req,"Firmware accepted; restarting. Verify version and health after reboot.");
    xTaskCreate(reboot_task,"ota_reboot",2048,NULL,3,NULL);
    return ESP_OK;
}
// Do NOT auto-confirm an OTA image merely because tasks were created.
// The user must verify audio, LEDs, Wi-Fi and MQTT, then confirm from the web UI.
// A reboot while PENDING_VERIFY without confirmation triggers ESP-IDF rollback.
static esp_err_t portal_ota_confirm(httpd_req_t *req) {
#if CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
    if (!ota_authorized(req))
        return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "Invalid OTA password");
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (!running || esp_ota_get_state_partition(running, &state) != ESP_OK ||
        state != ESP_OTA_IMG_PENDING_VERIFY)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No OTA image pending verification");
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA confirmation failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA confirmation failed");
    }
    ESP_LOGI(TAG, "OTA image manually confirmed: %s", running->label);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(req, "OTA image confirmed; rollback disabled for this image");
#else
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA rollback not enabled");
#endif
}

static esp_err_t portal_post(httpd_req_t *req) {
    if (req->content_len <= 0 || req->content_len > 2048) return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Invalid request");
    char *body=calloc((size_t)req->content_len+1,1);
    if (!body) return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"Out of memory");
    int got=0;
    while (got<req->content_len) {
        int n=httpd_req_recv(req,body+got,req->content_len-got);
        if(n<=0) {free(body);return httpd_resp_send_err(req,HTTPD_408_REQ_TIMEOUT,"Timeout");}
        got+=n;
    }
    char ssid[33],pass[65],host[128],user[96],pwd[128],portstr[8],ota_token[64];
    form_field(body,"ssid",ssid,sizeof(ssid));form_field(body,"wifi_pass",pass,sizeof(pass));
    form_field(body,"mqtt_host",host,sizeof(host));form_field(body,"mqtt_user",user,sizeof(user));
    form_field(body,"mqtt_pass",pwd,sizeof(pwd));form_field(body,"mqtt_port",portstr,sizeof(portstr));
    form_field(body,"ota_token",ota_token,sizeof(ota_token));
    free(body);
    if (ota_token[0] && strlen(ota_token)<12) return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"OTA password too short");
    int port=atoi(portstr);
    if(!ssid[0] || port<1 || port>65535) return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"SSID or port invalid");
    nvs_handle_t h;
    if(nvs_open("speaker_cfg",NVS_READWRITE,&h)!=ESP_OK) return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"NVS error");
    esp_err_t e=nvs_set_str(h,"ssid",ssid);
    // Empty password field means KEEP the old credential (including sdkconfig fallback).
    if(e==ESP_OK && pass[0]) e=nvs_set_str(h,"wifi_pass",pass);
    else if(e==ESP_OK) {
        size_t old_size=0;
        if(nvs_get_str(h,"wifi_pass",NULL,&old_size)==ESP_ERR_NVS_NOT_FOUND && wifi_password[0])
            e=nvs_set_str(h,"wifi_pass",wifi_password);
    }
    if(e==ESP_OK) e=nvs_set_str(h,"mqtt_host",host);
    if(e==ESP_OK) e=nvs_set_str(h,"mqtt_user",user);
    if(e==ESP_OK && pwd[0]) e=nvs_set_str(h,"mqtt_pass",pwd);
    else if(e==ESP_OK) {
        size_t old_size=0;
        if(nvs_get_str(h,"mqtt_pass",NULL,&old_size)==ESP_ERR_NVS_NOT_FOUND && mqtt_password[0])
            e=nvs_set_str(h,"mqtt_pass",mqtt_password);
    }
    if(e==ESP_OK) e=nvs_set_i32(h,"mqtt_port",port);
    if(e==ESP_OK && ota_token[0]) e=nvs_set_str(h,"ota_token",ota_token);
    // Migration marker is written only when all changes can be committed.
    if(e==ESP_OK) e=nvs_set_i32(h,"cfg_version",1);
    if(e==ESP_OK) e=nvs_commit(h);
    nvs_close(h);
    if(e==ESP_OK) ESP_LOGI(TAG,"Settings saved: mqtt_user_len=%u mqtt_password_updated=%s mqtt_host=%s:%d; rebooting",
                         (unsigned)strlen(user),pwd[0]?"yes":"no",host,port);
    if(e!=ESP_OK) return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"NVS save failed");
    httpd_resp_set_type(req,"text/html; charset=utf-8");
    httpd_resp_sendstr(req,"<meta charset='utf-8'><h2>Настройки сохранены</h2><p>Колонка перезагрузится. Подключитесь к домашней сети.</p>");
    xTaskCreate(reboot_task,"portal_reboot",2048,NULL,3,NULL);
    return ESP_OK;
}
static esp_err_t portal_status(httpd_req_t *req);
static esp_err_t portal_control(httpd_req_t *req);
static esp_err_t portal_assist_config(httpd_req_t *req) {
    if (!ota_authorized(req)) return httpd_resp_send_err(req,HTTPD_401_UNAUTHORIZED,"Invalid OTA password");
    if(req->content_len<=0 || req->content_len>1600) return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Invalid body");
    char body[1601]={0}; int total=0;
    while(total<req->content_len) {
        int n=httpd_req_recv(req,body+total,req->content_len-total);
        if(n<=0) return httpd_resp_send_err(req,HTTPD_408_REQ_TIMEOUT,"Timeout");
        total+=n;
    }
    char host[96]={0},port_text[8]={0},pipeline[96]={0},token[256]={0};
    form_field(body,"host",host,sizeof(host));
    form_field(body,"port",port_text,sizeof(port_text));
    form_field(body,"pipeline",pipeline,sizeof(pipeline));
    form_field(body,"token",token,sizeof(token));
    int port=atoi(port_text);
    if(!host[0] || port<1 || port>65535 || strchr(host,'/') || strchr(host,':') || strchr(host,' '))
        return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Invalid host or port");
    nvs_handle_t h;
    if(nvs_open("speaker_cfg",NVS_READWRITE,&h)!=ESP_OK)
        return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"NVS open failed");
    esp_err_t e=nvs_set_str(h,"ha_host",host);
    if(e==ESP_OK)e=nvs_set_i32(h,"ha_port",port);
    if(e==ESP_OK)e=nvs_set_str(h,"ha_pipe",pipeline);
    if(e==ESP_OK && token[0])e=nvs_set_str(h,"ha_token",token);
    if(e==ESP_OK)e=nvs_commit(h);
    nvs_close(h);
    memset(token,0,sizeof(token));memset(body,0,sizeof(body));
    if(e!=ESP_OK)return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"NVS save failed");
    ESP_LOGI(TAG,"ASSIST_CONFIG: saved to NVS; reboot scheduled (token value hidden)");
    esp_err_t reply=httpd_resp_sendstr(req,"Assist settings saved. Reboot scheduled.");
    if(reply==ESP_OK) {
        BaseType_t created=xTaskCreate(reboot_task,"assist_reboot",2048,NULL,3,NULL);
        if(created!=pdPASS) ESP_LOGE(TAG,"ASSIST_CONFIG: reboot task creation failed");
    }
    return reply;
}
static void portal_start(void) {
    httpd_config_t config=HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers=14;
    config.recv_wait_timeout=30;
    config.send_wait_timeout=30;
    config.stack_size=12288; // HTTP handlers parse POST bodies and allocate JSON
    if(httpd_start(&config_server,&config)!=ESP_OK) {ESP_LOGE(TAG,"Web portal start failed");return;}
    httpd_uri_t root={.uri="/",.method=HTTP_GET,.handler=portal_get};
    httpd_uri_t save={.uri="/save",.method=HTTP_POST,.handler=portal_post};
    httpd_register_uri_handler(config_server,&root);
    httpd_register_uri_handler(config_server,&save);
    httpd_uri_t favorites={.uri="/api/favorites",.method=HTTP_GET,.handler=portal_favorites};
    httpd_register_uri_handler(config_server,&favorites);
    httpd_uri_t status={.uri="/api/status",.method=HTTP_GET,.handler=portal_status};
    httpd_uri_t control={.uri="/api/control",.method=HTTP_POST,.handler=portal_control};
    httpd_register_uri_handler(config_server,&status);
    httpd_register_uri_handler(config_server,&control);
    httpd_uri_t ota={.uri="/api/ota",.method=HTTP_POST,.handler=portal_ota_upload};
    httpd_uri_t ota_status={.uri="/api/ota/status",.method=HTTP_GET,.handler=portal_ota_status};
    httpd_register_uri_handler(config_server,&ota);
    httpd_register_uri_handler(config_server,&ota_status);
    httpd_uri_t ota_confirm={.uri="/api/ota/confirm",.method=HTTP_POST,.handler=portal_ota_confirm};
    httpd_register_uri_handler(config_server,&ota_confirm);
    httpd_uri_t mic_wav={.uri="/api/mic/wav",.method=HTTP_GET,.handler=portal_mic_wav};
    httpd_register_uri_handler(config_server,&mic_wav);
    httpd_uri_t assist_cfg={.uri="/api/assist/config",.method=HTTP_POST,.handler=portal_assist_config};
    httpd_register_uri_handler(config_server,&assist_cfg);
    ESP_LOGI(TAG,"Setup portal available at http://192.168.4.1 or device STA IP");
}
static bool wifi_connect(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    wifi_events=xEventGroupCreate();
    wifi_init_config_t init=WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,wifi_handler,NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,IP_EVENT_STA_GOT_IP,wifi_handler,NULL));
    wifi_config_t ap={0};
    strlcpy((char*)ap.ap.ssid,"Mishka-Speaker-Setup",sizeof(ap.ap.ssid));
    ap.ap.ssid_len=strlen((char*)ap.ap.ssid);
    // Change this bootstrap password after first provisioning; setup AP stays available.
    strlcpy((char*)ap.ap.password,"MishkaSetup2026!",sizeof(ap.ap.password));
    ap.ap.authmode=WIFI_AUTH_WPA2_PSK;ap.ap.max_connection=2;
    ESP_ERROR_CHECK(esp_wifi_set_mode(wifi_ssid[0]?WIFI_MODE_APSTA:WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&ap));
    if(wifi_ssid[0]) {
        wifi_config_t sta={0};
        strlcpy((char*)sta.sta.ssid,wifi_ssid,sizeof(sta.sta.ssid));
        strlcpy((char*)sta.sta.password,wifi_password,sizeof(sta.sta.password));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA,&sta));
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    portal_start();
    if(!wifi_ssid[0]) {ESP_LOGW(TAG,"No Wi-Fi settings: setup AP active");return false;}
    esp_wifi_connect();
    EventBits_t bits=xEventGroupWaitBits(wifi_events,WIFI_CONNECTED_BIT|WIFI_FAILED_BIT,pdFALSE,pdFALSE,pdMS_TO_TICKS(30000));
    return (bits&WIFI_CONNECTED_BIT)!=0;
}

static uint16_t le16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint32_t le32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

static bool read_exact(esp_http_client_handle_t client, void *out, size_t length) {
    uint8_t *p = out;
    while (length) {
        int n = esp_http_client_read(client, (char *)p, length > 2048 ? 2048 : (int)length);
        if (n <= 0) return false;
        p += n;
        length -= n;
    }
    return true;
}

static bool skip_bytes(esp_http_client_handle_t client, uint32_t length) {
    uint8_t scratch[256];
    while (length) {
        size_t part = length > sizeof(scratch) ? sizeof(scratch) : length;
        if (!read_exact(client, scratch, part)) return false;
        length -= part;
    }
    return true;
}

// Parse RIFF/WAVE chunks rather than assuming a fixed 44-byte WAV header.
// Supports PCM16 mono 16000 Hz; other formats are rejected explicitly.
static bool wav_find_data(esp_http_client_handle_t client, uint32_t *data_bytes) {
    uint8_t riff[12];
    if (!read_exact(client, riff, sizeof(riff))) return false;
    if (memcmp(riff, "RIFF", 4) || memcmp(riff + 8, "WAVE", 4)) {
        ESP_LOGE(TAG, "Stream is not RIFF/WAVE PCM (MP3/AAC are not supported in v0.4.0)");
        return false;
    }
    bool fmt_ok = false;
    for (int chunk = 0; chunk < 64; ++chunk) {
        uint8_t h[8];
        if (!read_exact(client, h, sizeof(h))) return false;
        uint32_t len = le32(h + 4);
        if (!memcmp(h, "fmt ", 4)) {
            uint8_t fmt[16];
            if (len < 16 || len > 4096 || !read_exact(client, fmt, sizeof(fmt))) return false;
            uint16_t format = le16(fmt);
            uint16_t channels = le16(fmt + 2);
            uint32_t rate = le32(fmt + 4);
            uint16_t bits = le16(fmt + 14);
            ESP_LOGI(TAG, "WAV format=%u channels=%u sample_rate=%lu bits=%u",
                     format, channels, (unsigned long)rate, bits);
            fmt_ok = format == 1 && channels == 1 && rate == SAMPLE_RATE && bits == 16;
            if (!fmt_ok) ESP_LOGE(TAG, "Required: uncompressed PCM WAV, mono, 16000 Hz, 16-bit");
            if (!skip_bytes(client, len - 16 + (len & 1))) return false;
        } else if (!memcmp(h, "data", 4)) {
            if (!fmt_ok) { ESP_LOGE(TAG, "Missing/unsupported fmt chunk before data"); return false; }
            *data_bytes = len;
            return true;
        } else {
            if (len > 1024 * 1024) return false;
            if (!skip_bytes(client, len + (len & 1))) return false;
        }
    }
    return false;
}

static void write_silence(esp_codec_dev_handle_t dev, int ms) {
    int16_t zero[256] = {0};
    int blocks = (ms * speaker_rate * speaker_channels / 1000 + 255) / 256;
    for (int i = 0; i < blocks; i++) {
        if (esp_codec_dev_write(dev, zero, sizeof(zero)) != 0) break;
    }
}

// Buttons are active-low TCA9555 P1.1/P1.2/P1.3 (EXIO9/10/11).
// A complete WAV is buffered in PSRAM, so pausing never stalls an HTTP socket.
#define MAX_WAV_BYTES (4U * 1024U * 1024U)
static portMUX_TYPE controls_lock = portMUX_INITIALIZER_UNLOCKED;
static bool playback_paused = false;
static int requested_volume = CONFIG_MISHKA_VOLUME;
static bool volume_changed = false;
static bool playback_enabled = false;
static bool playback_active = false;
static bool playback_stop_requested = false;
static char current_url[1024] = CONFIG_MISHKA_AUDIO_URL;
static char pending_url[1024] = {0};
static volatile bool new_url_pending = false;
static esp_mqtt_client_handle_t mqtt_client = NULL;
static char mqtt_base[100], mqtt_state_topic[128], mqtt_volume_topic[128], mqtt_avail_topic[128];
static char mqtt_station_state_topic[128], mqtt_station_set_topic[128];
static volatile bool mqtt_connected = false;
static volatile bool state_dirty = true;
/* Assist state is published independently from radio state. */
static void assist_mqtt_task(void *arg) {
    (void)arg;
    char last[32]="";
    for (;;) {
        if (mqtt_connected && mqtt_client) {
            const char *value=mishka_assist_state();
            if (strcmp(last,value)) {
                char topic[128];
                snprintf(topic,sizeof(topic),"%s/assist/state",mqtt_base);
                esp_mqtt_client_publish(mqtt_client,topic,value,0,1,1);
                strlcpy(last,value,sizeof(last));
            }
        } else last[0]=0;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

#define FAVORITES_MAX 8
#define FAVORITE_URL_LEN 384
static char favorite_names[FAVORITES_MAX][64];
static char favorite_urls[FAVORITES_MAX][FAVORITE_URL_LEN];
static int favorite_count=0, favorite_index=-1;
/* NVS persistence only on user selection; never on the playback retry loop. */
static void station_save_selection(const char *url, int index) {
    if (!url || (strncmp(url, "http://", 7) && strncmp(url, "https://", 8))) return;
    nvs_handle_t h;
    esp_err_t e = nvs_open("mishka", NVS_READWRITE, &h);
    if (e != ESP_OK) { ESP_LOGW(TAG, "STATION_NVS open: %s", esp_err_to_name(e)); return; }
    e = nvs_set_str(h, "last_url", url);
    if (e == ESP_OK) e = nvs_set_i32(h, "last_idx", index);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "STATION_NVS save index=%d result=%s", index, esp_err_to_name(e));
}
static void station_restore_selection(void) {
    nvs_handle_t h;
    if (nvs_open("mishka", NVS_READONLY, &h) != ESP_OK) return;
    int32_t index = -1;
    char url[sizeof(current_url)] = {0};
    size_t n = sizeof(url);
    esp_err_t e = nvs_get_str(h, "last_url", url, &n);
    nvs_get_i32(h, "last_idx", &index);
    nvs_close(h);
    if (e != ESP_OK || (strncmp(url, "http://", 7) && strncmp(url, "https://", 8))) return;
    strlcpy(current_url, url, sizeof(current_url));
    favorite_index = -1;
    if (index >= 0 && index < favorite_count && !strcmp(favorite_urls[index], url)) {
        favorite_index = (int)index;
    } else {
        for (int i = 0; i < favorite_count; ++i) {
            if (!strcmp(favorite_urls[i], url)) { favorite_index = i; break; }
        }
    }
    playback_enabled = false;
    playback_paused = false;
    playback_active = false;
    new_url_pending = false;
    ESP_LOGI(TAG, "STATION_NVS restored index=%d, playback stopped until Play", favorite_index);
}

static void favorites_load(void) {
    nvs_handle_t h;
    if (nvs_open("speaker_fav",NVS_READONLY,&h)!=ESP_OK) return;
    int32_t count=0;
    if(nvs_get_i32(h,"count",&count)!=ESP_OK || count<0 || count>FAVORITES_MAX) count=0;
    favorite_count=0;
    for(int i=0;i<count;i++) {
        char name[64]={0}, url[FAVORITE_URL_LEN]={0}, key[16];
        size_t n=sizeof(name);
        snprintf(key,sizeof(key),"name%d",i);
        esp_err_t en=nvs_get_str(h,key,name,&n);
        n=sizeof(url);
        snprintf(key,sizeof(key),"url%d",i);
        esp_err_t eu=nvs_get_str(h,key,url,&n);
        if(en!=ESP_OK || eu!=ESP_OK || !name[0] ||
           (strncmp(url,"http://",7)!=0 && strncmp(url,"https://",8)!=0)) {
            ESP_LOGW(TAG,"Favorites: skipped invalid NVS entry %d (name=%s url=%s)",
                     i,esp_err_to_name(en),esp_err_to_name(eu));
            continue;
        }
        strlcpy(favorite_names[favorite_count],name,sizeof(favorite_names[0]));
        strlcpy(favorite_urls[favorite_count],url,sizeof(favorite_urls[0]));
        favorite_count++;
    }
    nvs_close(h);
    ESP_LOGI(TAG,"Favorites loaded: %d valid stations",favorite_count);
}
static bool favorites_save(void) {
    nvs_handle_t h;
    if(nvs_open("speaker_fav",NVS_READWRITE,&h)!=ESP_OK) return false;
    esp_err_t e=nvs_set_i32(h,"count",favorite_count);
    for(int i=0;i<favorite_count && e==ESP_OK;i++) {
        char key[16];snprintf(key,sizeof(key),"name%d",i);e=nvs_set_str(h,key,favorite_names[i]);
        if(e!=ESP_OK)break;
        snprintf(key,sizeof(key),"url%d",i);e=nvs_set_str(h,key,favorite_urls[i]);
    }
    if(e==ESP_OK)e=nvs_commit(h);
    nvs_close(h);return e==ESP_OK;
}
static void queue_station(const char *url) {
    portENTER_CRITICAL(&controls_lock);
    strlcpy(pending_url,url,sizeof(pending_url));
    new_url_pending=true;playback_stop_requested=true;playback_enabled=true;
    playback_paused=false;state_dirty=true;
    portEXIT_CRITICAL(&controls_lock);
}
static void station_publish(void) {
    if (!mqtt_client || !mqtt_connected) return;
    const char *name = favorite_index>=0 && favorite_index<favorite_count ? favorite_names[favorite_index] : "";
    esp_mqtt_client_publish(mqtt_client,mqtt_station_state_topic,name,0,1,1);
}
static bool select_favorite(int index) {
    if(index<0 || index>=favorite_count || !favorite_urls[index][0])return false;
    favorite_index=index;station_save_selection(favorite_urls[index], index);queue_station(favorite_urls[index]);station_publish();return true;
}


// Independent WS2812 control. LED output is owned exclusively by led_task.
static portMUX_TYPE led_lock = portMUX_INITIALIZER_UNLOCKED;
static bool led_on = true;
static uint8_t led_r = 0, led_g = 0, led_b = 80, led_brightness = 90;
static volatile bool led_dirty = true;
static uint8_t led_effect=0; // 0 static, 1 breathing, 2 chase, 3 audio reactive
/* LED settings are saved only when a command changes them, not per animation frame. */
static void led_save_settings(void) {
    bool on; uint8_t effect, r, g, b, brightness;
    portENTER_CRITICAL(&led_lock);
    on=led_on; effect=led_effect; r=led_r; g=led_g; b=led_b; brightness=led_brightness;
    portEXIT_CRITICAL(&led_lock);
    nvs_handle_t h;
    esp_err_t e=nvs_open("mishka",NVS_READWRITE,&h);
    if(e!=ESP_OK) { ESP_LOGW(TAG,"LED_NVS open: %s",esp_err_to_name(e)); return; }
    e=nvs_set_u8(h,"led_on",on?1:0);
    if(e==ESP_OK)e=nvs_set_u8(h,"led_fx",effect);
    if(e==ESP_OK)e=nvs_set_u8(h,"led_r",r);
    if(e==ESP_OK)e=nvs_set_u8(h,"led_g",g);
    if(e==ESP_OK)e=nvs_set_u8(h,"led_b",b);
    if(e==ESP_OK)e=nvs_set_u8(h,"led_br",brightness);
    if(e==ESP_OK)e=nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG,"LED_NVS saved effect=%u result=%s",(unsigned)effect,esp_err_to_name(e));
}
static void led_restore_settings(void) {
    nvs_handle_t h;
    if(nvs_open("mishka",NVS_READONLY,&h)!=ESP_OK)return;
    uint8_t v;
    if(nvs_get_u8(h,"led_on",&v)==ESP_OK)led_on=(v!=0);
    if(nvs_get_u8(h,"led_fx",&v)==ESP_OK && v<=3)led_effect=v;
    if(nvs_get_u8(h,"led_r",&v)==ESP_OK)led_r=v;
    if(nvs_get_u8(h,"led_g",&v)==ESP_OK)led_g=v;
    if(nvs_get_u8(h,"led_b",&v)==ESP_OK)led_b=v;
    if(nvs_get_u8(h,"led_br",&v)==ESP_OK)led_brightness=v;
    nvs_close(h);
    led_dirty=true;
    ESP_LOGI(TAG,"LED_NVS restored effect=%u on=%d brightness=%u",(unsigned)led_effect,led_on,(unsigned)led_brightness);
}
static uint8_t music_level=0;
static TickType_t music_last_tick=0;

// PCM amplitude envelope, sampled sparsely to avoid audio pipeline overhead.
static void music_feed_pcm(const uint8_t *pcm, size_t bytes) {
    if (!pcm || bytes < 2) return;
    const int16_t *samples=(const int16_t *)pcm;
    size_t count=bytes/2;
    uint32_t sum=0; size_t n=0;
    for (size_t i=0; i<count; i+=16) {
        int32_t v=samples[i]; if(v<0)v=-v;
        sum+=(uint32_t)v; ++n;
    }
    if (!n) return;
    uint32_t avg=sum/n;
    // Conservative gain and noise gate; user brightness remains master limit.
    uint8_t target=(uint8_t)(avg<140?0:avg>=8000?255:(avg-140)*255/7860);
    portENTER_CRITICAL(&led_lock);
    music_level=(uint8_t)(((unsigned)music_level*2+target)/3);
    music_last_tick=xTaskGetTickCount();
    portEXIT_CRITICAL(&led_lock);
}

static uint8_t led_speed __attribute__((unused)) = 5;
static char led_state_topic[128], led_set_topic[128];

static esp_err_t portal_status(httpd_req_t *req) {
    bool enabled,paused,active;
    int volume;
    portENTER_CRITICAL(&controls_lock);
    enabled=playback_enabled; paused=playback_paused; active=playback_active; volume=requested_volume;
    portEXIT_CRITICAL(&controls_lock);
    char buf[640];
    int rssi=0;
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap)==ESP_OK) rssi=ap.rssi;
    snprintf(buf,sizeof(buf),"{\"version\":\"0.5.8.23\",\"state\":\"%s\",\"volume\":%d,\"rssi\":%d,\"heap\":%u,\"ssid\":\"%s\",\"mqtt_host\":\"%s\",\"mqtt_port\":%d,\"mqtt_user\":\"%s\",\"mqtt_connected\":%s,\"microphone_muted\":%s,\"wake_model_embedded\":true,\"wake_inference_ready\":%s,\"assist_state\":\"%s\",\"assist_busy\":%s}",
      !enabled?"idle":paused?"paused":active?"playing":"buffering",volume,rssi,(unsigned)esp_get_free_heap_size(),wifi_ssid,mqtt_host,mqtt_port,mqtt_user,mqtt_connected?"true":"false",microphone_muted?"true":"false",mishka_wake_ready()?"true":"false",mishka_assist_state(),mishka_assist_busy()?"true":"false");
    httpd_resp_set_type(req,"application/json; charset=utf-8");
    httpd_resp_set_hdr(req,"Cache-Control","no-store");
    return httpd_resp_sendstr(req,buf);
}
static esp_err_t portal_control(httpd_req_t *req) {
    if(req->content_len<=0 || req->content_len>1200) return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Invalid request");
    char body[1201]={0}; int got=0;
    while(got<req->content_len) {int n=httpd_req_recv(req,body+got,req->content_len-got);if(n<=0)return httpd_resp_send_err(req,HTTPD_408_REQ_TIMEOUT,"Timeout");got+=n;}
    char action[24],value[1050];
    form_field(body,"action",action,sizeof(action));form_field(body,"value",value,sizeof(value));
    if(!strcmp(action,"volume")) {
        int n=atoi(value);if(n<0||n>100)return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Volume out of range");
        portENTER_CRITICAL(&controls_lock);requested_volume=n;volume_changed=true;state_dirty=true;portEXIT_CRITICAL(&controls_lock);
        nvs_handle_t h;if(nvs_open("mishka",NVS_READWRITE,&h)==ESP_OK){nvs_set_i32(h,"volume",n);nvs_commit(h);nvs_close(h);}
    } else if(!strcmp(action,"play")) {
        portENTER_CRITICAL(&controls_lock);playback_enabled=true;playback_paused=false;state_dirty=true;portEXIT_CRITICAL(&controls_lock);
    } else if(!strcmp(action,"pause")) {
        portENTER_CRITICAL(&controls_lock);playback_paused=!playback_paused;state_dirty=true;portEXIT_CRITICAL(&controls_lock);
    } else if(!strcmp(action,"stop")) {
        portENTER_CRITICAL(&controls_lock);playback_enabled=false;playback_paused=false;playback_stop_requested=true;state_dirty=true;portEXIT_CRITICAL(&controls_lock);
    } else if(!strcmp(action,"url")) {
        if(strlen(value)<8 || (strncmp(value,"http://",7) && strncmp(value,"https://",8)))return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"HTTP(S) URL required");
        favorite_index=-1;
        station_save_selection(value, -1);
        portENTER_CRITICAL(&controls_lock);strlcpy(pending_url,value,sizeof(pending_url));new_url_pending=true;playback_stop_requested=true;playback_enabled=true;playback_paused=false;state_dirty=true;portEXIT_CRITICAL(&controls_lock);
    } else if(!strcmp(action,"next") || !strcmp(action,"previous")) {
        if(!favorite_count)return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"No favorites");
        int idx=(favorite_index+(strcmp(action,"next")==0?1:-1)+favorite_count)%favorite_count;
        if(!select_favorite(idx))return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Invalid favorite");
    } else if(!strcmp(action,"favorite")) {
        int idx=atoi(value);
        if(!select_favorite(idx))return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Invalid favorite");
    } else if(!strcmp(action,"favorite_add")) {
        // value: name|url; delimiter is forbidden in station names.
        char *sep=strchr(value,'|');
        if(!sep || favorite_count>=FAVORITES_MAX)return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Invalid favorite");
        *sep++=0;
        if(!value[0] || strlen(value)>63 || strlen(sep)>=FAVORITE_URL_LEN ||
           (strncmp(sep,"http://",7) && strncmp(sep,"https://",8)))
            return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Invalid name or URL");
        strlcpy(favorite_names[favorite_count],value,sizeof(favorite_names[0]));
        strlcpy(favorite_urls[favorite_count],sep,sizeof(favorite_urls[0]));
        favorite_count++;
        if(!favorites_save()) { favorite_count--; return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"NVS write failed"); }
        ESP_LOGI(TAG,"Favorite added: slot=%d name_len=%u url_len=%u",favorite_count-1,(unsigned)strlen(value),(unsigned)strlen(sep));
    } else if(!strcmp(action,"favorite_clear")) {
        favorite_count=0;favorite_index=-1;
        station_save_selection(current_url, -1);
        if(!favorites_save())return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"NVS write failed");
    } else if(!strcmp(action,"light")) {
        if(!strcmp(value,"off")) {portENTER_CRITICAL(&led_lock);led_on=false;led_dirty=true;portEXIT_CRITICAL(&led_lock);}
        else {unsigned r,g,b,brightness;if(sscanf(value,"%2x%2x%2x,%u",&r,&g,&b,&brightness)!=4 || brightness>255)return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Invalid light");
          portENTER_CRITICAL(&led_lock);led_on=true;led_r=r;led_g=g;led_b=b;led_brightness=brightness;led_dirty=true;portEXIT_CRITICAL(&led_lock);}
    } else if(!strcmp(action,"effect")) {
        uint8_t mode=255;
        if(!strcmp(value,"static"))mode=0;
        if(!strcmp(value,"breathe"))mode=1;
        if(!strcmp(value,"chase"))mode=2;
        if(!strcmp(value,"music"))mode=3;
        if(mode==255)return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Invalid effect");
        portENTER_CRITICAL(&led_lock);led_effect=mode;led_dirty=true;portEXIT_CRITICAL(&led_lock);
    } else if(!strcmp(action,"reboot")) {
        httpd_resp_sendstr(req,"Restarting");xTaskCreate(reboot_task,"portal_reboot",2048,NULL,3,NULL);return ESP_OK;
    } else return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Unknown action");
    if (!strcmp(action,"light") || !strcmp(action,"effect")) led_save_settings();
    return httpd_resp_sendstr(req,"OK");
}


static void led_publish(void) {
    if (!mqtt_connected || !mqtt_client) return;
    bool on; uint8_t r,g,b,brightness;
    portENTER_CRITICAL(&led_lock);
    on=led_on; r=led_r; g=led_g; b=led_b; brightness=led_brightness;
    portEXIT_CRITICAL(&led_lock);
    char json[160];
    snprintf(json,sizeof(json),"{\"state\":\"%s\",\"brightness\":%u,\"color\":{\"r\":%u,\"g\":%u,\"b\":%u}}",
             on?"ON":"OFF",brightness,r,g,b);
    esp_mqtt_client_publish(mqtt_client,led_state_topic,json,0,1,1);
}

static esp_err_t portal_favorites(httpd_req_t *req) {
    cJSON *arr=cJSON_CreateArray();
    if(!arr)return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"Memory");
    for(int i=0;i<favorite_count;i++) {
        cJSON *obj=cJSON_CreateObject();
        cJSON_AddNumberToObject(obj,"index",i);
        cJSON_AddStringToObject(obj,"name",favorite_names[i]);
        cJSON_AddStringToObject(obj,"url",favorite_urls[i]);
        cJSON_AddItemToArray(arr,obj);
    }
    char *json=cJSON_PrintUnformatted(arr);
    cJSON_Delete(arr);
    if(!json)return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"Memory");
    httpd_resp_set_type(req,"application/json; charset=utf-8");
    esp_err_t e=httpd_resp_sendstr(req,json);free(json);return e;
}
static void led_handle_command(const char *data, int len) {
    if (len<=0 || len>512) return;
    char buf[513]; memcpy(buf,data,len); buf[len]=0;
    cJSON *obj=cJSON_Parse(buf);
    if (!obj) { ESP_LOGW(TAG,"LED: invalid JSON command"); return; }
    cJSON *state=cJSON_GetObjectItemCaseSensitive(obj,"state");
    cJSON *brightness=cJSON_GetObjectItemCaseSensitive(obj,"brightness");
    cJSON *color=cJSON_GetObjectItemCaseSensitive(obj,"color");
    portENTER_CRITICAL(&led_lock);
    if (cJSON_IsString(state)) {
        if (!strcmp(state->valuestring,"ON")) led_on=true;
        if (!strcmp(state->valuestring,"OFF")) led_on=false;
    }
    if (cJSON_IsNumber(brightness)) {
        int n=brightness->valueint; led_brightness=(uint8_t)(n<0?0:n>255?255:n);
    }
    if (cJSON_IsObject(color)) {
        cJSON *r=cJSON_GetObjectItemCaseSensitive(color,"r");
        cJSON *g=cJSON_GetObjectItemCaseSensitive(color,"g");
        cJSON *b=cJSON_GetObjectItemCaseSensitive(color,"b");
        if (cJSON_IsNumber(r)&&cJSON_IsNumber(g)&&cJSON_IsNumber(b)) {
            int rv=r->valueint,gv=g->valueint,bv=b->valueint;
            led_r=(uint8_t)(rv<0?0:rv>255?255:rv);
            led_g=(uint8_t)(gv<0?0:gv>255?255:gv);
            led_b=(uint8_t)(bv<0?0:bv>255?255:bv);
        }
    }
    cJSON *effect=cJSON_GetObjectItemCaseSensitive(obj,"effect");
    if (cJSON_IsString(effect)) {
        if (!strcmp(effect->valuestring,"Статичный")) led_effect=0;
        else if (!strcmp(effect->valuestring,"Дыхание")) led_effect=1;
        else if (!strcmp(effect->valuestring,"Бегущий огонь")) led_effect=2;
        else if (!strcmp(effect->valuestring,"Светомузыка")) led_effect=3;
    }
    led_dirty=true;
    portEXIT_CRITICAL(&led_lock);
    cJSON_Delete(obj);
    led_save_settings();
    led_publish();
}

typedef struct {rmt_channel_handle_t channel; rmt_encoder_handle_t encoder;} led_ctx_t;
static void led_task(void *arg) {
    led_ctx_t *ctx=arg;
    uint8_t pixels[LED_COUNT*3]={0};
    unsigned frame=0;
    for (;;) {
        bool changed=false,on=false; uint8_t r=0,g=0,b=0,brightness=0,effect=0,level=0; TickType_t last_audio=0;
        portENTER_CRITICAL(&led_lock);
        if (led_dirty) {
            changed=true; led_dirty=false;
            on=led_on; r=led_r; g=led_g; b=led_b; brightness=led_brightness;
        }
        on=led_on;r=led_r;g=led_g;b=led_b;brightness=led_brightness;
        effect=led_effect; level=music_level; last_audio=music_last_tick;
        if(effect) {changed=true;on=led_on;r=led_r;g=led_g;b=led_b;brightness=led_brightness;}
        portEXIT_CRITICAL(&led_lock);
        /* Assist overlay overrides music visualizer, including silence and LED OFF.
         * It never modifies led_on, led_effect, brightness, or NVS. */
        static char last_assist_state[32]="";
        static uint32_t error_started_ms=0;
        static bool overlay_was_active=false;
        const char *assist_state=mishka_assist_state();
        uint32_t ms=(uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS);
        bool flash=(int32_t)(mishka_assist_wake_flash_until()-ms)>0;
        bool error=!strcmp(assist_state,"auth_failed") ||
                   !strcmp(assist_state,"pipeline_error") ||
                   !strcmp(assist_state,"timeout_or_disconnected");
        bool listening=!strcmp(assist_state,"listening");
        bool processing=!strcmp(assist_state,"processing");
        bool speaking=!strcmp(assist_state,"speaking");
        bool state_changed=strcmp(last_assist_state,assist_state)!=0;
        if(state_changed) {
            strlcpy(last_assist_state,assist_state,sizeof(last_assist_state));
            if(error) error_started_ms=ms;
            ESP_LOGI("MISHKA_LED","Assist state=%s LED_ON=%d effect=%u brightness=%u",
                     assist_state,on,(unsigned)effect,(unsigned)brightness);
            changed=true;
        }
        bool show_error=error && (uint32_t)(ms-error_started_ms)<900U;
        bool overlay_active=flash || listening || processing || speaking || show_error;
        if(overlay_active) changed=true;
        if(overlay_was_active && !overlay_active) {
            ESP_LOGI("MISHKA_LED","Assist overlay=OFF, restoring effect=%u on=%d",
                     (unsigned)effect,on);
            changed=true;
        }
        if(overlay_active != overlay_was_active) {
            ESP_LOGI("MISHKA_LED","Assist overlay=%s",overlay_active?"ON":"OFF");
        }
        overlay_was_active=overlay_active;
        if (changed) {
            uint8_t rr=on?(uint8_t)((unsigned)r*brightness/255):0;
            uint8_t gg=on?(uint8_t)((unsigned)g*brightness/255):0;
            uint8_t bb=on?(uint8_t)((unsigned)b*brightness/255):0;
            if (overlay_active) {
                /* Minimum visible intensity when music visualization is dark. */
                uint8_t assist_brightness=brightness<48U?48U:brightness;
                if(flash) {
                    rr=0;gg=assist_brightness;bb=0;
                } else if(listening) {
                    rr=0;gg=(uint8_t)((unsigned)assist_brightness*160U/255U);bb=0;
                } else if(processing) {
                    unsigned phase=frame%40U;
                    unsigned strength=50U+(phase<20U?phase:40U-phase)*10U;
                    rr=(uint8_t)((unsigned)assist_brightness*strength/255U);gg=0;bb=rr;
                } else if(speaking) {
                    rr=0;gg=assist_brightness;bb=0;
                } else {
                    rr=((ms-error_started_ms)%300U)<150U?assist_brightness:0;gg=0;bb=0;
                }
                led_show(ctx->channel,ctx->encoder,pixels,rr,gg,bb);
                frame++;
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            if (effect==1 && on) {
                unsigned phase=frame%80; unsigned scale=phase<40?phase:80-phase;
                rr=(uint8_t)((unsigned)rr*(10+scale*245/40)/255);
                gg=(uint8_t)((unsigned)gg*(10+scale*245/40)/255);
                bb=(uint8_t)((unsigned)bb*(10+scale*245/40)/255);
            }
            if (effect==3 && on) {
                // 7-LED amplitude meter, animated rainbow, no microphone required.
                TickType_t age=xTaskGetTickCount()-last_audio;
                if (age>pdMS_TO_TICKS(250)) level=0;
                unsigned count=(unsigned)level*LED_COUNT/256;
                if(level>12 && count==0) count=1;
                memset(pixels,0,sizeof(pixels));
                for(unsigned i=0;i<count;i++) {
                    unsigned hue=(i*256/LED_COUNT + frame*2)%256;
                    uint8_t cr=(hue<85)?255-hue*3:(hue<170?0:(hue-170)*3);
                    uint8_t cg=(hue<85)?hue*3:(hue<170?255-(hue-85)*3:0);
                    uint8_t cb=(hue<85)?0:(hue<170?(hue-85)*3:255-(hue-170)*3);
                    pixels[i*3]=(uint8_t)((unsigned)cr*brightness/255);
                    pixels[i*3+1]=(uint8_t)((unsigned)cg*brightness/255);
                    pixels[i*3+2]=(uint8_t)((unsigned)cb*brightness/255);
                }
                rmt_transmit_config_t cfg={.loop_count=0};
                if(rmt_transmit(ctx->channel,ctx->encoder,pixels,sizeof(pixels),&cfg)==ESP_OK)
                    rmt_tx_wait_all_done(ctx->channel,portMAX_DELAY);
            } else if (effect==2 && on) {
                memset(pixels,0,sizeof(pixels));
                int i=(frame/2)%LED_COUNT;
                pixels[3*i]=rr;pixels[3*i+1]=gg;pixels[3*i+2]=bb;
                rmt_transmit_config_t cfg={.loop_count=0};
                if(rmt_transmit(ctx->channel,ctx->encoder,pixels,sizeof(pixels),&cfg)==ESP_OK)
                    rmt_tx_wait_all_done(ctx->channel,portMAX_DELAY);
            } else led_show(ctx->channel,ctx->encoder,pixels,rr,gg,bb);
        }
        frame++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


static void publish_state(void) {
    if (!mqtt_client || !mqtt_connected) return;
    bool paused, enabled, active;
    int vol;
    portENTER_CRITICAL(&controls_lock);
    paused = playback_paused;
    enabled = playback_enabled;
    active = playback_active;
    vol = requested_volume;
    portEXIT_CRITICAL(&controls_lock);
    const char *state = !enabled ? "idle" : paused ? "paused" : active ? "playing" : "buffering";
    char buf[16];
    snprintf(buf, sizeof(buf), "%.2f", vol / 100.0f);
    esp_mqtt_client_publish(mqtt_client, mqtt_state_topic, state, 0, 1, 1);
    esp_mqtt_client_publish(mqtt_client, mqtt_volume_topic, buf, 0, 1, 1);
    station_publish();
    state_dirty = false;
}
static void set_volume(int value) {
    if (value < 0) value = 0;
    if (value > 100) value = 100;
    portENTER_CRITICAL(&controls_lock);
    requested_volume = value;
    volume_changed = true;
    portEXIT_CRITICAL(&controls_lock);
    nvs_handle_t h;
    if (nvs_open("mishka", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "volume", value);
        nvs_commit(h);
        nvs_close(h);
    }
    state_dirty = true;
}
static void mqtt_event(void *arg, esp_event_base_t base, int32_t id, void *event_data) {
    esp_mqtt_event_handle_t e = event_data;
    if (id == MQTT_EVENT_CONNECTED) {
        mqtt_connected = true;
        char topic[128];
        snprintf(topic, sizeof(topic), "%s/command", mqtt_base);
        esp_mqtt_client_subscribe(e->client, topic, 1);
        snprintf(topic, sizeof(topic), "%s/volume/set", mqtt_base);
        esp_mqtt_client_subscribe(e->client, topic, 1);
        snprintf(topic, sizeof(topic), "%s/media/url/set", mqtt_base);
        esp_mqtt_client_subscribe(e->client, topic, 1);
        esp_mqtt_client_subscribe(e->client, led_set_topic, 1);
        snprintf(topic,sizeof(topic),"%s/station/set",mqtt_base);
        esp_mqtt_client_subscribe(e->client,topic,1);
        char discovery[1152];
        snprintf(discovery, sizeof(discovery),
            "{\"name\":\"Wi-Fi Speaker\",\"unique_id\":\"mishka_wifi_speaker\","
            "\"command_topic\":\"%s/command\",\"state_topic\":\"%s\","
            "\"volume_command_topic\":\"%s/volume/set\",\"volume_state_topic\":\"%s\","
            "\"availability_topic\":\"%s\",\"payload_available\":\"online\","
            "\"payload_not_available\":\"offline\",\"payload_play\":\"PLAY\","
            "\"payload_pause\":\"PAUSE\",\"payload_stop\":\"STOP\","
            "\"device\":{\"identifiers\":[\"mishka_wifi_speaker\"],\"name\":\"Mishka Wi-Fi Speaker\","
            "\"manufacturer\":\"DIY Waveshare\",\"model\":\"ESP32-S3 AUDIO\"}}",
            mqtt_base, mqtt_state_topic, mqtt_base, mqtt_volume_topic, mqtt_avail_topic);
        esp_mqtt_client_publish(e->client, "homeassistant/media_player/mishka_wifi_speaker/config", discovery, 0, 1, 1);
        esp_mqtt_client_publish(e->client, mqtt_avail_topic, "online", 0, 1, 1);
        /* MQTT Discovery sensor: same physical device, no duplicate media player. */
        char assist_discovery[640];
        snprintf(assist_discovery,sizeof(assist_discovery),
            "{\"name\":\"Состояние Assist\",\"unique_id\":\"mishka_wifi_assist_state\","
            "\"state_topic\":\"%s/assist/state\",\"availability_topic\":\"%s\","
            "\"entity_category\":\"diagnostic\","
            "\"device\":{\"identifiers\":[\"mishka_wifi_speaker\"],"
            "\"name\":\"Mishka Wi-Fi Speaker\"}}",mqtt_base,mqtt_avail_topic);
        esp_mqtt_client_publish(e->client,"homeassistant/sensor/mishka_wifi_assist_state/config",assist_discovery,0,1,1);
        char assist_topic[128];
        snprintf(assist_topic,sizeof(assist_topic),"%s/assist/state",mqtt_base);
        esp_mqtt_client_publish(e->client,assist_topic,mishka_assist_state(),0,1,1);

        // A native HA select lists favorite stations; the media player remains the audio transport.
        cJSON *sel=cJSON_CreateObject();
        if (sel) {
            cJSON_AddStringToObject(sel,"name","Радиостанция");
            cJSON_AddStringToObject(sel,"unique_id","mishka_wifi_station");
            cJSON_AddStringToObject(sel,"command_topic",mqtt_station_set_topic);
            cJSON_AddStringToObject(sel,"state_topic",mqtt_station_state_topic);
            cJSON_AddStringToObject(sel,"availability_topic",mqtt_avail_topic);
            cJSON *options=cJSON_AddArrayToObject(sel,"options");
            for (int i=0;i<favorite_count;i++) cJSON_AddItemToArray(options,cJSON_CreateString(favorite_names[i]));
            cJSON *device=cJSON_AddObjectToObject(sel,"device");
            cJSON *ids=cJSON_AddArrayToObject(device,"identifiers");
            cJSON_AddItemToArray(ids,cJSON_CreateString("mishka_wifi_speaker"));
            cJSON_AddStringToObject(device,"name","Mishka Wi-Fi Speaker");
            char *serialized=cJSON_PrintUnformatted(sel);
            if (serialized) {
                esp_mqtt_client_publish(e->client,"homeassistant/select/mishka_wifi_station/config",serialized,0,1,1);
                free(serialized);
            }
            cJSON_Delete(sel);
        }
        station_publish();
        // MQTT Discovery: native RGB light entity, separate from custom media_player.
        char light_config[800];
        int n=snprintf(light_config,sizeof(light_config),
            "{\"name\":\"Подсветка\",\"unique_id\":\"mishka_wifi_speaker_leds\","
            "\"schema\":\"json\",\"command_topic\":\"%s\",\"state_topic\":\"%s\","
            "\"brightness\":true,\"supported_color_modes\":[\"rgb\"],"
            "\"availability_topic\":\"%s\","
            "\"device\":{\"identifiers\":[\"mishka_wifi_speaker\"],"
            "\"name\":\"Mishka Wi-Fi Speaker\",\"manufacturer\":\"DIY Waveshare\","
            "\"model\":\"ESP32-S3 AUDIO\"}}",
            led_set_topic,led_state_topic,mqtt_avail_topic);
        if (n>0 && n<(int)sizeof(light_config))
            esp_mqtt_client_publish(e->client,"homeassistant/light/mishka_wifi_speaker_leds/config",light_config,0,1,1);
        else ESP_LOGE(TAG,"LED Discovery payload too large");
        led_publish();

        state_dirty = true;
        ESP_LOGI(TAG, "MQTT connected; Home Assistant discovery published");
    } else if (id == MQTT_EVENT_DISCONNECTED) {
        mqtt_connected = false;
    } else if (id == MQTT_EVENT_DATA) {
        if (e->topic_len == (int)strlen(led_set_topic) &&
            memcmp(e->topic,led_set_topic,e->topic_len)==0) {
            led_handle_command(e->data,e->data_len);
            return;
        }
        // Favorite selection by exact name (non-fragmented MQTT payload only).
        if (e->topic_len==(int)strlen(mqtt_station_set_topic) &&
            memcmp(e->topic,mqtt_station_set_topic,e->topic_len)==0) {
            if (e->current_data_offset || e->total_data_len!=e->data_len || e->data_len<=0 || e->data_len>=64) return;
            char name[64]; memcpy(name,e->data,e->data_len); name[e->data_len]=0;
            for(int i=0;i<favorite_count;i++) if (!strcmp(name,favorite_names[i])) {select_favorite(i);break;}
            return;
        }
        // URL payloads may be up to 511 bytes; never parse them as short commands.
        const char *suffix = "/media/url/set";
        int suffix_len = (int)strlen(suffix);
        if (e->topic_len >= suffix_len &&
            memcmp(e->topic + e->topic_len - suffix_len, suffix, suffix_len) == 0) {
            if (e->current_data_offset != 0 || e->total_data_len != e->data_len) {
                ESP_LOGE(TAG,"Fragmented MQTT media URL rejected (offset=%d, chunk=%d, total=%d)",
                         e->current_data_offset,e->data_len,e->total_data_len);
                return;
            }
            if (e->data_len < 8 || e->data_len >= (int)sizeof(pending_url)) {
                ESP_LOGE(TAG,"Invalid MQTT URL length=%d (capacity=%u)",e->data_len,(unsigned)sizeof(pending_url));
                return;
            }
            char url[sizeof(pending_url)];
            memcpy(url, e->data, e->data_len);
            url[e->data_len] = 0;
            if (strncmp(url, "http://", 7) != 0 && strncmp(url, "https://", 8) != 0) return;
            favorite_index = -1;
            station_save_selection(url, -1);
            portENTER_CRITICAL(&controls_lock);
            memcpy(pending_url, url, e->data_len + 1);
            new_url_pending = true;
            playback_stop_requested = true;
            playback_enabled = true;
            playback_paused = false;
            portEXIT_CRITICAL(&controls_lock);
            state_dirty = true;
            ESP_LOGI(TAG, "New media URL received (%d bytes, buffer=%u)", e->data_len, (unsigned)sizeof(pending_url));
            return;
        }
        char cmd[64];
        if (e->data_len <= 0 || e->data_len >= (int)sizeof(cmd)) return;
        memcpy(cmd, e->data, e->data_len);
        cmd[e->data_len] = 0;
        if (e->topic_len >= 11 && memcmp(e->topic + e->topic_len - 11, "/volume/set", 11) == 0) {
            char *end;
            float f = strtof(cmd, &end);
            if (end != cmd && *end == '\0' && f >= 0 && f <= 1) set_volume((int)(f * 100 + 0.5f));
        } else if (strcmp(cmd,"NEXT") == 0 || strcmp(cmd,"PREVIOUS") == 0) {
            if (favorite_count) {
                int delta=strcmp(cmd,"NEXT")==0?1:-1;
                int next=(favorite_index+delta+favorite_count)%favorite_count;
                select_favorite(next);
            }
        } else if (strcmp(cmd, "PAUSE") == 0) {
            portENTER_CRITICAL(&controls_lock); playback_paused = true; portEXIT_CRITICAL(&controls_lock);
        } else if (strcmp(cmd, "PLAY") == 0) {
            portENTER_CRITICAL(&controls_lock); playback_enabled = true; playback_paused = false; portEXIT_CRITICAL(&controls_lock);
        } else if (strcmp(cmd, "STOP") == 0) {
            portENTER_CRITICAL(&controls_lock); playback_enabled = false; playback_paused = false; playback_stop_requested = true; portEXIT_CRITICAL(&controls_lock);
        }
        state_dirty = true;
    }
}
static void mqtt_start(void) {
    if (mqtt_host[0] == 0) {
        ESP_LOGW(TAG, "MQTT disabled: set broker address in menuconfig");
        return;
    }
    snprintf(mqtt_base, sizeof(mqtt_base), "mishka/speaker");
    snprintf(mqtt_state_topic, sizeof(mqtt_state_topic), "%s/state", mqtt_base);
    snprintf(mqtt_volume_topic, sizeof(mqtt_volume_topic), "%s/volume/state", mqtt_base);
    snprintf(mqtt_avail_topic, sizeof(mqtt_avail_topic), "%s/availability", mqtt_base);
    snprintf(mqtt_station_set_topic,sizeof(mqtt_station_set_topic),"%s/station/set",mqtt_base);
    snprintf(mqtt_station_state_topic,sizeof(mqtt_station_state_topic),"%s/station/state",mqtt_base);
    snprintf(led_set_topic,sizeof(led_set_topic),"%s/led/set",mqtt_base);
    snprintf(led_state_topic,sizeof(led_state_topic),"%s/led/state",mqtt_base);
    char uri[256];
    snprintf(uri, sizeof(uri), "mqtt://%s:%d", mqtt_host, mqtt_port);
    esp_mqtt_client_config_t cfg = { .broker.address.uri = uri,
        .credentials.username = mqtt_user,
        .credentials.authentication.password = mqtt_password,
        .session.last_will = { .topic = mqtt_avail_topic, .msg = "offline", .msg_len = 7, .qos = 1, .retain = 1 } };
    static bool assist_mqtt_task_started=false;
    if (!assist_mqtt_task_started && xTaskCreate(assist_mqtt_task,"assist_mqtt",3072,NULL,3,NULL)==pdPASS)
        assist_mqtt_task_started=true;
    mqtt_client = esp_mqtt_client_init(&cfg);
    if (mqtt_client) {
        esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event, NULL);
        esp_mqtt_client_start(mqtt_client);
    }
}


static void controls_snapshot(bool *paused, int *volume, bool *changed) {
    portENTER_CRITICAL(&controls_lock);
    *paused = playback_paused;
    *volume = requested_volume;
    *changed = volume_changed;
    volume_changed = false;
    portEXIT_CRITICAL(&controls_lock);
}

static void set_microphone_muted(bool muted, const char *source) {
    portENTER_CRITICAL(&controls_lock);
    microphone_muted = muted;
    portEXIT_CRITICAL(&controls_lock);
    wav_reset(speaker_rate);
    mishka_wake_set_muted(muted);
    mishka_assist_mute(muted);
    ESP_LOGI("MIC_MUTE", "%s: %s", source, muted ? "ON" : "OFF");
    state_dirty = true;
}

static void buttons_task(void *arg) {
    i2c_master_dev_handle_t tca = (i2c_master_dev_handle_t)arg;
    gpio_config_t boot_cfg = {
        .pin_bit_mask = (1ULL << MIC_BOOT_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&boot_cfg));
    uint8_t previous = KEY_MASK;
    uint8_t stable = KEY_MASK;
    uint8_t previous_p0 = 0xFF, previous_p1 = 0xFF;
    bool have_snapshot = false;
    int boot_previous = 1, boot_stable = 1, boot_count = 0;
    int count = 0;
    for (;;) {
        uint8_t pins;
        uint8_t p0;
        if (reg_read(tca, 0x00, &p0) == ESP_OK && reg_read(tca, 0x01, &pins) == ESP_OK) {
            if (have_snapshot && (p0 != previous_p0 || pins != previous_p1)) {
                ESP_LOGI("KEY_SCAN", "P0 0x%02X->0x%02X changed=0x%02X; P1 0x%02X->0x%02X changed=0x%02X",
                         previous_p0, p0, previous_p0 ^ p0,
                         previous_p1, pins, previous_p1 ^ pins);
            }
            previous_p0 = p0;
            previous_p1 = pins;
            have_snapshot = true;
            uint8_t now = pins & KEY_MASK;
            if (now == previous) { if (count < 3) ++count; }
            else { previous = now; count = 1; }
            if (count == 3 && now != stable) {
                uint8_t pressed = stable & (uint8_t)~now;
                stable = now;
                int volume_to_save = -1;
                portENTER_CRITICAL(&controls_lock);
                if (pressed & 0x02) { requested_volume = requested_volume >= 5 ? requested_volume - 5 : 0; volume_changed = true; volume_to_save = requested_volume; }
                if (pressed & 0x04) { playback_enabled = true; playback_paused = !playback_paused; }
                if (pressed & 0x08) { requested_volume = requested_volume <= 95 ? requested_volume + 5 : 100; volume_changed = true; volume_to_save = requested_volume; }
                bool paused = playback_paused;
                int vol = requested_volume;
                portEXIT_CRITICAL(&controls_lock);
                if (pressed & 0x02) ESP_LOGI(TAG, "KEY1 volume down: %d%%", vol);
                if (pressed & 0x04) ESP_LOGI(TAG, "KEY2 %s", paused ? "PAUSE" : "RESUME");
                if (pressed & 0x08) ESP_LOGI(TAG, "KEY3 volume up: %d%%", vol);
                state_dirty = true;
                if (volume_to_save >= 0) {
                    nvs_handle_t h;
                    if (nvs_open("mishka", NVS_READWRITE, &h) == ESP_OK) {
                        nvs_set_i32(h, "volume", volume_to_save);
                        nvs_commit(h);
                        nvs_close(h);
                    }
                }
            }
        }
        /* BOOT GPIO0 is read independently of the TCA9555 I2C status. */
        int boot_now = gpio_get_level(MIC_BOOT_BUTTON);
        if (boot_now == boot_previous) {
            if (boot_count < 3) ++boot_count;
        } else {
            boot_previous = boot_now;
            boot_count = 1;
        }
        if (boot_count == 3 && boot_now != boot_stable) {
            boot_stable = boot_now;
            if (boot_now == 0) {
                ESP_LOGI("KEY_SCAN", "BOOT GPIO0 pressed");
                set_microphone_muted(!microphone_muted, "BOOT GPIO0");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

/* Assist owns the existing playback worker; never creates a second I2S writer. */
static volatile bool assist_radio_resume;
static volatile bool assist_session_in_progress;
static volatile bool assist_tts_pending;
static volatile bool assist_tts_active;
static char assist_audio_url[1024];
void mishka_assist_on_start(void) {
    portENTER_CRITICAL(&controls_lock);
    assist_radio_resume = playback_enabled && !playback_paused;
    assist_session_in_progress = true;
    playback_paused = true;
    playback_stop_requested = true; /* release radio's decoder and I2S owner */
    portEXIT_CRITICAL(&controls_lock);
    state_dirty = true;
}
void mishka_assist_on_finish(void) {
    char url[sizeof(assist_audio_url)];
    bool has_audio = mishka_assist_resolve_tts_url(url, sizeof(url));
    portENTER_CRITICAL(&controls_lock);
    assist_session_in_progress = false;
    if (has_audio) {
        memcpy(assist_audio_url, url, sizeof(assist_audio_url));
        assist_tts_pending = true;
    } else {
        if (assist_radio_resume && playback_enabled) playback_paused = false;
        assist_radio_resume = false;
    }
    portEXIT_CRITICAL(&controls_lock);
    state_dirty = true;
}
static void apply_volume(esp_codec_dev_handle_t dev, int volume, bool changed) {
    if (changed) {
        int rc = esp_codec_dev_set_out_vol(dev, volume);
        if (rc != 0) ESP_LOGW(TAG, "Volume change failed: %d", rc);
    }
}


#define MP3_STREAM_BUFFER 16384U
#define MP3_DECODE_CHUNK 2048U

// Parse a complete MPEG Layer III frame before handing it to the decoder.
// Reject false sync bytes in HTTP metadata / non-MP3 streams.
static size_t mp3_frame_length(const uint8_t *p, size_t n, int *rate, int *channels) {
    if (n < 4 || p[0] != 0xff || (p[1] & 0xe0) != 0xe0) return 0;
    unsigned version = (p[1] >> 3) & 3, layer = (p[1] >> 1) & 3;
    unsigned br = (p[2] >> 4) & 15, sr = (p[2] >> 2) & 3;
    if (version == 1 || layer != 1 || br == 0 || br == 15 || sr == 3) return 0;
    static const int rates[] = {44100,48000,32000};
    static const int br_mpeg1[] = {0,32,40,48,56,64,80,96,112,128,160,192,224,256,320};
    static const int br_mpeg2[] = {0,8,16,24,32,40,48,56,64,80,96,112,128,144,160};
    int hz = rates[sr] / (version == 3 ? 1 : version == 2 ? 2 : 4);
    int kbps = version == 3 ? br_mpeg1[br] : br_mpeg2[br];
    size_t bytes = (size_t)((version == 3 ? 144000 : 72000) * kbps / hz + ((p[2] >> 1) & 1));
    if (bytes < 24 || bytes > 2048) return 0;
    if (rate) *rate = hz;
    if (channels) *channels = ((p[3] >> 6) & 3) == 3 ? 1 : 2;
    return bytes;
}

static void drop_encoded(uint8_t *buf, size_t *buffered, size_t n, uint64_t *consumed) {
    if (n > *buffered) n = *buffered;
    *buffered -= n;
    *consumed += n;
    if (*buffered) memmove(buf, buf + n, *buffered);
}


static bool audio_format(esp_codec_dev_handle_t dev, int rate, int channels) {
    if (rate == speaker_rate && speaker_channels == 1) return true;
    if (rate < 8000 || rate > 48000 || (channels != 1 && channels != 2)) return false;
    // Only the playback worker uses the codec and TX channel.
    if (esp_codec_dev_close(dev) != 0) return false;
    // esp_codec_dev_close() already disables the I2S TX channel on this board.
    // A second disable returned ESP_ERR_INVALID_STATE and rejected all MP3 rates.
    // esp_codec_dev_close() has already disabled TX; avoid redundant disable.
    // ESP_ERR_INVALID_STATE was logged on every valid MP3 rate switch.
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(rate);
    i2s_std_slot_config_t slot = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
    if (i2s_channel_reconfig_std_clock(speaker_tx, &clk) != ESP_OK ||
        i2s_channel_reconfig_std_slot(speaker_tx, &slot) != ESP_OK ||
        i2s_channel_enable(speaker_tx) != ESP_OK) return false;
    esp_codec_dev_sample_info_t info = {.sample_rate=rate,.channel=1,.bits_per_sample=16};
    if (esp_codec_dev_open(dev, &info) != 0) return false;
    speaker_rate = rate;
    speaker_channels = 1;
    esp_codec_dev_set_out_vol(dev, requested_volume);
    ESP_LOGI(TAG, "Audio switched to %d Hz, %d channel(s)", rate, channels);
    return true;
}

// Radio servers may depend on a browser-like Accept/User-Agent and on
// ICY metadata being disabled. Log only safe response headers (never URLs/tokens).
static esp_err_t radio_http_event(esp_http_client_event_t *evt) {
    if (evt->event_id != HTTP_EVENT_ON_HEADER || !evt->header_key || !evt->header_value)
        return ESP_OK;
    const char *key = evt->header_key;
    if (!strcasecmp(key, "Content-Type") || !strcasecmp(key, "Content-Length") ||
        !strcasecmp(key, "Transfer-Encoding") || !strcasecmp(key, "Content-Encoding") ||
        !strcasecmp(key, "icy-metaint") || !strcasecmp(key, "icy-br") ||
        !strcasecmp(key, "icy-name") || !strcasecmp(key, "Connection")) {
        ESP_LOGI(TAG, "Radio HTTP header: %s: %.100s", key, evt->header_value);
    }
    return ESP_OK;
}
static unsigned radio_retry_profile = 0; // only written by playback task
static void radio_http_request_headers(esp_http_client_handle_t http) {
    esp_http_client_set_header(http, "User-Agent", "Mishka-WiFi-Speaker/0.5.7.9");
    esp_http_client_set_header(http, "Accept", "audio/aac, audio/aacp, audio/mpeg, audio/*;q=0.9, */*;q=0.5");
    esp_http_client_set_header(http, "Accept-Encoding", "identity");
    esp_http_client_set_header(http, "Icy-MetaData", "0");
    // First request keeps the known-good behavior. On a failed stream only,
    // force close semantics and disable intermediate caching. Some radio
    // gateways mishandle long-lived HTTP/1.1 keep-alive audio responses.
    if (radio_retry_profile >= 1) {
        esp_http_client_set_header(http, "Connection", "close");
        esp_http_client_set_header(http, "Cache-Control", "no-cache");
        ESP_LOGW(TAG, "Radio HTTP retry profile=%u: Connection: close; Cache-Control: no-cache",
                 radio_retry_profile);
    }
}

// esp_http_client_open() does not automatically follow HTTP redirects. The
// perform() API does, but this player uses open/read for streaming. Follow
// Location explicitly without buffering a potentially endless radio stream.
#define RADIO_MAX_REDIRECTS 5
static bool http_open_follow_redirects(esp_http_client_handle_t http,
                                       const char *purpose,
                                       int64_t *content_length,
                                       int *status_code) {
    for (int hop = 0; hop <= RADIO_MAX_REDIRECTS; ++hop) {
        esp_err_t err = esp_http_client_open(http, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s HTTP open failed: %s (redirect hop %d)",
                     purpose, esp_err_to_name(err), hop);
            return false;
        }
        int64_t len = esp_http_client_fetch_headers(http);
        int status = esp_http_client_get_status_code(http);
        if (status == 301 || status == 302 || status == 303 ||
            status == 307 || status == 308) {
            if (hop == RADIO_MAX_REDIRECTS) {
                ESP_LOGE(TAG, "%s redirect limit (%d) exceeded", purpose,
                         RADIO_MAX_REDIRECTS);
                esp_http_client_close(http);
                return false;
            }
            // set_redirection() reads the Location header parsed by
            // fetch_headers(). It also resolves relative redirect targets.
            err = esp_http_client_set_redirection(http);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "%s HTTP %d without usable Location: %s",
                         purpose, status, esp_err_to_name(err));
                esp_http_client_close(http);
                return false;
            }
            ESP_LOGI(TAG, "%s HTTP redirect %d (hop %d/%d)", purpose,
                     status, hop + 1, RADIO_MAX_REDIRECTS);
            esp_http_client_close(http);
            continue;
        }
        if (content_length) *content_length = len;
        if (status_code) *status_code = status;
        return true;
    }
    return false;
}

// Identify the actual HTTP payload rather than guessing from the URL suffix.
// The probe connection is closed before the selected player opens its own stream.
typedef enum { AUDIO_PROBE_UNKNOWN, AUDIO_PROBE_MP3, AUDIO_PROBE_WAV,
               AUDIO_PROBE_AAC, AUDIO_PROBE_OGG, AUDIO_PROBE_FLAC,
               AUDIO_PROBE_PLAYLIST, AUDIO_PROBE_HLS } audio_probe_t;

static audio_probe_t sniff_audio(const uint8_t *sig, int n) {
    if (n >= 12 && !memcmp(sig, "RIFF", 4) && !memcmp(sig + 8, "WAVE", 4)) return AUDIO_PROBE_WAV;
    if (n >= 4 && !memcmp(sig, "fLaC", 4)) return AUDIO_PROBE_FLAC;
    if (n >= 4 && !memcmp(sig, "OggS", 4)) return AUDIO_PROBE_OGG;
    if (n >= 3 && !memcmp(sig, "ID3", 3)) return AUDIO_PROBE_MP3;
    if (n >= 7 && !memcmp(sig, "#EXTM3U", 7)) return AUDIO_PROBE_PLAYLIST;
    if (n >= 10 && !memcmp(sig, "[playlist]", 10)) return AUDIO_PROBE_PLAYLIST;
    if (n >= 2 && sig[0] == 0xff && (sig[1] & 0xf6) == 0xf0) return AUDIO_PROBE_AAC; // ADTS
    if (n >= 4 && sig[0] == 0xff && (sig[1] & 0xe0) == 0xe0 &&
        ((sig[1] >> 1) & 3) == 1) return AUDIO_PROBE_MP3; // MPEG Layer III
    return AUDIO_PROBE_UNKNOWN;
}

static const char *audio_probe_name(audio_probe_t kind) {
    switch (kind) {
        case AUDIO_PROBE_MP3: return "MP3";
        case AUDIO_PROBE_WAV: return "WAV";
        case AUDIO_PROBE_AAC: return "AAC/ADTS";
        case AUDIO_PROBE_OGG: return "OGG (Vorbis/Opus)";
        case AUDIO_PROBE_FLAC: return "FLAC";
        case AUDIO_PROBE_PLAYLIST: return "M3U/PLS playlist";
        case AUDIO_PROBE_HLS: return "HLS playlist";
        default: return "unknown";
    }
}

// Probe does not consume the playback connection. This avoids feeding AAC,
// OGG, FLAC or playlist bytes into the MPEG Layer III decoder.
// Keep the successful probe HTTP connection open and hand its first bytes
// to the player. MP3 therefore avoids a second DNS/TLS handshake.
typedef struct {
    esp_http_client_handle_t http;
    audio_probe_t kind;
    uint8_t first[32];
    size_t first_len;
    int64_t content_length;
    int status;
    bool chunked;
    bool header_chunked;
    bool header_identity;
    uint32_t header_count;
} audio_source_t;

static void source_close(audio_source_t *src) {
    if (src->http) {
        esp_http_client_close(src->http);
        esp_http_client_cleanup(src->http);
        src->http = NULL;
    }
}

static bool source_open(const char *url, audio_source_t *src) {
    memset(src, 0, sizeof(*src));
    esp_http_client_config_t cfg = {.url=url, .timeout_ms=1500,
        .buffer_size=4096, .buffer_size_tx=2048, .crt_bundle_attach=esp_crt_bundle_attach,
        .event_handler=radio_http_event};
    src->http = esp_http_client_init(&cfg);
    if (!src->http) return false;
    radio_http_request_headers(src->http);
    if (!http_open_follow_redirects(src->http, "Audio", &src->content_length, &src->status) || src->status != 200) {
        ESP_LOGE(TAG, "Audio source HTTP status=%d", src->status);
        source_close(src);
        return false;
    }
    src->chunked = esp_http_client_is_chunked_response(src->http);
    ESP_LOGI(TAG, "HTTP transfer diagnosis: parser_chunked=%d content_length=%lld; "
             "ESP-IDF owns HTTP chunk decoding; decoder receives payload only",
             (int)src->chunked, (long long)src->content_length);
    ESP_LOGI(TAG, "HTTP framing detected by ESP-IDF: chunked=%d status=%d",
             (int)src->chunked, src->status);
    ESP_LOGI(TAG, "Audio HTTP framing: %s; Content-Length=%lld",
             src->chunked ? "chunked" : "identity", (long long)src->content_length);
    int n = esp_http_client_read(src->http, (char *)src->first, sizeof(src->first));
    if (n <= 0) {
        ESP_LOGE(TAG, "Audio probe read failed: %d", n);
        source_close(src);
        return false;
    }
    src->first_len = (size_t)n;
    src->kind = sniff_audio(src->first, n);
    ESP_LOGI(TAG, "Audio signature: %s (%d bytes), reused HTTP connection",
             audio_probe_name(src->kind), n);
    return true;
}

/* HA audio URLs are restricted to the configured HA origin. No auth redirects. */
static bool source_open_assist(const char *url, audio_source_t *src) {
    memset(src, 0, sizeof(*src));
    esp_http_client_config_t cfg = {
        .url = url, .timeout_ms = 12000, .buffer_size = 4096,
        .crt_bundle_attach = esp_crt_bundle_attach, .disable_auto_redirect = true
    };
    src->http = esp_http_client_init(&cfg);
    if (!src->http) return false;
    char authorization[300];
    if (!mishka_assist_authorization(authorization, sizeof(authorization))) {
        source_close(src); return false;
    }
    esp_http_client_set_header(src->http, "Authorization", authorization);
    memset(authorization, 0, sizeof(authorization));
    esp_http_client_set_header(src->http, "Accept-Encoding", "identity");
    if (esp_http_client_open(src->http, 0) != ESP_OK) {
        source_close(src); return false;
    }
    src->content_length = esp_http_client_fetch_headers(src->http);
    src->status = esp_http_client_get_status_code(src->http);
    if (src->status != 200) {
        ESP_LOGE("MISHKA_ASSIST", "TTS HTTP status=%d", src->status);
        source_close(src); return false;
    }
    int n = esp_http_client_read(src->http, (char *)src->first, sizeof(src->first));
    if (n <= 0) { source_close(src); return false; }
    src->first_len = (size_t)n;
    src->kind = sniff_audio(src->first, n);
    ESP_LOGI("MISHKA_ASSIST", "TTS audio format=%s", audio_probe_name(src->kind));
    return true;
}

// Print the status and a small sanitized HTTP error body. Never log signed URLs.
static void __attribute__((unused)) log_http_error(esp_http_client_handle_t http, const char *context, int status) {
    ESP_LOGE(TAG, "%s HTTP status=%d", context, status);
    char body[193] = {0};
    int n = esp_http_client_read(http, body, sizeof(body)-1);
    if (n > 0) {
        body[n] = 0;
        // Diagnostic body may contain HTML. Replace control characters.
        for (int i=0; i<n; ++i) if ((unsigned char)body[i]<32) body[i]=' ';
        ESP_LOGW(TAG, "%s response body (first %d bytes): %.192s", context, n, body);
    }
}

// Network producer and decoder are independent: HTTP stalls cannot block the
// decoder while compressed data remains buffered in PSRAM.
#define MP3_NETWORK_BUFFER (256U * 1024U)
#define MP3_PREBUFFER (24U * 1024U)
#define MP3_REBUFFER (64U * 1024U)
#define MP3_REBUFFER_TIMEOUT_MS 30000
#define MP3_LOW_WATERMARK (12U * 1024U)
// AAC uses the same tested HTTP producer and buffering policy as MP3.
#define AAC_STREAM_BUFFER MP3_STREAM_BUFFER
#define AAC_NETWORK_BUFFER MP3_NETWORK_BUFFER
#define AAC_PREBUFFER (8U * 1024U)
#define AAC_REBUFFER (16U * 1024U)
#define AAC_REBUFFER_TIMEOUT_MS MP3_REBUFFER_TIMEOUT_MS
#define AAC_LOW_WATERMARK (4U * 1024U)
typedef struct {
    esp_http_client_handle_t http;
    StreamBufferHandle_t stream;
    volatile bool cancel;
    volatile bool finished;
    volatile bool failed;
    volatile uint64_t received;
    volatile uint32_t last_data_tick;
    volatile uint32_t read_zero_count;
    volatile int last_read_error;
    volatile uint32_t read_calls;
    volatile uint32_t max_read_ms;
    volatile uint32_t transient_errors;
    volatile uint32_t last_errno;
    volatile uint32_t short_reads;
    volatile uint32_t successful_reads;
    volatile uint32_t read_timeouts;
    volatile uint32_t max_queued;
    volatile uint32_t last_report_tick;
    volatile uint64_t last_report_received;
    int64_t content_length;
    bool chunked;
    const char *codec_name;
    TaskHandle_t task;
} mp3_network_t;

static void audio_network_task(void *arg) {
    mp3_network_t *net = arg;
    // Short reads avoid waiting for a full 4 KB on low-bitrate/chunked stations.
    const int read_size = net->chunked ? 256 : 2048;
    uint8_t *block = malloc(2048);
    if (!block) { net->failed = true; net->finished = true; vTaskDelete(NULL); return; }
    uint32_t consecutive_zero_reads = 0;
    net->last_report_tick = (uint32_t)xTaskGetTickCount();
    net->last_report_received = net->received;
    while (!net->cancel) {
        // Never block indefinitely when the consumer pauses or switches tracks.
        if (xStreamBufferSpacesAvailable(net->stream) < (size_t)read_size) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        TickType_t read_start = xTaskGetTickCount();
        // ESP-IDF removes chunk framing internally. A small read prevents
        // waiting for a full 1024-byte block when a low-bitrate station
        // delivers a short final piece of a chunk.
        errno = 0;
        int n = esp_http_client_read(net->http, (char *)block, read_size);
        int socket_errno = errno;
        uint32_t elapsed_ms = pdTICKS_TO_MS(xTaskGetTickCount() - read_start);
        ++net->read_calls;
        if (elapsed_ms > net->max_read_ms) net->max_read_ms = elapsed_ms;
        if (elapsed_ms >= 1000) {
            ESP_LOGW(TAG, "%s HTTP read latency=%lu ms, bytes=%d, queued=%u, total=%llu",
                     net->codec_name, (unsigned long)elapsed_ms, n,
                     (unsigned)xStreamBufferBytesAvailable(net->stream),
                     (unsigned long long)net->received);
        }
        if (net->cancel) break;
        if (n <= 0) {
            ESP_LOGW(TAG, "%s HTTP read result=%d errno=%d complete=%d chunked=%d idle=%lu ms",
                     net->codec_name, n, socket_errno,
                     (int)esp_http_client_is_complete_data_received(net->http),
                     (int)net->chunked,
                     (unsigned long)pdTICKS_TO_MS(xTaskGetTickCount() - (TickType_t)net->last_data_tick));
            ESP_LOGW(TAG, "%s transport state: http_complete=%d http_chunked=%d "
                     "success=%lu short=%lu timeout=%lu max_read=%lu ms",
                     net->codec_name,
                     (int)esp_http_client_is_complete_data_received(net->http),
                     (int)esp_http_client_is_chunked_response(net->http),
                     (unsigned long)net->successful_reads,
                     (unsigned long)net->short_reads,
                     (unsigned long)net->read_timeouts,
                     (unsigned long)net->max_read_ms);
        }
        // 10-second rolling throughput and ring occupancy; detect slow servers
        // without misdiagnosing an AAC decoder failure.
        uint32_t now_tick = (uint32_t)xTaskGetTickCount();
        uint32_t window_ms = pdTICKS_TO_MS(now_tick - net->last_report_tick);
        if (window_ms >= 10000) {
            uint64_t delta = net->received - net->last_report_received;
            uint32_t bps = (uint32_t)((delta * 8000ULL) / window_ms);
            ESP_LOGI(TAG, "%s network: bitrate=%lu bit/s in %lu ms, queued=%u, peak_queued=%lu, timeouts=%lu",
                     net->codec_name, (unsigned long)bps, (unsigned long)window_ms,
                     (unsigned)xStreamBufferBytesAvailable(net->stream),
                     (unsigned long)net->max_queued, (unsigned long)net->transient_errors);
            net->last_report_tick = now_tick;
            net->last_report_received = net->received;
            net->max_queued = 0;
        }
        if (n > 0) {
            ++net->successful_reads;
            if (n < read_size) {
                ++net->short_reads;
                if (net->short_reads <= 4 || (net->short_reads % 50) == 0)
                    ESP_LOGI(TAG, "%s HTTP short read #%lu: got=%d requested=%d total_before=%llu",
                             net->codec_name, (unsigned long)net->short_reads,
                             n, read_size, (unsigned long long)net->received);
            }
            consecutive_zero_reads = 0;
            size_t sent = xStreamBufferSend(net->stream, block, (size_t)n, pdMS_TO_TICKS(100));
            if (sent != (size_t)n) { net->failed = true; break; }
            net->received += (uint32_t)n;
            uint32_t occupancy = (uint32_t)xStreamBufferBytesAvailable(net->stream);
            if (occupancy > net->max_queued) net->max_queued = occupancy;
            net->last_data_tick = (uint32_t)xTaskGetTickCount();
        } else if (n == 0) {
            ++net->read_zero_count;
            ++consecutive_zero_reads;
            if (net->content_length <= 0 && !esp_http_client_is_complete_data_received(net->http)) {
                // A short zero read is not EOF for a live radio stream.
                // A permanently stalled socket must eventually trigger reconnect.
                uint32_t idle_ms = pdTICKS_TO_MS(xTaskGetTickCount() - (TickType_t)net->last_data_tick);
                if (idle_ms >= 9000) {
                    ESP_LOGW(TAG, "%s live HTTP stalled: idle=%lu ms, zero_reads=%lu; reconnect required",
                             net->codec_name, (unsigned long)idle_ms,
                             (unsigned long)consecutive_zero_reads);
                    net->failed = true;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(80));
                continue;
            }
            if (net->content_length > 0 && net->received < (uint64_t)net->content_length) {
                ESP_LOGE(TAG, "%s HTTP truncated: %llu/%lld bytes",
                         net->codec_name,
                         (unsigned long long)net->received, (long long)net->content_length);
                net->failed = true;
            }
            break;
        } else {
            // esp_http_client_read() can return ESP_ERR_HTTP_EAGAIN (-28679)
            // on a socket/TLS timeout. This is not necessarily a dead stream.
            // Keep the decoder running on already queued data and retry until
            // the measured no-data interval exceeds the live-stream budget.
            if (n == -28679) {
                ++net->read_timeouts;
                ++net->transient_errors;
                uint32_t idle_ms = pdTICKS_TO_MS(xTaskGetTickCount() - (TickType_t)net->last_data_tick);
                if (net->transient_errors <= 3 || (net->transient_errors % 10) == 0)
                    ESP_LOGW(TAG, "%s temporary HTTP EAGAIN #%lu: idle=%lu ms queued=%u; retrying",
                             net->codec_name, (unsigned long)net->transient_errors,
                             (unsigned long)idle_ms, (unsigned)xStreamBufferBytesAvailable(net->stream));
                if (idle_ms < 9000) {
                    vTaskDelay(pdMS_TO_TICKS(80));
                    continue;
                }
                ESP_LOGE(TAG, "%s HTTP idle exceeded 9000 ms after EAGAIN; reconnect required",
                         net->codec_name);
            }
            net->last_read_error = n;
            ESP_LOGE(TAG, "%s network read error=%d, queued=%u, received=%llu, read_calls=%lu; reconnect required",
                     net->codec_name, n,
                     (unsigned)xStreamBufferBytesAvailable(net->stream),
                     (unsigned long long)net->received, (unsigned long)net->read_calls);
            net->failed = true;
            break;
        }
        // HTTP producer must fill PSRAM promptly; do not throttle each 4 KB read.
        taskYIELD();
    }
    free(block);
    net->finished = true;
    // Task handle cleared by consumer after waiting on finished; no task-owned heap.
    vTaskDelete(NULL);
}

static bool play_mp3(esp_codec_dev_handle_t dev, audio_source_t *src) {
    // Incremental HTTP -> bounded compressed buffer -> MP3 decoder -> I2S.
    // No Content-Length requirement, no full-file PSRAM allocation.
    esp_http_client_handle_t http = src->http;
    src->http = NULL; // playback now owns the open, already probed connection
    if (!http) return false;
    uint8_t *encoded = NULL, *pcm = NULL;
    esp_audio_simple_dec_handle_t decoder = NULL;
    bool ok = false, eof = false;
    mp3_network_t net = {0};
    size_t buffered = 0;
    uint64_t consumed_total = 0;
    uint32_t rebuffer_events = 0;
    uint32_t last_stats_tick = 0;
    bool rebuffering = true;
    TickType_t rebuffer_start = 0;
    int locked_rate = 0, locked_channels = 0;
    uint32_t rejected_format_changes = 0;
    uint32_t resync_bytes = 0, frames_decoded = 0, bad_frames = 0;
    bool frame_locked = false;
    int64_t content_length = src->content_length;
    ESP_LOGI(TAG, "MP3 streaming on probe connection: HTTP 200, Content-Length=%lld",
             (long long)content_length);
    encoded = heap_caps_malloc(MP3_STREAM_BUFFER, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    pcm = heap_caps_malloc(32768, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!encoded || !pcm) { ESP_LOGE(TAG, "MP3 stream buffer allocation failed"); goto cleanup; }
    // Keep switching responsive even if TLS radio server stops sending data.
    // ESP-IDF HTTP client handles chunked transfer framing internally.
    esp_http_client_set_timeout_ms(http, src->chunked ? 1500 : 1000);
    net.http = http;
    net.chunked = src->chunked;
    net.codec_name = "MP3";
    net.content_length = content_length;
    net.last_data_tick = (uint32_t)xTaskGetTickCount();
    ESP_LOGI(TAG, "%s HTTP producer: framing=%s, read=%d, timeout=%d ms",
             net.codec_name, net.chunked ? "chunked" : "identity",
             net.chunked ? 256 : 2048, net.chunked ? 1500 : 1000);
    net.stream = xStreamBufferCreateWithCaps(MP3_NETWORK_BUFFER, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!net.stream) { ESP_LOGE(TAG, "MP3 PSRAM ring allocation failed"); goto cleanup; }
    // Preserve all bytes consumed during format detection, in original order.
    if (src->first_len) {
        size_t sent = xStreamBufferSend(net.stream, src->first, src->first_len, 0);
        if (sent != src->first_len) { ESP_LOGE(TAG, "Probe prefix enqueue failed"); goto cleanup; }
        net.received = src->first_len;
    }
    esp_audio_simple_dec_cfg_t dc = {.dec_type=ESP_AUDIO_SIMPLE_DEC_TYPE_MP3, .use_frame_dec=true};
    if (esp_audio_simple_dec_open(&dc, &decoder) != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(TAG, "MP3 decoder open failed"); goto cleanup;
    }
    if (xTaskCreate(audio_network_task, "mishka_http", 6144, &net, 5, &net.task) != pdPASS) {
        ESP_LOGE(TAG, "MP3 network task creation failed"); goto cleanup;
    }
    ESP_LOGI(TAG, "MP3 PSRAM network buffer=%u KB, initial prebuffer=%u KB",
             (unsigned)(MP3_NETWORK_BUFFER / 1024), (unsigned)(MP3_PREBUFFER / 1024));
    portENTER_CRITICAL(&controls_lock);
    playback_active = true;
    playback_stop_requested = false;
    portEXIT_CRITICAL(&controls_lock);
    state_dirty = true;
    ok = true;
    for (;;) {
        bool stop, paused, changed;
        int volume;
        portENTER_CRITICAL(&controls_lock);
        stop = playback_stop_requested || !playback_enabled || new_url_pending;
        portEXIT_CRITICAL(&controls_lock);
        if (stop) { ok = false; break; }
        controls_snapshot(&paused, &volume, &changed);
        if (state_dirty) publish_state();
        apply_volume(dev, volume, changed);
        if (paused) {
            // Keep DMA supplied with silence. An HTTP server may time out during a long pause.
            write_silence(dev, 32);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        // Refill the compressed input buffer independently from the network task.
        // If a live station stalls, hold decoding until enough bytes accumulate.
        if (!eof && buffered <= MP3_STREAM_BUFFER - 4096) {
            size_t want = MP3_STREAM_BUFFER - buffered;
            if (want > 4096) want = 4096;
            size_t n = xStreamBufferReceive(net.stream, encoded + buffered, want, pdMS_TO_TICKS(10));
            if (n > 0) buffered += n;
            else if (net.finished && xStreamBufferBytesAvailable(net.stream) == 0) {
                eof = true;
                if (net.failed) { ok = false; break; }
            }
        }
        size_t queued = xStreamBufferBytesAvailable(net.stream);
        if (rebuffering) {
            size_t threshold = consumed_total == 0 ? MP3_PREBUFFER : MP3_REBUFFER;
            // A short finite file must not wait forever for 48 KB.
            if (!net.finished && buffered + queued < threshold) {
                if (rebuffer_start == 0) rebuffer_start = xTaskGetTickCount();
                if (xTaskGetTickCount() - rebuffer_start > pdMS_TO_TICKS(MP3_REBUFFER_TIMEOUT_MS)) {
                    ESP_LOGE(TAG, "MP3 rebuffer timeout: buffered=%u queued=%u received=%llu read_zeros=%lu",
                             (unsigned)buffered, (unsigned)queued,
                             (unsigned long long)net.received, (unsigned long)net.read_zero_count);
                    ok = false; break;
                }
                write_silence(dev, 20);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            if (consumed_total > 0) {
                ESP_LOGI(TAG, "MP3 rebuffer recovered #%lu: buffered=%u queued=%u",
                         (unsigned long)rebuffer_events, (unsigned)buffered, (unsigned)queued);
            }
            rebuffering = false;
            rebuffer_start = 0;
        }
        // Rebuffer before the entire compressed pipeline drains.
        // Do not enter this state while the producer has already finished.
        if (!eof && !net.finished && buffered + queued < MP3_LOW_WATERMARK) {
            ++rebuffer_events;
            rebuffering = true;
            rebuffer_start = xTaskGetTickCount();
            ESP_LOGW(TAG, "MP3 underrun -> rebuffer #%lu: received=%llu consumed=%llu queued=%u, read_zeros=%lu",
                     (unsigned long)rebuffer_events, (unsigned long long)net.received,
                     (unsigned long long)consumed_total, (unsigned)queued,
                     (unsigned long)net.read_zero_count);
            continue;
        }
        if (xTaskGetTickCount() - last_stats_tick >= pdMS_TO_TICKS(10000)) {
            last_stats_tick = xTaskGetTickCount();
            ESP_LOGI(TAG, "MP3 stats: received=%llu consumed=%llu buffered=%u queued=%u rebuffer=%lu read_zeros=%lu",
                     (unsigned long long)net.received, (unsigned long long)consumed_total,
                     (unsigned)buffered, (unsigned)queued, (unsigned long)rebuffer_events,
                     (unsigned long)net.read_zero_count);
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // yield to IDLE0 and avoid watchdog starvation
        if (buffered == 0 && eof) break;
        if (buffered == 0) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
        // Strip ID3v2 metadata at the beginning of an MP3 file.
        if (!frame_locked && buffered >= 10 && !memcmp(encoded, "ID3", 3)) {
            size_t id3_size = 10U + (((size_t)encoded[6] & 127) << 21)
                + (((size_t)encoded[7] & 127) << 14)
                + (((size_t)encoded[8] & 127) << 7) + ((size_t)encoded[9] & 127);
            if (encoded[5] & 0x10) id3_size += 10; // ID3 footer
            if (id3_size > MP3_STREAM_BUFFER) {
                ESP_LOGE(TAG, "ID3 tag too large (%u bytes)", (unsigned)id3_size);
                ok = false; break;
            }
            if (buffered < id3_size) continue;
            ESP_LOGI(TAG, "Skipping ID3 metadata: %u bytes", (unsigned)id3_size);
            drop_encoded(encoded, &buffered, id3_size, &consumed_total);
            continue;
        }
        if (buffered < 4) {
            if (eof) break;
            continue;
        }
        int header_rate = 0, header_channels = 0;
        size_t frame_bytes = mp3_frame_length(encoded, buffered, &header_rate, &header_channels);
        if (!frame_bytes || (locked_rate &&
            (header_rate != locked_rate || header_channels != locked_channels))) {
            // Scan at most 1024 bytes per iteration, then shift once. The
            // previous byte-by-byte memmove starved IDLE0 and tripped WDT.
            size_t skip = 1;
            size_t scan_limit = buffered >= 4 ? buffered - 3 : 0;
            if (scan_limit > 1024) scan_limit = 1024;
            for (; skip < scan_limit; ++skip) {
                int candidate_rate = 0, candidate_channels = 0;
                if (mp3_frame_length(encoded + skip, buffered - skip,
                                     &candidate_rate, &candidate_channels) &&
                    (!locked_rate || (candidate_rate == locked_rate &&
                                      candidate_channels == locked_channels))) break;
            }
            drop_encoded(encoded, &buffered, skip, &consumed_total);
            resync_bytes += (uint32_t)skip;
            if (resync_bytes <= 5 || (resync_bytes / 4096) != ((resync_bytes - skip) / 4096))
                ESP_LOGW(TAG, "MP3 frame resync: skipped %lu byte(s)", (unsigned long)resync_bytes);
            if (resync_bytes > 8192 && frames_decoded == 0) {
                ESP_LOGE(TAG, "No MP3 frame in first 8 KB: station may use AAC/OGG/playlist or incompatible MPEG stream");
                ok = false; break;
            }
            if (resync_bytes > 65536) {
                ESP_LOGE(TAG, "MP3 frame synchronization lost beyond 64 KB");
                ok = false; break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        if (buffered < frame_bytes) {
            if (eof) { ESP_LOGW(TAG, "Truncated final MP3 frame"); break; }
            continue;
        }
        // Confirm the next frame header before locking to an arbitrary sync word.
        if (!frame_locked && !eof && buffered < frame_bytes + 4) continue;
        if (!frame_locked && buffered >= frame_bytes + 4 &&
            !mp3_frame_length(encoded + frame_bytes, buffered - frame_bytes, NULL, NULL)) {
            drop_encoded(encoded, &buffered, 1, &consumed_total);
            ++resync_bytes;
            if (resync_bytes > 8192 && frames_decoded == 0) {
                ESP_LOGE(TAG, "No consecutive MP3 frames in first 8 KB; unsupported stream format");
                ok = false; break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        frame_locked = true;
        esp_audio_simple_dec_raw_t raw = {
            .buffer = encoded, .len = (uint32_t)frame_bytes,
            .eos = eof && frame_bytes == buffered
        };
        esp_audio_simple_dec_out_t out = {.buffer=pcm, .len=32768};
        esp_audio_err_t err = esp_audio_simple_dec_process(decoder, &raw, &out);
        if (err != ESP_AUDIO_ERR_OK) {
            ++bad_frames;
            ESP_LOGW(TAG, "MP3 frame decode error=%d, frame=%u, bad=%lu",
                     (int)err, (unsigned)frame_bytes, (unsigned long)bad_frames);
            if (bad_frames > 30) { ok = false; break; }
            drop_encoded(encoded, &buffered, frame_bytes, &consumed_total);
            continue;
        }
        if (raw.consumed > frame_bytes) { ok = false; break; }
        if (raw.consumed == 0 && out.decoded_size == 0) {
            ESP_LOGE(TAG, "Frame decoder made no progress on a complete %u-byte frame", (unsigned)frame_bytes);
            ok = false; break;
        }
        ++frames_decoded;
        if (out.decoded_size) {
            esp_audio_simple_dec_info_t info = {0};
            if (esp_audio_simple_dec_get_info(decoder, &info) != ESP_AUDIO_ERR_OK ||
                info.bits_per_sample != 16 || info.sample_rate < 8000 ||
                info.sample_rate > 48000 || (info.channel != 1 && info.channel != 2)) {
                ESP_LOGE(TAG, "MP3 decoder returned invalid format: rate=%lu channels=%u bits=%u",
                         (unsigned long)info.sample_rate, (unsigned)info.channel, (unsigned)info.bits_per_sample);
                ok = false; break;
            }
            // MP3 radio streams should not oscillate between unrelated sample rates.
            // A corrupt frame must not repeatedly close/reopen the I2S codec.
            if (locked_rate && ((int)info.sample_rate != locked_rate || (int)info.channel != locked_channels)) {
                ++rejected_format_changes;
                ESP_LOGW(TAG, "MP3 unexpected format change #%lu: %d/%d -> %lu/%u; skipping frame",
                         (unsigned long)rejected_format_changes, locked_rate, locked_channels,
                         (unsigned long)info.sample_rate, (unsigned)info.channel);
                out.decoded_size = 0;
            } else if (!audio_format(dev, (int)info.sample_rate, (int)info.channel)) {
                ESP_LOGE(TAG, "MP3 audio format unsupported: rate=%lu channels=%u bits=%u", (unsigned long)info.sample_rate, (unsigned)info.channel, (unsigned)info.bits_per_sample); ok = false; break;
            }
            if (out.decoded_size && !locked_rate) { locked_rate = (int)info.sample_rate; locked_channels = (int)info.channel; }
            if (out.decoded_size && info.channel == 2) {
                int16_t *samples = (int16_t *)pcm;
                size_t frames = out.decoded_size / 4;
                for (size_t i=0; i<frames; ++i) {
                    int32_t mixed = (int32_t)samples[i*2] + samples[i*2+1];
                    samples[i] = (int16_t)(mixed/2);
                }
                out.decoded_size = frames*2;
            }
            if (out.decoded_size) music_feed_pcm(pcm, out.decoded_size);
            if (out.decoded_size && esp_codec_dev_write(dev, pcm, out.decoded_size) != 0) {
                ESP_LOGE(TAG, "MP3 I2S write failed"); ok = false; break;
            }
        }
        // A frame-mode decoder is fed one complete frame at a time. Never
        // re-submit a fragment as though it were the start of a new frame.
        drop_encoded(encoded, &buffered, frame_bytes, &consumed_total);
    }
    ESP_LOGI(TAG, "MP3 frame stats: decoded=%lu bad=%lu resync_bytes=%lu",
             (unsigned long)frames_decoded, (unsigned long)bad_frames, (unsigned long)resync_bytes);
    ESP_LOGI(TAG, "MP3 stream %s: network_received=%llu consumed=%llu queued=%u bytes",
             ok ? "completed" : "interrupted",
             (unsigned long long)net.received, (unsigned long long)consumed_total,
             net.stream ? (unsigned)xStreamBufferBytesAvailable(net.stream) : 0);
cleanup:
    if (net.task) {
        net.cancel = true;
        // Network read has a bounded HTTP timeout; do not free shared state early.
        while (!net.finished) vTaskDelay(pdMS_TO_TICKS(20));
        net.task = NULL;
    }
    if (net.stream) vStreamBufferDelete(net.stream);
    if (decoder) esp_audio_simple_dec_close(decoder);
    if (http) { esp_http_client_close(http); esp_http_client_cleanup(http); }
    free(encoded);
    free(pcm);
    portENTER_CRITICAL(&controls_lock);
    playback_active = false;
    playback_stop_requested = false;
    portEXIT_CRITICAL(&controls_lock);
    state_dirty = true;
    return ok;
}

// ADTS frame size including the 7/9-byte transport header.
// This only identifies framing; decoder validates AAC profile and payload.
static size_t aac_adts_frame_length(const uint8_t *p, size_t n,
                                    int *sample_rate, int *channels) {
    static const int rates[16] = {96000,88200,64000,48000,44100,32000,
        24000,22050,16000,12000,11025,8000,7350,0,0,0};
    if (n < 7 || p[0] != 0xff || (p[1] & 0xf6) != 0xf0) return 0;
    int profile = (p[2] >> 6) & 3;
    int index = (p[2] >> 2) & 15;
    int ch = ((p[2] & 1) << 2) | ((p[3] >> 6) & 3);
    size_t bytes = ((size_t)(p[3] & 3) << 11) | ((size_t)p[4] << 3) | (p[5] >> 5);
    size_t header = (p[1] & 1) ? 7 : 9;
    if (profile == 3 || !rates[index] || ch < 1 || ch > 2 ||
        bytes < header || bytes > 8191) return 0;
    if (sample_rate) *sample_rate = rates[index];
    if (channels) *channels = ch;
    return bytes;
}

static bool play_aac_adts(esp_codec_dev_handle_t dev, audio_source_t *src) {
    // Incremental HTTP -> bounded compressed buffer -> AAC decoder -> I2S.
    // No Content-Length requirement, no full-file PSRAM allocation.
    esp_http_client_handle_t http = src->http;
    src->http = NULL; // playback now owns the open, already probed connection
    if (!http) return false;
    uint8_t *encoded = NULL, *pcm = NULL;
    esp_audio_simple_dec_handle_t decoder = NULL;
    bool ok = false, eof = false;
    mp3_network_t net = {0};
    size_t buffered = 0;
    uint64_t consumed_total = 0;
    uint32_t rebuffer_events = 0;
    uint32_t last_stats_tick = 0;
    bool rebuffering = true;
    TickType_t rebuffer_start = 0;
    int locked_rate = 0, locked_channels = 0;
    uint32_t rejected_format_changes = 0;
    uint32_t resync_bytes = 0, frames_decoded = 0, bad_frames = 0;
    bool frame_locked = false;
    int64_t content_length = src->content_length;
    ESP_LOGI(TAG, "AAC streaming on probe connection: HTTP 200, Content-Length=%lld",
             (long long)content_length);
    encoded = heap_caps_malloc(AAC_STREAM_BUFFER, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    pcm = heap_caps_malloc(32768, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!encoded || !pcm) { ESP_LOGE(TAG, "AAC stream buffer allocation failed"); goto cleanup; }
    // Keep switching responsive even if TLS radio server stops sending data.
    // ESP-IDF HTTP client handles chunked transfer framing internally.
    esp_http_client_set_timeout_ms(http, src->chunked ? 1500 : 1000);
    net.http = http;
    net.chunked = src->chunked;
    net.codec_name = "AAC";
    net.content_length = content_length;
    net.last_data_tick = (uint32_t)xTaskGetTickCount();
    ESP_LOGI(TAG, "%s HTTP producer: framing=%s, read=%d, timeout=%d ms",
             net.codec_name, net.chunked ? "chunked" : "identity",
             net.chunked ? 256 : 2048, net.chunked ? 1500 : 1000);
    net.stream = xStreamBufferCreateWithCaps(AAC_NETWORK_BUFFER, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!net.stream) { ESP_LOGE(TAG, "AAC PSRAM ring allocation failed"); goto cleanup; }
    // Preserve all bytes consumed during format detection, in original order.
    if (src->first_len) {
        size_t sent = xStreamBufferSend(net.stream, src->first, src->first_len, 0);
        if (sent != src->first_len) { ESP_LOGE(TAG, "Probe prefix enqueue failed"); goto cleanup; }
        net.received = src->first_len;
    }
    esp_audio_simple_dec_cfg_t dc = {.dec_type=ESP_AUDIO_SIMPLE_DEC_TYPE_AAC, .use_frame_dec=true};
    if (esp_audio_simple_dec_open(&dc, &decoder) != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(TAG, "AAC decoder open failed"); goto cleanup;
    }
    if (xTaskCreate(audio_network_task, "mishka_http", 6144, &net, 5, &net.task) != pdPASS) {
        ESP_LOGE(TAG, "AAC network task creation failed"); goto cleanup;
    }
    ESP_LOGI(TAG, "AAC PSRAM network buffer=%u KB, initial prebuffer=%u KB, rebuffer=%u KB",
             (unsigned)(AAC_NETWORK_BUFFER / 1024), (unsigned)(AAC_PREBUFFER / 1024),
             (unsigned)(AAC_REBUFFER / 1024));
    portENTER_CRITICAL(&controls_lock);
    playback_active = true;
    playback_stop_requested = false;
    portEXIT_CRITICAL(&controls_lock);
    state_dirty = true;
    ok = true;
    for (;;) {
        bool stop, paused, changed;
        int volume;
        portENTER_CRITICAL(&controls_lock);
        stop = playback_stop_requested || !playback_enabled || new_url_pending;
        portEXIT_CRITICAL(&controls_lock);
        if (stop) { ok = false; break; }
        controls_snapshot(&paused, &volume, &changed);
        if (state_dirty) publish_state();
        apply_volume(dev, volume, changed);
        if (paused) {
            // Keep DMA supplied with silence. An HTTP server may time out during a long pause.
            write_silence(dev, 32);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        // Refill the compressed input buffer independently from the network task.
        // If a live station stalls, hold decoding until enough bytes accumulate.
        if (!eof && buffered <= AAC_STREAM_BUFFER - 4096) {
            size_t want = AAC_STREAM_BUFFER - buffered;
            if (want > 4096) want = 4096;
            size_t n = xStreamBufferReceive(net.stream, encoded + buffered, want, pdMS_TO_TICKS(10));
            if (n > 0) buffered += n;
            else if (net.finished && xStreamBufferBytesAvailable(net.stream) == 0) {
                eof = true;
                if (net.failed) { ok = false; break; }
            }
        }
        size_t queued = xStreamBufferBytesAvailable(net.stream);
        if (rebuffering) {
            size_t threshold = consumed_total == 0 ? AAC_PREBUFFER : AAC_REBUFFER;
            // Start AAC decoding as soon as complete frames can be available.
            // Some live HTTPS servers deliver only ~20 KB before stalling;
            // waiting for MP3's 24 KB prebuffer prevented any consumption.
            // A short finite file must not wait forever for 48 KB.
            if (!net.finished && buffered + queued < threshold) {
                if (rebuffer_start == 0) rebuffer_start = xTaskGetTickCount();
                if (xTaskGetTickCount() - rebuffer_start > pdMS_TO_TICKS(AAC_REBUFFER_TIMEOUT_MS)) {
                    ESP_LOGE(TAG, "AAC rebuffer timeout: buffered=%u queued=%u received=%llu read_zeros=%lu",
                             (unsigned)buffered, (unsigned)queued,
                             (unsigned long long)net.received, (unsigned long)net.read_zero_count);
                    ok = false; break;
                }
                write_silence(dev, 20);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            if (consumed_total > 0) {
                ESP_LOGI(TAG, "AAC rebuffer recovered #%lu: buffered=%u queued=%u",
                         (unsigned long)rebuffer_events, (unsigned)buffered, (unsigned)queued);
            }
            rebuffering = false;
            rebuffer_start = 0;
        }
        // Rebuffer before the entire compressed pipeline drains.
        // Do not enter this state while the producer has already finished.
        if (!eof && !net.finished && buffered + queued < AAC_LOW_WATERMARK) {
            ++rebuffer_events;
            rebuffering = true;
            rebuffer_start = xTaskGetTickCount();
            ESP_LOGW(TAG, "AAC underrun -> rebuffer #%lu: received=%llu consumed=%llu queued=%u, read_zeros=%lu",
                     (unsigned long)rebuffer_events, (unsigned long long)net.received,
                     (unsigned long long)consumed_total, (unsigned)queued,
                     (unsigned long)net.read_zero_count);
            continue;
        }
        if (xTaskGetTickCount() - last_stats_tick >= pdMS_TO_TICKS(10000)) {
            last_stats_tick = xTaskGetTickCount();
            ESP_LOGI(TAG, "AAC stats: received=%llu consumed=%llu buffered=%u queued=%u rebuffer=%lu read_zeros=%lu",
                     (unsigned long long)net.received, (unsigned long long)consumed_total,
                     (unsigned)buffered, (unsigned)queued, (unsigned long)rebuffer_events,
                     (unsigned long)net.read_zero_count);
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // yield to IDLE0 and avoid watchdog starvation
        if (buffered == 0 && eof) break;
        if (buffered == 0) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
        if (buffered < 4) {
            if (eof) break;
            continue;
        }
        int header_rate = 0, header_channels = 0;
        size_t frame_bytes = aac_adts_frame_length(encoded, buffered, &header_rate, &header_channels);
        if (!frame_bytes || (locked_rate &&
            (header_rate != locked_rate || header_channels != locked_channels))) {
            // Scan at most 1024 bytes per iteration, then shift once. The
            // previous byte-by-byte memmove starved IDLE0 and tripped WDT.
            size_t skip = 1;
            size_t scan_limit = buffered >= 4 ? buffered - 3 : 0;
            if (scan_limit > 1024) scan_limit = 1024;
            for (; skip < scan_limit; ++skip) {
                int candidate_rate = 0, candidate_channels = 0;
                if (aac_adts_frame_length(encoded + skip, buffered - skip,
                                     &candidate_rate, &candidate_channels) &&
                    (!locked_rate || (candidate_rate == locked_rate &&
                                      candidate_channels == locked_channels))) break;
            }
            drop_encoded(encoded, &buffered, skip, &consumed_total);
            resync_bytes += (uint32_t)skip;
            if (resync_bytes <= 5 || (resync_bytes / 4096) != ((resync_bytes - skip) / 4096))
                ESP_LOGW(TAG, "AAC frame resync: skipped %lu byte(s)", (unsigned long)resync_bytes);
            if (resync_bytes > 8192 && frames_decoded == 0) {
                ESP_LOGE(TAG, "No AAC frame in first 8 KB: station may use AAC/OGG/playlist or incompatible MPEG stream");
                ok = false; break;
            }
            if (resync_bytes > 65536) {
                ESP_LOGE(TAG, "AAC frame synchronization lost beyond 64 KB");
                ok = false; break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        if (buffered < frame_bytes) {
            if (eof) { ESP_LOGW(TAG, "Truncated final AAC frame"); break; }
            continue;
        }
        // Confirm the next frame header before locking to an arbitrary sync word.
        if (!frame_locked && !eof && buffered < frame_bytes + 4) continue;
        if (!frame_locked && buffered >= frame_bytes + 4 &&
            !aac_adts_frame_length(encoded + frame_bytes, buffered - frame_bytes, NULL, NULL)) {
            drop_encoded(encoded, &buffered, 1, &consumed_total);
            ++resync_bytes;
            if (resync_bytes > 8192 && frames_decoded == 0) {
                ESP_LOGE(TAG, "No consecutive AAC frames in first 8 KB; unsupported stream format");
                ok = false; break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        frame_locked = true;
        esp_audio_simple_dec_raw_t raw = {
            .buffer = encoded, .len = (uint32_t)frame_bytes,
            .eos = eof && frame_bytes == buffered
        };
        esp_audio_simple_dec_out_t out = {.buffer=pcm, .len=32768};
        esp_audio_err_t err = esp_audio_simple_dec_process(decoder, &raw, &out);
        if (err != ESP_AUDIO_ERR_OK) {
            ++bad_frames;
            ESP_LOGW(TAG, "AAC frame decode error=%d, frame=%u, bad=%lu",
                     (int)err, (unsigned)frame_bytes, (unsigned long)bad_frames);
            if (bad_frames > 30) { ok = false; break; }
            drop_encoded(encoded, &buffered, frame_bytes, &consumed_total);
            continue;
        }
        if (raw.consumed > frame_bytes) { ok = false; break; }
        if (raw.consumed == 0 && out.decoded_size == 0) {
            ESP_LOGE(TAG, "Frame decoder made no progress on a complete %u-byte frame", (unsigned)frame_bytes);
            ok = false; break;
        }
        ++frames_decoded;
        if (out.decoded_size) {
            esp_audio_simple_dec_info_t info = {0};
            if (esp_audio_simple_dec_get_info(decoder, &info) != ESP_AUDIO_ERR_OK ||
                info.bits_per_sample != 16 || info.sample_rate < 8000 ||
                info.sample_rate > 48000 || (info.channel != 1 && info.channel != 2)) {
                ESP_LOGE(TAG, "AAC decoder returned invalid format: rate=%lu channels=%u bits=%u",
                         (unsigned long)info.sample_rate, (unsigned)info.channel, (unsigned)info.bits_per_sample);
                ok = false; break;
            }
            // AAC radio streams should not oscillate between unrelated sample rates.
            // A corrupt frame must not repeatedly close/reopen the I2S codec.
            if (locked_rate && ((int)info.sample_rate != locked_rate || (int)info.channel != locked_channels)) {
                ++rejected_format_changes;
                ESP_LOGW(TAG, "AAC unexpected format change #%lu: %d/%d -> %lu/%u; skipping frame",
                         (unsigned long)rejected_format_changes, locked_rate, locked_channels,
                         (unsigned long)info.sample_rate, (unsigned)info.channel);
                out.decoded_size = 0;
            } else if (!audio_format(dev, (int)info.sample_rate, (int)info.channel)) {
                ESP_LOGE(TAG, "AAC audio format unsupported: rate=%lu channels=%u bits=%u", (unsigned long)info.sample_rate, (unsigned)info.channel, (unsigned)info.bits_per_sample); ok = false; break;
            }
            if (out.decoded_size && !locked_rate) { locked_rate = (int)info.sample_rate; locked_channels = (int)info.channel; }
            if (out.decoded_size && info.channel == 2) {
                int16_t *samples = (int16_t *)pcm;
                size_t frames = out.decoded_size / 4;
                for (size_t i=0; i<frames; ++i) {
                    int32_t mixed = (int32_t)samples[i*2] + samples[i*2+1];
                    samples[i] = (int16_t)(mixed/2);
                }
                out.decoded_size = frames*2;
            }
            if (out.decoded_size) music_feed_pcm(pcm, out.decoded_size);
            if (out.decoded_size && esp_codec_dev_write(dev, pcm, out.decoded_size) != 0) {
                ESP_LOGE(TAG, "AAC I2S write failed"); ok = false; break;
            }
        }
        // A frame-mode decoder is fed one complete frame at a time. Never
        // re-submit a fragment as though it were the start of a new frame.
        drop_encoded(encoded, &buffered, frame_bytes, &consumed_total);
    }
    ESP_LOGI(TAG, "AAC HTTP diagnostics: read_calls=%lu max_read_ms=%lu last_error=%d zero_reads=%lu transient_EAGAIN=%lu",
             (unsigned long)net.read_calls, (unsigned long)net.max_read_ms,
             net.last_read_error, (unsigned long)net.read_zero_count,
             (unsigned long)net.transient_errors);
    ESP_LOGI(TAG, "AAC frame stats: decoded=%lu bad=%lu resync_bytes=%lu",
             (unsigned long)frames_decoded, (unsigned long)bad_frames, (unsigned long)resync_bytes);
    ESP_LOGI(TAG, "AAC stream %s: network_received=%llu consumed=%llu queued=%u bytes",
             ok ? "completed" : "interrupted",
             (unsigned long long)net.received, (unsigned long long)consumed_total,
             net.stream ? (unsigned)xStreamBufferBytesAvailable(net.stream) : 0);
cleanup:
    if (net.task) {
        net.cancel = true;
        // Network read has a bounded HTTP timeout; do not free shared state early.
        while (!net.finished) vTaskDelay(pdMS_TO_TICKS(20));
        net.task = NULL;
    }
    if (net.stream) vStreamBufferDelete(net.stream);
    if (decoder) esp_audio_simple_dec_close(decoder);
    if (http) { esp_http_client_close(http); esp_http_client_cleanup(http); }
    free(encoded);
    free(pcm);
    portENTER_CRITICAL(&controls_lock);
    playback_active = false;
    playback_stop_requested = false;
    portEXIT_CRITICAL(&controls_lock);
    state_dirty = true;
    return ok;
}

static bool play_wav(esp_codec_dev_handle_t dev) {
    esp_http_client_config_t cfg = {
        .url = current_url,
        .timeout_ms = 12000,
        .buffer_size = 4096,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .disable_auto_redirect = false,
    };
    esp_http_client_handle_t http = esp_http_client_init(&cfg);
    if (!http) return false;
    int status = 0;
    if (!http_open_follow_redirects(http, "WAV", NULL, &status)) {
        esp_http_client_cleanup(http);
        return false;
    }
    ESP_LOGI(TAG, "HTTP status=%d URL=%s", status, current_url);
    if (status != 200) {
        esp_http_client_close(http);
        esp_http_client_cleanup(http);
        return false;
    }
    if (!audio_format(dev, SAMPLE_RATE, 1)) {
        esp_http_client_close(http); esp_http_client_cleanup(http); return false;
    }
    uint32_t remaining = 0;
    if (!wav_find_data(http, &remaining)) {
        esp_http_client_close(http);
        esp_http_client_cleanup(http);
        return false;
    }
    if (remaining < 2 || remaining > MAX_WAV_BYTES || (remaining & 1)) {
        ESP_LOGE(TAG, "WAV size unsupported: %lu bytes (max 4 MiB)", (unsigned long)remaining);
        esp_http_client_close(http);
        esp_http_client_cleanup(http);
        return false;
    }
    uint8_t *pcm = heap_caps_malloc(remaining, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pcm) {
        ESP_LOGE(TAG, "PSRAM allocation failed: %lu bytes", (unsigned long)remaining);
        esp_http_client_close(http);
        esp_http_client_cleanup(http);
        return false;
    }
    ESP_LOGI(TAG, "Downloading WAV into PSRAM: %lu bytes", (unsigned long)remaining);
    bool ok = read_exact(http, pcm, remaining);
    esp_http_client_close(http);
    esp_http_client_cleanup(http);
    if (!ok) { ESP_LOGE(TAG, "WAV download interrupted"); free(pcm); return false; }
    ESP_LOGI(TAG, "Playing WAV from PSRAM: %lu bytes", (unsigned long)remaining);
    uint32_t played = 0;
    bool was_paused = false;
    portENTER_CRITICAL(&controls_lock);
    playback_active = true;
    portEXIT_CRITICAL(&controls_lock);
    state_dirty = true;
    while (played < remaining) {
        bool stop;
        portENTER_CRITICAL(&controls_lock);
        stop = playback_stop_requested || !playback_enabled || new_url_pending;
        portEXIT_CRITICAL(&controls_lock);
        if (stop) { ESP_LOGI(TAG, "Playback stopped by command"); ok = false; break; }

        bool paused, changed;
        int volume;
        controls_snapshot(&paused, &volume, &changed);
        if (state_dirty) publish_state();
        apply_volume(dev, volume, changed);
        if (paused) {
            if (!was_paused) {
                ESP_LOGI(TAG, "Playback paused at %lu/%lu bytes", (unsigned long)played, (unsigned long)remaining);
            }
            was_paused = true;
            // Keep the I2S DMA fed with PCM zeros throughout the pause.
            // A single silence write followed by a delay leaves the TX stream
            // starved and can cause the last samples to repeat or glitch.
            write_silence(dev, 32);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        if (was_paused) ESP_LOGI(TAG, "Playback resumed");
        was_paused = false;
        size_t chunk = remaining - played;
        if (chunk > 512) chunk = 512; // 16 ms PCM chunks for responsive buttons
        music_feed_pcm(pcm + played, chunk);
        int rc = esp_codec_dev_write(dev, pcm + played, chunk);
        if (rc != 0) { ESP_LOGE(TAG, "PCM write failed: %d", rc); ok = false; break; }
        played += chunk;
    }
    ESP_LOGI(TAG, "WAV playback %s, %lu bytes", ok ? "completed" : "interrupted", (unsigned long)played);
    portENTER_CRITICAL(&controls_lock);
    playback_active = false;
    playback_stop_requested = false;
    portEXIT_CRITICAL(&controls_lock);
    state_dirty = true;
    free(pcm);
    return ok;
}

typedef struct {
    esp_codec_dev_handle_t dev;
    rmt_channel_handle_t led_ch;
    rmt_encoder_handle_t enc;
    uint8_t pixels[LED_COUNT * 3];
} playback_ctx_t;

/* RIFF PCM16 mono TTS, streamed from the already authenticated HTTP source. */
typedef struct {
    audio_source_t *src;
    size_t consumed;
} assist_wav_reader_t;
static bool assist_wav_read(assist_wav_reader_t *r, void *dst, size_t len) {
    uint8_t *p = dst;
    while (len) {
        if (r->consumed < r->src->first_len) {
            size_t n = r->src->first_len - r->consumed;
            if (n > len) n = len;
            memcpy(p, r->src->first + r->consumed, n);
            r->consumed += n; p += n; len -= n;
        } else {
            int n = esp_http_client_read(r->src->http, (char *)p, len > 4096 ? 4096 : len);
            if (n <= 0) return false;
            p += n; len -= (size_t)n;
        }
    }
    return true;
}
static bool assist_wav_skip(assist_wav_reader_t *r, uint32_t len) {
    uint8_t tmp[256];
    while (len) {
        size_t n = len > sizeof(tmp) ? sizeof(tmp) : len;
        if (!assist_wav_read(r, tmp, n)) return false;
        len -= (uint32_t)n;
    }
    return true;
}
static bool assist_play_wav(esp_codec_dev_handle_t dev, audio_source_t *src) {
    assist_wav_reader_t rd = {.src = src};
    uint8_t hdr[12];
    if (!assist_wav_read(&rd, hdr, sizeof(hdr)) || memcmp(hdr, "RIFF", 4) ||
        memcmp(hdr + 8, "WAVE", 4)) return false;
    int rate = 0;
    uint32_t remaining = 0;
    bool found = false;
    for (int i = 0; i < 32; ++i) {
        uint8_t ch[8];
        if (!assist_wav_read(&rd, ch, sizeof(ch))) return false;
        uint32_t len = le32(ch + 4);
        if (!memcmp(ch, "fmt ", 4)) {
            uint8_t fmt[16];
            if (len < 16 || len > 4096 || !assist_wav_read(&rd, fmt, sizeof(fmt))) return false;
            if (le16(fmt) != 1 || le16(fmt + 2) != 1 || le16(fmt + 14) != 16) {
                ESP_LOGE("MISHKA_ASSIST", "TTS WAV must be PCM16 mono");
                return false;
            }
            rate = (int)le32(fmt + 4);
            if (!assist_wav_skip(&rd, len - 16 + (len & 1))) return false;
        } else if (!memcmp(ch, "data", 4)) {
            if (rate < 8000 || rate > 48000 || len > MAX_WAV_BYTES || (len & 1)) return false;
            remaining = len;
            found = true;
            break;
        } else {
            if (len > 1024U * 1024U || !assist_wav_skip(&rd, len + (len & 1))) return false;
        }
    }
    if (!found || !audio_format(dev, rate, 1)) return false;
    ESP_LOGI("MISHKA_ASSIST", "TTS WAV playback: rate=%d bytes=%lu", rate, (unsigned long)remaining);
    uint8_t pcm[1024];
    while (remaining) {
        if (playback_stop_requested || new_url_pending) return false;
        size_t n = remaining > sizeof(pcm) ? sizeof(pcm) : remaining;
        if (!assist_wav_read(&rd, pcm, n) || esp_codec_dev_write(dev, pcm, n) != 0) return false;
        remaining -= (uint32_t)n;
    }
    return true;
}

/* Called exclusively from playback_task: one I2S writer for radio and Assist. */
static void assist_play_pending(esp_codec_dev_handle_t dev) {
    char url[sizeof(assist_audio_url)];
    portENTER_CRITICAL(&controls_lock);
    memcpy(url, assist_audio_url, sizeof(url));
    assist_tts_pending = false;
    assist_tts_active = true;
    playback_stop_requested = false;
    playback_enabled = true;
    playback_paused = false;
    portEXIT_CRITICAL(&controls_lock);
    mishka_assist_set_tts_playing(true);
    bool ok = false;
    audio_source_t src = {0};
    if (source_open_assist(url, &src)) {
        if (src.kind == AUDIO_PROBE_MP3) ok = play_mp3(dev, &src);
        else if (src.kind == AUDIO_PROBE_AAC) ok = play_aac_adts(dev, &src);
        else if (src.kind == AUDIO_PROBE_WAV) ok = assist_play_wav(dev, &src);
        else ESP_LOGE("MISHKA_ASSIST", "Unsupported TTS format: %s", audio_probe_name(src.kind));
    }
    source_close(&src);
    mishka_assist_set_tts_playing(false);
    ESP_LOGI("MISHKA_ASSIST", "TTS playback %s", ok ? "complete" : "failed");
    portENTER_CRITICAL(&controls_lock);
    assist_tts_active = false;
    if (assist_radio_resume) {
        playback_enabled = true;
        playback_paused = false;
    } else {
        playback_enabled = false;
        playback_paused = false;
    }
    assist_radio_resume = false;
    playback_stop_requested = false;
    portEXIT_CRITICAL(&controls_lock);
    state_dirty = true;
}

static void playback_task(void *arg) {
    playback_ctx_t *ctx = arg;
    esp_codec_dev_handle_t dev = ctx->dev;
    ESP_LOGI(TAG, "Playback worker started; stack remaining %u bytes",
             (unsigned)uxTaskGetStackHighWaterMark(NULL));
    for (;;) {
        if (assist_tts_pending) {
            assist_play_pending(dev);
            continue;
        }
        if (assist_session_in_progress) {
            write_silence(dev, 32);
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        if (new_url_pending) {
            radio_retry_profile = 0;
            portENTER_CRITICAL(&controls_lock);
            memcpy(current_url, pending_url, sizeof(current_url));
            new_url_pending = false;
            playback_stop_requested = false;
            playback_enabled = true;
            playback_paused = false;
            portEXIT_CRITICAL(&controls_lock);
            ESP_LOGI(TAG,"New track URL length=%u, scheme=%s, signed=%s",
                     (unsigned)strlen(current_url),
                     strncmp(current_url,"https://",8)==0 ? "https" : "http",
                     strstr(current_url,"authSig=") ? "yes" : "no");
        }
        if (state_dirty) publish_state();
        bool enabled;
        portENTER_CRITICAL(&controls_lock); enabled = playback_enabled; portEXIT_CRITICAL(&controls_lock);
        if (!enabled) {
            write_silence(dev, 100);
            continue;
        }
        // LED strip is controlled by Home Assistant; playback does not override it.
        audio_source_t source = {0};
        bool ok = false;
        TickType_t start_tick = xTaskGetTickCount();
        if (source_open(current_url, &source)) {
            ESP_LOGI(TAG, "Station connect+probe: %lu ms",
                     (unsigned long)(pdTICKS_TO_MS(xTaskGetTickCount() - start_tick)));
            if (source.kind == AUDIO_PROBE_MP3 || source.kind == AUDIO_PROBE_UNKNOWN) {
                if (source.kind == AUDIO_PROBE_UNKNOWN)
                    ESP_LOGW(TAG, "Unknown signature; bounded MP3 frame scan");
                ok = play_mp3(dev, &source);
            } else if (source.kind == AUDIO_PROBE_AAC) {
                ok = play_aac_adts(dev, &source);
            } else if (source.kind == AUDIO_PROBE_WAV) {
                // WAV path still owns its own HTTP connection: it needs the RIFF
                // parser, which currently reads the header directly from HTTP.
                source_close(&source);
                ok = play_wav(dev);
            } else {
                ESP_LOGE(TAG, "Unsupported codec/container: %s",
                         audio_probe_name(source.kind));
            }
        }
        source_close(&source);
        if (ok) radio_retry_profile = 0;
        // Retry a failed live radio stream, without restarting when the user
        // has switched stations or explicitly stopped playback.
        bool retry = false;
        portENTER_CRITICAL(&controls_lock);
        retry = !assist_session_in_progress && !assist_tts_pending && !ok && !new_url_pending && playback_enabled && !playback_stop_requested;
        portEXIT_CRITICAL(&controls_lock);
        if (retry) {
            static char retry_url[1024] = {0};
            static unsigned retry_count = 0;
            if (strcmp(retry_url, current_url) != 0) {
                snprintf(retry_url, sizeof(retry_url), "%s", current_url);
                retry_count = 0;
            }
            if (retry_count < 3) {
                ++retry_count;
                radio_retry_profile = retry_count;
                uint32_t wait_ms = retry_count * 1000U;
                ESP_LOGW(TAG, "Stream failed: reconnect %u/3 in %lu ms",
                         retry_count, (unsigned long)wait_ms);
                for (uint32_t waited = 0; waited < wait_ms; waited += 100) {
                    if (new_url_pending || !playback_enabled || playback_stop_requested) break;
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                if (!new_url_pending && playback_enabled && !playback_stop_requested) continue;
            } else {
                ESP_LOGE(TAG, "Stream reconnect exhausted (3 attempts)");
                retry_count = 0;
                radio_retry_profile = 0;
                retry_url[0] = 0;
            }
        }
        portENTER_CRITICAL(&controls_lock);
        if (!new_url_pending && !assist_session_in_progress && !assist_tts_pending) playback_enabled = false;
        playback_active = false;
        portEXIT_CRITICAL(&controls_lock);
        state_dirty = true;
        write_silence(dev, 100);
        vTaskDelay(pdMS_TO_TICKS(10));

    }
    vTaskDelete(NULL);
}

void app_main(void) {
    ESP_LOGI(TAG, "Mishka Wi-Fi Speaker v0.5.8.23 OTA + mic WAV DSP + BOOT GPIO0 + music: AP setup portal + HTTP retry request profiles + transport diagnostics + AAC/MP3 pipeline + fast switching + MQTT, GPIO16");
    size_t wake_model_bytes = (size_t)(mishka_model_end - mishka_model_start);
    if (wake_model_bytes == 62200U &&
        memcmp(mishka_model_start + 4, "TFL3", 4) == 0) {
        ESP_LOGI("MISHKA_WAKE", "mishka.tflite embedded: %lu bytes, TFL3 header OK; inference=STARTING",
                 (unsigned long)wake_model_bytes);
    } else {
        ESP_LOGE("MISHKA_WAKE", "Unexpected model size/header: %lu bytes", (unsigned long)wake_model_bytes);
    }
    esp_err_t nvs = nvs_flash_init();
    // Never erase a user's existing credentials or volume on a migration failure.
    if (nvs != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed (%s). Preserving flash; restore backup or repair manually.", esp_err_to_name(nvs));
        return;
    }
    rmt_channel_handle_t led_ch = NULL;
    rmt_tx_channel_config_t rmt_cfg = {.gpio_num=LED_PIN,.clk_src=RMT_CLK_SRC_DEFAULT,
        .resolution_hz=10000000,.mem_block_symbols=64,.trans_queue_depth=4};
    ESP_ERROR_CHECK(rmt_new_tx_channel(&rmt_cfg,&led_ch));
    rmt_encoder_handle_t enc = NULL;
    rmt_bytes_encoder_config_t enc_cfg = {
        .bit0={.level0=1,.duration0=4,.level1=0,.duration1=9},
        .bit1={.level0=1,.duration0=8,.level1=0,.duration1=5},.flags.msb_first=1};
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&enc_cfg,&enc));
    ESP_ERROR_CHECK(rmt_enable(led_ch));
    uint8_t pixels[LED_COUNT*3] = {0};
    led_show(led_ch,enc,pixels,0,0,0);

    i2c_master_bus_handle_t bus = NULL;
    i2c_master_bus_config_t bus_cfg = {.i2c_port=I2C_NUM_0,.sda_io_num=SDA_PIN,
        .scl_io_num=SCL_PIN,.clk_source=I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt=7,.flags.enable_internal_pullup=true};
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg,&bus));
    i2c_master_dev_handle_t tca = NULL;
    i2c_device_config_t dc = {.dev_addr_length=I2C_ADDR_BIT_LEN_7,
        .device_address=0x20,.scl_speed_hz=100000};
    if (i2c_master_probe(bus,0x20,100) != ESP_OK) {
        ESP_LOGE(TAG,"TCA9555 not found at 0x20"); return;
    }
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus,&dc,&tca));
    if (!pa_prepare(tca)) { ESP_LOGE(TAG,"PA setup failed"); return; }
    microphone_probe(bus);
    nvs_handle_t saved;
    if (nvs_open("mishka", NVS_READONLY, &saved) == ESP_OK) {
        int32_t stored;
        if (nvs_get_i32(saved, "volume", &stored) == ESP_OK && stored >= 0 && stored <= 100)
            requested_volume = stored;
        nvs_close(saved);
    }
    esp_audio_err_t mp3_register = esp_audio_dec_register_default();
    if (mp3_register != ESP_AUDIO_ERR_OK) ESP_LOGW(TAG, "Audio decoder register returned %d", (int)mp3_register);
    esp_codec_dev_handle_t dev = audio_init(bus);
    if (!dev) return;
    esp_codec_dev_set_out_vol(dev, requested_volume);
    write_silence(dev, 100);
    microphone_pcm_init(bus);
    if (!pa_set(tca,true)) return;
    if (xTaskCreate(buttons_task, "mishka_buttons", 4096, tca, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Cannot create buttons task"); return;
    }
    ESP_LOGI(TAG, "Buttons ready: KEY1 volume-, KEY2 pause/resume, KEY3 volume+, BOOT GPIO0 microphone on/off");
    led_show(led_ch,enc,pixels,0,0,12);

    settings_load();
    if (!wifi_connect()) {
        ESP_LOGW(TAG,"Wi-Fi not connected; configuration portal remains available");
        led_show(led_ch,enc,pixels,18,8,0);
        return;
    }
    favorites_load();
    station_restore_selection();
    led_restore_settings();
    mqtt_start();
    led_ctx_t *led_ctx=calloc(1,sizeof(*led_ctx));
    if (!led_ctx) { ESP_LOGE(TAG,"LED context allocation failed"); return; }
    led_ctx->channel=led_ch; led_ctx->encoder=enc;
    if (xTaskCreate(led_task,"mishka_led",4096,led_ctx,3,NULL)!=pdPASS) {
        ESP_LOGE(TAG,"LED task creation failed"); free(led_ctx); return;
    }
    playback_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) { ESP_LOGE(TAG, "Playback context allocation failed"); return; }
    ctx->dev = dev;
    ctx->led_ch = led_ch;
    ctx->enc = enc;
    BaseType_t created = xTaskCreate(playback_task, "mishka_playback", 16384, ctx, 5, NULL);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Cannot create playback task");
        free(ctx);
        return;
    }
    ESP_LOGI(TAG, "OTA: awaiting manual confirmation if image is pending verification");
}
