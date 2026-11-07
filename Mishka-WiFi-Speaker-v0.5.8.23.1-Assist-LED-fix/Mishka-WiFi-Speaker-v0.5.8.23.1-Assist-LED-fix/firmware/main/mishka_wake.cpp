#include "mishka_wake.h"
#include "mishka_assist.h"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <cmath>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_allocator.h"
#include "tensorflow/lite/micro/micro_resource_variable.h"
#include "tensorflow/lite/micro/schema/schema_generated.h"
#include "tensorflow/lite/experimental/microfrontend/lib/frontend.h"
#include "tensorflow/lite/experimental/microfrontend/lib/frontend_util.h"

static const char *TAG = "MISHKA_WAKE";
static constexpr size_t BLOCK = 160;  // 10 ms, 16 kHz mono
static constexpr int FEATURES = 40;
static constexpr size_t ARENA_BYTES = 100000;
static QueueHandle_t queue = nullptr;
static volatile bool muted = false;
static volatile bool inference_ready = false;
extern "C" bool mishka_wake_ready(void) {return inference_ready;}
static volatile bool reset_requested = false;
static const uint8_t *model_bytes = nullptr;
static size_t model_length = 0;
static uint32_t dropped = 0;
struct AudioBlock { int16_t pcm[BLOCK]; };
static AudioBlock pending = {};
static size_t pending_count = 0;

extern "C" void mishka_wake_set_muted(bool value) {
    muted = value;
    reset_requested = true;
    pending_count = 0;
    if (queue) xQueueReset(queue);
}

extern "C" void mishka_wake_push(const int16_t *pcm, size_t count) {
    if (!queue || muted || !pcm) return;
    // Called only by the microphone I2S producer task.
    for (size_t i=0; i<count; ++i) {
        pending.pcm[pending_count++] = pcm[i];
        if (pending_count == BLOCK) {
            if (xQueueSend(queue, &pending, 0) != pdTRUE) ++dropped;
            pending_count = 0;
        }
    }
}

static bool register_ops(tflite::MicroMutableOpResolver<20> &ops) {
    return ops.AddCallOnce() == kTfLiteOk &&
        ops.AddVarHandle() == kTfLiteOk &&
        ops.AddReshape() == kTfLiteOk &&
        ops.AddReadVariable() == kTfLiteOk &&
        ops.AddStridedSlice() == kTfLiteOk &&
        ops.AddConcatenation() == kTfLiteOk &&
        ops.AddAssignVariable() == kTfLiteOk &&
        ops.AddConv2D() == kTfLiteOk &&
        ops.AddMul() == kTfLiteOk &&
        ops.AddAdd() == kTfLiteOk &&
        ops.AddMean() == kTfLiteOk &&
        ops.AddFullyConnected() == kTfLiteOk &&
        ops.AddLogistic() == kTfLiteOk &&
        ops.AddQuantize() == kTfLiteOk &&
        ops.AddDepthwiseConv2D() == kTfLiteOk &&
        ops.AddAveragePool2D() == kTfLiteOk &&
        ops.AddMaxPool2D() == kTfLiteOk &&
        ops.AddPad() == kTfLiteOk &&
        ops.AddPack() == kTfLiteOk &&
        ops.AddSplitV() == kTfLiteOk;
}

static void wake_task(void *) {
    const tflite::Model *model = tflite::GetModel(model_bytes);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "TFLite schema mismatch: %ld", (long)model->version());
        vTaskDelete(nullptr);
        return;
    }
    auto *arena = static_cast<uint8_t *>(heap_caps_malloc(ARENA_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    auto *var_arena = static_cast<uint8_t *>(heap_caps_malloc(16384, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!arena || !var_arena) {
        ESP_LOGE(TAG, "Tensor arena allocation failed");
        vTaskDelete(nullptr);
        return;
    }
    auto *allocator = tflite::MicroAllocator::Create(var_arena, 16384);
    auto *variables = tflite::MicroResourceVariables::Create(allocator, 20);
    tflite::MicroMutableOpResolver<20> resolver;
    if (!allocator || !variables || !register_ops(resolver)) {
        ESP_LOGE(TAG, "TFLM resolver/resource initialization failed");
        vTaskDelete(nullptr);
        return;
    }
    tflite::MicroInterpreter interpreter(model, resolver, arena, ARENA_BYTES, variables);
    if (interpreter.AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors failed: model or arena incompatible");
        vTaskDelete(nullptr);
        return;
    }
    TfLiteTensor *input = interpreter.input(0);
    TfLiteTensor *output = interpreter.output(0);
    if (!input || !output || input->type != kTfLiteInt8 || output->type != kTfLiteUInt8 ||
        input->dims->size != 3 || input->dims->data[0] != 1 || input->dims->data[2] != FEATURES ||
        output->dims->size != 2 || output->dims->data[0] != 1 || output->dims->data[1] != 1) {
        ESP_LOGE(TAG, "Model tensor shape/type incompatible with microWakeWord");
        vTaskDelete(nullptr);
        return;
    }
    const int stride = input->dims->data[1];
    if (stride <= 0 || stride > 64) {
        ESP_LOGE(TAG, "Invalid model stride %d", stride);
        vTaskDelete(nullptr);
        return;
    }
    FrontendConfig config = {};
    config.window.size_ms = 30;
    config.window.step_size_ms = 10;
    config.filterbank.num_channels = FEATURES;
    config.filterbank.lower_band_limit = 125.0f;
    config.filterbank.upper_band_limit = 7500.0f;
    config.noise_reduction.smoothing_bits = 10;
    config.noise_reduction.even_smoothing = 0.025f;
    config.noise_reduction.odd_smoothing = 0.06f;
    config.noise_reduction.min_signal_remaining = 0.05f;
    config.pcan_gain_control.enable_pcan = true;
    config.pcan_gain_control.strength = 0.95f;
    config.pcan_gain_control.offset = 80.0f;
    config.pcan_gain_control.gain_bits = 21;
    config.log_scale.enable_log = true;
    config.log_scale.scale_shift = 6;
    FrontendState frontend = {};
    if (!FrontendPopulateState(&config, &frontend, 16000)) {
        ESP_LOGE(TAG, "Audio frontend init failed");
        vTaskDelete(nullptr);
        return;
    }
    inference_ready = true;
    ESP_LOGI(TAG, "Inference ACTIVE, model=%u bytes, stride=%d, threshold=0.87", (unsigned)model_length, stride);
    int step = 0;
    uint8_t probabilities[5] = {};
    int prob_index = 0, prob_count = 0;
    uint32_t last_detect_ms = 0;
    AudioBlock block = {};
    uint32_t frames = 0;
    while (true) {
        if (reset_requested || muted) {
            FrontendReset(&frontend);
            step = 0;
            prob_index = prob_count = 0;
            memset(probabilities, 0, sizeof(probabilities));
            reset_requested = false;
            if (muted) { vTaskDelay(pdMS_TO_TICKS(30)); continue; }
        }
        if (xQueueReceive(queue, &block, pdMS_TO_TICKS(100)) != pdTRUE) continue;
        size_t consumed = 0;
        while (consumed < BLOCK && !muted && !reset_requested) {
            size_t read = 0;
            FrontendOutput f = FrontendProcessSamples(&frontend, block.pcm + consumed, BLOCK - consumed, &read);
            if (read == 0 && f.size == 0) break;
            consumed += read;
            if (f.size != FEATURES) continue;
            int8_t *dest = input->data.int8 + (step * FEATURES);
            for (int i=0; i<FEATURES; ++i) {
                int32_t value = (static_cast<int32_t>(f.values[i]) * 256 + 333) / 666 - 128;
                dest[i] = static_cast<int8_t>(std::clamp<int32_t>(value, -128, 127));
            }
            if (++step == stride) {
                step = 0;
                if (interpreter.Invoke() != kTfLiteOk) {
                    ESP_LOGE(TAG, "Invoke failed");
                    vTaskDelete(nullptr);
                    return;
                }
                probabilities[prob_index] = output->data.uint8[0];
                prob_index = (prob_index + 1) % 5;
                if (prob_count < 5) ++prob_count;
                int sum = 0;
                for (int j=0; j<prob_count; ++j) sum += probabilities[j];
                float mean = static_cast<float>(sum) / (prob_count * 255.0f);
                uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if (prob_count == 5 && mean >= 0.87f && (last_detect_ms == 0 || now - last_detect_ms > 2500)) {
                    ESP_LOGI(TAG, "DETECTED: Mishka probability=%.3f", static_cast<double>(mean));
                    last_detect_ms = now;
                    mishka_assist_detected();
                }
            }
        }
        if (++frames % 3000 == 0) ESP_LOGI(TAG, "running, dropped=%lu", (unsigned long)dropped);
    }
}

extern "C" void mishka_wake_init(const uint8_t *model, size_t size) {
    if (!model || size < 8 || memcmp(model + 4, "TFL3", 4) != 0) {
        ESP_LOGE(TAG, "Invalid model bytes");
        return;
    }
    model_bytes = model;
    model_length = size;
    queue = xQueueCreate(40, sizeof(AudioBlock));
    if (!queue || xTaskCreatePinnedToCore(wake_task, "mishka_wake", 8192, nullptr, 3, nullptr, 1) != pdPASS) {
        ESP_LOGE(TAG, "Cannot start wake task");
    }
}
