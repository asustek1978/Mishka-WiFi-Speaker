/* Home Assistant Assist Pipeline WebSocket transport.
 * This module handles wake-triggered STT streaming and pipeline events.
 * TTS is handed to the existing playback task after the pipeline finishes.
 */
#include "mishka_assist.h"
#include "esp_websocket_client.h"
#include "esp_log.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "cJSON.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define TAG "MISHKA_ASSIST"
#define CHUNK_SAMPLES 320
#define MAX_SESSION_MS 11000
#define SILENCE_TIMEOUT_MS 1100
#define SPEECH_RMS_GATE 230

typedef struct {uint16_t n; int16_t samples[CHUNK_SAMPLES];} audio_chunk_t;
static QueueHandle_t pcm_queue;
static TaskHandle_t worker;
static volatile bool muted;
static volatile bool tts_playing;
static volatile uint32_t wake_flash_until_ms;
static volatile bool session_active;
static volatile bool trigger_pending;
static volatile bool ws_connected;
static volatile bool ws_authenticated;
static volatile bool stream_ready;
static volatile bool stt_handler_registered;
static volatile bool pipeline_finished;
static volatile uint8_t audio_channel;
static char state[32] = "not_configured";
static char tts_url[768];
static char token[256];
static char pipeline[96];
static char host[96] = "192.168.1.67";
static int port=8123;
static esp_websocket_client_handle_t client;
static char json_buf[4096];
static size_t json_len;

static void state_set(const char *value) {
    snprintf(state,sizeof(state),"%s",value);
    ESP_LOGI(TAG,"state=%s",state);
}
const char *mishka_assist_state(void) {return tts_playing ? "speaking" : state;}
void mishka_assist_set_tts_playing(bool playing) {tts_playing=playing;}
uint32_t mishka_assist_wake_flash_until(void) {return wake_flash_until_ms;}
const char *mishka_assist_tts_url(void) {return tts_url;}
/* Do not return credentials through the HTTP API or log them. */
bool mishka_assist_authorization(char *out, size_t cap) {
    if (!out || !cap || !token[0]) return false;
    int n = snprintf(out, cap, "Bearer %s", token);
    return n > 0 && (size_t)n < cap;
}
/* Accept only same-origin HA TTS URLs, preventing token disclosure to other hosts. */
bool mishka_assist_resolve_tts_url(char *out, size_t cap) {
    if (!out || cap == 0 || !tts_url[0]) return false;
    char origin[160];
    snprintf(origin, sizeof(origin), "http://%s:%d", host, port);
    if (tts_url[0] == '/' && tts_url[1] != '/') {
        int n = snprintf(out, cap, "%s%s", origin, tts_url);
        return n > 0 && (size_t)n < cap;
    }
    size_t n = strlen(origin);
    if (strncmp(tts_url, origin, n) || tts_url[n] != '/') return false;
    return strlcpy(out, tts_url, cap) < cap;
}
bool mishka_assist_busy(void) {return session_active;}

void mishka_assist_mute(bool value) {
    muted=value;
    if(value) {
        trigger_pending=false;
        if(pcm_queue) xQueueReset(pcm_queue);
    }
}
void mishka_assist_detected(void) {
    if(!muted && !session_active && !trigger_pending && !tts_playing && token[0]) {
        wake_flash_until_ms=(uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS)+200U;
        trigger_pending=true;
    }
}
void mishka_assist_pcm(const int16_t *samples, size_t count) {
    if(!session_active || !stream_ready || muted || !pcm_queue) return;
    while(count) {
        audio_chunk_t b={0};
        b.n=(uint16_t)(count>CHUNK_SAMPLES?CHUNK_SAMPLES:count);
        memcpy(b.samples,samples,b.n*sizeof(int16_t));
        (void)xQueueSend(pcm_queue,&b,0);
        samples+=b.n;count-=b.n;
    }
}
static void send_json(const char *json) {
    if(client && ws_connected) esp_websocket_client_send_text(client,json,strlen(json),pdMS_TO_TICKS(1000));
}
static void parse_event(const char *json) {
    cJSON *root=cJSON_Parse(json);
    if(!root) return;
    cJSON *type=cJSON_GetObjectItemCaseSensitive(root,"type");
    const char *kind=cJSON_IsString(type)?type->valuestring:"";
    if(!strcmp(kind,"auth_required")) {
        /* cJSON guarantees correct escaping of the access token. */
        cJSON *auth=cJSON_CreateObject();
        cJSON_AddStringToObject(auth,"type","auth");
        cJSON_AddStringToObject(auth,"access_token",token);
        char *s=cJSON_PrintUnformatted(auth);
        if(s){send_json(s);free(s);}cJSON_Delete(auth);
    } else if(!strcmp(kind,"auth_ok")) {
        ESP_LOGI(TAG, "Home Assistant WebSocket auth_ok");
        ws_authenticated=true;
        cJSON *run=cJSON_CreateObject();
        cJSON_AddNumberToObject(run,"id",1);
        cJSON_AddStringToObject(run,"type","assist_pipeline/run");
        if(pipeline[0]) cJSON_AddStringToObject(run,"pipeline",pipeline);
        cJSON_AddStringToObject(run,"start_stage","stt");
        cJSON_AddStringToObject(run,"end_stage","tts");
        cJSON *input=cJSON_AddObjectToObject(run,"input");
        cJSON_AddNumberToObject(input,"sample_rate",16000);
        cJSON_AddStringToObject(input,"wake_word_phrase","Мишка");
        char *s=cJSON_PrintUnformatted(run);
        if(s){send_json(s);free(s);}cJSON_Delete(run);
        state_set("pipeline_starting");
    } else if(!strcmp(kind,"auth_invalid")) {
        stream_ready=false;stt_handler_registered=false;
        ESP_LOGE(TAG, "Home Assistant rejected token");
        state_set("auth_failed");pipeline_finished=true;
    } else if(!strcmp(kind,"result")) {
        cJSON *ok=cJSON_GetObjectItemCaseSensitive(root,"success");
        if(cJSON_IsFalse(ok)) {
            const cJSON *err=cJSON_GetObjectItemCaseSensitive(root,"error");
            const cJSON *code=cJSON_GetObjectItemCaseSensitive(err,"code");
            const cJSON *message=cJSON_GetObjectItemCaseSensitive(err,"message");
            ESP_LOGE(TAG,"Pipeline rejected: code=%s message=%s",
                cJSON_IsString(code)?code->valuestring:"unknown",
                cJSON_IsString(message)?message->valuestring:"unknown");
            stream_ready=false;stt_handler_registered=false;
            state_set("pipeline_error");pipeline_finished=true;
        }
    } else if(!strcmp(kind,"event")) {
        cJSON *event=cJSON_GetObjectItemCaseSensitive(root,"event");
        cJSON *evtype=cJSON_GetObjectItemCaseSensitive(event,"type");
        const char *e=cJSON_IsString(evtype)?evtype->valuestring:"";
        cJSON *data=cJSON_GetObjectItemCaseSensitive(event,"data");
        if(!strcmp(e,"run-start")) {
            cJSON *ch=cJSON_GetObjectItemCaseSensitive(data,"runner_data");
            cJSON *id=cJSON_GetObjectItemCaseSensitive(ch,"stt_binary_handler_id");
            if(cJSON_IsNumber(id) && id->valueint>0 && id->valueint<=255) {
                audio_channel=(uint8_t)id->valueint;
                stt_handler_registered=true;
                stream_ready=false; /* Wait for stt-start: handler is not yet guaranteed active. */
                ESP_LOGI(TAG,"Pipeline run-start, binary handler=%u",(unsigned)audio_channel);
                state_set("waiting_stt");
            } else {
                stream_ready=false;stt_handler_registered=false;
                state_set("no_audio_channel");pipeline_finished=true;
            }
        } else if(!strcmp(e,"stt-start")) {
            if(stt_handler_registered && ws_connected && !pipeline_finished) {
                stream_ready=true;
                state_set("listening");
            }
        } else if(!strcmp(e,"stt-end")) {
            /* The server has finished consuming this handler. Never send another PCM/EOF. */
            stream_ready=false;stt_handler_registered=false;
            cJSON *stt=cJSON_GetObjectItemCaseSensitive(data,"stt_output");
            cJSON *speech=cJSON_GetObjectItemCaseSensitive(stt,"text");
            if(cJSON_IsString(speech)) ESP_LOGI(TAG,"STT: %s",speech->valuestring);
        } else if(!strcmp(e,"tts-end")) {
            cJSON *out=cJSON_GetObjectItemCaseSensitive(data,"tts_output");
            cJSON *url=cJSON_GetObjectItemCaseSensitive(out,"url");
            if(cJSON_IsString(url)) {
                strlcpy(tts_url,url->valuestring,sizeof(tts_url));
                ESP_LOGI(TAG,"TTS audio URL received; queued after pipeline completion");
            }
        } else if(!strcmp(e,"run-end")) {
            stream_ready=false;stt_handler_registered=false;
            pipeline_finished=true;
            state_set("completed");
        } else if(!strcmp(e,"error")) {
            stream_ready=false;stt_handler_registered=false;
            pipeline_finished=true;
            state_set("pipeline_error");
            cJSON *code=cJSON_GetObjectItemCaseSensitive(data,"code");
            if(cJSON_IsString(code)) ESP_LOGE(TAG,"Assist error code=%s",code->valuestring);
        }
    }
    cJSON_Delete(root);
}
static void ws_event(void *arg, esp_event_base_t base, int32_t event_id, void *event_data) {
    (void)arg;(void)base;
    esp_websocket_event_data_t *d=event_data;
    if(event_id==WEBSOCKET_EVENT_CONNECTED) {ws_connected=true;state_set("authenticating");}
    else if(event_id==WEBSOCKET_EVENT_DISCONNECTED || event_id==WEBSOCKET_EVENT_ERROR) {
        ESP_LOGW(TAG,"WebSocket %s, authenticated=%s", 
                 event_id==WEBSOCKET_EVENT_ERROR?"error":"disconnected",
                 ws_authenticated?"yes":"no");
        ws_connected=false;stream_ready=false;stt_handler_registered=false;
    } else if(event_id==WEBSOCKET_EVENT_DATA && d && d->op_code==0x1) {
        if(d->payload_offset==0) json_len=0;
        if(d->data_len>0 && json_len+(size_t)d->data_len<sizeof(json_buf)) {
            memcpy(json_buf+json_len,d->data_ptr,d->data_len);
            json_len+=(size_t)d->data_len;
            if(d->payload_offset+d->data_len>=d->payload_len) {
                json_buf[json_len]=0;parse_event(json_buf);json_len=0;
            }
        } else {json_len=0;}
    }
}
extern void mishka_assist_on_start(void);
extern void mishka_assist_on_finish(void);
static void assist_task(void *arg) {
    (void)arg;
    audio_chunk_t block;
    char uri[160];
    for(;;) {
        if(!trigger_pending || muted || !token[0]) {vTaskDelay(pdMS_TO_TICKS(50));continue;}
        trigger_pending=false;session_active=true;pipeline_finished=false;
        mishka_assist_on_start();
        ws_connected=false;ws_authenticated=false;stream_ready=false;
        stt_handler_registered=false;
        tts_url[0]=0;
        xQueueReset(pcm_queue);
        if (strstr(host,"://") || strchr(host,'/') || strchr(host,':')) {
            ESP_LOGE(TAG,"Invalid HA host: use only IPv4 or hostname, without scheme/port/path");
            state_set("invalid_host");session_active=false;
            mishka_assist_on_finish();
            continue;
        }
        snprintf(uri,sizeof(uri),"ws://%s:%d/api/websocket",host,port);
        ESP_LOGI(TAG,"WebSocket target=%s (token hidden)",uri);
        esp_websocket_client_config_t cfg={.uri=uri,.disable_auto_reconnect=true,
                                           .buffer_size=4096,.network_timeout_ms=10000};
        client=esp_websocket_client_init(&cfg);
        if(!client){state_set("socket_alloc_error");session_active=false;continue;}
        esp_websocket_register_events(client,WEBSOCKET_EVENT_ANY,ws_event,NULL);
        if(esp_websocket_client_start(client)!=ESP_OK){state_set("connect_failed");goto cleanup;}
        uint32_t started=xTaskGetTickCount()*portTICK_PERIOD_MS;
        uint32_t last_voice=started;
        bool heard_speech=false;
        while(!pipeline_finished && !muted && xTaskGetTickCount()*portTICK_PERIOD_MS-started<MAX_SESSION_MS) {
            if(!stream_ready || pipeline_finished || !ws_connected) {
                vTaskDelay(pdMS_TO_TICKS(20));continue;
            }
            if(xQueueReceive(pcm_queue,&block,pdMS_TO_TICKS(100))==pdTRUE) {
                int64_t sum=0;
                for(int i=0;i<block.n;i++) sum+=(int64_t)block.samples[i]*block.samples[i];
                if(block.n && sum/block.n>(int64_t)SPEECH_RMS_GATE*SPEECH_RMS_GATE) {
                    last_voice=xTaskGetTickCount()*portTICK_PERIOD_MS;heard_speech=true;
                }
                char packet[1+CHUNK_SAMPLES*2];
                packet[0]=(char)audio_channel;
                memcpy(packet+1,block.samples,block.n*2);
                if(!stream_ready || pipeline_finished || !ws_connected) continue;
                if(esp_websocket_client_send_bin(client,packet,1+block.n*2,pdMS_TO_TICKS(500))<0) break;
            }
            uint32_t now=xTaskGetTickCount()*portTICK_PERIOD_MS;
            if(heard_speech && now-last_voice>SILENCE_TIMEOUT_MS) break;
        }
        if(stream_ready && stt_handler_registered && ws_connected && !pipeline_finished) {
            char end=(char)audio_channel;
            stream_ready=false;
            ESP_LOGI(TAG,"Sending STT EOF handler=%u",(unsigned)audio_channel);
            (void)esp_websocket_client_send_bin(client,&end,1,pdMS_TO_TICKS(1000));
        }
        if(ws_connected && !pipeline_finished && !muted) {
            state_set("processing");
            uint32_t wait=xTaskGetTickCount()*portTICK_PERIOD_MS;
            while(!pipeline_finished && ws_connected && !muted &&
                  xTaskGetTickCount()*portTICK_PERIOD_MS-wait<20000)
                vTaskDelay(pdMS_TO_TICKS(100));
        }
cleanup:
        stream_ready=false;stt_handler_registered=false;
        esp_websocket_client_stop(client);
        esp_websocket_client_destroy(client);
        client=NULL;
        session_active=false;
        mishka_assist_on_finish();
        if(muted)state_set("muted");
        else if(!pipeline_finished)state_set("timeout_or_disconnected");
    }
}
void mishka_assist_init(void) {
    nvs_handle_t n;
    if(nvs_open("speaker_cfg",NVS_READONLY,&n)==ESP_OK) {
        size_t len=sizeof(token);nvs_get_str(n,"ha_token",token,&len);
        len=sizeof(host);nvs_get_str(n,"ha_host",host,&len);
        len=sizeof(pipeline);nvs_get_str(n,"ha_pipe",pipeline,&len);
        int32_t p=0;if(nvs_get_i32(n,"ha_port",&p)==ESP_OK && p>0 && p<65536)port=p;
        nvs_close(n);
    }
    pcm_queue=xQueueCreate(48,sizeof(audio_chunk_t));
    if(!pcm_queue || xTaskCreate(assist_task,"mishka_assist",8192,NULL,4,&worker)!=pdPASS)
        state_set("task_error");
    else state_set(token[0]?"ready":"not_configured");
}
