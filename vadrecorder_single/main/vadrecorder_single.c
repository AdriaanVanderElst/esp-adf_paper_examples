/* Trigger recording of sample audio using Voice Activity Detection and transfer to HTTP server
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "esp_http_client.h"
#include "sdkconfig.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_common.h"
#include "board.h"
#include "http_stream.h"
#include "i2s_stream.h"
#include "raw_stream.h"
#include "wav_encoder.h"
#include "esp_peripherals.h"
#include "periph_wifi.h"
#include "audio_idf_version.h"
#include "esp_vad.h"
#include "esp_timer.h"
#include "esp_psram.h"

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 1, 0))
#include "esp_netif.h"
#else
#include "tcpip_adapter.h"
#endif

static const char *TAG = "VADRECORDER";

#define RECORD_SAMPLE_RATE 44100
#define RECORD_BIT_DEPTH 16
#define RECORD_AUDIO_CHANNELS 1
#define SILENCE_PERIOD 3000
#define BUFFERING_DELAY 500
#define VAD_AGGREGATION_COUNT 5

#define VAD_SAMPLE_RATE_HZ 16000
#define VAD_FRAME_LENGTH_MS 20
#define VAD_BUFFER_LENGTH (VAD_FRAME_LENGTH_MS * VAD_SAMPLE_RATE_HZ / 1000)

audio_pipeline_handle_t pipeline;
audio_element_handle_t i2s_stream_reader;
audio_element_handle_t http_stream_writer;
vad_handle_t vad_inst; // VAD handle

// HTTP stream event handler
esp_err_t _http_stream_event_handle(http_stream_event_msg_t *msg)
{
    esp_http_client_handle_t http = (esp_http_client_handle_t)msg->http_client;
    char len_buf[16];
    static int total_write = 0;

    if (msg->event_id == HTTP_STREAM_PRE_REQUEST)
    {
        // set header
        ESP_LOGI(TAG, "[ + ] HTTP client HTTP_STREAM_PRE_REQUEST, lenght=%d", msg->buffer_len);
        esp_http_client_set_method(http, HTTP_METHOD_POST);
        char dat[10] = {0};
        snprintf(dat, sizeof(dat), "%d", VAD_SAMPLE_RATE_HZ);
        esp_http_client_set_header(http, "x-audio-sample-rates", dat);
        memset(dat, 0, sizeof(dat));
        snprintf(dat, sizeof(dat), "%d", RECORD_BIT_DEPTH);
        esp_http_client_set_header(http, "x-audio-bits", dat);
        memset(dat, 0, sizeof(dat));
        snprintf(dat, sizeof(dat), "%d", RECORD_AUDIO_CHANNELS);
        esp_http_client_set_header(http, "x-audio-channel", dat);
        total_write = 0;
        return ESP_OK;
    }

    if (msg->event_id == HTTP_STREAM_ON_REQUEST)
    {
        // write data
        int wlen = sprintf(len_buf, "%x\r\n", msg->buffer_len);
        if (esp_http_client_write(http, len_buf, wlen) <= 0)
        {
            return ESP_FAIL;
        }
        if (esp_http_client_write(http, msg->buffer, msg->buffer_len) <= 0)
        {
            return ESP_FAIL;
        }
        if (esp_http_client_write(http, "\r\n", 2) <= 0)
        {
            return ESP_FAIL;
        }
        total_write += msg->buffer_len;
        printf("\033[A\33[2K\rTotal bytes written: %d\n", total_write);
        return msg->buffer_len;
    }

    if (msg->event_id == HTTP_STREAM_POST_REQUEST)
    {
        ESP_LOGI(TAG, "[ + ] HTTP client HTTP_STREAM_POST_REQUEST, write end chunked marker");
        if (esp_http_client_write(http, "0\r\n\r\n", 5) <= 0)
        {
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    if (msg->event_id == HTTP_STREAM_FINISH_REQUEST)
    {
        ESP_LOGI(TAG, "[ + ] HTTP client HTTP_STREAM_FINISH_REQUEST");
        char *buf = calloc(1, 64);
        assert(buf);
        int read_len = esp_http_client_read(http, buf, 64);
        if (read_len <= 0)
        {
            free(buf);
            return ESP_FAIL;
        }
        buf[read_len] = 0;
        ESP_LOGI(TAG, "Got HTTP Response = %s", (char *)buf);
        free(buf);
        return ESP_OK;
    }
    return ESP_OK;
}

// Check if the server is running
bool is_server_available(const char *server_uri)
{
    esp_http_client_config_t config = {
        .url = server_uri,
        .method = HTTP_METHOD_HEAD,
        .timeout_ms = 2000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);
    esp_http_client_cleanup(client);

    return (err == ESP_OK);
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    // esp_psram_init();

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

// Initialize Network Interface
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 1, 0))
    ESP_ERROR_CHECK(esp_netif_init());
#else
    tcpip_adapter_init();
#endif

    ESP_LOGI(TAG, "[ 1 ] Initialize Peripherals & Connect to wifi network");
    // Initialize peripherals management
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    periph_wifi_cfg_t wifi_cfg = {
        .wifi_config.sta.ssid = CONFIG_WIFI_SSID,
        .wifi_config.sta.password = CONFIG_WIFI_PASSWORD,
    };
    esp_periph_handle_t wifi_handle = periph_wifi_init(&wifi_cfg);

    // Start wifi peripheral
    esp_periph_start(set, wifi_handle);
    periph_wifi_wait_for_connected(wifi_handle, portMAX_DELAY);

    ESP_LOGI(TAG, "[ 2 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_ENCODE, AUDIO_HAL_CTRL_START);

    ESP_LOGI(TAG, "[3.0] Create audio pipeline for recording");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG, "[3.1] Create http stream to post data to server");
    http_stream_cfg_t http_cfg = HTTP_STREAM_CFG_DEFAULT();
    http_cfg.type = AUDIO_STREAM_WRITER;
    http_cfg.event_handle = _http_stream_event_handle;
    http_stream_writer = http_stream_init(&http_cfg);

    ESP_LOGI(TAG, "[3.2] Create i2s stream to read audio data from codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(CODEC_ADC_I2S_PORT, VAD_SAMPLE_RATE_HZ, RECORD_BIT_DEPTH, AUDIO_STREAM_READER);
    i2s_cfg.out_rb_size = 32 * 1024;
    // i2s_stream_set_channel_type(&i2s_cfg, I2S_CHANNEL_TYPE_ONLY_LEFT); // fixes sample rate issue when recording audio
    i2s_stream_reader = i2s_stream_init(&i2s_cfg);

    ESP_LOGI(TAG, "[3.3] Register all elements to pipeline");
    audio_pipeline_register(pipeline, i2s_stream_reader, "i2s");
    audio_pipeline_register(pipeline, http_stream_writer, "http");

    ESP_LOGI(TAG, "[3.4] Link elements together [codec_chip]-->i2s_stream-->http_stream-->>[http-server]");

    const char *link_tag_http[2] = {"i2s", "http"};
    audio_pipeline_link(pipeline, &link_tag_http[0], 2);

    ESP_LOGI(TAG, "[ 4 ] initialize VAD handle");
    vad_inst = vad_create(VAD_MODE_4);

    // Wait for the server to be available
    ESP_LOGI(TAG, "Waiting for server to be available...");
    while (!is_server_available(CONFIG_SERVER_URI))
    {
        ESP_LOGW(TAG, "Server not available, retrying...");
        vTaskDelay(pdMS_TO_TICKS(4000));
    }
    ESP_LOGI(TAG, "Server available, starting recording process...");

    ESP_LOGI(TAG, "[ 5 ] Monitoring voice activity and recording");
    int16_t *vad_buff = (int16_t *)malloc(VAD_BUFFER_LENGTH * sizeof(short));
    if (vad_buff == NULL)
    {
        ESP_LOGE(TAG, "Memory allocation failed!");
        goto abort_speech_detection;
    }

    i2s_stream_set_clk(i2s_stream_reader, VAD_SAMPLE_RATE_HZ, RECORD_BIT_DEPTH, RECORD_AUDIO_CHANNELS);

    bool audio_pipeline_is_running = false;
    int64_t silence_start_time = 0;

    while (true)
    {
        // read data from i2s stream
        int bytes_read = raw_stream_read(i2s_stream_reader, (char *)vad_buff, VAD_BUFFER_LENGTH * sizeof(short));
        if (bytes_read <= 0)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Detect voice activity
        vad_state_t vad_state = vad_process(vad_inst, vad_buff, VAD_SAMPLE_RATE_HZ, VAD_FRAME_LENGTH_MS);
        bool voice_active = vad_state == VAD_SPEECH;

        // Code to debug the VAD
        // if (voice_active) {
        //     ESP_LOGI(TAG, "Voice detected");
        // }
        // continue;

        if (voice_active)
        {
            silence_start_time = 0;

            // if pipeline is not running, start recording
            if (!audio_pipeline_is_running)
            {
                ESP_LOGI(TAG, "Detected voice activity");
                ESP_LOGI(TAG, "Starting recording...");
                audio_element_set_uri(http_stream_writer, CONFIG_SERVER_URI);
                audio_pipeline_run(pipeline);
                audio_pipeline_is_running = true;
            }
        }
        else
        {
            // Track silence
            if (audio_pipeline_is_running)
            {
                int64_t current_time = esp_timer_get_time() / 1000;

                if (silence_start_time == 0)
                {
                    silence_start_time = current_time;
                }
                else if (current_time - silence_start_time > SILENCE_PERIOD)
                {
                    ESP_LOGI(TAG, "Stopping recording due to detected silence");
                    audio_element_set_ringbuf_done(i2s_stream_reader);
                    audio_pipeline_stop(pipeline);
                    audio_pipeline_wait_for_stop(pipeline);
                    audio_pipeline_reset_ringbuffer(pipeline);
                    audio_pipeline_reset_elements(pipeline);
                    audio_pipeline_is_running = false;
                    vTaskDelay(pdMS_TO_TICKS(BUFFERING_DELAY));
                    silence_start_time = 0;
                }
            }
        }

        // Add a short delay to avoid excessive CPU usage
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    free(vad_buff);
    vad_buff = NULL;

abort_speech_detection:

    ESP_LOGI(TAG, "[ 6 ] Destroy VAD");
    vad_destroy(vad_inst);

    ESP_LOGI(TAG, "[ 7 ] Stop all resources");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);

    audio_pipeline_remove_listener(pipeline);
    esp_periph_set_stop_all(set);
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(http_stream_writer);
    audio_element_deinit(i2s_stream_reader);
    esp_periph_set_destroy(set);
}
