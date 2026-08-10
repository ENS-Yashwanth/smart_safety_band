#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs.h"
#include "driver/i2s_std.h"

#include "gl868_modem.h"
#include "safety_shared.h"

static const char *TAG = "VOICE_CALL";

#define VOICE_CALL_RETRY_DELAY_MS       10000
#define VOICE_CALL_SETUP_TIMEOUT_MS     45000
#define VOICE_CALL_MAX_DURATION_MS      300000
#define VOICE_CALL_POLL_MS               2000
#define VOICE_NUMBER_MAX                 48
#define VOICE_NUMBERS_MAX                4

/*
 * The SIM868 PCM pins must be wired directly to the selected codec's PCM/I2S
 * pins.  The ESP32 I2S interface is used only when the codec is also connected
 * to the ESP32 for local diagnostics/processing.  Override these pin macros in
 * the board configuration; the defaults deliberately leave it disconnected.
 */
#ifndef VOICE_CODEC_I2S_NUM
#define VOICE_CODEC_I2S_NUM I2S_NUM_0
#endif
#ifndef VOICE_CODEC_BCK_PIN
#define VOICE_CODEC_BCK_PIN GPIO_NUM_NC
#endif
#ifndef VOICE_CODEC_WS_PIN
#define VOICE_CODEC_WS_PIN GPIO_NUM_NC
#endif
#ifndef VOICE_CODEC_DATA_OUT_PIN
#define VOICE_CODEC_DATA_OUT_PIN GPIO_NUM_NC
#endif
#ifndef VOICE_CODEC_DATA_IN_PIN
#define VOICE_CODEC_DATA_IN_PIN GPIO_NUM_NC
#endif

typedef enum {
    CALL_STATE_SETUP,
    CALL_STATE_ACTIVE,
    CALL_STATE_ENDED,
    CALL_STATE_FAILED,
} call_state_t;

static TaskHandle_t s_voice_task;
static volatile bool s_cancel_requested;
static bool s_i2s_installed;
static i2s_chan_handle_t s_i2s_tx;
static i2s_chan_handle_t s_i2s_rx;

/* A board codec driver may override these hooks to program its I2C registers,
 * including AEC/NS. SIM868 has no generic AT command for codec DSP settings. */
bool __attribute__((weak)) safety_voice_codec_init(void) { return true; }
bool __attribute__((weak)) safety_voice_codec_configure(int microphone_gain_db,
                                                        int speaker_volume_percent,
                                                        bool aec_enabled, bool ns_enabled)
{
    (void)microphone_gain_db;
    (void)speaker_volume_percent;
    (void)aec_enabled;
    (void)ns_enabled;
    return true;
}
void __attribute__((weak)) safety_voice_codec_set_call_active(bool active) { (void)active; }
void __attribute__((weak)) safety_voice_codec_deinit(void) {}

static bool initialize_audio_codec(void)
{
    if (!safety_voice_codec_init()) {
        ESP_LOGE(TAG, "Board audio codec I2C initialization failed");
        return false;
    }

    /* Do not claim GPIO 0..3 on boards with a direct modem-to-codec PCM path. */
    if (VOICE_CODEC_BCK_PIN == GPIO_NUM_NC || VOICE_CODEC_WS_PIN == GPIO_NUM_NC ||
        VOICE_CODEC_DATA_OUT_PIN == GPIO_NUM_NC || VOICE_CODEC_DATA_IN_PIN == GPIO_NUM_NC) {
        ESP_LOGI(TAG, "PCM is modem-to-codec direct; ESP32 I2S diagnostic path disabled");
        return true;
    }

    i2s_chan_config_t channel_config = I2S_CHANNEL_DEFAULT_CONFIG(VOICE_CODEC_I2S_NUM, I2S_ROLE_MASTER);
    channel_config.dma_desc_num = 4;
    channel_config.dma_frame_num = 256;
    channel_config.auto_clear_after_cb = true;
    i2s_std_config_t config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(8000), /* GSM narrow-band PCM */
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = GPIO_NUM_NC,
            .bclk = VOICE_CODEC_BCK_PIN,
            .ws = VOICE_CODEC_WS_PIN,
            .dout = VOICE_CODEC_DATA_OUT_PIN,
            .din = VOICE_CODEC_DATA_IN_PIN,
        },
    };
    if (i2s_new_channel(&channel_config, &s_i2s_tx, &s_i2s_rx) != ESP_OK ||
        i2s_channel_init_std_mode(s_i2s_tx, &config) != ESP_OK ||
        i2s_channel_init_std_mode(s_i2s_rx, &config) != ESP_OK) {
        ESP_LOGE(TAG, "ESP32 codec I2S initialization failed");
        if (s_i2s_tx != NULL) i2s_del_channel(s_i2s_tx);
        if (s_i2s_rx != NULL) i2s_del_channel(s_i2s_rx);
        s_i2s_tx = NULL;
        s_i2s_rx = NULL;
        return false;
    }
    s_i2s_installed = true;
    return true;
}

static bool configure_audio_pipeline(void)
{
    char response[96] = {0};
    /* SIM868 controls its analogue microphone and earpiece gain. The codec hook
     * applies the corresponding hardware gain, volume, AEC and NS settings. */
    bool modem_ok = gl868_modem_send_at_command("AT+CMIC=0,10", response, sizeof(response), 3000);
    modem_ok &= gl868_modem_send_at_command("AT+CLVL=80", response, sizeof(response), 3000);
    modem_ok &= gl868_modem_send_at_command("AT+COLP=1", response, sizeof(response), 3000);
    if (!modem_ok) ESP_LOGW(TAG, "Modem audio gain configuration was not accepted");
    return safety_voice_codec_configure(10, 80, true, true) && modem_ok;
}

static bool valid_phone_number(const char *number)
{
    if (number == NULL || *number == '\0') return false;
    size_t digits = 0;
    for (const char *p = number; *p; ++p) {
        if ((*p == '+' && p == number) || (*p >= '0' && *p <= '9')) {
            if (*p != '+') ++digits;
            continue;
        }
        return false;
    }
    return digits >= 3 && strlen(number) < VOICE_NUMBER_MAX;
}

static size_t parse_numbers(const char *list, char numbers[][VOICE_NUMBER_MAX], size_t capacity)
{
    size_t count = 0;
    while (list && *list && count < capacity) {
        while (*list == ' ' || *list == ',' || *list == ';') ++list;
        const char *end = list;
        while (*end && *end != ',' && *end != ';') ++end;
        size_t len = (size_t)(end - list);
        while (len && list[len - 1] == ' ') --len;
        if (len && len < VOICE_NUMBER_MAX) {
            memcpy(numbers[count], list, len);
            numbers[count][len] = '\0';
            if (valid_phone_number(numbers[count])) ++count;
            else ESP_LOGW(TAG, "Ignoring invalid emergency phone number");
        }
        list = end;
    }
    return count;
}

static size_t read_emergency_numbers(char numbers[][VOICE_NUMBER_MAX], size_t capacity)
{
    char stored[VOICE_NUMBERS_MAX * VOICE_NUMBER_MAX] = {0};
    size_t stored_len = sizeof(stored);
    nvs_handle_t handle;
    esp_err_t err = nvs_open("safety", NVS_READONLY, &handle);
    if (err == ESP_OK) {
        err = nvs_get_str(handle, "emergency_numbers", stored, &stored_len);
        if (err != ESP_OK) {
            stored_len = sizeof(stored);
            err = nvs_get_str(handle, "emergency_number", stored, &stored_len);
        }
        nvs_close(handle);
    }
    if (err == ESP_OK) return parse_numbers(stored, numbers, capacity);

    ESP_LOGW(TAG, "No NVS emergency number list (%s); using configured fallback", esp_err_to_name(err));
    return parse_numbers(gl868_modem_get_emergency_call_number(), numbers, capacity);
}

static call_state_t get_call_state(void)
{
    char response[256] = {0};
    if (!gl868_modem_send_at_command("AT+CLCC?", response, sizeof(response), 5000)) {
        return CALL_STATE_FAILED;
    }
    if (strstr(response, "BUSY") || strstr(response, "NO CARRIER") || strstr(response, "NO ANSWER")) {
        return CALL_STATE_FAILED;
    }
    const char *call = strstr(response, "+CLCC:");
    if (!call) return CALL_STATE_ENDED;
    /* +CLCC: <idx>,<dir>,<stat>; 0 active, 2 dialing, 3 alerting, 6 disconnecting. */
    const char *first = strchr(call, ',');
    const char *second = first ? strchr(first + 1, ',') : NULL;
    if (!second) return CALL_STATE_FAILED;
    switch (second[1]) {
    case '0': return CALL_STATE_ACTIVE;
    case '2': case '3': case '4': return CALL_STATE_SETUP;
    default: return CALL_STATE_ENDED;
    }
}

static bool wait_for_call(void)
{
    TickType_t started = xTaskGetTickCount();
    bool became_active = false;
    while (!s_cancel_requested) {
        call_state_t state = get_call_state();
        if (state == CALL_STATE_ACTIVE) became_active = true;
        if (state == CALL_STATE_ENDED || state == CALL_STATE_FAILED) return false;
        TickType_t elapsed = xTaskGetTickCount() - started;
        if (!became_active && elapsed > pdMS_TO_TICKS(VOICE_CALL_SETUP_TIMEOUT_MS)) return false;
        if (became_active && elapsed > pdMS_TO_TICKS(VOICE_CALL_MAX_DURATION_MS)) return false;
        vTaskDelay(pdMS_TO_TICKS(VOICE_CALL_POLL_MS));
    }
    return false;
}

static void voice_call_task(void *arg)
{
    (void)arg;
    s_voice_task = xTaskGetCurrentTaskHandle();
    if (!initialize_audio_codec()) ESP_LOGW(TAG, "Voice calls remain available, but codec initialization failed");

    for (;;) {
        safety_event_t event;
        if (xQueueReceive(s_voice_events, &event, portMAX_DELAY) != pdTRUE) continue;
        char numbers[VOICE_NUMBERS_MAX][VOICE_NUMBER_MAX] = {{0}};
        size_t number_count = read_emergency_numbers(numbers, VOICE_NUMBERS_MAX);
        if (number_count == 0) {
            ESP_LOGE(TAG, "Emergency voice request rejected: no valid emergency numbers configured");
            continue;
        }

        s_cancel_requested = false;
        if (!configure_audio_pipeline()) {
            ESP_LOGW(TAG, "Voice codec filtering was not fully configured for this call");
        }
        ESP_LOGW(TAG, "Emergency voice request from %s", event.source ? event.source : "unknown");
        for (size_t attempt = 0; attempt < number_count && !s_cancel_requested; ++attempt) {
            if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(VOICE_CALL_RETRY_DELAY_MS));
            ESP_LOGI(TAG, "Dialing emergency contact %u of %u", (unsigned)(attempt + 1), (unsigned)number_count);
            if (gl868_modem_make_call_to(numbers[attempt])) {
                /* The physical SIM868 PCM <-> codec connection carries the full-duplex stream. */
                safety_voice_codec_set_call_active(true);
                (void)wait_for_call();
                safety_voice_codec_set_call_active(false);
            }
            gl868_modem_hangup_call();
        }
        ESP_LOGI(TAG, "Emergency voice session %s", s_cancel_requested ? "cancelled by user" : "complete");
    }
}

void voice_call_task_cancel(void)
{
    s_cancel_requested = true;
    if (s_voice_task != NULL) xTaskNotifyGive(s_voice_task);
}

void voice_call_task_start(void)
{
    xTaskCreate(voice_call_task, "voice_call", 8192, NULL, 8, &s_voice_task);
}
