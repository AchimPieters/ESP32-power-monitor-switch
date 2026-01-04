/**
   Copyright 2026 Achim Pieters | StudioPieters®

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   for more information visit https://www.studiopieters.nl
 **/

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <esp_log.h>
#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#include "esp32-lcm.h"
#include "custom_charcteristc.h"
#include <button.h>

// Step 4: BL0942 component is present and wired to HomeKit (UART).
// We initialize the driver and periodically push Voltage/Current/Power/Energy
// values into custom characteristics under the existing Outlet service.
#include "bl0942.h"

// -------- GPIO configuration (set these in sdkconfig) --------
#define BUTTON_GPIO      CONFIG_ESP_BUTTON_GPIO
#define SWITCH_GPIO      CONFIG_ESP_SWITCH_GPIO
#define RELAY_GPIO       CONFIG_ESP_RELAY_GPIO
#define BLUE_LED_GPIO    CONFIG_ESP_BLUE_LED_GPIO

static const char *RELAY_TAG   = "RELAY";
static const char *BUTTON_TAG  = "BUTTON";
static const char *IDENT_TAG   = "IDENT";
static const char *BL0942_TAG  = "BL0942";
static const char *ENERGY_TAG  = "ENERGY";

// Persisted cumulative energy so it survives reboots.
// We store energy in milli-Wh (mWh) to keep it compact and avoid float issues.
static const char *ENERGY_NVS_NAMESPACE = "bl0942";
static const char *ENERGY_NVS_KEY_MWH   = "energy_mwh";
static const char *CAL_NVS_KEY_VCAL     = "v_cal";
static const char *CAL_NVS_KEY_ICAL     = "i_cal";
static const char *CAL_NVS_KEY_PCAL     = "p_cal";

// Relay state (used by the HomeKit relay service and energy-meter task)
// NOTE: keep this definition above tasks that reference it.
static bool relay_on = false;

// Base cumulative energy loaded from NVS (mWh). Total energy = base + (energy since boot).
static uint64_t s_energy_mwh_base = 0;

// Flag set by the button handler when energy is reset. The measurement task
// uses it to re-sync its internal change tracking and force HomeKit updates.
static volatile bool s_energy_reset_requested = false;

static esp_err_t energy_load_from_nvs(void)
{
        nvs_handle_t nvs;
        esp_err_t err = nvs_open(ENERGY_NVS_NAMESPACE, NVS_READONLY, &nvs);
        if (err != ESP_OK) {
                ESP_LOGW(ENERGY_TAG, "NVS open (read) failed (%s). Using 0.", esp_err_to_name(err));
                s_energy_mwh_base = 0;
                return err;
        }

        uint64_t val = 0;
        err = nvs_get_u64(nvs, ENERGY_NVS_KEY_MWH, &val);
        nvs_close(nvs);

        if (err == ESP_OK) {
                s_energy_mwh_base = val;
                ESP_LOGI(ENERGY_TAG, "Loaded energy from NVS: %llu mWh", (unsigned long long)val);
                return ESP_OK;
        }

        if (err == ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGI(ENERGY_TAG, "No stored energy found in NVS yet (fresh install). Using 0.");
                s_energy_mwh_base = 0;
                return ESP_OK;
        }

        ESP_LOGW(ENERGY_TAG, "NVS get_u64 failed (%s). Using 0.", esp_err_to_name(err));
        s_energy_mwh_base = 0;
        return err;
}

static esp_err_t energy_save_to_nvs(uint64_t energy_mwh)
{
        nvs_handle_t nvs;
        esp_err_t err = nvs_open(ENERGY_NVS_NAMESPACE, NVS_READWRITE, &nvs);
        if (err != ESP_OK) {
                ESP_LOGW(ENERGY_TAG, "NVS open (write) failed (%s)", esp_err_to_name(err));
                return err;
        }

        err = nvs_set_u64(nvs, ENERGY_NVS_KEY_MWH, energy_mwh);
        if (err == ESP_OK) {
                err = nvs_commit(nvs);
        }
        nvs_close(nvs);

        if (err != ESP_OK) {
                ESP_LOGW(ENERGY_TAG, "NVS save failed (%s)", esp_err_to_name(err));
        }
        return err;
}

static void calibration_load_from_nvs(bl0942_config_t *cfg)
{
        if (!cfg) return;

        nvs_handle_t nvs;
        esp_err_t err = nvs_open(ENERGY_NVS_NAMESPACE, NVS_READONLY, &nvs);
        if (err != ESP_OK) {
                ESP_LOGW(ENERGY_TAG, "NVS open (cal read) failed (%s). Using menuconfig defaults.", esp_err_to_name(err));
                return;
        }

        float vcal = cfg->voltage_calibration;
        float ical = cfg->current_calibration;
        float pcal = cfg->power_calibration;

        size_t len = sizeof(float);
        if (nvs_get_blob(nvs, CAL_NVS_KEY_VCAL, &vcal, &len) != ESP_OK) vcal = cfg->voltage_calibration;
        len = sizeof(float);
        if (nvs_get_blob(nvs, CAL_NVS_KEY_ICAL, &ical, &len) != ESP_OK) ical = cfg->current_calibration;
        len = sizeof(float);
        if (nvs_get_blob(nvs, CAL_NVS_KEY_PCAL, &pcal, &len) != ESP_OK) pcal = cfg->power_calibration;

        nvs_close(nvs);

        // Clamp to sane ranges to prevent absurd Eve/Home displays.
        if (!isfinite(vcal) || vcal < 0.5f || vcal > 2.0f) vcal = cfg->voltage_calibration;
        if (!isfinite(ical) || ical < 0.5f || ical > 2.0f) ical = cfg->current_calibration;
        if (!isfinite(pcal) || pcal < 0.5f || pcal > 2.0f) pcal = cfg->power_calibration;

        cfg->voltage_calibration = vcal;
        cfg->current_calibration = ical;
        cfg->power_calibration   = pcal;

        ESP_LOGI(ENERGY_TAG, "Calibration (NVS/menuconfig): V=%.6f I=%.6f P=%.6f", (double)vcal, (double)ical, (double)pcal);
}

// BL0942 -> HomeKit update interval
// How often we read the BL0942 and push values to HomeKit.
// Can be overridden via menuconfig (StudioPieters -> BL0942 Update Period).
#if defined(CONFIG_ESP_BL0942_UPDATE_PERIOD_MS)
#define BL0942_UPDATE_PERIOD_MS CONFIG_ESP_BL0942_UPDATE_PERIOD_MS
#else
#ifndef BL0942_UPDATE_PERIOD_MS
#define BL0942_UPDATE_PERIOD_MS 1000
#endif
#endif

// These characteristics are defined later in this file (Step 3).
// Energy meter characteristics are defined in this file and attached to the Outlet service.
// Use these names everywhere to avoid duplicate/copy issues with homekit_characteristic_t.
extern homekit_characteristic_t energy_voltage;
extern homekit_characteristic_t energy_current;
extern homekit_characteristic_t energy_power;
extern homekit_characteristic_t energy_energy;

static void bl0942_homekit_task(void *arg)
{
        (void)arg;

        // Init + start driver (single place; do NOT init in app_main as well)
        bl0942_config_t cfg = bl0942_config_default();
        calibration_load_from_nvs(&cfg);

        esp_err_t err = bl0942_init(&cfg);
        if (err != ESP_OK) {
                ESP_LOGE(BL0942_TAG, "bl0942_init failed: %s", esp_err_to_name(err));
                vTaskDelete(NULL);
                return;
        }

        err = bl0942_start();
        if (err != ESP_OK) {
                ESP_LOGE(BL0942_TAG, "bl0942_start failed: %s", esp_err_to_name(err));
                vTaskDelete(NULL);
                return;
        }

        // Restore cumulative energy base from NVS.
        (void)energy_load_from_nvs();

        // Persist total energy every 60s (mWh integer), so it survives reboots.
        TickType_t next_persist = xTaskGetTickCount() + pdMS_TO_TICKS(60000);

        ESP_LOGI(ENERGY_TAG, "BL0942 -> HomeKit task started (period=%d ms)", BL0942_UPDATE_PERIOD_MS);

        // --- Eve/Home polish ---
        // 1) Smooth noisy signals (EMA)
        // 2) Round to stable decimals (Eve graphs hate raw jitter)
        // 3) Rate-limit notifications (avoid HomeKit notify spam)
        // 4) Threshold updates (only meaningful changes)
        const float EMA_ALPHA_V = 0.20f;
        const float EMA_ALPHA_I = 0.20f;
        const float EMA_ALPHA_P = 0.20f;

        // Notify thresholds
        const float V_DELTA = 0.5f;     // V
        const float I_DELTA = 0.02f;    // A
        const float P_DELTA = 1.0f;     // W
        const float E_DELTA = 0.001f;   // kWh (1Wh)

        // Notify pacing
        const int64_t MIN_NOTIFY_US = 2LL * 1000LL * 1000LL; // 2 seconds

        float v_f = NAN, c_f = NAN, p_f = NAN;
        float last_v = NAN, last_c = NAN, last_p = NAN, last_e_kwh = NAN;
        int64_t last_v_notify_us = 0, last_c_notify_us = 0, last_p_notify_us = 0, last_e_notify_us = 0;


        while (true) {
                if (s_energy_reset_requested) {
                        s_energy_reset_requested = false;

                        // Reset smoothing + change detection so the next loop publishes fresh values.
                        v_f = c_f = p_f = NAN;
                        last_v = last_c = last_p = last_e_kwh = NAN;
                        last_v_notify_us = last_c_notify_us = last_p_notify_us = last_e_notify_us = 0;

                        // Publish a zero snapshot immediately (Home app feedback).
                        energy_voltage.value = HOMEKIT_FLOAT(0.0f);
                        homekit_characteristic_notify(&energy_voltage, energy_voltage.value);

                        energy_current.value = HOMEKIT_FLOAT(0.0f);
                        homekit_characteristic_notify(&energy_current, energy_current.value);

                        energy_power.value = HOMEKIT_FLOAT(0.0f);
                        homekit_characteristic_notify(&energy_power, energy_power.value);

                        energy_energy.value = HOMEKIT_FLOAT(0.0f);
                        homekit_characteristic_notify(&energy_energy, energy_energy.value);
                }

                const bl0942_measurements_t m = bl0942_get();

                if (!m.valid && relay_on) {
                        ESP_LOGW(BL0942_TAG, "BL0942 readings invalid (check UART wiring/SEL/SCLK_BPS/baud)");
                }

                float v = (m.valid ? m.voltage_v : 0.0f);
                float c = (m.valid ? m.current_a : 0.0f);
                float p = (m.valid ? m.power_w   : 0.0f);

                // Clamp and sanitize (Eve/Home dislikes NaN/inf/negative spikes)
                if (!isfinite(v) || v < 0.0f) v = 0.0f;
                if (!isfinite(c) || c < 0.0f) c = 0.0f;
                if (!isfinite(p) || p < 0.0f) p = 0.0f;

                // Smooth (EMA) only when we have valid signals
                if (isnan(v_f)) v_f = v; else v_f = v_f * (1.0f - EMA_ALPHA_V) + v * EMA_ALPHA_V;
                if (isnan(c_f)) c_f = c; else c_f = c_f * (1.0f - EMA_ALPHA_I) + c * EMA_ALPHA_I;
                if (isnan(p_f)) p_f = p; else p_f = p_f * (1.0f - EMA_ALPHA_P) + p * EMA_ALPHA_P;

                // Rounding/formatting (stable UI)
                const float v_out = roundf(v_f * 10.0f) / 10.0f;         // 0.1V
                const float c_out = roundf(c_f * 1000.0f) / 1000.0f;     // 0.001A
                const float p_out = roundf(p_f);                         // 1W

                // Total energy: base (NVS) + since-boot counter (from driver)
                const uint64_t e_since_boot_mwh = (uint64_t)((double)m.energy_wh * 1000.0);
                const uint64_t e_total_mwh = s_energy_mwh_base + e_since_boot_mwh;
                float e_total_kwh = (float)e_total_mwh / 1000000.0f;     // mWh -> kWh
                if (!isfinite(e_total_kwh) || e_total_kwh < 0.0f) e_total_kwh = 0.0f;
                e_total_kwh = roundf(e_total_kwh * 1000.0f) / 1000.0f;   // 0.001 kWh

                const int64_t now_us = esp_timer_get_time();

                // Update HomeKit characteristics only when meaningful AND rate-limited.
                if (isnan(last_v) || (fabsf(v_out - last_v) >= V_DELTA && (now_us - last_v_notify_us) >= MIN_NOTIFY_US)) {
                        last_v = v_out;
                        last_v_notify_us = now_us;
                        energy_voltage.value = HOMEKIT_FLOAT(v_out);
                        homekit_characteristic_notify(&energy_voltage, energy_voltage.value);
                }

                if (isnan(last_c) || (fabsf(c_out - last_c) >= I_DELTA && (now_us - last_c_notify_us) >= MIN_NOTIFY_US)) {
                        last_c = c_out;
                        last_c_notify_us = now_us;
                        energy_current.value = HOMEKIT_FLOAT(c_out);
                        homekit_characteristic_notify(&energy_current, energy_current.value);
                }

                if (isnan(last_p) || (fabsf(p_out - last_p) >= P_DELTA && (now_us - last_p_notify_us) >= MIN_NOTIFY_US)) {
                        last_p = p_out;
                        last_p_notify_us = now_us;
                        energy_power.value = HOMEKIT_FLOAT(p_out);
                        homekit_characteristic_notify(&energy_power, energy_power.value);
                }

                if (isnan(last_e_kwh) || (fabsf(e_total_kwh - last_e_kwh) >= E_DELTA && (now_us - last_e_notify_us) >= MIN_NOTIFY_US)) {
                        last_e_kwh = e_total_kwh;
                        last_e_notify_us = now_us;
                        energy_energy.value = HOMEKIT_FLOAT(e_total_kwh);
                        homekit_characteristic_notify(&energy_energy, energy_energy.value);
                }

                // Persist cumulative energy periodically so it survives a reboot.
                if (xTaskGetTickCount() >= next_persist) {
                        next_persist = xTaskGetTickCount() + pdMS_TO_TICKS(60000);
                        // Save the current total. Do NOT change s_energy_mwh_base during runtime;
                        // it is the "energy at boot" base. This avoids double counting.
                        (void)energy_save_to_nvs(e_total_mwh);
                }

                vTaskDelay(pdMS_TO_TICKS(BL0942_UPDATE_PERIOD_MS));
        }
}

// ---------- Low-level GPIO helpers ----------

static inline void relay_write(bool on) {
        gpio_set_level(RELAY_GPIO, on ? 1 : 0);
}

static inline void blue_led_write(bool on) {
        // Blauwe LED: uitsluitend als aan/uit-indicator voor de relay (active low/high afhankelijk van hardware)
        gpio_set_level(BLUE_LED_GPIO, on ? 1 : 0);
}

// 3x korte blauwe LED blink (zonder de relay status te veranderen)
static void blink_blue_led_3x(void) {
        bool previous_led_state = relay_on;

        for (int i = 0; i < 3; i++) {
                blue_led_write(true);
                vTaskDelay(pdMS_TO_TICKS(100));
                blue_led_write(false);
                vTaskDelay(pdMS_TO_TICKS(100));
        }

        // Restore normal behavior (LED follows relay state)
        blue_led_write(previous_led_state);
}

// Forward declaration van de characteristic zodat we hem in functies kunnen gebruiken
extern homekit_characteristic_t relay_on_characteristic;

// Centrale functie: zet state, stuurt hardware aan en (optioneel) HomeKit-notify
static void relay_set_state(bool on, bool notify_homekit) {
        if (relay_on == on) {
                // Geen verandering, niets te doen
                return;
        }

        relay_on = on;

        // Hardware aansturen
        relay_write(relay_on);
        blue_led_write(relay_on); // Blauwe LED volgt altijd de relay-status

        ESP_LOGI(RELAY_TAG, "Relay state -> %s", relay_on ? "ON" : "OFF");

        // HomeKit characteristic-snapshot updaten
        relay_on_characteristic.value = HOMEKIT_BOOL(relay_on);

        // Eventueel HomeKit-clients informeren
        if (notify_homekit) {
                homekit_characteristic_notify(&relay_on_characteristic,
                                              relay_on_characteristic.value);
        }
}

// All GPIO Settings
void gpio_init(void) {
        // Relay
        gpio_reset_pin(RELAY_GPIO);
        gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);

        // Blauwe LED (aan/uit)
        gpio_reset_pin(BLUE_LED_GPIO);
        gpio_set_direction(BLUE_LED_GPIO, GPIO_MODE_OUTPUT);

        // Switch input (optioneel)
        gpio_reset_pin(SWITCH_GPIO);
        gpio_set_direction(SWITCH_GPIO, GPIO_MODE_INPUT);
        gpio_set_pull_mode(SWITCH_GPIO, GPIO_PULLUP_ONLY);

        // Initial state: alles uit, in sync brengen
        relay_on = false;
        relay_on_characteristic.value = HOMEKIT_BOOL(false);
        relay_write(false);
        blue_led_write(false);

        // Bij start is er nog geen WiFi -> status LED in WiFi-wachtstand
}

// ---------- Accessory identification (Blue LED) ----------

void accessory_identify_task(void *args) {
        // Blink BLUE LED to identify, then restore previous state
        bool previous_led_state = relay_on; // LED volgt normaal relay_on

        for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 2; j++) {
                        blue_led_write(true);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        blue_led_write(false);
                        vTaskDelay(pdMS_TO_TICKS(100));
                }
                vTaskDelay(pdMS_TO_TICKS(250));
        }

        // Zet LED terug naar de normale toestand (afhankelijk van relay_on)
        blue_led_write(previous_led_state);

        vTaskDelete(NULL);
}

void accessory_identify(homekit_value_t _value) {
        ESP_LOGI(IDENT_TAG, "Accessory identify");
        xTaskCreate(accessory_identify_task, "Accessory identify", configMINIMAL_STACK_SIZE,
                    NULL, 2, NULL);
}

// ---------- HomeKit characteristics ----------

#define DEVICE_NAME          "HomeKit Plug"
#define DEVICE_MANUFACTURER  "StudioPieters®"
#define DEVICE_SERIAL        "NLCC7DFD193A"
#define DEVICE_MODEL         "LS066NL/A"
#define FW_VERSION           "0.0.1"

homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, DEVICE_NAME);
homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER, DEVICE_MANUFACTURER);
homekit_characteristic_t serial = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, DEVICE_SERIAL);
homekit_characteristic_t model = HOMEKIT_CHARACTERISTIC_(MODEL, DEVICE_MODEL);
homekit_characteristic_t revision = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION, LIFECYCLE_DEFAULT_FW_VERSION);
homekit_characteristic_t ota_trigger = API_OTA_TRIGGER;

// Getter: HomeKit vraagt huidige toestand op
homekit_value_t relay_on_get() {
        return HOMEKIT_BOOL(relay_on);
}

// Setter: aangeroepen door HomeKit (Home-app / Siri / automations)
void relay_on_set(homekit_value_t value) {
        if (value.format != homekit_format_bool) {
                ESP_LOGE(RELAY_TAG, "Invalid value format: %d", value.format);
                return;
        }

        bool new_state = value.bool_value;

        // Via centrale functie, maar ZONDER notify (originator is HomeKit zelf)
        relay_set_state(new_state, false);
}

// We keep a handle to ON characteristic so we can notify on button presses
homekit_characteristic_t relay_on_characteristic =
        HOMEKIT_CHARACTERISTIC_(ON, false, .getter = relay_on_get, .setter = relay_on_set);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverride-init"
homekit_accessory_t *accessories[] = {
        HOMEKIT_ACCESSORY(
                .id = 1,
                .category = homekit_accessory_category_Switches, // Smart plug / outlet
                .services = (homekit_service_t *[]) {
                HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics = (homekit_characteristic_t *[]) {
                        &name,
                        &manufacturer,
                        &serial,
                        &model,
                        &revision,
                        HOMEKIT_CHARACTERISTIC(IDENTIFY, accessory_identify),
                        NULL
                }),
                HOMEKIT_SERVICE(SWITCH, .primary = true, .characteristics = (homekit_characteristic_t *[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "HomeKit Switch"),
                        &relay_on_characteristic,
                        &ota_trigger,
                        &energy_voltage,
                        &energy_current,
                        &energy_power,
                        &energy_energy,
                        NULL
                }),
                NULL
        }),
        NULL
};
#pragma GCC diagnostic pop

homekit_server_config_t config = {
        .accessories = accessories,
        .password = CONFIG_ESP_SETUP_CODE,
        .setupId = CONFIG_ESP_SETUP_ID,
};

// ---------- Button handling ----------

void button_callback(button_event_t event, void *context) {
        switch (event) {
        case button_event_single_press: {
                ESP_LOGI(BUTTON_TAG, "Single press -> toggle relay");

                bool new_state = !relay_on;

                // 1) Zelfde logica als HomeKit, maar nu MET notify
                relay_set_state(new_state, true);

                break;
        }
        case button_event_double_press:
                // Reset cumulative energy without wiping the whole device.
                ESP_LOGI(BUTTON_TAG, "Double press -> reset energy total");
                s_energy_mwh_base = 0;
                (void)energy_save_to_nvs(0);

                // Reset BL0942's internal accumulator. The periodic measurement task
                // will push the updated (0.0) energy value to HomeKit on the next tick.
                bl0942_reset_energy();
                s_energy_reset_requested = true;

                // Visual confirmation for the user (same idea as Identify)
                blink_blue_led_3x();
                break;
        case button_event_long_press:
                ESP_LOGI(BUTTON_TAG, "Long press (10s) -> factory reset + reboot");
                lifecycle_factory_reset_and_reboot();
                break;
        default:
                ESP_LOGI(BUTTON_TAG, "Unknown button event: %d", event);
                break;
        }
}

// ---------- Wi-Fi / HomeKit startup ----------

void on_wifi_ready() {
        static bool homekit_started = false;

        if (homekit_started) {
                ESP_LOGI("INFORMATION", "HomeKit server already running; skipping re-initialization");
                return;
        }

        ESP_LOGI("INFORMATION", "Starting HomeKit server...");
        homekit_server_init(&config);
        homekit_started = true;
}

// ---------- app_main ----------

void app_main(void) {
        ESP_ERROR_CHECK(lifecycle_nvs_init());
        lifecycle_log_post_reset_state("INFORMATION");
        ESP_ERROR_CHECK(lifecycle_configure_homekit(&revision, &ota_trigger, "INFORMATION"));

        gpio_init();

#if CONFIG_BL0942_ENABLE
        // Start the BL0942 measurement + HomeKit publisher task
        xTaskCreate(bl0942_homekit_task, "bl0942_hk", 4096, NULL, 5, NULL);
#else
        ESP_LOGI(BL0942_TAG, "BL0942 disabled in sdkconfig (CONFIG_BL0942_ENABLE=n)");
#endif

        button_config_t btn_cfg = button_config_default(button_active_low);
        btn_cfg.max_repeat_presses = 3;
        btn_cfg.long_press_time = 10000; // 10 seconds for lifecycle_factory_reset_and_reboot

        if (button_create(BUTTON_GPIO, btn_cfg, button_callback, NULL)) {
                ESP_LOGE(BUTTON_TAG, "Failed to initialize button");
        }

        esp_err_t wifi_err = wifi_start(on_wifi_ready);
        if (wifi_err == ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGW("WIFI", "WiFi configuration not found; provisioning required");
                // Geen geldige WiFi-config -> status LED WiFi-wachtstand
        } else if (wifi_err != ESP_OK) {
                ESP_LOGE("WIFI", "Failed to start WiFi: %s", esp_err_to_name(wifi_err));
                // Fout bij starten WiFi -> status LED WiFi-wachtstand
        }
}
