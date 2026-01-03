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
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/

#include "bl0942.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// BL0942 packet mode (UART) summary (datasheet section 3.2.6):
// MCU sends 2 bytes: READ_CMD + 0xAA, then BL0942 responds 23 bytes starting with 0x55.
// We implement checksum verification and convert I/V/P + energy.
//
// Datasheet formulas we use:
// - I_RMS pin-voltage (mV):  I_pin_mV = I_RMS_raw * Vref / 305978   (datasheet 2.5)
// - V_RMS pin-voltage (mV):  V_pin_mV = V_RMS_raw * Vref / 73989    (datasheet 2.5)
// - Power factor estimate from WATT_raw: PF = (WATT_raw * Vref^2) / (3537 * I_pin_mV * V_pin_mV) (datasheet 2.2)
// - Active energy: use CF_CNT delta and tCF relation (datasheet 2.4):
//      tCF = 1638.4 * 256 / WATT_raw  (seconds, proportional). This implies constant energy per CF pulse.
//      If P_line = k * WATT_raw, then Wh_per_pulse = (k * 1638.4 * 256) / 3600.

static const char *TAG = "BL0942";

typedef struct {
        bl0942_config_t cfg;

        TaskHandle_t task;
        SemaphoreHandle_t lock;

        bl0942_measurements_t last;
        uint32_t last_cf_cnt;
        bool have_cf_cnt;

        double energy_wh_accum;
        double last_k_w_per_raw; // k in W per WATT_raw for energy-per-pulse estimation
        bool running;
} bl0942_state_t;

static bl0942_state_t s;

static uint8_t bl0942_uart_read_cmd(uint8_t ic_addr)
{
        // {0,1,0,1,1,0,A2,A1} where (A2,A1) encode ic_addr (0..3)
        // Base is 0b01011000 = 0x58 (addr 0).
        return (uint8_t)(0x58u | ((ic_addr & 0x03u) << 0));
}

static uint8_t checksum_inverted_sum(const uint8_t *buf, size_t len)
{
        uint32_t sum = 0;
        for (size_t i = 0; i < len; i++) sum += buf[i];
        return (uint8_t)(~(sum & 0xFFu));
}

static void decode_packet_locked(const uint8_t *pkt23)
{
        // pkt23[0] = 0x55
        bl0942_measurements_t m = {0};

        m.i_rms_raw = (uint32_t)pkt23[1] | ((uint32_t)pkt23[2] << 8) | ((uint32_t)pkt23[3] << 16);
        m.v_rms_raw = (uint32_t)pkt23[4] | ((uint32_t)pkt23[5] << 8) | ((uint32_t)pkt23[6] << 16);
        m.i_fast_rms_raw = (uint32_t)pkt23[7] | ((uint32_t)pkt23[8] << 8) | ((uint32_t)pkt23[9] << 16);

        uint32_t watt_u = (uint32_t)pkt23[10] | ((uint32_t)pkt23[11] << 8) | ((uint32_t)pkt23[12] << 16);
        // 24-bit signed
        if (watt_u & 0x00800000u) {
                watt_u |= 0xFF000000u;
        }
        m.watt_raw = (int32_t)watt_u;

        m.cf_cnt_raw = (uint32_t)pkt23[13] | ((uint32_t)pkt23[14] << 8) | ((uint32_t)pkt23[15] << 16);

        m.freq_raw = (uint16_t)pkt23[16] | ((uint16_t)pkt23[17] << 8);
        m.status_raw = (uint16_t)pkt23[19] | ((uint16_t)pkt23[20] << 8);

        // Convert to physical units
        const double Vref = (s.cfg.vref_v > 0.0f) ? s.cfg.vref_v : 1.218;
        const double shunt_ohm = (s.cfg.shunt_uohm > 0) ? ((double)s.cfg.shunt_uohm * 1e-6) : 0.0;
        const double v_div = (s.cfg.v_div_ratio > 0.0f) ? s.cfg.v_div_ratio : 0.0;

        const double i_pin_mV = (double)m.i_rms_raw * Vref / 305978.0;
        const double v_pin_mV = (double)m.v_rms_raw * Vref / 73989.0;

        // Guard against invalid board params
        double current_a = NAN;
        double voltage_v = NAN;

        if (shunt_ohm > 0.0) {
                const double v_shunt_v = i_pin_mV / 1000.0; // mV -> V
                current_a = v_shunt_v / shunt_ohm;
        }
        if (v_div > 0.0) {
                const double v_pin_v = v_pin_mV / 1000.0;
                voltage_v = v_pin_v / v_div;
        }

        // Power factor from WATT_raw if inputs available
        double pf = NAN;
        if (fabs(i_pin_mV) > 1e-9 && fabs(v_pin_mV) > 1e-9) {
                pf = ((double)m.watt_raw * (Vref * Vref)) / (3537.0 * i_pin_mV * v_pin_mV);
                if (pf > 1.2) pf = 1.2;
                if (pf < -1.2) pf = -1.2;
        }

        double power_w = NAN;
        if (!isnan(voltage_v) && !isnan(current_a) && !isnan(pf)) {
                power_w = voltage_v * current_a * pf;
        }

        // Apply calibration trims (multiplicative)
        if (!isnan(voltage_v)) voltage_v *= (double)s.cfg.voltage_calibration;
        if (!isnan(current_a)) current_a *= (double)s.cfg.current_calibration;
        if (!isnan(power_w)) power_w *= (double)s.cfg.power_calibration;

        m.voltage_v = (float)voltage_v;
        m.current_a = (float)current_a;
        m.power_w = (float)power_w;

        m.valid_voltage = isfinite(m.voltage_v) && (m.v_rms_raw != 0);
        m.valid_current = isfinite(m.current_a) && (m.i_rms_raw != 0);
        m.valid_power = isfinite(m.power_w) && (m.watt_raw != 0);

        m.valid = m.valid_voltage || m.valid_current || m.valid_power;

        // Energy accumulation using CF_CNT delta
        if (s.have_cf_cnt) {
                const uint32_t prev = s.last_cf_cnt;
                const uint32_t now = m.cf_cnt_raw;
                const uint32_t delta = (now >= prev) ? (now - prev) : (0x01000000u - prev + now); // 24-bit wrap

                // Estimate k = W / raw-watt from current sample if possible
                if (m.valid_power && m.watt_raw != 0) {
                        s.last_k_w_per_raw = ((double)m.power_w) / (double)m.watt_raw;
                }

                if (delta > 0 && isfinite(s.last_k_w_per_raw) && s.last_k_w_per_raw != 0.0) {
                        const double wh_per_pulse = (s.last_k_w_per_raw * 1638.4 * 256.0) / 3600.0;
                        if (isfinite(wh_per_pulse) && wh_per_pulse > 0.0 && wh_per_pulse < 10.0) {
                                s.energy_wh_accum += wh_per_pulse * (double)delta;
                        }
                }
        } else {
            s.have_cf_cnt = true;
        }

        s.last_cf_cnt = m.cf_cnt_raw;
        m.energy_wh = (float)s.energy_wh_accum;

        s.last = m;
}

static esp_err_t uart_setup(const bl0942_config_t *cfg)
{
        uart_config_t uart_config = {
                .baud_rate = cfg->uart_baud,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                .source_clk = UART_SCLK_DEFAULT,
        };

        ESP_ERROR_CHECK(uart_driver_install(cfg->uart_num, 256, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(cfg->uart_num, &uart_config));

        if (cfg->uart_tx_gpio >= 0 && cfg->uart_rx_gpio >= 0) {
                ESP_ERROR_CHECK(uart_set_pin(cfg->uart_num, cfg->uart_tx_gpio, cfg->uart_rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        }

        // Half-duplex line discipline is handled by BL0942; we only TX a small request and read response.
        return ESP_OK;
}

static void bl0942_task(void *arg)
{
        (void)arg;

        const TickType_t tick = pdMS_TO_TICKS(s.cfg.sample_period_ms > 50 ? s.cfg.sample_period_ms : 200);

        uint8_t req[2] = {0};
        req[0] = bl0942_uart_read_cmd(s.cfg.ic_addr);
        req[1] = 0xAA;

        uint8_t pkt[23];

        while (s.running) {
                // Flush any noise from previous cycle
                uart_flush_input(s.cfg.uart_num);

                // Send request
                uart_write_bytes(s.cfg.uart_num, (const char *)req, sizeof(req));
                uart_wait_tx_done(s.cfg.uart_num, pdMS_TO_TICKS(50));

                // Read response (up to ~48ms at 4800bps per datasheet)
                int got = uart_read_bytes(s.cfg.uart_num, pkt, sizeof(pkt), pdMS_TO_TICKS(120));
                if (got == (int)sizeof(pkt) && pkt[0] == 0x55) {
                        // Verify checksum: checksum is last byte and is inverted sum of (READ_CMD + 0x55 + payload bytes ...)
                        uint8_t chk = pkt[22];
                        // Compose sum bytes: cmd + pkt[0..21]
                        uint8_t tmp[1 + 22];
                        tmp[0] = req[0];
                        memcpy(&tmp[1], pkt, 22);
                        uint8_t expect = checksum_inverted_sum(tmp, sizeof(tmp));

                        if (chk == expect) {
                                xSemaphoreTake(s.lock, portMAX_DELAY);
                                decode_packet_locked(pkt);
                                xSemaphoreGive(s.lock);
                        } else {
                                ESP_LOGW(TAG, "checksum mismatch (got 0x%02X expect 0x%02X)", chk, expect);
                        }
                } else {
                        // Partial read; leave last measurement as-is
                        // (No noisy logging to avoid spam when UART is not wired yet)
                }

                vTaskDelay(tick);
        }

        vTaskDelete(NULL);
}

bl0942_config_t bl0942_config_default(void)
{
        bl0942_config_t c = {
                .uart_num = UART_NUM_0,
                .uart_baud = 4800,
                .uart_tx_gpio = (gpio_num_t)-1,
                .uart_rx_gpio = (gpio_num_t)-1,

                .shunt_uohm = 1000,      // 1mΩ
                .v_div_ratio = 0.0f,
                .vref_v = 1.218f,

                .sample_period_ms = 400,

                .voltage_calibration = 1.0f,
                .current_calibration = 1.0f,
                .power_calibration = 1.0f,

                .ic_addr = 0,
        };
        return c;
}

esp_err_t bl0942_init(const bl0942_config_t *cfg)
{
        if (!cfg) return ESP_ERR_INVALID_ARG;
        if (s.lock) return ESP_ERR_INVALID_STATE;

        memset(&s, 0, sizeof(s));
        s.cfg = *cfg;

        if (s.cfg.uart_baud < 4800) s.cfg.uart_baud = 4800;
        if (s.cfg.uart_baud > 38400) s.cfg.uart_baud = 38400;
        s.cfg.ic_addr &= 0x03;

        s.lock = xSemaphoreCreateMutex();
        if (!s.lock) return ESP_ERR_NO_MEM;

        s.energy_wh_accum = 0.0;
        s.last_k_w_per_raw = NAN;

        ESP_RETURN_ON_ERROR(uart_setup(&s.cfg), TAG, "uart_setup failed");

        return ESP_OK;
}

esp_err_t bl0942_start(void)
{
        if (!s.lock) return ESP_ERR_INVALID_STATE;
        if (s.task) return ESP_OK;

        s.running = true;
        BaseType_t ok = xTaskCreate(bl0942_task, "bl0942", 3072, NULL, 5, &s.task);
        return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t bl0942_stop(void)
{
        if (!s.lock) return ESP_ERR_INVALID_STATE;

        s.running = false;
        // Task will self-delete; we just clear handle after a short delay.
        vTaskDelay(pdMS_TO_TICKS(20));
        s.task = NULL;
        return ESP_OK;
}

esp_err_t bl0942_deinit(void)
{
        if (!s.lock) return ESP_ERR_INVALID_STATE;

        bl0942_stop();
        uart_driver_delete(s.cfg.uart_num);

        vSemaphoreDelete(s.lock);
        memset(&s, 0, sizeof(s));
        return ESP_OK;
}

bl0942_measurements_t bl0942_get(void)
{
        bl0942_measurements_t out = {0};
        if (!s.lock) return out;

        xSemaphoreTake(s.lock, portMAX_DELAY);
        out = s.last;
        xSemaphoreGive(s.lock);
        return out;
}

void bl0942_reset_energy(void)
{
        if (!s.lock) return;
        xSemaphoreTake(s.lock, portMAX_DELAY);
        s.energy_wh_accum = 0.0;
        s.have_cf_cnt = false;
        s.last_cf_cnt = 0;
        s.last.energy_wh = 0.0f;
        xSemaphoreGive(s.lock);
}
