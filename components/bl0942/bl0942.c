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
 
#include "bl0942.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_check.h"
#include "sdkconfig.h"

static const char *TAG = "BL0942";

// UART protocol (datasheet v1.06, section 3.2)
//  write cmd: {1,0,1,0,1,0,A2,A1} -> 0xA8 | (addr & 0x03)
//  read  cmd: {0,1,0,1,1,0,A2,A1} -> 0x58 | (addr & 0x03)
#define BL0942_UART_CMD_WRITE(addr) (0xA8 | ((addr) & 0x03))
#define BL0942_UART_CMD_READ(addr)  (0x58 | ((addr) & 0x03))

// Packet mode request: send {READ_CMD, 0xAA} and the device returns 23 bytes
#define BL0942_PACKET_REQUEST 0xAA
#define BL0942_PACKET_HEAD    0x55
#define BL0942_PACKET_LEN     23

// Driver state
static bl0942_config_t s_cfg;
static bool s_inited = false;
static bool s_running = false;
static TaskHandle_t s_task = NULL;
static SemaphoreHandle_t s_mu = NULL;

static bl0942_measurements_t s_meas;

// CF_CNT is 24-bit.
static uint32_t s_last_cf_cnt = 0;

// Precomputed Wh per CF pulse.
static float s_wh_per_pulse = 0.0f;

static inline uint8_t checksum_inverted(const uint8_t *buf, size_t n)
{
        // checksum is (sum(buf[0..n-1]) & 0xff) then bitwise inverted
        uint32_t sum = 0;
        for (size_t i = 0; i < n; i++) sum += buf[i];
        return (uint8_t)(~(sum & 0xFF));
}

static inline uint32_t u24_le(const uint8_t *p)
{
        return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

static inline int32_t s24_le(const uint8_t *p)
{
        uint32_t u = u24_le(p);
        // sign-extend 24-bit
        if (u & 0x800000) u |= 0xFF000000;
        return (int32_t)u;
}

static float kconfig_strtof(const char *s, float fallback)
{
        if (!s) return fallback;
        char *end = NULL;
        float v = strtof(s, &end);
        if (end == s) return fallback;
        return v;
}

static inline uint16_t u16_le(const uint8_t *p)
{
        return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static esp_err_t uart_flush_input_safe(void)
{
        if (!s_inited) return ESP_ERR_INVALID_STATE;
        return uart_flush_input(s_cfg.uart_port);
}

static esp_err_t uart_write_all(const uint8_t *data, size_t len)
{
        int w = uart_write_bytes(s_cfg.uart_port, (const char *)data, len);
        if (w < 0 || (size_t)w != len) return ESP_FAIL;
        return uart_wait_tx_done(s_cfg.uart_port, pdMS_TO_TICKS(100));
}

static esp_err_t uart_read_exact(uint8_t *out, size_t len, TickType_t timeout)
{
        size_t got = 0;
        while (got < len) {
                int r = uart_read_bytes(s_cfg.uart_port, out + got, len - got, timeout);
                if (r <= 0) return ESP_ERR_TIMEOUT;
                got += (size_t)r;
        }
        return ESP_OK;
}

// Compute Wh per CF pulse based on datasheet equations.
// Using:
//  P = WATT * Vref^2 / (3537 * Rshunt * v_div_ratio * 1e6)
//  tCF(ms) = 1638.4 * 256 / WATT
//  E_pulse(Wh) = P * (tCF/1000) / 3600
static float compute_wh_per_pulse(const bl0942_config_t *c)
{
        const float r_shunt = (float)c->shunt_uohm * 1e-6f; // ohms
        if (r_shunt <= 0.0f || c->v_div_ratio <= 0.0f || c->vref_v <= 0.0f) return 0.0f;

        const float p_over_watt = (c->vref_v * c->vref_v) / (3537.0f * r_shunt * c->v_div_ratio * 1e6f);
        const float tcf_over_watt_s = (1638.4f * 256.0f) / 1000.0f; // seconds * WATT^-1

        // E_pulse = (WATT * p_over_watt) * (tcf_over_watt_s / WATT) / 3600
        return (p_over_watt * tcf_over_watt_s) / 3600.0f;
}

static void parse_and_update_from_packet(const uint8_t *pkt)
{
        // pkt[0] = head (0x55)
        const uint32_t i_rms = u24_le(pkt + 1);
        const uint32_t v_rms = u24_le(pkt + 4);
        const uint32_t i_fast = u24_le(pkt + 7);
        const int32_t watt = s24_le(pkt + 10);
        const uint32_t cf_cnt = u24_le(pkt + 13);
        const uint16_t freq = u16_le(pkt + 16);
        const uint32_t status = (uint32_t)pkt[19]; // lower bits are meaningful

        // Convert raw -> physical using datasheet equations.
        // I_RMS = 305978 * I_mV / Vref   (gain=16 default)
        // V_RMS = 73989  * V_mV / Vref
        const float vref = s_cfg.vref_v;
        const float r_shunt = (float)s_cfg.shunt_uohm * 1e-6f;

        float i_mV = 0.0f;
        float v_mV = 0.0f;
        float i_a = 0.0f;
        float v_line = 0.0f;
        float p_w = 0.0f;

        if (vref > 0.0f) {
                i_mV = (float)i_rms * vref / 305978.0f;
                v_mV = (float)v_rms * vref / 73989.0f;
        }

        if (r_shunt > 0.0f) {
                i_a = (i_mV / 1000.0f) / r_shunt;
        }

        if (s_cfg.v_div_ratio > 0.0f) {
                v_line = (v_mV / 1000.0f) / s_cfg.v_div_ratio;
        }

        // P = WATT * Vref^2 / (3537 * Rshunt * v_div_ratio * 1e6)
        if (r_shunt > 0.0f && s_cfg.v_div_ratio > 0.0f) {
                p_w = ((float)watt * (vref * vref)) / (3537.0f * r_shunt * s_cfg.v_div_ratio * 1e6f);
        }

        // Apply calibration multipliers
        v_line *= s_cfg.voltage_calibration;
        i_a    *= s_cfg.current_calibration;
        p_w    *= s_cfg.power_calibration;

        // Energy accumulation from CF_CNT delta
        uint32_t delta = (cf_cnt - s_last_cf_cnt) & 0xFFFFFFu;
        s_last_cf_cnt = cf_cnt;

        if (delta != 0 && s_wh_per_pulse > 0.0f) {
                s_meas.energy_wh += (float)delta * s_wh_per_pulse;
        }

        s_meas.voltage_v = v_line;
        s_meas.current_a = i_a;
        s_meas.power_w = p_w;

        s_meas.i_rms_raw = i_rms;
        s_meas.v_rms_raw = v_rms;
        s_meas.i_fast_rms_raw = i_fast;
        s_meas.watt_raw = watt;
        s_meas.cf_cnt_raw = cf_cnt;
        s_meas.freq_raw = freq;
        s_meas.status_raw = status;

        s_meas.valid = true;
}

static esp_err_t bl0942_read_packet(uint8_t *out_pkt)
{
        const uint8_t addr = s_cfg.address & 0x03;
        const uint8_t cmd = BL0942_UART_CMD_READ(addr);
        const uint8_t req[2] = { cmd, BL0942_PACKET_REQUEST };

        // Best effort: clear stale bytes
        (void)uart_flush_input_safe();

        esp_err_t err = uart_write_all(req, sizeof(req));
        if (err != ESP_OK) return err;

        // The datasheet notes up to ~48ms at 4800bps. We'll allow some margin.
        err = uart_read_exact(out_pkt, BL0942_PACKET_LEN, pdMS_TO_TICKS(120));
        if (err != ESP_OK) return err;

        if (out_pkt[0] != BL0942_PACKET_HEAD) return ESP_ERR_INVALID_RESPONSE;

        // checksum is over: cmd + head + data bytes (i.e., cmd + out_pkt[0..21])
        uint8_t tmp[1 + (BL0942_PACKET_LEN - 1)];
        tmp[0] = cmd;
        memcpy(tmp + 1, out_pkt, BL0942_PACKET_LEN - 1); // include head + data, exclude checksum byte
        const uint8_t expected = checksum_inverted(tmp, sizeof(tmp));
        const uint8_t got = out_pkt[BL0942_PACKET_LEN - 1];
        if (expected != got) return ESP_ERR_INVALID_CRC;

        return ESP_OK;
}

static void bl0942_task(void *arg)
{
        (void)arg;
        uint8_t pkt[BL0942_PACKET_LEN];

        const TickType_t period = pdMS_TO_TICKS(s_cfg.sample_period_ms);

        while (s_running) {
                esp_err_t err = bl0942_read_packet(pkt);

                xSemaphoreTake(s_mu, portMAX_DELAY);
                if (err == ESP_OK) {
                        parse_and_update_from_packet(pkt);
                } else {
                        s_meas.valid = false;
                }
                xSemaphoreGive(s_mu);

                if (err != ESP_OK) {
                        ESP_LOGW(TAG, "Packet read failed: %s", esp_err_to_name(err));
                }

                vTaskDelay(period);
        }

        s_task = NULL;
        vTaskDelete(NULL);
}

bl0942_config_t bl0942_config_default(void)
{
        bl0942_config_t c = {
                .uart_port = (uart_port_t)CONFIG_ESP_BL0942_UART_PORT,
                .uart_tx_gpio = CONFIG_ESP_BL0942_UART_TX_GPIO,
                .uart_rx_gpio = CONFIG_ESP_BL0942_UART_RX_GPIO,
                .baudrate = CONFIG_BL0942_UART_BAUD,
                .address = (uint8_t)CONFIG_BL0942_ADDRESS,
                .shunt_uohm = CONFIG_BL0942_SHUNT_UOHM,
                // NOTE: These Kconfig options are strings to allow decimals.
                .v_div_ratio = kconfig_strtof(CONFIG_BL0942_V_DIV_RATIO, 0.001f),
                .vref_v = kconfig_strtof(CONFIG_BL0942_VREF_V, 1.218f),
                .sample_period_ms = CONFIG_BL0942_SAMPLE_PERIOD_MS,
                .voltage_calibration = kconfig_strtof(CONFIG_BL0942_VOLTAGE_CAL, 1.0f),
                .current_calibration = kconfig_strtof(CONFIG_BL0942_CURRENT_CAL, 1.0f),
                .power_calibration = kconfig_strtof(CONFIG_BL0942_POWER_CAL, 1.0f),
        };
        return c;
}

esp_err_t bl0942_init(const bl0942_config_t *cfg)
{
        if (!cfg) return ESP_ERR_INVALID_ARG;
        if (s_inited) return ESP_OK;

        s_cfg = *cfg;
        s_wh_per_pulse = compute_wh_per_pulse(cfg);

        s_mu = xSemaphoreCreateMutex();
        if (!s_mu) return ESP_ERR_NO_MEM;

        uart_config_t ucfg = {
                .baud_rate = cfg->baudrate,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                .source_clk = UART_SCLK_DEFAULT,
        };

        ESP_RETURN_ON_ERROR(uart_param_config(cfg->uart_port, &ucfg), TAG, "uart_param_config");

        // Install driver. Small RX buffer is fine for 23-byte packets.
        ESP_RETURN_ON_ERROR(uart_driver_install(cfg->uart_port, 256, 0, 0, NULL, 0), TAG, "uart_driver_install");

        if (cfg->uart_tx_gpio >= 0 || cfg->uart_rx_gpio >= 0) {
                // If either is set, set both (use UART_PIN_NO_CHANGE for -1)
                const int tx = (cfg->uart_tx_gpio >= 0) ? cfg->uart_tx_gpio : UART_PIN_NO_CHANGE;
                const int rx = (cfg->uart_rx_gpio >= 0) ? cfg->uart_rx_gpio : UART_PIN_NO_CHANGE;
                ESP_RETURN_ON_ERROR(uart_set_pin(cfg->uart_port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "uart_set_pin");
        }

        memset(&s_meas, 0, sizeof(s_meas));
        s_last_cf_cnt = 0;

        s_inited = true;
        ESP_LOGI(TAG, "Init OK (UART%d, baud=%d, addr=%u, Wh/pulse=%.9f)", (int)cfg->uart_port, cfg->baudrate, cfg->address, (double)s_wh_per_pulse);
        return ESP_OK;
}

esp_err_t bl0942_start(void)
{
        if (!s_inited) return ESP_ERR_INVALID_STATE;
        if (s_running) return ESP_OK;

        s_running = true;
        BaseType_t ok = xTaskCreate(bl0942_task, "bl0942", 4096, NULL, 5, &s_task);
        if (ok != pdPASS) {
                s_running = false;
                return ESP_ERR_NO_MEM;
        }
        return ESP_OK;
}

esp_err_t bl0942_stop(void)
{
        if (!s_inited) return ESP_ERR_INVALID_STATE;
        if (!s_running) return ESP_OK;

        s_running = false;
        // Task exits on its own.
        return ESP_OK;
}

esp_err_t bl0942_deinit(void)
{
        if (!s_inited) return ESP_OK;

        (void)bl0942_stop();
        vTaskDelay(pdMS_TO_TICKS(10));

        uart_driver_delete(s_cfg.uart_port);

        if (s_mu) {
                vSemaphoreDelete(s_mu);
                s_mu = NULL;
        }

        s_inited = false;
        return ESP_OK;
}

bl0942_measurements_t bl0942_get(void)
{
        bl0942_measurements_t copy;
        memset(&copy, 0, sizeof(copy));

        if (!s_inited || !s_mu) return copy;

        xSemaphoreTake(s_mu, portMAX_DELAY);
        copy = s_meas;
        xSemaphoreGive(s_mu);

        return copy;
}

void bl0942_reset_energy(void)
{
        if (!s_inited || !s_mu) return;

        xSemaphoreTake(s_mu, portMAX_DELAY);
        s_meas.energy_wh = 0.0f;
        // Re-sync CF counter baseline so we don't count a big delta next tick.
        s_last_cf_cnt = s_meas.cf_cnt_raw & 0xFFFFFFu;
        xSemaphoreGive(s_mu);
}

esp_err_t bl0942_read_reg(uint8_t reg_addr, uint32_t *out_val)
{
        if (!s_inited) return ESP_ERR_INVALID_STATE;
        if (!out_val) return ESP_ERR_INVALID_ARG;

        const uint8_t addr = s_cfg.address & 0x03;
        const uint8_t cmd = BL0942_UART_CMD_READ(addr);

        uint8_t tx[2] = { cmd, reg_addr };
        (void)uart_flush_input_safe();

        ESP_RETURN_ON_ERROR(uart_write_all(tx, sizeof(tx)), TAG, "uart_write");

        // Read: 3 data bytes (low..high) + checksum
        uint8_t rx[4] = {0};
        esp_err_t err = uart_read_exact(rx, sizeof(rx), pdMS_TO_TICKS(50));
        if (err != ESP_OK) return err;

        // checksum over: cmd + reg + data0 + data1 + data2
        uint8_t chk_in[5] = { cmd, reg_addr, rx[0], rx[1], rx[2] };
        uint8_t expected = checksum_inverted(chk_in, sizeof(chk_in));
        if (expected != rx[3]) return ESP_ERR_INVALID_CRC;

        *out_val = u24_le(rx);
        return ESP_OK;
}

esp_err_t bl0942_write_reg(uint8_t reg_addr, uint32_t value)
{
        if (!s_inited) return ESP_ERR_INVALID_STATE;

        const uint8_t addr = s_cfg.address & 0x03;
        const uint8_t cmd = BL0942_UART_CMD_WRITE(addr);

        uint8_t d0 = (uint8_t)(value & 0xFF);
        uint8_t d1 = (uint8_t)((value >> 8) & 0xFF);
        uint8_t d2 = (uint8_t)((value >> 16) & 0xFF);

        uint8_t tx[6];
        tx[0] = cmd;
        tx[1] = reg_addr;
        tx[2] = d0;
        tx[3] = d1;
        tx[4] = d2;
        tx[5] = checksum_inverted(tx, 5);

        (void)uart_flush_input_safe();
        return uart_write_all(tx, sizeof(tx));
}

esp_err_t bl0942_unlock_user_writes(void)
{
        // Only 0x55 enables user operation register writes (datasheet register 0x1D).
        return bl0942_write_reg(BL0942_REG_USR_WRPROT, 0x55);
}

esp_err_t bl0942_soft_reset(void)
{
        // Writing 0x5a5a5a to 0x1C resets the chip.
        return bl0942_write_reg(BL0942_REG_SOFT_RESET, 0x5A5A5A);
}
