#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
        // UART wiring (BL0942 in UART mode: SEL=0, UART half-duplex)
        uart_port_t uart_num;          // usually UART_NUM_0
        int uart_baud;                 // 4800..38400 (see datasheet)
        gpio_num_t uart_tx_gpio;       // -1 keeps default pins
        gpio_num_t uart_rx_gpio;       // -1 keeps default pins

        // Board parameters for physical units:
        int shunt_uohm;                // shunt resistor in micro-ohms
        float v_div_ratio;             // Vpin_rms / Vline_rms
        float vref_v;                  // BL0942 reference voltage (typ. 1.218V)

        // Timing:
        int sample_period_ms;          // poll interval (packet request)

        // Calibration trim (optional, persisted in NVS by app):
        float voltage_calibration;
        float current_calibration;
        float power_calibration;

        // BL0942 UART address (SSOP10L is fixed 0; TSSOP14L can be 0..3)
        uint8_t ic_addr;               // 0..3
} bl0942_config_t;

typedef struct {
        float voltage_v;
        float current_a;
        float power_w;
        float energy_wh;

        // Raw values (register units) from packet:
        uint32_t i_rms_raw;
        uint32_t v_rms_raw;
        uint32_t i_fast_rms_raw;
        int32_t  watt_raw;
        uint32_t cf_cnt_raw;
        uint16_t freq_raw;
        uint16_t status_raw;

        bool valid;        // packet checksum ok and at least voltage/current decoded
        bool valid_voltage;
        bool valid_current;
        bool valid_power;
} bl0942_measurements_t;

bl0942_config_t bl0942_config_default(void);

esp_err_t bl0942_init(const bl0942_config_t *cfg);
esp_err_t bl0942_start(void);
esp_err_t bl0942_stop(void);
esp_err_t bl0942_deinit(void);

bl0942_measurements_t bl0942_get(void);
void bl0942_reset_energy(void);

/**
 * @brief Helper to compute v_div_ratio from resistor divider (Rtop from line to pin, Rbottom from pin to GND).
 *
 * v_div_ratio = Rbottom / (Rtop + Rbottom)
 */
static inline float bl0942_div_ratio_from_resistors(float r_top_ohm, float r_bottom_ohm)
{
        if (r_top_ohm <= 0.0f || r_bottom_ohm <= 0.0f) return 0.0f;
        return r_bottom_ohm / (r_top_ohm + r_bottom_ohm);
}

#ifdef __cplusplus
}
#endif
