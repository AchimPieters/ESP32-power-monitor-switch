#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// BL0942 registers (datasheet v1.06)
// Read-only
#define BL0942_REG_I_WAVE          0x01
#define BL0942_REG_V_WAVE          0x02
#define BL0942_REG_I_RMS           0x03
#define BL0942_REG_V_RMS           0x04
#define BL0942_REG_I_FAST_RMS      0x05
#define BL0942_REG_WATT            0x06
#define BL0942_REG_CF_CNT          0x07
#define BL0942_REG_FREQ            0x08
#define BL0942_REG_STATUS          0x09

// Read/write (user operation)
#define BL0942_REG_I_RMSOS         0x12
#define BL0942_REG_WA_CREEP        0x14
#define BL0942_REG_I_FAST_RMS_TH   0x15
#define BL0942_REG_I_FAST_RMS_CYC  0x16
#define BL0942_REG_FREQ_CYC        0x17
#define BL0942_REG_OT_FUNX         0x18
#define BL0942_REG_MODE            0x19
#define BL0942_REG_GAIN_CR         0x1A
#define BL0942_REG_SOFT_RESET      0x1C
#define BL0942_REG_USR_WRPROT      0x1D

// MODE bits (0x19)
#define BL0942_MODE_CF_EN                 (1u << 2)
#define BL0942_MODE_RMS_UPDATE_SEL        (1u << 3)
#define BL0942_MODE_FAST_RMS_SEL          (1u << 4)
#define BL0942_MODE_AC_FREQ_SEL           (1u << 5)
#define BL0942_MODE_CF_CNT_CLR_SEL        (1u << 6)
#define BL0942_MODE_CF_CNT_ADD_SEL        (1u << 7)
#define BL0942_MODE_UART_RATE_SEL_SHIFT   8

// STATUS bits (0x09)
#define BL0942_STATUS_CF_REVP_F           (1u << 0)
#define BL0942_STATUS_CREEP_F             (1u << 1)

typedef struct {
    // UART wiring (ESP side):
    uart_port_t uart_port;
    int uart_tx_gpio; // ESP TX (to BL0942 RX/SDI). Use -1 to keep UART default.
    int uart_rx_gpio; // ESP RX (to BL0942 TX/SDO). Use -1 to keep UART default.
    int baudrate;

    // BL0942 UART address (A2:A1) for TSSOP14 multi-drop. SSOP10 is always 0.
    uint8_t address;

    // Board parameters for physical units:
    int shunt_uohm;     // shunt resistor in micro-ohms
    float v_div_ratio;  // Vpin_rms / Vline_rms
    float vref_v;       // on-chip reference voltage (typ. 1.218V)

    // Sampling:
    int sample_period_ms;

    // Calibration multipliers (optional):
    float voltage_calibration;
    float current_calibration;
    float power_calibration;
} bl0942_config_t;

typedef struct {
    // Physical values:
    float voltage_v;
    float current_a;
    float power_w;
    float energy_wh;

    // Raw register values (for debugging/advanced use):
    uint32_t i_rms_raw;
    uint32_t v_rms_raw;
    uint32_t i_fast_rms_raw;
    int32_t  watt_raw;
    uint32_t cf_cnt_raw;
    uint16_t freq_raw;
    uint32_t status_raw;

    // Flags:
    bool valid;
} bl0942_measurements_t;

/**
 * @brief Get a config populated from Kconfig defaults.
 */
bl0942_config_t bl0942_config_default(void);

/**
 * @brief Initialize the driver (UART + internal state). Does not start polling.
 */
esp_err_t bl0942_init(const bl0942_config_t *cfg);

/**
 * @brief Start periodic measurements (packet mode).
 */
esp_err_t bl0942_start(void);

/**
 * @brief Stop periodic measurements.
 */
esp_err_t bl0942_stop(void);

/**
 * @brief Deinitialize driver and free resources.
 */
esp_err_t bl0942_deinit(void);

/**
 * @brief Get the latest measurements snapshot (thread-safe copy).
 */
bl0942_measurements_t bl0942_get(void);

/**
 * @brief Reset accumulated energy counter to zero (driver-side).
 */
void bl0942_reset_energy(void);

// ---- Full register access API (complete coverage) ----

/**
 * @brief Read a BL0942 register (always returns 24-bit value in *out_val).
 *
 * For registers narrower than 24 bits, upper bits are zero-filled.
 */
esp_err_t bl0942_read_reg(uint8_t reg_addr, uint32_t *out_val);

/**
 * @brief Write a BL0942 register (writes 24-bit value, MSB ignored by narrower regs).
 *
 * NOTE: user operation registers require USR_WRPROT to be set to 0x55 first.
 */
esp_err_t bl0942_write_reg(uint8_t reg_addr, uint32_t value);

/**
 * @brief Enable writing of user operation registers by writing 0x55 to USR_WRPROT.
 */
esp_err_t bl0942_unlock_user_writes(void);

/**
 * @brief Issue a software reset (write 0x5a5a5a to SOFT_RESET).
 */
esp_err_t bl0942_soft_reset(void);

#ifdef __cplusplus
}
#endif
