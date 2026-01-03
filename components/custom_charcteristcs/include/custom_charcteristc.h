#pragma once

// HomeKit headers rely on `size_t` (tlv/types/homekit). When compiled as a
// separate component, it may not yet be defined.
#include <stddef.h>

#include <homekit/homekit.h>

// Elgato Eve Energy characteristic UUIDs
// These UUIDs are used by the Eve app to recognize power meter values.
#define CUSTOM_VOLTAGE "E863F10A-079E-48FF-8F27-9C2605A29F52"
#define CUSTOM_CURRENT "E863F126-079E-48FF-8F27-9C2605A29F52"
#define CUSTOM_POWER   "E863F10D-079E-48FF-8F27-9C2605A29F52"
#define CUSTOM_ENERGY  "E863F10C-079E-48FF-8F27-9C2605A29F52"

// The HomeKit library expects the characteristic "type" to be a UUID string.
// Use the Eve UUID strings directly (no wrapper macro needed).
#define HOMEKIT_CHARACTERISTIC_CUSTOM_VOLTAGE CUSTOM_VOLTAGE
#define HOMEKIT_CHARACTERISTIC_CUSTOM_CURRENT CUSTOM_CURRENT
#define HOMEKIT_CHARACTERISTIC_CUSTOM_POWER   CUSTOM_POWER
#define HOMEKIT_CHARACTERISTIC_CUSTOM_ENERGY  CUSTOM_ENERGY

// Notes:
// - Eve Energy expects these to be numeric (float) characteristics.
// - The HAP library will expose them as vendor-specific characteristics.
// - We keep descriptions explicit so apps display sane labels.
#define HOMEKIT_DECLARE_CHARACTERISTIC_CUSTOM_VOLTAGE(_value, ...) \
    .type = HOMEKIT_CHARACTERISTIC_CUSTOM_VOLTAGE, \
    .description = "Voltage (V)", \
    .format = homekit_format_float, \
    .permissions = homekit_permissions_paired_read | homekit_permissions_notify, \
    .value = HOMEKIT_FLOAT_(_value), \
    ##__VA_ARGS__

#define HOMEKIT_DECLARE_CHARACTERISTIC_CUSTOM_CURRENT(_value, ...) \
    .type = HOMEKIT_CHARACTERISTIC_CUSTOM_CURRENT, \
    .description = "Current (A)", \
    .format = homekit_format_float, \
    .permissions = homekit_permissions_paired_read | homekit_permissions_notify, \
    .value = HOMEKIT_FLOAT_(_value), \
    ##__VA_ARGS__

#define HOMEKIT_DECLARE_CHARACTERISTIC_CUSTOM_POWER(_value, ...) \
    .type = HOMEKIT_CHARACTERISTIC_CUSTOM_POWER, \
    .description = "Power (W)", \
    .format = homekit_format_float, \
    .permissions = homekit_permissions_paired_read | homekit_permissions_notify, \
    .value = HOMEKIT_FLOAT_(_value), \
    ##__VA_ARGS__

#define HOMEKIT_DECLARE_CHARACTERISTIC_CUSTOM_ENERGY(_value, ...) \
    .type = HOMEKIT_CHARACTERISTIC_CUSTOM_ENERGY, \
    .description = "Total Energy (kWh)", \
    .format = homekit_format_float, \
    .permissions = homekit_permissions_paired_read | homekit_permissions_notify, \
    .value = HOMEKIT_FLOAT_(_value), \
    ##__VA_ARGS__

// Global characteristics (defined in custom_charcteristc.c)
extern homekit_characteristic_t energy_voltage;
extern homekit_characteristic_t energy_current;
extern homekit_characteristic_t energy_power;
extern homekit_characteristic_t energy_energy;
