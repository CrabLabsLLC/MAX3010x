/*
 * Copyright (c) 2025 Makani Science
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MAX3010X_TYPES_H
#define MAX3010X_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include "max3010x_regs.h"

// Device variant selection
typedef enum
{
	MAX3010X_VARIANT_MAX30101 = 0,
	MAX3010X_VARIANT_MAX30102 = 1,
} max3010x_variant_t;

// Operating modes
typedef enum
{
	// Heart rate mode (red LED only)
	MAX3010X_MODE_HEART_RATE = 2,
	// SpO2 mode (red + IR)
	MAX3010X_MODE_SPO2       = 3,
	// Multi-LED mode (slots control active LEDs)
	MAX3010X_MODE_MULTI_LED  = 7,
} max3010x_mode_t;

// LED slot selections
typedef enum
{
	// Slot disabled (no LED pulse)
	MAX3010X_SLOT_DISABLED       = 0,
	// Red LED uses LED1_PA
	MAX3010X_SLOT_RED_LED1_PA    = 1,
	// IR LED uses LED2_PA
	MAX3010X_SLOT_IR_LED2_PA     = 2,
	// Green LED uses LED3_PA
	MAX3010X_SLOT_GREEN_LED3_PA  = 3,
	// Red pilot LED uses PILOT_PA
	MAX3010X_SLOT_RED_PILOT_PA   = 5,
	// IR pilot LED uses PILOT_PA
	MAX3010X_SLOT_IR_PILOT_PA    = 6,
	// Green pilot LED uses PILOT_PA
	MAX3010X_SLOT_GREEN_PILOT_PA = 7,
} max3010x_slot_t;

// LED channel identifiers
typedef enum
{
	MAX3010X_LED_RED = 0,
	MAX3010X_LED_IR,
	MAX3010X_LED_GREEN,
} max3010x_led_channel_t;

// ADC ranges
typedef enum
{
	MAX3010X_ADC_RANGE_7P81  = 0,
	MAX3010X_ADC_RANGE_15P63 = 1,
	MAX3010X_ADC_RANGE_31P25 = 2,
	MAX3010X_ADC_RANGE_62P50 = 3,
} max3010x_adc_range_t;

// Sample rate codes (SPO2_SR[2:0])
typedef enum
{
	MAX3010X_SR_50HZ   = 0, // 50 samples/s
	MAX3010X_SR_100HZ  = 1, // 100 samples/s
	MAX3010X_SR_200HZ  = 2, // 200 samples/s
	MAX3010X_SR_400HZ  = 3, // 400 samples/s
	MAX3010X_SR_800HZ  = 4, // 800 samples/s
	MAX3010X_SR_1000HZ = 5, // 1000 samples/s
	MAX3010X_SR_1600HZ = 6, // 1600 samples/s
	MAX3010X_SR_3200HZ = 7, // 3200 samples/s
} max3010x_sample_rate_t;

// Pulse width codes
typedef enum
{
	MAX3010X_PW_69US  = 0,
	MAX3010X_PW_118US = 1,
	MAX3010X_PW_215US = 2,
	MAX3010X_PW_411US = 3,
} max3010x_pulse_width_t;

// FIFO sample averaging codes
typedef enum
{
	MAX3010X_FIFO_AVG_1   = 0,
	MAX3010X_FIFO_AVG_2   = 1,
	MAX3010X_FIFO_AVG_4   = 2,
	MAX3010X_FIFO_AVG_8   = 3,
	MAX3010X_FIFO_AVG_16  = 4,
	MAX3010X_FIFO_AVG_32  = 5,
	MAX3010X_FIFO_AVG_32B = 6,
	MAX3010X_FIFO_AVG_32C = 7,
} max3010x_fifo_avg_t;

// Slot configuration
typedef struct
{
	/// Slot selections (1-4); used only in multi-LED mode.
	max3010x_slot_t slot[4];
} max3010x_slot_config_t;

// FIFO configuration
typedef struct
{
	/// Sample averaging code (0-7)
	max3010x_fifo_avg_t sample_avg;
	/// FIFO almost-full threshold (0-15)
	uint8_t almost_full;
	/// Enable FIFO rollover
	bool rollover;
} max3010x_fifo_config_t;

// SpO2 configuration
typedef struct
{
	/// ADC range selection
	max3010x_adc_range_t adc_range;
	/// Sampling rate code
	max3010x_sample_rate_t sample_rate;
	/// Pulse width code
	max3010x_pulse_width_t pulse_width;
} max3010x_spo2_config_t;

// LED pulse amplitude configuration
typedef struct
{
	/// LED1 (red) amplitude
	uint8_t red;
	/// LED2 (IR) amplitude
	uint8_t ir;
	/// LED3 (green) amplitude
	uint8_t green;
	/// LED4 (green) amplitude
	uint8_t green2;
	/// Pilot LED amplitude
	uint8_t pilot;
} max3010x_led_pa_t;

// Interrupt enable masks
typedef struct
{
	/// INT_ENABLE_1 mask
	uint8_t int1_mask;
	/// INT_ENABLE_2 mask
	uint8_t int2_mask;
} max3010x_interrupt_config_t;

// Device configuration
typedef struct
{
	/// I2C device spec
	struct i2c_dt_spec i2c;
	/// Optional interrupt GPIO
	struct gpio_dt_spec int_gpio;
	/// Silicon variant
	max3010x_variant_t variant;
	/// Operating mode
	max3010x_mode_t mode;
	/// FIFO configuration
	max3010x_fifo_config_t fifo;
	/// SpO2 configuration
	max3010x_spo2_config_t spo2;
	/// Slot configuration
	max3010x_slot_config_t slots;
	/// LED amplitude configuration
	max3010x_led_pa_t led_pa;
	/// Interrupt enable configuration
	max3010x_interrupt_config_t interrupts;
	/// Proximity interrupt threshold
	uint8_t prox_int_thresh;
	/// Enable die temperature conversion
	bool temp_enable;
} max3010x_config_t;

// Cached raw samples (per FIFO channel)
typedef struct
{
	/// Raw 18-bit samples, one entry per active FIFO channel
	uint32_t raw[MAX3010X_MAX_NUM_CHANNELS];
} max3010x_data_t;

// Channel mapping (kept separate from data)
typedef struct
{
	/// Map LED channel -> FIFO index
	uint8_t fifo_index[MAX3010X_MAX_NUM_CHANNELS];
	/// Number of active FIFO channels
	uint8_t active_channels;
} max3010x_channel_map_t;

#endif /* MAX3010X_TYPES_H */
