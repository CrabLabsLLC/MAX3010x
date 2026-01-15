/*
 * Copyright (c) 2025 Makani Science
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MAX3010X_H
#define MAX3010X_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "max3010x_regs.h"
#include "max3010x_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Initialize the device (device-tree backed).
int max3010x_init(const struct device *dev);

/// Reset device and wait for completion.
int max3010x_reset(const struct device *dev);

/// Enable measurement (wake from shutdown).
int max3010x_enable(const struct device *dev);

/// Disable measurement (enter shutdown).
int max3010x_disable(const struct device *dev);

/// Set operating mode (heart rate, SpO2, multi-LED).
int max3010x_set_mode(const struct device *dev, max3010x_mode_t mode);

/// Apply FIFO configuration.
int max3010x_set_fifo_config(const struct device *dev,
			     const max3010x_fifo_config_t *cfg);

/// Apply SpO2 configuration.
int max3010x_set_spo2_config(const struct device *dev,
			     const max3010x_spo2_config_t *cfg);

/// Set ADC range only.
int max3010x_set_adc_range(const struct device *dev, max3010x_adc_range_t range);

/// Set sample rate only.
int max3010x_set_sample_rate(const struct device *dev, max3010x_sample_rate_t rate);

/// Set pulse width only.
int max3010x_set_pulse_width(const struct device *dev, max3010x_pulse_width_t width);

/// Set LED pulse amplitudes.
int max3010x_set_led_pa(const struct device *dev,
			const max3010x_led_pa_t *cfg);

/// Set a single LED pulse amplitude (red/ir/green).
int max3010x_set_led_channel_pa(const struct device *dev,
				max3010x_led_channel_t channel,
				uint8_t value);

/// Re-apply DT/Kconfig defaults (mode, FIFO, SpO2, LED currents, slots).
/// Useful after reset to restore configuration without full re-init.
int max3010x_apply_default_config(const struct device *dev);

/// Configure multi-LED slots.
int max3010x_set_slots(const struct device *dev,
		       const max3010x_slot_config_t *cfg);

/// Enable or disable interrupts.
int max3010x_set_interrupts(const struct device *dev,
			    uint8_t int1_mask,
			    uint8_t int2_mask);

/// Set proximity interrupt threshold.
int max3010x_set_prox_int_thresh(const struct device *dev, uint8_t threshold);

/// Enable or disable die temperature conversion.
int max3010x_set_temp_enable(const struct device *dev, bool enable);

/// Read die temperature in Celsius (triggers conversion and waits).
int max3010x_get_temp(const struct device *dev, float *temp_c);

/// Read and clear interrupt status.
int max3010x_get_interrupt_status(const struct device *dev,
				  uint8_t *int1,
				  uint8_t *int2);

/// Read FIFO bytes directly.
int max3010x_read_fifo(const struct device *dev, uint8_t *buf, size_t bytes);

/// Flush FIFO by resetting read/write pointers and overflow counter.
/// Call this before changing sample rate to avoid mixed-rate samples.
int max3010x_flush_fifo(const struct device *dev);

/// Read raw LED channel sample (red/ir/green) from cached data.
int max3010x_get_raw_channel(const struct device *dev,
			     max3010x_led_channel_t channel,
			     uint32_t *value);

/// Return number of active FIFO channels.
uint8_t max3010x_get_num_channels(const struct device *dev);

/* --------------------------------------------------------------------------
 * Interrupt Sources (INT_STATUS_1/2 registers)
 *
 * INT_STATUS_1 (0x00) / INT_ENABLE_1 (0x02):
 *   Bit 7: MAX3010X_INT_A_FULL    - FIFO almost full
 *   Bit 6: MAX3010X_INT_PPG_RDY   - New PPG data ready (most common)
 *   Bit 5: MAX3010X_INT_ALC_OVF   - Ambient light cancellation overflow
 *   Bit 4: MAX3010X_INT_PROX_INT  - Proximity interrupt
 *   Bit 0: MAX3010X_INT_PWR_RDY   - Power ready (status only, not enable)
 *
 * INT_STATUS_2 (0x01) / INT_ENABLE_2 (0x03):
 *   Bit 1: MAX3010X_INT_DIE_TEMP_RDY - Die temperature ready
 *
 * Use max3010x_set_interrupts() to configure which events trigger GPIO.
 * -------------------------------------------------------------------------- */

/**
 * Interrupt callback signature.
 * Called from ISR context when GPIO interrupt fires.
 *
 * @param dev       Device instance
 * @param user_data User context passed during registration
 */
typedef void (*max3010x_int_callback_t)(const struct device *dev, void *user_data);

/**
 * Register interrupt callback.
 * Callback is invoked from ISR context - keep processing minimal.
 *
 * @param dev       Device instance
 * @param callback  Callback function (NULL to disable)
 * @param user_data User context passed to callback
 * @return 0 on success, negative errno on failure
 */
int max3010x_set_int_callback(const struct device *dev,
			      max3010x_int_callback_t callback,
			      void *user_data);

/**
 * Enable GPIO interrupt on INT pin.
 * Call this after setting int sources and callback to start receiving interrupts.
 *
 * @param dev    Device instance
 * @param enable True to enable, false to disable
 * @return 0 on success, negative errno on failure
 */
int max3010x_enable_int(const struct device *dev, bool enable);

/**
 * Get sample counter (incremented on each interrupt).
 * Use this to check if samples are available without registering a callback.
 *
 * @param dev Device instance
 * @return Number of interrupts since last reset/read
 */
uint32_t max3010x_get_sample_count(const struct device *dev);

/**
 * Reset sample counter to zero.
 *
 * @param dev Device instance
 */
void max3010x_reset_sample_count(const struct device *dev);

/**
 * Get and reset sample counter atomically.
 * Useful for checking how many samples accumulated since last check.
 *
 * @param dev Device instance
 * @return Number of interrupts since last call
 */
uint32_t max3010x_get_and_reset_sample_count(const struct device *dev);

/**
 * Read the current state of the INT GPIO pin.
 * Useful for polling mode when interrupts are disabled.
 * The INT pin is active-low (0 = interrupt pending, 1 = no interrupt).
 *
 * @param dev   Device instance
 * @param state Output: GPIO pin state (0 = active/low, 1 = inactive/high)
 * @return 0 on success, -ENOTSUP if INT GPIO not configured, negative errno on failure
 */
int max3010x_read_int_gpio(const struct device *dev, int *state);

/**
 * Check if interrupt GPIO is configured and available.
 *
 * @param dev Device instance
 * @return true if INT GPIO is configured, false otherwise
 */
bool max3010x_has_int_gpio(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* MAX3010X_H */
