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

/// Re-apply device default configuration (DT/Kconfig).
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

/// Read and clear interrupt status.
int max3010x_get_interrupt_status(const struct device *dev,
				  uint8_t *int1,
				  uint8_t *int2);

/// Read FIFO bytes directly.
int max3010x_read_fifo(const struct device *dev, uint8_t *buf, size_t bytes);

/// Read raw LED channel sample (red/ir/green) from cached data.
int max3010x_get_raw_channel(const struct device *dev,
			     max3010x_led_channel_t channel,
			     uint32_t *value);

/// Return number of active FIFO channels.
uint8_t max3010x_get_num_channels(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* MAX3010X_H */
