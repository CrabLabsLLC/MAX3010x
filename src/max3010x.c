/**
 * @file max3010x.c
 * @brief MAX3010x PPG/SpO2 sensor - Zephyr device driver
 *
 * Single-file implementation for MAX30101/MAX30102. Direct I2C/GPIO
 * calls via Zephyr APIs. No HAL abstraction layer.
 *
 * Init sequence:
 *   1. Verify I2C ready
 *   2. Read and verify part ID
 *   3. Soft reset and wait
 *   4. Apply Kconfig/DT defaults (mode, FIFO, SpO2, LED currents, slots)
 *   5. Put device in SHUTDOWN mode
 *   6. Configure GPIO interrupt (but leave disabled)
 *
 * @author Orion Serup <orion@crablabs.io>
 *
 * @reviewer Daravuthy Ly <daravuthy@crablabs.io>
 */

/* Copyright (c) 2025 Crab Labs LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT maxim_max3010x

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>

#include "max3010x.h"

LOG_MODULE_REGISTER(max3010x, CONFIG_MAX3010X_LOG_LEVEL);

/** @brief Per-instance immutable config (from DT + Kconfig) */
typedef struct
{
	struct i2c_dt_spec i2c;				   ///< I2C bus and address
	struct gpio_dt_spec int_gpio;		   ///< Interrupt GPIO (optional)
	MAX3010xVariant variant;			   ///< MAX30101 or MAX30102
	MAX3010xMode mode;					   ///< Default operating mode
	MAX3010xFIFOConfig fifo;			   ///< FIFO configuration
	MAX3010xSPO2Config spo2;			   ///< SpO2/PPG ADC configuration
	MAX3010xSlotConfig slots;			   ///< Multi-LED slot configuration
	MAX3010xLEDAmplitude led_amplitude;	   ///< LED pulse amplitudes
	uint8_t interrupt_enable_1_mask;	   ///< INT_ENABLE_1 register value
	uint8_t interrupt_enable_2_mask;	   ///< INT_ENABLE_2 register value
	uint8_t proximity_interrupt_threshold; ///< Proximity interrupt threshold
	bool is_temp_enabled;				   ///< Enable die temperature conversion
} MAX3010xDriverConfig;

/** @brief Per-instance mutable driver state */
typedef struct
{
	MAX3010xData data;			   ///< Most recent FIFO sample data
	MAX3010xChannelMap map;		   ///< LED channel to FIFO index mapping
	MAX3010xCallback int_callback; ///< Application interrupt callback
	void* int_user_data;		   ///< User context for interrupt callback
	struct gpio_callback gpio_cb;  ///< Zephyr GPIO callback structure
	atomic_t sample_count;		   ///< Interrupt event counter
	const struct device* dev;	   ///< Back-reference to owning device
	struct k_mutex lock;			   ///< Shared register transaction lock
	uint8_t current_mode;		   ///< Current MODE bits [2:0] (cached to avoid RMW)
	uint8_t saved_led[5];		   ///< LED amp cache: [0]=LED1, [1]=LED2, [2]=LED3, [3]=LED4, [4]=PILOT
	bool is_initialized;		   ///< Initialization complete flag
} MAX3010xDriverData;

static int regRead(const struct device* const dev, const uint8_t reg, uint8_t* const value);
static int regWrite(const struct device* const dev, const uint8_t reg, const uint8_t value);
static int regWriteVerify(const struct device* const dev, const uint8_t reg, const uint8_t value, const uint8_t mask);
static int regReadBurst(const struct device* const dev, const uint8_t reg, uint8_t* const buffer, const size_t length);
static void buildChannelMap(const MAX3010xDriverConfig* const cfg, const MAX3010xSlotConfig* const slots, MAX3010xChannelMap* const map);
static MAX3010xSlot sanitizeSlot(const MAX3010xDriverConfig* const cfg, const MAX3010xSlot slot);
static bool hasGPIO(const struct gpio_dt_spec* const spec);
static void intGPIOHandler(const struct device* const port, struct gpio_callback* const cb, const gpio_port_pins_t pins);

int max3010xLock(const struct device* const dev, const k_timeout_t timeout)
{
	if (dev == NULL)
		return -EINVAL;

	MAX3010xDriverData* const data = dev->data;
	return k_mutex_lock(&data->lock, timeout);
}

void max3010xUnlock(const struct device* const dev)
{
	if (dev == NULL)
		return;

	MAX3010xDriverData* const data = dev->data;
	k_mutex_unlock(&data->lock);
}

int max3010xReset(const struct device* const dev)
{
	if (dev == NULL)
		return -EINVAL;

	int ret = regWrite(dev, MAX3010X_REG_MODE_CONFIG, MAX3010X_MODE_RESET);
	if (ret < 0)
		return ret;

	uint8_t mode_config = 0;
	int remaining_ms = MAX3010X_RESET_TIMEOUT_MS;

	do
	{
		k_msleep(1);
		ret = regRead(dev, MAX3010X_REG_MODE_CONFIG, &mode_config);
		if (ret < 0)
			return ret;
		remaining_ms--;
	} while ((mode_config & MAX3010X_MODE_RESET) && (remaining_ms > 0));

	if (remaining_ms <= 0)
	{
		LOG_ERR("Reset timeout");
		return -ETIMEDOUT;
	}

	/*
	 * Post-reset settling delay.
	 * The MAX30101 datasheet does not specify a minimum delay between
	 * the RESET bit self-clearing and register access. However, testing
	 * shows that immediate writes after reset can be silently lost.
	 * A 5 ms delay ensures the internal state machines have stabilized.
	 */
	k_msleep(5);

	/*
	 * Clear interrupt status registers after reset.
	 * PWR_RDY is set after reset and holds the INT pin low.
	 * Reading INT_STATUS_1/2 clears the flags and releases the pin,
	 * allowing future edge-triggered GPIO interrupts to fire.
	 */
	uint8_t int1_status = 0;
	uint8_t int2_status = 0;
	(void)regRead(dev, MAX3010X_REG_INT_STATUS_1, &int1_status);
	(void)regRead(dev, MAX3010X_REG_INT_STATUS_2, &int2_status);

	LOG_DBG("Reset complete (int1=0x%02x, int2=0x%02x)", int1_status, int2_status);
	return 0;
}

int max3010xEnable(const struct device* const dev)
{
	if (dev == NULL)
		return -EINVAL;

	MAX3010xDriverData* const data = dev->data;
	const MAX3010xDriverConfig* const cfg = dev->config;
	const uint8_t mode_register_value = data->current_mode & MAX3010X_MODE_MASK;

	/*
	 * Wake from shutdown by clearing the SHDN bit.
	 *
	 * All configuration registers persist through shutdown (per datasheet).
	 * Init writes FIFO/SpO2/LED/slot config, then enters SHDN|MODE.
	 * We just clear SHDN here to resume sampling.
	 *
	 * NOTE: If mode activation triggers a register reset (observed on
	 * some defective parts), the verify step below detects it and
	 * reconfigures from cached state.
	 */
	const int ret = regWrite(dev, MAX3010X_REG_MODE_CONFIG, mode_register_value);
	if (ret < 0)
	{
		LOG_ERR("Enable write failed: %d", ret);
		return ret;
	}

	k_msleep(5);

	// Verify config survived the SHDN->active transition
	uint8_t mode_readback = 0;
	uint8_t spo2_readback = 0;
	regRead(dev, MAX3010X_REG_MODE_CONFIG, &mode_readback);
	regRead(dev, MAX3010X_REG_SPO2_CONFIG, &spo2_readback);

	if ((mode_readback & MAX3010X_MODE_MASK) != mode_register_value || spo2_readback == 0x00)
	{
		LOG_WRN("Config lost after mode activation - reconfiguring");

		// Reconfigure: reset -> full config -> mode last
		const int reset_ret = max3010xReset(dev);
		if (reset_ret < 0)
		{
			LOG_ERR("Recovery reset failed: %d", reset_ret);
			return reset_ret;
		}
		max3010xSetFIFOConfig(dev, &cfg->fifo);
		max3010xSetSPO2Config(dev, &cfg->spo2);

		const MAX3010xLEDAmplitude leds =
			{
				.red = data->saved_led[0],
				.ir = data->saved_led[1],
				.green = data->saved_led[2],
				.green2 = data->saved_led[3],
				.pilot = data->saved_led[4],
			};
		max3010xSetLEDAmplitude(dev, &leds);
		max3010xSetSlots(dev, &cfg->slots);
		regWrite(dev, MAX3010X_REG_MODE_CONFIG, mode_register_value);
		k_msleep(5);
	}

	// Clear interrupt status
	uint8_t int_clear = 0;
	(void)regRead(dev, MAX3010X_REG_INT_STATUS_1, &int_clear);
	(void)regRead(dev, MAX3010X_REG_INT_STATUS_2, &int_clear);

	// Flush FIFO for clean data stream
	max3010xFlushFIFO(dev);

#if IS_ENABLED(CONFIG_LOG) && (CONFIG_MAX3010X_LOG_LEVEL >= LOG_LEVEL_DBG)
	max3010xDumpRegisters(dev);
#endif

	return 0;
}

int max3010xDisable(const struct device* const dev)
{
	if (dev == NULL)
		return -EINVAL;

	const MAX3010xDriverData* const data = dev->data;

	/*
	 * Enter shutdown mode - SHDN bit set, MODE bits preserved.
	 * All configuration registers retain their values (~0.7 uA standby).
	 * For true power-off, the app can cut the sensor_pwr regulator.
	 */
	return regWrite(dev, MAX3010X_REG_MODE_CONFIG,
					MAX3010X_MODE_SHDN | (data->current_mode & MAX3010X_MODE_MASK));
}

int max3010xSetMode(const struct device* const dev, const MAX3010xMode mode)
{
	if (dev == NULL)
		return -EINVAL;

	MAX3010xDriverData* const data = dev->data;
	data->current_mode = (uint8_t)(mode & MAX3010X_MODE_MASK);

	/* Preserve current SHDN state while updating mode bits.
	 * Changing mode while active resets FIFO pointers (per datasheet). */
	uint8_t current_mode_register = 0;
	const int ret = regRead(dev, MAX3010X_REG_MODE_CONFIG, &current_mode_register);
	if (ret < 0)
		return ret;

	const uint8_t new_mode_value = (current_mode_register & MAX3010X_MODE_SHDN) | data->current_mode;
	return regWrite(dev, MAX3010X_REG_MODE_CONFIG, new_mode_value);
}

int max3010xSetFIFOConfig(const struct device* const dev, const MAX3010xFIFOConfig* const config)
{
	if (dev == NULL || config == NULL)
		return -EINVAL;

	const uint8_t fifo_config =
		(uint8_t)(config->sample_average_mode << MAX3010X_FIFO_SAMPLE_AVERAGE_SHIFT) |
		(config->is_rollover_enabled ? MAX3010X_FIFO_ROLLOVER_EN : 0) |
		(config->almost_full_threshold & MAX3010X_FIFO_A_FULL_MASK);

	return regWriteVerify(dev, MAX3010X_REG_FIFO_CONFIG, fifo_config, 0xFF);
}

int max3010xSetSPO2Config(const struct device* const dev, const MAX3010xSPO2Config* const config)
{
	if (dev == NULL || config == NULL)
		return -EINVAL;

	const uint8_t spo2_config =
		(uint8_t)(config->adc_range << MAX3010X_SPO2_ADC_RANGE_SHIFT) |
		(uint8_t)(config->sample_rate << MAX3010X_SPO2_SAMPLE_RATE_SHIFT) |
		(uint8_t)(config->pulse_width << MAX3010X_SPO2_PULSE_WIDTH_SHIFT);

	return regWriteVerify(dev, MAX3010X_REG_SPO2_CONFIG, spo2_config, 0xFF);
}

int max3010xSetADCRange(const struct device* const dev, const MAX3010xADCRange range)
{
	if (dev == NULL)
		return -EINVAL;

	uint8_t spo2_config = 0;
	const int ret = regRead(dev, MAX3010X_REG_SPO2_CONFIG, &spo2_config);
	if (ret < 0)
		return ret;

	spo2_config &= (uint8_t)~MAX3010X_SPO2_ADC_RANGE_MASK;
	spo2_config |= (uint8_t)(range << MAX3010X_SPO2_ADC_RANGE_SHIFT);
	return regWrite(dev, MAX3010X_REG_SPO2_CONFIG, spo2_config);
}

int max3010xSetSampleRate(const struct device* const dev, const MAX3010xSampleRate rate)
{
	if (dev == NULL)
		return -EINVAL;

	uint8_t spo2_config = 0;
	const int ret = regRead(dev, MAX3010X_REG_SPO2_CONFIG, &spo2_config);
	if (ret < 0)
		return ret;

	spo2_config &= (uint8_t)~MAX3010X_SPO2_SAMPLE_RATE_MASK;
	spo2_config |= (uint8_t)(rate << MAX3010X_SPO2_SAMPLE_RATE_SHIFT);
	return regWrite(dev, MAX3010X_REG_SPO2_CONFIG, spo2_config);
}

int max3010xSetPulseWidth(const struct device* const dev, const MAX3010xPulseWidth width)
{
	if (dev == NULL)
		return -EINVAL;

	uint8_t spo2_config = 0;
	const int ret = regRead(dev, MAX3010X_REG_SPO2_CONFIG, &spo2_config);
	if (ret < 0)
		return ret;

	spo2_config &= (uint8_t)~MAX3010X_SPO2_PULSE_WIDTH_MASK;
	spo2_config |= (uint8_t)(width << MAX3010X_SPO2_PULSE_WIDTH_SHIFT);
	return regWrite(dev, MAX3010X_REG_SPO2_CONFIG, spo2_config);
}

int max3010xSetLEDAmplitude(const struct device* const dev, const MAX3010xLEDAmplitude* const config)
{
	if (dev == NULL || config == NULL)
		return -EINVAL;

	const MAX3010xDriverConfig* const cfg = dev->config;
	const bool is_max30101 = (cfg->variant == MAX3010X_VARIANT_MAX30101);
	MAX3010xDriverData* const data = dev->data;

	int ret = regWriteVerify(dev, MAX3010X_REG_LED1_PA, config->red, 0xFF);
	if (ret < 0)
		return ret;

	ret = regWriteVerify(dev, MAX3010X_REG_LED2_PA, config->ir, 0xFF);
	if (ret < 0)
		return ret;

	/* LED3_PA (0x0E) is MAX30101-only (Green LED).
	 * LED4_PA (0x0F) is reserved on both MAX30101 and MAX30102 (MAX30105 only).
	 * PILOT_PA (0x10) is valid on both variants (proximity pilot pulse). */
	if (is_max30101)
	{
		ret = regWriteVerify(dev, MAX3010X_REG_LED3_PA, config->green, 0xFF);
		if (ret < 0)
			return ret;
	}

	ret = regWriteVerify(dev, MAX3010X_REG_PILOT_PA, config->pilot, 0xFF);
	if (ret < 0)
		return ret;

	// Update saved LED cache for enable/disable cycling
	data->saved_led[0] = config->red;
	data->saved_led[1] = config->ir;
	data->saved_led[2] = config->green;
	data->saved_led[3] = config->green2;
	data->saved_led[4] = config->pilot;

	return 0;
}

int max3010xSetLEDChannelAmplitude(const struct device* const dev, const MAX3010xLEDChannel channel, const uint8_t value)
{
	if (dev == NULL)
		return -EINVAL;

	const MAX3010xDriverConfig* const cfg = dev->config;
	MAX3010xDriverData* const data = dev->data;
	int ret;

	switch (channel)
	{
		case MAX3010X_LED_RED:
			ret = regWrite(dev, MAX3010X_REG_LED1_PA, value);
			if (ret == 0)
				data->saved_led[0] = value;
			return ret;
		case MAX3010X_LED_IR:
			ret = regWrite(dev, MAX3010X_REG_LED2_PA, value);
			if (ret == 0)
				data->saved_led[1] = value;
			return ret;
		case MAX3010X_LED_GREEN:
			if (cfg->variant == MAX3010X_VARIANT_MAX30102)
				return -ENOTSUP;
			ret = regWrite(dev, MAX3010X_REG_LED3_PA, value);
			if (ret == 0)
				data->saved_led[2] = value;
			return ret;
		default:
			return -EINVAL;
	}
}

int max3010xSetSlots(const struct device* const dev, const MAX3010xSlotConfig* const config)
{
	if (dev == NULL || config == NULL)
		return -EINVAL;

	const MAX3010xDriverConfig* const cfg = dev->config;
	const MAX3010xSlot slot0_value = sanitizeSlot(cfg, config->slot[0]);
	const MAX3010xSlot slot1_value = sanitizeSlot(cfg, config->slot[1]);
	const MAX3010xSlot slot2_value = sanitizeSlot(cfg, config->slot[2]);
	const MAX3010xSlot slot3_value = sanitizeSlot(cfg, config->slot[3]);

	const uint8_t ctrl1 = (uint8_t)(slot1_value << 4) | (uint8_t)slot0_value;
	const uint8_t ctrl2 = (uint8_t)(slot3_value << 4) | (uint8_t)slot2_value;

	int ret = regWriteVerify(dev, MAX3010X_REG_MULTI_LED_CTRL1, ctrl1, 0x77);
	if (ret < 0)
		return ret;

	ret = regWriteVerify(dev, MAX3010X_REG_MULTI_LED_CTRL2, ctrl2, 0x77);
	if (ret < 0)
		return ret;

	MAX3010xDriverData* const data = dev->data;
	buildChannelMap(cfg, config, &data->map);
	return 0;
}

int max3010xApplyDefaultConfig(const struct device* const dev)
{
	if (dev == NULL)
		return -EINVAL;

	const MAX3010xDriverConfig* const cfg = dev->config;
	MAX3010xDriverData* const data = dev->data;

	/*
	 * Reset device to get clean POR state, then reconfigure.
	 * All config registers are writable in MODE=0 (POR default).
	 * max3010xEnable() activates sampling when the app starts.
	 */
	data->current_mode = (uint8_t)(cfg->mode & MAX3010X_MODE_MASK);

	int ret = max3010xReset(dev);
	if (ret < 0)
		return ret;

	ret = max3010xFlushFIFO(dev);
	if (ret < 0)
		return ret;

	ret = max3010xSetFIFOConfig(dev, &cfg->fifo);
	if (ret < 0)
		return ret;

	ret = max3010xSetSPO2Config(dev, &cfg->spo2);
	if (ret < 0)
		return ret;

	ret = max3010xSetLEDAmplitude(dev, &cfg->led_amplitude);
	if (ret < 0)
		return ret;

	ret = max3010xSetSlots(dev, &cfg->slots);
	if (ret < 0)
		return ret;

	// Cache LED amplitudes for enable/disable cycling
	data->saved_led[0] = cfg->led_amplitude.red;
	data->saved_led[1] = cfg->led_amplitude.ir;
	data->saved_led[2] = cfg->led_amplitude.green;
	data->saved_led[3] = cfg->led_amplitude.green2;
	data->saved_led[4] = cfg->led_amplitude.pilot;

	return 0;
}

int max3010xSetInterrupts(const struct device* const dev, const uint8_t interrupt_enable_1_mask, const uint8_t interrupt_enable_2_mask)
{
	if (dev == NULL)
		return -EINVAL;

	const int ret = regWrite(dev, MAX3010X_REG_INT_ENABLE_1, interrupt_enable_1_mask);
	if (ret < 0)
		return ret;

	return regWrite(dev, MAX3010X_REG_INT_ENABLE_2, interrupt_enable_2_mask);
}

int max3010xSetProximityThreshold(const struct device* const dev, const uint8_t threshold)
{
	if (dev == NULL)
		return -EINVAL;

	return regWrite(dev, MAX3010X_REG_PROX_INT_THRESH, threshold);
}

int max3010xSetTemperatureEnabled(const struct device* const dev, const bool is_enabled)
{
	if (dev == NULL)
		return -EINVAL;

	const uint8_t temp_config = is_enabled ? MAX3010X_TEMP_EN : 0;
	return regWrite(dev, MAX3010X_REG_TEMP_CONFIG, temp_config);
}

int max3010xGetTemperature(const struct device* const dev, float* const temperature_deg_c)
{
	if (dev == NULL || temperature_deg_c == NULL)
		return -EINVAL;

	/* Temperature conversion requires the chip to be out of shutdown.
	 * If currently in shutdown, temporarily wake it for the measurement. */
	uint8_t mode_cfg = 0;
	int ret = regRead(dev, MAX3010X_REG_MODE_CONFIG, &mode_cfg);
	if (ret < 0)
		return ret;

	const bool was_shutdown = (mode_cfg & MAX3010X_MODE_SHDN) != 0;
	const uint8_t mode_bits = mode_cfg & MAX3010X_MODE_MASK;
	if (was_shutdown)
	{
		/* Wake from shutdown with the chip's configured mode.
		 * Some parts trigger POR on mode activation, so we handle retry. */
		const uint8_t wake_mode = (mode_bits != 0) ? mode_bits : 0x07;
		ret = regWrite(dev, MAX3010X_REG_MODE_CONFIG, wake_mode);
		if (ret < 0)
			return ret;
		k_msleep(10); // Allow POR + oscillator startup

		// Check if POR occurred (resets MODE_CONFIG to 0x00)
		uint8_t mode_check = 0;
		regRead(dev, MAX3010X_REG_MODE_CONFIG, &mode_check);
		if ((mode_check & MAX3010X_MODE_MASK) != wake_mode)
		{
			// POR happened -- clear interrupt status and re-write mode
			uint8_t int_clear;
			regRead(dev, MAX3010X_REG_INT_STATUS_1, &int_clear);
			regRead(dev, MAX3010X_REG_INT_STATUS_2, &int_clear);
			regWrite(dev, MAX3010X_REG_MODE_CONFIG, wake_mode);
			k_msleep(10);
		}

		// Clear any pending interrupt status before triggering temp
		uint8_t int_clear;
		regRead(dev, MAX3010X_REG_INT_STATUS_1, &int_clear);
		regRead(dev, MAX3010X_REG_INT_STATUS_2, &int_clear);
	}

	/* Trigger temperature conversion.
	 * Per datasheet, conversion takes ~30 ms. TEMP_EN self-clears when done.
	 * On some chips, TEMP_EN clears before the ADC result updates the
	 * temperature registers, so we enforce a minimum wait. */
	ret = regWrite(dev, MAX3010X_REG_TEMP_CONFIG, MAX3010X_TEMP_EN);
	if (ret < 0)
	{
		if (was_shutdown)
		{
			const int restore_ret = regWrite(dev, MAX3010X_REG_MODE_CONFIG, mode_cfg);
			if (restore_ret < 0)
				LOG_WRN("Failed to restore shutdown after temp trigger error: %d", restore_ret);
		}
		return ret;
	}

	// Wait for conversion: poll TEMP_EN and enforce minimum 30 ms delay
	uint8_t temp_cfg = MAX3010X_TEMP_EN;
	int elapsed_ms = 0;

	do
	{
		k_msleep(5);
		elapsed_ms += 5;
		ret = regRead(dev, MAX3010X_REG_TEMP_CONFIG, &temp_cfg);
		if (ret < 0)
		{
			if (was_shutdown)
			{
				const int restore_ret = regWrite(dev, MAX3010X_REG_MODE_CONFIG, mode_cfg);
				if (restore_ret < 0)
					LOG_WRN("Failed to restore shutdown after temp poll error: %d", restore_ret);
			}
			return ret;
		}
	} while (((temp_cfg & MAX3010X_TEMP_EN) || elapsed_ms < 30) &&
			 elapsed_ms < MAX3010X_TEMP_CONV_WAIT_MS);

	if (elapsed_ms >= MAX3010X_TEMP_CONV_WAIT_MS)
	{
		LOG_WRN("Temperature conversion timeout");
		if (was_shutdown)
		{
			const int restore_ret = regWrite(dev, MAX3010X_REG_MODE_CONFIG, mode_cfg);
			if (restore_ret < 0)
				LOG_WRN("Failed to restore shutdown after temp timeout: %d", restore_ret);
		}
		return -ETIMEDOUT;
	}

	uint8_t temp_int = 0;
	ret = regRead(dev, MAX3010X_REG_TEMP_INT, &temp_int);
	if (ret < 0)
	{
		if (was_shutdown)
		{
			const int restore_ret = regWrite(dev, MAX3010X_REG_MODE_CONFIG, mode_cfg);
			if (restore_ret < 0)
				LOG_WRN("Failed to restore shutdown after temp_int read error: %d", restore_ret);
		}
		return ret;
	}

	uint8_t temp_frac = 0;
	ret = regRead(dev, MAX3010X_REG_TEMP_FRAC, &temp_frac);
	if (ret < 0)
	{
		if (was_shutdown)
		{
			const int restore_ret = regWrite(dev, MAX3010X_REG_MODE_CONFIG, mode_cfg);
			if (restore_ret < 0)
				LOG_WRN("Failed to restore shutdown after temp_frac read error: %d", restore_ret);
		}
		return ret;
	}

	// Restore shutdown if we woke the chip
	if (was_shutdown)
	{
		const int restore_ret = regWrite(dev, MAX3010X_REG_MODE_CONFIG, mode_cfg);
		if (restore_ret < 0)
			LOG_WRN("Failed to restore shutdown after temp read: %d", restore_ret);
	}

	// TEMP_INT is signed 8-bit, TEMP_FRAC is 4-bit (0.0625 deg C per LSB)
	*temperature_deg_c = (float)(int8_t)temp_int +
						 ((float)(temp_frac & MAX3010X_TEMP_FRAC_MASK) * 0.0625f);
	return 0;
}

int max3010xGetInterruptStatus(const struct device* const dev, uint8_t* const int1, uint8_t* const int2)
{
	if (dev == NULL)
		return -EINVAL;

	if (int1 != NULL)
	{
		const int ret = regRead(dev, MAX3010X_REG_INT_STATUS_1, int1);
		if (ret < 0)
			return ret;
	}

	if (int2 != NULL)
	{
		const int ret = regRead(dev, MAX3010X_REG_INT_STATUS_2, int2);
		if (ret < 0)
			return ret;
	}

	return 0;
}

int max3010xReadFIFO(const struct device* const dev, uint8_t* const buffer, const size_t size_bytes)
{
	if (dev == NULL || buffer == NULL || size_bytes == 0)
		return -EINVAL;

	return regReadBurst(dev, MAX3010X_REG_FIFO_DATA, buffer, size_bytes);
}

int max3010xFlushFIFO(const struct device* const dev)
{
	if (dev == NULL)
		return -EINVAL;

	int ret = regWrite(dev, MAX3010X_REG_FIFO_WR_PTR, 0);
	if (ret < 0)
		return ret;

	ret = regWrite(dev, MAX3010X_REG_OVF_COUNTER, 0);
	if (ret < 0)
		return ret;

	ret = regWrite(dev, MAX3010X_REG_FIFO_RD_PTR, 0);
	if (ret < 0)
		return ret;

	LOG_DBG("FIFO flushed");
	return 0;
}

int max3010xGetRawChannel(const struct device* const dev, const MAX3010xLEDChannel channel, uint32_t* const value)
{
	if (dev == NULL || value == NULL)
		return -EINVAL;

	const MAX3010xDriverData* const data = dev->data;
	const uint8_t fifo_index = data->map.fifo_index[channel];

	if (fifo_index >= MAX3010X_MAX_NUM_SLOTS)
		return -ENOTSUP;

	*value = data->data.raw[fifo_index];
	return 0;
}

uint8_t max3010xGetNumChannels(const struct device* const dev)
{
	if (dev == NULL)
		return 0;

	const MAX3010xDriverData* const data = dev->data;
	return data->map.active_channel_count;
}

int max3010xSetInterruptCallback(const struct device* const dev, const MAX3010xCallback callback, void* const user_data)
{
	if (dev == NULL)
		return -EINVAL;

	// Just store the pointer; interrupt enable/disable is handled by max3010xEnableInterrupt()
	MAX3010xDriverData* const data = dev->data;
	data->int_callback = callback;
	data->int_user_data = user_data;

	return 0;
}

int max3010xEnableInterrupt(const struct device* const dev, const bool is_enabled)
{
	if (dev == NULL)
		return -EINVAL;

#if !IS_ENABLED(CONFIG_MAX3010X_IRQ_ENABLE)
	return -ENOTSUP;
#else
	const MAX3010xDriverConfig* const cfg = dev->config;

	if (!hasGPIO(&cfg->int_gpio))
		return -ENOTSUP;

	if (is_enabled)
	{
		LOG_DBG("Enabling INT GPIO interrupt");
		return gpio_pin_interrupt_configure_dt(&cfg->int_gpio,
											   GPIO_INT_EDGE_TO_ACTIVE);
	}
	else
	{
		LOG_DBG("Disabling INT GPIO interrupt");
		return gpio_pin_interrupt_configure_dt(&cfg->int_gpio,
											   GPIO_INT_DISABLE);
	}
#endif
}

uint32_t max3010xGetSampleCount(const struct device* const dev)
{
	if (dev == NULL)
		return 0;

	const MAX3010xDriverData* const data = dev->data;
	return (uint32_t)atomic_get((const atomic_t*)&data->sample_count);
}

void max3010xResetSampleCount(const struct device* const dev)
{
	if (dev == NULL)
		return;

	MAX3010xDriverData* const data = dev->data;
	atomic_set(&data->sample_count, 0);
}

uint32_t max3010xGetAndResetSampleCount(const struct device* const dev)
{
	if (dev == NULL)
		return 0;

	MAX3010xDriverData* const data = dev->data;
	return (uint32_t)atomic_clear(&data->sample_count);
}

int max3010xDumpRegisters(const struct device* const dev)
{
	if (dev == NULL)
		return -EINVAL;

	uint8_t register_value = 0;

	regRead(dev, MAX3010X_REG_MODE_CONFIG, &register_value);
	LOG_DBG("REG MODE_CONFIG=0x%02x (SHDN=%u RST=%u MODE=%u)",
			register_value, (register_value >> 7) & 1, (register_value >> 6) & 1, register_value & 0x07);

	regRead(dev, MAX3010X_REG_FIFO_CONFIG, &register_value);
	LOG_DBG("REG FIFO_CONFIG=0x%02x (AVG=%u RO=%u AF=%u)",
			register_value, (register_value >> 5) & 7, (register_value >> 4) & 1, register_value & 0x0F);

	regRead(dev, MAX3010X_REG_SPO2_CONFIG, &register_value);
	LOG_DBG("REG SPO2_CONFIG=0x%02x (ADC=%u SR=%u PW=%u)",
			register_value, (register_value >> 5) & 3, (register_value >> 2) & 7, register_value & 3);

	regRead(dev, MAX3010X_REG_LED1_PA, &register_value);
	LOG_DBG("REG LED1_PA=0x%02x", register_value);

	regRead(dev, MAX3010X_REG_LED2_PA, &register_value);
	LOG_DBG("REG LED2_PA=0x%02x", register_value);

	regRead(dev, MAX3010X_REG_LED3_PA, &register_value);
	LOG_DBG("REG LED3_PA=0x%02x", register_value);

	regRead(dev, MAX3010X_REG_MULTI_LED_CTRL1, &register_value);
	LOG_DBG("REG CTRL1=0x%02x (slot2=%u slot1=%u)",
			register_value, (register_value >> 4) & 7, register_value & 7);

	regRead(dev, MAX3010X_REG_MULTI_LED_CTRL2, &register_value);
	LOG_DBG("REG CTRL2=0x%02x (slot4=%u slot3=%u)",
			register_value, (register_value >> 4) & 7, register_value & 7);

	regRead(dev, MAX3010X_REG_INT_ENABLE_1, &register_value);
	LOG_DBG("REG INT_EN1=0x%02x", register_value);

	uint8_t fifo_write_pointer = 0;
	uint8_t fifo_read_pointer = 0;
	uint8_t overflow_count = 0;
	regRead(dev, MAX3010X_REG_FIFO_WR_PTR, &fifo_write_pointer);
	regRead(dev, MAX3010X_REG_FIFO_RD_PTR, &fifo_read_pointer);
	regRead(dev, MAX3010X_REG_OVF_COUNTER, &overflow_count);
	LOG_DBG("REG FIFO WR=%u RD=%u OVF=%u",
			fifo_write_pointer & 0x1F, fifo_read_pointer & 0x1F, overflow_count & 0x1F);

	return 0;
}

int max3010xReadInterruptPin(const struct device* const dev, int* const state)
{
	if (dev == NULL || state == NULL)
		return -EINVAL;

	const MAX3010xDriverConfig* const cfg = dev->config;

	if (!hasGPIO(&cfg->int_gpio))
		return -ENOTSUP;

	const int pin_state = gpio_pin_get_dt(&cfg->int_gpio);
	if (pin_state < 0)
		return pin_state;

	*state = pin_state;
	return 0;
}

bool max3010xHasInterruptGPIO(const struct device* const dev)
{
	if (dev == NULL)
		return false;

	const MAX3010xDriverConfig* const cfg = dev->config;
	return hasGPIO(&cfg->int_gpio);
}

int max3010xGetAvailableSamples(const struct device* const dev, uint8_t* const count)
{
	if (dev == NULL || count == NULL)
		return -EINVAL;

	uint8_t write_pointer_raw = 0;
	int ret = regRead(dev, MAX3010X_REG_FIFO_WR_PTR, &write_pointer_raw);
	if (ret < 0)
		return ret;

	uint8_t read_pointer_raw = 0;
	ret = regRead(dev, MAX3010X_REG_FIFO_RD_PTR, &read_pointer_raw);
	if (ret < 0)
		return ret;

	const uint8_t write_pointer_masked = write_pointer_raw & MAX3010X_FIFO_PTR_MASK;
	const uint8_t read_pointer_masked = read_pointer_raw & MAX3010X_FIFO_PTR_MASK;

	if (write_pointer_masked >= read_pointer_masked)
		*count = write_pointer_masked - read_pointer_masked;
	else
		*count = (uint8_t)(MAX3010X_FIFO_DEPTH + write_pointer_masked - read_pointer_masked);

	return 0;
}

int max3010xGetOverflowCount(const struct device* const dev, uint8_t* const count)
{
	if (dev == NULL || count == NULL)
		return -EINVAL;

	uint8_t overflow_count = 0;
	const int ret = regRead(dev, MAX3010X_REG_OVF_COUNTER, &overflow_count);
	if (ret < 0)
		return ret;

	*count = overflow_count & MAX3010X_OVF_COUNTER_MASK;
	return 0;
}

int max3010xGetPartID(const struct device* const dev, uint8_t* const part_id)
{
	if (dev == NULL || part_id == NULL)
		return -EINVAL;

	return regRead(dev, MAX3010X_REG_PART_ID, part_id);
}

int max3010xGetRevisionID(const struct device* const dev, uint8_t* const rev_id)
{
	if (dev == NULL || rev_id == NULL)
		return -EINVAL;

	return regRead(dev, MAX3010X_REG_REV_ID, rev_id);
}

static int regRead(const struct device* const dev, const uint8_t reg, uint8_t* const value)
{
	const MAX3010xDriverConfig* const cfg = dev->config;
	return i2c_reg_read_byte_dt(&cfg->i2c, reg, value);
}

static int regWrite(const struct device* const dev, const uint8_t reg, const uint8_t value)
{
	const MAX3010xDriverConfig* const cfg = dev->config;
	return i2c_reg_write_byte_dt(&cfg->i2c, reg, value);
}

/**
 * @brief Write register with readback verification
 *
 * Writes a value, reads it back, and warns on mismatch.
 * Use mask to ignore read-only or self-clearing bits.
 *
 * @param[in] dev   Device handle
 * @param[in] reg   Register address
 * @param[in] value Value to write
 * @param[in] mask  Bits to verify (0xFF = verify all)
 * @return 0 on success, negative errno on failure, -EIO on verify mismatch
 */
static int regWriteVerify(const struct device* const dev, const uint8_t reg, const uint8_t value, const uint8_t mask)
{
	const int write_ret = regWrite(dev, reg, value);
	if (write_ret < 0)
	{
		LOG_ERR("Write reg 0x%02x=0x%02x failed: %d", reg, value, write_ret);
		return write_ret;
	}

	uint8_t readback = 0;
	const int read_ret = regRead(dev, reg, &readback);
	if (read_ret < 0)
	{
		LOG_ERR("Readback reg 0x%02x failed: %d", reg, read_ret);
		return read_ret;
	}

	if ((readback & mask) != (value & mask))
	{
		/*
		 * Some registers (notably SPO2_CONFIG) may not latch on first
		 * write. Retry up to 3 times with a 1 ms delay between attempts
		 * before reporting failure.
		 */
		for (int retry = 0; retry < 3; retry++)
		{
			k_msleep(1);

			const int rw_ret = regWrite(dev, reg, value);
			if (rw_ret < 0)
			{
				LOG_ERR("Retry %d write reg 0x%02x failed: %d", retry, reg, rw_ret);
				return rw_ret;
			}

			const int rr_ret = regRead(dev, reg, &readback);
			if (rr_ret < 0)
			{
				LOG_ERR("Retry %d readback reg 0x%02x failed: %d", retry, reg, rr_ret);
				return rr_ret;
			}

			if ((readback & mask) == (value & mask))
			{
				LOG_DBG("Reg 0x%02x verify OK after retry %d: 0x%02x", reg, retry, readback);
				return 0;
			}
		}

		LOG_ERR("Reg 0x%02x verify FAIL after retries: wrote 0x%02x, read 0x%02x (mask 0x%02x)",
				reg, value, readback, mask);
		return -EIO;
	}

	LOG_DBG("Reg 0x%02x verify OK: 0x%02x", reg, readback);
	return 0;
}

static int regReadBurst(const struct device* const dev, const uint8_t reg, uint8_t* const buffer, const size_t length)
{
	const MAX3010xDriverConfig* const cfg = dev->config;
	return i2c_burst_read_dt(&cfg->i2c, reg, buffer, length);
}

static bool hasGPIO(const struct gpio_dt_spec* const spec)
{
	return spec->port != NULL;
}

static void intGPIOHandler(const struct device* const port, struct gpio_callback* const cb, const gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	MAX3010xDriverData* const data =
		CONTAINER_OF(cb, MAX3010xDriverData, gpio_cb);

	atomic_inc(&data->sample_count);

	// Snapshot callback + user_data locally to avoid TOCTOU
	const MAX3010xCallback callback = data->int_callback;
	void* const user_data = data->int_user_data;
	if (callback != NULL)
		callback(user_data);
}

static void buildChannelMap(const MAX3010xDriverConfig* const cfg, const MAX3010xSlotConfig* const slots, MAX3010xChannelMap* const map)
{
	map->active_channel_count = 0;
	for (uint32_t channel_index = 0; channel_index < MAX3010X_MAX_NUM_CHANNELS; channel_index++)
		map->fifo_index[channel_index] = MAX3010X_MAX_NUM_SLOTS;

	// Iterate all 4 slots - each active slot produces 3 bytes in the FIFO
	for (uint32_t slot_idx = 0; slot_idx < MAX3010X_MAX_NUM_SLOTS; slot_idx++)
	{
		const uint8_t slot_value = (uint8_t)slots->slot[slot_idx] &
								   MAX3010X_MULTI_LED_SLOT_MASK;
		if (slot_value == 0)
			continue;

		/* Slot values 1-3 map to LED channels 0-2 (Red, IR, Green).
		 * Slot values 5-7 are pilot mode - also map to channels 0-2. */
		const uint8_t led_channel = (slot_value <= 3)
										? (slot_value - 1)
										: (slot_value - 5);

		if (cfg->variant == MAX3010X_VARIANT_MAX30102 &&
			led_channel == MAX3010X_LED_GREEN)
			continue;

		if (led_channel < MAX3010X_MAX_NUM_CHANNELS)
		{
			// First occurrence wins if a channel appears in multiple slots
			if (map->fifo_index[led_channel] >= MAX3010X_MAX_NUM_SLOTS)
				map->fifo_index[led_channel] = (uint8_t)map->active_channel_count;
		}
		map->active_channel_count++;
	}
}

static MAX3010xSlot sanitizeSlot(const MAX3010xDriverConfig* const cfg, const MAX3010xSlot slot)
{
	if (cfg->variant != MAX3010X_VARIANT_MAX30102)
		return slot;

	if (slot == MAX3010X_SLOT_GREEN_LED3_PA ||
		slot == MAX3010X_SLOT_GREEN_PILOT_PA)
		return MAX3010X_SLOT_DISABLED;

	return slot;
}

static int sensorSampleFetch(const struct device* const dev, const enum sensor_channel chan)
{
	ARG_UNUSED(chan);

	MAX3010xDriverData* const data = dev->data;
	const uint8_t channel_count = data->map.active_channel_count;
	const uint8_t bytes_to_read = channel_count * MAX3010X_BYTES_PER_CHANNEL;

	if (bytes_to_read == 0)
		return -ENODATA;

	uint8_t read_buffer[MAX3010X_MAX_NUM_SLOTS * MAX3010X_BYTES_PER_CHANNEL];
	const int ret = regReadBurst(dev, MAX3010X_REG_FIFO_DATA, read_buffer, bytes_to_read);
	if (ret < 0)
	{
		LOG_ERR("FIFO read failed: %d", ret);
		return ret;
	}

	for (uint32_t channel_index = 0; channel_index < channel_count; channel_index++)
	{
		const uint32_t base = channel_index * MAX3010X_BYTES_PER_CHANNEL;
		const uint32_t sample = ((uint32_t)read_buffer[base] << 16) |
								((uint32_t)read_buffer[base + 1] << 8) |
								(uint32_t)read_buffer[base + 2];
		data->data.raw[channel_index] = sample & MAX3010X_FIFO_DATA_MASK;
	}

	return 0;
}

static int sensorChannelGet(const struct device* const dev, const enum sensor_channel chan, struct sensor_value* const sensor_output)
{
	const MAX3010xDriverData* const data = dev->data;
	uint8_t led_channel;

	switch (chan)
	{
		case SENSOR_CHAN_RED:
			led_channel = MAX3010X_LED_RED;
			break;
		case SENSOR_CHAN_IR:
			led_channel = MAX3010X_LED_IR;
			break;
		case SENSOR_CHAN_GREEN:
			led_channel = MAX3010X_LED_GREEN;
			break;
		default:
			return -ENOTSUP;
	}

	const uint8_t fifo_index = data->map.fifo_index[led_channel];
	if (fifo_index >= MAX3010X_MAX_NUM_SLOTS)
		return -ENOTSUP;

	sensor_output->val1 = (int32_t)data->data.raw[fifo_index];
	sensor_output->val2 = 0;
	return 0;
}

static DEVICE_API(sensor, max3010x_api) =
	{
		.sample_fetch = sensorSampleFetch,
		.channel_get = sensorChannelGet,
};

static int max3010xInit(const struct device* const dev)
{
	MAX3010xDriverData* const data = dev->data;
	const MAX3010xDriverConfig* const cfg = dev->config;
	int ret;

	LOG_INF("Initializing MAX3010x at 0x%02x", cfg->i2c.addr);

	// Initialize runtime state
	data->dev = dev;
	data->int_callback = NULL;
	data->int_user_data = NULL;
	data->is_initialized = false;
	data->current_mode = (uint8_t)(cfg->mode & MAX3010X_MODE_MASK);
	k_mutex_init(&data->lock);
	atomic_set(&data->sample_count, 0);

	for (uint32_t slot_index = 0; slot_index < MAX3010X_MAX_NUM_SLOTS; slot_index++)
		data->data.raw[slot_index] = 0;

	for (uint32_t channel_index = 0; channel_index < MAX3010X_MAX_NUM_CHANNELS; channel_index++)
		data->map.fifo_index[channel_index] = MAX3010X_MAX_NUM_SLOTS;

	data->map.active_channel_count = 0;

	// Verify I2C bus ready
	if (!device_is_ready(cfg->i2c.bus))
	{
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	// Read and verify part ID (both MAX30101 and MAX30102 share 0x15)
	uint8_t part_id = 0;
	ret = regRead(dev, MAX3010X_REG_PART_ID, &part_id);
	if (ret < 0)
	{
		LOG_ERR("Failed to read part ID: %d", ret);
		return -EIO;
	}

	if (part_id != MAX3010X_PART_ID)
	{
		LOG_ERR("Unexpected part ID: 0x%02X (expected 0x%02X)",
				part_id, MAX3010X_PART_ID);
		return -ENODEV;
	}

	uint8_t rev_id = 0;
	ret = regRead(dev, MAX3010X_REG_REV_ID, &rev_id);
	if (ret < 0)
		LOG_WRN("Failed to read revision ID: %d", ret);

	LOG_DBG("Part ID: 0x%02X, Rev: 0x%02X, Variant: %s",
			part_id, rev_id,
			cfg->variant == MAX3010X_VARIANT_MAX30101 ? "MAX30101" : "MAX30102");

	// Soft reset and wait
	ret = max3010xReset(dev);
	if (ret < 0)
	{
		LOG_ERR("Reset failed: %d", ret);
		return ret;
	}

	// Flush FIFO pointers
	ret = max3010xFlushFIFO(dev);
	if (ret < 0)
	{
		LOG_ERR("Failed to flush FIFO: %d", ret);
		return ret;
	}

	/*
	 * Apply DT/Kconfig defaults.
	 *
	 * After reset, the device is in MODE=0 (POR default). All config
	 * registers are writable in this state. Configure everything here;
	 * max3010xEnable() will activate the mode when sampling starts.
	 */
	ret = max3010xSetFIFOConfig(dev, &cfg->fifo);
	if (ret < 0)
	{
		LOG_ERR("Failed to set FIFO config: %d", ret);
		return ret;
	}

	ret = max3010xSetSPO2Config(dev, &cfg->spo2);
	if (ret < 0)
	{
		LOG_ERR("Failed to set SpO2 config: %d", ret);
		return ret;
	}

	ret = max3010xSetLEDAmplitude(dev, &cfg->led_amplitude);
	if (ret < 0)
	{
		LOG_ERR("Failed to set LED amplitudes: %d", ret);
		return ret;
	}

	ret = max3010xSetSlots(dev, &cfg->slots);
	if (ret < 0)
	{
		LOG_ERR("Failed to set slots: %d", ret);
		return ret;
	}

	// Cache LED amplitudes for enable/disable cycling
	data->saved_led[0] = cfg->led_amplitude.red;
	data->saved_led[1] = cfg->led_amplitude.ir;
	data->saved_led[2] = cfg->led_amplitude.green;
	data->saved_led[3] = cfg->led_amplitude.green2;
	data->saved_led[4] = cfg->led_amplitude.pilot;

	// Apply Kconfig interrupt and proximity defaults
	if (cfg->interrupt_enable_1_mask != 0 || cfg->interrupt_enable_2_mask != 0)
	{
		ret = max3010xSetInterrupts(dev, cfg->interrupt_enable_1_mask, cfg->interrupt_enable_2_mask);
		if (ret < 0)
		{
			LOG_ERR("Failed to set interrupt masks: %d", ret);
			return ret;
		}
	}

	if (cfg->proximity_interrupt_threshold != 0)
	{
		ret = max3010xSetProximityThreshold(dev, cfg->proximity_interrupt_threshold);
		if (ret < 0)
		{
			LOG_ERR("Failed to set proximity threshold: %d", ret);
			return ret;
		}
	}

	if (cfg->is_temp_enabled)
	{
		ret = max3010xSetTemperatureEnabled(dev, true);
		if (ret < 0)
		{
			LOG_ERR("Failed to enable temperature: %d", ret);
			return ret;
		}
	}

	/*
	 * All configuration applied. Enter shutdown with mode configured.
	 * Enable() clears SHDN to start sampling. All register values
	 * persist through shutdown (per datasheet).
	 */
	ret = regWrite(dev, MAX3010X_REG_MODE_CONFIG,
				   MAX3010X_MODE_SHDN | data->current_mode);
	if (ret < 0)
	{
		LOG_ERR("Failed to enter shutdown: %d", ret);
		return ret;
	}

	LOG_DBG("Shutdown with MODE=0x%02x", data->current_mode);

	// Configure GPIO interrupt (but leave disabled)
#if IS_ENABLED(CONFIG_MAX3010X_IRQ_ENABLE)
	if (hasGPIO(&cfg->int_gpio))
	{
		if (!gpio_is_ready_dt(&cfg->int_gpio))
		{
			LOG_ERR("INT GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
		if (ret < 0)
		{
			LOG_ERR("INT GPIO config failed: %d", ret);
			return ret;
		}

		ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio,
											  GPIO_INT_DISABLE);
		if (ret < 0)
		{
			LOG_ERR("INT GPIO int disable failed: %d", ret);
			return ret;
		}

		gpio_init_callback(&data->gpio_cb, intGPIOHandler,
						   BIT(cfg->int_gpio.pin));
		ret = gpio_add_callback(cfg->int_gpio.port, &data->gpio_cb);
		if (ret < 0)
		{
			LOG_ERR("INT GPIO callback failed: %d", ret);
			return ret;
		}

		LOG_DBG("INT GPIO configured (disabled)");
	}
#else
	if (hasGPIO(&cfg->int_gpio))
	{
		if (gpio_is_ready_dt(&cfg->int_gpio))
		{
			ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
			if (ret < 0)
				LOG_WRN("INT GPIO config failed: %d (polling unavailable)", ret);
			else
				LOG_DBG("INT GPIO configured for polling (IRQ disabled)");
		}
	}
#endif

	data->is_initialized = true;
	LOG_INF("MAX3010x ready (configured, idle)");
	return 0;
}

// Default operating mode from Kconfig
#if defined(CONFIG_MAX3010X_MODE_HEART_RATE)
#define MAX3010X_DEFAULT_MODE MAX3010X_MODE_HEART_RATE
#elif defined(CONFIG_MAX3010X_MODE_SPO2)
#define MAX3010X_DEFAULT_MODE MAX3010X_MODE_SPO2
#else
#define MAX3010X_DEFAULT_MODE MAX3010X_MODE_MULTI_LED
#endif

#define MAX3010X_VARIANT_FROM_DT(node_id) \
	(DT_ENUM_IDX(node_id, variant) == 1   \
		 ? MAX3010X_VARIANT_MAX30102      \
		 : MAX3010X_VARIANT_MAX30101)

#define MAX3010X_DEFINE_NODE(node_id)                                         \
	static MAX3010xDriverData max3010x_data_##node_id;                        \
	static const MAX3010xDriverConfig max3010x_config_##node_id =             \
		{                                                                     \
			.i2c = I2C_DT_SPEC_GET(node_id),                                  \
			.int_gpio = GPIO_DT_SPEC_GET_OR(node_id, int_gpios, {0}),         \
			.variant = MAX3010X_VARIANT_FROM_DT(node_id),                     \
			.mode = MAX3010X_DEFAULT_MODE,                                    \
			.fifo =                                                           \
				{                                                             \
					.sample_average_mode = CONFIG_MAX3010X_SAMPLE_AVG,        \
					.almost_full_threshold = CONFIG_MAX3010X_FIFO_A_FULL,     \
					.is_rollover_enabled = CONFIG_MAX3010X_FIFO_ROLLOVER_EN,  \
				},                                                            \
			.spo2 =                                                           \
				{                                                             \
					.adc_range = CONFIG_MAX3010X_ADC_RANGE,                   \
					.sample_rate = CONFIG_MAX3010X_SAMPLE_RATE,               \
					.pulse_width = CONFIG_MAX3010X_PULSE_WIDTH,               \
				},                                                            \
			.slots =                                                          \
				{                                                             \
					.slot =                                                   \
						{                                                     \
							CONFIG_MAX3010X_SLOT1,                            \
							CONFIG_MAX3010X_SLOT2,                            \
							CONFIG_MAX3010X_SLOT3,                            \
							CONFIG_MAX3010X_SLOT4,                            \
						},                                                    \
				},                                                            \
			.led_amplitude =                                                  \
				{                                                             \
					.red = CONFIG_MAX3010X_LED1_PA,                           \
					.ir = CONFIG_MAX3010X_LED2_PA,                            \
					.green = CONFIG_MAX3010X_LED3_PA,                         \
					.green2 = CONFIG_MAX3010X_LED4_PA,                        \
					.pilot = CONFIG_MAX3010X_PILOT_PA,                        \
				},                                                            \
			.interrupt_enable_1_mask = CONFIG_MAX3010X_INT_ENABLE_1,          \
			.interrupt_enable_2_mask = CONFIG_MAX3010X_INT_ENABLE_2,          \
			.proximity_interrupt_threshold = CONFIG_MAX3010X_PROX_INT_THRESH, \
			.is_temp_enabled = IS_ENABLED(CONFIG_MAX3010X_TEMP_ENABLE),       \
	};                                                                        \
	DEVICE_DT_DEFINE(node_id, max3010xInit, NULL,                             \
					 &max3010x_data_##node_id,                                \
					 &max3010x_config_##node_id,                              \
					 POST_KERNEL, CONFIG_MAX3010X_INIT_PRIORITY,              \
					 &max3010x_api);

DT_FOREACH_STATUS_OKAY(maxim_max3010x, MAX3010X_DEFINE_NODE)
