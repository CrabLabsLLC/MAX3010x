/*
 * Copyright (c) 2025 Makani Science
 * SPDX-License-Identifier: Apache-2.0
 *
 * MAX3010x PPG/SpO2 sensor driver (MAX30101/MAX30102)
 *
 * Init sequence:
 *   1. Verify I2C ready
 *   2. Read and verify part ID
 *   3. Soft reset and wait
 *   4. Apply Kconfig/DT defaults (mode, FIFO, SpO2, LED currents, slots)
 *   5. Put device in SHUTDOWN mode
 *   6. Configure GPIO interrupt (but leave disabled)
 *
 * The device stays in shutdown until max3010x_enable() is called.
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "max3010x.h"

LOG_MODULE_REGISTER(max3010x, CONFIG_LOG_DEFAULT_LEVEL);

/* Runtime data - separate from const config */
typedef struct
{
	max3010x_data_t data;
	max3010x_channel_map_t map;
	max3010x_int_callback_t int_callback;
	void *int_user_data;
	struct gpio_callback gpio_cb;
	atomic_t sample_count;
	const struct device *dev;
} max3010x_runtime_t;

static int max3010x_reg_read(const max3010x_config_t *cfg, uint8_t reg, uint8_t *val)
{
	return i2c_reg_read_byte_dt(&cfg->i2c, reg, val);
}

static int max3010x_reg_write(const max3010x_config_t *cfg, uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte_dt(&cfg->i2c, reg, val);
}

static int max3010x_burst_read(const max3010x_config_t *cfg, uint8_t reg,
			       uint8_t *buf, size_t len)
{
	return i2c_burst_read_dt(&cfg->i2c, reg, buf, len);
}

static int max3010x_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	ARG_UNUSED(chan);

	max3010x_runtime_t *rt = dev->data;
	const max3010x_config_t *cfg = dev->config;
	uint8_t buffer[MAX3010X_MAX_NUM_CHANNELS * MAX3010X_BYTES_PER_CHANNEL];
	const int num_bytes = rt->map.active_channels * MAX3010X_BYTES_PER_CHANNEL;

	if (num_bytes == 0)
	{
		return -ENODATA;
	}

	if (max3010x_burst_read(cfg, MAX3010X_REG_FIFO_DATA, buffer, num_bytes))
	{
		LOG_ERR("FIFO read failed");
		return -EIO;
	}

	for (int i = 0; i < rt->map.active_channels; i++)
	{
		const int base = i * MAX3010X_BYTES_PER_CHANNEL;
		uint32_t sample = ((uint32_t)buffer[base] << 16) |
				  ((uint32_t)buffer[base + 1] << 8) |
				  (uint32_t)buffer[base + 2];
		rt->data.raw[i] = sample & MAX3010X_FIFO_DATA_MASK;
	}

	return 0;
}

static int max3010x_channel_get(const struct device *dev, enum sensor_channel chan,
			struct sensor_value *val)
{
	max3010x_runtime_t *rt = dev->data;
	uint8_t led_chan;
	uint8_t fifo_chan;

	switch (chan)
	{
	case SENSOR_CHAN_RED:
		led_chan = MAX3010X_LED_RED;
		break;
	case SENSOR_CHAN_IR:
		led_chan = MAX3010X_LED_IR;
		break;
	case SENSOR_CHAN_GREEN:
		led_chan = MAX3010X_LED_GREEN;
		break;
	default:
		return -ENOTSUP;
	}

	fifo_chan = rt->map.fifo_index[led_chan];
	if (fifo_chan >= MAX3010X_MAX_NUM_CHANNELS)
	{
		return -ENOTSUP;
	}

	val->val1 = rt->data.raw[fifo_chan];
	val->val2 = 0;
	return 0;
}

static void max3010x_gpio_handler(const struct device *port, struct gpio_callback *cb,
				  uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	max3010x_runtime_t *rt = CONTAINER_OF(cb, max3010x_runtime_t, gpio_cb);
	const struct device *dev = rt->dev;

	/* Always increment sample counter on interrupt */
	atomic_inc(&rt->sample_count);

	/* Invoke callback if registered (ISR context) */
	if (rt->int_callback)
	{
		rt->int_callback(dev, rt->int_user_data);
	}
}

static DEVICE_API(sensor, max3010x_api) = {
	.sample_fetch = max3010x_sample_fetch,
	.channel_get = max3010x_channel_get,
};

// Default operating mode from Kconfig
#if defined(CONFIG_MAX3010X_MODE_HEART_RATE)
#define MAX3010X_DEFAULT_MODE MAX3010X_MODE_HEART_RATE
#elif defined(CONFIG_MAX3010X_MODE_SPO2)
#define MAX3010X_DEFAULT_MODE MAX3010X_MODE_SPO2
#else
#define MAX3010X_DEFAULT_MODE MAX3010X_MODE_MULTI_LED
#endif

#if defined(CONFIG_SENSOR_INIT_PRIORITY)
#define MAX3010X_INIT_PRIORITY CONFIG_SENSOR_INIT_PRIORITY
#else
#define MAX3010X_INIT_PRIORITY 90
#endif

static void max3010x_build_channel_map(const max3010x_config_t *cfg,
				       max3010x_channel_map_t *map)
{
	map->active_channels = 0;
	for (int i = 0; i < MAX3010X_MAX_NUM_CHANNELS; i++)
	{
		map->fifo_index[i] = MAX3010X_MAX_NUM_CHANNELS;
	}

	for (int fifo = 0; fifo < MAX3010X_MAX_NUM_CHANNELS; fifo++)
	{
		const uint8_t slot = (uint8_t)cfg->slots.slot[fifo] & 0x03;
		if (slot == 0)
		{
			continue;
		}

		const uint8_t led_chan = slot - 1;
		if (cfg->variant == MAX3010X_VARIANT_MAX30102 && led_chan == MAX3010X_LED_GREEN)
		{
			continue;
		}

		if (led_chan < MAX3010X_MAX_NUM_CHANNELS)
		{
			map->fifo_index[led_chan] = fifo;
			map->active_channels++;
		}
	}
}

static max3010x_slot_t max3010x_sanitize_slot(const max3010x_config_t *cfg,
					      max3010x_slot_t slot)
{
	if (cfg->variant != MAX3010X_VARIANT_MAX30102)
	{
		return slot;
	}

	if (slot == MAX3010X_SLOT_GREEN_LED3_PA ||
	    slot == MAX3010X_SLOT_GREEN_PILOT_PA)
	{
		return MAX3010X_SLOT_DISABLED;
	}

	return slot;
}

int max3010x_reset(const struct device *dev)
{
	const max3010x_config_t *cfg = dev->config;
	uint8_t mode_cfg = 0;
	int timeout_ms = 100;

	if (max3010x_reg_write(cfg, MAX3010X_REG_MODE_CONFIG, MAX3010X_MODE_RESET))
	{
		return -EIO;
	}

	do
	{
		k_msleep(1);
		if (max3010x_reg_read(cfg, MAX3010X_REG_MODE_CONFIG, &mode_cfg))
		{
			return -EIO;
		}
		timeout_ms--;
	} while ((mode_cfg & MAX3010X_MODE_RESET) && (timeout_ms > 0));

	if (timeout_ms <= 0)
	{
		return -ETIMEDOUT;
	}

	return 0;
}

int max3010x_set_mode(const struct device *dev, max3010x_mode_t mode)
{
	const max3010x_config_t *cfg = dev->config;
	uint8_t mode_cfg = 0;

	if (max3010x_reg_read(cfg, MAX3010X_REG_MODE_CONFIG, &mode_cfg))
	{
		return -EIO;
	}

	mode_cfg &= (uint8_t)~MAX3010X_MODE_MASK;
	mode_cfg |= (uint8_t)(mode & MAX3010X_MODE_MASK);

	return max3010x_reg_write(cfg, MAX3010X_REG_MODE_CONFIG, mode_cfg);
}

int max3010x_enable(const struct device *dev)
{
	const max3010x_config_t *cfg = dev->config;
	uint8_t mode_cfg = 0;

	if (max3010x_reg_read(cfg, MAX3010X_REG_MODE_CONFIG, &mode_cfg))
	{
		return -EIO;
	}

	mode_cfg &= (uint8_t)~MAX3010X_MODE_SHDN;
	return max3010x_reg_write(cfg, MAX3010X_REG_MODE_CONFIG, mode_cfg);
}

int max3010x_disable(const struct device *dev)
{
	const max3010x_config_t *cfg = dev->config;
	uint8_t mode_cfg = 0;

	if (max3010x_reg_read(cfg, MAX3010X_REG_MODE_CONFIG, &mode_cfg))
	{
		return -EIO;
	}

	mode_cfg |= MAX3010X_MODE_SHDN;
	return max3010x_reg_write(cfg, MAX3010X_REG_MODE_CONFIG, mode_cfg);
}

int max3010x_set_fifo_config(const struct device *dev,
			     const max3010x_fifo_config_t *cfg_in)
{
	if (!cfg_in)
	{
		return -EINVAL;
	}

	const max3010x_config_t *cfg = dev->config;
	uint8_t fifo_cfg = (uint8_t)(cfg_in->sample_avg << MAX3010X_FIFO_SMP_AVE_SHIFT) |
			   (uint8_t)(cfg_in->almost_full & MAX3010X_FIFO_A_FULL_MASK);
	if (cfg_in->rollover)
	{
		fifo_cfg |= MAX3010X_FIFO_ROLLOVER_EN;
	}

	return max3010x_reg_write(cfg, MAX3010X_REG_FIFO_CONFIG, fifo_cfg);
}

int max3010x_set_spo2_config(const struct device *dev,
			     const max3010x_spo2_config_t *cfg_in)
{
	if (!cfg_in)
	{
		return -EINVAL;
	}

	const max3010x_config_t *cfg = dev->config;
	uint8_t spo2_cfg = (uint8_t)(cfg_in->adc_range << MAX3010X_SPO2_ADC_RGE_SHIFT) |
			   (uint8_t)(cfg_in->sample_rate << MAX3010X_SPO2_SR_SHIFT) |
			   (uint8_t)(cfg_in->pulse_width << MAX3010X_SPO2_PW_SHIFT);

	return max3010x_reg_write(cfg, MAX3010X_REG_SPO2_CONFIG, spo2_cfg);
}

int max3010x_set_adc_range(const struct device *dev, max3010x_adc_range_t range)
{
	const max3010x_config_t *cfg = dev->config;
	uint8_t spo2_cfg = 0;

	if (max3010x_reg_read(cfg, MAX3010X_REG_SPO2_CONFIG, &spo2_cfg))
	{
		return -EIO;
	}

	spo2_cfg &= (uint8_t)~MAX3010X_SPO2_ADC_RGE_MASK;
	spo2_cfg |= (uint8_t)(range << MAX3010X_SPO2_ADC_RGE_SHIFT);
	return max3010x_reg_write(cfg, MAX3010X_REG_SPO2_CONFIG, spo2_cfg);
}

int max3010x_set_sample_rate(const struct device *dev, max3010x_sample_rate_t rate)
{
	const max3010x_config_t *cfg = dev->config;
	uint8_t spo2_cfg = 0;

	if (max3010x_reg_read(cfg, MAX3010X_REG_SPO2_CONFIG, &spo2_cfg))
	{
		return -EIO;
	}

	spo2_cfg &= (uint8_t)~MAX3010X_SPO2_SR_MASK;
	spo2_cfg |= (uint8_t)(rate << MAX3010X_SPO2_SR_SHIFT);
	return max3010x_reg_write(cfg, MAX3010X_REG_SPO2_CONFIG, spo2_cfg);
}

int max3010x_set_pulse_width(const struct device *dev, max3010x_pulse_width_t width)
{
	const max3010x_config_t *cfg = dev->config;
	uint8_t spo2_cfg = 0;

	if (max3010x_reg_read(cfg, MAX3010X_REG_SPO2_CONFIG, &spo2_cfg))
	{
		return -EIO;
	}

	spo2_cfg &= (uint8_t)~MAX3010X_SPO2_PW_MASK;
	spo2_cfg |= (uint8_t)(width << MAX3010X_SPO2_PW_SHIFT);
	return max3010x_reg_write(cfg, MAX3010X_REG_SPO2_CONFIG, spo2_cfg);
}

int max3010x_set_led_pa(const struct device *dev, const max3010x_led_pa_t *cfg_in)
{
	if (!cfg_in)
	{
		return -EINVAL;
	}

	const max3010x_config_t *cfg = dev->config;
	max3010x_led_pa_t led_pa = *cfg_in;
	if (cfg->variant == MAX3010X_VARIANT_MAX30102)
	{
		led_pa.green = 0;
		led_pa.green2 = 0;
	}

	if (max3010x_reg_write(cfg, MAX3010X_REG_LED1_PA, led_pa.red))
	{
		return -EIO;
	}
	if (max3010x_reg_write(cfg, MAX3010X_REG_LED2_PA, led_pa.ir))
	{
		return -EIO;
	}
	if (max3010x_reg_write(cfg, MAX3010X_REG_LED3_PA, led_pa.green))
	{
		return -EIO;
	}
	if (max3010x_reg_write(cfg, MAX3010X_REG_LED4_PA, led_pa.green2))
	{
		return -EIO;
	}
	if (max3010x_reg_write(cfg, MAX3010X_REG_PILOT_PA, led_pa.pilot))
	{
		return -EIO;
	}

	return 0;
}

int max3010x_set_led_channel_pa(const struct device *dev,
				max3010x_led_channel_t channel,
				uint8_t value)
{
	const max3010x_config_t *cfg = dev->config;

	switch (channel)
	{
	case MAX3010X_LED_RED:
		return max3010x_reg_write(cfg, MAX3010X_REG_LED1_PA, value);
	case MAX3010X_LED_IR:
		return max3010x_reg_write(cfg, MAX3010X_REG_LED2_PA, value);
	case MAX3010X_LED_GREEN:
		if (cfg->variant == MAX3010X_VARIANT_MAX30102)
		{
			return -ENOTSUP;
		}
		return max3010x_reg_write(cfg, MAX3010X_REG_LED3_PA, value);
	default:
		return -EINVAL;
	}
}

int max3010x_set_slots(const struct device *dev, const max3010x_slot_config_t *cfg_in)
{
	if (!cfg_in)
	{
		return -EINVAL;
	}

	const max3010x_config_t *cfg = dev->config;
	max3010x_slot_t s0 = max3010x_sanitize_slot(cfg, cfg_in->slot[0]);
	max3010x_slot_t s1 = max3010x_sanitize_slot(cfg, cfg_in->slot[1]);
	max3010x_slot_t s2 = max3010x_sanitize_slot(cfg, cfg_in->slot[2]);
	max3010x_slot_t s3 = max3010x_sanitize_slot(cfg, cfg_in->slot[3]);
	uint8_t slot1_2 = (uint8_t)(s1 << 4) | (uint8_t)s0;
	uint8_t slot3_4 = (uint8_t)(s3 << 4) | (uint8_t)s2;

	if (max3010x_reg_write(cfg, MAX3010X_REG_MULTI_LED_CTRL1, slot1_2))
	{
		return -EIO;
	}
	if (max3010x_reg_write(cfg, MAX3010X_REG_MULTI_LED_CTRL2, slot3_4))
	{
		return -EIO;
	}

	max3010x_runtime_t *rt = dev->data;
	max3010x_build_channel_map(cfg, &rt->map);
	return 0;
}

int max3010x_set_interrupts(const struct device *dev, uint8_t int1_mask, uint8_t int2_mask)
{
	const max3010x_config_t *cfg = dev->config;
	if (max3010x_reg_write(cfg, MAX3010X_REG_INT_ENABLE_1, int1_mask))
	{
		return -EIO;
	}
	if (max3010x_reg_write(cfg, MAX3010X_REG_INT_ENABLE_2, int2_mask))
	{
		return -EIO;
	}
	return 0;
}

int max3010x_set_prox_int_thresh(const struct device *dev, uint8_t threshold)
{
	const max3010x_config_t *cfg = dev->config;
	return max3010x_reg_write(cfg, MAX3010X_REG_PROX_INT_THRESH, threshold);
}

int max3010x_set_temp_enable(const struct device *dev, bool enable)
{
	const max3010x_config_t *cfg = dev->config;
	uint8_t temp_cfg = enable ? MAX3010X_TEMP_EN : 0;
	return max3010x_reg_write(cfg, MAX3010X_REG_TEMP_CONFIG, temp_cfg);
}

int max3010x_get_temp(const struct device *dev, float *temp_c)
{
	if (!temp_c)
	{
		return -EINVAL;
	}

	const max3010x_config_t *cfg = dev->config;

	/* Trigger temperature conversion */
	int ret = max3010x_reg_write(cfg, MAX3010X_REG_TEMP_CONFIG, MAX3010X_TEMP_EN);
	if (ret)
	{
		return -EIO;
	}

	/* Wait for conversion (~30ms typical) */
	k_msleep(40);

	/* Read temperature registers */
	uint8_t temp_int, temp_frac;
	ret = max3010x_reg_read(cfg, MAX3010X_REG_TEMP_INT, &temp_int);
	if (ret)
	{
		return -EIO;
	}

	ret = max3010x_reg_read(cfg, MAX3010X_REG_TEMP_FRAC, &temp_frac);
	if (ret)
	{
		return -EIO;
	}

	/* Convert: TEMP_INT is signed integer part, TEMP_FRAC is 1/16 degree increments */
	*temp_c = (float)(int8_t)temp_int + ((float)(temp_frac & 0x0F) * 0.0625f);
	return 0;
}

int max3010x_get_interrupt_status(const struct device *dev, uint8_t *int1, uint8_t *int2)
{
	const max3010x_config_t *cfg = dev->config;
	if (int1 && max3010x_reg_read(cfg, MAX3010X_REG_INT_STATUS_1, int1))
	{
		return -EIO;
	}
	if (int2 && max3010x_reg_read(cfg, MAX3010X_REG_INT_STATUS_2, int2))
	{
		return -EIO;
	}
	return 0;
}

int max3010x_read_fifo(const struct device *dev, uint8_t *buf, size_t bytes)
{
	if (!buf || bytes == 0)
	{
		return -EINVAL;
	}

	const max3010x_config_t *cfg = dev->config;
	return max3010x_burst_read(cfg, MAX3010X_REG_FIFO_DATA, buf, bytes);
}

int max3010x_flush_fifo(const struct device *dev)
{
	const max3010x_config_t *cfg = dev->config;

	/* Reset FIFO pointers and overflow counter */
	if (max3010x_reg_write(cfg, MAX3010X_REG_FIFO_WR_PTR, 0))
	{
		return -EIO;
	}
	if (max3010x_reg_write(cfg, MAX3010X_REG_OVF_COUNTER, 0))
	{
		return -EIO;
	}
	if (max3010x_reg_write(cfg, MAX3010X_REG_FIFO_RD_PTR, 0))
	{
		return -EIO;
	}

	LOG_DBG("FIFO flushed");
	return 0;
}

int max3010x_get_raw_channel(const struct device *dev,
			     max3010x_led_channel_t channel,
			     uint32_t *value)
{
	if (!value)
	{
		return -EINVAL;
	}

	max3010x_runtime_t *rt = dev->data;
	uint8_t fifo_chan = rt->map.fifo_index[channel];
	if (fifo_chan >= MAX3010X_MAX_NUM_CHANNELS)
	{
		return -ENOTSUP;
	}

	*value = rt->data.raw[fifo_chan];
	return 0;
}

uint8_t max3010x_get_num_channels(const struct device *dev)
{
	const max3010x_runtime_t *rt = dev->data;
	return rt->map.active_channels;
}

int max3010x_init(const struct device *dev)
{
	max3010x_runtime_t *rt = dev->data;
	const max3010x_config_t *cfg = dev->config;
	uint8_t part_id = 0;
	int ret;

	LOG_INF("Initializing MAX3010x at 0x%02x", cfg->i2c.addr);

	/* Step 1: Verify I2C ready */
	if (!device_is_ready(cfg->i2c.bus))
	{
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Initialize runtime state */
	rt->dev = dev;
	rt->int_callback = NULL;
	rt->int_user_data = NULL;
	atomic_set(&rt->sample_count, 0);
	for (int i = 0; i < MAX3010X_MAX_NUM_CHANNELS; i++)
	{
		rt->data.raw[i] = 0;
		rt->map.fifo_index[i] = MAX3010X_MAX_NUM_CHANNELS;
	}
	rt->map.active_channels = 0;

	/* Step 2: Read and verify part ID */
	ret = max3010x_reg_read(cfg, MAX3010X_REG_PART_ID, &part_id);
	if (ret)
	{
		LOG_ERR("Failed to read part ID");
		return -EIO;
	}

	if (cfg->variant == MAX3010X_VARIANT_MAX30101 && part_id != MAX30101_PART_ID)
	{
		LOG_ERR("Unexpected part ID: 0x%02X (expected 0x%02X)", part_id, MAX30101_PART_ID);
		return -ENODEV;
	}
	if (cfg->variant == MAX3010X_VARIANT_MAX30102 && part_id != MAX30102_PART_ID)
	{
		LOG_ERR("Unexpected part ID: 0x%02X (expected 0x%02X)", part_id, MAX30102_PART_ID);
		return -ENODEV;
	}
	LOG_DBG("Part ID: 0x%02X", part_id);

	/* Step 3: Soft reset and wait */
	ret = max3010x_reset(dev);
	if (ret)
	{
		LOG_ERR("Reset failed: %d", ret);
		return ret;
	}

	/* Step 4: Apply Kconfig/DT defaults */
	ret = max3010x_set_mode(dev, cfg->mode);
	if (ret)
	{
		LOG_ERR("Failed to set mode");
		return -EIO;
	}

	ret = max3010x_set_fifo_config(dev, &cfg->fifo);
	if (ret)
	{
		LOG_ERR("Failed to set FIFO config");
		return -EIO;
	}

	ret = max3010x_set_spo2_config(dev, &cfg->spo2);
	if (ret)
	{
		LOG_ERR("Failed to set SpO2 config");
		return -EIO;
	}

	ret = max3010x_set_led_pa(dev, &cfg->led_pa);
	if (ret)
	{
		LOG_ERR("Failed to set LED currents");
		return -EIO;
	}

	ret = max3010x_set_slots(dev, &cfg->slots);
	if (ret)
	{
		LOG_ERR("Failed to set slots");
		return -EIO;
	}

	/* Step 5: Put device in SHUTDOWN mode */
	ret = max3010x_disable(dev);
	if (ret)
	{
		LOG_ERR("Failed to enter shutdown");
		return -EIO;
	}

	/* Step 6: Configure GPIO interrupt (but leave disabled) */
#if IS_ENABLED(CONFIG_MAX3010X_IRQ_ENABLE)
	if (cfg->int_gpio.port)
	{
		if (!gpio_is_ready_dt(&cfg->int_gpio))
		{
			LOG_ERR("GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
		if (ret < 0)
		{
			LOG_ERR("GPIO config failed: %d", ret);
			return ret;
		}

		ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_DISABLE);
		if (ret < 0)
		{
			LOG_ERR("GPIO int disable failed: %d", ret);
			return ret;
		}

		gpio_init_callback(&rt->gpio_cb, max3010x_gpio_handler, BIT(cfg->int_gpio.pin));
		ret = gpio_add_callback(cfg->int_gpio.port, &rt->gpio_cb);
		if (ret < 0)
		{
			LOG_ERR("GPIO callback failed: %d", ret);
			return ret;
		}
		LOG_DBG("GPIO interrupt configured (disabled)");
	}
#else
	/* IRQ support disabled via Kconfig - configure GPIO for polling only */
	if (cfg->int_gpio.port)
	{
		if (!gpio_is_ready_dt(&cfg->int_gpio))
		{
			LOG_WRN("INT GPIO not ready (polling mode)");
		}
		else
		{
			ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
			if (ret < 0)
			{
				LOG_WRN("INT GPIO config failed: %d (polling unavailable)", ret);
			}
			else
			{
				LOG_DBG("INT GPIO configured for polling (IRQ disabled via Kconfig)");
			}
		}
	}
#endif /* CONFIG_MAX3010X_IRQ_ENABLE */

	LOG_INF("MAX3010x ready (shutdown mode)");
	return 0;
}

int max3010x_apply_default_config(const struct device *dev)
{
	const max3010x_config_t *cfg = dev->config;
	int ret;

	ret = max3010x_set_mode(dev, cfg->mode);
	if (ret)
	{
		return ret;
	}

	ret = max3010x_set_fifo_config(dev, &cfg->fifo);
	if (ret)
	{
		return ret;
	}

	ret = max3010x_set_spo2_config(dev, &cfg->spo2);
	if (ret)
	{
		return ret;
	}

	ret = max3010x_set_led_pa(dev, &cfg->led_pa);
	if (ret)
	{
		return ret;
	}

	ret = max3010x_set_slots(dev, &cfg->slots);
	if (ret)
	{
		return ret;
	}

	return 0;
}

static int max3010x_zephyr_init(const struct device *dev)
{
	int ret = max3010x_init(dev);
	if (ret < 0)
	{
		LOG_ERR("Init failed: %d", ret);
	}
	return ret;
}

/* --------------------------------------------------------------------------
 * Interrupt Callback API Implementation
 * -------------------------------------------------------------------------- */

int max3010x_set_int_callback(const struct device *dev,
			      max3010x_int_callback_t callback,
			      void *user_data)
{
	if (!dev)
	{
		return -EINVAL;
	}

	max3010x_runtime_t *rt = dev->data;

	rt->int_callback = callback;
	rt->int_user_data = user_data;

	return 0;
}

int max3010x_enable_int(const struct device *dev, bool enable)
{
	if (!dev)
	{
		return -EINVAL;
	}

#if !IS_ENABLED(CONFIG_MAX3010X_IRQ_ENABLE)
	LOG_WRN("IRQ support disabled via Kconfig");
	return -ENOTSUP;
#else
	const max3010x_config_t *cfg = dev->config;

	if (!cfg->int_gpio.port)
	{
		LOG_DBG("INT GPIO not configured in devicetree");
		return -ENOTSUP;
	}

	if (enable)
	{
		/* MAX30101 INT pin is active low, open drain */
		LOG_DBG("Enabling INT GPIO interrupt");
		return gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	}
	else
	{
		LOG_DBG("Disabling INT GPIO interrupt");
		return gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_DISABLE);
	}
#endif /* CONFIG_MAX3010X_IRQ_ENABLE */
}

uint32_t max3010x_get_sample_count(const struct device *dev)
{
	if (!dev)
	{
		return 0;
	}

	max3010x_runtime_t *rt = dev->data;
	return atomic_get(&rt->sample_count);
}

void max3010x_reset_sample_count(const struct device *dev)
{
	if (!dev)
	{
		return;
	}

	max3010x_runtime_t *rt = dev->data;
	atomic_set(&rt->sample_count, 0);
}

uint32_t max3010x_get_and_reset_sample_count(const struct device *dev)
{
	if (!dev)
	{
		return 0;
	}

	max3010x_runtime_t *rt = dev->data;
	return atomic_clear(&rt->sample_count);
}

int max3010x_read_int_gpio(const struct device *dev, int *state)
{
	if (!dev || !state)
	{
		return -EINVAL;
	}

	const max3010x_config_t *cfg = dev->config;

	if (!cfg->int_gpio.port)
	{
		LOG_DBG("INT GPIO not configured");
		return -ENOTSUP;
	}

	int gpio_state = gpio_pin_get_dt(&cfg->int_gpio);
	if (gpio_state < 0)
	{
		LOG_ERR("Failed to read INT GPIO: %d", gpio_state);
		return gpio_state;
	}

	*state = gpio_state;
	return 0;
}

bool max3010x_has_int_gpio(const struct device *dev)
{
	if (!dev)
	{
		return false;
	}

	const max3010x_config_t *cfg = dev->config;
	return cfg->int_gpio.port != NULL;
}

#define MAX3010X_VARIANT_FROM_DT(node_id)                                    \
	(DT_ENUM_IDX(node_id, variant) == 1 ?                               \
		 MAX3010X_VARIANT_MAX30102 : MAX3010X_VARIANT_MAX30101)

#define MAX3010X_DEFINE_NODE(node_id)                                              \
	static max3010x_runtime_t max3010x_rt_##node_id;                           \
	static const max3010x_config_t max3010x_cfg_##node_id = {                  \
		.i2c = I2C_DT_SPEC_GET(node_id),                                   \
		.int_gpio = GPIO_DT_SPEC_GET_OR(node_id, int_gpios, {0}),          \
		.variant = MAX3010X_VARIANT_FROM_DT(node_id),                       \
		.mode = MAX3010X_DEFAULT_MODE,                                     \
		.fifo = {                                                          \
			.sample_avg = CONFIG_MAX3010X_SAMPLE_AVG,                  \
			.almost_full = CONFIG_MAX3010X_FIFO_A_FULL,                \
			.rollover = CONFIG_MAX3010X_FIFO_ROLLOVER_EN,              \
		},                                                                 \
		.spo2 = {                                                          \
			.adc_range = CONFIG_MAX3010X_ADC_RANGE,                    \
			.sample_rate = CONFIG_MAX3010X_SAMPLE_RATE,                \
			.pulse_width = CONFIG_MAX3010X_PULSE_WIDTH,                \
		},                                                                 \
		.slots = {                                                         \
			.slot = {                                                  \
				CONFIG_MAX3010X_SLOT1,                             \
				CONFIG_MAX3010X_SLOT2,                             \
				CONFIG_MAX3010X_SLOT3,                             \
				CONFIG_MAX3010X_SLOT4,                             \
			},                                                         \
		},                                                                 \
		.led_pa = {                                                        \
			.red = CONFIG_MAX3010X_LED1_PA,                           \
			.ir = CONFIG_MAX3010X_LED2_PA,                            \
			.green = CONFIG_MAX3010X_LED3_PA,                         \
			.green2 = CONFIG_MAX3010X_LED4_PA,                        \
			.pilot = CONFIG_MAX3010X_PILOT_PA,                        \
		},                                                                 \
		.interrupts = {                                                    \
			.int1_mask = CONFIG_MAX3010X_INT_ENABLE_1,                \
			.int2_mask = CONFIG_MAX3010X_INT_ENABLE_2,                \
		},                                                                 \
		.prox_int_thresh = CONFIG_MAX3010X_PROX_INT_THRESH,                \
		.temp_enable = IS_ENABLED(CONFIG_MAX3010X_TEMP_ENABLE),          \
	};                                                                     \
	DEVICE_DT_DEFINE(node_id, max3010x_zephyr_init, NULL,                    \
			 &max3010x_rt_##node_id, &max3010x_cfg_##node_id,        \
			 POST_KERNEL, MAX3010X_INIT_PRIORITY, &max3010x_api);

DT_FOREACH_STATUS_OKAY(maxim_max3010x, MAX3010X_DEFINE_NODE)
