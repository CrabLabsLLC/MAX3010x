/**
 * @file max3010x.h
 * @brief MAX3010x PPG/SpO2 sensor - public API
 *
 * Zephyr device driver for Maxim MAX30101/MAX30102 pulse oximeter
 * and heart rate sensors. Uses standard Zephyr device model.
 *
 * @author Orion Serup <orion@crablabs.io>
 * @author Claude <noreply@anthropic.com>
 *
 * @reviewer
 */

/* Copyright (c) 2025 Crab Labs LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#ifndef MAX3010X_H
#define MAX3010X_H

#include <zephyr/device.h>

#include "max3010x_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Lifecycle
 * ============================================================================ */

/**
 * @brief Reset device and wait for completion
 *
 * Performs a soft reset by writing the RESET bit and polling until cleared.
 *
 * @param[in] dev Zephyr device handle
 * @return 0 on success, -ETIMEDOUT if reset did not complete, negative errno
 */
int max3010xReset(const struct device* const dev);

/**
 * @brief Enable measurement (wake from shutdown)
 *
 * Clears the SHDN bit in MODE_CONFIG register.
 *
 * @param[in] dev Zephyr device handle
 * @return 0 on success, negative errno on failure
 */
int max3010xEnable(const struct device* const dev);

/**
 * @brief Disable measurement (enter shutdown)
 *
 * Sets the SHDN bit in MODE_CONFIG register.
 *
 * @param[in] dev Zephyr device handle
 * @return 0 on success, negative errno on failure
 */
int max3010xDisable(const struct device* const dev);

/* ============================================================================
 * Configuration
 * ============================================================================ */

/**
 * @brief Set operating mode
 *
 * @param[in] dev  Zephyr device handle
 * @param[in] mode Operating mode (heart rate, SpO2, multi-LED)
 * @return 0 on success, negative errno on failure
 */
int max3010xSetMode(const struct device* const dev, const MAX3010xMode mode);

/**
 * @brief Apply FIFO configuration
 *
 * @param[in] dev    Zephyr device handle
 * @param[in] config FIFO config. Must not be NULL.
 * @return 0 on success, negative errno on failure
 */
int max3010xSetFIFOConfig(const struct device* const dev, const MAX3010xFIFOConfig* const config);

/**
 * @brief Apply SpO2/PPG ADC configuration
 *
 * @param[in] dev    Zephyr device handle
 * @param[in] config SpO2 config. Must not be NULL.
 * @return 0 on success, negative errno on failure
 */
int max3010xSetSPO2Config(const struct device* const dev, const MAX3010xSPO2Config* const config);

/**
 * @brief Set ADC range only
 *
 * @param[in] dev   Zephyr device handle
 * @param[in] range ADC range selection
 * @return 0 on success, negative errno on failure
 */
int max3010xSetADCRange(const struct device* const dev, const MAX3010xADCRange range);

/**
 * @brief Set sample rate only
 *
 * @param[in] dev  Zephyr device handle
 * @param[in] rate Sample rate selection
 * @return 0 on success, negative errno on failure
 */
int max3010xSetSampleRate(const struct device* const dev, const MAX3010xSampleRate rate);

/**
 * @brief Set LED pulse width only
 *
 * @param[in] dev   Zephyr device handle
 * @param[in] width Pulse width selection
 * @return 0 on success, negative errno on failure
 */
int max3010xSetPulseWidth(const struct device* const dev, const MAX3010xPulseWidth width);

/**
 * @brief Set all LED pulse amplitudes
 *
 * Green channels are forced to 0 on MAX30102.
 *
 * @param[in] dev    Zephyr device handle
 * @param[in] config LED amplitude config. Must not be NULL.
 * @return 0 on success, negative errno on failure
 */
int max3010xSetLEDAmplitude(const struct device* const dev, const MAX3010xLEDAmplitude* const config);

/**
 * @brief Set a single LED channel amplitude
 *
 * @param[in] dev     Zephyr device handle
 * @param[in] channel LED channel (Red, IR, Green)
 * @param[in] value   Amplitude (0-255, 0.2 mA/LSB)
 * @return 0 on success, -ENOTSUP for Green on MAX30102, negative errno
 */
int max3010xSetLEDChannelAmplitude(const struct device* const dev, const MAX3010xLEDChannel channel, const uint8_t value);

/**
 * @brief Configure multi-LED slots
 *
 * Also rebuilds the internal channel map.
 *
 * @param[in] dev    Zephyr device handle
 * @param[in] config Slot config. Must not be NULL.
 * @return 0 on success, negative errno on failure
 */
int max3010xSetSlots(const struct device* const dev, const MAX3010xSlotConfig* const config);

/**
 * @brief Re-apply DT/Kconfig defaults
 *
 * Restores mode, FIFO, SpO2, LED currents, and slots from init config.
 * Useful after reset without full re-init.
 *
 * @param[in] dev Zephyr device handle
 * @return 0 on success, negative errno on failure
 */
int max3010xApplyDefaultConfig(const struct device* const dev);

/**
 * @brief Set interrupt enable masks
 *
 * @param[in] dev                      Zephyr device handle
 * @param[in] interrupt_enable_1_mask  INT_ENABLE_1 register value
 * @param[in] interrupt_enable_2_mask  INT_ENABLE_2 register value
 * @return 0 on success, negative errno on failure
 */
int max3010xSetInterrupts(const struct device* const dev, const uint8_t interrupt_enable_1_mask, const uint8_t interrupt_enable_2_mask);

/**
 * @brief Set proximity interrupt threshold
 *
 * @param[in] dev       Zephyr device handle
 * @param[in] threshold Threshold value (0 = disabled)
 * @return 0 on success, negative errno on failure
 */
int max3010xSetProximityThreshold(const struct device* const dev, const uint8_t threshold);

/**
 * @brief Enable or disable die temperature conversion
 *
 * @param[in] dev        Zephyr device handle
 * @param[in] is_enabled True to enable, false to disable
 * @return 0 on success, negative errno on failure
 */
int max3010xSetTemperatureEnabled(const struct device* const dev, const bool is_enabled);

/* ============================================================================
 * Data Acquisition
 * ============================================================================ */

/**
 * @brief Read die temperature in Celsius
 *
 * Triggers a one-shot conversion, polls DIE_TEMP_RDY until complete
 * (typically ~30 ms), then reads the result. Resolution: 0.0625 deg C.
 *
 * @param[in]  dev              Zephyr device handle
 * @param[out] temperature_deg_c Output: temperature in degrees Celsius
 * @return 0 on success, -ETIMEDOUT if conversion didn't complete, negative errno
 */
int max3010xGetTemperature(const struct device* const dev, float* const temperature_deg_c);

/**
 * @brief Read and clear interrupt status registers
 *
 * Either output pointer may be NULL to skip that register.
 *
 * @param[in]  dev  Zephyr device handle
 * @param[out] int1 Output: INT_STATUS_1 (or NULL)
 * @param[out] int2 Output: INT_STATUS_2 (or NULL)
 * @return 0 on success, negative errno on failure
 */
int max3010xGetInterruptStatus(const struct device* const dev, uint8_t* const int1, uint8_t* const int2);

/**
 * @brief Read FIFO data bytes
 *
 * @param[in]  dev        Zephyr device handle
 * @param[out] buffer     Output buffer
 * @param[in]  size_bytes Number of bytes to read
 * @return 0 on success, negative errno on failure
 */
int max3010xReadFIFO(const struct device* const dev, uint8_t* const buffer, const size_t size_bytes);

/**
 * @brief Flush FIFO by resetting pointers and overflow counter
 *
 * Call before changing sample rate to avoid mixed-rate data.
 *
 * @param[in] dev Zephyr device handle
 * @return 0 on success, negative errno on failure
 */
int max3010xFlushFIFO(const struct device* const dev);

/**
 * @brief Read raw LED channel sample from cached FIFO data
 *
 * @param[in]  dev     Zephyr device handle
 * @param[in]  channel LED channel to read
 * @param[out] value   Output: 18-bit raw sample
 * @return 0 on success, -ENOTSUP if channel inactive, negative errno
 */
int max3010xGetRawChannel(const struct device* const dev, const MAX3010xLEDChannel channel, uint32_t* const value);

/**
 * @brief Get number of active FIFO channels
 *
 * @param[in] dev Zephyr device handle
 * @return Number of active FIFO slots (0-4)
 */
uint8_t max3010xGetNumChannels(const struct device* const dev);

/**
 * @brief Get number of available samples in FIFO
 *
 * Reads FIFO write and read pointers and computes the difference.
 *
 * @param[in]  dev   Zephyr device handle
 * @param[out] count Output: number of unread samples
 * @return 0 on success, negative errno on failure
 */
int max3010xGetAvailableSamples(const struct device* const dev, uint8_t* const count);

/**
 * @brief Get FIFO overflow counter
 *
 * Returns the number of samples lost due to FIFO overflow (saturates at 31).
 *
 * @param[in]  dev   Zephyr device handle
 * @param[out] count Output: overflow count (0-31)
 * @return 0 on success, negative errno on failure
 */
int max3010xGetOverflowCount(const struct device* const dev, uint8_t* const count);

/**
 * @brief Read Part ID register
 *
 * Both MAX30101 and MAX30102 return 0x15.
 *
 * @param[in]  dev     Zephyr device handle
 * @param[out] part_id Output: part ID byte
 * @return 0 on success, negative errno on failure
 */
int max3010xGetPartID(const struct device* const dev, uint8_t* const part_id);

/**
 * @brief Read Revision ID register
 *
 * @param[in]  dev    Zephyr device handle
 * @param[out] rev_id Output: revision ID byte
 * @return 0 on success, negative errno on failure
 */
int max3010xGetRevisionID(const struct device* const dev, uint8_t* const rev_id);

/* ============================================================================
 * Interrupt Handling
 * ============================================================================ */

/**
 * @brief Register interrupt callback
 *
 * Callback fires from ISR context on INT pin falling edge.
 * Pass NULL to disable. Disables the GPIO interrupt during swap;
 * caller must call max3010xEnableInterrupt() afterwards.
 *
 * @note The application callback is responsible for reading the interrupt
 *       status registers (via max3010xGetInterruptStatus) to clear the
 *       hardware interrupt source. Failure to do so may cause the interrupt
 *       to stop firing on edge-triggered GPIOs.
 *
 * @param[in] dev       Zephyr device handle
 * @param[in] callback  Function to call, or NULL to disable
 * @param[in] user_data Context pointer passed to callback
 * @return 0 on success, negative errno on failure
 */
int max3010xSetInterruptCallback(const struct device* const dev, const MAX3010xCallback callback, void* const user_data);

/**
 * @brief Enable or disable GPIO interrupt on INT pin
 *
 * @param[in] dev        Zephyr device handle
 * @param[in] is_enabled True to enable, false to disable
 * @return 0 on success, -ENOTSUP if no INT GPIO, negative errno
 */
int max3010xEnableInterrupt(const struct device* const dev, const bool is_enabled);

/**
 * @brief Get interrupt event count since last reset
 *
 * @param[in] dev Zephyr device handle
 * @return Number of interrupt events
 */
uint32_t max3010xGetSampleCount(const struct device* const dev);

/**
 * @brief Reset sample counter to zero
 *
 * @param[in] dev Zephyr device handle
 */
void max3010xResetSampleCount(const struct device* const dev);

/**
 * @brief Get and atomically reset sample counter
 *
 * @param[in] dev Zephyr device handle
 * @return Count before reset
 */
uint32_t max3010xGetAndResetSampleCount(const struct device* const dev);

/* ============================================================================
 * Diagnostics and GPIO Polling
 * ============================================================================ */

/**
 * @brief Dump all critical registers for debugging
 *
 * Reads and logs MODE, FIFO, SpO2, LED, slot, interrupt, and pointer registers.
 *
 * @param[in] dev Zephyr device handle
 * @return 0 on success, negative errno on failure
 */
int max3010xDumpRegisters(const struct device* const dev);

/**
 * @brief Read INT GPIO pin state
 *
 * @param[in]  dev   Zephyr device handle
 * @param[out] state Output: 0 = interrupt active (active low), 1 = no interrupt
 * @return 0 on success, -ENOTSUP if no INT GPIO, negative errno
 */
int max3010xReadInterruptPin(const struct device* const dev, int* const state);

/**
 * @brief Check if interrupt GPIO is configured
 *
 * @param[in] dev Zephyr device handle
 * @return true if INT GPIO is available
 */
bool max3010xHasInterruptGPIO(const struct device* const dev);

#ifdef __cplusplus
}
#endif

#endif /* MAX3010X_H */
