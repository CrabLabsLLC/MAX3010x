/**
 * @file max3010x_types.h
 * @brief MAX3010x type definitions - enums, structs, callbacks
 *
 * All public types for the MAX3010x PPG/SpO2 sensor driver.
 * Separated from the function API header for clean dependency management.
 *
 * @author Orion Serup <orion@crablabs.io>
 *
 * @reviewer Daravuthy Ly <daravuthy@crablabs.io>
 */

/* Copyright (c) 2025 Crab Labs LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#ifndef MAX3010X_TYPES_H
#define MAX3010X_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#include "max3010x_regs.h"

#ifdef __cplusplus
extern "C"
{
#endif

	/** @brief Device variant selection */
	typedef enum
	{
		MAX3010X_VARIANT_MAX30101 = 0, ///< MAX30101: 3 LEDs (Red, IR, Green)
		MAX3010X_VARIANT_MAX30102 = 1, ///< MAX30102: 2 LEDs (Red, IR)
	} MAX3010xVariant;

	/** @brief Operating mode */
	typedef enum
	{
		MAX3010X_MODE_HEART_RATE = 2, ///< Heart rate (Red LED only)
		MAX3010X_MODE_SPO2 = 3,		  ///< SpO2 (Red + IR)
		MAX3010X_MODE_MULTI_LED = 7,  ///< Multi-LED (slot-controlled)
	} MAX3010xMode;

	/** @brief LED slot selection for multi-LED mode */
	typedef enum
	{
		MAX3010X_SLOT_DISABLED = 0,		  ///< Slot disabled
		MAX3010X_SLOT_RED_LED1_PA = 1,	  ///< Red LED (LED1_PA)
		MAX3010X_SLOT_IR_LED2_PA = 2,	  ///< IR LED (LED2_PA)
		MAX3010X_SLOT_GREEN_LED3_PA = 3,  ///< Green LED (LED3_PA)
		MAX3010X_SLOT_RED_PILOT_PA = 5,	  ///< Red pilot (PILOT_PA)
		MAX3010X_SLOT_IR_PILOT_PA = 6,	  ///< IR pilot (PILOT_PA)
		MAX3010X_SLOT_GREEN_PILOT_PA = 7, ///< Green pilot (PILOT_PA)
	} MAX3010xSlot;

	/** @brief LED channel identifiers */
	typedef enum
	{
		MAX3010X_LED_RED = 0,	///< Red LED
		MAX3010X_LED_IR = 1,	///< IR LED
		MAX3010X_LED_GREEN = 2, ///< Green LED (MAX30101 only)
	} MAX3010xLEDChannel;

	/** @brief ADC full-scale range selection */
	typedef enum
	{
		MAX3010X_ADC_RANGE_7P81 = 0,  ///< 7.81 pA/LSB (2048 nA FS)
		MAX3010X_ADC_RANGE_15P63 = 1, ///< 15.63 pA/LSB (4096 nA FS)
		MAX3010X_ADC_RANGE_31P25 = 2, ///< 31.25 pA/LSB (8192 nA FS)
		MAX3010X_ADC_RANGE_62P50 = 3, ///< 62.5 pA/LSB (16384 nA FS)
	} MAX3010xADCRange;

	/** @brief Sample rate selection (SPO2_SR[2:0]) */
	typedef enum
	{
		MAX3010X_SR_50HZ = 0,	///< 50 samples/s
		MAX3010X_SR_100HZ = 1,	///< 100 samples/s
		MAX3010X_SR_200HZ = 2,	///< 200 samples/s
		MAX3010X_SR_400HZ = 3,	///< 400 samples/s
		MAX3010X_SR_800HZ = 4,	///< 800 samples/s
		MAX3010X_SR_1000HZ = 5, ///< 1000 samples/s
		MAX3010X_SR_1600HZ = 6, ///< 1600 samples/s
		MAX3010X_SR_3200HZ = 7, ///< 3200 samples/s
	} MAX3010xSampleRate;

	/** @brief LED pulse width selection */
	typedef enum
	{
		MAX3010X_PW_69US = 0,  ///< 69 us (15-bit ADC)
		MAX3010X_PW_118US = 1, ///< 118 us (16-bit ADC)
		MAX3010X_PW_215US = 2, ///< 215 us (17-bit ADC)
		MAX3010X_PW_411US = 3, ///< 411 us (18-bit ADC)
	} MAX3010xPulseWidth;

	/** @brief FIFO sample averaging selection */
	typedef enum
	{
		MAX3010X_FIFO_AVG_1 = 0,   ///< 1 sample (no averaging)
		MAX3010X_FIFO_AVG_2 = 1,   ///< 2 samples averaged
		MAX3010X_FIFO_AVG_4 = 2,   ///< 4 samples averaged
		MAX3010X_FIFO_AVG_8 = 3,   ///< 8 samples averaged
		MAX3010X_FIFO_AVG_16 = 4,  ///< 16 samples averaged
		MAX3010X_FIFO_AVG_32 = 5,  ///< 32 samples averaged
		MAX3010X_FIFO_AVG_32B = 6, ///< 32 samples averaged (alt)
		MAX3010X_FIFO_AVG_32C = 7, ///< 32 samples averaged (alt)
	} MAX3010xFIFOAverage;

	/** @brief Multi-LED slot configuration */
	typedef struct
	{
		MAX3010xSlot slot[MAX3010X_MAX_NUM_SLOTS]; ///< Slot 1-4 LED selections
	} MAX3010xSlotConfig;

	/** @brief FIFO configuration */
	typedef struct
	{
		MAX3010xFIFOAverage sample_average_mode; ///< Sample averaging code
		uint8_t almost_full_threshold;			 ///< Almost-full threshold (0-15)
		bool is_rollover_enabled;				 ///< Enable FIFO rollover on full
	} MAX3010xFIFOConfig;

	/** @brief SpO2/PPG ADC configuration */
	typedef struct
	{
		MAX3010xADCRange adc_range;		///< ADC range selection
		MAX3010xSampleRate sample_rate; ///< Sampling rate
		MAX3010xPulseWidth pulse_width; ///< LED pulse width
	} MAX3010xSPO2Config;

	/** @brief LED pulse amplitude configuration */
	typedef struct
	{
		uint8_t red;	///< LED1 (Red) amplitude (0-255, 0.2 mA/LSB)
		uint8_t ir;		///< LED2 (IR) amplitude
		uint8_t green;	///< LED3 (Green) amplitude (MAX30101 only)
		uint8_t green2; ///< LED4 (Green2) amplitude (MAX30101 only)
		uint8_t pilot;	///< Pilot LED amplitude
	} MAX3010xLEDAmplitude;

	/** @brief Raw FIFO sample data (per active slot) */
	typedef struct
	{
		uint32_t raw[MAX3010X_MAX_NUM_SLOTS]; ///< 18-bit raw samples, one per active slot
	} MAX3010xData;

	/** @brief Channel mapping (LED channel to FIFO index) */
	typedef struct
	{
		uint8_t fifo_index[MAX3010X_MAX_NUM_CHANNELS]; ///< LED -> FIFO index
		uint8_t active_channel_count;				   ///< Active FIFO channels
	} MAX3010xChannelMap;

	/**
	 * @brief Interrupt callback (called from ISR context)
	 *
	 * @param[in] user_data Context pointer registered with the callback
	 */
	typedef void (*MAX3010xCallback)(void* user_data);

#ifdef __cplusplus
}
#endif

#endif // MAX3010X_TYPES_H
