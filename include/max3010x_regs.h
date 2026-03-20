/**
 * @file max3010x_regs.h
 * @brief MAX3010x register addresses and bit field definitions
 *
 * Register map for MAX30101/MAX30102 pulse oximeter and heart rate sensors.
 *
 * @author Orion Serup <orion@crablabs.io>
 * @author Claude <noreply@anthropic.com>
 *
 * @reviewer
 */

/* Copyright (c) 2025 Crab Labs LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#ifndef MAX3010X_REGS_H
#define MAX3010X_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Register Addresses
 * ============================================================================ */

/* Interrupt */
#define MAX3010X_REG_INT_STATUS_1        0x00
#define MAX3010X_REG_INT_STATUS_2        0x01
#define MAX3010X_REG_INT_ENABLE_1        0x02
#define MAX3010X_REG_INT_ENABLE_2        0x03

/* FIFO */
#define MAX3010X_REG_FIFO_WR_PTR         0x04
#define MAX3010X_REG_OVF_COUNTER         0x05
#define MAX3010X_REG_FIFO_RD_PTR         0x06
#define MAX3010X_REG_FIFO_DATA           0x07

/* Configuration */
#define MAX3010X_REG_FIFO_CONFIG         0x08
#define MAX3010X_REG_MODE_CONFIG         0x09
#define MAX3010X_REG_SPO2_CONFIG         0x0A

/* LED Pulse Amplitude */
#define MAX3010X_REG_LED1_PA             0x0C
#define MAX3010X_REG_LED2_PA             0x0D
#define MAX3010X_REG_LED3_PA             0x0E
#define MAX3010X_REG_LED4_PA             0x0F
#define MAX3010X_REG_PILOT_PA            0x10

/* Multi-LED Mode Control */
#define MAX3010X_REG_MULTI_LED_CTRL1     0x11
#define MAX3010X_REG_MULTI_LED_CTRL2     0x12

/* Temperature */
#define MAX3010X_REG_TEMP_INT            0x1F
#define MAX3010X_REG_TEMP_FRAC           0x20
#define MAX3010X_REG_TEMP_CONFIG         0x21

/* Proximity */
#define MAX3010X_REG_PROX_INT_THRESH     0x30

/* Part ID */
#define MAX3010X_REG_REV_ID              0xFE
#define MAX3010X_REG_PART_ID             0xFF

/* ============================================================================
 * Part IDs
 *
 * NOTE: MAX30101 and MAX30102 share the same Part ID (0x15).
 * Runtime variant detection from Part ID alone is not possible.
 * The variant is selected via device tree ("variant" property).
 * ============================================================================ */

#define MAX3010X_PART_ID                 0x15

/* ============================================================================
 * INT_STATUS_1 / INT_ENABLE_1 Bit Fields
 * ============================================================================ */

#define MAX3010X_INT_PWR_RDY             0x01   /* Power ready (STATUS ONLY, no enable bit) */
#define MAX3010X_INT_PROX_INT            0x10   /* Proximity interrupt (MAX30101 only) */
#define MAX3010X_INT_ALC_OVF             0x20   /* Ambient light cancellation overflow */
#define MAX3010X_INT_PPG_RDY             0x40   /* New FIFO sample ready */
#define MAX3010X_INT_A_FULL              0x80   /* FIFO almost full */

#define MAX3010X_INT_EN_PROX_INT         0x10
#define MAX3010X_INT_EN_ALC_OVF          0x20
#define MAX3010X_INT_EN_PPG_RDY          0x40
#define MAX3010X_INT_EN_A_FULL           0x80

/* ============================================================================
 * INT_STATUS_2 / INT_ENABLE_2 Bit Fields
 * ============================================================================ */

#define MAX3010X_INT_DIE_TEMP_RDY        0x02
#define MAX3010X_INT_EN_DIE_TEMP_RDY     0x02

/* ============================================================================
 * FIFO_CONFIG Bit Fields
 * ============================================================================ */

#define MAX3010X_FIFO_SAMPLE_AVERAGE_SHIFT  5
#define MAX3010X_FIFO_SAMPLE_AVERAGE_MASK   0xE0
#define MAX3010X_FIFO_ROLLOVER_EN        0x10
#define MAX3010X_FIFO_A_FULL_MASK        0x0F

/* ============================================================================
 * MODE_CONFIG Bit Fields
 * ============================================================================ */

#define MAX3010X_MODE_SHDN               0x80   /* Shutdown control */
#define MAX3010X_MODE_RESET              0x40   /* Reset control */
#define MAX3010X_MODE_MASK               0x07   /* Mode bits [2:0] */

/* ============================================================================
 * SPO2_CONFIG Bit Fields
 * ============================================================================ */

#define MAX3010X_SPO2_ADC_RANGE_SHIFT    5
#define MAX3010X_SPO2_ADC_RANGE_MASK     0x60
#define MAX3010X_SPO2_SAMPLE_RATE_SHIFT  2
#define MAX3010X_SPO2_SAMPLE_RATE_MASK   0x1C
#define MAX3010X_SPO2_PULSE_WIDTH_SHIFT  0
#define MAX3010X_SPO2_PULSE_WIDTH_MASK   0x03

/* ============================================================================
 * Multi-LED Control Bit Fields
 * ============================================================================ */

#define MAX3010X_MULTI_LED_SLOT_MASK     0x07
#define MAX3010X_MULTI_LED_SLOT1_SHIFT   0
#define MAX3010X_MULTI_LED_SLOT2_SHIFT   4
#define MAX3010X_MULTI_LED_SLOT3_SHIFT   0
#define MAX3010X_MULTI_LED_SLOT4_SHIFT   4

/* ============================================================================
 * Temperature Bit Fields
 * ============================================================================ */

#define MAX3010X_TEMP_EN                 0x01
#define MAX3010X_TEMP_FRAC_MASK          0x0F

/* ============================================================================
 * FIFO Pointer / Overflow Masks
 * ============================================================================ */

#define MAX3010X_FIFO_PTR_MASK           0x1F   /* 5-bit FIFO pointer (0-31) */
#define MAX3010X_OVF_COUNTER_MASK        0x1F   /* 5-bit overflow counter (saturates at 31) */

/* ============================================================================
 * FIFO Data Constants
 * ============================================================================ */

#define MAX3010X_FIFO_DATA_BITS          18
#define MAX3010X_FIFO_DATA_MASK          0x3FFFF
#define MAX3010X_BYTES_PER_CHANNEL       3
#define MAX3010X_MAX_NUM_CHANNELS        3      /* Distinct LED types: Red, IR, Green */
#define MAX3010X_MAX_NUM_SLOTS           4      /* Multi-LED mode has 4 time slots */
#define MAX3010X_FIFO_DEPTH              32

/* ============================================================================
 * Timing Constants
 * ============================================================================ */

#define MAX3010X_RESET_TIMEOUT_MS        100
#define MAX3010X_TEMP_CONV_WAIT_MS       40

#ifdef __cplusplus
}
#endif

#endif /* MAX3010X_REGS_H */
