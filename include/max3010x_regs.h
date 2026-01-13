/*
 * Copyright (c) 2025 Makani Science
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MAX3010X_REGS_H
#define MAX3010X_REGS_H

#include <stdint.h>

// Register map (MAX30101/MAX30102)
#define MAX3010X_REG_INT_STATUS_1        0x00
#define MAX3010X_REG_INT_STATUS_2        0x01
#define MAX3010X_REG_INT_ENABLE_1        0x02
#define MAX3010X_REG_INT_ENABLE_2        0x03
#define MAX3010X_REG_FIFO_WR_PTR         0x04
#define MAX3010X_REG_OVF_COUNTER         0x05
#define MAX3010X_REG_FIFO_RD_PTR         0x06
#define MAX3010X_REG_FIFO_DATA           0x07
#define MAX3010X_REG_FIFO_CONFIG         0x08
#define MAX3010X_REG_MODE_CONFIG         0x09
#define MAX3010X_REG_SPO2_CONFIG         0x0A
#define MAX3010X_REG_RESERVED_0B         0x0B
#define MAX3010X_REG_LED1_PA             0x0C
#define MAX3010X_REG_LED2_PA             0x0D
#define MAX3010X_REG_LED3_PA             0x0E
#define MAX3010X_REG_LED4_PA             0x0F
#define MAX3010X_REG_PILOT_PA            0x10
#define MAX3010X_REG_MULTI_LED_CTRL1     0x11
#define MAX3010X_REG_MULTI_LED_CTRL2     0x12
#define MAX3010X_REG_RESERVED_13         0x13
#define MAX3010X_REG_RESERVED_17         0x17
#define MAX3010X_REG_RESERVED_18         0x18
#define MAX3010X_REG_RESERVED_1E         0x1E
#define MAX3010X_REG_TEMP_INT            0x1F
#define MAX3010X_REG_TEMP_FRAC           0x20
#define MAX3010X_REG_TEMP_CONFIG         0x21
#define MAX3010X_REG_RESERVED_22         0x22
#define MAX3010X_REG_RESERVED_2F         0x2F
#define MAX3010X_REG_PROX_INT_THRESH     0x30
#define MAX3010X_REG_REV_ID              0xFE
#define MAX3010X_REG_PART_ID             0xFF

// Part IDs
#define MAX30101_PART_ID                 0x15
#define MAX30102_PART_ID                 0x15

// INT_STATUS_1 bits
#define MAX3010X_INT_PWR_RDY             0x01 // Power ready
#define MAX3010X_INT_PROX_INT            0x10 // Proximity interrupt
#define MAX3010X_INT_ALC_OVF             0x20 // Ambient light cancellation overflow
#define MAX3010X_INT_PPG_RDY             0x40 // New FIFO sample ready
#define MAX3010X_INT_A_FULL              0x80 // FIFO almost full
#define MAX3010X_INT_STATUS1_MASK        0xF1
#define MAX3010X_INT_STATUS1_RESERVED    0x0E

// INT_ENABLE_1 bits
#define MAX3010X_INT_EN_PROX_INT         0x10 // Proximity interrupt enable
#define MAX3010X_INT_EN_ALC_OVF          0x20 // Ambient light cancellation overflow enable
#define MAX3010X_INT_EN_PPG_RDY          0x40 // New FIFO sample enable
#define MAX3010X_INT_EN_A_FULL           0x80 // FIFO almost full enable
#define MAX3010X_INT_ENABLE1_MASK        0xF0
#define MAX3010X_INT_ENABLE1_RESERVED    0x0F

// INT_STATUS_2 bits
#define MAX3010X_INT_DIE_TEMP_RDY        0x02 // Die temperature ready
#define MAX3010X_INT_STATUS2_MASK        0x02
#define MAX3010X_INT_STATUS2_RESERVED    0xFD

// INT_ENABLE_2 bits
#define MAX3010X_INT_EN_DIE_TEMP_RDY     0x02 // Die temperature ready enable
#define MAX3010X_INT_ENABLE2_MASK        0x02
#define MAX3010X_INT_ENABLE2_RESERVED    0xFD

// FIFO pointer registers (FIFO_WR_PTR, FIFO_RD_PTR, OVF_COUNTER)
#define MAX3010X_FIFO_PTR_MASK           0x1F // Bits [4:0]
#define MAX3010X_FIFO_PTR_SHIFT          0

// FIFO_CONFIG
#define MAX3010X_FIFO_SMP_AVE_SHIFT      5
#define MAX3010X_FIFO_SMP_AVE_MASK       0xE0 // Bits [7:5]
#define MAX3010X_FIFO_ROLLOVER_EN        0x10 // Bit 4
#define MAX3010X_FIFO_A_FULL_SHIFT       0
#define MAX3010X_FIFO_A_FULL_MASK        0x0F // Bits [3:0]
#define MAX3010X_FIFO_CONFIG_MASK        0xFF

// MODE_CONFIG
#define MAX3010X_MODE_SHDN               0x80 // Bit 7
#define MAX3010X_MODE_RESET              0x40 // Bit 6
#define MAX3010X_MODE_RESERVED_MASK      0x38 // Bits [5:3]
#define MAX3010X_MODE_SHIFT              0
#define MAX3010X_MODE_MASK               0x07 // Bits [2:0]

// SPO2_CONFIG
#define MAX3010X_SPO2_ADC_RGE_SHIFT      5
#define MAX3010X_SPO2_ADC_RGE_MASK       0x60 // Bits [6:5]
#define MAX3010X_SPO2_SR_SHIFT           2
#define MAX3010X_SPO2_SR_MASK            0x1C // Bits [4:2]
#define MAX3010X_SPO2_PW_SHIFT           0
#define MAX3010X_SPO2_PW_MASK            0x03 // Bits [1:0]
#define MAX3010X_SPO2_CONFIG_MASK        0x7F
#define MAX3010X_SPO2_RESERVED_MASK      0x80

// LED pulse amplitude registers (LED1_PA/LED2_PA/LED3_PA/LED4_PA/PILOT_PA)
#define MAX3010X_LED_PA_SHIFT            0
#define MAX3010X_LED_PA_MASK             0xFF
#define MAX3010X_LED_PA_MIN              0x00
#define MAX3010X_LED_PA_MAX              0xFF

// MULTI_LED_CTRL1/2 slots
#define MAX3010X_MULTI_LED_SLOT_MASK     0x07 // Slot field width
#define MAX3010X_MULTI_LED_SLOT1_SHIFT   0
#define MAX3010X_MULTI_LED_SLOT2_SHIFT   4
#define MAX3010X_MULTI_LED_SLOT3_SHIFT   0
#define MAX3010X_MULTI_LED_SLOT4_SHIFT   4
#define MAX3010X_MULTI_LED_CTRL_MASK     0x77
#define MAX3010X_MULTI_LED_RESERVED_MASK 0x88

// MULTI_LED slot values
#define MAX3010X_SLOT_VAL_DISABLED       0x00
#define MAX3010X_SLOT_VAL_LED1_RED       0x01
#define MAX3010X_SLOT_VAL_LED2_IR        0x02
#define MAX3010X_SLOT_VAL_LED3_GREEN     0x03
#define MAX3010X_SLOT_VAL_LED4_GREEN     0x03
#define MAX3010X_SLOT_VAL_DISABLED_ALT   0x04

// TEMP_CONFIG
#define MAX3010X_TEMP_EN                 0x01 // Bit 0
#define MAX3010X_TEMP_RESERVED_MASK      0xFE

// TEMP_INT / TEMP_FRAC
#define MAX3010X_TEMP_INT_SHIFT          0
#define MAX3010X_TEMP_INT_MASK           0xFF
#define MAX3010X_TEMP_FRAC_SHIFT         0
#define MAX3010X_TEMP_FRAC_MASK          0x0F
#define MAX3010X_TEMP_FRAC_RESERVED_MASK 0xF0

// PROX_INT_THRESH
#define MAX3010X_PROX_INT_THRESH_SHIFT   0
#define MAX3010X_PROX_INT_THRESH_MASK    0xFF

// FIFO data
#define MAX3010X_FIFO_DATA_BITS          18
#define MAX3010X_FIFO_DATA_MASK          0x3FFFF
#define MAX3010X_BYTES_PER_CHANNEL       3
#define MAX3010X_MAX_NUM_CHANNELS        3

#endif /* MAX3010X_REGS_H */
