/****************************************************************************
 * arch/risc-v/src/esp32c3/hardware/esp32c3_sha.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_ESP32C3_HARDWARE_ESP32C3_SHA_H
#define __ARCH_RISCV_SRC_ESP32C3_HARDWARE_ESP32C3_SHA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32c3_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SHA_MODE_REG register
 * Defines the algorithm of SHA accelerator
 */

#define SHA_MODE_REG (DR_REG_SHA_BASE + 0x0)

/* SHA_MODE : R/W; bitpos: [2:0]; default: 0;
 * Defines the SHA algorithm.
 */

#define SHA_MODE    0x00000007
#define SHA_MODE_M  (SHA_MODE_V << SHA_MODE_S)
#define SHA_MODE_V  0x00000007
#define SHA_MODE_S  0

/* SHA_T_STRING_REG register
 * String content register for calculating initial Hash Value (only
 * effective for SHA-512/t)
 */

#define SHA_T_STRING_REG (DR_REG_SHA_BASE + 0x4)

/* SHA_T_STRING : R/W; bitpos: [31:0]; default: 0;
 * Defines t_string for calculating the initial Hash value for SHA-512/t.
 */

#define SHA_T_STRING    0xFFFFFFFF
#define SHA_T_STRING_M  (SHA_T_STRING_V << SHA_T_STRING_S)
#define SHA_T_STRING_V  0xFFFFFFFF
#define SHA_T_STRING_S  0

/* SHA_T_LENGTH_REG register
 * String length register for calculating initial Hash Value (only effective
 * for SHA-512/t)
 */

#define SHA_T_LENGTH_REG (DR_REG_SHA_BASE + 0x8)

/* SHA_T_LENGTH : R/W; bitpos: [5:0]; default: 0;
 * Defines t_string for calculating the initial Hash value for SHA-512/t.
 */

#define SHA_T_LENGTH    0x0000003F
#define SHA_T_LENGTH_M  (SHA_T_LENGTH_V << SHA_T_LENGTH_S)
#define SHA_T_LENGTH_V  0x0000003F
#define SHA_T_LENGTH_S  0

/* SHA_DMA_BLOCK_NUM_REG register
 * Block number register (only effective for DMA-SHA)
 */

#define SHA_DMA_BLOCK_NUM_REG (DR_REG_SHA_BASE + 0xc)

/* SHA_DMA_BLOCK_NUM : R/W; bitpos: [5:0]; default: 0;
 * Defines the DMA-SHA block number.
 */

#define SHA_DMA_BLOCK_NUM    0x0000003F
#define SHA_DMA_BLOCK_NUM_M  (SHA_DMA_BLOCK_NUM_V << SHA_DMA_BLOCK_NUM_S)
#define SHA_DMA_BLOCK_NUM_V  0x0000003F
#define SHA_DMA_BLOCK_NUM_S  0

/* SHA_START_REG register
 * Starts the SHA accelerator for Typical SHA operation
 */

#define SHA_START_REG (DR_REG_SHA_BASE + 0x10)

/* SHA_START : WO; bitpos: [0]; default: 0;
 * Write 1 to start Typical SHA calculation.
 */

#define SHA_START    (BIT(0))
#define SHA_START_M  (SHA_START_V << SHA_START_S)
#define SHA_START_V  0x00000001
#define SHA_START_S  0

/* SHA_CONTINUE_REG register
 * Continues SHA operation (only effective in Typical SHA mode)
 */

#define SHA_CONTINUE_REG (DR_REG_SHA_BASE + 0x14)

/* SHA_CONTINUE : WO; bitpos: [0]; default: 0;
 * Write 1 to continue Typical SHA calculation.
 */

#define SHA_CONTINUE    (BIT(0))
#define SHA_CONTINUE_M  (SHA_CONTINUE_V << SHA_CONTINUE_S)
#define SHA_CONTINUE_V  0x00000001
#define SHA_CONTINUE_S  0

/* SHA_BUSY_REG register
 * Indicates if SHA Accelerator is busy or not
 */

#define SHA_BUSY_REG (DR_REG_SHA_BASE + 0x18)

/* SHA_BUSY_STATE : RO; bitpos: [0]; default: 0;
 * Indicates the states of SHA accelerator.
 * #1'h0: idle
 * #1'h1: busy
 */

#define SHA_BUSY_STATE    (BIT(0))
#define SHA_BUSY_STATE_M  (SHA_BUSY_STATE_V << SHA_BUSY_STATE_S)
#define SHA_BUSY_STATE_V  0x00000001
#define SHA_BUSY_STATE_S  0

/* SHA_DMA_START_REG register
 * Starts the SHA accelerator for DMA-SHA operation
 */

#define SHA_DMA_START_REG (DR_REG_SHA_BASE + 0x1c)

/* SHA_DMA_START : WO; bitpos: [0]; default: 0;
 * Write 1 to start DMA-SHA calculation.
 */

#define SHA_DMA_START    (BIT(0))
#define SHA_DMA_START_M  (SHA_DMA_START_V << SHA_DMA_START_S)
#define SHA_DMA_START_V  0x00000001
#define SHA_DMA_START_S  0

/* SHA_DMA_CONTINUE_REG register
 * Continues SHA operation (only effective in DMA-SHA mode)
 */

#define SHA_DMA_CONTINUE_REG (DR_REG_SHA_BASE + 0x20)

/* SHA_DMA_CONTINUE : WO; bitpos: [0]; default: 0;
 * Write 1 to continue DMA-SHA calculation.
 */

#define SHA_DMA_CONTINUE    (BIT(0))
#define SHA_DMA_CONTINUE_M  (SHA_DMA_CONTINUE_V << SHA_DMA_CONTINUE_S)
#define SHA_DMA_CONTINUE_V  0x00000001
#define SHA_DMA_CONTINUE_S  0

/* SHA_INT_CLEAR_REG register
 * DMA-SHA interrupt clear register
 */

#define SHA_INT_CLEAR_REG (DR_REG_SHA_BASE + 0x24)

/* SHA_CLEAR_INTERRUPT : WO; bitpos: [0]; default: 0;
 * Clears DMA-SHA interrupt.
 */

#define SHA_CLEAR_INTERRUPT    (BIT(0))
#define SHA_CLEAR_INTERRUPT_M  (SHA_CLEAR_INTERRUPT_V << SHA_CLEAR_INTERRUPT_S)
#define SHA_CLEAR_INTERRUPT_V  0x00000001
#define SHA_CLEAR_INTERRUPT_S  0

/* SHA_INT_ENA_REG register
 * DMA-SHA interrupt enable register
 */

#define SHA_INT_ENA_REG (DR_REG_SHA_BASE + 0x28)

/* SHA_INTERRUPT_ENA : R/W; bitpos: [0]; default: 0;
 * Enables DMA-SHA interrupt.
 */

#define SHA_INTERRUPT_ENA    (BIT(0))
#define SHA_INTERRUPT_ENA_M  (SHA_INTERRUPT_ENA_V << SHA_INTERRUPT_ENA_S)
#define SHA_INTERRUPT_ENA_V  0x00000001
#define SHA_INTERRUPT_ENA_S  0

/* SHA_DATE_REG register
 * Version control register.
 */

#define SHA_DATE_REG (DR_REG_SHA_BASE + 0x2c)

/* SHA_DATE : R/W; bitpos: [29:0]; default: 538510338;
 * Version control register
 */

#define SHA_DATE    0x3FFFFFFF
#define SHA_DATE_M  (SHA_DATE_V << SHA_DATE_S)
#define SHA_DATE_V  0x3FFFFFFF
#define SHA_DATE_S  0

/* SHA_H_0_REG register
 * Hash value
 */

#define SHA_H_0_REG (DR_REG_SHA_BASE + 0x40)

/* SHA_H_0 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 0th 32-bit piece of the Hash value.
 */

#define SHA_H_0    0xFFFFFFFF
#define SHA_H_0_M  (SHA_H_0_V << SHA_H_0_S)
#define SHA_H_0_V  0xFFFFFFFF
#define SHA_H_0_S  0

/* SHA_H_1_REG register
 * Hash value
 */

#define SHA_H_1_REG (DR_REG_SHA_BASE + 0x44)

/* SHA_H_1 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 1th 32-bit piece of the Hash value.
 */

#define SHA_H_1    0xFFFFFFFF
#define SHA_H_1_M  (SHA_H_1_V << SHA_H_1_S)
#define SHA_H_1_V  0xFFFFFFFF
#define SHA_H_1_S  0

/* SHA_H_2_REG register
 * Hash value
 */

#define SHA_H_2_REG (DR_REG_SHA_BASE + 0x48)

/* SHA_H_2 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 2th 32-bit piece of the Hash value.
 */

#define SHA_H_2    0xFFFFFFFF
#define SHA_H_2_M  (SHA_H_2_V << SHA_H_2_S)
#define SHA_H_2_V  0xFFFFFFFF
#define SHA_H_2_S  0

/* SHA_H_3_REG register
 * Hash value
 */

#define SHA_H_3_REG (DR_REG_SHA_BASE + 0x4c)

/* SHA_H_3 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 3th 32-bit piece of the Hash value.
 */

#define SHA_H_3    0xFFFFFFFF
#define SHA_H_3_M  (SHA_H_3_V << SHA_H_3_S)
#define SHA_H_3_V  0xFFFFFFFF
#define SHA_H_3_S  0

/* SHA_H_4_REG register
 * Hash value
 */

#define SHA_H_4_REG (DR_REG_SHA_BASE + 0x50)

/* SHA_H_4 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 4th 32-bit piece of the Hash value.
 */

#define SHA_H_4    0xFFFFFFFF
#define SHA_H_4_M  (SHA_H_4_V << SHA_H_4_S)
#define SHA_H_4_V  0xFFFFFFFF
#define SHA_H_4_S  0

/* SHA_H_5_REG register
 * Hash value
 */

#define SHA_H_5_REG (DR_REG_SHA_BASE + 0x54)

/* SHA_H_5 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 5th 32-bit piece of the Hash value.
 */

#define SHA_H_5    0xFFFFFFFF
#define SHA_H_5_M  (SHA_H_5_V << SHA_H_5_S)
#define SHA_H_5_V  0xFFFFFFFF
#define SHA_H_5_S  0

/* SHA_H_6_REG register
 * Hash value
 */

#define SHA_H_6_REG (DR_REG_SHA_BASE + 0x58)

/* SHA_H_6 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 6th 32-bit piece of the Hash value.
 */

#define SHA_H_6    0xFFFFFFFF
#define SHA_H_6_M  (SHA_H_6_V << SHA_H_6_S)
#define SHA_H_6_V  0xFFFFFFFF
#define SHA_H_6_S  0

/* SHA_H_7_REG register
 * Hash value
 */

#define SHA_H_7_REG (DR_REG_SHA_BASE + 0x5c)

/* SHA_H_7 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 7th 32-bit piece of the Hash value.
 */

#define SHA_H_7    0xFFFFFFFF
#define SHA_H_7_M  (SHA_H_7_V << SHA_H_7_S)
#define SHA_H_7_V  0xFFFFFFFF
#define SHA_H_7_S  0

/* SHA_H_8_REG register
 * Hash value
 */

#define SHA_H_8_REG (DR_REG_SHA_BASE + 0x60)

/* SHA_H_8 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 8th 32-bit piece of the Hash value.
 */

#define SHA_H_8    0xFFFFFFFF
#define SHA_H_8_M  (SHA_H_8_V << SHA_H_8_S)
#define SHA_H_8_V  0xFFFFFFFF
#define SHA_H_8_S  0

/* SHA_H_9_REG register
 * Hash value
 */

#define SHA_H_9_REG (DR_REG_SHA_BASE + 0x64)

/* SHA_H_9 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 9th 32-bit piece of the Hash value.
 */

#define SHA_H_9    0xFFFFFFFF
#define SHA_H_9_M  (SHA_H_9_V << SHA_H_9_S)
#define SHA_H_9_V  0xFFFFFFFF
#define SHA_H_9_S  0

/* SHA_H_10_REG register
 * Hash value
 */

#define SHA_H_10_REG (DR_REG_SHA_BASE + 0x68)

/* SHA_H_10 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 10th 32-bit piece of the Hash value.
 */

#define SHA_H_10    0xFFFFFFFF
#define SHA_H_10_M  (SHA_H_10_V << SHA_H_10_S)
#define SHA_H_10_V  0xFFFFFFFF
#define SHA_H_10_S  0

/* SHA_H_11_REG register
 * Hash value
 */

#define SHA_H_11_REG (DR_REG_SHA_BASE + 0x6c)

/* SHA_H_11 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 11th 32-bit piece of the Hash value.
 */

#define SHA_H_11    0xFFFFFFFF
#define SHA_H_11_M  (SHA_H_11_V << SHA_H_11_S)
#define SHA_H_11_V  0xFFFFFFFF
#define SHA_H_11_S  0

/* SHA_H_12_REG register
 * Hash value
 */

#define SHA_H_12_REG (DR_REG_SHA_BASE + 0x70)

/* SHA_H_12 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 12th 32-bit piece of the Hash value.
 */

#define SHA_H_12    0xFFFFFFFF
#define SHA_H_12_M  (SHA_H_12_V << SHA_H_12_S)
#define SHA_H_12_V  0xFFFFFFFF
#define SHA_H_12_S  0

/* SHA_H_13_REG register
 * Hash value
 */

#define SHA_H_13_REG (DR_REG_SHA_BASE + 0x74)

/* SHA_H_13 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 13th 32-bit piece of the Hash value.
 */

#define SHA_H_13    0xFFFFFFFF
#define SHA_H_13_M  (SHA_H_13_V << SHA_H_13_S)
#define SHA_H_13_V  0xFFFFFFFF
#define SHA_H_13_S  0

/* SHA_H_14_REG register
 * Hash value
 */

#define SHA_H_14_REG (DR_REG_SHA_BASE + 0x78)

/* SHA_H_14 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 14th 32-bit piece of the Hash value.
 */

#define SHA_H_14    0xFFFFFFFF
#define SHA_H_14_M  (SHA_H_14_V << SHA_H_14_S)
#define SHA_H_14_V  0xFFFFFFFF
#define SHA_H_14_S  0

/* SHA_H_15_REG register
 * Hash value
 */

#define SHA_H_15_REG (DR_REG_SHA_BASE + 0x7c)

/* SHA_H_15 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 15th 32-bit piece of the Hash value.
 */

#define SHA_H_15    0xFFFFFFFF
#define SHA_H_15_M  (SHA_H_15_V << SHA_H_15_S)
#define SHA_H_15_V  0xFFFFFFFF
#define SHA_H_15_S  0

/* SHA_M_0_REG register
 * Message
 */

#define SHA_M_0_REG (DR_REG_SHA_BASE + 0x80)

/* SHA_M_0 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 0th 32-bit piece of the message.
 */

#define SHA_M_0    0xFFFFFFFF
#define SHA_M_0_M  (SHA_M_0_V << SHA_M_0_S)
#define SHA_M_0_V  0xFFFFFFFF
#define SHA_M_0_S  0

/* SHA_M_1_REG register
 * Message
 */

#define SHA_M_1_REG (DR_REG_SHA_BASE + 0x84)

/* SHA_M_1 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 1th 32-bit piece of the message.
 */

#define SHA_M_1    0xFFFFFFFF
#define SHA_M_1_M  (SHA_M_1_V << SHA_M_1_S)
#define SHA_M_1_V  0xFFFFFFFF
#define SHA_M_1_S  0

/* SHA_M_2_REG register
 * Message
 */

#define SHA_M_2_REG (DR_REG_SHA_BASE + 0x88)

/* SHA_M_2 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 2th 32-bit piece of the message.
 */

#define SHA_M_2    0xFFFFFFFF
#define SHA_M_2_M  (SHA_M_2_V << SHA_M_2_S)
#define SHA_M_2_V  0xFFFFFFFF
#define SHA_M_2_S  0

/* SHA_M_3_REG register
 * Message
 */

#define SHA_M_3_REG (DR_REG_SHA_BASE + 0x8c)

/* SHA_M_3 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 3th 32-bit piece of the message.
 */

#define SHA_M_3    0xFFFFFFFF
#define SHA_M_3_M  (SHA_M_3_V << SHA_M_3_S)
#define SHA_M_3_V  0xFFFFFFFF
#define SHA_M_3_S  0

/* SHA_M_4_REG register
 * Message
 */

#define SHA_M_4_REG (DR_REG_SHA_BASE + 0x90)

/* SHA_M_4 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 4th 32-bit piece of the message.
 */

#define SHA_M_4    0xFFFFFFFF
#define SHA_M_4_M  (SHA_M_4_V << SHA_M_4_S)
#define SHA_M_4_V  0xFFFFFFFF
#define SHA_M_4_S  0

/* SHA_M_5_REG register
 * Message
 */

#define SHA_M_5_REG (DR_REG_SHA_BASE + 0x94)

/* SHA_M_5 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 5th 32-bit piece of the message.
 */

#define SHA_M_5    0xFFFFFFFF
#define SHA_M_5_M  (SHA_M_5_V << SHA_M_5_S)
#define SHA_M_5_V  0xFFFFFFFF
#define SHA_M_5_S  0

/* SHA_M_6_REG register
 * Message
 */

#define SHA_M_6_REG (DR_REG_SHA_BASE + 0x98)

/* SHA_M_6 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 6th 32-bit piece of the message.
 */

#define SHA_M_6    0xFFFFFFFF
#define SHA_M_6_M  (SHA_M_6_V << SHA_M_6_S)
#define SHA_M_6_V  0xFFFFFFFF
#define SHA_M_6_S  0

/* SHA_M_7_REG register
 * Message
 */

#define SHA_M_7_REG (DR_REG_SHA_BASE + 0x9c)

/* SHA_M_7 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 7th 32-bit piece of the message.
 */

#define SHA_M_7    0xFFFFFFFF
#define SHA_M_7_M  (SHA_M_7_V << SHA_M_7_S)
#define SHA_M_7_V  0xFFFFFFFF
#define SHA_M_7_S  0

/* SHA_M_8_REG register
 * Message
 */

#define SHA_M_8_REG (DR_REG_SHA_BASE + 0xa0)

/* SHA_M_8 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 8th 32-bit piece of the message.
 */

#define SHA_M_8    0xFFFFFFFF
#define SHA_M_8_M  (SHA_M_8_V << SHA_M_8_S)
#define SHA_M_8_V  0xFFFFFFFF
#define SHA_M_8_S  0

/* SHA_M_9_REG register
 * Message
 */

#define SHA_M_9_REG (DR_REG_SHA_BASE + 0xa4)

/* SHA_M_9 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 9th 32-bit piece of the message.
 */

#define SHA_M_9    0xFFFFFFFF
#define SHA_M_9_M  (SHA_M_9_V << SHA_M_9_S)
#define SHA_M_9_V  0xFFFFFFFF
#define SHA_M_9_S  0

/* SHA_M_10_REG register
 * Message
 */

#define SHA_M_10_REG (DR_REG_SHA_BASE + 0xa8)

/* SHA_M_10 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 10th 32-bit piece of the message.
 */

#define SHA_M_10    0xFFFFFFFF
#define SHA_M_10_M  (SHA_M_10_V << SHA_M_10_S)
#define SHA_M_10_V  0xFFFFFFFF
#define SHA_M_10_S  0

/* SHA_M_11_REG register
 * Message
 */

#define SHA_M_11_REG (DR_REG_SHA_BASE + 0xac)

/* SHA_M_11 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 11th 32-bit piece of the message.
 */

#define SHA_M_11    0xFFFFFFFF
#define SHA_M_11_M  (SHA_M_11_V << SHA_M_11_S)
#define SHA_M_11_V  0xFFFFFFFF
#define SHA_M_11_S  0

/* SHA_M_12_REG register
 * Message
 */

#define SHA_M_12_REG (DR_REG_SHA_BASE + 0xb0)

/* SHA_M_12 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 12th 32-bit piece of the message.
 */

#define SHA_M_12    0xFFFFFFFF
#define SHA_M_12_M  (SHA_M_12_V << SHA_M_12_S)
#define SHA_M_12_V  0xFFFFFFFF
#define SHA_M_12_S  0

/* SHA_M_13_REG register
 * Message
 */

#define SHA_M_13_REG (DR_REG_SHA_BASE + 0xb4)

/* SHA_M_13 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 13th 32-bit piece of the message.
 */

#define SHA_M_13    0xFFFFFFFF
#define SHA_M_13_M  (SHA_M_13_V << SHA_M_13_S)
#define SHA_M_13_V  0xFFFFFFFF
#define SHA_M_13_S  0

/* SHA_M_14_REG register
 * Message
 */

#define SHA_M_14_REG (DR_REG_SHA_BASE + 0xb8)

/* SHA_M_14 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 14th 32-bit piece of the message.
 */

#define SHA_M_14    0xFFFFFFFF
#define SHA_M_14_M  (SHA_M_14_V << SHA_M_14_S)
#define SHA_M_14_V  0xFFFFFFFF
#define SHA_M_14_S  0

/* SHA_M_15_REG register
 * Message
 */

#define SHA_M_15_REG (DR_REG_SHA_BASE + 0xbc)

/* SHA_M_15 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 15th 32-bit piece of the message.
 */

#define SHA_M_15    0xFFFFFFFF
#define SHA_M_15_M  (SHA_M_15_V << SHA_M_15_S)
#define SHA_M_15_V  0xFFFFFFFF
#define SHA_M_15_S  0

/* SHA_M_16_REG register
 * Message
 */

#define SHA_M_16_REG (DR_REG_SHA_BASE + 0xc0)

/* SHA_M_16 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 16th 32-bit piece of the message.
 */

#define SHA_M_16    0xFFFFFFFF
#define SHA_M_16_M  (SHA_M_16_V << SHA_M_16_S)
#define SHA_M_16_V  0xFFFFFFFF
#define SHA_M_16_S  0

/* SHA_M_17_REG register
 * Message
 */

#define SHA_M_17_REG (DR_REG_SHA_BASE + 0xc4)

/* SHA_M_17 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 17th 32-bit piece of the message.
 */

#define SHA_M_17    0xFFFFFFFF
#define SHA_M_17_M  (SHA_M_17_V << SHA_M_17_S)
#define SHA_M_17_V  0xFFFFFFFF
#define SHA_M_17_S  0

/* SHA_M_18_REG register
 * Message
 */

#define SHA_M_18_REG (DR_REG_SHA_BASE + 0xc8)

/* SHA_M_18 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 18th 32-bit piece of the message.
 */

#define SHA_M_18    0xFFFFFFFF
#define SHA_M_18_M  (SHA_M_18_V << SHA_M_18_S)
#define SHA_M_18_V  0xFFFFFFFF
#define SHA_M_18_S  0

/* SHA_M_19_REG register
 * Message
 */

#define SHA_M_19_REG (DR_REG_SHA_BASE + 0xcc)

/* SHA_M_19 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 19th 32-bit piece of the message.
 */

#define SHA_M_19    0xFFFFFFFF
#define SHA_M_19_M  (SHA_M_19_V << SHA_M_19_S)
#define SHA_M_19_V  0xFFFFFFFF
#define SHA_M_19_S  0

/* SHA_M_20_REG register
 * Message
 */

#define SHA_M_20_REG (DR_REG_SHA_BASE + 0xd0)

/* SHA_M_20 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 20th 32-bit piece of the message.
 */

#define SHA_M_20    0xFFFFFFFF
#define SHA_M_20_M  (SHA_M_20_V << SHA_M_20_S)
#define SHA_M_20_V  0xFFFFFFFF
#define SHA_M_20_S  0

/* SHA_M_21_REG register
 * Message
 */

#define SHA_M_21_REG (DR_REG_SHA_BASE + 0xd4)

/* SHA_M_21 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 21th 32-bit piece of the message.
 */

#define SHA_M_21    0xFFFFFFFF
#define SHA_M_21_M  (SHA_M_21_V << SHA_M_21_S)
#define SHA_M_21_V  0xFFFFFFFF
#define SHA_M_21_S  0

/* SHA_M_22_REG register
 * Message
 */

#define SHA_M_22_REG (DR_REG_SHA_BASE + 0xd8)

/* SHA_M_22 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 22th 32-bit piece of the message.
 */

#define SHA_M_22    0xFFFFFFFF
#define SHA_M_22_M  (SHA_M_22_V << SHA_M_22_S)
#define SHA_M_22_V  0xFFFFFFFF
#define SHA_M_22_S  0

/* SHA_M_23_REG register
 * Message
 */

#define SHA_M_23_REG (DR_REG_SHA_BASE + 0xdc)

/* SHA_M_23 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 23th 32-bit piece of the message.
 */

#define SHA_M_23    0xFFFFFFFF
#define SHA_M_23_M  (SHA_M_23_V << SHA_M_23_S)
#define SHA_M_23_V  0xFFFFFFFF
#define SHA_M_23_S  0

/* SHA_M_24_REG register
 * Message
 */

#define SHA_M_24_REG (DR_REG_SHA_BASE + 0xe0)

/* SHA_M_24 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 24th 32-bit piece of the message.
 */

#define SHA_M_24    0xFFFFFFFF
#define SHA_M_24_M  (SHA_M_24_V << SHA_M_24_S)
#define SHA_M_24_V  0xFFFFFFFF
#define SHA_M_24_S  0

/* SHA_M_25_REG register
 * Message
 */

#define SHA_M_25_REG (DR_REG_SHA_BASE + 0xe4)

/* SHA_M_25 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 25th 32-bit piece of the message.
 */

#define SHA_M_25    0xFFFFFFFF
#define SHA_M_25_M  (SHA_M_25_V << SHA_M_25_S)
#define SHA_M_25_V  0xFFFFFFFF
#define SHA_M_25_S  0

/* SHA_M_26_REG register
 * Message
 */

#define SHA_M_26_REG (DR_REG_SHA_BASE + 0xe8)

/* SHA_M_26 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 26th 32-bit piece of the message.
 */

#define SHA_M_26    0xFFFFFFFF
#define SHA_M_26_M  (SHA_M_26_V << SHA_M_26_S)
#define SHA_M_26_V  0xFFFFFFFF
#define SHA_M_26_S  0

/* SHA_M_27_REG register
 * Message
 */

#define SHA_M_27_REG (DR_REG_SHA_BASE + 0xec)

/* SHA_M_27 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 27th 32-bit piece of the message.
 */

#define SHA_M_27    0xFFFFFFFF
#define SHA_M_27_M  (SHA_M_27_V << SHA_M_27_S)
#define SHA_M_27_V  0xFFFFFFFF
#define SHA_M_27_S  0

/* SHA_M_28_REG register
 * Message
 */

#define SHA_M_28_REG (DR_REG_SHA_BASE + 0xf0)

/* SHA_M_28 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 28th 32-bit piece of the message.
 */

#define SHA_M_28    0xFFFFFFFF
#define SHA_M_28_M  (SHA_M_28_V << SHA_M_28_S)
#define SHA_M_28_V  0xFFFFFFFF
#define SHA_M_28_S  0

/* SHA_M_29_REG register
 * Message
 */

#define SHA_M_29_REG (DR_REG_SHA_BASE + 0xf4)

/* SHA_M_29 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 29th 32-bit piece of the message.
 */

#define SHA_M_29    0xFFFFFFFF
#define SHA_M_29_M  (SHA_M_29_V << SHA_M_29_S)
#define SHA_M_29_V  0xFFFFFFFF
#define SHA_M_29_S  0

/* SHA_M_30_REG register
 * Message
 */

#define SHA_M_30_REG (DR_REG_SHA_BASE + 0xf8)

/* SHA_M_30 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 30th 32-bit piece of the message.
 */

#define SHA_M_30    0xFFFFFFFF
#define SHA_M_30_M  (SHA_M_30_V << SHA_M_30_S)
#define SHA_M_30_V  0xFFFFFFFF
#define SHA_M_30_S  0

/* SHA_M_31_REG register
 * Message
 */

#define SHA_M_31_REG (DR_REG_SHA_BASE + 0xfc)

/* SHA_M_31 : R/W; bitpos: [31:0]; default: 0;
 * Stores the 31th 32-bit piece of the message.
 */

#define SHA_M_31    0xFFFFFFFF
#define SHA_M_31_M  (SHA_M_31_V << SHA_M_31_S)
#define SHA_M_31_V  0xFFFFFFFF
#define SHA_M_31_S  0

#endif /* __ARCH_RISCV_SRC_ESP32C3_HARDWARE_ESP32C3_SHA_H */
