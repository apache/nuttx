/****************************************************************************
 * arch/risc-v/src/esp32c3/hardware/esp32c3_aes.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_HARDWARE_ESP32C3_AES_H
#define __ARCH_RISCV_SRC_ESP32C3_HARDWARE_ESP32C3_AES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32c3_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AES_KEY_0_REG register
 * AES key register 0
 */

#define AES_KEY_0_REG (DR_REG_AES_BASE + 0x0)

/* AES_KEY_0 : R/W; bitpos: [31:0]; default: 0;
 * Stores AES keys.
 */

#define AES_KEY_0    0xFFFFFFFF
#define AES_KEY_0_M  (AES_KEY_0_V << AES_KEY_0_S)
#define AES_KEY_0_V  0xFFFFFFFF
#define AES_KEY_0_S  0

/* AES_KEY_1_REG register
 * AES key register 1
 */

#define AES_KEY_1_REG (DR_REG_AES_BASE + 0x4)

/* AES_KEY_1 : R/W; bitpos: [31:0]; default: 0;
 * Stores AES keys.
 */

#define AES_KEY_1    0xFFFFFFFF
#define AES_KEY_1_M  (AES_KEY_1_V << AES_KEY_1_S)
#define AES_KEY_1_V  0xFFFFFFFF
#define AES_KEY_1_S  0

/* AES_KEY_2_REG register
 * AES key register 2
 */

#define AES_KEY_2_REG (DR_REG_AES_BASE + 0x8)

/* AES_KEY_2 : R/W; bitpos: [31:0]; default: 0;
 * Stores AES keys.
 */

#define AES_KEY_2    0xFFFFFFFF
#define AES_KEY_2_M  (AES_KEY_2_V << AES_KEY_2_S)
#define AES_KEY_2_V  0xFFFFFFFF
#define AES_KEY_2_S  0

/* AES_KEY_3_REG register
 * AES key register 3
 */

#define AES_KEY_3_REG (DR_REG_AES_BASE + 0xc)

/* AES_KEY_3 : R/W; bitpos: [31:0]; default: 0;
 * Stores AES keys.
 */

#define AES_KEY_3    0xFFFFFFFF
#define AES_KEY_3_M  (AES_KEY_3_V << AES_KEY_3_S)
#define AES_KEY_3_V  0xFFFFFFFF
#define AES_KEY_3_S  0

/* AES_KEY_4_REG register
 * AES key register 4
 */

#define AES_KEY_4_REG (DR_REG_AES_BASE + 0x10)

/* AES_KEY_4 : R/W; bitpos: [31:0]; default: 0;
 * Stores AES keys.
 */

#define AES_KEY_4    0xFFFFFFFF
#define AES_KEY_4_M  (AES_KEY_4_V << AES_KEY_4_S)
#define AES_KEY_4_V  0xFFFFFFFF
#define AES_KEY_4_S  0

/* AES_KEY_5_REG register
 * AES key register 5
 */

#define AES_KEY_5_REG (DR_REG_AES_BASE + 0x14)

/* AES_KEY_5 : R/W; bitpos: [31:0]; default: 0;
 * Stores AES keys.
 */

#define AES_KEY_5    0xFFFFFFFF
#define AES_KEY_5_M  (AES_KEY_5_V << AES_KEY_5_S)
#define AES_KEY_5_V  0xFFFFFFFF
#define AES_KEY_5_S  0

/* AES_KEY_6_REG register
 * AES key register 6
 */

#define AES_KEY_6_REG (DR_REG_AES_BASE + 0x18)

/* AES_KEY_6 : R/W; bitpos: [31:0]; default: 0;
 * Stores AES keys.
 */

#define AES_KEY_6    0xFFFFFFFF
#define AES_KEY_6_M  (AES_KEY_6_V << AES_KEY_6_S)
#define AES_KEY_6_V  0xFFFFFFFF
#define AES_KEY_6_S  0

/* AES_KEY_7_REG register
 * AES key register 7
 */

#define AES_KEY_7_REG (DR_REG_AES_BASE + 0x1c)

/* AES_KEY_7 : R/W; bitpos: [31:0]; default: 0;
 * Stores AES keys.
 */

#define AES_KEY_7    0xFFFFFFFF
#define AES_KEY_7_M  (AES_KEY_7_V << AES_KEY_7_S)
#define AES_KEY_7_V  0xFFFFFFFF
#define AES_KEY_7_S  0

/* AES_TEXT_IN_0_REG register
 * Source data register 0
 */

#define AES_TEXT_IN_0_REG (DR_REG_AES_BASE + 0x20)

/* AES_TEXT_IN_0 : R/W; bitpos: [31:0]; default: 0;
 * Stores the source data when the AES Accelerator operates in the Typical
 * AES working mode.
 */

#define AES_TEXT_IN_0    0xFFFFFFFF
#define AES_TEXT_IN_0_M  (AES_TEXT_IN_0_V << AES_TEXT_IN_0_S)
#define AES_TEXT_IN_0_V  0xFFFFFFFF
#define AES_TEXT_IN_0_S  0

/* AES_TEXT_IN_1_REG register
 * Source data register 1
 */

#define AES_TEXT_IN_1_REG (DR_REG_AES_BASE + 0x24)

/* AES_TEXT_IN_1 : R/W; bitpos: [31:0]; default: 0;
 * Stores the source data when the AES Accelerator operates in the Typical
 * AES working mode.
 */

#define AES_TEXT_IN_1    0xFFFFFFFF
#define AES_TEXT_IN_1_M  (AES_TEXT_IN_1_V << AES_TEXT_IN_1_S)
#define AES_TEXT_IN_1_V  0xFFFFFFFF
#define AES_TEXT_IN_1_S  0

/* AES_TEXT_IN_2_REG register
 * Source data register 2
 */

#define AES_TEXT_IN_2_REG (DR_REG_AES_BASE + 0x28)

/* AES_TEXT_IN_2 : R/W; bitpos: [31:0]; default: 0;
 * Stores the source data when the AES Accelerator operates in the Typical
 * AES working mode.
 */

#define AES_TEXT_IN_2    0xFFFFFFFF
#define AES_TEXT_IN_2_M  (AES_TEXT_IN_2_V << AES_TEXT_IN_2_S)
#define AES_TEXT_IN_2_V  0xFFFFFFFF
#define AES_TEXT_IN_2_S  0

/* AES_TEXT_IN_3_REG register
 * Source data register 3
 */

#define AES_TEXT_IN_3_REG (DR_REG_AES_BASE + 0x2c)

/* AES_TEXT_IN_3 : R/W; bitpos: [31:0]; default: 0;
 * Stores the source data when the AES Accelerator operates in the Typical
 * AES working mode.
 */

#define AES_TEXT_IN_3    0xFFFFFFFF
#define AES_TEXT_IN_3_M  (AES_TEXT_IN_3_V << AES_TEXT_IN_3_S)
#define AES_TEXT_IN_3_V  0xFFFFFFFF
#define AES_TEXT_IN_3_S  0

/* AES_TEXT_OUT_0_REG register
 * Result data register 0
 */

#define AES_TEXT_OUT_0_REG (DR_REG_AES_BASE + 0x30)

/* AES_TEXT_OUT_0 : R/W; bitpos: [31:0]; default: 0;
 * Stores the result data when the AES Accelerator operates in the Typical
 * AES working mode.
 */

#define AES_TEXT_OUT_0    0xFFFFFFFF
#define AES_TEXT_OUT_0_M  (AES_TEXT_OUT_0_V << AES_TEXT_OUT_0_S)
#define AES_TEXT_OUT_0_V  0xFFFFFFFF
#define AES_TEXT_OUT_0_S  0

/* AES_TEXT_OUT_1_REG register
 * Result data register 1
 */

#define AES_TEXT_OUT_1_REG (DR_REG_AES_BASE + 0x34)

/* AES_TEXT_OUT_1 : R/W; bitpos: [31:0]; default: 0;
 * Stores the result data when the AES Accelerator operates in the Typical
 * AES working mode.
 */

#define AES_TEXT_OUT_1    0xFFFFFFFF
#define AES_TEXT_OUT_1_M  (AES_TEXT_OUT_1_V << AES_TEXT_OUT_1_S)
#define AES_TEXT_OUT_1_V  0xFFFFFFFF
#define AES_TEXT_OUT_1_S  0

/* AES_TEXT_OUT_2_REG register
 * Result data register 2
 */

#define AES_TEXT_OUT_2_REG (DR_REG_AES_BASE + 0x38)

/* AES_TEXT_OUT_2 : R/W; bitpos: [31:0]; default: 0;
 * Stores the result data when the AES Accelerator operates in the Typical
 * AES working mode.
 */

#define AES_TEXT_OUT_2    0xFFFFFFFF
#define AES_TEXT_OUT_2_M  (AES_TEXT_OUT_2_V << AES_TEXT_OUT_2_S)
#define AES_TEXT_OUT_2_V  0xFFFFFFFF
#define AES_TEXT_OUT_2_S  0

/* AES_TEXT_OUT_3_REG register
 * Result data register 3
 */

#define AES_TEXT_OUT_3_REG (DR_REG_AES_BASE + 0x3c)

/* AES_TEXT_OUT_3 : R/W; bitpos: [31:0]; default: 0;
 * Stores the result data when the AES Accelerator operates in the Typical
 * AES working mode.
 */

#define AES_TEXT_OUT_3    0xFFFFFFFF
#define AES_TEXT_OUT_3_M  (AES_TEXT_OUT_3_V << AES_TEXT_OUT_3_S)
#define AES_TEXT_OUT_3_V  0xFFFFFFFF
#define AES_TEXT_OUT_3_S  0

/* AES_MODE_REG register
 * AES working mode configuration register
 */

#define AES_MODE_REG (DR_REG_AES_BASE + 0x40)

/* AES_MODE : R/W; bitpos: [2:0]; default: 0;
 * Defines the operation type of the AES Accelerator operating under the
 * Typical AES working mode.
 * &
 * 0x0(AES_EN_128): AES-EN-128 #
 * 0x1(AES_EN_192): AES-EN-192 #
 * 0x2(AES_EN_256): AES-EN-256 #
 * 0x4(AES_DE_128): AES-DE-128 #
 * 0x5(AES_DE_192): AES-DE-192 #
 * 0x6(AES_DE_256): AES-DE-256
 * &
 */

#define AES_MODE    0x00000007
#define AES_MODE_M  (AES_MODE_V << AES_MODE_S)
#define AES_MODE_V  0x00000007
#define AES_MODE_S  0

/* AES_ENDIAN_REG register
 * Endian configuration register
 */

#define AES_ENDIAN_REG (DR_REG_AES_BASE + 0x44)

/* AES_ENDIAN : R/W; bitpos: [5:0]; default: 0;
 * Defines the endianness of input and output texts.
 * &
 * [1:0] key endian #
 * [3:2] text_in endian or in_stream endian #
 * [5:4] text_out endian or out_stream endian #
 * &
 */

#define AES_ENDIAN    0x0000003F
#define AES_ENDIAN_M  (AES_ENDIAN_V << AES_ENDIAN_S)
#define AES_ENDIAN_V  0x0000003F
#define AES_ENDIAN_S  0

/* AES_TRIGGER_REG register
 * Operation start controlling register
 */

#define AES_TRIGGER_REG (DR_REG_AES_BASE + 0x48)

/* AES_TRIGGER : WO; bitpos: [0]; default: 0;
 * Set this bit to 1 to start AES operation.
 */

#define AES_TRIGGER    (BIT(0))
#define AES_TRIGGER_M  (AES_TRIGGER_V << AES_TRIGGER_S)
#define AES_TRIGGER_V  0x00000001
#define AES_TRIGGER_S  0

/* AES_STATE_REG register
 * Operation status register
 */

#define AES_STATE_REG (DR_REG_AES_BASE + 0x4c)

/* AES_STATE : RO; bitpos: [1:0]; default: 0;
 * Stores the working status of the AES Accelerator. For details, see Table
 * 3 for Typical AES working mode and Table 9 for DMA AES working mode.
 * For typical AES; 0 = idle; 1 = busy.
 * For DMA-AES; 0 = idle; 1 = busy; 2 = calculation_done.
 */

#define AES_STATE    0x00000003
#define AES_STATE_M  (AES_STATE_V << AES_STATE_S)
#define AES_STATE_V  0x00000003
#define AES_STATE_S  0

/* AES_IV_0_REG register
 * initialization vector
 */

#define AES_IV_0_REG (DR_REG_AES_BASE + 0x50)

/* AES_IV_0 : R/W; bitpos: [31:0]; default: 0;
 * This register stores the 0th 32-bit piece of 128-bit initialization vector
 */

#define AES_IV_0    0xFFFFFFFF
#define AES_IV_0_M  (AES_IV_0_V << AES_IV_0_S)
#define AES_IV_0_V  0xFFFFFFFF
#define AES_IV_0_S  0

/* AES_IV_1_REG register
 * initialization vector
 */

#define AES_IV_1_REG (DR_REG_AES_BASE + 0x54)

/* AES_IV_1 : R/W; bitpos: [31:0]; default: 0;
 * This register stores the 1th 32-bit piece of 128-bit initialization vector
 */

#define AES_IV_1    0xFFFFFFFF
#define AES_IV_1_M  (AES_IV_1_V << AES_IV_1_S)
#define AES_IV_1_V  0xFFFFFFFF
#define AES_IV_1_S  0

/* AES_IV_2_REG register
 * initialization vector
 */

#define AES_IV_2_REG (DR_REG_AES_BASE + 0x58)

/* AES_IV_2 : R/W; bitpos: [31:0]; default: 0;
 * This register stores the 2th 32-bit piece of 128-bit initialization vector
 */

#define AES_IV_2    0xFFFFFFFF
#define AES_IV_2_M  (AES_IV_2_V << AES_IV_2_S)
#define AES_IV_2_V  0xFFFFFFFF
#define AES_IV_2_S  0

/* AES_IV_3_REG register
 * initialization vector
 */

#define AES_IV_3_REG (DR_REG_AES_BASE + 0x5c)

/* AES_IV_3 : R/W; bitpos: [31:0]; default: 0;
 * This register stores the 3th 32-bit piece of 128-bit initialization vector
 */

#define AES_IV_3    0xFFFFFFFF
#define AES_IV_3_M  (AES_IV_3_V << AES_IV_3_S)
#define AES_IV_3_V  0xFFFFFFFF
#define AES_IV_3_S  0

/* AES_H_0_REG register
 * GCM hash subkey
 */

#define AES_H_0_REG (DR_REG_AES_BASE + 0x60)

/* AES_H_0 : RO; bitpos: [31:0]; default: 0;
 * GCM hash subkey
 */

#define AES_H_0    0xFFFFFFFF
#define AES_H_0_M  (AES_H_0_V << AES_H_0_S)
#define AES_H_0_V  0xFFFFFFFF
#define AES_H_0_S  0

/* AES_H_1_REG register
 * GCM hash subkey
 */

#define AES_H_1_REG (DR_REG_AES_BASE + 0x64)

/* AES_H_1 : RO; bitpos: [31:0]; default: 0;
 * GCM hash subkey
 */

#define AES_H_1    0xFFFFFFFF
#define AES_H_1_M  (AES_H_1_V << AES_H_1_S)
#define AES_H_1_V  0xFFFFFFFF
#define AES_H_1_S  0

/* AES_H_2_REG register
 * GCM hash subkey
 */

#define AES_H_2_REG (DR_REG_AES_BASE + 0x68)

/* AES_H_2 : RO; bitpos: [31:0]; default: 0;
 * GCM hash subkey
 */

#define AES_H_2    0xFFFFFFFF
#define AES_H_2_M  (AES_H_2_V << AES_H_2_S)
#define AES_H_2_V  0xFFFFFFFF
#define AES_H_2_S  0

/* AES_H_3_REG register
 * GCM hash subkey
 */

#define AES_H_3_REG (DR_REG_AES_BASE + 0x6c)

/* AES_H_3 : RO; bitpos: [31:0]; default: 0;
 * GCM hash subkey
 */

#define AES_H_3    0xFFFFFFFF
#define AES_H_3_M  (AES_H_3_V << AES_H_3_S)
#define AES_H_3_V  0xFFFFFFFF
#define AES_H_3_S  0

/* AES_J0_0_REG register
 * J0
 */

#define AES_J0_0_REG (DR_REG_AES_BASE + 0x70)

/* AES_J0_0 : R/W; bitpos: [31:0]; default: 0;
 * This register stores the 0th 32-bit piece of 128-bit J0
 */

#define AES_J0_0    0xFFFFFFFF
#define AES_J0_0_M  (AES_J0_0_V << AES_J0_0_S)
#define AES_J0_0_V  0xFFFFFFFF
#define AES_J0_0_S  0

/* AES_J0_1_REG register
 * J0
 */

#define AES_J0_1_REG (DR_REG_AES_BASE + 0x74)

/* AES_J0_1 : R/W; bitpos: [31:0]; default: 0;
 * This register stores the 1th 32-bit piece of 128-bit J0
 */

#define AES_J0_1    0xFFFFFFFF
#define AES_J0_1_M  (AES_J0_1_V << AES_J0_1_S)
#define AES_J0_1_V  0xFFFFFFFF
#define AES_J0_1_S  0

/* AES_J0_2_REG register
 * J0
 */

#define AES_J0_2_REG (DR_REG_AES_BASE + 0x78)

/* AES_J0_2 : R/W; bitpos: [31:0]; default: 0;
 * This register stores the 2th 32-bit piece of 128-bit J0
 */

#define AES_J0_2    0xFFFFFFFF
#define AES_J0_2_M  (AES_J0_2_V << AES_J0_2_S)
#define AES_J0_2_V  0xFFFFFFFF
#define AES_J0_2_S  0

/* AES_J0_3_REG register
 * J0
 */

#define AES_J0_3_REG (DR_REG_AES_BASE + 0x7c)

/* AES_J0_3 : R/W; bitpos: [31:0]; default: 0;
 * This register stores the 3th 32-bit piece of 128-bit J0
 */

#define AES_J0_3    0xFFFFFFFF
#define AES_J0_3_M  (AES_J0_3_V << AES_J0_3_S)
#define AES_J0_3_V  0xFFFFFFFF
#define AES_J0_3_S  0

/* AES_T0_0_REG register
 * T0
 */

#define AES_T0_0_REG (DR_REG_AES_BASE + 0x80)

/* AES_T0_0 : RO; bitpos: [31:0]; default: 0;
 * This register stores the 0th 32-bit piece of 128-bit T0
 */

#define AES_T0_0    0xFFFFFFFF
#define AES_T0_0_M  (AES_T0_0_V << AES_T0_0_S)
#define AES_T0_0_V  0xFFFFFFFF
#define AES_T0_0_S  0

/* AES_T0_1_REG register
 * T0
 */

#define AES_T0_1_REG (DR_REG_AES_BASE + 0x84)

/* AES_T0_1 : RO; bitpos: [31:0]; default: 0;
 * This register stores the 1th 32-bit piece of 128-bit T0
 */

#define AES_T0_1    0xFFFFFFFF
#define AES_T0_1_M  (AES_T0_1_V << AES_T0_1_S)
#define AES_T0_1_V  0xFFFFFFFF
#define AES_T0_1_S  0

/* AES_T0_2_REG register
 * T0
 */

#define AES_T0_2_REG (DR_REG_AES_BASE + 0x88)

/* AES_T0_2 : RO; bitpos: [31:0]; default: 0;
 * This register stores the 2th 32-bit piece of 128-bit T0
 */

#define AES_T0_2    0xFFFFFFFF
#define AES_T0_2_M  (AES_T0_2_V << AES_T0_2_S)
#define AES_T0_2_V  0xFFFFFFFF
#define AES_T0_2_S  0

/* AES_T0_3_REG register
 * T0
 */

#define AES_T0_3_REG (DR_REG_AES_BASE + 0x8c)

/* AES_T0_3 : RO; bitpos: [31:0]; default: 0;
 * This register stores the 3th 32-bit piece of 128-bit T0
 */

#define AES_T0_3    0xFFFFFFFF
#define AES_T0_3_M  (AES_T0_3_V << AES_T0_3_S)
#define AES_T0_3_V  0xFFFFFFFF
#define AES_T0_3_S  0

/* AES_DMA_ENABLE_REG register
 * DMA enable register
 */

#define AES_DMA_ENABLE_REG (DR_REG_AES_BASE + 0x90)

/* AES_DMA_ENABLE : R/W; bitpos: [0]; default: 0;
 * Defines the working mode of the AES Accelerator. For details, see Table 1.
 * 1'h0: typical AES operation
 * 1'h1: DMA-AES operation
 */

#define AES_DMA_ENABLE    (BIT(0))
#define AES_DMA_ENABLE_M  (AES_DMA_ENABLE_V << AES_DMA_ENABLE_S)
#define AES_DMA_ENABLE_V  0x00000001
#define AES_DMA_ENABLE_S  0

/* AES_BLOCK_MODE_REG register
 * Block operation type register
 */

#define AES_BLOCK_MODE_REG (DR_REG_AES_BASE + 0x94)

/* AES_BLOCK_MODE : R/W; bitpos: [2:0]; default: 0;
 * Defines the operation type of the AES Accelerator operating under the
 * DMA-AES working mode. For details, see Table 8.
 * &
 * 3'h0(BLOCK_MODE_ECB): ECB #
 * 3'h1(BLOCK_MODE_CBC): CBC #
 * 3'h2(BLOCK_MODE_OFB): OFB #
 * 3'h3(BLOCK_MODE_CTR): CTR #
 * 3'h4(BLOCK_MODE_CFB8): CFB-8 #
 * 3'h5(BLOCK_MODE_CFB128): CFB-128 #
 * 3'h6(BLOCK_MODE_GCM): GCM
 * &
 */

#define AES_BLOCK_MODE    0x00000007
#define AES_BLOCK_MODE_M  (AES_BLOCK_MODE_V << AES_BLOCK_MODE_S)
#define AES_BLOCK_MODE_V  0x00000007
#define AES_BLOCK_MODE_S  0

/* AES_BLOCK_NUM_REG register
 * Block number configuration register
 */

#define AES_BLOCK_NUM_REG (DR_REG_AES_BASE + 0x98)

/* AES_BLOCK_NUM : R/W; bitpos: [31:0]; default: 0;
 * Stores the Block Number of plaintext or cipertext when the AES
 * Accelerator operates under the DMA-AES working mode. For details, see
 * Section 1.5.4.
 */

#define AES_BLOCK_NUM    0xFFFFFFFF
#define AES_BLOCK_NUM_M  (AES_BLOCK_NUM_V << AES_BLOCK_NUM_S)
#define AES_BLOCK_NUM_V  0xFFFFFFFF
#define AES_BLOCK_NUM_S  0

/* AES_INC_SEL_REG register
 * Standard incrementing function register
 */

#define AES_INC_SEL_REG (DR_REG_AES_BASE + 0x9c)

/* AES_INC_SEL : R/W; bitpos: [0]; default: 0;
 * Defines the Standard Incrementing Function for CTR block operation. Set
 * this bit to 0 or 1 to choose INC 32 or INC 128 .
 */

#define AES_INC_SEL    (BIT(0))
#define AES_INC_SEL_M  (AES_INC_SEL_V << AES_INC_SEL_S)
#define AES_INC_SEL_V  0x00000001
#define AES_INC_SEL_S  0

/* AES_AAD_BLOCK_NUM_REG register
 * AAD block number configuration register
 */

#define AES_AAD_BLOCK_NUM_REG (DR_REG_AES_BASE + 0xa0)

/* AES_AAD_BLOCK_NUM : R/W; bitpos: [31:0]; default: 0;
 * Stores the ADD Block Number for the GCM operation.
 */

#define AES_AAD_BLOCK_NUM    0xFFFFFFFF
#define AES_AAD_BLOCK_NUM_M  (AES_AAD_BLOCK_NUM_V << AES_AAD_BLOCK_NUM_S)
#define AES_AAD_BLOCK_NUM_V  0xFFFFFFFF
#define AES_AAD_BLOCK_NUM_S  0

/* AES_REMAINDER_BIT_NUM_REG register
 * Remainder bit number of plaintext/ciphertext
 */

#define AES_REMAINDER_BIT_NUM_REG (DR_REG_AES_BASE + 0xa4)

/* AES_REMAINDER_BIT_NUM : R/W; bitpos: [6:0]; default: 0;
 * Stores the Remainder Bit Number for the GCM operation.
 */

#define AES_REMAINDER_BIT_NUM    0x0000007F
#define AES_REMAINDER_BIT_NUM_M  (AES_REMAINDER_BIT_NUM_V << AES_REMAINDER_BIT_NUM_S)
#define AES_REMAINDER_BIT_NUM_V  0x0000007F
#define AES_REMAINDER_BIT_NUM_S  0

/* AES_CONTINUE_REG register
 * Operation continue controlling register
 */

#define AES_CONTINUE_REG (DR_REG_AES_BASE + 0xa8)

/* AES_CONTINUE : WO; bitpos: [0]; default: 0;
 * Set this bit to 1 to continue AES operation.
 */

#define AES_CONTINUE    (BIT(0))
#define AES_CONTINUE_M  (AES_CONTINUE_V << AES_CONTINUE_S)
#define AES_CONTINUE_V  0x00000001
#define AES_CONTINUE_S  0

/* AES_INT_CLR_REG register
 * DMA-AES interrupt clear register
 */

#define AES_INT_CLR_REG (DR_REG_AES_BASE + 0xac)

/* AES_INT_CLR : WO; bitpos: [0]; default: 0;
 * Set this bit to 1 to clear AES interrupt.
 */

#define AES_INT_CLR    (BIT(0))
#define AES_INT_CLR_M  (AES_INT_CLR_V << AES_INT_CLR_S)
#define AES_INT_CLR_V  0x00000001
#define AES_INT_CLR_S  0

/* AES_INT_ENA_REG register
 * DMA-AES interrupt enable register
 */

#define AES_INT_ENA_REG (DR_REG_AES_BASE + 0xb0)

/* AES_INT_ENA : R/W; bitpos: [0]; default: 0;
 * Set this bit to 1 to enable AES interrupt and 0 to disable interrupt.
 */

#define AES_INT_ENA    (BIT(0))
#define AES_INT_ENA_M  (AES_INT_ENA_V << AES_INT_ENA_S)
#define AES_INT_ENA_V  0x00000001
#define AES_INT_ENA_S  0

/* AES_DATE_REG register
 * Version control register
 */

#define AES_DATE_REG (DR_REG_AES_BASE + 0xb4)

/* AES_DATE : R/W; bitpos: [29:0]; default: 538510612;
 * Version control register
 */

#define AES_DATE    0x3FFFFFFF
#define AES_DATE_M  (AES_DATE_V << AES_DATE_S)
#define AES_DATE_V  0x3FFFFFFF
#define AES_DATE_S  0

/* AES_DMA_EXIT_REG register
 * Operation exit controlling register
 */

#define AES_DMA_EXIT_REG (DR_REG_AES_BASE + 0xb8)

/* AES_DMA_EXIT : WO; bitpos: [0]; default: 0;
 * Set this bit to 1 to exit AES operation. This register is only effective
 * for DMA-AES operation.
 */

#define AES_DMA_EXIT    (BIT(0))
#define AES_DMA_EXIT_M  (AES_DMA_EXIT_V << AES_DMA_EXIT_S)
#define AES_DMA_EXIT_V  0x00000001
#define AES_DMA_EXIT_S  0

#endif /* __ARCH_RISCV_SRC_ESP32C3_HARDWARE_ESP32C3_AES_H */
