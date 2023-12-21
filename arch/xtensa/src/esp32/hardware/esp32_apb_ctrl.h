/****************************************************************************
 * arch/xtensa/src/esp32/hardware/esp32_apb_ctrl.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_APB_CTRL_H
#define __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_APB_CTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* APB_CTRL_SYSCLK_CONF_REG register */

#define APB_CTRL_SYSCLK_CONF_REG (DR_REG_APB_CTRL_BASE + 0x0)

/* APB_CTRL_QUICK_CLK_CHNG : RW; bitpos: [13]; default: 1; */

#define APB_CTRL_QUICK_CLK_CHNG    (BIT(13))
#define APB_CTRL_QUICK_CLK_CHNG_M  (APB_CTRL_QUICK_CLK_CHNG_V << APB_CTRL_QUICK_CLK_CHNG_S)
#define APB_CTRL_QUICK_CLK_CHNG_V  0x00000001
#define APB_CTRL_QUICK_CLK_CHNG_S  13

/* APB_CTRL_RST_TICK_CNT : RW; bitpos: [12]; default: 0; */

#define APB_CTRL_RST_TICK_CNT    (BIT(12))
#define APB_CTRL_RST_TICK_CNT_M  (APB_CTRL_RST_TICK_CNT_V << APB_CTRL_RST_TICK_CNT_S)
#define APB_CTRL_RST_TICK_CNT_V  0x00000001
#define APB_CTRL_RST_TICK_CNT_S  12

/* APB_CTRL_CLK_EN : RW; bitpos: [11]; default: 0; */

#define APB_CTRL_CLK_EN    (BIT(11))
#define APB_CTRL_CLK_EN_M  (APB_CTRL_CLK_EN_V << APB_CTRL_CLK_EN_S)
#define APB_CTRL_CLK_EN_V  0x00000001
#define APB_CTRL_CLK_EN_S  11

/* APB_CTRL_CLK_320M_EN : RW; bitpos: [10]; default: 0; */

#define APB_CTRL_CLK_320M_EN    (BIT(10))
#define APB_CTRL_CLK_320M_EN_M  (APB_CTRL_CLK_320M_EN_V << APB_CTRL_CLK_320M_EN_S)
#define APB_CTRL_CLK_320M_EN_V  0x00000001
#define APB_CTRL_CLK_320M_EN_S  10

/* APB_CTRL_PRE_DIV_CNT : RW; bitpos: [9:0]; default: 0; */

#define APB_CTRL_PRE_DIV_CNT    0x000003ff
#define APB_CTRL_PRE_DIV_CNT_M  (APB_CTRL_PRE_DIV_CNT_V << APB_CTRL_PRE_DIV_CNT_S)
#define APB_CTRL_PRE_DIV_CNT_V  0x000003ff
#define APB_CTRL_PRE_DIV_CNT_S  0

/* APB_CTRL_XTAL_TICK_CONF_REG register */

#define APB_CTRL_XTAL_TICK_CONF_REG (DR_REG_APB_CTRL_BASE + 0x4)

/* APB_CTRL_XTAL_TICK_NUM : RW; bitpos: [7:0]; default: 39; */

#define APB_CTRL_XTAL_TICK_NUM    0x000000ff
#define APB_CTRL_XTAL_TICK_NUM_M  (APB_CTRL_XTAL_TICK_NUM_V << APB_CTRL_XTAL_TICK_NUM_S)
#define APB_CTRL_XTAL_TICK_NUM_V  0x000000ff
#define APB_CTRL_XTAL_TICK_NUM_S  0

/* APB_CTRL_PLL_TICK_CONF_REG register */

#define APB_CTRL_PLL_TICK_CONF_REG (DR_REG_APB_CTRL_BASE + 0x8)

/* APB_CTRL_PLL_TICK_NUM : RW; bitpos: [7:0]; default: 79; */

#define APB_CTRL_PLL_TICK_NUM    0x000000ff
#define APB_CTRL_PLL_TICK_NUM_M  (APB_CTRL_PLL_TICK_NUM_V << APB_CTRL_PLL_TICK_NUM_S)
#define APB_CTRL_PLL_TICK_NUM_V  0x000000ff
#define APB_CTRL_PLL_TICK_NUM_S  0

/* APB_CTRL_CK8M_TICK_CONF_REG register */

#define APB_CTRL_CK8M_TICK_CONF_REG (DR_REG_APB_CTRL_BASE + 0xc)

/* APB_CTRL_CK8M_TICK_NUM : RW; bitpos: [7:0]; default: 11; */

#define APB_CTRL_CK8M_TICK_NUM    0x000000ff
#define APB_CTRL_CK8M_TICK_NUM_M  (APB_CTRL_CK8M_TICK_NUM_V << APB_CTRL_CK8M_TICK_NUM_S)
#define APB_CTRL_CK8M_TICK_NUM_V  0x000000ff
#define APB_CTRL_CK8M_TICK_NUM_S  0

/* APB_CTRL_APB_SARADC_CTRL_REG register */

#define APB_CTRL_APB_SARADC_CTRL_REG (DR_REG_APB_CTRL_BASE + 0x10)

/* APB_CTRL_SARADC_DATA_TO_I2S : RW; bitpos: [26]; default: 0;
 * 1: I2S input data is from SAR ADC (for DMA)  0: I2S input data is from
 * GPIO matrix
 */

#define APB_CTRL_SARADC_DATA_TO_I2S    (BIT(26))
#define APB_CTRL_SARADC_DATA_TO_I2S_M  (APB_CTRL_SARADC_DATA_TO_I2S_V << APB_CTRL_SARADC_DATA_TO_I2S_S)
#define APB_CTRL_SARADC_DATA_TO_I2S_V  0x00000001
#define APB_CTRL_SARADC_DATA_TO_I2S_S  26

/* APB_CTRL_SARADC_DATA_SAR_SEL : RW; bitpos: [25]; default: 0;
 * 1: sar_sel will be coded by the MSB of the 16-bit output data  in this
 * case the resolution should not be larger than 11 bits.
 */

#define APB_CTRL_SARADC_DATA_SAR_SEL    (BIT(25))
#define APB_CTRL_SARADC_DATA_SAR_SEL_M  (APB_CTRL_SARADC_DATA_SAR_SEL_V << APB_CTRL_SARADC_DATA_SAR_SEL_S)
#define APB_CTRL_SARADC_DATA_SAR_SEL_V  0x00000001
#define APB_CTRL_SARADC_DATA_SAR_SEL_S  25

/* APB_CTRL_SARADC_SAR2_PATT_P_CLEAR : RW; bitpos: [24]; default: 0;
 * clear the pointer of pattern table for DIG ADC2 CTRL
 */

#define APB_CTRL_SARADC_SAR2_PATT_P_CLEAR    (BIT(24))
#define APB_CTRL_SARADC_SAR2_PATT_P_CLEAR_M  (APB_CTRL_SARADC_SAR2_PATT_P_CLEAR_V << APB_CTRL_SARADC_SAR2_PATT_P_CLEAR_S)
#define APB_CTRL_SARADC_SAR2_PATT_P_CLEAR_V  0x00000001
#define APB_CTRL_SARADC_SAR2_PATT_P_CLEAR_S  24

/* APB_CTRL_SARADC_SAR1_PATT_P_CLEAR : RW; bitpos: [23]; default: 0;
 * clear the pointer of pattern table for DIG ADC1 CTRL
 */

#define APB_CTRL_SARADC_SAR1_PATT_P_CLEAR    (BIT(23))
#define APB_CTRL_SARADC_SAR1_PATT_P_CLEAR_M  (APB_CTRL_SARADC_SAR1_PATT_P_CLEAR_V << APB_CTRL_SARADC_SAR1_PATT_P_CLEAR_S)
#define APB_CTRL_SARADC_SAR1_PATT_P_CLEAR_V  0x00000001
#define APB_CTRL_SARADC_SAR1_PATT_P_CLEAR_S  23

/* APB_CTRL_SARADC_SAR2_PATT_LEN : RW; bitpos: [22:19]; default: 15;
 * 0 ~ 15 means length 1 ~ 16
 */

#define APB_CTRL_SARADC_SAR2_PATT_LEN    0x0000000f
#define APB_CTRL_SARADC_SAR2_PATT_LEN_M  (APB_CTRL_SARADC_SAR2_PATT_LEN_V << APB_CTRL_SARADC_SAR2_PATT_LEN_S)
#define APB_CTRL_SARADC_SAR2_PATT_LEN_V  0x0000000f
#define APB_CTRL_SARADC_SAR2_PATT_LEN_S  19

/* APB_CTRL_SARADC_SAR1_PATT_LEN : RW; bitpos: [18:15]; default: 15;
 * 0 ~ 15 means length 1 ~ 16
 */

#define APB_CTRL_SARADC_SAR1_PATT_LEN    0x0000000f
#define APB_CTRL_SARADC_SAR1_PATT_LEN_M  (APB_CTRL_SARADC_SAR1_PATT_LEN_V << APB_CTRL_SARADC_SAR1_PATT_LEN_S)
#define APB_CTRL_SARADC_SAR1_PATT_LEN_V  0x0000000f
#define APB_CTRL_SARADC_SAR1_PATT_LEN_S  15

/* APB_CTRL_SARADC_SAR_CLK_DIV : RW; bitpos: [14:7]; default: 4;
 * SAR clock divider
 */

#define APB_CTRL_SARADC_SAR_CLK_DIV    0x000000ff
#define APB_CTRL_SARADC_SAR_CLK_DIV_M  (APB_CTRL_SARADC_SAR_CLK_DIV_V << APB_CTRL_SARADC_SAR_CLK_DIV_S)
#define APB_CTRL_SARADC_SAR_CLK_DIV_V  0x000000ff
#define APB_CTRL_SARADC_SAR_CLK_DIV_S  7

/* APB_CTRL_SARADC_SAR_CLK_GATED : RW; bitpos: [6]; default: 1; */

#define APB_CTRL_SARADC_SAR_CLK_GATED    (BIT(6))
#define APB_CTRL_SARADC_SAR_CLK_GATED_M  (APB_CTRL_SARADC_SAR_CLK_GATED_V << APB_CTRL_SARADC_SAR_CLK_GATED_S)
#define APB_CTRL_SARADC_SAR_CLK_GATED_V  0x00000001
#define APB_CTRL_SARADC_SAR_CLK_GATED_S  6

/* APB_CTRL_SARADC_SAR_SEL : RW; bitpos: [5]; default: 0;
 * 0: SAR1  1: SAR2  only work for single SAR mode
 */

#define APB_CTRL_SARADC_SAR_SEL    (BIT(5))
#define APB_CTRL_SARADC_SAR_SEL_M  (APB_CTRL_SARADC_SAR_SEL_V << APB_CTRL_SARADC_SAR_SEL_S)
#define APB_CTRL_SARADC_SAR_SEL_V  0x00000001
#define APB_CTRL_SARADC_SAR_SEL_S  5

/* APB_CTRL_SARADC_WORK_MODE : RW; bitpos: [4:3]; default: 0;
 * 0: single mode  1: double mode  2: alternate mode
 */

#define APB_CTRL_SARADC_WORK_MODE    0x00000003
#define APB_CTRL_SARADC_WORK_MODE_M  (APB_CTRL_SARADC_WORK_MODE_V << APB_CTRL_SARADC_WORK_MODE_S)
#define APB_CTRL_SARADC_WORK_MODE_V  0x00000003
#define APB_CTRL_SARADC_WORK_MODE_S  3

/* APB_CTRL_SARADC_SAR2_MUX : RW; bitpos: [2]; default: 0;
 * 1: SAR ADC2 is controlled by DIG ADC2 CTRL  0: SAR ADC2 is controlled by
 * PWDET CTRL
 */

#define APB_CTRL_SARADC_SAR2_MUX    (BIT(2))
#define APB_CTRL_SARADC_SAR2_MUX_M  (APB_CTRL_SARADC_SAR2_MUX_V << APB_CTRL_SARADC_SAR2_MUX_S)
#define APB_CTRL_SARADC_SAR2_MUX_V  0x00000001
#define APB_CTRL_SARADC_SAR2_MUX_S  2

/* APB_CTRL_SARADC_START : RW; bitpos: [1]; default: 0; */

#define APB_CTRL_SARADC_START    (BIT(1))
#define APB_CTRL_SARADC_START_M  (APB_CTRL_SARADC_START_V << APB_CTRL_SARADC_START_S)
#define APB_CTRL_SARADC_START_V  0x00000001
#define APB_CTRL_SARADC_START_S  1

/* APB_CTRL_SARADC_START_FORCE : RW; bitpos: [0]; default: 0; */

#define APB_CTRL_SARADC_START_FORCE    (BIT(0))
#define APB_CTRL_SARADC_START_FORCE_M  (APB_CTRL_SARADC_START_FORCE_V << APB_CTRL_SARADC_START_FORCE_S)
#define APB_CTRL_SARADC_START_FORCE_V  0x00000001
#define APB_CTRL_SARADC_START_FORCE_S  0

/* APB_CTRL_APB_SARADC_CTRL2_REG register */

#define APB_CTRL_APB_SARADC_CTRL2_REG (DR_REG_APB_CTRL_BASE + 0x14)

/* APB_CTRL_SARADC_SAR2_INV : RW; bitpos: [10]; default: 0;
 * 1: data to DIG ADC2 CTRL is inverted  otherwise not
 */

#define APB_CTRL_SARADC_SAR2_INV    (BIT(10))
#define APB_CTRL_SARADC_SAR2_INV_M  (APB_CTRL_SARADC_SAR2_INV_V << APB_CTRL_SARADC_SAR2_INV_S)
#define APB_CTRL_SARADC_SAR2_INV_V  0x00000001
#define APB_CTRL_SARADC_SAR2_INV_S  10

/* APB_CTRL_SARADC_SAR1_INV : RW; bitpos: [9]; default: 0;
 * 1: data to DIG ADC1 CTRL is inverted  otherwise not
 */

#define APB_CTRL_SARADC_SAR1_INV    (BIT(9))
#define APB_CTRL_SARADC_SAR1_INV_M  (APB_CTRL_SARADC_SAR1_INV_V << APB_CTRL_SARADC_SAR1_INV_S)
#define APB_CTRL_SARADC_SAR1_INV_V  0x00000001
#define APB_CTRL_SARADC_SAR1_INV_S  9

/* APB_CTRL_SARADC_MAX_MEAS_NUM : RW; bitpos: [8:1]; default: 255;
 * max conversion number
 */

#define APB_CTRL_SARADC_MAX_MEAS_NUM    0x000000ff
#define APB_CTRL_SARADC_MAX_MEAS_NUM_M  (APB_CTRL_SARADC_MAX_MEAS_NUM_V << APB_CTRL_SARADC_MAX_MEAS_NUM_S)
#define APB_CTRL_SARADC_MAX_MEAS_NUM_V  0x000000ff
#define APB_CTRL_SARADC_MAX_MEAS_NUM_S  1

/* APB_CTRL_SARADC_MEAS_NUM_LIMIT : RW; bitpos: [0]; default: 0; */

#define APB_CTRL_SARADC_MEAS_NUM_LIMIT    (BIT(0))
#define APB_CTRL_SARADC_MEAS_NUM_LIMIT_M  (APB_CTRL_SARADC_MEAS_NUM_LIMIT_V << APB_CTRL_SARADC_MEAS_NUM_LIMIT_S)
#define APB_CTRL_SARADC_MEAS_NUM_LIMIT_V  0x00000001
#define APB_CTRL_SARADC_MEAS_NUM_LIMIT_S  0

/* APB_CTRL_APB_SARADC_FSM_REG register */

#define APB_CTRL_APB_SARADC_FSM_REG (DR_REG_APB_CTRL_BASE + 0x18)

/* APB_CTRL_SARADC_SAMPLE_CYCLE : RW; bitpos: [31:24]; default: 2;
 * sample cycles
 */

#define APB_CTRL_SARADC_SAMPLE_CYCLE    0x000000ff
#define APB_CTRL_SARADC_SAMPLE_CYCLE_M  (APB_CTRL_SARADC_SAMPLE_CYCLE_V << APB_CTRL_SARADC_SAMPLE_CYCLE_S)
#define APB_CTRL_SARADC_SAMPLE_CYCLE_V  0x000000ff
#define APB_CTRL_SARADC_SAMPLE_CYCLE_S  24

/* APB_CTRL_SARADC_START_WAIT : RW; bitpos: [23:16]; default: 8; */

#define APB_CTRL_SARADC_START_WAIT    0x000000ff
#define APB_CTRL_SARADC_START_WAIT_M  (APB_CTRL_SARADC_START_WAIT_V << APB_CTRL_SARADC_START_WAIT_S)
#define APB_CTRL_SARADC_START_WAIT_V  0x000000ff
#define APB_CTRL_SARADC_START_WAIT_S  16

/* APB_CTRL_SARADC_STANDBY_WAIT : RW; bitpos: [15:8]; default: 255; */

#define APB_CTRL_SARADC_STANDBY_WAIT    0x000000ff
#define APB_CTRL_SARADC_STANDBY_WAIT_M  (APB_CTRL_SARADC_STANDBY_WAIT_V << APB_CTRL_SARADC_STANDBY_WAIT_S)
#define APB_CTRL_SARADC_STANDBY_WAIT_V  0x000000ff
#define APB_CTRL_SARADC_STANDBY_WAIT_S  8

/* APB_CTRL_SARADC_RSTB_WAIT : RW; bitpos: [7:0]; default: 8; */

#define APB_CTRL_SARADC_RSTB_WAIT    0x000000ff
#define APB_CTRL_SARADC_RSTB_WAIT_M  (APB_CTRL_SARADC_RSTB_WAIT_V << APB_CTRL_SARADC_RSTB_WAIT_S)
#define APB_CTRL_SARADC_RSTB_WAIT_V  0x000000ff
#define APB_CTRL_SARADC_RSTB_WAIT_S  0

/* APB_CTRL_APB_SARADC_SAR1_PATT_TAB1_REG register */

#define APB_CTRL_APB_SARADC_SAR1_PATT_TAB1_REG (DR_REG_APB_CTRL_BASE + 0x1c)

/* APB_CTRL_SARADC_SAR1_PATT_TAB1 : RW; bitpos: [31:0]; default: 252645135;
 * item 0 ~ 3 for pattern table 1 (each item one byte)
 */

#define APB_CTRL_SARADC_SAR1_PATT_TAB1    0xffffffff
#define APB_CTRL_SARADC_SAR1_PATT_TAB1_M  (APB_CTRL_SARADC_SAR1_PATT_TAB1_V << APB_CTRL_SARADC_SAR1_PATT_TAB1_S)
#define APB_CTRL_SARADC_SAR1_PATT_TAB1_V  0xffffffff
#define APB_CTRL_SARADC_SAR1_PATT_TAB1_S  0

/* APB_CTRL_APB_SARADC_SAR1_PATT_TAB2_REG register */

#define APB_CTRL_APB_SARADC_SAR1_PATT_TAB2_REG (DR_REG_APB_CTRL_BASE + 0x20)

/* APB_CTRL_SARADC_SAR1_PATT_TAB2 : RW; bitpos: [31:0]; default: 252645135;
 * Item 4 ~ 7 for pattern table 1 (each item one byte)
 */

#define APB_CTRL_SARADC_SAR1_PATT_TAB2    0xffffffff
#define APB_CTRL_SARADC_SAR1_PATT_TAB2_M  (APB_CTRL_SARADC_SAR1_PATT_TAB2_V << APB_CTRL_SARADC_SAR1_PATT_TAB2_S)
#define APB_CTRL_SARADC_SAR1_PATT_TAB2_V  0xffffffff
#define APB_CTRL_SARADC_SAR1_PATT_TAB2_S  0

/* APB_CTRL_APB_SARADC_SAR1_PATT_TAB3_REG register */

#define APB_CTRL_APB_SARADC_SAR1_PATT_TAB3_REG (DR_REG_APB_CTRL_BASE + 0x24)

/* APB_CTRL_SARADC_SAR1_PATT_TAB3 : RW; bitpos: [31:0]; default: 252645135;
 * Item 8 ~ 11 for pattern table 1 (each item one byte)
 */

#define APB_CTRL_SARADC_SAR1_PATT_TAB3    0xffffffff
#define APB_CTRL_SARADC_SAR1_PATT_TAB3_M  (APB_CTRL_SARADC_SAR1_PATT_TAB3_V << APB_CTRL_SARADC_SAR1_PATT_TAB3_S)
#define APB_CTRL_SARADC_SAR1_PATT_TAB3_V  0xffffffff
#define APB_CTRL_SARADC_SAR1_PATT_TAB3_S  0

/* APB_CTRL_APB_SARADC_SAR1_PATT_TAB4_REG register */

#define APB_CTRL_APB_SARADC_SAR1_PATT_TAB4_REG (DR_REG_APB_CTRL_BASE + 0x28)

/* APB_CTRL_SARADC_SAR1_PATT_TAB4 : RW; bitpos: [31:0]; default: 252645135;
 * Item 12 ~ 15 for pattern table 1 (each item one byte)
 */

#define APB_CTRL_SARADC_SAR1_PATT_TAB4    0xffffffff
#define APB_CTRL_SARADC_SAR1_PATT_TAB4_M  (APB_CTRL_SARADC_SAR1_PATT_TAB4_V << APB_CTRL_SARADC_SAR1_PATT_TAB4_S)
#define APB_CTRL_SARADC_SAR1_PATT_TAB4_V  0xffffffff
#define APB_CTRL_SARADC_SAR1_PATT_TAB4_S  0

/* APB_CTRL_APB_SARADC_SAR2_PATT_TAB1_REG register */

#define APB_CTRL_APB_SARADC_SAR2_PATT_TAB1_REG (DR_REG_APB_CTRL_BASE + 0x2c)

/* APB_CTRL_SARADC_SAR2_PATT_TAB1 : RW; bitpos: [31:0]; default: 252645135;
 * item 0 ~ 3 for pattern table 2 (each item one byte)
 */

#define APB_CTRL_SARADC_SAR2_PATT_TAB1    0xffffffff
#define APB_CTRL_SARADC_SAR2_PATT_TAB1_M  (APB_CTRL_SARADC_SAR2_PATT_TAB1_V << APB_CTRL_SARADC_SAR2_PATT_TAB1_S)
#define APB_CTRL_SARADC_SAR2_PATT_TAB1_V  0xffffffff
#define APB_CTRL_SARADC_SAR2_PATT_TAB1_S  0

/* APB_CTRL_APB_SARADC_SAR2_PATT_TAB2_REG register */

#define APB_CTRL_APB_SARADC_SAR2_PATT_TAB2_REG (DR_REG_APB_CTRL_BASE + 0x30)

/* APB_CTRL_SARADC_SAR2_PATT_TAB2 : RW; bitpos: [31:0]; default: 252645135;
 * Item 4 ~ 7 for pattern table 2 (each item one byte)
 */

#define APB_CTRL_SARADC_SAR2_PATT_TAB2    0xffffffff
#define APB_CTRL_SARADC_SAR2_PATT_TAB2_M  (APB_CTRL_SARADC_SAR2_PATT_TAB2_V << APB_CTRL_SARADC_SAR2_PATT_TAB2_S)
#define APB_CTRL_SARADC_SAR2_PATT_TAB2_V  0xffffffff
#define APB_CTRL_SARADC_SAR2_PATT_TAB2_S  0

/* APB_CTRL_APB_SARADC_SAR2_PATT_TAB3_REG register */

#define APB_CTRL_APB_SARADC_SAR2_PATT_TAB3_REG (DR_REG_APB_CTRL_BASE + 0x34)

/* APB_CTRL_SARADC_SAR2_PATT_TAB3 : RW; bitpos: [31:0]; default: 252645135;
 * Item 8 ~ 11 for pattern table 2 (each item one byte)
 */

#define APB_CTRL_SARADC_SAR2_PATT_TAB3    0xffffffff
#define APB_CTRL_SARADC_SAR2_PATT_TAB3_M  (APB_CTRL_SARADC_SAR2_PATT_TAB3_V << APB_CTRL_SARADC_SAR2_PATT_TAB3_S)
#define APB_CTRL_SARADC_SAR2_PATT_TAB3_V  0xffffffff
#define APB_CTRL_SARADC_SAR2_PATT_TAB3_S  0

/* APB_CTRL_APB_SARADC_SAR2_PATT_TAB4_REG register */

#define APB_CTRL_APB_SARADC_SAR2_PATT_TAB4_REG (DR_REG_APB_CTRL_BASE + 0x38)

/* APB_CTRL_SARADC_SAR2_PATT_TAB4 : RW; bitpos: [31:0]; default: 252645135;
 * Item 12 ~ 15 for pattern table 2 (each item one byte)
 */

#define APB_CTRL_SARADC_SAR2_PATT_TAB4    0xffffffff
#define APB_CTRL_SARADC_SAR2_PATT_TAB4_M  (APB_CTRL_SARADC_SAR2_PATT_TAB4_V << APB_CTRL_SARADC_SAR2_PATT_TAB4_S)
#define APB_CTRL_SARADC_SAR2_PATT_TAB4_V  0xffffffff
#define APB_CTRL_SARADC_SAR2_PATT_TAB4_S  0

/* APB_CTRL_APLL_TICK_CONF_REG register */

#define APB_CTRL_APLL_TICK_CONF_REG (DR_REG_APB_CTRL_BASE + 0x3c)

/* APB_CTRL_APLL_TICK_NUM : RW; bitpos: [7:0]; default: 99; */

#define APB_CTRL_APLL_TICK_NUM    0x000000ff
#define APB_CTRL_APLL_TICK_NUM_M  (APB_CTRL_APLL_TICK_NUM_V << APB_CTRL_APLL_TICK_NUM_S)
#define APB_CTRL_APLL_TICK_NUM_V  0x000000ff
#define APB_CTRL_APLL_TICK_NUM_S  0

/* APB_CTRL_DATE_REG register */

#define APB_CTRL_DATE_REG (DR_REG_APB_CTRL_BASE + 0x7c)

/* APB_CTRL_DATE : RW; bitpos: [31:0]; default: 369369088; */

#define APB_CTRL_DATE    0xffffffff
#define APB_CTRL_DATE_M  (APB_CTRL_DATE_V << APB_CTRL_DATE_S)
#define APB_CTRL_DATE_V  0xffffffff
#define APB_CTRL_DATE_S  0

#endif /* __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_APB_CTRL_H */
