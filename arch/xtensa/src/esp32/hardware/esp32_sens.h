/****************************************************************************
 * arch/xtensa/src/esp32/hardware/esp32_sens.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_SENS_H
#define __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_SENS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SENS_SAR_READ_CTRL_REG register */

#define SENS_SAR_READ_CTRL_REG (DR_REG_SENS_BASE + 0x0)

/* SENS_SAR1_DATA_INV : RW; bitpos: [28]; default: 0;
 * Invert SAR ADC1 data
 */

#define SENS_SAR1_DATA_INV    (BIT(28))
#define SENS_SAR1_DATA_INV_M  (SENS_SAR1_DATA_INV_V << SENS_SAR1_DATA_INV_S)
#define SENS_SAR1_DATA_INV_V  0x00000001
#define SENS_SAR1_DATA_INV_S  28

/* SENS_SAR1_DIG_FORCE : RW; bitpos: [27]; default: 0;
 * 1: SAR ADC1 controlled by DIG ADC1 CTRL  0: SAR ADC1 controlled by RTC
 * ADC1 CTRL
 */

#define SENS_SAR1_DIG_FORCE    (BIT(27))
#define SENS_SAR1_DIG_FORCE_M  (SENS_SAR1_DIG_FORCE_V << SENS_SAR1_DIG_FORCE_S)
#define SENS_SAR1_DIG_FORCE_V  0x00000001
#define SENS_SAR1_DIG_FORCE_S  27

/* SENS_SAR1_SAMPLE_NUM : RW; bitpos: [26:19]; default: 0; */

#define SENS_SAR1_SAMPLE_NUM    0x000000ff
#define SENS_SAR1_SAMPLE_NUM_M  (SENS_SAR1_SAMPLE_NUM_V << SENS_SAR1_SAMPLE_NUM_S)
#define SENS_SAR1_SAMPLE_NUM_V  0x000000ff
#define SENS_SAR1_SAMPLE_NUM_S  19

/* SENS_SAR1_CLK_GATED : RW; bitpos: [18]; default: 1; */

#define SENS_SAR1_CLK_GATED    (BIT(18))
#define SENS_SAR1_CLK_GATED_M  (SENS_SAR1_CLK_GATED_V << SENS_SAR1_CLK_GATED_S)
#define SENS_SAR1_CLK_GATED_V  0x00000001
#define SENS_SAR1_CLK_GATED_S  18

/* SENS_SAR1_SAMPLE_BIT : RW; bitpos: [17:16]; default: 3;
 * 00: for 9-bit width  01: for 10-bit width  10: for 11-bit width  11: for
 * 12-bit width
 */

#define SENS_SAR1_SAMPLE_BIT    0x00000003
#define SENS_SAR1_SAMPLE_BIT_M  (SENS_SAR1_SAMPLE_BIT_V << SENS_SAR1_SAMPLE_BIT_S)
#define SENS_SAR1_SAMPLE_BIT_V  0x00000003
#define SENS_SAR1_SAMPLE_BIT_S  16

/* SENS_SAR1_SAMPLE_CYCLE : RW; bitpos: [15:8]; default: 9;
 * sample cycles for SAR ADC1
 */

#define SENS_SAR1_SAMPLE_CYCLE    0x000000ff
#define SENS_SAR1_SAMPLE_CYCLE_M  (SENS_SAR1_SAMPLE_CYCLE_V << SENS_SAR1_SAMPLE_CYCLE_S)
#define SENS_SAR1_SAMPLE_CYCLE_V  0x000000ff
#define SENS_SAR1_SAMPLE_CYCLE_S  8

/* SENS_SAR1_CLK_DIV : RW; bitpos: [7:0]; default: 2;
 * clock divider
 */

#define SENS_SAR1_CLK_DIV    0x000000ff
#define SENS_SAR1_CLK_DIV_M  (SENS_SAR1_CLK_DIV_V << SENS_SAR1_CLK_DIV_S)
#define SENS_SAR1_CLK_DIV_V  0x000000ff
#define SENS_SAR1_CLK_DIV_S  0

/* SENS_SAR_READ_STATUS1_REG register */

#define SENS_SAR_READ_STATUS1_REG (DR_REG_SENS_BASE + 0x4)

/* SENS_SAR1_READER_STATUS : R; bitpos: [31:0]; default: 0; */

#define SENS_SAR1_READER_STATUS    0xffffffff
#define SENS_SAR1_READER_STATUS_M  (SENS_SAR1_READER_STATUS_V << SENS_SAR1_READER_STATUS_S)
#define SENS_SAR1_READER_STATUS_V  0xffffffff
#define SENS_SAR1_READER_STATUS_S  0

/* SENS_SAR_MEAS_WAIT1_REG register */

#define SENS_SAR_MEAS_WAIT1_REG (DR_REG_SENS_BASE + 0x8)

/* SENS_SAR_AMP_WAIT2 : RW; bitpos: [31:16]; default: 10; */

#define SENS_SAR_AMP_WAIT2    0x0000ffff
#define SENS_SAR_AMP_WAIT2_M  (SENS_SAR_AMP_WAIT2_V << SENS_SAR_AMP_WAIT2_S)
#define SENS_SAR_AMP_WAIT2_V  0x0000ffff
#define SENS_SAR_AMP_WAIT2_S  16

/* SENS_SAR_AMP_WAIT1 : RW; bitpos: [15:0]; default: 10; */

#define SENS_SAR_AMP_WAIT1    0x0000ffff
#define SENS_SAR_AMP_WAIT1_M  (SENS_SAR_AMP_WAIT1_V << SENS_SAR_AMP_WAIT1_S)
#define SENS_SAR_AMP_WAIT1_V  0x0000ffff
#define SENS_SAR_AMP_WAIT1_S  0

/* SENS_SAR_MEAS_WAIT2_REG register */

#define SENS_SAR_MEAS_WAIT2_REG (DR_REG_SENS_BASE + 0xc)

/* SENS_SAR2_RSTB_WAIT : RW; bitpos: [27:20]; default: 2; */

#define SENS_SAR2_RSTB_WAIT    0x000000ff
#define SENS_SAR2_RSTB_WAIT_M  (SENS_SAR2_RSTB_WAIT_V << SENS_SAR2_RSTB_WAIT_S)
#define SENS_SAR2_RSTB_WAIT_V  0x000000ff
#define SENS_SAR2_RSTB_WAIT_S  20

/* SENS_FORCE_XPD_SAR : RW; bitpos: [19:18]; default: 0; */

#define SENS_FORCE_XPD_SAR    0x00000003
#define SENS_FORCE_XPD_SAR_M  (SENS_FORCE_XPD_SAR_V << SENS_FORCE_XPD_SAR_S)
#define SENS_FORCE_XPD_SAR_V  0x00000003
#define SENS_FORCE_XPD_SAR_S  18

/* SENS_FORCE_XPD_AMP : RW; bitpos: [17:16]; default: 0; */

#define SENS_FORCE_XPD_AMP    0x00000003
#define SENS_FORCE_XPD_AMP_M  (SENS_FORCE_XPD_AMP_V << SENS_FORCE_XPD_AMP_S)
#define SENS_FORCE_XPD_AMP_V  0x00000003
#define SENS_FORCE_XPD_AMP_S  16

/* SENS_SAR_AMP_WAIT3 : RW; bitpos: [15:0]; default: 10; */

#define SENS_SAR_AMP_WAIT3    0x0000ffff
#define SENS_SAR_AMP_WAIT3_M  (SENS_SAR_AMP_WAIT3_V << SENS_SAR_AMP_WAIT3_S)
#define SENS_SAR_AMP_WAIT3_V  0x0000ffff
#define SENS_SAR_AMP_WAIT3_S  0

/* SENS_FORCE_XPD_SAR_SW : RW; bitpos: [-1:0]; default: 0; */

#define SENS_FORCE_XPD_SAR_SW    0x00000000
#define SENS_FORCE_XPD_SAR_SW_M  (SENS_FORCE_XPD_SAR_SW_V << SENS_FORCE_XPD_SAR_SW_S)
#define SENS_FORCE_XPD_SAR_SW_V  0x00000000
#define SENS_FORCE_XPD_SAR_SW_S  0

/* SENS_SAR_MEAS_CTRL_REG register */

#define SENS_SAR_MEAS_CTRL_REG (DR_REG_SENS_BASE + 0x10)

/* SENS_SAR2_XPD_WAIT : RW; bitpos: [31:24]; default: 7; */

#define SENS_SAR2_XPD_WAIT    0x000000ff
#define SENS_SAR2_XPD_WAIT_M  (SENS_SAR2_XPD_WAIT_V << SENS_SAR2_XPD_WAIT_S)
#define SENS_SAR2_XPD_WAIT_V  0x000000ff
#define SENS_SAR2_XPD_WAIT_S  24

/* SENS_SAR_RSTB_FSM : RW; bitpos: [23:20]; default: 0; */

#define SENS_SAR_RSTB_FSM    0x0000000f
#define SENS_SAR_RSTB_FSM_M  (SENS_SAR_RSTB_FSM_V << SENS_SAR_RSTB_FSM_S)
#define SENS_SAR_RSTB_FSM_V  0x0000000f
#define SENS_SAR_RSTB_FSM_S  20

/* SENS_XPD_SAR_FSM : RW; bitpos: [19:16]; default: 7; */

#define SENS_XPD_SAR_FSM    0x0000000f
#define SENS_XPD_SAR_FSM_M  (SENS_XPD_SAR_FSM_V << SENS_XPD_SAR_FSM_S)
#define SENS_XPD_SAR_FSM_V  0x0000000f
#define SENS_XPD_SAR_FSM_S  16

/* SENS_AMP_SHORT_REF_GND_FSM : RW; bitpos: [15:12]; default: 3; */

#define SENS_AMP_SHORT_REF_GND_FSM    0x0000000f
#define SENS_AMP_SHORT_REF_GND_FSM_M  (SENS_AMP_SHORT_REF_GND_FSM_V << SENS_AMP_SHORT_REF_GND_FSM_S)
#define SENS_AMP_SHORT_REF_GND_FSM_V  0x0000000f
#define SENS_AMP_SHORT_REF_GND_FSM_S  12

/* SENS_AMP_SHORT_REF_FSM : RW; bitpos: [11:8]; default: 3; */

#define SENS_AMP_SHORT_REF_FSM    0x0000000f
#define SENS_AMP_SHORT_REF_FSM_M  (SENS_AMP_SHORT_REF_FSM_V << SENS_AMP_SHORT_REF_FSM_S)
#define SENS_AMP_SHORT_REF_FSM_V  0x0000000f
#define SENS_AMP_SHORT_REF_FSM_S  8

/* SENS_AMP_RST_FB_FSM : RW; bitpos: [7:4]; default: 8; */

#define SENS_AMP_RST_FB_FSM    0x0000000f
#define SENS_AMP_RST_FB_FSM_M  (SENS_AMP_RST_FB_FSM_V << SENS_AMP_RST_FB_FSM_S)
#define SENS_AMP_RST_FB_FSM_V  0x0000000f
#define SENS_AMP_RST_FB_FSM_S  4

/* SENS_XPD_SAR_AMP_FSM : RW; bitpos: [3:0]; default: 15; */

#define SENS_XPD_SAR_AMP_FSM    0x0000000f
#define SENS_XPD_SAR_AMP_FSM_M  (SENS_XPD_SAR_AMP_FSM_V << SENS_XPD_SAR_AMP_FSM_S)
#define SENS_XPD_SAR_AMP_FSM_V  0x0000000f
#define SENS_XPD_SAR_AMP_FSM_S  0

/* SENS_SAR_READ_STATUS2_REG register */

#define SENS_SAR_READ_STATUS2_REG (DR_REG_SENS_BASE + 0x14)

/* SENS_SAR2_READER_STATUS : R; bitpos: [31:0]; default: 0; */

#define SENS_SAR2_READER_STATUS    0xffffffff
#define SENS_SAR2_READER_STATUS_M  (SENS_SAR2_READER_STATUS_V << SENS_SAR2_READER_STATUS_S)
#define SENS_SAR2_READER_STATUS_V  0xffffffff
#define SENS_SAR2_READER_STATUS_S  0

/* SENS_ULP_CP_SLEEP_CYC0_REG register */

#define SENS_ULP_CP_SLEEP_CYC0_REG (DR_REG_SENS_BASE + 0x18)

/* SENS_SLEEP_CYCLES_S0 : RW; bitpos: [31:0]; default: 200;
 * sleep cycles for ULP-coprocessor timer
 */

#define SENS_SLEEP_CYCLES_S0    0xffffffff
#define SENS_SLEEP_CYCLES_S0_M  (SENS_SLEEP_CYCLES_S0_V << SENS_SLEEP_CYCLES_S0_S)
#define SENS_SLEEP_CYCLES_S0_V  0xffffffff
#define SENS_SLEEP_CYCLES_S0_S  0

/* SENS_ULP_CP_SLEEP_CYC1_REG register */

#define SENS_ULP_CP_SLEEP_CYC1_REG (DR_REG_SENS_BASE + 0x1c)

/* SENS_SLEEP_CYCLES_S1 : RW; bitpos: [31:0]; default: 100; */

#define SENS_SLEEP_CYCLES_S1    0xffffffff
#define SENS_SLEEP_CYCLES_S1_M  (SENS_SLEEP_CYCLES_S1_V << SENS_SLEEP_CYCLES_S1_S)
#define SENS_SLEEP_CYCLES_S1_V  0xffffffff
#define SENS_SLEEP_CYCLES_S1_S  0

/* SENS_ULP_CP_SLEEP_CYC2_REG register */

#define SENS_ULP_CP_SLEEP_CYC2_REG (DR_REG_SENS_BASE + 0x20)

/* SENS_SLEEP_CYCLES_S2 : RW; bitpos: [31:0]; default: 50; */

#define SENS_SLEEP_CYCLES_S2    0xffffffff
#define SENS_SLEEP_CYCLES_S2_M  (SENS_SLEEP_CYCLES_S2_V << SENS_SLEEP_CYCLES_S2_S)
#define SENS_SLEEP_CYCLES_S2_V  0xffffffff
#define SENS_SLEEP_CYCLES_S2_S  0

/* SENS_ULP_CP_SLEEP_CYC3_REG register */

#define SENS_ULP_CP_SLEEP_CYC3_REG (DR_REG_SENS_BASE + 0x24)

/* SENS_SLEEP_CYCLES_S3 : RW; bitpos: [31:0]; default: 40; */

#define SENS_SLEEP_CYCLES_S3    0xffffffff
#define SENS_SLEEP_CYCLES_S3_M  (SENS_SLEEP_CYCLES_S3_V << SENS_SLEEP_CYCLES_S3_S)
#define SENS_SLEEP_CYCLES_S3_V  0xffffffff
#define SENS_SLEEP_CYCLES_S3_S  0

/* SENS_ULP_CP_SLEEP_CYC4_REG register */

#define SENS_ULP_CP_SLEEP_CYC4_REG (DR_REG_SENS_BASE + 0x28)

/* SENS_SLEEP_CYCLES_S4 : RW; bitpos: [31:0]; default: 20; */

#define SENS_SLEEP_CYCLES_S4    0xffffffff
#define SENS_SLEEP_CYCLES_S4_M  (SENS_SLEEP_CYCLES_S4_V << SENS_SLEEP_CYCLES_S4_S)
#define SENS_SLEEP_CYCLES_S4_V  0xffffffff
#define SENS_SLEEP_CYCLES_S4_S  0

/* SENS_SAR_START_FORCE_REG register */

#define SENS_SAR_START_FORCE_REG (DR_REG_SENS_BASE + 0x2c)

/* SENS_SAR2_PWDET_EN : RW; bitpos: [24]; default: 0;
 * N/A
 */

#define SENS_SAR2_PWDET_EN    (BIT(24))
#define SENS_SAR2_PWDET_EN_M  (SENS_SAR2_PWDET_EN_V << SENS_SAR2_PWDET_EN_S)
#define SENS_SAR2_PWDET_EN_V  0x00000001
#define SENS_SAR2_PWDET_EN_S  24

/* SENS_SAR1_STOP : RW; bitpos: [23]; default: 0;
 * stop SAR ADC1 conversion
 */

#define SENS_SAR1_STOP    (BIT(23))
#define SENS_SAR1_STOP_M  (SENS_SAR1_STOP_V << SENS_SAR1_STOP_S)
#define SENS_SAR1_STOP_V  0x00000001
#define SENS_SAR1_STOP_S  23

/* SENS_SAR2_STOP : RW; bitpos: [22]; default: 0;
 * stop SAR ADC2 conversion
 */

#define SENS_SAR2_STOP    (BIT(22))
#define SENS_SAR2_STOP_M  (SENS_SAR2_STOP_V << SENS_SAR2_STOP_S)
#define SENS_SAR2_STOP_V  0x00000001
#define SENS_SAR2_STOP_S  22

/* SENS_PC_INIT : RW; bitpos: [21:11]; default: 0;
 * initialized PC for ULP-coprocessor
 */

#define SENS_PC_INIT    0x000007ff
#define SENS_PC_INIT_M  (SENS_PC_INIT_V << SENS_PC_INIT_S)
#define SENS_PC_INIT_V  0x000007ff
#define SENS_PC_INIT_S  11

/* SENS_SARCLK_EN : RW; bitpos: [10]; default: 0; */

#define SENS_SARCLK_EN    (BIT(10))
#define SENS_SARCLK_EN_M  (SENS_SARCLK_EN_V << SENS_SARCLK_EN_S)
#define SENS_SARCLK_EN_V  0x00000001
#define SENS_SARCLK_EN_S  10

/* SENS_ULP_CP_START_TOP : RW; bitpos: [9]; default: 0;
 * Write 1 to start ULP-coprocessor  only active when
 * reg_ulp_cp_force_start_top = 1
 */

#define SENS_ULP_CP_START_TOP    (BIT(9))
#define SENS_ULP_CP_START_TOP_M  (SENS_ULP_CP_START_TOP_V << SENS_ULP_CP_START_TOP_S)
#define SENS_ULP_CP_START_TOP_V  0x00000001
#define SENS_ULP_CP_START_TOP_S  9

/* SENS_ULP_CP_FORCE_START_TOP : RW; bitpos: [8]; default: 0;
 * 1: ULP-coprocessor is started by SW  0: ULP-coprocessor is started by
 * timer
 */

#define SENS_ULP_CP_FORCE_START_TOP    (BIT(8))
#define SENS_ULP_CP_FORCE_START_TOP_M  (SENS_ULP_CP_FORCE_START_TOP_V << SENS_ULP_CP_FORCE_START_TOP_S)
#define SENS_ULP_CP_FORCE_START_TOP_V  0x00000001
#define SENS_ULP_CP_FORCE_START_TOP_S  8

/* SENS_SAR2_PWDET_CCT : RW; bitpos: [7:5]; default: 0;
 * SAR2_PWDET_CCT  PA power detector capacitance tuning.
 */

#define SENS_SAR2_PWDET_CCT    0x00000007
#define SENS_SAR2_PWDET_CCT_M  (SENS_SAR2_PWDET_CCT_V << SENS_SAR2_PWDET_CCT_S)
#define SENS_SAR2_PWDET_CCT_V  0x00000007
#define SENS_SAR2_PWDET_CCT_S  5

/* SENS_SAR2_EN_TEST : RW; bitpos: [4]; default: 0;
 * SAR2_EN_TEST  only active when reg_sar2_dig_force = 0
 */

#define SENS_SAR2_EN_TEST    (BIT(4))
#define SENS_SAR2_EN_TEST_M  (SENS_SAR2_EN_TEST_V << SENS_SAR2_EN_TEST_S)
#define SENS_SAR2_EN_TEST_V  0x00000001
#define SENS_SAR2_EN_TEST_S  4

/* SENS_SAR2_BIT_WIDTH : RW; bitpos: [3:2]; default: 3;
 * 00: 9 bit  01: 10 bits  10: 11bits  11: 12bits
 */

#define SENS_SAR2_BIT_WIDTH    0x00000003
#define SENS_SAR2_BIT_WIDTH_M  (SENS_SAR2_BIT_WIDTH_V << SENS_SAR2_BIT_WIDTH_S)
#define SENS_SAR2_BIT_WIDTH_V  0x00000003
#define SENS_SAR2_BIT_WIDTH_S  2

/* SENS_SAR1_BIT_WIDTH : RW; bitpos: [1:0]; default: 3;
 * 00: 9 bit  01: 10 bits  10: 11bits  11: 12bits
 */

#define SENS_SAR1_BIT_WIDTH    0x00000003
#define SENS_SAR1_BIT_WIDTH_M  (SENS_SAR1_BIT_WIDTH_V << SENS_SAR1_BIT_WIDTH_S)
#define SENS_SAR1_BIT_WIDTH_V  0x00000003
#define SENS_SAR1_BIT_WIDTH_S  0

/* SENS_SAR_MEM_WR_CTRL_REG register */

#define SENS_SAR_MEM_WR_CTRL_REG (DR_REG_SENS_BASE + 0x30)

/* SENS_RTC_MEM_WR_OFFST_CLR : W; bitpos: [22]; default: 0; */

#define SENS_RTC_MEM_WR_OFFST_CLR    (BIT(22))
#define SENS_RTC_MEM_WR_OFFST_CLR_M  (SENS_RTC_MEM_WR_OFFST_CLR_V << SENS_RTC_MEM_WR_OFFST_CLR_S)
#define SENS_RTC_MEM_WR_OFFST_CLR_V  0x00000001
#define SENS_RTC_MEM_WR_OFFST_CLR_S  22

/* SENS_MEM_WR_ADDR_SIZE : RW; bitpos: [21:11]; default: 512; */

#define SENS_MEM_WR_ADDR_SIZE    0x000007ff
#define SENS_MEM_WR_ADDR_SIZE_M  (SENS_MEM_WR_ADDR_SIZE_V << SENS_MEM_WR_ADDR_SIZE_S)
#define SENS_MEM_WR_ADDR_SIZE_V  0x000007ff
#define SENS_MEM_WR_ADDR_SIZE_S  11

/* SENS_MEM_WR_ADDR_INIT : RW; bitpos: [10:0]; default: 512; */

#define SENS_MEM_WR_ADDR_INIT    0x000007ff
#define SENS_MEM_WR_ADDR_INIT_M  (SENS_MEM_WR_ADDR_INIT_V << SENS_MEM_WR_ADDR_INIT_S)
#define SENS_MEM_WR_ADDR_INIT_V  0x000007ff
#define SENS_MEM_WR_ADDR_INIT_S  0

/* SENS_SAR_ATTEN1_REG register */

#define SENS_SAR_ATTEN1_REG (DR_REG_SENS_BASE + 0x34)

/* SENS_SAR1_ATTEN : RW; bitpos: [31:0]; default: 4294967295;
 * 2-bit attenuation for each pad  11:1dB  10:6dB  01:3dB  00:0dB
 */

#define SENS_SAR1_ATTEN    0xffffffff
#define SENS_SAR1_ATTEN_M  (SENS_SAR1_ATTEN_V << SENS_SAR1_ATTEN_S)
#define SENS_SAR1_ATTEN_V  0xffffffff
#define SENS_SAR1_ATTEN_S  0

/* SENS_SAR_ATTEN2_REG register */

#define SENS_SAR_ATTEN2_REG (DR_REG_SENS_BASE + 0x38)

/* SENS_SAR2_ATTEN : RW; bitpos: [31:0]; default: 4294967295;
 * 2-bit attenuation for each pad  11:1dB  10:6dB  01:3dB  00:0dB
 */

#define SENS_SAR2_ATTEN    0xffffffff
#define SENS_SAR2_ATTEN_M  (SENS_SAR2_ATTEN_V << SENS_SAR2_ATTEN_S)
#define SENS_SAR2_ATTEN_V  0xffffffff
#define SENS_SAR2_ATTEN_S  0

/* SENS_SAR_SLAVE_ADDR1_REG register */

#define SENS_SAR_SLAVE_ADDR1_REG (DR_REG_SENS_BASE + 0x3c)

/* SENS_MEAS_STATUS : R; bitpos: [29:22]; default: 0; */

#define SENS_MEAS_STATUS    0x000000ff
#define SENS_MEAS_STATUS_M  (SENS_MEAS_STATUS_V << SENS_MEAS_STATUS_S)
#define SENS_MEAS_STATUS_V  0x000000ff
#define SENS_MEAS_STATUS_S  22

/* SENS_I2C_SLAVE_ADDR0 : RW; bitpos: [21:11]; default: 0; */

#define SENS_I2C_SLAVE_ADDR0    0x000007ff
#define SENS_I2C_SLAVE_ADDR0_M  (SENS_I2C_SLAVE_ADDR0_V << SENS_I2C_SLAVE_ADDR0_S)
#define SENS_I2C_SLAVE_ADDR0_V  0x000007ff
#define SENS_I2C_SLAVE_ADDR0_S  11

/* SENS_I2C_SLAVE_ADDR1 : RW; bitpos: [10:0]; default: 0; */

#define SENS_I2C_SLAVE_ADDR1    0x000007ff
#define SENS_I2C_SLAVE_ADDR1_M  (SENS_I2C_SLAVE_ADDR1_V << SENS_I2C_SLAVE_ADDR1_S)
#define SENS_I2C_SLAVE_ADDR1_V  0x000007ff
#define SENS_I2C_SLAVE_ADDR1_S  0

/* SENS_SAR_SLAVE_ADDR2_REG register */

#define SENS_SAR_SLAVE_ADDR2_REG (DR_REG_SENS_BASE + 0x40)

/* SENS_I2C_SLAVE_ADDR2 : RW; bitpos: [21:11]; default: 0; */

#define SENS_I2C_SLAVE_ADDR2    0x000007ff
#define SENS_I2C_SLAVE_ADDR2_M  (SENS_I2C_SLAVE_ADDR2_V << SENS_I2C_SLAVE_ADDR2_S)
#define SENS_I2C_SLAVE_ADDR2_V  0x000007ff
#define SENS_I2C_SLAVE_ADDR2_S  11

/* SENS_I2C_SLAVE_ADDR3 : RW; bitpos: [10:0]; default: 0; */

#define SENS_I2C_SLAVE_ADDR3    0x000007ff
#define SENS_I2C_SLAVE_ADDR3_M  (SENS_I2C_SLAVE_ADDR3_V << SENS_I2C_SLAVE_ADDR3_S)
#define SENS_I2C_SLAVE_ADDR3_V  0x000007ff
#define SENS_I2C_SLAVE_ADDR3_S  0

/* SENS_SAR_SLAVE_ADDR3_REG register */

#define SENS_SAR_SLAVE_ADDR3_REG (DR_REG_SENS_BASE + 0x44)

/* SENS_TSENS_RDY_OUT : R; bitpos: [30]; default: 0;
 * indicate temperature sensor out ready
 */

#define SENS_TSENS_RDY_OUT    (BIT(30))
#define SENS_TSENS_RDY_OUT_M  (SENS_TSENS_RDY_OUT_V << SENS_TSENS_RDY_OUT_S)
#define SENS_TSENS_RDY_OUT_V  0x00000001
#define SENS_TSENS_RDY_OUT_S  30

/* SENS_TSENS_OUT : R; bitpos: [29:22]; default: 0;
 * temperature sensor data out
 */

#define SENS_TSENS_OUT    0x000000ff
#define SENS_TSENS_OUT_M  (SENS_TSENS_OUT_V << SENS_TSENS_OUT_S)
#define SENS_TSENS_OUT_V  0x000000ff
#define SENS_TSENS_OUT_S  22

/* SENS_I2C_SLAVE_ADDR4 : RW; bitpos: [21:11]; default: 0; */

#define SENS_I2C_SLAVE_ADDR4    0x000007ff
#define SENS_I2C_SLAVE_ADDR4_M  (SENS_I2C_SLAVE_ADDR4_V << SENS_I2C_SLAVE_ADDR4_S)
#define SENS_I2C_SLAVE_ADDR4_V  0x000007ff
#define SENS_I2C_SLAVE_ADDR4_S  11

/* SENS_I2C_SLAVE_ADDR5 : RW; bitpos: [10:0]; default: 0; */

#define SENS_I2C_SLAVE_ADDR5    0x000007ff
#define SENS_I2C_SLAVE_ADDR5_M  (SENS_I2C_SLAVE_ADDR5_V << SENS_I2C_SLAVE_ADDR5_S)
#define SENS_I2C_SLAVE_ADDR5_V  0x000007ff
#define SENS_I2C_SLAVE_ADDR5_S  0

/* SENS_SAR_SLAVE_ADDR4_REG register */

#define SENS_SAR_SLAVE_ADDR4_REG (DR_REG_SENS_BASE + 0x48)

/* SENS_I2C_DONE : R; bitpos: [30]; default: 0;
 * indicate I2C done
 */

#define SENS_I2C_DONE    (BIT(30))
#define SENS_I2C_DONE_M  (SENS_I2C_DONE_V << SENS_I2C_DONE_S)
#define SENS_I2C_DONE_V  0x00000001
#define SENS_I2C_DONE_S  30

/* SENS_I2C_RDATA : R; bitpos: [29:22]; default: 0;
 * I2C read data
 */

#define SENS_I2C_RDATA    0x000000ff
#define SENS_I2C_RDATA_M  (SENS_I2C_RDATA_V << SENS_I2C_RDATA_S)
#define SENS_I2C_RDATA_V  0x000000ff
#define SENS_I2C_RDATA_S  22

/* SENS_I2C_SLAVE_ADDR6 : RW; bitpos: [21:11]; default: 0; */

#define SENS_I2C_SLAVE_ADDR6    0x000007ff
#define SENS_I2C_SLAVE_ADDR6_M  (SENS_I2C_SLAVE_ADDR6_V << SENS_I2C_SLAVE_ADDR6_S)
#define SENS_I2C_SLAVE_ADDR6_V  0x000007ff
#define SENS_I2C_SLAVE_ADDR6_S  11

/* SENS_I2C_SLAVE_ADDR7 : RW; bitpos: [10:0]; default: 0; */

#define SENS_I2C_SLAVE_ADDR7    0x000007ff
#define SENS_I2C_SLAVE_ADDR7_M  (SENS_I2C_SLAVE_ADDR7_V << SENS_I2C_SLAVE_ADDR7_S)
#define SENS_I2C_SLAVE_ADDR7_V  0x000007ff
#define SENS_I2C_SLAVE_ADDR7_S  0

/* SENS_SAR_TSENS_CTRL_REG register */

#define SENS_SAR_TSENS_CTRL_REG (DR_REG_SENS_BASE + 0x4c)

/* SENS_TSENS_DUMP_OUT : RW; bitpos: [26]; default: 0;
 * temperature sensor dump out  only active when reg_tsens_power_up_force = 1
 */

#define SENS_TSENS_DUMP_OUT    (BIT(26))
#define SENS_TSENS_DUMP_OUT_M  (SENS_TSENS_DUMP_OUT_V << SENS_TSENS_DUMP_OUT_S)
#define SENS_TSENS_DUMP_OUT_V  0x00000001
#define SENS_TSENS_DUMP_OUT_S  26

/* SENS_TSENS_POWER_UP_FORCE : RW; bitpos: [25]; default: 0;
 * 1: dump out & power up controlled by SW  0: by FSM
 */

#define SENS_TSENS_POWER_UP_FORCE    (BIT(25))
#define SENS_TSENS_POWER_UP_FORCE_M  (SENS_TSENS_POWER_UP_FORCE_V << SENS_TSENS_POWER_UP_FORCE_S)
#define SENS_TSENS_POWER_UP_FORCE_V  0x00000001
#define SENS_TSENS_POWER_UP_FORCE_S  25

/* SENS_TSENS_POWER_UP : RW; bitpos: [24]; default: 0;
 * temperature sensor power up
 */

#define SENS_TSENS_POWER_UP    (BIT(24))
#define SENS_TSENS_POWER_UP_M  (SENS_TSENS_POWER_UP_V << SENS_TSENS_POWER_UP_S)
#define SENS_TSENS_POWER_UP_V  0x00000001
#define SENS_TSENS_POWER_UP_S  24

/* SENS_TSENS_CLK_DIV : RW; bitpos: [23:16]; default: 6;
 * temperature sensor clock divider
 */

#define SENS_TSENS_CLK_DIV    0x000000ff
#define SENS_TSENS_CLK_DIV_M  (SENS_TSENS_CLK_DIV_V << SENS_TSENS_CLK_DIV_S)
#define SENS_TSENS_CLK_DIV_V  0x000000ff
#define SENS_TSENS_CLK_DIV_S  16

/* SENS_TSENS_IN_INV : RW; bitpos: [15]; default: 0;
 * invert temperature sensor data
 */

#define SENS_TSENS_IN_INV    (BIT(15))
#define SENS_TSENS_IN_INV_M  (SENS_TSENS_IN_INV_V << SENS_TSENS_IN_INV_S)
#define SENS_TSENS_IN_INV_V  0x00000001
#define SENS_TSENS_IN_INV_S  15

/* SENS_TSENS_CLK_GATED : RW; bitpos: [14]; default: 1; */

#define SENS_TSENS_CLK_GATED    (BIT(14))
#define SENS_TSENS_CLK_GATED_M  (SENS_TSENS_CLK_GATED_V << SENS_TSENS_CLK_GATED_S)
#define SENS_TSENS_CLK_GATED_V  0x00000001
#define SENS_TSENS_CLK_GATED_S  14

/* SENS_TSENS_CLK_INV : RW; bitpos: [13]; default: 1; */

#define SENS_TSENS_CLK_INV    (BIT(13))
#define SENS_TSENS_CLK_INV_M  (SENS_TSENS_CLK_INV_V << SENS_TSENS_CLK_INV_S)
#define SENS_TSENS_CLK_INV_V  0x00000001
#define SENS_TSENS_CLK_INV_S  13

/* SENS_TSENS_XPD_FORCE : RW; bitpos: [12]; default: 0; */

#define SENS_TSENS_XPD_FORCE    (BIT(12))
#define SENS_TSENS_XPD_FORCE_M  (SENS_TSENS_XPD_FORCE_V << SENS_TSENS_XPD_FORCE_S)
#define SENS_TSENS_XPD_FORCE_V  0x00000001
#define SENS_TSENS_XPD_FORCE_S  12

/* SENS_TSENS_XPD_WAIT : RW; bitpos: [11:0]; default: 2; */

#define SENS_TSENS_XPD_WAIT    0x00000fff
#define SENS_TSENS_XPD_WAIT_M  (SENS_TSENS_XPD_WAIT_V << SENS_TSENS_XPD_WAIT_S)
#define SENS_TSENS_XPD_WAIT_V  0x00000fff
#define SENS_TSENS_XPD_WAIT_S  0

/* SENS_SAR_I2C_CTRL_REG register */

#define SENS_SAR_I2C_CTRL_REG (DR_REG_SENS_BASE + 0x50)

/* SENS_SAR_I2C_START_FORCE : RW; bitpos: [29]; default: 0;
 * 1: I2C started by SW  0: I2C started by FSM
 */

#define SENS_SAR_I2C_START_FORCE    (BIT(29))
#define SENS_SAR_I2C_START_FORCE_M  (SENS_SAR_I2C_START_FORCE_V << SENS_SAR_I2C_START_FORCE_S)
#define SENS_SAR_I2C_START_FORCE_V  0x00000001
#define SENS_SAR_I2C_START_FORCE_S  29

/* SENS_SAR_I2C_START : RW; bitpos: [28]; default: 0;
 * start I2C  only active when reg_sar_i2c_start_force = 1
 */

#define SENS_SAR_I2C_START    (BIT(28))
#define SENS_SAR_I2C_START_M  (SENS_SAR_I2C_START_V << SENS_SAR_I2C_START_S)
#define SENS_SAR_I2C_START_V  0x00000001
#define SENS_SAR_I2C_START_S  28

/* SENS_SAR_I2C_CTRL : RW; bitpos: [27:0]; default: 0;
 * I2C control data  only active when reg_sar_i2c_start_force = 1
 */

#define SENS_SAR_I2C_CTRL    0x0fffffff
#define SENS_SAR_I2C_CTRL_M  (SENS_SAR_I2C_CTRL_V << SENS_SAR_I2C_CTRL_S)
#define SENS_SAR_I2C_CTRL_V  0x0fffffff
#define SENS_SAR_I2C_CTRL_S  0

/* SENS_SAR_MEAS_START1_REG register */

#define SENS_SAR_MEAS_START1_REG (DR_REG_SENS_BASE + 0x54)

/* SENS_SAR1_EN_PAD_FORCE : RW; bitpos: [31]; default: 0;
 * 1: SAR ADC1 pad enable bitmap is controlled by SW  0: SAR ADC1 pad enable
 * bitmap is controlled by ULP-coprocessor
 */

#define SENS_SAR1_EN_PAD_FORCE    (BIT(31))
#define SENS_SAR1_EN_PAD_FORCE_M  (SENS_SAR1_EN_PAD_FORCE_V << SENS_SAR1_EN_PAD_FORCE_S)
#define SENS_SAR1_EN_PAD_FORCE_V  0x00000001
#define SENS_SAR1_EN_PAD_FORCE_S  31

/* SENS_SAR1_EN_PAD : RW; bitpos: [30:19]; default: 0;
 * SAR ADC1 pad enable bitmap  only active when reg_sar1_en_pad_force = 1
 */

#define SENS_SAR1_EN_PAD    0x00000fff
#define SENS_SAR1_EN_PAD_M  (SENS_SAR1_EN_PAD_V << SENS_SAR1_EN_PAD_S)
#define SENS_SAR1_EN_PAD_V  0x00000fff
#define SENS_SAR1_EN_PAD_S  19

/* SENS_MEAS1_START_FORCE : RW; bitpos: [18]; default: 0;
 * 1: SAR ADC1 controller (in RTC) is started by SW  0: SAR ADC1 controller
 * is started by ULP-coprocessor
 */

#define SENS_MEAS1_START_FORCE    (BIT(18))
#define SENS_MEAS1_START_FORCE_M  (SENS_MEAS1_START_FORCE_V << SENS_MEAS1_START_FORCE_S)
#define SENS_MEAS1_START_FORCE_V  0x00000001
#define SENS_MEAS1_START_FORCE_S  18

/* SENS_MEAS1_START_SAR : RW; bitpos: [17]; default: 0;
 * SAR ADC1 controller (in RTC) starts conversion  only active when
 * reg_meas1_start_force = 1
 */

#define SENS_MEAS1_START_SAR    (BIT(17))
#define SENS_MEAS1_START_SAR_M  (SENS_MEAS1_START_SAR_V << SENS_MEAS1_START_SAR_S)
#define SENS_MEAS1_START_SAR_V  0x00000001
#define SENS_MEAS1_START_SAR_S  17

/* SENS_MEAS1_DONE_SAR : R; bitpos: [16]; default: 0;
 * SAR ADC1 conversion done indication
 */

#define SENS_MEAS1_DONE_SAR    (BIT(16))
#define SENS_MEAS1_DONE_SAR_M  (SENS_MEAS1_DONE_SAR_V << SENS_MEAS1_DONE_SAR_S)
#define SENS_MEAS1_DONE_SAR_V  0x00000001
#define SENS_MEAS1_DONE_SAR_S  16

/* SENS_MEAS1_DATA_SAR : R; bitpos: [15:0]; default: 0;
 * SAR ADC1 data
 */

#define SENS_MEAS1_DATA_SAR    0x0000ffff
#define SENS_MEAS1_DATA_SAR_M  (SENS_MEAS1_DATA_SAR_V << SENS_MEAS1_DATA_SAR_S)
#define SENS_MEAS1_DATA_SAR_V  0x0000ffff
#define SENS_MEAS1_DATA_SAR_S  0

/* SENS_SAR_TOUCH_CTRL1_REG register */

#define SENS_SAR_TOUCH_CTRL1_REG (DR_REG_SENS_BASE + 0x58)

/* SENS_HALL_PHASE_FORCE : RW; bitpos: [27]; default: 0;
 * 1: HALL PHASE is controlled by SW  0: HALL PHASE is controlled by FSM in
 * ULP-coprocessor
 */

#define SENS_HALL_PHASE_FORCE    (BIT(27))
#define SENS_HALL_PHASE_FORCE_M  (SENS_HALL_PHASE_FORCE_V << SENS_HALL_PHASE_FORCE_S)
#define SENS_HALL_PHASE_FORCE_V  0x00000001
#define SENS_HALL_PHASE_FORCE_S  27

/* SENS_XPD_HALL_FORCE : RW; bitpos: [26]; default: 0;
 * 1: XPD HALL is controlled by SW. 0: XPD HALL is controlled by FSM in
 * ULP-coprocessor
 */

#define SENS_XPD_HALL_FORCE    (BIT(26))
#define SENS_XPD_HALL_FORCE_M  (SENS_XPD_HALL_FORCE_V << SENS_XPD_HALL_FORCE_S)
#define SENS_XPD_HALL_FORCE_V  0x00000001
#define SENS_XPD_HALL_FORCE_S  26

/* SENS_TOUCH_OUT_1EN : RW; bitpos: [25]; default: 1;
 * 1: wakeup interrupt is generated if SET1 is "touched"  0: wakeup
 * interrupt is generated only if SET1 & SET2 is both "touched"
 */

#define SENS_TOUCH_OUT_1EN    (BIT(25))
#define SENS_TOUCH_OUT_1EN_M  (SENS_TOUCH_OUT_1EN_V << SENS_TOUCH_OUT_1EN_S)
#define SENS_TOUCH_OUT_1EN_V  0x00000001
#define SENS_TOUCH_OUT_1EN_S  25

/* SENS_TOUCH_OUT_SEL : RW; bitpos: [24]; default: 0;
 * 1: when the counter is greater then the threshold  the touch pad is
 * considered as "touched"  0: when the counter is less than the threshold
 * the touch pad is considered as "touched"
 */

#define SENS_TOUCH_OUT_SEL    (BIT(24))
#define SENS_TOUCH_OUT_SEL_M  (SENS_TOUCH_OUT_SEL_V << SENS_TOUCH_OUT_SEL_S)
#define SENS_TOUCH_OUT_SEL_V  0x00000001
#define SENS_TOUCH_OUT_SEL_S  24

/* SENS_TOUCH_XPD_WAIT : RW; bitpos: [23:16]; default: 4;
 * the waiting cycles (in 8MHz) between TOUCH_START and TOUCH_XPD
 */

#define SENS_TOUCH_XPD_WAIT    0x000000ff
#define SENS_TOUCH_XPD_WAIT_M  (SENS_TOUCH_XPD_WAIT_V << SENS_TOUCH_XPD_WAIT_S)
#define SENS_TOUCH_XPD_WAIT_V  0x000000ff
#define SENS_TOUCH_XPD_WAIT_S  16

/* SENS_TOUCH_MEAS_DELAY : RW; bitpos: [15:0]; default: 4096;
 * the meas length (in 8MHz)
 */

#define SENS_TOUCH_MEAS_DELAY    0x0000ffff
#define SENS_TOUCH_MEAS_DELAY_M  (SENS_TOUCH_MEAS_DELAY_V << SENS_TOUCH_MEAS_DELAY_S)
#define SENS_TOUCH_MEAS_DELAY_V  0x0000ffff
#define SENS_TOUCH_MEAS_DELAY_S  0

/* SENS_SAR_TOUCH_THRES1_REG register */

#define SENS_SAR_TOUCH_THRES1_REG (DR_REG_SENS_BASE + 0x5c)

/* SENS_TOUCH_OUT_TH0 : RW; bitpos: [31:16]; default: 0;
 * the threshold for touch pad 0
 */

#define SENS_TOUCH_OUT_TH0    0x0000ffff
#define SENS_TOUCH_OUT_TH0_M  (SENS_TOUCH_OUT_TH0_V << SENS_TOUCH_OUT_TH0_S)
#define SENS_TOUCH_OUT_TH0_V  0x0000ffff
#define SENS_TOUCH_OUT_TH0_S  16

/* SENS_TOUCH_OUT_TH1 : RW; bitpos: [15:0]; default: 0;
 * the threshold for touch pad 1
 */

#define SENS_TOUCH_OUT_TH1    0x0000ffff
#define SENS_TOUCH_OUT_TH1_M  (SENS_TOUCH_OUT_TH1_V << SENS_TOUCH_OUT_TH1_S)
#define SENS_TOUCH_OUT_TH1_V  0x0000ffff
#define SENS_TOUCH_OUT_TH1_S  0

/* SENS_SAR_TOUCH_THRES2_REG register */

#define SENS_SAR_TOUCH_THRES2_REG (DR_REG_SENS_BASE + 0x60)

/* SENS_TOUCH_OUT_TH2 : RW; bitpos: [31:16]; default: 0;
 * the threshold for touch pad 2
 */

#define SENS_TOUCH_OUT_TH2    0x0000ffff
#define SENS_TOUCH_OUT_TH2_M  (SENS_TOUCH_OUT_TH2_V << SENS_TOUCH_OUT_TH2_S)
#define SENS_TOUCH_OUT_TH2_V  0x0000ffff
#define SENS_TOUCH_OUT_TH2_S  16

/* SENS_TOUCH_OUT_TH3 : RW; bitpos: [15:0]; default: 0;
 * the threshold for touch pad 3
 */

#define SENS_TOUCH_OUT_TH3    0x0000ffff
#define SENS_TOUCH_OUT_TH3_M  (SENS_TOUCH_OUT_TH3_V << SENS_TOUCH_OUT_TH3_S)
#define SENS_TOUCH_OUT_TH3_V  0x0000ffff
#define SENS_TOUCH_OUT_TH3_S  0

/* SENS_SAR_TOUCH_THRES3_REG register */

#define SENS_SAR_TOUCH_THRES3_REG (DR_REG_SENS_BASE + 0x64)

/* SENS_TOUCH_OUT_TH4 : RW; bitpos: [31:16]; default: 0;
 * the threshold for touch pad 4
 */

#define SENS_TOUCH_OUT_TH4    0x0000ffff
#define SENS_TOUCH_OUT_TH4_M  (SENS_TOUCH_OUT_TH4_V << SENS_TOUCH_OUT_TH4_S)
#define SENS_TOUCH_OUT_TH4_V  0x0000ffff
#define SENS_TOUCH_OUT_TH4_S  16

/* SENS_TOUCH_OUT_TH5 : RW; bitpos: [15:0]; default: 0;
 * the threshold for touch pad 5
 */

#define SENS_TOUCH_OUT_TH5    0x0000ffff
#define SENS_TOUCH_OUT_TH5_M  (SENS_TOUCH_OUT_TH5_V << SENS_TOUCH_OUT_TH5_S)
#define SENS_TOUCH_OUT_TH5_V  0x0000ffff
#define SENS_TOUCH_OUT_TH5_S  0

/* SENS_SAR_TOUCH_THRES4_REG register */

#define SENS_SAR_TOUCH_THRES4_REG (DR_REG_SENS_BASE + 0x68)

/* SENS_TOUCH_OUT_TH6 : RW; bitpos: [31:16]; default: 0;
 * the threshold for touch pad 6
 */

#define SENS_TOUCH_OUT_TH6    0x0000ffff
#define SENS_TOUCH_OUT_TH6_M  (SENS_TOUCH_OUT_TH6_V << SENS_TOUCH_OUT_TH6_S)
#define SENS_TOUCH_OUT_TH6_V  0x0000ffff
#define SENS_TOUCH_OUT_TH6_S  16

/* SENS_TOUCH_OUT_TH7 : RW; bitpos: [15:0]; default: 0;
 * the threshold for touch pad 7
 */

#define SENS_TOUCH_OUT_TH7    0x0000ffff
#define SENS_TOUCH_OUT_TH7_M  (SENS_TOUCH_OUT_TH7_V << SENS_TOUCH_OUT_TH7_S)
#define SENS_TOUCH_OUT_TH7_V  0x0000ffff
#define SENS_TOUCH_OUT_TH7_S  0

/* SENS_SAR_TOUCH_THRES5_REG register */

#define SENS_SAR_TOUCH_THRES5_REG (DR_REG_SENS_BASE + 0x6c)

/* SENS_TOUCH_OUT_TH8 : RW; bitpos: [31:16]; default: 0;
 * the threshold for touch pad 8
 */

#define SENS_TOUCH_OUT_TH8    0x0000ffff
#define SENS_TOUCH_OUT_TH8_M  (SENS_TOUCH_OUT_TH8_V << SENS_TOUCH_OUT_TH8_S)
#define SENS_TOUCH_OUT_TH8_V  0x0000ffff
#define SENS_TOUCH_OUT_TH8_S  16

/* SENS_TOUCH_OUT_TH9 : RW; bitpos: [15:0]; default: 0;
 * the threshold for touch pad 9
 */

#define SENS_TOUCH_OUT_TH9    0x0000ffff
#define SENS_TOUCH_OUT_TH9_M  (SENS_TOUCH_OUT_TH9_V << SENS_TOUCH_OUT_TH9_S)
#define SENS_TOUCH_OUT_TH9_V  0x0000ffff
#define SENS_TOUCH_OUT_TH9_S  0

/* SENS_SAR_TOUCH_OUT1_REG register */

#define SENS_SAR_TOUCH_OUT1_REG (DR_REG_SENS_BASE + 0x70)

/* SENS_TOUCH_MEAS_OUT0 : R; bitpos: [31:16]; default: 0;
 * the counter for touch pad 0
 */

#define SENS_TOUCH_MEAS_OUT0    0x0000ffff
#define SENS_TOUCH_MEAS_OUT0_M  (SENS_TOUCH_MEAS_OUT0_V << SENS_TOUCH_MEAS_OUT0_S)
#define SENS_TOUCH_MEAS_OUT0_V  0x0000ffff
#define SENS_TOUCH_MEAS_OUT0_S  16

/* SENS_TOUCH_MEAS_OUT1 : R; bitpos: [15:0]; default: 0;
 * the counter for touch pad 1
 */

#define SENS_TOUCH_MEAS_OUT1    0x0000ffff
#define SENS_TOUCH_MEAS_OUT1_M  (SENS_TOUCH_MEAS_OUT1_V << SENS_TOUCH_MEAS_OUT1_S)
#define SENS_TOUCH_MEAS_OUT1_V  0x0000ffff
#define SENS_TOUCH_MEAS_OUT1_S  0

/* SENS_SAR_TOUCH_OUT2_REG register */

#define SENS_SAR_TOUCH_OUT2_REG (DR_REG_SENS_BASE + 0x74)

/* SENS_TOUCH_MEAS_OUT2 : R; bitpos: [31:16]; default: 0;
 * the counter for touch pad 2
 */

#define SENS_TOUCH_MEAS_OUT2    0x0000ffff
#define SENS_TOUCH_MEAS_OUT2_M  (SENS_TOUCH_MEAS_OUT2_V << SENS_TOUCH_MEAS_OUT2_S)
#define SENS_TOUCH_MEAS_OUT2_V  0x0000ffff
#define SENS_TOUCH_MEAS_OUT2_S  16

/* SENS_TOUCH_MEAS_OUT3 : R; bitpos: [15:0]; default: 0;
 * the counter for touch pad 3
 */

#define SENS_TOUCH_MEAS_OUT3    0x0000ffff
#define SENS_TOUCH_MEAS_OUT3_M  (SENS_TOUCH_MEAS_OUT3_V << SENS_TOUCH_MEAS_OUT3_S)
#define SENS_TOUCH_MEAS_OUT3_V  0x0000ffff
#define SENS_TOUCH_MEAS_OUT3_S  0

/* SENS_SAR_TOUCH_OUT3_REG register */

#define SENS_SAR_TOUCH_OUT3_REG (DR_REG_SENS_BASE + 0x78)

/* SENS_TOUCH_MEAS_OUT4 : R; bitpos: [31:16]; default: 0;
 * the counter for touch pad 4
 */

#define SENS_TOUCH_MEAS_OUT4    0x0000ffff
#define SENS_TOUCH_MEAS_OUT4_M  (SENS_TOUCH_MEAS_OUT4_V << SENS_TOUCH_MEAS_OUT4_S)
#define SENS_TOUCH_MEAS_OUT4_V  0x0000ffff
#define SENS_TOUCH_MEAS_OUT4_S  16

/* SENS_TOUCH_MEAS_OUT5 : R; bitpos: [15:0]; default: 0;
 * the counter for touch pad 5
 */

#define SENS_TOUCH_MEAS_OUT5    0x0000ffff
#define SENS_TOUCH_MEAS_OUT5_M  (SENS_TOUCH_MEAS_OUT5_V << SENS_TOUCH_MEAS_OUT5_S)
#define SENS_TOUCH_MEAS_OUT5_V  0x0000ffff
#define SENS_TOUCH_MEAS_OUT5_S  0

/* SENS_SAR_TOUCH_OUT4_REG register */

#define SENS_SAR_TOUCH_OUT4_REG (DR_REG_SENS_BASE + 0x7c)

/* SENS_TOUCH_MEAS_OUT6 : R; bitpos: [31:16]; default: 0;
 * the counter for touch pad 6
 */

#define SENS_TOUCH_MEAS_OUT6    0x0000ffff
#define SENS_TOUCH_MEAS_OUT6_M  (SENS_TOUCH_MEAS_OUT6_V << SENS_TOUCH_MEAS_OUT6_S)
#define SENS_TOUCH_MEAS_OUT6_V  0x0000ffff
#define SENS_TOUCH_MEAS_OUT6_S  16

/* SENS_TOUCH_MEAS_OUT7 : R; bitpos: [15:0]; default: 0;
 * the counter for touch pad 7
 */

#define SENS_TOUCH_MEAS_OUT7    0x0000ffff
#define SENS_TOUCH_MEAS_OUT7_M  (SENS_TOUCH_MEAS_OUT7_V << SENS_TOUCH_MEAS_OUT7_S)
#define SENS_TOUCH_MEAS_OUT7_V  0x0000ffff
#define SENS_TOUCH_MEAS_OUT7_S  0

/* SENS_SAR_TOUCH_OUT5_REG register */

#define SENS_SAR_TOUCH_OUT5_REG (DR_REG_SENS_BASE + 0x80)

/* SENS_TOUCH_MEAS_OUT8 : R; bitpos: [31:16]; default: 0;
 * the counter for touch pad 8
 */

#define SENS_TOUCH_MEAS_OUT8    0x0000ffff
#define SENS_TOUCH_MEAS_OUT8_M  (SENS_TOUCH_MEAS_OUT8_V << SENS_TOUCH_MEAS_OUT8_S)
#define SENS_TOUCH_MEAS_OUT8_V  0x0000ffff
#define SENS_TOUCH_MEAS_OUT8_S  16

/* SENS_TOUCH_MEAS_OUT9 : R; bitpos: [15:0]; default: 0;
 * the counter for touch pad 9
 */

#define SENS_TOUCH_MEAS_OUT9    0x0000ffff
#define SENS_TOUCH_MEAS_OUT9_M  (SENS_TOUCH_MEAS_OUT9_V << SENS_TOUCH_MEAS_OUT9_S)
#define SENS_TOUCH_MEAS_OUT9_V  0x0000ffff
#define SENS_TOUCH_MEAS_OUT9_S  0

/* SENS_SAR_TOUCH_CTRL2_REG register */

#define SENS_SAR_TOUCH_CTRL2_REG (DR_REG_SENS_BASE + 0x84)

/* SENS_TOUCH_MEAS_EN_CLR : W; bitpos: [30]; default: 0;
 * to clear reg_touch_meas_en
 */

#define SENS_TOUCH_MEAS_EN_CLR    (BIT(30))
#define SENS_TOUCH_MEAS_EN_CLR_M  (SENS_TOUCH_MEAS_EN_CLR_V << SENS_TOUCH_MEAS_EN_CLR_S)
#define SENS_TOUCH_MEAS_EN_CLR_V  0x00000001
#define SENS_TOUCH_MEAS_EN_CLR_S  30

/* SENS_TOUCH_SLEEP_CYCLES : RW; bitpos: [29:14]; default: 256;
 * sleep cycles for timer
 */

#define SENS_TOUCH_SLEEP_CYCLES    0x0000ffff
#define SENS_TOUCH_SLEEP_CYCLES_M  (SENS_TOUCH_SLEEP_CYCLES_V << SENS_TOUCH_SLEEP_CYCLES_S)
#define SENS_TOUCH_SLEEP_CYCLES_V  0x0000ffff
#define SENS_TOUCH_SLEEP_CYCLES_S  14

/* SENS_TOUCH_START_FORCE : RW; bitpos: [13]; default: 0;
 * 1: to start touch fsm by SW  0: to start touch fsm by timer
 */

#define SENS_TOUCH_START_FORCE    (BIT(13))
#define SENS_TOUCH_START_FORCE_M  (SENS_TOUCH_START_FORCE_V << SENS_TOUCH_START_FORCE_S)
#define SENS_TOUCH_START_FORCE_V  0x00000001
#define SENS_TOUCH_START_FORCE_S  13

/* SENS_TOUCH_START_EN : RW; bitpos: [12]; default: 0;
 * 1: start touch fsm  valid when reg_touch_start_force is set
 */

#define SENS_TOUCH_START_EN    (BIT(12))
#define SENS_TOUCH_START_EN_M  (SENS_TOUCH_START_EN_V << SENS_TOUCH_START_EN_S)
#define SENS_TOUCH_START_EN_V  0x00000001
#define SENS_TOUCH_START_EN_S  12

/* SENS_TOUCH_START_FSM_EN : RW; bitpos: [11]; default: 1;
 * 1: TOUCH_START & TOUCH_XPD is controlled by touch fsm  0: TOUCH_START &
 * TOUCH_XPD is controlled by registers
 */

#define SENS_TOUCH_START_FSM_EN    (BIT(11))
#define SENS_TOUCH_START_FSM_EN_M  (SENS_TOUCH_START_FSM_EN_V << SENS_TOUCH_START_FSM_EN_S)
#define SENS_TOUCH_START_FSM_EN_V  0x00000001
#define SENS_TOUCH_START_FSM_EN_S  11

/* SENS_TOUCH_MEAS_DONE : R; bitpos: [10]; default: 0;
 * fsm set 1 to indicate touch touch meas is done
 */

#define SENS_TOUCH_MEAS_DONE    (BIT(10))
#define SENS_TOUCH_MEAS_DONE_M  (SENS_TOUCH_MEAS_DONE_V << SENS_TOUCH_MEAS_DONE_S)
#define SENS_TOUCH_MEAS_DONE_V  0x00000001
#define SENS_TOUCH_MEAS_DONE_S  10

/* SENS_TOUCH_MEAS_EN : R; bitpos: [9:0]; default: 0;
 * 10-bit register to indicate which pads are "touched"
 */

#define SENS_TOUCH_MEAS_EN    0x000003ff
#define SENS_TOUCH_MEAS_EN_M  (SENS_TOUCH_MEAS_EN_V << SENS_TOUCH_MEAS_EN_S)
#define SENS_TOUCH_MEAS_EN_V  0x000003ff
#define SENS_TOUCH_MEAS_EN_S  0

/* SENS_SAR_TOUCH_ENABLE_REG register */

#define SENS_SAR_TOUCH_ENABLE_REG (DR_REG_SENS_BASE + 0x8c)

/* SENS_TOUCH_PAD_OUTEN1 : RW; bitpos: [29:20]; default: 1023;
 * Bitmap defining SET1 for generating wakeup interrupt. SET1 is "touched"
 * only if at least one of touch pad in SET1 is "touched".
 */

#define SENS_TOUCH_PAD_OUTEN1    0x000003ff
#define SENS_TOUCH_PAD_OUTEN1_M  (SENS_TOUCH_PAD_OUTEN1_V << SENS_TOUCH_PAD_OUTEN1_S)
#define SENS_TOUCH_PAD_OUTEN1_V  0x000003ff
#define SENS_TOUCH_PAD_OUTEN1_S  20

/* SENS_TOUCH_PAD_OUTEN2 : RW; bitpos: [19:10]; default: 1023;
 * Bitmap defining SET2 for generating wakeup interrupt. SET2 is "touched"
 * only if at least one of touch pad in SET2 is "touched".
 */

#define SENS_TOUCH_PAD_OUTEN2    0x000003ff
#define SENS_TOUCH_PAD_OUTEN2_M  (SENS_TOUCH_PAD_OUTEN2_V << SENS_TOUCH_PAD_OUTEN2_S)
#define SENS_TOUCH_PAD_OUTEN2_V  0x000003ff
#define SENS_TOUCH_PAD_OUTEN2_S  10

/* SENS_TOUCH_PAD_WORKEN : RW; bitpos: [9:0]; default: 1023;
 * Bitmap defining the working set during the measurement.
 */

#define SENS_TOUCH_PAD_WORKEN    0x000003ff
#define SENS_TOUCH_PAD_WORKEN_M  (SENS_TOUCH_PAD_WORKEN_V << SENS_TOUCH_PAD_WORKEN_S)
#define SENS_TOUCH_PAD_WORKEN_V  0x000003ff
#define SENS_TOUCH_PAD_WORKEN_S  0

/* SENS_SAR_READ_CTRL2_REG register */

#define SENS_SAR_READ_CTRL2_REG (DR_REG_SENS_BASE + 0x90)

/* SENS_SAR2_DATA_INV : RW; bitpos: [29]; default: 0;
 * Invert SAR ADC2 data
 */

#define SENS_SAR2_DATA_INV    (BIT(29))
#define SENS_SAR2_DATA_INV_M  (SENS_SAR2_DATA_INV_V << SENS_SAR2_DATA_INV_S)
#define SENS_SAR2_DATA_INV_V  0x00000001
#define SENS_SAR2_DATA_INV_S  29

/* SENS_SAR2_DIG_FORCE : RW; bitpos: [28]; default: 0;
 * 1: SAR ADC2 controlled by DIG ADC2 CTRL or PWDET CTRL  0: SAR ADC2
 * controlled by RTC ADC2 CTRL
 */

#define SENS_SAR2_DIG_FORCE    (BIT(28))
#define SENS_SAR2_DIG_FORCE_M  (SENS_SAR2_DIG_FORCE_V << SENS_SAR2_DIG_FORCE_S)
#define SENS_SAR2_DIG_FORCE_V  0x00000001
#define SENS_SAR2_DIG_FORCE_S  28

/* SENS_SAR2_PWDET_FORCE : RW; bitpos: [27]; default: 0; */

#define SENS_SAR2_PWDET_FORCE    (BIT(27))
#define SENS_SAR2_PWDET_FORCE_M  (SENS_SAR2_PWDET_FORCE_V << SENS_SAR2_PWDET_FORCE_S)
#define SENS_SAR2_PWDET_FORCE_V  0x00000001
#define SENS_SAR2_PWDET_FORCE_S  27

/* SENS_SAR2_SAMPLE_NUM : RW; bitpos: [26:19]; default: 0; */

#define SENS_SAR2_SAMPLE_NUM    0x000000ff
#define SENS_SAR2_SAMPLE_NUM_M  (SENS_SAR2_SAMPLE_NUM_V << SENS_SAR2_SAMPLE_NUM_S)
#define SENS_SAR2_SAMPLE_NUM_V  0x000000ff
#define SENS_SAR2_SAMPLE_NUM_S  19

/* SENS_SAR2_CLK_GATED : RW; bitpos: [18]; default: 1; */

#define SENS_SAR2_CLK_GATED    (BIT(18))
#define SENS_SAR2_CLK_GATED_M  (SENS_SAR2_CLK_GATED_V << SENS_SAR2_CLK_GATED_S)
#define SENS_SAR2_CLK_GATED_V  0x00000001
#define SENS_SAR2_CLK_GATED_S  18

/* SENS_SAR2_SAMPLE_BIT : RW; bitpos: [17:16]; default: 3;
 * 00: for 9-bit width  01: for 10-bit width  10: for 11-bit width  11: for
 * 12-bit width
 */

#define SENS_SAR2_SAMPLE_BIT    0x00000003
#define SENS_SAR2_SAMPLE_BIT_M  (SENS_SAR2_SAMPLE_BIT_V << SENS_SAR2_SAMPLE_BIT_S)
#define SENS_SAR2_SAMPLE_BIT_V  0x00000003
#define SENS_SAR2_SAMPLE_BIT_S  16

/* SENS_SAR2_SAMPLE_CYCLE : RW; bitpos: [15:8]; default: 9;
 * sample cycles for SAR ADC2
 */

#define SENS_SAR2_SAMPLE_CYCLE    0x000000ff
#define SENS_SAR2_SAMPLE_CYCLE_M  (SENS_SAR2_SAMPLE_CYCLE_V << SENS_SAR2_SAMPLE_CYCLE_S)
#define SENS_SAR2_SAMPLE_CYCLE_V  0x000000ff
#define SENS_SAR2_SAMPLE_CYCLE_S  8

/* SENS_SAR2_CLK_DIV : RW; bitpos: [7:0]; default: 2;
 * clock divider
 */

#define SENS_SAR2_CLK_DIV    0x000000ff
#define SENS_SAR2_CLK_DIV_M  (SENS_SAR2_CLK_DIV_V << SENS_SAR2_CLK_DIV_S)
#define SENS_SAR2_CLK_DIV_V  0x000000ff
#define SENS_SAR2_CLK_DIV_S  0

/* SENS_SAR_MEAS_START2_REG register */

#define SENS_SAR_MEAS_START2_REG (DR_REG_SENS_BASE + 0x94)

/* SENS_SAR2_EN_PAD_FORCE : RW; bitpos: [31]; default: 0;
 * 1: SAR ADC2 pad enable bitmap is controlled by SW  0: SAR ADC2 pad enable
 * bitmap is controlled by ULP-coprocessor
 */

#define SENS_SAR2_EN_PAD_FORCE    (BIT(31))
#define SENS_SAR2_EN_PAD_FORCE_M  (SENS_SAR2_EN_PAD_FORCE_V << SENS_SAR2_EN_PAD_FORCE_S)
#define SENS_SAR2_EN_PAD_FORCE_V  0x00000001
#define SENS_SAR2_EN_PAD_FORCE_S  31

/* SENS_SAR2_EN_PAD : RW; bitpos: [30:19]; default: 0;
 * SAR ADC2 pad enable bitmap  only active when reg_sar2_en_pad_force = 1
 */

#define SENS_SAR2_EN_PAD    0x00000fff
#define SENS_SAR2_EN_PAD_M  (SENS_SAR2_EN_PAD_V << SENS_SAR2_EN_PAD_S)
#define SENS_SAR2_EN_PAD_V  0x00000fff
#define SENS_SAR2_EN_PAD_S  19

/* SENS_MEAS2_START_FORCE : RW; bitpos: [18]; default: 0;
 * 1: SAR ADC2 controller (in RTC) is started by SW  0: SAR ADC2 controller
 * is started by ULP-coprocessor
 */

#define SENS_MEAS2_START_FORCE    (BIT(18))
#define SENS_MEAS2_START_FORCE_M  (SENS_MEAS2_START_FORCE_V << SENS_MEAS2_START_FORCE_S)
#define SENS_MEAS2_START_FORCE_V  0x00000001
#define SENS_MEAS2_START_FORCE_S  18

/* SENS_MEAS2_START_SAR : RW; bitpos: [17]; default: 0;
 * SAR ADC2 controller (in RTC) starts conversion  only active when
 * reg_meas2_start_force = 1
 */

#define SENS_MEAS2_START_SAR    (BIT(17))
#define SENS_MEAS2_START_SAR_M  (SENS_MEAS2_START_SAR_V << SENS_MEAS2_START_SAR_S)
#define SENS_MEAS2_START_SAR_V  0x00000001
#define SENS_MEAS2_START_SAR_S  17

/* SENS_MEAS2_DONE_SAR : R; bitpos: [16]; default: 0;
 * SAR ADC2 conversion done indication
 */

#define SENS_MEAS2_DONE_SAR    (BIT(16))
#define SENS_MEAS2_DONE_SAR_M  (SENS_MEAS2_DONE_SAR_V << SENS_MEAS2_DONE_SAR_S)
#define SENS_MEAS2_DONE_SAR_V  0x00000001
#define SENS_MEAS2_DONE_SAR_S  16

/* SENS_MEAS2_DATA_SAR : R; bitpos: [15:0]; default: 0;
 * SAR ADC2 data
 */

#define SENS_MEAS2_DATA_SAR    0x0000ffff
#define SENS_MEAS2_DATA_SAR_M  (SENS_MEAS2_DATA_SAR_V << SENS_MEAS2_DATA_SAR_S)
#define SENS_MEAS2_DATA_SAR_V  0x0000ffff
#define SENS_MEAS2_DATA_SAR_S  0

/* SENS_SAR_DAC_CTRL1_REG register */

#define SENS_SAR_DAC_CTRL1_REG (DR_REG_SENS_BASE + 0x98)

/* SENS_DAC_CLK_INV : RW; bitpos: [25]; default: 0;
 * 1: invert PDAC_CLK
 */

#define SENS_DAC_CLK_INV    (BIT(25))
#define SENS_DAC_CLK_INV_M  (SENS_DAC_CLK_INV_V << SENS_DAC_CLK_INV_S)
#define SENS_DAC_CLK_INV_V  0x00000001
#define SENS_DAC_CLK_INV_S  25

/* SENS_DAC_CLK_FORCE_HIGH : RW; bitpos: [24]; default: 0;
 * 1: force PDAC_CLK to high
 */

#define SENS_DAC_CLK_FORCE_HIGH    (BIT(24))
#define SENS_DAC_CLK_FORCE_HIGH_M  (SENS_DAC_CLK_FORCE_HIGH_V << SENS_DAC_CLK_FORCE_HIGH_S)
#define SENS_DAC_CLK_FORCE_HIGH_V  0x00000001
#define SENS_DAC_CLK_FORCE_HIGH_S  24

/* SENS_DAC_CLK_FORCE_LOW : RW; bitpos: [23]; default: 0;
 * 1: force PDAC_CLK to low
 */

#define SENS_DAC_CLK_FORCE_LOW    (BIT(23))
#define SENS_DAC_CLK_FORCE_LOW_M  (SENS_DAC_CLK_FORCE_LOW_V << SENS_DAC_CLK_FORCE_LOW_S)
#define SENS_DAC_CLK_FORCE_LOW_V  0x00000001
#define SENS_DAC_CLK_FORCE_LOW_S  23

/* SENS_DAC_DIG_FORCE : RW; bitpos: [22]; default: 0;
 * 1: DAC1 & DAC2 use DMA  0: DAC1 & DAC2 do not use DMA
 */

#define SENS_DAC_DIG_FORCE    (BIT(22))
#define SENS_DAC_DIG_FORCE_M  (SENS_DAC_DIG_FORCE_V << SENS_DAC_DIG_FORCE_S)
#define SENS_DAC_DIG_FORCE_V  0x00000001
#define SENS_DAC_DIG_FORCE_S  22

/* SENS_DEBUG_BIT_SEL : RW; bitpos: [21:17]; default: 0; */

#define SENS_DEBUG_BIT_SEL    0x0000001f
#define SENS_DEBUG_BIT_SEL_M  (SENS_DEBUG_BIT_SEL_V << SENS_DEBUG_BIT_SEL_S)
#define SENS_DEBUG_BIT_SEL_V  0x0000001f
#define SENS_DEBUG_BIT_SEL_S  17

/* SENS_SW_TONE_EN : RW; bitpos: [16]; default: 0;
 * 1: enable CW generator  0: disable CW generator
 */

#define SENS_SW_TONE_EN    (BIT(16))
#define SENS_SW_TONE_EN_M  (SENS_SW_TONE_EN_V << SENS_SW_TONE_EN_S)
#define SENS_SW_TONE_EN_V  0x00000001
#define SENS_SW_TONE_EN_S  16

/* SENS_SW_FSTEP : RW; bitpos: [15:0]; default: 0;
 * frequency step for CW generator  can be used to adjust the frequency
 */

#define SENS_SW_FSTEP    0x0000ffff
#define SENS_SW_FSTEP_M  (SENS_SW_FSTEP_V << SENS_SW_FSTEP_S)
#define SENS_SW_FSTEP_V  0x0000ffff
#define SENS_SW_FSTEP_S  0

/* SENS_SAR_DAC_CTRL2_REG register */

#define SENS_SAR_DAC_CTRL2_REG (DR_REG_SENS_BASE + 0x9c)

/* SENS_DAC_CW_EN2 : RW; bitpos: [25]; default: 1;
 * 1: to select CW generator as source to PDAC2_DAC[7:0]  0: to select
 * register reg_pdac2_dac[7:0] as source to PDAC2_DAC[7:0]
 */

#define SENS_DAC_CW_EN2    (BIT(25))
#define SENS_DAC_CW_EN2_M  (SENS_DAC_CW_EN2_V << SENS_DAC_CW_EN2_S)
#define SENS_DAC_CW_EN2_V  0x00000001
#define SENS_DAC_CW_EN2_S  25

/* SENS_DAC_CW_EN1 : RW; bitpos: [24]; default: 1;
 * 1: to select CW generator as source to PDAC1_DAC[7:0]  0: to select
 * register reg_pdac1_dac[7:0] as source to PDAC1_DAC[7:0]
 */

#define SENS_DAC_CW_EN1    (BIT(24))
#define SENS_DAC_CW_EN1_M  (SENS_DAC_CW_EN1_V << SENS_DAC_CW_EN1_S)
#define SENS_DAC_CW_EN1_V  0x00000001
#define SENS_DAC_CW_EN1_S  24

/* SENS_DAC_INV2 : RW; bitpos: [23:22]; default: 0;
 * 00: do not invert any bits  01: invert all bits  10: invert MSB  11:
 * invert all bits except MSB
 */

#define SENS_DAC_INV2    0x00000003
#define SENS_DAC_INV2_M  (SENS_DAC_INV2_V << SENS_DAC_INV2_S)
#define SENS_DAC_INV2_V  0x00000003
#define SENS_DAC_INV2_S  22

/* SENS_DAC_INV1 : RW; bitpos: [21:20]; default: 0;
 * 00: do not invert any bits  01: invert all bits  10: invert MSB  11:
 * invert all bits except MSB
 */

#define SENS_DAC_INV1    0x00000003
#define SENS_DAC_INV1_M  (SENS_DAC_INV1_V << SENS_DAC_INV1_S)
#define SENS_DAC_INV1_V  0x00000003
#define SENS_DAC_INV1_S  20

/* SENS_DAC_SCALE2 : RW; bitpos: [19:18]; default: 0;
 * 00: no scale  01: scale to 1/2  10: scale to 1/4  scale to 1/8
 */

#define SENS_DAC_SCALE2    0x00000003
#define SENS_DAC_SCALE2_M  (SENS_DAC_SCALE2_V << SENS_DAC_SCALE2_S)
#define SENS_DAC_SCALE2_V  0x00000003
#define SENS_DAC_SCALE2_S  18

/* SENS_DAC_SCALE1 : RW; bitpos: [17:16]; default: 0;
 * 00: no scale  01: scale to 1/2  10: scale to 1/4  scale to 1/8
 */

#define SENS_DAC_SCALE1    0x00000003
#define SENS_DAC_SCALE1_M  (SENS_DAC_SCALE1_V << SENS_DAC_SCALE1_S)
#define SENS_DAC_SCALE1_V  0x00000003
#define SENS_DAC_SCALE1_S  16

/* SENS_DAC_DC2 : RW; bitpos: [15:8]; default: 0;
 * DC offset for DAC2 CW generator
 */

#define SENS_DAC_DC2    0x000000ff
#define SENS_DAC_DC2_M  (SENS_DAC_DC2_V << SENS_DAC_DC2_S)
#define SENS_DAC_DC2_V  0x000000ff
#define SENS_DAC_DC2_S  8

/* SENS_DAC_DC1 : RW; bitpos: [7:0]; default: 0;
 * DC offset for DAC1 CW generator
 */

#define SENS_DAC_DC1    0x000000ff
#define SENS_DAC_DC1_M  (SENS_DAC_DC1_V << SENS_DAC_DC1_S)
#define SENS_DAC_DC1_V  0x000000ff
#define SENS_DAC_DC1_S  0

/* SENS_SAR_MEAS_CTRL2_REG register */

#define SENS_SAR_MEAS_CTRL2_REG (DR_REG_SENS_BASE + 0xa0)

/* SENS_AMP_SHORT_REF_GND_FORCE : RW; bitpos: [18:17]; default: 0; */

#define SENS_AMP_SHORT_REF_GND_FORCE    0x00000003
#define SENS_AMP_SHORT_REF_GND_FORCE_M  (SENS_AMP_SHORT_REF_GND_FORCE_V << SENS_AMP_SHORT_REF_GND_FORCE_S)
#define SENS_AMP_SHORT_REF_GND_FORCE_V  0x00000003
#define SENS_AMP_SHORT_REF_GND_FORCE_S  17

/* SENS_AMP_SHORT_REF_FORCE : RW; bitpos: [16:15]; default: 0; */

#define SENS_AMP_SHORT_REF_FORCE    0x00000003
#define SENS_AMP_SHORT_REF_FORCE_M  (SENS_AMP_SHORT_REF_FORCE_V << SENS_AMP_SHORT_REF_FORCE_S)
#define SENS_AMP_SHORT_REF_FORCE_V  0x00000003
#define SENS_AMP_SHORT_REF_FORCE_S  15

/* SENS_AMP_RST_FB_FORCE : RW; bitpos: [14:13]; default: 0; */

#define SENS_AMP_RST_FB_FORCE    0x00000003
#define SENS_AMP_RST_FB_FORCE_M  (SENS_AMP_RST_FB_FORCE_V << SENS_AMP_RST_FB_FORCE_S)
#define SENS_AMP_RST_FB_FORCE_V  0x00000003
#define SENS_AMP_RST_FB_FORCE_S  13

/* SENS_SAR2_RSTB_FORCE : RW; bitpos: [12:11]; default: 0; */

#define SENS_SAR2_RSTB_FORCE    0x00000003
#define SENS_SAR2_RSTB_FORCE_M  (SENS_SAR2_RSTB_FORCE_V << SENS_SAR2_RSTB_FORCE_S)
#define SENS_SAR2_RSTB_FORCE_V  0x00000003
#define SENS_SAR2_RSTB_FORCE_S  11

/* SENS_SAR_RSTB_FSM_IDLE : RW; bitpos: [10]; default: 0; */

#define SENS_SAR_RSTB_FSM_IDLE    (BIT(10))
#define SENS_SAR_RSTB_FSM_IDLE_M  (SENS_SAR_RSTB_FSM_IDLE_V << SENS_SAR_RSTB_FSM_IDLE_S)
#define SENS_SAR_RSTB_FSM_IDLE_V  0x00000001
#define SENS_SAR_RSTB_FSM_IDLE_S  10

/* SENS_XPD_SAR_FSM_IDLE : RW; bitpos: [9]; default: 0; */

#define SENS_XPD_SAR_FSM_IDLE    (BIT(9))
#define SENS_XPD_SAR_FSM_IDLE_M  (SENS_XPD_SAR_FSM_IDLE_V << SENS_XPD_SAR_FSM_IDLE_S)
#define SENS_XPD_SAR_FSM_IDLE_V  0x00000001
#define SENS_XPD_SAR_FSM_IDLE_S  9

/* SENS_AMP_SHORT_REF_GND_FSM_IDLE : RW; bitpos: [8]; default: 0; */

#define SENS_AMP_SHORT_REF_GND_FSM_IDLE    (BIT(8))
#define SENS_AMP_SHORT_REF_GND_FSM_IDLE_M  (SENS_AMP_SHORT_REF_GND_FSM_IDLE_V << SENS_AMP_SHORT_REF_GND_FSM_IDLE_S)
#define SENS_AMP_SHORT_REF_GND_FSM_IDLE_V  0x00000001
#define SENS_AMP_SHORT_REF_GND_FSM_IDLE_S  8

/* SENS_AMP_SHORT_REF_FSM_IDLE : RW; bitpos: [7]; default: 0; */

#define SENS_AMP_SHORT_REF_FSM_IDLE    (BIT(7))
#define SENS_AMP_SHORT_REF_FSM_IDLE_M  (SENS_AMP_SHORT_REF_FSM_IDLE_V << SENS_AMP_SHORT_REF_FSM_IDLE_S)
#define SENS_AMP_SHORT_REF_FSM_IDLE_V  0x00000001
#define SENS_AMP_SHORT_REF_FSM_IDLE_S  7

/* SENS_AMP_RST_FB_FSM_IDLE : RW; bitpos: [6]; default: 0; */

#define SENS_AMP_RST_FB_FSM_IDLE    (BIT(6))
#define SENS_AMP_RST_FB_FSM_IDLE_M  (SENS_AMP_RST_FB_FSM_IDLE_V << SENS_AMP_RST_FB_FSM_IDLE_S)
#define SENS_AMP_RST_FB_FSM_IDLE_V  0x00000001
#define SENS_AMP_RST_FB_FSM_IDLE_S  6

/* SENS_XPD_SAR_AMP_FSM_IDLE : RW; bitpos: [5]; default: 0; */

#define SENS_XPD_SAR_AMP_FSM_IDLE    (BIT(5))
#define SENS_XPD_SAR_AMP_FSM_IDLE_M  (SENS_XPD_SAR_AMP_FSM_IDLE_V << SENS_XPD_SAR_AMP_FSM_IDLE_S)
#define SENS_XPD_SAR_AMP_FSM_IDLE_V  0x00000001
#define SENS_XPD_SAR_AMP_FSM_IDLE_S  5

/* SENS_SAR1_DAC_XPD_FSM_IDLE : RW; bitpos: [4]; default: 0; */

#define SENS_SAR1_DAC_XPD_FSM_IDLE    (BIT(4))
#define SENS_SAR1_DAC_XPD_FSM_IDLE_M  (SENS_SAR1_DAC_XPD_FSM_IDLE_V << SENS_SAR1_DAC_XPD_FSM_IDLE_S)
#define SENS_SAR1_DAC_XPD_FSM_IDLE_V  0x00000001
#define SENS_SAR1_DAC_XPD_FSM_IDLE_S  4

/* SENS_SAR1_DAC_XPD_FSM : RW; bitpos: [3:0]; default: 3; */

#define SENS_SAR1_DAC_XPD_FSM    0x0000000f
#define SENS_SAR1_DAC_XPD_FSM_M  (SENS_SAR1_DAC_XPD_FSM_V << SENS_SAR1_DAC_XPD_FSM_S)
#define SENS_SAR1_DAC_XPD_FSM_V  0x0000000f
#define SENS_SAR1_DAC_XPD_FSM_S  0

/* SENS_SAR_NOUSE_REG register */

#define SENS_SAR_NOUSE_REG (DR_REG_SENS_BASE + 0xf8)

/* SENS_SAR_NOUSE : RW; bitpos: [31:0]; default: 0; */

#define SENS_SAR_NOUSE    0xffffffff
#define SENS_SAR_NOUSE_M  (SENS_SAR_NOUSE_V << SENS_SAR_NOUSE_S)
#define SENS_SAR_NOUSE_V  0xffffffff
#define SENS_SAR_NOUSE_S  0

/* SENS_SARDATE_REG register */

#define SENS_SARDATE_REG (DR_REG_SENS_BASE + 0xfc)

/* SENS_SAR_DATE : RW; bitpos: [27:0]; default: 23089536; */

#define SENS_SAR_DATE    0x0fffffff
#define SENS_SAR_DATE_M  (SENS_SAR_DATE_V << SENS_SAR_DATE_S)
#define SENS_SAR_DATE_V  0x0fffffff
#define SENS_SAR_DATE_S  0

#endif /* __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_SENS_H */
