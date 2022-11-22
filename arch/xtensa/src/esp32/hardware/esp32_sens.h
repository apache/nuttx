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

/* SENS_SAR1_DATA_INV : R/W; bitpos: [28]; default: 0; */

#define SENS_SAR1_DATA_INV    (BIT(28))
#define SENS_SAR1_DATA_INV_M  (SENS_SAR1_DATA_INV_V << SENS_SAR1_DATA_INV_S)
#define SENS_SAR1_DATA_INV_V  0x1
#define SENS_SAR1_DATA_INV_S  28

/* SENS_SAR1_DIG_FORCE : R/W; bitpos: [27]; default: 0; */

#define SENS_SAR1_DIG_FORCE    (BIT(27))
#define SENS_SAR1_DIG_FORCE_M  (SENS_SAR1_DIG_FORCE_V << \
                                SENS_SAR1_DIG_FORCE_S)
#define SENS_SAR1_DIG_FORCE_V  0x1
#define SENS_SAR1_DIG_FORCE_S  27

/* SENS_SAR1_SAMPLE_BIT : R/W; bitpos: [17:16]; default: 3; */

#define SENS_SAR1_SAMPLE_BIT    0x00000003
#define SENS_SAR1_SAMPLE_BIT_M  (SENS_SAR1_SAMPLE_BIT_V << \
                                 SENS_SAR1_SAMPLE_BIT_S)
#define SENS_SAR1_SAMPLE_BIT_V  0x3
#define SENS_SAR1_SAMPLE_BIT_S  16

/* SENS_SAR1_SAMPLE_CYCLE : R/W; bitpos: [15:8]; default: 9; */

#define SENS_SAR1_SAMPLE_CYCLE    0x000000ff
#define SENS_SAR1_SAMPLE_CYCLE_M  (SENS_SAR1_SAMPLE_CYCLE_V << \
                                   SENS_SAR1_SAMPLE_CYCLE_S)
#define SENS_SAR1_SAMPLE_CYCLE_V  0xff
#define SENS_SAR1_SAMPLE_CYCLE_S  8

/* SENS_SAR1_CLK_DIV : R/W; bitpos: [7:0]; default: 2; */

#define SENS_SAR1_CLK_DIV    0x000000ff
#define SENS_SAR1_CLK_DIV_M  (SENS_SAR1_CLK_DIV_V << SENS_SAR1_CLK_DIV_S)
#define SENS_SAR1_CLK_DIV_V  0xff
#define SENS_SAR1_CLK_DIV_S  0

/* SENS_ULP_CP_SLEEP_CYC0_REG register */

#define SENS_ULP_CP_SLEEP_CYC0_REG (DR_REG_SENS_BASE + 0x18)

/* SENS_ULP_CP_SLEEP_CYC0 : R/W; bitpos: [31:0]; default: 200; */

#define SENS_ULP_CP_SLEEP_CYC0    0xffffffff
#define SENS_ULP_CP_SLEEP_CYC0_M  (SENS_ULP_CP_SLEEP_CYC0_V << \
                                   SENS_ULP_CP_SLEEP_CYC0_S)
#define SENS_ULP_CP_SLEEP_CYC0_V  0xffffffff
#define SENS_ULP_CP_SLEEP_CYC0_S  0

/* SENS_SAR_START_FORCE_REG register */

#define SENS_SAR_START_FORCE_REG (DR_REG_SENS_BASE + 0x2c)

/* SENS_SAR1_STOP : R/W; bitpos: [23]; default: 0; */

#define SENS_SAR1_STOP    (BIT(23))
#define SENS_SAR1_STOP_M  (SENS_SAR1_STOP_V << SENS_SAR1_STOP_S)
#define SENS_SAR1_STOP_V  0x1
#define SENS_SAR1_STOP_S  23

/* SENS_SAR2_STOP : R/W; bitpos: [22]; default: 0; */

#define SENS_SAR2_STOP    (BIT(22))
#define SENS_SAR2_STOP_M  (SENS_SAR2_STOP_V << SENS_SAR2_STOP_S)
#define SENS_SAR2_STOP_V  0x1
#define SENS_SAR2_STOP_S  22

/* SENS_PC_INIT : R/W; bitpos: [21:11]; default: 0; */

#define SENS_PC_INIT    0x000007ff
#define SENS_PC_INIT_M  (SENS_PC_INIT_V << SENS_PC_INIT_S)
#define SENS_PC_INIT_V  0x7ff
#define SENS_PC_INIT_S  11

/* SENS_ULP_CP_START_TOP : R/W; bitpos: [9]; default: 0; */

#define SENS_ULP_CP_START_TOP    (BIT(9))
#define SENS_ULP_CP_START_TOP_M  (SENS_ULP_CP_START_TOP_V << \
                                  SENS_ULP_CP_START_TOP_S)
#define SENS_ULP_CP_START_TOP_V  0x1
#define SENS_ULP_CP_START_TOP_S  9

/* SENS_ULP_CP_FORCE_START_TOP : R/W; bitpos: [8]; default: 0; */

#define SENS_ULP_CP_FORCE_START_TOP    (BIT(8))
#define SENS_ULP_CP_FORCE_START_TOP_M  (SENS_ULP_CP_FORCE_START_TOP_V << \
                                        SENS_ULP_CP_FORCE_START_TOP_S)
#define SENS_ULP_CP_FORCE_START_TOP_V  0x1
#define SENS_ULP_CP_FORCE_START_TOP_S  8

/* SENS_SAR2_PWDET_CCT : R/W; bitpos: [7:5]; default: 0; */

#define SENS_SAR2_PWDET_CCT    0x00000007
#define SENS_SAR2_PWDET_CCT_M  (SENS_SAR2_PWDET_CCT_V << \
                                SENS_SAR2_PWDET_CCT_S)
#define SENS_SAR2_PWDET_CCT_V  0x7
#define SENS_SAR2_PWDET_CCT_S  5

/* SENS_SAR2_EN_TEST : R/W; bitpos: [4]; default: 0; */

#define SENS_SAR2_EN_TEST    (BIT(4))
#define SENS_SAR2_EN_TEST_M  (SENS_SAR2_EN_TEST_V << SENS_SAR2_EN_TEST_S)
#define SENS_SAR2_EN_TEST_V  0x1
#define SENS_SAR2_EN_TEST_S  4

/* SENS_SAR2_BIT_WIDTH : R/W; bitpos: [3:2]; default: 3; */

#define SENS_SAR2_BIT_WIDTH    0x00000003
#define SENS_SAR2_BIT_WIDTH_M  (SENS_SAR2_BIT_WIDTH_V << \
                                SENS_SAR2_BIT_WIDTH_S)
#define SENS_SAR2_BIT_WIDTH_V  0x3
#define SENS_SAR2_BIT_WIDTH_S  2

/* SENS_SAR1_BIT_WIDTH : R/W; bitpos: [1:0]; default: 3; */

#define SENS_SAR1_BIT_WIDTH    0x00000003
#define SENS_SAR1_BIT_WIDTH_M  (SENS_SAR1_BIT_WIDTH_V << \
                                SENS_SAR1_BIT_WIDTH_S)
#define SENS_SAR1_BIT_WIDTH_V  0x3
#define SENS_SAR1_BIT_WIDTH_S  0

/* SENS_SAR_ATTEN1_REG register */

#define SENS_SAR_ATTEN1_REG (DR_REG_SENS_BASE + 0x34)

/* SENS_SAR_ATTEN1 : R/W; bitpos: [31:0]; default: 0xffffffff; */

#define SENS_SAR_ATTEN1    0xffffffff
#define SENS_SAR_ATTEN1_M  (SENS_SAR_ATTEN1_V << SENS_SAR_ATTEN1_S)
#define SENS_SAR_ATTEN1_V  0xffffffff
#define SENS_SAR_ATTEN1_S  0

/* SENS_SAR_ATTEN2_REG register */

#define SENS_SAR_ATTEN2_REG (DR_REG_SENS_BASE + 0x38)

/* SENS_SAR_ATTEN2 : R/W; bitpos: [31:0]; default: 0xffffffff; */

#define SENS_SAR_ATTEN2    0xffffffff
#define SENS_SAR_ATTEN2_M  (SENS_SAR_ATTEN2_V << SENS_SAR_ATTEN2_S)
#define SENS_SAR_ATTEN2_V  0xffffffff
#define SENS_SAR_ATTEN2_S  0

/* SENS_SAR_MEAS_START1_REG register */

#define SENS_SAR_MEAS_START1_REG (DR_REG_SENS_BASE + 0x54)

/* SENS_SAR1_EN_PAD_FORCE : R/W; bitpos: [31]; default: 0; */

#define SENS_SAR1_EN_PAD_FORCE    (BIT(31))
#define SENS_SAR1_EN_PAD_FORCE_M  (SENS_SAR1_EN_PAD_FORCE_V << \
                                   SENS_SAR1_EN_PAD_FORCE_S)
#define SENS_SAR1_EN_PAD_FORCE_V  0x1
#define SENS_SAR1_EN_PAD_FORCE_S  31

/* SENS_SAR1_EN_PAD : R/W; bitpos: [30:19]; default: 0; */

#define SENS_SAR1_EN_PAD    0x000003ff
#define SENS_SAR1_EN_PAD_M  (SENS_SAR1_EN_PAD_V << SENS_SAR1_EN_PAD_S)
#define SENS_SAR1_EN_PAD_V  0x3ff
#define SENS_SAR1_EN_PAD_S  19

/* SENS_MEAS1_START_FORCE : R/W; bitpos: [18]; default: 0; */

#define SENS_MEAS1_START_FORCE    (BIT(18))
#define SENS_MEAS1_START_FORCE_M  (SENS_MEAS1_START_FORCE_V << \
                                   SENS_MEAS1_START_FORCE_S)
#define SENS_MEAS1_START_FORCE_V  0x1
#define SENS_MEAS1_START_FORCE_S  18

/* SENS_MEAS1_START_SAR : R/W; bitpos: [17]; default: 0; */

#define SENS_MEAS1_START_SAR    (BIT(17))
#define SENS_MEAS1_START_SAR_M  (SENS_MEAS1_START_SAR_V << \
                                 SENS_MEAS1_START_SAR_S)
#define SENS_MEAS1_START_SAR_V  0x1
#define SENS_MEAS1_START_SAR_S  17

/* SENS_MEAS1_DONE_SAR : RO; bitpos: [16]; default: 0; */

#define SENS_MEAS1_DONE_SAR    (BIT(16))
#define SENS_MEAS1_DONE_SAR_M  (SENS_MEAS1_DONE_SAR_V << \
                                SENS_MEAS1_DONE_SAR_S)
#define SENS_MEAS1_DONE_SAR_V  0x1
#define SENS_MEAS1_DONE_SAR_S  16

/* SENS_MEAS1_DATA_SAR : RO; bitpos: [15:0]; default: 0; */

#define SENS_MEAS1_DATA_SAR    0x0000ffff
#define SENS_MEAS1_DATA_SAR_M  (SENS_MEAS1_DATA_SAR_V << \
                                SENS_MEAS1_DATA_SAR_S)
#define SENS_MEAS1_DATA_SAR_V  0xffff
#define SENS_MEAS1_DATA_SAR_S  0

/* SENS_SAR_TOUCH_CTRL1_REG register */

#define SENS_SAR_TOUCH_CTRL1_REG (DR_REG_SENS_BASE + 0x58)

/* SENS_HALL_PHASE_FORCE : R/W; bitpos: [27]; default: 0; */

#define SENS_HALL_PHASE_FORCE    (BIT(27))
#define SENS_HALL_PHASE_FORCE_M  (SENS_HALL_PHASE_FORCE_V << \
                                  SENS_HALL_PHASE_FORCE_S)
#define SENS_HALL_PHASE_FORCE_V  0x1
#define SENS_HALL_PHASE_FORCE_S  27

/* SENS_XPD_HALL_FORCE : R/W; bitpos: [26]; default: 0; */

#define SENS_XPD_HALL_FORCE    (BIT(26))
#define SENS_XPD_HALL_FORCE_M  (SENS_XPD_HALL_FORCE_V << \
                                SENS_XPD_HALL_FORCE_S)
#define SENS_XPD_HALL_FORCE_V  0x1
#define SENS_XPD_HALL_FORCE_S  26

/* SENS_TOUCH_OUT_1EN : R/W; bitpos: [25]; default: 1; */

#define SENS_TOUCH_OUT_1EN    (BIT(25))
#define SENS_TOUCH_OUT_1EN_M  (SENS_TOUCH_OUT_1EN_V << SENS_TOUCH_OUT_1EN_S)
#define SENS_TOUCH_OUT_1EN_V  0x1
#define SENS_TOUCH_OUT_1EN_S  25

/* SENS_TOUCH_OUT_SEL : R/W; bitpos: [24]; default: 0; */

#define SENS_TOUCH_OUT_SEL    (BIT(24))
#define SENS_TOUCH_OUT_SEL_M  (SENS_TOUCH_OUT_SEL_V << SENS_TOUCH_OUT_SEL_S)
#define SENS_TOUCH_OUT_SEL_V  0x1
#define SENS_TOUCH_OUT_SEL_S  24

/* SENS_TOUCH_XPD_WAIT : R/W; bitpos: [23:16]; default: 4; */

#define SENS_TOUCH_XPD_WAIT    0x00000007
#define SENS_TOUCH_XPD_WAIT_M  (SENS_TOUCH_XPD_WAIT_V << \
                                SENS_TOUCH_XPD_WAIT_S)
#define SENS_TOUCH_XPD_WAIT_V  0x7
#define SENS_TOUCH_XPD_WAIT_S  16

/* SENS_TOUCH_MEAS_DELAY : R/W; bitpos: [15:0]; default: 0x1000; */

#define SENS_TOUCH_MEAS_DELAY    0x0000ffff
#define SENS_TOUCH_MEAS_DELAY_M  (SENS_TOUCH_MEAS_DELAY_V << \
                                  SENS_TOUCH_MEAS_DELAY_S)
#define SENS_TOUCH_MEAS_DELAY_V  0xffff
#define SENS_TOUCH_MEAS_DELAY_S  0

/* SENS_SAR_TOUCH_THRES1_REG register */

#define SENS_SAR_TOUCH_THRES1_REG (DR_REG_SENS_BASE + 0x5c)

/* SENS_TOUCH_OUT_TH0 : R/W; bitpos: [31:16]; default: 0; */

#define SENS_TOUCH_OUT_TH0    0x0000ffff
#define SENS_TOUCH_OUT_TH0_M  (SENS_TOUCH_OUT_TH0_V << SENS_TOUCH_OUT_TH0_S)
#define SENS_TOUCH_OUT_TH0_V  0xffff
#define SENS_TOUCH_OUT_TH0_S  16

/* SENS_TOUCH_OUT_TH1 : R/W; bitpos: [15:0]; default: 0; */

#define SENS_TOUCH_OUT_TH1    0x0000ffff
#define SENS_TOUCH_OUT_TH1_M  (SENS_TOUCH_OUT_TH1_V << SENS_TOUCH_OUT_TH1_S)
#define SENS_TOUCH_OUT_TH1_V  0xffff
#define SENS_TOUCH_OUT_TH1_S  0

/* SENS_SAR_TOUCH_THRES2_REG register */

#define SENS_SAR_TOUCH_THRES2_REG (DR_REG_SENS_BASE + 0x60)

/* SENS_TOUCH_OUT_TH2 : R/W; bitpos: [31:16]; default: 0; */

#define SENS_TOUCH_OUT_TH2    0x0000ffff
#define SENS_TOUCH_OUT_TH2_M  (SENS_TOUCH_OUT_TH2_V << SENS_TOUCH_OUT_TH2_S)
#define SENS_TOUCH_OUT_TH2_V  0xffff
#define SENS_TOUCH_OUT_TH2_S  16

/* SENS_TOUCH_OUT_TH3 : R/W; bitpos: [15:0]; default: 0; */

#define SENS_TOUCH_OUT_TH3    0x0000ffff
#define SENS_TOUCH_OUT_TH3_M  (SENS_TOUCH_OUT_TH3_V << SENS_TOUCH_OUT_TH3_S)
#define SENS_TOUCH_OUT_TH3_V  0xffff
#define SENS_TOUCH_OUT_TH3_S  0

/* SENS_SAR_TOUCH_THRES3_REG register */

#define SENS_SAR_TOUCH_THRES3_REG (DR_REG_SENS_BASE + 0x64)

/* SENS_TOUCH_OUT_TH4 : R/W; bitpos: [31:16]; default: 0; */

#define SENS_TOUCH_OUT_TH4    0x0000ffff
#define SENS_TOUCH_OUT_TH4_M  (SENS_TOUCH_OUT_TH4_V << SENS_TOUCH_OUT_TH4_S)
#define SENS_TOUCH_OUT_TH4_V  0xffff
#define SENS_TOUCH_OUT_TH4_S  16

/* SENS_TOUCH_OUT_TH5 : R/W; bitpos: [15:0]; default: 0; */

#define SENS_TOUCH_OUT_TH5    0x0000ffff
#define SENS_TOUCH_OUT_TH5_M  (SENS_TOUCH_OUT_TH5_V << SENS_TOUCH_OUT_TH5_S)
#define SENS_TOUCH_OUT_TH5_V  0xffff
#define SENS_TOUCH_OUT_TH5_S  0

/* SENS_SAR_TOUCH_THRES4_REG register */

#define SENS_SAR_TOUCH_THRES4_REG (DR_REG_SENS_BASE + 0x68)

/* SENS_TOUCH_OUT_TH6 : R/W; bitpos: [31:16]; default: 0; */

#define SENS_TOUCH_OUT_TH6    0x0000ffff
#define SENS_TOUCH_OUT_TH6_M  (SENS_TOUCH_OUT_TH6_V << SENS_TOUCH_OUT_TH6_S)
#define SENS_TOUCH_OUT_TH6_V  0xffff
#define SENS_TOUCH_OUT_TH6_S  16

/* SENS_TOUCH_OUT_TH7 : R/W; bitpos: [15:0]; default: 0; */

#define SENS_TOUCH_OUT_TH7    0x0000ffff
#define SENS_TOUCH_OUT_TH7_M  (SENS_TOUCH_OUT_TH7_V << SENS_TOUCH_OUT_TH7_S)
#define SENS_TOUCH_OUT_TH7_V  0xffff
#define SENS_TOUCH_OUT_TH7_S  0

/* SENS_SAR_TOUCH_THRES5_REG register */

#define SENS_SAR_TOUCH_THRES5_REG (DR_REG_SENS_BASE + 0x6c)

/* SENS_TOUCH_OUT_TH8 : R/W; bitpos: [31:16]; default: 0; */

#define SENS_TOUCH_OUT_TH8    0x0000ffff
#define SENS_TOUCH_OUT_TH8_M  (SENS_TOUCH_OUT_TH8_V << SENS_TOUCH_OUT_TH8_S)
#define SENS_TOUCH_OUT_TH8_V  0xffff
#define SENS_TOUCH_OUT_TH8_S  16

/* SENS_TOUCH_OUT_TH9 : R/W; bitpos: [15:0]; default: 0; */

#define SENS_TOUCH_OUT_TH9    0x0000ffff
#define SENS_TOUCH_OUT_TH9_M  (SENS_TOUCH_OUT_TH9_V << SENS_TOUCH_OUT_TH9_S)
#define SENS_TOUCH_OUT_TH9_V  0xffff
#define SENS_TOUCH_OUT_TH9_S  0

/* SENS_SAR_TOUCH_OUT1_REG register */

#define SENS_SAR_TOUCH_OUT1_REG (DR_REG_SENS_BASE + 0x70)

/* SENS_TOUCH_MEAS_OUT0 : RO; bitpos: [31:16]; default: 0; */

#define SENS_TOUCH_MEAS_OUT0    0x0000ffff
#define SENS_TOUCH_MEAS_OUT0_M  (SENS_TOUCH_MEAS_OUT0_V << \
                                 SENS_TOUCH_MEAS_OUT0_S)
#define SENS_TOUCH_MEAS_OUT0_V  0xffff
#define SENS_TOUCH_MEAS_OUT0_S  16

/* SENS_TOUCH_MEAS_OUT1 : RO; bitpos: [15:0]; default: 0; */

#define SENS_TOUCH_MEAS_OUT1    0x0000ffff
#define SENS_TOUCH_MEAS_OUT1_M  (SENS_TOUCH_MEAS_OUT1_V << \
                                 SENS_TOUCH_MEAS_OUT1_S)
#define SENS_TOUCH_MEAS_OUT1_V  0xffff
#define SENS_TOUCH_MEAS_OUT1_S  0

/* SENS_SAR_TOUCH_OUT2_REG register */

#define SENS_SAR_TOUCH_OUT2_REG (DR_REG_SENS_BASE + 0x74)

/* SENS_TOUCH_MEAS_OUT2 : RO; bitpos: [31:16]; default: 0; */

#define SENS_TOUCH_MEAS_OUT2    0x0000ffff
#define SENS_TOUCH_MEAS_OUT2_M  (SENS_TOUCH_MEAS_OUT2_V << \
                                 SENS_TOUCH_MEAS_OUT2_S)
#define SENS_TOUCH_MEAS_OUT2_V  0xffff
#define SENS_TOUCH_MEAS_OUT2_S  16

/* SENS_TOUCH_MEAS_OUT3 : RO; bitpos: [15:0]; default: 0; */

#define SENS_TOUCH_MEAS_OUT3    0x0000ffff
#define SENS_TOUCH_MEAS_OUT3_M  (SENS_TOUCH_MEAS_OUT3_V << \
                                 SENS_TOUCH_MEAS_OUT3_S)
#define SENS_TOUCH_MEAS_OUT3_V  0xffff
#define SENS_TOUCH_MEAS_OUT3_S  0

/* SENS_SAR_TOUCH_OUT3_REG register */

#define SENS_SAR_TOUCH_OUT3_REG (DR_REG_SENS_BASE + 0x78)

/* SENS_TOUCH_MEAS_OUT4 : RO; bitpos: [31:16]; default: 0; */

#define SENS_TOUCH_MEAS_OUT4    0x0000ffff
#define SENS_TOUCH_MEAS_OUT4_M  (SENS_TOUCH_MEAS_OUT4_V << \
                                 SENS_TOUCH_MEAS_OUT4_S)
#define SENS_TOUCH_MEAS_OUT4_V  0xffff
#define SENS_TOUCH_MEAS_OUT4_S  16

/* SENS_TOUCH_MEAS_OUT5 : RO; bitpos: [15:0]; default: 0; */

#define SENS_TOUCH_MEAS_OUT5    0x0000ffff
#define SENS_TOUCH_MEAS_OUT5_M  (SENS_TOUCH_MEAS_OUT5_V << \
                                 SENS_TOUCH_MEAS_OUT5_S)
#define SENS_TOUCH_MEAS_OUT5_V  0xffff
#define SENS_TOUCH_MEAS_OUT5_S  0

/* SENS_SAR_TOUCH_OUT4_REG register */

#define SENS_SAR_TOUCH_OUT4_REG (DR_REG_SENS_BASE + 0x7c)

/* SENS_TOUCH_MEAS_OUT6 : RO; bitpos: [31:16]; default: 0; */

#define SENS_TOUCH_MEAS_OUT6    0x0000ffff
#define SENS_TOUCH_MEAS_OUT6_M  (SENS_TOUCH_MEAS_OUT6_V << \
                                 SENS_TOUCH_MEAS_OUT6_S)
#define SENS_TOUCH_MEAS_OUT6_V  0xffff
#define SENS_TOUCH_MEAS_OUT6_S  16

/* SENS_TOUCH_MEAS_OUT7 : RO; bitpos: [15:0]; default: 0; */

#define SENS_TOUCH_MEAS_OUT7    0x0000ffff
#define SENS_TOUCH_MEAS_OUT7_M  (SENS_TOUCH_MEAS_OUT7_V << \
                                 SENS_TOUCH_MEAS_OUT7_S)
#define SENS_TOUCH_MEAS_OUT7_V  0xffff
#define SENS_TOUCH_MEAS_OUT7_S  0

/* SENS_SAR_TOUCH_OUT5_REG register */

#define SENS_SAR_TOUCH_OUT5_REG (DR_REG_SENS_BASE + 0x80)

/* SENS_TOUCH_MEAS_OUT8 : RO; bitpos: [31:16]; default: 0; */

#define SENS_TOUCH_MEAS_OUT8    0x0000ffff
#define SENS_TOUCH_MEAS_OUT8_M  (SENS_TOUCH_MEAS_OUT8_V << \
                                 SENS_TOUCH_MEAS_OUT8_S)
#define SENS_TOUCH_MEAS_OUT8_V  0xffff
#define SENS_TOUCH_MEAS_OUT8_S  16

/* SENS_TOUCH_MEAS_OUT9 : RO; bitpos: [15:0]; default: 0; */

#define SENS_TOUCH_MEAS_OUT9    0x0000ffff
#define SENS_TOUCH_MEAS_OUT9_M  (SENS_TOUCH_MEAS_OUT9_V << \
                                 SENS_TOUCH_MEAS_OUT9_S)
#define SENS_TOUCH_MEAS_OUT9_V  0xffff
#define SENS_TOUCH_MEAS_OUT9_S  0

/* SENS_SAR_TOUCH_CTRL2_REG register */

#define SENS_SAR_TOUCH_CTRL2_REG (DR_REG_SENS_BASE + 0x84)

/* SENS_TOUCH_MEAS_EN_CLR : WO; bitpos: [30]; default: 0; */

#define SENS_TOUCH_MEAS_EN_CLR    (BIT(30))
#define SENS_TOUCH_MEAS_EN_CLR_M  (SENS_TOUCH_MEAS_EN_CLR_V << \
                                   SENS_TOUCH_MEAS_EN_CLR_S)
#define SENS_TOUCH_MEAS_EN_CLR_V  0x1
#define SENS_TOUCH_MEAS_EN_CLR_S  30

/* SENS_TOUCH_SLEEP_CYCLES : R/W; bitpos: [29:14]; default: 0x100; */

#define SENS_TOUCH_SLEEP_CYCLES    0x0000ffff
#define SENS_TOUCH_SLEEP_CYCLES_M  (SENS_TOUCH_SLEEP_CYCLES_V << \
                                    SENS_TOUCH_SLEEP_CYCLES_S)
#define SENS_TOUCH_SLEEP_CYCLES_V  0xffff
#define SENS_TOUCH_SLEEP_CYCLES_S  14

/* SENS_TOUCH_START_FORCE : R/W; bitpos: [13]; default: 0; */

#define SENS_TOUCH_START_FORCE    (BIT(13))
#define SENS_TOUCH_START_FORCE_M  (SENS_TOUCH_START_FORCE_V << \
                                   SENS_TOUCH_START_FORCE_S)
#define SENS_TOUCH_START_FORCE_V  0x1
#define SENS_TOUCH_START_FORCE_S  13

/* SENS_TOUCH_START_EN : R/W; bitpos: [12]; default: 0; */

#define SENS_TOUCH_START_EN    (BIT(12))
#define SENS_TOUCH_START_EN_M  (SENS_TOUCH_START_EN_V << \
                                SENS_TOUCH_START_EN_S)
#define SENS_TOUCH_START_EN_V  0x1
#define SENS_TOUCH_START_EN_S  12

/* SENS_TOUCH_START_FSM_EN : R/W; bitpos: [11]; default: 1; */

#define SENS_TOUCH_START_FSM_EN    (BIT(11))
#define SENS_TOUCH_START_FSM_EN_M  (SENS_TOUCH_START_FSM_EN_V << \
                                    SENS_TOUCH_START_FSM_EN_S)
#define SENS_TOUCH_START_FSM_EN_V  0x1
#define SENS_TOUCH_START_FSM_EN_S  11

/* SENS_TOUCH_MEAS_DONE : RO; bitpos: [10]; default: 0; */

#define SENS_TOUCH_MEAS_DONE    (BIT(10))
#define SENS_TOUCH_MEAS_DONE_M  (SENS_TOUCH_MEAS_DONE_V << \
                                 SENS_TOUCH_MEAS_DONE_S)
#define SENS_TOUCH_MEAS_DONE_V  0x1
#define SENS_TOUCH_MEAS_DONE_S  10

/* SENS_TOUCH_MEAS_EN : RO; bitpos: [9:0]; default: 0; */

#define SENS_TOUCH_MEAS_EN    0x000003ff
#define SENS_TOUCH_MEAS_EN_M  (SENS_TOUCH_MEAS_EN_V << SENS_TOUCH_MEAS_EN_S)
#define SENS_TOUCH_MEAS_EN_V  0x3ff
#define SENS_TOUCH_MEAS_EN_S  0

/* SENS_SAR_TOUCH_ENABLE_REG register */

#define SENS_SAR_TOUCH_ENABLE_REG (DR_REG_SENS_BASE + 0x8c)

/* SENS_TOUCH_PAD_OUTEN1 : R/W; bitpos: [29:20]; default: 0x3ff; */

#define SENS_TOUCH_PAD_OUTEN1    0x000003ff
#define SENS_TOUCH_PAD_OUTEN1_M  (SENS_TOUCH_PAD_OUTEN1_V << \
                                  SENS_TOUCH_PAD_OUTEN1_S)
#define SENS_TOUCH_PAD_OUTEN1_V  0x3ff
#define SENS_TOUCH_PAD_OUTEN1_S  20

/* SENS_TOUCH_PAD_OUTEN2 : R/W; bitpos: [19:10]; default: 0x3ff; */

#define SENS_TOUCH_PAD_OUTEN2    0x000003ff
#define SENS_TOUCH_PAD_OUTEN2_M  (SENS_TOUCH_PAD_OUTEN2_V << \
                                  SENS_TOUCH_PAD_OUTEN2_S)
#define SENS_TOUCH_PAD_OUTEN2_V  0x3ff
#define SENS_TOUCH_PAD_OUTEN2_S  10

/* SENS_TOUCH_PAD_WORKEN : R/W; bitpos: [9:0]; default: 0x3ff; */

#define SENS_TOUCH_PAD_WORKEN    0x000003ff
#define SENS_TOUCH_PAD_WORKEN_M  (SENS_TOUCH_PAD_WORKEN_V << \
                                  SENS_TOUCH_PAD_WORKEN_S)
#define SENS_TOUCH_PAD_WORKEN_V  0x3ff
#define SENS_TOUCH_PAD_WORKEN_S  0

/* SENS_SAR_READ_CTRL2_REG register */

#define SENS_SAR_READ_CTRL2_REG (DR_REG_SENS_BASE + 0x90)

/* SENS_SAR2_DATA_INV : R/W; bitpos: [29]; default: 0; */

#define SENS_SAR2_DATA_INV    (BIT(29))
#define SENS_SAR2_DATA_INV_M  (SENS_SAR2_DATA_INV_V << SENS_SAR2_DATA_INV_S)
#define SENS_SAR2_DATA_INV_V  0x1
#define SENS_SAR2_DATA_INV_S  29

/* SENS_SAR2_DIG_FORCE : R/W; bitpos: [28]; default: 0; */

#define SENS_SAR2_DIG_FORCE    (BIT(28))
#define SENS_SAR2_DIG_FORCE_M  (SENS_SAR2_DIG_FORCE_V << \
                                SENS_SAR2_DIG_FORCE_S)
#define SENS_SAR2_DIG_FORCE_V  0x1
#define SENS_SAR2_DIG_FORCE_S  28

/* SENS_SAR2_SAMPLE_BIT : R/W; bitpos: [17:16]; default: 3; */

#define SENS_SAR2_SAMPLE_BIT    0x00000003
#define SENS_SAR2_SAMPLE_BIT_M  (SENS_SAR2_SAMPLE_BIT_V << \
                                 SENS_SAR2_SAMPLE_BIT_S)
#define SENS_SAR2_SAMPLE_BIT_V  0x3
#define SENS_SAR2_SAMPLE_BIT_S  16

/* SENS_SAR2_SAMPLE_CYCLE : R/W; bitpos: [15:8]; default: 9; */

#define SENS_SAR2_SAMPLE_CYCLE    0x000000ff
#define SENS_SAR2_SAMPLE_CYCLE_M  (SENS_SAR2_SAMPLE_CYCLE_V << \
                                   SENS_SAR2_SAMPLE_CYCLE_S)
#define SENS_SAR2_SAMPLE_CYCLE_V  0xff
#define SENS_SAR2_SAMPLE_CYCLE_S  8

/* SENS_SAR2_CLK_DIV : R/W; bitpos: [7:0]; default: 2; */

#define SENS_SAR2_CLK_DIV    0x000000ff
#define SENS_SAR2_CLK_DIV_M  (SENS_SAR2_CLK_DIV_V << SENS_SAR2_CLK_DIV_S)
#define SENS_SAR2_CLK_DIV_V  0xff
#define SENS_SAR2_CLK_DIV_S  0

/* SENS_SAR_MEAS_START2_REG register */

#define SENS_SAR_MEAS_START2_REG (DR_REG_SENS_BASE + 0x94)

/* SENS_SAR2_EN_PAD_FORCE : R/W; bitpos: [31]; default: 0; */

#define SENS_SAR2_EN_PAD_FORCE    (BIT(31))
#define SENS_SAR2_EN_PAD_FORCE_M  (SENS_SAR2_EN_PAD_FORCE_V << \
                                   SENS_SAR2_EN_PAD_FORCE_S)
#define SENS_SAR2_EN_PAD_FORCE_V  0x1
#define SENS_SAR2_EN_PAD_FORCE_S  31

/* SENS_SAR2_EN_PAD : R/W; bitpos: [30:19]; default: 0; */

#define SENS_SAR2_EN_PAD    0x000003ff
#define SENS_SAR2_EN_PAD_M  (SENS_SAR2_EN_PAD_V << SENS_SAR2_EN_PAD_S)
#define SENS_SAR2_EN_PAD_V  0x3ff
#define SENS_SAR2_EN_PAD_S  19

/* SENS_MEAS2_START_FORCE : R/W; bitpos: [18]; default: 0; */

#define SENS_MEAS2_START_FORCE    (BIT(18))
#define SENS_MEAS2_START_FORCE_M  (SENS_MEAS2_START_FORCE_V << \
                                   SENS_MEAS2_START_FORCE_S)
#define SENS_MEAS2_START_FORCE_V  0x1
#define SENS_MEAS2_START_FORCE_S  18

/* SENS_MEAS2_START_SAR : R/W; bitpos: [17]; default: 0; */

#define SENS_MEAS2_START_SAR    (BIT(17))
#define SENS_MEAS2_START_SAR_M  (SENS_MEAS2_START_SAR_V << \
                                 SENS_MEAS2_START_SAR_S)
#define SENS_MEAS2_START_SAR_V  0x1
#define SENS_MEAS2_START_SAR_S  17

/* SENS_MEAS2_DONE_SAR : RO; bitpos: [16]; default: 0; */

#define SENS_MEAS2_DONE_SAR    (BIT(16))
#define SENS_MEAS2_DONE_SAR_M  (SENS_MEAS2_DONE_SAR_V << \
                                SENS_MEAS2_DONE_SAR_S)
#define SENS_MEAS2_DONE_SAR_V  0x1
#define SENS_MEAS2_DONE_SAR_S  16

/* SENS_MEAS2_DATA_SAR : RO; bitpos: [15:0]; default: 0; */

#define SENS_MEAS2_DATA_SAR    0x0000ffff
#define SENS_MEAS2_DATA_SAR_M  (SENS_MEAS2_DATA_SAR_V << \
                                SENS_MEAS2_DATA_SAR_S)
#define SENS_MEAS2_DATA_SAR_V  0xffff
#define SENS_MEAS2_DATA_SAR_S  0

/* SENS_SAR_DAC_CTRL1_REG register */

#define SENS_SAR_DAC_CTRL1_REG (DR_REG_SENS_BASE + 0x98)

/* SENS_DAC_CLK_INV : R/W; bitpos: [25]; default: 0; */

#define SENS_DAC_CLK_INV    (BIT(25))
#define SENS_DAC_CLK_INV_M  (SENS_DAC_CLK_INV_V << SENS_DAC_CLK_INV_S)
#define SENS_DAC_CLK_INV_V  0x1
#define SENS_DAC_CLK_INV_S  25

/* SENS_DAC_CLK_FORCE_HIGH : R/W; bitpos: [24]; default: 0; */

#define SENS_DAC_CLK_FORCE_HIGH    (BIT(24))
#define SENS_DAC_CLK_FORCE_HIGH_M  (SENS_DAC_CLK_FORCE_HIGH_V << \
                                    SENS_DAC_CLK_FORCE_HIGH_S)
#define SENS_DAC_CLK_FORCE_HIGH_V  0x1
#define SENS_DAC_CLK_FORCE_HIGH_S  24

/* SENS_DAC_CLK_FORCE_LOW : R/W; bitpos: [23]; default: 0; */

#define SENS_DAC_CLK_FORCE_LOW    (BIT(23))
#define SENS_DAC_CLK_FORCE_LOW_M  (SENS_DAC_CLK_FORCE_LOW_V << \
                                   SENS_DAC_CLK_FORCE_LOW_S)
#define SENS_DAC_CLK_FORCE_LOW_V  0x1
#define SENS_DAC_CLK_FORCE_LOW_S  23

/* SENS_DAC_DIG_FORCE : R/W; bitpos: [22]; default: 0; */

#define SENS_DAC_DIG_FORCE    (BIT(22))
#define SENS_DAC_DIG_FORCE_M  (SENS_DAC_DIG_FORCE_V << SENS_DAC_DIG_FORCE_S)
#define SENS_DAC_DIG_FORCE_V  0x1
#define SENS_DAC_DIG_FORCE_S  22

/* SENS_SW_TONE_EN : R/W; bitpos: [16]; default: 0; */

#define SENS_SW_TONE_EN    (BIT(16))
#define SENS_SW_TONE_EN_M  (SENS_SW_TONE_EN_V << SENS_SW_TONE_EN_S)
#define SENS_SW_TONE_EN_V  0x1
#define SENS_SW_TONE_EN_S  16

/* SENS_SW_FSTEP : R/W; bitpos: [15:0]; default: 0; */

#define SENS_SW_FSTEP    0x0000ffff
#define SENS_SW_FSTEP_M  (SENS_SW_FSTEP_V << SENS_SW_FSTEP_S)
#define SENS_SW_FSTEP_V  0xffff
#define SENS_SW_FSTEP_S  0

/* SENS_SAR_DAC_CTRL2_REG register */

#define SENS_SAR_DAC_CTRL2_REG (DR_REG_SENS_BASE + 0x9c)

/* SENS_DAC_CW_EN2 : R/W; bitpos: [25]; default: 1; */

#define SENS_DAC_CW_EN2    (BIT(25))
#define SENS_DAC_CW_EN2_M  (SENS_DAC_CW_EN2_V << SENS_DAC_CW_EN2_S)
#define SENS_DAC_CW_EN2_V  0x1
#define SENS_DAC_CW_EN2_S  25

/* SENS_DAC_CW_EN1 : R/W; bitpos: [24]; default: 1; */

#define SENS_DAC_CW_EN1    (BIT(24))
#define SENS_DAC_CW_EN1_M  (SENS_DAC_CW_EN1_V << SENS_DAC_CW_EN1_S)
#define SENS_DAC_CW_EN1_V  0x1
#define SENS_DAC_CW_EN1_S  24

/* SENS_DAC_INV2 : R/W; bitpos: [23:22]; default: 0; */

#define SENS_DAC_INV2    0x00000003
#define SENS_DAC_INV2_M  (SENS_DAC_INV2_V << SENS_DAC_INV2_S)
#define SENS_DAC_INV2_V  0x3
#define SENS_DAC_INV2_S  22

/* SENS_DAC_INV1 : R/W; bitpos: [21:20]; default: 0; */

#define SENS_DAC_INV1    0x00000003
#define SENS_DAC_INV1_M  (SENS_DAC_INV1_V << SENS_DAC_INV1_S)
#define SENS_DAC_INV1_V  0x3
#define SENS_DAC_INV1_S  20

/* SENS_DAC_SCALE2 : R/W; bitpos: [19:18]; default: 0; */

#define SENS_DAC_SCALE2    0x00000003
#define SENS_DAC_SCALE2_M  (SENS_DAC_SCALE2_V << SENS_DAC_SCALE2_S)
#define SENS_DAC_SCALE2_V  0x3
#define SENS_DAC_SCALE2_S  18

/* SENS_DAC_SCALE1 : R/W; bitpos: [17:16]; default: 0; */

#define SENS_DAC_SCALE1    0x00000003
#define SENS_DAC_SCALE1_M  (SENS_DAC_SCALE1_V << SENS_DAC_SCALE1_S)
#define SENS_DAC_SCALE1_V  0x3
#define SENS_DAC_SCALE1_S  16

/* SENS_DAC_DC2 : R/W; bitpos: [15:8]; default: 0; */

#define SENS_DAC_DC2    0x000000ff
#define SENS_DAC_DC2_M  (SENS_DAC_DC2_V << SENS_DAC_DC2_S)
#define SENS_DAC_DC2_V  0xff
#define SENS_DAC_DC2_S  8

/* SENS_DAC_DC1 : R/W; bitpos: [7:0]; default: 0; */

#define SENS_DAC_DC1    0x000000ff
#define SENS_DAC_DC1_M  (SENS_DAC_DC1_V << SENS_DAC_DC1_S)
#define SENS_DAC_DC1_V  0xff
#define SENS_DAC_DC1_S  0

#endif /* __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_SENS_H */
