/****************************************************************************
 * arch/xtensa/src/esp32/hardware/esp32_pcnt.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_PCNT_H
#define __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_PCNT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PCNT_U0_CONF0_REG register
 * Configuration register 0 for unit 0
 */

#define PCNT_U0_CONF0_REG (DR_REG_PCNT_BASE + 0x0)

/* PCNT_CH1_LCTRL_MODE_U0 : R/W; bitpos: [31:30]; default: 0;
 * This register configures how the CH0_POS_MODE/CH0_NEG_MODE settings will
 * be modified when the control signal is low.
 *
 * 0: No modification;1: Invert behavior (increase -> decrease, decrease ->
 * increase);2, 3: Inhibit counter modification
 */

#define PCNT_CH1_LCTRL_MODE_U0    0x00000003
#define PCNT_CH1_LCTRL_MODE_U0_M  (PCNT_CH1_LCTRL_MODE_U0_V << PCNT_CH1_LCTRL_MODE_U0_S)
#define PCNT_CH1_LCTRL_MODE_U0_V  0x00000003
#define PCNT_CH1_LCTRL_MODE_U0_S  30

/* PCNT_CH1_HCTRL_MODE_U0 : R/W; bitpos: [29:28]; default: 0;
 * This register configures how the CH0_POS_MODE/CH0_NEG_MODE settings will
 * be modified when the control signal is high.
 *
 * 0: No modification;1: Invert behavior (increase -> decrease, decrease ->
 * increase);2, 3: Inhibit counter modification
 */

#define PCNT_CH1_HCTRL_MODE_U0    0x00000003
#define PCNT_CH1_HCTRL_MODE_U0_M  (PCNT_CH1_HCTRL_MODE_U0_V << PCNT_CH1_HCTRL_MODE_U0_S)
#define PCNT_CH1_HCTRL_MODE_U0_V  0x00000003
#define PCNT_CH1_HCTRL_MODE_U0_S  28

/* PCNT_CH1_POS_MODE_U0 : R/W; bitpos: [27:26]; default: 0;
 * This register sets the behavior when the signal input of channel 1
 * detects a positive edge.
 *
 * 1: Increment the counter;2: Decrement the counter;0, 3: No effect on
 * counter
 */

#define PCNT_CH1_POS_MODE_U0    0x00000003
#define PCNT_CH1_POS_MODE_U0_M  (PCNT_CH1_POS_MODE_U0_V << PCNT_CH1_POS_MODE_U0_S)
#define PCNT_CH1_POS_MODE_U0_V  0x00000003
#define PCNT_CH1_POS_MODE_U0_S  26

/* PCNT_CH1_NEG_MODE_U0 : R/W; bitpos: [25:24]; default: 0;
 * This register sets the behavior when the signal input of channel 1
 * detects a negative edge.
 *
 * 1: Increment the counter;2: Decrement the counter;0, 3: No effect on
 * counter
 */

#define PCNT_CH1_NEG_MODE_U0    0x00000003
#define PCNT_CH1_NEG_MODE_U0_M  (PCNT_CH1_NEG_MODE_U0_V << PCNT_CH1_NEG_MODE_U0_S)
#define PCNT_CH1_NEG_MODE_U0_V  0x00000003
#define PCNT_CH1_NEG_MODE_U0_S  24

/* PCNT_CH0_LCTRL_MODE_U0 : R/W; bitpos: [23:22]; default: 0;
 * This register configures how the CH0_POS_MODE/CH0_NEG_MODE settings will
 * be modified when the control signal is low.
 *
 * 0: No modification;1: Invert behavior (increase -> decrease, decrease ->
 * increase);2, 3: Inhibit counter modification
 */

#define PCNT_CH0_LCTRL_MODE_U0    0x00000003
#define PCNT_CH0_LCTRL_MODE_U0_M  (PCNT_CH0_LCTRL_MODE_U0_V << PCNT_CH0_LCTRL_MODE_U0_S)
#define PCNT_CH0_LCTRL_MODE_U0_V  0x00000003
#define PCNT_CH0_LCTRL_MODE_U0_S  22

/* PCNT_CH0_HCTRL_MODE_U0 : R/W; bitpos: [21:20]; default: 0;
 * This register configures how the CH0_POS_MODE/CH0_NEG_MODE settings will
 * be modified when the control signal is high.
 *
 * 0: No modification;1: Invert behavior (increase -> decrease, decrease ->
 * increase);2, 3: Inhibit counter modification
 */

#define PCNT_CH0_HCTRL_MODE_U0    0x00000003
#define PCNT_CH0_HCTRL_MODE_U0_M  (PCNT_CH0_HCTRL_MODE_U0_V << PCNT_CH0_HCTRL_MODE_U0_S)
#define PCNT_CH0_HCTRL_MODE_U0_V  0x00000003
#define PCNT_CH0_HCTRL_MODE_U0_S  20

/* PCNT_CH0_POS_MODE_U0 : R/W; bitpos: [19:18]; default: 0;
 * This register sets the behavior when the signal input of channel 0
 * detects a positive edge.
 *
 * 1: Increase the counter;2: Decrease the counter;0, 3: No effect on counter
 */

#define PCNT_CH0_POS_MODE_U0    0x00000003
#define PCNT_CH0_POS_MODE_U0_M  (PCNT_CH0_POS_MODE_U0_V << PCNT_CH0_POS_MODE_U0_S)
#define PCNT_CH0_POS_MODE_U0_V  0x00000003
#define PCNT_CH0_POS_MODE_U0_S  18

/* PCNT_CH0_NEG_MODE_U0 : R/W; bitpos: [17:16]; default: 0;
 * This register sets the behavior when the signal input of channel 0
 * detects a negative edge.
 *
 * 1: Increase the counter;2: Decrease the counter;0, 3: No effect on counter
 */

#define PCNT_CH0_NEG_MODE_U0    0x00000003
#define PCNT_CH0_NEG_MODE_U0_M  (PCNT_CH0_NEG_MODE_U0_V << PCNT_CH0_NEG_MODE_U0_S)
#define PCNT_CH0_NEG_MODE_U0_V  0x00000003
#define PCNT_CH0_NEG_MODE_U0_S  16

/* PCNT_THR_THRES1_EN_U0 : R/W; bitpos: [15]; default: 0;
 * This is the enable bit for unit 0's thres1 comparator.
 */

#define PCNT_THR_THRES1_EN_U0    (BIT(15))
#define PCNT_THR_THRES1_EN_U0_M  (PCNT_THR_THRES1_EN_U0_V << PCNT_THR_THRES1_EN_U0_S)
#define PCNT_THR_THRES1_EN_U0_V  0x00000001
#define PCNT_THR_THRES1_EN_U0_S  15

/* PCNT_THR_THRES0_EN_U0 : R/W; bitpos: [14]; default: 0;
 * This is the enable bit for unit 0's thres0 comparator.
 */

#define PCNT_THR_THRES0_EN_U0    (BIT(14))
#define PCNT_THR_THRES0_EN_U0_M  (PCNT_THR_THRES0_EN_U0_V << PCNT_THR_THRES0_EN_U0_S)
#define PCNT_THR_THRES0_EN_U0_V  0x00000001
#define PCNT_THR_THRES0_EN_U0_S  14

/* PCNT_THR_L_LIM_EN_U0 : R/W; bitpos: [13]; default: 1;
 * This is the enable bit for unit 0's thr_l_lim comparator.
 */

#define PCNT_THR_L_LIM_EN_U0    (BIT(13))
#define PCNT_THR_L_LIM_EN_U0_M  (PCNT_THR_L_LIM_EN_U0_V << PCNT_THR_L_LIM_EN_U0_S)
#define PCNT_THR_L_LIM_EN_U0_V  0x00000001
#define PCNT_THR_L_LIM_EN_U0_S  13

/* PCNT_THR_H_LIM_EN_U0 : R/W; bitpos: [12]; default: 1;
 * This is the enable bit for unit 0's thr_h_lim comparator.
 */

#define PCNT_THR_H_LIM_EN_U0    (BIT(12))
#define PCNT_THR_H_LIM_EN_U0_M  (PCNT_THR_H_LIM_EN_U0_V << PCNT_THR_H_LIM_EN_U0_S)
#define PCNT_THR_H_LIM_EN_U0_V  0x00000001
#define PCNT_THR_H_LIM_EN_U0_S  12

/* PCNT_THR_ZERO_EN_U0 : R/W; bitpos: [11]; default: 1;
 * This is the enable bit for unit 0's zero comparator.
 */

#define PCNT_THR_ZERO_EN_U0    (BIT(11))
#define PCNT_THR_ZERO_EN_U0_M  (PCNT_THR_ZERO_EN_U0_V << PCNT_THR_ZERO_EN_U0_S)
#define PCNT_THR_ZERO_EN_U0_V  0x00000001
#define PCNT_THR_ZERO_EN_U0_S  11

/* PCNT_FILTER_EN_U0 : R/W; bitpos: [10]; default: 1;
 * This is the enable bit for unit 0's input filter.
 */

#define PCNT_FILTER_EN_U0    (BIT(10))
#define PCNT_FILTER_EN_U0_M  (PCNT_FILTER_EN_U0_V << PCNT_FILTER_EN_U0_S)
#define PCNT_FILTER_EN_U0_V  0x00000001
#define PCNT_FILTER_EN_U0_S  10

/* PCNT_FILTER_THRES_U0 : R/W; bitpos: [9:0]; default: 16;
 * This sets the maximum threshold, in APB_CLK cycles, for the filter.
 *
 * Any pulses with width less than this will be ignored when the filter is
 * enabled.
 */

#define PCNT_FILTER_THRES_U0    0x000003ff
#define PCNT_FILTER_THRES_U0_M  (PCNT_FILTER_THRES_U0_V << PCNT_FILTER_THRES_U0_S)
#define PCNT_FILTER_THRES_U0_V  0x000003ff
#define PCNT_FILTER_THRES_U0_S  0

/* PCNT_U0_CONF1_REG register
 * Configuration register 1 for unit 0
 */

#define PCNT_U0_CONF1_REG (DR_REG_PCNT_BASE + 0x4)

/* PCNT_CNT_THRES1_U0 : R/W; bitpos: [31:16]; default: 0;
 * This register is used to configure the thres1 value for unit 0.
 */

#define PCNT_CNT_THRES1_U0    0x0000ffff
#define PCNT_CNT_THRES1_U0_M  (PCNT_CNT_THRES1_U0_V << PCNT_CNT_THRES1_U0_S)
#define PCNT_CNT_THRES1_U0_V  0x0000ffff
#define PCNT_CNT_THRES1_U0_S  16

/* PCNT_CNT_THRES0_U0 : R/W; bitpos: [15:0]; default: 0;
 * This register is used to configure the thres0 value for unit 0.
 */

#define PCNT_CNT_THRES0_U0    0x0000ffff
#define PCNT_CNT_THRES0_U0_M  (PCNT_CNT_THRES0_U0_V << PCNT_CNT_THRES0_U0_S)
#define PCNT_CNT_THRES0_U0_V  0x0000ffff
#define PCNT_CNT_THRES0_U0_S  0

/* PCNT_U0_CONF2_REG register
 * Configuration register 2 for unit 0
 */

#define PCNT_U0_CONF2_REG (DR_REG_PCNT_BASE + 0x8)

/* PCNT_CNT_L_LIM_U0 : R/W; bitpos: [31:16]; default: 0;
 * This register is used to configure the thr_l_lim value for unit 0.
 */

#define PCNT_CNT_L_LIM_U0    0x0000ffff
#define PCNT_CNT_L_LIM_U0_M  (PCNT_CNT_L_LIM_U0_V << PCNT_CNT_L_LIM_U0_S)
#define PCNT_CNT_L_LIM_U0_V  0x0000ffff
#define PCNT_CNT_L_LIM_U0_S  16

/* PCNT_CNT_H_LIM_U0 : R/W; bitpos: [15:0]; default: 0;
 * This register is used to configure the thr_h_lim value for unit 0.
 */

#define PCNT_CNT_H_LIM_U0    0x0000ffff
#define PCNT_CNT_H_LIM_U0_M  (PCNT_CNT_H_LIM_U0_V << PCNT_CNT_H_LIM_U0_S)
#define PCNT_CNT_H_LIM_U0_V  0x0000ffff
#define PCNT_CNT_H_LIM_U0_S  0

/* PCNT_U1_CONF0_REG register
 * Configuration register 0 for unit 1
 */

#define PCNT_U1_CONF0_REG (DR_REG_PCNT_BASE + 0xc)

/* PCNT_CH1_LCTRL_MODE_U1 : R/W; bitpos: [31:30]; default: 0;
 * This register configures how the CH1_POS_MODE/CH1_NEG_MODE settings will
 * be modified when the control signal is low.
 *
 * 0: No modification;1: Invert behavior (increase -> decrease, decrease ->
 * increase);2, 3: Inhibit counter modification
 */

#define PCNT_CH1_LCTRL_MODE_U1    0x00000003
#define PCNT_CH1_LCTRL_MODE_U1_M  (PCNT_CH1_LCTRL_MODE_U1_V << PCNT_CH1_LCTRL_MODE_U1_S)
#define PCNT_CH1_LCTRL_MODE_U1_V  0x00000003
#define PCNT_CH1_LCTRL_MODE_U1_S  30

/* PCNT_CH1_HCTRL_MODE_U1 : R/W; bitpos: [29:28]; default: 0;
 * This register configures how the CH1_POS_MODE/CH1_NEG_MODE settings will
 * be modified when the control signal is high.
 *
 * 0: No modification;1: Invert behavior (increase -> decrease, decrease ->
 * increase);2, 3: Inhibit counter modification
 */

#define PCNT_CH1_HCTRL_MODE_U1    0x00000003
#define PCNT_CH1_HCTRL_MODE_U1_M  (PCNT_CH1_HCTRL_MODE_U1_V << PCNT_CH1_HCTRL_MODE_U1_S)
#define PCNT_CH1_HCTRL_MODE_U1_V  0x00000003
#define PCNT_CH1_HCTRL_MODE_U1_S  28

/* PCNT_CH1_POS_MODE_U1 : R/W; bitpos: [27:26]; default: 0;
 * This register sets the behavior when the signal input of channel 1
 * detects a positive edge.
 *
 * 1: Increment the counter;2: Decrement the counter;0, 3: No effect on
 * counter
 */

#define PCNT_CH1_POS_MODE_U1    0x00000003
#define PCNT_CH1_POS_MODE_U1_M  (PCNT_CH1_POS_MODE_U1_V << PCNT_CH1_POS_MODE_U1_S)
#define PCNT_CH1_POS_MODE_U1_V  0x00000003
#define PCNT_CH1_POS_MODE_U1_S  26

/* PCNT_CH1_NEG_MODE_U1 : R/W; bitpos: [25:24]; default: 0;
 * This register sets the behavior when the signal input of channel 1
 * detects a negative edge.
 *
 * 1: Increment the counter;2: Decrement the counter;0, 3: No effect on
 * counter
 */

#define PCNT_CH1_NEG_MODE_U1    0x00000003
#define PCNT_CH1_NEG_MODE_U1_M  (PCNT_CH1_NEG_MODE_U1_V << PCNT_CH1_NEG_MODE_U1_S)
#define PCNT_CH1_NEG_MODE_U1_V  0x00000003
#define PCNT_CH1_NEG_MODE_U1_S  24

/* PCNT_CH0_LCTRL_MODE_U1 : R/W; bitpos: [23:22]; default: 0;
 * This register configures how the CH1_POS_MODE/CH1_NEG_MODE settings will
 * be modified when the control signal is low.
 *
 * 0: No modification;1: Invert behavior (increase -> decrease, decrease ->
 * increase);2, 3: Inhibit counter modification
 */

#define PCNT_CH0_LCTRL_MODE_U1    0x00000003
#define PCNT_CH0_LCTRL_MODE_U1_M  (PCNT_CH0_LCTRL_MODE_U1_V << PCNT_CH0_LCTRL_MODE_U1_S)
#define PCNT_CH0_LCTRL_MODE_U1_V  0x00000003
#define PCNT_CH0_LCTRL_MODE_U1_S  22

/* PCNT_CH0_HCTRL_MODE_U1 : R/W; bitpos: [21:20]; default: 0;
 * This register configures how the CH1_POS_MODE/CH1_NEG_MODE settings will
 * be modified when the control signal is high.
 *
 * 0: No modification;1: Invert behavior (increase -> decrease, decrease ->
 * increase);2, 3: Inhibit counter modification
 */

#define PCNT_CH0_HCTRL_MODE_U1    0x00000003
#define PCNT_CH0_HCTRL_MODE_U1_M  (PCNT_CH0_HCTRL_MODE_U1_V << PCNT_CH0_HCTRL_MODE_U1_S)
#define PCNT_CH0_HCTRL_MODE_U1_V  0x00000003
#define PCNT_CH0_HCTRL_MODE_U1_S  20

/* PCNT_CH0_POS_MODE_U1 : R/W; bitpos: [19:18]; default: 0;
 * This register sets the behavior when the signal input of channel 0
 * detects a positive edge.
 *
 * 1: Increase the counter;2: Decrease the counter;0, 3: No effect on counter
 */

#define PCNT_CH0_POS_MODE_U1    0x00000003
#define PCNT_CH0_POS_MODE_U1_M  (PCNT_CH0_POS_MODE_U1_V << PCNT_CH0_POS_MODE_U1_S)
#define PCNT_CH0_POS_MODE_U1_V  0x00000003
#define PCNT_CH0_POS_MODE_U1_S  18

/* PCNT_CH0_NEG_MODE_U1 : R/W; bitpos: [17:16]; default: 0;
 * This register sets the behavior when the signal input of channel 0
 * detects a negative edge.
 *
 * 1: Increase the counter;2: Decrease the counter;0, 3: No effect on counter
 */

#define PCNT_CH0_NEG_MODE_U1    0x00000003
#define PCNT_CH0_NEG_MODE_U1_M  (PCNT_CH0_NEG_MODE_U1_V << PCNT_CH0_NEG_MODE_U1_S)
#define PCNT_CH0_NEG_MODE_U1_V  0x00000003
#define PCNT_CH0_NEG_MODE_U1_S  16

/* PCNT_THR_THRES1_EN_U1 : R/W; bitpos: [15]; default: 0;
 * This is the enable bit for unit 1's thres1 comparator.
 */

#define PCNT_THR_THRES1_EN_U1    (BIT(15))
#define PCNT_THR_THRES1_EN_U1_M  (PCNT_THR_THRES1_EN_U1_V << PCNT_THR_THRES1_EN_U1_S)
#define PCNT_THR_THRES1_EN_U1_V  0x00000001
#define PCNT_THR_THRES1_EN_U1_S  15

/* PCNT_THR_THRES0_EN_U1 : R/W; bitpos: [14]; default: 0;
 * This is the enable bit for unit 1's thres0 comparator.
 */

#define PCNT_THR_THRES0_EN_U1    (BIT(14))
#define PCNT_THR_THRES0_EN_U1_M  (PCNT_THR_THRES0_EN_U1_V << PCNT_THR_THRES0_EN_U1_S)
#define PCNT_THR_THRES0_EN_U1_V  0x00000001
#define PCNT_THR_THRES0_EN_U1_S  14

/* PCNT_THR_L_LIM_EN_U1 : R/W; bitpos: [13]; default: 1;
 * This is the enable bit for unit 1's thr_l_lim comparator.
 */

#define PCNT_THR_L_LIM_EN_U1    (BIT(13))
#define PCNT_THR_L_LIM_EN_U1_M  (PCNT_THR_L_LIM_EN_U1_V << PCNT_THR_L_LIM_EN_U1_S)
#define PCNT_THR_L_LIM_EN_U1_V  0x00000001
#define PCNT_THR_L_LIM_EN_U1_S  13

/* PCNT_THR_H_LIM_EN_U1 : R/W; bitpos: [12]; default: 1;
 * This is the enable bit for unit 1's thr_h_lim comparator.
 */

#define PCNT_THR_H_LIM_EN_U1    (BIT(12))
#define PCNT_THR_H_LIM_EN_U1_M  (PCNT_THR_H_LIM_EN_U1_V << PCNT_THR_H_LIM_EN_U1_S)
#define PCNT_THR_H_LIM_EN_U1_V  0x00000001
#define PCNT_THR_H_LIM_EN_U1_S  12

/* PCNT_THR_ZERO_EN_U1 : R/W; bitpos: [11]; default: 1;
 * This is the enable bit for unit 1's zero comparator.
 */

#define PCNT_THR_ZERO_EN_U1    (BIT(11))
#define PCNT_THR_ZERO_EN_U1_M  (PCNT_THR_ZERO_EN_U1_V << PCNT_THR_ZERO_EN_U1_S)
#define PCNT_THR_ZERO_EN_U1_V  0x00000001
#define PCNT_THR_ZERO_EN_U1_S  11

/* PCNT_FILTER_EN_U1 : R/W; bitpos: [10]; default: 1;
 * This is the enable bit for unit 1's input filter.
 */

#define PCNT_FILTER_EN_U1    (BIT(10))
#define PCNT_FILTER_EN_U1_M  (PCNT_FILTER_EN_U1_V << PCNT_FILTER_EN_U1_S)
#define PCNT_FILTER_EN_U1_V  0x00000001
#define PCNT_FILTER_EN_U1_S  10

/* PCNT_FILTER_THRES_U1 : R/W; bitpos: [9:0]; default: 16;
 * This sets the maximum threshold, in APB_CLK cycles, for the filter.
 *
 * Any pulses with width less than this will be ignored when the filter is
 * enabled.
 */

#define PCNT_FILTER_THRES_U1    0x000003ff
#define PCNT_FILTER_THRES_U1_M  (PCNT_FILTER_THRES_U1_V << PCNT_FILTER_THRES_U1_S)
#define PCNT_FILTER_THRES_U1_V  0x000003ff
#define PCNT_FILTER_THRES_U1_S  0

/* PCNT_U1_CONF1_REG register
 * Configuration register 1 for unit 1
 */

#define PCNT_U1_CONF1_REG (DR_REG_PCNT_BASE + 0x10)

/* PCNT_CNT_THRES1_U1 : R/W; bitpos: [31:16]; default: 0;
 * This register is used to configure the thres1 value for unit 1.
 */

#define PCNT_CNT_THRES1_U1    0x0000ffff
#define PCNT_CNT_THRES1_U1_M  (PCNT_CNT_THRES1_U1_V << PCNT_CNT_THRES1_U1_S)
#define PCNT_CNT_THRES1_U1_V  0x0000ffff
#define PCNT_CNT_THRES1_U1_S  16

/* PCNT_CNT_THRES0_U1 : R/W; bitpos: [15:0]; default: 0;
 * This register is used to configure the thres0 value for unit 1.
 */

#define PCNT_CNT_THRES0_U1    0x0000ffff
#define PCNT_CNT_THRES0_U1_M  (PCNT_CNT_THRES0_U1_V << PCNT_CNT_THRES0_U1_S)
#define PCNT_CNT_THRES0_U1_V  0x0000ffff
#define PCNT_CNT_THRES0_U1_S  0

/* PCNT_U1_CONF2_REG register
 * Configuration register 2 for unit 1
 */

#define PCNT_U1_CONF2_REG (DR_REG_PCNT_BASE + 0x14)

/* PCNT_CNT_L_LIM_U1 : R/W; bitpos: [31:16]; default: 0;
 * This register is used to configure the thr_l_lim value for unit 1.
 */

#define PCNT_CNT_L_LIM_U1    0x0000ffff
#define PCNT_CNT_L_LIM_U1_M  (PCNT_CNT_L_LIM_U1_V << PCNT_CNT_L_LIM_U1_S)
#define PCNT_CNT_L_LIM_U1_V  0x0000ffff
#define PCNT_CNT_L_LIM_U1_S  16

/* PCNT_CNT_H_LIM_U1 : R/W; bitpos: [15:0]; default: 0;
 * This register is used to configure the thr_h_lim value for unit 1.
 */

#define PCNT_CNT_H_LIM_U1    0x0000ffff
#define PCNT_CNT_H_LIM_U1_M  (PCNT_CNT_H_LIM_U1_V << PCNT_CNT_H_LIM_U1_S)
#define PCNT_CNT_H_LIM_U1_V  0x0000ffff
#define PCNT_CNT_H_LIM_U1_S  0

/* PCNT_U2_CONF0_REG register
 * Configuration register 0 for unit 2
 */

#define PCNT_U2_CONF0_REG (DR_REG_PCNT_BASE + 0x18)

/* PCNT_CH1_LCTRL_MODE_U2 : R/W; bitpos: [31:30]; default: 0;
 * This register configures how the CH2_POS_MODE/CH2_NEG_MODE settings will
 * be modified when the control signal is low.
 *
 * 0: No modification;1: Invert behavior (increase -> decrease, decrease ->
 * increase);2, 3: Inhibit counter modification
 */

#define PCNT_CH1_LCTRL_MODE_U2    0x00000003
#define PCNT_CH1_LCTRL_MODE_U2_M  (PCNT_CH1_LCTRL_MODE_U2_V << PCNT_CH1_LCTRL_MODE_U2_S)
#define PCNT_CH1_LCTRL_MODE_U2_V  0x00000003
#define PCNT_CH1_LCTRL_MODE_U2_S  30

/* PCNT_CH1_HCTRL_MODE_U2 : R/W; bitpos: [29:28]; default: 0;
 * This register configures how the CH2_POS_MODE/CH2_NEG_MODE settings will
 * be modified when the control signal is high.
 *
 * 0: No modification;1: Invert behavior (increase -> decrease, decrease ->
 * increase);2, 3: Inhibit counter modification
 */

#define PCNT_CH1_HCTRL_MODE_U2    0x00000003
#define PCNT_CH1_HCTRL_MODE_U2_M  (PCNT_CH1_HCTRL_MODE_U2_V << PCNT_CH1_HCTRL_MODE_U2_S)
#define PCNT_CH1_HCTRL_MODE_U2_V  0x00000003
#define PCNT_CH1_HCTRL_MODE_U2_S  28

/* PCNT_CH1_POS_MODE_U2 : R/W; bitpos: [27:26]; default: 0;
 * This register sets the behavior when the signal input of channel 1
 * detects a positive edge.
 *
 * 1: Increment the counter;2: Decrement the counter;0, 3: No effect on
 * counter
 */

#define PCNT_CH1_POS_MODE_U2    0x00000003
#define PCNT_CH1_POS_MODE_U2_M  (PCNT_CH1_POS_MODE_U2_V << PCNT_CH1_POS_MODE_U2_S)
#define PCNT_CH1_POS_MODE_U2_V  0x00000003
#define PCNT_CH1_POS_MODE_U2_S  26

/* PCNT_CH1_NEG_MODE_U2 : R/W; bitpos: [25:24]; default: 0;
 * This register sets the behavior when the signal input of channel 1
 * detects a negative edge.
 *
 * 1: Increment the counter;2: Decrement the counter;0, 3: No effect on
 * counter
 */

#define PCNT_CH1_NEG_MODE_U2    0x00000003
#define PCNT_CH1_NEG_MODE_U2_M  (PCNT_CH1_NEG_MODE_U2_V << PCNT_CH1_NEG_MODE_U2_S)
#define PCNT_CH1_NEG_MODE_U2_V  0x00000003
#define PCNT_CH1_NEG_MODE_U2_S  24

/* PCNT_CH0_LCTRL_MODE_U2 : R/W; bitpos: [23:22]; default: 0;
 * This register configures how the CH2_POS_MODE/CH2_NEG_MODE settings will
 * be modified when the control signal is low.
 *
 * 0: No modification;1: Invert behavior (increase -> decrease, decrease ->
 * increase);2, 3: Inhibit counter modification
 */

#define PCNT_CH0_LCTRL_MODE_U2    0x00000003
#define PCNT_CH0_LCTRL_MODE_U2_M  (PCNT_CH0_LCTRL_MODE_U2_V << PCNT_CH0_LCTRL_MODE_U2_S)
#define PCNT_CH0_LCTRL_MODE_U2_V  0x00000003
#define PCNT_CH0_LCTRL_MODE_U2_S  22

/* PCNT_CH0_HCTRL_MODE_U2 : R/W; bitpos: [21:20]; default: 0;
 * This register configures how the CH2_POS_MODE/CH2_NEG_MODE settings will
 * be modified when the control signal is high.
 *
 * 0: No modification;1: Invert behavior (increase -> decrease, decrease ->
 * increase);2, 3: Inhibit counter modification
 */

#define PCNT_CH0_HCTRL_MODE_U2    0x00000003
#define PCNT_CH0_HCTRL_MODE_U2_M  (PCNT_CH0_HCTRL_MODE_U2_V << PCNT_CH0_HCTRL_MODE_U2_S)
#define PCNT_CH0_HCTRL_MODE_U2_V  0x00000003
#define PCNT_CH0_HCTRL_MODE_U2_S  20

/* PCNT_CH0_POS_MODE_U2 : R/W; bitpos: [19:18]; default: 0;
 * This register sets the behavior when the signal input of channel 0
 * detects a positive edge.
 *
 * 1: Increase the counter;2: Decrease the counter;0, 3: No effect on counter
 */

#define PCNT_CH0_POS_MODE_U2    0x00000003
#define PCNT_CH0_POS_MODE_U2_M  (PCNT_CH0_POS_MODE_U2_V << PCNT_CH0_POS_MODE_U2_S)
#define PCNT_CH0_POS_MODE_U2_V  0x00000003
#define PCNT_CH0_POS_MODE_U2_S  18

/* PCNT_CH0_NEG_MODE_U2 : R/W; bitpos: [17:16]; default: 0;
 * This register sets the behavior when the signal input of channel 0
 * detects a negative edge.
 *
 * 1: Increase the counter;2: Decrease the counter;0, 3: No effect on counter
 */

#define PCNT_CH0_NEG_MODE_U2    0x00000003
#define PCNT_CH0_NEG_MODE_U2_M  (PCNT_CH0_NEG_MODE_U2_V << PCNT_CH0_NEG_MODE_U2_S)
#define PCNT_CH0_NEG_MODE_U2_V  0x00000003
#define PCNT_CH0_NEG_MODE_U2_S  16

/* PCNT_THR_THRES1_EN_U2 : R/W; bitpos: [15]; default: 0;
 * This is the enable bit for unit 2's thres1 comparator.
 */

#define PCNT_THR_THRES1_EN_U2    (BIT(15))
#define PCNT_THR_THRES1_EN_U2_M  (PCNT_THR_THRES1_EN_U2_V << PCNT_THR_THRES1_EN_U2_S)
#define PCNT_THR_THRES1_EN_U2_V  0x00000001
#define PCNT_THR_THRES1_EN_U2_S  15

/* PCNT_THR_THRES0_EN_U2 : R/W; bitpos: [14]; default: 0;
 * This is the enable bit for unit 2's thres0 comparator.
 */

#define PCNT_THR_THRES0_EN_U2    (BIT(14))
#define PCNT_THR_THRES0_EN_U2_M  (PCNT_THR_THRES0_EN_U2_V << PCNT_THR_THRES0_EN_U2_S)
#define PCNT_THR_THRES0_EN_U2_V  0x00000001
#define PCNT_THR_THRES0_EN_U2_S  14

/* PCNT_THR_L_LIM_EN_U2 : R/W; bitpos: [13]; default: 1;
 * This is the enable bit for unit 2's thr_l_lim comparator.
 */

#define PCNT_THR_L_LIM_EN_U2    (BIT(13))
#define PCNT_THR_L_LIM_EN_U2_M  (PCNT_THR_L_LIM_EN_U2_V << PCNT_THR_L_LIM_EN_U2_S)
#define PCNT_THR_L_LIM_EN_U2_V  0x00000001
#define PCNT_THR_L_LIM_EN_U2_S  13

/* PCNT_THR_H_LIM_EN_U2 : R/W; bitpos: [12]; default: 1;
 * This is the enable bit for unit 2's thr_h_lim comparator.
 */

#define PCNT_THR_H_LIM_EN_U2    (BIT(12))
#define PCNT_THR_H_LIM_EN_U2_M  (PCNT_THR_H_LIM_EN_U2_V << PCNT_THR_H_LIM_EN_U2_S)
#define PCNT_THR_H_LIM_EN_U2_V  0x00000001
#define PCNT_THR_H_LIM_EN_U2_S  12

/* PCNT_THR_ZERO_EN_U2 : R/W; bitpos: [11]; default: 1;
 * This is the enable bit for unit 2's zero comparator.
 */

#define PCNT_THR_ZERO_EN_U2    (BIT(11))
#define PCNT_THR_ZERO_EN_U2_M  (PCNT_THR_ZERO_EN_U2_V << PCNT_THR_ZERO_EN_U2_S)
#define PCNT_THR_ZERO_EN_U2_V  0x00000001
#define PCNT_THR_ZERO_EN_U2_S  11

/* PCNT_FILTER_EN_U2 : R/W; bitpos: [10]; default: 1;
 * This is the enable bit for unit 2's input filter.
 */

#define PCNT_FILTER_EN_U2    (BIT(10))
#define PCNT_FILTER_EN_U2_M  (PCNT_FILTER_EN_U2_V << PCNT_FILTER_EN_U2_S)
#define PCNT_FILTER_EN_U2_V  0x00000001
#define PCNT_FILTER_EN_U2_S  10

/* PCNT_FILTER_THRES_U2 : R/W; bitpos: [9:0]; default: 16;
 * This sets the maximum threshold, in APB_CLK cycles, for the filter.
 *
 * Any pulses with width less than this will be ignored when the filter is
 * enabled.
 */

#define PCNT_FILTER_THRES_U2    0x000003ff
#define PCNT_FILTER_THRES_U2_M  (PCNT_FILTER_THRES_U2_V << PCNT_FILTER_THRES_U2_S)
#define PCNT_FILTER_THRES_U2_V  0x000003ff
#define PCNT_FILTER_THRES_U2_S  0

/* PCNT_U2_CONF1_REG register
 * Configuration register 1 for unit 2
 */

#define PCNT_U2_CONF1_REG (DR_REG_PCNT_BASE + 0x1c)

/* PCNT_CNT_THRES1_U2 : R/W; bitpos: [31:16]; default: 0;
 * This register is used to configure the thres1 value for unit 2.
 */

#define PCNT_CNT_THRES1_U2    0x0000ffff
#define PCNT_CNT_THRES1_U2_M  (PCNT_CNT_THRES1_U2_V << PCNT_CNT_THRES1_U2_S)
#define PCNT_CNT_THRES1_U2_V  0x0000ffff
#define PCNT_CNT_THRES1_U2_S  16

/* PCNT_CNT_THRES0_U2 : R/W; bitpos: [15:0]; default: 0;
 * This register is used to configure the thres0 value for unit 2.
 */

#define PCNT_CNT_THRES0_U2    0x0000ffff
#define PCNT_CNT_THRES0_U2_M  (PCNT_CNT_THRES0_U2_V << PCNT_CNT_THRES0_U2_S)
#define PCNT_CNT_THRES0_U2_V  0x0000ffff
#define PCNT_CNT_THRES0_U2_S  0

/* PCNT_U2_CONF2_REG register
 * Configuration register 2 for unit 2
 */

#define PCNT_U2_CONF2_REG (DR_REG_PCNT_BASE + 0x20)

/* PCNT_CNT_L_LIM_U2 : R/W; bitpos: [31:16]; default: 0;
 * This register is used to configure the thr_l_lim value for unit 2.
 */

#define PCNT_CNT_L_LIM_U2    0x0000ffff
#define PCNT_CNT_L_LIM_U2_M  (PCNT_CNT_L_LIM_U2_V << PCNT_CNT_L_LIM_U2_S)
#define PCNT_CNT_L_LIM_U2_V  0x0000ffff
#define PCNT_CNT_L_LIM_U2_S  16

/* PCNT_CNT_H_LIM_U2 : R/W; bitpos: [15:0]; default: 0;
 * This register is used to configure the thr_h_lim value for unit 2.
 */

#define PCNT_CNT_H_LIM_U2    0x0000ffff
#define PCNT_CNT_H_LIM_U2_M  (PCNT_CNT_H_LIM_U2_V << PCNT_CNT_H_LIM_U2_S)
#define PCNT_CNT_H_LIM_U2_V  0x0000ffff
#define PCNT_CNT_H_LIM_U2_S  0

/* PCNT_U3_CONF0_REG register
 * Configuration register 0 for unit 3
 */

#define PCNT_U3_CONF0_REG (DR_REG_PCNT_BASE + 0x24)

/* PCNT_CH1_LCTRL_MODE_U3 : R/W; bitpos: [31:30]; default: 0;
 * This register configures how the CH3_POS_MODE/CH3_NEG_MODE settings will
 * be modified when the control signal is low.
 *
 * 0: No modification;1: Invert behavior (increase -> decrease, decrease ->
 * increase);2, 3: Inhibit counter modification
 */

#define PCNT_CH1_LCTRL_MODE_U3    0x00000003
#define PCNT_CH1_LCTRL_MODE_U3_M  (PCNT_CH1_LCTRL_MODE_U3_V << PCNT_CH1_LCTRL_MODE_U3_S)
#define PCNT_CH1_LCTRL_MODE_U3_V  0x00000003
#define PCNT_CH1_LCTRL_MODE_U3_S  30

/* PCNT_CH1_HCTRL_MODE_U3 : R/W; bitpos: [29:28]; default: 0;
 * This register configures how the CH3_POS_MODE/CH3_NEG_MODE settings will
 * be modified when the control signal is high.
 *
 * 0: No modification;1: Invert behavior (increase -> decrease, decrease ->
 * increase);2, 3: Inhibit counter modification
 */

#define PCNT_CH1_HCTRL_MODE_U3    0x00000003
#define PCNT_CH1_HCTRL_MODE_U3_M  (PCNT_CH1_HCTRL_MODE_U3_V << PCNT_CH1_HCTRL_MODE_U3_S)
#define PCNT_CH1_HCTRL_MODE_U3_V  0x00000003
#define PCNT_CH1_HCTRL_MODE_U3_S  28

/* PCNT_CH1_POS_MODE_U3 : R/W; bitpos: [27:26]; default: 0;
 * This register sets the behavior when the signal input of channel 1
 * detects a positive edge.
 *
 * 1: Increment the counter;2: Decrement the counter;0, 3: No effect on
 * counter
 */

#define PCNT_CH1_POS_MODE_U3    0x00000003
#define PCNT_CH1_POS_MODE_U3_M  (PCNT_CH1_POS_MODE_U3_V << PCNT_CH1_POS_MODE_U3_S)
#define PCNT_CH1_POS_MODE_U3_V  0x00000003
#define PCNT_CH1_POS_MODE_U3_S  26

/* PCNT_CH1_NEG_MODE_U3 : R/W; bitpos: [25:24]; default: 0;
 * This register sets the behavior when the signal input of channel 1
 * detects a negative edge.
 *
 * 1: Increment the counter;2: Decrement the counter;0, 3: No effect on
 * counter
 */

#define PCNT_CH1_NEG_MODE_U3    0x00000003
#define PCNT_CH1_NEG_MODE_U3_M  (PCNT_CH1_NEG_MODE_U3_V << PCNT_CH1_NEG_MODE_U3_S)
#define PCNT_CH1_NEG_MODE_U3_V  0x00000003
#define PCNT_CH1_NEG_MODE_U3_S  24

/* PCNT_CH0_LCTRL_MODE_U3 : R/W; bitpos: [23:22]; default: 0;
 * This register configures how the CH3_POS_MODE/CH3_NEG_MODE settings will
 * be modified when the control signal is low.
 *
 * 0: No modification;1: Invert behavior (increase -> decrease, decrease ->
 * increase);2, 3: Inhibit counter modification
 */

#define PCNT_CH0_LCTRL_MODE_U3    0x00000003
#define PCNT_CH0_LCTRL_MODE_U3_M  (PCNT_CH0_LCTRL_MODE_U3_V << PCNT_CH0_LCTRL_MODE_U3_S)
#define PCNT_CH0_LCTRL_MODE_U3_V  0x00000003
#define PCNT_CH0_LCTRL_MODE_U3_S  22

/* PCNT_CH0_HCTRL_MODE_U3 : R/W; bitpos: [21:20]; default: 0;
 * This register configures how the CH3_POS_MODE/CH3_NEG_MODE settings will
 * be modified when the control signal is high.
 *
 * 0: No modification;1: Invert behavior (increase -> decrease, decrease ->
 * increase);2, 3: Inhibit counter modification
 */

#define PCNT_CH0_HCTRL_MODE_U3    0x00000003
#define PCNT_CH0_HCTRL_MODE_U3_M  (PCNT_CH0_HCTRL_MODE_U3_V << PCNT_CH0_HCTRL_MODE_U3_S)
#define PCNT_CH0_HCTRL_MODE_U3_V  0x00000003
#define PCNT_CH0_HCTRL_MODE_U3_S  20

/* PCNT_CH0_POS_MODE_U3 : R/W; bitpos: [19:18]; default: 0;
 * This register sets the behavior when the signal input of channel 0
 * detects a positive edge.
 *
 * 1: Increase the counter;2: Decrease the counter;0, 3: No effect on counter
 */

#define PCNT_CH0_POS_MODE_U3    0x00000003
#define PCNT_CH0_POS_MODE_U3_M  (PCNT_CH0_POS_MODE_U3_V << PCNT_CH0_POS_MODE_U3_S)
#define PCNT_CH0_POS_MODE_U3_V  0x00000003
#define PCNT_CH0_POS_MODE_U3_S  18

/* PCNT_CH0_NEG_MODE_U3 : R/W; bitpos: [17:16]; default: 0;
 * This register sets the behavior when the signal input of channel 0
 * detects a negative edge.
 *
 * 1: Increase the counter;2: Decrease the counter;0, 3: No effect on counter
 */

#define PCNT_CH0_NEG_MODE_U3    0x00000003
#define PCNT_CH0_NEG_MODE_U3_M  (PCNT_CH0_NEG_MODE_U3_V << PCNT_CH0_NEG_MODE_U3_S)
#define PCNT_CH0_NEG_MODE_U3_V  0x00000003
#define PCNT_CH0_NEG_MODE_U3_S  16

/* PCNT_THR_THRES1_EN_U3 : R/W; bitpos: [15]; default: 0;
 * This is the enable bit for unit 3's thres1 comparator.
 */

#define PCNT_THR_THRES1_EN_U3    (BIT(15))
#define PCNT_THR_THRES1_EN_U3_M  (PCNT_THR_THRES1_EN_U3_V << PCNT_THR_THRES1_EN_U3_S)
#define PCNT_THR_THRES1_EN_U3_V  0x00000001
#define PCNT_THR_THRES1_EN_U3_S  15

/* PCNT_THR_THRES0_EN_U3 : R/W; bitpos: [14]; default: 0;
 * This is the enable bit for unit 3's thres0 comparator.
 */

#define PCNT_THR_THRES0_EN_U3    (BIT(14))
#define PCNT_THR_THRES0_EN_U3_M  (PCNT_THR_THRES0_EN_U3_V << PCNT_THR_THRES0_EN_U3_S)
#define PCNT_THR_THRES0_EN_U3_V  0x00000001
#define PCNT_THR_THRES0_EN_U3_S  14

/* PCNT_THR_L_LIM_EN_U3 : R/W; bitpos: [13]; default: 1;
 * This is the enable bit for unit 3's thr_l_lim comparator.
 */

#define PCNT_THR_L_LIM_EN_U3    (BIT(13))
#define PCNT_THR_L_LIM_EN_U3_M  (PCNT_THR_L_LIM_EN_U3_V << PCNT_THR_L_LIM_EN_U3_S)
#define PCNT_THR_L_LIM_EN_U3_V  0x00000001
#define PCNT_THR_L_LIM_EN_U3_S  13

/* PCNT_THR_H_LIM_EN_U3 : R/W; bitpos: [12]; default: 1;
 * This is the enable bit for unit 3's thr_h_lim comparator.
 */

#define PCNT_THR_H_LIM_EN_U3    (BIT(12))
#define PCNT_THR_H_LIM_EN_U3_M  (PCNT_THR_H_LIM_EN_U3_V << PCNT_THR_H_LIM_EN_U3_S)
#define PCNT_THR_H_LIM_EN_U3_V  0x00000001
#define PCNT_THR_H_LIM_EN_U3_S  12

/* PCNT_THR_ZERO_EN_U3 : R/W; bitpos: [11]; default: 1;
 * This is the enable bit for unit 3's zero comparator.
 */

#define PCNT_THR_ZERO_EN_U3    (BIT(11))
#define PCNT_THR_ZERO_EN_U3_M  (PCNT_THR_ZERO_EN_U3_V << PCNT_THR_ZERO_EN_U3_S)
#define PCNT_THR_ZERO_EN_U3_V  0x00000001
#define PCNT_THR_ZERO_EN_U3_S  11

/* PCNT_FILTER_EN_U3 : R/W; bitpos: [10]; default: 1;
 * This is the enable bit for unit 3's input filter.
 */

#define PCNT_FILTER_EN_U3    (BIT(10))
#define PCNT_FILTER_EN_U3_M  (PCNT_FILTER_EN_U3_V << PCNT_FILTER_EN_U3_S)
#define PCNT_FILTER_EN_U3_V  0x00000001
#define PCNT_FILTER_EN_U3_S  10

/* PCNT_FILTER_THRES_U3 : R/W; bitpos: [9:0]; default: 16;
 * This sets the maximum threshold, in APB_CLK cycles, for the filter.
 *
 * Any pulses with width less than this will be ignored when the filter is
 * enabled.
 */

#define PCNT_FILTER_THRES_U3    0x000003ff
#define PCNT_FILTER_THRES_U3_M  (PCNT_FILTER_THRES_U3_V << PCNT_FILTER_THRES_U3_S)
#define PCNT_FILTER_THRES_U3_V  0x000003ff
#define PCNT_FILTER_THRES_U3_S  0

/* PCNT_U3_CONF1_REG register
 * Configuration register 1 for unit 3
 */

#define PCNT_U3_CONF1_REG (DR_REG_PCNT_BASE + 0x28)

/* PCNT_CNT_THRES1_U3 : R/W; bitpos: [31:16]; default: 0;
 * This register is used to configure the thres1 value for unit 3.
 */

#define PCNT_CNT_THRES1_U3    0x0000ffff
#define PCNT_CNT_THRES1_U3_M  (PCNT_CNT_THRES1_U3_V << PCNT_CNT_THRES1_U3_S)
#define PCNT_CNT_THRES1_U3_V  0x0000ffff
#define PCNT_CNT_THRES1_U3_S  16

/* PCNT_CNT_THRES0_U3 : R/W; bitpos: [15:0]; default: 0;
 * This register is used to configure the thres0 value for unit 3.
 */

#define PCNT_CNT_THRES0_U3    0x0000ffff
#define PCNT_CNT_THRES0_U3_M  (PCNT_CNT_THRES0_U3_V << PCNT_CNT_THRES0_U3_S)
#define PCNT_CNT_THRES0_U3_V  0x0000ffff
#define PCNT_CNT_THRES0_U3_S  0

/* PCNT_U3_CONF2_REG register
 * Configuration register 2 for unit 3
 */

#define PCNT_U3_CONF2_REG (DR_REG_PCNT_BASE + 0x2c)

/* PCNT_CNT_L_LIM_U3 : R/W; bitpos: [31:16]; default: 0;
 * This register is used to configure the thr_l_lim value for unit 3.
 */

#define PCNT_CNT_L_LIM_U3    0x0000ffff
#define PCNT_CNT_L_LIM_U3_M  (PCNT_CNT_L_LIM_U3_V << PCNT_CNT_L_LIM_U3_S)
#define PCNT_CNT_L_LIM_U3_V  0x0000ffff
#define PCNT_CNT_L_LIM_U3_S  16

/* PCNT_CNT_H_LIM_U3 : R/W; bitpos: [15:0]; default: 0;
 * This register is used to configure the thr_h_lim value for unit 3.
 */

#define PCNT_CNT_H_LIM_U3    0x0000ffff
#define PCNT_CNT_H_LIM_U3_M  (PCNT_CNT_H_LIM_U3_V << PCNT_CNT_H_LIM_U3_S)
#define PCNT_CNT_H_LIM_U3_V  0x0000ffff
#define PCNT_CNT_H_LIM_U3_S  0

#define PCNT_U4_CONF0_REG          (DR_REG_PCNT_BASE + 0x0030)

/* PCNT_CH1_LCTRL_MODE_U4 : R/W ;bitpos:[31:30] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel1's low
 * control signal for unit4.
 * 0: increase when control signal is low
 * 1: decrease when control signal is high
 * others: forbidden
 */

#define PCNT_CH1_LCTRL_MODE_U4  0x00000003
#define PCNT_CH1_LCTRL_MODE_U4_M  ((PCNT_CH1_LCTRL_MODE_U4_V) << (PCNT_CH1_LCTRL_MODE_U4_S))
#define PCNT_CH1_LCTRL_MODE_U4_V  0x3
#define PCNT_CH1_LCTRL_MODE_U4_S  30

/* PCNT_CH1_HCTRL_MODE_U4 : R/W ;bitpos:[29:28] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel1's high
 * control signal for unit4.
 * 0: increase when control signal is low
 * 1: decrease when control signal is high
 * others: forbidden
 */

#define PCNT_CH1_HCTRL_MODE_U4  0x00000003
#define PCNT_CH1_HCTRL_MODE_U4_M  ((PCNT_CH1_HCTRL_MODE_U4_V) << (PCNT_CH1_HCTRL_MODE_U4_S))
#define PCNT_CH1_HCTRL_MODE_U4_V  0x3
#define PCNT_CH1_HCTRL_MODE_U4_S  28

/* PCNT_CH1_POS_MODE_U4 : R/W ;bitpos:[27:26] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel1's input
 * posedge signal for unit4.
 * 1: increase at the posedge of input signal
 * 2: decrease at the posedge of input signal
 * others: forbidden
 */

#define PCNT_CH1_POS_MODE_U4  0x00000003
#define PCNT_CH1_POS_MODE_U4_M  ((PCNT_CH1_POS_MODE_U4_V) << (PCNT_CH1_POS_MODE_U4_S))
#define PCNT_CH1_POS_MODE_U4_V  0x3
#define PCNT_CH1_POS_MODE_U4_S  26

/* PCNT_CH1_NEG_MODE_U4 : R/W ;bitpos:[25:24] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel1's input
 * negedge signal for unit4.
 * 1: increase at the negedge of input signal
 * 2: decrease at the negedge of input signal
 * others: forbidden
 */

#define PCNT_CH1_NEG_MODE_U4  0x00000003
#define PCNT_CH1_NEG_MODE_U4_M  ((PCNT_CH1_NEG_MODE_U4_V) << (PCNT_CH1_NEG_MODE_U4_S))
#define PCNT_CH1_NEG_MODE_U4_V  0x3
#define PCNT_CH1_NEG_MODE_U4_S  24

/* PCNT_CH0_LCTRL_MODE_U4 : R/W ;bitpos:[23:22] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel0's low
 * control signal for unit4.
 * 0: increase when control signal is low
 * 1: decrease when control signal is high
 * others: forbidden
 */

#define PCNT_CH0_LCTRL_MODE_U4  0x00000003
#define PCNT_CH0_LCTRL_MODE_U4_M  ((PCNT_CH0_LCTRL_MODE_U4_V) << (PCNT_CH0_LCTRL_MODE_U4_S))
#define PCNT_CH0_LCTRL_MODE_U4_V  0x3
#define PCNT_CH0_LCTRL_MODE_U4_S  22

/* PCNT_CH0_HCTRL_MODE_U4 : R/W ;bitpos:[21:20] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel0's high
 * control signal for unit4.
 * 0: increase when control signal is low
 * 1: decrease when control signal is high
 * others: forbidden
 */

#define PCNT_CH0_HCTRL_MODE_U4  0x00000003
#define PCNT_CH0_HCTRL_MODE_U4_M  ((PCNT_CH0_HCTRL_MODE_U4_V) << (PCNT_CH0_HCTRL_MODE_U4_S))
#define PCNT_CH0_HCTRL_MODE_U4_V  0x3
#define PCNT_CH0_HCTRL_MODE_U4_S  20

/* PCNT_CH0_POS_MODE_U4 : R/W ;bitpos:[19:18] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel0's input
 * posedge signal for unit4.
 * 1: increase at the posedge of input signal
 * 2: decrease at the posedge of input signal
 * others: forbidden
 */

#define PCNT_CH0_POS_MODE_U4  0x00000003
#define PCNT_CH0_POS_MODE_U4_M  ((PCNT_CH0_POS_MODE_U4_V) << (PCNT_CH0_POS_MODE_U4_S))
#define PCNT_CH0_POS_MODE_U4_V  0x3
#define PCNT_CH0_POS_MODE_U4_S  18

/* PCNT_CH0_NEG_MODE_U4 : R/W ;bitpos:[17:16] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel0's input
 * negedge signal for unit4.
 * 1: increase at the negedge of input signal
 * 2: decrease at the negedge of input signal
 * others: forbidden
 */

#define PCNT_CH0_NEG_MODE_U4  0x00000003
#define PCNT_CH0_NEG_MODE_U4_M  ((PCNT_CH0_NEG_MODE_U4_V) << (PCNT_CH0_NEG_MODE_U4_S))
#define PCNT_CH0_NEG_MODE_U4_V  0x3
#define PCNT_CH0_NEG_MODE_U4_S  16

/* PCNT_THR_THRES1_EN_U4 : R/W ;bitpos:[15] ;default: 1'b0 ;
 * Description: This is the enable bit for comparing unit4's count with
 * thres1 value.
 */

#define PCNT_THR_THRES1_EN_U4  (BIT(15))
#define PCNT_THR_THRES1_EN_U4_M  (BIT(15))
#define PCNT_THR_THRES1_EN_U4_V  0x1
#define PCNT_THR_THRES1_EN_U4_S  15

/* PCNT_THR_THRES0_EN_U4 : R/W ;bitpos:[14] ;default: 1'b0 ;
 * Description: This is the enable bit for comparing unit4's count with
 * thres0 value.
 */

#define PCNT_THR_THRES0_EN_U4  (BIT(14))
#define PCNT_THR_THRES0_EN_U4_M  (BIT(14))
#define PCNT_THR_THRES0_EN_U4_V  0x1
#define PCNT_THR_THRES0_EN_U4_S  14

/* PCNT_THR_L_LIM_EN_U4 : R/W ;bitpos:[13] ;default: 1'b1 ;
 * Description: This is the enable bit for comparing unit4's count with
 * thr_l_lim  value.
 */

#define PCNT_THR_L_LIM_EN_U4  (BIT(13))
#define PCNT_THR_L_LIM_EN_U4_M  (BIT(13))
#define PCNT_THR_L_LIM_EN_U4_V  0x1
#define PCNT_THR_L_LIM_EN_U4_S  13

/* PCNT_THR_H_LIM_EN_U4 : R/W ;bitpos:[12] ;default: 1'b1 ;
 * Description: This is the enable bit for comparing unit4's count with
 * thr_h_lim value.
 */

#define PCNT_THR_H_LIM_EN_U4  (BIT(12))
#define PCNT_THR_H_LIM_EN_U4_M  (BIT(12))
#define PCNT_THR_H_LIM_EN_U4_V  0x1
#define PCNT_THR_H_LIM_EN_U4_S  12

/* PCNT_THR_ZERO_EN_U4 : R/W ;bitpos:[11] ;default: 1'b1 ;
 * Description: This is the enable bit for comparing unit4's count
 * with 0 value.
 */

#define PCNT_THR_ZERO_EN_U4  (BIT(11))
#define PCNT_THR_ZERO_EN_U4_M  (BIT(11))
#define PCNT_THR_ZERO_EN_U4_V  0x1
#define PCNT_THR_ZERO_EN_U4_S  11

/* PCNT_FILTER_EN_U4 : R/W ;bitpos:[10] ;default: 1'b1 ;
 * Description: This is the enable bit for filtering input signals for unit4.
 */

#define PCNT_FILTER_EN_U4  (BIT(10))
#define PCNT_FILTER_EN_U4_M  (BIT(10))
#define PCNT_FILTER_EN_U4_V  0x1
#define PCNT_FILTER_EN_U4_S  10

/* PCNT_FILTER_THRES_U4 : R/W ;bitpos:[9:0] ;default: 10'h10 ;
 * Description: This register is used to filter pluse whose width is smaller
 * than this value for unit4.
 */

#define PCNT_FILTER_THRES_U4  0x000003FF
#define PCNT_FILTER_THRES_U4_M  ((PCNT_FILTER_THRES_U4_V) << (PCNT_FILTER_THRES_U4_S))
#define PCNT_FILTER_THRES_U4_V  0x3FF
#define PCNT_FILTER_THRES_U4_S  0

#define PCNT_U4_CONF1_REG          (DR_REG_PCNT_BASE + 0x0034)

/* PCNT_CNT_THRES1_U4 : R/W ;bitpos:[31:16] ;default: 10'h0 ;
 * Description: This register is used to configure thres1 value for unit4.
 */

#define PCNT_CNT_THRES1_U4  0x0000FFFF
#define PCNT_CNT_THRES1_U4_M  ((PCNT_CNT_THRES1_U4_V) << (PCNT_CNT_THRES1_U4_S))
#define PCNT_CNT_THRES1_U4_V  0xFFFF
#define PCNT_CNT_THRES1_U4_S  16

/* PCNT_CNT_THRES0_U4 : R/W ;bitpos:[15:0] ;default: 10'h0 ;
 * Description: This register is used to configure thres0 value for unit4.
 */

#define PCNT_CNT_THRES0_U4  0x0000FFFF
#define PCNT_CNT_THRES0_U4_M  ((PCNT_CNT_THRES0_U4_V) << (PCNT_CNT_THRES0_U4_S))
#define PCNT_CNT_THRES0_U4_V  0xFFFF
#define PCNT_CNT_THRES0_U4_S  0

#define PCNT_U4_CONF2_REG          (DR_REG_PCNT_BASE + 0x0038)

/* PCNT_CNT_L_LIM_U4 : R/W ;bitpos:[31:16] ;default: 10'h0 ;
 * Description: This register is used to confiugre thr_l_lim value for unit4.
 */

#define PCNT_CNT_L_LIM_U4  0x0000FFFF
#define PCNT_CNT_L_LIM_U4_M  ((PCNT_CNT_L_LIM_U4_V) << (PCNT_CNT_L_LIM_U4_S))
#define PCNT_CNT_L_LIM_U4_V  0xFFFF
#define PCNT_CNT_L_LIM_U4_S  16

/* PCNT_CNT_H_LIM_U4 : R/W ;bitpos:[15:0] ;default: 10'h0 ;
 * Description: This register is used to configure thr_h_lim value for unit4.
 */

#define PCNT_CNT_H_LIM_U4  0x0000FFFF
#define PCNT_CNT_H_LIM_U4_M  ((PCNT_CNT_H_LIM_U4_V) << (PCNT_CNT_H_LIM_U4_S))
#define PCNT_CNT_H_LIM_U4_V  0xFFFF
#define PCNT_CNT_H_LIM_U4_S  0

#define PCNT_U5_CONF0_REG          (DR_REG_PCNT_BASE + 0x003c)

/* PCNT_CH1_LCTRL_MODE_U5 : R/W ;bitpos:[31:30] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel1's low
 * control signal for unit5.
 * 0: increase when control signal is low
 * 1: decrease when control signal is high
 * others: forbidden
 */

#define PCNT_CH1_LCTRL_MODE_U5  0x00000003
#define PCNT_CH1_LCTRL_MODE_U5_M  ((PCNT_CH1_LCTRL_MODE_U5_V) << (PCNT_CH1_LCTRL_MODE_U5_S))
#define PCNT_CH1_LCTRL_MODE_U5_V  0x3
#define PCNT_CH1_LCTRL_MODE_U5_S  30

/* PCNT_CH1_HCTRL_MODE_U5 : R/W ;bitpos:[29:28] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel1's high
 * control signal for unit5.
 * 0: increase when control signal is low
 * 1: decrease when control signal is high
 * others: forbidden
 */

#define PCNT_CH1_HCTRL_MODE_U5  0x00000003
#define PCNT_CH1_HCTRL_MODE_U5_M  ((PCNT_CH1_HCTRL_MODE_U5_V) << (PCNT_CH1_HCTRL_MODE_U5_S))
#define PCNT_CH1_HCTRL_MODE_U5_V  0x3
#define PCNT_CH1_HCTRL_MODE_U5_S  28

/* PCNT_CH1_POS_MODE_U5 : R/W ;bitpos:[27:26] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel1's input
 * posedge signal for unit5.
 * 1: increase at the posedge of input signal
 * 2: decrease at the posedge of input signal
 * others: forbidden
 */

#define PCNT_CH1_POS_MODE_U5  0x00000003
#define PCNT_CH1_POS_MODE_U5_M  ((PCNT_CH1_POS_MODE_U5_V) << (PCNT_CH1_POS_MODE_U5_S))
#define PCNT_CH1_POS_MODE_U5_V  0x3
#define PCNT_CH1_POS_MODE_U5_S  26

/* PCNT_CH1_NEG_MODE_U5 : R/W ;bitpos:[25:24] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel1's input
 * negedge signal for unit5.
 * 1: increase at the negedge of input signal
 * 2: decrease at the negedge of input signal
 * others: forbidden
 */

#define PCNT_CH1_NEG_MODE_U5  0x00000003
#define PCNT_CH1_NEG_MODE_U5_M  ((PCNT_CH1_NEG_MODE_U5_V) << (PCNT_CH1_NEG_MODE_U5_S))
#define PCNT_CH1_NEG_MODE_U5_V  0x3
#define PCNT_CH1_NEG_MODE_U5_S  24

/* PCNT_CH0_LCTRL_MODE_U5 : R/W ;bitpos:[23:22] ;default: 2'd0 ; */

/* Description: This register is used to control the mode of channel0's low
 * control signal for unit5.
 * 0: increase when control signal is low
 * 1: decrease when control signal is high
 * others: forbidden
 */

#define PCNT_CH0_LCTRL_MODE_U5  0x00000003
#define PCNT_CH0_LCTRL_MODE_U5_M  ((PCNT_CH0_LCTRL_MODE_U5_V) << (PCNT_CH0_LCTRL_MODE_U5_S))
#define PCNT_CH0_LCTRL_MODE_U5_V  0x3
#define PCNT_CH0_LCTRL_MODE_U5_S  22

/* PCNT_CH0_HCTRL_MODE_U5 : R/W ;bitpos:[21:20] ;default: 2'd0 ; */

/* Description: This register is used to control the mode of channel0's high
 * control signal for unit5.
 * 0:increase when control signal is low
 * 1: decrease when control signal is high
 * others: forbidden
 */

#define PCNT_CH0_HCTRL_MODE_U5  0x00000003
#define PCNT_CH0_HCTRL_MODE_U5_M  ((PCNT_CH0_HCTRL_MODE_U5_V) << (PCNT_CH0_HCTRL_MODE_U5_S))
#define PCNT_CH0_HCTRL_MODE_U5_V  0x3
#define PCNT_CH0_HCTRL_MODE_U5_S  20

/* PCNT_CH0_POS_MODE_U5 : R/W ;bitpos:[19:18] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel0's input
 * posedge signal for unit5.
 * 1: increase at the posedge of input signal
 * 2: decrease at the posedge of input signal
 * others: forbidden
 */

#define PCNT_CH0_POS_MODE_U5  0x00000003
#define PCNT_CH0_POS_MODE_U5_M  ((PCNT_CH0_POS_MODE_U5_V) << (PCNT_CH0_POS_MODE_U5_S))
#define PCNT_CH0_POS_MODE_U5_V  0x3
#define PCNT_CH0_POS_MODE_U5_S  18

/* PCNT_CH0_NEG_MODE_U5 : R/W ;bitpos:[17:16] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel0's input
 * negedge signal for unit5.
 * 1: increase at the negedge of input signal
 * 2: decrease at the negedge of input signal
 * others: forbidden
 */

#define PCNT_CH0_NEG_MODE_U5  0x00000003
#define PCNT_CH0_NEG_MODE_U5_M  ((PCNT_CH0_NEG_MODE_U5_V) << (PCNT_CH0_NEG_MODE_U5_S))
#define PCNT_CH0_NEG_MODE_U5_V  0x3
#define PCNT_CH0_NEG_MODE_U5_S  16

/* PCNT_THR_THRES1_EN_U5 : R/W ;bitpos:[15] ;default: 1'b0 ;
 * Description: This is the enable bit for comparing unit5's count with
 * thres1 value.
 */

#define PCNT_THR_THRES1_EN_U5  (BIT(15))
#define PCNT_THR_THRES1_EN_U5_M  (BIT(15))
#define PCNT_THR_THRES1_EN_U5_V  0x1
#define PCNT_THR_THRES1_EN_U5_S  15

/* PCNT_THR_THRES0_EN_U5 : R/W ;bitpos:[14] ;default: 1'b0 ;
 * Description: This is the enable bit for comparing unit5's count with
 * thres0 value.
 */

#define PCNT_THR_THRES0_EN_U5  (BIT(14))
#define PCNT_THR_THRES0_EN_U5_M  (BIT(14))
#define PCNT_THR_THRES0_EN_U5_V  0x1
#define PCNT_THR_THRES0_EN_U5_S  14

/* PCNT_THR_L_LIM_EN_U5 : R/W ;bitpos:[13] ;default: 1'b1 ;
 * Description: This is the enable bit for comparing unit5's count with
 * thr_l_lim value.
 */

#define PCNT_THR_L_LIM_EN_U5  (BIT(13))
#define PCNT_THR_L_LIM_EN_U5_M  (BIT(13))
#define PCNT_THR_L_LIM_EN_U5_V  0x1
#define PCNT_THR_L_LIM_EN_U5_S  13

/* PCNT_THR_H_LIM_EN_U5 : R/W ;bitpos:[12] ;default: 1'b1 ;
 * Description: This is the enable bit for comparing unit5's count with
 * thr_h_lim value.
 */

#define PCNT_THR_H_LIM_EN_U5  (BIT(12))
#define PCNT_THR_H_LIM_EN_U5_M  (BIT(12))
#define PCNT_THR_H_LIM_EN_U5_V  0x1
#define PCNT_THR_H_LIM_EN_U5_S  12

/* PCNT_THR_ZERO_EN_U5 : R/W ;bitpos:[11] ;default: 1'b1 ;
 * Description: This is the enable bit for comparing unit5's count
 * with 0 value.
 */

#define PCNT_THR_ZERO_EN_U5  (BIT(11))
#define PCNT_THR_ZERO_EN_U5_M  (BIT(11))
#define PCNT_THR_ZERO_EN_U5_V  0x1
#define PCNT_THR_ZERO_EN_U5_S  11

/* PCNT_FILTER_EN_U5 : R/W ;bitpos:[10] ;default: 1'b1 ;
 * Description: This is the enable bit for filtering input signals for unit5.
 */

#define PCNT_FILTER_EN_U5  (BIT(10))
#define PCNT_FILTER_EN_U5_M  (BIT(10))
#define PCNT_FILTER_EN_U5_V  0x1
#define PCNT_FILTER_EN_U5_S  10

/* PCNT_FILTER_THRES_U5 : R/W ;bitpos:[9:0] ;default: 10'h10 ;
 * Description: This register is used to filter pluse whose width is smaller
 * than this value for unit5.
 */

#define PCNT_FILTER_THRES_U5  0x000003FF
#define PCNT_FILTER_THRES_U5_M  ((PCNT_FILTER_THRES_U5_V) << (PCNT_FILTER_THRES_U5_S))
#define PCNT_FILTER_THRES_U5_V  0x3FF
#define PCNT_FILTER_THRES_U5_S  0

#define PCNT_U5_CONF1_REG          (DR_REG_PCNT_BASE + 0x0040)

/* PCNT_CNT_THRES1_U5 : R/W ;bitpos:[31:16] ;default: 10'h0 ;
 * Description: This register is used to configure  thres1 value for unit5.
 */

#define PCNT_CNT_THRES1_U5  0x0000FFFF
#define PCNT_CNT_THRES1_U5_M  ((PCNT_CNT_THRES1_U5_V) << (PCNT_CNT_THRES1_U5_S))
#define PCNT_CNT_THRES1_U5_V  0xFFFF
#define PCNT_CNT_THRES1_U5_S  16

/* PCNT_CNT_THRES0_U5 : R/W ;bitpos:[15:0] ;default: 10'h0 ;
 * Description: This register is used to configure thres0 value for unit5.
 */

#define PCNT_CNT_THRES0_U5  0x0000FFFF
#define PCNT_CNT_THRES0_U5_M  ((PCNT_CNT_THRES0_U5_V) << (PCNT_CNT_THRES0_U5_S))
#define PCNT_CNT_THRES0_U5_V  0xFFFF
#define PCNT_CNT_THRES0_U5_S  0

#define PCNT_U5_CONF2_REG          (DR_REG_PCNT_BASE + 0x0044)

/* PCNT_CNT_L_LIM_U5 : R/W ;bitpos:[31:16] ;default: 10'h0 ;
 * Description: This register is used to confiugre thr_l_lim value for unit5.
 */

#define PCNT_CNT_L_LIM_U5  0x0000FFFF
#define PCNT_CNT_L_LIM_U5_M  ((PCNT_CNT_L_LIM_U5_V) << (PCNT_CNT_L_LIM_U5_S))
#define PCNT_CNT_L_LIM_U5_V  0xFFFF
#define PCNT_CNT_L_LIM_U5_S  16

/* PCNT_CNT_H_LIM_U5 : R/W ;bitpos:[15:0] ;default: 10'h0 ;
 * Description: This register is used to configure thr_h_lim value for unit5.
 */

#define PCNT_CNT_H_LIM_U5  0x0000FFFF
#define PCNT_CNT_H_LIM_U5_M  ((PCNT_CNT_H_LIM_U5_V) << (PCNT_CNT_H_LIM_U5_S))
#define PCNT_CNT_H_LIM_U5_V  0xFFFF
#define PCNT_CNT_H_LIM_U5_S  0

#define PCNT_U6_CONF0_REG          (DR_REG_PCNT_BASE + 0x0048)

/* PCNT_CH1_LCTRL_MODE_U6 : R/W ;bitpos:[31:30] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel1's
 * low control signal for unit6.
 * 0: increase when control signal is low
 * 1: decrease when control signal is high
 * others: forbidden
 */

#define PCNT_CH1_LCTRL_MODE_U6  0x00000003
#define PCNT_CH1_LCTRL_MODE_U6_M  ((PCNT_CH1_LCTRL_MODE_U6_V) << (PCNT_CH1_LCTRL_MODE_U6_S))
#define PCNT_CH1_LCTRL_MODE_U6_V  0x3
#define PCNT_CH1_LCTRL_MODE_U6_S  30

/* PCNT_CH1_HCTRL_MODE_U6 : R/W ;bitpos:[29:28] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel1's high
 * control signal for unit6.
 * 0: increase when control signal is low
 * 1: decrease when control signal is high
 * others: forbidden
 */

#define PCNT_CH1_HCTRL_MODE_U6  0x00000003
#define PCNT_CH1_HCTRL_MODE_U6_M  ((PCNT_CH1_HCTRL_MODE_U6_V) << (PCNT_CH1_HCTRL_MODE_U6_S))
#define PCNT_CH1_HCTRL_MODE_U6_V  0x3
#define PCNT_CH1_HCTRL_MODE_U6_S  28

/* PCNT_CH1_POS_MODE_U6 : R/W ;bitpos:[27:26] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel1's input
 * posedge signal for unit6.
 * 1: increase at the posedge of input signal
 * 2: decrease at the posedge of input signal
 * others: forbidden
 */

#define PCNT_CH1_POS_MODE_U6  0x00000003
#define PCNT_CH1_POS_MODE_U6_M  ((PCNT_CH1_POS_MODE_U6_V) << (PCNT_CH1_POS_MODE_U6_S))
#define PCNT_CH1_POS_MODE_U6_V  0x3
#define PCNT_CH1_POS_MODE_U6_S  26

/* PCNT_CH1_NEG_MODE_U6 : R/W ;bitpos:[25:24] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel1's
 * input negedge signal for unit6.
 * 1: increase at the negedge of input signal
 * 2: decrease at the negedge of input signal
 * others: forbidden
 */

#define PCNT_CH1_NEG_MODE_U6  0x00000003
#define PCNT_CH1_NEG_MODE_U6_M  ((PCNT_CH1_NEG_MODE_U6_V) << (PCNT_CH1_NEG_MODE_U6_S))
#define PCNT_CH1_NEG_MODE_U6_V  0x3
#define PCNT_CH1_NEG_MODE_U6_S  24

/* PCNT_CH0_LCTRL_MODE_U6 : R/W ;bitpos:[23:22] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel0's low
 * control signal for unit6.
 * 0: increase when control signal is low
 * 1: decrease when control signal is high
 * others: forbidden
 */

#define PCNT_CH0_LCTRL_MODE_U6  0x00000003
#define PCNT_CH0_LCTRL_MODE_U6_M  ((PCNT_CH0_LCTRL_MODE_U6_V) << (PCNT_CH0_LCTRL_MODE_U6_S))
#define PCNT_CH0_LCTRL_MODE_U6_V  0x3
#define PCNT_CH0_LCTRL_MODE_U6_S  22

/* PCNT_CH0_HCTRL_MODE_U6 : R/W ;bitpos:[21:20] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel0's
 * high control signal for unit6.
 * 0: increase when control signal is low
 * 1: decrease when control signal is high
 * others: forbidden
 */

#define PCNT_CH0_HCTRL_MODE_U6  0x00000003
#define PCNT_CH0_HCTRL_MODE_U6_M  ((PCNT_CH0_HCTRL_MODE_U6_V) << (PCNT_CH0_HCTRL_MODE_U6_S))
#define PCNT_CH0_HCTRL_MODE_U6_V  0x3
#define PCNT_CH0_HCTRL_MODE_U6_S  20

/* PCNT_CH0_POS_MODE_U6 : R/W ;bitpos:[19:18] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel0's input
 * posedge signal for unit6.
 * 1: increase at the posedge of input signal
 * 2: decrease at the posedge of input signal
 * others: forbidden
 */

#define PCNT_CH0_POS_MODE_U6  0x00000003
#define PCNT_CH0_POS_MODE_U6_M  ((PCNT_CH0_POS_MODE_U6_V) << (PCNT_CH0_POS_MODE_U6_S))
#define PCNT_CH0_POS_MODE_U6_V  0x3
#define PCNT_CH0_POS_MODE_U6_S  18

/* PCNT_CH0_NEG_MODE_U6 : R/W ;bitpos:[17:16] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel0's input
 * negedge signal for unit6.
 * 1: increase at the negedge of input signal
 * 2: decrease at the negedge of input signal
 * others: forbidden
 */

#define PCNT_CH0_NEG_MODE_U6  0x00000003
#define PCNT_CH0_NEG_MODE_U6_M  ((PCNT_CH0_NEG_MODE_U6_V) << (PCNT_CH0_NEG_MODE_U6_S))
#define PCNT_CH0_NEG_MODE_U6_V  0x3
#define PCNT_CH0_NEG_MODE_U6_S  16

/* PCNT_THR_THRES1_EN_U6 : R/W ;bitpos:[15] ;default: 1'b0 ;
 * Description: This is the enable bit for  comparing  unit6's count with
 * thres1 value.
 */

#define PCNT_THR_THRES1_EN_U6  (BIT(15))
#define PCNT_THR_THRES1_EN_U6_M  (BIT(15))
#define PCNT_THR_THRES1_EN_U6_V  0x1
#define PCNT_THR_THRES1_EN_U6_S  15

/* PCNT_THR_THRES0_EN_U6 : R/W ;bitpos:[14] ;default: 1'b0 ;
 * Description: This is the enable bit for comparing unit6's count with
 * thres0 value.
 */

#define PCNT_THR_THRES0_EN_U6  (BIT(14))
#define PCNT_THR_THRES0_EN_U6_M  (BIT(14))
#define PCNT_THR_THRES0_EN_U6_V  0x1
#define PCNT_THR_THRES0_EN_U6_S  14

/* PCNT_THR_L_LIM_EN_U6 : R/W ;bitpos:[13] ;default: 1'b1 ;
 * Description: This is the enable bit for comparing unit6's count with
 * thr_l_lim value.
 */

#define PCNT_THR_L_LIM_EN_U6  (BIT(13))
#define PCNT_THR_L_LIM_EN_U6_M  (BIT(13))
#define PCNT_THR_L_LIM_EN_U6_V  0x1
#define PCNT_THR_L_LIM_EN_U6_S  13

/* PCNT_THR_H_LIM_EN_U6 : R/W ;bitpos:[12] ;default: 1'b1 ;
 * Description: This is the enable bit for  comparing unit6's count with
 * thr_h_lim value.
 */

#define PCNT_THR_H_LIM_EN_U6  (BIT(12))
#define PCNT_THR_H_LIM_EN_U6_M  (BIT(12))
#define PCNT_THR_H_LIM_EN_U6_V  0x1
#define PCNT_THR_H_LIM_EN_U6_S  12

/* PCNT_THR_ZERO_EN_U6 : R/W ;bitpos:[11] ;default: 1'b1 ;
 * Description: This is the enable bit for comparing unit6's count with 0
 * value.
 */

#define PCNT_THR_ZERO_EN_U6  (BIT(11))
#define PCNT_THR_ZERO_EN_U6_M  (BIT(11))
#define PCNT_THR_ZERO_EN_U6_V  0x1
#define PCNT_THR_ZERO_EN_U6_S  11

/* PCNT_FILTER_EN_U6 : R/W ;bitpos:[10] ;default: 1'b1 ;
 * Description: This is the enable bit for filtering input signals for unit6.
 */

#define PCNT_FILTER_EN_U6  (BIT(10))
#define PCNT_FILTER_EN_U6_M  (BIT(10))
#define PCNT_FILTER_EN_U6_V  0x1
#define PCNT_FILTER_EN_U6_S  10

/* PCNT_FILTER_THRES_U6 : R/W ;bitpos:[9:0] ;default: 10'h10 ;
 * Description: This register is used to filter pluse whose width is
 * smaller than this value for unit6.
 */

#define PCNT_FILTER_THRES_U6  0x000003FF
#define PCNT_FILTER_THRES_U6_M  ((PCNT_FILTER_THRES_U6_V) << (PCNT_FILTER_THRES_U6_S))
#define PCNT_FILTER_THRES_U6_V  0x3FF
#define PCNT_FILTER_THRES_U6_S  0

#define PCNT_U6_CONF1_REG          (DR_REG_PCNT_BASE + 0x004c)

/* PCNT_CNT_THRES1_U6 : R/W ;bitpos:[31:16] ;default: 10'h0 ;
 * Description: This register is used to configure  thres1 value for unit6.
 */

#define PCNT_CNT_THRES1_U6  0x0000FFFF
#define PCNT_CNT_THRES1_U6_M  ((PCNT_CNT_THRES1_U6_V) << (PCNT_CNT_THRES1_U6_S))
#define PCNT_CNT_THRES1_U6_V  0xFFFF
#define PCNT_CNT_THRES1_U6_S  16

/* PCNT_CNT_THRES0_U6 : R/W ;bitpos:[15:0] ;default: 10'h0 ;
 * Description: This register is used to configure thres0 value for unit6.
 */

#define PCNT_CNT_THRES0_U6  0x0000FFFF
#define PCNT_CNT_THRES0_U6_M  ((PCNT_CNT_THRES0_U6_V) << (PCNT_CNT_THRES0_U6_S))
#define PCNT_CNT_THRES0_U6_V  0xFFFF
#define PCNT_CNT_THRES0_U6_S  0

#define PCNT_U6_CONF2_REG          (DR_REG_PCNT_BASE + 0x0050)

/* PCNT_CNT_L_LIM_U6 : R/W ;bitpos:[31:16] ;default: 10'h0 ;
 * Description: This register is used to confiugre thr_l_lim value for unit6.
 */

#define PCNT_CNT_L_LIM_U6  0x0000FFFF
#define PCNT_CNT_L_LIM_U6_M  ((PCNT_CNT_L_LIM_U6_V) << (PCNT_CNT_L_LIM_U6_S))
#define PCNT_CNT_L_LIM_U6_V  0xFFFF
#define PCNT_CNT_L_LIM_U6_S  16

/* PCNT_CNT_H_LIM_U6 : R/W ;bitpos:[15:0] ;default: 10'h0 ;
 * Description: This register is used to configure thr_h_lim value for unit6.
 */

#define PCNT_CNT_H_LIM_U6  0x0000FFFF
#define PCNT_CNT_H_LIM_U6_M  ((PCNT_CNT_H_LIM_U6_V) << (PCNT_CNT_H_LIM_U6_S))
#define PCNT_CNT_H_LIM_U6_V  0xFFFF
#define PCNT_CNT_H_LIM_U6_S  0

#define PCNT_U7_CONF0_REG          (DR_REG_PCNT_BASE + 0x0054)

/* PCNT_CH1_LCTRL_MODE_U7 : R/W ;bitpos:[31:30] ;
 * default: 0 ;
 * Description: This register is used to control the mode of channel1's low
 * control signal for unit7.
 * 0: increase when control signal is low
 * 1: decrease when control signal is high
 * others: forbidden
 */

#define PCNT_CH1_LCTRL_MODE_U7  0x00000003
#define PCNT_CH1_LCTRL_MODE_U7_M  ((PCNT_CH1_LCTRL_MODE_U7_V) << (PCNT_CH1_LCTRL_MODE_U7_S))
#define PCNT_CH1_LCTRL_MODE_U7_V  0x3
#define PCNT_CH1_LCTRL_MODE_U7_S  30

/* PCNT_CH1_HCTRL_MODE_U7 : R/W ;bitpos:[29:28] ;
 * default: 0 ;
 * Description: This register is used to control the mode of channel1's high
 * control signal for unit7.
 * 0: increase when control signal is low
 * 1: decrease when control signal is high
 * others: forbidden
 */

#define PCNT_CH1_HCTRL_MODE_U7  0x00000003
#define PCNT_CH1_HCTRL_MODE_U7_M  ((PCNT_CH1_HCTRL_MODE_U7_V) << (PCNT_CH1_HCTRL_MODE_U7_S))
#define PCNT_CH1_HCTRL_MODE_U7_V  0x3
#define PCNT_CH1_HCTRL_MODE_U7_S  28

/* PCNT_CH1_POS_MODE_U7 : R/W ;bitpos:[27:26] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel1's input
 * posedge signal for unit7.
 * 1: increase at the posedge of input signal
 * 2:decrease at the posedge of input signal
 * others: forbidden
 */

#define PCNT_CH1_POS_MODE_U7  0x00000003
#define PCNT_CH1_POS_MODE_U7_M  ((PCNT_CH1_POS_MODE_U7_V) << (PCNT_CH1_POS_MODE_U7_S))
#define PCNT_CH1_POS_MODE_U7_V  0x3
#define PCNT_CH1_POS_MODE_U7_S  26

/* PCNT_CH1_NEG_MODE_U7 : R/W ;bitpos:[25:24] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel1's input
 * negedge signal for unit7.
 * 1: increase at the negedge of input signal
 * 2: decrease at the negedge of input signal
 * others: forbidden
 */

#define PCNT_CH1_NEG_MODE_U7  0x00000003
#define PCNT_CH1_NEG_MODE_U7_M  ((PCNT_CH1_NEG_MODE_U7_V) << (PCNT_CH1_NEG_MODE_U7_S))
#define PCNT_CH1_NEG_MODE_U7_V  0x3
#define PCNT_CH1_NEG_MODE_U7_S  24

/* PCNT_CH0_LCTRL_MODE_U7 : R/W ;bitpos:[23:22] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel0's low
 * control signal for unit7.
 * 0: increase when control signal is low
 * 1: decrease when control signal is high
 * others: forbidden
 */

#define PCNT_CH0_LCTRL_MODE_U7  0x00000003
#define PCNT_CH0_LCTRL_MODE_U7_M  ((PCNT_CH0_LCTRL_MODE_U7_V) << (PCNT_CH0_LCTRL_MODE_U7_S))
#define PCNT_CH0_LCTRL_MODE_U7_V  0x3
#define PCNT_CH0_LCTRL_MODE_U7_S  22

/* PCNT_CH0_HCTRL_MODE_U7 : R/W ;bitpos:[21:20] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel0's high
 * control signal for unit7.
 * 0: increase when control signal is low
 * 1: decrease when control signal is high
 * others: forbidden
 */

#define PCNT_CH0_HCTRL_MODE_U7  0x00000003
#define PCNT_CH0_HCTRL_MODE_U7_M  ((PCNT_CH0_HCTRL_MODE_U7_V) << (PCNT_CH0_HCTRL_MODE_U7_S))
#define PCNT_CH0_HCTRL_MODE_U7_V  0x3
#define PCNT_CH0_HCTRL_MODE_U7_S  20

/* PCNT_CH0_POS_MODE_U7 : R/W ;bitpos:[19:18] ;
 * default: 0 ;
 * Description: This register is used to control the mode of channel0's input
 * posedge signal for unit7.
 * 1: increase at the posedge of input signal
 * 2: decrease at the posedge of input signal
 * others: forbidden
 */

#define PCNT_CH0_POS_MODE_U7  0x00000003
#define PCNT_CH0_POS_MODE_U7_M  ((PCNT_CH0_POS_MODE_U7_V) << (PCNT_CH0_POS_MODE_U7_S))
#define PCNT_CH0_POS_MODE_U7_V  0x3
#define PCNT_CH0_POS_MODE_U7_S  18

/* PCNT_CH0_NEG_MODE_U7 : R/W ;bitpos:[17:16] ;default: 2'd0 ;
 * Description: This register is used to control the mode of channel0's
 * input negedge signal for unit7.
 * 1: increase at the negedge of input signal
 * 2: decrease at the negedge of input signal
 * others: forbidden
 */

#define PCNT_CH0_NEG_MODE_U7  0x00000003
#define PCNT_CH0_NEG_MODE_U7_M  ((PCNT_CH0_NEG_MODE_U7_V) << (PCNT_CH0_NEG_MODE_U7_S))
#define PCNT_CH0_NEG_MODE_U7_V  0x3
#define PCNT_CH0_NEG_MODE_U7_S  16

/* PCNT_THR_THRES1_EN_U7 : R/W ;bitpos:[15] ;default: 1'b0 ;
 * Description: This is the enable bit for  comparing  unit7's count with
 * thres1 value.
 */

#define PCNT_THR_THRES1_EN_U7  (BIT(15))
#define PCNT_THR_THRES1_EN_U7_M  (BIT(15))
#define PCNT_THR_THRES1_EN_U7_V  0x1
#define PCNT_THR_THRES1_EN_U7_S  15

/* PCNT_THR_THRES0_EN_U7 : R/W ;bitpos:[14] ;default: 1'b0 ;
 * Description: This is the enable bit for comparing unit7's count with
 * thres0 value.
 */

#define PCNT_THR_THRES0_EN_U7  (BIT(14))
#define PCNT_THR_THRES0_EN_U7_M  (BIT(14))
#define PCNT_THR_THRES0_EN_U7_V  0x1
#define PCNT_THR_THRES0_EN_U7_S  14

/* PCNT_THR_L_LIM_EN_U7 : R/W ;bitpos:[13] ;default: 1'b1 ;
 * Description: This is the enable bit for comparing unit7's count with
 * thr_l_lim value.
 */

#define PCNT_THR_L_LIM_EN_U7  (BIT(13))
#define PCNT_THR_L_LIM_EN_U7_M  (BIT(13))
#define PCNT_THR_L_LIM_EN_U7_V  0x1
#define PCNT_THR_L_LIM_EN_U7_S  13

/* PCNT_THR_H_LIM_EN_U7 : R/W ;bitpos:[12] ;default: 1'b1 ;
 * Description: This is the enable bit for  comparing unit7's count
 * with thr_h_lim value.
 */

#define PCNT_THR_H_LIM_EN_U7  (BIT(12))
#define PCNT_THR_H_LIM_EN_U7_M  (BIT(12))
#define PCNT_THR_H_LIM_EN_U7_V  0x1
#define PCNT_THR_H_LIM_EN_U7_S  12

/* PCNT_THR_ZERO_EN_U7 : R/W ;bitpos:[11] ;default: 1'b1 ;
 * Description: This is the enable bit for comparing unit7's count with 0
 * value.
 */

#define PCNT_THR_ZERO_EN_U7  (BIT(11))
#define PCNT_THR_ZERO_EN_U7_M  (BIT(11))
#define PCNT_THR_ZERO_EN_U7_V  0x1
#define PCNT_THR_ZERO_EN_U7_S  11

/* PCNT_FILTER_EN_U7 : R/W ;bitpos:[10] ;default: 1'b1 ;
 * Description: This is the enable bit for filtering input signals for unit7.
 */

#define PCNT_FILTER_EN_U7  (BIT(10))
#define PCNT_FILTER_EN_U7_M  (BIT(10))
#define PCNT_FILTER_EN_U7_V  0x1
#define PCNT_FILTER_EN_U7_S  10

/* PCNT_FILTER_THRES_U7 : R/W ;bitpos:[9:0] ;default: 10'h10 ;
 * Description: This register is used to filter pluse whose width is smaller
 * than this value for unit7.
 */

#define PCNT_FILTER_THRES_U7  0x000003FF
#define PCNT_FILTER_THRES_U7_M  ((PCNT_FILTER_THRES_U7_V) << (PCNT_FILTER_THRES_U7_S))
#define PCNT_FILTER_THRES_U7_V  0x3FF
#define PCNT_FILTER_THRES_U7_S  0

#define PCNT_U7_CONF1_REG          (DR_REG_PCNT_BASE + 0x0058)

/* PCNT_CNT_THRES1_U7 : R/W ;bitpos:[31:16] ;default: 10'h0 ;
 * Description: This register is used to configure  thres1 value for unit7.
 */

#define PCNT_CNT_THRES1_U7  0x0000FFFF
#define PCNT_CNT_THRES1_U7_M  ((PCNT_CNT_THRES1_U7_V) << (PCNT_CNT_THRES1_U7_S))
#define PCNT_CNT_THRES1_U7_V  0xFFFF
#define PCNT_CNT_THRES1_U7_S  16

/* PCNT_CNT_THRES0_U7 : R/W ;bitpos:[15:0] ;default: 10'h0 ;
 * Description: This register is used to configure thres0 value for unit7.
 */

#define PCNT_CNT_THRES0_U7  0x0000FFFF
#define PCNT_CNT_THRES0_U7_M  ((PCNT_CNT_THRES0_U7_V) << (PCNT_CNT_THRES0_U7_S))
#define PCNT_CNT_THRES0_U7_V  0xFFFF
#define PCNT_CNT_THRES0_U7_S  0

#define PCNT_U7_CONF2_REG          (DR_REG_PCNT_BASE + 0x005c)

/* PCNT_CNT_L_LIM_U7 : R/W ;bitpos:[31:16] ;default: 10'h0 ;
 * Description: This register is used to confiugre thr_l_lim value for unit7.
 */

#define PCNT_CNT_L_LIM_U7  0x0000FFFF
#define PCNT_CNT_L_LIM_U7_M  ((PCNT_CNT_L_LIM_U7_V) << (PCNT_CNT_L_LIM_U7_S))
#define PCNT_CNT_L_LIM_U7_V  0xFFFF
#define PCNT_CNT_L_LIM_U7_S  16

/* PCNT_CNT_H_LIM_U7 : R/W ;bitpos:[15:0] ;default: 10'h0 ;
 * Description: This register is used to configure thr_h_lim value for unit7.
 */

#define PCNT_CNT_H_LIM_U7  0x0000FFFF
#define PCNT_CNT_H_LIM_U7_M  ((PCNT_CNT_H_LIM_U7_V) << (PCNT_CNT_H_LIM_U7_S))
#define PCNT_CNT_H_LIM_U7_V  0xFFFF
#define PCNT_CNT_H_LIM_U7_S  0

#define PCNT_U0_CNT_REG          (DR_REG_PCNT_BASE + 0x0060)

/* PCNT_PLUS_CNT_U0 : RO ;bitpos:[15:0] ;default: 16'h0 ;
 * Description: This register stores the current pulse count value for unit0.
 */

#define PCNT_PLUS_CNT_U0  0x0000FFFF
#define PCNT_PLUS_CNT_U0_M  ((PCNT_PLUS_CNT_U0_V) << (PCNT_PLUS_CNT_U0_S))
#define PCNT_PLUS_CNT_U0_V  0xFFFF
#define PCNT_PLUS_CNT_U0_S  0

#define PCNT_U1_CNT_REG          (DR_REG_PCNT_BASE + 0x0064)

/* PCNT_PLUS_CNT_U1 : RO ;bitpos:[15:0] ;default: 16'h0 ;
 * Description: This register stores the current pulse count value for unit1.
 */

#define PCNT_PLUS_CNT_U1  0x0000FFFF
#define PCNT_PLUS_CNT_U1_M  ((PCNT_PLUS_CNT_U1_V) << (PCNT_PLUS_CNT_U1_S))
#define PCNT_PLUS_CNT_U1_V  0xFFFF
#define PCNT_PLUS_CNT_U1_S  0

#define PCNT_U2_CNT_REG          (DR_REG_PCNT_BASE + 0x0068)

/* PCNT_PLUS_CNT_U2 : RO ;bitpos:[15:0] ;default: 16'h0 ;
 * Description: This register stores the current pulse count value for unit2.
 */

#define PCNT_PLUS_CNT_U2  0x0000FFFF
#define PCNT_PLUS_CNT_U2_M  ((PCNT_PLUS_CNT_U2_V) << (PCNT_PLUS_CNT_U2_S))
#define PCNT_PLUS_CNT_U2_V  0xFFFF
#define PCNT_PLUS_CNT_U2_S  0

#define PCNT_U3_CNT_REG          (DR_REG_PCNT_BASE + 0x006c)

/* PCNT_PLUS_CNT_U3 : RO ;bitpos:[15:0] ;default: 16'h0 ;
 * Description: This register stores the current pulse count value for unit3.
 */

#define PCNT_PLUS_CNT_U3  0x0000FFFF
#define PCNT_PLUS_CNT_U3_M  ((PCNT_PLUS_CNT_U3_V) << (PCNT_PLUS_CNT_U3_S))
#define PCNT_PLUS_CNT_U3_V  0xFFFF
#define PCNT_PLUS_CNT_U3_S  0

#define PCNT_U4_CNT_REG          (DR_REG_PCNT_BASE + 0x0070)

/* PCNT_PLUS_CNT_U4 : RO ;bitpos:[15:0] ;default: 16'h0 ;
 * Description: This register stores the current pulse count value for unit4.
 */

#define PCNT_PLUS_CNT_U4  0x0000FFFF
#define PCNT_PLUS_CNT_U4_M  ((PCNT_PLUS_CNT_U4_V) << (PCNT_PLUS_CNT_U4_S))
#define PCNT_PLUS_CNT_U4_V  0xFFFF
#define PCNT_PLUS_CNT_U4_S  0

#define PCNT_U5_CNT_REG          (DR_REG_PCNT_BASE + 0x0074)

/* PCNT_PLUS_CNT_U5 : RO ;bitpos:[15:0] ;default: 16'h0 ;
 * Description: This register stores the current pulse count value for unit5.
 */

#define PCNT_PLUS_CNT_U5  0x0000FFFF
#define PCNT_PLUS_CNT_U5_M  ((PCNT_PLUS_CNT_U5_V) << (PCNT_PLUS_CNT_U5_S))
#define PCNT_PLUS_CNT_U5_V  0xFFFF
#define PCNT_PLUS_CNT_U5_S  0

#define PCNT_U6_CNT_REG          (DR_REG_PCNT_BASE + 0x0078)

/* PCNT_PLUS_CNT_U6 : RO ;bitpos:[15:0] ;default: 16'h0 ;
 * Description: This register stores the current pulse count value for unit6.
 */

#define PCNT_PLUS_CNT_U6  0x0000FFFF
#define PCNT_PLUS_CNT_U6_M  ((PCNT_PLUS_CNT_U6_V) << (PCNT_PLUS_CNT_U6_S))
#define PCNT_PLUS_CNT_U6_V  0xFFFF
#define PCNT_PLUS_CNT_U6_S  0

#define PCNT_U7_CNT_REG          (DR_REG_PCNT_BASE + 0x007c)

/* PCNT_PLUS_CNT_U7 : RO ;bitpos:[15:0] ;default: 16'h0 ;
 * Description: This register stores the current pulse count value for unit7.
 */

#define PCNT_PLUS_CNT_U7  0x0000FFFF
#define PCNT_PLUS_CNT_U7_M  ((PCNT_PLUS_CNT_U7_V) << (PCNT_PLUS_CNT_U7_S))
#define PCNT_PLUS_CNT_U7_V  0xFFFF
#define PCNT_PLUS_CNT_U7_S  0

#define PCNT_INT_RAW_REG          (DR_REG_PCNT_BASE + 0x0080)

/* PCNT_CNT_THR_EVENT_U7_INT_RAW : RO ;bitpos:[7] ;default: 1'b0 ;
 * Description: This is the interrupt raw bit for channel7 event.
 */

#define PCNT_CNT_THR_EVENT_U7_INT_RAW  (BIT(7))
#define PCNT_CNT_THR_EVENT_U7_INT_RAW_M  (BIT(7))
#define PCNT_CNT_THR_EVENT_U7_INT_RAW_V  0x1
#define PCNT_CNT_THR_EVENT_U7_INT_RAW_S  7

/* PCNT_CNT_THR_EVENT_U6_INT_RAW : RO ;bitpos:[6] ;default: 1'b0 ;
 * Description: This is the interrupt raw bit for channel6 event.
 */

#define PCNT_CNT_THR_EVENT_U6_INT_RAW  (BIT(6))
#define PCNT_CNT_THR_EVENT_U6_INT_RAW_M  (BIT(6))
#define PCNT_CNT_THR_EVENT_U6_INT_RAW_V  0x1
#define PCNT_CNT_THR_EVENT_U6_INT_RAW_S  6

/* PCNT_CNT_THR_EVENT_U5_INT_RAW : RO ;bitpos:[5] ;default: 1'b0 ;
 * Description: This is the interrupt raw bit for channel5 event.
 */

#define PCNT_CNT_THR_EVENT_U5_INT_RAW  (BIT(5))
#define PCNT_CNT_THR_EVENT_U5_INT_RAW_M  (BIT(5))
#define PCNT_CNT_THR_EVENT_U5_INT_RAW_V  0x1
#define PCNT_CNT_THR_EVENT_U5_INT_RAW_S  5

/* PCNT_CNT_THR_EVENT_U4_INT_RAW : RO ;bitpos:[4] ;default: 1'b0 ;
 * Description: This is the interrupt raw bit for channel4 event.
 */

#define PCNT_CNT_THR_EVENT_U4_INT_RAW  (BIT(4))
#define PCNT_CNT_THR_EVENT_U4_INT_RAW_M  (BIT(4))
#define PCNT_CNT_THR_EVENT_U4_INT_RAW_V  0x1
#define PCNT_CNT_THR_EVENT_U4_INT_RAW_S  4

/* PCNT_CNT_THR_EVENT_U3_INT_RAW : RO ;bitpos:[3] ;default: 1'b0 ;
 * Description: This is the interrupt raw bit for channel3 event.
 */

#define PCNT_CNT_THR_EVENT_U3_INT_RAW  (BIT(3))
#define PCNT_CNT_THR_EVENT_U3_INT_RAW_M  (BIT(3))
#define PCNT_CNT_THR_EVENT_U3_INT_RAW_V  0x1
#define PCNT_CNT_THR_EVENT_U3_INT_RAW_S  3

/* PCNT_CNT_THR_EVENT_U2_INT_RAW : RO ;bitpos:[2] ;default: 1'b0 ;
 * Description: This is the interrupt raw bit for channel2 event.
 */

#define PCNT_CNT_THR_EVENT_U2_INT_RAW  (BIT(2))
#define PCNT_CNT_THR_EVENT_U2_INT_RAW_M  (BIT(2))
#define PCNT_CNT_THR_EVENT_U2_INT_RAW_V  0x1
#define PCNT_CNT_THR_EVENT_U2_INT_RAW_S  2

/* PCNT_CNT_THR_EVENT_U1_INT_RAW : RO ;bitpos:[1] ;default: 1'b0 ;
 * Description: This is the interrupt raw bit for channel1 event.
 */

#define PCNT_CNT_THR_EVENT_U1_INT_RAW  (BIT(1))
#define PCNT_CNT_THR_EVENT_U1_INT_RAW_M  (BIT(1))
#define PCNT_CNT_THR_EVENT_U1_INT_RAW_V  0x1
#define PCNT_CNT_THR_EVENT_U1_INT_RAW_S  1

/* PCNT_CNT_THR_EVENT_U0_INT_RAW : RO ;bitpos:[0] ;default: 1'b0 ;
 * Description: This is the interrupt raw bit for channel0 event.
 */

#define PCNT_CNT_THR_EVENT_U0_INT_RAW  (BIT(0))
#define PCNT_CNT_THR_EVENT_U0_INT_RAW_M  (BIT(0))
#define PCNT_CNT_THR_EVENT_U0_INT_RAW_V  0x1
#define PCNT_CNT_THR_EVENT_U0_INT_RAW_S  0

#define PCNT_INT_ST_REG          (DR_REG_PCNT_BASE + 0x0084)

/* PCNT_CNT_THR_EVENT_U7_INT_ST : RO ;bitpos:[7] ;default: 1'b0 ;
 * Description: This is the  interrupt status bit for channel7 event.
 */

#define PCNT_CNT_THR_EVENT_U7_INT_ST  (BIT(7))
#define PCNT_CNT_THR_EVENT_U7_INT_ST_M  (BIT(7))
#define PCNT_CNT_THR_EVENT_U7_INT_ST_V  0x1
#define PCNT_CNT_THR_EVENT_U7_INT_ST_S  7

/* PCNT_CNT_THR_EVENT_U6_INT_ST : RO ;bitpos:[6] ;default: 1'b0 ;
 * Description: This is the  interrupt status bit for channel6 event.
 */

#define PCNT_CNT_THR_EVENT_U6_INT_ST  (BIT(6))
#define PCNT_CNT_THR_EVENT_U6_INT_ST_M  (BIT(6))
#define PCNT_CNT_THR_EVENT_U6_INT_ST_V  0x1
#define PCNT_CNT_THR_EVENT_U6_INT_ST_S  6

/* PCNT_CNT_THR_EVENT_U5_INT_ST : RO ;bitpos:[5] ;default: 1'b0 ;
 * Description: This is the  interrupt status bit for channel5 event.
 */

#define PCNT_CNT_THR_EVENT_U5_INT_ST  (BIT(5))
#define PCNT_CNT_THR_EVENT_U5_INT_ST_M  (BIT(5))
#define PCNT_CNT_THR_EVENT_U5_INT_ST_V  0x1
#define PCNT_CNT_THR_EVENT_U5_INT_ST_S  5

/* PCNT_CNT_THR_EVENT_U4_INT_ST : RO ;bitpos:[4] ;default: 1'b0 ;
 * Description: This is the  interrupt status bit for channel4 event.
 */

#define PCNT_CNT_THR_EVENT_U4_INT_ST  (BIT(4))
#define PCNT_CNT_THR_EVENT_U4_INT_ST_M  (BIT(4))
#define PCNT_CNT_THR_EVENT_U4_INT_ST_V  0x1
#define PCNT_CNT_THR_EVENT_U4_INT_ST_S  4

/* PCNT_CNT_THR_EVENT_U3_INT_ST : RO ;bitpos:[3] ;default: 1'b0 ;
 * Description: This is the  interrupt status bit for channel3 event.
 */

#define PCNT_CNT_THR_EVENT_U3_INT_ST  (BIT(3))
#define PCNT_CNT_THR_EVENT_U3_INT_ST_M  (BIT(3))
#define PCNT_CNT_THR_EVENT_U3_INT_ST_V  0x1
#define PCNT_CNT_THR_EVENT_U3_INT_ST_S  3

/* PCNT_CNT_THR_EVENT_U2_INT_ST : RO; bitpos: [2]; default: 0;
 * The masked interrupt status bit for the PCNT_CNT_THR_EVENT_U2_INT
 * interrupt.
 */

#define PCNT_CNT_THR_EVENT_U2_INT_ST    (BIT(2))
#define PCNT_CNT_THR_EVENT_U2_INT_ST_M  (PCNT_CNT_THR_EVENT_U2_INT_ST_V << PCNT_CNT_THR_EVENT_U2_INT_ST_S)
#define PCNT_CNT_THR_EVENT_U2_INT_ST_V  0x00000001
#define PCNT_CNT_THR_EVENT_U2_INT_ST_S  2

/* PCNT_CNT_THR_EVENT_U1_INT_ST : RO; bitpos: [1]; default: 0;
 * The masked interrupt status bit for the PCNT_CNT_THR_EVENT_U1_INT
 * interrupt.
 */

#define PCNT_CNT_THR_EVENT_U1_INT_ST    (BIT(1))
#define PCNT_CNT_THR_EVENT_U1_INT_ST_M  (PCNT_CNT_THR_EVENT_U1_INT_ST_V << PCNT_CNT_THR_EVENT_U1_INT_ST_S)
#define PCNT_CNT_THR_EVENT_U1_INT_ST_V  0x00000001
#define PCNT_CNT_THR_EVENT_U1_INT_ST_S  1

/* PCNT_CNT_THR_EVENT_U0_INT_ST : RO; bitpos: [0]; default: 0;
 * The masked interrupt status bit for the PCNT_CNT_THR_EVENT_U0_INT
 * interrupt.
 */

#define PCNT_CNT_THR_EVENT_U0_INT_ST    (BIT(0))
#define PCNT_CNT_THR_EVENT_U0_INT_ST_M  (PCNT_CNT_THR_EVENT_U0_INT_ST_V << PCNT_CNT_THR_EVENT_U0_INT_ST_S)
#define PCNT_CNT_THR_EVENT_U0_INT_ST_V  0x00000001
#define PCNT_CNT_THR_EVENT_U0_INT_ST_S  0

/* PCNT_INT_ENA_REG register
 * Interrupt enable register
 */

#define PCNT_INT_ENA_REG          (DR_REG_PCNT_BASE + 0x0088)

/* PCNT_CNT_THR_EVENT_U7_INT_ENA : R/W ;bitpos:[7] ;default: 1'b0 ;
 * Description: This is the  interrupt enable bit for channel7 event.
 */

#define PCNT_CNT_THR_EVENT_U7_INT_ENA  (BIT(7))
#define PCNT_CNT_THR_EVENT_U7_INT_ENA_M  (BIT(7))
#define PCNT_CNT_THR_EVENT_U7_INT_ENA_V  0x1
#define PCNT_CNT_THR_EVENT_U7_INT_ENA_S  7

/* PCNT_CNT_THR_EVENT_U6_INT_ENA : R/W ;bitpos:[6] ;default: 1'b0 ;
 * Description: This is the  interrupt enable bit for channel6 event.
 */

#define PCNT_CNT_THR_EVENT_U6_INT_ENA  (BIT(6))
#define PCNT_CNT_THR_EVENT_U6_INT_ENA_M  (BIT(6))
#define PCNT_CNT_THR_EVENT_U6_INT_ENA_V  0x1
#define PCNT_CNT_THR_EVENT_U6_INT_ENA_S  6

/* PCNT_CNT_THR_EVENT_U5_INT_ENA : R/W ;bitpos:[5] ;default: 1'b0 ;
 * Description: This is the  interrupt enable bit for channel5 event.
 */

#define PCNT_CNT_THR_EVENT_U5_INT_ENA  (BIT(5))
#define PCNT_CNT_THR_EVENT_U5_INT_ENA_M  (BIT(5))
#define PCNT_CNT_THR_EVENT_U5_INT_ENA_V  0x1
#define PCNT_CNT_THR_EVENT_U5_INT_ENA_S  5

/* PCNT_CNT_THR_EVENT_U4_INT_ENA : R/W ;bitpos:[4] ;default: 1'b0 ;
 * Description: This is the  interrupt enable bit for channel4 event.
 */

#define PCNT_CNT_THR_EVENT_U4_INT_ENA  (BIT(4))
#define PCNT_CNT_THR_EVENT_U4_INT_ENA_M  (BIT(4))
#define PCNT_CNT_THR_EVENT_U4_INT_ENA_V  0x1
#define PCNT_CNT_THR_EVENT_U4_INT_ENA_S  4

/* PCNT_CNT_THR_EVENT_U3_INT_ENA : R/W ;bitpos:[3] ;default: 1'b0 ;
 * Description: This is the  interrupt enable bit for channel3 event.
 */

#define PCNT_CNT_THR_EVENT_U3_INT_ENA  (BIT(3))
#define PCNT_CNT_THR_EVENT_U3_INT_ENA_M  (BIT(3))
#define PCNT_CNT_THR_EVENT_U3_INT_ENA_V  0x1
#define PCNT_CNT_THR_EVENT_U3_INT_ENA_S  3

/* PCNT_CNT_THR_EVENT_U2_INT_ENA : R/W ;bitpos:[2] ;default: 1'b0 ;
 * Description: This is the  interrupt enable bit for channel2 event.
 */

#define PCNT_CNT_THR_EVENT_U2_INT_ENA  (BIT(2))
#define PCNT_CNT_THR_EVENT_U2_INT_ENA_M  (BIT(2))
#define PCNT_CNT_THR_EVENT_U2_INT_ENA_V  0x1
#define PCNT_CNT_THR_EVENT_U2_INT_ENA_S  2

/* PCNT_CNT_THR_EVENT_U1_INT_ENA : R/W ;bitpos:[1] ;default: 1'b0 ;
 * Description: This is the  interrupt enable bit for channel1 event.
 */

#define PCNT_CNT_THR_EVENT_U1_INT_ENA  (BIT(1))
#define PCNT_CNT_THR_EVENT_U1_INT_ENA_M  (BIT(1))
#define PCNT_CNT_THR_EVENT_U1_INT_ENA_V  0x1
#define PCNT_CNT_THR_EVENT_U1_INT_ENA_S  1

/* PCNT_CNT_THR_EVENT_U0_INT_ENA : R/W ;bitpos:[0] ;default: 1'b0 ;
 * Description: This is the  interrupt enable bit for channel0 event.
 */

#define PCNT_CNT_THR_EVENT_U0_INT_ENA  (BIT(0))
#define PCNT_CNT_THR_EVENT_U0_INT_ENA_M  (BIT(0))
#define PCNT_CNT_THR_EVENT_U0_INT_ENA_V  0x1
#define PCNT_CNT_THR_EVENT_U0_INT_ENA_S  0

#define PCNT_INT_CLR_REG          (DR_REG_PCNT_BASE + 0x008c)

/* PCNT_CNT_THR_EVENT_U7_INT_CLR : WO ;bitpos:[7] ;default: 1'b0 ;
 * Description: Set this bit to clear channel7 event interrupt.
 */

#define PCNT_CNT_THR_EVENT_U7_INT_CLR  (BIT(7))
#define PCNT_CNT_THR_EVENT_U7_INT_CLR_M  (BIT(7))
#define PCNT_CNT_THR_EVENT_U7_INT_CLR_V  0x1
#define PCNT_CNT_THR_EVENT_U7_INT_CLR_S  7

/* PCNT_CNT_THR_EVENT_U6_INT_CLR : WO ;bitpos:[6] ;default: 1'b0 ;
 * Description: Set this bit to clear channel6 event interrupt.
 */

#define PCNT_CNT_THR_EVENT_U6_INT_CLR  (BIT(6))
#define PCNT_CNT_THR_EVENT_U6_INT_CLR_M  (BIT(6))
#define PCNT_CNT_THR_EVENT_U6_INT_CLR_V  0x1
#define PCNT_CNT_THR_EVENT_U6_INT_CLR_S  6

/* PCNT_CNT_THR_EVENT_U5_INT_CLR : WO ;bitpos:[5] ;default: 1'b0 ;
 * Description: Set this bit to clear channel5 event interrupt.
 */

#define PCNT_CNT_THR_EVENT_U5_INT_CLR  (BIT(5))
#define PCNT_CNT_THR_EVENT_U5_INT_CLR_M  (BIT(5))
#define PCNT_CNT_THR_EVENT_U5_INT_CLR_V  0x1
#define PCNT_CNT_THR_EVENT_U5_INT_CLR_S  5

/* PCNT_CNT_THR_EVENT_U4_INT_CLR : WO ;bitpos:[4] ;default: 1'b0 ;
 * Description: Set this bit to clear channel4 event interrupt.
 */

#define PCNT_CNT_THR_EVENT_U4_INT_CLR  (BIT(4))
#define PCNT_CNT_THR_EVENT_U4_INT_CLR_M  (BIT(4))
#define PCNT_CNT_THR_EVENT_U4_INT_CLR_V  0x1
#define PCNT_CNT_THR_EVENT_U4_INT_CLR_S  4

/* PCNT_CNT_THR_EVENT_U3_INT_CLR : WO ;bitpos:[3] ;default: 1'b0 ;
 * Description: Set this bit to clear channel3 event interrupt.
 */

#define PCNT_CNT_THR_EVENT_U3_INT_CLR  (BIT(3))
#define PCNT_CNT_THR_EVENT_U3_INT_CLR_M  (BIT(3))
#define PCNT_CNT_THR_EVENT_U3_INT_CLR_V  0x1
#define PCNT_CNT_THR_EVENT_U3_INT_CLR_S  3

/* PCNT_CNT_THR_EVENT_U2_INT_CLR : WO; bitpos: [2]; default: 0;
 * Set this bit to clear the PCNT_CNT_THR_EVENT_U2_INT interrupt.
 */

#define PCNT_CNT_THR_EVENT_U2_INT_CLR    (BIT(2))
#define PCNT_CNT_THR_EVENT_U2_INT_CLR_M  (PCNT_CNT_THR_EVENT_U2_INT_CLR_V << PCNT_CNT_THR_EVENT_U2_INT_CLR_S)
#define PCNT_CNT_THR_EVENT_U2_INT_CLR_V  0x00000001
#define PCNT_CNT_THR_EVENT_U2_INT_CLR_S  2

/* PCNT_CNT_THR_EVENT_U1_INT_CLR : WO ;bitpos:[1] ;default: 1'b0 ;
 * Description: Set this bit to clear channel1 event interrupt.
 */

#define PCNT_CNT_THR_EVENT_U1_INT_CLR  (BIT(1))
#define PCNT_CNT_THR_EVENT_U1_INT_CLR_M  (BIT(1))
#define PCNT_CNT_THR_EVENT_U1_INT_CLR_V  0x1
#define PCNT_CNT_THR_EVENT_U1_INT_CLR_S  1

/* PCNT_CNT_THR_EVENT_U0_INT_CLR : WO ;bitpos:[0] ;default: 1'b0 ;
 * Description: Set this bit to clear channel0 event interrupt.
 */

#define PCNT_CNT_THR_EVENT_U0_INT_CLR  (BIT(0))
#define PCNT_CNT_THR_EVENT_U0_INT_CLR_M  (BIT(0))
#define PCNT_CNT_THR_EVENT_U0_INT_CLR_V  0x1
#define PCNT_CNT_THR_EVENT_U0_INT_CLR_S  0

#define PCNT_U0_STATUS_REG          (DR_REG_PCNT_BASE + 0x0090)

/* PCNT_CORE_STATUS_U0 : RO ;bitpos:[31:0] ;default: 32'h0 ; */

#define PCNT_CORE_STATUS_U0  0xFFFFFFFF
#define PCNT_CORE_STATUS_U0_M  ((PCNT_CORE_STATUS_U0_V) << (PCNT_CORE_STATUS_U0_S))
#define PCNT_CORE_STATUS_U0_V  0xFFFFFFFF
#define PCNT_CORE_STATUS_U0_S  0

/* 0: positive value to zero;
 * 1: negative value to zero;
 * 2: counter value negative;
 * 3: counter value positive
 */

#define PCNT_STATUS_CNT_MODE    0x3
#define PCNT_STATUS_CNT_MODE_M  ((PCNT_STATUS_CNT_MODE_V) << (PCNT_STATUS_CNT_MODE_S))
#define PCNT_STATUS_CNT_MODE_V  0x3
#define PCNT_STATUS_CNT_MODE_S  0

/* counter value equals to thresh1 */

#define PCNT_STATUS_THRES1    BIT(2)
#define PCNT_STATUS_THRES1_M  BIT(2)
#define PCNT_STATUS_THRES1_V  0x1
#define PCNT_STATUS_THRES1_S  2

/* counter value equals to thresh0 */

#define PCNT_STATUS_THRES0    BIT(3)
#define PCNT_STATUS_THRES0_M  BIT(3)
#define PCNT_STATUS_THRES0_V  0x1
#define PCNT_STATUS_THRES0_S  3

/* counter value reaches h_lim */

#define PCNT_STATUS_L_LIM    BIT(4)
#define PCNT_STATUS_L_LIM_M  BIT(4)
#define PCNT_STATUS_L_LIM_V  0x1
#define PCNT_STATUS_L_LIM_S  4

/* counter value reaches l_lim */

#define PCNT_STATUS_H_LIM    BIT(5)
#define PCNT_STATUS_H_LIM_M  BIT(5)
#define PCNT_STATUS_H_LIM_V  0x1
#define PCNT_STATUS_H_LIM_S  5

/* counter value equals to zero */

#define PCNT_STATUS_ZERO    BIT(6)
#define PCNT_STATUS_ZERO_M  BIT(6)
#define PCNT_STATUS_ZERO_V  0x1
#define PCNT_STATUS_ZERO_S  6

#define PCNT_U1_STATUS_REG          (DR_REG_PCNT_BASE + 0x0094)

/* PCNT_CORE_STATUS_U1 : RO ;bitpos:[31:0] ;default: 32'h0 ; */

/* Description: */

#define PCNT_CORE_STATUS_U1  0xFFFFFFFF
#define PCNT_CORE_STATUS_U1_M  ((PCNT_CORE_STATUS_U1_V) << (PCNT_CORE_STATUS_U1_S))
#define PCNT_CORE_STATUS_U1_V  0xFFFFFFFF
#define PCNT_CORE_STATUS_U1_S  0

#define PCNT_U2_STATUS_REG          (DR_REG_PCNT_BASE + 0x0098)

/* PCNT_CORE_STATUS_U2 : RO ;bitpos:[31:0] ;default: 32'h0 ; */

#define PCNT_CORE_STATUS_U2  0xFFFFFFFF
#define PCNT_CORE_STATUS_U2_M  ((PCNT_CORE_STATUS_U2_V) << (PCNT_CORE_STATUS_U2_S))
#define PCNT_CORE_STATUS_U2_V  0xFFFFFFFF
#define PCNT_CORE_STATUS_U2_S  0

#define PCNT_U3_STATUS_REG          (DR_REG_PCNT_BASE + 0x009c)

/* PCNT_CORE_STATUS_U3 : RO ;bitpos:[31:0] ;default: 32'h0 ; */

#define PCNT_CORE_STATUS_U3  0xFFFFFFFF
#define PCNT_CORE_STATUS_U3_M  ((PCNT_CORE_STATUS_U3_V) << (PCNT_CORE_STATUS_U3_S))
#define PCNT_CORE_STATUS_U3_V  0xFFFFFFFF
#define PCNT_CORE_STATUS_U3_S  0

#define PCNT_U4_STATUS_REG          (DR_REG_PCNT_BASE + 0x00a0)
/* PCNT_CORE_STATUS_U4 : RO ;bitpos:[31:0] ;default: 32'h0 ; */

/* Description: */

#define PCNT_CORE_STATUS_U4  0xFFFFFFFF
#define PCNT_CORE_STATUS_U4_M  ((PCNT_CORE_STATUS_U4_V) << (PCNT_CORE_STATUS_U4_S))
#define PCNT_CORE_STATUS_U4_V  0xFFFFFFFF
#define PCNT_CORE_STATUS_U4_S  0

#define PCNT_U5_STATUS_REG          (DR_REG_PCNT_BASE + 0x00a4)
/* PCNT_CORE_STATUS_U5 : RO ;bitpos:[31:0] ;default: 32'h0 ; */

/* Description: */

#define PCNT_CORE_STATUS_U5  0xFFFFFFFF
#define PCNT_CORE_STATUS_U5_M  ((PCNT_CORE_STATUS_U5_V) << (PCNT_CORE_STATUS_U5_S))
#define PCNT_CORE_STATUS_U5_V  0xFFFFFFFF
#define PCNT_CORE_STATUS_U5_S  0

#define PCNT_U6_STATUS_REG          (DR_REG_PCNT_BASE + 0x00a8)
/* PCNT_CORE_STATUS_U6 : RO ;bitpos:[31:0] ;default: 32'h0 ; */

/* Description: */

#define PCNT_CORE_STATUS_U6  0xFFFFFFFF
#define PCNT_CORE_STATUS_U6_M  ((PCNT_CORE_STATUS_U6_V) << (PCNT_CORE_STATUS_U6_S))
#define PCNT_CORE_STATUS_U6_V  0xFFFFFFFF
#define PCNT_CORE_STATUS_U6_S  0

#define PCNT_U7_STATUS_REG          (DR_REG_PCNT_BASE + 0x00ac)

/* PCNT_CORE_STATUS_U7 : RO ;bitpos:[31:0] ;default: 32'h0 ; */

#define PCNT_CORE_STATUS_U7  0xFFFFFFFF
#define PCNT_CORE_STATUS_U7_M  ((PCNT_CORE_STATUS_U7_V) << (PCNT_CORE_STATUS_U7_S))
#define PCNT_CORE_STATUS_U7_V  0xFFFFFFFF
#define PCNT_CORE_STATUS_U7_S  0

#define PCNT_CTRL_REG          (DR_REG_PCNT_BASE + 0x00b0)

/* PCNT_CLK_EN : R/W ;bitpos:[16] ;default: 1'b0 ; */

#define PCNT_CLK_EN  (BIT(16))
#define PCNT_CLK_EN_M  (BIT(16))
#define PCNT_CLK_EN_V  0x1
#define PCNT_CLK_EN_S  16

/* PCNT_CNT_PAUSE_U7 : R/W ;bitpos:[15] ;default: 1'b0 ; */

/* Description: Set this bit to pause unit7's counter. */

#define PCNT_CNT_PAUSE_U7  (BIT(15))
#define PCNT_CNT_PAUSE_U7_M  (BIT(15))
#define PCNT_CNT_PAUSE_U7_V  0x1
#define PCNT_CNT_PAUSE_U7_S  15

/* PCNT_PLUS_CNT_RST_U7 : R/W ;bitpos:[14] ;default: 1'b1 ; */

/* Description: Set this bit to clear unit7's counter. */

#define PCNT_PLUS_CNT_RST_U7  (BIT(14))
#define PCNT_PLUS_CNT_RST_U7_M  (BIT(14))
#define PCNT_PLUS_CNT_RST_U7_V  0x1
#define PCNT_PLUS_CNT_RST_U7_S  14

/* PCNT_CNT_PAUSE_U6 : R/W ;bitpos:[13] ;default: 1'b0 ; */

/* Description: Set this bit to pause unit6's counter. */

#define PCNT_CNT_PAUSE_U6  (BIT(13))
#define PCNT_CNT_PAUSE_U6_M  (BIT(13))
#define PCNT_CNT_PAUSE_U6_V  0x1
#define PCNT_CNT_PAUSE_U6_S  13

/* PCNT_PLUS_CNT_RST_U6 : R/W ;bitpos:[12] ;default: 1'b1 ; */

/* Description: Set this bit to clear unit6's counter. */

#define PCNT_PLUS_CNT_RST_U6  (BIT(12))
#define PCNT_PLUS_CNT_RST_U6_M  (BIT(12))
#define PCNT_PLUS_CNT_RST_U6_V  0x1
#define PCNT_PLUS_CNT_RST_U6_S  12

/* PCNT_CNT_PAUSE_U5 : R/W ;bitpos:[11] ;default: 1'b0 ; */

/* Description: Set this bit to pause unit5's counter. */

#define PCNT_CNT_PAUSE_U5  (BIT(11))
#define PCNT_CNT_PAUSE_U5_M  (BIT(11))
#define PCNT_CNT_PAUSE_U5_V  0x1
#define PCNT_CNT_PAUSE_U5_S  11

/* PCNT_PLUS_CNT_RST_U5 : R/W ;bitpos:[10] ;default: 1'b1 ; */

/* Description: Set this bit to clear unit5's counter. */

#define PCNT_PLUS_CNT_RST_U5  (BIT(10))
#define PCNT_PLUS_CNT_RST_U5_M  (BIT(10))
#define PCNT_PLUS_CNT_RST_U5_V  0x1
#define PCNT_PLUS_CNT_RST_U5_S  10

/* PCNT_CNT_PAUSE_U4 : R/W ;bitpos:[9] ;default: 1'b0 ; */

/* Description: Set this bit to pause unit4's counter. */

#define PCNT_CNT_PAUSE_U4  (BIT(9))
#define PCNT_CNT_PAUSE_U4_M  (BIT(9))
#define PCNT_CNT_PAUSE_U4_V  0x1
#define PCNT_CNT_PAUSE_U4_S  9

/* PCNT_PLUS_CNT_RST_U4 : R/W ;bitpos:[8] ;default: 1'b1 ; */

/* Description: Set this bit to clear unit4's counter. */

#define PCNT_PLUS_CNT_RST_U4  (BIT(8))
#define PCNT_PLUS_CNT_RST_U4_M  (BIT(8))
#define PCNT_PLUS_CNT_RST_U4_V  0x1
#define PCNT_PLUS_CNT_RST_U4_S  8

/* PCNT_CNT_PAUSE_U3 : R/W ;bitpos:[7] ;default: 1'b0 ; */

/* Description: Set this bit to pause unit3's counter. */

#define PCNT_CNT_PAUSE_U3  (BIT(7))
#define PCNT_CNT_PAUSE_U3_M  (BIT(7))
#define PCNT_CNT_PAUSE_U3_V  0x1
#define PCNT_CNT_PAUSE_U3_S  7

/* PCNT_PLUS_CNT_RST_U3 : R/W ;bitpos:[6] ;default: 1'b1 ; */

/* Description: Set this bit to clear unit3's counter. */

#define PCNT_PLUS_CNT_RST_U3  (BIT(6))
#define PCNT_PLUS_CNT_RST_U3_M  (BIT(6))
#define PCNT_PLUS_CNT_RST_U3_V  0x1
#define PCNT_PLUS_CNT_RST_U3_S  6

/* PCNT_CNT_PAUSE_U2 : R/W ;bitpos:[5] ;default: 1'b0 ; */

/* Description: Set this bit to pause unit2's counter. */

#define PCNT_CNT_PAUSE_U2  (BIT(5))
#define PCNT_CNT_PAUSE_U2_M  (BIT(5))
#define PCNT_CNT_PAUSE_U2_V  0x1
#define PCNT_CNT_PAUSE_U2_S  5

/* PCNT_PLUS_CNT_RST_U2 : R/W ;bitpos:[4] ;default: 1'b1 ; */

/* Description: Set this bit to clear unit2's counter. */

#define PCNT_PLUS_CNT_RST_U2  (BIT(4))
#define PCNT_PLUS_CNT_RST_U2_M  (BIT(4))
#define PCNT_PLUS_CNT_RST_U2_V  0x1
#define PCNT_PLUS_CNT_RST_U2_S  4

/* PCNT_CNT_PAUSE_U1 : R/W ;bitpos:[3] ;default: 1'b0 ; */

/* Description: Set this bit to pause unit1's counter. */

#define PCNT_CNT_PAUSE_U1  (BIT(3))
#define PCNT_CNT_PAUSE_U1_M  (BIT(3))
#define PCNT_CNT_PAUSE_U1_V  0x1
#define PCNT_CNT_PAUSE_U1_S  3

/* PCNT_PLUS_CNT_RST_U1 : R/W ;bitpos:[2] ;default: 1'b1 ; */

/* Description: Set this bit to clear unit1's counter. */

#define PCNT_PLUS_CNT_RST_U1  (BIT(2))
#define PCNT_PLUS_CNT_RST_U1_M  (BIT(2))
#define PCNT_PLUS_CNT_RST_U1_V  0x1
#define PCNT_PLUS_CNT_RST_U1_S  2

/* PCNT_CNT_PAUSE_U0 : R/W ;bitpos:[1] ;default: 1'b0 ; */

/* Description: Set this bit to pause unit0's counter. */

#define PCNT_CNT_PAUSE_U0  (BIT(1))
#define PCNT_CNT_PAUSE_U0_M  (BIT(1))
#define PCNT_CNT_PAUSE_U0_V  0x1
#define PCNT_CNT_PAUSE_U0_S  1

/* PCNT_PLUS_CNT_RST_U0 : R/W ;bitpos:[0] ;default: 1'b1 ; */

/* Description: Set this bit to clear unit0's counter. */

#define PCNT_PLUS_CNT_RST_U0  (BIT(0))
#define PCNT_PLUS_CNT_RST_U0_M  (BIT(0))
#define PCNT_PLUS_CNT_RST_U0_V  0x1
#define PCNT_PLUS_CNT_RST_U0_S  0

#define PCNT_DATE_REG          (DR_REG_PCNT_BASE + 0x00fc)

/* PCNT_DATE : R/W ;bitpos:[31:0] ;default: 32'h14122600 ; */

#define PCNT_DATE    0xFFFFFFFF
#define PCNT_DATE_M  ((PCNT_DATE_V) << (PCNT_DATE_S))
#define PCNT_DATE_V  0xFFFFFFFF
#define PCNT_DATE_S  0

/* Index macros for CONF0/1/2 */

#define PCNT_CONF0_U(X)   (PCNT_U0_CONF0_REG + (X) * 12)
#define PCNT_CONF1_U(X)   (PCNT_U0_CONF1_REG + (X) * 12)
#define PCNT_CONF2_U(X)   (PCNT_U0_CONF2_REG + (X) * 12)

/* Index macros for CONT */

#define PCNT_CNT_U(X)     (PCNT_U0_CNT_REG + (X) * 4)

/* Index macros for STATUS */

#define PCNT_STATUS_U(X)  (PCNT_U0_STATUS_REG + (X) * 4)

/* PCNT Reset bit, Even bits: 0, 2, 4, 6 */

#define PCNT_CNT_RST_U(X) (1 << ((X) * 2))

/* PCNT channel action on control level */

#define PCNT_MODE_KEEP    0 /* Control mode: won't change counter mode */
#define PCNT_MODE_REVERSE 1 /* Control mode: invert counter mode(increase -> decrease, decrease -> increase) */
#define PCNT_MODE_DISABLE 2 /* Control mode: Inhibit counter(counter value will not change in this condition) */
#define PCNT_MODE_MAX     3

/* PCNT channel action on signal edge */

#define PCNT_COUNT_DIS    0 /* Counter mode: Inhibit counter(counter value will not change in this condition) */
#define PCNT_COUNT_INC    1 /* Counter mode: Increase counter value */
#define PCNT_COUNT_DEC    2 /* Counter mode: Decrease counter value */
#define PCNT_COUNT_MAX    3

#endif /* __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_PCNT_H */
