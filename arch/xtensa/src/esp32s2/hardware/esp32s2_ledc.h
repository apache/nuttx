/****************************************************************************
 * arch/xtensa/src/esp32s2/hardware/esp32s2_ledc.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_LEDC_H
#define __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_LEDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32s2_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDC_CH0_CONF0_REG register
 * Configuration register 0 for channel 0
 */

#define LEDC_CH0_CONF0_REG (DR_REG_LEDC_BASE + 0x0)

/* LEDC_OVF_CNT_RESET_ST_CH0 : R/WTC/SC; bitpos: [17]; default: 0;
 * This is the status bit of LEDC_OVF_CNT_RESET_CH0.
 */

#define LEDC_OVF_CNT_RESET_ST_CH0    (BIT(17))
#define LEDC_OVF_CNT_RESET_ST_CH0_M  (LEDC_OVF_CNT_RESET_ST_CH0_V << LEDC_OVF_CNT_RESET_ST_CH0_S)
#define LEDC_OVF_CNT_RESET_ST_CH0_V  0x00000001
#define LEDC_OVF_CNT_RESET_ST_CH0_S  17

/* LEDC_OVF_CNT_RESET_CH0 : WT; bitpos: [16]; default: 0;
 * Set this bit to reset the ovf_cnt of channel 0.
 */

#define LEDC_OVF_CNT_RESET_CH0    (BIT(16))
#define LEDC_OVF_CNT_RESET_CH0_M  (LEDC_OVF_CNT_RESET_CH0_V << LEDC_OVF_CNT_RESET_CH0_S)
#define LEDC_OVF_CNT_RESET_CH0_V  0x00000001
#define LEDC_OVF_CNT_RESET_CH0_S  16

/* LEDC_OVF_CNT_EN_CH0 : R/W; bitpos: [15]; default: 0;
 * This bit is used to enable the ovf_cnt of channel 0.
 */

#define LEDC_OVF_CNT_EN_CH0    (BIT(15))
#define LEDC_OVF_CNT_EN_CH0_M  (LEDC_OVF_CNT_EN_CH0_V << LEDC_OVF_CNT_EN_CH0_S)
#define LEDC_OVF_CNT_EN_CH0_V  0x00000001
#define LEDC_OVF_CNT_EN_CH0_S  15

/* LEDC_OVF_NUM_CH0 : R/W; bitpos: [14:5]; default: 0;
 * This register is used to configure the maximum times of overflow minus 1.
 *
 * The LEDC_OVF_CNT_CH0_INT interrupt will be triggered when channel 0
 * overflows for (LEDC_OVF_NUM_CH0 + 1) times.
 */

#define LEDC_OVF_NUM_CH0    0x000003ff
#define LEDC_OVF_NUM_CH0_M  (LEDC_OVF_NUM_CH0_V << LEDC_OVF_NUM_CH0_S)
#define LEDC_OVF_NUM_CH0_V  0x000003ff
#define LEDC_OVF_NUM_CH0_S  5

/* LEDC_PARA_UP_CH0 : WT; bitpos: [4]; default: 0;
 * This bit is used to update LEDC_HPOINT_CH0, LEDC_DUTY_START_CH0,
 * LEDC_SIG_OUT_EN_CH0, LEDC_TIMER_SEL_CH0, LEDC_DUTY_NUM_CH0,
 * LEDC_DUTY_CYCLE_CH0, LEDC_DUTY_SCALE_CH0, LEDC_DUTY_INC_CH0, and
 * LEDC_OVF_CNT_EN_CH0 fields for channel 0, and will be automatically
 * cleared by hardware.
 */

#define LEDC_PARA_UP_CH0    (BIT(4))
#define LEDC_PARA_UP_CH0_M  (LEDC_PARA_UP_CH0_V << LEDC_PARA_UP_CH0_S)
#define LEDC_PARA_UP_CH0_V  0x00000001
#define LEDC_PARA_UP_CH0_S  4

/* LEDC_IDLE_LV_CH0 : R/W; bitpos: [3]; default: 0;
 * This bit is used to control the output value when channel 0 is inactive
 * (when LEDC_SIG_OUT_EN_CH0 is 0).
 */

#define LEDC_IDLE_LV_CH0    (BIT(3))
#define LEDC_IDLE_LV_CH0_M  (LEDC_IDLE_LV_CH0_V << LEDC_IDLE_LV_CH0_S)
#define LEDC_IDLE_LV_CH0_V  0x00000001
#define LEDC_IDLE_LV_CH0_S  3

/* LEDC_SIG_OUT_EN_CH0 : R/W; bitpos: [2]; default: 0;
 * Set this bit to enable signal output on channel 0.
 */

#define LEDC_SIG_OUT_EN_CH0    (BIT(2))
#define LEDC_SIG_OUT_EN_CH0_M  (LEDC_SIG_OUT_EN_CH0_V << LEDC_SIG_OUT_EN_CH0_S)
#define LEDC_SIG_OUT_EN_CH0_V  0x00000001
#define LEDC_SIG_OUT_EN_CH0_S  2

/* LEDC_TIMER_SEL_CH0 : R/W; bitpos: [1:0]; default: 0;
 * This field is used to select one of timers for channel 0.
 *
 * 0: select timer0
 *
 * 1: select timer1
 *
 * 2: select timer2
 *
 * 3: select timer3
 */

#define LEDC_TIMER_SEL_CH0    0x00000003
#define LEDC_TIMER_SEL_CH0_M  (LEDC_TIMER_SEL_CH0_V << LEDC_TIMER_SEL_CH0_S)
#define LEDC_TIMER_SEL_CH0_V  0x00000003
#define LEDC_TIMER_SEL_CH0_S  0

/* LEDC_CH0_HPOINT_REG register
 * High point register for channel 0
 */

#define LEDC_CH0_HPOINT_REG (DR_REG_LEDC_BASE + 0x4)

/* LEDC_HPOINT_CH0 : R/W; bitpos: [13:0]; default: 0;
 * The output value changes to high when the selected timers has reached the
 * value specified by this register.
 */

#define LEDC_HPOINT_CH0    0x00003fff
#define LEDC_HPOINT_CH0_M  (LEDC_HPOINT_CH0_V << LEDC_HPOINT_CH0_S)
#define LEDC_HPOINT_CH0_V  0x00003fff
#define LEDC_HPOINT_CH0_S  0

/* LEDC_CH0_DUTY_REG register
 * Initial duty cycle for channel 0
 */

#define LEDC_CH0_DUTY_REG (DR_REG_LEDC_BASE + 0x8)

/* LEDC_DUTY_CH0 : R/W; bitpos: [18:0]; default: 0;
 * This register is used to change the output duty by controlling the Lpoint.
 *
 * The output value turns to low when the selected timers has reached the
 * Lpoint.
 */

#define LEDC_DUTY_CH0    0x0007ffff
#define LEDC_DUTY_CH0_M  (LEDC_DUTY_CH0_V << LEDC_DUTY_CH0_S)
#define LEDC_DUTY_CH0_V  0x0007ffff
#define LEDC_DUTY_CH0_S  0

/* LEDC_CH0_CONF1_REG register
 * Configuration register 1 for channel 0
 */

#define LEDC_CH0_CONF1_REG (DR_REG_LEDC_BASE + 0xc)

/* LEDC_DUTY_START_CH0 : R/W/SC; bitpos: [31]; default: 0;
 * Other configured fields in LEDC_CH0_CONF1_REG will start to take effect
 * when this bit is set to 1.
 */

#define LEDC_DUTY_START_CH0    (BIT(31))
#define LEDC_DUTY_START_CH0_M  (LEDC_DUTY_START_CH0_V << LEDC_DUTY_START_CH0_S)
#define LEDC_DUTY_START_CH0_V  0x00000001
#define LEDC_DUTY_START_CH0_S  31

/* LEDC_DUTY_INC_CH0 : R/W; bitpos: [30]; default: 1;
 * This register is used to increase or decrease the duty of output signal
 * on channel 0. 1: Increase; 0: Decrease.
 */

#define LEDC_DUTY_INC_CH0    (BIT(30))
#define LEDC_DUTY_INC_CH0_M  (LEDC_DUTY_INC_CH0_V << LEDC_DUTY_INC_CH0_S)
#define LEDC_DUTY_INC_CH0_V  0x00000001
#define LEDC_DUTY_INC_CH0_S  30

/* LEDC_DUTY_NUM_CH0 : R/W; bitpos: [29:20]; default: 0;
 * This register is used to control the number of times the duty cycle will
 * be changed.
 */

#define LEDC_DUTY_NUM_CH0    0x000003ff
#define LEDC_DUTY_NUM_CH0_M  (LEDC_DUTY_NUM_CH0_V << LEDC_DUTY_NUM_CH0_S)
#define LEDC_DUTY_NUM_CH0_V  0x000003ff
#define LEDC_DUTY_NUM_CH0_S  20

/* LEDC_DUTY_CYCLE_CH0 : R/W; bitpos: [19:10]; default: 0;
 * The duty will change every LEDC_DUTY_CYCLE_CH0 on channel 0.
 */

#define LEDC_DUTY_CYCLE_CH0    0x000003ff
#define LEDC_DUTY_CYCLE_CH0_M  (LEDC_DUTY_CYCLE_CH0_V << LEDC_DUTY_CYCLE_CH0_S)
#define LEDC_DUTY_CYCLE_CH0_V  0x000003ff
#define LEDC_DUTY_CYCLE_CH0_S  10

/* LEDC_DUTY_SCALE_CH0 : R/W; bitpos: [9:0]; default: 0;
 * This register is used to configure the changing step scale of duty on
 * channel 0.
 */

#define LEDC_DUTY_SCALE_CH0    0x000003ff
#define LEDC_DUTY_SCALE_CH0_M  (LEDC_DUTY_SCALE_CH0_V << LEDC_DUTY_SCALE_CH0_S)
#define LEDC_DUTY_SCALE_CH0_V  0x000003ff
#define LEDC_DUTY_SCALE_CH0_S  0

/* LEDC_CH0_DUTY_R_REG register
 * Current duty cycle for channel 0
 */

#define LEDC_CH0_DUTY_R_REG (DR_REG_LEDC_BASE + 0x10)

/* LEDC_DUTY_R_CH0 : RO; bitpos: [18:0]; default: 0;
 * This register stores the current duty of output signal on channel 0.
 */

#define LEDC_DUTY_R_CH0    0x0007ffff
#define LEDC_DUTY_R_CH0_M  (LEDC_DUTY_R_CH0_V << LEDC_DUTY_R_CH0_S)
#define LEDC_DUTY_R_CH0_V  0x0007ffff
#define LEDC_DUTY_R_CH0_S  0

/* LEDC_CH1_CONF0_REG register
 * Configuration register 0 for channel 1
 */

#define LEDC_CH1_CONF0_REG (DR_REG_LEDC_BASE + 0x14)

/* LEDC_OVF_CNT_RESET_ST_CH0 : R/WTC/SC; bitpos: [17]; default: 0;
 * This is the status bit of LEDC_OVF_CNT_RESET_CH1.
 */

#define LEDC_OVF_CNT_RESET_ST_CH0    (BIT(17))
#define LEDC_OVF_CNT_RESET_ST_CH0_M  (LEDC_OVF_CNT_RESET_ST_CH0_V << LEDC_OVF_CNT_RESET_ST_CH0_S)
#define LEDC_OVF_CNT_RESET_ST_CH0_V  0x00000001
#define LEDC_OVF_CNT_RESET_ST_CH0_S  17

/* LEDC_OVF_CNT_RESET_CH0 : WT; bitpos: [16]; default: 0;
 * Set this bit to reset the ovf_cnt of channel 1.
 */

#define LEDC_OVF_CNT_RESET_CH0    (BIT(16))
#define LEDC_OVF_CNT_RESET_CH0_M  (LEDC_OVF_CNT_RESET_CH0_V << LEDC_OVF_CNT_RESET_CH0_S)
#define LEDC_OVF_CNT_RESET_CH0_V  0x00000001
#define LEDC_OVF_CNT_RESET_CH0_S  16

/* LEDC_OVF_CNT_EN_CH0 : R/W; bitpos: [15]; default: 0;
 * This bit is used to enable the ovf_cnt of channel 1.
 */

#define LEDC_OVF_CNT_EN_CH0    (BIT(15))
#define LEDC_OVF_CNT_EN_CH0_M  (LEDC_OVF_CNT_EN_CH0_V << LEDC_OVF_CNT_EN_CH0_S)
#define LEDC_OVF_CNT_EN_CH0_V  0x00000001
#define LEDC_OVF_CNT_EN_CH0_S  15

/* LEDC_OVF_NUM_CH0 : R/W; bitpos: [14:5]; default: 0;
 * This register is used to configure the maximum times of overflow minus 1.
 *
 * The LEDC_OVF_CNT_CH1_INT interrupt will be triggered when channel 1
 * overflows for (LEDC_OVF_NUM_CH1 + 1) times.
 */

#define LEDC_OVF_NUM_CH0    0x000003ff
#define LEDC_OVF_NUM_CH0_M  (LEDC_OVF_NUM_CH0_V << LEDC_OVF_NUM_CH0_S)
#define LEDC_OVF_NUM_CH0_V  0x000003ff
#define LEDC_OVF_NUM_CH0_S  5

/* LEDC_PARA_UP_CH0 : WT; bitpos: [4]; default: 0;
 * This bit is used to update LEDC_HPOINT_CH1, LEDC_DUTY_START_CH1,
 * LEDC_SIG_OUT_EN_CH1, LEDC_TIMER_SEL_CH1, LEDC_DUTY_NUM_CH1,
 * LEDC_DUTY_CYCLE_CH1, LEDC_DUTY_SCALE_CH1, LEDC_DUTY_INC_CH1, and
 * LEDC_OVF_CNT_EN_CH1 fields for channel 1, and will be automatically
 * cleared by hardware.
 */

#define LEDC_PARA_UP_CH0    (BIT(4))
#define LEDC_PARA_UP_CH0_M  (LEDC_PARA_UP_CH0_V << LEDC_PARA_UP_CH0_S)
#define LEDC_PARA_UP_CH0_V  0x00000001
#define LEDC_PARA_UP_CH0_S  4

/* LEDC_IDLE_LV_CH0 : R/W; bitpos: [3]; default: 0;
 * This bit is used to control the output value when channel 1 is inactive
 * (when LEDC_SIG_OUT_EN_CH1 is 0).
 */

#define LEDC_IDLE_LV_CH0    (BIT(3))
#define LEDC_IDLE_LV_CH0_M  (LEDC_IDLE_LV_CH0_V << LEDC_IDLE_LV_CH0_S)
#define LEDC_IDLE_LV_CH0_V  0x00000001
#define LEDC_IDLE_LV_CH0_S  3

/* LEDC_SIG_OUT_EN_CH0 : R/W; bitpos: [2]; default: 0;
 * Set this bit to enable signal output on channel 1.
 */

#define LEDC_SIG_OUT_EN_CH0    (BIT(2))
#define LEDC_SIG_OUT_EN_CH0_M  (LEDC_SIG_OUT_EN_CH0_V << LEDC_SIG_OUT_EN_CH0_S)
#define LEDC_SIG_OUT_EN_CH0_V  0x00000001
#define LEDC_SIG_OUT_EN_CH0_S  2

/* LEDC_TIMER_SEL_CH0 : R/W; bitpos: [1:0]; default: 0;
 * This field is used to select one of timers for channel 1.
 *
 * 0: select timer0
 *
 * 1: select timer1
 *
 * 2: select timer2
 *
 * 3: select timer3
 */

#define LEDC_TIMER_SEL_CH0    0x00000003
#define LEDC_TIMER_SEL_CH0_M  (LEDC_TIMER_SEL_CH0_V << LEDC_TIMER_SEL_CH0_S)
#define LEDC_TIMER_SEL_CH0_V  0x00000003
#define LEDC_TIMER_SEL_CH0_S  0

/* LEDC_CH1_HPOINT_REG register
 * High point register for channel 1
 */

#define LEDC_CH1_HPOINT_REG (DR_REG_LEDC_BASE + 0x18)

/* LEDC_HPOINT_CH0 : R/W; bitpos: [13:0]; default: 0;
 * The output value changes to high when the selected timers has reached the
 * value specified by this register.
 */

#define LEDC_HPOINT_CH0    0x00003fff
#define LEDC_HPOINT_CH0_M  (LEDC_HPOINT_CH0_V << LEDC_HPOINT_CH0_S)
#define LEDC_HPOINT_CH0_V  0x00003fff
#define LEDC_HPOINT_CH0_S  0

/* LEDC_CH1_DUTY_REG register
 * Initial duty cycle for channel 1
 */

#define LEDC_CH1_DUTY_REG (DR_REG_LEDC_BASE + 0x1c)

/* LEDC_DUTY_CH0 : R/W; bitpos: [18:0]; default: 0;
 * This register is used to change the output duty by controlling the Lpoint.
 *
 * The output value turns to low when the selected timers has reached the
 * Lpoint.
 */

#define LEDC_DUTY_CH0    0x0007ffff
#define LEDC_DUTY_CH0_M  (LEDC_DUTY_CH0_V << LEDC_DUTY_CH0_S)
#define LEDC_DUTY_CH0_V  0x0007ffff
#define LEDC_DUTY_CH0_S  0

/* LEDC_CH1_CONF1_REG register
 * Configuration register 1 for channel 1
 */

#define LEDC_CH1_CONF1_REG (DR_REG_LEDC_BASE + 0x20)

/* LEDC_DUTY_START_CH0 : R/W/SC; bitpos: [31]; default: 0;
 * Other configured fields in LEDC_CH1_CONF1_REG will start to take effect
 * when this bit is set to 1.
 */

#define LEDC_DUTY_START_CH0    (BIT(31))
#define LEDC_DUTY_START_CH0_M  (LEDC_DUTY_START_CH0_V << LEDC_DUTY_START_CH0_S)
#define LEDC_DUTY_START_CH0_V  0x00000001
#define LEDC_DUTY_START_CH0_S  31

/* LEDC_DUTY_INC_CH0 : R/W; bitpos: [30]; default: 1;
 * This register is used to increase or decrease the duty of output signal
 * on channel 1. 1: Increase; 0: Decrease.
 */

#define LEDC_DUTY_INC_CH0    (BIT(30))
#define LEDC_DUTY_INC_CH0_M  (LEDC_DUTY_INC_CH0_V << LEDC_DUTY_INC_CH0_S)
#define LEDC_DUTY_INC_CH0_V  0x00000001
#define LEDC_DUTY_INC_CH0_S  30

/* LEDC_DUTY_NUM_CH0 : R/W; bitpos: [29:20]; default: 0;
 * This register is used to control the number of times the duty cycle will
 * be changed.
 */

#define LEDC_DUTY_NUM_CH0    0x000003ff
#define LEDC_DUTY_NUM_CH0_M  (LEDC_DUTY_NUM_CH0_V << LEDC_DUTY_NUM_CH0_S)
#define LEDC_DUTY_NUM_CH0_V  0x000003ff
#define LEDC_DUTY_NUM_CH0_S  20

/* LEDC_DUTY_CYCLE_CH0 : R/W; bitpos: [19:10]; default: 0;
 * The duty will change every LEDC_DUTY_CYCLE_CH1 on channel 1.
 */

#define LEDC_DUTY_CYCLE_CH0    0x000003ff
#define LEDC_DUTY_CYCLE_CH0_M  (LEDC_DUTY_CYCLE_CH0_V << LEDC_DUTY_CYCLE_CH0_S)
#define LEDC_DUTY_CYCLE_CH0_V  0x000003ff
#define LEDC_DUTY_CYCLE_CH0_S  10

/* LEDC_DUTY_SCALE_CH0 : R/W; bitpos: [9:0]; default: 0;
 * This register is used to configure the changing step scale of duty on
 * channel 1.
 */

#define LEDC_DUTY_SCALE_CH0    0x000003ff
#define LEDC_DUTY_SCALE_CH0_M  (LEDC_DUTY_SCALE_CH0_V << LEDC_DUTY_SCALE_CH0_S)
#define LEDC_DUTY_SCALE_CH0_V  0x000003ff
#define LEDC_DUTY_SCALE_CH0_S  0

/* LEDC_CH1_DUTY_R_REG register
 * Current duty cycle for channel 1
 */

#define LEDC_CH1_DUTY_R_REG (DR_REG_LEDC_BASE + 0x24)

/* LEDC_DUTY_R_CH0 : RO; bitpos: [18:0]; default: 0;
 * This register stores the current duty of output signal on channel 1.
 */

#define LEDC_DUTY_R_CH0    0x0007ffff
#define LEDC_DUTY_R_CH0_M  (LEDC_DUTY_R_CH0_V << LEDC_DUTY_R_CH0_S)
#define LEDC_DUTY_R_CH0_V  0x0007ffff
#define LEDC_DUTY_R_CH0_S  0

/* LEDC_CH2_CONF0_REG register
 * Configuration register 0 for channel 2
 */

#define LEDC_CH2_CONF0_REG (DR_REG_LEDC_BASE + 0x28)

/* LEDC_OVF_CNT_RESET_ST_CH0 : R/WTC/SC; bitpos: [17]; default: 0;
 * This is the status bit of LEDC_OVF_CNT_RESET_CH2.
 */

#define LEDC_OVF_CNT_RESET_ST_CH0    (BIT(17))
#define LEDC_OVF_CNT_RESET_ST_CH0_M  (LEDC_OVF_CNT_RESET_ST_CH0_V << LEDC_OVF_CNT_RESET_ST_CH0_S)
#define LEDC_OVF_CNT_RESET_ST_CH0_V  0x00000001
#define LEDC_OVF_CNT_RESET_ST_CH0_S  17

/* LEDC_OVF_CNT_RESET_CH0 : WT; bitpos: [16]; default: 0;
 * Set this bit to reset the ovf_cnt of channel 2.
 */

#define LEDC_OVF_CNT_RESET_CH0    (BIT(16))
#define LEDC_OVF_CNT_RESET_CH0_M  (LEDC_OVF_CNT_RESET_CH0_V << LEDC_OVF_CNT_RESET_CH0_S)
#define LEDC_OVF_CNT_RESET_CH0_V  0x00000001
#define LEDC_OVF_CNT_RESET_CH0_S  16

/* LEDC_OVF_CNT_EN_CH0 : R/W; bitpos: [15]; default: 0;
 * This bit is used to enable the ovf_cnt of channel 2.
 */

#define LEDC_OVF_CNT_EN_CH0    (BIT(15))
#define LEDC_OVF_CNT_EN_CH0_M  (LEDC_OVF_CNT_EN_CH0_V << LEDC_OVF_CNT_EN_CH0_S)
#define LEDC_OVF_CNT_EN_CH0_V  0x00000001
#define LEDC_OVF_CNT_EN_CH0_S  15

/* LEDC_OVF_NUM_CH0 : R/W; bitpos: [14:5]; default: 0;
 * This register is used to configure the maximum times of overflow minus 1.
 *
 * The LEDC_OVF_CNT_CH2_INT interrupt will be triggered when channel 2
 * overflows for (LEDC_OVF_NUM_CH2 + 1) times.
 */

#define LEDC_OVF_NUM_CH0    0x000003ff
#define LEDC_OVF_NUM_CH0_M  (LEDC_OVF_NUM_CH0_V << LEDC_OVF_NUM_CH0_S)
#define LEDC_OVF_NUM_CH0_V  0x000003ff
#define LEDC_OVF_NUM_CH0_S  5

/* LEDC_PARA_UP_CH0 : WT; bitpos: [4]; default: 0;
 * This bit is used to update LEDC_HPOINT_CH2, LEDC_DUTY_START_CH2,
 * LEDC_SIG_OUT_EN_CH2, LEDC_TIMER_SEL_CH2, LEDC_DUTY_NUM_CH2,
 * LEDC_DUTY_CYCLE_CH2, LEDC_DUTY_SCALE_CH2, LEDC_DUTY_INC_CH2, and
 * LEDC_OVF_CNT_EN_CH2 fields for channel 2, and will be automatically
 * cleared by hardware.
 */

#define LEDC_PARA_UP_CH0    (BIT(4))
#define LEDC_PARA_UP_CH0_M  (LEDC_PARA_UP_CH0_V << LEDC_PARA_UP_CH0_S)
#define LEDC_PARA_UP_CH0_V  0x00000001
#define LEDC_PARA_UP_CH0_S  4

/* LEDC_IDLE_LV_CH0 : R/W; bitpos: [3]; default: 0;
 * This bit is used to control the output value when channel 2 is inactive
 * (when LEDC_SIG_OUT_EN_CH2 is 0).
 */

#define LEDC_IDLE_LV_CH0    (BIT(3))
#define LEDC_IDLE_LV_CH0_M  (LEDC_IDLE_LV_CH0_V << LEDC_IDLE_LV_CH0_S)
#define LEDC_IDLE_LV_CH0_V  0x00000001
#define LEDC_IDLE_LV_CH0_S  3

/* LEDC_SIG_OUT_EN_CH0 : R/W; bitpos: [2]; default: 0;
 * Set this bit to enable signal output on channel 2.
 */

#define LEDC_SIG_OUT_EN_CH0    (BIT(2))
#define LEDC_SIG_OUT_EN_CH0_M  (LEDC_SIG_OUT_EN_CH0_V << LEDC_SIG_OUT_EN_CH0_S)
#define LEDC_SIG_OUT_EN_CH0_V  0x00000001
#define LEDC_SIG_OUT_EN_CH0_S  2

/* LEDC_TIMER_SEL_CH0 : R/W; bitpos: [1:0]; default: 0;
 * This field is used to select one of timers for channel 2.
 *
 * 0: select timer0
 *
 * 1: select timer1
 *
 * 2: select timer2
 *
 * 3: select timer3
 */

#define LEDC_TIMER_SEL_CH0    0x00000003
#define LEDC_TIMER_SEL_CH0_M  (LEDC_TIMER_SEL_CH0_V << LEDC_TIMER_SEL_CH0_S)
#define LEDC_TIMER_SEL_CH0_V  0x00000003
#define LEDC_TIMER_SEL_CH0_S  0

/* LEDC_CH2_HPOINT_REG register
 * High point register for channel 2
 */

#define LEDC_CH2_HPOINT_REG (DR_REG_LEDC_BASE + 0x2c)

/* LEDC_HPOINT_CH0 : R/W; bitpos: [13:0]; default: 0;
 * The output value changes to high when the selected timers has reached the
 * value specified by this register.
 */

#define LEDC_HPOINT_CH0    0x00003fff
#define LEDC_HPOINT_CH0_M  (LEDC_HPOINT_CH0_V << LEDC_HPOINT_CH0_S)
#define LEDC_HPOINT_CH0_V  0x00003fff
#define LEDC_HPOINT_CH0_S  0

/* LEDC_CH2_DUTY_REG register
 * Initial duty cycle for channel 2
 */

#define LEDC_CH2_DUTY_REG (DR_REG_LEDC_BASE + 0x30)

/* LEDC_DUTY_CH0 : R/W; bitpos: [18:0]; default: 0;
 * This register is used to change the output duty by controlling the Lpoint.
 *
 * The output value turns to low when the selected timers has reached the
 * Lpoint.
 */

#define LEDC_DUTY_CH0    0x0007ffff
#define LEDC_DUTY_CH0_M  (LEDC_DUTY_CH0_V << LEDC_DUTY_CH0_S)
#define LEDC_DUTY_CH0_V  0x0007ffff
#define LEDC_DUTY_CH0_S  0

/* LEDC_CH2_CONF1_REG register
 * Configuration register 1 for channel 2
 */

#define LEDC_CH2_CONF1_REG (DR_REG_LEDC_BASE + 0x34)

/* LEDC_DUTY_START_CH0 : R/W/SC; bitpos: [31]; default: 0;
 * Other configured fields in LEDC_CH2_CONF1_REG will start to take effect
 * when this bit is set to 1.
 */

#define LEDC_DUTY_START_CH0    (BIT(31))
#define LEDC_DUTY_START_CH0_M  (LEDC_DUTY_START_CH0_V << LEDC_DUTY_START_CH0_S)
#define LEDC_DUTY_START_CH0_V  0x00000001
#define LEDC_DUTY_START_CH0_S  31

/* LEDC_DUTY_INC_CH0 : R/W; bitpos: [30]; default: 1;
 * This register is used to increase or decrease the duty of output signal
 * on channel 2. 1: Increase; 0: Decrease.
 */

#define LEDC_DUTY_INC_CH0    (BIT(30))
#define LEDC_DUTY_INC_CH0_M  (LEDC_DUTY_INC_CH0_V << LEDC_DUTY_INC_CH0_S)
#define LEDC_DUTY_INC_CH0_V  0x00000001
#define LEDC_DUTY_INC_CH0_S  30

/* LEDC_DUTY_NUM_CH0 : R/W; bitpos: [29:20]; default: 0;
 * This register is used to control the number of times the duty cycle will
 * be changed.
 */

#define LEDC_DUTY_NUM_CH0    0x000003ff
#define LEDC_DUTY_NUM_CH0_M  (LEDC_DUTY_NUM_CH0_V << LEDC_DUTY_NUM_CH0_S)
#define LEDC_DUTY_NUM_CH0_V  0x000003ff
#define LEDC_DUTY_NUM_CH0_S  20

/* LEDC_DUTY_CYCLE_CH0 : R/W; bitpos: [19:10]; default: 0;
 * The duty will change every LEDC_DUTY_CYCLE_CH2 on channel 2.
 */

#define LEDC_DUTY_CYCLE_CH0    0x000003ff
#define LEDC_DUTY_CYCLE_CH0_M  (LEDC_DUTY_CYCLE_CH0_V << LEDC_DUTY_CYCLE_CH0_S)
#define LEDC_DUTY_CYCLE_CH0_V  0x000003ff
#define LEDC_DUTY_CYCLE_CH0_S  10

/* LEDC_DUTY_SCALE_CH0 : R/W; bitpos: [9:0]; default: 0;
 * This register is used to configure the changing step scale of duty on
 * channel 2.
 */

#define LEDC_DUTY_SCALE_CH0    0x000003ff
#define LEDC_DUTY_SCALE_CH0_M  (LEDC_DUTY_SCALE_CH0_V << LEDC_DUTY_SCALE_CH0_S)
#define LEDC_DUTY_SCALE_CH0_V  0x000003ff
#define LEDC_DUTY_SCALE_CH0_S  0

/* LEDC_CH2_DUTY_R_REG register
 * Current duty cycle for channel 2
 */

#define LEDC_CH2_DUTY_R_REG (DR_REG_LEDC_BASE + 0x38)

/* LEDC_DUTY_R_CH0 : RO; bitpos: [18:0]; default: 0;
 * This register stores the current duty of output signal on channel 2.
 */

#define LEDC_DUTY_R_CH0    0x0007ffff
#define LEDC_DUTY_R_CH0_M  (LEDC_DUTY_R_CH0_V << LEDC_DUTY_R_CH0_S)
#define LEDC_DUTY_R_CH0_V  0x0007ffff
#define LEDC_DUTY_R_CH0_S  0

/* LEDC_CH3_CONF0_REG register
 * Configuration register 0 for channel 3
 */

#define LEDC_CH3_CONF0_REG (DR_REG_LEDC_BASE + 0x3c)

/* LEDC_OVF_CNT_RESET_ST_CH0 : R/WTC/SC; bitpos: [17]; default: 0;
 * This is the status bit of LEDC_OVF_CNT_RESET_CH3.
 */

#define LEDC_OVF_CNT_RESET_ST_CH0    (BIT(17))
#define LEDC_OVF_CNT_RESET_ST_CH0_M  (LEDC_OVF_CNT_RESET_ST_CH0_V << LEDC_OVF_CNT_RESET_ST_CH0_S)
#define LEDC_OVF_CNT_RESET_ST_CH0_V  0x00000001
#define LEDC_OVF_CNT_RESET_ST_CH0_S  17

/* LEDC_OVF_CNT_RESET_CH0 : WT; bitpos: [16]; default: 0;
 * Set this bit to reset the ovf_cnt of channel 3.
 */

#define LEDC_OVF_CNT_RESET_CH0    (BIT(16))
#define LEDC_OVF_CNT_RESET_CH0_M  (LEDC_OVF_CNT_RESET_CH0_V << LEDC_OVF_CNT_RESET_CH0_S)
#define LEDC_OVF_CNT_RESET_CH0_V  0x00000001
#define LEDC_OVF_CNT_RESET_CH0_S  16

/* LEDC_OVF_CNT_EN_CH0 : R/W; bitpos: [15]; default: 0;
 * This bit is used to enable the ovf_cnt of channel 3.
 */

#define LEDC_OVF_CNT_EN_CH0    (BIT(15))
#define LEDC_OVF_CNT_EN_CH0_M  (LEDC_OVF_CNT_EN_CH0_V << LEDC_OVF_CNT_EN_CH0_S)
#define LEDC_OVF_CNT_EN_CH0_V  0x00000001
#define LEDC_OVF_CNT_EN_CH0_S  15

/* LEDC_OVF_NUM_CH0 : R/W; bitpos: [14:5]; default: 0;
 * This register is used to configure the maximum times of overflow minus 1.
 *
 * The LEDC_OVF_CNT_CH3_INT interrupt will be triggered when channel 3
 * overflows for (LEDC_OVF_NUM_CH3 + 1) times.
 */

#define LEDC_OVF_NUM_CH0    0x000003ff
#define LEDC_OVF_NUM_CH0_M  (LEDC_OVF_NUM_CH0_V << LEDC_OVF_NUM_CH0_S)
#define LEDC_OVF_NUM_CH0_V  0x000003ff
#define LEDC_OVF_NUM_CH0_S  5

/* LEDC_PARA_UP_CH0 : WT; bitpos: [4]; default: 0;
 * This bit is used to update LEDC_HPOINT_CH3, LEDC_DUTY_START_CH3,
 * LEDC_SIG_OUT_EN_CH3, LEDC_TIMER_SEL_CH3, LEDC_DUTY_NUM_CH3,
 * LEDC_DUTY_CYCLE_CH3, LEDC_DUTY_SCALE_CH3, LEDC_DUTY_INC_CH3, and
 * LEDC_OVF_CNT_EN_CH3 fields for channel 3, and will be automatically
 * cleared by hardware.
 */

#define LEDC_PARA_UP_CH0    (BIT(4))
#define LEDC_PARA_UP_CH0_M  (LEDC_PARA_UP_CH0_V << LEDC_PARA_UP_CH0_S)
#define LEDC_PARA_UP_CH0_V  0x00000001
#define LEDC_PARA_UP_CH0_S  4

/* LEDC_IDLE_LV_CH0 : R/W; bitpos: [3]; default: 0;
 * This bit is used to control the output value when channel 3 is inactive
 * (when LEDC_SIG_OUT_EN_CH3 is 0).
 */

#define LEDC_IDLE_LV_CH0    (BIT(3))
#define LEDC_IDLE_LV_CH0_M  (LEDC_IDLE_LV_CH0_V << LEDC_IDLE_LV_CH0_S)
#define LEDC_IDLE_LV_CH0_V  0x00000001
#define LEDC_IDLE_LV_CH0_S  3

/* LEDC_SIG_OUT_EN_CH0 : R/W; bitpos: [2]; default: 0;
 * Set this bit to enable signal output on channel 3.
 */

#define LEDC_SIG_OUT_EN_CH0    (BIT(2))
#define LEDC_SIG_OUT_EN_CH0_M  (LEDC_SIG_OUT_EN_CH0_V << LEDC_SIG_OUT_EN_CH0_S)
#define LEDC_SIG_OUT_EN_CH0_V  0x00000001
#define LEDC_SIG_OUT_EN_CH0_S  2

/* LEDC_TIMER_SEL_CH0 : R/W; bitpos: [1:0]; default: 0;
 * This field is used to select one of timers for channel 3.
 *
 * 0: select timer0
 *
 * 1: select timer1
 *
 * 2: select timer2
 *
 * 3: select timer3
 */

#define LEDC_TIMER_SEL_CH0    0x00000003
#define LEDC_TIMER_SEL_CH0_M  (LEDC_TIMER_SEL_CH0_V << LEDC_TIMER_SEL_CH0_S)
#define LEDC_TIMER_SEL_CH0_V  0x00000003
#define LEDC_TIMER_SEL_CH0_S  0

/* LEDC_CH3_HPOINT_REG register
 * High point register for channel 3
 */

#define LEDC_CH3_HPOINT_REG (DR_REG_LEDC_BASE + 0x40)

/* LEDC_HPOINT_CH0 : R/W; bitpos: [13:0]; default: 0;
 * The output value changes to high when the selected timers has reached the
 * value specified by this register.
 */

#define LEDC_HPOINT_CH0    0x00003fff
#define LEDC_HPOINT_CH0_M  (LEDC_HPOINT_CH0_V << LEDC_HPOINT_CH0_S)
#define LEDC_HPOINT_CH0_V  0x00003fff
#define LEDC_HPOINT_CH0_S  0

/* LEDC_CH3_DUTY_REG register
 * Initial duty cycle for channel 3
 */

#define LEDC_CH3_DUTY_REG (DR_REG_LEDC_BASE + 0x44)

/* LEDC_DUTY_CH0 : R/W; bitpos: [18:0]; default: 0;
 * This register is used to change the output duty by controlling the Lpoint.
 *
 * The output value turns to low when the selected timers has reached the
 * Lpoint.
 */

#define LEDC_DUTY_CH0    0x0007ffff
#define LEDC_DUTY_CH0_M  (LEDC_DUTY_CH0_V << LEDC_DUTY_CH0_S)
#define LEDC_DUTY_CH0_V  0x0007ffff
#define LEDC_DUTY_CH0_S  0

/* LEDC_CH3_CONF1_REG register
 * Configuration register 1 for channel 3
 */

#define LEDC_CH3_CONF1_REG (DR_REG_LEDC_BASE + 0x48)

/* LEDC_DUTY_START_CH0 : R/W/SC; bitpos: [31]; default: 0;
 * Other configured fields in LEDC_CH3_CONF1_REG will start to take effect
 * when this bit is set to 1.
 */

#define LEDC_DUTY_START_CH0    (BIT(31))
#define LEDC_DUTY_START_CH0_M  (LEDC_DUTY_START_CH0_V << LEDC_DUTY_START_CH0_S)
#define LEDC_DUTY_START_CH0_V  0x00000001
#define LEDC_DUTY_START_CH0_S  31

/* LEDC_DUTY_INC_CH0 : R/W; bitpos: [30]; default: 1;
 * This register is used to increase or decrease the duty of output signal
 * on channel 3. 1: Increase; 0: Decrease.
 */

#define LEDC_DUTY_INC_CH0    (BIT(30))
#define LEDC_DUTY_INC_CH0_M  (LEDC_DUTY_INC_CH0_V << LEDC_DUTY_INC_CH0_S)
#define LEDC_DUTY_INC_CH0_V  0x00000001
#define LEDC_DUTY_INC_CH0_S  30

/* LEDC_DUTY_NUM_CH0 : R/W; bitpos: [29:20]; default: 0;
 * This register is used to control the number of times the duty cycle will
 * be changed.
 */

#define LEDC_DUTY_NUM_CH0    0x000003ff
#define LEDC_DUTY_NUM_CH0_M  (LEDC_DUTY_NUM_CH0_V << LEDC_DUTY_NUM_CH0_S)
#define LEDC_DUTY_NUM_CH0_V  0x000003ff
#define LEDC_DUTY_NUM_CH0_S  20

/* LEDC_DUTY_CYCLE_CH0 : R/W; bitpos: [19:10]; default: 0;
 * The duty will change every LEDC_DUTY_CYCLE_CH3 on channel 3.
 */

#define LEDC_DUTY_CYCLE_CH0    0x000003ff
#define LEDC_DUTY_CYCLE_CH0_M  (LEDC_DUTY_CYCLE_CH0_V << LEDC_DUTY_CYCLE_CH0_S)
#define LEDC_DUTY_CYCLE_CH0_V  0x000003ff
#define LEDC_DUTY_CYCLE_CH0_S  10

/* LEDC_DUTY_SCALE_CH0 : R/W; bitpos: [9:0]; default: 0;
 * This register is used to configure the changing step scale of duty on
 * channel 3.
 */

#define LEDC_DUTY_SCALE_CH0    0x000003ff
#define LEDC_DUTY_SCALE_CH0_M  (LEDC_DUTY_SCALE_CH0_V << LEDC_DUTY_SCALE_CH0_S)
#define LEDC_DUTY_SCALE_CH0_V  0x000003ff
#define LEDC_DUTY_SCALE_CH0_S  0

/* LEDC_CH3_DUTY_R_REG register
 * Current duty cycle for channel 3
 */

#define LEDC_CH3_DUTY_R_REG (DR_REG_LEDC_BASE + 0x4c)

/* LEDC_DUTY_R_CH0 : RO; bitpos: [18:0]; default: 0;
 * This register stores the current duty of output signal on channel 3.
 */

#define LEDC_DUTY_R_CH0    0x0007ffff
#define LEDC_DUTY_R_CH0_M  (LEDC_DUTY_R_CH0_V << LEDC_DUTY_R_CH0_S)
#define LEDC_DUTY_R_CH0_V  0x0007ffff
#define LEDC_DUTY_R_CH0_S  0

/* LEDC_CH4_CONF0_REG register
 * Configuration register 0 for channel 4
 */

#define LEDC_CH4_CONF0_REG (DR_REG_LEDC_BASE + 0x50)

/* LEDC_OVF_CNT_RESET_ST_CH0 : R/WTC/SC; bitpos: [17]; default: 0;
 * This is the status bit of LEDC_OVF_CNT_RESET_CH4.
 */

#define LEDC_OVF_CNT_RESET_ST_CH0    (BIT(17))
#define LEDC_OVF_CNT_RESET_ST_CH0_M  (LEDC_OVF_CNT_RESET_ST_CH0_V << LEDC_OVF_CNT_RESET_ST_CH0_S)
#define LEDC_OVF_CNT_RESET_ST_CH0_V  0x00000001
#define LEDC_OVF_CNT_RESET_ST_CH0_S  17

/* LEDC_OVF_CNT_RESET_CH0 : WT; bitpos: [16]; default: 0;
 * Set this bit to reset the ovf_cnt of channel 4.
 */

#define LEDC_OVF_CNT_RESET_CH0    (BIT(16))
#define LEDC_OVF_CNT_RESET_CH0_M  (LEDC_OVF_CNT_RESET_CH0_V << LEDC_OVF_CNT_RESET_CH0_S)
#define LEDC_OVF_CNT_RESET_CH0_V  0x00000001
#define LEDC_OVF_CNT_RESET_CH0_S  16

/* LEDC_OVF_CNT_EN_CH0 : R/W; bitpos: [15]; default: 0;
 * This bit is used to enable the ovf_cnt of channel 4.
 */

#define LEDC_OVF_CNT_EN_CH0    (BIT(15))
#define LEDC_OVF_CNT_EN_CH0_M  (LEDC_OVF_CNT_EN_CH0_V << LEDC_OVF_CNT_EN_CH0_S)
#define LEDC_OVF_CNT_EN_CH0_V  0x00000001
#define LEDC_OVF_CNT_EN_CH0_S  15

/* LEDC_OVF_NUM_CH0 : R/W; bitpos: [14:5]; default: 0;
 * This register is used to configure the maximum times of overflow minus 1.
 *
 * The LEDC_OVF_CNT_CH4_INT interrupt will be triggered when channel 4
 * overflows for (LEDC_OVF_NUM_CH4 + 1) times.
 */

#define LEDC_OVF_NUM_CH0    0x000003ff
#define LEDC_OVF_NUM_CH0_M  (LEDC_OVF_NUM_CH0_V << LEDC_OVF_NUM_CH0_S)
#define LEDC_OVF_NUM_CH0_V  0x000003ff
#define LEDC_OVF_NUM_CH0_S  5

/* LEDC_PARA_UP_CH0 : WT; bitpos: [4]; default: 0;
 * This bit is used to update LEDC_HPOINT_CH4, LEDC_DUTY_START_CH4,
 * LEDC_SIG_OUT_EN_CH4, LEDC_TIMER_SEL_CH4, LEDC_DUTY_NUM_CH4,
 * LEDC_DUTY_CYCLE_CH4, LEDC_DUTY_SCALE_CH4, LEDC_DUTY_INC_CH4, and
 * LEDC_OVF_CNT_EN_CH4 fields for channel 4, and will be automatically
 * cleared by hardware.
 */

#define LEDC_PARA_UP_CH0    (BIT(4))
#define LEDC_PARA_UP_CH0_M  (LEDC_PARA_UP_CH0_V << LEDC_PARA_UP_CH0_S)
#define LEDC_PARA_UP_CH0_V  0x00000001
#define LEDC_PARA_UP_CH0_S  4

/* LEDC_IDLE_LV_CH0 : R/W; bitpos: [3]; default: 0;
 * This bit is used to control the output value when channel 4 is inactive
 * (when LEDC_SIG_OUT_EN_CH4 is 0).
 */

#define LEDC_IDLE_LV_CH0    (BIT(3))
#define LEDC_IDLE_LV_CH0_M  (LEDC_IDLE_LV_CH0_V << LEDC_IDLE_LV_CH0_S)
#define LEDC_IDLE_LV_CH0_V  0x00000001
#define LEDC_IDLE_LV_CH0_S  3

/* LEDC_SIG_OUT_EN_CH0 : R/W; bitpos: [2]; default: 0;
 * Set this bit to enable signal output on channel 4.
 */

#define LEDC_SIG_OUT_EN_CH0    (BIT(2))
#define LEDC_SIG_OUT_EN_CH0_M  (LEDC_SIG_OUT_EN_CH0_V << LEDC_SIG_OUT_EN_CH0_S)
#define LEDC_SIG_OUT_EN_CH0_V  0x00000001
#define LEDC_SIG_OUT_EN_CH0_S  2

/* LEDC_TIMER_SEL_CH0 : R/W; bitpos: [1:0]; default: 0;
 * This field is used to select one of timers for channel 4.
 *
 * 0: select timer0
 *
 * 1: select timer1
 *
 * 2: select timer2
 *
 * 3: select timer3
 */

#define LEDC_TIMER_SEL_CH0    0x00000003
#define LEDC_TIMER_SEL_CH0_M  (LEDC_TIMER_SEL_CH0_V << LEDC_TIMER_SEL_CH0_S)
#define LEDC_TIMER_SEL_CH0_V  0x00000003
#define LEDC_TIMER_SEL_CH0_S  0

/* LEDC_CH4_HPOINT_REG register
 * High point register for channel 4
 */

#define LEDC_CH4_HPOINT_REG (DR_REG_LEDC_BASE + 0x54)

/* LEDC_HPOINT_CH0 : R/W; bitpos: [13:0]; default: 0;
 * The output value changes to high when the selected timers has reached the
 * value specified by this register.
 */

#define LEDC_HPOINT_CH0    0x00003fff
#define LEDC_HPOINT_CH0_M  (LEDC_HPOINT_CH0_V << LEDC_HPOINT_CH0_S)
#define LEDC_HPOINT_CH0_V  0x00003fff
#define LEDC_HPOINT_CH0_S  0

/* LEDC_CH4_DUTY_REG register
 * Initial duty cycle for channel 4
 */

#define LEDC_CH4_DUTY_REG (DR_REG_LEDC_BASE + 0x58)

/* LEDC_DUTY_CH0 : R/W; bitpos: [18:0]; default: 0;
 * This register is used to change the output duty by controlling the Lpoint.
 *
 * The output value turns to low when the selected timers has reached the
 * Lpoint.
 */

#define LEDC_DUTY_CH0    0x0007ffff
#define LEDC_DUTY_CH0_M  (LEDC_DUTY_CH0_V << LEDC_DUTY_CH0_S)
#define LEDC_DUTY_CH0_V  0x0007ffff
#define LEDC_DUTY_CH0_S  0

/* LEDC_CH4_CONF1_REG register
 * Configuration register 1 for channel 4
 */

#define LEDC_CH4_CONF1_REG (DR_REG_LEDC_BASE + 0x5c)

/* LEDC_DUTY_START_CH0 : R/W/SC; bitpos: [31]; default: 0;
 * Other configured fields in LEDC_CH4_CONF1_REG will start to take effect
 * when this bit is set to 1.
 */

#define LEDC_DUTY_START_CH0    (BIT(31))
#define LEDC_DUTY_START_CH0_M  (LEDC_DUTY_START_CH0_V << LEDC_DUTY_START_CH0_S)
#define LEDC_DUTY_START_CH0_V  0x00000001
#define LEDC_DUTY_START_CH0_S  31

/* LEDC_DUTY_INC_CH0 : R/W; bitpos: [30]; default: 1;
 * This register is used to increase or decrease the duty of output signal
 * on channel 4. 1: Increase; 0: Decrease.
 */

#define LEDC_DUTY_INC_CH0    (BIT(30))
#define LEDC_DUTY_INC_CH0_M  (LEDC_DUTY_INC_CH0_V << LEDC_DUTY_INC_CH0_S)
#define LEDC_DUTY_INC_CH0_V  0x00000001
#define LEDC_DUTY_INC_CH0_S  30

/* LEDC_DUTY_NUM_CH0 : R/W; bitpos: [29:20]; default: 0;
 * This register is used to control the number of times the duty cycle will
 * be changed.
 */

#define LEDC_DUTY_NUM_CH0    0x000003ff
#define LEDC_DUTY_NUM_CH0_M  (LEDC_DUTY_NUM_CH0_V << LEDC_DUTY_NUM_CH0_S)
#define LEDC_DUTY_NUM_CH0_V  0x000003ff
#define LEDC_DUTY_NUM_CH0_S  20

/* LEDC_DUTY_CYCLE_CH0 : R/W; bitpos: [19:10]; default: 0;
 * The duty will change every LEDC_DUTY_CYCLE_CH4 on channel 4.
 */

#define LEDC_DUTY_CYCLE_CH0    0x000003ff
#define LEDC_DUTY_CYCLE_CH0_M  (LEDC_DUTY_CYCLE_CH0_V << LEDC_DUTY_CYCLE_CH0_S)
#define LEDC_DUTY_CYCLE_CH0_V  0x000003ff
#define LEDC_DUTY_CYCLE_CH0_S  10

/* LEDC_DUTY_SCALE_CH0 : R/W; bitpos: [9:0]; default: 0;
 * This register is used to configure the changing step scale of duty on
 * channel 4.
 */

#define LEDC_DUTY_SCALE_CH0    0x000003ff
#define LEDC_DUTY_SCALE_CH0_M  (LEDC_DUTY_SCALE_CH0_V << LEDC_DUTY_SCALE_CH0_S)
#define LEDC_DUTY_SCALE_CH0_V  0x000003ff
#define LEDC_DUTY_SCALE_CH0_S  0

/* LEDC_CH4_DUTY_R_REG register
 * Current duty cycle for channel 4
 */

#define LEDC_CH4_DUTY_R_REG (DR_REG_LEDC_BASE + 0x60)

/* LEDC_DUTY_R_CH0 : RO; bitpos: [18:0]; default: 0;
 * This register stores the current duty of output signal on channel 4.
 */

#define LEDC_DUTY_R_CH0    0x0007ffff
#define LEDC_DUTY_R_CH0_M  (LEDC_DUTY_R_CH0_V << LEDC_DUTY_R_CH0_S)
#define LEDC_DUTY_R_CH0_V  0x0007ffff
#define LEDC_DUTY_R_CH0_S  0

/* LEDC_CH5_CONF0_REG register
 * Configuration register 0 for channel 5
 */

#define LEDC_CH5_CONF0_REG (DR_REG_LEDC_BASE + 0x64)

/* LEDC_OVF_CNT_RESET_ST_CH0 : R/WTC/SC; bitpos: [17]; default: 0;
 * This is the status bit of LEDC_OVF_CNT_RESET_CH5.
 */

#define LEDC_OVF_CNT_RESET_ST_CH0    (BIT(17))
#define LEDC_OVF_CNT_RESET_ST_CH0_M  (LEDC_OVF_CNT_RESET_ST_CH0_V << LEDC_OVF_CNT_RESET_ST_CH0_S)
#define LEDC_OVF_CNT_RESET_ST_CH0_V  0x00000001
#define LEDC_OVF_CNT_RESET_ST_CH0_S  17

/* LEDC_OVF_CNT_RESET_CH0 : WT; bitpos: [16]; default: 0;
 * Set this bit to reset the ovf_cnt of channel 5.
 */

#define LEDC_OVF_CNT_RESET_CH0    (BIT(16))
#define LEDC_OVF_CNT_RESET_CH0_M  (LEDC_OVF_CNT_RESET_CH0_V << LEDC_OVF_CNT_RESET_CH0_S)
#define LEDC_OVF_CNT_RESET_CH0_V  0x00000001
#define LEDC_OVF_CNT_RESET_CH0_S  16

/* LEDC_OVF_CNT_EN_CH0 : R/W; bitpos: [15]; default: 0;
 * This bit is used to enable the ovf_cnt of channel 5.
 */

#define LEDC_OVF_CNT_EN_CH0    (BIT(15))
#define LEDC_OVF_CNT_EN_CH0_M  (LEDC_OVF_CNT_EN_CH0_V << LEDC_OVF_CNT_EN_CH0_S)
#define LEDC_OVF_CNT_EN_CH0_V  0x00000001
#define LEDC_OVF_CNT_EN_CH0_S  15

/* LEDC_OVF_NUM_CH0 : R/W; bitpos: [14:5]; default: 0;
 * This register is used to configure the maximum times of overflow minus 1.
 *
 * The LEDC_OVF_CNT_CH5_INT interrupt will be triggered when channel 5
 * overflows for (LEDC_OVF_NUM_CH5 + 1) times.
 */

#define LEDC_OVF_NUM_CH0    0x000003ff
#define LEDC_OVF_NUM_CH0_M  (LEDC_OVF_NUM_CH0_V << LEDC_OVF_NUM_CH0_S)
#define LEDC_OVF_NUM_CH0_V  0x000003ff
#define LEDC_OVF_NUM_CH0_S  5

/* LEDC_PARA_UP_CH0 : WT; bitpos: [4]; default: 0;
 * This bit is used to update LEDC_HPOINT_CH5, LEDC_DUTY_START_CH5,
 * LEDC_SIG_OUT_EN_CH5, LEDC_TIMER_SEL_CH5, LEDC_DUTY_NUM_CH5,
 * LEDC_DUTY_CYCLE_CH5, LEDC_DUTY_SCALE_CH5, LEDC_DUTY_INC_CH5, and
 * LEDC_OVF_CNT_EN_CH5 fields for channel 5, and will be automatically
 * cleared by hardware.
 */

#define LEDC_PARA_UP_CH0    (BIT(4))
#define LEDC_PARA_UP_CH0_M  (LEDC_PARA_UP_CH0_V << LEDC_PARA_UP_CH0_S)
#define LEDC_PARA_UP_CH0_V  0x00000001
#define LEDC_PARA_UP_CH0_S  4

/* LEDC_IDLE_LV_CH0 : R/W; bitpos: [3]; default: 0;
 * This bit is used to control the output value when channel 5 is inactive
 * (when LEDC_SIG_OUT_EN_CH5 is 0).
 */

#define LEDC_IDLE_LV_CH0    (BIT(3))
#define LEDC_IDLE_LV_CH0_M  (LEDC_IDLE_LV_CH0_V << LEDC_IDLE_LV_CH0_S)
#define LEDC_IDLE_LV_CH0_V  0x00000001
#define LEDC_IDLE_LV_CH0_S  3

/* LEDC_SIG_OUT_EN_CH0 : R/W; bitpos: [2]; default: 0;
 * Set this bit to enable signal output on channel 5.
 */

#define LEDC_SIG_OUT_EN_CH0    (BIT(2))
#define LEDC_SIG_OUT_EN_CH0_M  (LEDC_SIG_OUT_EN_CH0_V << LEDC_SIG_OUT_EN_CH0_S)
#define LEDC_SIG_OUT_EN_CH0_V  0x00000001
#define LEDC_SIG_OUT_EN_CH0_S  2

/* LEDC_TIMER_SEL_CH0 : R/W; bitpos: [1:0]; default: 0;
 * This field is used to select one of timers for channel 5.
 *
 * 0: select timer0
 *
 * 1: select timer1
 *
 * 2: select timer2
 *
 * 3: select timer3
 */

#define LEDC_TIMER_SEL_CH0    0x00000003
#define LEDC_TIMER_SEL_CH0_M  (LEDC_TIMER_SEL_CH0_V << LEDC_TIMER_SEL_CH0_S)
#define LEDC_TIMER_SEL_CH0_V  0x00000003
#define LEDC_TIMER_SEL_CH0_S  0

/* LEDC_CH5_HPOINT_REG register
 * High point register for channel 5
 */

#define LEDC_CH5_HPOINT_REG (DR_REG_LEDC_BASE + 0x68)

/* LEDC_HPOINT_CH0 : R/W; bitpos: [13:0]; default: 0;
 * The output value changes to high when the selected timers has reached the
 * value specified by this register.
 */

#define LEDC_HPOINT_CH0    0x00003fff
#define LEDC_HPOINT_CH0_M  (LEDC_HPOINT_CH0_V << LEDC_HPOINT_CH0_S)
#define LEDC_HPOINT_CH0_V  0x00003fff
#define LEDC_HPOINT_CH0_S  0

/* LEDC_CH5_DUTY_REG register
 * Initial duty cycle for channel 5
 */

#define LEDC_CH5_DUTY_REG (DR_REG_LEDC_BASE + 0x6c)

/* LEDC_DUTY_CH0 : R/W; bitpos: [18:0]; default: 0;
 * This register is used to change the output duty by controlling the Lpoint.
 *
 * The output value turns to low when the selected timers has reached the
 * Lpoint.
 */

#define LEDC_DUTY_CH0    0x0007ffff
#define LEDC_DUTY_CH0_M  (LEDC_DUTY_CH0_V << LEDC_DUTY_CH0_S)
#define LEDC_DUTY_CH0_V  0x0007ffff
#define LEDC_DUTY_CH0_S  0

/* LEDC_CH5_CONF1_REG register
 * Configuration register 1 for channel 5
 */

#define LEDC_CH5_CONF1_REG (DR_REG_LEDC_BASE + 0x70)

/* LEDC_DUTY_START_CH0 : R/W/SC; bitpos: [31]; default: 0;
 * Other configured fields in LEDC_CH5_CONF1_REG will start to take effect
 * when this bit is set to 1.
 */

#define LEDC_DUTY_START_CH0    (BIT(31))
#define LEDC_DUTY_START_CH0_M  (LEDC_DUTY_START_CH0_V << LEDC_DUTY_START_CH0_S)
#define LEDC_DUTY_START_CH0_V  0x00000001
#define LEDC_DUTY_START_CH0_S  31

/* LEDC_DUTY_INC_CH0 : R/W; bitpos: [30]; default: 1;
 * This register is used to increase or decrease the duty of output signal
 * on channel 5. 1: Increase; 0: Decrease.
 */

#define LEDC_DUTY_INC_CH0    (BIT(30))
#define LEDC_DUTY_INC_CH0_M  (LEDC_DUTY_INC_CH0_V << LEDC_DUTY_INC_CH0_S)
#define LEDC_DUTY_INC_CH0_V  0x00000001
#define LEDC_DUTY_INC_CH0_S  30

/* LEDC_DUTY_NUM_CH0 : R/W; bitpos: [29:20]; default: 0;
 * This register is used to control the number of times the duty cycle will
 * be changed.
 */

#define LEDC_DUTY_NUM_CH0    0x000003ff
#define LEDC_DUTY_NUM_CH0_M  (LEDC_DUTY_NUM_CH0_V << LEDC_DUTY_NUM_CH0_S)
#define LEDC_DUTY_NUM_CH0_V  0x000003ff
#define LEDC_DUTY_NUM_CH0_S  20

/* LEDC_DUTY_CYCLE_CH0 : R/W; bitpos: [19:10]; default: 0;
 * The duty will change every LEDC_DUTY_CYCLE_CH5 on channel 5.
 */

#define LEDC_DUTY_CYCLE_CH0    0x000003ff
#define LEDC_DUTY_CYCLE_CH0_M  (LEDC_DUTY_CYCLE_CH0_V << LEDC_DUTY_CYCLE_CH0_S)
#define LEDC_DUTY_CYCLE_CH0_V  0x000003ff
#define LEDC_DUTY_CYCLE_CH0_S  10

/* LEDC_DUTY_SCALE_CH0 : R/W; bitpos: [9:0]; default: 0;
 * This register is used to configure the changing step scale of duty on
 * channel 5.
 */

#define LEDC_DUTY_SCALE_CH0    0x000003ff
#define LEDC_DUTY_SCALE_CH0_M  (LEDC_DUTY_SCALE_CH0_V << LEDC_DUTY_SCALE_CH0_S)
#define LEDC_DUTY_SCALE_CH0_V  0x000003ff
#define LEDC_DUTY_SCALE_CH0_S  0

/* LEDC_CH5_DUTY_R_REG register
 * Current duty cycle for channel 5
 */

#define LEDC_CH5_DUTY_R_REG (DR_REG_LEDC_BASE + 0x74)

/* LEDC_DUTY_R_CH0 : RO; bitpos: [18:0]; default: 0;
 * This register stores the current duty of output signal on channel 5.
 */

#define LEDC_DUTY_R_CH0    0x0007ffff
#define LEDC_DUTY_R_CH0_M  (LEDC_DUTY_R_CH0_V << LEDC_DUTY_R_CH0_S)
#define LEDC_DUTY_R_CH0_V  0x0007ffff
#define LEDC_DUTY_R_CH0_S  0

/* LEDC_CH6_CONF0_REG register
 * Configuration register 0 for channel 6
 */

#define LEDC_CH6_CONF0_REG (DR_REG_LEDC_BASE + 0x78)

/* LEDC_OVF_CNT_RESET_ST_CH0 : R/WTC/SC; bitpos: [17]; default: 0;
 * This is the status bit of LEDC_OVF_CNT_RESET_CH6.
 */

#define LEDC_OVF_CNT_RESET_ST_CH0    (BIT(17))
#define LEDC_OVF_CNT_RESET_ST_CH0_M  (LEDC_OVF_CNT_RESET_ST_CH0_V << LEDC_OVF_CNT_RESET_ST_CH0_S)
#define LEDC_OVF_CNT_RESET_ST_CH0_V  0x00000001
#define LEDC_OVF_CNT_RESET_ST_CH0_S  17

/* LEDC_OVF_CNT_RESET_CH0 : WT; bitpos: [16]; default: 0;
 * Set this bit to reset the ovf_cnt of channel 6.
 */

#define LEDC_OVF_CNT_RESET_CH0    (BIT(16))
#define LEDC_OVF_CNT_RESET_CH0_M  (LEDC_OVF_CNT_RESET_CH0_V << LEDC_OVF_CNT_RESET_CH0_S)
#define LEDC_OVF_CNT_RESET_CH0_V  0x00000001
#define LEDC_OVF_CNT_RESET_CH0_S  16

/* LEDC_OVF_CNT_EN_CH0 : R/W; bitpos: [15]; default: 0;
 * This bit is used to enable the ovf_cnt of channel 6.
 */

#define LEDC_OVF_CNT_EN_CH0    (BIT(15))
#define LEDC_OVF_CNT_EN_CH0_M  (LEDC_OVF_CNT_EN_CH0_V << LEDC_OVF_CNT_EN_CH0_S)
#define LEDC_OVF_CNT_EN_CH0_V  0x00000001
#define LEDC_OVF_CNT_EN_CH0_S  15

/* LEDC_OVF_NUM_CH0 : R/W; bitpos: [14:5]; default: 0;
 * This register is used to configure the maximum times of overflow minus 1.
 *
 * The LEDC_OVF_CNT_CH6_INT interrupt will be triggered when channel 6
 * overflows for (LEDC_OVF_NUM_CH6 + 1) times.
 */

#define LEDC_OVF_NUM_CH0    0x000003ff
#define LEDC_OVF_NUM_CH0_M  (LEDC_OVF_NUM_CH0_V << LEDC_OVF_NUM_CH0_S)
#define LEDC_OVF_NUM_CH0_V  0x000003ff
#define LEDC_OVF_NUM_CH0_S  5

/* LEDC_PARA_UP_CH0 : WT; bitpos: [4]; default: 0;
 * This bit is used to update LEDC_HPOINT_CH6, LEDC_DUTY_START_CH6,
 * LEDC_SIG_OUT_EN_CH6, LEDC_TIMER_SEL_CH6, LEDC_DUTY_NUM_CH6,
 * LEDC_DUTY_CYCLE_CH6, LEDC_DUTY_SCALE_CH6, LEDC_DUTY_INC_CH6, and
 * LEDC_OVF_CNT_EN_CH6 fields for channel 6, and will be automatically
 * cleared by hardware.
 */

#define LEDC_PARA_UP_CH0    (BIT(4))
#define LEDC_PARA_UP_CH0_M  (LEDC_PARA_UP_CH0_V << LEDC_PARA_UP_CH0_S)
#define LEDC_PARA_UP_CH0_V  0x00000001
#define LEDC_PARA_UP_CH0_S  4

/* LEDC_IDLE_LV_CH0 : R/W; bitpos: [3]; default: 0;
 * This bit is used to control the output value when channel 6 is inactive
 * (when LEDC_SIG_OUT_EN_CH6 is 0).
 */

#define LEDC_IDLE_LV_CH0    (BIT(3))
#define LEDC_IDLE_LV_CH0_M  (LEDC_IDLE_LV_CH0_V << LEDC_IDLE_LV_CH0_S)
#define LEDC_IDLE_LV_CH0_V  0x00000001
#define LEDC_IDLE_LV_CH0_S  3

/* LEDC_SIG_OUT_EN_CH0 : R/W; bitpos: [2]; default: 0;
 * Set this bit to enable signal output on channel 6.
 */

#define LEDC_SIG_OUT_EN_CH0    (BIT(2))
#define LEDC_SIG_OUT_EN_CH0_M  (LEDC_SIG_OUT_EN_CH0_V << LEDC_SIG_OUT_EN_CH0_S)
#define LEDC_SIG_OUT_EN_CH0_V  0x00000001
#define LEDC_SIG_OUT_EN_CH0_S  2

/* LEDC_TIMER_SEL_CH0 : R/W; bitpos: [1:0]; default: 0;
 * This field is used to select one of timers for channel 6.
 *
 * 0: select timer0
 *
 * 1: select timer1
 *
 * 2: select timer2
 *
 * 3: select timer3
 */

#define LEDC_TIMER_SEL_CH0    0x00000003
#define LEDC_TIMER_SEL_CH0_M  (LEDC_TIMER_SEL_CH0_V << LEDC_TIMER_SEL_CH0_S)
#define LEDC_TIMER_SEL_CH0_V  0x00000003
#define LEDC_TIMER_SEL_CH0_S  0

/* LEDC_CH6_HPOINT_REG register
 * High point register for channel 6
 */

#define LEDC_CH6_HPOINT_REG (DR_REG_LEDC_BASE + 0x7c)

/* LEDC_HPOINT_CH0 : R/W; bitpos: [13:0]; default: 0;
 * The output value changes to high when the selected timers has reached the
 * value specified by this register.
 */

#define LEDC_HPOINT_CH0    0x00003fff
#define LEDC_HPOINT_CH0_M  (LEDC_HPOINT_CH0_V << LEDC_HPOINT_CH0_S)
#define LEDC_HPOINT_CH0_V  0x00003fff
#define LEDC_HPOINT_CH0_S  0

/* LEDC_CH6_DUTY_REG register
 * Initial duty cycle for channel 6
 */

#define LEDC_CH6_DUTY_REG (DR_REG_LEDC_BASE + 0x80)

/* LEDC_DUTY_CH0 : R/W; bitpos: [18:0]; default: 0;
 * This register is used to change the output duty by controlling the Lpoint.
 *
 * The output value turns to low when the selected timers has reached the
 * Lpoint.
 */

#define LEDC_DUTY_CH0    0x0007ffff
#define LEDC_DUTY_CH0_M  (LEDC_DUTY_CH0_V << LEDC_DUTY_CH0_S)
#define LEDC_DUTY_CH0_V  0x0007ffff
#define LEDC_DUTY_CH0_S  0

/* LEDC_CH6_CONF1_REG register
 * Configuration register 1 for channel 6
 */

#define LEDC_CH6_CONF1_REG (DR_REG_LEDC_BASE + 0x84)

/* LEDC_DUTY_START_CH0 : R/W/SC; bitpos: [31]; default: 0;
 * Other configured fields in LEDC_CH6_CONF1_REG will start to take effect
 * when this bit is set to 1.
 */

#define LEDC_DUTY_START_CH0    (BIT(31))
#define LEDC_DUTY_START_CH0_M  (LEDC_DUTY_START_CH0_V << LEDC_DUTY_START_CH0_S)
#define LEDC_DUTY_START_CH0_V  0x00000001
#define LEDC_DUTY_START_CH0_S  31

/* LEDC_DUTY_INC_CH0 : R/W; bitpos: [30]; default: 1;
 * This register is used to increase or decrease the duty of output signal
 * on channel 6. 1: Increase; 0: Decrease.
 */

#define LEDC_DUTY_INC_CH0    (BIT(30))
#define LEDC_DUTY_INC_CH0_M  (LEDC_DUTY_INC_CH0_V << LEDC_DUTY_INC_CH0_S)
#define LEDC_DUTY_INC_CH0_V  0x00000001
#define LEDC_DUTY_INC_CH0_S  30

/* LEDC_DUTY_NUM_CH0 : R/W; bitpos: [29:20]; default: 0;
 * This register is used to control the number of times the duty cycle will
 * be changed.
 */

#define LEDC_DUTY_NUM_CH0    0x000003ff
#define LEDC_DUTY_NUM_CH0_M  (LEDC_DUTY_NUM_CH0_V << LEDC_DUTY_NUM_CH0_S)
#define LEDC_DUTY_NUM_CH0_V  0x000003ff
#define LEDC_DUTY_NUM_CH0_S  20

/* LEDC_DUTY_CYCLE_CH0 : R/W; bitpos: [19:10]; default: 0;
 * The duty will change every LEDC_DUTY_CYCLE_CH6 on channel 6.
 */

#define LEDC_DUTY_CYCLE_CH0    0x000003ff
#define LEDC_DUTY_CYCLE_CH0_M  (LEDC_DUTY_CYCLE_CH0_V << LEDC_DUTY_CYCLE_CH0_S)
#define LEDC_DUTY_CYCLE_CH0_V  0x000003ff
#define LEDC_DUTY_CYCLE_CH0_S  10

/* LEDC_DUTY_SCALE_CH0 : R/W; bitpos: [9:0]; default: 0;
 * This register is used to configure the changing step scale of duty on
 * channel 6.
 */

#define LEDC_DUTY_SCALE_CH0    0x000003ff
#define LEDC_DUTY_SCALE_CH0_M  (LEDC_DUTY_SCALE_CH0_V << LEDC_DUTY_SCALE_CH0_S)
#define LEDC_DUTY_SCALE_CH0_V  0x000003ff
#define LEDC_DUTY_SCALE_CH0_S  0

/* LEDC_CH6_DUTY_R_REG register
 * Current duty cycle for channel 6
 */

#define LEDC_CH6_DUTY_R_REG (DR_REG_LEDC_BASE + 0x88)

/* LEDC_DUTY_R_CH0 : RO; bitpos: [18:0]; default: 0;
 * This register stores the current duty of output signal on channel 6.
 */

#define LEDC_DUTY_R_CH0    0x0007ffff
#define LEDC_DUTY_R_CH0_M  (LEDC_DUTY_R_CH0_V << LEDC_DUTY_R_CH0_S)
#define LEDC_DUTY_R_CH0_V  0x0007ffff
#define LEDC_DUTY_R_CH0_S  0

/* LEDC_CH7_CONF0_REG register
 * Configuration register 0 for channel 7
 */

#define LEDC_CH7_CONF0_REG (DR_REG_LEDC_BASE + 0x8c)

/* LEDC_OVF_CNT_RESET_ST_CH0 : R/WTC/SC; bitpos: [17]; default: 0;
 * This is the status bit of LEDC_OVF_CNT_RESET_CH7.
 */

#define LEDC_OVF_CNT_RESET_ST_CH0    (BIT(17))
#define LEDC_OVF_CNT_RESET_ST_CH0_M  (LEDC_OVF_CNT_RESET_ST_CH0_V << LEDC_OVF_CNT_RESET_ST_CH0_S)
#define LEDC_OVF_CNT_RESET_ST_CH0_V  0x00000001
#define LEDC_OVF_CNT_RESET_ST_CH0_S  17

/* LEDC_OVF_CNT_RESET_CH0 : WT; bitpos: [16]; default: 0;
 * Set this bit to reset the ovf_cnt of channel 7.
 */

#define LEDC_OVF_CNT_RESET_CH0    (BIT(16))
#define LEDC_OVF_CNT_RESET_CH0_M  (LEDC_OVF_CNT_RESET_CH0_V << LEDC_OVF_CNT_RESET_CH0_S)
#define LEDC_OVF_CNT_RESET_CH0_V  0x00000001
#define LEDC_OVF_CNT_RESET_CH0_S  16

/* LEDC_OVF_CNT_EN_CH0 : R/W; bitpos: [15]; default: 0;
 * This bit is used to enable the ovf_cnt of channel 7.
 */

#define LEDC_OVF_CNT_EN_CH0    (BIT(15))
#define LEDC_OVF_CNT_EN_CH0_M  (LEDC_OVF_CNT_EN_CH0_V << LEDC_OVF_CNT_EN_CH0_S)
#define LEDC_OVF_CNT_EN_CH0_V  0x00000001
#define LEDC_OVF_CNT_EN_CH0_S  15

/* LEDC_OVF_NUM_CH0 : R/W; bitpos: [14:5]; default: 0;
 * This register is used to configure the maximum times of overflow minus 1.
 *
 * The LEDC_OVF_CNT_CH7_INT interrupt will be triggered when channel 7
 * overflows for (LEDC_OVF_NUM_CH7 + 1) times.
 */

#define LEDC_OVF_NUM_CH0    0x000003ff
#define LEDC_OVF_NUM_CH0_M  (LEDC_OVF_NUM_CH0_V << LEDC_OVF_NUM_CH0_S)
#define LEDC_OVF_NUM_CH0_V  0x000003ff
#define LEDC_OVF_NUM_CH0_S  5

/* LEDC_PARA_UP_CH0 : WT; bitpos: [4]; default: 0;
 * This bit is used to update LEDC_HPOINT_CH7, LEDC_DUTY_START_CH7,
 * LEDC_SIG_OUT_EN_CH7, LEDC_TIMER_SEL_CH7, LEDC_DUTY_NUM_CH7,
 * LEDC_DUTY_CYCLE_CH7, LEDC_DUTY_SCALE_CH7, LEDC_DUTY_INC_CH7, and
 * LEDC_OVF_CNT_EN_CH7 fields for channel 7, and will be automatically
 * cleared by hardware.
 */

#define LEDC_PARA_UP_CH0    (BIT(4))
#define LEDC_PARA_UP_CH0_M  (LEDC_PARA_UP_CH0_V << LEDC_PARA_UP_CH0_S)
#define LEDC_PARA_UP_CH0_V  0x00000001
#define LEDC_PARA_UP_CH0_S  4

/* LEDC_IDLE_LV_CH0 : R/W; bitpos: [3]; default: 0;
 * This bit is used to control the output value when channel 7 is inactive
 * (when LEDC_SIG_OUT_EN_CH7 is 0).
 */

#define LEDC_IDLE_LV_CH0    (BIT(3))
#define LEDC_IDLE_LV_CH0_M  (LEDC_IDLE_LV_CH0_V << LEDC_IDLE_LV_CH0_S)
#define LEDC_IDLE_LV_CH0_V  0x00000001
#define LEDC_IDLE_LV_CH0_S  3

/* LEDC_SIG_OUT_EN_CH0 : R/W; bitpos: [2]; default: 0;
 * Set this bit to enable signal output on channel 7.
 */

#define LEDC_SIG_OUT_EN_CH0    (BIT(2))
#define LEDC_SIG_OUT_EN_CH0_M  (LEDC_SIG_OUT_EN_CH0_V << LEDC_SIG_OUT_EN_CH0_S)
#define LEDC_SIG_OUT_EN_CH0_V  0x00000001
#define LEDC_SIG_OUT_EN_CH0_S  2

/* LEDC_TIMER_SEL_CH0 : R/W; bitpos: [1:0]; default: 0;
 * This field is used to select one of timers for channel 7.
 *
 * 0: select timer0
 *
 * 1: select timer1
 *
 * 2: select timer2
 *
 * 3: select timer3
 */

#define LEDC_TIMER_SEL_CH0    0x00000003
#define LEDC_TIMER_SEL_CH0_M  (LEDC_TIMER_SEL_CH0_V << LEDC_TIMER_SEL_CH0_S)
#define LEDC_TIMER_SEL_CH0_V  0x00000003
#define LEDC_TIMER_SEL_CH0_S  0

/* LEDC_CH7_HPOINT_REG register
 * High point register for channel 7
 */

#define LEDC_CH7_HPOINT_REG (DR_REG_LEDC_BASE + 0x90)

/* LEDC_HPOINT_CH0 : R/W; bitpos: [13:0]; default: 0;
 * The output value changes to high when the selected timers has reached the
 * value specified by this register.
 */

#define LEDC_HPOINT_CH0    0x00003fff
#define LEDC_HPOINT_CH0_M  (LEDC_HPOINT_CH0_V << LEDC_HPOINT_CH0_S)
#define LEDC_HPOINT_CH0_V  0x00003fff
#define LEDC_HPOINT_CH0_S  0

/* LEDC_CH7_DUTY_REG register
 * Initial duty cycle for channel 7
 */

#define LEDC_CH7_DUTY_REG (DR_REG_LEDC_BASE + 0x94)

/* LEDC_DUTY_CH0 : R/W; bitpos: [18:0]; default: 0;
 * This register is used to change the output duty by controlling the Lpoint.
 *
 * The output value turns to low when the selected timers has reached the
 * Lpoint.
 */

#define LEDC_DUTY_CH0    0x0007ffff
#define LEDC_DUTY_CH0_M  (LEDC_DUTY_CH0_V << LEDC_DUTY_CH0_S)
#define LEDC_DUTY_CH0_V  0x0007ffff
#define LEDC_DUTY_CH0_S  0

/* LEDC_CH7_CONF1_REG register
 * Configuration register 1 for channel 7
 */

#define LEDC_CH7_CONF1_REG (DR_REG_LEDC_BASE + 0x98)

/* LEDC_DUTY_START_CH0 : R/W/SC; bitpos: [31]; default: 0;
 * Other configured fields in LEDC_CH7_CONF1_REG will start to take effect
 * when this bit is set to 1.
 */

#define LEDC_DUTY_START_CH0    (BIT(31))
#define LEDC_DUTY_START_CH0_M  (LEDC_DUTY_START_CH0_V << LEDC_DUTY_START_CH0_S)
#define LEDC_DUTY_START_CH0_V  0x00000001
#define LEDC_DUTY_START_CH0_S  31

/* LEDC_DUTY_INC_CH0 : R/W; bitpos: [30]; default: 1;
 * This register is used to increase or decrease the duty of output signal
 * on channel 7. 1: Increase; 0: Decrease.
 */

#define LEDC_DUTY_INC_CH0    (BIT(30))
#define LEDC_DUTY_INC_CH0_M  (LEDC_DUTY_INC_CH0_V << LEDC_DUTY_INC_CH0_S)
#define LEDC_DUTY_INC_CH0_V  0x00000001
#define LEDC_DUTY_INC_CH0_S  30

/* LEDC_DUTY_NUM_CH0 : R/W; bitpos: [29:20]; default: 0;
 * This register is used to control the number of times the duty cycle will
 * be changed.
 */

#define LEDC_DUTY_NUM_CH0    0x000003ff
#define LEDC_DUTY_NUM_CH0_M  (LEDC_DUTY_NUM_CH0_V << LEDC_DUTY_NUM_CH0_S)
#define LEDC_DUTY_NUM_CH0_V  0x000003ff
#define LEDC_DUTY_NUM_CH0_S  20

/* LEDC_DUTY_CYCLE_CH0 : R/W; bitpos: [19:10]; default: 0;
 * The duty will change every LEDC_DUTY_CYCLE_CH7 on channel 7.
 */

#define LEDC_DUTY_CYCLE_CH0    0x000003ff
#define LEDC_DUTY_CYCLE_CH0_M  (LEDC_DUTY_CYCLE_CH0_V << LEDC_DUTY_CYCLE_CH0_S)
#define LEDC_DUTY_CYCLE_CH0_V  0x000003ff
#define LEDC_DUTY_CYCLE_CH0_S  10

/* LEDC_DUTY_SCALE_CH0 : R/W; bitpos: [9:0]; default: 0;
 * This register is used to configure the changing step scale of duty on
 * channel 7.
 */

#define LEDC_DUTY_SCALE_CH0    0x000003ff
#define LEDC_DUTY_SCALE_CH0_M  (LEDC_DUTY_SCALE_CH0_V << LEDC_DUTY_SCALE_CH0_S)
#define LEDC_DUTY_SCALE_CH0_V  0x000003ff
#define LEDC_DUTY_SCALE_CH0_S  0

/* LEDC_CH7_DUTY_R_REG register
 * Current duty cycle for channel 7
 */

#define LEDC_CH7_DUTY_R_REG (DR_REG_LEDC_BASE + 0x9c)

/* LEDC_DUTY_R_CH0 : RO; bitpos: [18:0]; default: 0;
 * This register stores the current duty of output signal on channel 7.
 */

#define LEDC_DUTY_R_CH0    0x0007ffff
#define LEDC_DUTY_R_CH0_M  (LEDC_DUTY_R_CH0_V << LEDC_DUTY_R_CH0_S)
#define LEDC_DUTY_R_CH0_V  0x0007ffff
#define LEDC_DUTY_R_CH0_S  0

/* LEDC_TIMER0_CONF_REG register
 * Timer 0 configuration
 */

#define LEDC_TIMER0_CONF_REG (DR_REG_LEDC_BASE + 0xa0)

/* LEDC_TIMER0_PARA_UP : WT; bitpos: [25]; default: 0;
 * Set this bit to update LEDC_CLK_DIV_TIMER0 and LEDC_TIMER0_DUTY_RES.
 */

#define LEDC_TIMER0_PARA_UP    (BIT(25))
#define LEDC_TIMER0_PARA_UP_M  (LEDC_TIMER0_PARA_UP_V << LEDC_TIMER0_PARA_UP_S)
#define LEDC_TIMER0_PARA_UP_V  0x00000001
#define LEDC_TIMER0_PARA_UP_S  25

/* LEDC_TICK_SEL_TIMER0 : R/W; bitpos: [24]; default: 0;
 * This bit is used to select clock for timer 0. When this bit is set to 1
 * LEDC_APB_CLK_SEL[1:0] should be 1, otherwise the timer clock may be not
 * accurate.
 *
 * 1'h0: SLOW_CLK 1'h1: REF_TICK
 */

#define LEDC_TICK_SEL_TIMER0    (BIT(24))
#define LEDC_TICK_SEL_TIMER0_M  (LEDC_TICK_SEL_TIMER0_V << LEDC_TICK_SEL_TIMER0_S)
#define LEDC_TICK_SEL_TIMER0_V  0x00000001
#define LEDC_TICK_SEL_TIMER0_S  24

/* LEDC_TIMER0_RST : R/W; bitpos: [23]; default: 1;
 * This bit is used to reset timer 0. The counter will show 0 after reset.
 */

#define LEDC_TIMER0_RST    (BIT(23))
#define LEDC_TIMER0_RST_M  (LEDC_TIMER0_RST_V << LEDC_TIMER0_RST_S)
#define LEDC_TIMER0_RST_V  0x00000001
#define LEDC_TIMER0_RST_S  23

/* LEDC_TIMER0_PAUSE : R/W; bitpos: [22]; default: 0;
 * This bit is used to suspend the counter in timer 0.
 */

#define LEDC_TIMER0_PAUSE    (BIT(22))
#define LEDC_TIMER0_PAUSE_M  (LEDC_TIMER0_PAUSE_V << LEDC_TIMER0_PAUSE_S)
#define LEDC_TIMER0_PAUSE_V  0x00000001
#define LEDC_TIMER0_PAUSE_S  22

/* LEDC_CLK_DIV_TIMER0 : R/W; bitpos: [21:4]; default: 0;
 * This register is used to configure the divisor for the divider in timer 0.
 *
 * The least significant eight bits represent the fractional part.
 */

#define LEDC_CLK_DIV_TIMER0    0x0003ffff
#define LEDC_CLK_DIV_TIMER0_M  (LEDC_CLK_DIV_TIMER0_V << LEDC_CLK_DIV_TIMER0_S)
#define LEDC_CLK_DIV_TIMER0_V  0x0003ffff
#define LEDC_CLK_DIV_TIMER0_S  4

/* LEDC_TIMER0_DUTY_RES : R/W; bitpos: [3:0]; default: 0;
 * This register is used to control the range of the counter in timer 0.
 */

#define LEDC_TIMER0_DUTY_RES    0x0000000f
#define LEDC_TIMER0_DUTY_RES_M  (LEDC_TIMER0_DUTY_RES_V << LEDC_TIMER0_DUTY_RES_S)
#define LEDC_TIMER0_DUTY_RES_V  0x0000000f
#define LEDC_TIMER0_DUTY_RES_S  0

/* LEDC_TIMER0_VALUE_REG register
 * Timer 0 current counter value
 */

#define LEDC_TIMER0_VALUE_REG (DR_REG_LEDC_BASE + 0xa4)

/* LEDC_TIMER0_CNT : RO; bitpos: [13:0]; default: 0;
 * This register stores the current counter value of timer 0.
 */

#define LEDC_TIMER0_CNT    0x00003fff
#define LEDC_TIMER0_CNT_M  (LEDC_TIMER0_CNT_V << LEDC_TIMER0_CNT_S)
#define LEDC_TIMER0_CNT_V  0x00003fff
#define LEDC_TIMER0_CNT_S  0

/* LEDC_TIMER1_CONF_REG register
 * Timer 1 configuration
 */

#define LEDC_TIMER1_CONF_REG (DR_REG_LEDC_BASE + 0xa8)

/* LEDC_TIMER0_PARA_UP : WT; bitpos: [25]; default: 0;
 * Set this bit to update LEDC_CLK_DIV_TIMER1 and LEDC_TIMER1_DUTY_RES.
 */

#define LEDC_TIMER0_PARA_UP    (BIT(25))
#define LEDC_TIMER0_PARA_UP_M  (LEDC_TIMER0_PARA_UP_V << LEDC_TIMER0_PARA_UP_S)
#define LEDC_TIMER0_PARA_UP_V  0x00000001
#define LEDC_TIMER0_PARA_UP_S  25

/* LEDC_TICK_SEL_TIMER0 : R/W; bitpos: [24]; default: 0;
 * This bit is used to select clock for timer 1. When this bit is set to 1
 * LEDC_APB_CLK_SEL[1:0] should be 1, otherwise the timer clock may be not
 * accurate.
 *
 * 1'h0: SLOW_CLK 1'h1: REF_TICK
 */

#define LEDC_TICK_SEL_TIMER0    (BIT(24))
#define LEDC_TICK_SEL_TIMER0_M  (LEDC_TICK_SEL_TIMER0_V << LEDC_TICK_SEL_TIMER0_S)
#define LEDC_TICK_SEL_TIMER0_V  0x00000001
#define LEDC_TICK_SEL_TIMER0_S  24

/* LEDC_TIMER0_RST : R/W; bitpos: [23]; default: 1;
 * This bit is used to reset timer 1. The counter will show 0 after reset.
 */

#define LEDC_TIMER0_RST    (BIT(23))
#define LEDC_TIMER0_RST_M  (LEDC_TIMER0_RST_V << LEDC_TIMER0_RST_S)
#define LEDC_TIMER0_RST_V  0x00000001
#define LEDC_TIMER0_RST_S  23

/* LEDC_TIMER0_PAUSE : R/W; bitpos: [22]; default: 0;
 * This bit is used to suspend the counter in timer 1.
 */

#define LEDC_TIMER0_PAUSE    (BIT(22))
#define LEDC_TIMER0_PAUSE_M  (LEDC_TIMER0_PAUSE_V << LEDC_TIMER0_PAUSE_S)
#define LEDC_TIMER0_PAUSE_V  0x00000001
#define LEDC_TIMER0_PAUSE_S  22

/* LEDC_CLK_DIV_TIMER0 : R/W; bitpos: [21:4]; default: 0;
 * This register is used to configure the divisor for the divider in timer 1.
 *
 * The least significant eight bits represent the fractional part.
 */

#define LEDC_CLK_DIV_TIMER0    0x0003ffff
#define LEDC_CLK_DIV_TIMER0_M  (LEDC_CLK_DIV_TIMER0_V << LEDC_CLK_DIV_TIMER0_S)
#define LEDC_CLK_DIV_TIMER0_V  0x0003ffff
#define LEDC_CLK_DIV_TIMER0_S  4

/* LEDC_TIMER0_DUTY_RES : R/W; bitpos: [3:0]; default: 0;
 * This register is used to control the range of the counter in timer 1.
 */

#define LEDC_TIMER0_DUTY_RES    0x0000000f
#define LEDC_TIMER0_DUTY_RES_M  (LEDC_TIMER0_DUTY_RES_V << LEDC_TIMER0_DUTY_RES_S)
#define LEDC_TIMER0_DUTY_RES_V  0x0000000f
#define LEDC_TIMER0_DUTY_RES_S  0

/* LEDC_TIMER1_VALUE_REG register
 * Timer 1 current counter value
 */

#define LEDC_TIMER1_VALUE_REG (DR_REG_LEDC_BASE + 0xac)

/* LEDC_TIMER0_CNT : RO; bitpos: [13:0]; default: 0;
 * This register stores the current counter value of timer 1.
 */

#define LEDC_TIMER0_CNT    0x00003fff
#define LEDC_TIMER0_CNT_M  (LEDC_TIMER0_CNT_V << LEDC_TIMER0_CNT_S)
#define LEDC_TIMER0_CNT_V  0x00003fff
#define LEDC_TIMER0_CNT_S  0

/* LEDC_TIMER2_CONF_REG register
 * Timer 2 configuration
 */

#define LEDC_TIMER2_CONF_REG (DR_REG_LEDC_BASE + 0xb0)

/* LEDC_TIMER0_PARA_UP : WT; bitpos: [25]; default: 0;
 * Set this bit to update LEDC_CLK_DIV_TIMER2 and LEDC_TIMER2_DUTY_RES.
 */

#define LEDC_TIMER0_PARA_UP    (BIT(25))
#define LEDC_TIMER0_PARA_UP_M  (LEDC_TIMER0_PARA_UP_V << LEDC_TIMER0_PARA_UP_S)
#define LEDC_TIMER0_PARA_UP_V  0x00000001
#define LEDC_TIMER0_PARA_UP_S  25

/* LEDC_TICK_SEL_TIMER0 : R/W; bitpos: [24]; default: 0;
 * This bit is used to select clock for timer 2. When this bit is set to 1
 * LEDC_APB_CLK_SEL[1:0] should be 1, otherwise the timer clock may be not
 * accurate.
 *
 * 1'h0: SLOW_CLK 1'h1: REF_TICK
 */

#define LEDC_TICK_SEL_TIMER0    (BIT(24))
#define LEDC_TICK_SEL_TIMER0_M  (LEDC_TICK_SEL_TIMER0_V << LEDC_TICK_SEL_TIMER0_S)
#define LEDC_TICK_SEL_TIMER0_V  0x00000001
#define LEDC_TICK_SEL_TIMER0_S  24

/* LEDC_TIMER0_RST : R/W; bitpos: [23]; default: 1;
 * This bit is used to reset timer 2. The counter will show 0 after reset.
 */

#define LEDC_TIMER0_RST    (BIT(23))
#define LEDC_TIMER0_RST_M  (LEDC_TIMER0_RST_V << LEDC_TIMER0_RST_S)
#define LEDC_TIMER0_RST_V  0x00000001
#define LEDC_TIMER0_RST_S  23

/* LEDC_TIMER0_PAUSE : R/W; bitpos: [22]; default: 0;
 * This bit is used to suspend the counter in timer 2.
 */

#define LEDC_TIMER0_PAUSE    (BIT(22))
#define LEDC_TIMER0_PAUSE_M  (LEDC_TIMER0_PAUSE_V << LEDC_TIMER0_PAUSE_S)
#define LEDC_TIMER0_PAUSE_V  0x00000001
#define LEDC_TIMER0_PAUSE_S  22

/* LEDC_CLK_DIV_TIMER0 : R/W; bitpos: [21:4]; default: 0;
 * This register is used to configure the divisor for the divider in timer 2.
 *
 * The least significant eight bits represent the fractional part.
 */

#define LEDC_CLK_DIV_TIMER0    0x0003ffff
#define LEDC_CLK_DIV_TIMER0_M  (LEDC_CLK_DIV_TIMER0_V << LEDC_CLK_DIV_TIMER0_S)
#define LEDC_CLK_DIV_TIMER0_V  0x0003ffff
#define LEDC_CLK_DIV_TIMER0_S  4

/* LEDC_TIMER0_DUTY_RES : R/W; bitpos: [3:0]; default: 0;
 * This register is used to control the range of the counter in timer 2.
 */

#define LEDC_TIMER0_DUTY_RES    0x0000000f
#define LEDC_TIMER0_DUTY_RES_M  (LEDC_TIMER0_DUTY_RES_V << LEDC_TIMER0_DUTY_RES_S)
#define LEDC_TIMER0_DUTY_RES_V  0x0000000f
#define LEDC_TIMER0_DUTY_RES_S  0

/* LEDC_TIMER2_VALUE_REG register
 * Timer 2 current counter value
 */

#define LEDC_TIMER2_VALUE_REG (DR_REG_LEDC_BASE + 0xb4)

/* LEDC_TIMER0_CNT : RO; bitpos: [13:0]; default: 0;
 * This register stores the current counter value of timer 2.
 */

#define LEDC_TIMER0_CNT    0x00003fff
#define LEDC_TIMER0_CNT_M  (LEDC_TIMER0_CNT_V << LEDC_TIMER0_CNT_S)
#define LEDC_TIMER0_CNT_V  0x00003fff
#define LEDC_TIMER0_CNT_S  0

/* LEDC_TIMER3_CONF_REG register
 * Timer 3 configuration
 */

#define LEDC_TIMER3_CONF_REG (DR_REG_LEDC_BASE + 0xb8)

/* LEDC_TIMER0_PARA_UP : WT; bitpos: [25]; default: 0;
 * Set this bit to update LEDC_CLK_DIV_TIMER3 and LEDC_TIMER3_DUTY_RES.
 */

#define LEDC_TIMER0_PARA_UP    (BIT(25))
#define LEDC_TIMER0_PARA_UP_M  (LEDC_TIMER0_PARA_UP_V << LEDC_TIMER0_PARA_UP_S)
#define LEDC_TIMER0_PARA_UP_V  0x00000001
#define LEDC_TIMER0_PARA_UP_S  25

/* LEDC_TICK_SEL_TIMER0 : R/W; bitpos: [24]; default: 0;
 * This bit is used to select clock for timer 3. When this bit is set to 1
 * LEDC_APB_CLK_SEL[1:0] should be 1, otherwise the timer clock may be not
 * accurate.
 *
 * 1'h0: SLOW_CLK 1'h1: REF_TICK
 */

#define LEDC_TICK_SEL_TIMER0    (BIT(24))
#define LEDC_TICK_SEL_TIMER0_M  (LEDC_TICK_SEL_TIMER0_V << LEDC_TICK_SEL_TIMER0_S)
#define LEDC_TICK_SEL_TIMER0_V  0x00000001
#define LEDC_TICK_SEL_TIMER0_S  24

/* LEDC_TIMER0_RST : R/W; bitpos: [23]; default: 1;
 * This bit is used to reset timer 3. The counter will show 0 after reset.
 */

#define LEDC_TIMER0_RST    (BIT(23))
#define LEDC_TIMER0_RST_M  (LEDC_TIMER0_RST_V << LEDC_TIMER0_RST_S)
#define LEDC_TIMER0_RST_V  0x00000001
#define LEDC_TIMER0_RST_S  23

/* LEDC_TIMER0_PAUSE : R/W; bitpos: [22]; default: 0;
 * This bit is used to suspend the counter in timer 3.
 */

#define LEDC_TIMER0_PAUSE    (BIT(22))
#define LEDC_TIMER0_PAUSE_M  (LEDC_TIMER0_PAUSE_V << LEDC_TIMER0_PAUSE_S)
#define LEDC_TIMER0_PAUSE_V  0x00000001
#define LEDC_TIMER0_PAUSE_S  22

/* LEDC_CLK_DIV_TIMER0 : R/W; bitpos: [21:4]; default: 0;
 * This register is used to configure the divisor for the divider in timer 3.
 *
 * The least significant eight bits represent the fractional part.
 */

#define LEDC_CLK_DIV_TIMER0    0x0003ffff
#define LEDC_CLK_DIV_TIMER0_M  (LEDC_CLK_DIV_TIMER0_V << LEDC_CLK_DIV_TIMER0_S)
#define LEDC_CLK_DIV_TIMER0_V  0x0003ffff
#define LEDC_CLK_DIV_TIMER0_S  4

/* LEDC_TIMER0_DUTY_RES : R/W; bitpos: [3:0]; default: 0;
 * This register is used to control the range of the counter in timer 3.
 */

#define LEDC_TIMER0_DUTY_RES    0x0000000f
#define LEDC_TIMER0_DUTY_RES_M  (LEDC_TIMER0_DUTY_RES_V << LEDC_TIMER0_DUTY_RES_S)
#define LEDC_TIMER0_DUTY_RES_V  0x0000000f
#define LEDC_TIMER0_DUTY_RES_S  0

/* LEDC_TIMER3_VALUE_REG register
 * Timer 3 current counter value
 */

#define LEDC_TIMER3_VALUE_REG (DR_REG_LEDC_BASE + 0xbc)

/* LEDC_TIMER0_CNT : RO; bitpos: [13:0]; default: 0;
 * This register stores the current counter value of timer 3.
 */

#define LEDC_TIMER0_CNT    0x00003fff
#define LEDC_TIMER0_CNT_M  (LEDC_TIMER0_CNT_V << LEDC_TIMER0_CNT_S)
#define LEDC_TIMER0_CNT_V  0x00003fff
#define LEDC_TIMER0_CNT_S  0

/* LEDC_INT_RAW_REG register
 * Raw interrupt status
 */

#define LEDC_INT_RAW_REG (DR_REG_LEDC_BASE + 0xc0)

/* LEDC_OVF_CNT_CH7_INT_RAW : R/WTC/SS; bitpos: [19]; default: 0;
 * Interrupt raw bit for channel 7. Triggered when the ovf_cnt has reached
 * the value specified by LEDC_OVF_NUM_CH7.
 */

#define LEDC_OVF_CNT_CH7_INT_RAW    (BIT(19))
#define LEDC_OVF_CNT_CH7_INT_RAW_M  (LEDC_OVF_CNT_CH7_INT_RAW_V << LEDC_OVF_CNT_CH7_INT_RAW_S)
#define LEDC_OVF_CNT_CH7_INT_RAW_V  0x00000001
#define LEDC_OVF_CNT_CH7_INT_RAW_S  19

/* LEDC_OVF_CNT_CH6_INT_RAW : R/WTC/SS; bitpos: [18]; default: 0;
 * Interrupt raw bit for channel 6. Triggered when the ovf_cnt has reached
 * the value specified by LEDC_OVF_NUM_CH6.
 */

#define LEDC_OVF_CNT_CH6_INT_RAW    (BIT(18))
#define LEDC_OVF_CNT_CH6_INT_RAW_M  (LEDC_OVF_CNT_CH6_INT_RAW_V << LEDC_OVF_CNT_CH6_INT_RAW_S)
#define LEDC_OVF_CNT_CH6_INT_RAW_V  0x00000001
#define LEDC_OVF_CNT_CH6_INT_RAW_S  18

/* LEDC_OVF_CNT_CH5_INT_RAW : R/WTC/SS; bitpos: [17]; default: 0;
 * Interrupt raw bit for channel 5. Triggered when the ovf_cnt has reached
 * the value specified by LEDC_OVF_NUM_CH5.
 */

#define LEDC_OVF_CNT_CH5_INT_RAW    (BIT(17))
#define LEDC_OVF_CNT_CH5_INT_RAW_M  (LEDC_OVF_CNT_CH5_INT_RAW_V << LEDC_OVF_CNT_CH5_INT_RAW_S)
#define LEDC_OVF_CNT_CH5_INT_RAW_V  0x00000001
#define LEDC_OVF_CNT_CH5_INT_RAW_S  17

/* LEDC_OVF_CNT_CH4_INT_RAW : R/WTC/SS; bitpos: [16]; default: 0;
 * Interrupt raw bit for channel 4. Triggered when the ovf_cnt has reached
 * the value specified by LEDC_OVF_NUM_CH4.
 */

#define LEDC_OVF_CNT_CH4_INT_RAW    (BIT(16))
#define LEDC_OVF_CNT_CH4_INT_RAW_M  (LEDC_OVF_CNT_CH4_INT_RAW_V << LEDC_OVF_CNT_CH4_INT_RAW_S)
#define LEDC_OVF_CNT_CH4_INT_RAW_V  0x00000001
#define LEDC_OVF_CNT_CH4_INT_RAW_S  16

/* LEDC_OVF_CNT_CH3_INT_RAW : R/WTC/SS; bitpos: [15]; default: 0;
 * Interrupt raw bit for channel 3. Triggered when the ovf_cnt has reached
 * the value specified by LEDC_OVF_NUM_CH3.
 */

#define LEDC_OVF_CNT_CH3_INT_RAW    (BIT(15))
#define LEDC_OVF_CNT_CH3_INT_RAW_M  (LEDC_OVF_CNT_CH3_INT_RAW_V << LEDC_OVF_CNT_CH3_INT_RAW_S)
#define LEDC_OVF_CNT_CH3_INT_RAW_V  0x00000001
#define LEDC_OVF_CNT_CH3_INT_RAW_S  15

/* LEDC_OVF_CNT_CH2_INT_RAW : R/WTC/SS; bitpos: [14]; default: 0;
 * Interrupt raw bit for channel 2. Triggered when the ovf_cnt has reached
 * the value specified by LEDC_OVF_NUM_CH2.
 */

#define LEDC_OVF_CNT_CH2_INT_RAW    (BIT(14))
#define LEDC_OVF_CNT_CH2_INT_RAW_M  (LEDC_OVF_CNT_CH2_INT_RAW_V << LEDC_OVF_CNT_CH2_INT_RAW_S)
#define LEDC_OVF_CNT_CH2_INT_RAW_V  0x00000001
#define LEDC_OVF_CNT_CH2_INT_RAW_S  14

/* LEDC_OVF_CNT_CH1_INT_RAW : R/WTC/SS; bitpos: [13]; default: 0;
 * Interrupt raw bit for channel 1. Triggered when the ovf_cnt has reached
 * the value specified by LEDC_OVF_NUM_CH1.
 */

#define LEDC_OVF_CNT_CH1_INT_RAW    (BIT(13))
#define LEDC_OVF_CNT_CH1_INT_RAW_M  (LEDC_OVF_CNT_CH1_INT_RAW_V << LEDC_OVF_CNT_CH1_INT_RAW_S)
#define LEDC_OVF_CNT_CH1_INT_RAW_V  0x00000001
#define LEDC_OVF_CNT_CH1_INT_RAW_S  13

/* LEDC_OVF_CNT_CH0_INT_RAW : R/WTC/SS; bitpos: [12]; default: 0;
 * Interrupt raw bit for channel 0. Triggered when the ovf_cnt has reached
 * the value specified by LEDC_OVF_NUM_CH0.
 */

#define LEDC_OVF_CNT_CH0_INT_RAW    (BIT(12))
#define LEDC_OVF_CNT_CH0_INT_RAW_M  (LEDC_OVF_CNT_CH0_INT_RAW_V << LEDC_OVF_CNT_CH0_INT_RAW_S)
#define LEDC_OVF_CNT_CH0_INT_RAW_V  0x00000001
#define LEDC_OVF_CNT_CH0_INT_RAW_S  12

/* LEDC_DUTY_CHNG_END_CH7_INT_RAW : R/WTC/SS; bitpos: [11]; default: 0;
 * Interrupt raw bit for channel 7. Triggered when the gradual change of
 * duty has finished.
 */

#define LEDC_DUTY_CHNG_END_CH7_INT_RAW    (BIT(11))
#define LEDC_DUTY_CHNG_END_CH7_INT_RAW_M  (LEDC_DUTY_CHNG_END_CH7_INT_RAW_V << LEDC_DUTY_CHNG_END_CH7_INT_RAW_S)
#define LEDC_DUTY_CHNG_END_CH7_INT_RAW_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH7_INT_RAW_S  11

/* LEDC_DUTY_CHNG_END_CH6_INT_RAW : R/WTC/SS; bitpos: [10]; default: 0;
 * Interrupt raw bit for channel 6. Triggered when the gradual change of
 * duty has finished.
 */

#define LEDC_DUTY_CHNG_END_CH6_INT_RAW    (BIT(10))
#define LEDC_DUTY_CHNG_END_CH6_INT_RAW_M  (LEDC_DUTY_CHNG_END_CH6_INT_RAW_V << LEDC_DUTY_CHNG_END_CH6_INT_RAW_S)
#define LEDC_DUTY_CHNG_END_CH6_INT_RAW_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH6_INT_RAW_S  10

/* LEDC_DUTY_CHNG_END_CH5_INT_RAW : R/WTC/SS; bitpos: [9]; default: 0;
 * Interrupt raw bit for channel 5. Triggered when the gradual change of
 * duty has finished.
 */

#define LEDC_DUTY_CHNG_END_CH5_INT_RAW    (BIT(9))
#define LEDC_DUTY_CHNG_END_CH5_INT_RAW_M  (LEDC_DUTY_CHNG_END_CH5_INT_RAW_V << LEDC_DUTY_CHNG_END_CH5_INT_RAW_S)
#define LEDC_DUTY_CHNG_END_CH5_INT_RAW_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH5_INT_RAW_S  9

/* LEDC_DUTY_CHNG_END_CH4_INT_RAW : R/WTC/SS; bitpos: [8]; default: 0;
 * Interrupt raw bit for channel 4. Triggered when the gradual change of
 * duty has finished.
 */

#define LEDC_DUTY_CHNG_END_CH4_INT_RAW    (BIT(8))
#define LEDC_DUTY_CHNG_END_CH4_INT_RAW_M  (LEDC_DUTY_CHNG_END_CH4_INT_RAW_V << LEDC_DUTY_CHNG_END_CH4_INT_RAW_S)
#define LEDC_DUTY_CHNG_END_CH4_INT_RAW_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH4_INT_RAW_S  8

/* LEDC_DUTY_CHNG_END_CH3_INT_RAW : R/WTC/SS; bitpos: [7]; default: 0;
 * Interrupt raw bit for channel 3. Triggered when the gradual change of
 * duty has finished.
 */

#define LEDC_DUTY_CHNG_END_CH3_INT_RAW    (BIT(7))
#define LEDC_DUTY_CHNG_END_CH3_INT_RAW_M  (LEDC_DUTY_CHNG_END_CH3_INT_RAW_V << LEDC_DUTY_CHNG_END_CH3_INT_RAW_S)
#define LEDC_DUTY_CHNG_END_CH3_INT_RAW_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH3_INT_RAW_S  7

/* LEDC_DUTY_CHNG_END_CH2_INT_RAW : R/WTC/SS; bitpos: [6]; default: 0;
 * Interrupt raw bit for channel 2. Triggered when the gradual change of
 * duty has finished.
 */

#define LEDC_DUTY_CHNG_END_CH2_INT_RAW    (BIT(6))
#define LEDC_DUTY_CHNG_END_CH2_INT_RAW_M  (LEDC_DUTY_CHNG_END_CH2_INT_RAW_V << LEDC_DUTY_CHNG_END_CH2_INT_RAW_S)
#define LEDC_DUTY_CHNG_END_CH2_INT_RAW_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH2_INT_RAW_S  6

/* LEDC_DUTY_CHNG_END_CH1_INT_RAW : R/WTC/SS; bitpos: [5]; default: 0;
 * Interrupt raw bit for channel 1. Triggered when the gradual change of
 * duty has finished.
 */

#define LEDC_DUTY_CHNG_END_CH1_INT_RAW    (BIT(5))
#define LEDC_DUTY_CHNG_END_CH1_INT_RAW_M  (LEDC_DUTY_CHNG_END_CH1_INT_RAW_V << LEDC_DUTY_CHNG_END_CH1_INT_RAW_S)
#define LEDC_DUTY_CHNG_END_CH1_INT_RAW_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH1_INT_RAW_S  5

/* LEDC_DUTY_CHNG_END_CH0_INT_RAW : R/WTC/SS; bitpos: [4]; default: 0;
 * Interrupt raw bit for channel 0. Triggered when the gradual change of
 * duty has finished.
 */

#define LEDC_DUTY_CHNG_END_CH0_INT_RAW    (BIT(4))
#define LEDC_DUTY_CHNG_END_CH0_INT_RAW_M  (LEDC_DUTY_CHNG_END_CH0_INT_RAW_V << LEDC_DUTY_CHNG_END_CH0_INT_RAW_S)
#define LEDC_DUTY_CHNG_END_CH0_INT_RAW_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH0_INT_RAW_S  4

/* LEDC_TIMER3_OVF_INT_RAW : R/WTC/SS; bitpos: [3]; default: 0;
 * Triggered when the timer3 has reached its maximum counter value.
 */

#define LEDC_TIMER3_OVF_INT_RAW    (BIT(3))
#define LEDC_TIMER3_OVF_INT_RAW_M  (LEDC_TIMER3_OVF_INT_RAW_V << LEDC_TIMER3_OVF_INT_RAW_S)
#define LEDC_TIMER3_OVF_INT_RAW_V  0x00000001
#define LEDC_TIMER3_OVF_INT_RAW_S  3

/* LEDC_TIMER2_OVF_INT_RAW : R/WTC/SS; bitpos: [2]; default: 0;
 * Triggered when the timer2 has reached its maximum counter value.
 */

#define LEDC_TIMER2_OVF_INT_RAW    (BIT(2))
#define LEDC_TIMER2_OVF_INT_RAW_M  (LEDC_TIMER2_OVF_INT_RAW_V << LEDC_TIMER2_OVF_INT_RAW_S)
#define LEDC_TIMER2_OVF_INT_RAW_V  0x00000001
#define LEDC_TIMER2_OVF_INT_RAW_S  2

/* LEDC_TIMER1_OVF_INT_RAW : R/WTC/SS; bitpos: [1]; default: 0;
 * Triggered when the timer1 has reached its maximum counter value.
 */

#define LEDC_TIMER1_OVF_INT_RAW    (BIT(1))
#define LEDC_TIMER1_OVF_INT_RAW_M  (LEDC_TIMER1_OVF_INT_RAW_V << LEDC_TIMER1_OVF_INT_RAW_S)
#define LEDC_TIMER1_OVF_INT_RAW_V  0x00000001
#define LEDC_TIMER1_OVF_INT_RAW_S  1

/* LEDC_TIMER0_OVF_INT_RAW : R/WTC/SS; bitpos: [0]; default: 0;
 * Triggered when the timer0 has reached its maximum counter value.
 */

#define LEDC_TIMER0_OVF_INT_RAW    (BIT(0))
#define LEDC_TIMER0_OVF_INT_RAW_M  (LEDC_TIMER0_OVF_INT_RAW_V << LEDC_TIMER0_OVF_INT_RAW_S)
#define LEDC_TIMER0_OVF_INT_RAW_V  0x00000001
#define LEDC_TIMER0_OVF_INT_RAW_S  0

/* LEDC_INT_ST_REG register
 * Masked interrupt status
 */

#define LEDC_INT_ST_REG (DR_REG_LEDC_BASE + 0xc4)

/* LEDC_OVF_CNT_CH7_INT_ST : RO; bitpos: [19]; default: 0;
 * This is the masked interrupt status bit for the LEDC_OVF_CNT_CH7_INT
 * interrupt when LEDC_OVF_CNT_CH7_INT_ENA is set to 1.
 */

#define LEDC_OVF_CNT_CH7_INT_ST    (BIT(19))
#define LEDC_OVF_CNT_CH7_INT_ST_M  (LEDC_OVF_CNT_CH7_INT_ST_V << LEDC_OVF_CNT_CH7_INT_ST_S)
#define LEDC_OVF_CNT_CH7_INT_ST_V  0x00000001
#define LEDC_OVF_CNT_CH7_INT_ST_S  19

/* LEDC_OVF_CNT_CH6_INT_ST : RO; bitpos: [18]; default: 0;
 * This is the masked interrupt status bit for the LEDC_OVF_CNT_CH6_INT
 * interrupt when LEDC_OVF_CNT_CH6_INT_ENA is set to 1.
 */

#define LEDC_OVF_CNT_CH6_INT_ST    (BIT(18))
#define LEDC_OVF_CNT_CH6_INT_ST_M  (LEDC_OVF_CNT_CH6_INT_ST_V << LEDC_OVF_CNT_CH6_INT_ST_S)
#define LEDC_OVF_CNT_CH6_INT_ST_V  0x00000001
#define LEDC_OVF_CNT_CH6_INT_ST_S  18

/* LEDC_OVF_CNT_CH5_INT_ST : RO; bitpos: [17]; default: 0;
 * This is the masked interrupt status bit for the LEDC_OVF_CNT_CH5_INT
 * interrupt when LEDC_OVF_CNT_CH5_INT_ENA is set to 1.
 */

#define LEDC_OVF_CNT_CH5_INT_ST    (BIT(17))
#define LEDC_OVF_CNT_CH5_INT_ST_M  (LEDC_OVF_CNT_CH5_INT_ST_V << LEDC_OVF_CNT_CH5_INT_ST_S)
#define LEDC_OVF_CNT_CH5_INT_ST_V  0x00000001
#define LEDC_OVF_CNT_CH5_INT_ST_S  17

/* LEDC_OVF_CNT_CH4_INT_ST : RO; bitpos: [16]; default: 0;
 * This is the masked interrupt status bit for the LEDC_OVF_CNT_CH4_INT
 * interrupt when LEDC_OVF_CNT_CH4_INT_ENA is set to 1.
 */

#define LEDC_OVF_CNT_CH4_INT_ST    (BIT(16))
#define LEDC_OVF_CNT_CH4_INT_ST_M  (LEDC_OVF_CNT_CH4_INT_ST_V << LEDC_OVF_CNT_CH4_INT_ST_S)
#define LEDC_OVF_CNT_CH4_INT_ST_V  0x00000001
#define LEDC_OVF_CNT_CH4_INT_ST_S  16

/* LEDC_OVF_CNT_CH3_INT_ST : RO; bitpos: [15]; default: 0;
 * This is the masked interrupt status bit for the LEDC_OVF_CNT_CH3_INT
 * interrupt when LEDC_OVF_CNT_CH3_INT_ENA is set to 1.
 */

#define LEDC_OVF_CNT_CH3_INT_ST    (BIT(15))
#define LEDC_OVF_CNT_CH3_INT_ST_M  (LEDC_OVF_CNT_CH3_INT_ST_V << LEDC_OVF_CNT_CH3_INT_ST_S)
#define LEDC_OVF_CNT_CH3_INT_ST_V  0x00000001
#define LEDC_OVF_CNT_CH3_INT_ST_S  15

/* LEDC_OVF_CNT_CH2_INT_ST : RO; bitpos: [14]; default: 0;
 * This is the masked interrupt status bit for the LEDC_OVF_CNT_CH2_INT
 * interrupt when LEDC_OVF_CNT_CH2_INT_ENA is set to 1.
 */

#define LEDC_OVF_CNT_CH2_INT_ST    (BIT(14))
#define LEDC_OVF_CNT_CH2_INT_ST_M  (LEDC_OVF_CNT_CH2_INT_ST_V << LEDC_OVF_CNT_CH2_INT_ST_S)
#define LEDC_OVF_CNT_CH2_INT_ST_V  0x00000001
#define LEDC_OVF_CNT_CH2_INT_ST_S  14

/* LEDC_OVF_CNT_CH1_INT_ST : RO; bitpos: [13]; default: 0;
 * This is the masked interrupt status bit for the LEDC_OVF_CNT_CH1_INT
 * interrupt when LEDC_OVF_CNT_CH1_INT_ENA is set to 1.
 */

#define LEDC_OVF_CNT_CH1_INT_ST    (BIT(13))
#define LEDC_OVF_CNT_CH1_INT_ST_M  (LEDC_OVF_CNT_CH1_INT_ST_V << LEDC_OVF_CNT_CH1_INT_ST_S)
#define LEDC_OVF_CNT_CH1_INT_ST_V  0x00000001
#define LEDC_OVF_CNT_CH1_INT_ST_S  13

/* LEDC_OVF_CNT_CH0_INT_ST : RO; bitpos: [12]; default: 0;
 * This is the masked interrupt status bit for the LEDC_OVF_CNT_CH0_INT
 * interrupt when LEDC_OVF_CNT_CH0_INT_ENA is set to 1.
 */

#define LEDC_OVF_CNT_CH0_INT_ST    (BIT(12))
#define LEDC_OVF_CNT_CH0_INT_ST_M  (LEDC_OVF_CNT_CH0_INT_ST_V << LEDC_OVF_CNT_CH0_INT_ST_S)
#define LEDC_OVF_CNT_CH0_INT_ST_V  0x00000001
#define LEDC_OVF_CNT_CH0_INT_ST_S  12

/* LEDC_DUTY_CHNG_END_CH7_INT_ST : RO; bitpos: [11]; default: 0;
 * This is the masked interrupt status bit for the
 * LEDC_DUTY_CHNG_END_CH7_INT interrupt when
 * LEDC_DUTY_CHNG_END_CH7_INT_ENAIS set to 1.
 */

#define LEDC_DUTY_CHNG_END_CH7_INT_ST    (BIT(11))
#define LEDC_DUTY_CHNG_END_CH7_INT_ST_M  (LEDC_DUTY_CHNG_END_CH7_INT_ST_V << LEDC_DUTY_CHNG_END_CH7_INT_ST_S)
#define LEDC_DUTY_CHNG_END_CH7_INT_ST_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH7_INT_ST_S  11

/* LEDC_DUTY_CHNG_END_CH6_INT_ST : RO; bitpos: [10]; default: 0;
 * This is the masked interrupt status bit for the
 * LEDC_DUTY_CHNG_END_CH6_INT interrupt when
 * LEDC_DUTY_CHNG_END_CH6_INT_ENAIS set to 1.
 */

#define LEDC_DUTY_CHNG_END_CH6_INT_ST    (BIT(10))
#define LEDC_DUTY_CHNG_END_CH6_INT_ST_M  (LEDC_DUTY_CHNG_END_CH6_INT_ST_V << LEDC_DUTY_CHNG_END_CH6_INT_ST_S)
#define LEDC_DUTY_CHNG_END_CH6_INT_ST_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH6_INT_ST_S  10

/* LEDC_DUTY_CHNG_END_CH5_INT_ST : RO; bitpos: [9]; default: 0;
 * This is the masked interrupt status bit for the
 * LEDC_DUTY_CHNG_END_CH5_INT interrupt when
 * LEDC_DUTY_CHNG_END_CH5_INT_ENAIS set to 1.
 */

#define LEDC_DUTY_CHNG_END_CH5_INT_ST    (BIT(9))
#define LEDC_DUTY_CHNG_END_CH5_INT_ST_M  (LEDC_DUTY_CHNG_END_CH5_INT_ST_V << LEDC_DUTY_CHNG_END_CH5_INT_ST_S)
#define LEDC_DUTY_CHNG_END_CH5_INT_ST_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH5_INT_ST_S  9

/* LEDC_DUTY_CHNG_END_CH4_INT_ST : RO; bitpos: [8]; default: 0;
 * This is the masked interrupt status bit for the
 * LEDC_DUTY_CHNG_END_CH4_INT interrupt when
 * LEDC_DUTY_CHNG_END_CH4_INT_ENAIS set to 1.
 */

#define LEDC_DUTY_CHNG_END_CH4_INT_ST    (BIT(8))
#define LEDC_DUTY_CHNG_END_CH4_INT_ST_M  (LEDC_DUTY_CHNG_END_CH4_INT_ST_V << LEDC_DUTY_CHNG_END_CH4_INT_ST_S)
#define LEDC_DUTY_CHNG_END_CH4_INT_ST_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH4_INT_ST_S  8

/* LEDC_DUTY_CHNG_END_CH3_INT_ST : RO; bitpos: [7]; default: 0;
 * This is the masked interrupt status bit for the
 * LEDC_DUTY_CHNG_END_CH3_INT interrupt when
 * LEDC_DUTY_CHNG_END_CH3_INT_ENAIS set to 1.
 */

#define LEDC_DUTY_CHNG_END_CH3_INT_ST    (BIT(7))
#define LEDC_DUTY_CHNG_END_CH3_INT_ST_M  (LEDC_DUTY_CHNG_END_CH3_INT_ST_V << LEDC_DUTY_CHNG_END_CH3_INT_ST_S)
#define LEDC_DUTY_CHNG_END_CH3_INT_ST_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH3_INT_ST_S  7

/* LEDC_DUTY_CHNG_END_CH2_INT_ST : RO; bitpos: [6]; default: 0;
 * This is the masked interrupt status bit for the
 * LEDC_DUTY_CHNG_END_CH2_INT interrupt when
 * LEDC_DUTY_CHNG_END_CH2_INT_ENAIS set to 1.
 */

#define LEDC_DUTY_CHNG_END_CH2_INT_ST    (BIT(6))
#define LEDC_DUTY_CHNG_END_CH2_INT_ST_M  (LEDC_DUTY_CHNG_END_CH2_INT_ST_V << LEDC_DUTY_CHNG_END_CH2_INT_ST_S)
#define LEDC_DUTY_CHNG_END_CH2_INT_ST_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH2_INT_ST_S  6

/* LEDC_DUTY_CHNG_END_CH1_INT_ST : RO; bitpos: [5]; default: 0;
 * This is the masked interrupt status bit for the
 * LEDC_DUTY_CHNG_END_CH1_INT interrupt when
 * LEDC_DUTY_CHNG_END_CH1_INT_ENAIS set to 1.
 */

#define LEDC_DUTY_CHNG_END_CH1_INT_ST    (BIT(5))
#define LEDC_DUTY_CHNG_END_CH1_INT_ST_M  (LEDC_DUTY_CHNG_END_CH1_INT_ST_V << LEDC_DUTY_CHNG_END_CH1_INT_ST_S)
#define LEDC_DUTY_CHNG_END_CH1_INT_ST_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH1_INT_ST_S  5

/* LEDC_DUTY_CHNG_END_CH0_INT_ST : RO; bitpos: [4]; default: 0;
 * This is the masked interrupt status bit for the
 * LEDC_DUTY_CHNG_END_CH0_INT interrupt when
 * LEDC_DUTY_CHNG_END_CH0_INT_ENAIS set to 1.
 */

#define LEDC_DUTY_CHNG_END_CH0_INT_ST    (BIT(4))
#define LEDC_DUTY_CHNG_END_CH0_INT_ST_M  (LEDC_DUTY_CHNG_END_CH0_INT_ST_V << LEDC_DUTY_CHNG_END_CH0_INT_ST_S)
#define LEDC_DUTY_CHNG_END_CH0_INT_ST_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH0_INT_ST_S  4

/* LEDC_TIMER3_OVF_INT_ST : RO; bitpos: [3]; default: 0;
 * This is the masked interrupt status bit for the LEDC_TIMER3_OVF_INT
 * interrupt when LEDC_TIMER3_OVF_INT_ENA is set to 1.
 */

#define LEDC_TIMER3_OVF_INT_ST    (BIT(3))
#define LEDC_TIMER3_OVF_INT_ST_M  (LEDC_TIMER3_OVF_INT_ST_V << LEDC_TIMER3_OVF_INT_ST_S)
#define LEDC_TIMER3_OVF_INT_ST_V  0x00000001
#define LEDC_TIMER3_OVF_INT_ST_S  3

/* LEDC_TIMER2_OVF_INT_ST : RO; bitpos: [2]; default: 0;
 * This is the masked interrupt status bit for the LEDC_TIMER2_OVF_INT
 * interrupt when LEDC_TIMER2_OVF_INT_ENA is set to 1.
 */

#define LEDC_TIMER2_OVF_INT_ST    (BIT(2))
#define LEDC_TIMER2_OVF_INT_ST_M  (LEDC_TIMER2_OVF_INT_ST_V << LEDC_TIMER2_OVF_INT_ST_S)
#define LEDC_TIMER2_OVF_INT_ST_V  0x00000001
#define LEDC_TIMER2_OVF_INT_ST_S  2

/* LEDC_TIMER1_OVF_INT_ST : RO; bitpos: [1]; default: 0;
 * This is the masked interrupt status bit for the LEDC_TIMER1_OVF_INT
 * interrupt when LEDC_TIMER1_OVF_INT_ENA is set to 1.
 */

#define LEDC_TIMER1_OVF_INT_ST    (BIT(1))
#define LEDC_TIMER1_OVF_INT_ST_M  (LEDC_TIMER1_OVF_INT_ST_V << LEDC_TIMER1_OVF_INT_ST_S)
#define LEDC_TIMER1_OVF_INT_ST_V  0x00000001
#define LEDC_TIMER1_OVF_INT_ST_S  1

/* LEDC_TIMER0_OVF_INT_ST : RO; bitpos: [0]; default: 0;
 * This is the masked interrupt status bit for the LEDC_TIMER0_OVF_INT
 * interrupt when LEDC_TIMER0_OVF_INT_ENA is set to 1.
 */

#define LEDC_TIMER0_OVF_INT_ST    (BIT(0))
#define LEDC_TIMER0_OVF_INT_ST_M  (LEDC_TIMER0_OVF_INT_ST_V << LEDC_TIMER0_OVF_INT_ST_S)
#define LEDC_TIMER0_OVF_INT_ST_V  0x00000001
#define LEDC_TIMER0_OVF_INT_ST_S  0

/* LEDC_INT_ENA_REG register
 * Interrupt enable bits
 */

#define LEDC_INT_ENA_REG (DR_REG_LEDC_BASE + 0xc8)

/* LEDC_OVF_CNT_CH7_INT_ENA : R/W; bitpos: [19]; default: 0;
 * The interrupt enable bit for the LEDC_OVF_CNT_CH7_INT interrupt.
 */

#define LEDC_OVF_CNT_CH7_INT_ENA    (BIT(19))
#define LEDC_OVF_CNT_CH7_INT_ENA_M  (LEDC_OVF_CNT_CH7_INT_ENA_V << LEDC_OVF_CNT_CH7_INT_ENA_S)
#define LEDC_OVF_CNT_CH7_INT_ENA_V  0x00000001
#define LEDC_OVF_CNT_CH7_INT_ENA_S  19

/* LEDC_OVF_CNT_CH6_INT_ENA : R/W; bitpos: [18]; default: 0;
 * The interrupt enable bit for the LEDC_OVF_CNT_CH6_INT interrupt.
 */

#define LEDC_OVF_CNT_CH6_INT_ENA    (BIT(18))
#define LEDC_OVF_CNT_CH6_INT_ENA_M  (LEDC_OVF_CNT_CH6_INT_ENA_V << LEDC_OVF_CNT_CH6_INT_ENA_S)
#define LEDC_OVF_CNT_CH6_INT_ENA_V  0x00000001
#define LEDC_OVF_CNT_CH6_INT_ENA_S  18

/* LEDC_OVF_CNT_CH5_INT_ENA : R/W; bitpos: [17]; default: 0;
 * The interrupt enable bit for the LEDC_OVF_CNT_CH5_INT interrupt.
 */

#define LEDC_OVF_CNT_CH5_INT_ENA    (BIT(17))
#define LEDC_OVF_CNT_CH5_INT_ENA_M  (LEDC_OVF_CNT_CH5_INT_ENA_V << LEDC_OVF_CNT_CH5_INT_ENA_S)
#define LEDC_OVF_CNT_CH5_INT_ENA_V  0x00000001
#define LEDC_OVF_CNT_CH5_INT_ENA_S  17

/* LEDC_OVF_CNT_CH4_INT_ENA : R/W; bitpos: [16]; default: 0;
 * The interrupt enable bit for the LEDC_OVF_CNT_CH4_INT interrupt.
 */

#define LEDC_OVF_CNT_CH4_INT_ENA    (BIT(16))
#define LEDC_OVF_CNT_CH4_INT_ENA_M  (LEDC_OVF_CNT_CH4_INT_ENA_V << LEDC_OVF_CNT_CH4_INT_ENA_S)
#define LEDC_OVF_CNT_CH4_INT_ENA_V  0x00000001
#define LEDC_OVF_CNT_CH4_INT_ENA_S  16

/* LEDC_OVF_CNT_CH3_INT_ENA : R/W; bitpos: [15]; default: 0;
 * The interrupt enable bit for the LEDC_OVF_CNT_CH3_INT interrupt.
 */

#define LEDC_OVF_CNT_CH3_INT_ENA    (BIT(15))
#define LEDC_OVF_CNT_CH3_INT_ENA_M  (LEDC_OVF_CNT_CH3_INT_ENA_V << LEDC_OVF_CNT_CH3_INT_ENA_S)
#define LEDC_OVF_CNT_CH3_INT_ENA_V  0x00000001
#define LEDC_OVF_CNT_CH3_INT_ENA_S  15

/* LEDC_OVF_CNT_CH2_INT_ENA : R/W; bitpos: [14]; default: 0;
 * The interrupt enable bit for the LEDC_OVF_CNT_CH2_INT interrupt.
 */

#define LEDC_OVF_CNT_CH2_INT_ENA    (BIT(14))
#define LEDC_OVF_CNT_CH2_INT_ENA_M  (LEDC_OVF_CNT_CH2_INT_ENA_V << LEDC_OVF_CNT_CH2_INT_ENA_S)
#define LEDC_OVF_CNT_CH2_INT_ENA_V  0x00000001
#define LEDC_OVF_CNT_CH2_INT_ENA_S  14

/* LEDC_OVF_CNT_CH1_INT_ENA : R/W; bitpos: [13]; default: 0;
 * The interrupt enable bit for the LEDC_OVF_CNT_CH1_INT interrupt.
 */

#define LEDC_OVF_CNT_CH1_INT_ENA    (BIT(13))
#define LEDC_OVF_CNT_CH1_INT_ENA_M  (LEDC_OVF_CNT_CH1_INT_ENA_V << LEDC_OVF_CNT_CH1_INT_ENA_S)
#define LEDC_OVF_CNT_CH1_INT_ENA_V  0x00000001
#define LEDC_OVF_CNT_CH1_INT_ENA_S  13

/* LEDC_OVF_CNT_CH0_INT_ENA : R/W; bitpos: [12]; default: 0;
 * The interrupt enable bit for the LEDC_OVF_CNT_CH0_INT interrupt.
 */

#define LEDC_OVF_CNT_CH0_INT_ENA    (BIT(12))
#define LEDC_OVF_CNT_CH0_INT_ENA_M  (LEDC_OVF_CNT_CH0_INT_ENA_V << LEDC_OVF_CNT_CH0_INT_ENA_S)
#define LEDC_OVF_CNT_CH0_INT_ENA_V  0x00000001
#define LEDC_OVF_CNT_CH0_INT_ENA_S  12

/* LEDC_DUTY_CHNG_END_CH7_INT_ENA : R/W; bitpos: [11]; default: 0;
 * The interrupt enable bit for the LEDC_DUTY_CHNG_END_CH7_INT interrupt.
 */

#define LEDC_DUTY_CHNG_END_CH7_INT_ENA    (BIT(11))
#define LEDC_DUTY_CHNG_END_CH7_INT_ENA_M  (LEDC_DUTY_CHNG_END_CH7_INT_ENA_V << LEDC_DUTY_CHNG_END_CH7_INT_ENA_S)
#define LEDC_DUTY_CHNG_END_CH7_INT_ENA_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH7_INT_ENA_S  11

/* LEDC_DUTY_CHNG_END_CH6_INT_ENA : R/W; bitpos: [10]; default: 0;
 * The interrupt enable bit for the LEDC_DUTY_CHNG_END_CH6_INT interrupt.
 */

#define LEDC_DUTY_CHNG_END_CH6_INT_ENA    (BIT(10))
#define LEDC_DUTY_CHNG_END_CH6_INT_ENA_M  (LEDC_DUTY_CHNG_END_CH6_INT_ENA_V << LEDC_DUTY_CHNG_END_CH6_INT_ENA_S)
#define LEDC_DUTY_CHNG_END_CH6_INT_ENA_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH6_INT_ENA_S  10

/* LEDC_DUTY_CHNG_END_CH5_INT_ENA : R/W; bitpos: [9]; default: 0;
 * The interrupt enable bit for the LEDC_DUTY_CHNG_END_CH5_INT interrupt.
 */

#define LEDC_DUTY_CHNG_END_CH5_INT_ENA    (BIT(9))
#define LEDC_DUTY_CHNG_END_CH5_INT_ENA_M  (LEDC_DUTY_CHNG_END_CH5_INT_ENA_V << LEDC_DUTY_CHNG_END_CH5_INT_ENA_S)
#define LEDC_DUTY_CHNG_END_CH5_INT_ENA_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH5_INT_ENA_S  9

/* LEDC_DUTY_CHNG_END_CH4_INT_ENA : R/W; bitpos: [8]; default: 0;
 * The interrupt enable bit for the LEDC_DUTY_CHNG_END_CH4_INT interrupt.
 */

#define LEDC_DUTY_CHNG_END_CH4_INT_ENA    (BIT(8))
#define LEDC_DUTY_CHNG_END_CH4_INT_ENA_M  (LEDC_DUTY_CHNG_END_CH4_INT_ENA_V << LEDC_DUTY_CHNG_END_CH4_INT_ENA_S)
#define LEDC_DUTY_CHNG_END_CH4_INT_ENA_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH4_INT_ENA_S  8

/* LEDC_DUTY_CHNG_END_CH3_INT_ENA : R/W; bitpos: [7]; default: 0;
 * The interrupt enable bit for the LEDC_DUTY_CHNG_END_CH3_INT interrupt.
 */

#define LEDC_DUTY_CHNG_END_CH3_INT_ENA    (BIT(7))
#define LEDC_DUTY_CHNG_END_CH3_INT_ENA_M  (LEDC_DUTY_CHNG_END_CH3_INT_ENA_V << LEDC_DUTY_CHNG_END_CH3_INT_ENA_S)
#define LEDC_DUTY_CHNG_END_CH3_INT_ENA_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH3_INT_ENA_S  7

/* LEDC_DUTY_CHNG_END_CH2_INT_ENA : R/W; bitpos: [6]; default: 0;
 * The interrupt enable bit for the LEDC_DUTY_CHNG_END_CH2_INT interrupt.
 */

#define LEDC_DUTY_CHNG_END_CH2_INT_ENA    (BIT(6))
#define LEDC_DUTY_CHNG_END_CH2_INT_ENA_M  (LEDC_DUTY_CHNG_END_CH2_INT_ENA_V << LEDC_DUTY_CHNG_END_CH2_INT_ENA_S)
#define LEDC_DUTY_CHNG_END_CH2_INT_ENA_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH2_INT_ENA_S  6

/* LEDC_DUTY_CHNG_END_CH1_INT_ENA : R/W; bitpos: [5]; default: 0;
 * The interrupt enable bit for the LEDC_DUTY_CHNG_END_CH1_INT interrupt.
 */

#define LEDC_DUTY_CHNG_END_CH1_INT_ENA    (BIT(5))
#define LEDC_DUTY_CHNG_END_CH1_INT_ENA_M  (LEDC_DUTY_CHNG_END_CH1_INT_ENA_V << LEDC_DUTY_CHNG_END_CH1_INT_ENA_S)
#define LEDC_DUTY_CHNG_END_CH1_INT_ENA_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH1_INT_ENA_S  5

/* LEDC_DUTY_CHNG_END_CH0_INT_ENA : R/W; bitpos: [4]; default: 0;
 * The interrupt enable bit for the LEDC_DUTY_CHNG_END_CH0_INT interrupt.
 */

#define LEDC_DUTY_CHNG_END_CH0_INT_ENA    (BIT(4))
#define LEDC_DUTY_CHNG_END_CH0_INT_ENA_M  (LEDC_DUTY_CHNG_END_CH0_INT_ENA_V << LEDC_DUTY_CHNG_END_CH0_INT_ENA_S)
#define LEDC_DUTY_CHNG_END_CH0_INT_ENA_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH0_INT_ENA_S  4

/* LEDC_TIMER3_OVF_INT_ENA : R/W; bitpos: [3]; default: 0;
 * The interrupt enable bit for the LEDC_TIMER3_OVF_INT interrupt.
 */

#define LEDC_TIMER3_OVF_INT_ENA    (BIT(3))
#define LEDC_TIMER3_OVF_INT_ENA_M  (LEDC_TIMER3_OVF_INT_ENA_V << LEDC_TIMER3_OVF_INT_ENA_S)
#define LEDC_TIMER3_OVF_INT_ENA_V  0x00000001
#define LEDC_TIMER3_OVF_INT_ENA_S  3

/* LEDC_TIMER2_OVF_INT_ENA : R/W; bitpos: [2]; default: 0;
 * The interrupt enable bit for the LEDC_TIMER2_OVF_INT interrupt.
 */

#define LEDC_TIMER2_OVF_INT_ENA    (BIT(2))
#define LEDC_TIMER2_OVF_INT_ENA_M  (LEDC_TIMER2_OVF_INT_ENA_V << LEDC_TIMER2_OVF_INT_ENA_S)
#define LEDC_TIMER2_OVF_INT_ENA_V  0x00000001
#define LEDC_TIMER2_OVF_INT_ENA_S  2

/* LEDC_TIMER1_OVF_INT_ENA : R/W; bitpos: [1]; default: 0;
 * The interrupt enable bit for the LEDC_TIMER1_OVF_INT interrupt.
 */

#define LEDC_TIMER1_OVF_INT_ENA    (BIT(1))
#define LEDC_TIMER1_OVF_INT_ENA_M  (LEDC_TIMER1_OVF_INT_ENA_V << LEDC_TIMER1_OVF_INT_ENA_S)
#define LEDC_TIMER1_OVF_INT_ENA_V  0x00000001
#define LEDC_TIMER1_OVF_INT_ENA_S  1

/* LEDC_TIMER0_OVF_INT_ENA : R/W; bitpos: [0]; default: 0;
 * The interrupt enable bit for the LEDC_TIMER0_OVF_INT interrupt.
 */

#define LEDC_TIMER0_OVF_INT_ENA    (BIT(0))
#define LEDC_TIMER0_OVF_INT_ENA_M  (LEDC_TIMER0_OVF_INT_ENA_V << LEDC_TIMER0_OVF_INT_ENA_S)
#define LEDC_TIMER0_OVF_INT_ENA_V  0x00000001
#define LEDC_TIMER0_OVF_INT_ENA_S  0

/* LEDC_INT_CLR_REG register
 * Interrupt clear bits
 */

#define LEDC_INT_CLR_REG (DR_REG_LEDC_BASE + 0xcc)

/* LEDC_OVF_CNT_CH7_INT_CLR : WT; bitpos: [19]; default: 0;
 * Set this bit to clear the LEDC_OVF_CNT_CH7_INT interrupt.
 */

#define LEDC_OVF_CNT_CH7_INT_CLR    (BIT(19))
#define LEDC_OVF_CNT_CH7_INT_CLR_M  (LEDC_OVF_CNT_CH7_INT_CLR_V << LEDC_OVF_CNT_CH7_INT_CLR_S)
#define LEDC_OVF_CNT_CH7_INT_CLR_V  0x00000001
#define LEDC_OVF_CNT_CH7_INT_CLR_S  19

/* LEDC_OVF_CNT_CH6_INT_CLR : WT; bitpos: [18]; default: 0;
 * Set this bit to clear the LEDC_OVF_CNT_CH6_INT interrupt.
 */

#define LEDC_OVF_CNT_CH6_INT_CLR    (BIT(18))
#define LEDC_OVF_CNT_CH6_INT_CLR_M  (LEDC_OVF_CNT_CH6_INT_CLR_V << LEDC_OVF_CNT_CH6_INT_CLR_S)
#define LEDC_OVF_CNT_CH6_INT_CLR_V  0x00000001
#define LEDC_OVF_CNT_CH6_INT_CLR_S  18

/* LEDC_OVF_CNT_CH5_INT_CLR : WT; bitpos: [17]; default: 0;
 * Set this bit to clear the LEDC_OVF_CNT_CH5_INT interrupt.
 */

#define LEDC_OVF_CNT_CH5_INT_CLR    (BIT(17))
#define LEDC_OVF_CNT_CH5_INT_CLR_M  (LEDC_OVF_CNT_CH5_INT_CLR_V << LEDC_OVF_CNT_CH5_INT_CLR_S)
#define LEDC_OVF_CNT_CH5_INT_CLR_V  0x00000001
#define LEDC_OVF_CNT_CH5_INT_CLR_S  17

/* LEDC_OVF_CNT_CH4_INT_CLR : WT; bitpos: [16]; default: 0;
 * Set this bit to clear the LEDC_OVF_CNT_CH4_INT interrupt.
 */

#define LEDC_OVF_CNT_CH4_INT_CLR    (BIT(16))
#define LEDC_OVF_CNT_CH4_INT_CLR_M  (LEDC_OVF_CNT_CH4_INT_CLR_V << LEDC_OVF_CNT_CH4_INT_CLR_S)
#define LEDC_OVF_CNT_CH4_INT_CLR_V  0x00000001
#define LEDC_OVF_CNT_CH4_INT_CLR_S  16

/* LEDC_OVF_CNT_CH3_INT_CLR : WT; bitpos: [15]; default: 0;
 * Set this bit to clear the LEDC_OVF_CNT_CH3_INT interrupt.
 */

#define LEDC_OVF_CNT_CH3_INT_CLR    (BIT(15))
#define LEDC_OVF_CNT_CH3_INT_CLR_M  (LEDC_OVF_CNT_CH3_INT_CLR_V << LEDC_OVF_CNT_CH3_INT_CLR_S)
#define LEDC_OVF_CNT_CH3_INT_CLR_V  0x00000001
#define LEDC_OVF_CNT_CH3_INT_CLR_S  15

/* LEDC_OVF_CNT_CH2_INT_CLR : WT; bitpos: [14]; default: 0;
 * Set this bit to clear the LEDC_OVF_CNT_CH2_INT interrupt.
 */

#define LEDC_OVF_CNT_CH2_INT_CLR    (BIT(14))
#define LEDC_OVF_CNT_CH2_INT_CLR_M  (LEDC_OVF_CNT_CH2_INT_CLR_V << LEDC_OVF_CNT_CH2_INT_CLR_S)
#define LEDC_OVF_CNT_CH2_INT_CLR_V  0x00000001
#define LEDC_OVF_CNT_CH2_INT_CLR_S  14

/* LEDC_OVF_CNT_CH1_INT_CLR : WT; bitpos: [13]; default: 0;
 * Set this bit to clear the LEDC_OVF_CNT_CH1_INT interrupt.
 */

#define LEDC_OVF_CNT_CH1_INT_CLR    (BIT(13))
#define LEDC_OVF_CNT_CH1_INT_CLR_M  (LEDC_OVF_CNT_CH1_INT_CLR_V << LEDC_OVF_CNT_CH1_INT_CLR_S)
#define LEDC_OVF_CNT_CH1_INT_CLR_V  0x00000001
#define LEDC_OVF_CNT_CH1_INT_CLR_S  13

/* LEDC_OVF_CNT_CH0_INT_CLR : WT; bitpos: [12]; default: 0;
 * Set this bit to clear the LEDC_OVF_CNT_CH0_INT interrupt.
 */

#define LEDC_OVF_CNT_CH0_INT_CLR    (BIT(12))
#define LEDC_OVF_CNT_CH0_INT_CLR_M  (LEDC_OVF_CNT_CH0_INT_CLR_V << LEDC_OVF_CNT_CH0_INT_CLR_S)
#define LEDC_OVF_CNT_CH0_INT_CLR_V  0x00000001
#define LEDC_OVF_CNT_CH0_INT_CLR_S  12

/* LEDC_DUTY_CHNG_END_CH7_INT_CLR : WT; bitpos: [11]; default: 0;
 * Set this bit to clear the LEDC_DUTY_CHNG_END_CH7_INT interrupt.
 */

#define LEDC_DUTY_CHNG_END_CH7_INT_CLR    (BIT(11))
#define LEDC_DUTY_CHNG_END_CH7_INT_CLR_M  (LEDC_DUTY_CHNG_END_CH7_INT_CLR_V << LEDC_DUTY_CHNG_END_CH7_INT_CLR_S)
#define LEDC_DUTY_CHNG_END_CH7_INT_CLR_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH7_INT_CLR_S  11

/* LEDC_DUTY_CHNG_END_CH6_INT_CLR : WT; bitpos: [10]; default: 0;
 * Set this bit to clear the LEDC_DUTY_CHNG_END_CH6_INT interrupt.
 */

#define LEDC_DUTY_CHNG_END_CH6_INT_CLR    (BIT(10))
#define LEDC_DUTY_CHNG_END_CH6_INT_CLR_M  (LEDC_DUTY_CHNG_END_CH6_INT_CLR_V << LEDC_DUTY_CHNG_END_CH6_INT_CLR_S)
#define LEDC_DUTY_CHNG_END_CH6_INT_CLR_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH6_INT_CLR_S  10

/* LEDC_DUTY_CHNG_END_CH5_INT_CLR : WT; bitpos: [9]; default: 0;
 * Set this bit to clear the LEDC_DUTY_CHNG_END_CH5_INT interrupt.
 */

#define LEDC_DUTY_CHNG_END_CH5_INT_CLR    (BIT(9))
#define LEDC_DUTY_CHNG_END_CH5_INT_CLR_M  (LEDC_DUTY_CHNG_END_CH5_INT_CLR_V << LEDC_DUTY_CHNG_END_CH5_INT_CLR_S)
#define LEDC_DUTY_CHNG_END_CH5_INT_CLR_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH5_INT_CLR_S  9

/* LEDC_DUTY_CHNG_END_CH4_INT_CLR : WT; bitpos: [8]; default: 0;
 * Set this bit to clear the LEDC_DUTY_CHNG_END_CH4_INT interrupt.
 */

#define LEDC_DUTY_CHNG_END_CH4_INT_CLR    (BIT(8))
#define LEDC_DUTY_CHNG_END_CH4_INT_CLR_M  (LEDC_DUTY_CHNG_END_CH4_INT_CLR_V << LEDC_DUTY_CHNG_END_CH4_INT_CLR_S)
#define LEDC_DUTY_CHNG_END_CH4_INT_CLR_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH4_INT_CLR_S  8

/* LEDC_DUTY_CHNG_END_CH3_INT_CLR : WT; bitpos: [7]; default: 0;
 * Set this bit to clear the LEDC_DUTY_CHNG_END_CH3_INT interrupt.
 */

#define LEDC_DUTY_CHNG_END_CH3_INT_CLR    (BIT(7))
#define LEDC_DUTY_CHNG_END_CH3_INT_CLR_M  (LEDC_DUTY_CHNG_END_CH3_INT_CLR_V << LEDC_DUTY_CHNG_END_CH3_INT_CLR_S)
#define LEDC_DUTY_CHNG_END_CH3_INT_CLR_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH3_INT_CLR_S  7

/* LEDC_DUTY_CHNG_END_CH2_INT_CLR : WT; bitpos: [6]; default: 0;
 * Set this bit to clear the LEDC_DUTY_CHNG_END_CH2_INT interrupt.
 */

#define LEDC_DUTY_CHNG_END_CH2_INT_CLR    (BIT(6))
#define LEDC_DUTY_CHNG_END_CH2_INT_CLR_M  (LEDC_DUTY_CHNG_END_CH2_INT_CLR_V << LEDC_DUTY_CHNG_END_CH2_INT_CLR_S)
#define LEDC_DUTY_CHNG_END_CH2_INT_CLR_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH2_INT_CLR_S  6

/* LEDC_DUTY_CHNG_END_CH1_INT_CLR : WT; bitpos: [5]; default: 0;
 * Set this bit to clear the LEDC_DUTY_CHNG_END_CH1_INT interrupt.
 */

#define LEDC_DUTY_CHNG_END_CH1_INT_CLR    (BIT(5))
#define LEDC_DUTY_CHNG_END_CH1_INT_CLR_M  (LEDC_DUTY_CHNG_END_CH1_INT_CLR_V << LEDC_DUTY_CHNG_END_CH1_INT_CLR_S)
#define LEDC_DUTY_CHNG_END_CH1_INT_CLR_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH1_INT_CLR_S  5

/* LEDC_DUTY_CHNG_END_CH0_INT_CLR : WT; bitpos: [4]; default: 0;
 * Set this bit to clear the LEDC_DUTY_CHNG_END_CH0_INT interrupt.
 */

#define LEDC_DUTY_CHNG_END_CH0_INT_CLR    (BIT(4))
#define LEDC_DUTY_CHNG_END_CH0_INT_CLR_M  (LEDC_DUTY_CHNG_END_CH0_INT_CLR_V << LEDC_DUTY_CHNG_END_CH0_INT_CLR_S)
#define LEDC_DUTY_CHNG_END_CH0_INT_CLR_V  0x00000001
#define LEDC_DUTY_CHNG_END_CH0_INT_CLR_S  4

/* LEDC_TIMER3_OVF_INT_CLR : WT; bitpos: [3]; default: 0;
 * Set this bit to clear the LEDC_TIMER3_OVF_INT interrupt.
 */

#define LEDC_TIMER3_OVF_INT_CLR    (BIT(3))
#define LEDC_TIMER3_OVF_INT_CLR_M  (LEDC_TIMER3_OVF_INT_CLR_V << LEDC_TIMER3_OVF_INT_CLR_S)
#define LEDC_TIMER3_OVF_INT_CLR_V  0x00000001
#define LEDC_TIMER3_OVF_INT_CLR_S  3

/* LEDC_TIMER2_OVF_INT_CLR : WT; bitpos: [2]; default: 0;
 * Set this bit to clear the LEDC_TIMER2_OVF_INT interrupt.
 */

#define LEDC_TIMER2_OVF_INT_CLR    (BIT(2))
#define LEDC_TIMER2_OVF_INT_CLR_M  (LEDC_TIMER2_OVF_INT_CLR_V << LEDC_TIMER2_OVF_INT_CLR_S)
#define LEDC_TIMER2_OVF_INT_CLR_V  0x00000001
#define LEDC_TIMER2_OVF_INT_CLR_S  2

/* LEDC_TIMER1_OVF_INT_CLR : WT; bitpos: [1]; default: 0;
 * Set this bit to clear the LEDC_TIMER1_OVF_INT interrupt.
 */

#define LEDC_TIMER1_OVF_INT_CLR    (BIT(1))
#define LEDC_TIMER1_OVF_INT_CLR_M  (LEDC_TIMER1_OVF_INT_CLR_V << LEDC_TIMER1_OVF_INT_CLR_S)
#define LEDC_TIMER1_OVF_INT_CLR_V  0x00000001
#define LEDC_TIMER1_OVF_INT_CLR_S  1

/* LEDC_TIMER0_OVF_INT_CLR : WT; bitpos: [0]; default: 0;
 * Set this bit to clear the LEDC_TIMER0_OVF_INT interrupt.
 */

#define LEDC_TIMER0_OVF_INT_CLR    (BIT(0))
#define LEDC_TIMER0_OVF_INT_CLR_M  (LEDC_TIMER0_OVF_INT_CLR_V << LEDC_TIMER0_OVF_INT_CLR_S)
#define LEDC_TIMER0_OVF_INT_CLR_V  0x00000001
#define LEDC_TIMER0_OVF_INT_CLR_S  0

/* LEDC_CONF_REG register
 * Global ledc configuration register
 */

#define LEDC_CONF_REG (DR_REG_LEDC_BASE + 0xd0)

/* LEDC_CLK_EN : R/W; bitpos: [31]; default: 0;
 * This bit is used to control clock.
 *
 * 1'b1: Force clock on for register. 1'h0: Support clock only when
 * application writes registers.
 */

#define LEDC_CLK_EN    (BIT(31))
#define LEDC_CLK_EN_M  (LEDC_CLK_EN_V << LEDC_CLK_EN_S)
#define LEDC_CLK_EN_V  0x00000001
#define LEDC_CLK_EN_S  31

/* LEDC_APB_CLK_SEL : R/W; bitpos: [1:0]; default: 0;
 * This bit is used to select clock source for the 4 timers .
 *
 * 2'd1: APB_CLK 2'd2: RTC8M_CLK 2'd3: XTAL_CLK
 */

#define LEDC_APB_CLK_SEL    0x00000003
#define LEDC_APB_CLK_SEL_M  (LEDC_APB_CLK_SEL_V << LEDC_APB_CLK_SEL_S)
#define LEDC_APB_CLK_SEL_V  0x00000003
#define LEDC_APB_CLK_SEL_S  0

/* LEDC_DATE_REG register
 * Version control register
 */

#define LEDC_DATE_REG (DR_REG_LEDC_BASE + 0xfc)

/* LEDC_DATE : R/W; bitpos: [31:0]; default: 419693056;
 * This is the version control register.
 */

#define LEDC_DATE    0xffffffff
#define LEDC_DATE_M  (LEDC_DATE_V << LEDC_DATE_S)
#define LEDC_DATE_V  0xffffffff
#define LEDC_DATE_S  0

#endif /* __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_LEDC_H */
