/****************************************************************************
 * arch/arm/src/rtl8730e/ameba_pwm_chip.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_RTL8730E_AMEBA_PWM_CHIP_H
#define __ARCH_ARM_SRC_RTL8730E_AMEBA_PWM_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip PWM wiring for RTL8730E (amebasmart CA32).  The shared driver
 * (arch/arm/src/common/ameba/ameba_pwm.c) includes this header to learn
 * which timer generates PWM, how many output channels it drives, its
 * register base, input clock, peripheral-clock masks and pad-mux codes.
 *
 * Sources: component/soc/amebasmart/fwlib/include/hal_platform.h (base),
 * sysreg_lsys.h (APBPeriph masks), ameba_pinmux.h (pad-mux code),
 * ameba_vector_table.h (IRQ), ameba_pwmtimer.h (IS_TIM_PWM_TIM,
 * IS_TIM_40M_TIM, PWM_CHAN_MAX and the two InitTypeDef layouts).
 */

#define AMEBA_PWM_TIMER_IDX       8            /* TIM8 (IS_TIM_PWM_TIM)     */

/* PWM_CHAN_MAX is 6 on amebasmart, not the 8 of the amebadplus/amebalite
 * TIM8 -- the CCR/TIM_IT_CCx sets stop at channel 5 here.
 */

#define AMEBA_PWM_NCHAN           6            /* CCR0..CCR5 (PWM_CHAN_MAX) */

/* NON-secure TIM8 base (TIMER8_REG_BASE); the secure alias TIM8_S at
 * 0x5100a000 must not be used from the non-secure world.
 */

#define AMEBA_PWM_BASE            0x4100a000ul /* NON-secure TIM8 base      */

/* TIM8 is one of the 40 MHz timers (IS_TIM_40M_TIM lists TIM8 and TIM9), so
 * f_out = CLKFREQ / ((PSC + 1) * (ARR + 1)).  The SDK's own pwmout_api.c
 * hardcodes the same 40 when converting microseconds to timer ticks.
 */

#define AMEBA_PWM_CLKFREQ         40000000ul   /* TIM8 input clock (40 MHz) */
#define AMEBA_PWM_IRQ             57           /* TIMER8_IRQ                */

/* amebasmart has a single generic PINMUX_FUNCTION_PWM (10) shared by every
 * PWM pad instead of the contiguous per-channel codes of amebadplus or the
 * grouped per-timer codes of amebagreen2/RTL8720F, so every channel in this
 * per-channel table carries the same value.
 */

#define AMEBA_PWM_PINMUX_FIDS     { 10, 10, 10, 10, 10, 10 }

/* APBPeriph_TIM8 (function) and APBPeriph_TIM8_CLOCK masks.  Equal on this
 * chip, but note the group selector: the amebasmart timer block is group 2
 * (bit30 = 2), unlike the SPI block here (group 1) or the I2C block (group
 * 0), so each chip must supply its own values.
 */

#define AMEBA_PWM_APBPERIPH       (((uint32_t)2 << 30) | ((uint32_t)1 << 24))
#define AMEBA_PWM_APBPERIPH_CLK   (((uint32_t)2 << 30) | ((uint32_t)1 << 24))

#endif /* __ARCH_ARM_SRC_RTL8730E_AMEBA_PWM_CHIP_H */
