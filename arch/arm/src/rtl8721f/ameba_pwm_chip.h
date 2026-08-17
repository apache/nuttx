/****************************************************************************
 * arch/arm/src/rtl8721f/ameba_pwm_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8721F_AMEBA_PWM_CHIP_H
#define __ARCH_ARM_SRC_RTL8721F_AMEBA_PWM_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip PWM wiring for RTL8721F (amebagreen2).  The shared driver
 * (arch/arm/src/common/ameba/ameba_pwm.c) includes this header to learn
 * which timer generates PWM, how many output channels it drives, its
 * register base, input clock, peripheral-clock masks and crossbar pad-mux
 * codes.  The shared driver is never edited (it reads only the macros below
 * -- the pad-mux codes are a per-channel table, NOT computed).
 *
 * Unlike amebadplus (one 8-channel TIM8), amebagreen2 spreads PWM across
 * four timers of four channels each (TIM4..TIM7, see IS_TIM_PWM_TIM in the
 * SDK ameba_pwmtimer.h).  This port drives TIM4 as the single time base with
 * four compare channels, matching the shared driver's model.  All values
 * below are taken from the SDK fwlib headers (verified, not guessed):
 *
 *   - AMEBA_PWM_TIMER_IDX / _NCHAN: TIM4 is timer index 4 with four CCRs.
 *   - AMEBA_PWM_BASE: the NON-secure TIM4 base (TIMER4_REG_BASE in the SDK
 *     hal_platform.h); the secure alias (0x51000000) must not be used from
 *     the non-secure world the KM4 runs in (same rule as the GPIO/SPI
 *     drivers).
 *   - AMEBA_PWM_CLKFREQ: TIM4 clocked from the 40 MHz XTAL (IS_TIM_40M_TIM
 *     in the SDK); f_out = CLKFREQ / ((PSC + 1) * (ARR + 1)) with 16-bit
 *     PSC and ARR.
 *   - AMEBA_PWM_PINMUX_FIDS: the per-channel crossbar function codes
 *     PINMUX_FUNCTION_TIM4_PWM0..3 (111..114).
 *   - AMEBA_PWM_APBPERIPH / _CLK: the "function" and "clock" args to
 *     RCC_PeriphClockCmd().  Unlike amebadplus, amebagreen2 uses distinct
 *     function-enable and clock-enable bits (APBPeriph_PWM0 = bit23,
 *     APBPeriph_PWM0_CLOCK = bit24, both in group 0).
 *   - AMEBA_PWM_IRQ: RTIM_TimeBaseInit() takes it even though this polling
 *     driver registers no callback (NULL); TIM4 is IRQ 11 (TIMER4_IRQ).
 */

#define AMEBA_PWM_TIMER_IDX       4            /* TIM4 (IS_TIM_PWM_TIM)     */
#define AMEBA_PWM_NCHAN           4            /* CCR0..CCR3 (PWM_CHAN_MAX) */
#define AMEBA_PWM_BASE            0x41000000ul /* NON-secure TIM4 base      */
#define AMEBA_PWM_CLKFREQ         40000000ul   /* TIM4 input clock (40 MHz) */
#define AMEBA_PWM_IRQ             11           /* TIMER4_IRQ                */

/* Crossbar pad-mux function code per channel, indexed by channel number
 * minus one (channel n uses AMEBA_PWM_PINMUX_FIDS[n - 1]).  On amebagreen2
 * these are PINMUX_FUNCTION_TIM4_PWM0..3 (111..114).
 */

#define AMEBA_PWM_PINMUX_FIDS     { 111, 112, 113, 114 }

/* APBPeriph_PWM0 (function) and APBPeriph_PWM0_CLOCK masks.  The group
 * selector (bit30) is 0; amebagreen2 splits function-enable (bit23) and
 * clock-enable (bit24) into distinct bits.
 */

#define AMEBA_PWM_APBPERIPH       (((uint32_t)0 << 30) | ((uint32_t)1 << 23))
#define AMEBA_PWM_APBPERIPH_CLK   (((uint32_t)0 << 30) | ((uint32_t)1 << 24))

#endif /* __ARCH_ARM_SRC_RTL8721F_AMEBA_PWM_CHIP_H */
