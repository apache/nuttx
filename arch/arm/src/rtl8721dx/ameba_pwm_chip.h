/****************************************************************************
 * arch/arm/src/rtl8721dx/ameba_pwm_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8721DX_AMEBA_PWM_CHIP_H
#define __ARCH_ARM_SRC_RTL8721DX_AMEBA_PWM_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip PWM wiring for RTL8721DX (amebadplus).  The shared driver
 * (arch/arm/src/common/ameba/ameba_pwm.c) includes this header to learn
 * which timer generates PWM, how many output channels it drives, its
 * register base, input clock, peripheral-clock masks and crossbar pad-mux
 * codes.  A port to another Ameba chip supplies a same-named header on the
 * chip include path; the shared driver is never edited (it reads only the
 * macros below -- the pad-mux codes are a per-channel table, NOT computed).
 *
 * What differs per chip, from the SDK fwlib headers (verified, not guessed):
 *
 *   chip         PWM timer(s)   pad-mux codes (per channel)
 *   -----------  ------------   ---------------------------------------
 *   amebadplus   TIM8           PWM0..PWM7        = 52..59 (8, contig)
 *   amebalite    TIM8           PWM0..            = 55..   (8, contig)
 *   amebasmart   TIM8           PWM               = 10     (single code)
 *   amebagreen2  TIM4/5/6..     TIM4_PWM0..3      = 111..  (4 per timer)
 *   RTL8720F     TIM4/TIM5      TIM4_PWM0..3      = 45..   (4 per timer)
 *
 * The three pad-mux layouts (contiguous / single shared code / grouped four
 * per timer) are all expressed the same way: AMEBA_PWM_PINMUX_FIDS is a
 * per-channel list.  Contiguous chips list 52,53,...; a single-code chip
 * (amebasmart) lists 10,10,...; a grouped chip lists that timer's codes.
 * So a new chip only edits this header -- see the SPI/I2C/UART chip headers,
 * which pass their pad-mux codes as the same kind of per-signal table.
 *
 * Other per-chip macros:
 *   - AMEBA_PWM_TIMER_IDX / _NCHAN: the timer index this driver drives and
 *     its compare-channel count (8 CCRs on amebadplus, 4 on green2/8720F).
 *   - AMEBA_PWM_BASE: the NON-secure timer alias (0x411xxxxx); the secure
 *     alias (0x511xxxxx) must not be used from the non-secure world the KM4
 *     runs in (same rule as the GPIO/SPI drivers).
 *   - AMEBA_PWM_CLKFREQ: TIM8 is clocked from the 40 MHz XTAL on amebadplus;
 *     f_out = CLKFREQ / ((PSC + 1) * (ARR + 1)) with 16-bit PSC and ARR.
 *   - AMEBA_PWM_APBPERIPH / _CLK: the "function" and "clock" args to
 *     RCC_PeriphClockCmd(); equal on this chip, bit30 group selector 0.
 *   - AMEBA_PWM_IRQ: RTIM_TimeBaseInit() takes it even though this polling
 *     driver registers no callback (NULL); TIM8 is IRQ 18.
 */

#define AMEBA_PWM_TIMER_IDX       8            /* TIM8 (IS_TIM_PWM_TIM)     */
#define AMEBA_PWM_NCHAN           8            /* CCR0..CCR7 (PWM_CHAN_MAX) */
#define AMEBA_PWM_BASE            0x41100000ul /* NON-secure TIM8 base      */
#define AMEBA_PWM_CLKFREQ         40000000ul   /* TIM8 input clock (40 MHz) */
#define AMEBA_PWM_IRQ             18           /* TIMER8_IRQ                */

/* Crossbar pad-mux function code per channel, indexed by channel number
 * minus one (channel n uses AMEBA_PWM_PINMUX_FIDS[n - 1]).  On amebadplus
 * these are the contiguous PINMUX_FUNCTION_PWM0..PWM7 (52..59).
 */

#define AMEBA_PWM_PINMUX_FIDS     { 52, 53, 54, 55, 56, 57, 58, 59 }

/* APBPeriph_PWM0 (function) and APBPeriph_PWM0_CLOCK masks.  Equal on this
 * chip; the group selector (bit30) is 0 for the PWM block on amebadplus.
 */

#define AMEBA_PWM_APBPERIPH       (((uint32_t)0 << 30) | ((uint32_t)1 << 23))
#define AMEBA_PWM_APBPERIPH_CLK   (((uint32_t)0 << 30) | ((uint32_t)1 << 23))

#endif /* __ARCH_ARM_SRC_RTL8721DX_AMEBA_PWM_CHIP_H */
