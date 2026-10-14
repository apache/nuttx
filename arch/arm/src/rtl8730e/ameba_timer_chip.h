/****************************************************************************
 * arch/arm/src/rtl8730e/ameba_timer_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8730E_AMEBA_TIMER_CHIP_H
#define __ARCH_ARM_SRC_RTL8730E_AMEBA_TIMER_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip general-purpose timer wiring for RTL8730E (amebasmart, CA32).
 * The shared driver (arch/arm/src/common/ameba/ameba_timer.c) includes this
 * header to learn which timer instances it may expose and, for each one, its
 * register base, input clock, RCC clock masks and interrupt line.  The
 * shared driver is never edited: it reads only the macros below and the
 * instance table, computing the period from clkfreq with one formula.
 *
 * amebasmart carries fourteen timers (TIM0..TIM13) sharing the one RTIM
 * register layout and fwlib API, grouped by the SoC as follows (see the
 * APBPeriph_TIMx comment in sysreg_lsys.h): TIM0..TIM7 are the 32-bit LP
 * "basic" timers, TIM8 is the PWM timer, TIM9 the input-pulse timer and
 * TIM10..TIM13 the HP basic timers.  Values below were read out of the SoC
 * hal_platform.h, sysreg_lsys.h and the CA32 section of
 * ameba_vector_table.h, not guessed.
 *
 * This port exposes two of the fourteen, both from the 32-bit LP basic bank
 * so a single lower-half serves them uniformly:
 *
 *   instance  timer   base         clock       RCC gate       NuttX IRQ
 *   --------  ------   ----------   ---------   ------------   -------------
 *   0         TIM2     0x4200b400   32.768 kHz  APBPeriph_TIM2  IRQ_TIMER2
 *   1         TIM3     0x4200b600   32.768 kHz  APBPeriph_TIM3  IRQ_TIMER3
 *
 * The LP basic timers are clocked from the 32.768 kHz LS clock (the vendor
 * us_ticker.c states GTIMER_CLK_HZ = 32768 for this bank): ~30.5 us per
 * tick, and a full 32-bit auto-reload -- "TIM_Period is 16bits for TIM8-9,
 * and 32bits for TIM0-TIM7" per the fwlib RTIM_TimeBaseInitTypeDef comment
 * -- gives ~36 h in one shot, covering the whole microsecond timer API
 * range.  That is why this bank is exposed rather than TIM8..TIM13: TIM8/
 * TIM9 are only 16-bit (max ~1.638 ms at 40 MHz) and are already claimed by
 * the PWM and input-capture blocks, TIM8 being what CONFIG_AMEBA_PWM drives.
 *
 * TIM0 and TIM1 are deliberately NOT exposed, both being claimed by vendor
 * code this port keeps working:
 *
 *   - TIM0 is the always-on system timer.  The vendor ameba_delay.h says
 *     "TIM0 is used as systimer, so TIM0 can not be used for other purpose",
 *     and DelayUs/DelayMs/SYSTIMER_* all rely on it.
 *   - TIM1 is the mbed-HAL application timer (us_ticker.c APP_TIM_ID = 1)
 *     and doubles as the serial_api.c receive timeout timer (UART_TIMER_ID
 *     = 1, "TIM1").
 *
 * So the basic bank is exposed starting at TIM2, unlike the amebadplus port
 * which starts at TIM1 because only TIM0 is reserved there.
 */

/* RCC "function" and "clock" masks.  Note the group selector: the amebasmart
 * timer block is group 2 (bit30 = 2) with the per-timer bit at 16 + n, so
 * TIM2 is bit18 and TIM3 is bit19 (APBPeriph_TIM2 / APBPeriph_TIM3 and their
 * _CLOCK twins, which are equal on this chip).  The SPI block here is group
 * 1 and the I2C block group 0, so each peripheral -- not just each chip --
 * must supply its own values.  They are written out here rather than pulled
 * from <sysreg_lsys.h> to keep the vendor headers out of the NuttX include
 * world (same rule as the PWM chip header).
 */

#define AMEBA_TIMER_TIM2_MASK     (((uint32_t)2 << 30) | ((uint32_t)1 << 18))
#define AMEBA_TIMER_TIM3_MASK     (((uint32_t)2 << 30) | ((uint32_t)1 << 19))

/* Number of timer instances this chip exposes and the data table that
 * describes each one.  The shared driver defines struct
 * ameba_timer_config_s and instantiates this table; each row is
 * { base, periph, clk, clkfreq, irq, idx }.
 *
 * The bases are the non-secure LP_APB aliases (TIMER2_REG_BASE /
 * TIMER3_REG_BASE); the secure aliases at 0x5200b400 / 0x5200b600 must not
 * be used from the non-secure world.
 */

#define AMEBA_TIMER_NINSTANCES    2

#define AMEBA_TIMER_CONFIG_TABLE                                             \
{                                                                            \
  {                                                                          \
    0x4200b400ul, AMEBA_TIMER_TIM2_MASK, AMEBA_TIMER_TIM2_MASK,              \
    32768ul, RTL8730E_IRQ_TIMER2, 2                                          \
  },                                                                         \
  {                                                                          \
    0x4200b600ul, AMEBA_TIMER_TIM3_MASK, AMEBA_TIMER_TIM3_MASK,              \
    32768ul, RTL8730E_IRQ_TIMER3, 3                                          \
  },                                                                         \
}

#endif /* __ARCH_ARM_SRC_RTL8730E_AMEBA_TIMER_CHIP_H */
