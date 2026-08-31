/****************************************************************************
 * arch/arm/src/rtl8721f/ameba_timer_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8721F_AMEBA_TIMER_CHIP_H
#define __ARCH_ARM_SRC_RTL8721F_AMEBA_TIMER_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip general-purpose timer wiring for RTL8721F (amebagreen2).  The
 * shared driver (arch/arm/src/common/ameba/ameba_timer.c) includes this
 * header to learn which timer instances it may expose and, for each one, its
 * register base, input clock, RCC clock masks and interrupt line.  A port to
 * another Ameba chip supplies a same-named header on the chip include path;
 * the shared driver is never edited -- it reads only the macros below and
 * the instance table, computing the period from clkfreq with one formula.
 *
 * The Ameba SoC carries a bank of timers that all share the one RTIM
 * register layout and fwlib API, so the register access, the
 * microsecond<->tick conversion and the whole lower-half live in the shared
 * driver.  What differs per instance -- and per chip -- is only data: the
 * base address, the clock that feeds it, its RCC gate masks and its NuttX
 * IRQ number.  These sit in AMEBA_TIMER_CONFIG_TABLE so a new chip (or
 * a different instance choice) edits this header alone.
 *
 * This port exposes two of the 32-bit "basic" (LTIM) timers so a single
 * lower-half serves them uniformly (verified against the SoC hal_platform.h
 * / sysreg_lsys.h / vector table, not guessed):
 *
 *   instance  timer   base         clock       RCC gate         NuttX IRQ
 *   --------  ------   ----------   ---------   --------------   ----------
 *   0         TIM1     0x40819200   32.768 kHz  APBPeriph_LTIM1  IRQ_TIMER1
 *   1         TIM2     0x40819400   32.768 kHz  APBPeriph_LTIM2  IRQ_TIMER2
 *
 * TIM1 and TIM2 are two of the six "basic" (LTIM) timers clocked from the
 * 32.768 kHz LS clock: ~30.5 us per tick, up to ~36 h in one shot.  A full
 * 32-bit auto-reload lets them cover the whole microsecond API range, which
 * is why the driver exposes this bank rather than the "high" (HTIM) timers,
 * whose 16-bit auto-reload suits sub-millisecond fine timing only.  The
 * non-secure base alias (0x40xxxxxx) is used, matching the core this port
 * runs on.
 *
 * TIM0 is deliberately NOT exposed: the boot ROM claims it as the always-on
 * system timer (see the vendor ameba_delay.h), and DelayUs/DelayMs and the
 * SYSTIMER_* helpers all rely on it.  Reprogramming TIM0 would break every
 * SDK delay, so the
 * driver starts the basic bank at TIM1.
 *
 * The RCC gate masks are the fwlib APBPeriph_LTIM1 / APBPeriph_LTIM2 values
 * (function and clock masks are identical on this chip).  They are written
 * out here rather than pulled from <sysreg_lsys.h> to keep the vendor
 * headers out of the NuttX include world (same rule as the PWM chip header).
 */

/* RCC "function" and "clock" masks, bit30 group selector then bit index. */

#define AMEBA_TIMER_LTIM1_MASK    (((uint32_t)1 << 30) | ((uint32_t)1 << 13))
#define AMEBA_TIMER_LTIM2_MASK    (((uint32_t)1 << 30) | ((uint32_t)1 << 14))

/* Number of timer instances this chip exposes and the data table that
 * describes each one.  The shared driver defines struct ameba_timer_config_s
 * and instantiates this table; each row is
 * { base, periph, clk, clkfreq, irq, idx }.
 */

#define AMEBA_TIMER_NINSTANCES    2

#define AMEBA_TIMER_CONFIG_TABLE                                             \
{                                                                           \
  {                                                                         \
    0x40819200ul, AMEBA_TIMER_LTIM1_MASK, AMEBA_TIMER_LTIM1_MASK,           \
    32768ul, RTL8721F_IRQ_TIMER1, 1                                         \
  },                                                                        \
  {                                                                         \
    0x40819400ul, AMEBA_TIMER_LTIM2_MASK, AMEBA_TIMER_LTIM2_MASK,           \
    32768ul, RTL8721F_IRQ_TIMER2, 2                                         \
  },                                                                        \
}

#endif /* __ARCH_ARM_SRC_RTL8721F_AMEBA_TIMER_CHIP_H */
