/****************************************************************************
 * arch/arm/src/rtl8721f/ameba_rtc_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8721F_AMEBA_RTC_CHIP_H
#define __ARCH_ARM_SRC_RTL8721F_AMEBA_RTC_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip RTC wiring for RTL8721F (amebagreen2).  The shared driver
 * (arch/arm/src/common/ameba/ameba_rtc.c) includes this header to learn the
 * RTC interrupt line, the peripheral-clock masks and the calendar base year.
 * A port to another Ameba chip supplies a same-named header on the chip
 * include path; the shared driver is never edited -- it reads only the
 * macros below, and it drives the RTC through the SDK fwlib API, which takes
 * no register base (it selects the secure/non-secure alias internally), so
 * no register address appears here at all.
 *
 * The RTC register model, structures and constants (year + day-of-year time,
 * RTC_Format_BIN, the alarm mask values, RTC_BASE_YEAR = 1900) are identical
 * on every current Ameba chip (verified against amebadplus/amebalite/
 * amebasmart/amebagreen2/RTL8720F fwlib ameba_rtc.h), so they live in the
 * shared driver.  The one thing that genuinely differs per chip is the RTC
 * interrupt vector, so only that is abstracted here:
 *
 *   chip         RTC_IRQ vector (SDK ameba_vector_table.h)
 *   -----------  -----------------------------------------
 *   amebadplus   46
 *   amebalite    53
 *   amebasmart   12
 *   amebagreen2  41   (this header: RTL8721F_IRQ_RTC)
 *   RTL8720F     33
 *
 * A new chip only edits this header: AMEBA_RTC_IRQ carries the NuttX IRQ
 * number of its RTC line -- see the ADC/PWM chip headers for the same
 * data-driven, never-computed pattern.
 */

#define AMEBA_RTC_IRQ             RTL8721F_IRQ_RTC

/* APBPeriph_RTC (function) and APBPeriph_RTC_CLOCK masks, from the SDK
 * sysreg_aon.h: bit group selector (3 << 30) plus the RTC enable bit
 * (1 << 3).  Both are equal on this chip.  The driver gates the RTC clock
 * with these before the first fwlib call.
 */

#define AMEBA_RTC_APBPERIPH       (((uint32_t)3 << 30) | ((uint32_t)1 << 3))
#define AMEBA_RTC_APBPERIPH_CLK   (((uint32_t)3 << 30) | ((uint32_t)1 << 3))

/* Calendar base year the fwlib RTC uses for RTC_TimeTypeDef.RTC_Year
 * (RTC_BASE_YEAR); the hardware stores year + day-of-year, not month/day.
 */

#define AMEBA_RTC_BASE_YEAR       1900

#endif /* __ARCH_ARM_SRC_RTL8721F_AMEBA_RTC_CHIP_H */
