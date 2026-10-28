/****************************************************************************
 * arch/arm/src/rtl8730e/ameba_rtc_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8730E_AMEBA_RTC_CHIP_H
#define __ARCH_ARM_SRC_RTL8730E_AMEBA_RTC_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip RTC wiring for RTL8730E (AmebaSmart, CA32).  The shared driver
 * (arch/arm/src/common/ameba/ameba_rtc.c) reads only the macros below.  The
 * fwlib RTC layer is RAM-side here (the AP ROM has no RTC_*) and addresses
 * the always-on block at RTC_BASE (0x4200e000) itself, so no register base
 * appears in this header.
 *
 * The RTC interrupt is GIC SPI 12 (SDK ameba_vector_table.h, CA32 section),
 * i.e. NuttX IRQ 12 + 32.
 */

#define AMEBA_RTC_IRQ             RTL8730E_IRQ_RTC

/* APBPeriph_RTC (function, bit 19) and APBPeriph_RTC_CLOCK (bit 3) from the
 * SDK sysreg_aon.h; unlike RTL8721Dx the two masks differ here.
 */

#define AMEBA_RTC_APBPERIPH       (((uint32_t)3 << 30) | ((uint32_t)1 << 19))
#define AMEBA_RTC_APBPERIPH_CLK   (((uint32_t)3 << 30) | ((uint32_t)1 << 3))

/* Calendar base year of RTC_TimeTypeDef.RTC_Year (RTC_BASE_YEAR). */

#define AMEBA_RTC_BASE_YEAR       1900

#endif /* __ARCH_ARM_SRC_RTL8730E_AMEBA_RTC_CHIP_H */
