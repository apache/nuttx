/****************************************************************************
 * arch/arm/src/rtl8730e/ameba_gpio_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8730E_AMEBA_GPIO_CHIP_H
#define __ARCH_ARM_SRC_RTL8730E_AMEBA_GPIO_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions (register map for local helpers)
 ****************************************************************************/

#define RTL8730E_GPIO_BASE   UINT32_C(0x4200D000)
#define RTL8730E_GPIO_STRIDE UINT32_C(0x400)
#define RTL8730E_GPIO_INT_STATUS UINT32_C(0x40)
#define RTL8730E_GPIO_INT_EOI    UINT32_C(0x4C)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip GPIO parameters for the shared driver
 * (arch/arm/src/common/ameba/ameba_gpio.c).
 *
 * RTL8730E (CA32) exposes three GPIO ports: A, B, and C, each a 32-pin
 * bank with its own GIC SPI interrupt.  The CA32 IRQ numbers are the GIC
 * SPI numbers (9/10/11) plus RTL8730E_SPI_IRQ_BASE (32).
 */

/* Number of GPIO ports (banks) this chip exposes. */

#define AMEBA_GPIO_NPORTS      3

/* GIC SPI vectors for GPIOA/B/C on CA32, indexed by port number. */

#define AMEBA_GPIO_PORT_IRQS   { 41, 42, 43 }

/* RCC_PeriphClockCmd(periph, clock, state) takes two separate bit masks:
 *   periph - APBPeriph_GPIO:       bit 23, enables the GPIO function
 *   clock  - APBPeriph_GPIO_CLOCK: bit  8, enables the GPIO bus clock
 * On RTL8730E these are distinct; on the KM4-based Ameba parts they
 * happen to be the same value, so the shared driver passes one macro
 * for both.  Define AMEBA_APBPERIPH_GPIO_CLK to override the clock arg.
 */

#define AMEBA_APBPERIPH_GPIO     ((uint32_t)1 << 23)
#define AMEBA_APBPERIPH_GPIO_CLK ((uint32_t)1 <<  8)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/* GPIO_INTStatusGet / GPIO_INTStatusClearEdge are absent from the RTL8730E
 * ROM and fwlib; provide them here as inlines so no separate .c file is
 * needed (unlike the KM4-based parts where these symbols are in the ROM).
 */

static inline uint32_t GPIO_INTStatusGet(uint32_t port)
{
  volatile uint32_t *r =
    (volatile uint32_t *)(RTL8730E_GPIO_BASE + port * RTL8730E_GPIO_STRIDE +
                          RTL8730E_GPIO_INT_STATUS);
  return *r;
}

static inline void GPIO_INTStatusClearEdge(uint32_t port)
{
  volatile uint32_t *r =
    (volatile uint32_t *)(RTL8730E_GPIO_BASE + port * RTL8730E_GPIO_STRIDE +
                          RTL8730E_GPIO_INT_EOI);
  *r = UINT32_C(0xffffffff);
}

#endif /* __ARCH_ARM_SRC_RTL8730E_AMEBA_GPIO_CHIP_H */
