/****************************************************************************
 * arch/arm/src/rtl8721dx/ameba_spi_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8721DX_AMEBA_SPI_CHIP_H
#define __ARCH_ARM_SRC_RTL8721DX_AMEBA_SPI_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip SPI wiring for RTL8721DX (amebadplus).  The shared driver
 * (arch/arm/src/common/ameba/ameba_spi.c) includes this header to learn how
 * many SPI (DesignWare SSI) controllers the chip exposes and, for each, its
 * register base, peripheral-clock masks and crossbar pad-mux codes.
 *
 * Contract for the other Ameba chips (amebalite / amebasmart / amebagreen2 /
 * RTL8720F): supply a same-named header on the chip include path with the
 * macros below.  What differs per chip, from the fwlib audit:
 *
 *   1. Controller count and bases: amebadplus exposes two high-speed SSI
 *      masters, SPI0 and SPI1, in the PERI_HCLK domain.  The bases below are
 *      the NON-secure peripheral aliases (0x4012xxxx); the secure aliases
 *      (0x5012xxxx) must not be used from the non-secure world.
 *
 *   2. APBPeriph "function" and "clock" masks are two separate lists because
 *      RCC_PeriphClockCmd() takes them as distinct arguments.  They are
 *      equal on this chip.  NOTE the group selector bit30 is 0 here (unlike
 *      the Ameba UART/I2C blocks where bit30 is 1), so each chip must supply
 *      its own values.
 *
 *   3. Pad mux: SPI0 (master) shares one generic PINMUX_FUNCTION_SPI code
 *      (8) on all four signals, whereas SPI1 uses per-signal codes
 *      (CLK/MOSI/MISO/CS).  The codes are therefore supplied as one list per
 *      signal, indexed by controller.  The chip-select entries are only used
 *      when hardware CS is selected; this driver drives CS as a plain GPIO
 *      (software CS), so the CS pad-mux code is informational.
 */

#define AMEBA_NSPI                2

/* NON-secure SPI register bases (SPI0_REG_BASE / SPI1_REG_BASE). */

#define AMEBA_SPI_BASES           { 0x40124000ul, 0x40125000ul }

/* APBPeriph_SPIx (function) and APBPeriph_SPIx_CLOCK masks.  Equal on this
 * chip; kept as two lists so chips where they differ can supply both.  The
 * group selector (bit30) is 0 for the SPI block on amebadplus.
 */

#define AMEBA_SPI_APBPERIPH       \
        { (((uint32_t)0 << 30) | ((uint32_t)1 << 14)), \
          (((uint32_t)0 << 30) | ((uint32_t)1 << 15)) }

#define AMEBA_SPI_APBPERIPH_CLK   \
        { (((uint32_t)0 << 30) | ((uint32_t)1 << 14)), \
          (((uint32_t)0 << 30) | ((uint32_t)1 << 15)) }

/* Crossbar pad-mux function codes, one list per signal, indexed by
 * controller.  SPI0 uses the generic PINMUX_FUNCTION_SPI (8) on every
 * signal; SPI1 uses its per-signal codes CLK=29 / MISO=30 / MOSI=31 / CS=32.
 */

#define AMEBA_SPI_CLKFID          { 8, 29 }  /* SPI0 generic, SPI1_CLK  */
#define AMEBA_SPI_MOSIFID         { 8, 31 }  /* SPI0 generic, SPI1_MOSI */
#define AMEBA_SPI_MISOFID         { 8, 30 }  /* SPI0 generic, SPI1_MISO */
#define AMEBA_SPI_CSFID           { 8, 32 }  /* SPI0 generic, SPI1_CS   */

/* SSI peripheral clock: ip_clk = PLL_ClkGet() / (HPERI divider + 1), the
 * frequency the SPI baud divider divides down to make SCLK.  On amebadplus
 * the HPERI divider field is bits[10:8] of REG_LSYS_CKD_GRP0 in the system-
 * control block (non-secure alias 0x41008000 + 0x0220).  BOTH the register
 * address and the field position are chip-specific -- e.g. on amebalite that
 * same bit field is not HPERI -- so the whole computation lives here rather
 * than in the shared driver. PLL_ClkGet() is the amebadplus fwlib PLL query;
 * each chip header declares the clock-source query its AMEBA_SPI_IPCLK()
 * uses (the shared driver includes this header before the macro expands).
 */

extern uint32_t PLL_ClkGet(void);

#define AMEBA_SPI_CKD_GRP0        0x41008220ul
#define AMEBA_SPI_IPCLK() \
        (PLL_ClkGet() / \
         ((((*(volatile uint32_t *)AMEBA_SPI_CKD_GRP0) >> 8) & 0x7) + 1))

#endif /* __ARCH_ARM_SRC_RTL8721DX_AMEBA_SPI_CHIP_H */
