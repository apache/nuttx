/****************************************************************************
 * arch/arm/src/rtl8720f/ameba_spi_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8720F_AMEBA_SPI_CHIP_H
#define __ARCH_ARM_SRC_RTL8720F_AMEBA_SPI_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip SPI wiring for RTL8720F.  The shared driver
 * (arch/arm/src/common/ameba/ameba_spi.c) includes this header to learn how
 * many SPI (DesignWare SSI) controllers the chip exposes and, for each, its
 * register base, peripheral-clock masks and crossbar pad-mux codes.  See the
 * amebadplus (rtl8721dx) header for the field-by-field contract; only the
 * RTL8720F-specific values (from the RTL8720F fwlib) are noted here:
 *
 *   1. Two high-speed SSI masters, SPI0 and SPI1, in the PERI_HCLK domain.
 *      The bases below are the NON-secure aliases (SPI0/1_REG_BASE in
 *      hal_platform.h, 0x401C_1000 / 0x401C_2000); the secure aliases
 *      (0x501C_xxxx) must not be used from the non-secure world.
 *
 *   2. APBPeriph_SPIx (function) and APBPeriph_SPIx_CLOCK masks live in
 *      REG_LSYS_FEN_GRP0 / REG_LSYS_CKE_GRP0 (group selector bit30 == 0),
 *      SPI0 at bit14 and SPI1 at bit15 -- identical to amebadplus.
 *
 *   3. Pad mux: RTL8720F has NO generic PINMUX_FUNCTION_SPI code, so every
 *      signal uses its per-signal code from ameba_pinmux.h
 *      (SPI0 CLK/MISO/MOSI/CS = 37/38/39/40, SPI1 = 41/42/43/44).  The CS
 *      entries are informational: this driver drives CS as a plain GPIO.
 */

#define AMEBA_NSPI                2

/* NON-secure SPI register bases (SPI0_REG_BASE / SPI1_REG_BASE). */

#define AMEBA_SPI_BASES           { 0x401c1000ul, 0x401c2000ul }

/* APBPeriph_SPIx (function) and APBPeriph_SPIx_CLOCK masks.  Equal on this
 * chip; kept as two lists so chips where they differ can supply both.  The
 * group selector (bit30) is 0 for the SPI block on RTL8720F.
 */

#define AMEBA_SPI_APBPERIPH       \
        { (((uint32_t)0 << 30) | ((uint32_t)1 << 14)), \
          (((uint32_t)0 << 30) | ((uint32_t)1 << 15)) }

#define AMEBA_SPI_APBPERIPH_CLK   \
        { (((uint32_t)0 << 30) | ((uint32_t)1 << 14)), \
          (((uint32_t)0 << 30) | ((uint32_t)1 << 15)) }

/* Crossbar pad-mux function codes, one list per signal, indexed by
 * controller.  RTL8720F uses per-signal codes for both controllers:
 * SPI0 CLK=37 / MISO=38 / MOSI=39 / CS=40, SPI1 CLK=41 / MISO=42 /
 * MOSI=43 / CS=44.
 */

#define AMEBA_SPI_CLKFID          { 37, 41 }  /* SPI0_CLK,  SPI1_CLK  */
#define AMEBA_SPI_MOSIFID         { 39, 43 }  /* SPI0_MOSI, SPI1_MOSI */
#define AMEBA_SPI_MISOFID         { 38, 42 }  /* SPI0_MISO, SPI1_MISO */
#define AMEBA_SPI_CSFID           { 40, 44 }  /* SPI0_CS,   SPI1_CS   */

/* SSI peripheral clock: ip_clk = SYS_PLL_ClkGet() / (HPERI divider + 1), the
 * frequency the SPI baud divider divides down to make SCLK.  On RTL8720F the
 * HPERI divider is bits[7:4] of REG_LSYS_CKD_SYS_PLL_GRP0 in the system-
 * control block (non-secure alias SYSTEM_CTRL_BASE 0x40801000 + 0x0250),
 * used when the HPERI source is SYS_PLL (the SDK default for the 100 MHz SPI
 * bus clock).  BOTH the register address and the field position are
 * chip-specific -- amebadplus uses PLL_ClkGet() with bits[10:8] of a
 * different register -- so the whole computation lives here rather than in
 * the shared driver.
 */

extern uint32_t SYS_PLL_ClkGet(void);

#define AMEBA_SPI_CKD_SYS_PLL     0x40801250ul
#define AMEBA_SPI_IPCLK() \
        (SYS_PLL_ClkGet() / \
         ((((*(volatile uint32_t *)AMEBA_SPI_CKD_SYS_PLL) >> 4) & 0xf) + 1))

#endif /* __ARCH_ARM_SRC_RTL8720F_AMEBA_SPI_CHIP_H */
