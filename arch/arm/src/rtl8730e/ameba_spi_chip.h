/****************************************************************************
 * arch/arm/src/rtl8730e/ameba_spi_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8730E_AMEBA_SPI_CHIP_H
#define __ARCH_ARM_SRC_RTL8730E_AMEBA_SPI_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip SPI wiring for RTL8730E (amebasmart CA32).  The shared driver
 * (arch/arm/src/common/ameba/ameba_spi.c) includes this header to learn how
 * many SPI (DesignWare SSI) controllers the chip exposes and, for each, its
 * register base, peripheral-clock masks, pad-mux codes and IP clock.
 *
 * Sources: component/soc/amebasmart/fwlib/include/hal_platform.h (bases),
 * sysreg_lsys.h (APBPeriph masks, CKD_HBUS), sysreg_hsys.h (SPIx_MST),
 * ameba_pinmux.h (pad-mux code), fwlib/ram_hp/ameba_spi.c (SPI_DEV_TABLE)
 * and hal/src/spi_api.c (spi_get_ipclk()).
 */

#define AMEBA_NSPI                2

/* SPI register bases.  Both controllers are the HS (high-speed) SSI masters
 * on APB4 in the HS_AHB clock domain.
 *
 * These MUST be the NON-secure aliases.  Besides the usual non-secure world
 * access rule, SSI_SetRole() identifies the controller by comparing the
 * pointer it is given against the fwlib's own SPI0_DEV / SPI1_DEV, which are
 * the non-secure 0x400e8000 / 0x400e9000 (see SPI_DEV_TABLE in
 * fwlib/ram_hp/ameba_spi.c).  Passing the secure aliases SPI0_REG_BASE_S /
 * SPI1_REG_BASE_S (0x500e8000 / 0x500e9000) would make that comparison fail
 * for SPI0 and set the SPI1 master bit instead, leaving SPI0 a slave.
 */

#define AMEBA_SPI_BASES           { 0x400e8000ul, 0x400e9000ul }

/* APBPeriph_SPIx (function) and APBPeriph_SPIx_CLOCK masks.
 *
 * Unlike the other Ameba peripherals, these two are NOT equal on amebasmart:
 * the function enables sit one bit above the clock enables (SPI0 function
 * bit9 / clock bit8, SPI1 function bit10 / clock bit9), so the two lists
 * really do carry different values and must not be collapsed into one.
 * The group selector bit30 is 1 for the SPI block here (it is 0 for the
 * amebasmart I2C block and for the amebadplus SPI block).
 */

#define AMEBA_SPI_APBPERIPH       \
        { (((uint32_t)1 << 30) | ((uint32_t)1 << 9)), \
          (((uint32_t)1 << 30) | ((uint32_t)1 << 10)) }

#define AMEBA_SPI_APBPERIPH_CLK   \
        { (((uint32_t)1 << 30) | ((uint32_t)1 << 8)), \
          (((uint32_t)1 << 30) | ((uint32_t)1 << 9)) }

/* amebasmart has a single generic PINMUX_FUNCTION_SPI (3) shared by every
 * SPI pad and by both controllers; there are no per-signal CLK/MOSI/MISO/CS
 * crossbar codes as on amebadplus, so every list carries the same value.
 * (PINMUX_FUNCTION_SPIF (6) is the flash SPI controller, not these.)
 *
 * One function id does not mean the pads are a free crossbar: as with the
 * amebasmart I2C block, each pad is hard-wired to one controller per the
 * chip pin-mux table (SPI0 on PB18-PB21; SPI1 on either PB25-PB28 or
 * PA9-PA12), so the board table must pick pads from a set belonging to the
 * bus it is registering.
 */

#define AMEBA_SPI_CLKFID          { 3, 3 }  /* PINMUX_FUNCTION_SPI */
#define AMEBA_SPI_MOSIFID         { 3, 3 }  /* PINMUX_FUNCTION_SPI */
#define AMEBA_SPI_MISOFID         { 3, 3 }  /* PINMUX_FUNCTION_SPI */
#define AMEBA_SPI_CSFID           { 3, 3 }  /* PINMUX_FUNCTION_SPI */

/* SSI peripheral clock ip_clk in Hz: the frequency the SSI baud divider
 * divides down to produce SCLK.  Both controllers sit on HS_AHB, which is
 * NP_PLL divided by REG_LSYS_CKD_GRP0.CKD_HBUS + 1.
 *
 * NP_PLL is itself a PLL setting rather than a constant, so this reads it
 * out of the PLL block the same way the SDK's spi_get_ipclk() does (see
 * hal/src/spi_api.c) instead of tabulating a nominal rate: 40 MHz crystal
 * times the N divider PLL_NPPLL_CTRL1.NPLL_DIVN_SDM + 2.  Both the register
 * addresses and the field positions are chip-specific, so the whole
 * computation lives here rather than in the shared driver.
 */

#define AMEBA_SPI_XTAL_HZ         40000000ul

/* PLL_NPPLL_CTRL1: PLL_BASE (0x42008840) + 0x004.  NPLL_DIVN_SDM is bits
 * 5-10 and the effective N divider is that value + 2.
 */

#define AMEBA_SPI_NPPLL_CTRL1     (0x42008840ul + 0x004)
#define AMEBA_SPI_DIVN_SHIFT      5
#define AMEBA_SPI_DIVN_MASK       0x3fu

/* REG_LSYS_CKD_GRP0: SYSTEM_CTRL_BASE_LP (0x42008000) + 0x228.  CKD_HBUS is
 * bits 12-15 and the effective divider is that value + 1.
 */

#define AMEBA_SPI_CKD_GRP0        (0x42008000ul + 0x228)
#define AMEBA_SPI_CKD_HBUS_SHIFT  12
#define AMEBA_SPI_CKD_HBUS_MASK   0xfu

static inline uint32_t ameba_spi_chip_ipclk(void)
{
  uint32_t divn;
  uint32_t div;

  divn = ((*(volatile uint32_t *)AMEBA_SPI_NPPLL_CTRL1) >>
          AMEBA_SPI_DIVN_SHIFT) & AMEBA_SPI_DIVN_MASK;
  div  = ((*(volatile uint32_t *)AMEBA_SPI_CKD_GRP0) >>
          AMEBA_SPI_CKD_HBUS_SHIFT) & AMEBA_SPI_CKD_HBUS_MASK;

  return (AMEBA_SPI_XTAL_HZ * (divn + 2)) / (div + 1);
}

#define AMEBA_SPI_IPCLK()         ameba_spi_chip_ipclk()

#endif /* __ARCH_ARM_SRC_RTL8730E_AMEBA_SPI_CHIP_H */
