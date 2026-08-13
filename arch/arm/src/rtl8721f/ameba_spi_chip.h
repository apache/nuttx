/****************************************************************************
 * arch/arm/src/rtl8721f/ameba_spi_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8721F_AMEBA_SPI_CHIP_H
#define __ARCH_ARM_SRC_RTL8721F_AMEBA_SPI_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip SPI wiring for RTL8721F (amebagreen2).  The shared driver
 * (arch/arm/src/common/ameba/ameba_spi.c) includes this header to learn how
 * many SPI (DesignWare SSI) controllers the chip exposes and, for each, its
 * register base, peripheral-clock masks and crossbar pad-mux codes.  See the
 * amebadplus (rtl8721dx) header for the field-by-field contract; only the
 * RTL8721F-specific values (from the amebagreen2 fwlib) are noted here:
 *
 *   1. Two high-speed SSI masters, SPI0 and SPI1, in the PERI_HCLK domain.
 *      The bases below are the NON-secure aliases (SPI0/1_REG_BASE in
 *      hal_platform.h, 0x4012_1000 / 0x4012_2000); the secure aliases
 *      (0x5012_xxxx) must not be used from the non-secure world.
 *
 *   2. APBPeriph_SPIx (function) and APBPeriph_SPIx_CLOCK masks live in
 *      REG_LSYS_FEN_GRP0 / REG_LSYS_CKE_GRP0 (group selector bit30 == 0),
 *      SPI0 at bit14 and SPI1 at bit15.
 *
 *   3. Pad mux: amebagreen2 has a generic PINMUX_FUNCTION_SPIx code (9/10)
 *      but the shared driver programs each signal individually, so the
 *      per-signal codes from ameba_pinmux.h are used (SPI0 CLK/MISO/MOSI/CS
 *      = 75/76/77/78, SPI1 = 79/80/81/82).  The CS entries are
 *      informational: this driver drives CS as a plain GPIO.
 */

#define AMEBA_NSPI                2

/* NON-secure SPI register bases (SPI0_REG_BASE / SPI1_REG_BASE). */

#define AMEBA_SPI_BASES           { 0x40121000ul, 0x40122000ul }

/* APBPeriph_SPIx (function) and APBPeriph_SPIx_CLOCK masks.  Equal on this
 * chip; kept as two lists so chips where they differ can supply both.  The
 * group selector (bit30) is 0 for the SPI block on amebagreen2.
 */

#define AMEBA_SPI_APBPERIPH       \
        { (((uint32_t)0 << 30) | ((uint32_t)1 << 14)), \
          (((uint32_t)0 << 30) | ((uint32_t)1 << 15)) }

#define AMEBA_SPI_APBPERIPH_CLK   \
        { (((uint32_t)0 << 30) | ((uint32_t)1 << 14)), \
          (((uint32_t)0 << 30) | ((uint32_t)1 << 15)) }

/* Crossbar pad-mux function codes, one list per signal, indexed by
 * controller.  amebagreen2 per-signal codes: SPI0 CLK=75 / MISO=76 /
 * MOSI=77 / CS=78, SPI1 CLK=79 / MISO=80 / MOSI=81 / CS=82.
 */

#define AMEBA_SPI_CLKFID          { 75, 79 }  /* SPI0_CLK,  SPI1_CLK  */
#define AMEBA_SPI_MOSIFID         { 77, 81 }  /* SPI0_MOSI, SPI1_MOSI */
#define AMEBA_SPI_MISOFID         { 76, 80 }  /* SPI0_MISO, SPI1_MISO */
#define AMEBA_SPI_CSFID           { 78, 82 }  /* SPI0_CS,   SPI1_CS   */

/* SSI peripheral clock: ip_clk is the PERI_HCLK-domain clock the SPI baud
 * divider divides down to make SCLK.  On amebagreen2 the fwlib exposes this
 * domain directly through HPERI_ClkGet() (it resolves the HPERI source PLL
 * and divider from the RRAM clock-info backup), so the whole computation is
 * a single ROM call here rather than register poking as on the other ICs.
 */

extern uint32_t HPERI_ClkGet(void);

#define AMEBA_SPI_IPCLK()         (HPERI_ClkGet())

#endif /* __ARCH_ARM_SRC_RTL8721F_AMEBA_SPI_CHIP_H */
