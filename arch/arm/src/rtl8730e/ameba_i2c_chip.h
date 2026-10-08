/****************************************************************************
 * arch/arm/src/rtl8730e/ameba_i2c_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8730E_AMEBA_I2C_CHIP_H
#define __ARCH_ARM_SRC_RTL8730E_AMEBA_I2C_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip I2C wiring for RTL8730E (amebasmart CA32).  The shared driver
 * (arch/arm/src/common/ameba/ameba_i2c.c) includes this header to learn how
 * many I2C controllers the chip exposes and, for each, its register base,
 * peripheral-clock masks, pad-mux codes and IP clock.  It also learns the
 * chip's I2C_InitTypeDef layout through AMEBA_I2C_HAS_DMA_FIELDS.
 *
 * Sources: component/soc/amebasmart/fwlib/include/hal_platform.h (bases),
 * sysreg_lsys.h (APBPeriph masks), ameba_pinmux.h (pad-mux code),
 * ameba_i2c.h (I2C_InitTypeDef) and hal/src/i2c_api.c (IP clocks).
 */

#define AMEBA_NI2C                3

/* I2C register bases.  I2C0 lives in the LP (low-power) domain on the
 * LP_APB bus; I2C1 and I2C2 are the HS (high-speed) controllers on APB4.
 * These are the NON-secure aliases (the secure aliases I2C1_REG_BASE_S /
 * I2C2_REG_BASE_S at 0x500ef000 / 0x500f0000 must not be used -- see the
 * note in ameba_i2c.c); the LP I2C0 has no secure alias at all.
 */

#define AMEBA_I2C_BASES           \
        { 0x4200f000ul, 0x400ef000ul, 0x400f0000ul }

/* APBPeriph_I2Cx (function) and APBPeriph_I2Cx_CLOCK masks.  amebasmart
 * encodes these in bit 25/26/27 with the group selector bit30 = 0, unlike
 * the KM4-based Ameba parts which use (bit30 | bit10/11).  Equal for the
 * function and clock arguments on this chip, but kept as two lists because
 * RCC_PeriphClockCmd() takes them as distinct arguments.
 */

#define AMEBA_I2C_APBPERIPH       \
        { ((uint32_t)1 << 25), \
          ((uint32_t)1 << 26), \
          ((uint32_t)1 << 27) }

#define AMEBA_I2C_APBPERIPH_CLK   \
        { ((uint32_t)1 << 25), \
          ((uint32_t)1 << 26), \
          ((uint32_t)1 << 27) }

/* amebasmart has a single generic PINMUX_FUNCTION_I2C (7) shared by every
 * I2C pad; there are no per-signal SCL/SDA crossbar codes as on the other
 * Ameba parts, so both lists carry the same value for every controller.
 */

#define AMEBA_I2C_SCLFID          { 7, 7, 7 }  /* PINMUX_FUNCTION_I2C */
#define AMEBA_I2C_SDAFID          { 7, 7, 7 }  /* PINMUX_FUNCTION_I2C */

/* I2C IP (reference) clock in Hz, per controller.
 *
 * amebasmart needs this where the other Ameba parts do not.  Elsewhere
 * I2C_StructInit() fills in a correct I2C0_1_IPCLK (XTAL_ClkGet() on
 * amebadplus/amebagreen2, PLL_GetHBUSClk() on amebalite, HPERI_ClkGet() on
 * RTL8720F); the amebasmart one hardcodes a 10 MHz placeholder, so without
 * an override I2C_SetSpeed() miscomputes the SCL high/low counts and the
 * bus runs at the wrong rate.  Chips that leave AMEBA_I2C_IPCLK undefined
 * keep the fwlib default and are unaffected.
 *
 * The two HS controllers live on HS_AHB, which is NP_PLL (800 MHz) divided
 * by REG_LSYS_CKD_GRP0.CKD_HBUS, so the rate is a board/PLL setting rather
 * than a constant: this macro reads the divider instead of assuming it.
 * (Measured on an EVB: CKD_HBUS = 8 -> div9 -> 88.9 MHz, while the SDK's
 * own I2CCLK_TABLE in hal/src/i2c_api.c claims a flat 100 MHz and the
 * register's reset value would give 80 MHz -- neither matches, which is
 * why this is computed and not tabulated.)  The LP-domain I2C0 is fed from
 * a fixed 20 MHz and is not affected by CKD_HBUS.
 */

#define AMEBA_I2C_NPPLL_HZ        800000000ul

/* REG_LSYS_CKD_GRP0: SYSTEM_CTRL_BASE_LP (0x42008000) + 0x228.  CKD_HBUS is
 * bits 12-15 and encodes 7 -> div8, 8 -> div9, otherwise div(value + 1).
 */

#define AMEBA_I2C_CKD_GRP0        (0x42008000ul + 0x228)
#define AMEBA_I2C_CKD_HBUS_SHIFT  12
#define AMEBA_I2C_CKD_HBUS_MASK   0xfu

/* Resolved at run time, per controller: AMEBA_I2C_IPCLK_FN(bus) is called
 * once when a bus is registered.  Controller 0 (LP domain) has a fixed
 * 20 MHz reference; controllers 1 and 2 follow the HS_AHB divider.
 */

static inline uint32_t ameba_i2c_ipclk(int bus)
{
  uint32_t div;

  if (bus == 0)
    {
      return 20000000ul;
    }

  div = (*(volatile uint32_t *)AMEBA_I2C_CKD_GRP0 >>
         AMEBA_I2C_CKD_HBUS_SHIFT) & AMEBA_I2C_CKD_HBUS_MASK;
  div = (div == 7) ? 8u : (div == 8) ? 9u : (div + 1u);

  return AMEBA_I2C_NPPLL_HZ / div;
}

#define AMEBA_I2C_IPCLK_FN(bus)   ameba_i2c_ipclk(bus)

/* The amebasmart I2C_InitTypeDef omits the DMA request-level fields. */

#undef AMEBA_I2C_HAS_DMA_FIELDS

#endif /* __ARCH_ARM_SRC_RTL8730E_AMEBA_I2C_CHIP_H */
