/****************************************************************************
 * arch/arm/src/tms570/hardware/tms570_sys2.h
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

/* References:
 * TMS570LS04x/03x 16/32-Bit RISC Flash Microcontroller,
 * Technical Reference Manual, Texas Instruments,
 * Literature Number: SPNU517A, September 2013
 */

#ifndef __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_SYS2_H
#define __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_SYS2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/tms570_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define TMS570_SYS2_STCCLKDIV_OFFSET    0x0008  /* CPU Logic BIST Clock Divider */
#define TMS570_SYS2_CLKSLIP_OFFSET      0x0070  /* Clock Slip Register */
#define TMS570_SYS2_EFC_CTLREG_OFFSET   0x00ec  /* EFUSE Controller Control Register */
#define TMS570_SYS2_DIEDL_REG0_OFFSET   0x00f0  /* Die Identification Register Lower Word */
#define TMS570_SYS2_DIEDH_REG1_OFFSET   0x00f4  /* Die Identification Register Upper Word */
#define TMS570_SYS2_DIEDL_REG2_OFFSET   0x00f8  /* Die Identification Register Lower Word */
#define TMS570_SYS2_DIEDH_REG3_OFFSET   0x00fc  /* Die Identification Register Upper Word */
#define TMS570_SYS2_CLK2CNTRL_OFFSET    0x003c
#define TMS570_SYS2_VCLKACON1_OFFSET    0x0040

/* Register Addresses *******************************************************/

#define TMS570_SYS2_STCCLKDIV           (TMS570_SYS2_BASE+TMS570_SYS2_STCCLKDIV_OFFSET)
#define TMS570_SYS2_CLKSLIP             (TMS570_SYS2_BASE+TMS570_SYS2_CLKSLIP_OFFSET)
#define TMS570_SYS2_EFC_CTLREG          (TMS570_SYS2_BASE+TMS570_SYS2_EFC_CTLREG_OFFSET)
#define TMS570_SYS2_DIEDL_REG0          (TMS570_SYS2_BASE+TMS570_SYS2_DIEDL_REG0_OFFSET)
#define TMS570_SYS2_DIEDH_REG1          (TMS570_SYS2_BASE+TMS570_SYS2_DIEDH_REG1_OFFSET)
#define TMS570_SYS2_DIEDL_REG2          (TMS570_SYS2_BASE+TMS570_SYS2_DIEDL_REG2_OFFSET)
#define TMS570_SYS2_DIEDH_REG3          (TMS570_SYS2_BASE+TMS570_SYS2_DIEDH_REG3_OFFSET)
#define TMS570_SYS2_CLK2CNTRL           (TMS570_SYS2_BASE+TMS570_SYS2_CLK2CNTRL_OFFSET)
#define TMS570_SYS2_VCLKACON1           (TMS570_SYS2_BASE+TMS570_SYS2_VCLKACON1_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* CPU Logic BIST Clock Divider */

#define SYS2_STCCLKDIV_

/* Clock Slip Register */

#define SYS2_CLKSLIP_

/* EFUSE Controller Control Register */

#define SYS2_EFC_CTLREG_

/* Die Identification Register Lower Word */

#define SYS2_DIEDL_REG0_

/* Die Identification Register Upper Word */

#define SYS2_DIEDH_REG1_

/* Die Identification Register Lower Word */

#define SYS2_DIEDL_REG2_

/* Die Identification Register Upper Word */

#define SYS2_DIEDH_REG3_

#if defined(CONFIG_ARCH_CHIP_TMS570LS3137ZWT)
#define SYS_VCLKASRC_VCLKA3R_SHIFT      (0)      /* Bits 0-3: Peripheral asynchronous clock2 source */
#define SYS_VCLKASRC_VCLKA3R_MASK       (15 << SYS_VCLKASRC_VCLKA3R_SHIFT)
#  define SYS_VCLKASRC_VCLKA3R(n)       ((uint32_t)(n) << SYS_VCLKASRC_VCLKA3R_SHIFT)
#  define SYS_VCLKASRC_VCLKA3R_OSC        SYS_VCLKASRC_VCLKA3R(0)
#  define SYS_VCLKASRC_VCLKA3R_PLL1       SYS_VCLKASRC_VCLKA3R(1)
#  define SYS_VCLKASRC_VCLKA3R_EXTERNAL1  SYS_VCLKASRC_VCLKA3R(3)
#  define SYS_VCLKASRC_VCLKA3R_LPOLOW     SYS_VCLKASRC_VCLKA3R(4)
#  define SYS_VCLKASRC_VCLKA3R_LPOHIGH    SYS_VCLKASRC_VCLKA3R(5)
#  define SYS_VCLKASRC_VCLKA3R_PLL2       SYS_VCLKASRC_VCLKA3R(6)
#  define SYS_VCLKASRC_VCLKA3R_EXTERNAL2  SYS_VCLKASRC_VCLKA3R(7)
#  define SYS_VCLKASRC_VCLKA3R_VCLK       SYS_VCLKASRC_VCLKA3R(9)

#define SYS_VCLKASRC_VCLKA3R_DIV_ENABLE  (1 << 4)
#define SYS_VCLKASRC_VCLKA3R_DIV_DISABLE (0 << 4)

#define SYS_VCLKASRC_VCLKA3R_DIV_SHIFT  (8)
#define SYS_VCLKASRC_VCLKA3R_DIV_MASK   (7 << SYS_VCLKASRC_VCLKA3R_SHIFT)
#  define SYS_VCLKASRC_VCLKA3R_DIV(n)   ((uint32_t)(n) << SYS_VCLKASRC_VCLKA3R_SHIFT)
#  define SYS_VCLKASRC_VCLKA3R_DIV1     SYS_VCLKASRC_VCLKA3R_DIV_MASK(0)
#  define SYS_VCLKASRC_VCLKA3R_DIV2     SYS_VCLKASRC_VCLKA3R_DIV_MASK(1)
#  define SYS_VCLKASRC_VCLKA3R_DIV3     SYS_VCLKASRC_VCLKA3R_DIV_MASK(2)
#  define SYS_VCLKASRC_VCLKA3R_DIV4     SYS_VCLKASRC_VCLKA3R_DIV_MASK(3)
#  define SYS_VCLKASRC_VCLKA3R_DIV5     SYS_VCLKASRC_VCLKA3R_DIV_MASK(4)
#  define SYS_VCLKASRC_VCLKA3R_DIV6     SYS_VCLKASRC_VCLKA3R_DIV_MASK(5)
#  define SYS_VCLKASRC_VCLKA3R_DIV7     SYS_VCLKASRC_VCLKA3R_DIV_MASK(6)
#  define SYS_VCLKASRC_VCLKA3R_DIV8     SYS_VCLKASRC_VCLKA3R_DIV_MASK(7)

#define SYS_VCLKASRC_VCLKA4S_SHIFT      (16)
#define SYS_VCLKASRC_VCLKA4S_MASK       (15 << SYS_VCLKASRC_VCLKA4S_SHIFT)
#  define SYS_VCLKASRC_VCLKA4S(n)       ((uint32_t)(n) << SYS_VCLKASRC_VCLKA4S_SHIFT)
#  define SYS_VCLKASRC_VCLKA4S_OSC        SYS_VCLKASRC_VCLKA4S(0)
#  define SYS_VCLKASRC_VCLKA4S_PLL1       SYS_VCLKASRC_VCLKA4S(1)
#  define SYS_VCLKASRC_VCLKA4S_EXTERNAL1  SYS_VCLKASRC_VCLKA4S(3)
#  define SYS_VCLKASRC_VCLKA4S_LPOLOW     SYS_VCLKASRC_VCLKA4S(4)
#  define SYS_VCLKASRC_VCLKA4S_LPOHIGH    SYS_VCLKASRC_VCLKA4S(5)
#  define SYS_VCLKASRC_VCLKA4S_PLL2       SYS_VCLKASRC_VCLKA4S(6)
#  define SYS_VCLKASRC_VCLKA4S_EXTERNAL2  SYS_VCLKASRC_VCLKA4S(7)
#  define SYS_VCLKASRC_VCLKA4S_VCLK       SYS_VCLKASRC_VCLKA4S(9)

#define SYS_VCLKASRC_VCLKA4S_DIV_ENABLE  (1 << 20)
#define SYS_VCLKASRC_VCLKA4S_DIV_DISABLE (0 << 20)

#define SYS_VCLKASRC_VCLKA4S_DIV_SHIFT  (24)      /* Bits 0-3: Peripheral asynchronous clock2 source */
#define SYS_VCLKASRC_VCLKA4S_DIV_MASK   (7 << SYS_VCLKASRC_VCLKA4S_DIV_SHIFT)
#  define SYS_VCLKASRC_VCLKA4S_DIV(n)   ((uint32_t)(n) << SYS_VCLKASRC_VCLKA4S_DIV_SHIFT)
#  define SYS_VCLKASRC_VCLKA4S_DIV1     SYS_VCLKASRC_VCLKA4S_DIV(0)
#  define SYS_VCLKASRC_VCLKA4S_DIV2     SYS_VCLKASRC_VCLKA4S_DIV(1)
#  define SYS_VCLKASRC_VCLKA4S_DIV3     SYS_VCLKASRC_VCLKA4S_DIV(2)
#  define SYS_VCLKASRC_VCLKA4S_DIV4     SYS_VCLKASRC_VCLKA4S_DIV(3)
#  define SYS_VCLKASRC_VCLKA4S_DIV5     SYS_VCLKASRC_VCLKA4S_DIV(4)
#  define SYS_VCLKASRC_VCLKA4S_DIV6     SYS_VCLKASRC_VCLKA4S_DIV(5)
#  define SYS_VCLKASRC_VCLKA4S_DIV7     SYS_VCLKASRC_VCLKA4S_DIV(6)
#  define SYS_VCLKASRC_VCLKA4S_DIV8     SYS_VCLKASRC_VCLKA4S_DIV(7)

#define SYS_CLK2CNTL_VCLK3R_SHIFT       (0)      /* Bits 16-19: VBUS clock ratio */
#define SYS_CLKC2NTL_VCLK3R_MASK        (15 << SYS_CLK2CNTL_VCLK3R_SHIFT)
#  define SYS_CLK2CNTL_VCLK3R_DIV1      (0 << SYS_CLK2CNTL_VCLK3R_SHIFT)
#  define SYS_CLK2CNTL_VCLK3R_DIV2      (1 << SYS_CLK2CNTL_VCLK3R_SHIFT)

#endif

#endif /* __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_SYS2_H */
