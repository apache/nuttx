/****************************************************************************
 * arch/mips/include/jz4780/chip.h
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

#ifndef __ARCH_MIPS_INCLUDE_JZ4780_CHIP_H
#define __ARCH_MIPS_INCLUDE_JZ4780_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CKPCR               0xB0003040
#define CKPCR_CK32CTL_RTCLK (3 << 1)

#define JZINTC_BASE         0xB0001000

#define ICMSR0              (JZINTC_BASE + 0x08)
#define ICMCR0              (JZINTC_BASE + 0x0C)
#define ICPR0               (JZINTC_BASE + 0x10)
#define ICMSR1              (JZINTC_BASE + 0x28)
#define ICMCR1              (JZINTC_BASE + 0x2C)
#define ICPR1               (JZINTC_BASE + 0x30)

#define JZTMR_BASE          0xB0002000

#define WD_TDR              (JZTMR_BASE + 0x00) // u16: Watchdog Timer Data
#define WD_TCER             (JZTMR_BASE + 0x04) // u8 : Watchdog Counter Enable
#define WD_TCNT             (JZTMR_BASE + 0x08) // u16: Watchdog Timer Counter
#define WD_TCSR             (JZTMR_BASE + 0x0C) // u16: Watchdog Timer Control

#define TCSR_PRESCALE(x)    ((x) << 3)
#define TCSR_DIV_16         2

#define TCSR_EXT_EN         (1 << 2)
#define TCSR_RTC_EN         (1 << 1)
#define TCSR_PCK_EN         (1 << 0)

#define TCER_TCEN           (1 << 0)

#define OSTDR               (JZTMR_BASE + 0xe0)
#define OSTCNTL             (JZTMR_BASE + 0xe4)
#define OSTCNTH             (JZTMR_BASE + 0xe8)
#define OSTCSR              (JZTMR_BASE + 0xec)

#define TESR                (JZTMR_BASE + 0x14)
#  define TESR_OSTEN        (1 << 15)
#  define TESR_TCEN(n)      (1 << (n))

#define TMSR                (JZTMR_BASE + 0x34)
#define TMCR                (JZTMR_BASE + 0x38)
#  define TMCR_OSTMCL       (1 << 15)
#  define TMCR_FMCL5        (1 << 5)
#  define TMCR_FMCL(n)      (1 << (n))

#define TFR                 (JZTMR_BASE + 0x20)

#define TFCR                (JZTMR_BASE + 0x28)
#  define TFCR_OSTFCL       (1 << 15)
#  define TFCR_FFCL5        (1 << 5)
#  define TFCR_FFCL(n)      (1 << (n))

#define TCNT(n)             (JZTMR_BASE + 0x48 + (n)*0x10)
#define TCSR(n)             (JZTMR_BASE + 0x4C + (n)*0x10)
#define TDFR(n)             (JZTMR_BASE + 0x40 + (n)*0x10)

#define CLKGR0_REG          0xb0000020
#  define CLKGR0_OTG0       (1 << 2)
#  define CLKGR0_UHC        (1 << 24)
#  define CLKGR0_LCD        (1 << 28)
#  define CLKGR0_TVE        (1 << 27)

#define OPCR_REG            0xb0000024
#  define OPCR_SPENDN1      (1 << 6)

#define CLKGR1_REG          0xb0000028
#  define CLKGR1_HDMI       (1 << 9)

#define USBPCR_REG          0xb000003c
#  define USBPCR_POR        (1 << 22)

#define USBPCR1_REG         0xb0000048
#  define REFCLK_DIV_MSK    0xfcffffff  /* clears bits 25:24 */
#  define REFCLK_DIV_48MHZ  0x02000000  /* bits 25:24 */
#  define WORD_IF_16BIT     0x000c0000  /* bits 19:18 */

#define LP1CDR_REG          0xb0000054
#  define LPCDR_VAL         0x15
#  define LPCS_VPLL         0X80000000
#  define CE_LCD            (1 << 28)
#  define LCD_BUSY          (1 << 27)

#define HDMICDR_REG         0xb000008c

#define SRBC_REG            0xb00000c4
#  define SRBC_UHC_SR       (1 << 14)

#define TICKS_PER_MS        (48000/16)

#define CHIP_JZ4780         1
#define CHIP_NPORTS         6  /* 6 ports (A-F) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: jz_physregaddr
 *
 * Description:
 *   Give the virtual address of a register, return the physical address of
 *   the register
 *
 ****************************************************************************/

uintptr_t jz_physregaddr(uintptr_t virtregaddr);

/****************************************************************************
 * Name: jz_physramaddr
 *
 * Description:
 *   Give the virtual address of a RAM memory location, return the physical
 *   address of that location.
 *
 ****************************************************************************/

uintptr_t jz_physramaddr(uintptr_t vramaddr);

/****************************************************************************
 * Name: jz_virtramaddr
 *
 * Description:
 *   Give the physical address of a RAM memory location, return the virtual
 *   address of that location.
 *
 ****************************************************************************/

uintptr_t jz_virtramaddr(uintptr_t physramaddr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_INCLUDE_JZ4780_CHIP_H */
