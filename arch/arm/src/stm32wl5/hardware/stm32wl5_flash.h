/****************************************************************************
 * arch/arm/src/stm32wl5/hardware/stm32wl5_flash.h
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

#ifndef __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_FLASH_H
#define __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flash size is known from the chip selection:
 *
 *  When CONFIG_STM32WL5_FLASH_OVERRIDE_DEFAULT is set the
 *  CONFIG_STM32WL5_FLASH_CONFIG_x selects the default FLASH size based
 *  on the chip part number. This value can be overridden with
 *  CONFIG_STM32WL5_FLASH_OVERRIDE_x. For example:
 *
 *  Parts STM32WL5xx8 have 64KiB of FLASH
 *  Parts STM32WL5xxB have 128KiB of FLASH
 *  Parts STM32WL5xxC have 256KiB of FLASH
 *
 * STM32WL5xxx has only single bank flash and page size 2KiB
 */

#if !defined(CONFIG_STM32WL5_FLASH_OVERRIDE_DEFAULT) && \
    !defined(CONFIG_STM32WL5_FLASH_OVERRIDE_8) && \
    !defined(CONFIG_STM32WL5_FLASH_OVERRIDE_B) && \
    !defined(CONFIG_STM32WL5_FLASH_OVERRIDE_C) && \
    !defined(CONFIG_STM32WL5_FLASH_OVERRIDE_E) && \
    !defined(CONFIG_STM32WL5_FLASH_OVERRIDE_G) && \
    !defined(CONFIG_STM32WL5_FLASH_CONFIG_8) && \
    !defined(CONFIG_STM32WL5_FLASH_CONFIG_B) && \
    !defined(CONFIG_STM32WL5_FLASH_CONFIG_C) && \
    !defined(CONFIG_STM32WL5_FLASH_CONFIG_E) && \
    !defined(CONFIG_STM32WL5_FLASH_CONFIG_G)
#  define CONFIG_STM32WL5_FLASH_OVERRIDE_E
#  warning "Flash size not defined defaulting to 512KiB (E)"
#endif

/* Override of the Flash has been chosen */

#if !defined(CONFIG_STM32WL5_FLASH_OVERRIDE_DEFAULT)
#  undef CONFIG_STM32WL5_FLASH_CONFIG_8
#  undef CONFIG_STM32WL5_FLASH_CONFIG_B
#  undef CONFIG_STM32WL5_FLASH_CONFIG_C
#  undef CONFIG_STM32WL5_FLASH_CONFIG_E
#  undef CONFIG_STM32WL5_FLASH_CONFIG_G
#  if defined(CONFIG_STM32WL5_FLASH_OVERRIDE_8)
#    define CONFIG_STM32WL5_FLASH_CONFIG_8
#  elif defined(CONFIG_STM32WL5_FLASH_OVERRIDE_B)
#    define CONFIG_STM32WL5_FLASH_CONFIG_B
#  elif defined(CONFIG_STM32WL5_FLASH_OVERRIDE_C)
#    define CONFIG_STM32WL5_FLASH_CONFIG_C
#  elif defined(CONFIG_STM32WL5_FLASH_OVERRIDE_E)
#    define CONFIG_STM32WL5_FLASH_CONFIG_E
#  elif defined(CONFIG_STM32WL5_FLASH_OVERRIDE_G)
#    define CONFIG_STM32WL5_FLASH_CONFIG_G
#  endif
#endif

/* Define the valid configuration  */

#if defined(CONFIG_STM32WL5_FLASH_CONFIG_8) /* 64 kB */
#  define STM32WL5_FLASH_NPAGES      32
#  define STM32WL5_FLASH_PAGESIZE    2048
#elif defined(CONFIG_STM32WL5_FLASH_CONFIG_B) /* 128 kB */
#  define STM32WL5_FLASH_NPAGES      64
#  define STM32WL5_FLASH_PAGESIZE    2048
#elif defined(CONFIG_STM32WL5_FLASH_CONFIG_C) /* 256 kB */
#  define STM32WL5_FLASH_NPAGES      128
#  define STM32WL5_FLASH_PAGESIZE    2048
#elif defined(CONFIG_STM32WL5_FLASH_CONFIG_E) /* 512 kB */
#  define STM32WL5_FLASH_NPAGES      256
#  define STM32WL5_FLASH_PAGESIZE    2048
#elif defined(CONFIG_STM32WL5_FLASH_CONFIG_G) /* 1 MB */
#  define STM32WL5_FLASH_NPAGES      512
#  define STM32WL5_FLASH_PAGESIZE    2048
#else
#  error "unknown flash configuration!"
#endif

#define STM32WL5_FLASH_SIZE          (STM32WL5_FLASH_NPAGES * STM32WL5_FLASH_PAGESIZE)

/* Register Offsets *********************************************************/

#define STM32WL5_FLASH_ACR_OFFSET       0x0000
#define STM32WL5_FLASH_ACR2_OFFSET      0x0004
#define STM32WL5_FLASH_KEYR_OFFSET      0x0008
#define STM32WL5_FLASH_OPTKEYR_OFFSET   0x000c
#define STM32WL5_FLASH_SR_OFFSET        0x0010
#define STM32WL5_FLASH_CR_OFFSET        0x0014
#define STM32WL5_FLASH_ECCR_OFFSET      0x0018
#define STM32WL5_FLASH_OPTR_OFFSET      0x0020
#define STM32WL5_FLASH_PCROP1ASR_OFFSET 0x0024
#define STM32WL5_FLASH_PCROP1AER_OFFSET 0x0028
#define STM32WL5_FLASH_WRP1AR_OFFSET    0x002c
#define STM32WL5_FLASH_WRP1BR_OFFSET    0x0030
#define STM32WL5_FLASH_PCROP1BSR_OFFSET 0x0034
#define STM32WL5_FLASH_PCROP1BER_OFFSET 0x0038
#define STM32WL5_FLASH_IPCCBR_OFFSET    0x003c
#define STM32WL5_FLASH_C2ACR_OFFSET     0x005c
#define STM32WL5_FLASH_C2SR_OFFSET      0x0060
#define STM32WL5_FLASH_C2CR_OFFSET      0x0064
#define STM32WL5_FLASH_SFR_OFFSET       0x0080
#define STM32WL5_FLASH_SRRVR_OFFSET     0x0084

/* Register Addresses *******************************************************/

#define STM32WL5_FLASH_ACR          (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_ACR_OFFSET)
#define STM32WL5_FLASH_ACR2         (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_ACR2_OFFSET)
#define STM32WL5_FLASH_KEYR         (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_KEYR_OFFSET)
#define STM32WL5_FLASH_OPTKEYR      (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_OPTKEYR_OFFSET)
#define STM32WL5_FLASH_SR           (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_SR_OFFSET)
#define STM32WL5_FLASH_CR           (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_CR_OFFSET)
#define STM32WL5_FLASH_ECCR         (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_ECCR_OFFSET)
#define STM32WL5_FLASH_OPTR         (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_OPTR_OFFSET)
#define STM32WL5_FLASH_PCROP1ASR    (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_PCROP1ASR_OFFSET)
#define STM32WL5_FLASH_PCROP1AER    (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_PCROP1AER_OFFSET)
#define STM32WL5_FLASH_WRP1AR       (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_WRP1AR_OFFSET)
#define STM32WL5_FLASH_WRP1BR       (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_WRP1BR_OFFSET)
#define STM32WL5_FLASH_PCROP1BSR    (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_PCROP1BSR_OFFSET)
#define STM32WL5_FLASH_PCROP1BER    (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_PCROP1BER_OFFSET)
#define STM32WL5_FLASH_IPCCBR       (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_IPCCBR_OFFSET)
#define STM32WL5_FLASH_C2ACR        (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_C2ACR_OFFSET)
#define STM32WL5_FLASH_C2SR         (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_C2SR_OFFSET)
#define STM32WL5_FLASH_C2CR         (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_C2CR_OFFSET)
#define STM32WL5_FLASH_SFR          (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_SFR_OFFSET)
#define STM32WL5_FLASH_SRRVR        (STM32WL5_FLASHIF_BASE+STM32WL5_FLASH_SRRVR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Flash Access Control Register (ACR) */

#define FLASH_ACR_LATENCY_SHIFT    (0)
#define FLASH_ACR_LATENCY_MASK     (7 << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY(n)     ((n) << FLASH_ACR_LATENCY_SHIFT)  /* n wait states, for Vcore range 1 and 2. */
#  define FLASH_ACR_LATENCY_0      (0 << FLASH_ACR_LATENCY_SHIFT)    /* 000: Zero wait states  */
#  define FLASH_ACR_LATENCY_1      (1 << FLASH_ACR_LATENCY_SHIFT)    /* 001: One wait state    */
#  define FLASH_ACR_LATENCY_2      (2 << FLASH_ACR_LATENCY_SHIFT)    /* 010: Two wait states   */

#define FLASH_ACR_PRFTEN           (1 <<  8) /* Bit  8:  Prefetch enable */
#define FLASH_ACR_ICEN             (1 <<  9) /* Bit  9:  Instruction cache enable */
#define FLASH_ACR_DCEN             (1 << 10) /* Bit 10: Data cache enable */
#define FLASH_ACR_ICRST            (1 << 11) /* Bit 11: Instruction cache reset */
#define FLASH_ACR_DCRST            (1 << 12) /* Bit 12: Data cache reset */
#define FLASH_ACR_PES              (1 << 15) /* Bit 15: Suspend flash program */
#define FLASH_ACR_EMPTY            (1 << 16) /* Bit 16: Is user flash empty */

/* Flash Access Control Register 2 (ACR2) */

#define FLASH_ACR2_PRIVMODE        (1 <<  0) /* Bit  0: Enable flash priviliged access mode */
#define FLASH_ACR2_HDPADIS         (1 <<  1) /* Bit  1: Disable user flash hide protection area access */
#define FLASH_ACR2_C2SWDBGEN       (1 <<  2) /* Bit  2: Enable cpu2 debug access */

/* Flash Status Register (SR) */

#define FLASH_SR_EOP               (1 <<  0) /* Bit  0: End of operation */
#define FLASH_SR_OPERR             (1 <<  1) /* Bit  1: Operation error */
#define FLASH_SR_PROGERR           (1 <<  3) /* Bit  3: Programming error */
#define FLASH_SR_WRPERR            (1 <<  4) /* Bit  4: Write protection error */
#define FLASH_SR_PGAERR            (1 <<  5) /* Bit  5: Programming alignment error */
#define FLASH_SR_SIZERR            (1 <<  6) /* Bit  6: Size error */
#define FLASH_SR_PGSERR            (1 <<  7) /* Bit  7: Programming sequence error */
#define FLASH_SR_MISERR            (1 <<  8) /* Bit  8: Fast programming data miss error */
#define FLASH_SR_FASTERR           (1 <<  9) /* Bit  9: Fast programming error */
#define FLASH_SR_OPTNV             (1 << 13) /* Bit 13: User option OPTVAL indication */
#define FLASH_SR_RDERR             (1 << 14) /* Bit 14: PCROP read error */
#define FLASH_SR_OPTVERR           (1 << 15) /* Bit 15: Option validity error */
#define FLASH_SR_BSY               (1 << 16) /* Bit 16: Busy */
#define FLASH_SR_CFGBSY            (1 << 18) /* Bit 18: Program or erase configuration busy */
#define FLASH_SR_PESD              (1 << 19) /* Bit 19: Program or erase operation suspended */

/* Flash Control Register (CR) */

#define FLASH_CR_PG                (1 << 0)  /* Bit 0 : Program Page */
#define FLASH_CR_PER               (1 << 1)  /* Bit 1 : Page Erase */
#define FLASH_CR_MER               (1 << 2)  /* Bit 2 : Mass Erase */

#define FLASH_CR_PNB_SHIFT         (3)       /* Bits 3-9: Page number */
#define FLASH_CR_PNB_MASK          (0x7F << FLASH_CR_PNB_SHIFT)
#define FLASH_CR_PNB(n)            ((n)  << FLASH_CR_PNB_SHIFT) /* Page n, n=0..127 */

#define FLASH_CR_START             (1 << 16) /* Bit 16: Start Erase */
#define FLASH_CR_OPTSTRT           (1 << 17) /* Bit 17: Options modification Start */
#define FLASH_CR_FSTPG             (1 << 18) /* Bit 18: Fast programming */
#define FLASH_CR_EOPIE             (1 << 24) /* Bit 24: End of operation interrupt enable */
#define FLASH_CR_ERRIE             (1 << 25) /* Bit 25: Error interrupt enable */
#define FLASH_CR_RDERRIE           (1 << 26) /* Bit 26: PCROP read error interrupt enable */
#define FLASH_CR_OBL_LAUNCH        (1 << 27) /* Bit 27: Option Byte Loading */
#define FLASH_CR_OPTLOCK           (1 << 30) /* Bit 30: Option Lock */
#define FLASH_CR_LOCK              (1 << 31) /* Bit 31: Lock */

/* Flash ECC Register (ECCR) */

#define FLASH_ECCR_ADDR_ECC_SHIFT  (0)       /* Bits 0-16: ECC fail address */
#  define FLASH_ECCR_ADDR_ECC_MASK (0x1ffff << FLASH_ECCR_ADDR_ECC_SHIFT)
#define FLASH_ECCR_SYSF_ECC        (1 << 20) /* Bit 20: System Flash ECC fail */
#define FLASH_ECCR_ECCCIE          (1 << 24) /* Bit 24: ECC correction interrupt enable */
#define FLASH_ECCR_CPUID_SHIFT     (26)
#  define FLASH_ECCR_CPUID_MASK    (0x7 << FLASH_ECCR_CPUID_SHIFT)
#  define FLASH_ECCR_CPUID_CPU1    (0x0 << FLASH_ECCR_CPUID_SHIFT) /* 000: cpu1 access caused ECC failure */
#  define FLASH_ECCR_CPUID_CPU2    (0x1 << FLASH_ECCR_CPUID_SHIFT) /* 001: cpu2 access caused ECC failure */

#define FLASH_ECCR_ECCC            (1 << 30) /* Bit 30: ECC correction */
#define FLASH_ECCR_ECCD            (1 << 31) /* Bit 31: ECC detection */

/* Flash Option Register (OPTR) */

#define FLASH_OPTR_RDP_SHIFT       (0)       /* Bits 0-7: Read Protection Level */
#define FLASH_OPTR_RDP_MASK        (0xFF << FLASH_OPTR_RDP_SHIFT)
#define FLASH_OPTR_RDP_NONE        (0xAA << FLASH_OPTR_RDP_SHIFT)
#define FLASH_OPTR_RDP_CHIP        (0xCC << FLASH_OPTR_RDP_SHIFT) /* WARNING, CANNOT BE REVERSED !! */

#define FLASH_OPTR_ESE             (1 << 8)  /* Bit 8: System security flag */

#define FLASH_OPTR_BORLEV_SHIFT    (9)       /* Bits 9-11: BOR reset Level */
#define FLASH_OPTR_BORLEV_MASK     (7 << FLASH_OPTR_BORLEV_SHIFT)
#define FLASH_OPTR_VBOR0           (0 << FLASH_OPTR_BORLEV_SHIFT) /* 000: BOR Level 0 (1.7 V) */
#define FLASH_OPTR_VBOR1           (1 << FLASH_OPTR_BORLEV_SHIFT) /* 001: BOR Level 1 (2.0 V) */
#define FLASH_OPTR_VBOR2           (2 << FLASH_OPTR_BORLEV_SHIFT) /* 010: BOR Level 2 (2.2 V) */
#define FLASH_OPTR_VBOR3           (3 << FLASH_OPTR_BORLEV_SHIFT) /* 011: BOR Level 3 (2.5 V) */
#define FLASH_OPTR_VBOR4           (4 << FLASH_OPTR_BORLEV_SHIFT) /* 100: BOR Level 4 (2.8 V) */

#define FLASH_OPTR_NRST_STOP       (1 << 12) /* Bit 12: Generate reset when entering the Stop mode */
#define FLASH_OPTR_NRST_STDBY      (1 << 13) /* Bit 13: Generate reset when entering the Standby mode */
#define FLASH_OPTR_NRST_SHDW       (1 << 14) /* Bit 14: Generate reset when entering the Shutdown mode */
#define FLASH_OPTR_IWDG_SW         (1 << 16) /* Bit 16: Independent watchdog selection */
#define FLASH_OPTR_IWDG_STOP       (1 << 17) /* Bit 17: Independent watchdog counter freeze in Stop mode */
#define FLASH_OPTR_IWDG_STDBY      (1 << 18) /* Bit 18: Independent watchdog counter freeze in Standby mode*/
#define FLASH_OPTR_WWDG_SW         (1 << 19) /* Bit 19: Window watchdog selection */
#define FLASH_OPTR_NBOOT1          (1 << 23) /* Bit 23: Boot configuration */
#define FLASH_OPTR_SRAM2_PE        (1 << 24) /* Bit 24: SRAM2 parity check enable */
#define FLASH_OPTR_SRAM_RST        (1 << 25) /* Bit 25: SRAM1/2 Erase when system reset */
#define FLASH_OPTR_NSWBOOT0        (1 << 26) /* Bit 26: Software BOOT0 */
#define FLASH_OPTR_NBOOT0          (1 << 27) /* Bit 27: nBOOT0 option bit */

#define FLASH_OPTR_C1BOOT_LOCK     (1 << 30) /* Bit 30: Enable cpu1 boot lock */
#define FLASH_OPTR_C2BOOT_LOCK     (1 << 31) /* Bit 31: Enable cpu2 boot lock */

/* Flash CPU2 Access Control Register (C2ACR) */

#define FLASH_C2ACR_PRFTEN         (1 <<  8) /* Bit  8: Enable cpu2 prefetch */
#define FLASH_C2ACR_ICEN           (1 <<  9) /* Bit  9: Enable cpu2 instruction cache */
#define FLASH_C2ACR_ICRST          (1 << 11) /* Bit 11: Reset cpu2 instruction cache */
#define FLASH_C2ACR_PES            (1 << 15) /* Bit 15: Suspend cpu2 flash program/erase */

/* Flash CPU2 Status Register (C2SR) */

#define FLASH_C2SR_EOP             (1 <<  0) /* Bit  0: End of operation flag */
#define FLASH_C2SR_OPERR           (1 <<  1) /* Bit  1: Operation */
#define FLASH_C2SR_PROGERR         (1 <<  3) /* Bit  3: Programming */
#define FLASH_C2SR_WRPERR          (1 <<  4) /* Bit  4: Write protection */
#define FLASH_C2SR_PGAERR          (1 <<  5) /* Bit  5: Programming alignment */
#define FLASH_C2SR_SIZERR          (1 <<  6) /* Bit  6: Size */
#define FLASH_C2SR_PGSERR          (1 <<  7) /* Bit  7: Programming sequence */
#define FLASH_C2SR_MISSERR         (1 <<  8) /* Bit  8: Fast programming data miss */
#define FLASH_C2SR_FASTERR         (1 <<  9) /* Bit  9: Fast programming */
#define FLASH_C2SR_RDERR           (1 << 14) /* Bit 14: PCROP read */
#define FLASH_C2SR_BSY             (1 << 16) /* Bit 16: Busy flag */
#define FLASH_C2SR_CFGBSY          (1 << 18) /* Bit 18: Program or erase configuration busy */
#define FLASH_C2SR_PESD            (1 << 19) /* Bit 19: Program/erase operation suspended */

/* Flash CPU2 Control Register (C2CR) */

#define FLASH_C2CR_PG              (1 << 0)  /* Bit 0 : Program Page */
#define FLASH_C2CR_PER             (1 << 1)  /* Bit 1 : Page Erase */
#define FLASH_C2CR_MER             (1 << 2)  /* Bit 2 : Mass Erase */

#define FLASH_C2CR_PNB_SHIFT       (3)       /* Bits 3-9: Page number */
#define FLASH_C2CR_PNB_MASK        (0x7F << FLASH_C2CR_PNB_SHIFT)
#define FLASH_C2CR_PNB(n)          ((n)  << FLASH_C2CR_PNB_SHIFT) /* Page n, n=0..127 */

#define FLASH_C2CR_START           (1 << 16) /* Bit 16: Start Erase */
#define FLASH_C2CR_FSTPG           (1 << 18) /* Bit 23: Fast programming */
#define FLASH_C2CR_EOPIE           (1 << 24) /* Bit 24: End of operation interrupt enable */
#define FLASH_C2CR_ERRIE           (1 << 25) /* Bit 25: Error interrupt enable */
#define FLASH_C2CR_RDERRIE         (1 << 26) /* Bit 26: PCROP read error interrupt enable */

#endif /* __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_FLASH_H */
