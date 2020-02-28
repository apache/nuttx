/************************************************************************************
 * arch/arm/src/stm32l4/hardware/stm32l4_flash.h
 *
 *   Copyright (C) 2009, 2011, 2015, 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *            Juha Niskanen <juha.niskanen@haltian.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_FLASH_H
#define __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_FLASH_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Flash size is known from the chip selection:
 *
 *   When CONFIG_STM32L4_FLASH_OVERRIDE_DEFAULT is set the
 *   CONFIG_STM32L4_FLASH_CONFIG_x selects the default FLASH size based on the chip
 *   part number. This value can be overridden with CONFIG_STM32L4_FLASH_OVERRIDE_x
 *
 *   Parts STM32L4xxE have 512Kb of FLASH
 *   Parts STM32L4xxG have 1024Kb of FLASH
 *
 *   N.B. Only Single bank mode is supported
 */

#define _K(x) ((x)*1024)

#if !defined(CONFIG_STM32L4_FLASH_OVERRIDE_DEFAULT) && \
    !defined(CONFIG_STM32L4_FLASH_OVERRIDE_8) && \
    !defined(CONFIG_STM32L4_FLASH_OVERRIDE_B) && \
    !defined(CONFIG_STM32L4_FLASH_OVERRIDE_C) && \
    !defined(CONFIG_STM32L4_FLASH_OVERRIDE_E) && \
    !defined(CONFIG_STM32L4_FLASH_OVERRIDE_G) && \
    !defined(CONFIG_STM32L4_FLASH_OVERRIDE_I) && \
    !defined(CONFIG_STM32L4_FLASH_CONFIG_8) && \
    !defined(CONFIG_STM32L4_FLASH_CONFIG_B) && \
    !defined(CONFIG_STM32L4_FLASH_CONFIG_C) && \
    !defined(CONFIG_STM32L4_FLASH_CONFIG_E) && \
    !defined(CONFIG_STM32L4_FLASH_CONFIG_G) && \
    !defined(CONFIG_STM32L4_FLASH_CONFIG_I)
#  define CONFIG_STM32L4_FLASH_OVERRIDE_E
#  warning "Flash size not defined defaulting to 512KiB (E)"
#endif

/* Override of the Flash has been chosen */

#if !defined(CONFIG_STM32L4_FLASH_OVERRIDE_DEFAULT)
#  undef CONFIG_STM32L4_FLASH_CONFIG_8
#  undef CONFIG_STM32L4_FLASH_CONFIG_B
#  undef CONFIG_STM32L4_FLASH_CONFIG_C
#  undef CONFIG_STM32L4_FLASH_CONFIG_E
#  undef CONFIG_STM32L4_FLASH_CONFIG_G
#  undef CONFIG_STM32L4_FLASH_CONFIG_I
#  if defined(CONFIG_STM32L4_FLASH_OVERRIDE_8)
#    define CONFIG_STM32L4_FLASH_CONFIG_8
#  elif defined(CONFIG_STM32L4_FLASH_OVERRIDE_B)
#    define CONFIG_STM32L4_FLASH_CONFIG_B
#  elif defined(CONFIG_STM32L4_FLASH_OVERRIDE_C)
#    define CONFIG_STM32L4_FLASH_CONFIG_C
#  elif defined(CONFIG_STM32L4_FLASH_OVERRIDE_E)
#    define CONFIG_STM32L4_FLASH_CONFIG_E
#  elif defined(CONFIG_STM32L4_FLASH_OVERRIDE_G)
#    define CONFIG_STM32L4_FLASH_CONFIG_G
#  elif defined(CONFIG_STM32L4_FLASH_OVERRIDE_I)
#    define CONFIG_STM32L4_FLASH_CONFIG_I
#  endif
#endif

/* Define the valid configuration  */

#if defined(CONFIG_STM32L4_FLASH_CONFIG_8) /* 64 kB */
#  define STM32L4_FLASH_NPAGES      32
#  define STM32L4_FLASH_PAGESIZE    2048
#elif defined(CONFIG_STM32L4_FLASH_CONFIG_B) /* 128 kB */
#  define STM32L4_FLASH_NPAGES      64
#  define STM32L4_FLASH_PAGESIZE    2048
#elif defined(CONFIG_STM32L4_FLASH_CONFIG_C) /* 256 kB */
#  define STM32L4_FLASH_NPAGES      128
#  define STM32L4_FLASH_PAGESIZE    2048
#elif defined(CONFIG_STM32L4_FLASH_CONFIG_E) /* 512 kB */
#  define STM32L4_FLASH_NPAGES      256
#  define STM32L4_FLASH_PAGESIZE    2048
#elif defined(CONFIG_STM32L4_FLASH_CONFIG_G) /* 1 MB */
#  define STM32L4_FLASH_NPAGES      512
#  define STM32L4_FLASH_PAGESIZE    2048
#elif defined(CONFIG_STM32L4_FLASH_CONFIG_I) /* 2 MB */
#  define STM32L4_FLASH_NPAGES      256
#  define STM32L4_FLASH_PAGESIZE    8192
#else
#  error "unknown flash configuration!"
#endif

#ifdef STM32L4_FLASH_PAGESIZE
#  define STM32L4_FLASH_SIZE            (STM32L4_FLASH_NPAGES * STM32L4_FLASH_PAGESIZE)
#endif

/* Register Offsets *****************************************************************/

#define STM32L4_FLASH_ACR_OFFSET      0x0000
#define STM32L4_FLASH_PDKEYR_OFFSET   0x0004
#define STM32L4_FLASH_KEYR_OFFSET     0x0008
#define STM32L4_FLASH_OPTKEYR_OFFSET  0x000c
#define STM32L4_FLASH_SR_OFFSET       0x0010
#define STM32L4_FLASH_CR_OFFSET       0x0014
#define STM32L4_FLASH_ECCR_OFFSET     0x0018
#define STM32L4_FLASH_OPTR_OFFSET     0x0020
#define STM32L4_FLASH_PCROP1SR_OFFSET 0x0024
#define STM32L4_FLASH_PCROP1ER_OFFSET 0x0028
#define STM32L4_FLASH_WRP1AR_OFFSET   0x002c
#define STM32L4_FLASH_WRP1BR_OFFSET   0x0030
#if defined(CONFIG_STM32L4_STM32L4X5) || defined(CONFIG_STM32L4_STM32L4X6) || \
    defined(CONFIG_STM32L4_STM32L4XR)
#  define STM32L4_FLASH_PCROP2SR_OFFSET 0x0044
#  define STM32L4_FLASH_PCROP2ER_OFFSET 0x0048
#  define STM32L4_FLASH_WRP2AR_OFFSET   0x004c
#  define STM32L4_FLASH_WRP2BR_OFFSET   0x0050
#endif
#if defined(CONFIG_STM32L4_STM32L4XR)
#  define STM32L4_FLASH_CFGR_OFFSET     0x0130
#endif

/* Register Addresses ***************************************************************/

#define STM32L4_FLASH_ACR            (STM32L4_FLASHIF_BASE+STM32L4_FLASH_ACR_OFFSET)
#define STM32L4_FLASH_PDKEYR         (STM32L4_FLASHIF_BASE+STM32L4_FLASH_PDKEYR_OFFSET)
#define STM32L4_FLASH_KEYR           (STM32L4_FLASHIF_BASE+STM32L4_FLASH_KEYR_OFFSET)
#define STM32L4_FLASH_OPTKEYR        (STM32L4_FLASHIF_BASE+STM32L4_FLASH_OPTKEYR_OFFSET)
#define STM32L4_FLASH_SR             (STM32L4_FLASHIF_BASE+STM32L4_FLASH_SR_OFFSET)
#define STM32L4_FLASH_CR             (STM32L4_FLASHIF_BASE+STM32L4_FLASH_CR_OFFSET)
#define STM32L4_FLASH_ECCR           (STM32L4_FLASHIF_BASE+STM32L4_FLASH_ECCR_OFFSET)
#define STM32L4_FLASH_OPTR           (STM32L4_FLASHIF_BASE+STM32L4_FLASH_OPTR_OFFSET)
#define STM32L4_FLASH_PCROP1SR       (STM32L4_FLASHIF_BASE+STM32L4_FLASH_PCROP1SR_OFFSET)
#define STM32L4_FLASH_PCROP1ER       (STM32L4_FLASHIF_BASE+STM32L4_FLASH_PCROP1ER_OFFSET)
#define STM32L4_FLASH_WRP1AR         (STM32L4_FLASHIF_BASE+STM32L4_FLASH_WRP1AR_OFFSET)
#define STM32L4_FLASH_WRP1BR         (STM32L4_FLASHIF_BASE+STM32L4_FLASH_WRP1BR_OFFSET)
#if defined(CONFIG_STM32L4_STM32L4X5) || defined(CONFIG_STM32L4_STM32L4X6) || \
    defined(CONFIG_STM32L4_STM32L4XR)
#  define STM32L4_FLASH_PCROP2SR     (STM32L4_FLASHIF_BASE+STM32L4_FLASH_PCROP2SR_OFFSET)
#  define STM32L4_FLASH_PCROP2ER     (STM32L4_FLASHIF_BASE+STM32L4_FLASH_PCROP2ER_OFFSET)
#  define STM32L4_FLASH_WRP2AR       (STM32L4_FLASHIF_BASE+STM32L4_FLASH_WRP2AR_OFFSET)
#  define STM32L4_FLASH_WRP2BR       (STM32L4_FLASHIF_BASE+STM32L4_FLASH_WRP2BR_OFFSET)
#endif
#if defined(CONFIG_STM32L4_STM32L4XR)
#  define STM32L4_FLASH_CFGR         (STM32L4_FLASHIF_BASE+STM32L4_FLASH_CFGR_OFFSET)
#endif

/* Register Bitfield Definitions ****************************************************/
/* Flash Access Control Register (ACR) */

#define FLASH_ACR_LATENCY_SHIFT   (0)
#define FLASH_ACR_LATENCY_MASK    (7 << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY(n)    ((n) << FLASH_ACR_LATENCY_SHIFT)  /* n wait states, for Vcore range 1 and 2. */
#  define FLASH_ACR_LATENCY_0     (0 << FLASH_ACR_LATENCY_SHIFT)    /* 000: Zero wait states  */
#  define FLASH_ACR_LATENCY_1     (1 << FLASH_ACR_LATENCY_SHIFT)    /* 001: One wait state    */
#  define FLASH_ACR_LATENCY_2     (2 << FLASH_ACR_LATENCY_SHIFT)    /* 010: Two wait states   */
#  define FLASH_ACR_LATENCY_3     (3 << FLASH_ACR_LATENCY_SHIFT)    /* 011: Three wait states */
#  define FLASH_ACR_LATENCY_4     (4 << FLASH_ACR_LATENCY_SHIFT)    /* 100: Four wait states  */
#  define FLASH_ACR_LATENCY_5     (5 << FLASH_ACR_LATENCY_SHIFT)    /* 101: Five wait states  */

#define FLASH_ACR_PRFTEN            (1 << 8)  /* Bit 8:  Prefetch enable */
#define FLASH_ACR_ICEN              (1 << 9)  /* Bit 9:  Instruction cache enable */
#define FLASH_ACR_DCEN              (1 << 10) /* Bit 10: Data cache enable */
#define FLASH_ACR_ICRST             (1 << 11) /* Bit 11: Instruction cache reset */
#define FLASH_ACR_DCRST             (1 << 12) /* Bit 12: Data cache reset */
#define FLASH_ACR_RUN_PD            (1 << 13) /* Bit 13: Flash mode during Run */
#define FLASH_ACR_SLEEP_PD          (1 << 14) /* Bit 14: Flash mode during Sleep */

/* Flash Status Register (SR) */

#define FLASH_SR_EOP                (1 << 0)  /* Bit 0:  End of operation */
#define FLASH_SR_OPERR              (1 << 1)  /* Bit 1:  Operation error */
#define FLASH_SR_PROGERR            (1 << 3)  /* Bit 3:  Programming error */
#define FLASH_SR_WRPERR             (1 << 4)  /* Bit 4:  Write protection error */
#define FLASH_SR_PGAERR             (1 << 5)  /* Bit 5:  Programming alignment error */
#define FLASH_SR_SIZERR             (1 << 6)  /* Bit 6:  Size error */
#define FLASH_SR_PGSERR             (1 << 7)  /* Bit 7:  Programming sequence error */
#define FLASH_SR_MISERR             (1 << 8)  /* Bit 8:  Fast programming data miss error */
#define FLASH_SR_FASTERR            (1 << 9)  /* Bit 9:  Fast programming error */
#define FLASH_SR_RDERR              (1 << 14) /* Bit 14: PCROP read error */
#define FLASH_SR_OPTVERR            (1 << 15) /* Bit 15: Option validity error */
#define FLASH_SR_BSY                (1 << 16) /* Bit 16: Busy */
#if defined(CONFIG_STM32L4_STM32L4X3) || defined(CONFIG_STM32L4_STM32L4XR)
#  define FLASH_SR_PEMPTY           (1 << 17) /* Bit 17: Program empty */
#endif

/* Flash Control Register (CR) */

#define FLASH_CR_PG                 (1 << 0)                /* Bit 0 : Program Page */
#define FLASH_CR_PER                (1 << 1)                /* Bit 1 : Page Erase */
#define FLASH_CR_MER1               (1 << 2)                /* Bit 2 : Mass Erase Bank 1 */

#define FLASH_CR_PNB_SHIFT          (3)                     /* Bits 3-10: Page number */
#define FLASH_CR_PNB_MASK           (0xFF << FLASH_CR_PNB_SHIFT)
#define FLASH_CR_PNB(n)             ((n)  << FLASH_CR_PNB_SHIFT) /* Page n (if BKER=0) or n+256 (if BKER=1), n=0..255 */

#if defined(CONFIG_STM32L4_STM32L4X5) || defined(CONFIG_STM32L4_STM32L4X6) || \
    defined(CONFIG_STM32L4_STM32L4XR)
#  define FLASH_CR_BKER             (1 << 11)               /* Bit 11: Page number MSB (Bank selection) */
#  define FLASH_CR_MER2             (1 << 15)               /* Bit 15: Mass Erase Bank 2 */
#endif
#define FLASH_CR_START              (1 << 16)               /* Bit 16: Start Erase */
#define FLASH_CR_OPTSTRT            (1 << 17)               /* Bit 17: Options modification Start */
#define FLASH_CR_FSTPG              (1 << 23)               /* Bit 23: Fast programming */
#define FLASH_CR_EOPIE              (1 << 24)               /* Bit 24: End of operation interrupt enable */
#define FLASH_CR_ERRIE              (1 << 25)               /* Bit 25: Error interrupt enable */
#define FLASH_CR_RDERRIE            (1 << 26)               /* Bit 26: PCROP read error interrupt enable */
#define FLASH_CR_OBL_LAUNCH         (1 << 27)               /* Bit 27: Option Byte Loading */
#define FLASH_CR_OPTLOCK            (1 << 30)               /* Bit 30: Option Lock */
#define FLASH_CR_LOCK               (1 << 31)               /* Bit 31: Lock */

/* Flash ECC Register (ECCR) */

#define FLASH_ECCR_ADDR_ECC_SHIFT   (0)                    /* Bits 8-15: Read protect */
#define FLASH_ECCR_ADDR_ECC_MASK    (0x07ffff << FLASH_ECCR_ADDR_ECC_SHIFT)
#if defined(CONFIG_STM32L4_STM32L4X5) || defined(CONFIG_STM32L4_STM32L4X6) || \
    defined(CONFIG_STM32L4_STM32L4XR)
#  define FLASH_ECCR_BK_ECC         (1 << 19)               /* Bit 19: ECC fail bank */
#endif
#define FLASH_ECCR_SYSF_ECC         (1 << 20)               /* Bit 20: System Flash ECC fail */
#define FLASH_ECCR_ECCCIE           (1 << 24)               /* Bit 24: ECC correction interrupt enable */
#define FLASH_ECCR_ECCC             (1 << 30)               /* Bit 30: ECC correction */
#define FLASH_ECCR_ECCD             (1 << 31)               /* Bit 31: ECC detection */

/* Flash Option Control Register (OPTCR) */

#define FLASH_OPTCR_NRST_STOP       (1 << 12)               /* Bit 12: Generate reset when entering the Stop mode */
#define FLASH_OPTCR_NRST_STDBY      (1 << 13)               /* Bit 13: Generate reset when entering the Standby mode */
#define FLASH_OPTCR_NRST_SHDW       (1 << 14)               /* Bit 14: Generate reset when entering the Shutdown mode */
#define FLASH_OPTCR_IWDG_SW         (1 << 16)               /* Bit 16: Independent watchdog selection */
#define FLASH_OPTCR_IWDG_STOP       (1 << 17)               /* Bit 17: Independent watchdog counter freeze in Stop mode */
#define FLASH_OPTCR_IWDG_STDBY      (1 << 18)               /* Bit 18: Independent watchdog counter freeze in Standby mode*/
#define FLASH_OPTCR_WWDG_SW         (1 << 19)               /* Bit 19: Window watchdog selection */
#if defined(CONFIG_STM32L4_STM32L4X5) || defined(CONFIG_STM32L4_STM32L4X6) || \
    defined(CONFIG_STM32L4_STM32L4XR)
#  define FLASH_OPTCR_BFB2          (1 << 20)               /* Bit 20: Dual bank boot */
#  define FLASH_OPTCR_DUALBANK      (1 << 21)               /* Bit 21: Dual bank enable */
#endif
#define FLASH_OPTCR_NBOOT1          (1 << 23)               /* Bit 23: Boot configuration */
#define FLASH_OPTCR_SRAM2_PE        (1 << 24)               /* Bit 24: SRAM2 parity check enable */
#define FLASH_OPTCR_SRAM2_RST       (1 << 25)               /* Bit 25: SRAM2 Erase when system reset */
#if defined(CONFIG_STM32L4_STM32L4X3) || defined(CONFIG_STM32L4_STM32L496XX) || defined(CONFIG_STM32L4_STM32L4XR)
#  define FLASH_OPTCR_NSWBOOT0      (1 << 26)               /* Bit 26: Software BOOT0 */
#  define FLASH_OPTCR_NBOOT0        (1 << 27)               /* Bit 27: nBOOT0 option bit */
#endif

#define FLASH_OPTCR_BORLEV_SHIFT    (8)                     /* Bits 8-10: BOR reset Level */
#define FLASH_OPTCR_BORLEV_MASK     (7 << FLASH_OPTCR_BORLEV_SHIFT)
#define FLASH_OPTCR_VBOR0           (0 << FLASH_OPTCR_BORLEV_SHIFT) /* 000: BOR Level 0 (1.7 V) */
#define FLASH_OPTCR_VBOR1           (1 << FLASH_OPTCR_BORLEV_SHIFT) /* 001: BOR Level 1 (2.0 V) */
#define FLASH_OPTCR_VBOR2           (2 << FLASH_OPTCR_BORLEV_SHIFT) /* 010: BOR Level 2 (2.2 V) */
#define FLASH_OPTCR_VBOR3           (3 << FLASH_OPTCR_BORLEV_SHIFT) /* 011: BOR Level 3 (2.5 V) */
#define FLASH_OPTCR_VBOR4           (4 << FLASH_OPTCR_BORLEV_SHIFT) /* 100: BOR Level 4 (2.8 V) */
#define FLASH_OPTCR_RDP_SHIFT       (0)                     /* Bits 0-7: Read Protection Level */
#define FLASH_OPTCR_RDP_MASK        (0xFF << FLASH_OPTCR_RDP_SHIFT)
#define FLASH_OPTCR_RDP_NONE        (0xAA << FLASH_OPTCR_RDP_SHIFT)
#define FLASH_OPTCR_RDP_CHIP        (0xCC << FLASH_OPTCR_RDP_SHIFT) /* WARNING, CANNOT BE REVERSED !! */

/* Flash Configuration Register (CFGR) */

#if defined(CONFIG_STM32L4_STM32L4XR)
#  define FLASH_CFGR_LVEN           (1 << 0)                /* Bit 0: Low voltage enable */
#endif

#endif /* __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_FLASH_H */
