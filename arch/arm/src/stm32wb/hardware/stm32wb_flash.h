/****************************************************************************
 * arch/arm/src/stm32wb/hardware/stm32wb_flash.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_FLASH_H
#define __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flash size is known from the chip selection:
 *
 *   When CONFIG_STM32WB_FLASH_OVERRIDE_DEFAULT is set the
 *   CONFIG_STM32WB_FLASH_CONFIG_x selects the default FLASH size based on
 *   the chip part number.  This value can be overridden with
 *   CONFIG_STM32WB_FLASH_OVERRIDE_x
 *
 *   Parts STM32WB3xxC have 256Kb of FLASH
 *   Parts STM32WB5xxC have 256Kb of FLASH
 *   Parts STM32WB1xxC have 320Kb of FLASH
 *   Parts STM32WBxxxE have 512Kb of FLASH
 *   Parts STM32WBxxxY have 640Kb of FLASH
 *   Parts STM32WBxxxG have 1024Kb of FLASH
 *
 *   N.B. Only Single bank mode is supported
 */

#if !defined(CONFIG_STM32WB_FLASH_OVERRIDE_DEFAULT) && \
    !defined(CONFIG_STM32WB_FLASH_OVERRIDE_C_256) && \
    !defined(CONFIG_STM32WB_FLASH_OVERRIDE_C_320) && \
    !defined(CONFIG_STM32WB_FLASH_OVERRIDE_E_512) && \
    !defined(CONFIG_STM32WB_FLASH_OVERRIDE_Y_640) && \
    !defined(CONFIG_STM32WB_FLASH_OVERRIDE_G_1024) && \
    !defined(CONFIG_STM32WB_FLASH_CONFIG_C_256) && \
    !defined(CONFIG_STM32WB_FLASH_CONFIG_C_320) && \
    !defined(CONFIG_STM32WB_FLASH_CONFIG_E_512) && \
    !defined(CONFIG_STM32WB_FLASH_CONFIG_Y_640) && \
    !defined(CONFIG_STM32WB_FLASH_CONFIG_G_1024)
#  error "Flash size not defined"
#endif

/* Override of the Flash has been chosen */

#if !defined(CONFIG_STM32WB_FLASH_OVERRIDE_DEFAULT)
#  undef CONFIG_STM32WB_FLASH_CONFIG_C_256
#  undef CONFIG_STM32WB_FLASH_CONFIG_C_320
#  undef CONFIG_STM32WB_FLASH_CONFIG_E_512
#  undef CONFIG_STM32WB_FLASH_CONFIG_Y_640
#  undef CONFIG_STM32WB_FLASH_CONFIG_G_1024
#  if defined(CONFIG_STM32WB_FLASH_OVERRIDE_C_256)
#    define CONFIG_STM32WB_FLASH_CONFIG_C_256
#  elif defined(CONFIG_STM32WB_FLASH_OVERRIDE_C_320)
#    define CONFIG_STM32WB_FLASH_CONFIG_C_320
#  elif defined(CONFIG_STM32WB_FLASH_OVERRIDE_E_512)
#    define CONFIG_STM32WB_FLASH_CONFIG_E_512
#  elif defined(CONFIG_STM32WB_FLASH_OVERRIDE_Y_640)
#    define CONFIG_STM32WB_FLASH_CONFIG_Y_640
#  elif defined(CONFIG_STM32WB_FLASH_OVERRIDE_G_1024)
#    define CONFIG_STM32WB_FLASH_CONFIG_G_1024
#  endif
#endif

/* Define the valid configuration  */

#define STM32WB_FLASH_PAGESIZE      4096

#if defined(CONFIG_STM32WB_FLASH_CONFIG_C_256)   /* 256 kB */
#  define STM32WB_FLASH_NPAGES      64
#elif defined(CONFIG_STM32WB_FLASH_CONFIG_C_320) /* 320 kB */
#  define STM32WB_FLASH_NPAGES      80
#elif defined(CONFIG_STM32WB_FLASH_CONFIG_E_512) /* 512 kB */
#  define STM32WB_FLASH_NPAGES      128
#elif defined(CONFIG_STM32WB_FLASH_CONFIG_Y_640) /* 640 kB */
#  define STM32WB_FLASH_NPAGES      160
#elif defined(CONFIG_STM32WB_FLASH_CONFIG_G_1024) /* 1 MB */
#  define STM32WB_FLASH_NPAGES      256
#else
#  error "Unknown flash configuration!"
#endif

#define STM32WB_FLASH_SIZE        (STM32WB_FLASH_NPAGES * STM32WB_FLASH_PAGESIZE)

/* Register Offsets *********************************************************/

#define STM32WB_FLASH_ACR_OFFSET        0x0000  /* Flash Access Control Register */
#define STM32WB_FLASH_KEYR_OFFSET       0x0008  /* Flash Key Register */
#define STM32WB_FLASH_OPTKEYR_OFFSET    0x000c  /* Flash Option Key Register */
#define STM32WB_FLASH_SR_OFFSET         0x0010  /* Flash Status Register */
#define STM32WB_FLASH_CR_OFFSET         0x0014  /* Flash Control Register */
#define STM32WB_FLASH_ECCR_OFFSET       0x0018  /* Flash ECC Register */
#define STM32WB_FLASH_OPTR_OFFSET       0x0020  /* Flash Option Register */
#define STM32WB_FLASH_PCROP1ASR_OFFSET  0x0024  /* Flash PCROP zone A Start address Register */
#define STM32WB_FLASH_PCROP1AER_OFFSET  0x0028  /* Flash PCROP zone A End address Register */
#define STM32WB_FLASH_WRP1AR_OFFSET     0x002c  /* Flash WRP area A Address Register */
#define STM32WB_FLASH_WRP1BR_OFFSET     0x0030  /* Flash WRP area B Address Register */
#define STM32WB_FLASH_PCROP1BSR_OFFSET  0x0034  /* Flash PCROP zone B Start address Register */
#define STM32WB_FLASH_PCROP1BER_OFFSET  0x0038  /* Flash PCROP zone B End address Register */
#define STM32WB_FLASH_IPCCBR_OFFSET     0x003C  /* Flash IPCC mailbox data buffer address Register */
#define STM32WB_FLASH_C2ACR_OFFSET      0x005C  /* CPU2 flash Access Control Register */
#define STM32WB_FLASH_C2SR_OFFSET       0x0060  /* CPU2 flash Status Register */
#define STM32WB_FLASH_C2CR_OFFSET       0x0064  /* CPU2 flash Control Register */
#define STM32WB_FLASH_SFR_OFFSET        0x0080  /* Secure Flash start address Register */
#define STM32WB_FLASH_SRRVR_OFFSET      0x0084  /* SRAM2 start address and CPU2 Reset Vector Register */

/* Register Addresses *******************************************************/

#define STM32WB_FLASH_ACR               (STM32WB_FLASHREG_BASE + STM32WB_FLASH_ACR_OFFSET)
#define STM32WB_FLASH_KEYR              (STM32WB_FLASHREG_BASE + STM32WB_FLASH_KEYR_OFFSET)
#define STM32WB_FLASH_OPTKEYR           (STM32WB_FLASHREG_BASE + STM32WB_FLASH_OPTKEYR_OFFSET)
#define STM32WB_FLASH_SR                (STM32WB_FLASHREG_BASE + STM32WB_FLASH_SR_OFFSET)
#define STM32WB_FLASH_CR                (STM32WB_FLASHREG_BASE + STM32WB_FLASH_CR_OFFSET)
#define STM32WB_FLASH_ECCR              (STM32WB_FLASHREG_BASE + STM32WB_FLASH_ECCR_OFFSET)
#define STM32WB_FLASH_OPTR              (STM32WB_FLASHREG_BASE + STM32WB_FLASH_OPTR_OFFSET)
#define STM32WB_FLASH_PCROP1ASR         (STM32WB_FLASHREG_BASE + STM32WB_FLASH_PCROP1ASR_OFFSET)
#define STM32WB_FLASH_PCROP1AER         (STM32WB_FLASHREG_BASE + STM32WB_FLASH_PCROP1AER_OFFSET)
#define STM32WB_FLASH_WRP1AR            (STM32WB_FLASHREG_BASE + STM32WB_FLASH_WRP1AR_OFFSET)
#define STM32WB_FLASH_WRP1BR            (STM32WB_FLASHREG_BASE + STM32WB_FLASH_WRP1BR_OFFSET)
#define STM32WB_FLASH_PCROP1BSR         (STM32WB_FLASHREG_BASE + STM32WB_FLASH_PCROP1BSR_OFFSET)
#define STM32WB_FLASH_PCROP1BER         (STM32WB_FLASHREG_BASE + STM32WB_FLASH_PCROP1BER_OFFSET)
#define STM32WB_FLASH_IPCCBR            (STM32WB_FLASHREG_BASE + STM32WB_FLASH_IPCCBR_OFFSET)
#define STM32WB_FLASH_C2ACR             (STM32WB_FLASHREG_BASE + STM32WB_FLASH_C2ACR_OFFSET)
#define STM32WB_FLASH_C2SR              (STM32WB_FLASHREG_BASE + STM32WB_FLASH_C2SR_OFFSET)
#define STM32WB_FLASH_C2CR              (STM32WB_FLASHREG_BASE + STM32WB_FLASH_C2CR_OFFSET)
#define STM32WB_FLASH_SFR               (STM32WB_FLASHREG_BASE + STM32WB_FLASH_SFR_OFFSET)
#define STM32WB_FLASH_SRRVR             (STM32WB_FLASHREG_BASE + STM32WB_FLASH_SRRVR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Flash Access Control Register (ACR) */

#define FLASH_ACR_LATENCY_SHIFT     (0)       /* Bits 0-2: Flash memory access latency */
#define FLASH_ACR_LATENCY_MASK      (0x7 << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY(n)      ((n) << FLASH_ACR_LATENCY_SHIFT)  /* n wait states, n = 0..3 */
#  define FLASH_ACR_LATENCY_0       (0x0 << FLASH_ACR_LATENCY_SHIFT)  /* 000: Zero wait states  */
#  define FLASH_ACR_LATENCY_1       (0x1 << FLASH_ACR_LATENCY_SHIFT)  /* 001: One wait state    */
#  define FLASH_ACR_LATENCY_2       (0x2 << FLASH_ACR_LATENCY_SHIFT)  /* 010: Two wait states   */
#  define FLASH_ACR_LATENCY_3       (0x3 << FLASH_ACR_LATENCY_SHIFT)  /* 011: Three wait states */

#define FLASH_ACR_PRFTEN            (1 << 8)  /* Bit 8: Prefetch enable */
#define FLASH_ACR_ICEN              (1 << 9)  /* Bit 9: Instruction cache enable */
#define FLASH_ACR_DCEN              (1 << 10) /* Bit 10: Data cache enable */
#define FLASH_ACR_ICRST             (1 << 11) /* Bit 11: Instruction cache reset */
#define FLASH_ACR_DCRST             (1 << 12) /* Bit 12: Data cache reset */
#define FLASH_ACR_PES               (1 << 15) /* Bit 15: Program/Erase suspend request */
#define FLASH_ACR_EMPTY             (1 << 16) /* Bit 16: Flash memory user area empty */

/* Flash Key Register (KEYR) */

#define FLASH_KEYR_KEY1             (0x45670123)
#define FLASH_KEYR_KEY2             (0xcdef89ab)

/* Flash Option Key Register (OPTKEYR) */

#define FLASH_OPTKEYR_KEY1          (0x08192a3b)
#define FLASH_OPTKEYR_KEY2          (0x4c5d6e7f)

/* Flash Status Register (SR) */

#define FLASH_SR_EOP                (1 << 0)  /* Bit 0: End of operation */
#define FLASH_SR_OPERR              (1 << 1)  /* Bit 1: Operation error */
#define FLASH_SR_PROGERR            (1 << 3)  /* Bit 3: Programming error */
#define FLASH_SR_WRPERR             (1 << 4)  /* Bit 4: Write protection error */
#define FLASH_SR_PGAERR             (1 << 5)  /* Bit 5: Programming alignment error */
#define FLASH_SR_SIZERR             (1 << 6)  /* Bit 6: Size error */
#define FLASH_SR_PGSERR             (1 << 7)  /* Bit 7: Programming sequence error */
#define FLASH_SR_MISERR             (1 << 8)  /* Bit 8: Fast programming data miss error */
#define FLASH_SR_FASTERR            (1 << 9)  /* Bit 9: Fast programming error */
#define FLASH_SR_OPTNV              (1 << 13) /* Bit 13: User option OPTVAL indication */
#define FLASH_SR_RDERR              (1 << 14) /* Bit 14: PCROP read error */
#define FLASH_SR_OPTVERR            (1 << 15) /* Bit 15: Option and Engineering bit validity error */
#define FLASH_SR_BSY                (1 << 16) /* Bit 16: Busy */
#define FLASH_SR_CFGBSY             (1 << 18) /* Bit 18: Programming or erase configuration busy */
#define FLASH_SR_PESD               (1 << 19) /* Bit 19: Programming / erase configuration suspended */

/* Flash Control Register (CR) */

#define FLASH_CR_PG                 (1 << 0)  /* Bit 0: Programming */
#define FLASH_CR_PER                (1 << 1)  /* Bit 1: Page Erase */
#define FLASH_CR_MER                (1 << 2)  /* Bit 2: Mass Erase */
#define FLASH_CR_PNB_SHIFT          (3)       /* Bits 3-10: Page number selection */
#define FLASH_CR_PNB_MASK           (0xff << FLASH_CR_PNB_SHIFT)
#  define FLASH_CR_PNB(n)           ((n)  << FLASH_CR_PNB_SHIFT) /* Page n, n = 0..255 */

#define FLASH_CR_STRT               (1 << 16) /* Bit 16: Start Erase */
#define FLASH_CR_OPTSTRT            (1 << 17) /* Bit 17: Options modification Start */
#define FLASH_CR_FSTPG              (1 << 18) /* Bit 18: Fast programming */
#define FLASH_CR_EOPIE              (1 << 24) /* Bit 24: End of operation interrupt enable */
#define FLASH_CR_ERRIE              (1 << 25) /* Bit 25: Error interrupt enable */
#define FLASH_CR_RDERRIE            (1 << 26) /* Bit 26: PCROP read error interrupt enable */
#define FLASH_CR_OBL_LAUNCH         (1 << 27) /* Bit 27: Force option byte loading */
#define FLASH_CR_OPTLOCK            (1 << 30) /* Bit 30: Options Lock */
#define FLASH_CR_LOCK               (1 << 31) /* Bit 31: FLASH_CR register Lock */

/* Flash ECC Register (ECCR) */

#define FLASH_ECCR_ADDR_ECC_SHIFT   (0)       /* Bits 0-16: ECC fail address */
#define FLASH_ECCR_ADDR_ECC_MASK    (0x1ffff << FLASH_ECCR_ADDR_ECC_SHIFT)
#define FLASH_ECCR_SYSF_ECC         (1 << 20) /* Bit 20: System Flash ECC fail */
#define FLASH_ECCR_ECCCIE           (1 << 24) /* Bit 24: ECC correction interrupt enable */
#define FLASH_ECCR_CPUID_SHIFT      (26)      /* Bits 26-28: CPU identification */
#define FLASH_ECCR_CPUID_MASK       (0x7 << FLASH_ECCR_CPUID_SHIFT)
#define FLASH_ECCR_ECCC             (1 << 30) /* Bit 30: ECC correction */
#define FLASH_ECCR_ECCD             (1 << 31) /* Bit 31: ECC detection */

/* Flash Option Register (OPTR) */

#define FLASH_OPTR_RDP_SHIFT       (0)        /* Bits 0-7: Read protection level  */
#define FLASH_OPTR_RDP_MASK        (0xff << FLASH_OPTR_RDP_SHIFT)
#  define FLASH_OPTR_RDPLEV0       (0xaa << FLASH_OPTR_RDP_SHIFT) /* 0xAA: Level 0, read protection not active */
#  define FLASH_OPTR_RDPLEV2       (0xcc << FLASH_OPTR_RDP_SHIFT) /* 0xCC: Level 2, chip read protection active */
#  define FLASH_OPTR_RDPLEV1       (0x01 << FLASH_OPTR_RDP_SHIFT) /* Others: Level 1, memories read protection active */

#define FLASH_OPTR_ESE             (1 << 8)   /* Bit 8: System security enabled flag */
#define FLASH_OPTR_BORLEV_SHIFT    (9)        /* Bits 9-11: BOR level reset */
#define FLASH_OPTR_BORLEV_MASK     (0x7 << FLASH_OPTR_BORLEV_SHIFT)
#  define FLASH_OPTR_BORLEV0       (0x0 << FLASH_OPTR_BORLEV_SHIFT) /* 000: BOR Level 0 (1.7 V) */
#  define FLASH_OPTR_BORLEV1       (0x1 << FLASH_OPTR_BORLEV_SHIFT) /* 001: BOR Level 1 (2.0 V) */
#  define FLASH_OPTR_BORLEV2       (0x2 << FLASH_OPTR_BORLEV_SHIFT) /* 010: BOR Level 2 (2.2 V) */
#  define FLASH_OPTR_BORLEV3       (0x3 << FLASH_OPTR_BORLEV_SHIFT) /* 011: BOR Level 3 (2.5 V) */
#  define FLASH_OPTR_BORLEV4       (0x4 << FLASH_OPTR_BORLEV_SHIFT) /* 100: BOR Level 4 (2.8 V) */

#define FLASH_OPTR_NRST_STOP       (1 << 12)  /* Bit 12: Not generate reset when entering the Stop mode */
#define FLASH_OPTR_NRST_STDBY      (1 << 13)  /* Bit 13: Not generate reset when entering the Standby mode */
#define FLASH_OPTR_NRST_SHDW       (1 << 14)  /* Bit 14: Not generate reset when entering the Shutdown mode */
#define FLASH_OPTR_IWDG_SW         (1 << 16)  /* Bit 16: Independent watchdog selection */
#define FLASH_OPTR_IWDG_STOP       (1 << 17)  /* Bit 17: Independent watchdog counter freeze in Stop mode */
#define FLASH_OPTR_IWDG_STDBY      (1 << 18)  /* Bit 18: Independent watchdog counter freeze in Standby mode*/
#define FLASH_OPTR_WWDG_SW         (1 << 19)  /* Bit 19: Window watchdog selection */
#define FLASH_OPTR_NBOOT1          (1 << 23)  /* Bit 23: Boot configuration */
#define FLASH_OPTR_SRAM2_PE        (1 << 24)  /* Bit 24: SRAM2 parity check enable */
#define FLASH_OPTR_SRAM2_RST       (1 << 25)  /* Bit 25: SRAM2 Erase when system reset */
#define FLASH_OPTR_NSWBOOT0        (1 << 26)  /* Bit 26: Software BOOT0 selection */
#define FLASH_OPTR_NBOOT0          (1 << 27)  /* Bit 27: nBOOT0 option bit */
#define FLASH_OPTR_AGC_TRIM_SHIFT  (29)       /* Bits 29-31: Radio automatic gain control trimming */
#define FLASH_OPTR_AGC_TRIM_MASK   (0x7 << FLASH_OPTR_AGC_TRIM_SHIFT)

/* Flash memory PCROP zone A start address register (PCROP1ASR) */

#define FLASH_PCROP1ASR_STRT_SHIFT (0)        /* Bits 0-8: PCROP1A area start offset */
#define FLASH_PCROP1ASR_STRT_MASK  (0x1ff << FLASH_PCROP1ASR_STRT_SHIFT)

/* Flash memory PCROP zone A end address register (PCROP1AER) */

#define FLASH_PCROP1AER_END_SHIFT  (0)        /* Bits 0-8: PCROP1A area end offset */
#define FLASH_PCROP1AER_END_MASK   (0x1ff << FLASH_PCROP1AER_END_SHIFT)
#define FLASH_PCROP1AER_PCROP_RDP  (1 << 31)  /* Bit 31: PCROP area not preserved on RDP erase */

/* Flash memory WRP area A address register (WRP1AR) */

#define FLASH_WRP1AR_STRT_SHIFT    (0)        /* Bits 0-7: WRP first area A start offset */
#define FLASH_WRP1AR_STRT_MASK     (0xff << FLASH_WRP1AR_STRT_SHIFT)
#define FLASH_WRP1AR_END_SHIFT     (16)       /* Bits 16-23: WRP first area A end offset */
#define FLASH_WRP1AR_END_MASK      (0xff << FLASH_WRP1AR_END_SHIFT)

/* Flash memory WRP area B address register (WRP1BR) */

#define FLASH_WRP1BR_STRT_SHIFT    (0)        /* Bits 0-7: WRP second area B start offset */
#define FLASH_WRP1BR_STRT_MASK     (0xff << FLASH_WRP1BR_STRT_SHIFT)
#define FLASH_WRP1BR_END_SHIFT     (16)       /* Bits 16-23: WRP second area B end offset */
#define FLASH_WRP1BR_END_MASK      (0xff << FLASH_WRP1BR_END_SHIFT)

/* Flash memory PCROP zone B start address register (PCROP1BSR) */

#define FLASH_PCROP1BSR_STRT_SHIFT (0)        /* Bits 0-8: PCROP1B area start offset */
#define FLASH_PCROP1BSR_STRT_MASK  (0x1ff << FLASH_PCROP1BSR_STRT_SHIFT)

/* Flash PCROP zone B End Register (PCROP1BER) */

#define FLASH_PCROP1BER_END_SHIFT  (0)        /* Bits 0-8: PCROP1B area end offset */
#define FLASH_PCROP1BER_END_MASK   (0x1ff << FLASH_PCROP1BER_END_SHIFT)

/* Flash IPCC mailbox data Buffer Register (IPCCBR) */

#define FLASH_IPCCBR_IPCCBDA_SHIFT (0)        /* Bits 0-13: IPCC mailbox data buffer base address offset */
#define FLASH_IPCCBR_IPCCBDA_MASK  (0x3fff << FLASH_IPCCBR_IPCCBDA_SHIFT)

/* CPU2 Flash Access Control Register (C2ACR) */

#define FLASH_C2ACR_PRFTEN         (1 << 8)  /* Bit 8: CPU2 Prefetch enable */
#define FLASH_C2ACR_ICEN           (1 << 9)  /* Bit 9: CPU2 Instruction cache enable */
#define FLASH_C2ACR_ICRST          (1 << 11) /* Bit 11: CPU2 Instruction cache reset */
#define FLASH_C2ACR_PES            (1 << 15) /* Bit 15: CPU2 Program/Erase suspend request */

/* CPU2 Flash Status Register (C2SR) */

#define FLASH_C2SR_EOP              (1 << 0)  /* Bit 0: CPU2 End of operation */
#define FLASH_C2SR_OPERR            (1 << 1)  /* Bit 1: CPU2 Operation error */
#define FLASH_C2SR_PROGERR          (1 << 3)  /* Bit 3: CPU2 Programming error */
#define FLASH_C2SR_WRPERR           (1 << 4)  /* Bit 4: CPU2 Write protection error */
#define FLASH_C2SR_PGAERR           (1 << 5)  /* Bit 5: CPU2 Programming alignment error */
#define FLASH_C2SR_SIZERR           (1 << 6)  /* Bit 6: CPU2 Size error */
#define FLASH_C2SR_PGSERR           (1 << 7)  /* Bit 7: CPU2 Programming sequence error */
#define FLASH_C2SR_MISERR           (1 << 8)  /* Bit 8: CPU2 Fast programming data miss error */
#define FLASH_C2SR_FASTERR          (1 << 9)  /* Bit 9: CPU2 Fast programming error */
#define FLASH_C2SR_RDERR            (1 << 14) /* Bit 14: CPU2 PCROP read error */
#define FLASH_C2SR_BSY              (1 << 16) /* Bit 16: CPU2 Busy */
#define FLASH_C2SR_CFGBSY           (1 << 18) /* Bit 18: CPU2 Programming or erase configuration busy */
#define FLASH_C2SR_PESD             (1 << 19) /* Bit 19: CPU2 Programming / erase configuration suspended */

/* CPU2 Flash Control Register (C2CR) */

#define FLASH_C2CR_PG               (1 << 0)  /* Bit 0: CPU2 Programming */
#define FLASH_C2CR_PER              (1 << 1)  /* Bit 1: CPU2 Page Erase */
#define FLASH_C2CR_MER              (1 << 2)  /* Bit 2: CPU2 Mass Erase */
#define FLASH_C2CR_PNB_SHIFT        (3)       /* Bits 3-10: Page number selection */
#define FLASH_C2CR_PNB_MASK         (0xff << FLASH_C2CR_PNB_SHIFT)
#  define FLASH_C2CR_PNB(n)         ((n)  << FLASH_C2CR_PNB_SHIFT) /* Page n, n = 0..255 */

#define FLASH_C2CR_STRT             (1 << 16) /* Bit 16: CPU2 Start Erase */
#define FLASH_C2CR_FSTPG            (1 << 18) /* Bit 18: CPU2 Fast programming */
#define FLASH_C2CR_EOPIE            (1 << 24) /* Bit 24: CPU2 End of operation interrupt enable */
#define FLASH_C2CR_ERRIE            (1 << 25) /* Bit 25: CPU2 Error interrupt enable */
#define FLASH_C2CR_RDERRIE          (1 << 26) /* Bit 26: CPU2 PCROP read error interrupt enable */

/* Secure Flash start address Register (SFR) */

#define FLASH_SFR_SFSA_SHIFT        (0)       /* Bits 0-7: Secure flash start address */
#define FLASH_SFR_SFSA_MASK         (0xff << FLASH_SFR_SFSA_SHIFT)
#define FLASH_SFR_FSD               (1 << 8)  /* Bit 8: Flash security disabled */
#define FLASH_SFR_DDS               (1 << 12) /* Bit 12: Disable CPU2 debug access */

/* SRAM2 start address and CPU2 Reset Vector Register (SRRVR) */

#define FLASH_SRRVR_SBRV_SHIFT      (0)       /* Bits 0-17: CPU2 boot reset vector */
#define FLASH_SRRVR_SBRV_MASK       (0x3ffff << FLASH_SRRVR_SBRV_SHIFT)
#define FLASH_SRRVR_SBRSA_SHIFT     (18)      /* Bits 18-22: Secure backup SRAM2a start address */
#define FLASH_SRRVR_SBRSA_MASK      (0x1f << FLASH_SRRVR_SBRSA_SHIFT)
#define FLASH_SRRVR_BRSD            (1 << 23) /* Bit 23: Backup SRAM2a security disable */
#define FLASH_SRRVR_SNBRSA_SHIFT    (25)      /* Bits 25-29: Secure non-backup SRAM2b start address */
#define FLASH_SRRVR_SNBRSA_MASK     (0x1f << FLASH_SRRVR_SNBRSA_SHIFT)
#define FLASH_SRRVR_NBRSD           (1 << 30) /* Bit 30: Non-backup SRAM2a security disable */
#define FLASH_SRRVR_C2OPT           (1 << 31) /* Bit 31: CPU2 boot reset vector memory selection */
#  define FLASH_SRRVR_C2OPT_SRAM    (0 << 31) /* 0: SBRV offset addresses SRAM1/2, starting from 0x20000000 */
#  define FLASH_SRRVR_C2OPT_FLASH   (1 << 31) /* 1: SBRV offset addresses Flash, starting from 0x08000000 */

#endif /* __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_FLASH_H */
