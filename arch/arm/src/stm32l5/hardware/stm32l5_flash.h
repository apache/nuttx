/****************************************************************************
 * arch/arm/src/stm32l5/hardware/stm32l5_flash.h
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

#ifndef __ARCH_ARM_SRC_STM32L5_HARDWARE_STM32L5_FLASH_H
#define __ARCH_ARM_SRC_STM32L5_HARDWARE_STM32L5_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flash size is known from the chip selection:
 *
 *   When CONFIG_STM32L5_FLASH_OVERRIDE_DEFAULT is set the
 *   CONFIG_STM32L5_FLASH_CONFIG_x selects the default FLASH size based on
 *   the chip part number.  This value can be overridden with
 *   CONFIG_STM32L5_FLASH_OVERRIDE_x
 *
 *   Parts STM32L552xC and STM32L562xC have 256Kb of FLASH
 *   Parts STM32L552xE and STM32L562xE have 512Kb of FLASH
 *
 *   N.B. Only Single bank mode is supported
 */

#if !defined(CONFIG_STM32L5_FLASH_OVERRIDE_DEFAULT) && \
    !defined(CONFIG_STM32L5_FLASH_OVERRIDE_C) && \
    !defined(CONFIG_STM32L5_FLASH_OVERRIDE_E) && \
    !defined(CONFIG_STM32L5_FLASH_CONFIG_C) && \
    !defined(CONFIG_STM32L5_FLASH_CONFIG_E)
#  define CONFIG_STM32L5_FLASH_OVERRIDE_E
#  warning "Flash size not defined defaulting to 512KiB (E)"
#endif

/* Override of the Flash has been chosen */

#if !defined(CONFIG_STM32L5_FLASH_OVERRIDE_DEFAULT)
#  undef CONFIG_STM32L5_FLASH_CONFIG_C
#  undef CONFIG_STM32L5_FLASH_CONFIG_E
#  if defined(CONFIG_STM32L5_FLASH_OVERRIDE_C)
#    define CONFIG_STM32L5_FLASH_CONFIG_C
#  elif defined(CONFIG_STM32L5_FLASH_OVERRIDE_E)
#    define CONFIG_STM32L5_FLASH_CONFIG_E
#  endif
#endif

/* Define the valid configuration  */

#if defined(CONFIG_STM32L5_FLASH_CONFIG_C) /* 256 kB */
#  define STM32L5_FLASH_NPAGES      64
#  define STM32L5_FLASH_PAGESIZE    4096
#elif defined(CONFIG_STM32L5_FLASH_CONFIG_E) /* 512 kB */
#  define STM32L5_FLASH_NPAGES      128
#  define STM32L5_FLASH_PAGESIZE    4096
#else
#  error "unknown flash configuration!"
#endif

#ifdef STM32L5_FLASH_PAGESIZE
#  define STM32L5_FLASH_SIZE        (STM32L5_FLASH_NPAGES * STM32L5_FLASH_PAGESIZE)
#endif

/* Register Offsets *********************************************************/

#define STM32L5_FLASH_ACR_OFFSET           0x0000
#define STM32L5_FLASH_PDKEYR_OFFSET        0x0004
#define STM32L5_FLASH_NSKEYR_OFFSET        0x0008
#define STM32L5_FLASH_SECKEYR_OFFSET       0x000c
#define STM32L5_FLASH_OPTKEYR_OFFSET       0x0010
#define STM32L5_FLASH_LVEKEYR_OFFSET       0x0014
#define STM32L5_FLASH_NSSR_OFFSET          0x0020
#define STM32L5_FLASH_SECSR_OFFSET         0x0024
#define STM32L5_FLASH_NSCR_OFFSET          0x0028
#define STM32L5_FLASH_SECCR_OFFSET         0x002c
#define STM32L5_FLASH_ECCR_OFFSET          0x0030
#define STM32L5_FLASH_OPTR_OFFSET          0x0040
#define STM32L5_FLASH_NSBOOTADDR0R_OFFSET  0x0044
#define STM32L5_FLASH_NSBOOTADDR1R_OFFSET  0x0048
#define STM32L5_FLASH_SECBOOTADDR0R_OFFSET 0x004c
#define STM32L5_FLASH_SECWM1R1_OFFSET      0x0050
#define STM32L5_FLASH_SECWM1R2_OFFSET      0x0054
#define STM32L5_FLASH_WRP1AR_OFFSET        0x0058
#define STM32L5_FLASH_WRP1BR_OFFSET        0x005c
#define STM32L5_FLASH_SECWM2R1_OFFSET      0x0060
#define STM32L5_FLASH_SECWM2R2_OFFSET      0x0064
#define STM32L5_FLASH_WRP2AR_OFFSET        0x0068
#define STM32L5_FLASH_WRP2BR_OFFSET        0x006c
#define STM32L5_FLASH_SECBB1R1_OFFSET      0x0080
#define STM32L5_FLASH_SECBB1R2_OFFSET      0x0084
#define STM32L5_FLASH_SECBB1R3_OFFSET      0x0088
#define STM32L5_FLASH_SECBB1R4_OFFSET      0x008c
#define STM32L5_FLASH_SECBB2R1_OFFSET      0x00a0
#define STM32L5_FLASH_SECBB2R2_OFFSET      0x00a4
#define STM32L5_FLASH_SECBB2R3_OFFSET      0x00a8
#define STM32L5_FLASH_SECBB2R4_OFFSET      0x00ac
#define STM32L5_FLASH_SECHDPCR_OFFSET      0x00c0
#define STM32L5_FLASH_PRIVCFGR_OFFSET      0x00c4

/* Register Addresses *******************************************************/

#define STM32L5_FLASH_ACR           (STM32L5_FLASHIF_BASE + STM32L5_FLASH_ACR_OFFSET)
#define STM32L5_FLASH_PDKEYR        (STM32L5_FLASHIF_BASE + STM32L5_FLASH_PDKEYR_OFFSET)
#define STM32L5_FLASH_NSKEYR        (STM32L5_FLASHIF_BASE + STM32L5_FLASH_NSKEYR_OFFSET)
#define STM32L5_FLASH_SECKEYR       (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECKEYR_OFFSET)
#define STM32L5_FLASH_OPTKEYR       (STM32L5_FLASHIF_BASE + STM32L5_FLASH_OPTKEYR_OFFSET)
#define STM32L5_FLASH_LVEKEYR       (STM32L5_FLASHIF_BASE + STM32L5_FLASH_LVEKEYR_OFFSET)
#define STM32L5_FLASH_NSSR          (STM32L5_FLASHIF_BASE + STM32L5_FLASH_NSSR_OFFSET)
#define STM32L5_FLASH_SECSR         (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECSR_OFFSET)
#define STM32L5_FLASH_NSCR          (STM32L5_FLASHIF_BASE + STM32L5_FLASH_NSCR_OFFSET)
#define STM32L5_FLASH_SECCR         (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECCR_OFFSET)
#define STM32L5_FLASH_ECCR          (STM32L5_FLASHIF_BASE + STM32L5_FLASH_ECCR_OFFSET)
#define STM32L5_FLASH_OPTR          (STM32L5_FLASHIF_BASE + STM32L5_FLASH_OPTR_OFFSET)
#define STM32L5_FLASH_NSBOOTADDR0R  (STM32L5_FLASHIF_BASE + STM32L5_FLASH_NSBOOTADDR0R_OFFSET)
#define STM32L5_FLASH_NSBOOTADDR1R  (STM32L5_FLASHIF_BASE + STM32L5_FLASH_NSBOOTADDR1R_OFFSET)
#define STM32L5_FLASH_SECBOOTADDR0R (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECBOOTADDR0R_OFFSET)
#define STM32L5_FLASH_SECWM1R1      (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECWM1R1_OFFSET)
#define STM32L5_FLASH_SECWM1R2      (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECWM1R2_OFFSET)
#define STM32L5_FLASH_WRP1AR        (STM32L5_FLASHIF_BASE + STM32L5_FLASH_WRP1AR_OFFSET)
#define STM32L5_FLASH_WRP1BR        (STM32L5_FLASHIF_BASE + STM32L5_FLASH_WRP1BR_OFFSET)
#define STM32L5_FLASH_SECWM2R1      (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECWM2R1_OFFSET)
#define STM32L5_FLASH_SECWM2R2      (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECWM2R2_OFFSET)
#define STM32L5_FLASH_WRP2AR        (STM32L5_FLASHIF_BASE + STM32L5_FLASH_WRP2AR_OFFSET)
#define STM32L5_FLASH_WRP2BR        (STM32L5_FLASHIF_BASE + STM32L5_FLASH_WRP2BR_OFFSET)
#define STM32L5_FLASH_SECBB1R1      (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECBB1R1_OFFSET)
#define STM32L5_FLASH_SECBB1R2      (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECBB1R2_OFFSET)
#define STM32L5_FLASH_SECBB1R3      (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECBB1R3_OFFSET)
#define STM32L5_FLASH_SECBB1R4      (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECBB1R4_OFFSET)
#define STM32L5_FLASH_SECBB2R1      (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECBB2R1_OFFSET)
#define STM32L5_FLASH_SECBB2R2      (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECBB2R2_OFFSET)
#define STM32L5_FLASH_SECBB2R3      (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECBB2R3_OFFSET)
#define STM32L5_FLASH_SECBB2R4      (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECBB2R4_OFFSET)
#define STM32L5_FLASH_SECHDPCR      (STM32L5_FLASHIF_BASE + STM32L5_FLASH_SECHDPCR_OFFSET)
#define STM32L5_FLASH_PRIVCFGR      (STM32L5_FLASHIF_BASE + STM32L5_FLASH_PRIVCFGR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Flash Access Control Register (ACR) */

#define FLASH_ACR_LATENCY_SHIFT   (0)
#define FLASH_ACR_LATENCY_MASK    (0xF << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY(n)    ((n) << FLASH_ACR_LATENCY_SHIFT)  /* n wait states, for Vcore range 1 and 2. */
#  define FLASH_ACR_LATENCY_0     ( 0 << FLASH_ACR_LATENCY_SHIFT)   /* 0000: Zero wait states    */
#  define FLASH_ACR_LATENCY_1     ( 1 << FLASH_ACR_LATENCY_SHIFT)   /* 0001: One wait state      */
#  define FLASH_ACR_LATENCY_2     ( 2 << FLASH_ACR_LATENCY_SHIFT)   /* 0010: Two wait states     */
#  define FLASH_ACR_LATENCY_3     ( 3 << FLASH_ACR_LATENCY_SHIFT)   /* 0011: Three wait states   */
#  define FLASH_ACR_LATENCY_4     ( 4 << FLASH_ACR_LATENCY_SHIFT)   /* 0100: Four wait states    */
#  define FLASH_ACR_LATENCY_5     ( 5 << FLASH_ACR_LATENCY_SHIFT)   /* 0101: Five wait states    */
#  define FLASH_ACR_LATENCY_6     ( 6 << FLASH_ACR_LATENCY_SHIFT)   /* 0110: Six wait states     */
#  define FLASH_ACR_LATENCY_7     ( 7 << FLASH_ACR_LATENCY_SHIFT)   /* 0111: Seven wait state    */
#  define FLASH_ACR_LATENCY_8     ( 8 << FLASH_ACR_LATENCY_SHIFT)   /* 1000: Eight wait states   */
#  define FLASH_ACR_LATENCY_9     ( 9 << FLASH_ACR_LATENCY_SHIFT)   /* 1001: Nine wait states    */
#  define FLASH_ACR_LATENCY_10    (10 << FLASH_ACR_LATENCY_SHIFT)   /* 1010: Ten wait states     */
#  define FLASH_ACR_LATENCY_11    (11 << FLASH_ACR_LATENCY_SHIFT)   /* 1011: Eleven wait states  */
#  define FLASH_ACR_LATENCY_12    (12 << FLASH_ACR_LATENCY_SHIFT)   /* 1100: Twelve wait states  */
#  define FLASH_ACR_LATENCY_13    (13 << FLASH_ACR_LATENCY_SHIFT)   /* 1101: Thirteen wait state */
#  define FLASH_ACR_LATENCY_14    (14 << FLASH_ACR_LATENCY_SHIFT)   /* 1110: Fourteen states     */
#  define FLASH_ACR_LATENCY_15    (15 << FLASH_ACR_LATENCY_SHIFT)   /* 1111: Fifteen wait states */

#define FLASH_ACR_RUN_PD            (1 << 13) /* Bit 13: Flash mode during Run    */
#define FLASH_ACR_SLEEP_PD          (1 << 14) /* Bit 14: Flash mode during Sleep  */
#define FLASH_ACR_LVE               (1 << 15) /* Bit 15: Flash low-voltage enable */

/* Flash Status Register (SR) */

#define FLASH_SR_EOP                (1 << 0)  /* Bit 0:  End of operation */
#define FLASH_SR_OPERR              (1 << 1)  /* Bit 1:  Operation error */
#define FLASH_SR_PROGERR            (1 << 3)  /* Bit 3:  Programming error */
#define FLASH_SR_WRPERR             (1 << 4)  /* Bit 4:  Write protection error */
#define FLASH_SR_PGAERR             (1 << 5)  /* Bit 5:  Programming alignment error */
#define FLASH_SR_SIZERR             (1 << 6)  /* Bit 6:  Size error */
#define FLASH_SR_PGSERR             (1 << 7)  /* Bit 7:  Programming sequence error */
#define FLASH_SR_OPTWERR            (1 << 13) /* Bit 13: Option write error */
#define FLASH_SR_BSY                (1 << 16) /* Bit 16: Busy */

/* Flash Control Register (CR) */

#define FLASH_CR_PG                 (1 << 0)                /* Bit 0 : Program Page */
#define FLASH_CR_PER                (1 << 1)                /* Bit 1 : Page Erase */
#define FLASH_CR_MER1               (1 << 2)                /* Bit 2 : Mass Erase Bank 1 */

#define FLASH_CR_PNB_SHIFT          (3)                     /* Bits 3-9: Page number */
#define FLASH_CR_PNB_MASK           (0x7F << FLASH_CR_PNB_SHIFT)
#define FLASH_CR_PNB(n)             ((n)  << FLASH_CR_PNB_SHIFT) /* Page n (if BKER=0) or n+128 (if BKER=1), n=0..127 */

#define FLASH_CR_BKER               (1 << 11)               /* Bit 11: Page number MSB (Bank selection) */
#define FLASH_CR_MER2               (1 << 15)               /* Bit 15: Mass Erase Bank 2 */
#define FLASH_CR_START              (1 << 16)               /* Bit 16: Start Erase */
#define FLASH_CR_OPTSTRT            (1 << 17)               /* Bit 17: Options modification Start */
#define FLASH_CR_EOPIE              (1 << 24)               /* Bit 24: End of operation interrupt enable */
#define FLASH_CR_ERRIE              (1 << 25)               /* Bit 25: Error interrupt enable */
#define FLASH_CR_OBL_LAUNCH         (1 << 27)               /* Bit 27: Option Byte Loading */
#define FLASH_CR_OPTLOCK            (1 << 30)               /* Bit 30: Option Lock */
#define FLASH_CR_LOCK               (1 << 31)               /* Bit 31: Lock */

/* Flash ECC Register (ECCR) */

#define FLASH_ECCR_ADDR_ECC_SHIFT   (0)                     /* Bits 0-18: ECC fail address */
#define FLASH_ECCR_ADDR_ECC_MASK    (0x07ffff << FLASH_ECCR_ADDR_ECC_SHIFT)
#define FLASH_ECCR_BK_ECC           (1 << 21)               /* Bit 21: ECC fail bank */
#define FLASH_ECCR_SYSF_ECC         (1 << 22)               /* Bit 22: System Flash ECC fail */
#define FLASH_ECCR_ECCCIE           (1 << 24)               /* Bit 24: ECC correction interrupt enable */
#define FLASH_ECCR_ECCC2            (1 << 28)               /* Bit 28: ECC2 correction */
#define FLASH_ECCR_ECCD2            (1 << 29)               /* Bit 29: ECC2 detection */
#define FLASH_ECCR_ECCC             (1 << 30)               /* Bit 30: ECC correction */
#define FLASH_ECCR_ECCD             (1 << 31)               /* Bit 31: ECC detection */

/* Flash Option Register (OPTR) */

#define FLASH_OPTR_NRST_STOP       (1 << 12)               /* Bit 12: Generate reset when entering the Stop mode */
#define FLASH_OPTR_NRST_STDBY      (1 << 13)               /* Bit 13: Generate reset when entering the Standby mode */
#define FLASH_OPTR_NRST_SHDW       (1 << 14)               /* Bit 14: Generate reset when entering the Shutdown mode */
#define FLASH_OPTR_IWDG_SW         (1 << 16)               /* Bit 16: Independent watchdog selection */
#define FLASH_OPTR_IWDG_STOP       (1 << 17)               /* Bit 17: Independent watchdog counter freeze in Stop mode */
#define FLASH_OPTR_IWDG_STDBY      (1 << 18)               /* Bit 18: Independent watchdog counter freeze in Standby mode*/
#define FLASH_OPTR_WWDG_SW         (1 << 19)               /* Bit 19: Window watchdog selection */
#define FLASH_OPTR_SWAP_BANK       (1 << 20)               /* Bit 20: Swap banks */
#define FLASH_OPTR_DB256K          (1 << 21)               /* Bit 21: Dual bank on 256K flash memory devices */
#define FLASH_OPTR_DBANK           (1 << 22)               /* Bit 22: Dual bank enable */
#define FLASH_OPTR_SRAM2_PE        (1 << 24)               /* Bit 24: SRAM2 parity check enable */
#define FLASH_OPTR_SRAM2_RST       (1 << 25)               /* Bit 25: SRAM2 Erase when system reset */
#define FLASH_OPTR_NSWBOOT0        (1 << 26)               /* Bit 26: Software BOOT0 */
#define FLASH_OPTR_NBOOT0          (1 << 27)               /* Bit 27: nBOOT0 option bit */
#define FLASH_OPTR_PA15_PUPEN      (1 << 28)               /* Bit 28: PA15 pull-up enable */
#define FLASH_OPTR_TZEN            (1 << 31)               /* Bit 31: Global TrustZone security enable */

#define FLASH_OPTR_BORLEV_SHIFT    (8)                     /* Bits 8-10: BOR reset Level */
#define FLASH_OPTR_BORLEV_MASK     (7 << FLASH_OPTR_BORLEV_SHIFT)
#define FLASH_OPTR_VBOR0           (0 << FLASH_OPTR_BORLEV_SHIFT) /* 000: BOR Level 0 (1.7 V) */
#define FLASH_OPTR_VBOR1           (1 << FLASH_OPTR_BORLEV_SHIFT) /* 001: BOR Level 1 (2.0 V) */
#define FLASH_OPTR_VBOR2           (2 << FLASH_OPTR_BORLEV_SHIFT) /* 010: BOR Level 2 (2.2 V) */
#define FLASH_OPTR_VBOR3           (3 << FLASH_OPTR_BORLEV_SHIFT) /* 011: BOR Level 3 (2.5 V) */
#define FLASH_OPTR_VBOR4           (4 << FLASH_OPTR_BORLEV_SHIFT) /* 100: BOR Level 4 (2.8 V) */
#define FLASH_OPTR_RDP_SHIFT       (0)                            /* Bits 0-7: Read Protection Level */
#define FLASH_OPTR_RDP_MASK        (0xFF << FLASH_OPTR_RDP_SHIFT)
#define FLASH_OPTR_RDP_NONE        (0xAA << FLASH_OPTR_RDP_SHIFT)
#define FLASH_OPTR_RDP_NSDBG       (0x55 << FLASH_OPTR_RDP_SHIFT)
#define FLASH_OPTR_RDP_CHIP        (0xCC << FLASH_OPTR_RDP_SHIFT) /* WARNING, CANNOT BE REVERSED !! */

#endif /* __ARCH_ARM_SRC_STM32L5_HARDWARE_STM32L5_FLASH_H */
