/****************************************************************************
 * arch/arm/src/stm32u5/hardware/stm32_flash.h
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

#ifndef __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32_FLASH_H
#define __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flash size is known from the chip selection:
 *
 *   When CONFIG_STM32U5_FLASH_OVERRIDE_DEFAULT is set the
 *   CONFIG_STM32U5_FLASH_CONFIG_x selects the default FLASH size based on
 *   the chip part number.  This value can be overridden with
 *   CONFIG_STM32U5_FLASH_OVERRIDE_x
 *
 *   Parts STM32U585 and STM32U575 have 2048Kb of FLASH
 */

#if !defined(CONFIG_STM32U5_FLASH_OVERRIDE_DEFAULT) && \
    !defined(CONFIG_STM32U5_FLASH_OVERRIDE_C) && \
    !defined(CONFIG_STM32U5_FLASH_OVERRIDE_E) && \
    !defined(CONFIG_STM32U5_FLASH_CONFIG_C) && \
    !defined(CONFIG_STM32U5_FLASH_CONFIG_E)
#  define CONFIG_STM32U5_FLASH_OVERRIDE_E
#  warning "Flash size not defined defaulting to 512KiB (E)"
#endif

/* Override of the Flash has been chosen */

#if !defined(CONFIG_STM32U5_FLASH_OVERRIDE_DEFAULT)
#  undef CONFIG_STM32U5_FLASH_CONFIG_C
#  undef CONFIG_STM32U5_FLASH_CONFIG_E
#  if defined(CONFIG_STM32U5_FLASH_OVERRIDE_C)
#    define CONFIG_STM32U5_FLASH_CONFIG_C
#  elif defined(CONFIG_STM32U5_FLASH_OVERRIDE_E)
#    define CONFIG_STM32U5_FLASH_CONFIG_E
#  endif
#endif

/* Define the valid configuration  */

#if defined(CONFIG_STM32U5_FLASH_CONFIG_I) /* 2048 kB */
#  define STM32_FLASH_NPAGES      256
#  define STM32_FLASH_PAGESIZE    8192
#else
#  error "unknown flash configuration!"
#endif

#ifdef STM32_FLASH_PAGESIZE
#  define STM32_FLASH_SIZE        (STM32_FLASH_NPAGES * STM32_FLASH_PAGESIZE)
#endif

/* Register Offsets *********************************************************/

#define STM32_FLASH_ACR_OFFSET           0x0000
#define STM32_FLASH_NSKEYR_OFFSET        0x0008
#define STM32_FLASH_SECKEYR_OFFSET       0x000c
#define STM32_FLASH_OPTKEYR_OFFSET       0x0010
#define STM32_FLASH_PDKEY1R_OFFSET       0x0018
#define STM32_FLASH_PDKEY2R_OFFSET       0x001c
#define STM32_FLASH_NSSR_OFFSET          0x0020
#define STM32_FLASH_SECSR_OFFSET         0x0024
#define STM32_FLASH_NSCR_OFFSET          0x0028
#define STM32_FLASH_SECCR_OFFSET         0x002c
#define STM32_FLASH_ECCR_OFFSET          0x0030
#define STM32_FLASH_OPSR_OFFSET          0x0034
#define STM32_FLASH_OPTR_OFFSET          0x0040
#define STM32_FLASH_NSBOOTADDR0R_OFFSET  0x0044
#define STM32_FLASH_NSBOOTADDR1R_OFFSET  0x0048
#define STM32_FLASH_SECBOOTADDR0R_OFFSET 0x004c
#define STM32_FLASH_SECWM1R1_OFFSET      0x0050
#define STM32_FLASH_SECWM1R2_OFFSET      0x0054
#define STM32_FLASH_WRP1AR_OFFSET        0x0058
#define STM32_FLASH_WRP1BR_OFFSET        0x005c
#define STM32_FLASH_SECWM2R1_OFFSET      0x0060
#define STM32_FLASH_SECWM2R2_OFFSET      0x0064
#define STM32_FLASH_WRP2AR_OFFSET        0x0068
#define STM32_FLASH_WRP2BR_OFFSET        0x006c
#define STM32_FLASH_OEM1KEYR1_OFFSET     0x0070
#define STM32_FLASH_OEM1KEYR2_OFFSET     0x0074
#define STM32_FLASH_OEM2KEYR1_OFFSET     0x0078
#define STM32_FLASH_OEM2KEYR2_OFFSET     0x007c
#define STM32_FLASH_SECBB1R1_OFFSET      0x0080
#define STM32_FLASH_SECBB1R2_OFFSET      0x0084
#define STM32_FLASH_SECBB1R3_OFFSET      0x0088
#define STM32_FLASH_SECBB1R4_OFFSET      0x008c
#define STM32_FLASH_SECBB2R1_OFFSET      0x00a0
#define STM32_FLASH_SECBB2R2_OFFSET      0x00a4
#define STM32_FLASH_SECBB2R3_OFFSET      0x00a8
#define STM32_FLASH_SECBB2R4_OFFSET      0x00ac
#define STM32_FLASH_SECHDPCR_OFFSET      0x00c0
#define STM32_FLASH_PRIVCFGR_OFFSET      0x00c4
#define STM32_FLASH_PRIVBB1R1_OFFSET     0x00d0
#define STM32_FLASH_PRIVBB1R2_OFFSET     0x00d4
#define STM32_FLASH_PRIVBB1R3_OFFSET     0x00d8
#define STM32_FLASH_PRIVBB1R4_OFFSET     0x00dc
#define STM32_FLASH_PRIVBB2R1_OFFSET     0x00f0
#define STM32_FLASH_PRIVBB2R2_OFFSET     0x00f4
#define STM32_FLASH_PRIVBB2R3_OFFSET     0x00f8
#define STM32_FLASH_PRIVBB2R4_OFFSET     0x00fc

/* Register Addresses *******************************************************/

#define STM32_FLASH_ACR           (STM32_FLASHIF_BASE + STM32_FLASH_ACR_OFFSET)
#define STM32_FLASH_NSKEYR        (STM32_FLASHIF_BASE + STM32_FLASH_NSKEYR_OFFSET)
#define STM32_FLASH_SECKEYR       (STM32_FLASHIF_BASE + STM32_FLASH_SECKEYR_OFFSET)
#define STM32_FLASH_OPTKEYR       (STM32_FLASHIF_BASE + STM32_FLASH_OPTKEYR_OFFSET)
#define STM32_FLASH_PDKEY1R       (STM32_FLASHIF_BASE + STM32_FLASH_PDKEY1R_OFFSET)
#define STM32_FLASH_PDKEY2R       (STM32_FLASHIF_BASE + STM32_FLASH_PDKEY2R_OFFSET)
#define STM32_FLASH_NSSR          (STM32_FLASHIF_BASE + STM32_FLASH_NSSR_OFFSET)
#define STM32_FLASH_SECSR         (STM32_FLASHIF_BASE + STM32_FLASH_SECSR_OFFSET)
#define STM32_FLASH_NSCR          (STM32_FLASHIF_BASE + STM32_FLASH_NSCR_OFFSET)
#define STM32_FLASH_SECCR         (STM32_FLASHIF_BASE + STM32_FLASH_SECCR_OFFSET)
#define STM32_FLASH_ECCR          (STM32_FLASHIF_BASE + STM32_FLASH_ECCR_OFFSET)
#define STM32_FLASH_OPSR          (STM32_FLASHIF_BASE + STM32_FLASH_OPSR_OFFSET)
#define STM32_FLASH_OPTR          (STM32_FLASHIF_BASE + STM32_FLASH_OPTR_OFFSET)
#define STM32_FLASH_NSBOOTADDR0R  (STM32_FLASHIF_BASE + STM32_FLASH_NSBOOTADDR0R_OFFSET)
#define STM32_FLASH_NSBOOTADDR1R  (STM32_FLASHIF_BASE + STM32_FLASH_NSBOOTADDR1R_OFFSET)
#define STM32_FLASH_SECBOOTADDR0R (STM32_FLASHIF_BASE + STM32_FLASH_SECBOOTADDR0R_OFFSET)
#define STM32_FLASH_SECWM1R1      (STM32_FLASHIF_BASE + STM32_FLASH_SECWM1R1_OFFSET)
#define STM32_FLASH_SECWM1R2      (STM32_FLASHIF_BASE + STM32_FLASH_SECWM1R2_OFFSET)
#define STM32_FLASH_WRP1AR        (STM32_FLASHIF_BASE + STM32_FLASH_WRP1AR_OFFSET)
#define STM32_FLASH_WRP1BR        (STM32_FLASHIF_BASE + STM32_FLASH_WRP1BR_OFFSET)
#define STM32_FLASH_SECWM2R1      (STM32_FLASHIF_BASE + STM32_FLASH_SECWM2R1_OFFSET)
#define STM32_FLASH_SECWM2R2      (STM32_FLASHIF_BASE + STM32_FLASH_SECWM2R2_OFFSET)
#define STM32_FLASH_WRP2AR        (STM32_FLASHIF_BASE + STM32_FLASH_WRP2AR_OFFSET)
#define STM32_FLASH_WRP2BR        (STM32_FLASHIF_BASE + STM32_FLASH_WRP2BR_OFFSET)
#define STM32_FLASH_OEM1KEYR1     (STM32_FLASHIF_BASE + STM32_FLASH_OEM1KEYR1_OFFSET)
#define STM32_FLASH_OEM1KEYR2     (STM32_FLASHIF_BASE + STM32_FLASH_OEM1KEYR2_OFFSET)
#define STM32_FLASH_OEM2KEYR1     (STM32_FLASHIF_BASE + STM32_FLASH_OEM2KEYR1_OFFSET)
#define STM32_FLASH_OEM2KEYR2     (STM32_FLASHIF_BASE + STM32_FLASH_OEM2KEYR2_OFFSET)
#define STM32_FLASH_SECBB1R1      (STM32_FLASHIF_BASE + STM32_FLASH_SECBB1R1_OFFSET)
#define STM32_FLASH_SECBB1R2      (STM32_FLASHIF_BASE + STM32_FLASH_SECBB1R2_OFFSET)
#define STM32_FLASH_SECBB1R3      (STM32_FLASHIF_BASE + STM32_FLASH_SECBB1R3_OFFSET)
#define STM32_FLASH_SECBB1R4      (STM32_FLASHIF_BASE + STM32_FLASH_SECBB1R4_OFFSET)
#define STM32_FLASH_SECBB2R1      (STM32_FLASHIF_BASE + STM32_FLASH_SECBB2R1_OFFSET)
#define STM32_FLASH_SECBB2R2      (STM32_FLASHIF_BASE + STM32_FLASH_SECBB2R2_OFFSET)
#define STM32_FLASH_SECBB2R3      (STM32_FLASHIF_BASE + STM32_FLASH_SECBB2R3_OFFSET)
#define STM32_FLASH_SECBB2R4      (STM32_FLASHIF_BASE + STM32_FLASH_SECBB2R4_OFFSET)
#define STM32_FLASH_SECHDPCR      (STM32_FLASHIF_BASE + STM32_FLASH_SECHDPCR_OFFSET)
#define STM32_FLASH_PRIVCFGR      (STM32_FLASHIF_BASE + STM32_FLASH_PRIVCFGR_OFFSET)
#define STM32_FLASH_PRIVBB1R1     (STM32_FLASHIF_BASE + STM32_FLASH_PRIVBB1R1_OFFSET)
#define STM32_FLASH_PRIVBB1R2     (STM32_FLASHIF_BASE + STM32_FLASH_PRIVBB1R2_OFFSET)
#define STM32_FLASH_PRIVBB1R3     (STM32_FLASHIF_BASE + STM32_FLASH_PRIVBB1R3_OFFSET)
#define STM32_FLASH_PRIVBB1R4     (STM32_FLASHIF_BASE + STM32_FLASH_PRIVBB1R4_OFFSET)
#define STM32_FLASH_PRIVBB2R1     (STM32_FLASHIF_BASE + STM32_FLASH_PRIVBB2R1_OFFSET)
#define STM32_FLASH_PRIVBB2R2     (STM32_FLASHIF_BASE + STM32_FLASH_PRIVBB2R2_OFFSET)
#define STM32_FLASH_PRIVBB2R3     (STM32_FLASHIF_BASE + STM32_FLASH_PRIVBB2R3_OFFSET)
#define STM32_FLASH_PRIVBB2R4     (STM32_FLASHIF_BASE + STM32_FLASH_PRIVBB2R4_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Flash Access Control Register (ACR) */

#define FLASH_ACR_LATENCY_SHIFT   (0)
#define FLASH_ACR_LATENCY_MASK    (0xF << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY(n)    ((n) << FLASH_ACR_LATENCY_SHIFT)  /* Latency */
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
#define FLASH_ACR_PRFTEN          (1 << 8)                          /* Bit  8: Prefetch enable */
#define FLASH_ACR_LPRM            (1 << 11)                         /* Bit 11: Low-power read mode */
#define FLASH_ACR_PDREQ1          (1 << 12)                         /* Bit 12: Bank 1 power-down mode request */
#define FLASH_ACR_PDREQ2          (1 << 13)                         /* Bit 13: Bank 2 power-down mode request */
#define FLASH_ACR_SLEEP_PD        (1 << 14)                         /* Bit 14: Flash mode during Sleep  */

/* Flash non-secure status register (NSSR) */

#define FLASH_NSSR_EOP            (1 << 0)  /* Bit 0:  Non-secure end of operation */
#define FLASH_NSSR_OPERR          (1 << 1)  /* Bit 1:  Non-secure peration error */
#define FLASH_NSSR_PROGERR        (1 << 3)  /* Bit 3:  Non-secure programming error */
#define FLASH_NSSR_WRPERR         (1 << 4)  /* Bit 4:  Non-secure write protection error */
#define FLASH_NSSR_PGAERR         (1 << 5)  /* Bit 5:  Non-secure rogramming alignment error */
#define FLASH_NSSR_SIZERR         (1 << 6)  /* Bit 6:  Non-secure size error */
#define FLASH_NSSR_PGSERR         (1 << 7)  /* Bit 7:  Non-secure programming sequence error */
#define FLASH_NSSR_OPTWERR        (1 << 13) /* Bit 13: Option write error */
#define FLASH_NSSR_BSY            (1 << 16) /* Bit 16: Non-secure busy */
#define FLASH_NSSR_WDW            (1 << 17) /* Bit 17: Non-secure wait data to write */
#define FLASH_NSSR_OEM1LOCK       (1 << 18) /* Bit 18: OEM1 lock */
#define FLASH_NSSR_OEM2LOCK       (1 << 19) /* Bit 19: OEM2 lock */
#define FLASH_NSSR_PD1            (1 << 20) /* Bit 20: Bank 1 in power-down mode */
#define FLASH_NSSR_PD2            (1 << 21) /* Bit 21: Bank 2 in power-down mode */

/* Flash non-secure control register (NSCR) */

#define FLASH_NSCR_PG             (1 << 0)                       /* Bit 0 : Non-secure programming */
#define FLASH_NSCR_PER            (1 << 1)                       /* Bit 1 : Non-secure page Erase */
#define FLASH_NSCR_MER1           (1 << 2)                       /* Bit 2 : Non-secure bank 1 mass erase */
#define FLASH_NSCR_PNB_SHIFT      (3)                            /* Bits 3-9: Non-secure page number selection */
#define FLASH_NSCR_PNB_MASK       (0x7F << FLASH_NSCR_PNB_SHIFT)
#define FLASH_NSCR_PNB(n)         ((n)  << FLASH_NSCR_PNB_SHIFT) /* Page n, n = 0..127 */
#define FLASH_NSCR_BKER           (1 << 11)                      /* Bit 11: Non-secure bank selection for page erase */
#define FLASH_NSCR_BWR            (1 << 14)                      /* Bit 14: Non-secure burst write programming mode */ 
#define FLASH_NSCR_MER2           (1 << 15)                      /* Bit 15: Non-secure bank 2 mass erase */
#define FLASH_NSCR_STRT           (1 << 16)                      /* Bit 16: Non-secure start */
#define FLASH_NSCR_OPTSTRT        (1 << 17)                      /* Bit 17: Options modification start */
#define FLASH_NSCR_EOPIE          (1 << 24)                      /* Bit 24: Non-secure end of operation interrupt enable */
#define FLASH_NSCR_ERRIE          (1 << 25)                      /* Bit 25: Non-secure error interrupt enable */
#define FLASH_NSCR_OBL_LAUNCH     (1 << 27)                      /* Bit 27: Force the option byte loading */
#define FLASH_NSCR_OPTLOCK        (1 << 30)                      /* Bit 30: Option Lock */
#define FLASH_NSCR_LOCK           (1 << 31)                      /* Bit 31: Non-secure lock */

/* Flash option register (OPTR) */

#define FLASH_OPTR_RDP_SHIFT       0                                 /* Bits 0-7: Readout protection level */
#define FLASH_OPTR_RDP_MASK        (0xff << FLASH_OPTR_RDP_SHIFT)
#define FLASH_OPTR_RDP_LEVEL_0     (0xaa << FLASH_OPTR_RDP_SHIFT)    /* 0xAA: Level 0 (readout protection not active) */
#define FLASH_OPTR_RDP_LEVEL_0_5   (0x55 << FLASH_OPTR_RDP_SHIFT)    /* 0x55: Level 0.5 (readout protection not active, only non-secure debug access is possible) */
#define FLASH_OPTR_RDP_LEVEL_1     (0x11 << FLASH_OPTR_RDP_SHIFT)    /* Others: Level 1 (memories readout protection active) */
#define FLASH_OPTR_RDP_LEVEL_2     (0xCC << FLASH_OPTR_RDP_SHIFT)    /* 0xCC: Level 2 (chip readout protection active) */
#define FLASH_OPTR_BOR_LEVEL_SHIFT 8                                 /* Bits 8-10: BOR reset level */
#define FLASH_OPTR_BOR_LEVEL_MASK  (7 << FLASH_OPTR_BOR_LEVEL_SHIFT)
#define FLASH_OPTR_BOR_LEVEL_0     (0 << FLASH_OPTR_BOR_LEVEL_SHIFT) /* 000: BOR level 0 (reset level threshold around 1.7V) */
#define FLASH_OPTR_BOR_LEVEL_1     (1 << FLASH_OPTR_BOR_LEVEL_SHIFT) /* 001: BOR level 1 (reset level threshold around 2.0V) */
#define FLASH_OPTR_BOR_LEVEL_2     (2 << FLASH_OPTR_BOR_LEVEL_SHIFT) /* 010: BOR level 2 (reset level threshold around 2.2V) */
#define FLASH_OPTR_BOR_LEVEL_3     (3 << FLASH_OPTR_BOR_LEVEL_SHIFT) /* 011: BOR level 3 (reset level threshold around 2.5V) */
#define FLASH_OPTR_BOR_LEVEL_4     (4 << FLASH_OPTR_BOR_LEVEL_SHIFT) /* 100: BOR level 4 (reset level threshold around 2.8V) */
#define FLASH_OPTR_NRST_STOP       (1 << 12)                         /* Bit 12: Reset generation in Stop mode */
#define FLASH_OPTR_NRST_STDBY      (1 << 13)                         /* Bit 13: Reset generation in Standby mode */
#define FLASH_OPTR_NRST_SHDW       (1 << 14)                         /* Bit 14: Reset generation in Shutdown mode */
#define FLASH_OPTR_SRAM1345_RST    (1 << 15)                         /* Bit 15: SRAM1, SRAM3, SRAM4 and SRAM4 erase upon system reset */
#define FLASH_OPTR_IWDG_SW         (1 << 16)                         /* Bit 16: Independent watchdog selection */
#define FLASH_OPTR_IWDG_STOP       (1 << 17)                         /* Bit 17: Independent watchdog counter freeze in Stop mode */
#define FLASH_OPTR_IWDG_STDBY      (1 << 18)                         /* Bit 18: Independent watchdog counter freeze in Standby mode*/
#define FLASH_OPTR_WWDG_SW         (1 << 19)                         /* Bit 19: Window watchdog selection */
#define FLASH_OPTR_SWAP_BANK       (1 << 20)                         /* Bit 20: Swap banks */
#define FLASH_OPTR_DUALBANK        (1 << 21)                         /* Bit 21: Dual bank on 1-Mbyte flash memory devices */
#define FLASH_OPTR_BKPRAM_ECC      (1 << 22)                         /* Bit 22: Backup RAM ECC detection and correction enable */
#define FLASH_OPTR_SRAM3_ECC       (1 << 23)                         /* Bit 23: SRAM3 ECC detection and correction enable */
#define FLASH_OPTR_SRAM2_ECC       (1 << 24)                         /* Bit 24: SRAM2 ECC detection and correction enable */
#define FLASH_OPTR_SRAM2_RST       (1 << 25)                         /* Bit 25: SRAM2 Erase when system reset */
#define FLASH_OPTR_NSWBOOT0        (1 << 26)                         /* Bit 26: Software BOOT0 */
#define FLASH_OPTR_NBOOT0          (1 << 27)                         /* Bit 27: nBOOT0 option bit */
#define FLASH_OPTR_PA15_PUPEN      (1 << 28)                         /* Bit 28: PA15 pull-up enable */
#define FLASH_OPTR_IO_VDD_HSLV     (1 << 29)                         /* Bit 29: High-speed IO at low V_DD voltage configuration bit */
#define FLASH_OPTR_IO_VDDIO2_HSLV  (1 << 30)                         /* Bit 30: High-speed IO at low V_DDIO2 voltage configuration bit */
#define FLASH_OPTR_TZEN            (1 << 31)                         /* Bit 31: Global TrustZone security enable */

#if 0

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
#endif

#endif /* __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32_FLASH_H */
