/****************************************************************************
 * arch/arm/src/stm32u0/hardware/stm32u0_flash.h
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

#ifndef __ARCH_ARM_SRC_STM32U0_HARDWARE_STM32U0_FLASH_H
#define __ARCH_ARM_SRC_STM32U0_HARDWARE_STM32U0_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_FLASH_ACR_OFFSET           0x0000 /* Access control register */
#define STM32_FLASH_KEYR_OFFSET          0x0008 /* Key register */
#define STM32_FLASH_OPTKEYR_OFFSET       0x000c /* Option key register */
#define STM32_FLASH_SR_OFFSET            0x0010 /* Status register */
#define STM32_FLASH_CR_OFFSET            0x0014 /* Control register */
#define STM32_FLASH_ECCR_OFFSET          0x0018 /* ECC register */
#define STM32_FLASH_OPTR_OFFSET          0x0020 /* Option register */
#define STM32_FLASH_WRP1AR_OFFSET        0x002c /* WRP area A address reg */
#define STM32_FLASH_WRP1BR_OFFSET        0x0030 /* WRP area B address reg */
#define STM32_FLASH_SECR_OFFSET          0x0080 /* Security register */
#define STM32_FLASH_OEM1KEYW0R_OFFSET    0x0088 /* OEM1 key register 1 */
#define STM32_FLASH_OEM1KEYW1R_OFFSET    0x008c /* OEM1 key register 2 */
#define STM32_FLASH_OEM1KEYW2R_OFFSET    0x0090 /* OEM1 key register 3 */
#define STM32_FLASH_OEM1KEYW3R_OFFSET    0x0094 /* OEM1 key register 4 */
#define STM32_FLASH_OEM2KEYW0R_OFFSET    0x0098 /* OEM2 key register 1 */
#define STM32_FLASH_OEM2KEYW1R_OFFSET    0x009c /* OEM2 key register 2 */
#define STM32_FLASH_OEM2KEYW2R_OFFSET    0x00a0 /* OEM2 key register 3 */
#define STM32_FLASH_OEM2KEYW3R_OFFSET    0x00a4 /* OEM2 key register 4 */
#define STM32_FLASH_OEMKEYSR_OFFSET      0x00a8 /* OEM key status register */
#define STM32_FLASH_HDPCR_OFFSET         0x00ac /* HDP control register */
#define STM32_FLASH_HDPEXTR_OFFSET       0x00b0 /* HDP extension register */

/* Register Addresses *******************************************************/

#define STM32_FLASH_ACR                  (STM32_FLASHIF_BASE+STM32_FLASH_ACR_OFFSET)
#define STM32_FLASH_KEYR                 (STM32_FLASHIF_BASE+STM32_FLASH_KEYR_OFFSET)
#define STM32_FLASH_OPTKEYR              (STM32_FLASHIF_BASE+STM32_FLASH_OPTKEYR_OFFSET)
#define STM32_FLASH_SR                   (STM32_FLASHIF_BASE+STM32_FLASH_SR_OFFSET)
#define STM32_FLASH_CR                   (STM32_FLASHIF_BASE+STM32_FLASH_CR_OFFSET)
#define STM32_FLASH_ECCR                 (STM32_FLASHIF_BASE+STM32_FLASH_ECCR_OFFSET)
#define STM32_FLASH_OPTR                 (STM32_FLASHIF_BASE+STM32_FLASH_OPTR_OFFSET)
#define STM32_FLASH_WRP1AR               (STM32_FLASHIF_BASE+STM32_FLASH_WRP1AR_OFFSET)
#define STM32_FLASH_WRP1BR               (STM32_FLASHIF_BASE+STM32_FLASH_WRP1BR_OFFSET)
#define STM32_FLASH_SECR                 (STM32_FLASHIF_BASE+STM32_FLASH_SECR_OFFSET)
#define STM32_FLASH_OEM1KEYW0R           (STM32_FLASHIF_BASE+STM32_FLASH_OEM1KEYW0R_OFFSET)
#define STM32_FLASH_OEM1KEYW1R           (STM32_FLASHIF_BASE+STM32_FLASH_OEM1KEYW1R_OFFSET)
#define STM32_FLASH_OEM1KEYW2R           (STM32_FLASHIF_BASE+STM32_FLASH_OEM1KEYW2R_OFFSET)
#define STM32_FLASH_OEM1KEYW3R           (STM32_FLASHIF_BASE+STM32_FLASH_OEM1KEYW3R_OFFSET)
#define STM32_FLASH_OEM2KEYW0R           (STM32_FLASHIF_BASE+STM32_FLASH_OEM2KEYW0R_OFFSET)
#define STM32_FLASH_OEM2KEYW1R           (STM32_FLASHIF_BASE+STM32_FLASH_OEM2KEYW1R_OFFSET)
#define STM32_FLASH_OEM2KEYW2R           (STM32_FLASHIF_BASE+STM32_FLASH_OEM2KEYW2R_OFFSET)
#define STM32_FLASH_OEM2KEYW3R           (STM32_FLASHIF_BASE+STM32_FLASH_OEM2KEYW3R_OFFSET)
#define STM32_FLASH_OEMKEYSR             (STM32_FLASHIF_BASE+STM32_FLASH_OEMKEYSR_OFFSET)
#define STM32_FLASH_HDPCR                (STM32_FLASHIF_BASE+STM32_FLASH_HDPCR_OFFSET)
#define STM32_FLASH_HDPEXTR              (STM32_FLASHIF_BASE+STM32_FLASH_HDPEXTR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Flash Access Control Register (ACR) */

#define FLASH_ACR_LATENCY_SHIFT          (0) /* Bits 0-2: Flash memory access latency */
#define FLASH_ACR_LATENCY_MASK           (7 << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY(n)           ((n) << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY_0            (0 << FLASH_ACR_LATENCY_SHIFT) /* 000: Zero wait states */
#  define FLASH_ACR_LATENCY_1            (1 << FLASH_ACR_LATENCY_SHIFT) /* 001: One wait state */
#  define FLASH_ACR_LATENCY_2            (2 << FLASH_ACR_LATENCY_SHIFT) /* 010: Two wait states */
                                                                        /* Bits 3-7: Reserved */
#define FLASH_ACR_PRFTEN                 (1 << 8)                       /* Bit 8: Prefetch enable */
#define FLASH_ACR_ICEN                   (1 << 9)                       /* Bit 9: Instruction cache enable */
                                                                        /* Bit 10: Reserved */
#define FLASH_ACR_ICRST                  (1 << 11)                      /* Bit 11: Instruction cache reset */
                                                                        /* Bits 12-15: Reserved */
#define FLASH_ACR_EMPTY                  (1 << 16)                      /* Bit 16: Main Flash memory area empty */
                                                                        /* Bit 17: Reserved */
#define FLASH_ACR_DBGSWEN                (1 << 18)                      /* Bit 18: Debug access software enable */
                                                                        /* Bits 19-31: Reserved */

/* Flash Status Register (SR) */

#define FLASH_SR_EOP                     (1 << 0)  /* Bit 0: End of operation */
#define FLASH_SR_OPERR                   (1 << 1)  /* Bit 1: Operation error */
                                                   /* Bit 2: Reserved */
#define FLASH_SR_PROGERR                 (1 << 3)  /* Bit 3: Programming error */
#define FLASH_SR_WRPERR                  (1 << 4)  /* Bit 4: Write protection error */
#define FLASH_SR_PGAERR                  (1 << 5)  /* Bit 5: Programming alignment error */
#define FLASH_SR_SIZERR                  (1 << 6)  /* Bit 6: Size error */
#define FLASH_SR_PGSERR                  (1 << 7)  /* Bit 7: Programming sequence error */
#define FLASH_SR_MISSERR                 (1 << 8)  /* Bit 8: Fast programming data miss error */
#define FLASH_SR_FASTERR                 (1 << 9)  /* Bit 9: Fast programming error */
                                                   /* Bit 10: Reserved */
#define FLASH_SR_HDPOPTWERR              (1 << 11) /* Bit 11: HDP option bytes write error */
#define FLASH_SR_OEMOPTWERR              (1 << 12) /* Bit 12: OEM option bytes write error */
                                                   /* Bits 13-14: Reserved */
#define FLASH_SR_OPTVERR                 (1 << 15) /* Bit 15: Option validity error */
#define FLASH_SR_BSY1                    (1 << 16) /* Bit 16: Busy */
                                                   /* Bit 17: Reserved */
#define FLASH_SR_CFGBSY                  (1 << 18) /* Bit 18: Programming or erase configuration busy */
                                                   /* Bit 19: Reserved */
#define FLASH_SR_OEM1LOCK                (1 << 20) /* Bit 20: OEM1 lock */
#define FLASH_SR_OEM2LOCK                (1 << 21) /* Bit 21: OEM2 lock */
                                                   /* Bits 22-31: Reserved */

/* Flash Control Register (CR) */

#define FLASH_CR_PG                      (1 << 0) /* Bit 0: Programming enable */
#define FLASH_CR_PER                     (1 << 1) /* Bit 1: Page erase enable */
#define FLASH_CR_MER1                    (1 << 2) /* Bit 2: Mass erase */
#define FLASH_CR_PNB_SHIFT               (3)      /* Bits 3-9: Page number */
#define FLASH_CR_PNB_MASK                (0x7f << FLASH_CR_PNB_SHIFT)
#  define FLASH_CR_PNB(n)                ((n) << FLASH_CR_PNB_SHIFT)
                                                   /* Bits 10-15: Reserved */
#define FLASH_CR_STRT                    (1 << 16) /* Bit 16: Start erase operation */
#define FLASH_CR_OPTSTRT                 (1 << 17) /* Bit 17: Start of modification of option bytes */
#define FLASH_CR_FSTPG                   (1 << 18) /* Bit 18: Fast programming enable */
                                                   /* Bits 19-23: Reserved */
#define FLASH_CR_EOPIE                   (1 << 24) /* Bit 24: End-of-operation interrupt enable */
#define FLASH_CR_ERRIE                   (1 << 25) /* Bit 25: Error interrupt enable */
                                                   /* Bit 26: Reserved */
#define FLASH_CR_OBL_LAUNCH              (1 << 27) /* Bit 27: Option byte load launch */
                                                   /* Bits 28-29: Reserved */
#define FLASH_CR_OPTLOCK                 (1 << 30) /* Bit 30: Options lock */
#define FLASH_CR_LOCK                    (1 << 31) /* Bit 31: FLASH_CR lock */

/* Flash ECC register (ECCR) */

#define FLASH_ECCR_ADDR_ECC_SHIFT        (0) /* Bits 0-13: ECC fail double-word address offset */
#define FLASH_ECCR_ADDR_ECC_MASK         (0x3fff << FLASH_ECCR_ADDR_ECC_SHIFT)
                                                   /* Bits 14-19: Reserved */
#define FLASH_ECCR_SYSF_ECC              (1 << 20) /* Bit 20: System flash memory ECC fail */
                                                   /* Bits 21-23: Reserved */
#define FLASH_ECCR_ECCCIE                (1 << 24) /* Bit 24: ECC correction interrupt enable */
                                                   /* Bits 25-29: Reserved */
#define FLASH_ECCR_ECCC                  (1 << 30) /* Bit 30: ECC correction */
#define FLASH_ECCR_ECCD                  (1 << 31) /* Bit 31: ECC detection */

/* Flash Option Register (OPTR) */

#define FLASH_OPTR_RDP_SHIFT             (0) /* Bits 0-7: Read protection level */
#define FLASH_OPTR_RDP_MASK              (0xff << FLASH_OPTR_RDP_SHIFT)
#define FLASH_OPTR_BOR_EN                (1 << 8) /* Bit 8: BOR enable */
#define FLASH_OPTR_BOR_LEV_SHIFT         (9)      /* Bits 9-10: BOR threshold */
#define FLASH_OPTR_BOR_LEV_MASK          (3 << FLASH_OPTR_BOR_LEV_SHIFT)
                                                   /* Bits 11-12: Reserved */
#define FLASH_OPTR_NRST_STOP             (1 << 13) /* Bit 13: Reset in Stop */
#define FLASH_OPTR_NRST_STDBY            (1 << 14) /* Bit 14: Reset in Standby */
#define FLASH_OPTR_NRST_SHDW             (1 << 15) /* Bit 15: Reset in Shutdown */
#define FLASH_OPTR_IWDG_SW               (1 << 16) /* Bit 16: Independent watchdog selection */
#define FLASH_OPTR_IWDG_STOP             (1 << 17) /* Bit 17: IWDG counter freeze in Stop mode */
#define FLASH_OPTR_IWDG_STDBY            (1 << 18) /* Bit 18: IWDG counter freeze in Standby mode */
#define FLASH_OPTR_WWDG_SW               (1 << 19) /* Bit 19: Window watchdog selection */
                                                   /* Bit 20: Reserved */
#define FLASH_OPTR_BDRST                 (1 << 21) /* Bit 21: Backup domain reset */
#define FLASH_OPTR_RAM_PARITY_CHECK      (1 << 22) /* Bit 22: SRAM parity check control */
#define FLASH_OPTR_BKPSRAM_HW_ERASE      (1 << 23) /* Bit 23: Backup SRAM erase on tamper detection */
#define FLASH_OPTR_NBOOT_SEL             (1 << 24) /* Bit 24: Boot0 selection */
#define FLASH_OPTR_NBOOT1                (1 << 25) /* Bit 25: Boot1 config */
#define FLASH_OPTR_NBOOT0                (1 << 26) /* Bit 26: Boot0 option bit */
#define FLASH_OPTR_NRST_MODE_SHIFT       (27)      /* Bits 27-28: NRST mode */
#define FLASH_OPTR_NRST_MODE_MASK        (3 << FLASH_OPTR_NRST_MODE_SHIFT)
#define FLASH_OPTR_IRHEN                 (1 << 29) /* Bit 29: Internal reset holder enable */

/* Flash WRP area A address register (WRP1AR) */

#define FLASH_WRP1AR_WRP1A_STRT_SHIFT    (0) /* Bits 0-15: Area A start */
#define FLASH_WRP1AR_WRP1A_STRT_MASK     (0xffff << FLASH_WRP1AR_WRP1A_STRT_SHIFT)
#define FLASH_WRP1AR_WRP1A_END_SHIFT     (16) /* Bits 16-31: Area A end */
#define FLASH_WRP1AR_WRP1A_END_MASK      (0xffff << FLASH_WRP1AR_WRP1A_END_SHIFT)

/* Flash WRP area B address register (WRP1BR) */

#define FLASH_WRP1BR_WRP1B_STRT_SHIFT    (0) /* Bits 0-15: Area B start */
#define FLASH_WRP1BR_WRP1B_STRT_MASK     (0xffff << FLASH_WRP1BR_WRP1B_STRT_SHIFT)
#define FLASH_WRP1BR_WRP1B_END_SHIFT     (16) /* Bits 16-31: Area B end */
#define FLASH_WRP1BR_WRP1B_END_MASK      (0xffff << FLASH_WRP1BR_WRP1B_END_SHIFT)

/* Flash Security register (SECR) */

#define FLASH_SECR_HDP1_PEND_SHIFT       (0) /* Bits 0-5: HDP1 area end page */
#define FLASH_SECR_HDP1_PEND_MASK        (0x3f << FLASH_SECR_HDP1_PEND_SHIFT)
                                                   /* Bits 6-15: Reserved */
#define FLASH_SECR_BOOT_LOCK             (1 << 16) /* Bit 16: Force boot from user area */
                                                   /* Bits 17-23: Reserved */
#define FLASH_SECR_HDP1EN_SHIFT          (24)      /* Bits 24-31: HDP1 area enable */
#define FLASH_SECR_HDP1EN_MASK           (0xff << FLASH_SECR_HDP1EN_SHIFT)

/* Flash OEM key status register (OEMKEYSR) */

#define FLASH_OEMKEYSR_OEM1KEYCRC_SHIFT  (0) /* Bits 0-7: OEM1 key CRC */
#define FLASH_OEMKEYSR_OEM1KEYCRC_MASK   (0xff << FLASH_OEMKEYSR_OEM1KEYCRC_SHIFT)
                                              /* Bits 8-15: Reserved */
#define FLASH_OEMKEYSR_OEM2KEYCRC_SHIFT  (16) /* Bits 16-23: OEM2 key CRC */
#define FLASH_OEMKEYSR_OEM2KEYCRC_MASK   (0xff << FLASH_OEMKEYSR_OEM2KEYCRC_SHIFT)
                                             /* Bits 24-31: Reserved */

/* Flash HDP control register (HDPCR) */

#define FLASH_HDPCR_HDP1_ACCDIS_SHIFT    (0) /* Bits 0-7: HDP1 access disable */
#define FLASH_HDPCR_HDP1_ACCDIS_MASK     (0xff << FLASH_HDPCR_HDP1_ACCDIS_SHIFT)
                                              /* Bits 8-15: Reserved */
#define FLASH_HDPCR_HDP1EXT_ACCDIS_SHIFT (16) /* Bits 16-23: HDP1 extension access disable */
#define FLASH_HDPCR_HDP1EXT_ACCDIS_MASK  (0xff << FLASH_HDPCR_HDP1EXT_ACCDIS_SHIFT)
                                              /* Bits 24-31: Reserved */

/* Flash HDP extension register (HDPEXTR) */

#define FLASH_HDPEXTR_HDP1_EXT_SHIFT     (0) /* Bits 0-5: HDP1 extension area size */
#define FLASH_HDPEXTR_HDP1_EXT_MASK      (0x3f << FLASH_HDPEXTR_HDP1_EXT_SHIFT)
                                         /* Bits 6-31: Reserved */

#endif /* __ARCH_ARM_SRC_STM32U0_HARDWARE_STM32U0_FLASH_H */
