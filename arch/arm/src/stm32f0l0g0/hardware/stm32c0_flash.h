/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32c0_flash.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32C0_FLASH_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32C0_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_FLASH_ACR_OFFSET       0x0000
#define STM32_FLASH_KEYR_OFFSET      0x0008
#define STM32_FLASH_OPTKEYR_OFFSET   0x000c
#define STM32_FLASH_SR_OFFSET        0x0010
#define STM32_FLASH_CR_OFFSET        0x0014
#define STM32_FLASH_OPTR_OFFSET      0x0020
#define STM32_FLASH_PCROP1ASE_OFFSET 0x0024
#define STM32_FLASH_PCROP1AER_OFFSET 0x0028
#define STM32_FLASH_WPR1AR_OFFSET    0x002c
#define STM32_FLASH_WPR1BR_OFFSET    0x0030
#define STM32_FLASH_WPR1BSR_OFFSET   0x0034
#define STM32_FLASH_PCROP1BER_OFFSET 0x0038
#define STM32_FLASH_SECR_OFFSET      0x0080

/* Register Addresses *******************************************************/

#define STM32_FLASH_ACR              (STM32_FLASHIF_BASE+STM32_FLASH_ACR_OFFSET)
#define STM32_FLASH_KEYR             (STM32_FLASHIF_BASE+STM32_FLASH_KEYR_OFFSET)
#define STM32_FLASH_OPTKEYR          (STM32_FLASHIF_BASE+STM32_FLASH_OPTKEYR_OFFSET)
#define STM32_FLASH_SR               (STM32_FLASHIF_BASE+STM32_FLASH_SR_OFFSET)
#define STM32_FLASH_CR               (STM32_FLASHIF_BASE+STM32_FLASH_CR_OFFSET)
#define STM32_FLASH_OPTR             (STM32_FLASHIF_BASE+STM32_FLASH_OPTR_OFFSET)
#define STM32_FLASH_PCROP1ASE        (STM32_FLASHIF_BASE+STM32_FLASH_PCROP1ASE_OFFSET)
#define STM32_FLASH_PCROP1AER        (STM32_FLASHIF_BAER+STM32_FLASH_PCROP1AER_OFFSET)
#define STM32_FLASH_WPR1AR           (STM32_FLASHIF_BASE+STM32_FLASH_WPR1AR_OFFSET)
#define STM32_FLASH_WPR1BR           (STM32_FLASHIF_BASE+STM32_FLASH_WPR1BR_OFFSET)
#define STM32_FLASH_WPR1BSR          (STM32_FLASHIF_BASE+STM32_FLASH_WPR1BSR_OFFSET)
#define STM32_FLASH_PCROP1BER        (STM32_FLASHIF_BASE+STM32_FLASH_PCROP1BER_OFFSET)
#define STM32_FLASH_SECR             (STM32_FLASHIF_BASE+STM32_FLASH_SECR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Flash Access Control Register (ACR) */

#define FLASH_ACR_LATENCY_SHIFT     (0)       /* Bits 0-2: Flash memory access latency*/
#define FLASH_ACR_LATENCY_MASK      (7 << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY(n)      ((n) << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY_0       (0 << FLASH_ACR_LATENCY_SHIFT)    /* 000: Zero wait states  */
#  define FLASH_ACR_LATENCY_1       (1 << FLASH_ACR_LATENCY_SHIFT)    /* 001: One wait state    */

                                              /* Bits 3-7: Reserved */
#define FLASH_ACR_PRFTEN            (1 << 8)  /* Bit 8: Prefetch enable */
#define FLASH_ACR_ICEN              (1 << 9)  /* Bit 9: Instruction cache enable */
                                              /* Bit 10:Reserved */
#define FLASH_ACR_ICRST             (1 << 11) /* Bit 11:Instruction cache reset */
                                              /* Bits 12-15: Reserved */
#define FLASH_ACR_EMPTY             (1 << 16) /* Bit 16: Main Flash memory area empty */
                                              /* Bit 17: Reserved */
#define FLASH_ACR_DBGSWEN           (1 << 18) /* Bit 18: Debug access software enable */
                                              /* Bits 19-31: Reserved */

/* Flash Status Register (SR) */

#define FLASH_SR_EOP                 (1)       /* Bit 0: End of operation */
#define FLASH_SR_OPERR               (1 << 1)  /* Bit 1: Operation error */
                                               /* Bit 2: Reserved */
#define FLASH_SR_PROGERR             (1 << 3)  /* Bit 3: Programming error */
#define FLASH_SR_WRPERR              (1 << 4)  /* Bit 4: Write protection error */
#define FLASH_SR_PGAERR              (1 << 5)  /* Bit 5: Programming alignment error */
#define FLASH_SR_SIZERR              (1 << 6)  /* Bit 6: Size error */
#define FLASH_SR_PGSERR              (1 << 7)  /* Bit 7: Programming sequence error */
#define FLASH_SR_MISSERR             (1 << 8)  /* Bit 8: Fast programming data miss error */
#define FLASH_SR_FASTERR             (1 << 9)  /* Bit 9: Fast programming error */
                                               /* Bits 10-13: Reserved */
#define FLASH_SR_RDERR               (1 << 14) /* Bit 14: PCROP read error */
#define FLASH_SR_OPTVERR             (1 << 15) /* Bit 15: Option and engineering bits loading validity error */
#define FLASH_SR_BSY1                (1 << 16) /* Bit 16: Busy */
                                               /* Bit 17: Reserved */
#define FLASH_SR_CFGBSY              (1 << 18) /* Bit 18: Programming or erase configuration busy */
                                               /* Bits 19-31: Reserved */

/* Flash Control Register (CR) */

#define FLASH_CR_PG                  (1)      /* Bit 0: Flash memory programming enable */
#define FLASH_CR_PER                 (1 << 1) /* Bit 1: Page erase enable */
#define FLASH_CR_MER1                (1 << 2) /* Bit 2: Mass erase */
#define FLASH_CR_PNB_SHIFT           (3)      /* Bits 3-9: Page number selection */
#define FLASH_CR_PNB_MASK            (0x7f << FLASH_CR_PNB_SHIFT)
#  define FLASH_CR_PNB(n)            ((n) << FLASH_CR_PNB_SHIFT)

#define FLASH_CR_STRT                (1 << 16) /* Bit 16: Start erase operation */
#define FLASH_CR_OPTSTRT             (1 << 17) /* Bit 17: Start of modification of option bytes */
#define FLASH_CR_FSTPG               (1 << 18) /* Bit 18: Fast programming enable */
                                               /* Bits 19-23: Reserved */
#define FLASH_CR_EOPIE               (1 << 24) /* Bit 24: End-of-operation interrupt enable */
#define FLASH_CR_ERRIE               (1 << 25) /* Bit 25: Error interrupt enable */
#define FLASH_CR_RDERRIE             (1 << 26) /* Bit 26: PCROP read error interrupt enable */
#define FLASH_CR_OBL_LAUNCH          (1 << 27) /* Bit 27: Option byte load launch */
#define FLASH_CR_SEC_PROT            (1 << 28) /* Bit 28: Securable memory area protection enable */
                                               /* Bit 29: Reserved */
#define FLASH_CR_OPTLOCK             (1 << 30) /* Bit 30: Options Lock */
#define FLASH_CR_LOCK                (1 << 31) /* Bit 31: FLASH_CR Lock */

/* Flash Option Register (OPTR) */

#define FLASH_OPTR_RDP_SHIFT         (0)
#define FLASH_OPTR_RDP_MASK          (0xff << FLASH_OPTR_RDP_SHIFT)
#define FLASH_OPTR_BOR_EN            (1 << 8) /* Brown out reset enable */
#define FLASH_OPTR_BORR_LEV_SHIFT    (9)      /* BOR threshold at rising Vdd supply */
#define FLASH_OPTR_BORR_LEV_MASK     (0x3 << FLASH_OPTR_BORR_LEV_SHIFT)
#define FLASH_OPTR_BORF_LEV_SHIFT    (11)     /* BOR thresholda t falling Vdd supply */
#define FLASH_OPTR_BORF_LEV_MASK     (0x3 << FLASH_OPTR_BORF_LEV_SHIFT)
#define FLASH_OPTR_NRST_STOP         (1 << 13)
#define FLASH_OPTR_NRST_STDBY        (1 << 14)
#define FLASH_OPTR_NRSTS_SHDW        (1 << 15)
#define FLASH_OPTR_IDWG_SW           (1 << 16) /* Bit 16: Independent watchdog selection */
#define FLASH_OPTR_IDWG_STOP         (1 << 17) /* Bit 17: Independent watchdog counter freeze in stop mode */
#define FLASH_OPTR_IDWG_STDBY        (1 << 18) /* Bit 18: Independent watchdog counter freeze in Standby mode */
#define FLASH_OPTR_WWDG_SW           (1 << 19) /* Bit 19: Window watchdog selection */
                                               /* Bit 20: Reserved */
#define FLASH_OPTR_HSE_NOT_REMAPPED  (1 << 21) /* Bit 21: HSE remapping enable/disable */
#define FLASH_OPTR_RAM_PARITY_CHECK  (1 << 22) /* Bit 22: SRAM parity check control */
#define FLASH_OPTR_SECURE_MUXING_EN  (1 << 23) /* Bit 23: Multiple-bonding security */
                                               /* Bit 23: Reserved */
#define FLASH_OPTR_NBOOT_SEL         (1 << 24) /* Bit 24: BOOT0 signal source selection */
#define FLASH_OPTR_NBOOT1            (1 << 25) /* Bit 25: NBOOT1 boot configuration */
#define FLASH_OPTR_NBOOT0            (1 << 26) /* Bit 26: NBOOT0 option bit */
#define FLASH_OPTR_NRST_MODE_SHIFT   (27)      /* Bits 27-28: PF2-NRST pin configuration */
#define FLASH_OPTR_NRST_MODE_MASK    (0x3 << FLASH_OPTR_NRST_MODE_SHIFT)
#define FLASH_OPTR_IRHEN             (1 << 29) /* Bit 29: Internal reset holder enable */
#define FLASH_OPTR_FDCAN_BLCK_SHIFT  (30)      /* Bits 30-31: FDCAN bootloader clock source */
#define FLASH_OPTR_FDCAN_BLCK_MASK   (0x3 << FLASH_OPTR_FDCAN_BLCK_SHIFT)

/* Flash PCROP area A start address register (PCROP1ASR) */

#define FLASH_PCROP1ASR_STRT_SHIFT          (0)
#define FLASH_PCROP1ASR_STRT_MASK           (0x1ff << FLASH_PCROP1ASR_STRT_SHIFT)

/* Flash PCROP area A end address register (PCROP1AER) */

#define FLASH_PCROP1AER_PCROP1A_END_SHIFT   (0)
#define FLASH_PCROP1AER_PCROP1A_END_MASK    (0x1ff << FLASH_PCROP1AER_PCROP1A_END_SHIFT)
#define FLASH_PCROP1AER_PCROP_RDP           (1 << 31)

/* Flash WRP area A address register (WRP1AR) */

#define FLASH_WRP1AR_WRP1A_STRT_SHIFT       (0)
#define FLASH_WRP1AR_WRP1A_STRT_MASK        (0x7f << FLASH_WRP1AR_WRP1A_STRT_SHIFT)
#define FLASH_WRP1AR_WRP1A_END_SHIFT        (16)
#define FLASH_WRP1AR_WRP1A_END_MASK         (0x7f << FLASH_WRP1AR_WRP1A_END_SHIFT)

/* Flash WRP area B address register (WRP1BR) */

#define FLASH_WRP1BR_WRP1B_STRT_SHIFT       (0)
#define FLASH_WRP1BR_WRP1B_STRT_MASK        (0x7f << FLASH_WRP1BR_WRP1B_STRT_SHIFT)
#define FLASH_WRP1BR_WRP1B_END_SHIFT        (16)
#define FLASH_WRP1BR_WRP1B_END_MASK         (0x7f << FLASH_WRP1BR_WRP1B_END_SHIFT)

/* Flash PCROP area B start address register (PCROP1BSR) */

#define FLASH_PCROP1BSR_PCROP1B_STRT_SHIFT  (0)
#define FLASH_PCROP1BSR_PCROP1B_STRT_MASK   (0x1ff << FLASH_PCROP1BSR_PCROP1B_STRT_SHIFT)

/* Flash PCROP area B end address register (PCROP1BER) */

#define FLASH_PCROP1BER_PCROP1B_END_SHIFT   (0)
#define FLASH_PCROP1BER_PCROP1B_END_MASK    (0x1ff << FLASH_PCROP1BER_PCROP1B_END_SHIFT)

/* Flash Security register (SECR) */

#define FLASH_SECR_SEC_SIZE_SHIFT           (0)       /* Bits 0-7: Securable memory area size */
#define FLASH_SECR_SEC_SIZE_MASK            (0xff << FLASH_SECR_SEC_SIZE_SHIFT)
                                                      /* Bits 8-15: Reserved */
#define FLASH_SECR_BOOT_LOCK                (1 << 16) /* Bit 16: Used to force boot from user area */
                                                      /* Bits 20-31: Reserved */

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32C0_FLASH_H */
