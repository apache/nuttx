/****************************************************************************
 * arch/arm/src/stm32u5/hardware/stm32_pwr.h
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

#ifndef __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32_PWR_H
#define __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_PWR_CR1_OFFSET      0x0000  /* PWR control register 1 */
#define STM32_PWR_CR2_OFFSET      0x0004  /* PWR control register 2 */
#define STM32_PWR_CR3_OFFSET      0x0008  /* PWR control register 3 */
#define STM32_PWR_VOSR_OFFSET     0x000c  /* PWR voltage scaling register */
#define STM32_PWR_SVMCR_OFFSET    0x0010  /* PWR supply voltage monitoring control register */
#define STM32_PWR_WUCR1_OFFSET    0x0014  /* PWR wakeup control register 1 */
#define STM32_PWR_WUCR2_OFFSET    0x0018  /* PWR wakeup control register 2 */
#define STM32_PWR_WUCR3_OFFSET    0x001c  /* PWR wakeup control register 3 */
#define STM32_PWR_BDCR1_OFFSET    0x0020  /* PWR Backup domain control register 1 */
#define STM32_PWR_BDCR2_OFFSET    0x0024  /* PWR Backup domain control register 2 */
#define STM32_PWR_DBPR_OFFSET     0x0028  /* PWR disable Backup domain register */
#define STM32_PWR_UCPDR_OFFSET    0x002c  /* PWR USB Type-C and Power Delivery register */
#define STM32_PWR_SECCFGR_OFFSET  0x0030  /* PWR security configuration register */
#define STM32_PWR_PRIVCFGR_OFFSET 0x0034  /* PWR privilege control register */
#define STM32_PWR_SR_OFFSET       0x0038  /* PWR status register */
#define STM32_PWR_SVMSR_OFFSET    0x003c  /* PWR supply voltage monitoring register */
#define STM32_PWR_BDSR_OFFSET     0x0040  /* PWR Backup domain status register */
#define STM32_PWR_WUSR_OFFSET     0x0044  /* PWR wakeup status register */
#define STM32_PWR_WUSCR_OFFSET    0x0048  /* PWR wakeup status clear register */
#define STM32_PWR_APCR_OFFSET     0x004c  /* PWR apply pull configuration register */
#define STM32_PWR_PUCRA_OFFSET    0x0050  /* PWR Port A pull-up control register */
#define STM32_PWR_PDCRA_OFFSET    0x0054  /* PWR Port A pull-down control register */
#define STM32_PWR_PUCRB_OFFSET    0x0058  /* PWR Port B pull-up control register */
#define STM32_PWR_PDCRB_OFFSET    0x005C  /* PWR Port B pull-down control register */
#define STM32_PWR_PUCRC_OFFSET    0x0060  /* PWR Port C pull-up control register */
#define STM32_PWR_PDCRC_OFFSET    0x0064  /* PWR Port C pull-down control register */
#define STM32_PWR_PUCRD_OFFSET    0x0068  /* PWR Port D pull-up control register */
#define STM32_PWR_PDCRD_OFFSET    0x006C  /* PWR Port D pull-down control register */
#define STM32_PWR_PUCRE_OFFSET    0x0070  /* PWR Port E pull-up control register */
#define STM32_PWR_PDCRE_OFFSET    0x0074  /* PWR Port E pull-down control register */
#define STM32_PWR_PUCRF_OFFSET    0x0078  /* PWR Port F pull-up control register */
#define STM32_PWR_PDCRF_OFFSET    0x007C  /* PWR Port F pull-down control register */
#define STM32_PWR_PUCRG_OFFSET    0x0080  /* PWR Port G pull-up control register */
#define STM32_PWR_PDCRG_OFFSET    0x0084  /* PWR Port G pull-down control register */
#define STM32_PWR_PUCRH_OFFSET    0x0088  /* PWR Port H pull-up control register */
#define STM32_PWR_PDCRH_OFFSET    0x008C  /* PWR Port H pull-down control register */
#define STM32_PWR_PUCRI_OFFSET    0x0090  /* PWR Port I pull-up control register */
#define STM32_PWR_PDCRI_OFFSET    0x0094  /* PWR Port I pull-down control register */

/* Register Addresses *******************************************************/

#define STM32_PWR_CR1      (STM32_PWR_BASE + STM32_PWR_CR1_OFFSET)
#define STM32_PWR_CR2      (STM32_PWR_BASE + STM32_PWR_CR2_OFFSET)
#define STM32_PWR_CR3      (STM32_PWR_BASE + STM32_PWR_CR3_OFFSET)
#define STM32_PWR_VOSR     (STM32_PWR_BASE + STM32_PWR_VOSR_OFFSET)
#define STM32_PWR_SVMCR    (STM32_PWR_BASE + STM32_PWR_SVMCR_OFFSET)
#define STM32_PWR_WUCR1    (STM32_PWR_BASE + STM32_PWR_WUCR1_OFFSET)
#define STM32_PWR_WUCR2    (STM32_PWR_BASE + STM32_PWR_WUCR2_OFFSET)
#define STM32_PWR_WUCR3    (STM32_PWR_BASE + STM32_PWR_WUCR3_OFFSET)
#define STM32_PWR_BDCR1    (STM32_PWR_BASE + STM32_PWR_BDCR1_OFFSET)
#define STM32_PWR_BDCR2    (STM32_PWR_BASE + STM32_PWR_BDCR2_OFFSET)
#define STM32_PWR_DBPR     (STM32_PWR_BASE + STM32_PWR_DBPR_OFFSET)
#define STM32_PWR_UCPDR    (STM32_PWR_BASE + STM32_PWR_UCPDR_OFFSET)
#define STM32_PWR_SECCFGR  (STM32_PWR_BASE + STM32_PWR_SECCFGR_OFFSET)
#define STM32_PWR_PRIVCFGR (STM32_PWR_BASE + STM32_PWR_PRIVCFGR_OFFSET)
#define STM32_PWR_SR       (STM32_PWR_BASE + STM32_PWR_SR_OFFSET)
#define STM32_PWR_SVMSR    (STM32_PWR_BASE + STM32_PWR_SVMSR_OFFSET)
#define STM32_PWR_BDSR     (STM32_PWR_BASE + STM32_PWR_BDSR_OFFSET)
#define STM32_PWR_WUSR     (STM32_PWR_BASE + STM32_PWR_WUSR_OFFSET)
#define STM32_PWR_WUSCR    (STM32_PWR_BASE + STM32_PWR_WUSCR_OFFSET)
#define STM32_PWR_APCR     (STM32_PWR_BASE + STM32_PWR_APCR_OFFSET)
#define STM32_PWR_PUCRA    (STM32_PWR_BASE + STM32_PWR_PUCRA_OFFSET)
#define STM32_PWR_PDCRA    (STM32_PWR_BASE + STM32_PWR_PDCRA_OFFSET)
#define STM32_PWR_PUCRB    (STM32_PWR_BASE + STM32_PWR_PUCRB_OFFSET)
#define STM32_PWR_PDCRB    (STM32_PWR_BASE + STM32_PWR_PDCRB_OFFSET)
#define STM32_PWR_PUCRC    (STM32_PWR_BASE + STM32_PWR_PUCRC_OFFSET)
#define STM32_PWR_PDCRC    (STM32_PWR_BASE + STM32_PWR_PDCRC_OFFSET)
#define STM32_PWR_PUCRD    (STM32_PWR_BASE + STM32_PWR_PUCRD_OFFSET)
#define STM32_PWR_PDCRD    (STM32_PWR_BASE + STM32_PWR_PDCRD_OFFSET)
#define STM32_PWR_PUCRE    (STM32_PWR_BASE + STM32_PWR_PUCRE_OFFSET)
#define STM32_PWR_PDCRE    (STM32_PWR_BASE + STM32_PWR_PDCRE_OFFSET)
#define STM32_PWR_PUCRF    (STM32_PWR_BASE + STM32_PWR_PUCRF_OFFSET)
#define STM32_PWR_PDCRF    (STM32_PWR_BASE + STM32_PWR_PDCRF_OFFSET)
#define STM32_PWR_PUCRG    (STM32_PWR_BASE + STM32_PWR_PUCRG_OFFSET)
#define STM32_PWR_PDCRG    (STM32_PWR_BASE + STM32_PWR_PDCRG_OFFSET)
#define STM32_PWR_PUCRH    (STM32_PWR_BASE + STM32_PWR_PUCRH_OFFSET)
#define STM32_PWR_PDCRH    (STM32_PWR_BASE + STM32_PWR_PDCRH_OFFSET)
#define STM32_PWR_PUCRI    (STM32_PWR_BASE + STM32_PWR_PUCRI_OFFSET)
#define STM32_PWR_PDCRI    (STM32_PWR_BASE + STM32_PWR_PDCRI_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* PWR control register 1 */

#define PWR_CR1_LPMS_SHIFT       0
#define PWR_CR1_LPMS_MASK        (7 << PWR_CR1_LPMS_SHIFT) /* Bits 0-2: Low-power mode selection */
#  define PWR_CR1_LPMS_STOP0     (0 << PWR_CR1_LPMS_SHIFT) /* 000: Stop 0 mode */
#  define PWR_CR1_LPMS_STOP1     (1 << PWR_CR1_LPMS_SHIFT) /* 001: Stop 1 mode */
#  define PWR_CR1_LPMS_STOP2     (2 << PWR_CR1_LPMS_SHIFT) /* 010: Stop 2 mode */
#  define PWR_CR1_LPMS_STOP3     (3 << PWR_CR1_LPMS_SHIFT) /* 011: Stop 3 mode */
#  define PWR_CR1_LPMS_STANDBY   (4 << PWR_CR1_LPMS_SHIFT) /* 10x: Standby mode */
#  define PWR_CR1_LPMS_SHUTDOWN  (6 << PWR_CR1_LPMS_SHIFT) /* 11x: Shutdown mode */
#define PWR_CR1_RRSB1            (1 <<  5)                 /* Bit  5: SRAM2 page 1 retention in Stop 3 and Standby mode */
#define PWR_CR1_RRSB2            (1 <<  6)                 /* Bit  6: SRAM2 page 2 retention in Stop 3 and Standby mode */
#define PWR_CR1_ULPMEN           (1 <<  7)                 /* Bit  7: BOR ultra-low power mode */
#define PWR_CR1_SRAM1PD          (1 <<  8)                 /* Bit  8: SRAM1 power down */
#define PWR_CR1_SRAM2PD          (1 <<  9)                 /* Bit  9: SRAM2 power down */
#define PWR_CR1_SRAM3PD          (1 << 10)                 /* Bit 10: SRAM3 power down */
#define PWR_CR1_SRAM4PD          (1 << 11)                 /* Bit 11: SRAM4 power down */

/* PWR control register 2 */

#define PWR_CR2_SRAM1PDS1        (1 <<  0)                 /* Bit  0: SRAM1 page 1 (64 Kbytes) power-down in Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_SRAM1PDS2        (1 <<  1)                 /* Bit  1: SRAM1 page 2 (64 Kbytes) power-down in Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_SRAM1PDS3        (1 <<  2)                 /* Bit  2: SRAM1 page 3 (64 Kbytes) power-down in Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_SRAM2PDS1        (1 <<  4)                 /* Bit  4: SRAM2 page 1 (8 Kbytes) power-down in Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_SRAM2PDS2        (1 <<  5)                 /* Bit  5: SRAM2 page 2 (56 Kbytes) power-down in Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_SRAM4PDS         (1 <<  6)                 /* Bit  6: SRAM4 power-down in Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_ICRAMPDS         (1 <<  8)                 /* Bit  8: ICACHE SRAM power-down Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_DC1RAMPDS        (1 <<  9)                 /* Bit  9: DCACHE1 SRAM power-down Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_DMA2DRAMPDS      (1 << 10)                 /* Bit 10: DMA2D SRAM power-down Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_PRAMPDS          (1 << 11)                 /* Bit 11: FMAC, FDCAN and USB peripherals SRAM power-down Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_PKARAMPDS        (1 << 12)                 /* Bit 12: PKA SRAM power-down Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_SRAM4FWU         (1 << 13)                 /* Bit 13: SRAM4 fast wakeup from Stop 0, Stop 1 and Stop 2 modes */
#define PWR_CR2_FLASHFWU         (1 << 14)                 /* Bit 14: Flash memory fast wakeup from Stop 0, Stop 1 and Stop 2 modes */
#define PWR_CR2_SRAM3PDS1        (1 << 16)                 /* Bit 16: SRAM3 page 1 (64 Kbytes) power-down in Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_SRAM3PDS2        (1 << 17)                 /* Bit 17: SRAM3 page 2 (64 Kbytes) power-down in Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_SRAM3PDS3        (1 << 18)                 /* Bit 18: SRAM3 page 3 (64 Kbytes) power-down in Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_SRAM3PDS4        (1 << 19)                 /* Bit 19: SRAM3 page 4 (64 Kbytes) power-down in Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_SRAM3PDS5        (1 << 20)                 /* Bit 20: SRAM3 page 5 (64 Kbytes) power-down in Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_SRAM3PDS6        (1 << 21)                 /* Bit 21: SRAM3 page 6 (64 Kbytes) power-down in Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_SRAM3PDS7        (1 << 22)                 /* Bit 22: SRAM3 page 7 (64 Kbytes) power-down in Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_SRAM3PDS8        (1 << 23)                 /* Bit 23: SRAM3 page 8 (64 Kbytes) power-down in Stop modes (Stop 0, 1, 2, 3) */
#define PWR_CR2_SRDRUN           (1 << 31)                 /* Bit 31: SmartRun domain in Run mode */

/* PWR control register 3 */

#define PWR_CR3_REGSEL           (1 << 1)                  /* Bit 1: Regulator selection */
#define PWR_CR3_REGSEL_LDO       0
#define PWR_CR3_REGSEL_SMPS      PWR_CR3_REGSEL
#define PWR_CR3_FSTEN            (1 << 2)                  /* Bit 2: Fast soft start */

/* PWR voltage scaling register */

#define PWR_VOSR_BOOSTRDY        (1 << 14)                 /* Bit 14: EPOD booster ready */
#define PWR_VOSR_VOSRDY          (1 << 15)                 /* Bit 15: Ready bit for V_CORE voltage scaling output selection */
#define PWR_VOSR_VOS_SHIFT       16
#define PWR_VOSR_VOS_MASK        (3 << PWR_VOSR_VOS_SHIFT) /* Bits 16-17: Voltage scaling range selection */
#define PWR_VOSR_VOS_RANGE4      (0 << PWR_VOSR_VOS_SHIFT) /* 00: Range 4 (lowest power) */
#define PWR_VOSR_VOS_RANGE3      (1 << PWR_VOSR_VOS_SHIFT) /* 01: Range 3 */
#define PWR_VOSR_VOS_RANGE2      (2 << PWR_VOSR_VOS_SHIFT) /* 10: Range 2 */
#define PWR_VOSR_VOS_RANGE1      (3 << PWR_VOSR_VOS_SHIFT) /* 11: Range 1 (highest frequency) */
#define PWR_VOSR_BOOSTEN         (1 << 18)                 /* Bit 18: EPOD booster enable */

/* PWR Disable backup domain register */

#define PWR_DBPR_DBP             (1 <<  0)                 /* Bit  0: Disable Backup domain write protection. */ 

/* PWR Supply voltage monitoring status register */

#define PWR_SVMSR_REGS           (1 <<  1)                 /* Bit 1: Regulator selection */
#define PWR_SVMSR_REGS_LDO       0                         /* 0: LDO selected */
#define PWR_SVMSR_REGS_SMPS      PWR_SVMSR_REGS            /* 1: SMPS selected */
#define PWR_SVMSR_PVDO           (1 <<  4)                 /* Bit 4: Programmable voltage detector output */
#define PWR_SVMSR_ACTVOSRDY      (1 << 15)                 /* Bit 15: Voltage level ready for currenty used VOS */
#define PWR_SVMSR_ACTVOS_SHIFT   16
#define PWR_SVMSR_ACTVOS_MASK    (3 << PWR_SVMSR_ACTVOS_SHIFT) /* Bits 16-17: VOS currently applied to V_CORE */
#define PWR_SVMSR_ACTVOS_RANGE4  (0 << PWR_SVMSR_ACTVOS_SHIFT) /* 00: Range 4 (lowest power) */
#define PWR_SVMSR_ACTVOS_RANGE3  (1 << PWR_SVMSR_ACTVOS_SHIFT) /* 01: Range 3 */
#define PWR_SVMSR_ACTVOS_RANGE2  (2 << PWR_SVMSR_ACTVOS_SHIFT) /* 10: Range 2 */
#define PWR_SVMSR_ACTVOS_RANGE1  (3 << PWR_SVMSR_ACTVOS_SHIFT) /* 11: Range 1 (highest frequency) */
#define PWR_SVMSR_VDDUSBRDY      (1 << 24)                     /* Bit 24: V_DDUSB ready */
#define PWR_SVMSR_VDDIO2RDY      (1 << 25)                     /* Bit 25: V_DDIO2 ready */
#define PWR_SVMSR_VDDA1RDY       (1 << 26)                     /* Bit 26: V_DDA is equal or above ~1.6V */
#define PWR_SVMSR_VDDA2RDY       (1 << 27)                     /* Bit 27: V_DDA is equal or above ~1.8V */

#endif /* __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32_PWR_H */
