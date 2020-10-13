/************************************************************************************
 * arch/arm/src/stm32/hardware/stm32f33xxx_hrtim.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_HRTIM_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_HRTIM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define STM32_HRTIM_MASTER_OFFSET       0x0000      /* HRTIM Master Timer base address offset */
#define STM32_HRTIM_TIMERA_OFFSET       0x0080      /* HRTIM Timer A base address offset */
#define STM32_HRTIM_TIMERB_OFFSET       0x0100      /* HRTIM Timer B base address offset */
#define STM32_HRTIM_TIMERC_OFFSET       0x0180      /* HRTIM Timer C base address offset */
#define STM32_HRTIM_TIMERD_OFFSET       0x0200      /* HRTIM Timer D base address offset */
#define STM32_HRTIM_TIMERE_OFFSET       0x0280      /* HRTIM Timer E base address offset */
                                                    /* 0x300-0x37F: Reserved */
#define STM32_HRTIM_CMN_OFFSET          0x0380      /* HRTIM Common registers base address offset */

#define STM32_HRTIM1_MASTER_BASE        (STM32_HRTIM_MASTER_OFFSET+STM32_HRTIM1_BASE)
#define STM32_HRTIM1_TIMERA_BASE        (STM32_HRTIM_TIMERA_OFFSET+STM32_HRTIM1_BASE)
#define STM32_HRTIM1_TIMERB_BASE        (STM32_HRTIM_TIMERB_OFFSET+STM32_HRTIM1_BASE)
#define STM32_HRTIM1_TIMERC_BASE        (STM32_HRTIM_TIMERC_OFFSET+STM32_HRTIM1_BASE)
#define STM32_HRTIM1_TIMERD_BASE        (STM32_HRTIM_TIMERD_OFFSET+STM32_HRTIM1_BASE)
#define STM32_HRTIM1_TIMERE_BASE        (STM32_HRTIM_TIMERE_OFFSET+STM32_HRTIM1_BASE)
#define STM32_HRTIM1_CMN_BASE           (STM32_HRTIM_CMN_OFFSET+STM32_HRTIM1_BASE)

/* Register Offsets *****************************************************************/

/* Register Offsets Common for Master Timer and Timer X */

#define STM32_HRTIM_TIM_CR_OFFSET       0x0000 /* HRTIM Timer Control Register */
#define STM32_HRTIM_TIM_ISR_OFFSET      0x0004 /* HRTIM Timer Interrupt Status Register */
#define STM32_HRTIM_TIM_ICR_OFFSET      0x0008 /* HRTIM Timer Interrupt Clear Register */
#define STM32_HRTIM_TIM_DIER_OFFSET     0x000C /* HRTIM Timer DMA/Interrupt Enable Register */
#define STM32_HRTIM_TIM_CNTR_OFFSET     0x0010 /* HRTIM Timer Counter Register */
#define STM32_HRTIM_TIM_PER_OFFSET      0x0014 /* HRTIM Timer Period Register */
#define STM32_HRTIM_TIM_REPR_OFFSET     0x0018 /* HRTIM Timer Repetition Register */
#define STM32_HRTIM_TIM_CMP1R_OFFSET    0x001C /* HRTIM Timer Compare 1 Register */
#define STM32_HRTIM_TIM_CMP2R_OFFSET    0x0024 /* HRTIM Timer Compare 2 Register */
#define STM32_HRTIM_TIM_CMP3R_OFFSET    0x0028 /* HRTIM Timer Compare 3 Register */
#define STM32_HRTIM_TIM_CMP4R_OFFSET    0x002C /* HRTIM Timer Compare 4 Register */

/* Register offsets Specific for Timer A-E */

#define STM32_HRTIM_TIM_CMP1CR_OFFSET  0x0020 /* HRTIM Timer Compare 1 Compound Register */
#define STM32_HRTIM_TIM_CPT1R_OFFSET   0x0030 /* HRTIM Timer Capture 1 Register */
#define STM32_HRTIM_TIM_CPT2R_OFFSET   0x0034 /* HRTIM Timer Capture 2 Register */
#define STM32_HRTIM_TIM_DTR_OFFSET     0x0038 /* HRTIM Timer Deadtime Register */
#define STM32_HRTIM_TIM_SET1R_OFFSET   0x003C /* HRTIM Timer Output1 Set Register */
#define STM32_HRTIM_TIM_RST1R_OFFSET   0x0040 /* HRTIM Timer Output1 Reset Register */
#define STM32_HRTIM_TIM_SET2R_OFFSET   0x0044 /* HRTIM Timer Output2 Set Register */
#define STM32_HRTIM_TIM_RST2R_OFFSET   0x0048 /* HRTIM Timer Output2 Reset Register */
#define STM32_HRTIM_TIM_EEFR1_OFFSET   0x004C /* HRTIM Timer External Event Filtering Register 1 */
#define STM32_HRTIM_TIM_EEFR2_OFFSET   0x0050 /* HRTIM Timer External Event Filtering  Register 2 */
#define STM32_HRTIM_TIM_RSTR_OFFSET    0x0054 /* HRTIM Timer Reset Register */
#define STM32_HRTIM_TIM_CHPR_OFFSET    0x0058 /* HRTIM Timer Chopper Register */
#define STM32_HRTIM_TIM_CPT1CR_OFFSET  0x005C /* HRTIM Timer Capture 1 Control Register */
#define STM32_HRTIM_TIM_CPT2CR_OFFSET  0x0060 /* HRTIM Timer Capture 2 Control Register */
#define STM32_HRTIM_TIM_OUTR_OFFSET    0x0064 /* HRTIM Timer Output Register */
#define STM32_HRTIM_TIM_FLTR_OFFSET    0x0068 /* HRTIM Timer Fault Register */

/* Register Offset for HRTIM Common */

#define STM32_HRTIM_CMN_CR1_OFFSET      0x0000 /* HRTIM Control Register 1 */
#define STM32_HRTIM_CMN_CR2_OFFSET      0x0004 /* HRTIM Control Register 2 */
#define STM32_HRTIM_CMN_ISR_OFFSET      0x0008 /* HRTIM Interrupt Status Register */
#define STM32_HRTIM_CMN_ICR_OFFSET      0x000C /* HRTIM Interrupt Clear Register */
#define STM32_HRTIM_CMN_IER_OFFSET      0x0010 /* HRTIM Interrupt Enable Register */
#define STM32_HRTIM_CMN_OENR_OFFSET     0x0014 /* HRTIM Output Enable Register */
#define STM32_HRTIM_CMN_ODISR_OFFSET    0x0018 /* HRTIM Output Disable Register */
#define STM32_HRTIM_CMN_ODSR_OFFSET     0x001C /* HRTIM Output Disable Status Register */
#define STM32_HRTIM_CMN_BMCR_OFFSET     0x0020 /* HRTIM Burst Mode Control Register */
#define STM32_HRTIM_CMN_BMTRGR_OFFSET   0x0024 /* HRTIM Burst Mode Trigger Register */
#define STM32_HRTIM_CMN_BMCMPR_OFFSET   0x0028 /* HRTIM Burst Mode Compare Register */
#define STM32_HRTIM_CMN_BMPER_OFFSET    0x002C /* HRTIM Burst Mode Period Register */
#define STM32_HRTIM_CMN_EECR1_OFFSET    0x0030 /* HRTIM Timer External Event Control Register 1 */
#define STM32_HRTIM_CMN_EECR2_OFFSET    0x0034 /* HRTIM Timer External Event Control Register 2 */
#define STM32_HRTIM_CMN_EECR3_OFFSET    0x0038 /* HRTIM Timer External Event Control Register 3 */
#define STM32_HRTIM_CMN_ADC1R_OFFSET    0x003C /* HRTIM ADC Trigger 1 Register */
#define STM32_HRTIM_CMN_ADC2R_OFFSET    0x0040 /* HRTIM ADC Trigger 2 Register */
#define STM32_HRTIM_CMN_ADC3R_OFFSET    0x0044 /* HRTIM ADC Trigger 3 Register */
#define STM32_HRTIM_CMN_ADC4R_OFFSET    0x0048 /* HRTIM ADC Trigger 4 Register */
#define STM32_HRTIM_CMN_DLLCR_OFFSET    0x004C /* HRTIM DLL Control Register */
#define STM32_HRTIM_CMN_FLTINR1_OFFSET  0x0050 /* HRTIM Fault Input Register 1 */
#define STM32_HRTIM_CMN_FLTINR2_OFFSET  0x0054 /* HRTIM Fault Input Register 2 */
#define STM32_HRTIM_CMN_BDMUPDR_OFFSET  0x0058 /* HRTIM Master Timer Update Register */
#define STM32_HRTIM_CMN_BDTAUPR_OFFSET  0x005C /* HRTIM Timer A Update Register */
#define STM32_HRTIM_CMN_BDTBUPR_OFFSET  0x0060 /* HRTIM Timer B Update Register */
#define STM32_HRTIM_CMN_BDTCUPR_OFFSET  0x0064 /* HRTIM Timer C Update Register */
#define STM32_HRTIM_CMN_BDTDUPR_OFFSET  0x0068 /* HRTIM Timer D Update Register */
#define STM32_HRTIM_CMN_BDTEUPR_OFFSET  0x006C /* HRTIM Timer E Update Register */
#define STM32_HRTIM_CMN_BDMADR_OFFSET   0x0070 /* HRTIM DMA Data Register */

/* Register Bitfield Definitions ****************************************************/

/* Control Register Bits Common to Master Timer and Timer A-E */

#define HRTIM_CMNCR_CKPSC_SHIFT      0         /* Bits 0-2: Clock prescaler */
#define HRTIM_CMNCR_CKPSC_MASK       (7 << HRTIM_CMNCR_CKPSC_SHIFT)
#  define HRTIM_CMNCR_CKPSC_NODIV    (0 << HRTIM_CMNCR_CKPSC_SHIFT)
#  define HRTIM_CMNCR_CKPSC_d2       (1 << HRTIM_CMNCR_CKPSC_SHIFT)
#  define HRTIM_CMNCR_CKPSC_d4       (2 << HRTIM_CMNCR_CKPSC_SHIFT)
#  define HRTIM_CMNCR_CKPSC_d8       (3 << HRTIM_CMNCR_CKPSC_SHIFT)
#  define HRTIM_CMNCR_CKPSC_d16      (4 << HRTIM_CMNCR_CKPSC_SHIFT)
#  define HRTIM_CMNCR_CKPSC_d32      (5 << HRTIM_CMNCR_CKPSC_SHIFT)
#  define HRTIM_CMNCR_CKPSC_d64      (6 << HRTIM_CMNCR_CKPSC_SHIFT)
#  define HRTIM_CMNCR_CKPSC_d128     (7 << HRTIM_CMNCR_CKPSC_SHIFT)
#define HRTIM_CMNCR_CONT             (1 << 3)  /* Bit 3: Continuous mode */
#define HRTIM_CMNCR_RETRIG           (1 << 4)  /* Bit 4: Re-triggerable mode */
#define HRTIM_CMNCR_HALF             (1 << 5)  /* Bit 5: Half mode */
                                               /* Bits 6-9 differs */
#define HRTIM_CMNCR_SYNCRST          (1 << 10) /* Bit 10: Synchronization Resets Master */
#define HRTIM_CMNCR_SYNCSTRTM        (1 << 11) /* Bit 11: Synchronization Starts Master */
                                               /* Bits 12-24 differs */
#define HRTIM_CMNCR_DACSYNC_SHIFT    25        /* Bits 25-26: DAC Synchronization*/
#define HRTIM_CMNCR_DACSYNC_MASK     (3 << HRTIM_CMNCR_DACSYNC_SHIFT)
#  define HRTIM_CMNCR_DACSYNC_00     (0 << HRTIM_CMNCR_DACSYNC_SHIFT) /* 00: */
#  define HRTIM_CMNCR_DACSYNC_01     (1 << HRTIM_CMNCR_DACSYNC_SHIFT) /* 01: */
#  define HRTIM_CMNCR_DACSYNC_10     (2 << HRTIM_CMNCR_DACSYNC_SHIFT) /* 10: */
#  define HRTIM_CMNCR_DACSYNC_11     (3 << HRTIM_CMNCR_DACSYNC_SHIFT) /* 11: */
#define HRTIM_CMNCR_PREEN            (1 << 27)                        /* Bit 27: Preload enable */
                                                                      /* Bits 29-31 differs */

/* Control Register Bits specific to Master Timer */

#define HRTIM_MCR_SYNCIN_SHIFT     8                             /* Bits 8-9: Synchronization input */
#define HRTIM_MCR_SYNCIN_MASK      (3 << HRTIM_MCR_SYNCIN_SHIFT)
#  define HRTIM_MCR_SYNCIN_DIS     (0 << HRTIM_MCR_SYNCIN_SHIFT) /* 00 disabled */
#  define HRTIM_MCR_SYNCIN_INTE    (2 << HRTIM_MCR_SYNCIN_SHIFT) /* 10: Internal Event */
#  define HRTIM_MCR_SYNCIN_EXTE    (3 << HRTIM_MCR_SYNCIN_SHIFT) /* 11: External Event */
#define HRTIM_MCR_SYNCOUT_SHIFT    12                            /* Bits 12-13: Synchronization output */
#define HRTIM_MCR_SYNCOUT_MASK     (3 << HRTIM_MCR_SYNCOUT_SHIFT)
#  define HRTIM_MCR_SYNCOUT_DIS    (0 << HRTIM_MCR_SYNCOUT_SHIFT) /* 00: Disabled */
#  define HRTIM_MCR_SYNCOUT_POS    (2 << HRTIM_MCR_SYNCOUT_SHIFT) /* 10: Positive pulse on SCOUT */
#  define HRTIM_MCR_SYNCOUT_NEG    (3 << HRTIM_MCR_SYNCOUT_SHIFT) /* 11: Negative pulse on SCOUT */
#define HRTIM_MCR_SYNCSRC_SHIFT    14                             /* Bits 14-15: Synchronization source*/
#define HRTIM_MCR_SYNCSRC_MASK     (3 << HRTIM_MCR_SYNCSRC_SHIFT)
#  define HRTIM_MCR_SYNCSRC_MSTRT  (0 << HRTIM_MCR_SYNCSRC_SHIFT) /* 00: Master timer Start */
#  define HRTIM_MCR_SYNCSRC_MCMP1  (1 << HRTIM_MCR_SYNCSRC_SHIFT) /* 01: Master timer Compare 1 Event */
#  define HRTIM_MCR_SYNCSRC_TASTRT (2 << HRTIM_MCR_SYNCSRC_SHIFT) /* 10: Timer A start/reset */
#  define HRTIM_MCR_SYNCSRC_TACMP1 (3 << HRTIM_MCR_SYNCSRC_SHIFT) /* 11: Timer A Compare 1 Event */
#define HRTIM_MCR_TCEN_SHIFT       (16)
#define HRTIM_MCR_MCEN             (1 << 16) /* Bit 16: Master timer counter enable */
#define HRTIM_MCR_TACEN            (1 << 17) /* Bit 17: Timer A counter enable */
#define HRTIM_MCR_TBCEN            (1 << 18) /* Bit 18: Timer B counter enable */
#define HRTIM_MCR_TCCEN            (1 << 19) /* Bit 19: Timer C counter enable */
#define HRTIM_MCR_TDCEN            (1 << 20) /* Bit 20: Timer D counter enable */
#define HRTIM_MCR_TECEN            (1 << 21) /* Bit 21: Timer E counter enable */
                                             /* Bits 22-24 reserved */
                                             /* Bits 25-27 common */
#define HRTIM_MCR_MREPU            (1 << 29) /* Bit 29: Master Timer Repetition Update */
#define HRTIM_MCR_BRSTDMA_SHIFT    30        /* Bits 30-31: Burs DMA Update*/
#define HRTIM_MCR_BRSTDMA_MASK     (3 << HRTIM_MCR_BRSTDMA_SHIFT)
#  define HRTIM_MCR_BRSTDMA_00     (0 << HRTIM_MCR_BRSTDMA_SHIFT) /* 00 */
#  define HRTIM_MCR_BRSTDMA_01     (1 << HRTIM_MCR_BRSTDMA_SHIFT) /* 01 */
#  define HRTIM_MCR_BRSTDMA_10     (2 << HRTIM_MCR_BRSTDMA_SHIFT) /* 10 */

/* Master Timer Interrupt Status Register */

#define HRTIM_MISR_MCMP1           (1 << 0) /* Bit 0: Master Compare 1 Interrupt Flag */
#define HRTIM_MISR_MCMP2           (1 << 1) /* Bit 1: Master Compare 2 Interrupt Flag */
#define HRTIM_MISR_MCMP3           (1 << 2) /* Bit 2: Master Compare 3 Interrupt Flag */
#define HRTIM_MISR_MCMP4           (1 << 3) /* Bit 3: Master Compare 4 Interrupt Flag */
#define HRTIM_MISR_MREP            (1 << 4) /* Bit 4: Master Repetition Interrupt Flag */
#define HRTIM_MISR_SYNC            (1 << 5) /* Bit 5: Sync Input Interrupt Flag */
#define HRTIM_MISR_MUPD            (1 << 6) /* Bit 6: Master Update Interrupt Flag */

/* Master Timer Interrupt Clear Register */

#define HRTIM_MICR_MCMP1C          (1 << 0) /* Bit 0: Master Compare 1 Interrupt Flag Clear */
#define HRTIM_MICR_MCMP2C          (1 << 1) /* Bit 1: Master Compare 2 Interrupt Flag Clear  */
#define HRTIM_MICR_MCMP3C          (1 << 2) /* Bit 2: Master Compare 3 Interrupt Flag Clear  */
#define HRTIM_MICR_MCMP4C          (1 << 3) /* Bit 3: Master Compare 4 Interrupt Flag Clear  */
#define HRTIM_MICR_MREPC           (1 << 4) /* Bit 4: Master Repetition Interrupt Flag Clear  */
#define HRTIM_MICR_SYNCC           (1 << 5) /* Bit 5: Sync Input Interrupt Flag Clear  */
#define HRTIM_MICR_MUPDC           (1 << 6) /* Bit 6: Master Update Interrupt Flag Clear  */

/* Master Timer DMA/Interrupt Clear Register */

#define HRTIM_MDIER_MCMP1IE        (1 << 0)  /* Bit 0: Master Compare 1 Interrupt Enable */
#define HRTIM_MDIER_MCMP2IE        (1 << 1)  /* Bit 1: Master Compare 2 Interrupt Enable */
#define HRTIM_MDIER_MCMP3IE        (1 << 2)  /* Bit 2: Master Compare 3 Interrupt Enable */
#define HRTIM_MDIER_MCMP4IE        (1 << 3)  /* Bit 3: Master Compare 4 Interrupt Enable */
#define HRTIM_MDIER_MREPIE         (1 << 4)  /* Bit 4: Master Repetition Interrupt Enable */
#define HRTIM_MDIER_SYNCIE         (1 << 5)  /* Bit 5: Sync Input Interrupt Enable */
#define HRTIM_MDIER_MUPDIE         (1 << 6)  /* Bit 6: Master Update Interrupt Enable */
#define HRTIM_MDIER_MCMP1DE        (1 << 16) /* Bit 16 */
#define HRTIM_MDIER_MCMP2DE        (1 << 17) /* Bit 17 */
#define HRTIM_MDIER_MCMP3DE        (1 << 18) /* Bit 18 */
#define HRTIM_MDIER_MCMP4DE        (1 << 19) /* Bit 19 */
#define HRTIM_MDIER_MREPDE         (1 << 20) /* Bit 20 */
#define HRTIM_MDIER_SYNCDE         (1 << 21) /* Bit 21 */
#define HRTIM_MDIER_MUPDDE         (1 << 22) /* Bit 22 */

/* Master Timer Counter Register */

#define HRTIM_MCNTR_SHIFT          0 /* Bits 0-15: Counter Value */
#define HRTIM_MCNTR_MASK           (0xffff << HRTIM_MCNTR_SHIFT)

/* Master Timer Period Register */

#define HRTIM_MPER_SHIFT           0 /* Bits 0-15: Master Timer Period value */
#define HRTIM_MPER_MASK            (0xffff << HRTIM_MPER_SHIFT)

/* Master Timer Repetition Register */

#define HRTIM_MREP_SHIFT           0 /* Bits 0-8: Master Timer Repetition period value */
#define HRTIM_MREP_MASK            (0xff << HRTIM_MREP_SHIFT)

/* Master Timer Compare 1 Register */

#define HRTIM_MCMP1_SHIFT           0 /* Bits 0-15: Master Timer Compare 1 value */
#define HRTIM_MCMP1_MASK            (0xffff << HRTIM_MCMP1_SHIFT)

/* Master Timer Compare 2 Register */

#define HRTIM_MCMP2_SHIFT           0 /* Bits 0-15: Master Timer Compare 2 value */
#define HRTIM_MCMP2_MASK            (0xffff << HRTIM_MCMP2_SHIFT)

/* Master Timer Compare 3 Register */

#define HRTIM_MCMP3_SHIFT           0 /* Bits 0-15: Master Timer Compare 3 value  */
#define HRTIM_MCMP3_MASK            (0xffff << HRTIM_MCMP3_SHIFT)

/* Master Timer Compare 4 Register */

#define HRTIM_MCMP4_SHIFT           0 /* Bits 0-15: Master Timer Compare 4 value  */
#define HRTIM_MCMP4_MASK            (0xffff << HRTIM_MCMP4_SHIFT)

/* Timer A-E Control Register */

#define HRTIM_TIMCR_PSHPLL          (1 << 6)                         /* Bit 6:Push-Pull mode enable */
                                                                     /* Bits 10-11 common */
#define HRTIM_TIMCR_DELCMP2_SHIFT   12                               /* Bits 12-13: CMP2 auto-delayed mode */
#define HRTIM_TIMCR_DELCMP2_MASK    (3 << HRTIM_TIMCR_DELCMP2_SHIFT)
#  define HRTIM_TIMCR_DELCMP2_00    (0 << HRTIM_TIMCR_DELCMP2_SHIFT) /* 00: */
#  define HRTIM_TIMCR_DELCMP2_01    (1 << HRTIM_TIMCR_DELCMP2_SHIFT) /* 01: */
#  define HRTIM_TIMCR_DELCMP2_10    (2 << HRTIM_TIMCR_DELCMP2_SHIFT) /* 10: */
#  define HRTIM_TIMCR_DELCMP2_11    (3 << HRTIM_TIMCR_DELCMP2_SHIFT) /* 11: */
#define HRTIM_TIMCR_DELCMP4_SHIFT   12                               /* Bits 14-15: CMP4 auto-delayed mode */
#define HRTIM_TIMCR_DELCMP4_MASK    (3 << HRTIM_TIMCR_DELCMP4_SHIFT)
#  define HRTIM_TIMCR_DELCMP4_00    (0 << HRTIM_TIMCR_DELCMP4_SHIFT) /* 00: */
#  define HRTIM_TIMCR_DELCMP4_01    (1 << HRTIM_TIMCR_DELCMP4_SHIFT) /* 01: */
#  define HRTIM_TIMCR_DELCMP4_10    (2 << HRTIM_TIMCR_DELCMP4_SHIFT) /* 10: */
#  define HRTIM_TIMCR_DELCMP4_11    (3 << HRTIM_TIMCR_DELCMP4_SHIFT) /* 11: */
#define HRTIM_TIMCR_REPU            (1 << 17)                        /* Bit 17: Timer X Repetition Update */
#define HRTIM_TIMCR_RSTU            (1 << 18)                        /* Bit 18: Timer X Reset Update */
#define HRTIM_TIMCR_TAU             (1 << 19)                        /* Bit 19: Timer A Update */
#define HRTIM_TIMCR_TBU             (1 << 20)                        /* Bit 20: Timer B Update */
#define HRTIM_TIMCR_TCU             (1 << 21)                        /* Bit 21: Timer C Update */
#define HRTIM_TIMCR_TDU             (1 << 22)                        /* Bit 22: Timer D Update */
#define HRTIM_TIMCR_TEU             (1 << 23)                        /* Bit 23: Timer E Update */
#define HRTIM_TIMCR_MSTU            (1 << 24)                        /* Bit 24: Master Timer Update */
                                                                     /* Bits 25-27 common */
#define HRTIM_TIMCR_UPDGAT_SHIFT    28                               /* Bits 28-31: Update Gating */
#define HRTIM_TIMCR_UPDGAT_MASK     (15 << HRTIM_TIMCR_UPDGAT_SHIFT)
#  define HRTIM_TIMCR_UPDGAT_0000   (0 << HRTIM_TIMCR_UPDGAT_SHIFT)  /* 0000: */
#  define HRTIM_TIMCR_UPDGAT_0001   (1 << HRTIM_TIMCR_UPDGAT_SHIFT)  /* 0001: */
#  define HRTIM_TIMCR_UPDGAT_0010   (2 << HRTIM_TIMCR_UPDGAT_SHIFT)  /* 0010: */
#  define HRTIM_TIMCR_UPDGAT_0011   (3 << HRTIM_TIMCR_UPDGAT_SHIFT)  /* 0011: */
#  define HRTIM_TIMCR_UPDGAT_0100   (4 << HRTIM_TIMCR_UPDGAT_SHIFT)  /* 0100: */
#  define HRTIM_TIMCR_UPDGAT_0101   (5 << HRTIM_TIMCR_UPDGAT_SHIFT)  /* 0101: */
#  define HRTIM_TIMCR_UPDGAT_0110   (6 << HRTIM_TIMCR_UPDGAT_SHIFT)  /* 0110: */
#  define HRTIM_TIMCR_UPDGAT_0111   (7 << HRTIM_TIMCR_UPDGAT_SHIFT)  /* 0111: */
#  define HRTIM_TIMCR_UPDGAT_1000   (8 << HRTIM_TIMCR_UPDGAT_SHIFT)  /* 1000: */

/* Timer X Interrupt Status Register */

#define HRTIM_TIMISR_CMP1           (1 << 0)  /* Bit 0: Compare 1 Interrupt Flag */
#define HRTIM_TIMISR_CMP2           (1 << 1)  /* Bit 1: Compare 2 Interrupt Flag */
#define HRTIM_TIMISR_CMP3           (1 << 2)  /* Bit 2: Compare 3 Interrupt Flag */
#define HRTIM_TIMISR_CMP4           (1 << 3)  /* Bit 3: Compare 4 Interrupt Flag */
#define HRTIM_TIMISR_REP            (1 << 4)  /* Bit 4: Repetition Interrupt Flag */
#define HRTIM_TIMISR_UPD            (1 << 6)  /* Bit 6: Update Interrupt Flag */
#define HRTIM_TIMISR_CPT1           (1 << 7)  /* Bit 7: Capture 1 Interrupt Flag */
#define HRTIM_TIMISR_CPT2           (1 << 8)  /* Bit 8: Capture 2 Interrupt Flag */
#define HRTIM_TIMISR_SET1           (1 << 9)  /* Bit 9: Output 1 Set Interrupt Flag */
#define HRTIM_TIMISR_RST1           (1 << 10) /* Bit 10: Output 1 Reset Interrupt Flag  */
#define HRTIM_TIMISR_SET2           (1 << 11) /* Bit 11: Output 2 Set Interrupt Flag  */
#define HRTIM_TIMISR_RST2           (1 << 12) /* Bit 12: Output 2 Reset Interrupt Flag  */
#define HRTIM_TIMISR_RST            (1 << 13) /* Bit 13: Reset and/or roll-over Interrupt Flag  */
#define HRTIM_TIMISR_DLYPRT         (1 << 14) /* Bit 14: Delayed Protection Flag */
#define HRTIM_TIMISR_CPPSTAT        (1 << 16) /* Bit 16: Current Push Pull Status */
#define HRTIM_TIMISR_IPPSTAT        (1 << 17) /* Bit 17: Idle Push Pull Status */
#define HRTIM_TIMISR_O1STAT         (1 << 18) /* Bit 18: Output 1 Status */
#define HRTIM_TIMISR_O2STAT         (1 << 19) /* Bit 19: Output 2 Status */
#define HRTIM_TIMISR_O1CPY          (1 << 20) /* Bit 20: Output 1 Copy */
#define HRTIM_TIMISR_O2CPY          (1 << 21) /* Bit 21: Output 2 Copy */

/* Timer X Interrupt Clear Register */

#define HRTIM_TIMICR_CMP1C          (1 << 0)  /* Bit 0: Compare 1 Interrupt Flag Clear */
#define HRTIM_TIMICR_CMP2C          (1 << 1)  /* Bit 1: Compare 2 Interrupt Flag Clear */
#define HRTIM_TIMICR_CMP3C          (1 << 2)  /* Bit 2: Compare 3 Interrupt Flag Clear */
#define HRTIM_TIMICR_CMP4C          (1 << 3)  /* Bit 3: Compare 4 Interrupt Flag Clear */
#define HRTIM_TIMICR_REPC           (1 << 4)  /* Bit 4: Repetition Interrupt Flag Clear */
#define HRTIM_TIMICR_UPDC           (1 << 6)  /* Bit 6: Update Interrupt Flag Clear */
#define HRTIM_TIMICR_CPT1C          (1 << 7)  /* Bit 7: Capture 1 Interrupt Flag Clear */
#define HRTIM_TIMICR_CPT2C          (1 << 8)  /* Bit 8: Capture 2 Interrupt Flag Clear */
#define HRTIM_TIMICR_SET1C          (1 << 9)  /* Bit 9: Output 1 Set Flag Clear */
#define HRTIM_TIMICR_RST1C          (1 << 10) /* Bit 10: Output 1 Reset Flag Clear */
#define HRTIM_TIMICR_SET2C          (1 << 11) /* Bit 11: Output 2 Set Flag Clear */
#define HRTIM_TIMICR_RST2C          (1 << 12) /* Bit 12: Output 2 Reset Flag Clear */
#define HRTIM_TIMICR_RSTC           (1 << 13) /* Bit 13: Reset Interrupt Flag Clear */
#define HRTIM_TIMICR_DLYPRTC        (1 << 14) /* Bit 14: Delayed Protection Flag Clear */

/* Timer X  DMA/Interrupt Enable Register */

#define HRTIM_TIMDIER_CMP1IE        (1 << 0)  /* Bit 0: Compare 1 Interrupt Enable */
#define HRTIM_TIMDIER_CMP2IE        (1 << 1)  /* Bit 1: Compare 2 Interrupt Enable */
#define HRTIM_TIMDIER_CMP3IE        (1 << 2)  /* Bit 2: Compare 3 Interrupt Enable */
#define HRTIM_TIMDIER_CMP4IE        (1 << 3)  /* Bit 3: Compare 4 Interrupt Enable */
#define HRTIM_TIMDIER_REPIE         (1 << 4)  /* Bit 4: Repetition Interrupt Enable */
#define HRTIM_TIMDIER_UPDIE         (1 << 6)  /* Bit 6: Update Interrupt Enable */
#define HRTIM_TIMDIER_CPT1IE        (1 << 7)  /* Bit 7: Capture 1 Interrupt Enable */
#define HRTIM_TIMDIER_CPT2IE        (1 << 8)  /* Bit 8: Capture 2 Interrupt Enable */
#define HRTIM_TIMDIER_SET1IE        (1 << 9)  /* Bit 9: Output 1 Set Interrupt Enable */
#define HRTIM_TIMDIER_RST1IE        (1 << 10) /* Bit 10: Output 1 Reset Interrupt Enable  */
#define HRTIM_TIMDIER_SET2IE        (1 << 11) /* Bit 11: Output 2 Set Interrupt Enable  */
#define HRTIM_TIMDIER_RST2IE        (1 << 12) /* Bit 12: Output 2 Reset Interrupt Enable  */
#define HRTIM_TIMDIER_RSTIE         (1 << 13) /* Bit 13: Reset/roll-over Interrupt Enable */
#define HRTIM_TIMDIER_DLYPRTIE      (1 << 14) /* Bit 14: Delayed Protection Interrupt Enable */
#define HRTIM_TIMDIER_CMP1DE        (1 << 16) /* Bit 16: Compare 1 DMA Request Enable */
#define HRTIM_TIMDIER_CMP2DE        (1 << 17) /* Bit 17: Compare 2 DMA Request Enable */
#define HRTIM_TIMDIER_CMP3DE        (1 << 18) /* Bit 18: Compare 3 DMA Request Enable */
#define HRTIM_TIMDIER_CMP4DE        (1 << 19) /* Bit 19: Compare 4 DMA Request Enable */
#define HRTIM_TIMDIER_REPDE         (1 << 20) /* Bit 20: Repetition DMA Request Enable */
#define HRTIM_TIMDIER_UPDDE         (1 << 22) /* Bit 22: Update DMA Request Enable  */
#define HRTIM_TIMDIER_CPT1DE        (1 << 23) /* Bit 23: Capture 1 DMA Request Enable */
#define HRTIM_TIMDIER_CPT2DE        (1 << 24) /* Bit 24: Capture 2 DMA Request Enable */
#define HRTIM_TIMDIER_SET1DE        (1 << 25) /* Bit 25: Output 1 Set DMA Request Enable */
#define HRTIM_TIMDIER_RST1DE        (1 << 26) /* Bit 26: Output 1 Reset DMA Request Enable */
#define HRTIM_TIMDIER_SET2DE        (1 << 27) /* Bit 27: Output 2 Set DMA Request Enable */
#define HRTIM_TIMDIER_RST2DE        (1 << 28) /* Bit 28: Output 2 Reset DMA Request Enable */
#define HRTIM_TIMDIER_RSTDE         (1 << 29) /* Bit 29: Reset/roll-over DMA Request Enable */
#define HRTIM_TIMDIER_DLYPRTDE      (1 << 30) /* Bit 30: Delayed Protection DMA Request Enable */

/* Timer X Counter Register */

#define HRTIM_TIMCNTR_SHIFT         0 /* Bits 0-15: Timer X Counter Value */
#define HRTIM_TIMCNTR_MASK          (0xffff << HRTIM_TIMCNTR_SHIFT)

/* Timer X Period Register */

#define HRTIM_TIMPER_SHIFT          0 /* Bits 0-15: Timer X Period Value */
#define HRTIM_TIMPER_MASK           (0xffff << HRTIM_TIMPER_SHIFT)

/* Timer X Repetition Register */

#define HRTIM_TIMREP_SHIFT          0 /* Bits 0-8: Timer X Repetition Value */
#define HRTIM_TIMREP_MASK           (0xff << HRTIM_TIMREP_SHIFT)

/* Timer X Compare 1 Register */

#define HRTIM_TIMCMP1_SHIFT         0 /* Bits 0-15: Timer X Compare 1 Value */
#define HRTIM_TIMCMP1_MASK          (0xffff << HRTIM_TIMCMP1_SHIFT)

/* Timer X Compare 1 Compound Register */

#define HRTIM_TIMCMP1C_SHIFT        0 /* Bits 0-15: Timer X Compare 1 Value */
#define HRTIM_TIMCMP1C_MASK         (0xffff << HRTIM_TIMCMP1C_SHIFT)
#define HRTIM_TIMREPC_SHIFT         0 /* Bits 0-8: Timer X Repetition Value */
#define HRTIM_TIMREPC_MASK          (0xff << HRTIM_TIMCMP1C_SHIFT)

/* Timer X Compare 2 Register */

#define HRTIM_TIMCMP2_SHIFT         0 /* Bits 0-15: Timer X Compare 2 Value */
#define HRTIM_TIMCMP2_MASK          (0xffff << HRTIM_TIMCMP2_SHIFT)

/* Timer X Compare 3 Register */

#define HRTIM_TIMCMP3_SHIFT         0 /* Bits 0-15: Timer X Compare 3 Value */
#define HRTIM_TIMCMP3_MASK          (0xffff << HRTIM_TIMCMP3_SHIFT)

/* Timer X Compare 4 Register */

#define HRTIM_TIMCMP4_SHIFT         0 /* Bits 0-15: Timer X Compare 4 Value */
#define HRTIM_TIMCMP4_MASK          (0xffff << HRTIM_TIMCMP4_SHIFT)

/* Timer X Capture 1 Register */

#define HRTIM_TIMCPT1_SHIFT         0 /* Bits 0-15: Timer X Capture 1 Value */
#define HRTIM_TIMCPT1_MASK          (0xffff << HRTIM_TIMCPT1_SHIFT)

/* Timer X Capture 2 Register */

#define HRTIM_TIMCPT2_SHIFT         0 /* Bits 0-15: Timer X Capture 2 Value */
#define HRTIM_TIMCPT2_MASK          (0xffff << HRTIM_TIMCPT2_SHIFT)

/* Timer X Deadtime Register */

#define HRTIM_TIMDT_DTR_SHIFT       0         /* Bits 0-8: Deadtime Rising Value */
#define HRTIM_TIMDT_DTR_MASK        (0xff << HRTIM_TIMDT_DTR_SHIFT)
#define HRTIM_TIMDT_SDTR            (1 << 9)  /* Bit 9: Sign Deadtime Rising Value */
#define HRTIM_TIMDT_DTPRSC_SHIFT    10        /* Bits 10-12: Deadtime Prescaler */
#define HRTIM_TIMDT_DTPRSC_MASK     (7 << HRTIM_TIMDT_DTPRSC_SHIFT)
#  define HRTIM_TIMDT_DTPRSC_000    (0 << HRTIM_TIMDT_DTPRSC_SHIFT)
#  define HRTIM_TIMDT_DTPRSC_001    (1 << HRTIM_TIMDT_DTPRSC_SHIFT)
#  define HRTIM_TIMDT_DTPRSC_010    (2 << HRTIM_TIMDT_DTPRSC_SHIFT)
#  define HRTIM_TIMDT_DTPRSC_011    (3 << HRTIM_TIMDT_DTPRSC_SHIFT)
#  define HRTIM_TIMDT_DTPRSC_100    (4 << HRTIM_TIMDT_DTPRSC_SHIFT)
#  define HRTIM_TIMDT_DTPRSC_101    (5 << HRTIM_TIMDT_DTPRSC_SHIFT)
#  define HRTIM_TIMDT_DTPRSC_110    (6 << HRTIM_TIMDT_DTPRSC_SHIFT)
#  define HRTIM_TIMDT_DTPRSC_111    (7 << HRTIM_TIMDT_DTPRSC_SHIFT)
#define HRTIM_TIMDT_DTRSLK          (1 << 14) /* Bit 14: Deadtime Rising Sign Lock */
#define HRTIM_TIMDT_DTRLK           (1 << 15) /* Bit 15: Deadtime Rising Lock */
#define HRTIM_TIMDT_DTF_SHIFT       16        /* Bits 16-24: Deadtime Falling Value */
#define HRTIM_TIMDT_DTF_MASK        (0x1ff << HRTIM_TIMDT_DTF_SHIFT)
#define HRTIM_TIMDT_SDTF            (1 << 25) /* Bit 25: Sign Deadtime Falling Value */
#define HRTIM_TIMDT_DTFSLK          (1 << 30) /* Bit 30: Deadtime Falling Sign Lock */
#define HRTIM_TIMDT_DTFLK           (1 << 31) /* Bit 31: Deadtime Falling Lock */

/* Timer X Output1 Set Register */

#define HRTIM_TIMSET1_SST           (1 << 0)  /* Bit 0: Software Set trigger */
#define HRTIM_TIMSET1_RESYNC        (1 << 1)  /* Bit 1: Timer A resynchronization */
#define HRTIM_TIMSET1_PER           (1 << 2)  /* Bit 2: Timer X Period */
#define HRTIM_TIMSET1_CMP1          (1 << 3)  /* Bit 3: Timer X Compare 1 */
#define HRTIM_TIMSET1_CMP2          (1 << 4)  /* Bit 4: Timer X Compare 2 */
#define HRTIM_TIMSET1_CMP3          (1 << 5)  /* Bit 5: Timer X Compare 3 */
#define HRTIM_TIMSET1_CMP4          (1 << 6)  /* Bit 6: Timer X Compare 4 */
#define HRTIM_TIMSET1_MSTPER        (1 << 7)  /* Bit 7: Master Period */
#define HRTIM_TIMSET1_MSTCMP1       (1 << 8)  /* Bit 8: Master Compare 1 */
#define HRTIM_TIMSET1_MSTCMP2       (1 << 9)  /* Bit 9: Master Compare 2 */
#define HRTIM_TIMSET1_MSTCMP3       (1 << 10) /* Bit 10: Master Compare 3 */
#define HRTIM_TIMSET1_MSTCMP4       (1 << 11) /* Bit 11: Master Compare 4 */
#define HRTIM_TIMSET1_TIMEVNT1      (1 << 12) /* Bit 12: Timer Event 1 */
#define HRTIM_TIMSET1_TIMEVNT2      (1 << 13) /* Bit 13: Timer Event 2 */
#define HRTIM_TIMSET1_TIMEVNT3      (1 << 14) /* Bit 14: Timer Event 3 */
#define HRTIM_TIMSET1_TIMEVNT4      (1 << 15) /* Bit 15: Timer Event 4 */
#define HRTIM_TIMSET1_TIMEVNT5      (1 << 16) /* Bit 16: Timer Event 5 */
#define HRTIM_TIMSET1_TIMEVNT6      (1 << 17) /* Bit 17: Timer Event 6 */
#define HRTIM_TIMSET1_TIMEVNT7      (1 << 18) /* Bit 18: Timer Event 7 */
#define HRTIM_TIMSET1_TIMEVNT8      (1 << 19) /* Bit 19: Timer Event 8 */
#define HRTIM_TIMSET1_TIMEVNT9      (1 << 20) /* Bit 20: Timer Event 9 */
#define HRTIM_TIMSET1_EXTEVNT1      (1 << 21) /* Bit 21: External Event 1 */
#define HRTIM_TIMSET1_EXTEVNT2      (1 << 22) /* Bit 22: External Event 2 */
#define HRTIM_TIMSET1_EXTEVNT3      (1 << 23) /* Bit 23: External Event 3 */
#define HRTIM_TIMSET1_EXTEVNT4      (1 << 24) /* Bit 24: External Event 4 */
#define HRTIM_TIMSET1_EXTEVNT5      (1 << 25) /* Bit 25: External Event 5 */
#define HRTIM_TIMSET1_EXTEVNT6      (1 << 26) /* Bit 26: External Event 6 */
#define HRTIM_TIMSET1_EXTEVNT7      (1 << 27) /* Bit 27: External Event 7 */
#define HRTIM_TIMSET1_EXTEVNT8      (1 << 28) /* Bit 28: External Event 8 */
#define HRTIM_TIMSET1_EXTEVNT9      (1 << 29) /* Bit 29: External Event 9 */
#define HRTIM_TIMSET1_EXTEVNT10     (1 << 30) /* Bit 30: External Event 10 */
#define HRTIM_TIMSET1_UPDATE        (1 << 31) /* Bit 31: Registers Update */

/* Timer X Output1 Reset Register */

#define HRTIM_TIMRST1_SST           (1 << 0)  /* Bit 0 */
#define HRTIM_TIMRST1_RESYNC        (1 << 1)  /* Bit 1 */
#define HRTIM_TIMRST1_PER           (1 << 2)  /* Bit 2 */
#define HRTIM_TIMRST1_CMP1          (1 << 3)  /* Bit 3 */
#define HRTIM_TIMRST1_CMP2          (1 << 4)  /* Bit 4 */
#define HRTIM_TIMRST1_CMP3          (1 << 5)  /* Bit 5 */
#define HRTIM_TIMRST1_CMP4          (1 << 6)  /* Bit 6 */
#define HRTIM_TIMRST1_MSTPER        (1 << 7)  /* Bit 7 */
#define HRTIM_TIMRST1_MSTCMP1       (1 << 8)  /* Bit 8 */
#define HRTIM_TIMRST1_MSTCMP2       (1 << 9)  /* Bit 9 */
#define HRTIM_TIMRST1_MSTCMP3       (1 << 10) /* Bit 10 */
#define HRTIM_TIMRST1_MSTCMP4       (1 << 11) /* Bit 11 */
#define HRTIM_TIMRST1_TIMEVNT1      (1 << 12) /* Bit 12 */
#define HRTIM_TIMRST1_TIMEVNT2      (1 << 13) /* Bit 13 */
#define HRTIM_TIMRST1_TIMEVNT3      (1 << 14) /* Bit 14 */
#define HRTIM_TIMRST1_TIMEVNT4      (1 << 15) /* Bit 15 */
#define HRTIM_TIMRST1_TIMEVNT5      (1 << 16) /* Bit 16 */
#define HRTIM_TIMRST1_TIMEVNT6      (1 << 17) /* Bit 17 */
#define HRTIM_TIMRST1_TIMEVNT7      (1 << 18) /* Bit 18 */
#define HRTIM_TIMRST1_TIMEVNT8      (1 << 19) /* Bit 19 */
#define HRTIM_TIMRST1_TIMEVNT9      (1 << 20) /* Bit 20 */
#define HRTIM_TIMRST1_EXTEVNT1      (1 << 21) /* Bit 21 */
#define HRTIM_TIMRST1_EXTEVNT2      (1 << 22) /* Bit 22 */
#define HRTIM_TIMRST1_EXTEVNT3      (1 << 23) /* Bit 23 */
#define HRTIM_TIMRST1_EXTEVNT4      (1 << 24) /* Bit 24 */
#define HRTIM_TIMRST1_EXTEVNT5      (1 << 25) /* Bit 25 */
#define HRTIM_TIMRST1_EXTEVNT6      (1 << 26) /* Bit 26 */
#define HRTIM_TIMRST1_EXTEVNT7      (1 << 27) /* Bit 27 */
#define HRTIM_TIMRST1_EXTEVNT8      (1 << 28) /* Bit 28 */
#define HRTIM_TIMRST1_EXTEVNT9      (1 << 29) /* Bit 29 */
#define HRTIM_TIMRST1_EXTEVNT10     (1 << 30) /* Bit 30 */
#define HRTIM_TIMRST1_UPDATE        (1 << 31) /* Bit 31 */

/* Timer X Output2 Set Register */

#define HRTIM_TIMSET2_SST           (1 << 0)  /* Bit 0 */
#define HRTIM_TIMSET2_RESYNC        (1 << 1)  /* Bit 1 */
#define HRTIM_TIMSET2_PER           (1 << 2)  /* Bit 2 */
#define HRTIM_TIMSET2_CMP1          (1 << 3)  /* Bit 3 */
#define HRTIM_TIMSET2_CMP2          (1 << 4)  /* Bit 4 */
#define HRTIM_TIMSET2_CMP3          (1 << 5)  /* Bit 5 */
#define HRTIM_TIMSET2_CMP4          (1 << 6)  /* Bit 6 */
#define HRTIM_TIMSET2_MSTPER        (1 << 7)  /* Bit 7 */
#define HRTIM_TIMSET2_MSTCMP1       (1 << 8)  /* Bit 8 */
#define HRTIM_TIMSET2_MSTCMP2       (1 << 9)  /* Bit 9 */
#define HRTIM_TIMSET2_MSTCMP3       (1 << 10) /* Bit 10 */
#define HRTIM_TIMSET2_MSTCMP4       (1 << 11) /* Bit 11 */
#define HRTIM_TIMSET2_TIMEVNT1      (1 << 12) /* Bit 12 */
#define HRTIM_TIMSET2_TIMEVNT2      (1 << 13) /* Bit 13 */
#define HRTIM_TIMSET2_TIMEVNT3      (1 << 14) /* Bit 14 */
#define HRTIM_TIMSET2_TIMEVNT4      (1 << 15) /* Bit 15 */
#define HRTIM_TIMSET2_TIMEVNT5      (1 << 16) /* Bit 16 */
#define HRTIM_TIMSET2_TIMEVNT6      (1 << 17) /* Bit 17 */
#define HRTIM_TIMSET2_TIMEVNT7      (1 << 18) /* Bit 18 */
#define HRTIM_TIMSET2_TIMEVNT8      (1 << 19) /* Bit 19 */
#define HRTIM_TIMSET2_TIMEVNT9      (1 << 20) /* Bit 20 */
#define HRTIM_TIMSET2_EXTEVNT1      (1 << 21) /* Bit 21 */
#define HRTIM_TIMSET2_EXTEVNT2      (1 << 22) /* Bit 22 */
#define HRTIM_TIMSET2_EXTEVNT3      (1 << 23) /* Bit 23 */
#define HRTIM_TIMSET2_EXTEVNT4      (1 << 24) /* Bit 24 */
#define HRTIM_TIMSET2_EXTEVNT5      (1 << 25) /* Bit 25 */
#define HRTIM_TIMSET2_EXTEVNT6      (1 << 26) /* Bit 26 */
#define HRTIM_TIMSET2_EXTEVNT7      (1 << 27) /* Bit 27 */
#define HRTIM_TIMSET2_EXTEVNT8      (1 << 28) /* Bit 28 */
#define HRTIM_TIMSET2_EXTEVNT9      (1 << 29) /* Bit 29 */
#define HRTIM_TIMSET2_EXTEVNT10     (1 << 30) /* Bit 30 */
#define HRTIM_TIMSET2_UPDATE        (1 << 31) /* Bit 31 */

/* Timer X Output2 Reset Register */

#define HRTIM_TIMRST2_SST           (1 << 0)  /* Bit 0 */
#define HRTIM_TIMRST2_RESYNC        (1 << 1)  /* Bit 1 */
#define HRTIM_TIMRST2_PER           (1 << 2)  /* Bit 2 */
#define HRTIM_TIMRST2_CMP1          (1 << 3)  /* Bit 3 */
#define HRTIM_TIMRST2_CMP2          (1 << 4)  /* Bit 4 */
#define HRTIM_TIMRST2_CMP3          (1 << 5)  /* Bit 5 */
#define HRTIM_TIMRST2_CMP4          (1 << 6)  /* Bit 6 */
#define HRTIM_TIMRST2_MSTPER        (1 << 7)  /* Bit 7 */
#define HRTIM_TIMRST2_MSTCMP1       (1 << 8)  /* Bit 8 */
#define HRTIM_TIMRST2_MSTCMP2       (1 << 9)  /* Bit 9 */
#define HRTIM_TIMRST2_MSTCMP3       (1 << 10) /* Bit 10 */
#define HRTIM_TIMRST2_MSTCMP4       (1 << 11) /* Bit 11 */
#define HRTIM_TIMRST2_TIMEVNT1      (1 << 12) /* Bit 12 */
#define HRTIM_TIMRST2_TIMEVNT2      (1 << 13) /* Bit 13 */
#define HRTIM_TIMRST2_TIMEVNT3      (1 << 14) /* Bit 14 */
#define HRTIM_TIMRST2_TIMEVNT4      (1 << 15) /* Bit 15 */
#define HRTIM_TIMRST2_TIMEVNT5      (1 << 16) /* Bit 16 */
#define HRTIM_TIMRST2_TIMEVNT6      (1 << 17) /* Bit 17 */
#define HRTIM_TIMRST2_TIMEVNT7      (1 << 18) /* Bit 18 */
#define HRTIM_TIMRST2_TIMEVNT8      (1 << 19) /* Bit 19 */
#define HRTIM_TIMRST2_TIMEVNT9      (1 << 20) /* Bit 20 */
#define HRTIM_TIMRST2_EXTEVNT1      (1 << 21) /* Bit 21 */
#define HRTIM_TIMRST2_EXTEVNT2      (1 << 22) /* Bit 22 */
#define HRTIM_TIMRST2_EXTEVNT3      (1 << 23) /* Bit 23 */
#define HRTIM_TIMRST2_EXTEVNT4      (1 << 24) /* Bit 24 */
#define HRTIM_TIMRST2_EXTEVNT5      (1 << 25) /* Bit 25 */
#define HRTIM_TIMRST2_EXTEVNT6      (1 << 26) /* Bit 26 */
#define HRTIM_TIMRST2_EXTEVNT7      (1 << 27) /* Bit 27 */
#define HRTIM_TIMRST2_EXTEVNT8      (1 << 28) /* Bit 28 */
#define HRTIM_TIMRST2_EXTEVNT9      (1 << 29) /* Bit 29 */
#define HRTIM_TIMRST2_EXTEVNT10     (1 << 30) /* Bit 30 */
#define HRTIM_TIMRST2_UPDATE        (1 << 31) /* Bit 31 */

/* Timer X External Event Filtering Register 1 */

#define HRTIM_TIMEEF1_EE1LTCH       (1 << 0) /* Bit 0: External Event 1 Latch */
#define HRTIM_TIMEEF1_EE1FLT_SHIFT  1        /* Bits 1-4: External Event 1 Filter */
#define HRTIM_TIMEEF1_EE1FLT_MASK   (15 << HRTIM_TIMEEF1_EE1FLT_SHIFT)
#  define HRTIM_TIMEEF1_EE1FLT_0    (0 << HRTIM_TIMEEF1_EE1FLT_SHIFT)  /* 0000: No filtering */
#  define HRTIM_TIMEEF1_EE1FLT_1    (1 << HRTIM_TIMEEF1_EE1FLT_SHIFT)  /* 0001: Blanking from counter reset/roll-over to Compare 1 */
#  define HRTIM_TIMEEF1_EE1FLT_2    (2 << HRTIM_TIMEEF1_EE1FLT_SHIFT)  /* 0010: Blanking from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF1_EE1FLT_3    (3 << HRTIM_TIMEEF1_EE1FLT_SHIFT)  /* 0011: Blanking from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF1_EE1FLT_4    (4 << HRTIM_TIMEEF1_EE1FLT_SHIFT)  /* 0100: Blanking from counter reset/roll-over to Compare 4 */
#  define HRTIM_TIMEEF1_EE1FLT_5    (5 << HRTIM_TIMEEF1_EE1FLT_SHIFT)  /* 0101: Blanking from TIMFLTR1 source */
#  define HRTIM_TIMEEF1_EE1FLT_6    (6 << HRTIM_TIMEEF1_EE1FLT_SHIFT)  /* 0110: Blanking from TIMFLTR2 source */
#  define HRTIM_TIMEEF1_EE1FLT_7    (7 << HRTIM_TIMEEF1_EE1FLT_SHIFT)  /* 0111: Blanking from TIMFLTR3 source */
#  define HRTIM_TIMEEF1_EE1FLT_8    (8 << HRTIM_TIMEEF1_EE1FLT_SHIFT)  /* 1000: Blanking from TIMFLTR4 source */
#  define HRTIM_TIMEEF1_EE1FLT_9    (9 << HRTIM_TIMEEF1_EE1FLT_SHIFT)  /* 1001: Blanking from TIMFLTR5 source */
#  define HRTIM_TIMEEF1_EE1FLT_10   (10 << HRTIM_TIMEEF1_EE1FLT_SHIFT) /* 1010: Blanking from TIMFLTR6 source */
#  define HRTIM_TIMEEF1_EE1FLT_11   (11 << HRTIM_TIMEEF1_EE1FLT_SHIFT) /* 1011: Blanking from TIMFLTR7 source */
#  define HRTIM_TIMEEF1_EE1FLT_12   (12 << HRTIM_TIMEEF1_EE1FLT_SHIFT) /* 1100: Blanking from TIMFLTR8 source */
#  define HRTIM_TIMEEF1_EE1FLT_13   (13 << HRTIM_TIMEEF1_EE1FLT_SHIFT) /* 1101: Windowing from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF1_EE1FLT_14   (14 << HRTIM_TIMEEF1_EE1FLT_SHIFT) /* 1110: Windowing from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF1_EE1FLT_15   (15 << HRTIM_TIMEEF1_EE1FLT_SHIFT) /* 1111: Windowing from TIMWIN source */
#define HRTIM_TIMEEF1_EE2LTCH       (1 << 6)                           /* Bit 6: External Event 2 Lack */
#define HRTIM_TIMEEF1_EE2FLT_SHIFT  7                                  /* Bits 7-10: Externl Event 2 Filter */
#define HRTIM_TIMEEF1_EE2FLT_MASK   (15 << HRTIM_TIMEEF1_EE2FLT_SHIFT)
#  define HRTIM_TIMEEF1_EE2FLT_0    (0 << HRTIM_TIMEEF1_EE2FLT_SHIFT)  /* 0000: No filtering */
#  define HRTIM_TIMEEF1_EE2FLT_1    (1 << HRTIM_TIMEEF1_EE2FLT_SHIFT)  /* 0001: Blanking from counter reset/roll-over to Compare 1 */
#  define HRTIM_TIMEEF1_EE2FLT_2    (2 << HRTIM_TIMEEF1_EE2FLT_SHIFT)  /* 0010: Blanking from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF1_EE2FLT_3    (3 << HRTIM_TIMEEF1_EE2FLT_SHIFT)  /* 0011: Blanking from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF1_EE2FLT_4    (4 << HRTIM_TIMEEF1_EE2FLT_SHIFT)  /* 0100: Blanking from counter reset/roll-over to Compare 4 */
#  define HRTIM_TIMEEF1_EE2FLT_5    (5 << HRTIM_TIMEEF1_EE2FLT_SHIFT)  /* 0101: Blanking from TIMFLTR1 source */
#  define HRTIM_TIMEEF1_EE2FLT_6    (6 << HRTIM_TIMEEF1_EE2FLT_SHIFT)  /* 0110: Blanking from TIMFLTR2 source */
#  define HRTIM_TIMEEF1_EE2FLT_7    (7 << HRTIM_TIMEEF1_EE2FLT_SHIFT)  /* 0111: Blanking from TIMFLTR3 source */
#  define HRTIM_TIMEEF1_EE2FLT_8    (8 << HRTIM_TIMEEF1_EE2FLT_SHIFT)  /* 1000: Blanking from TIMFLTR4 source */
#  define HRTIM_TIMEEF1_EE2FLT_9    (9 << HRTIM_TIMEEF1_EE2FLT_SHIFT)  /* 1001: Blanking from TIMFLTR5 source */
#  define HRTIM_TIMEEF1_EE2FLT_10   (10 << HRTIM_TIMEEF1_EE2FLT_SHIFT) /* 1010: Blanking from TIMFLTR6 source */
#  define HRTIM_TIMEEF1_EE2FLT_11   (11 << HRTIM_TIMEEF1_EE2FLT_SHIFT) /* 1011: Blanking from TIMFLTR7 source */
#  define HRTIM_TIMEEF1_EE2FLT_12   (12 << HRTIM_TIMEEF1_EE2FLT_SHIFT) /* 1100: Blanking from TIMFLTR8 source */
#  define HRTIM_TIMEEF1_EE2FLT_13   (13 << HRTIM_TIMEEF1_EE2FLT_SHIFT) /* 1101: Windowing from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF1_EE2FLT_14   (14 << HRTIM_TIMEEF1_EE2FLT_SHIFT) /* 1110: Windowing from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF1_EE2FLT_15   (15 << HRTIM_TIMEEF1_EE2FLT_SHIFT) /* 1111: Windowing from TIMWIN source */
#define HRTIM_TIMEEF1_EE3LTCH       (1 << 12)                          /* Bit 12: External Event 3 Lack */
#define HRTIM_TIMEEF1_EE3FLT_SHIFT  13                                 /* Bits 13-16: Externl Event 3 Filter */
#define HRTIM_TIMEEF1_EE3FLT_MASK   (15 << HRTIM_TIMEEF1_EE3FLT_SHIFT)
#  define HRTIM_TIMEEF1_EE3FLT_0    (0 << HRTIM_TIMEEF1_EE3FLT_SHIFT)  /* 0000: No filtering */
#  define HRTIM_TIMEEF1_EE3FLT_1    (1 << HRTIM_TIMEEF1_EE3FLT_SHIFT)  /* 0001: Blanking from counter reset/roll-over to Compare 1 */
#  define HRTIM_TIMEEF1_EE3FLT_2    (2 << HRTIM_TIMEEF1_EE3FLT_SHIFT)  /* 0010: Blanking from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF1_EE3FLT_3    (3 << HRTIM_TIMEEF1_EE3FLT_SHIFT)  /* 0011: Blanking from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF1_EE3FLT_4    (4 << HRTIM_TIMEEF1_EE3FLT_SHIFT)  /* 0100: Blanking from counter reset/roll-over to Compare 4 */
#  define HRTIM_TIMEEF1_EE3FLT_5    (5 << HRTIM_TIMEEF1_EE3FLT_SHIFT)  /* 0101: Blanking from TIMFLTR1 source */
#  define HRTIM_TIMEEF1_EE3FLT_6    (6 << HRTIM_TIMEEF1_EE3FLT_SHIFT)  /* 0110: Blanking from TIMFLTR2 source */
#  define HRTIM_TIMEEF1_EE3FLT_7    (7 << HRTIM_TIMEEF1_EE3FLT_SHIFT)  /* 0111: Blanking from TIMFLTR3 source */
#  define HRTIM_TIMEEF1_EE3FLT_8    (8 << HRTIM_TIMEEF1_EE3FLT_SHIFT)  /* 1000: Blanking from TIMFLTR4 source */
#  define HRTIM_TIMEEF1_EE3FLT_9    (9 << HRTIM_TIMEEF1_EE3FLT_SHIFT)  /* 1001: Blanking from TIMFLTR5 source */
#  define HRTIM_TIMEEF1_EE3FLT_10   (10 << HRTIM_TIMEEF1_EE3FLT_SHIFT) /* 1010: Blanking from TIMFLTR6 source */
#  define HRTIM_TIMEEF1_EE3FLT_11   (11 << HRTIM_TIMEEF1_EE3FLT_SHIFT) /* 1011: Blanking from TIMFLTR7 source */
#  define HRTIM_TIMEEF1_EE3FLT_12   (12 << HRTIM_TIMEEF1_EE3FLT_SHIFT) /* 1100: Blanking from TIMFLTR8 source */
#  define HRTIM_TIMEEF1_EE3FLT_13   (13 << HRTIM_TIMEEF1_EE3FLT_SHIFT) /* 1101: Windowing from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF1_EE3FLT_14   (14 << HRTIM_TIMEEF1_EE3FLT_SHIFT) /* 1110: Windowing from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF1_EE3FLT_15   (15 << HRTIM_TIMEEF1_EE3FLT_SHIFT) /* 1111: Windowing from TIMWIN source */
#define HRTIM_TIMEEF1_EE4LTCH       (1 << 18)                          /* Bit 18: External Event 4 Lack */
#define HRTIM_TIMEEF1_EE4FLT_SHIFT  19                                 /* Bits 19-22: Externl Event 4 Filter */
#define HRTIM_TIMEEF1_EE4FLT_MASK   (15 << HRTIM_TIMEEF1_EE4FLT_SHIFT)
#  define HRTIM_TIMEEF1_EE4FLT_0    (0 << HRTIM_TIMEEF1_EE4FLT_SHIFT)  /* 0000: No filtering */
#  define HRTIM_TIMEEF1_EE4FLT_1    (1 << HRTIM_TIMEEF1_EE4FLT_SHIFT)  /* 0001: Blanking from counter reset/roll-over to Compare 1 */
#  define HRTIM_TIMEEF1_EE4FLT_2    (2 << HRTIM_TIMEEF1_EE4FLT_SHIFT)  /* 0010: Blanking from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF1_EE4FLT_3    (3 << HRTIM_TIMEEF1_EE4FLT_SHIFT)  /* 0011: Blanking from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF1_EE4FLT_4    (4 << HRTIM_TIMEEF1_EE4FLT_SHIFT)  /* 0100: Blanking from counter reset/roll-over to Compare 4 */
#  define HRTIM_TIMEEF1_EE4FLT_5    (5 << HRTIM_TIMEEF1_EE4FLT_SHIFT)  /* 0101: Blanking from TIMFLTR1 source */
#  define HRTIM_TIMEEF1_EE4FLT_6    (6 << HRTIM_TIMEEF1_EE4FLT_SHIFT)  /* 0110: Blanking from TIMFLTR2 source */
#  define HRTIM_TIMEEF1_EE4FLT_7    (7 << HRTIM_TIMEEF1_EE4FLT_SHIFT)  /* 0111: Blanking from TIMFLTR3 source */
#  define HRTIM_TIMEEF1_EE4FLT_8    (8 << HRTIM_TIMEEF1_EE4FLT_SHIFT)  /* 1000: Blanking from TIMFLTR4 source */
#  define HRTIM_TIMEEF1_EE4FLT_9    (9 << HRTIM_TIMEEF1_EE4FLT_SHIFT)  /* 1001: Blanking from TIMFLTR5 source */
#  define HRTIM_TIMEEF1_EE4FLT_10   (10 << HRTIM_TIMEEF1_EE4FLT_SHIFT) /* 1010: Blanking from TIMFLTR6 source */
#  define HRTIM_TIMEEF1_EE4FLT_11   (11 << HRTIM_TIMEEF1_EE4FLT_SHIFT) /* 1011: Blanking from TIMFLTR7 source */
#  define HRTIM_TIMEEF1_EE4FLT_12   (12 << HRTIM_TIMEEF1_EE4FLT_SHIFT) /* 1100: Blanking from TIMFLTR8 source */
#  define HRTIM_TIMEEF1_EE4FLT_13   (13 << HRTIM_TIMEEF1_EE4FLT_SHIFT) /* 1101: Windowing from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF1_EE4FLT_14   (14 << HRTIM_TIMEEF1_EE4FLT_SHIFT) /* 1110: Windowing from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF1_EE4FLT_15   (15 << HRTIM_TIMEEF1_EE4FLT_SHIFT) /* 1111: Windowing from TIMWIN source */
#define HRTIM_TIMEEF1_EE5LTCH       (1 << 24)                          /* Bit 24: External Event 5 Lack */
#define HRTIM_TIMEEF1_EE5FLT_SHIFT  25                                 /* Bits 25-28: Externl Event 5 Filter */
#define HRTIM_TIMEEF1_EE5FLT_MASK   (15 << HRTIM_TIMEEF1_EE5FLT_SHIFT)
#  define HRTIM_TIMEEF1_EE5FLT_0    (0 << HRTIM_TIMEEF1_EE5FLT_SHIFT)  /* 0000: No filtering */
#  define HRTIM_TIMEEF1_EE5FLT_1    (1 << HRTIM_TIMEEF1_EE5FLT_SHIFT)  /* 0001: Blanking from counter reset/roll-over to Compare 1 */
#  define HRTIM_TIMEEF1_EE5FLT_2    (2 << HRTIM_TIMEEF1_EE5FLT_SHIFT)  /* 0010: Blanking from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF1_EE5FLT_3    (3 << HRTIM_TIMEEF1_EE5FLT_SHIFT)  /* 0011: Blanking from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF1_EE5FLT_4    (4 << HRTIM_TIMEEF1_EE5FLT_SHIFT)  /* 0100: Blanking from counter reset/roll-over to Compare 4 */
#  define HRTIM_TIMEEF1_EE5FLT_5    (5 << HRTIM_TIMEEF1_EE5FLT_SHIFT)  /* 0101: Blanking from TIMFLTR1 source */
#  define HRTIM_TIMEEF1_EE5FLT_6    (6 << HRTIM_TIMEEF1_EE5FLT_SHIFT)  /* 0110: Blanking from TIMFLTR2 source */
#  define HRTIM_TIMEEF1_EE5FLT_7    (7 << HRTIM_TIMEEF1_EE5FLT_SHIFT)  /* 0111: Blanking from TIMFLTR3 source */
#  define HRTIM_TIMEEF1_EE5FLT_8    (8 << HRTIM_TIMEEF1_EE5FLT_SHIFT)  /* 1000: Blanking from TIMFLTR4 source */
#  define HRTIM_TIMEEF1_EE5FLT_9    (9 << HRTIM_TIMEEF1_EE5FLT_SHIFT)  /* 1001: Blanking from TIMFLTR5 source */
#  define HRTIM_TIMEEF1_EE5FLT_10   (10 << HRTIM_TIMEEF1_EE5FLT_SHIFT) /* 1010: Blanking from TIMFLTR6 source */
#  define HRTIM_TIMEEF1_EE5FLT_11   (11 << HRTIM_TIMEEF1_EE5FLT_SHIFT) /* 1011: Blanking from TIMFLTR7 source */
#  define HRTIM_TIMEEF1_EE5FLT_12   (12 << HRTIM_TIMEEF1_EE5FLT_SHIFT) /* 1100: Blanking from TIMFLTR8 source */
#  define HRTIM_TIMEEF1_EE5FLT_13   (13 << HRTIM_TIMEEF1_EE5FLT_SHIFT) /* 1101: Windowing from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF1_EE5FLT_14   (14 << HRTIM_TIMEEF1_EE5FLT_SHIFT) /* 1110: Windowing from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF1_EE5FLT_15   (15 << HRTIM_TIMEEF1_EE5FLT_SHIFT) /* 1111: Windowing from TIMWIN source */

/* Timer X External Event Filtering Register 2 */

#define HRTIM_TIMEEF2_EE6LTCH       (1 << 0)                           /* Bit 0 */
#define HRTIM_TIMEEF2_EE6FLT_SHIFT  1                                  /* Bits 1-4 */
#define HRTIM_TIMEEF2_EE6FLT_MASK   (15 << HRTIM_TIMEEF2_EE6FLT_SHIFT)
#  define HRTIM_TIMEEF2_EE6FLT_0    (0 << HRTIM_TIMEEF2_EE6FLT_SHIFT)  /* 0000: No filtering */
#  define HRTIM_TIMEEF2_EE6FLT_1    (1 << HRTIM_TIMEEF2_EE6FLT_SHIFT)  /* 0001: Blanking from counter reset/roll-over to Compare 1 */
#  define HRTIM_TIMEEF2_EE6FLT_2    (2 << HRTIM_TIMEEF2_EE6FLT_SHIFT)  /* 0010: Blanking from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF2_EE6FLT_3    (3 << HRTIM_TIMEEF2_EE6FLT_SHIFT)  /* 0011: Blanking from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF2_EE6FLT_4    (4 << HRTIM_TIMEEF2_EE6FLT_SHIFT)  /* 0100: Blanking from counter reset/roll-over to Compare 4 */
#  define HRTIM_TIMEEF2_EE6FLT_5    (5 << HRTIM_TIMEEF2_EE6FLT_SHIFT)  /* 0101: Blanking from TIMFLTR1 source */
#  define HRTIM_TIMEEF2_EE6FLT_6    (6 << HRTIM_TIMEEF2_EE6FLT_SHIFT)  /* 0110: Blanking from TIMFLTR2 source */
#  define HRTIM_TIMEEF2_EE6FLT_7    (7 << HRTIM_TIMEEF2_EE6FLT_SHIFT)  /* 0111: Blanking from TIMFLTR3 source */
#  define HRTIM_TIMEEF2_EE6FLT_8    (8 << HRTIM_TIMEEF2_EE6FLT_SHIFT)  /* 1000: Blanking from TIMFLTR4 source */
#  define HRTIM_TIMEEF2_EE6FLT_9    (9 << HRTIM_TIMEEF2_EE6FLT_SHIFT)  /* 1001: Blanking from TIMFLTR5 source */
#  define HRTIM_TIMEEF2_EE6FLT_10   (10 << HRTIM_TIMEEF2_EE6FLT_SHIFT) /* 1010: Blanking from TIMFLTR6 source */
#  define HRTIM_TIMEEF2_EE6FLT_11   (11 << HRTIM_TIMEEF2_EE6FLT_SHIFT) /* 1011: Blanking from TIMFLTR7 source */
#  define HRTIM_TIMEEF2_EE6FLT_12   (12 << HRTIM_TIMEEF2_EE6FLT_SHIFT) /* 1100: Blanking from TIMFLTR8 source */
#  define HRTIM_TIMEEF2_EE6FLT_13   (13 << HRTIM_TIMEEF2_EE6FLT_SHIFT) /* 1101: Windowing from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF2_EE6FLT_14   (14 << HRTIM_TIMEEF2_EE6FLT_SHIFT) /* 1110: Windowing from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF2_EE6FLT_15   (15 << HRTIM_TIMEEF2_EE6FLT_SHIFT) /* 1111: Windowing from TIMWIN source */
#define HRTIM_TIMEEF2_EE7LTCH       (1 << 6)                           /* Bit 6 */
#define HRTIM_TIMEEF2_EE7FLT_SHIFT  7                                  /* Bits 7-10 */
#define HRTIM_TIMEEF2_EE7FLT_MASK   (15 << HRTIM_TIMEEF2_EE7FLT_SHIFT)
#  define HRTIM_TIMEEF2_EE7FLT_0    (0 << HRTIM_TIMEEF2_EE7FLT_SHIFT)  /* 0000: No filtering */
#  define HRTIM_TIMEEF2_EE7FLT_1    (1 << HRTIM_TIMEEF2_EE7FLT_SHIFT)  /* 0001: Blanking from counter reset/roll-over to Compare 1 */
#  define HRTIM_TIMEEF2_EE7FLT_2    (2 << HRTIM_TIMEEF2_EE7FLT_SHIFT)  /* 0010: Blanking from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF2_EE7FLT_3    (3 << HRTIM_TIMEEF2_EE7FLT_SHIFT)  /* 0011: Blanking from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF2_EE7FLT_4    (4 << HRTIM_TIMEEF2_EE7FLT_SHIFT)  /* 0100: Blanking from counter reset/roll-over to Compare 4 */
#  define HRTIM_TIMEEF2_EE7FLT_5    (5 << HRTIM_TIMEEF2_EE7FLT_SHIFT)  /* 0101: Blanking from TIMFLTR1 source */
#  define HRTIM_TIMEEF2_EE7FLT_6    (6 << HRTIM_TIMEEF2_EE7FLT_SHIFT)  /* 0110: Blanking from TIMFLTR2 source */
#  define HRTIM_TIMEEF2_EE7FLT_7    (7 << HRTIM_TIMEEF2_EE7FLT_SHIFT)  /* 0111: Blanking from TIMFLTR3 source */
#  define HRTIM_TIMEEF2_EE7FLT_8    (8 << HRTIM_TIMEEF2_EE7FLT_SHIFT)  /* 1000: Blanking from TIMFLTR4 source */
#  define HRTIM_TIMEEF2_EE7FLT_9    (9 << HRTIM_TIMEEF2_EE7FLT_SHIFT)  /* 1001: Blanking from TIMFLTR5 source */
#  define HRTIM_TIMEEF2_EE7FLT_10   (10 << HRTIM_TIMEEF2_EE7FLT_SHIFT) /* 1010: Blanking from TIMFLTR6 source */
#  define HRTIM_TIMEEF2_EE7FLT_11   (11 << HRTIM_TIMEEF2_EE7FLT_SHIFT) /* 1011: Blanking from TIMFLTR7 source */
#  define HRTIM_TIMEEF2_EE7FLT_12   (12 << HRTIM_TIMEEF2_EE7FLT_SHIFT) /* 1100: Blanking from TIMFLTR8 source */
#  define HRTIM_TIMEEF2_EE7FLT_13   (13 << HRTIM_TIMEEF2_EE7FLT_SHIFT) /* 1101: Windowing from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF2_EE7FLT_14   (14 << HRTIM_TIMEEF2_EE7FLT_SHIFT) /* 1110: Windowing from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF2_EE7FLT_15   (15 << HRTIM_TIMEEF2_EE7FLT_SHIFT) /* 1111: Windowing from TIMWIN source */
#define HRTIM_TIMEEF2_EE8LTCH       (1 << 12)                          /* Bit 12 */
#define HRTIM_TIMEEF2_EE8FLT_SHIFT  13                                 /* Bits 13-16 */
#define HRTIM_TIMEEF2_EE8FLT_MASK   (15 << HRTIM_TIMEEF2_EE8FLT_SHIFT)
#  define HRTIM_TIMEEF2_EE8FLT_0    (0 << HRTIM_TIMEEF2_EE8FLT_SHIFT)  /* 0000: No filtering */
#  define HRTIM_TIMEEF2_EE8FLT_1    (1 << HRTIM_TIMEEF2_EE8FLT_SHIFT)  /* 0001: Blanking from counter reset/roll-over to Compare 1 */
#  define HRTIM_TIMEEF2_EE8FLT_2    (2 << HRTIM_TIMEEF2_EE8FLT_SHIFT)  /* 0010: Blanking from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF2_EE8FLT_3    (3 << HRTIM_TIMEEF2_EE8FLT_SHIFT)  /* 0011: Blanking from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF2_EE8FLT_4    (4 << HRTIM_TIMEEF2_EE8FLT_SHIFT)  /* 0100: Blanking from counter reset/roll-over to Compare 4 */
#  define HRTIM_TIMEEF2_EE8FLT_5    (5 << HRTIM_TIMEEF2_EE8FLT_SHIFT)  /* 0101: Blanking from TIMFLTR1 source */
#  define HRTIM_TIMEEF2_EE8FLT_6    (6 << HRTIM_TIMEEF2_EE8FLT_SHIFT)  /* 0110: Blanking from TIMFLTR2 source */
#  define HRTIM_TIMEEF2_EE8FLT_7    (7 << HRTIM_TIMEEF2_EE8FLT_SHIFT)  /* 0111: Blanking from TIMFLTR3 source */
#  define HRTIM_TIMEEF2_EE8FLT_8    (8 << HRTIM_TIMEEF2_EE8FLT_SHIFT)  /* 1000: Blanking from TIMFLTR4 source */
#  define HRTIM_TIMEEF2_EE8FLT_9    (9 << HRTIM_TIMEEF2_EE8FLT_SHIFT)  /* 1001: Blanking from TIMFLTR5 source */
#  define HRTIM_TIMEEF2_EE8FLT_10   (10 << HRTIM_TIMEEF2_EE8FLT_SHIFT) /* 1010: Blanking from TIMFLTR6 source */
#  define HRTIM_TIMEEF2_EE8FLT_11   (11 << HRTIM_TIMEEF2_EE8FLT_SHIFT) /* 1011: Blanking from TIMFLTR7 source */
#  define HRTIM_TIMEEF2_EE8FLT_12   (12 << HRTIM_TIMEEF2_EE8FLT_SHIFT) /* 1100: Blanking from TIMFLTR8 source */
#  define HRTIM_TIMEEF2_EE8FLT_13   (13 << HRTIM_TIMEEF2_EE8FLT_SHIFT) /* 1101: Windowing from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF2_EE8FLT_14   (14 << HRTIM_TIMEEF2_EE8FLT_SHIFT) /* 1110: Windowing from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF2_EE8FLT_15   (15 << HRTIM_TIMEEF2_EE8FLT_SHIFT) /* 1111: Windowing from TIMWIN source */
#define HRTIM_TIMEEF2_EE9LTCH       (1 << 18)                          /* Bit 18 */
#define HRTIM_TIMEEF2_EE9FLT_SHIFT  19                                 /* Bits 19-22 */
#define HRTIM_TIMEEF2_EE9FLT_MASK   (15 << HRTIM_TIMEEF2_EE9FLT_SHIFT)
#  define HRTIM_TIMEEF2_EE9FLT_0    (0 << HRTIM_TIMEEF2_EE9FLT_SHIFT)  /* 0000: No filtering */
#  define HRTIM_TIMEEF2_EE9FLT_1    (1 << HRTIM_TIMEEF2_EE9FLT_SHIFT)  /* 0001: Blanking from counter reset/roll-over to Compare 1 */
#  define HRTIM_TIMEEF2_EE9FLT_2    (2 << HRTIM_TIMEEF2_EE9FLT_SHIFT)  /* 0010: Blanking from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF2_EE9FLT_3    (3 << HRTIM_TIMEEF2_EE9FLT_SHIFT)  /* 0011: Blanking from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF2_EE9FLT_4    (4 << HRTIM_TIMEEF2_EE9FLT_SHIFT)  /* 0100: Blanking from counter reset/roll-over to Compare 4 */
#  define HRTIM_TIMEEF2_EE9FLT_5    (5 << HRTIM_TIMEEF2_EE9FLT_SHIFT)  /* 0101: Blanking from TIMFLTR1 source */
#  define HRTIM_TIMEEF2_EE9FLT_6    (6 << HRTIM_TIMEEF2_EE9FLT_SHIFT)  /* 0110: Blanking from TIMFLTR2 source */
#  define HRTIM_TIMEEF2_EE9FLT_7    (7 << HRTIM_TIMEEF2_EE9FLT_SHIFT)  /* 0111: Blanking from TIMFLTR3 source */
#  define HRTIM_TIMEEF2_EE9FLT_8    (8 << HRTIM_TIMEEF2_EE9FLT_SHIFT)  /* 1000: Blanking from TIMFLTR4 source */
#  define HRTIM_TIMEEF2_EE9FLT_9    (9 << HRTIM_TIMEEF2_EE9FLT_SHIFT)  /* 1001: Blanking from TIMFLTR5 source */
#  define HRTIM_TIMEEF2_EE9FLT_10   (10 << HRTIM_TIMEEF2_EE9FLT_SHIFT) /* 1010: Blanking from TIMFLTR6 source */
#  define HRTIM_TIMEEF2_EE9FLT_11   (11 << HRTIM_TIMEEF2_EE9FLT_SHIFT) /* 1011: Blanking from TIMFLTR7 source */
#  define HRTIM_TIMEEF2_EE9FLT_12   (12 << HRTIM_TIMEEF2_EE9FLT_SHIFT) /* 1100: Blanking from TIMFLTR8 source */
#  define HRTIM_TIMEEF2_EE9FLT_13   (13 << HRTIM_TIMEEF2_EE9FLT_SHIFT) /* 1101: Windowing from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF2_EE9FLT_14   (14 << HRTIM_TIMEEF2_EE9FLT_SHIFT) /* 1110: Windowing from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF2_EE9FLT_15   (15 << HRTIM_TIMEEF2_EE9FLT_SHIFT) /* 1111: Windowing from TIMWIN source */
#define HRTIM_TIMEEF2_EE10LTCH      (1 << 24)                          /* Bit 24 */
#define HRTIM_TIMEEF2_EE10FLT_SHIFT 25                                 /* Bits 25-28 */
#define HRTIM_TIMEEF2_EE10FLT_MASK  (15 << HRTIM_TIMEEF2_EE10FLT_SHIFT)
#  define HRTIM_TIMEEF2_EE10FLT_0   (0 << HRTIM_TIMEEF2_EE10FLT_SHIFT)  /* 0000: No filtering */
#  define HRTIM_TIMEEF2_EE10FLT_1   (1 << HRTIM_TIMEEF2_EE10FLT_SHIFT)  /* 0001: Blanking from counter reset/roll-over to Compare 1 */
#  define HRTIM_TIMEEF2_EE10FLT_2   (2 << HRTIM_TIMEEF2_EE10FLT_SHIFT)  /* 0010: Blanking from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF2_EE10FLT_3   (3 << HRTIM_TIMEEF2_EE10FLT_SHIFT)  /* 0011: Blanking from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF2_EE10FLT_4   (4 << HRTIM_TIMEEF2_EE10FLT_SHIFT)  /* 0100: Blanking from counter reset/roll-over to Compare 4 */
#  define HRTIM_TIMEEF2_EE10FLT_5   (5 << HRTIM_TIMEEF2_EE10FLT_SHIFT)  /* 0101: Blanking from TIMFLTR1 source */
#  define HRTIM_TIMEEF2_EE10FLT_6   (6 << HRTIM_TIMEEF2_EE10FLT_SHIFT)  /* 0110: Blanking from TIMFLTR2 source */
#  define HRTIM_TIMEEF2_EE10FLT_7   (7 << HRTIM_TIMEEF2_EE10FLT_SHIFT)  /* 0111: Blanking from TIMFLTR3 source */
#  define HRTIM_TIMEEF2_EE10FLT_8   (8 << HRTIM_TIMEEF2_EE10FLT_SHIFT)  /* 1000: Blanking from TIMFLTR4 source */
#  define HRTIM_TIMEEF2_EE10FLT_9   (9 << HRTIM_TIMEEF2_EE10FLT_SHIFT)  /* 1001: Blanking from TIMFLTR5 source */
#  define HRTIM_TIMEEF2_EE10FLT_10  (10 << HRTIM_TIMEEF2_EE10FLT_SHIFT) /* 1010: Blanking from TIMFLTR6 source */
#  define HRTIM_TIMEEF2_EE10FLT_11  (11 << HRTIM_TIMEEF2_EE10FLT_SHIFT) /* 1011: Blanking from TIMFLTR7 source */
#  define HRTIM_TIMEEF2_EE10FLT_12  (12 << HRTIM_TIMEEF2_EE10FLT_SHIFT) /* 1100: Blanking from TIMFLTR8 source */
#  define HRTIM_TIMEEF2_EE10FLT_13  (13 << HRTIM_TIMEEF2_EE10FLT_SHIFT) /* 1101: Windowing from counter reset/roll-over to Compare 2 */
#  define HRTIM_TIMEEF2_EE10FLT_14  (14 << HRTIM_TIMEEF2_EE10FLT_SHIFT) /* 1110: Windowing from counter reset/roll-over to Compare 3 */
#  define HRTIM_TIMEEF2_EE10FLT_15  (15 << HRTIM_TIMEEF2_EE10FLT_SHIFT) /* 1111: Windowing from TIMWIN source */

/* Timer X Reset Register */

#define HRTIM_TIMARST_UPDT            (1 << 1)  /* Bit 1 */
#define HRTIM_TIMARST_CMP2            (1 << 2)  /* Bit 2 */
#define HRTIM_TIMARST_CMP4            (1 << 3)  /* Bit 3 */
#define HRTIM_TIMARST_MSTPER          (1 << 4)  /* Bit 4 */
#define HRTIM_TIMARST_MSTCMP1         (1 << 5)  /* Bit 5 */
#define HRTIM_TIMARST_MSTCMP2         (1 << 6)  /* Bit 6 */
#define HRTIM_TIMARST_MSTCMP3         (1 << 7)  /* Bit 7 */
#define HRTIM_TIMARST_MSTCMP4         (1 << 8)  /* Bit 8 */
#define HRTIM_TIMARST_EXTEVNT1        (1 << 9)  /* Bit 9 */
#define HRTIM_TIMARST_EXTEVNT2        (1 << 10) /* Bit 10 */
#define HRTIM_TIMARST_EXTEVNT3        (1 << 11) /* Bit 11 */
#define HRTIM_TIMARST_EXTEVNT4        (1 << 12) /* Bit 12 */
#define HRTIM_TIMARST_EXTEVNT5        (1 << 13) /* Bit 13 */
#define HRTIM_TIMARST_EXTEVNT6        (1 << 14) /* Bit 14 */
#define HRTIM_TIMARST_EXTEVNT7        (1 << 15) /* Bit 15 */
#define HRTIM_TIMARST_EXTEVNT8        (1 << 16) /* Bit 16 */
#define HRTIM_TIMARST_EXTEVNT9        (1 << 17) /* Bit 17 */
#define HRTIM_TIMARST_EXTEVNT10       (1 << 18) /* Bit 18 */
#define HRTIM_TIMARST_TIMBCMP1        (1 << 19) /* Bit 19 */
#define HRTIM_TIMARST_TIMBCMP2        (1 << 20) /* Bit 20 */
#define HRTIM_TIMARST_TIMBCMP4        (1 << 21) /* Bit 21 */
#define HRTIM_TIMARST_TIMCCMP1        (1 << 22) /* Bit 22 */
#define HRTIM_TIMARST_TIMCCMP2        (1 << 23) /* Bit 23 */
#define HRTIM_TIMARST_TIMCCMP4        (1 << 24) /* Bit 24 */
#define HRTIM_TIMARST_TIMDCMP1        (1 << 25) /* Bit 25 */
#define HRTIM_TIMARST_TIMDCMP2        (1 << 26) /* Bit 26 */
#define HRTIM_TIMARST_TIMDCMP4        (1 << 27) /* Bit 27 */
#define HRTIM_TIMARST_TIMECMP1        (1 << 28) /* Bit 28 */
#define HRTIM_TIMARST_TIMECMP2        (1 << 29) /* Bit 29 */
#define HRTIM_TIMARST_TIMECMP4        (1 << 30) /* Bit 30 */

#define HRTIM_TIMBRST_UPDT            (1 << 1)  /* Bit 1 */
#define HRTIM_TIMBRST_CMP2            (1 << 2)  /* Bit 2 */
#define HRTIM_TIMBRST_CMP4            (1 << 3)  /* Bit 3 */
#define HRTIM_TIMBRST_MSTPER          (1 << 4)  /* Bit 4 */
#define HRTIM_TIMBRST_MSTCMP1         (1 << 5)  /* Bit 5 */
#define HRTIM_TIMBRST_MSTCMP2         (1 << 6)  /* Bit 6 */
#define HRTIM_TIMBRST_MSTCMP3         (1 << 7)  /* Bit 7 */
#define HRTIM_TIMBRST_MSTCMP4         (1 << 8)  /* Bit 8 */
#define HRTIM_TIMBRST_EXTEVNT1        (1 << 9)  /* Bit 9 */
#define HRTIM_TIMBRST_EXTEVNT2        (1 << 10) /* Bit 10 */
#define HRTIM_TIMBRST_EXTEVNT3        (1 << 11) /* Bit 11 */
#define HRTIM_TIMBRST_EXTEVNT4        (1 << 12) /* Bit 12 */
#define HRTIM_TIMBRST_EXTEVNT5        (1 << 13) /* Bit 13 */
#define HRTIM_TIMBRST_EXTEVNT6        (1 << 14) /* Bit 14 */
#define HRTIM_TIMBRST_EXTEVNT7        (1 << 15) /* Bit 15 */
#define HRTIM_TIMBRST_EXTEVNT8        (1 << 16) /* Bit 16 */
#define HRTIM_TIMBRST_EXTEVNT9        (1 << 17) /* Bit 17 */
#define HRTIM_TIMBRST_EXTEVNT10       (1 << 18) /* Bit 18 */
#define HRTIM_TIMBRST_TIMACMP1        (1 << 19) /* Bit 19 */
#define HRTIM_TIMBRST_TIMACMP2        (1 << 20) /* Bit 20 */
#define HRTIM_TIMBRST_TIMACMP4        (1 << 21) /* Bit 21 */
#define HRTIM_TIMBRST_TIMCCMP1        (1 << 22) /* Bit 22 */
#define HRTIM_TIMBRST_TIMCCMP2        (1 << 23) /* Bit 23 */
#define HRTIM_TIMBRST_TIMCCMP4        (1 << 24) /* Bit 24 */
#define HRTIM_TIMBRST_TIMDCMP1        (1 << 25) /* Bit 25 */
#define HRTIM_TIMBRST_TIMDCMP2        (1 << 26) /* Bit 26 */
#define HRTIM_TIMBRST_TIMDCMP4        (1 << 27) /* Bit 27 */
#define HRTIM_TIMBRST_TIMECMP1        (1 << 28) /* Bit 28 */
#define HRTIM_TIMBRST_TIMECMP2        (1 << 29) /* Bit 29 */
#define HRTIM_TIMBRST_TIMECMP4        (1 << 30) /* Bit 30 */

#define HRTIM_TIMCRST_UPDT            (1 << 1)  /* Bit 1 */
#define HRTIM_TIMCRST_CMP2            (1 << 2)  /* Bit 2 */
#define HRTIM_TIMCRST_CMP4            (1 << 3)  /* Bit 3 */
#define HRTIM_TIMCRST_MSTPER          (1 << 4)  /* Bit 4 */
#define HRTIM_TIMCRST_MSTCMP1         (1 << 5)  /* Bit 5 */
#define HRTIM_TIMCRST_MSTCMP2         (1 << 6)  /* Bit 6 */
#define HRTIM_TIMCRST_MSTCMP3         (1 << 7)  /* Bit 7 */
#define HRTIM_TIMCRST_MSTCMP4         (1 << 8)  /* Bit 8 */
#define HRTIM_TIMCRST_EXTEVNT1        (1 << 9)  /* Bit 9 */
#define HRTIM_TIMCRST_EXTEVNT2        (1 << 10) /* Bit 10 */
#define HRTIM_TIMCRST_EXTEVNT3        (1 << 11) /* Bit 11 */
#define HRTIM_TIMCRST_EXTEVNT4        (1 << 12) /* Bit 12 */
#define HRTIM_TIMCRST_EXTEVNT5        (1 << 13) /* Bit 13 */
#define HRTIM_TIMCRST_EXTEVNT6        (1 << 14) /* Bit 14 */
#define HRTIM_TIMCRST_EXTEVNT7        (1 << 15) /* Bit 15 */
#define HRTIM_TIMCRST_EXTEVNT8        (1 << 16) /* Bit 16 */
#define HRTIM_TIMCRST_EXTEVNT9        (1 << 17) /* Bit 17 */
#define HRTIM_TIMCRST_EXTEVNT10       (1 << 18) /* Bit 18 */
#define HRTIM_TIMCRST_TIMACMP1        (1 << 19) /* Bit 19 */
#define HRTIM_TIMCRST_TIMACMP2        (1 << 20) /* Bit 20 */
#define HRTIM_TIMCRST_TIMACMP4        (1 << 21) /* Bit 21 */
#define HRTIM_TIMCRST_TIMBCMP1        (1 << 22) /* Bit 22 */
#define HRTIM_TIMCRST_TIMBCMP2        (1 << 23) /* Bit 23 */
#define HRTIM_TIMCRST_TIMBCMP4        (1 << 24) /* Bit 24 */
#define HRTIM_TIMCRST_TIMDCMP1        (1 << 25) /* Bit 25 */
#define HRTIM_TIMCRST_TIMDCMP2        (1 << 26) /* Bit 26 */
#define HRTIM_TIMCRST_TIMDCMP4        (1 << 27) /* Bit 27 */
#define HRTIM_TIMCRST_TIMECMP1        (1 << 28) /* Bit 28 */
#define HRTIM_TIMCRST_TIMECMP2        (1 << 29) /* Bit 29 */
#define HRTIM_TIMCRST_TIMECMP4        (1 << 30) /* Bit 30 */

#define HRTIM_TIMDRST_UPDT            (1 << 1)  /* Bit 1 */
#define HRTIM_TIMDRST_CMP2            (1 << 2)  /* Bit 2 */
#define HRTIM_TIMDRST_CMP4            (1 << 3)  /* Bit 3 */
#define HRTIM_TIMDRST_MSTPER          (1 << 4)  /* Bit 4 */
#define HRTIM_TIMDRST_MSTCMP1         (1 << 5)  /* Bit 5 */
#define HRTIM_TIMDRST_MSTCMP2         (1 << 6)  /* Bit 6 */
#define HRTIM_TIMDRST_MSTCMP3         (1 << 7)  /* Bit 7 */
#define HRTIM_TIMDRST_MSTCMP4         (1 << 8)  /* Bit 8 */
#define HRTIM_TIMDRST_EXTEVNT1        (1 << 9)  /* Bit 9 */
#define HRTIM_TIMDRST_EXTEVNT2        (1 << 10) /* Bit 10 */
#define HRTIM_TIMDRST_EXTEVNT3        (1 << 11) /* Bit 11 */
#define HRTIM_TIMDRST_EXTEVNT4        (1 << 12) /* Bit 12 */
#define HRTIM_TIMDRST_EXTEVNT5        (1 << 13) /* Bit 13 */
#define HRTIM_TIMDRST_EXTEVNT6        (1 << 14) /* Bit 14 */
#define HRTIM_TIMDRST_EXTEVNT7        (1 << 15) /* Bit 15 */
#define HRTIM_TIMDRST_EXTEVNT8        (1 << 16) /* Bit 16 */
#define HRTIM_TIMDRST_EXTEVNT9        (1 << 17) /* Bit 17 */
#define HRTIM_TIMDRST_EXTEVNT10       (1 << 18) /* Bit 18 */
#define HRTIM_TIMDRST_TIMACMP1        (1 << 19) /* Bit 19 */
#define HRTIM_TIMDRST_TIMACMP2        (1 << 20) /* Bit 20 */
#define HRTIM_TIMDRST_TIMACMP4        (1 << 21) /* Bit 21 */
#define HRTIM_TIMDRST_TIMBCMP1        (1 << 22) /* Bit 22 */
#define HRTIM_TIMDRST_TIMBCMP2        (1 << 23) /* Bit 23 */
#define HRTIM_TIMDRST_TIMBCMP4        (1 << 24) /* Bit 24 */
#define HRTIM_TIMDRST_TIMCCMP1        (1 << 25) /* Bit 25 */
#define HRTIM_TIMDRST_TIMCCMP2        (1 << 26) /* Bit 26 */
#define HRTIM_TIMDRST_TIMCCMP4        (1 << 27) /* Bit 27 */
#define HRTIM_TIMDRST_TIMECMP1        (1 << 28) /* Bit 28 */
#define HRTIM_TIMDRST_TIMECMP2        (1 << 29) /* Bit 29 */
#define HRTIM_TIMDRST_TIMECMP4        (1 << 30) /* Bit 30 */

#define HRTIM_TIMERST_UPDT            (1 << 1)  /* Bit 1 */
#define HRTIM_TIMERST_CMP2            (1 << 2)  /* Bit 2 */
#define HRTIM_TIMERST_CMP4            (1 << 3)  /* Bit 3 */
#define HRTIM_TIMERST_MSTPER          (1 << 4)  /* Bit 4 */
#define HRTIM_TIMERST_MSTCMP1         (1 << 5)  /* Bit 5 */
#define HRTIM_TIMERST_MSTCMP2         (1 << 6)  /* Bit 6 */
#define HRTIM_TIMERST_MSTCMP3         (1 << 7)  /* Bit 7 */
#define HRTIM_TIMERST_MSTCMP4         (1 << 8)  /* Bit 8 */
#define HRTIM_TIMERST_EXTEVNT1        (1 << 9)  /* Bit 9 */
#define HRTIM_TIMERST_EXTEVNT2        (1 << 10) /* Bit 10 */
#define HRTIM_TIMERST_EXTEVNT3        (1 << 11) /* Bit 11 */
#define HRTIM_TIMERST_EXTEVNT4        (1 << 12) /* Bit 12 */
#define HRTIM_TIMERST_EXTEVNT5        (1 << 13) /* Bit 13 */
#define HRTIM_TIMERST_EXTEVNT6        (1 << 14) /* Bit 14 */
#define HRTIM_TIMERST_EXTEVNT7        (1 << 15) /* Bit 15 */
#define HRTIM_TIMERST_EXTEVNT8        (1 << 16) /* Bit 16 */
#define HRTIM_TIMERST_EXTEVNT9        (1 << 17) /* Bit 17 */
#define HRTIM_TIMERST_EXTEVNT10       (1 << 18) /* Bit 18 */
#define HRTIM_TIMERST_TIMACMP1        (1 << 19) /* Bit 19 */
#define HRTIM_TIMERST_TIMACMP2        (1 << 20) /* Bit 20 */
#define HRTIM_TIMERST_TIMACMP4        (1 << 21) /* Bit 21 */
#define HRTIM_TIMERST_TIMBCMP1        (1 << 22) /* Bit 22 */
#define HRTIM_TIMERST_TIMBCMP2        (1 << 23) /* Bit 23 */
#define HRTIM_TIMERST_TIMBCMP4        (1 << 24) /* Bit 24 */
#define HRTIM_TIMERST_TIMCCMP1        (1 << 25) /* Bit 25 */
#define HRTIM_TIMERST_TIMCCMP2        (1 << 26) /* Bit 26 */
#define HRTIM_TIMERST_TIMCCMP4        (1 << 27) /* Bit 27 */
#define HRTIM_TIMERST_TIMDCMP1        (1 << 28) /* Bit 28 */
#define HRTIM_TIMERST_TIMDCMP2        (1 << 29) /* Bit 29 */
#define HRTIM_TIMERST_TIMDCMP4        (1 << 30) /* Bit 30 */

/* Timer X Chopper Register */

#define HRTIM_TIMCHP_CARFRQ_SHIFT     0 /* Bits 0-3: Chopper carrier frequency */
#define HRTIM_TIMCHP_CARFRQ_MASK      (15 << HRTIM_TIMCHP_CARFRQ_SHIFT)

#define HRTIM_TIMCHP_CARDTY_SHIFT     4 /* Bits 4-6: Chopper duty cycle */
#define HRTIM_TIMCHP_CARDTY_MASK      (7 << HRTIM_TIMCHP_CARDTY_SHIFT)

#define HRTIM_TIMCHP_STRTPW_SHIFT     7 /* Bits 7-10: Chopper start pulsewidth */
#define HRTIM_TIMCHP_STRTPW_MASK      (15 << HRTIM_TIMCHP_STRTPW_SHIFT)

/* Timer X Capture 12 Control Register */

#define HRTIM_TIMCPT12CR_SWCPT        (1 << 0)
#define HRTIM_TIMCPT12CR_UPDCPT       (1 << 1)
#define HRTIM_TIMCPT12CR_EXEV1CPT     (1 << 2)
#define HRTIM_TIMCPT12CR_EXEV2CPT     (1 << 3)
#define HRTIM_TIMCPT12CR_EXEV3CPT     (1 << 4)
#define HRTIM_TIMCPT12CR_EXEV4CPT     (1 << 5)
#define HRTIM_TIMCPT12CR_EXEV5CPT     (1 << 6)
#define HRTIM_TIMCPT12CR_EXEV6CPT     (1 << 7)
#define HRTIM_TIMCPT12CR_EXEV7CPT     (1 << 8)
#define HRTIM_TIMCPT12CR_EXEV8CPT     (1 << 9)
#define HRTIM_TIMCPT12CR_EXEV9CPT     (1 << 10)
#define HRTIM_TIMCPT12CR_EXEV10CPT    (1 << 11)
#define HRTIM_TIMCPT12CR_TA1SET       (1 << 12)
#define HRTIM_TIMCPT12CR_TA1RST       (1 << 13)
#define HRTIM_TIMCPT12CR_TACMP1       (1 << 14)
#define HRTIM_TIMCPT12CR_TACMP2       (1 << 15)
#define HRTIM_TIMCPT12CR_TB1SET       (1 << 16)
#define HRTIM_TIMCPT12CR_TB1RST       (1 << 17)
#define HRTIM_TIMCPT12CR_TBCMP1       (1 << 18)
#define HRTIM_TIMCPT12CR_TBCMP2       (1 << 19)
#define HRTIM_TIMCPT12CR_TC1SET       (1 << 20)
#define HRTIM_TIMCPT12CR_TC1RST       (1 << 21)
#define HRTIM_TIMCPT12CR_TCCMP1       (1 << 22)
#define HRTIM_TIMCPT12CR_TCCMP2       (1 << 23)
#define HRTIM_TIMCPT12CR_TD1SET       (1 << 24)
#define HRTIM_TIMCPT12CR_TD1RST       (1 << 25)
#define HRTIM_TIMCPT12CR_TDCMP1       (1 << 26)
#define HRTIM_TIMCPT12CR_TDCMP2       (1 << 27)
#define HRTIM_TIMCPT12CR_TE1SET       (1 << 28)
#define HRTIM_TIMCPT12CR_TE1RST       (1 << 29)
#define HRTIM_TIMCPT12CR_TECMP1       (1 << 30)
#define HRTIM_TIMCPT12CR_TECMP2       (1 << 31)

/* Timer X Output Register */

#define HRTIM_TIMOUT_POL1              (1 << 1)                         /* Bit 1: Output 1 polarity */
#define HRTIM_TIMOUT_IDLEM1            (1 << 2)                         /* Bit 2: Output 1 IDLE mode */
#define HRTIM_TIMOUT_IDLES1            (1 << 3)                         /* Bit 3: Output 1 IDLE state*/
#define HRTIM_TIMOUT_FAULT1_SHIFT      4                                /* Bit 4-5: Output 1 Fault state */
#define HRTIM_TIMOUT_FAULT1_MASK       (3 << HRTIM_TIMOUT_FAULT1_SHIFT)
#  define HRTIM_TIMOUT_FAULT1_0        (0 << HRTIM_TIMOUT_FAULT1_SHIFT) /* 00: No action */
#  define HRTIM_TIMOUT_FAULT1_1        (1 << HRTIM_TIMOUT_FAULT1_SHIFT) /* 01: Active */
#  define HRTIM_TIMOUT_FAULT1_2        (2 << HRTIM_TIMOUT_FAULT1_SHIFT) /* 10: Inactive */
#  define HRTIM_TIMOUT_FAULT1_3        (3 << HRTIM_TIMOUT_FAULT1_SHIFT) /* 11: High-Z */
#define HRTIM_TIMOUT_CHP1              (1 << 6)                         /* Bit 6: Output 1 Chopper enable */
#define HRTIM_TIMOUT_DIDL1             (1 << 7)                         /* Bit 7: Output 1 Deadtime upon burst mode IDLE entry */
#define HRTIM_TIMOUT_DTEN              (1 << 8)                         /* Bit 8: Deadtime enable */
#define HRTIM_TIMOUT_DLYPRTEN          (1 << 9)                         /* Bit 9: Delayed Protection enable */
#define HRTIM_TIMOUT_DLYPRT_SHIFT      10                               /* Bits 10-12: Delayed Protection*/
#define HRTIM_TIMOUT_DLYPRT_MASK       (3 << HRTIM_TIMOUT_DLYPRT_SHIFT)
#  define HRTIM_TIMOUT_DLYPRT_0        (0 << HRTIM_TIMOUT_DLYPRT_SHIFT) /* 000: */
#  define HRTIM_TIMOUT_DLYPRT_1        (1 << HRTIM_TIMOUT_DLYPRT_SHIFT) /* 001: */
#  define HRTIM_TIMOUT_DLYPRT_2        (2 << HRTIM_TIMOUT_DLYPRT_SHIFT) /* 010: */
#  define HRTIM_TIMOUT_DLYPRT_3        (3 << HRTIM_TIMOUT_DLYPRT_SHIFT) /* 011: */
#  define HRTIM_TIMOUT_DLYPRT_4        (4 << HRTIM_TIMOUT_DLYPRT_SHIFT) /* 100: */
#  define HRTIM_TIMOUT_DLYPRT_5        (5 << HRTIM_TIMOUT_DLYPRT_SHIFT) /* 101: */
#  define HRTIM_TIMOUT_DLYPRT_6        (6 << HRTIM_TIMOUT_DLYPRT_SHIFT) /* 110: */
#  define HRTIM_TIMOUT_DLYPRT_7        (7 << HRTIM_TIMOUT_DLYPRT_SHIFT) /* 111: */
                                                                        /* Bit 12-16: Resered  */
#define HRTIM_TIMOUT_POL2             (1 << 17)                         /* Bit 17: Output 2 polarity */
#define HRTIM_TIMOUT_IDLEM2           (1 << 18)                         /* Bit 18: Output 2 IDLE mode */
#define HRTIM_TIMOUT_IDLES2           (1 << 19)                         /* Bit 19: Output 2 IDLE state */
#define HRTIM_TIMOUT_FAULT2_SHIFT     20                                /* Bit 20-21: Output 2 Fault state */
#define HRTIM_TIMOUT_FAULT2_MASK      (3 << HRTIM_TIMOUT_FAULT2_SHIFT)
#  define HRTIM_TIMOUT_FAULT2_0       (0 << HRTIM_TIMOUT_FAULT2_SHIFT)  /* 00: No action*/
#  define HRTIM_TIMOUT_FAULT2_1       (1 << HRTIM_TIMOUT_FAULT2_SHIFT)  /* 01: Active */
#  define HRTIM_TIMOUT_FAULT2_2       (2 << HRTIM_TIMOUT_FAULT2_SHIFT)  /* 10: Inactive */
#  define HRTIM_TIMOUT_FAULT2_3       (3 << HRTIM_TIMOUT_FAULT2_SHIFT)  /* 11: High-Z*/
#define HRTIM_TIMOUT_CHP2             (1 << 22)                         /* Bit 22: Output 2 Chopper enable */
#define HRTIM_TIMOUT_DIDL2            (1 << 23)                         /* Bit 23: Output 2 Deadtime upon burst mode IDLE entry */

/* Timer X Fault Register */

#define HRTIM_TIMFLT_FLT1EN           (1 << 0)  /* Bit 0: Fault1 enable */
#define HRTIM_TIMFLT_FLT2EN           (1 << 1)  /* Bit 1: Fault 2 enable */
#define HRTIM_TIMFLT_FLT3EN           (1 << 2)  /* Bit 2: Fault 3 enable*/
#define HRTIM_TIMFLT_FLT4EN           (1 << 3)  /* Bit 3: Fault 4 enable */
#define HRTIM_TIMFLT_FLT5EN           (1 << 4)  /* Bit 4: Fault 5 enable */
#define HRTIM_TIMFLT_FLTLCK           (1 << 31) /* Bit 31: Fault sources lock*/

/* Common Control Register 1 */

#define HRTIM_CR1_MUDIS               (1 << 0) /* Bit 0: Master Update Disable */
#define HRTIM_CR1_TAUDIS              (1 << 1) /* Bit 1: Timer A Update Disable */
#define HRTIM_CR1_TBUDIS              (1 << 2) /* Bit 2: Timer B Update Disable */
#define HRTIM_CR1_TCUDIS              (1 << 3) /* Bit 3: Timer C Update Disable */
#define HRTIM_CR1_TDUDIS              (1 << 4) /* Bit 4: Timer D Update Disable */
#define HRTIM_CR1_TEUDIS              (1 << 5) /* Bit 5: Timer E Update Disable */
#define HRTIM_CR1_AD1USRC_SHIFT       16       /* Bits 16-18: ADC Trigger 1 Update Source  */
#define HRTIM_CR1_AD1USRC_MASK        (7 << HRTIM_CR1_AD1USRC_SHIFT)
#  define HRTIM_CR1_AD1USRC_MT        (0 << HRTIM_CR1_AD1USRC_SHIFT) /* 000: Master Timer */
#  define HRTIM_CR1_AD1USRC_TA        (1 << HRTIM_CR1_AD1USRC_SHIFT) /* 001: Timer A */
#  define HRTIM_CR1_AD1USRC_TB        (2 << HRTIM_CR1_AD1USRC_SHIFT) /* 010: Timer B */
#  define HRTIM_CR1_AD1USRC_TC        (3 << HRTIM_CR1_AD1USRC_SHIFT) /* 011: Timer C */
#  define HRTIM_CR1_AD1USRC_TD        (4 << HRTIM_CR1_AD1USRC_SHIFT) /* 100: Timer D */
#  define HRTIM_CR1_AD1USRC_TE        (5 << HRTIM_CR1_AD1USRC_SHIFT) /* 101: Timer A */
#define HRTIM_CR1_AD2USRC_SHIFT       19                             /* Bits 19-21: ADC Trigger 2 Update Source  */
#define HRTIM_CR1_AD2USRC_MASK        (7 << HRTIM_CR1_AD2USRC_SHIFT)
#  define HRTIM_CR1_AD2USRC_MT        (0 << HRTIM_CR1_AD2USRC_SHIFT) /* 000: Master Timer */
#  define HRTIM_CR1_AD2USRC_TA        (1 << HRTIM_CR1_AD2USRC_SHIFT) /* 001: Timer A */
#  define HRTIM_CR1_AD2USRC_TB        (2 << HRTIM_CR1_AD2USRC_SHIFT) /* 010: Timer B */
#  define HRTIM_CR1_AD2USRC_TC        (3 << HRTIM_CR1_AD2USRC_SHIFT) /* 011: Timer C */
#  define HRTIM_CR1_AD2USRC_TD        (4 << HRTIM_CR1_AD2USRC_SHIFT) /* 100: Timer D */
#  define HRTIM_CR1_AD2USRC_TE        (5 << HRTIM_CR1_AD2USRC_SHIFT) /* 101: Timer A */
#define HRTIM_CR1_AD3USRC_SHIFT       22                             /* Bits 22-24: ADC Trigger 3 Update Source  */
#define HRTIM_CR1_AD3USRC_MASK        (7 << HRTIM_CR1_AD3USRC_SHIFT)
#  define HRTIM_CR1_AD3USRC_MT        (0 << HRTIM_CR1_AD3USRC_SHIFT) /* 000: Master Timer */
#  define HRTIM_CR1_AD3USRC_TA        (1 << HRTIM_CR1_AD3USRC_SHIFT) /* 001: Timer A */
#  define HRTIM_CR1_AD3USRC_TB        (2 << HRTIM_CR1_AD3USRC_SHIFT) /* 010: Timer B */
#  define HRTIM_CR1_AD3USRC_TC        (3 << HRTIM_CR1_AD3USRC_SHIFT) /* 011: Timer C */
#  define HRTIM_CR1_AD3USRC_TD        (4 << HRTIM_CR1_AD3USRC_SHIFT) /* 100: Timer D */
#  define HRTIM_CR1_AD3USRC_TE        (5 << HRTIM_CR1_AD3USRC_SHIFT) /* 101: Timer A */
#define HRTIM_CR1_AD4USRC_SHIFT       25                             /* Bits 25-27: ADC Trigger 4 Update Source  */
#define HRTIM_CR1_AD4USRC_MASK        (7 << HRTIM_CR1_AD4USRC_SHIFT)
#  define HRTIM_CR1_AD4USRC_MT        (0 << HRTIM_CR1_AD4USRC_SHIFT) /* 000: Master Timer */
#  define HRTIM_CR1_AD4USRC_TA        (1 << HRTIM_CR1_AD4USRC_SHIFT) /* 001: Timer A */
#  define HRTIM_CR1_AD4USRC_TB        (2 << HRTIM_CR1_AD4USRC_SHIFT) /* 010: Timer B */
#  define HRTIM_CR1_AD4USRC_TC        (3 << HRTIM_CR1_AD4USRC_SHIFT) /* 011: Timer C */
#  define HRTIM_CR1_AD4USRC_TD        (4 << HRTIM_CR1_AD4USRC_SHIFT) /* 100: Timer D */
#  define HRTIM_CR1_AD4USRC_TE        (5 << HRTIM_CR1_AD4USRC_SHIFT) /* 101: Timer A */

/* Common Control Register 2 */

#define HRTIM_CR2_MSWU                (1 << 0)  /* Bit 0: Master Timer Software Update */
#define HRTIM_CR2_TASWU               (1 << 1)  /* Bit 1: Timer A Software Update */
#define HRTIM_CR2_TBSWU               (1 << 2)  /* Bit 2: Timer B Software Update */
#define HRTIM_CR2_TCSWU               (1 << 3)  /* Bit 3: Timer C Software Update */
#define HRTIM_CR2_TDSWU               (1 << 4)  /* Bit 4: Timer D Software Update */
#define HRTIM_CR2_TESWU               (1 << 5)  /* Bit 5: Timer E Software Update */
#define HRTIM_CR2_MRST                (1 << 8)  /* Bit 8: Master Counter Software Reset*/
#define HRTIM_CR2_TARST               (1 << 9)  /* Bit 9: Timer A Counter Software Reset*/
#define HRTIM_CR2_TBRST               (1 << 10) /* Bit 10: Timer B Counter Software Reset*/
#define HRTIM_CR2_TCRST               (1 << 11) /* Bit 11: Timer C Counter Software Reset*/
#define HRTIM_CR2_TDRST               (1 << 12) /* Bit 12: Timer D Counter Software Reset*/
#define HRTIM_CR2_TERST               (1 << 13) /* Bit 13: Timer E Counter Software Reset*/

/* Common Interrupt Status Register */

#define HRTIM_ISR_FLT1                (1 << 0)  /* Bit 0: Fault 1 Interrupt Flag */
#define HRTIM_ISR_FLT2                (1 << 1)  /* Bit 1: Fault 2 Interrupt Flag */
#define HRTIM_ISR_FLT3                (1 << 2)  /* Bit 2: Fault 3 Interrupt Flag */
#define HRTIM_ISR_FLT4                (1 << 3)  /* Bit 3: Fault 4 Interrupt Flag */
#define HRTIM_ISR_FLT5                (1 << 4)  /* Bit 4: Fault 5 Interrupt Flag */
#define HRTIM_ISR_SYSFLT              (1 << 5)  /* Bit 5: System Fault Interrupt Flag */
#define HRTIM_ISR_DLLRDY              (1 << 16) /* Bit 16: DLL Ready Interrupt Flag */
#define HRTIM_ISR_BMPER               (1 << 17) /* Bit 17: Burst mode Period Interrupt Flag */

/* Common Interrupt Clear Register */

#define HRTIM_ICR_FLT1C                (1 << 0)  /* Bit 0: Fault 1 Interrupt Flag Clear */
#define HRTIM_ICR_FLT2C                (1 << 1)  /* Bit 1: Fault 2 Interrupt Flag Clear */
#define HRTIM_ICR_FLT3C                (1 << 2)  /* Bit 2: Fault 3 Interrupt Flag Clear */
#define HRTIM_ICR_FLT4C                (1 << 3)  /* Bit 3: Fault 4 Interrupt Flag Clear */
#define HRTIM_ICR_FLT5C                (1 << 4)  /* Bit 4: Fault 5 Interrupt Flag Clear */
#define HRTIM_ICR_SYSFLTC              (1 << 5)  /* Bit 5: System Fault Interrupt Flag Clear */
#define HRTIM_ICR_DLLRDYC              (1 << 16) /* Bit 16: DLL Ready Interrupt Flag Clear */
#define HRTIM_ICR_BMPERC               (1 << 17) /* Bit 17: Burst mode Period Interrupt Flag Clear */

/* Common Interrupt Enable Register */

#define HRTIM_IER_FLT1IE               (1 << 0)  /* Bit 0: Fault 1 Interrupt Enable */
#define HRTIM_IER_FLT2IE               (1 << 1)  /* Bit 1: Fault 2 Interrupt Enable */
#define HRTIM_IER_FLT3IE               (1 << 2)  /* Bit 2: Fault 3 Interrupt Enable */
#define HRTIM_IER_FLT4IE               (1 << 3)  /* Bit 3: Fault 4 Interrupt Enable */
#define HRTIM_IER_FLT5IE               (1 << 4)  /* Bit 4: Fault 5 Interrupt Enable */
#define HRTIM_IER_SYSFLTIE             (1 << 5)  /* Bit 5: System Fault Interrupt Enable */
#define HRTIM_IER_DLLRDYIE             (1 << 16) /* Bit 16: DLL Ready Interrupt Enable */
#define HRTIM_IER_BMPERIE              (1 << 17) /* Bit 17: Burst mode Period Interrupt Enable */

/* Common Output Enable Register */

#define HRTIM_OENR_TA1OE                (1 << 0) /* Bit 0: Timer A Output 1 Enable */
#define HRTIM_OENR_TA2OE                (1 << 1) /* Bit 1: Timer A Output 2 Enable */
#define HRTIM_OENR_TB1OE                (1 << 2) /* Bit 2: Timer B Output 1 Enable */
#define HRTIM_OENR_TB2OE                (1 << 3) /* Bit 3: Timer B Output 2 Enable */
#define HRTIM_OENR_TC1OE                (1 << 4) /* Bit 4: Timer C Output 1 Enable */
#define HRTIM_OENR_TC2OE                (1 << 5) /* Bit 5: Timer C Output 2 Enable */
#define HRTIM_OENR_TD1OE                (1 << 6) /* Bit 6: Timer D Output 1 Enable */
#define HRTIM_OENR_TD2OE                (1 << 7) /* Bit 7: Timer D Output 2 Enable */
#define HRTIM_OENR_TE1OE                (1 << 8) /* Bit 8: Timer E Output 1 Enable */
#define HRTIM_OENR_TE2OE                (1 << 9) /* Bit 9: Timer E Output 2 Enable */

/* Common Output Disable Register */

#define HRTIM_ODISR_TA1ODIS             (1 << 0) /* Bit 0: Timer A Output 1 Disable */
#define HRTIM_ODISR_TA2ODIS             (1 << 1) /* Bit 1: Timer A Output 2 Disable */
#define HRTIM_ODISR_TB1ODIS             (1 << 2) /* Bit 2: Timer B Output 1 Disable */
#define HRTIM_ODISR_TB2ODIS             (1 << 3) /* Bit 3: Timer B Output 2 Disable */
#define HRTIM_ODISR_TC1ODIS             (1 << 4) /* Bit 4: Timer C Output 1 Disable */
#define HRTIM_ODISR_TC2ODIS             (1 << 5) /* Bit 5: Timer C Output 2 Disable */
#define HRTIM_ODISR_TD1ODIS             (1 << 6) /* Bit 6: Timer D Output 1 Disable */
#define HRTIM_ODISR_TD2ODIS             (1 << 7) /* Bit 7: Timer D Output 2 Disable */
#define HRTIM_ODISR_TE1ODIS             (1 << 8) /* Bit 8: Timer E Output 1 Disable */
#define HRTIM_ODISR_TE2ODIS             (1 << 9) /* Bit 9: Timer E Output 2 Disable */

/* Common Output Disable Status Register */

#define HRTIM_ODSR_TA1ODS               (1 << 0) /* Bit 0: Timer A Output 1 Disable Status */
#define HRTIM_ODSR_TA2ODS               (1 << 1) /* Bit 1: Timer A Output 2 Disable Status */
#define HRTIM_ODSR_TB1ODS               (1 << 2) /* Bit 2: Timer B Output 1 Disable Status */
#define HRTIM_ODSR_TB2ODS               (1 << 3) /* Bit 3: Timer B Output 2 Disable Status */
#define HRTIM_ODSR_TC1ODS               (1 << 4) /* Bit 4: Timer C Output 1 Disable Status */
#define HRTIM_ODSR_TC2ODS               (1 << 5) /* Bit 5: Timer C Output 2 Disable Status */
#define HRTIM_ODSR_TD1ODS               (1 << 6) /* Bit 6: Timer D Output 1 Disable Status */
#define HRTIM_ODSR_TD2ODS               (1 << 7) /* Bit 7: Timer D Output 2 Disable Status */
#define HRTIM_ODSR_TE1ODS               (1 << 8) /* Bit 8: Timer E Output 1 Disable Status */
#define HRTIM_ODSR_TE2ODS               (1 << 9) /* Bit 9: Timer E Output 2 Disable Status */

/* Common Burst Mode Control Register */

#define HRTIM_BMCR_BME                  (1 << 0) /* Bit 0: Burst Mode Enable */
#define HRTIM_BMCR_BMOM                 (1 << 1) /* Bit 1: Burst Mode Operating Mode */
#define HRTIM_BMCR_BMCLK_SHIFT          2        /* Bits 2-5: Burst Mode Clock Source */
#define HRTIM_BMCR_BMCLK_MASK           (15 << HRTIM_BMCR_BMCLK_SHIFT)
#  define HRTIM_BMCR_BMCLK_0            (0 << HRTIM_BMCR_BMCLK_SHIFT)  /* 0000: Master Timer Counter Reset/roll-over */
#  define HRTIM_BMCR_BMCLK_1            (1 << HRTIM_BMCR_BMCLK_SHIFT)  /* 0001: Timer A counter reset/roll-over */
#  define HRTIM_BMCR_BMCLK_2            (2 << HRTIM_BMCR_BMCLK_SHIFT)  /* 0010: Timer B counter reset/roll-over */
#  define HRTIM_BMCR_BMCLK_3            (3 << HRTIM_BMCR_BMCLK_SHIFT)  /* 0011: Timer C counter reset/roll-over */
#  define HRTIM_BMCR_BMCLK_4            (4 << HRTIM_BMCR_BMCLK_SHIFT)  /* 0100: Timer D counter reset/roll-over */
#  define HRTIM_BMCR_BMCLK_5            (5 << HRTIM_BMCR_BMCLK_SHIFT)  /* 0101: Timer E counter reset/roll-over */
#  define HRTIM_BMCR_BMCLK_6            (6 << HRTIM_BMCR_BMCLK_SHIFT)  /* 0110: On-chip Event 1 acting as a burst mode counter clock  */
#  define HRTIM_BMCR_BMCLK_7            (7 << HRTIM_BMCR_BMCLK_SHIFT)  /* 0111: On-chip Event 2 acting as a burst mode counter clock  */
#  define HRTIM_BMCR_BMCLK_8            (8 << HRTIM_BMCR_BMCLK_SHIFT)  /* 1000: On-chip Event 3 acting as a burst mode counter clock  */
#  define HRTIM_BMCR_BMCLK_9            (9 << HRTIM_BMCR_BMCLK_SHIFT)  /* 1001: On-chip Event 4 acting as a burst mode counter clock  */
#  define HRTIM_BMCR_BMCLK_10           (10 << HRTIM_BMCR_BMCLK_SHIFT) /* 1010: Prescaled fHRTIM clock */
#define HRTIM_BMCR_BMPRSC_SHIFT         6                              /* Bits 6-9: Burst Mode Prescaler */
#define HRTIM_BMCR_BMPRSC_MASK          (15 << HRTIM_BMCR_BMPRSC_SHIFT)
#  define HRTIM_BMCR_BMPRSC_PSCOFF      (0 << HRTIM_BMCR_BMPRSC_SHIFT)  /* 0000: Clock not divided */
#  define HRTIM_BMCR_BMPRSC_d2          (1 << HRTIM_BMCR_BMPRSC_SHIFT)  /* 0001: Division by 2 */
#  define HRTIM_BMCR_BMPRSC_d4          (2 << HRTIM_BMCR_BMPRSC_SHIFT)  /* 0010: Division by 4 */
#  define HRTIM_BMCR_BMPRSC_d8          (3 << HRTIM_BMCR_BMPRSC_SHIFT)  /* 0011: Division by 8 */
#  define HRTIM_BMCR_BMPRSC_d16         (4 << HRTIM_BMCR_BMPRSC_SHIFT)  /* 0100: Division by 16 */
#  define HRTIM_BMCR_BMPRSC_d32         (5 << HRTIM_BMCR_BMPRSC_SHIFT)  /* 0101: Division by 32 */
#  define HRTIM_BMCR_BMPRSC_d64         (6 << HRTIM_BMCR_BMPRSC_SHIFT)  /* 0110: Division by 64 */
#  define HRTIM_BMCR_BMPRSC_d128        (7 << HRTIM_BMCR_BMPRSC_SHIFT)  /* 0111: Division by 128 */
#  define HRTIM_BMCR_BMPRSC_d256        (8 << HRTIM_BMCR_BMPRSC_SHIFT)  /* 1000: Division by 256 */
#  define HRTIM_BMCR_BMPRSC_d512        (9 << HRTIM_BMCR_BMPRSC_SHIFT)  /* 1001: Division by 512 */
#  define HRTIM_BMCR_BMPRSC_d1024       (10 << HRTIM_BMCR_BMPRSC_SHIFT) /* 1010: Division by 1024 */
#  define HRTIM_BMCR_BMPRSC_d2048       (11 << HRTIM_BMCR_BMPRSC_SHIFT) /* 1011: Division by 2048 */
#  define HRTIM_BMCR_BMPRSC_d4096       (12 << HRTIM_BMCR_BMPRSC_SHIFT) /* 1100: Division by 4096 */
#  define HRTIM_BMCR_BMPRSC_d8192       (13 << HRTIM_BMCR_BMPRSC_SHIFT) /* 1101: Division by 8192 */
#  define HRTIM_BMCR_BMPRSC_d16384      (14 << HRTIM_BMCR_BMPRSC_SHIFT) /* 1110: Division by 16384 */
#  define HRTIM_BMCR_BMPRSC_d32769      (15 << HRTIM_BMCR_BMPRSC_SHIFT) /* 1111: Division by 32768 */
#define HRTIM_BMCR_BMPREN               (1 << 10)                       /* Bit 10: Burst Mode Preload Enable */
#define HRTIM_BMCR_MTBM                 (1 << 16)                       /* Bit 16: Master Timer Burst Mode */
#define HRTIM_BMCR_TABM                 (1 << 17)                       /* Bit 17: Timer A Burst Mode */
#define HRTIM_BMCR_TBBM                 (1 << 18)                       /* Bit 18: Timer B Burst Mode */
#define HRTIM_BMCR_TCBM                 (1 << 19)                       /* Bit 19: Timer C Burst Mode */
#define HRTIM_BMCR_TDBM                 (1 << 20)                       /* Bit 20: Timer D Burst Mode */
#define HRTIM_BMCR_TEBM                 (1 << 21)                       /* Bit 21: Timer E Burst Mode */
#define HRTIM_BMCR_BMSTAT               (1 << 31)                       /* Bit 31: Burst Mode Status */

/* Common Burst Mode Trigger Register */

#define HRTIM_BMTRGR_SW                 (1 << 0)  /* Bit 0: Software start  */
#define HRTIM_BMTRGR_MSTRST             (1 << 1)  /* Bit 1: Master reset or roll-over */
#define HRTIM_BMTRGR_MSTREP             (1 << 2)  /* Bit 2: Master repetition */
#define HRTIM_BMTRGR_MSTCMP1            (1 << 3)  /* Bit 3: Master Compare 1 */
#define HRTIM_BMTRGR_MSTCMP2            (1 << 4)  /* Bit 4: Master Compare 2 */
#define HRTIM_BMTRGR_MSTCMP3            (1 << 5)  /* Bit 5: Master Compare 3 */
#define HRTIM_BMTRGR_MSTCMP4            (1 << 6)  /* Bit 6: Master Compare 4 */
#define HRTIM_BMTRGR_TARST              (1 << 7)  /* Bit 7: Timer A reset or roll-over */
#define HRTIM_BMTRGR_TAREP              (1 << 8)  /* Bit 8: Timer A repetition */
#define HRTIM_BMTRGR_TACMP1             (1 << 9)  /* Bit 9: Timer A Compare 1 */
#define HRTIM_BMTRGR_TACMP2             (1 << 10) /* Bit 10: Timer A Compare 2 */
#define HRTIM_BMTRGR_TBRST              (1 << 11) /* Bit 11: Timer B reset or roll-over */
#define HRTIM_BMTRGR_TBREP              (1 << 12) /* Bit 12: Timer B repetition */
#define HRTIM_BMTRGR_TBCMP1             (1 << 13) /* Bit 13: Timer B Compare 1 */
#define HRTIM_BMTRGR_TBCMP2             (1 << 14) /* Bit 14: Timer B Compare 2 */
#define HRTIM_BMTRGR_TCRST              (1 << 15) /* Bit 15: Timer C reset or roll-over */
#define HRTIM_BMTRGR_TCREP              (1 << 16) /* Bit 16: Timer C repetition */
#define HRTIM_BMTRGR_TCCMP1             (1 << 17) /* Bit 17: Timer C Compare 1 */
#define HRTIM_BMTRGR_TCCMP2             (1 << 18) /* Bit 18: Timer C Compare 2 */
#define HRTIM_BMTRGR_TDRST              (1 << 19) /* Bit 19: Timer D reset or roll-over */
#define HRTIM_BMTRGR_TDREP              (1 << 20) /* Bit 20: Timer D repetition */
#define HRTIM_BMTRGR_TDCMP1             (1 << 21) /* Bit 21: Timer D Compare 1 */
#define HRTIM_BMTRGR_TDCMP2             (1 << 22) /* Bit 22: Timer D Compare 2 */
#define HRTIM_BMTRGR_TERST              (1 << 23) /* Bit 23: Timer E reset or roll-over */
#define HRTIM_BMTRGR_TEREP              (1 << 24) /* Bit 24: Timer E repetition */
#define HRTIM_BMTRGR_TECMP1             (1 << 25) /* Bit 25: Timer E Compare 1 */
#define HRTIM_BMTRGR_TECMP2             (1 << 26) /* Bit 26: Timer E Compare 2 */
#define HRTIM_BMTRGR_TAEEV7             (1 << 27) /* Bit 27: Timer A period following External Event 7 */
#define HRTIM_BMTRGR_TDEEV8             (1 << 28) /* Bit 28: Timer D period following External Event 8 */
#define HRTIM_BMTRGR_EEV7               (1 << 29) /* Bit 29: External Event 7 */
#define HRTIM_BMTRGR_EEV8               (1 << 30) /* Bit 30: External Event 8 */
#define HRTIM_BMTRGR_OCHPEV             (1 << 31) /* Bit 31: On-chip Event */

/* Common Burst Mode Compare Register */

#define HRTIM_BMCMPR_SHIFT              0         /* Bits 0-15: Burst mode compare value */
#define HRTIM_BMCMPR_MASK               (0xffff << HRTIM_BMCMPR_SHIFT)

/* Common Burst Mode Period Register */

#define HRTIM_BMPER_SHIFT              0          /* Bits 0-15: Burst mode Period */
#define HRTIM_BMPER_MASK               (0xffff << HRTIM_BMPER_SHIFT)

/* Common External Event Control Register 1 */

#define HRTIM_EECR1_EE1SRC_SHIFT      0                               /* Bits 0-1: External Event 1 Source */
#define HRTIM_EECR1_EE1SRC_MASK       (3 << HRTIM_EECR1_EE1SRC_SHIFT)
#  define HRTIM_EECR1_EE1SRC_SRC1     (0 << HRTIM_EECR1_EE1SRC_SHIFT) /* 00: EE1 Src1 */
#  define HRTIM_EECR1_EE1SRC_SRC2     (1 << HRTIM_EECR1_EE1SRC_SHIFT) /* 00: EE1 Src2 */
#  define HRTIM_EECR1_EE1SRC_SRC3     (2 << HRTIM_EECR1_EE1SRC_SHIFT) /* 00: EE1 Src3 */
#  define HRTIM_EECR1_EE1SRC_SRC4     (3 << HRTIM_EECR1_EE1SRC_SHIFT) /* 00: EE1 Src4 */
#define HRTIM_EECR1_EE1POL            (1 << 2)                        /* Bit 2: External Event 1 Polarity */
#define HRTIM_EECR1_EE1SNS_SHIFT      3                               /* Bits 3-4: External Event 1 Sensitivity */
#define HRTIM_EECR1_EE1SNS_MASK       (3 << HRTIM_EECR1_EE1SNS_SHIFT)
#  define HRTIM_EECR1_EE1SNS_ACTIV    (0 << HRTIM_EECR1_EE1SNS_SHIFT) /* 00: On active level defined by EE1POL bit */
#  define HRTIM_EECR1_EE1SNS_REDGE    (1 << HRTIM_EECR1_EE1SNS_SHIFT) /* 01: Rising edge, whatever EE1POL bit value */
#  define HRTIM_EECR1_EE1SNS_FEDGE    (2 << HRTIM_EECR1_EE1SNS_SHIFT) /* 10: Falling edge, whatever EE1POL bit value */
#  define HRTIM_EECR1_EE1SNS_BEDGE    (3 << HRTIM_EECR1_EE1SNS_SHIFT) /* 11: Both edges, whatever EE1POL bit value */
#define HRTIM_EECR1_EE1FAST           (1 << 5)                        /* Bit 5: External Event 1 Fast mode */
#define HRTIM_EECR1_EE2SRC_SHIFT      6                               /* Bits 6-7: External Event 2 Source */
#define HRTIM_EECR1_EE2SRC_MASK       (3 << HRTIM_EECR1_EE2SRC_SHIFT)
#  define HRTIM_EECR1_EE2SRC_SRC1     (0 << HRTIM_EECR1_EE2SRC_SHIFT) /* 00: EE2 Src1 */
#  define HRTIM_EECR1_EE2SRC_SRC2     1 << HRTIM_EECR1_EE2SRC_SHIFT)  /* 00: EE2 Src2 */
#  define HRTIM_EECR1_EE2SRC_SRC3     (2 << HRTIM_EECR1_EE2SRC_SHIFT) /* 00: EE2 Src3 */
#  define HRTIM_EECR1_EE2SRC_SRC4     (3 << HRTIM_EECR1_EE2SRC_SHIFT) /* 00: EE2 Src4 */
#define HRTIM_EECR1_EE2POL            (1 << 8)                        /* Bit 8: External Event 2 Polarity */
#define HRTIM_EECR1_EE2SNS_SHIFT      9                               /* Bits 9-10: External Event 2 Sensitivity */
#define HRTIM_EECR1_EE2SNS_MASK       (3 << HRTIM_EECR1_EE2SNS_SHIFT)
#  define HRTIM_EECR1_EE2SNS_ACTIV    (0 << HRTIM_EECR1_EE2SNS_SHIFT) /* 00: On active level defined by EE2POL bit */
#  define HRTIM_EECR1_EE2SNS_REDGE    (1 << HRTIM_EECR1_EE2SNS_SHIFT) /* 01: Rising edge, whatever EE2POL bit value */
#  define HRTIM_EECR1_EE2SNS_FEDGE    (2 << HRTIM_EECR1_EE2SNS_SHIFT) /* 10: Falling edge, whatever EE2POL bit value */
#  define HRTIM_EECR1_EE2SNS_BEDGE    (3 << HRTIM_EECR1_EE2SNS_SHIFT) /* 11: Both edges, whatever EE2POL bit value */
#define HRTIM_EECR1_EE2FAST           (1 << 11)                       /* Bit 11: External Event 2 Fast mode */
#define HRTIM_EECR1_EE3SRC_SHIFT      12                              /* Bits 6-7: External Event 3 Source */
#define HRTIM_EECR1_EE3SRC_MASK       (3 << HRTIM_EECR1_EE3SRC_SHIFT)
#  define HRTIM_EECR1_EE3SRC_SRC1     (0 << HRTIM_EECR1_EE3SRC_SHIFT) /* 00: EE3 Src1 */
#  define HRTIM_EECR1_EE3SRC_SRC2     (1 << HRTIM_EECR1_EE3SRC_SHIFT) /* 00: EE3 Src2 */
#  define HRTIM_EECR1_EE3SRC_SRC3     (2 << HRTIM_EECR1_EE3SRC_SHIFT) /* 00: EE3 Src3 */
#  define HRTIM_EECR1_EE3SRC_SRC4     (3 << HRTIM_EECR1_EE3SRC_SHIFT) /* 00: EE3 Src4 */
#define HRTIM_EECR1_EE3POL            (1 << 14)                       /* Bit 14: External Event 3 Polarity */
#define HRTIM_EECR1_EE3SNS_SHIFT      15                              /* Bits 15-16: External Event 3 Sensitivity */
#define HRTIM_EECR1_EE3SNS_MASK       (3 << HRTIM_EECR1_EE3SNS_SHIFT)
#  define HRTIM_EECR1_EE3SNS_ACTIV    (0 << HRTIM_EECR1_EE3SNS_SHIFT) /* 00: On active level defined by EE3POL bit */
#  define HRTIM_EECR1_EE3SNS_REDGE    (1 << HRTIM_EECR1_EE3SNS_SHIFT) /* 01: Rising edge, whatever EE3POL bit value */
#  define HRTIM_EECR1_EE3SNS_FEDGE    (2 << HRTIM_EECR1_EE3SNS_SHIFT) /* 10: Falling edge, whatever EE3POL bit value */
#  define HRTIM_EECR1_EE3SNS_BEDGE    (3 << HRTIM_EECR1_EE3SNS_SHIFT) /* 11: Both edges, whatever EE3POL bit value */
#define HRTIM_EECR1_EE3FAST           (1 << 17)                       /* Bit 17: External Event 3 Fast mode */
#define HRTIM_EECR1_EE4SRC_SHIFT      18                              /* Bits 18-19: External Event 4 Source */
#define HRTIM_EECR1_EE4SRC_MASK       (3 << HRTIM_EECR1_EE4SRC_SHIFT)
#  define HRTIM_EECR1_EE4SRC_SRC1     (0 << HRTIM_EECR1_EE4SRC_SHIFT) /* 00: EE4 Src1 */
#  define HRTIM_EECR1_EE4SRC_SRC2     (1 << HRTIM_EECR1_EE4SRC_SHIFT) /* 00: EE4 Src2 */
#  define HRTIM_EECR1_EE4SRC_SRC3     (2 << HRTIM_EECR1_EE4SRC_SHIFT) /* 00: EE4 Src3 */
#  define HRTIM_EECR1_EE4SRC_SRC4     (3 << HRTIM_EECR1_EE4SRC_SHIFT) /* 00: EE4 Src4 */
#define HRTIM_EECR1_EE4POL            (1 << 20)                       /* Bit 20: External Event 4 Polarity */
#define HRTIM_EECR1_EE4SNS_SHIFT      21                              /* Bits 21-22: External Event 4 Sensitivity */
#define HRTIM_EECR1_EE4SNS_MASK       (3 << HRTIM_EECR1_EE4SNS_SHIFT)
#  define HRTIM_EECR1_EE4SNS_ACTIV    (0 << HRTIM_EECR1_EE4SNS_SHIFT) /* 00: On active level defined by EE4POL bit */
#  define HRTIM_EECR1_EE4SNS_REDGE    (1 << HRTIM_EECR1_EE4SNS_SHIFT) /* 01: Rising edge, whatever EE4POL bit value */
#  define HRTIM_EECR1_EE4SNS_FEDGE    (2 << HRTIM_EECR1_EE4SNS_SHIFT) /* 10: Falling edge, whatever EE4POL bit value */
#  define HRTIM_EECR1_EE4SNS_BEDGE    (3 << HRTIM_EECR1_EE4SNS_SHIFT) /* 11: Both edges, whatever EE4POL bit value */
#define HRTIM_EECR1_EE4FAST           (1 << 23)                       /* Bit 23: External Event 4 Fast mode */
#define HRTIM_EECR1_EE5SRC_SHIFT      24                              /* Bits 24-25: External Event 5 Source */
#define HRTIM_EECR1_EE5SRC_MASK       (3 << HRTIM_EECR1_EE5SRC_SHIFT)
#  define HRTIM_EECR1_EE5SRC_SRC1     (0 << HRTIM_EECR1_EE5SRC_SHIFT) /* 00: EE5 Src1 */
#  define HRTIM_EECR1_EE5SRC_SRC2     (1 << HRTIM_EECR1_EE5SRC_SHIFT) /* 00: EE5 Src2 */
#  define HRTIM_EECR1_EE5SRC_SRC3     (2 << HRTIM_EECR1_EE5SRC_SHIFT) /* 00: EE5 Src3 */
#  define HRTIM_EECR1_EE5SRC_SRC4     (3 << HRTIM_EECR1_EE5SRC_SHIFT) /* 00: EE5 Src4 */
#define HRTIM_EECR1_EE5POL            (1 << 26)                       /* Bit 26: External Event 5 Polarity */
#define HRTIM_EECR1_EE5SNS_SHIFT      27                              /* Bits 27-28: External Event 5 Sensitivity */
#define HRTIM_EECR1_EE5SNS_MASK       (3 << HRTIM_EECR1_EE5SNS_SHIFT)
#  define HRTIM_EECR1_EE5SNS_ACTIV    (0 << HRTIM_EECR1_EE5SNS_SHIFT) /* 00: On active level defined by EE5POL bit */
#  define HRTIM_EECR1_EE5SNS_REDGE    (1 << HRTIM_EECR1_EE5SNS_SHIFT) /* 01: Rising edge, whatever EE5POL bit value */
#  define HRTIM_EECR1_EE5SNS_FEDGE    (2 << HRTIM_EECR1_EE5SNS_SHIFT) /* 10: Falling edge, whatever EE5POL bit value */
#  define HRTIM_EECR1_EE5SNS_BEDGE    (3 << HRTIM_EECR1_EE5SNS_SHIFT) /* 11: Both edges, whatever EE5POL bit value */
#define HRTIM_EECR1_EE5FAST           (1 << 29)                       /* Bit 29: External Event 5 Fast mode */

/* Common External Event Control Register 2 */

#define HRTIM_EECR2_EE6SRC_SHIFT        0                               /* Bits 0-1: External Event 6 Source */
#define HRTIM_EECR2_EE6SRC_MASK         (3 << HRTIM_EECR2_EE6SRC_SHIFT)
#  define HRTIM_EECR2_EE6SRC_SRC1       (0 << HRTIM_EECR2_EE6SRC_SHIFT) /* 00: EE6 Src1 */
#  define HRTIM_EECR2_EE6SRC_SRC2       (1 << HRTIM_EECR2_EE6SRC_SHIFT) /* 00: EE6 Src2 */
#  define HRTIM_EECR2_EE6SRC_SRC3       (2 << HRTIM_EECR2_EE6SRC_SHIFT) /* 00: EE6 Src3 */
#  define HRTIM_EECR2_EE6SRC_SRC4       (3 << HRTIM_EECR2_EE6SRC_SHIFT) /* 00: EE6 Src4 */
#define HRTIM_EECR2_EE6POL              (1 << 3)                        /* Bit 3: External Event 6 Polarity */
#define HRTIM_EECR2_EE6SNS_SHIFT        3                               /* Bits 3-4: External Event 6 Sensitivity */
#define HRTIM_EECR2_EE6SNS_MASK         (3 << HRTIM_EECR2_EE6SNS_SHIFT)
#  define HRTIM_EECR2_EE6SNS_ACTIV      (0 << HRTIM_EECR2_EE6SNS_SHIFT) /* 00: On active level defined by EE6POL bit */
#  define HRTIM_EECR2_EE6SNS_REDGE      (1 << HRTIM_EECR2_EE6SNS_SHIFT) /* 01: Rising edge, whatever EE6POL bit value */
#  define HRTIM_EECR2_EE6SNS_FEDGE      (2 << HRTIM_EECR2_EE6SNS_SHIFT) /* 10: Falling edge, whatever EE6POL bit value */
#  define HRTIM_EECR2_EE6SNS_BEDGE      (3 << HRTIM_EECR2_EE6SNS_SHIFT) /* 11: Both edges, whatever EE6POL bit value */
#define HRTIM_EECR2_EE7SRC_SHIFT        6                               /* Bits 6-7: External Event 7 Source */
#define HRTIM_EECR2_EE7SRC_MASK         (3 << HRTIM_EECR2_EE7SRC_SHIFT)
#  define HRTIM_EECR2_EE7SRC_SRC1       (0 << HRTIM_EECR2_EE7SRC_SHIFT) /* 00: EE7 Src1 */
#  define HRTIM_EECR2_EE7SRC_SRC2       (1 << HRTIM_EECR2_EE7SRC_SHIFT) /* 00: EE7 Src2 */
#  define HRTIM_EECR2_EE7SRC_SRC3       (2 << HRTIM_EECR2_EE7SRC_SHIFT) /* 00: EE7 Src3 */
#  define HRTIM_EECR2_EE7SRC_SRC4       (3 << HRTIM_EECR2_EE7SRC_SHIFT) /* 00: EE7 Src4 */
#define HRTIM_EECR2_EE7POL              (1 << 8)                        /* Bit 8: External Event 7 Polarity */
#define HRTIM_EECR2_EE7SNS_SHIFT        9                               /* Bits 9-10: External Event 7 Sensitivity */
#define HRTIM_EECR2_EE7SNS_MASK         (3 << HRTIM_EECR2_EE7SNS_SHIFT)
#  define HRTIM_EECR2_EE7SNS_ACTIV      (0 << HRTIM_EECR2_EE7SNS_SHIFT) /* 00: On active level defined by EE7POL bit */
#  define HRTIM_EECR2_EE7SNS_REDGE      (1 << HRTIM_EECR2_EE7SNS_SHIFT) /* 01: Rising edge, whatever EE7POL bit value */
#  define HRTIM_EECR2_EE7SNS_FEDGE      (2 << HRTIM_EECR2_EE7SNS_SHIFT) /* 10: Falling edge, whatever EE7POL bit value */
#  define HRTIM_EECR2_EE7SNS_BEDGE      (3 << HRTIM_EECR2_EE7SNS_SHIFT) /* 11: Both edges, whatever EE7POL bit value */
#define HRTIM_EECR2_EE8SRC_SHIFT        12                              /* Bits 12-13: External Event 8 Source */
#define HRTIM_EECR2_EE8SRC_MASK         (3 << HRTIM_EECR2_EE8SRC_SHIFT)
#  define HRTIM_EECR2_EE8SRC_SRC1       (0 << HRTIM_EECR2_EE8SRC_SHIFT) /* 00: EE8 Src1 */
#  define HRTIM_EECR2_EE8SRC_SRC2       (1 << HRTIM_EECR2_EE8SRC_SHIFT) /* 00: EE8 Src2 */
#  define HRTIM_EECR2_EE8SRC_SRC3       (2 << HRTIM_EECR2_EE8SRC_SHIFT) /* 00: EE8 Src3 */
#  define HRTIM_EECR2_EE8SRC_SRC4       (3 << HRTIM_EECR2_EE8SRC_SHIFT) /* 00: EE8 Src4 */
#define HRTIM_EECR2_EE8POL              (1 << 14)                       /* Bit 14: External Event 8 Polarity */
#define HRTIM_EECR2_EE8SNS_SHIFT        15                              /* Bits 15-16: External Event 8 Sensitivity */
#define HRTIM_EECR2_EE8SNS_MASK         (3 << HRTIM_EECR2_EE8SNS_SHIFT)
#  define HRTIM_EECR2_EE8SNS_ACTIV      (0 << HRTIM_EECR2_EE8SNS_SHIFT) /* 00: On active level defined by EE8POL bit */
#  define HRTIM_EECR2_EE8SNS_REDGE      (1 << HRTIM_EECR2_EE8SNS_SHIFT) /* 01: Rising edge, whatever EE8POL bit value */
#  define HRTIM_EECR2_EE8SNS_FEDGE      (2 << HRTIM_EECR2_EE8SNS_SHIFT) /* 10: Falling edge, whatever EE8POL bit value */
#  define HRTIM_EECR2_EE8SNS_BEDGE      (3 << HRTIM_EECR2_EE8SNS_SHIFT) /* 11: Both edges, whatever EE8POL bit value */
#define HRTIM_EECR2_EE9SRC_SHIFT        18                              /* Bits 18-19: External Event 9 Source */
#define HRTIM_EECR2_EE9SRC_MASK         (3 << HRTIM_EECR2_EE9SRC_SHIFT)
#  define HRTIM_EECR2_EE9SRC_SRC1       (0 << HRTIM_EECR2_EE9SRC_SHIFT) /* 00: EE9 Src1 */
#  define HRTIM_EECR2_EE9SRC_SRC2       (1 << HRTIM_EECR2_EE9SRC_SHIFT) /* 00: EE9 Src2 */
#  define HRTIM_EECR2_EE9SRC_SRC3       (2 << HRTIM_EECR2_EE9SRC_SHIFT) /* 00: EE9 Src3 */
#  define HRTIM_EECR2_EE9SRC_SRC4       (3 << HRTIM_EECR2_EE9SRC_SHIFT) /* 00: EE9 Src4 */
#define HRTIM_EECR2_EE9POL              (1 << 20)                       /* Bit 20: External Event 9 Polarity */
#define HRTIM_EECR2_EE9SNS_SHIFT        21                              /* Bits 21-22: External Event 9 Sensitivity */
#define HRTIM_EECR2_EE9SNS_MASK         (3 << HRTIM_EECR2_EE9SNS_SHIFT)
#  define HRTIM_EECR2_EE9SNS_ACTIV      (0 << HRTIM_EECR2_EE9SNS_SHIFT) /* 00: On active level defined by EE9POL bit */
#  define HRTIM_EECR2_EE9SNS_REDGE      (1 << HRTIM_EECR2_EE9SNS_SHIFT) /* 01: Rising edge, whatever EE9POL bit value */
#  define HRTIM_EECR2_EE9SNS_FEDGE      (2 << HRTIM_EECR2_EE9SNS_SHIFT) /* 10: Falling edge, whatever EE9POL bit value */
#  define HRTIM_EECR2_EE9SNS_BEDGE      (3 << HRTIM_EECR2_EE9SNS_SHIFT) /* 11: Both edges, whatever EE9POL bit value */
#define HRTIM_EECR2_EE10SRC_SHIFT       24                              /* Bits 24-25: External Event 10 Source */
#define HRTIM_EECR2_EE10SRC_MASK        (3 << HRTIM_EECR2_EE10SRC_SHIFT)
#  define HRTIM_EECR2_EE10SRC_SRC1      (0 << HRTIM_EECR2_EE10SRC_SHIFT) /* 00: EE10 Src1 */
#  define HRTIM_EECR2_EE10SRC_SRC2      (1 << HRTIM_EECR2_EE10SRC_SHIFT) /* 00: EE10 Src2 */
#  define HRTIM_EECR2_EE10SRC_SRC3      (2 << HRTIM_EECR2_EE10SRC_SHIFT) /* 00: EE10 Src3 */
#  define HRTIM_EECR2_EE10SRC_SRC4      (3 << HRTIM_EECR2_EE10SRC_SHIFT) /* 00: EE10 Src4 */
#define HRTIM_EECR2_EE10POL             (1 << 26)                        /* Bit 26: External Event 10 Polarity */
#define HRTIM_EECR2_EE10SNS_SHIFT       28                               /* Bits 27-28: External Event 10 Sensitivity */
#define HRTIM_EECR2_EE10SNS_MASK        (3 << HRTIM_EECR2_EE10SNS_SHIFT)
#  define HRTIM_EECR2_EE10SNS_ACTIV     (0 << HRTIM_EECR2_EE10SNS_SHIFT) /* 00: On active level defined by EE10POL bit */
#  define HRTIM_EECR2_EE10SNS_REDGE     (1 << HRTIM_EECR2_EE10SNS_SHIFT) /* 01: Rising edge, whatever EE10POL bit value */
#  define HRTIM_EECR2_EE10SNS_FEDGE     (2 << HRTIM_EECR2_EE10SNS_SHIFT) /* 10: Falling edge, whatever EE10POL bit value */
#  define HRTIM_EECR2_EE10SNS_BEDGE     (3 << HRTIM_EECR2_EE10SNS_SHIFT) /* 11: Both edges, whatever EE10POL bit value */

/* Common External Event Control Register 3 */

#define HRTIM_EECR3_EE6F_SHIFT          0        /* Bits 0-3: External Event 6 Filter */
#define HRTIM_EECR3_EE6F_MASK           (15 << HRTIM_EECR3_EE6F_SHIFT)
#  define HRTIM_EECR3_EE6F_NOFLT        (0 << HRTIM_EECR3_EE6F_SHIFT)  /* 0000: No filter, FLT5 acts asynchronously */
#  define HRTIM_EECR3_EE6F_HRTN2        (1 << HRTIM_EECR3_EE6F_SHIFT)  /* 0001: fSAMPLING = fHRTIM, N=2 */
#  define HRTIM_EECR3_EE6F_HRTN4        (2 << HRTIM_EECR3_EE6F_SHIFT)  /* 0010: fSAMPLING = fHRTIM, N=4 */
#  define HRTIM_EECR3_EE6F_HRTN8        (3 << HRTIM_EECR3_EE6F_SHIFT)  /* 0011: fSAMPLING = fHRTIM, N=8 */
#  define HRTIM_EECR3_EE6F_EEVS2N6      (4 << HRTIM_EECR3_EE6F_SHIFT)  /* 0100: fSAMPLING = fEEVS/2, N=6 */
#  define HRTIM_EECR3_EE6F_EEVS2N8      (5 << HRTIM_EECR3_EE6F_SHIFT)  /* 0101: fSAMPLING = fEEVS/2, N=8 */
#  define HRTIM_EECR3_EE6F_EEVS4N6      (6 << HRTIM_EECR3_EE6F_SHIFT)  /* 0110: fSAMPLING = fEEVS/4, N=6 */
#  define HRTIM_EECR3_EE6F_EEVS4N8      (7 << HRTIM_EECR3_EE6F_SHIFT)  /* 0111: fSAMPLING = fEEVS/4, N=8 */
#  define HRTIM_EECR3_EE6F_EEVS8N6      (8 << HRTIM_EECR3_EE6F_SHIFT)  /* 1000: fSAMPLING = fEEVS/8, N=6 */
#  define HRTIM_EECR3_EE6F_EEVS8N8      (9 << HRTIM_EECR3_EE6F_SHIFT)  /* 1001: fSAMPLING = fEEVS/8, N=8 */
#  define HRTIM_EECR3_EE6F_EEVS16N5     (10 << HRTIM_EECR3_EE6F_SHIFT) /* 1010: fSAMPLING = fEEVS/16, N=5 */
#  define HRTIM_EECR3_EE6F_EEVS16N6     (11 << HRTIM_EECR3_EE6F_SHIFT) /* 1011: fSAMPLING = fEEVS/16, N=6 */
#  define HRTIM_EECR3_EE6F_EEVS16N8     (12 << HRTIM_EECR3_EE6F_SHIFT) /* 1100: fSAMPLING = fEEVS/16, N=8 */
#  define HRTIM_EECR3_EE6F_EEVS32N5     (13 << HRTIM_EECR3_EE6F_SHIFT) /* 1101: fSAMPLING = fEEVS/32, N=5 */
#  define HRTIM_EECR3_EE6F_EEVS32N6     (14 << HRTIM_EECR3_EE6F_SHIFT) /* 1110: fSAMPLING = fEEVS/32, N=6 */
#  define HRTIM_EECR3_EE6F_EEVS32N8     (15 << HRTIM_EECR3_EE6F_SHIFT) /* 1111: fSAMPLING = fEEVS/32, N=8 */
#define HRTIM_EECR3_EE7F_SHIFT          6                              /* Bits 6-9: External Event 7 Filter */
#define HRTIM_EECR3_EE7F_MASK           (15 << HRTIM_EECR3_EE7F_SHIFT)
#  define HRTIM_EECR3_EE7F_NOFLT        (0 << HRTIM_EECR3_EE7F_SHIFT)  /* 0000: No filter, FLT5 acts asynchronously */
#  define HRTIM_EECR3_EE7F_HRTN2        (1 << HRTIM_EECR3_EE7F_SHIFT)  /* 0001: fSAMPLING = fHRTIM, N=2 */
#  define HRTIM_EECR3_EE7F_HRTN4        (2 << HRTIM_EECR3_EE7F_SHIFT)  /* 0010: fSAMPLING = fHRTIM, N=4 */
#  define HRTIM_EECR3_EE7F_HRTN8        (3 << HRTIM_EECR3_EE7F_SHIFT)  /* 0011: fSAMPLING = fHRTIM, N=8 */
#  define HRTIM_EECR3_EE7F_EEVS2N6      (4 << HRTIM_EECR3_EE7F_SHIFT)  /* 0100: fSAMPLING = fEEVS/2, N=6 */
#  define HRTIM_EECR3_EE7F_EEVS2N8      (5 << HRTIM_EECR3_EE7F_SHIFT)  /* 0101: fSAMPLING = fEEVS/2, N=8 */
#  define HRTIM_EECR3_EE7F_EEVS4N6      (6 << HRTIM_EECR3_EE7F_SHIFT)  /* 0110: fSAMPLING = fEEVS/4, N=6 */
#  define HRTIM_EECR3_EE7F_EEVS4N8      (7 << HRTIM_EECR3_EE7F_SHIFT)  /* 0111: fSAMPLING = fEEVS/4, N=8 */
#  define HRTIM_EECR3_EE7F_EEVS8N6      (8 << HRTIM_EECR3_EE7F_SHIFT)  /* 1000: fSAMPLING = fEEVS/8, N=6 */
#  define HRTIM_EECR3_EE7F_EEVS8N8      (9 << HRTIM_EECR3_EE7F_SHIFT)  /* 1001: fSAMPLING = fEEVS/8, N=8 */
#  define HRTIM_EECR3_EE7F_EEVS16N5     (10 << HRTIM_EECR3_EE7F_SHIFT) /* 1010: fSAMPLING = fEEVS/16, N=5 */
#  define HRTIM_EECR3_EE7F_EEVS16N6     (11 << HRTIM_EECR3_EE7F_SHIFT) /* 1011: fSAMPLING = fEEVS/16, N=6 */
#  define HRTIM_EECR3_EE7F_EEVS16N8     (12 << HRTIM_EECR3_EE7F_SHIFT) /* 1100: fSAMPLING = fEEVS/16, N=8 */
#  define HRTIM_EECR3_EE7F_EEVS32N5     (13 << HRTIM_EECR3_EE7F_SHIFT) /* 1101: fSAMPLING = fEEVS/32, N=5 */
#  define HRTIM_EECR3_EE7F_EEVS32N6     (14 << HRTIM_EECR3_EE7F_SHIFT) /* 1110: fSAMPLING = fEEVS/32, N=6 */
#  define HRTIM_EECR3_EE7F_EEVS32N8     (15 << HRTIM_EECR3_EE7F_SHIFT) /* 1111: fSAMPLING = fEEVS/32, N=8 */
#define HRTIM_EECR3_EE8F_SHIFT          12                             /* Bits 12-15: External Event 8 Filter */
#define HRTIM_EECR3_EE8F_MASK           (15 << HRTIM_EECR3_EE8F_SHIFT)
#  define HRTIM_EECR3_EE8F_NOFLT        (0 << HRTIM_EECR3_EE8F_SHIFT)  /* 0000: No filter, FLT5 acts asynchronously */
#  define HRTIM_EECR3_EE8F_HRTN2        (1 << HRTIM_EECR3_EE8F_SHIFT)  /* 0001: fSAMPLING = fHRTIM, N=2 */
#  define HRTIM_EECR3_EE8F_HRTN4        (2 << HRTIM_EECR3_EE8F_SHIFT)  /* 0010: fSAMPLING = fHRTIM, N=4 */
#  define HRTIM_EECR3_EE8F_HRTN8        (3 << HRTIM_EECR3_EE8F_SHIFT)  /* 0011: fSAMPLING = fHRTIM, N=8 */
#  define HRTIM_EECR3_EE8F_EEVS2N6      (4 << HRTIM_EECR3_EE8F_SHIFT)  /* 0100: fSAMPLING = fEEVS/2, N=6 */
#  define HRTIM_EECR3_EE8F_EEVS2N8      (5 << HRTIM_EECR3_EE8F_SHIFT)  /* 0101: fSAMPLING = fEEVS/2, N=8 */
#  define HRTIM_EECR3_EE8F_EEVS4N6      (6 << HRTIM_EECR3_EE8F_SHIFT)  /* 0110: fSAMPLING = fEEVS/4, N=6 */
#  define HRTIM_EECR3_EE8F_EEVS4N8      (7 << HRTIM_EECR3_EE8F_SHIFT)  /* 0111: fSAMPLING = fEEVS/4, N=8 */
#  define HRTIM_EECR3_EE8F_EEVS8N6      (8 << HRTIM_EECR3_EE8F_SHIFT)  /* 1000: fSAMPLING = fEEVS/8, N=6 */
#  define HRTIM_EECR3_EE8F_EEVS8N8      (9 << HRTIM_EECR3_EE8F_SHIFT)  /* 1001: fSAMPLING = fEEVS/8, N=8 */
#  define HRTIM_EECR3_EE8F_EEVS16N5     (10 << HRTIM_EECR3_EE8F_SHIFT) /* 1010: fSAMPLING = fEEVS/16, N=5 */
#  define HRTIM_EECR3_EE8F_EEVS16N6     (11 << HRTIM_EECR3_EE8F_SHIFT) /* 1011: fSAMPLING = fEEVS/16, N=6 */
#  define HRTIM_EECR3_EE8F_EEVS16N8     (12 << HRTIM_EECR3_EE8F_SHIFT) /* 1100: fSAMPLING = fEEVS/16, N=8 */
#  define HRTIM_EECR3_EE8F_EEVS32N5     (13 << HRTIM_EECR3_EE8F_SHIFT) /* 1101: fSAMPLING = fEEVS/32, N=5 */
#  define HRTIM_EECR3_EE8F_EEVS32N6     (14 << HRTIM_EECR3_EE8F_SHIFT) /* 1110: fSAMPLING = fEEVS/32, N=6 */
#  define HRTIM_EECR3_EE8F_EEVS32N8     (15 << HRTIM_EECR3_EE8F_SHIFT) /* 1111: fSAMPLING = fEEVS/32, N=8 */
#define HRTIM_EECR3_EE9F_SHIFT          18                             /* Bits 18-21: External Event 9 Filter */
#define HRTIM_EECR3_EE9F_MASK           (15 << HRTIM_EECR3_EE9F_SHIFT)
#  define HRTIM_EECR3_EE9F_NOFLT        (0 << HRTIM_EECR3_EE9F_SHIFT)  /* 0000: No filter, FLT5 acts asynchronously */
#  define HRTIM_EECR3_EE9F_HRTN2        (1 << HRTIM_EECR3_EE9F_SHIFT)  /* 0001: fSAMPLING = fHRTIM, N=2 */
#  define HRTIM_EECR3_EE9F_HRTN4        (2 << HRTIM_EECR3_EE9F_SHIFT)  /* 0010: fSAMPLING = fHRTIM, N=4 */
#  define HRTIM_EECR3_EE9F_HRTN8        (3 << HRTIM_EECR3_EE9F_SHIFT)  /* 0011: fSAMPLING = fHRTIM, N=8 */
#  define HRTIM_EECR3_EE9F_EEVS2N6      (4 << HRTIM_EECR3_EE9F_SHIFT)  /* 0100: fSAMPLING = fEEVS/2, N=6 */
#  define HRTIM_EECR3_EE9F_EEVS2N8      (5 << HRTIM_EECR3_EE9F_SHIFT)  /* 0101: fSAMPLING = fEEVS/2, N=8 */
#  define HRTIM_EECR3_EE9F_EEVS4N6      (6 << HRTIM_EECR3_EE9F_SHIFT)  /* 0110: fSAMPLING = fEEVS/4, N=6 */
#  define HRTIM_EECR3_EE9F_EEVS4N8      (7 << HRTIM_EECR3_EE9F_SHIFT)  /* 0111: fSAMPLING = fEEVS/4, N=8 */
#  define HRTIM_EECR3_EE9F_EEVS8N6      (8 << HRTIM_EECR3_EE9F_SHIFT)  /* 1000: fSAMPLING = fEEVS/8, N=6 */
#  define HRTIM_EECR3_EE9F_EEVS8N8      (9 << HRTIM_EECR3_EE9F_SHIFT)  /* 1001: fSAMPLING = fEEVS/8, N=8 */
#  define HRTIM_EECR3_EE9F_EEVS16N5     (10 << HRTIM_EECR3_EE9F_SHIFT) /* 1010: fSAMPLING = fEEVS/16, N=5 */
#  define HRTIM_EECR3_EE9F_EEVS16N6     (11 << HRTIM_EECR3_EE9F_SHIFT) /* 1011: fSAMPLING = fEEVS/16, N=6 */
#  define HRTIM_EECR3_EE9F_EEVS16N8     (12 << HRTIM_EECR3_EE9F_SHIFT) /* 1100: fSAMPLING = fEEVS/16, N=8 */
#  define HRTIM_EECR3_EE9F_EEVS32N5     (13 << HRTIM_EECR3_EE9F_SHIFT) /* 1101: fSAMPLING = fEEVS/32, N=5 */
#  define HRTIM_EECR3_EE9F_EEVS32N6     (14 << HRTIM_EECR3_EE9F_SHIFT) /* 1110: fSAMPLING = fEEVS/32, N=6 */
#  define HRTIM_EECR3_EE9F_EEVS32N8     (15 << HRTIM_EECR3_EE9F_SHIFT) /* 1111: fSAMPLING = fEEVS/32, N=8 */
#define HRTIM_EECR3_EE10F_SHIFT         24                             /* Bits 24-27: External Event 10 Filter */
#define HRTIM_EECR3_EE10F_MASK          (15 << HRTIM_EECR3_EE10F_SHIFT)
#  define HRTIM_EECR3_EE10F_NOFLT       (0 << HRTIM_EECR3_EE10F_SHIFT)  /* 0000: No filter, FLT5 acts asynchronously */
#  define HRTIM_EECR3_EE10F_HRTN2       (1 << HRTIM_EECR3_EE10F_SHIFT)  /* 0001: fSAMPLING = fHRTIM, N=2 */
#  define HRTIM_EECR3_EE10F_HRTN4       (2 << HRTIM_EECR3_EE10F_SHIFT)  /* 0010: fSAMPLING = fHRTIM, N=4 */
#  define HRTIM_EECR3_EE10F_HRTN8       (3 << HRTIM_EECR3_EE10F_SHIFT)  /* 0011: fSAMPLING = fHRTIM, N=8 */
#  define HRTIM_EECR3_EE10F_EEVS2N6     (4 << HRTIM_EECR3_EE10F_SHIFT)  /* 0100: fSAMPLING = fEEVS/2, N=6 */
#  define HRTIM_EECR3_EE10F_EEVS2N8     (5 << HRTIM_EECR3_EE10F_SHIFT)  /* 0101: fSAMPLING = fEEVS/2, N=8 */
#  define HRTIM_EECR3_EE10F_EEVS4N6     (6 << HRTIM_EECR3_EE10F_SHIFT)  /* 0110: fSAMPLING = fEEVS/4, N=6 */
#  define HRTIM_EECR3_EE10F_EEVS4N8     (7 << HRTIM_EECR3_EE10F_SHIFT)  /* 0111: fSAMPLING = fEEVS/4, N=8 */
#  define HRTIM_EECR3_EE10F_EEVS8N6     (8 << HRTIM_EECR3_EE10F_SHIFT)  /* 1000: fSAMPLING = fEEVS/8, N=6 */
#  define HRTIM_EECR3_EE10F_EEVS8N8     (9 << HRTIM_EECR3_EE10F_SHIFT)  /* 1001: fSAMPLING = fEEVS/8, N=8 */
#  define HRTIM_EECR3_EE10F_EEVS16N5    (10 << HRTIM_EECR3_EE10F_SHIFT) /* 1010: fSAMPLING = fEEVS/16, N=5 */
#  define HRTIM_EECR3_EE10F_EEVS16N6    (11 << HRTIM_EECR3_EE10F_SHIFT) /* 1011: fSAMPLING = fEEVS/16, N=6 */
#  define HRTIM_EECR3_EE10F_EEVS16N8    (12 << HRTIM_EECR3_EE10F_SHIFT) /* 1100: fSAMPLING = fEEVS/16, N=8 */
#  define HRTIM_EECR3_EE10F_EEVS32N5    (13 << HRTIM_EECR3_EE10F_SHIFT) /* 1101: fSAMPLING = fEEVS/32, N=5 */
#  define HRTIM_EECR3_EE10F_EEVS32N6    (14 << HRTIM_EECR3_EE10F_SHIFT) /* 1110: fSAMPLING = fEEVS/32, N=6 */
#  define HRTIM_EECR3_EE10F_EEVS32N8    (15 << HRTIM_EECR3_EE10F_SHIFT) /* 1111: fSAMPLING = fEEVS/32, N=8 */
#define HRTIM_EECR3_EEVSD_SHIFT         30                              /* Bits 30-31: External Event Sampling clock division */
#define HRTIM_EECR3_EEVSD_MASK          (3 << HRTIM_EECR3_EEVSD_SHIFT)
#define HRTIM_EECR3_EEVSD_NODIV         (0 << HRTIM_EECR3_EEVSD_SHIFT) /* 00: fEEVS=fHRTIM */
#define HRTIM_EECR3_EEVSD_d2            (1 << HRTIM_EECR3_EEVSD_SHIFT) /* 01: fEEVS=fHRTIM/2 */
#define HRTIM_EECR3_EEVSD_d4            (2 << HRTIM_EECR3_EEVSD_SHIFT) /* 10: fEEVS=fHRTIM/4 */
#define HRTIM_EECR3_EEVSD_d8            (3 << HRTIM_EECR3_EEVSD_SHIFT) /* 11: fEEVS=fHRTIM/8 */

/* Common ADC Trigger 1 Register */

#define HRTIM_ADC1R_AD1MC1             (1 << 0)  /* Bit 0: ADC trigger 1 on Master Compare 1 */
#define HRTIM_ADC1R_AD1MC2             (1 << 1)  /* Bit 1: ADC trigger 1 on Master Compare 2 */
#define HRTIM_ADC1R_AD1MC3             (1 << 2)  /* Bit 2: ADC trigger 1 on Master Compare 3 */
#define HRTIM_ADC1R_AD1MC4             (1 << 3)  /* Bit 3: ADC trigger 1 on Master Compare 4 */
#define HRTIM_ADC1R_AD1MPER            (1 << 4)  /* Bit 4: ADC trigger 1 on Master Period*/
#define HRTIM_ADC1R_AD1EEV1            (1 << 5)  /* Bit 5: ADC trigger 1 on External Event 1 */
#define HRTIM_ADC1R_AD1EEV2            (1 << 6)  /* Bit 6: ADC trigger 1 on External Event 2 */
#define HRTIM_ADC1R_AD1EEV3            (1 << 7)  /* Bit 7: ADC trigger 1 on External Event 3 */
#define HRTIM_ADC1R_AD1EEV4            (1 << 8)  /* Bit 8: ADC trigger 1 on External Event 4 */
#define HRTIM_ADC1R_AD1EEV5            (1 << 9)  /* Bit 9: ADC trigger 1 on External Event 5 */
#define HRTIM_ADC1R_AD1TAC2            (1 << 10) /* Bit 10: ADC trigger 1 on Timer A Compare 2 */
#define HRTIM_ADC1R_AD1TAC3            (1 << 11) /* Bit 11: ADC trigger 1 on Timer A Compare 3 */
#define HRTIM_ADC1R_AD1TAC4            (1 << 12) /* Bit 12: ADC trigger 1 on Timer A Compare 4 */
#define HRTIM_ADC1R_AD1TAPER           (1 << 13) /* Bit 13: ADC trigger 1 on Timer A Period */
#define HRTIM_ADC1R_AD1TARST           (1 << 14) /* Bit 14: ADC trigger 1 on Timer A Reset and counter roll-over*/
#define HRTIM_ADC1R_AD1TBC2            (1 << 15) /* Bit 15: ADC trigger 1 on Timer B Compare 2 */
#define HRTIM_ADC1R_AD1TBC3            (1 << 16) /* Bit 16: ADC trigger 1 on Timer B Compare 3 */
#define HRTIM_ADC1R_AD1TBC4            (1 << 17) /* Bit 17: ADC trigger 1 on Timer B Compare 4 */
#define HRTIM_ADC1R_AD1TBPER           (1 << 18) /* Bit 18: ADC trigger 1 on Timer B Period */
#define HRTIM_ADC1R_AD1TBRST           (1 << 19) /* Bit 19: ADC trigger 1 on Timer B Reset and counter roll-over */
#define HRTIM_ADC1R_AD1TCC2            (1 << 20) /* Bit 20: ADC trigger 1 on Timer C Compare 2 */
#define HRTIM_ADC1R_AD1TCC3            (1 << 21) /* Bit 21: ADC trigger 1 on Timer C Compare 3 */
#define HRTIM_ADC1R_AD1TCC4            (1 << 22) /* Bit 22: ADC trigger 1 on Timer C Compare 4 */
#define HRTIM_ADC1R_AD1TCPER           (1 << 23) /* Bit 23: ADC trigger 1 on Timer C Period*/
#define HRTIM_ADC1R_AD1TDC2            (1 << 24) /* Bit 24: ADC trigger 1 on Timer D Compare 2 */
#define HRTIM_ADC1R_AD1TDC3            (1 << 25) /* Bit 25: ADC trigger 1 on Timer D Compare 3 */
#define HRTIM_ADC1R_AD1TDC4            (1 << 26) /* Bit 26: ADC trigger 1 on Timer D Compare 4 */
#define HRTIM_ADC1R_AD1TDPER           (1 << 27) /* Bit 27: ADC trigger 1 on Timer D Period*/
#define HRTIM_ADC1R_AD1TEC2            (1 << 28) /* Bit 28: ADC trigger 1 on Timer E Compare 2 */
#define HRTIM_ADC1R_AD1TEC3            (1 << 29) /* Bit 29: ADC trigger 1 on Timer E Compare 3 */
#define HRTIM_ADC1R_AD1TEC4            (1 << 30) /* Bit 30: ADC trigger 1 on Timer E Compare 4 */
#define HRTIM_ADC1R_AD1TEPER           (1 << 31) /* Bit 31: ADC trigger 1 on Timer E Period */

/* Common ADC Trigger 2 Register */

#define HRTIM_ADC2R_AD2MC1             (1 << 0)  /* Bit 0: ADC trigger 2 on Master Compare 1 */
#define HRTIM_ADC2R_AD2MC2             (1 << 1)  /* Bit 1: ADC trigger 2 on Master Compare 2 */
#define HRTIM_ADC2R_AD2MC3             (1 << 2)  /* Bit 2: ADC trigger 2 on Master Compare 3 */
#define HRTIM_ADC2R_AD2MC4             (1 << 3)  /* Bit 3: ADC trigger 2 on Master Compare 4 */
#define HRTIM_ADC2R_AD2MPER            (1 << 4)  /* Bit 4: ADC trigger 2 on Master Period*/
#define HRTIM_ADC2R_AD2EEV6            (1 << 5)  /* Bit 5: ADC trigger 2 on External Event 6 */
#define HRTIM_ADC2R_AD2EEV7            (1 << 6)  /* Bit 6: ADC trigger 2 on External Event 7 */
#define HRTIM_ADC2R_AD2EEV8            (1 << 7)  /* Bit 7: ADC trigger 2 on External Event 8 */
#define HRTIM_ADC2R_AD2EEV9            (1 << 8)  /* Bit 8: ADC trigger 2 on External Event 9 */
#define HRTIM_ADC2R_AD2EEV10           (1 << 9)  /* Bit 9: ADC trigger 2 on External Event 10 */
#define HRTIM_ADC2R_AD2TAC2            (1 << 10) /* Bit 10: ADC trigger 2 on Timer A Compare 2 */
#define HRTIM_ADC2R_AD2TAC3            (1 << 11) /* Bit 11: ADC trigger 2 on Timer A Compare 3 */
#define HRTIM_ADC2R_AD2TAC4            (1 << 12) /* Bit 12: ADC trigger 2 on Timer A Compare 4 */
#define HRTIM_ADC2R_AD2TAPER           (1 << 13) /* Bit 13: ADC trigger 2 on Timer A Period */
#define HRTIM_ADC2R_AD2TBC2            (1 << 14) /* Bit 14: ADC trigger 2 on Timer B Compare 2 */
#define HRTIM_ADC2R_AD2TBC3            (1 << 15) /* Bit 15: ADC trigger 2 on Timer B Compare 3 */
#define HRTIM_ADC2R_AD2TBC4            (1 << 16) /* Bit 16: ADC trigger 2 on Timer B Compare 4 */
#define HRTIM_ADC2R_AD2TBPER           (1 << 17) /* Bit 18: ADC trigger 2 on Timer B Period */
#define HRTIM_ADC2R_AD2TCC2            (1 << 18) /* Bit 19: ADC trigger 2 on Timer C Compare 2 */
#define HRTIM_ADC2R_AD2TCC3            (1 << 19) /* Bit 20: ADC trigger 2 on Timer C Compare 3 */
#define HRTIM_ADC2R_AD2TCC4            (1 << 20) /* Bit 21: ADC trigger 2 on Timer C Compare 4 */
#define HRTIM_ADC2R_AD2TCPER           (1 << 21) /* Bit 22: ADC trigger 2 on Timer C Period*/
#define HRTIM_ADC2R_AD2TCRST           (1 << 22) /* Bit 22: ADC trigger 2 on Timer C Reset and counter roll-over*/
#define HRTIM_ADC2R_AD2TDC2            (1 << 23) /* Bit 23: ADC trigger 2 on Timer D Compare 2 */
#define HRTIM_ADC2R_AD2TDC3            (1 << 24) /* Bit 24: ADC trigger 2 on Timer D Compare 3 */
#define HRTIM_ADC2R_AD2TDC4            (1 << 25) /* Bit 25: ADC trigger 2 on Timer D Compare 4 */
#define HRTIM_ADC2R_AD2TDPER           (1 << 26) /* Bit 26: ADC trigger 2 on Timer D Period*/
#define HRTIM_ADC2R_AD2TDRST           (1 << 27) /* Bit 27: ADC trigger 2 on Timer D Reset and counter roll-over*/
#define HRTIM_ADC2R_AD2TEC2            (1 << 28) /* Bit 28: ADC trigger 2 on Timer E Compare 2 */
#define HRTIM_ADC2R_AD2TEC3            (1 << 29) /* Bit 29: ADC trigger 2 on Timer E Compare 3 */
#define HRTIM_ADC2R_AD2TEC4            (1 << 30) /* Bit 30: ADC trigger 2 on Timer E Compare 4 */
#define HRTIM_ADC2R_AD2TERST           (1 << 31) /* Bit 31: ADC trigger 2 on Timer E Reset and counter roll-over */

/* Common ADC Trigger 3 Register */

#define HRTIM_ADC3R_AD3MC1             (1 << 0)  /* Bit 0: ADC trigger 3 on Master Compare 1 */
#define HRTIM_ADC3R_AD3MC2             (1 << 1)  /* Bit 1: ADC trigger 3 on Master Compare 2 */
#define HRTIM_ADC3R_AD3MC3             (1 << 2)  /* Bit 2: ADC trigger 3 on Master Compare 3 */
#define HRTIM_ADC3R_AD3MC4             (1 << 3)  /* Bit 3: ADC trigger 3 on Master Compare 4 */
#define HRTIM_ADC3R_AD3MPER            (1 << 4)  /* Bit 4: ADC trigger 3 on Master Period*/
#define HRTIM_ADC3R_AD3EEV1            (1 << 5)  /* Bit 5: ADC trigger 3 on External Event 1 */
#define HRTIM_ADC3R_AD3EEV2            (1 << 6)  /* Bit 6: ADC trigger 3 on External Event 2 */
#define HRTIM_ADC3R_AD3EEV3            (1 << 7)  /* Bit 7: ADC trigger 3 on External Event 3 */
#define HRTIM_ADC3R_AD3EEV4            (1 << 8)  /* Bit 8: ADC trigger 3 on External Event 4 */
#define HRTIM_ADC3R_AD3EEV5            (1 << 9)  /* Bit 9: ADC trigger 3 on External Event 5 */
#define HRTIM_ADC3R_AD3TAC2            (1 << 10) /* Bit 10: ADC trigger 3 on Timer A Compare 2 */
#define HRTIM_ADC3R_AD3TAC3            (1 << 11) /* Bit 11: ADC trigger 3 on Timer A Compare 3 */
#define HRTIM_ADC3R_AD3TAC4            (1 << 12) /* Bit 12: ADC trigger 3 on Timer A Compare 4 */
#define HRTIM_ADC3R_AD3TAPER           (1 << 13) /* Bit 13: ADC trigger 3 on Timer A Period */
#define HRTIM_ADC3R_AD3TARST           (1 << 14) /* Bit 14: ADC trigger 3 on Timer A Reset and counter roll-over*/
#define HRTIM_ADC3R_AD3TBC2            (1 << 15) /* Bit 15: ADC trigger 3 on Timer B Compare 2 */
#define HRTIM_ADC3R_AD3TBC3            (1 << 16) /* Bit 16: ADC trigger 3 on Timer B Compare 3 */
#define HRTIM_ADC3R_AD3TBC4            (1 << 17) /* Bit 17: ADC trigger 3 on Timer B Compare 4 */
#define HRTIM_ADC3R_AD3TBPER           (1 << 18) /* Bit 18: ADC trigger 3 on Timer B Period */
#define HRTIM_ADC3R_AD3TBRST           (1 << 19) /* Bit 19: ADC trigger 3 on Timer B Reset and counter roll-over */
#define HRTIM_ADC3R_AD3TCC2            (1 << 20) /* Bit 20: ADC trigger 3 on Timer C Compare 2 */
#define HRTIM_ADC3R_AD3TCC3            (1 << 21) /* Bit 21: ADC trigger 3 on Timer C Compare 3 */
#define HRTIM_ADC3R_AD3TCC4            (1 << 22) /* Bit 22: ADC trigger 3 on Timer C Compare 4 */
#define HRTIM_ADC3R_AD3TCPER           (1 << 23) /* Bit 23: ADC trigger 3 on Timer C Period*/
#define HRTIM_ADC3R_AD3TDC2            (1 << 24) /* Bit 24: ADC trigger 3 on Timer D Compare 2 */
#define HRTIM_ADC3R_AD3TDC3            (1 << 25) /* Bit 25: ADC trigger 3 on Timer D Compare 3 */
#define HRTIM_ADC3R_AD3TDC4            (1 << 26) /* Bit 26: ADC trigger 3 on Timer D Compare 4 */
#define HRTIM_ADC3R_AD3TDPER           (1 << 27) /* Bit 27: ADC trigger 3 on Timer D Period*/
#define HRTIM_ADC3R_AD3TEC2            (1 << 28) /* Bit 28: ADC trigger 3 on Timer E Compare 2 */
#define HRTIM_ADC3R_AD3TEC3            (1 << 29) /* Bit 29: ADC trigger 3 on Timer E Compare 3 */
#define HRTIM_ADC3R_AD3TEC4            (1 << 30) /* Bit 30: ADC trigger 3 on Timer E Compare 4 */
#define HRTIM_ADC3R_AD3TEPER           (1 << 31) /* Bit 31: ADC trigger 3 on Timer E Period */

/* Common ADC Trigger 4 Register */

#define HRTIM_ADC4R_AD4MC1             (1 << 0)  /* Bit 0: ADC trigger 4 on Master Compare 1 */
#define HRTIM_ADC4R_AD4MC2             (1 << 1)  /* Bit 1: ADC trigger 4 on Master Compare 2 */
#define HRTIM_ADC4R_AD4MC3             (1 << 2)  /* Bit 2: ADC trigger 4 on Master Compare 3 */
#define HRTIM_ADC4R_AD4MC4             (1 << 3)  /* Bit 3: ADC trigger 4 on Master Compare 4 */
#define HRTIM_ADC4R_AD4MPER            (1 << 4)  /* Bit 4: ADC trigger 4 on Master Period*/
#define HRTIM_ADC4R_AD4EEV6            (1 << 5)  /* Bit 5: ADC trigger 4 on External Event 6 */
#define HRTIM_ADC4R_AD4EEV7            (1 << 6)  /* Bit 6: ADC trigger 4 on External Event 7 */
#define HRTIM_ADC4R_AD4EEV8            (1 << 7)  /* Bit 7: ADC trigger 4 on External Event 8 */
#define HRTIM_ADC4R_AD4EEV9            (1 << 8)  /* Bit 8: ADC trigger 4 on External Event 9 */
#define HRTIM_ADC4R_AD4EEV10           (1 << 9)  /* Bit 9: ADC trigger 4 on External Event 10 */
#define HRTIM_ADC4R_AD4TAC2            (1 << 10) /* Bit 10: ADC trigger 4 on Timer A Compare 2 */
#define HRTIM_ADC4R_AD4TAC3            (1 << 11) /* Bit 11: ADC trigger 4 on Timer A Compare 3 */
#define HRTIM_ADC4R_AD4TAC4            (1 << 12) /* Bit 12: ADC trigger 4 on Timer A Compare 4 */
#define HRTIM_ADC4R_AD4TAPER           (1 << 13) /* Bit 13: ADC trigger 4 on Timer A Period */
#define HRTIM_ADC4R_AD4TBC2            (1 << 14) /* Bit 14: ADC trigger 4 on Timer B Compare 2 */
#define HRTIM_ADC4R_AD4TBC3            (1 << 15) /* Bit 15: ADC trigger 4 on Timer B Compare 3 */
#define HRTIM_ADC4R_AD4TBC4            (1 << 16) /* Bit 16: ADC trigger 4 on Timer B Compare 4 */
#define HRTIM_ADC4R_AD4TBPER           (1 << 17) /* Bit 18: ADC trigger 4 on Timer B Period */
#define HRTIM_ADC4R_AD4TCC2            (1 << 18) /* Bit 19: ADC trigger 4 on Timer C Compare 2 */
#define HRTIM_ADC4R_AD4TCC3            (1 << 19) /* Bit 20: ADC trigger 4 on Timer C Compare 3 */
#define HRTIM_ADC4R_AD4TCC4            (1 << 20) /* Bit 21: ADC trigger 4 on Timer C Compare 4 */
#define HRTIM_ADC4R_AD4TCPER           (1 << 21) /* Bit 22: ADC trigger 4 on Timer C Period*/
#define HRTIM_ADC4R_AD4TCRST           (1 << 22) /* Bit 22: ADC trigger 4 on Timer C Reset and counter roll-over*/
#define HRTIM_ADC4R_AD4TDC2            (1 << 23) /* Bit 23: ADC trigger 4 on Timer D Compare 2 */
#define HRTIM_ADC4R_AD4TDC3            (1 << 24) /* Bit 24: ADC trigger 4 on Timer D Compare 3 */
#define HRTIM_ADC4R_AD4TDC4            (1 << 25) /* Bit 25: ADC trigger 4 on Timer D Compare 4 */
#define HRTIM_ADC4R_AD4TDPER           (1 << 26) /* Bit 26: ADC trigger 4 on Timer D Period*/
#define HRTIM_ADC4R_AD4TDRST           (1 << 27) /* Bit 27: ADC trigger 4 on Timer D Reset and counter roll-over*/
#define HRTIM_ADC4R_AD4TEC2            (1 << 28) /* Bit 28: ADC trigger 4 on Timer E Compare 2 */
#define HRTIM_ADC4R_AD4TEC3            (1 << 29) /* Bit 29: ADC trigger 4 on Timer E Compare 3 */
#define HRTIM_ADC4R_AD4TEC4            (1 << 30) /* Bit 30: ADC trigger 4 on Timer E Compare 4 */
#define HRTIM_ADC4R_AD4TERST           (1 << 31) /* Bit 31: ADC trigger 4 on Timer E Reset and counter roll-over */

/* Common DLL Control Register */

#define HRTIM_DLLCR_CAL                (1 << 0)  /* Bit 0: DLL Calibration Start */
#define HRTIM_DLLCR_CALEN              (1 << 1)  /* Bit 1: DLL Calibration Enable */
#define HRTIM_DLLCR_CALRTE_SHIFT       2         /* Bits 2-3: DLL Calibration rate */
#define HRTIM_DLLCR_CALRTE_MASK        (3 << HRTIM_DLLCR_CALRTE_SHIFT)
#  define HRTIM_DLLCR_CALRTE_1048576   (0 << HRTIM_DLLCR_CALRTE_SHIFT) /* 00: 1048576 * tHRTIM */
#  define HRTIM_DLLCR_CALRTE_131072    (1 << HRTIM_DLLCR_CALRTE_SHIFT) /* 01: 131072 * tHRTIM */
#  define HRTIM_DLLCR_CALRTE_16384     (2 << HRTIM_DLLCR_CALRTE_SHIFT) /* 10: 16384 * tHRTIM */
#  define HRTIM_DLLCR_CALRTE_2048      (3 << HRTIM_DLLCR_CALRTE_SHIFT) /* 11: 2048 * tHRTIM */

/* Common Fault Input Register 1 */

#define HRTIM_FLTINR1_FLT1E            (1 << 0) /* Bit 0: Fault 1 enable */
#define HRTIM_FLTINR1_FLT1P            (1 << 1) /* Bit 1: Fault 1 polarity */
#define HRTIM_FLTINR1_FLT1SRC          (1 << 2) /* Bit 2: Fault 1 source */
#define HRTIM_FLTINR1_FLT1F_SHIFT      3        /* Bits 3-6: Fault 1 source */
#define HRTIM_FLTINR1_FLT1F_MASK       (15 << HRTIM_FLTINR1_FLT1F_SHIFT)
#  define HRTIM_FLTINR1_FLT1F_NOFLT    (0 << HRTIM_FLTINR1_FLT1F_SHIFT)  /* 0000: No filter, FLT5 acts asynchronously */
#  define HRTIM_FLTINR1_FLT1F_HRTN2    (1 << HRTIM_FLTINR1_FLT1F_SHIFT)  /* 0001: fSAMPLING = fHRTIM, N=2 */
#  define HRTIM_FLTINR1_FLT1F_HRTN4    (2 << HRTIM_FLTINR1_FLT1F_SHIFT)  /* 0010: fSAMPLING = fHRTIM, N=4 */
#  define HRTIM_FLTINR1_FLT1F_HRTN8    (3 << HRTIM_FLTINR1_FLT1F_SHIFT)  /* 0011: fSAMPLING = fHRTIM, N=8 */
#  define HRTIM_FLTINR1_FLT1F_FLTS2N6  (4 << HRTIM_FLTINR1_FLT1F_SHIFT)  /* 0100: fSAMPLING = fFLTS/2, N=6 */
#  define HRTIM_FLTINR1_FLT1F_FLTS2N8  (5 << HRTIM_FLTINR1_FLT1F_SHIFT)  /* 0101: fSAMPLING = fFLTS/2, N=8 */
#  define HRTIM_FLTINR1_FLT1F_FLTS4N6  (6 << HRTIM_FLTINR1_FLT1F_SHIFT)  /* 0110: fSAMPLING = fFLTS/4, N=6 */
#  define HRTIM_FLTINR1_FLT1F_FLTS4N8  (7 << HRTIM_FLTINR1_FLT1F_SHIFT)  /* 0111: fSAMPLING = fFLTS/4, N=8 */
#  define HRTIM_FLTINR1_FLT1F_FLTS8N6  (8 << HRTIM_FLTINR1_FLT1F_SHIFT)  /* 1000: fSAMPLING = fFLTS/8, N=6 */
#  define HRTIM_FLTINR1_FLT1F_FLTS8N8  (9 << HRTIM_FLTINR1_FLT1F_SHIFT)  /* 1001: fSAMPLING = fFLTS/8, N=8 */
#  define HRTIM_FLTINR1_FLT1F_FLTS16N5 (10 << HRTIM_FLTINR1_FLT1F_SHIFT) /* 1010: fSAMPLING = fFLTS/16, N=5 */
#  define HRTIM_FLTINR1_FLT1F_FLTS16N6 (11 << HRTIM_FLTINR1_FLT1F_SHIFT) /* 1011: fSAMPLING = fFLTS/16, N=6 */
#  define HRTIM_FLTINR1_FLT1F_FLTS16N8 (12 << HRTIM_FLTINR1_FLT1F_SHIFT) /* 1100: fSAMPLING = fFLTS/16, N=8 */
#  define HRTIM_FLTINR1_FLT1F_FLTS32N5 (13 << HRTIM_FLTINR1_FLT1F_SHIFT) /* 1101: fSAMPLING = fFLTS/32, N=5 */
#  define HRTIM_FLTINR1_FLT1F_FLTS32N6 (14 << HRTIM_FLTINR1_FLT1F_SHIFT) /* 1110: fSAMPLING = fFLTS/32, N=6 */
#  define HRTIM_FLTINR1_FLT1F_FLTS32N8 (15 << HRTIM_FLTINR1_FLT1F_SHIFT) /* 1111: fSAMPLING = fFLTS/32, N=8 */
#define HRTIM_FLTINR1_FLT1LCK          (1 << 7)                          /* Bit 7: Fault 1 lock */
#define HRTIM_FLTINR1_FLT2E            (1 << 8)                          /* Bit 8: Fault 2 enable */
#define HRTIM_FLTINR1_FLT2P            (1 << 9)                          /* Bit 9: Fault 2 polarity */
#define HRTIM_FLTINR1_FLT2SRC          (1 << 10)                         /* Bit 10: Fault 2 source */
#define HRTIM_FLTINR1_FLT2F_SHIFT      11                                /* Bits 11-14: Fault 2 source */
#define HRTIM_FLTINR1_FLT2F_MASK       (15 << HRTIM_FLTINR1_FLT2F_SHIFT)
#  define HRTIM_FLTINR1_FLT2F_NOFLT    (0 << HRTIM_FLTINR1_FLT2F_SHIFT)  /* 0000: No filter, FLT5 acts asynchronously */
#  define HRTIM_FLTINR1_FLT2F_HRTN2    (1 << HRTIM_FLTINR1_FLT2F_SHIFT)  /* 0001: fSAMPLING = fHRTIM, N=2 */
#  define HRTIM_FLTINR1_FLT2F_HRTN4    (2 << HRTIM_FLTINR1_FLT2F_SHIFT)  /* 0010: fSAMPLING = fHRTIM, N=4 */
#  define HRTIM_FLTINR1_FLT2F_HRTN8    (3 << HRTIM_FLTINR1_FLT2F_SHIFT)  /* 0011: fSAMPLING = fHRTIM, N=8 */
#  define HRTIM_FLTINR1_FLT2F_FLTS2N6  (4 << HRTIM_FLTINR1_FLT2F_SHIFT)  /* 0100: fSAMPLING = fFLTS/2, N=6 */
#  define HRTIM_FLTINR1_FLT2F_FLTS2N8  (5 << HRTIM_FLTINR1_FLT2F_SHIFT)  /* 0101: fSAMPLING = fFLTS/2, N=8 */
#  define HRTIM_FLTINR1_FLT2F_FLTS4N6  (6 << HRTIM_FLTINR1_FLT2F_SHIFT)  /* 0110: fSAMPLING = fFLTS/4, N=6 */
#  define HRTIM_FLTINR1_FLT2F_FLTS4N8  (7 << HRTIM_FLTINR1_FLT2F_SHIFT)  /* 0111: fSAMPLING = fFLTS/4, N=8 */
#  define HRTIM_FLTINR1_FLT2F_FLTS8N6  (8 << HRTIM_FLTINR1_FLT2F_SHIFT)  /* 1000: fSAMPLING = fFLTS/8, N=6 */
#  define HRTIM_FLTINR1_FLT2F_FLTS8N8  (9 << HRTIM_FLTINR1_FLT2F_SHIFT)  /* 1001: fSAMPLING = fFLTS/8, N=8 */
#  define HRTIM_FLTINR1_FLT2F_FLTS16N5 (10 << HRTIM_FLTINR1_FLT2F_SHIFT) /* 1010: fSAMPLING = fFLTS/16, N=5 */
#  define HRTIM_FLTINR1_FLT2F_FLTS16N6 (11 << HRTIM_FLTINR1_FLT2F_SHIFT) /* 1011: fSAMPLING = fFLTS/16, N=6 */
#  define HRTIM_FLTINR1_FLT2F_FLTS16N8 (12 << HRTIM_FLTINR1_FLT2F_SHIFT) /* 1100: fSAMPLING = fFLTS/16, N=8 */
#  define HRTIM_FLTINR1_FLT2F_FLTS32N5 (13 << HRTIM_FLTINR1_FLT2F_SHIFT) /* 1101: fSAMPLING = fFLTS/32, N=5 */
#  define HRTIM_FLTINR1_FLT2F_FLTS32N6 (14 << HRTIM_FLTINR1_FLT2F_SHIFT) /* 1110: fSAMPLING = fFLTS/32, N=6 */
#  define HRTIM_FLTINR1_FLT2F_FLTS32N8 (15 << HRTIM_FLTINR1_FLT2F_SHIFT) /* 1111: fSAMPLING = fFLTS/32, N=8 */
#define HRTIM_FLTINR1_FLT2LCK          (1 << 15)                         /* Bit 15: Fault 2 lock */
#define HRTIM_FLTINR1_FLT3E            (1 << 16)                         /* Bit 16: Fault 3 enable */
#define HRTIM_FLTINR1_FLT3P            (1 << 17)                         /* Bit 17: Fault 3 polarity */
#define HRTIM_FLTINR1_FLT3SRC          (1 << 18)                         /* Bit 18: Fault 3 source */
#define HRTIM_FLTINR1_FLT3F_SHIFT      19                                /* Bits 19-22: Fault 3 source */
#define HRTIM_FLTINR1_FLT3F_MASK       (15 << HRTIM_FLTINR1_FLT3F_SHIFT)
#  define HRTIM_FLTINR1_FLT3F_NOFLT    (0 << HRTIM_FLTINR1_FLT3F_SHIFT)  /* 0000: No filter, FLT5 acts asynchronously */
#  define HRTIM_FLTINR1_FLT3F_HRTN2    (1 << HRTIM_FLTINR1_FLT3F_SHIFT)  /* 0001: fSAMPLING = fHRTIM, N=2 */
#  define HRTIM_FLTINR1_FLT3F_HRTN4    (2 << HRTIM_FLTINR1_FLT3F_SHIFT)  /* 0010: fSAMPLING = fHRTIM, N=4 */
#  define HRTIM_FLTINR1_FLT3F_HRTN8    (3 << HRTIM_FLTINR1_FLT3F_SHIFT)  /* 0011: fSAMPLING = fHRTIM, N=8 */
#  define HRTIM_FLTINR1_FLT3F_FLTS2N6  (4 << HRTIM_FLTINR1_FLT3F_SHIFT)  /* 0100: fSAMPLING = fFLTS/2, N=6 */
#  define HRTIM_FLTINR1_FLT3F_FLTS2N8  (5 << HRTIM_FLTINR1_FLT3F_SHIFT)  /* 0101: fSAMPLING = fFLTS/2, N=8 */
#  define HRTIM_FLTINR1_FLT3F_FLTS4N6  (6 << HRTIM_FLTINR1_FLT3F_SHIFT)  /* 0110: fSAMPLING = fFLTS/4, N=6 */
#  define HRTIM_FLTINR1_FLT3F_FLTS4N8  (7 << HRTIM_FLTINR1_FLT3F_SHIFT)  /* 0111: fSAMPLING = fFLTS/4, N=8 */
#  define HRTIM_FLTINR1_FLT3F_FLTS8N6  (8 << HRTIM_FLTINR1_FLT3F_SHIFT)  /* 1000: fSAMPLING = fFLTS/8, N=6 */
#  define HRTIM_FLTINR1_FLT3F_FLTS8N8  (9 << HRTIM_FLTINR1_FLT3F_SHIFT)  /* 1001: fSAMPLING = fFLTS/8, N=8 */
#  define HRTIM_FLTINR1_FLT3F_FLTS16N5 (10 << HRTIM_FLTINR1_FLT3F_SHIFT) /* 1010: fSAMPLING = fFLTS/16, N=5 */
#  define HRTIM_FLTINR1_FLT3F_FLTS16N6 (11 << HRTIM_FLTINR1_FLT3F_SHIFT) /* 1011: fSAMPLING = fFLTS/16, N=6 */
#  define HRTIM_FLTINR1_FLT3F_FLTS16N8 (12 << HRTIM_FLTINR1_FLT3F_SHIFT) /* 1100: fSAMPLING = fFLTS/16, N=8 */
#  define HRTIM_FLTINR1_FLT3F_FLTS32N5 (13 << HRTIM_FLTINR1_FLT3F_SHIFT) /* 1101: fSAMPLING = fFLTS/32, N=5 */
#  define HRTIM_FLTINR1_FLT3F_FLTS32N6 (14 << HRTIM_FLTINR1_FLT3F_SHIFT) /* 1110: fSAMPLING = fFLTS/32, N=6 */
#  define HRTIM_FLTINR1_FLT3F_FLTS32N8 (15 << HRTIM_FLTINR1_FLT3F_SHIFT) /* 1111: fSAMPLING = fFLTS/32, N=8 */
#define HRTIM_FLTINR1_FLT3LCK          (1 << 23)                         /* Bit 23: Fault 3 lock */
#define HRTIM_FLTINR1_FLT4E            (1 << 24)                         /* Bit 24: Fault 4 enable */
#define HRTIM_FLTINR1_FLT4P            (1 << 25)                         /* Bit 25: Fault 4 polarity */
#define HRTIM_FLTINR1_FLT4SRC          (1 << 26)                         /* Bit 26: Fault 4 source */
#define HRTIM_FLTINR1_FLT4F_SHIFT      27                                /* Bits 27-30: Fault 4 source */
#define HRTIM_FLTINR1_FLT4F_MASK       (15 << HRTIM_FLTINR1_FLT4F_SHIFT)
#  define HRTIM_FLTINR1_FLT4F_NOFLT    (0 << HRTIM_FLTINR1_FLT4F_SHIFT)  /* 0000: No filter, FLT5 acts asynchronously */
#  define HRTIM_FLTINR1_FLT4F_HRTN2    (1 << HRTIM_FLTINR1_FLT4F_SHIFT)  /* 0001: fSAMPLING = fHRTIM, N=2 */
#  define HRTIM_FLTINR1_FLT4F_HRTN4    (2 << HRTIM_FLTINR1_FLT4F_SHIFT)  /* 0010: fSAMPLING = fHRTIM, N=4 */
#  define HRTIM_FLTINR1_FLT4F_HRTN8    (3 << HRTIM_FLTINR1_FLT4F_SHIFT)  /* 0011: fSAMPLING = fHRTIM, N=8 */
#  define HRTIM_FLTINR1_FLT4F_FLTS2N6  (4 << HRTIM_FLTINR1_FLT4F_SHIFT)  /* 0100: fSAMPLING = fFLTS/2, N=6 */
#  define HRTIM_FLTINR1_FLT4F_FLTS2N8  (5 << HRTIM_FLTINR1_FLT4F_SHIFT)  /* 0101: fSAMPLING = fFLTS/2, N=8 */
#  define HRTIM_FLTINR1_FLT4F_FLTS4N6  (6 << HRTIM_FLTINR1_FLT4F_SHIFT)  /* 0110: fSAMPLING = fFLTS/4, N=6 */
#  define HRTIM_FLTINR1_FLT4F_FLTS4N8  (7 << HRTIM_FLTINR1_FLT4F_SHIFT)  /* 0111: fSAMPLING = fFLTS/4, N=8 */
#  define HRTIM_FLTINR1_FLT4F_FLTS8N6  (8 << HRTIM_FLTINR1_FLT4F_SHIFT)  /* 1000: fSAMPLING = fFLTS/8, N=6 */
#  define HRTIM_FLTINR1_FLT4F_FLTS8N8  (9 << HRTIM_FLTINR1_FLT4F_SHIFT)  /* 1001: fSAMPLING = fFLTS/8, N=8 */
#  define HRTIM_FLTINR1_FLT4F_FLTS16N5 (10 << HRTIM_FLTINR1_FLT4F_SHIFT) /* 1010: fSAMPLING = fFLTS/16, N=5 */
#  define HRTIM_FLTINR1_FLT4F_FLTS16N6 (11 << HRTIM_FLTINR1_FLT4F_SHIFT) /* 1011: fSAMPLING = fFLTS/16, N=6 */
#  define HRTIM_FLTINR1_FLT4F_FLTS16N8 (12 << HRTIM_FLTINR1_FLT4F_SHIFT) /* 1100: fSAMPLING = fFLTS/16, N=8 */
#  define HRTIM_FLTINR1_FLT4F_FLTS32N5 (13 << HRTIM_FLTINR1_FLT4F_SHIFT) /* 1101: fSAMPLING = fFLTS/32, N=5 */
#  define HRTIM_FLTINR1_FLT4F_FLTS32N6 (14 << HRTIM_FLTINR1_FLT4F_SHIFT) /* 1110: fSAMPLING = fFLTS/32, N=6 */
#  define HRTIM_FLTINR1_FLT4F_FLTS32N8 (15 << HRTIM_FLTINR1_FLT4F_SHIFT) /* 1111: fSAMPLING = fFLTS/32, N=8 */
#define HRTIM_FLTINR1_FLT4LCK          (1 << 31)                         /* Bit 31: Fault 4 lock */

/* Common Fault Input Register 2 */

#define HRTIM_FLTINR2_FLT5E            (1 << 0) /* Bit 0: Fault 5 enable */
#define HRTIM_FLTINR2_FLT5P            (1 << 1) /* Bit 1: Fault 5 polarity */
#define HRTIM_FLTINR2_FLT5SRC          (1 << 2) /* Bit 2: Fault 5 source */
#define HRTIM_FLTINR2_FLT5F_SHIFT      3        /* Bits 3-6: Fault 5 source */
#define HRTIM_FLTINR2_FLT5F_MASK       (15 << HRTIM_FLTINR2_FLT5F_SHIFT)
#  define HRTIM_FLTINR2_FLT5F_NOFLT    (0 << HRTIM_FLTINR2_FLT5F_SHIFT)  /* 0000: No filter, FLT5 acts asynchronously */
#  define HRTIM_FLTINR2_FLT5F_HRTN2    (1 << HRTIM_FLTINR2_FLT5F_SHIFT)  /* 0001: fSAMPLING = fHRTIM, N=2 */
#  define HRTIM_FLTINR2_FLT5F_HRTN4    (2 << HRTIM_FLTINR2_FLT5F_SHIFT)  /* 0010: fSAMPLING = fHRTIM, N=4 */
#  define HRTIM_FLTINR2_FLT5F_HRTN8    (3 << HRTIM_FLTINR2_FLT5F_SHIFT)  /* 0011: fSAMPLING = fHRTIM, N=8 */
#  define HRTIM_FLTINR2_FLT5F_FLTS2N6  (4 << HRTIM_FLTINR2_FLT5F_SHIFT)  /* 0100: fSAMPLING = fFLTS/2, N=6 */
#  define HRTIM_FLTINR2_FLT5F_FLTS2N8  (5 << HRTIM_FLTINR2_FLT5F_SHIFT)  /* 0101: fSAMPLING = fFLTS/2, N=8 */
#  define HRTIM_FLTINR2_FLT5F_FLTS4N6  (6 << HRTIM_FLTINR2_FLT5F_SHIFT)  /* 0110: fSAMPLING = fFLTS/4, N=6 */
#  define HRTIM_FLTINR2_FLT5F_FLTS4N8  (7 << HRTIM_FLTINR2_FLT5F_SHIFT)  /* 0111: fSAMPLING = fFLTS/4, N=8 */
#  define HRTIM_FLTINR2_FLT5F_FLTS8N6  (8 << HRTIM_FLTINR2_FLT5F_SHIFT)  /* 1000: fSAMPLING = fFLTS/8, N=6 */
#  define HRTIM_FLTINR2_FLT5F_FLTS8N8  (9 << HRTIM_FLTINR2_FLT5F_SHIFT)  /* 1001: fSAMPLING = fFLTS/8, N=8 */
#  define HRTIM_FLTINR2_FLT5F_FLTS16N5 (10 << HRTIM_FLTINR2_FLT5F_SHIFT) /* 1010: fSAMPLING = fFLTS/16, N=5 */
#  define HRTIM_FLTINR2_FLT5F_FLTS16N6 (11 << HRTIM_FLTINR2_FLT5F_SHIFT) /* 1011: fSAMPLING = fFLTS/16, N=6 */
#  define HRTIM_FLTINR2_FLT5F_FLTS16N8 (12 << HRTIM_FLTINR2_FLT5F_SHIFT) /* 1100: fSAMPLING = fFLTS/16, N=8 */
#  define HRTIM_FLTINR2_FLT5F_FLTS32N5 (13 << HRTIM_FLTINR2_FLT5F_SHIFT) /* 1101: fSAMPLING = fFLTS/32, N=5 */
#  define HRTIM_FLTINR2_FLT5F_FLTS32N6 (14 << HRTIM_FLTINR2_FLT5F_SHIFT) /* 1110: fSAMPLING = fFLTS/32, N=6 */
#  define HRTIM_FLTINR2_FLT5F_FLTS32N8 (15 << HRTIM_FLTINR2_FLT5F_SHIFT) /* 1111: fSAMPLING = fFLTS/32, N=8 */
#define HRTIM_FLTINR2_FLT5LCK          (1 << 7)                          /* Bit 7: Fault 5 lock */
#define HRTIM_FLTINR2_FLTSD_SWITCH     24                                /* Bits 24-25: Fault Sampling clock division */
#define HRTIM_FLTINR2_FLTSD_MASK       (3 << HRTIM_FLTINR2_FLTSD_SWITCH)
#  define HRTIM_FLTINR2_FLTSD_NODIV    (0 << HRTIM_FLTINR2_FLTSD_SWITCH) /* 00: fFLTS=fHRTIM */
#  define HRTIM_FLTINR2_FLTSD_d2       (1 << HRTIM_FLTINR2_FLTSD_SWITCH) /* 01: fFLTS=fHRTIM/2 */
#  define HRTIM_FLTINR2_FLTSD_d4       (2 << HRTIM_FLTINR2_FLTSD_SWITCH) /* 10: fFLTS=fHRTIM/4 */
#  define HRTIM_FLTINR2_FLTSD_d8       (3 << HRTIM_FLTINR2_FLTSD_SWITCH) /* 11: fFLTS=fHRTIM/8 */

/* Common Burst DMA Master Timer Update Register */

#define HRTIM_BDMUPR_MCR               (1 << 0)  /* Bit 0: MCR register update enable */
#define HRTIM_BDMUPR_MICR              (1 << 1)  /* Bit 1: MICR register update enable  */
#define HRTIM_BDMUPR_MDIER             (1 << 2)  /* Bit 2: MDIER register update enable  */
#define HRTIM_BDMUPR_MCNT              (1 << 3)  /* Bit 3: MCNTR register update enable */
#define HRTIM_BDMUPR_MPER              (1 << 4)  /* Bit 4: MPER register update enable */
#define HRTIM_BDMUPR_MREP              (1 << 5)  /* Bit 5: MREP register update enable */
#define HRTIM_BDMUPR_MCMP1             (1 << 6)  /* Bit 6: MCMP1R register update enable */
#define HRTIM_BDMUPR_MCMP2             (1 << 7)  /* Bit 7: MCMP2R register update enable */
#define HRTIM_BDMUPR_MCMP3             (1 << 8)  /* Bit 8: MCMP3R register update enable */
#define HRTIM_BDMUPR_MCMP4             (1 << 9)  /* Bit 9: MCMP4R register update enable */

/* Common Burst DMA Timer X Update Register (Timer A-E) */

#define HRTIM_BDTxUPR_CR               (1 << 0)  /* Bit 0: HRTIM_TIMxCR register update enablce */
#define HRTIM_BDTxUPR_ICR              (1 << 1)  /* Bit 1: HRTIM_TIMxICR register update enablce */
#define HRTIM_BDTxUPR_DIER             (1 << 2)  /* Bit 2: HRTIM_TIMxDIER register update enablce */
#define HRTIM_BDTxUPR_CNT              (1 << 3)  /* Bit 3: HRTIM_CNTxR register update enablce */
#define HRTIM_BDTxUPR_PER              (1 << 4)  /* Bit 4: HRTIM_PERxR register update enablce */
#define HRTIM_BDTxUPR_REP              (1 << 5)  /* Bit 5: HRTIM_REPxR register update enablce */
#define HRTIM_BDTxUPR_CMP1             (1 << 6)  /* Bit 6: HRTIM_CMP1xR register update enablce */
#define HRTIM_BDTxUPR_CMP2             (1 << 7)  /* Bit 7: HRTIM_CMP2xR register update enablce */
#define HRTIM_BDTxUPR_CMP3             (1 << 8)  /* Bit 8: HRTIM_CMP3xR register update enablce */
#define HRTIM_BDTxUPR_CMP4             (1 << 9)  /* Bit 9: HRTIM_CMP4xR register update enablce */
#define HRTIM_BDTxUPR_DTR              (1 << 10) /* Bit 10: HRTIM_DTxR register update enablce */
#define HRTIM_BDTxUPR_SET1R            (1 << 11) /* Bit 11: HRTIM_SET1xR register update enablce */
#define HRTIM_BDTxUPR_RST1R            (1 << 12) /* Bit 12: HRTIM_RST1xR register update enablce */
#define HRTIM_BDTxUPR_SET2R            (1 << 13) /* Bit 13: HRTIM_SET2xR register update enablce */
#define HRTIM_BDTxUPR_RST2R            (1 << 14) /* Bit 14: HRTIM_RST2xR register update enablce */
#define HRTIM_BDTxUPR_EEFR1            (1 << 15) /* Bit 15: HRTIM_EEFxR1 register update enablce */
#define HRTIM_BDTxUPR_EEFR2            (1 << 16) /* Bit 16: HRTIM_EEFxR2 register update enablce */
#define HRTIM_BDTxUPR_RSTR             (1 << 17) /* Bit 17: HRTIM_RSTxR register update enablce */
#define HRTIM_BDTxUPR_CHPR             (1 << 18) /* Bit 18: HRTIM_CHRxR register update enablce */
#define HRTIM_BDTxUPR_OUTR             (1 << 19) /* Bit 19: HRTIM_OUTxR register update enablce */
#define HRTIM_BDTxUPR_FLTR             (1 << 20) /* Bit 20: HRTIM_FLTxR register update enablce */

/* Common Burst DMA Data Register */

#define HRTIM_BDMADR_SHIFT             0         /* Bits 0-31: Burst DMA Data register */
#define HRTIM_BDMADR_MASK              (0xffffffff << HRTIM_BDMADR_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_HRTIM_H */
