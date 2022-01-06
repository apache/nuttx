/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32gxxxxx_dac.h
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32GXXXXX_DAC_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32GXXXXX_DAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_IP_DAC_V2
#undef HAVE_IP_DAC_V1           /* No DAC IPv1 */

/* Register Offsets *********************************************************/

#define STM32_DAC_CR_OFFSET            0x0000                         /* DAC control register */
#define STM32_DAC_SWTRIGR_OFFSET       0x0004                         /* DAC software trigger register */
#define STM32_DAC_DHR12R1_OFFSET       0x0008                         /* DAC channel 1 12-bit right-aligned data holding register */
#define STM32_DAC_DHR12L1_OFFSET       0x000c                         /* DAC channel 1 12-bit left aligned data holding register */
#define STM32_DAC_DHR8R1_OFFSET        0x0010                         /* DAC channel 1 8-bit right aligned data holding register */
#define STM32_DAC_DHR12R2_OFFSET       0x0014                         /* DAC channel 2 12-bit right aligned data holding register */
#define STM32_DAC_DHR12L2_OFFSET       0x0018                         /* DAC channel 2 12-bit left aligned data holding register */
#define STM32_DAC_DHR8R2_OFFSET        0x001c                         /* DAC channel 2 8-bit right-aligned data holding register */
#define STM32_DAC_DHR12RD_OFFSET       0x0020                         /* Dual DAC 12-bit right-aligned data holding register */
#define STM32_DAC_DHR12LD_OFFSET       0x0024                         /* DUAL DAC 12-bit left aligned data holding register */
#define STM32_DAC_DHR8RD_OFFSET        0x0028                         /* DUAL DAC 8-bit right aligned data holding register */
#define STM32_DAC_DOR1_OFFSET          0x002c                         /* DAC channel 1 data output register */
#define STM32_DAC_DOR2_OFFSET          0x0030                         /* DAC channel 2 data output register */
#define STM32_DAC_SR_OFFSET            0x0034                         /* DAC status register */
#define STM32_DAC_CCR_OFFSET           0x0038                         /* DAC calibration control register */
#define STM32_DAC_MCR_OFFSET           0x003c                         /* DAC mode control register */
#define STM32_DAC_SHSR1_OFFSET         0x0040                         /* DAC channel 1 sample and hold time register */
#define STM32_DAC_SHSR2_OFFSET         0x0044                         /* DAC channel 2 sample and hold time register */
#define STM32_DAC_SHHR_OFFSET          0x0048                         /* DAC sample and hold time register */
#define STM32_DAC_SHRR_OFFSET          0x004c                         /* DAC sample and hold refresh time register */
#define STM32_DAC_STR1_OFFSET          0x0058                         /* DAC channel 1 sawtooth register */
#define STM32_DAC_STR2_OFFSET          0x005c                         /* DAC channel 2 sawtooth register */
#define STM32_DAC_STMODR_OFFSET        0x0060                         /* DAC sawtooth mode register */

/* Register Addresses *******************************************************/

#ifdef CONFIG_STM32_HAVE_DAC1
/* DAC1 */

#  define STM32_DAC1_CR                (STM32_DAC1_BASE + STM32_DAC_CR_OFFSET)
#  define STM32_DAC1_SWTRIGR           (STM32_DAC1_BASE + STM32_DAC_SWTRIGR_OFFSET)
#  define STM32_DAC1_DHR12R1           (STM32_DAC1_BASE + STM32_DAC_DHR12R1_OFFSET)
#  define STM32_DAC1_DHR12L1           (STM32_DAC1_BASE + STM32_DAC_DHR12L1_OFFSET)
#  define STM32_DAC1_DHR8R1            (STM32_DAC1_BASE + STM32_DAC_DHR8R1_OFFSET)
#  define STM32_DAC1_DHR12R2           (STM32_DAC1_BASE + STM32_DAC_DHR12R2_OFFSET)
#  define STM32_DAC1_DHR12L2           (STM32_DAC1_BASE + STM32_DAC_DHR12L2_OFFSET)
#  define STM32_DAC1_DHR8R2            (STM32_DAC1_BASE + STM32_DAC_DHR8R2_OFFSET)
#  define STM32_DAC1_DHR12RD           (STM32_DAC1_BASE + STM32_DAC_DHR12RD_OFFSET)
#  define STM32_DAC1_DHR12LD           (STM32_DAC1_BASE + STM32_DAC_DHR12LD_OFFSET)
#  define STM32_DAC1_DHR8RD            (STM32_DAC1_BASE + STM32_DAC_DHR8RD_OFFSET)
#  define STM32_DAC1_DOR1              (STM32_DAC1_BASE + STM32_DAC_DOR1_OFFSET)
#  define STM32_DAC1_DOR2              (STM32_DAC1_BASE + STM32_DAC_DOR2_OFFSET)
#  define STM32_DAC1_SR                (STM32_DAC1_BASE + STM32_DAC_SR_OFFSET)
#  define STM32_DAC1_CCR               (STM32_DAC1_BASE + STM32_DAC_CCR_OFFSET)
#  define STM32_DAC1_MCR               (STM32_DAC1_BASE + STM32_DAC_MCR_OFFSET)
#  define STM32_DAC1_SHSR1             (STM32_DAC1_BASE + STM32_DAC_SHSR1_OFFSET)
#  define STM32_DAC1_SHSR2             (STM32_DAC1_BASE + STM32_DAC_SHSR2_OFFSET)
#  define STM32_DAC1_SHHR              (STM32_DAC1_BASE + STM32_DAC_SHHR_OFFSET)
#  define STM32_DAC1_SHRR              (STM32_DAC1_BASE + STM32_DAC_SHRR_OFFSET)
#  define STM32_DAC1_STR1              (STM32_DAC1_BASE + STM32_DAC_STR1_OFFSET)
#  define STM32_DAC1_STR2              (STM32_DAC1_BASE + STM32_DAC_STR2_OFFSET)
#  define STM32_DAC1_STMODR            (STM32_DAC1_BASE + STM32_DAC_STMODR_OFFSET)

#endif /* CONFIG_STM32_HAVE_DAC1 */

#ifdef CONFIG_STM32_HAVE_DAC2
/* DAC2 */

#  define STM32_DAC2_CR                (STM32_DAC2_BASE + STM32_DAC_CR_OFFSET)
#  define STM32_DAC2_SWTRIGR           (STM32_DAC2_BASE + STM32_DAC_SWTRIGR_OFFSET)
#  define STM32_DAC2_DHR12R1           (STM32_DAC2_BASE + STM32_DAC_DHR12R1_OFFSET)
#  define STM32_DAC2_DHR12L1           (STM32_DAC2_BASE + STM32_DAC_DHR12L1_OFFSET)
#  define STM32_DAC2_DHR8R1            (STM32_DAC2_BASE + STM32_DAC_DHR8R1_OFFSET)
#  define STM32_DAC2_DHR12R2           (STM32_DAC2_BASE + STM32_DAC_DHR12R2_OFFSET)
#  define STM32_DAC2_DHR12L2           (STM32_DAC2_BASE + STM32_DAC_DHR12L2_OFFSET)
#  define STM32_DAC2_DHR8R2            (STM32_DAC2_BASE + STM32_DAC_DHR8R2_OFFSET)
#  define STM32_DAC2_DHR12RD           (STM32_DAC2_BASE + STM32_DAC_DHR12RD_OFFSET)
#  define STM32_DAC2_DHR12LD           (STM32_DAC2_BASE + STM32_DAC_DHR12LD_OFFSET)
#  define STM32_DAC2_DHR8RD            (STM32_DAC2_BASE + STM32_DAC_DHR8RD_OFFSET)
#  define STM32_DAC2_DOR1              (STM32_DAC2_BASE + STM32_DAC_DOR1_OFFSET)
#  define STM32_DAC2_DOR2              (STM32_DAC2_BASE + STM32_DAC_DOR2_OFFSET)
#  define STM32_DAC2_SR                (STM32_DAC2_BASE + STM32_DAC_SR_OFFSET)
#  define STM32_DAC2_CCR               (STM32_DAC2_BASE + STM32_DAC_CCR_OFFSET)
#  define STM32_DAC2_MCR               (STM32_DAC2_BASE + STM32_DAC_MCR_OFFSET)
#  define STM32_DAC2_SHSR1             (STM32_DAC2_BASE + STM32_DAC_SHSR1_OFFSET)
#  define STM32_DAC2_SHSR2             (STM32_DAC2_BASE + STM32_DAC_SHSR2_OFFSET)
#  define STM32_DAC2_SHHR              (STM32_DAC2_BASE + STM32_DAC_SHHR_OFFSET)
#  define STM32_DAC2_SHRR              (STM32_DAC2_BASE + STM32_DAC_SHRR_OFFSET)
#  define STM32_DAC2_STR1              (STM32_DAC2_BASE + STM32_DAC_STR1_OFFSET)
#  define STM32_DAC2_STR2              (STM32_DAC2_BASE + STM32_DAC_STR2_OFFSET)
#  define STM32_DAC2_STMODR            (STM32_DAC2_BASE + STM32_DAC_STMODR_OFFSET)

#endif /* CONFIG_STM32_HAVE_DAC2 */

#ifdef CONFIG_STM32_HAVE_DAC3
/* DAC3 */

#  define STM32_DAC3_CR                (STM32_DAC3_BASE + STM32_DAC_CR_OFFSET)
#  define STM32_DAC3_SWTRIGR           (STM32_DAC3_BASE + STM32_DAC_SWTRIGR_OFFSET)
#  define STM32_DAC3_DHR12R1           (STM32_DAC3_BASE + STM32_DAC_DHR12R1_OFFSET)
#  define STM32_DAC3_DHR12L1           (STM32_DAC3_BASE + STM32_DAC_DHR12L1_OFFSET)
#  define STM32_DAC3_DHR8R1            (STM32_DAC3_BASE + STM32_DAC_DHR8R1_OFFSET)
#  define STM32_DAC3_DHR12R2           (STM32_DAC3_BASE + STM32_DAC_DHR12R2_OFFSET)
#  define STM32_DAC3_DHR12L2           (STM32_DAC3_BASE + STM32_DAC_DHR12L2_OFFSET)
#  define STM32_DAC3_DHR8R2            (STM32_DAC3_BASE + STM32_DAC_DHR8R2_OFFSET)
#  define STM32_DAC3_DHR12RD           (STM32_DAC3_BASE + STM32_DAC_DHR12RD_OFFSET)
#  define STM32_DAC3_DHR12LD           (STM32_DAC3_BASE + STM32_DAC_DHR12LD_OFFSET)
#  define STM32_DAC3_DHR8RD            (STM32_DAC3_BASE + STM32_DAC_DHR8RD_OFFSET)
#  define STM32_DAC3_DOR1              (STM32_DAC3_BASE + STM32_DAC_DOR1_OFFSET)
#  define STM32_DAC3_DOR2              (STM32_DAC3_BASE + STM32_DAC_DOR2_OFFSET)
#  define STM32_DAC3_SR                (STM32_DAC3_BASE + STM32_DAC_SR_OFFSET)
#  define STM32_DAC3_CCR               (STM32_DAC3_BASE + STM32_DAC_CCR_OFFSET)
#  define STM32_DAC3_MCR               (STM32_DAC3_BASE + STM32_DAC_MCR_OFFSET)
#  define STM32_DAC3_SHSR1             (STM32_DAC3_BASE + STM32_DAC_SHSR1_OFFSET)
#  define STM32_DAC3_SHSR2             (STM32_DAC3_BASE + STM32_DAC_SHSR2_OFFSET)
#  define STM32_DAC3_SHHR              (STM32_DAC3_BASE + STM32_DAC_SHHR_OFFSET)
#  define STM32_DAC3_SHRR              (STM32_DAC3_BASE + STM32_DAC_SHRR_OFFSET)
#  define STM32_DAC3_STR1              (STM32_DAC3_BASE + STM32_DAC_STR1_OFFSET)
#  define STM32_DAC3_STR2              (STM32_DAC3_BASE + STM32_DAC_STR2_OFFSET)
#  define STM32_DAC3_STMODR            (STM32_DAC3_BASE + STM32_DAC_STMODR_OFFSET)

#endif /* CONFIG_STM32_HAVE_DAC3 */

/* Register Bitfield Definitions ********************************************/

/* DAC control register (CR) */

/* These definitions may be used for 16-bit values of either channel: Leave
 * as-is for use with DAC channel 1 or shift left by 16 for use with DAC
 * channel 2.
 */

#define DAC_CR_EN                      (1 << 0)                       /* Bit 0: DAC channel enable */
#define DAC_CR_TEN                     (1 << 1)                       /* Bit 2: DAC channel trigger enable */
#define DAC_CR_TSEL_SHIFT              (2)                            /* Bits 2-5: DAC channel trigger selection */
#define DAC_CR_TSEL_MASK               (0xf << DAC_CR_TSEL_SHIFT)     /* Possible values for TSEL follow: */
#  define DAC_CR_TSEL_SW               (0x0 << DAC_CR_TSEL_SHIFT)     /* SWTRIG1 */
#  define DAC_CR_TSEL_TIM8             (0x1 << DAC_CR_TSEL_SHIFT)     /* dac_chx_trg1 - TIM8_TRGO */
#  define DAC_CR_TSEL_TIM7             (0x2 << DAC_CR_TSEL_SHIFT)     /* dac_chx_trg2 - TIM7_TRGO */
#  define DAC_CR_TSEL_TIM15            (0x3 << DAC_CR_TSEL_SHIFT)     /* dac_chx_trg3 - TIM15_TRGO */
#  define DAC_CR_TSEL_TIM2             (0x4 << DAC_CR_TSEL_SHIFT)     /* dac_chx_trg4 - TIM2_TRGO */
#  define DAC_CR_TSEL_TIM4             (0x5 << DAC_CR_TSEL_SHIFT)     /* dac_chx_trg5 - TIM4_TRGO */
#  define DAC_CR_TSEL_EXTI9            (0x6 << DAC_CR_TSEL_SHIFT)     /* dac_chx_trg6 - EXTI9 */
#  define DAC_CR_TSEL_TIM6             (0x7 << DAC_CR_TSEL_SHIFT)     /* dac_chx_trg7 - TIM6_TRGO */
#  define DAC_CR_TSEL_TIM3             (0x8 << DAC_CR_TSEL_SHIFT)     /* dac_chx_trg8 - TIM3_TRGO */
#  define DAC_CR_TSEL_HRT1RTRG1        (0x9 << DAC_CR_TSEL_SHIFT)     /* dac_chx_trg9 -  htrim_dac_reset_trg1 */
#  define DAC_CR_TSEL_HRT1RTRG2        (0xa << DAC_CR_TSEL_SHIFT)     /* dac_chx_trg10 - htrim_dac_reset_trg2 */
#  define DAC_CR_TSEL_HRT1RTRG3        (0xb << DAC_CR_TSEL_SHIFT)     /* dac_chx_trg11 - htrim_dac_reset_trg3 */
#  define DAC_CR_TSEL_HRT1RTRG4        (0xc << DAC_CR_TSEL_SHIFT)     /* dac_chx_trg12 - htrim_dac_reset_trg4 */
#  define DAC_CR_TSEL_HRT1RTRG5        (0xd << DAC_CR_TSEL_SHIFT)     /* dac_chx_trg13 - htrim_dac_reset_trg5 */
#  define DAC_CR_TSEL_HRT1RTRG6        (0xe << DAC_CR_TSEL_SHIFT)     /* dac_chx_trg14 - htrim_dac_reset_trg6 */
#  define DAC_CR_TSEL_HRT1TRG1         (0xf << DAC_CR_TSEL_SHIFT)     /* dac_chx_trg15 - htrim_dac_trg1 */
#define DAC_CR_WAVE_SHIFT              (6)                            /* Bits 6-7: DAC channel noise/triangle wave generation enable */
#define DAC_CR_WAVE_MASK               (0x3 << DAC_CR_WAVE_SHIFT)     /* Possible values for WAVE follow: */
#  define DAC_CR_WAVE_DISABLED         (0x0 << DAC_CR_WAVE_SHIFT)     /* Wave generation disabled */
#  define DAC_CR_WAVE_NOISE            (0x1 << DAC_CR_WAVE_SHIFT)     /* Noise wave generation enabled */
#  define DAC_CR_WAVE_TRIANGLE         (0x2 << DAC_CR_WAVE_SHIFT)     /* Triangle wave generation enabled */
#  define DAC_CR_WAVE_SAWTOOTH         (0x3 << DAC_CR_WAVE_SHIFT)     /* Sawtooth wave generation enabled */
#define DAC_CR_MAMP_SHIFT              (8)                            /* Bits 8-11: DAC channel mask/amplitude selector */
#define DAC_CR_MAMP_MASK               (0xf << DAC_CR_MAMP_SHIFT)     /* Possible values for MASK follow: */
#  define DAC_CR_MAMP_BIT0             (0x0 << DAC_CR_MAMP_SHIFT)     /* In wave generation mode, unmask bit 0 of LFSR */
#  define DAC_CR_MAMP_BITS1_0          (0x1 << DAC_CR_MAMP_SHIFT)     /* In wave generation mode, unmask bits[1:0] of LFSR */
#  define DAC_CR_MAMP_BITS2_0          (0x2 << DAC_CR_MAMP_SHIFT)     /* In wave generation mode, unmask bits[2:0] of LFSR */
#  define DAC_CR_MAMP_BITS3_0          (0x3 << DAC_CR_MAMP_SHIFT)     /* In wave generation mode, unmask bits[3:0] of LFSR */
#  define DAC_CR_MAMP_BITS4_0          (0x4 << DAC_CR_MAMP_SHIFT)     /* In wave generation mode, unmask bits[4:0] of LFSR */
#  define DAC_CR_MAMP_BITS5_0          (0x5 << DAC_CR_MAMP_SHIFT)     /* In wave generation mode, unmask bits[5:0] of LFSR */
#  define DAC_CR_MAMP_BITS6_0          (0x6 << DAC_CR_MAMP_SHIFT)     /* In wave generation mode, unmask bits[6:0] of LFSR */
#  define DAC_CR_MAMP_BITS7_0          (0x7 << DAC_CR_MAMP_SHIFT)     /* In wave generation mode, unmask bits[7:0] of LFSR */
#  define DAC_CR_MAMP_BITS8_0          (0x8 << DAC_CR_MAMP_SHIFT)     /* In wave generation mode, unmask bits[8:0] of LFSR */
#  define DAC_CR_MAMP_BITS9_0          (0x9 << DAC_CR_MAMP_SHIFT)     /* In wave generation mode, unmask bits[9:0] of LFSR */
#  define DAC_CR_MAMP_BITS10_0         (0xa << DAC_CR_MAMP_SHIFT)     /* In wave generation mode, unmask bits[10:0] of LFSR */
#  define DAC_CR_MAMP_BITS11_0         (0xb << DAC_CR_MAMP_SHIFT)     /* In wave generation mode, unmask bits[11:0] of LFSR */
#  define DAC_CR_MAMP_AMP1             (0x0 << DAC_CR_MAMP_SHIFT)     /* In triangle generation mode, triangle amplitude 1 */
#  define DAC_CR_MAMP_AMP3             (0x1 << DAC_CR_MAMP_SHIFT)     /* In triangle generation mode, triangle amplitude 3 */
#  define DAC_CR_MAMP_AMP7             (0x2 << DAC_CR_MAMP_SHIFT)     /* In triangle generation mode, triangle amplitude 7 */
#  define DAC_CR_MAMP_AMP15            (0x3 << DAC_CR_MAMP_SHIFT)     /* In triangle generation mode, triangle amplitude 15 */
#  define DAC_CR_MAMP_AMP31            (0x4 << DAC_CR_MAMP_SHIFT)     /* In triangle generation mode, triangle amplitude 31 */
#  define DAC_CR_MAMP_AMP63            (0x5 << DAC_CR_MAMP_SHIFT)     /* In triangle generation mode, triangle amplitude 63 */
#  define DAC_CR_MAMP_AMP127           (0x6 << DAC_CR_MAMP_SHIFT)     /* In triangle generation mode, triangle amplitude 127 */
#  define DAC_CR_MAMP_AMP255           (0x7 << DAC_CR_MAMP_SHIFT)     /* In triangle generation mode, triangle amplitude 255 */
#  define DAC_CR_MAMP_AMP511           (0x8 << DAC_CR_MAMP_SHIFT)     /* In triangle generation mode, triangle amplitude 511 */
#  define DAC_CR_MAMP_AMP1023          (0x9 << DAC_CR_MAMP_SHIFT)     /* In triangle generation mode, triangle amplitude 1023 */
#  define DAC_CR_MAMP_AMP2047          (0xa << DAC_CR_MAMP_SHIFT)     /* In triangle generation mode, triangle amplitude 2047 */
#  define DAC_CR_MAMP_AMP4095          (0xb << DAC_CR_MAMP_SHIFT)     /* In triangle generation mode, triangle amplitude 4095 */
#define DAC_CR_DMAEN                   (1 << 12)                      /* Bit 12: DAC channel DMA enable */
#define DAC_CR_DMAUDRIE                (1 << 13)                      /* Bit 13: DAC channel DMA underrun interrupt enable */
#define DAC_CR_CEN                     (1 << 14)                      /* Bit 14: DAC channel calibration enable */

/* These definitions may be used with the full, 32-bit register */

#define DAC_CR_EN1                     (1 << 0)                       /* Bit 0: DAC channel 1 enable */
#define DAC_CR_TEN1                    (1 << 1)                       /* Bit 2: DAC channel 1 trigger enable */
#define DAC_CR_TSEL1_SHIFT             (2)                            /* Bits 2-5: DAC channel 1 trigger selection */
#define DAC_CR_TSEL1_MASK              (0xf << DAC_CR_TSEL1_SHIFT)    /* Possible values for TSEL1 follow: */
#  define DAC_CR_TSEL1_SW              (0x0 << DAC_CR_TSEL1_SHIFT)    /* SWTRIG1 */
#  define DAC_CR_TSEL1_TIM8            (0x1 << DAC_CR_TSEL1_SHIFT)    /* dac_ch1_trg1 - TIM8_TRGO */
#  define DAC_CR_TSEL1_TIM7            (0x2 << DAC_CR_TSEL1_SHIFT)    /* dac_ch1_trg2 - TIM7_TRGO */
#  define DAC_CR_TSEL1_TIM15           (0x3 << DAC_CR_TSEL1_SHIFT)    /* dac_ch1_trg3 - TIM15_TRGO */
#  define DAC_CR_TSEL1_TIM2            (0x4 << DAC_CR_TSEL1_SHIFT)    /* dac_ch1_trg4 - TIM2_TRGO */
#  define DAC_CR_TSEL1_TIM4            (0x5 << DAC_CR_TSEL1_SHIFT)    /* dac_ch1_trg5 - TIM4_TRGO */
#  define DAC_CR_TSEL1_EXTI9           (0x6 << DAC_CR_TSEL1_SHIFT)    /* dac_ch1_trg6 - EXTI9 */
#  define DAC_CR_TSEL1_TIM6            (0x7 << DAC_CR_TSEL1_SHIFT)    /* dac_ch1_trg7 - TIM6_TRGO */
#  define DAC_CR_TSEL1_TIM3            (0x8 << DAC_CR_TSEL1_SHIFT)    /* dac_ch1_trg8 - TIM3_TRGO */
#  define DAC_CR_TSEL1_HRT1RTRG1       (0x9 << DAC_CR_TSEL1_SHIFT)    /* dac_ch1_trg9 -  htrim_dac_reset_trg1 */
#  define DAC_CR_TSEL1_HRT1RTRG2       (0xa << DAC_CR_TSEL1_SHIFT)    /* dac_ch1_trg10 - htrim_dac_reset_trg2 */
#  define DAC_CR_TSEL1_HRT1RTRG3       (0xb << DAC_CR_TSEL1_SHIFT)    /* dac_ch1_trg11 - htrim_dac_reset_trg3 */
#  define DAC_CR_TSEL1_HRT1RTRG4       (0xc << DAC_CR_TSEL1_SHIFT)    /* dac_ch1_trg12 - htrim_dac_reset_trg4 */
#  define DAC_CR_TSEL1_HRT1RTRG5       (0xd << DAC_CR_TSEL1_SHIFT)    /* dac_ch1_trg13 - htrim_dac_reset_trg5 */
#  define DAC_CR_TSEL1_HRT1RTRG6       (0xe << DAC_CR_TSEL1_SHIFT)    /* dac_ch1_trg14 - htrim_dac_reset_trg6 */
#  define DAC_CR_TSEL1_HRT1TRG1        (0xf << DAC_CR_TSEL1_SHIFT)    /* dac_ch1_trg15 - htrim_dac_trg1 */
#define DAC_CR_WAVE1_SHIFT             (6)                            /* Bits 6-7: DAC channel 1 noise/triangle wave generation enable */
#define DAC_CR_WAVE1_MASK              (0x3 << DAC_CR_WAVE1_SHIFT)    /* Possible values for WAVE1 follow: */
#  define DAC_CR_WAVE1_DISABLED        (0x0 << DAC_CR_WAVE1_SHIFT)    /* Wave generation disabled */
#  define DAC_CR_WAVE1_NOISE           (0x1 << DAC_CR_WAVE1_SHIFT)    /* Noise wave generation enabled */
#  define DAC_CR_WAVE1_TRIANGLE        (0x2 << DAC_CR_WAVE1_SHIFT)    /* Triangle wave generation enabled */
#  define DAC_CR_WAVE1_SAWTOOTH        (0x3 << DAC_CR_WAVE1_SHIFT)    /* Sawtooth wave generation enabled */
#define DAC_CR_MAMP1_SHIFT             (8)                            /* Bits 8-11: DAC channel 1 mask/amplitude selector */
#define DAC_CR_MAMP1_MASK              (0xf << DAC_CR_MAMP1_SHIFT)    /* Possible values for MASK1 follow: */
#  define DAC_CR_MAMP1_BIT0            (0x0 << DAC_CR_MAMP1_SHIFT)    /* In wave generation mode, unmask bit 0 of LFSR */
#  define DAC_CR_MAMP1_BITS1_0         (0x1 << DAC_CR_MAMP1_SHIFT)    /* In wave generation mode, unmask bits[1:0] of LFSR */
#  define DAC_CR_MAMP1_BITS2_0         (0x2 << DAC_CR_MAMP1_SHIFT)    /* In wave generation mode, unmask bits[2:0] of LFSR */
#  define DAC_CR_MAMP1_BITS3_0         (0x3 << DAC_CR_MAMP1_SHIFT)    /* In wave generation mode, unmask bits[3:0] of LFSR */
#  define DAC_CR_MAMP1_BITS4_0         (0x4 << DAC_CR_MAMP1_SHIFT)    /* In wave generation mode, unmask bits[4:0] of LFSR */
#  define DAC_CR_MAMP1_BITS5_0         (0x5 << DAC_CR_MAMP1_SHIFT)    /* In wave generation mode, unmask bits[5:0] of LFSR */
#  define DAC_CR_MAMP1_BITS6_0         (0x6 << DAC_CR_MAMP1_SHIFT)    /* In wave generation mode, unmask bits[6:0] of LFSR */
#  define DAC_CR_MAMP1_BITS7_0         (0x7 << DAC_CR_MAMP1_SHIFT)    /* In wave generation mode, unmask bits[7:0] of LFSR */
#  define DAC_CR_MAMP1_BITS8_0         (0x8 << DAC_CR_MAMP1_SHIFT)    /* In wave generation mode, unmask bits[8:0] of LFSR */
#  define DAC_CR_MAMP1_BITS9_0         (0x9 << DAC_CR_MAMP1_SHIFT)    /* In wave generation mode, unmask bits[9:0] of LFSR */
#  define DAC_CR_MAMP1_BITS10_0        (0xa << DAC_CR_MAMP1_SHIFT)    /* In wave generation mode, unmask bits[10:0] of LFSR */
#  define DAC_CR_MAMP1_BITS11_0        (0xb << DAC_CR_MAMP1_SHIFT)    /* In wave generation mode, unmask bits[11:0] of LFSR */
#  define DAC_CR_MAMP1_AMP1            (0x0 << DAC_CR_MAMP1_SHIFT)    /* In triangle generation mode, triangle amplitude 1 */
#  define DAC_CR_MAMP1_AMP3            (0x1 << DAC_CR_MAMP1_SHIFT)    /* In triangle generation mode, triangle amplitude 3 */
#  define DAC_CR_MAMP1_AMP7            (0x2 << DAC_CR_MAMP1_SHIFT)    /* In triangle generation mode, triangle amplitude 7 */
#  define DAC_CR_MAMP1_AMP15           (0x3 << DAC_CR_MAMP1_SHIFT)    /* In triangle generation mode, triangle amplitude 15 */
#  define DAC_CR_MAMP1_AMP31           (0x4 << DAC_CR_MAMP1_SHIFT)    /* In triangle generation mode, triangle amplitude 31 */
#  define DAC_CR_MAMP1_AMP63           (0x5 << DAC_CR_MAMP1_SHIFT)    /* In triangle generation mode, triangle amplitude 63 */
#  define DAC_CR_MAMP1_AMP127          (0x6 << DAC_CR_MAMP1_SHIFT)    /* In triangle generation mode, triangle amplitude 127 */
#  define DAC_CR_MAMP1_AMP255          (0x7 << DAC_CR_MAMP1_SHIFT)    /* In triangle generation mode, triangle amplitude 255 */
#  define DAC_CR_MAMP1_AMP511          (0x8 << DAC_CR_MAMP1_SHIFT)    /* In triangle generation mode, triangle amplitude 511 */
#  define DAC_CR_MAMP1_AMP1023         (0x9 << DAC_CR_MAMP1_SHIFT)    /* In triangle generation mode, triangle amplitude 1023 */
#  define DAC_CR_MAMP1_AMP2047         (0xa << DAC_CR_MAMP1_SHIFT)    /* In triangle generation mode, triangle amplitude 2047 */
#  define DAC_CR_MAMP1_AMP4095         (0xb << DAC_CR_MAMP1_SHIFT)    /* In triangle generation mode, triangle amplitude 4095 */
#define DAC_CR_DMAEN1                  (1 << 12)                      /* Bit 12: DAC channel 1 DMA enable */
#define DAC_CR_DMAUDRIE1               (1 << 13)                      /* Bit 13: DAC channel 1 DMA underrun interrupt enable */
#define DAC_CR_CEN1                    (1 << 14)                      /* Bit 14: DAC channel 1 calibration enable */

#define DAC_CR_EN2                     (1 << 16)                      /* Bit 16: DAC channel 2 enable */
#define DAC_CR_TEN2                    (1 << 17)                      /* Bit 17: DAC channel 2 trigger enable */
#define DAC_CR_TSEL2_SHIFT             (18)                           /* Bits 18-21: DAC channel 2 trigger selection */
#define DAC_CR_TSEL2_MASK              (0xf << DAC_CR_TSEL2_SHIFT)    /* Possible values for TSEL2 follow: */
#  define DAC_CR_TSEL2_SW              (0x0 << DAC_CR_TSEL2_SHIFT)    /* SWTRIG2 */
#  define DAC_CR_TSEL2_TIM8            (0x1 << DAC_CR_TSEL2_SHIFT)    /* dac_ch2_trg1 - TIM8_TRGO */
#  define DAC_CR_TSEL2_TIM7            (0x2 << DAC_CR_TSEL2_SHIFT)    /* dac_ch2_trg2 - TIM7_TRGO */
#  define DAC_CR_TSEL2_TIM15           (0x3 << DAC_CR_TSEL2_SHIFT)    /* dac_ch2_trg3 - TIM15_TRGO */
#  define DAC_CR_TSEL2_TIM2            (0x4 << DAC_CR_TSEL2_SHIFT)    /* dac_ch2_trg4 - TIM2_TRGO */
#  define DAC_CR_TSEL2_TIM4            (0x5 << DAC_CR_TSEL2_SHIFT)    /* dac_ch2_trg5 - TIM4_TRGO */
#  define DAC_CR_TSEL2_EXTI9           (0x6 << DAC_CR_TSEL2_SHIFT)    /* dac_ch2_trg6 - EXTI9 */
#  define DAC_CR_TSEL2_TIM6            (0x7 << DAC_CR_TSEL2_SHIFT)    /* dac_ch2_trg7 - TIM6_TRGO */
#  define DAC_CR_TSEL2_TIM3            (0x8 << DAC_CR_TSEL2_SHIFT)    /* dac_ch2_trg8 - TIM3_TRGO */
#  define DAC_CR_TSEL2_HRT1RTRG1       (0x9 << DAC_CR_TSEL2_SHIFT)    /* dac_ch2_trg9 -  htrim_dac_reset_trg1 */
#  define DAC_CR_TSEL2_HRT1RTRG2       (0xa << DAC_CR_TSEL2_SHIFT)    /* dac_ch2_trg10 - htrim_dac_reset_trg2 */
#  define DAC_CR_TSEL2_HRT1RTRG3       (0xb << DAC_CR_TSEL2_SHIFT)    /* dac_ch2_trg11 - htrim_dac_reset_trg3 */
#  define DAC_CR_TSEL2_HRT1RTRG4       (0xc << DAC_CR_TSEL2_SHIFT)    /* dac_ch2_trg12 - htrim_dac_reset_trg4 */
#  define DAC_CR_TSEL2_HRT1RTRG5       (0xd << DAC_CR_TSEL2_SHIFT)    /* dac_ch2_trg13 - htrim_dac_reset_trg5 */
#  define DAC_CR_TSEL2_HRT1RTRG6       (0xe << DAC_CR_TSEL2_SHIFT)    /* dac_ch2_trg14 - htrim_dac_reset_trg6 */
#  define DAC_CR_TSEL2_HRT1TRG1        (0xf << DAC_CR_TSEL2_SHIFT)    /* dac_ch2_trg15 - htrim_dac_trg1 */
#define DAC_CR_WAVE2_SHIFT             (22)                           /* Bits 22-23: DAC channel 2 noise/triangle wave generation enable */
#define DAC_CR_WAVE2_MASK              (0x3 << DAC_CR_WAVE2_SHIFT)    /* Possible values for WAVE2 follow: */
#  define DAC_CR_WAVE2_DISABLED        (0x0 << DAC_CR_WAVE2_SHIFT)    /* Wave generation disabled */
#  define DAC_CR_WAVE2_NOISE           (0x1 << DAC_CR_WAVE2_SHIFT)    /* Noise wave generation enabled */
#  define DAC_CR_WAVE2_TRIANGLE        (0x2 << DAC_CR_WAVE2_SHIFT)    /* Triangle wave generation enabled */
#  define DAC_CR_WAVE2_SAWTOOTH        (0x3 << DAC_CR_WAVE2_SHIFT)    /* Sawtooth wave generation enabled */
#define DAC_CR_MAMP2_SHIFT             (24)                           /* Bits 24-27: DAC channel 2 mask/amplitude selector */
#define DAC_CR_MAMP2_MASK              (0xf << DAC_CR_MAMP2_SHIFT)    /* Possible values for MASK2 follow: */
#  define DAC_CR_MAMP2_BIT0            (0x0 << DAC_CR_MAMP2_SHIFT)    /* In wave generation mode, unmask bit 0 of LFSR */
#  define DAC_CR_MAMP2_BITS1_0         (0x1 << DAC_CR_MAMP2_SHIFT)    /* In wave generation mode, unmask bits[1:0] of LFSR */
#  define DAC_CR_MAMP2_BITS2_0         (0x2 << DAC_CR_MAMP2_SHIFT)    /* In wave generation mode, unmask bits[2:0] of LFSR */
#  define DAC_CR_MAMP2_BITS3_0         (0x3 << DAC_CR_MAMP2_SHIFT)    /* In wave generation mode, unmask bits[3:0] of LFSR */
#  define DAC_CR_MAMP2_BITS4_0         (0x4 << DAC_CR_MAMP2_SHIFT)    /* In wave generation mode, unmask bits[4:0] of LFSR */
#  define DAC_CR_MAMP2_BITS5_0         (0x5 << DAC_CR_MAMP2_SHIFT)    /* In wave generation mode, unmask bits[5:0] of LFSR */
#  define DAC_CR_MAMP2_BITS6_0         (0x6 << DAC_CR_MAMP2_SHIFT)    /* In wave generation mode, unmask bits[6:0] of LFSR */
#  define DAC_CR_MAMP2_BITS7_0         (0x7 << DAC_CR_MAMP2_SHIFT)    /* In wave generation mode, unmask bits[7:0] of LFSR */
#  define DAC_CR_MAMP2_BITS8_0         (0x8 << DAC_CR_MAMP2_SHIFT)    /* In wave generation mode, unmask bits[8:0] of LFSR */
#  define DAC_CR_MAMP2_BITS9_0         (0x9 << DAC_CR_MAMP2_SHIFT)    /* In wave generation mode, unmask bits[9:0] of LFSR */
#  define DAC_CR_MAMP2_BITS10_0        (0xa << DAC_CR_MAMP2_SHIFT)    /* In wave generation mode, unmask bits[10:0] of LFSR */
#  define DAC_CR_MAMP2_BITS11_0        (0xb << DAC_CR_MAMP2_SHIFT)    /* In wave generation mode, unmask bits[11:0] of LFSR */
#  define DAC_CR_MAMP2_AMP1            (0x0 << DAC_CR_MAMP2_SHIFT)    /* In triangle generation mode, triangle amplitude 1 */
#  define DAC_CR_MAMP2_AMP3            (0x1 << DAC_CR_MAMP2_SHIFT)    /* In triangle generation mode, triangle amplitude 3 */
#  define DAC_CR_MAMP2_AMP7            (0x2 << DAC_CR_MAMP2_SHIFT)    /* In triangle generation mode, triangle amplitude 7 */
#  define DAC_CR_MAMP2_AMP15           (0x3 << DAC_CR_MAMP2_SHIFT)    /* In triangle generation mode, triangle amplitude 15 */
#  define DAC_CR_MAMP2_AMP31           (0x4 << DAC_CR_MAMP2_SHIFT)    /* In triangle generation mode, triangle amplitude 31 */
#  define DAC_CR_MAMP2_AMP63           (0x5 << DAC_CR_MAMP2_SHIFT)    /* In triangle generation mode, triangle amplitude 63 */
#  define DAC_CR_MAMP2_AMP127          (0x6 << DAC_CR_MAMP2_SHIFT)    /* In triangle generation mode, triangle amplitude 127 */
#  define DAC_CR_MAMP2_AMP255          (0x7 << DAC_CR_MAMP2_SHIFT)    /* In triangle generation mode, triangle amplitude 255 */
#  define DAC_CR_MAMP2_AMP511          (0x8 << DAC_CR_MAMP2_SHIFT)    /* In triangle generation mode, triangle amplitude 511 */
#  define DAC_CR_MAMP2_AMP1023         (0x9 << DAC_CR_MAMP2_SHIFT)    /* In triangle generation mode, triangle amplitude 1023 */
#  define DAC_CR_MAMP2_AMP2047         (0xa << DAC_CR_MAMP2_SHIFT)    /* In triangle generation mode, triangle amplitude 2047 */
#  define DAC_CR_MAMP2_AMP4095         (0xb << DAC_CR_MAMP2_SHIFT)    /* In triangle generation mode, triangle amplitude 4095 */
#define DAC_CR_DMAEN2                  (1 << 28)                      /* Bit 28: DAC channel 2 DMA enable */
#define DAC_CR_DMAUDRIE2               (1 << 29)                      /* Bit 29: DAC channel 2 DMA underrun interrupt enable */
#define DAC_CR_CEN2                    (1 << 30)                      /* Bit 30: DAC channel 2 calibration enable */

/* DAC software trigger register (SWTRGR) */

#define DAC_SWTRIGR_SWTRIG(n)          (1 << ((n) - 1))               /* Software trigger */
#define DAC_SWTRIGR_SWTRIG1            (1 << 0)                       /* Bit 0: DAC channel 1 software trigger */
#define DAC_SWTRIGR_SWTRIG2            (1 << 1)                       /* Bit 1: DAC channel 2 software trigger */

#define DAC_SWTRIGR_SWTRIGB(n)         (1 << ((n) - 1))               /* Software trigger B for sawtooth generation */
#define DAC_SWTRIGR_SWTRIGB1           (1 << 0)                       /* Bit 16: DAC channel 1 software trigger B for sawtooth increment */
#define DAC_SWTRIGR_SWTRIGB2           (1 << 1)                       /* Bit 17: DAC channel 2 software trigger B for sawtooth increment */

/* DAC channels 1/2 12-bit right-aligned data holding register
 * (DHR12R1, DHR12R2)
 */

#define DAC_DHR12R_MASK_SHIFT          (0)                            /* DAC channel 1/2 right-aligned data */
#define DAC_DHR12R_MASK                (0xfff << DAC_DHR12R_MASK_SHIFT)
#define DAC_DHR12R_MASKB_SHIFT         (16)                           /* DAC channel 1/2 right-aligned data B for Double Data Mode */
#define DAC_DHR12R_MASKB               (0xfff << DAC_DHR12R_MASKB_SHIFT)

/* DAC channel 1/2 12-bit left aligned data holding register
 * (DHR12L1, DHR12L2)
 */

#define DAC_DHR12L_MASK_SHIFT          (4)                            /* DAC channel 1/2 left-aligned data */
#define DAC_DHR12L_MASK                (0xfff << DAC_DHR12L_MASK_SHIFT)
#define DAC_DHR12L_MASKB_SHIFT         (20)                           /* DAC channel 1/2 left-aligned data B for Double Data Mode */
#define DAC_DHR12L_MASKB               (0xfff << DAC_DHR12L_MASKB_SHIFT)

/* DAC channel 1/2 8-bit right aligned data holding register
 * (DHR8R1, DHR8R2)
 */

#define DAC_DHR8R_MASK_SHIFT           (0)                            /* DAC channel 1/2 right-aligned data */
#define DAC_DHR8R_MASK                 (0xff << DAC_DHR8R_MASK_SHIFT)
#define DAC_DHR8R_MASKB_SHIFT          (8)                            /* DAC channel 1/2 right-aligned data B for Double Data Mode */
#define DAC_DHR8R_MASKB                (0xff << DAC_DHR8R_MASKB_SHIFT)

/* Dual DAC 12-bit right-aligned data holding register (DHR12RD) */

#define DAC_DHR12RD_DACC_SHIFT(n)      (((n) - 1) << 4)
#define DAC_DHR12RD_DACC_MASK(n)       (0xfff << DAC_DHR12RD_DACC_SHIFT(n))

#define DAC_DHR12RD_DACC1_SHIFT        (0)                            /* Bits 0-11: DAC channel 1 12-bit right-aligned data */
#define DAC_DHR12RD_DACC1_MASK         (0xfff << DAC_DHR12RD_DACC1_SHIFT)
#define DAC_DHR12RD_DACC2_SHIFT        (16)                           /* Bits 16-27: DAC channel 2 12-bit right-aligned data */
#define DAC_DHR12RD_DACC2_MASK         (0xfff << DAC_DHR12RD_DACC2_SHIFT)

/* Dual DAC 12-bit left-aligned data holding register (DHR12LD) */

#define DAC_DHR12LD_DACC_SHIFT(n)      ((((n) - 1) << 4) + 4)
#define DAC_DHR12LD_DACC_MASK(n)       (0xfff << DAC_DHR12LD_DACC_SHIFT(n))

#define DAC_DHR12LD_DACC1_SHIFT        (4)                            /* Bits 4-15: DAC channel 1 12-bit left-aligned data */
#define DAC_DHR12LD_DACC1_MASK         (0xfff << DAC_DHR12LD_DACC1_SHIFT)
#define DAC_DHR12LD_DACC2_SHIFT        (20)                           /* Bits 20-31: DAC channel 2 12-bit left-aligned data */
#define DAC_DHR12LD_DACC2_MASK         (0xfff << DAC_DHR12LD_DACC2_SHIFT)

/* DUAL DAC 8-bit right aligned data holding register (DHR8RD) */

#define DAC_DHR8RD_DACC_SHIFT(n)       (((n) - 1) << 3)
#define DAC_DHR8RD_DACC_MASK(n)        (0xff << DAC_DHR8RD_DACC_SHIFT(n))

#define DAC_DHR8RD_DACC1_SHIFT         (0)                            /* Bits 0-7: DAC channel 1 8-bit right-aligned data */
#define DAC_DHR8RD_DACC1_MASK          (0xff << DAC_DHR8RD_DACC1_SHIFT)
#define DAC_DHR8RD_DACC2_SHIFT         (8)                            /* Bits 8-15: DAC channel 2 8-bit right-aligned data */
#define DAC_DHR8RD_DACC2_MASK          (0xff << DAC_DHR8RD_DACC2_SHIFT)

/* DAC channel 1/2 data output register (DOR1, DOR2) */

#define DAC_DOR_MASK_SHIFT             (0)                            /* DAC channel 1/2 data output */
#define DAC_DOR_MASK                   (0xfff << DAC_DOR_MASK_SHIFT)
#define DAC_DOR_MASKB_SHIFT            (16)                           /* DAC channel 1/2 data output B for Double Data Mode */
#define DAC_DOR_MASKB                  (0xfff << DAC_DOR_MASKB_SHIFT)

/* DAC status register (SR) */

#define DAC_SR_DACRDY(n)               (1 << ((((n) - 1) << 4) + 11))
#define DAC_SR_DAC1RDY                 (1 << 11)                      /* Bit 13: DAC channel 1 ready status bit */
#define DAC_SR_DAC2RDY                 (1 << 27)                      /* Bit 27: DAC channel 2 ready status bit */

#define DAC_SR_DORSTAT(n)              (1 << ((((n) - 1) << 4) + 12))
#define DAC_SR_DORSTAT1                (1 << 12)                      /* Bit 13: DAC channel 1 output register status bit */
#define DAC_SR_DORSTAT2                (1 << 28)                      /* Bit 29: DAC channel 2 output register status bit */

#define DAC_SR_DMAUDR(n)               (1 << ((((n) - 1) << 4) + 13))
#define DAC_SR_DMAUDR1                 (1 << 13)                      /* Bit 13: DAC channel 1 DMA underrun flag */
#define DAC_SR_DMAUDR2                 (1 << 29)                      /* Bit 29: DAC channel 2 DMA underrun flag */

#define DAC_SR_CAL_FLAG(n)             (1 << ((((n) - 1) << 4) + 14))
#define DAC_SR_CAL_FLAG1               (1 << 14)                      /* Bit 13: DAC channel 1 calibration offset status */
#define DAC_SR_CAL_FLAG2               (1 << 30)                      /* Bit 29: DAC channel 2 calibration offset status */

#define DAC_SR_BWST(n)                 (1 << ((((n) - 1) << 4) + 15))
#define DAC_SR_BWST1                   (1 << 15)                      /* Bit 13: DAC channel 1 busy writing sample time flag */
#define DAC_SR_BWST2                   (1 << 31)                      /* Bit 29: DAC channel 2 busy writing sample time flag */

/* DAC calibration control register (CCR) */

#define DAC_CCR_OTRIM_SHIFT(n)         (((n) - 1) << 4)
#define DAC_CCR_OTRIM_MASK(n)          (0x1f << DAC_CCR_OTRIM_SHIFT(n))

#define DAC_CCR_OTRIM1_SHIFT           (0)                            /* DAC channel 1 offset trimming value */
#define DAC_CCR_OTRIM1_MASK            (0x1f << DAC_CCR_OTRIM1_SHIFT)
#define DAC_CCR_OTRIM2_SHIFT           (16)                           /* DAC channel 2 offset trimming value */
#define DAC_CCR_OTRIM2_MASK            (0x1f << DAC_CCR_OTRIM2_SHIFT)

/* DAC mode control register (MCR) */

#define DAC_MCR_MODE_SHIFT(n)          (((n) - 1) << 4)               /* DAC channel 1/2 mode */
#define DAC_MCR_MODE_MASK(n)           (0x7 << DAC_MCR_MODE_SHIFT(n))
#  define DAC_MCR_MODE_0(n)            (0x0 << DAC_MCR_MODE_SHIFT(n)) /* Normal mode, connect to external pin with buffer enabled */
#  define DAC_MCR_MODE_1(n)            (0x1 << DAC_MCR_MODE_SHIFT(n)) /* Normal mode, connect to external pin and to on-chip peripherals with buffer enabled */
#  define DAC_MCR_MODE_2(n)            (0x2 << DAC_MCR_MODE_SHIFT(n)) /* Normal mode, connect to external pin with buffer disabled */
#  define DAC_MCR_MODE_3(n)            (0x3 << DAC_MCR_MODE_SHIFT(n)) /* Normal mode, connect to on-chip peripherals with buffer disabled */
#  define DAC_MCR_MODE_4(n)            (0x4 << DAC_MCR_MODE_SHIFT(n)) /* Sample & hold mode, connect to external pin with buffer enabled */
#  define DAC_MCR_MODE_5(n)            (0x5 << DAC_MCR_MODE_SHIFT(n)) /* Sample & hold mode, connect to external pin and to on-chip peripherals with buffer enabled */
#  define DAC_MCR_MODE_6(n)            (0x6 << DAC_MCR_MODE_SHIFT(n)) /* Sample & hold mode, connect to external pin and to on-chip peripherals with buffer disabled */
#  define DAC_MCR_MODE_7(n)            (0x7 << DAC_MCR_MODE_SHIFT(n)) /* Sample & hold mode, connect to on-chip peripherals with buffer disabled */

#define DAC_MCR_MODE1_SHIFT            (0)                            /* DAC channel 1 mode */
#define DAC_MCR_MODE1_MASK             (0x7 << DAC_MCR_MODE1_SHIFT)
#  define DAC_MCR_MODE1_0              (0x0 << DAC_MCR_MODE1_SHIFT)   /* Normal mode, connect to external pin with buffer enabled */
#  define DAC_MCR_MODE1_1              (0x1 << DAC_MCR_MODE1_SHIFT)   /* Normal mode, connect to external pin and to on-chip peripherals with buffer enabled */
#  define DAC_MCR_MODE1_2              (0x2 << DAC_MCR_MODE1_SHIFT)   /* Normal mode, connect to external pin with buffer disabled */
#  define DAC_MCR_MODE1_3              (0x3 << DAC_MCR_MODE1_SHIFT)   /* Normal mode, connect to on-chip peripherals with buffer disabled */
#  define DAC_MCR_MODE1_4              (0x4 << DAC_MCR_MODE1_SHIFT)   /* Sample & hold mode, connect to external pin with buffer enabled */
#  define DAC_MCR_MODE1_5              (0x5 << DAC_MCR_MODE1_SHIFT)   /* Sample & hold mode, connect to external pin and to on-chip peripherals with buffer enabled */
#  define DAC_MCR_MODE1_6              (0x6 << DAC_MCR_MODE1_SHIFT)   /* Sample & hold mode, connect to external pin and to on-chip peripherals with buffer disabled */
#  define DAC_MCR_MODE1_7              (0x7 << DAC_MCR_MODE1_SHIFT)   /* Sample & hold mode, connect to on-chip peripherals with buffer disabled */

#define DAC_MCR_MODE2_SHIFT            (16)                           /* DAC channel 2 mode */
#define DAC_MCR_MODE2_MASK             (0x7 << DAC_MCR_MODE2_SHIFT)
#  define DAC_MCR_MODE2_0              (0x0 << DAC_MCR_MODE2_SHIFT)   /* Normal mode, connect to external pin with buffer enabled */
#  define DAC_MCR_MODE2_1              (0x1 << DAC_MCR_MODE2_SHIFT)   /* Normal mode, connect to external pin and to on-chip peripherals with buffer enabled */
#  define DAC_MCR_MODE2_2              (0x2 << DAC_MCR_MODE2_SHIFT)   /* Normal mode, connect to external pin with buffer disabled */
#  define DAC_MCR_MODE2_3              (0x3 << DAC_MCR_MODE2_SHIFT)   /* Normal mode, connect to on-chip peripherals with buffer disabled */
#  define DAC_MCR_MODE2_4              (0x4 << DAC_MCR_MODE2_SHIFT)   /* Sample & hold mode, connect to external pin with buffer enabled */
#  define DAC_MCR_MODE2_5              (0x5 << DAC_MCR_MODE2_SHIFT)   /* Sample & hold mode, connect to external pin and to on-chip peripherals with buffer enabled */
#  define DAC_MCR_MODE2_6              (0x6 << DAC_MCR_MODE2_SHIFT)   /* Sample & hold mode, connect to external pin and to on-chip peripherals with buffer disabled */
#  define DAC_MCR_MODE2_7              (0x7 << DAC_MCR_MODE2_SHIFT)   /* Sample & hold mode, connect to on-chip peripherals with buffer disabled */

#define DAC_MCR_DMADOUBLE(n)           (1 << ((((n) - 1) << 4) + 8))  /* DAC Channel 1/2 DMA double data mode */
#define DAC_MCR_DMADOUBLE1             (1 << 8)                       /* DAC Channel 1 DMA double data mode */
#define DAC_MCR_DMADOUBLE2             (1 << 24)                      /* DAC Channel 2 DMA double data mode */

#define DAC_MCR_SINFORMAT(n)           (1 << ((((n) - 1) << 4) + 9))  /* DAC Channel 1/2 enable signed format */
#define DAC_MCR_SINFORMAT1             (1 << 9)                       /* DAC Channel 1 enable signed format */
#define DAC_MCR_SINFORMAT2             (1 << 25)                      /* DAC Channel 2 enable signed format */

#define DAC_MCR_HFSEL_SHIFT            (14)                           /* High-frequency interface mode selection */
#define DAC_MCR_HFSEL_MASK             (0x3 << DAC_MCR_HFSEL_SHIFT)
#  define DAC_MCR_HFSEL_DISABLED       (0x0 << DAC_MCR_HFSEL_SHIFT)   /* High-frequency disabled */
#  define DAC_MCR_HFSEL_AHB_80MHz      (0x1 << DAC_MCR_HFSEL_SHIFT)   /* High-frequency for AHB > 80 MHz */
#  define DAC_MCR_HFSEL_AHB_160MHz     (0x2 << DAC_MCR_HFSEL_SHIFT)   /* High-frequency for AHB > 160 MHz */
#  define DAC_MCR_HFSEL_RESERVED       (0x3 << DAC_MCR_HFSEL_SHIFT)   /* Reserved */

/* DAC channel 1/2 sample and hold sample time register (SHSR1, SHSR2) */

#define DAC_SHSR_TSAMPLE_SHIFT         (0)                            /* DAC channel 1/2 Sample phase time = (TSAMPLE + 1) * LSI/LSE clock period */
#define DAC_SHSR_TSAMPLE_MASK          (0x3ff << DAC_SHSR1_TSAMPLE1_SHIFT)

/* DAC sample and hold time register (SHHR) */

#define DAC_SHHR_THOLD_SHIFT(n)        (((n) - 1) << 4)               /* DAC channel 1/2 Hold time = THOLD * LSI/LSE clock period */
#define DAC_SHHR_THOLD_MASK(n)         (0x3ff << DAC_SHHR_THOLD_SHIFT(n))

#define DAC_SHHR_THOLD1_SHIFT          (0)                            /* DAC channel 1 Hold time */
#define DAC_SHHR_THOLD1_MASK           (0x3ff << DAC_SHHR_THOLD1_SHIFT)
#define DAC_SHHR_THOLD2_SHIFT          (16)                           /* DAC channel 2 Hold time */
#define DAC_SHHR_THOLD2_MASK           (0x3ff << DAC_SHHR_THOLD2_SHIFT)

/* DAC sample and hold refresh time register (SHRR) */

#define DAC_SHHR_TREFRESH_SHIFT(n)     (((n) - 1) << 4)               /* DAC channel 1/2 Refresh time = TREFRESH * LSI/LSE clock period */
#define DAC_SHHR_TREFRESH_MASK(n)      (0xff << DAC_SHHR_THOLD_SHIFT(n))

#define DAC_SHRR_TREFRESH1_SHIFT       (0)                            /* DAC channel 1 Refresh time */
#define DAC_SHRR_TREFRESH1_MASK        (0xff << DAC_SHRR_TREFRESH1_SHIFT)
#define DAC_SHRR_TREFRESH2_SHIFT       (16)                           /* DAC channel 2 Refresh time */
#define DAC_SHRR_TREFRESH2_MASK        (0xff << DAC_SHRR_TREFRESH2_SHIFT)

/* DAC channel 1/2 sawtooth register register (STR1, STR2) */

#define DAC_STR_STRSTDATA_SHIFT        (0)                            /* DAC channel 1 Sawtooth starting value */
#define DAC_STR_STRSTDATA_MASK         (0xfff << DAC_STR1_STRSTDATA1_SHIFT)
#define DAC_STR_STDIR                  (1 << 12)                      /* DAC channel 1 Sawtooth direction setting */
#define DAC_STR_STINCDATA_SHIFT        (16)                           /* DAC channel 1 Sawtooth increment value (12.4 bit format) */
#define DAC_STR_STINCDATA_MASK         (0xffff << DAC_STR_STINCDATA_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32GXXXXX_DAC_H */
