/****************************************************************************
 * arch/risc-v/src/gd32vw55x/hardware/gd32vw55x_adc.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_ADC_H
#define __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32vw55x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The GD32VW55x has a single 12-bit successive approximation ADC with 11
 * external/internal channels (0..10).  Channel 9 is the temperature sensor
 * and channel 10 is the internal reference voltage; both are enabled with
 * the TSVREN bit of ADC_CCTL.  The conversion resolution is selectable
 * between 12, 10, 8 and 6 bits.
 *
 * The routine (regular) sequence holds up to 9 conversions: ranks 0..5 in
 * ADC_RSQ2 and ranks 6..8 in ADC_RSQ1.  The inserted sequence holds up to
 * 4 conversions.
 */

#define GD32VW55X_ADC_NCHANNELS      11  /* Channels 0..10 */
#define GD32VW55X_ADC_MAX_SAMPLES    9   /* Routine sequence length */

/* Register offsets *********************************************************/

#define GD32VW55X_ADC_STAT_OFFSET       0x0000  /* Status */
#define GD32VW55X_ADC_CTL0_OFFSET       0x0004  /* Control 0 */
#define GD32VW55X_ADC_CTL1_OFFSET       0x0008  /* Control 1 */
#define GD32VW55X_ADC_SAMPT0_OFFSET     0x000c  /* Sampling time 0 */
#define GD32VW55X_ADC_SAMPT1_OFFSET     0x0010  /* Sampling time 1 */
#define GD32VW55X_ADC_IOFF0_OFFSET      0x0014  /* Inserted data offset 0 */
#define GD32VW55X_ADC_IOFF1_OFFSET      0x0018  /* Inserted data offset 1 */
#define GD32VW55X_ADC_IOFF2_OFFSET      0x001c  /* Inserted data offset 2 */
#define GD32VW55X_ADC_IOFF3_OFFSET      0x0020  /* Inserted data offset 3 */
#define GD32VW55X_ADC_WDHT_OFFSET       0x0024  /* Watchdog high threshold */
#define GD32VW55X_ADC_WDLT_OFFSET       0x0028  /* Watchdog low threshold */
#define GD32VW55X_ADC_RSQ0_OFFSET       0x002c  /* Routine sequence 0 */
#define GD32VW55X_ADC_RSQ1_OFFSET       0x0030  /* Routine sequence 1 */
#define GD32VW55X_ADC_RSQ2_OFFSET       0x0034  /* Routine sequence 2 */
#define GD32VW55X_ADC_ISQ_OFFSET        0x0038  /* Inserted sequence */
#define GD32VW55X_ADC_IDATA0_OFFSET     0x003c  /* Inserted data 0 */
#define GD32VW55X_ADC_IDATA1_OFFSET     0x0040  /* Inserted data 1 */
#define GD32VW55X_ADC_IDATA2_OFFSET     0x0044  /* Inserted data 2 */
#define GD32VW55X_ADC_IDATA3_OFFSET     0x0048  /* Inserted data 3 */
#define GD32VW55X_ADC_RDATA_OFFSET      0x004c  /* Routine data */
#define GD32VW55X_ADC_OVSAMPCTL_OFFSET  0x0080  /* Oversampling control */
#define GD32VW55X_ADC_CCTL_OFFSET       0x0304  /* Common control */

/* Register addresses *******************************************************/

#define GD32VW55X_ADC_STAT \
  (GD32VW55X_ADC_BASE + GD32VW55X_ADC_STAT_OFFSET)
#define GD32VW55X_ADC_CTL0 \
  (GD32VW55X_ADC_BASE + GD32VW55X_ADC_CTL0_OFFSET)
#define GD32VW55X_ADC_CTL1 \
  (GD32VW55X_ADC_BASE + GD32VW55X_ADC_CTL1_OFFSET)
#define GD32VW55X_ADC_SAMPT0 \
  (GD32VW55X_ADC_BASE + GD32VW55X_ADC_SAMPT0_OFFSET)
#define GD32VW55X_ADC_SAMPT1 \
  (GD32VW55X_ADC_BASE + GD32VW55X_ADC_SAMPT1_OFFSET)
#define GD32VW55X_ADC_WDHT \
  (GD32VW55X_ADC_BASE + GD32VW55X_ADC_WDHT_OFFSET)
#define GD32VW55X_ADC_WDLT \
  (GD32VW55X_ADC_BASE + GD32VW55X_ADC_WDLT_OFFSET)
#define GD32VW55X_ADC_RSQ0 \
  (GD32VW55X_ADC_BASE + GD32VW55X_ADC_RSQ0_OFFSET)
#define GD32VW55X_ADC_RSQ1 \
  (GD32VW55X_ADC_BASE + GD32VW55X_ADC_RSQ1_OFFSET)
#define GD32VW55X_ADC_RSQ2 \
  (GD32VW55X_ADC_BASE + GD32VW55X_ADC_RSQ2_OFFSET)
#define GD32VW55X_ADC_ISQ \
  (GD32VW55X_ADC_BASE + GD32VW55X_ADC_ISQ_OFFSET)
#define GD32VW55X_ADC_RDATA \
  (GD32VW55X_ADC_BASE + GD32VW55X_ADC_RDATA_OFFSET)
#define GD32VW55X_ADC_OVSAMPCTL \
  (GD32VW55X_ADC_BASE + GD32VW55X_ADC_OVSAMPCTL_OFFSET)
#define GD32VW55X_ADC_CCTL \
  (GD32VW55X_ADC_BASE + GD32VW55X_ADC_CCTL_OFFSET)

/* Register bit definitions *************************************************/

/* Status register (STAT) */

#define ADC_STAT_WDE                 (1 << 0)  /* Analog watchdog event */
#define ADC_STAT_EOC                 (1 << 1)  /* End of conversion */
#define ADC_STAT_EOIC                (1 << 2)  /* Inserted end of conv. */
#define ADC_STAT_STIC                (1 << 3)  /* Inserted start flag */
#define ADC_STAT_STRC                (1 << 4)  /* Routine start flag */
#define ADC_STAT_ROVF                (1 << 5)  /* Routine data overflow */

/* Control register 0 (CTL0) */

#define ADC_CTL0_WDCHSEL_SHIFT       (0)       /* Watchdog channel */
#define ADC_CTL0_WDCHSEL_MASK        (31 << ADC_CTL0_WDCHSEL_SHIFT)
#define ADC_CTL0_EOCIE               (1 << 5)  /* EOC interrupt enable */
#define ADC_CTL0_WDEIE               (1 << 6)  /* Watchdog int enable */
#define ADC_CTL0_EOICIE              (1 << 7)  /* Inserted EOC int enable */
#define ADC_CTL0_SM                  (1 << 8)  /* Scan mode */
#define ADC_CTL0_WDSC                (1 << 9)  /* Watchdog single channel */
#define ADC_CTL0_ICA                 (1 << 10) /* Inserted auto conversion */
#define ADC_CTL0_DISRC               (1 << 11) /* Routine discontinuous */
#define ADC_CTL0_DISIC               (1 << 12) /* Inserted discontinuous */
#define ADC_CTL0_DISNUM_SHIFT        (13)      /* Discontinuous count */
#define ADC_CTL0_DISNUM_MASK         (7 << ADC_CTL0_DISNUM_SHIFT)
#define ADC_CTL0_IWDEN               (1 << 22) /* Inserted watchdog enable */
#define ADC_CTL0_RWDEN               (1 << 23) /* Routine watchdog enable */
#define ADC_CTL0_DRES_SHIFT          (24)      /* Data resolution */
#define ADC_CTL0_DRES_MASK           (3 << ADC_CTL0_DRES_SHIFT)
#  define ADC_CTL0_DRES_12B          (0 << ADC_CTL0_DRES_SHIFT)
#  define ADC_CTL0_DRES_10B          (1 << ADC_CTL0_DRES_SHIFT)
#  define ADC_CTL0_DRES_8B           (2 << ADC_CTL0_DRES_SHIFT)
#  define ADC_CTL0_DRES_6B           (3 << ADC_CTL0_DRES_SHIFT)
#define ADC_CTL0_ROVFIE              (1 << 26) /* Overflow int enable */

/* Control register 1 (CTL1) */

#define ADC_CTL1_ADCON               (1 << 0)  /* ADC on */
#define ADC_CTL1_CTN                 (1 << 1)  /* Continuous conversion */
#define ADC_CTL1_DMA                 (1 << 8)  /* DMA request enable */
#define ADC_CTL1_DDM                 (1 << 9)  /* DMA disable mode */
#define ADC_CTL1_EOCM                (1 << 10) /* End of conversion mode */
#define ADC_CTL1_DAL                 (1 << 11) /* Data alignment: left */
#define ADC_CTL1_ETSIC_SHIFT         (16)      /* Inserted trigger source */
#define ADC_CTL1_ETSIC_MASK          (15 << ADC_CTL1_ETSIC_SHIFT)
#define ADC_CTL1_ETMIC_SHIFT         (20)      /* Inserted trigger mode */
#define ADC_CTL1_ETMIC_MASK          (3 << ADC_CTL1_ETMIC_SHIFT)
#define ADC_CTL1_SWICST              (1 << 22) /* Inserted software start */
#define ADC_CTL1_ETSRC_SHIFT         (24)      /* Routine trigger source */
#define ADC_CTL1_ETSRC_MASK          (15 << ADC_CTL1_ETSRC_SHIFT)
#define ADC_CTL1_ETMRC_SHIFT         (28)      /* Routine trigger mode */
#define ADC_CTL1_ETMRC_MASK          (3 << ADC_CTL1_ETMRC_SHIFT)
#  define ADC_CTL1_ETMRC_DISABLE     (0 << ADC_CTL1_ETMRC_SHIFT)
#  define ADC_CTL1_ETMRC_RISING      (1 << ADC_CTL1_ETMRC_SHIFT)
#  define ADC_CTL1_ETMRC_FALLING     (2 << ADC_CTL1_ETMRC_SHIFT)
#  define ADC_CTL1_ETMRC_BOTH        (3 << ADC_CTL1_ETMRC_SHIFT)
#define ADC_CTL1_SWRCST              (1 << 30) /* Routine software start */

/* Routine channel external trigger sources (ETSRC field) */

#define ADC_ETSRC_T0_CH0             0   /* TIMER0 CH0 */
#define ADC_ETSRC_T0_CH1             1   /* TIMER0 CH1 */
#define ADC_ETSRC_T0_CH2             2   /* TIMER0 CH2 */
#define ADC_ETSRC_T1_CH1             3   /* TIMER1 CH1 */
#define ADC_ETSRC_T1_CH2             4   /* TIMER1 CH2 */
#define ADC_ETSRC_T1_CH3             5   /* TIMER1 CH3 */
#define ADC_ETSRC_T1_TRGO            6   /* TIMER1 TRGO */
#define ADC_ETSRC_T2_CH0             7   /* TIMER2 CH0 */
#define ADC_ETSRC_T2_TRGO            8   /* TIMER2 TRGO */
#define ADC_ETSRC_T2_CH2             9   /* TIMER2 CH2 */
#define ADC_ETSRC_T15_CH0            10  /* TIMER15 CH0 */
#define ADC_ETSRC_T16_CH0            11  /* TIMER16 CH0 */
#define ADC_ETSRC_T5_TRGO            14  /* TIMER5 TRGO */
#define ADC_ETSRC_EXTI11             15  /* EXTI line 11 */

/* Sampling time registers (SAMPT0, SAMPT1).
 *
 * Each channel owns a 4-bit field.  SAMPT1 holds channels 0..7 and SAMPT0
 * holds channels 8..10.
 */

#define ADC_SAMPT_LENGTH             4
#define ADC_SAMPT_MASK               15

#define ADC_SAMPLETIME_1POINT5       0   /* 1.5 sampling cycles */
#define ADC_SAMPLETIME_2POINT5       1   /* 2.5 sampling cycles */
#define ADC_SAMPLETIME_14POINT5      2   /* 14.5 sampling cycles */
#define ADC_SAMPLETIME_27POINT5      3   /* 27.5 sampling cycles */
#define ADC_SAMPLETIME_55POINT5      4   /* 55.5 sampling cycles */
#define ADC_SAMPLETIME_83POINT5      5   /* 83.5 sampling cycles */
#define ADC_SAMPLETIME_111POINT5     6   /* 111.5 sampling cycles */
#define ADC_SAMPLETIME_143POINT5     7   /* 143.5 sampling cycles */
#define ADC_SAMPLETIME_479POINT5     8   /* 479.5 sampling cycles */

/* Watchdog threshold registers (WDHT, WDLT) */

#define ADC_WDHT_MASK                0x0fff
#define ADC_WDLT_MASK                0x0fff

/* Routine sequence registers (RSQ0, RSQ1, RSQ2).
 *
 * Each rank owns a 5-bit field.  RSQ2 holds ranks 0..5 and RSQ1 holds ranks
 * 6..8.  RSQ0 only holds the sequence length.
 */

#define ADC_RSQ_LENGTH               5
#define ADC_RSQ_MASK                 31

#define ADC_RSQ0_RL_SHIFT            (20)      /* Routine sequence length */
#define ADC_RSQ0_RL_MASK             (15 << ADC_RSQ0_RL_SHIFT)
#define ADC_RSQ0_RL(n)               (((n) - 1) << ADC_RSQ0_RL_SHIFT)

/* Inserted sequence register (ISQ) */

#define ADC_ISQ_IL_SHIFT             (20)      /* Inserted seq. length */
#define ADC_ISQ_IL_MASK              (3 << ADC_ISQ_IL_SHIFT)

/* Routine data register (RDATA) */

#define ADC_RDATA_MASK               0xffff

/* Oversampling control register (OVSAMPCTL) */

#define ADC_OVSAMPCTL_OVSEN          (1 << 0)  /* Oversampling enable */
#define ADC_OVSAMPCTL_OVSR_SHIFT     (2)       /* Oversampling ratio */
#define ADC_OVSAMPCTL_OVSR_MASK      (7 << ADC_OVSAMPCTL_OVSR_SHIFT)
#define ADC_OVSAMPCTL_OVSS_SHIFT     (5)       /* Oversampling shift */
#define ADC_OVSAMPCTL_OVSS_MASK      (15 << ADC_OVSAMPCTL_OVSS_SHIFT)
#define ADC_OVSAMPCTL_TOVS           (1 << 9)  /* Triggered oversampling */

/* Common control register (CCTL) */

#define ADC_CCTL_ADCCK_SHIFT         (16)      /* ADC clock selection */
#define ADC_CCTL_ADCCK_MASK          (7 << ADC_CCTL_ADCCK_SHIFT)
#  define ADC_CCTL_ADCCK_PCLK2_DIV2  (0 << ADC_CCTL_ADCCK_SHIFT)
#  define ADC_CCTL_ADCCK_PCLK2_DIV4  (1 << ADC_CCTL_ADCCK_SHIFT)
#  define ADC_CCTL_ADCCK_PCLK2_DIV6  (2 << ADC_CCTL_ADCCK_SHIFT)
#  define ADC_CCTL_ADCCK_PCLK2_DIV8  (3 << ADC_CCTL_ADCCK_SHIFT)
#  define ADC_CCTL_ADCCK_HCLK_DIV5   (4 << ADC_CCTL_ADCCK_SHIFT)
#  define ADC_CCTL_ADCCK_HCLK_DIV6   (5 << ADC_CCTL_ADCCK_SHIFT)
#  define ADC_CCTL_ADCCK_HCLK_DIV10  (6 << ADC_CCTL_ADCCK_SHIFT)
#  define ADC_CCTL_ADCCK_HCLK_DIV20  (7 << ADC_CCTL_ADCCK_SHIFT)
#define ADC_CCTL_TSVREN              (1 << 23) /* Temp sensor and VREFINT */

#endif /* __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_ADC_H */
