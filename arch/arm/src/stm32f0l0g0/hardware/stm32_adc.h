/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32_adc.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_ADC_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* STM32 M0 ADC driver:
 * - no injected channels
 * - no offset registers
 * - the F0/L0 family support one sampling time configuration for all
 *   channels
 * - the G0 family support two sampling time configurations
 */

/* Support for battery voltage */

#if 0
#  define HAVE_ADC_VBAT
#else
#  undef HAVE_ADC_VBAT
#endif

/* Support for ADC clock prescaler */

#if defined(CONFIG_STM32F0L0G0_STM32L0) || defined(CONFIG_STM32F0L0G0_STM32G0)
#  define HAVE_ADC_PRE
#else
#  undef HAVE_ADC_PRE
#endif

/* Support for LCD voltage */

#ifdef CONFIG_STM32F0L0G0_HAVE_LCD
#  define  HAVE_ADC_VLCD
#else
#  undef  HAVE_ADC_VLCD
#endif

/* Support for Low frequency mode */

#ifdef CONFIG_STM32F0L0G0_ENERGYLITE
#  define  HAVE_ADC_LFM
#else
#  undef  HAVE_ADC_LFM
#endif

#undef ADC_HAVE_INJECTED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_ADCCMN_OFFSET         0x0300

/* ADC1, ADC2 common - ADC2 not present on STM32 M0/M0+ devices */

#define STM32_ADC12CMN_BASE         (STM32_ADCCMN_OFFSET+STM32_ADC1_BASE)

/* Register Offsets *********************************************************/

#define STM32_ADC_ISR_OFFSET        0x0000  /* ADC interrupt and status register */
#define STM32_ADC_IER_OFFSET        0x0004  /* ADC interrupt enable register */
#define STM32_ADC_CR_OFFSET         0x0008  /* ADC control register */
#define STM32_ADC_CFGR1_OFFSET      0x000c  /* ADC configuration register 1 */
#define STM32_ADC_CFGR2_OFFSET      0x0010  /* ADC configuration register 2 */
#define STM32_ADC_SMPR_OFFSET       0x0014  /* ADC sample time register */
#define STM32_ADC_TR_OFFSET         0x0020  /* ADC watchdog threshold register */
#define STM32_ADC_CHSELR_OFFSET     0x0028  /* ADC channel selection register */
#define STM32_ADC_DR_OFFSET         0x0040  /* ADC regular data register */

/* Master and Slave ADC Common Registers */

#define STM32_ADC_CCR_OFFSET         0x0008  /* Common control register */

/* Register Addresses *******************************************************/

#define STM32_ADC1_ISR              (STM32_ADC1_BASE + STM32_ADC_ISR_OFFSET)
#define STM32_ADC1_IER              (STM32_ADC1_BASE + STM32_ADC_IER_OFFSET)
#define STM32_ADC1_CR               (STM32_ADC1_BASE + STM32_ADC_CR_OFFSET)
#define STM32_ADC1_CFGR1            (STM32_ADC1_BASE + STM32_ADC_CFGR1_OFFSET)
#define STM32_ADC1_CFGR2            (STM32_ADC1_BASE + STM32_ADC_CFGR2_OFFSET)
#define STM32_ADC1_SMPR             (STM32_ADC1_BASE + STM32_ADC_SMPR_OFFSET)
#define STM32_ADC1_TR               (STM32_ADC1_BASE + STM32_ADC_TR_OFFSET)
#define STM32_ADC1_CHSELR           (STM32_ADC1_BASE + STM32_ADC_CHSELR_OFFSET)
#define STM32_ADC1_DR               (STM32_ADC1_BASE + STM32_ADC_DR_OFFSET)
#define STM32_ADC1_CCR              (STM32_ADC1_BASE + STM32_ADC_CCR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* ADC interrupt and status register (ISR) and
 * ADC interrupt enable register (IER)
 */

#define ADC_INT_ARDY                (1 << 0)  /* Bit 0:  ADC ready */
#define ADC_INT_EOSMP               (1 << 1)  /* Bit 1:  End of sampling flag */
#define ADC_INT_EOC                 (1 << 2)  /* Bit 2:  End of conversion */
#define ADC_INT_EOS                 (1 << 3)  /* Bit 3:  End of regular sequence flag */
#define ADC_INT_OVR                 (1 << 4)  /* Bit 4:  Overrun */
#define ADC_INT_AWD                 (1 << 7)  /* Bit 7:  Analog watchdog flag */

/* ADC control register */

#define ADC_CR_ADEN                 (1 << 0)  /* Bit 0: ADC enable control */
#define ADC_CR_ADDIS                (1 << 1)  /* Bit 1: ADC disable command */
#define ADC_CR_ADSTART              (1 << 2)  /* Bit 2: ADC start of regular conversion */
#define ADC_CR_ADSTP                (1 << 4)  /* Bit 4: ADC stop of regular conversion command */
#define ADC_CR_ADVREGEN             (1 << 28) /* Bit 28: ADC Voltage Regulator Enable */
#define ADC_CR_ADCAL                (1 << 31) /* Bit 31: ADC calibration */

/* ADC configuration register 1 */

#define ADC_CFGR1_DMAEN             (1 << 0)  /* Bit 0: Direct memory access enable */
#define ADC_CFGR1_DMACFG            (1 << 1)  /* Bit 1: Direct memory access configuration */
#define ADC_CFGR1_SCANDIR           (1 << 2)  /* Bit 2: Scan sequence direction */
#define ADC_CFGR1_RES_SHIFT         (3)       /* Bits 3-4: Data resolution */
#define ADC_CFGR1_RES_MASK          (3 << ADC_CFGR1_RES_SHIFT)
#  define ADC_CFGR1_RES_12BIT       (0 << ADC_CFGR1_RES_SHIFT) /* 15 ADCCLK clyes */
#  define ADC_CFGR1_RES_10BIT       (1 << ADC_CFGR1_RES_SHIFT) /* 13 ADCCLK clyes */
#  define ADC_CFGR1_RES_8BIT        (2 << ADC_CFGR1_RES_SHIFT) /* 11 ADCCLK clyes */
#  define ADC_CFGR1_RES_6BIT        (3 << ADC_CFGR1_RES_SHIFT) /* 9 ADCCLK clyes */

#define ADC_CFGR1_ALIGN             (1 << 5)  /* Bit 5:  Data Alignment */
#define ADC_CFGR1_EXTSEL_SHIFT      (6)       /* Bits 6-8: External trigger selection */
#define ADC_CFGR1_EXTSEL_MASK       (7 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_TRG0   (0 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_TRG1   (1 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_TRG2   (2 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_TRG3   (3 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_TRG4   (4 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_TRG5   (5 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_TRG6   (6 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_TRG7   (7 << ADC_CFGR1_EXTSEL_SHIFT)
#define ADC_CFGR1_EXTEN_SHIFT       (10)      /* Bits 10-11: External trigger/polarity selection regular channels */
#define ADC_CFGR1_EXTEN_MASK        (3 << ADC_CFGR1_EXTEN_SHIFT)
#  define ADC_CFGR1_EXTEN_NONE      (0 << ADC_CFGR1_EXTEN_SHIFT) /* Trigger detection disabled */
#  define ADC_CFGR1_EXTEN_RISING    (1 << ADC_CFGR1_EXTEN_SHIFT) /* Trigger detection on the rising edge */
#  define ADC_CFGR1_EXTEN_FALLING   (2 << ADC_CFGR1_EXTEN_SHIFT) /* Trigger detection on the falling edge */
#  define ADC_CFGR1_EXTEN_BOTH      (3 << ADC_CFGR1_EXTEN_SHIFT) /* Trigger detection on both edges */

#define ADC_CFGR1_OVRMOD            (1 << 12) /* Bit 12: Overrun Mode */
#define ADC_CFGR1_CONT              (1 << 13) /* Bit 13: Continuous mode for regular conversions */
#define ADC_CFGR1_WAIT              (1 << 14) /* Bit 14: Wait conversion mode */
#define ADC_CFGR1_AUTOFF            (1 << 15) /* Bit 15: Auto-off mode */
#define ADC_CFGR1_DISCEN            (1 << 16) /* Bit 16: Discontinuous mode on regular channels */
#define ADC_CFGR1_AWDSGL            (1 << 22) /* Bit 22: Enable watchdog on single/all channels */
#define ADC_CFGR1_AWDEN             (1 << 23) /* Bit 23: Analog watchdog enable */
#define ADC_CFGR1_AWDCH_SHIFT       (26)      /* Bits 26-30: Analog watchdog 1 channel select bits */
#define ADC_CFGR1_AWDCH_MASK        (31 << ADC_CFGR1_AWDCH_SHIFT)
#  define ADC_CFGR1_AWDCH_DISABLED  (0 << ADC_CFGR1_AWDCH_SHIFT)

/* ADC configuration register 2 */

#define ADC_CFGR2_CKMODE_SHIFT      (30)      /* Bits 30-31: ADC clock mode */
#define ADC_CFGR2_CKMODE_MASK       (3 << ADC_CFGR2_CKMODE_SHIFT)
#  define ADC_CFGR2_CKMODE_ADCCLK   (0 << ADC_CFGR2_CKMODE_SHIFT) /* 00: ADCCLK (Asynchronous) generated at product level */
#  define ADC_CFGR2_CKMODE_PCLKd2   (1 << ADC_CFGR2_CKMODE_SHIFT) /* 01: PCLK/2 (Synchronous clock mode) */
#  define ADC_CFGR2_CKMODE_PCLKd4   (2 << ADC_CFGR2_CKMODE_SHIFT) /* 10: PCLK/4 (Synchronous clock mode) */

/* ADC sample time register */

#define ADC_SMPR_1p5                (0)       /* 000: 1.5 cycles */
#define ADC_SMPR_7p5                (1)       /* 001: 7.5 cycles */
#define ADC_SMPR_13p5               (2)       /* 010: 13.5 cycles */
#define ADC_SMPR_28p5               (3)       /* 011: 28.5 cycles */
#define ADC_SMPR_41p5               (4)       /* 100: 41.5 cycles */
#define ADC_SMPR_55p5               (5)       /* 101: 55.5 cycles */
#define ADC_SMPR_71p5               (6)       /* 110: 71.5 cycles */
#define ADC_SMPR_239p5              (7)       /* 111: 239.5 cycles */

#define ADC_SMPR_SMP1_SHIFT         (0)       /* Bits 0-2: Sampling time selection 1 */
#define ADC_SMPR_SMP1_MASK          (7 << ADC_SMPR_SMP_SHIFT)
#define ADC_SMPR_SMP2_SHIFT         (4)       /* Bits 4-6: Sampling time selection 2 */
#define ADC_SMPR_SMP2_MASK          (7 << ADC_SMPR_SMP_SHIFT)
#define ADC_SMPR_SMPSEL_SHIFT       (8)       /* Bits 8-26: channel-x sampling time selection */
#define ADC_SMPR_SMPSEL(ch, smp)    (smp << ADC_SMPR_SMPSEL_SHIFT)

/* ADC watchdog threshold register */

#define ADC_TR_LT_SHIFT             (0)      /* Bits 0-11: Analog watchdog lower threshold */
#define ADC_TR_LT_MASK              (0x0fff << ADC_TR_LT_SHIFT)
#define ADC_TR_HT_SHIFT             (16)     /* Bits 16-27: Analog watchdog higher threshold */
#define ADC_TR_HT_MASK              (0x0fff << ADC_TR_HT_SHIFT)

/* ADC channel selection register */

#define ADC_CHSELR_CHSEL0           (1 << 0)    /* Select channel 0 */
#define ADC_CHSELR_CHSEL1           (1 << 1)    /* Select channel 1 */
#define ADC_CHSELR_CHSEL2           (1 << 2)    /* Select channel 2 */
#define ADC_CHSELR_CHSEL3           (1 << 3)    /* Select channel 3 */
#define ADC_CHSELR_CHSEL4           (1 << 4)    /* Select channel 4 */
#define ADC_CHSELR_CHSEL5           (1 << 5)    /* Select channel 5 */
#define ADC_CHSELR_CHSEL6           (1 << 6)    /* Select channel 6 */
#define ADC_CHSELR_CHSEL7           (1 << 7)    /* Select channel 7 */
#define ADC_CHSELR_CHSEL8           (1 << 8)    /* Select channel 8 */
#define ADC_CHSELR_CHSEL9           (1 << 9)    /* Select channel 9 */
#define ADC_CHSELR_CHSEL10          (1 << 10)   /* Select channel 10 */
#define ADC_CHSELR_CHSEL11          (1 << 11)   /* Select channel 11 */
#define ADC_CHSELR_CHSEL12          (1 << 12)   /* Select channel 12 */
#define ADC_CHSELR_CHSEL13          (1 << 13)   /* Select channel 13 */
#define ADC_CHSELR_CHSEL14          (1 << 14)   /* Select channel 14 */
#define ADC_CHSELR_CHSEL15          (1 << 15)   /* Select channel 15 */
#define ADC_CHSELR_CHSEL16          (1 << 16)   /* Select channel 16 */
#define ADC_CHSELR_CHSEL17          (1 << 17)   /* Select channel 17 */
#define ADC_CHSELR_CHSEL18          (1 << 18)   /* Select channel 18 */
#define ADC_CHSELR_CHSEL(ch)        (1 << ch)

#define ADC_DR_RDATA_SHIFT          (0)
#define ADC_DR_RDATA_MASK           (0xffff << ADC_DR_RDATA_SHIFT)

/* Common configuration register */

#ifdef HAVE_ADC_VLCD
#  define ADC_CCR_PRESC_SHIFT       (18)       /* ADC Prescaler */
#  define ADC_CCR_PRESC_MASK        (0xf << ADC_CCR_PRESC_SHIFT)
#endif
#define ADC_CCR_VREFEN              (1 << 22)  /* Bit 22: VREFINT enable */
#define ADC_CCR_TSEN                (1 << 23)  /* Bit 23: Temperature sensor enable */
#ifdef HAVE_ADC_VBAT
#  define ADC_CCR_VBATEN            (1 << 24)  /* Bit 24: VBAT enable */
#endif
#ifdef HAVE_ADC_VLCD
#  define ADC_CCR_VLCDEN            (1 << 24)  /* Bit 24: VLCD enable */
#endif
#ifdef HAVE_ADC_LFM
#  define ADC_CCR_LFMEN             (1 << 25)  /* Bit 25: Low Frequency Mode enable */
#endif

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_ADC_H */
