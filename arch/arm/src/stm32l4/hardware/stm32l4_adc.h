/****************************************************************************
 * arch/arm/src/stm32l4/hardware/stm32l4_adc.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_ADC_H
#define __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Register Offsets for each ADC (ADC1-3).  At offset 0x0000 for master and
 * offset 0x0100 for slave.
 */

#define STM32L4_ADC_ISR_OFFSET         0x0000  /* ADC interrupt and status register */
#define STM32L4_ADC_IER_OFFSET         0x0004  /* ADC interrupt enable register */
#define STM32L4_ADC_CR_OFFSET          0x0008  /* ADC control register */
#define STM32L4_ADC_CFGR_OFFSET        0x000c  /* ADC configuration register */
#define STM32L4_ADC_CFGR2_OFFSET       0x0010  /* ADC configuration register 2 */
#define STM32L4_ADC_SMPR1_OFFSET       0x0014  /* ADC sample time register 1 */
#define STM32L4_ADC_SMPR2_OFFSET       0x0018  /* ADC sample time register 2 */
#define STM32L4_ADC_TR1_OFFSET         0x0020  /* ADC watchdog threshold register 1 */
#define STM32L4_ADC_TR2_OFFSET         0x0024  /* ADC watchdog threshold register 2 */
#define STM32L4_ADC_TR3_OFFSET         0x0028  /* ADC watchdog threshold register 3 */
#define STM32L4_ADC_SQR1_OFFSET        0x0030  /* ADC regular sequence register 1 */
#define STM32L4_ADC_SQR2_OFFSET        0x0034  /* ADC regular sequence register 2 */
#define STM32L4_ADC_SQR3_OFFSET        0x0038  /* ADC regular sequence register 3 */
#define STM32L4_ADC_SQR4_OFFSET        0x003c  /* ADC regular sequence register 4 */
#define STM32L4_ADC_DR_OFFSET          0x0040  /* ADC regular data register */
#define STM32L4_ADC_JSQR_OFFSET        0x004c  /* ADC injected sequence register */
#define STM32L4_ADC_OFR1_OFFSET        0x0060  /* ADC offset register 1 */
#define STM32L4_ADC_OFR2_OFFSET        0x0064  /* ADC offset register 2 */
#define STM32L4_ADC_OFR3_OFFSET        0x0068  /* ADC offset register 3 */
#define STM32L4_ADC_OFR4_OFFSET        0x006c  /* ADC offset register 4 */
#define STM32L4_ADC_JDR1_OFFSET        0x0080  /* ADC injected data register 1 */
#define STM32L4_ADC_JDR2_OFFSET        0x0084  /* ADC injected data register 2 */
#define STM32L4_ADC_JDR3_OFFSET        0x0088  /* ADC injected data register 3 */
#define STM32L4_ADC_JDR4_OFFSET        0x008c  /* ADC injected data register 4 */
#define STM32L4_ADC_AWD2CR_OFFSET      0x00a0  /* ADC analog watchdog 2 configuration register */
#define STM32L4_ADC_AWD3CR_OFFSET      0x00a4  /* ADC analog watchdog 3 configuration register */
#define STM32L4_ADC_DIFSEL_OFFSET      0x00b0  /* ADC differential mode selection register 2 */
#define STM32L4_ADC_CALFACT_OFFSET     0x00b4  /* ADC calibration factors */

/* Master and Slave ADC Common Registers */

#define STM32L4_ADC_CSR_OFFSET         0x0000  /* Common status register */
#define STM32L4_ADC_CCR_OFFSET         0x0008  /* Common control register */
#ifndef CONFIG_STM32L4_STM32L4X3
#  define STM32L4_ADC_CDR_OFFSET       0x000c  /* Common regular data register for dual mode */
#endif

/* Register Addresses *******************************************************/

#define STM32L4_ADC1_ISR               (STM32L4_ADC1_BASE+STM32L4_ADC_ISR_OFFSET)
#define STM32L4_ADC1_IER               (STM32L4_ADC1_BASE+STM32L4_ADC_IER_OFFSET)
#define STM32L4_ADC1_CR                (STM32L4_ADC1_BASE+STM32L4_ADC_CR_OFFSET)
#define STM32L4_ADC1_CFGR              (STM32L4_ADC1_BASE+STM32L4_ADC_CFGR_OFFSET)
#define STM32L4_ADC1_CFGR2             (STM32L4_ADC1_BASE+STM32L4_ADC_CFGR2_OFFSET)
#define STM32L4_ADC1_SMPR1             (STM32L4_ADC1_BASE+STM32L4_ADC_SMPR1_OFFSET)
#define STM32L4_ADC1_SMPR2             (STM32L4_ADC1_BASE+STM32L4_ADC_SMPR2_OFFSET)
#define STM32L4_ADC1_TR1               (STM32L4_ADC1_BASE+STM32L4_ADC_TR1_OFFSET)
#define STM32L4_ADC1_TR2               (STM32L4_ADC1_BASE+STM32L4_ADC_TR2_OFFSET)
#define STM32L4_ADC1_TR3               (STM32L4_ADC1_BASE+STM32L4_ADC_TR3_OFFSET)
#define STM32L4_ADC1_SQR1              (STM32L4_ADC1_BASE+STM32L4_ADC_SQR1_OFFSET)
#define STM32L4_ADC1_SQR2              (STM32L4_ADC1_BASE+STM32L4_ADC_SQR2_OFFSET)
#define STM32L4_ADC1_SQR3              (STM32L4_ADC1_BASE+STM32L4_ADC_SQR3_OFFSET)
#define STM32L4_ADC1_SQR4              (STM32L4_ADC1_BASE+STM32L4_ADC_SQR4_OFFSET)
#define STM32L4_ADC1_DR                (STM32L4_ADC1_BASE+STM32L4_ADC_DR_OFFSET)
#define STM32L4_ADC1_JSQR              (STM32L4_ADC1_BASE+STM32L4_ADC_JSQR_OFFSET)
#define STM32L4_ADC1_OFR1              (STM32L4_ADC1_BASE+STM32L4_ADC_OFR1_OFFSET)
#define STM32L4_ADC1_OFR2              (STM32L4_ADC1_BASE+STM32L4_ADC_OFR2_OFFSET)
#define STM32L4_ADC1_OFR3              (STM32L4_ADC1_BASE+STM32L4_ADC_OFR3_OFFSET)
#define STM32L4_ADC1_OFR4              (STM32L4_ADC1_BASE+STM32L4_ADC_OFR4_OFFSET)
#define STM32L4_ADC1_JDR1              (STM32L4_ADC1_BASE+STM32L4_ADC_JDR1_OFFSET)
#define STM32L4_ADC1_JDR2              (STM32L4_ADC1_BASE+STM32L4_ADC_JDR2_OFFSET)
#define STM32L4_ADC1_JDR3              (STM32L4_ADC1_BASE+STM32L4_ADC_JDR3_OFFSET)
#define STM32L4_ADC1_JDR4              (STM32L4_ADC1_BASE+STM32L4_ADC_JDR4_OFFSET)
#define STM32L4_ADC1_AWD2CR            (STM32L4_ADC1_BASE+STM32L4_ADC_AWD2CR_OFFSET)
#define STM32L4_ADC1_AWD3CR            (STM32L4_ADC1_BASE+STM32L4_ADC_AWD3CR_OFFSET)
#define STM32L4_ADC1_DIFSEL            (STM32L4_ADC1_BASE+STM32L4_ADC_DIFSEL_OFFSET)
#define STM32L4_ADC1_CALFACT           (STM32L4_ADC1_BASE+STM32L4_ADC_CALFACT_OFFSET)

#define STM32L4_ADC2_ISR               (STM32L4_ADC2_BASE+STM32L4_ADC_ISR_OFFSET)
#define STM32L4_ADC2_IER               (STM32L4_ADC2_BASE+STM32L4_ADC_IER_OFFSET)
#define STM32L4_ADC2_CR                (STM32L4_ADC2_BASE+STM32L4_ADC_CR_OFFSET)
#define STM32L4_ADC2_CFGR              (STM32L4_ADC2_BASE+STM32L4_ADC_CFGR_OFFSET)
#define STM32L4_ADC2_CFGR2             (STM32L4_ADC2_BASE+STM32L4_ADC_CFGR2_OFFSET)
#define STM32L4_ADC2_SMPR1             (STM32L4_ADC2_BASE+STM32L4_ADC_SMPR1_OFFSET)
#define STM32L4_ADC2_SMPR2             (STM32L4_ADC2_BASE+STM32L4_ADC_SMPR2_OFFSET)
#define STM32L4_ADC2_TR1               (STM32L4_ADC2_BASE+STM32L4_ADC_TR1_OFFSET)
#define STM32L4_ADC2_TR2               (STM32L4_ADC2_BASE+STM32L4_ADC_TR2_OFFSET)
#define STM32L4_ADC2_TR3               (STM32L4_ADC2_BASE+STM32L4_ADC_TR3_OFFSET)
#define STM32L4_ADC2_SQR1              (STM32L4_ADC2_BASE+STM32L4_ADC_SQR1_OFFSET)
#define STM32L4_ADC2_SQR2              (STM32L4_ADC2_BASE+STM32L4_ADC_SQR2_OFFSET)
#define STM32L4_ADC2_SQR3              (STM32L4_ADC2_BASE+STM32L4_ADC_SQR3_OFFSET)
#define STM32L4_ADC2_SQR4              (STM32L4_ADC2_BASE+STM32L4_ADC_SQR4_OFFSET)
#define STM32L4_ADC2_DR                (STM32L4_ADC2_BASE+STM32L4_ADC_DR_OFFSET)
#define STM32L4_ADC2_JSQR              (STM32L4_ADC2_BASE+STM32L4_ADC_JSQR_OFFSET)
#define STM32L4_ADC2_OFR1              (STM32L4_ADC2_BASE+STM32L4_ADC_OFR1_OFFSET)
#define STM32L4_ADC2_OFR2              (STM32L4_ADC2_BASE+STM32L4_ADC_OFR2_OFFSET)
#define STM32L4_ADC2_OFR3              (STM32L4_ADC2_BASE+STM32L4_ADC_OFR3_OFFSET)
#define STM32L4_ADC2_OFR4              (STM32L4_ADC2_BASE+STM32L4_ADC_OFR4_OFFSET)
#define STM32L4_ADC2_JDR1              (STM32L4_ADC2_BASE+STM32L4_ADC_JDR1_OFFSET)
#define STM32L4_ADC2_JDR2              (STM32L4_ADC2_BASE+STM32L4_ADC_JDR2_OFFSET)
#define STM32L4_ADC2_JDR3              (STM32L4_ADC2_BASE+STM32L4_ADC_JDR3_OFFSET)
#define STM32L4_ADC2_JDR4              (STM32L4_ADC2_BASE+STM32L4_ADC_JDR4_OFFSET)
#define STM32L4_ADC2_AWD2CR            (STM32L4_ADC2_BASE+STM32L4_ADC_AWD2CR_OFFSET)
#define STM32L4_ADC2_AWD3CR            (STM32L4_ADC2_BASE+STM32L4_ADC_AWD3CR_OFFSET)
#define STM32L4_ADC2_DIFSEL            (STM32L4_ADC2_BASE+STM32L4_ADC_DIFSEL_OFFSET)
#define STM32L4_ADC2_CALFACT           (STM32L4_ADC2_BASE+STM32L4_ADC_CALFACT_OFFSET)

#define STM32L4_ADC3_ISR               (STM32L4_ADC3_BASE+STM32L4_ADC_ISR_OFFSET)
#define STM32L4_ADC3_IER               (STM32L4_ADC3_BASE+STM32L4_ADC_IER_OFFSET)
#define STM32L4_ADC3_CR                (STM32L4_ADC3_BASE+STM32L4_ADC_CR_OFFSET)
#define STM32L4_ADC3_CFGR              (STM32L4_ADC3_BASE+STM32L4_ADC_CFGR_OFFSET)
#define STM32L4_ADC3_CFGR2             (STM32L4_ADC3_BASE+STM32L4_ADC_CFGR2_OFFSET)
#define STM32L4_ADC3_SMPR1             (STM32L4_ADC3_BASE+STM32L4_ADC_SMPR1_OFFSET)
#define STM32L4_ADC3_SMPR2             (STM32L4_ADC3_BASE+STM32L4_ADC_SMPR2_OFFSET)
#define STM32L4_ADC3_TR1               (STM32L4_ADC3_BASE+STM32L4_ADC_TR1_OFFSET)
#define STM32L4_ADC3_TR2               (STM32L4_ADC3_BASE+STM32L4_ADC_TR2_OFFSET)
#define STM32L4_ADC3_TR3               (STM32L4_ADC3_BASE+STM32L4_ADC_TR3_OFFSET)
#define STM32L4_ADC3_SQR1              (STM32L4_ADC3_BASE+STM32L4_ADC_SQR1_OFFSET)
#define STM32L4_ADC3_SQR2              (STM32L4_ADC3_BASE+STM32L4_ADC_SQR2_OFFSET)
#define STM32L4_ADC3_SQR3              (STM32L4_ADC3_BASE+STM32L4_ADC_SQR3_OFFSET)
#define STM32L4_ADC3_SQR4              (STM32L4_ADC3_BASE+STM32L4_ADC_SQR4_OFFSET)
#define STM32L4_ADC3_DR                (STM32L4_ADC3_BASE+STM32L4_ADC_DR_OFFSET)
#define STM32L4_ADC3_JSQR              (STM32L4_ADC3_BASE+STM32L4_ADC_JSQR_OFFSET)
#define STM32L4_ADC3_OFR1              (STM32L4_ADC3_BASE+STM32L4_ADC_OFR1_OFFSET)
#define STM32L4_ADC3_OFR2              (STM32L4_ADC3_BASE+STM32L4_ADC_OFR2_OFFSET)
#define STM32L4_ADC3_OFR3              (STM32L4_ADC3_BASE+STM32L4_ADC_OFR3_OFFSET)
#define STM32L4_ADC3_OFR4              (STM32L4_ADC3_BASE+STM32L4_ADC_OFR4_OFFSET)
#define STM32L4_ADC3_JDR1              (STM32L4_ADC3_BASE+STM32L4_ADC_JDR1_OFFSET)
#define STM32L4_ADC3_JDR2              (STM32L4_ADC3_BASE+STM32L4_ADC_JDR2_OFFSET)
#define STM32L4_ADC3_JDR3              (STM32L4_ADC3_BASE+STM32L4_ADC_JDR3_OFFSET)
#define STM32L4_ADC3_JDR4              (STM32L4_ADC3_BASE+STM32L4_ADC_JDR4_OFFSET)
#define STM32L4_ADC3_AWD2CR            (STM32L4_ADC3_BASE+STM32L4_ADC_AWD2CR_OFFSET)
#define STM32L4_ADC3_AWD3CR            (STM32L4_ADC3_BASE+STM32L4_ADC_AWD3CR_OFFSET)
#define STM32L4_ADC3_DIFSEL            (STM32L4_ADC3_BASE+STM32L4_ADC_DIFSEL_OFFSET)
#define STM32L4_ADC3_CALFACT           (STM32L4_ADC3_BASE+STM32L4_ADC_CALFACT_OFFSET)

#define STM32L4_ADC_CSR                (STM32L4_ADCCMN_BASE+STM32L4_ADC_CSR_OFFSET)
#define STM32L4_ADC_CCR                (STM32L4_ADCCMN_BASE+STM32L4_ADC_CCR_OFFSET)
#ifndef CONFIG_STM32L4_STM32L4X3
#  define STM32L4_ADC_CDR              (STM32L4_ADCCMN_BASE+STM32L4_ADC_CDR_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* ADC interrupt and status register (ISR) and
 * ADC interrupt enable register (IER)
 */

#define ADC_INT_ADRDY                (1 << 0)  /* Bit 0:  ADC ready */
#define ADC_INT_EOSMP                (1 << 1)  /* Bit 1:  End of sampling flag */
#define ADC_INT_EOC                  (1 << 2)  /* Bit 2:  End of conversion */
#define ADC_INT_EOS                  (1 << 3)  /* Bit 3:  End of regular sequence flag */
#define ADC_INT_OVR                  (1 << 4)  /* Bit 4:  Overrun */
#define ADC_INT_JEOC                 (1 << 5)  /* Bit 5:  Injected channel end of conversion */
#define ADC_INT_JEOS                 (1 << 6)  /* Bit 6:  Injected channel end of sequence flag */
#define ADC_INT_AWD1                 (1 << 7)  /* Bit 7:  Analog watchdog 1 flag */
#define ADC_INT_AWD2                 (1 << 8)  /* Bit 8:  Analog watchdog 2 flag */
#define ADC_INT_AWD3                 (1 << 9)  /* Bit 9:  Analog watchdog 3 flag */
#define ADC_INT_JQOVF                (1 << 10) /* Bit 10: Injected context queue overflow */

#define ADC_INT_MASK                 (0x7ff)

/* ADC control register */

#define ADC_CR_ADEN                  (1 << 0)  /* Bit 0: ADC enable control */
#define ADC_CR_ADDIS                 (1 << 1)  /* Bit 1: ADC disable command */
#define ADC_CR_ADSTART               (1 << 2)  /* Bit 2: ADC start of regular conversion */
#define ADC_CR_JADSTART              (1 << 3)  /* Bit 3: ADC start of injected conversion */
#define ADC_CR_ADSTP                 (1 << 4)  /* Bit 4: ADC stop of regular conversion command */
#define ADC_CR_JADSTP                (1 << 5)  /* Bit 5: ADC stop of injected conversion command */
#define ADC_CR_ADVREGEN              (1 << 28) /* Bit 28: ADC voltage regulator enable */
#define ADC_CR_DEEPPWD               (1 << 29) /* Bit 29: Deep-power-down enable */
#define ADC_CR_ADCALDIF              (1 << 30) /* Bit 30: Differential mode for calibration */
#define ADC_CR_ADCAL                 (1 << 31) /* Bit 31: ADC calibration */

/* ADC configuration register */

#define ADC_CFGR_DMAEN               (1 << 0)  /* Bit 0: Direct memory access enable */
#define ADC_CFGR_DMACFG              (1 << 1)  /* Bit 1: Direct memory access configuration */
#define ADC_CFGR_DFSDMCFG            (1 << 2)  /* Bit 2: DFSDM mode configuration */
#define ADC_CFGR_RES_SHIFT           (3)       /* Bits 3-4: Data resolution */
#define ADC_CFGR_RES_MASK            (3 << ADC_CFGR_RES_SHIFT)
#  define ADC_CFGR_RES_12BIT         (0 << ADC_CFGR_RES_SHIFT)         /* 12-bit resolution */
#  define ADC_CFGR_RES_10BIT         (1 << ADC_CFGR_RES_SHIFT)         /* 10-bit resolution */
#  define ADC_CFGR_RES_8BIT          (2 << ADC_CFGR_RES_SHIFT)         /* 8-bit resolution */
#  define ADC_CFGR_RES_6BIT          (3 << ADC_CFGR_RES_SHIFT)         /* 6-bit resolution */
#define ADC_CFGR_ALIGN               (1 << 5)                          /* Bit 5: Data Alignment */
#define ADC_CFGR_EXTSEL_SHIFT        (6)                               /* Bits 6-9: External Event Select for regular group */
#define ADC_CFGR_EXTSEL_MASK         (15 << ADC_CFGR_EXTSEL_SHIFT)
#  define ADC_CFGR_EXTSEL(event)     ((event) << ADC_CFGR_EXTSEL_SHIFT)/* Event = 0..15 */
#  define ADC_CFGR_EXTSEL_T1CC1      (0x0 << ADC_CFGR_EXTSEL_SHIFT)    /* 0000: Timer 1 CC1 event */
#  define ADC_CFGR_EXTSEL_T1CC2      (0x01 << ADC_CFGR_EXTSEL_SHIFT)   /* 0001: Timer 1 CC2 event */
#  define ADC_CFGR_EXTSEL_T1CC3      (0x02 << ADC_CFGR_EXTSEL_SHIFT)   /* 0010: Timer 1 CC3 event */
#  define ADC_CFGR_EXTSEL_T2CC4      (0x03 << ADC_CFGR_EXTSEL_SHIFT)   /* 0011: Timer 2 CC4 event */
#  define ADC_CFGR_EXTSEL_T3TRGO     (0x04 << ADC_CFGR_EXTSEL_SHIFT)   /* 0100: Timer 3 TRGO event */
#  if !defined(CONFIG_STM32L4_STM32L4X3)
#    define ADC_CFGR_EXTSEL_T4CC4    (0x05 << ADC_CFGR_EXTSEL_SHIFT)   /* 0101: Timer 4 CC4 event */
#  endif
#  define ADC_CFGR_EXTSEL_EXTI11     (0x06 << ADC_CFGR_EXTSEL_SHIFT)   /* 0110: EXTI line 11 */
#  if !defined(CONFIG_STM32L4_STM32L4X3)
#    define ADC_CFGR_EXTSEL_T8TRGO   (0x07 << ADC_CFGR_EXTSEL_SHIFT)   /* 0111: Timer 8 TRGO event */
#    define ADC_CFGR_EXTSEL_T8TRGO2  (0x08 << ADC_CFGR_EXTSEL_SHIFT)   /* 1000: Timer 8 TRGO2 event */
#  endif
#  define ADC_CFGR_EXTSEL_T1TRGO     (0x09 << ADC_CFGR_EXTSEL_SHIFT)   /* 1001: Timer 1 TRGO event */
#  define ADC_CFGR_EXTSEL_T1TRGO2    (0x0a << ADC_CFGR_EXTSEL_SHIFT)   /* 1010: Timer 1 TRGO2 event */
#  define ADC_CFGR_EXTSEL_T2TRGO     (0x0b << ADC_CFGR_EXTSEL_SHIFT)   /* 1011: Timer 2 TRGO event */
#  if !defined(CONFIG_STM32L4_STM32L4X3)
#    define ADC_CFGR_EXTSEL_T4TRGO   (0x0c << ADC_CFGR_EXTSEL_SHIFT)   /* 1100: Timer 4 TRGO event */
#  endif
#  define ADC_CFGR_EXTSEL_T6TRGO     (0x0d << ADC_CFGR_EXTSEL_SHIFT)   /* 1101: Timer 6 TRGO event */
#  define ADC_CFGR_EXTSEL_T15TRGO    (0x0e << ADC_CFGR_EXTSEL_SHIFT)   /* 1110: Timer 15 TRGO event */
#  if !defined(CONFIG_STM32L4_STM32L4X3)
#    define ADC_CFGR_EXTSEL_T3CC4    (0x0f << ADC_CFGR_EXTSEL_SHIFT)   /* 1111: Timer 3 CC4 event */
#  endif
#define ADC_CFGR_EXTEN_SHIFT         (10)                              /* Bits 10-11: External trigger/polarity selection regular channels */
#define ADC_CFGR_EXTEN_MASK          (3 << ADC_CFGR_EXTEN_SHIFT)
#  define ADC_CFGR_EXTEN_NONE        (0 << ADC_CFGR_EXTEN_SHIFT)       /* Trigger detection disabled */
#  define ADC_CFGR_EXTEN_RISING      (1 << ADC_CFGR_EXTEN_SHIFT)       /* Trigger detection on the rising edge */
#  define ADC_CFGR_EXTEN_FALLING     (2 << ADC_CFGR_EXTEN_SHIFT)       /* Trigger detection on the falling edge */
#  define ADC_CFGR_EXTEN_BOTH        (3 << ADC_CFGR_EXTEN_SHIFT)       /* Trigger detection on both edges */
#define ADC_CFGR_OVRMOD              (1 << 12)                         /* Bit 12: Overrun Mode */
#define ADC_CFGR_CONT                (1 << 13)                         /* Bit 13: Continuous mode for regular conversions */
#define ADC_CFGR_AUTDLY              (1 << 14)                         /* Bit 14: Delayed conversion mode */
#define ADC_CFGR_DISCEN              (1 << 16)                         /* Bit 16: Discontinuous mode on regular channels */
#define ADC_CFGR_DISCNUM_SHIFT       (17)                              /* Bits 17-19: Discontinuous mode channel count */
#define ADC_CFGR_DISCNUM_MASK        (7 << ADC_CFGR_DISCNUM_SHIFT)
#  define ADC_CFGR_DISCNUM(n)        (((n) - 1) << ADC_CFGR_DISCNUM_SHIFT)
                                                                       /* n = 1..8 channels */

#define ADC_CFGR_JDISCEN             (1 << 20) /* Bit 20: Discontinuous mode on injected channels */
#define ADC_CFGR_JQM                 (1 << 21) /* Bit 21: JSQR queue mode */
#define ADC_CFGR_AWD1SGL             (1 << 22) /* Bit 22: Enable watchdog on single/all channels */
#define ADC_CFGR_AWD1EN              (1 << 23) /* Bit 23: Analog watchdog enable 1 regular channels */
#define ADC_CFGR_JAWD1EN             (1 << 22) /* Bit 22: Analog watchdog enable 1 injected channels */
#define ADC_CFGR_JAUTO               (1 << 25) /* Bit 25: Automatic Injected Group conversion */
#define ADC_CFGR_AWD1CH_SHIFT        (26)      /* Bits 26-30: Analog watchdog 1 channel select bits */
#define ADC_CFGR_AWD1CH_MASK         (31 << ADC_CFGR_AWD1CH_SHIFT)
#  define ADC_CFGR_AWD1CH_DISABLED   (0 << ADC_CFGR_AWD1CH_SHIFT)
#define ADC_CFGR_JQDIS               (1 << 31) /* Bit 31: Injected Queue disable */

/* ADC configuration register 2 */

#define ADC_CFGR2_ROVSE              (1 << 0)  /* Bit 0: Regular Oversampling Enable */
#define ADC_CFGR2_JOVSE              (1 << 1)  /* Bit 1: Injected Oversampling Enable */
#define ADC_CFGR2_OVSR_SHIFT         (2)       /* Bits 2-4: Oversampling ratio */
#define ADC_CFGR2_OVSR_MASK          (7 << ADC_CFGR2_OVSR_SHIFT)
#  define ADC_CFGR2_OVSR_2X          (0 << ADC_CFGR2_OVSR_SHIFT) /* 2X oversampling */
#  define ADC_CFGR2_OVSR_4X          (1 << ADC_CFGR2_OVSR_SHIFT) /* 4X oversampling */
#  define ADC_CFGR2_OVSR_8X          (2 << ADC_CFGR2_OVSR_SHIFT) /* 8X oversampling */
#  define ADC_CFGR2_OVSR_16X         (3 << ADC_CFGR2_OVSR_SHIFT) /* 16X oversampling */
#  define ADC_CFGR2_OVSR_32X         (4 << ADC_CFGR2_OVSR_SHIFT) /* 32X oversampling */
#  define ADC_CFGR2_OVSR_64X         (5 << ADC_CFGR2_OVSR_SHIFT) /* 64X oversampling */
#  define ADC_CFGR2_OVSR_128X        (6 << ADC_CFGR2_OVSR_SHIFT) /* 128X oversampling */
#  define ADC_CFGR2_OVSR_256X        (7 << ADC_CFGR2_OVSR_SHIFT) /* 256X oversampling */
#define ADC_CFGR2_OVSS_SHIFT         (5)                         /* Bits 5-8: Oversampling shift */
#define ADC_CFGR2_OVSS_MASK          (0xf << ADC_CFGR2_OVSS_SHIFT)
#  define ADC_CFGR2_OVSS(value)      ((value) << ADC_CFGR2_OVSS_SHIFT)
                                                                /* Value = 0..8 */
#define ADC_CFGR2_TROVS              (1 << 9)                   /* Bit 9: Triggered Regular Oversampling */
#define ADC_CFGR2_ROVSM              (1 << 10)                  /* Bit 10: Regular Oversampling mode */

/* ADC sample time register 1 */

#define ADC_SMPR_2p5                 0         /* 000: 2.5 cycles */
#define ADC_SMPR_6p5                 1         /* 001: 6.5 cycles */
#define ADC_SMPR_12p5                2         /* 010: 12.5 cycles */
#define ADC_SMPR_24p5                3         /* 011: 24.5 cycles */
#define ADC_SMPR_47p5                4         /* 100: 47.5 cycles */
#define ADC_SMPR_92p5                5         /* 101: 92.5 cycles */
#define ADC_SMPR_247p5               6         /* 110: 247.5 cycles */
#define ADC_SMPR_640p5               7         /* 111: 640.5 cycles */

#define ADC_SMPR1_SMP0_SHIFT         (0)       /* Bits 2-0: Channel 0 Sample time selection */
#define ADC_SMPR1_SMP0_MASK          (7 << ADC_SMPR1_SMP0_SHIFT)

#define ADC_SMPR1_SMP1_SHIFT         (3)       /* Bits 5-3: Channel 1 Sample time selection */
#define ADC_SMPR1_SMP1_MASK          (7 << ADC_SMPR1_SMP1_SHIFT)
#define ADC_SMPR1_SMP2_SHIFT         (6)       /* Bits 8-6: Channel 2 Sample time selection */
#define ADC_SMPR1_SMP2_MASK          (7 << ADC_SMPR1_SMP2_SHIFT)
#define ADC_SMPR1_SMP3_SHIFT         (9)       /* Bits 11-9: Channel 3 Sample time selection */
#define ADC_SMPR1_SMP3_MASK          (7 << ADC_SMPR1_SMP3_SHIFT)
#define ADC_SMPR1_SMP4_SHIFT         (12)      /* Bits 14-12: Channel 4 Sample time selection */
#define ADC_SMPR1_SMP4_MASK          (7 << ADC_SMPR1_SMP4_SHIFT)
#define ADC_SMPR1_SMP5_SHIFT         (15)      /* Bits 17-15: Channel 5 Sample time selection */
#define ADC_SMPR1_SMP5_MASK          (7 << ADC_SMPR1_SMP5_SHIFT)
#define ADC_SMPR1_SMP6_SHIFT         (18)      /* Bits 20-18: Channel 6 Sample time selection */
#define ADC_SMPR1_SMP6_MASK          (7 << ADC_SMPR1_SMP6_SHIFT)
#define ADC_SMPR1_SMP7_SHIFT         (21)      /* Bits 23-21: Channel 7 Sample time selection */
#define ADC_SMPR1_SMP7_MASK          (7 << ADC_SMPR1_SMP7_SHIFT)
#define ADC_SMPR1_SMP8_SHIFT         (24)      /* Bits 26-24: Channel 8 Sample time selection */
#define ADC_SMPR1_SMP8_MASK          (7 << ADC_SMPR1_SMP8_SHIFT)
#define ADC_SMPR1_SMP9_SHIFT         (27)      /* Bits 29-27: Channel 9 Sample time selection */
#define ADC_SMPR1_SMP9_MASK          (7 << ADC_SMPR1_SMP9_SHIFT)

/* ADC sample time register 2 */

#define ADC_SMPR2_SMP10_SHIFT        (0)       /* Bits 0-2: Channel 10 Sample time selection */
#define ADC_SMPR2_SMP10_MASK         (7 << ADC_SMPR2_SMP10_SHIFT)
#define ADC_SMPR2_SMP11_SHIFT        (3)       /* Bits 3-5: Channel 11 Sample time selection */
#define ADC_SMPR2_SMP11_MASK         (7 << ADC_SMPR2_SMP11_SHIFT)
#define ADC_SMPR2_SMP12_SHIFT        (6)       /* Bits 6-8: Channel 12 Sample time selection */
#define ADC_SMPR2_SMP12_MASK         (7 << ADC_SMPR2_SMP12_SHIFT)
#define ADC_SMPR2_SMP13_SHIFT        (9)       /* Bits 9-11: Channel 13 Sample time selection */
#define ADC_SMPR2_SMP13_MASK         (7 << ADC_SMPR2_SMP13_SHIFT)
#define ADC_SMPR2_SMP14_SHIFT        (12)      /* Bits 12-14: Channel 14 Sample time selection */
#define ADC_SMPR2_SMP14_MASK         (7 << ADC_SMPR2_SMP14_SHIFT)
#define ADC_SMPR2_SMP15_SHIFT        (15)      /* Bits 15-17: Channel 15 Sample time selection */
#define ADC_SMPR2_SMP15_MASK         (7 << ADC_SMPR2_SMP15_SHIFT)
#define ADC_SMPR2_SMP16_SHIFT        (18)      /* Bits 18-20: Channel 16 Sample time selection */
#define ADC_SMPR2_SMP16_MASK         (7 << ADC_SMPR2_SMP16_SHIFT)
#define ADC_SMPR2_SMP17_SHIFT        (21)      /* Bits 21-23: Channel 17 Sample time selection */
#define ADC_SMPR2_SMP17_MASK         (7 << ADC_SMPR2_SMP17_SHIFT)
#define ADC_SMPR2_SMP18_SHIFT        (24)      /* Bits 24-26: Channel 18 Sample time selection */
#define ADC_SMPR2_SMP18_MASK         (7 << ADC_SMPR2_SMP18_SHIFT)

/* ADC watchdog threshold register 1 */

#define ADC_TR1_LT_SHIFT             (0)      /* Bits 0-11: Analog watchdog 1 lower threshold */
#define ADC_TR1_LT_MASK              (0x0fff << ADC_TR1_LT_SHIFT)
#define ADC_TR1_HT_SHIFT             (16)     /* Bits 16-27: Analog watchdog 1 higher threshold */
#define ADC_TR1_HT_MASK              (0x0fff << ADC_TR1_HT_SHIFT)

/* ADC watchdog threshold register 2 */

#define ADC_TR2_LT_SHIFT             (0)      /* Bits 0-7: Analog watchdog 2 lower threshold */
#define ADC_TR2_LT_MASK              (0xff << ADC_TR2_LT_SHIFT)
#define ADC_TR2_HT_SHIFT             (16)     /* Bits 16-23: Analog watchdog 2 higher threshold */
#define ADC_TR2_HT_MASK              (0xff << ADC_TR2_HT_SHIFT)

/* ADC watchdog threshold register 3 */

#define ADC_TR3_LT_SHIFT             (0)      /* Bits 0-7: Analog watchdog 3 lower threshold */
#define ADC_TR3_LT_MASK              (0xff << ADC_TR3_LT_SHIFT)
#define ADC_TR3_HT_SHIFT             (16)     /* Bits 16-23: Analog watchdog 3 higher threshold */
#define ADC_TR3_HT_MASK              (0xff << ADC_TR3_HT_SHIFT)

/* Offset between SQ bits */

#define ADC_SQ_OFFSET                (6)

/* ADC regular sequence register 1 */

#define ADC_SQR1_L_SHIFT             (0)         /* Bits 0-3:   Regular channel sequence length */
#define ADC_SQR1_L_MASK              (0x0f << ADC_SQR1_L_SHIFT)
#define ADC_SQR1_SQ1_SHIFT           (6)         /* Bits 6-10:  1st conversion in regular sequence */
#define ADC_SQR1_SQ1_MASK            (0x1f << ADC_SQR1_SQ1_SHIFT)
#define ADC_SQR1_SQ2_SHIFT           (12)        /* Bits 12-16: 2nd conversion in regular sequence */
#define ADC_SQR1_SQ2_MASK            (0x1f << ADC_SQR1_SQ2_SHIFT)
#define ADC_SQR1_SQ3_SHIFT           (18)        /* Bits 18-22: 3rd conversion in regular sequence */
#define ADC_SQR1_SQ3_MASK            (0x1f << ADC_SQR1_SQ3_SHIFT)
#define ADC_SQR1_SQ4_SHIFT           (24)        /* Bits 24-28: 4th conversion in regular sequence */
#define ADC_SQR1_SQ4_MASK            (0x1f << ADC_SQR1_SQ4_SHIFT)
#define ADC_SQR1_RESERVED            (0xe0820830)
#define ADC_SQR1_FIRST               (1)
#define ADC_SQR1_LAST                (4)
#define ADC_SQR1_SQ_OFFSET           (1*ADC_SQ_OFFSET)

/* ADC regular sequence register 2 */

#define ADC_SQR2_SQ5_SHIFT           (0)         /* Bits 4-0:   5th conversion in regular sequence */
#define ADC_SQR2_SQ5_MASK            (0x1f << ADC_SQR2_SQ5_SHIFT)
#define ADC_SQR2_SQ6_SHIFT           (6)         /* Bits 6-10:  6th conversion in regular sequence */
#define ADC_SQR2_SQ6_MASK            (0x1f << ADC_SQR2_SQ6_SHIFT)
#define ADC_SQR2_SQ7_SHIFT           (12)        /* Bits 12-16: 7th conversion in regular sequence */
#define ADC_SQR2_SQ7_MASK            (0x1f << ADC_SQR2_SQ7_SHIFT)
#define ADC_SQR2_SQ8_SHIFT           (18)        /* Bits 18-22: 8th conversion in regular sequence */
#define ADC_SQR2_SQ8_MASK            (0x1f << ADC_SQR2_SQ8_SHIFT)
#define ADC_SQR2_SQ9_SHIFT           (24)        /* Bits 24-28: 9th conversion in regular sequence */
#define ADC_SQR2_SQ9_MASK            (0x1f << ADC_SQR2_SQ9_SHIFT )
#define ADC_SQR2_RESERVED            (0xe0820820)
#define ADC_SQR2_FIRST               (5)
#define ADC_SQR2_LAST                (9)
#define ADC_SQR2_SQ_OFFSET           (0)

/* ADC regular sequence register 3 */

#define ADC_SQR3_SQ10_SHIFT          (0)         /* Bits 4-0:   10th conversion in regular sequence */
#define ADC_SQR3_SQ10_MASK           (0x1f << ADC_SQR3_SQ10_SHIFT)
#define ADC_SQR3_SQ11_SHIFT          (6)         /* Bits 6-10:  11th conversion in regular sequence */
#define ADC_SQR3_SQ11_MASK           (0x1f << ADC_SQR3_SQ11_SHIFT)
#define ADC_SQR3_SQ12_SHIFT          (12)        /* Bits 12-16: 12th conversion in regular sequence */
#define ADC_SQR3_SQ12_MASK           (0x1f << ADC_SQR3_SQ12_SHIFT)
#define ADC_SQR3_SQ13_SHIFT          (18)        /* Bits 18-22: 13th conversion in regular sequence */
#define ADC_SQR3_SQ13_MASK           (0x1f << ADC_SQR3_SQ13_SHIFT)
#define ADC_SQR3_SQ14_SHIFT          (24)        /* Bits 24-28: 14th conversion in regular sequence */
#define ADC_SQR3_SQ14_MASK           (0x1f << ADC_SQR3_SQ14_SHIFT)
#define ADC_SQR3_RESERVED            (0xe0820820)
#define ADC_SQR3_FIRST               (10)
#define ADC_SQR3_LAST                (14)
#define ADC_SQR3_SQ_OFFSET           (0)

/* ADC regular sequence register 4 */

#define ADC_SQR4_SQ15_SHIFT          (0)         /* Bits 4-0:   14th conversion in regular sequence */
#define ADC_SQR4_SQ15_MASK           (0x1f << ADC_SQR4_SQ15_SHIFT)
#define ADC_SQR4_SQ16_SHIFT          (6)         /* Bits 6-10:  15th conversion in regular sequence */
#define ADC_SQR4_SQ16_MASK           (0x1f << ADC_SQR4_SQ16_SHIFT)
#define ADC_SQR4_RESERVED            (0xfffff820)
#define ADC_SQR4_FIRST               (15)
#define ADC_SQR4_LAST                (16)
#define ADC_SQR4_SQ_OFFSET           (0)

/* ADC regular data register */

#define ADC_DR_MASK                  (0xffff)

/* ADC injected sequence register */

#define ADC_JSQR_JL_SHIFT            (0)                          /* Bits 0-1: Injected Sequence length */
#define ADC_JSQR_JL_MASK             (3 << ADC_JSQR_JL_SHIFT)
#  define ADC_JSQR_JL(n)             (((n)-1) << ADC_JSQR_JL_SHIFT)
                                                                  /* n=1..4 */
#define ADC_JSQR_JEXTSEL_SHIFT       (2)                          /* Bits 2-5: External Trigger Selection for injected group */
#define ADC_JSQR_JEXTSEL_MASK        (15 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC_JSQR_JEXTSEL(event)    ((event) << ADC_JSQR_JEXTSEL_SHIFT)
                                                                  /* Event = 0..15 */
#  define ADC_JEXTSEL_T1TRGO         ADC_JSQR_JEXTSEL(0)          /* 0000 TIM1_TRGO */
#  define ADC_JEXTSEL_T1CC4          ADC_JSQR_JEXTSEL(1)          /* 0001 TIM1_CH4 */
#  define ADC_JEXTSEL_T2TRGO         ADC_JSQR_JEXTSEL(2)          /* 0010 TIM2_TRGO */
#  define ADC_JEXTSEL_T2CC1          ADC_JSQR_JEXTSEL(3)          /* 0011 TIM2_CH1 */
#  define ADC_JEXTSEL_T3CC4          ADC_JSQR_JEXTSEL(4)          /* 0100 TIM3_CH4 */
#  define ADC_JEXTSEL_T4TRGO         ADC_JSQR_JEXTSEL(5)          /* 0101 TIM4_TRGO */
#  define ADC_JEXTSEL_EXTI15         ADC_JSQR_JEXTSEL(6)          /* 0110 EXTI line 15 */
#  define ADC_JEXTSEL_T8CC4          ADC_JSQR_JEXTSEL(7)          /* 0111 TIM8_CH4 */
#  define ADC_JEXTSEL_T1TRGO2        ADC_JSQR_JEXTSEL(8)          /* 1000 TIM1_TRGO2 */
#  define ADC_JEXTSEL_T8TRGO         ADC_JSQR_JEXTSEL(9)          /* 1001 TIM8_TRGO */
#  define ADC_JEXTSEL_T8TRGO2        ADC_JSQR_JEXTSEL(10)         /* 1010 TIM8_TRG02 */
#  define ADC_JEXTSEL_T3CC3          ADC_JSQR_JEXTSEL(11)         /* 1011 TIM3_CH3 */
#  define ADC_JEXTSEL_T3TRGO         ADC_JSQR_JEXTSEL(12)         /* 1011 TIM3_TRGO */
#  define ADC_JEXTSEL_T3CC1          ADC_JSQR_JEXTSEL(13)         /* 1101 TIM3_CH1 */
#  define ADC_JEXTSEL_T6TRGO         ADC_JSQR_JEXTSEL(14)         /* 1110 TIM6_TRGO */
#  define ADC_JEXTSEL_T15TRGO        ADC_JSQR_JEXTSEL(15)         /* 1111 TIM15_TRGO */
#define ADC_JSQR_JEXTEN_SHIFT        (6)                          /* Bits 6-7: External trigger selection for injected greoup */
#define ADC_JSQR_JEXTEN_MASK         (3 << ADC_JSQR_JEXTEN_SHIFT)
#  define ADC_JSQR_JEXTEN_NONE       (0 << ADC_JSQR_JEXTEN_SHIFT) /* 00: Trigger detection disabled */
#  define ADC_JSQR_JEXTEN_RISING     (1 << ADC_JSQR_JEXTEN_SHIFT) /* 01: Trigger detection on the rising edge */
#  define ADC_JSQR_JEXTEN_FALLING    (2 << ADC_JSQR_JEXTEN_SHIFT) /* 10: Trigger detection on the falling edge */
#  define ADC_JSQR_JEXTEN_BOTH       (3 << ADC_JSQR_JEXTEN_SHIFT) /* 11: Trigger detection on both the rising and falling edges */
#define ADC_JSQR_JSQ_SHIFT           (6)
#define ADC_JSQR_JSQ1_SHIFT          (8)                          /* Bits 8-12: 1st conversion in injected sequence */
#define ADC_JSQR_JSQ1_MASK           (0x1f << ADC_JSQR_JSQ1_SHIFT)
#  define ADC_JSQR_JSQ1(ch)          ((ch) << ADC_JSQR_JSQ1_SHIFT)/* Channel number 1..18 */
#define ADC_JSQR_JSQ2_SHIFT          (14)                         /* Bits 14-18: 2nd conversion in injected sequence */
#define ADC_JSQR_JSQ2_MASK           (0x1f << ADC_JSQR_JSQ2_MASK)
#  define ADC_JSQR_JSQ2(ch)          ((ch) << ADC_JSQR_JSQ2_MASK) /* Channel number 1..18 */
#define ADC_JSQR_JSQ3_SHIFT          (20)                         /* Bits 20-24: 3rd conversion in injected sequence */
#define ADC_JSQR_JSQ3_MASK           (0x1f << ADC_JSQR_JSQ3_SHIFT)
#  define ADC_JSQR_JSQ3(ch)          ((ch) << ADC_JSQR_JSQ3_SHIFT)/* Channel number 1..18 */
#define ADC_JSQR_JSQ4_SHIFT          (26)                         /* Bits 26-30: 4th conversion in injected sequence */
#define ADC_JSQR_JSQ4_MASK           (0x1f << ADC_JSQR_JSQ4_SHIFT)
#  define ADC_JSQR_JSQ4(ch)          ((ch) << ADC_JSQR_JSQ4_SHIFT)/* Channel number 1..18 */

/* ADC offset register 1, 2, 3, and 4 */

#define ADC_OFR_OFFSETY_SHIFT        (0)        /* Bits 0-11: Data offset y for channel OFFSETY_CH */
#define ADC_OFR_OFFSETY_MASK         (0x0fff << ADC_OFR_OFFSETY_SHIFT)
#  define ADC_OFR_OFFSETY(offset)    ((offset) << ADC_OFR_OFFSETY_SHIFT)
#define ADC_OFR_OFFSETY_CH_SHIFT     (26)       /* Bits 26-30: Channel selection for data offset y */
#define ADC_OFR_OFFSETY_CH_MASK      (31 << ADC_OFR_OFFSETY_CH_SHIFT)
#  define ADC_OFR_OFFSETY_CH(ch)     ((ch) << ADC_OFR_OFFSETY_CH_SHIFT)
#define ADC_OFR_OFFSETY_EN           (1 << 31)  /* Bit 31: Offset y enable */

/* ADC injected data register 1, 2, 3, and 4 */

#define ADC_JDR_MASK                 (0xffff)

/* ADC analog watchdog 2 configuration register */

#define ADC_AWD2CR_CH_SHIFT          (1)       /* Bits 1-18: Analog watchdog 2 channel selection */
#define ADC_AWD2CR_CH_MASK           (0x3ffff << ADC_AWD2CR_CH_SHIFT)
#  define ADC_AWD2CR_CH(n)           (1 << (n)) /* Channel n=1..18 */

/* ADC analog watchdog 3 configuration register */

#define ADC_AWD3CR_CH_SHIFT         (1)        /* Bits 1-18: Analog watchdog 2 channel selection */
#define ADC_AWD3CR_CH_MASK          (0x3ffff << ADC_AWD3CR_CH_SHIFT)
#  define ADC_AWD3CR_CH(n)          (1 << (n)) /* Channel n=1..18 */

/* ADC differential mode selection register */

#define ADC_DIFSEL_CH_SHIFT         (1)        /* Bits 1-18: Analog watchdog 2 channel selection */
#define ADC_DIFSEL_CH_MASK          (0x3ffff << ADC_DIFSEL_CH_SHIFT)
#  define ADC_DIFSEL_CH(n)          (1 << (n)) /* Channel n=1..18 */

/* ADC calibration factors */

#define ADC_CALFACT_S_SHIFT          (0)       /* Bits 0-6: Calibration factors in single-ended mode */
#define ADC_CALFACT_S_MASK           (0x7f << ADC_CALFACT_S_SHIFT)
#define ADC_CALFACT_D_SHIFT          (16)       * Bits 16-22: Calibration Factors indifferential mode */
#define ADC_CALFACT_D_MASK           (0x7f << ADC_CALFACT_D_SHIFT)

/* Common status register */

#define ADC_CSR_ADRDY_MST            (1 << 0)  /* Bit 0: Master ADC ready */
#define ADC_CSR_EOSMP_MST            (1 << 1)  /* Bit 1: End of Sampling phase flag (master ADC) */
#define ADC_CSR_EOC_MST              (1 << 2)  /* Bit 2: End of regular conversion (master ADC) */
#define ADC_CSR_EOS_MST              (1 << 3)  /* Bit 3: End of regular sequence flag (master ADC) */
#define ADC_CSR_OVR_MST              (1 << 4)  /* Bit 4: Overrun flag (master ADC) */
#define ADC_CSR_JEOC_MST             (1 << 5)  /* Bit 5: End of injected conversion flag (master ADC) */
#define ADC_CSR_JEOS_MST             (1 << 6)  /* Bit 6: End of injected sequence flag (master ADC) */
#define ADC_CSR_AWD1_MST             (1 << 7)  /* Bit 7: Analog watchdog 1 flag (master ADC) */
#define ADC_CSR_AWD2_MST             (1 << 8)  /* Bit 8: Analog watchdog 2 flag (master ADC) */
#define ADC_CSR_AWD3_MST             (1 << 9)  /* Bit 9: Analog watchdog 3 flag (master ADC) */
#define ADC_CSR_JQOVF_MST            (1 << 10) /* Bit 10: Injected Context Queue Overflow flag (master ADC) */

#ifndef CONFIG_STM32L4_STM32L4X3
#  define ADC_CSR_ADRDY_SLV          (1 << 16) /* Bit 16: Slave ADC ready */
#  define ADC_CSR_EOSMP_SLV          (1 << 17) /* Bit 17: End of Sampling phase flag (slave ADC) */
#  define ADC_CSR_EOC_SLV            (1 << 18) /* Bit 18: End of regular conversion (slave ADC) */
#  define ADC_CSR_EOS_SLV            (1 << 19) /* Bit 19: End of regular sequence flag (slave ADC) */
#  define ADC_CSR_OVR_SLV            (1 << 20) /* Bit 20: Overrun flag (slave ADC) */
#  define ADC_CSR_JEOC_SLV           (1 << 21) /* Bit 21: End of injected conversion flag (slave ADC) */
#  define ADC_CSR_JEOS_SLV           (1 << 22) /* Bit 22: End of injected sequence flag (slave ADC) */
#  define ADC_CSR_AWD1_SLV           (1 << 23) /* Bit 23: Analog watchdog 1 flag (slave ADC) */
#  define ADC_CSR_AWD2_SLV           (1 << 24) /* Bit 24: Analog watchdog 2 flag (slave ADC) */
#  define ADC_CSR_AWD3_SLV           (1 << 25) /* Bit 25: Analog watchdog 3 flag (slave ADC) */
#  define ADC_CSR_JQOVF_SLV          (1 << 26) /* Bit 26: Injected Context Queue Overflow flag (slave ADC) */
#endif

/* Common control register */

#ifndef CONFIG_STM32L4_STM32L4X3
#  define ADC_CCR_DUAL_SHIFT         (0)                         /* Bits 0-4: Dual ADC mode selection */
#  define ADC_CCR_DUAL_MASK          (31 << ADC_CCR_DUAL_SHIFT)
#    define ADC_CCR_DUAL_IND         (0 << ADC_CCR_DUAL_SHIFT)   /* Independent mode */
#    define ADC_CCR_DUAL_DUAL        (1 << ADC_CCR_DUAL_SHIFT)   /* Dual mode, master/slave ADCs together */
#    define ADC_CCR_DUAL_SIMINJ      (1 << ADC_CCR_DUAL_SHIFT)   /* Combined regular sim. + injected sim. */
#    define ADC_CCR_DUAL_SIMALT      (2 << ADC_CCR_DUAL_SHIFT)   /* Combined regular sim. + alternate trigger */
#    define ADC_CCR_DUAL_INJECTED    (5 << ADC_CCR_DUAL_SHIFT)   /* Injected simultaneous mode only */
#    define ADC_CCR_DUAL_SIM         (6 << ADC_CCR_DUAL_SHIFT)   /* Regular simultaneous mode only */
#    define ADC_CCR_DUAL_INTERLEAVE  (7 << ADC_CCR_DUAL_SHIFT)   /* Interleaved mode only */
#    define ADC_CCR_DUAL_ALT         (9 << ADC_CCR_DUAL_SHIFT)   /* Alternate trigger mode only */
#  define ADC_CCR_DELAY_SHIFT        (8)                         /* Bits 8-11: Delay between 2 sampling phases */
#  define ADC_CCR_DELAY_MASK         (15 << ADC_CCR_DELAY_SHIFT)
#    define ADC_CCR_DELAY(n)         (((n)-1) << ADC_CCR_DELAY_SHIFT)
                                                                 /* n * TADCCLK, 1-13 */
#  define ADC_CCR_DMACFG             (1 << 13)                   /* Bit 13: DMA configuration (for dual ADC mode) */
#  define ADC_CCR_MDMA_SHIFT         (14)                        /* Bits 14-15: Direct memory access mode for dual ADC mode */
#  define ADC_CCR_MDMA_MASK          (3 << ADC_CCR_MDMA_SHIFT)
#    define ADC_CCR_MDMA_DISABLE     (0 << ADC_CCR_MDMA_SHIFT)   /* MDMA mode disabled */
#    define ADC_CCR_MDMA_10_12       (2 << ADC_CCR_MDMA_SHIFT)   /* MDMA mode enabled (12 / 10-bit) */
#    define ADC_CCR_MDMA_6_8         (3 << ADC_CCR_MDMA_SHIFT)   /* MDMA mode enabled (8 / 6-bit) */
#endif
#define ADC_CCR_CKMODE_SHIFT         (16)                        /* Bits 16-17: ADC clock mode */
#define ADC_CCR_CKMODE_MASK          (3 << ADC_CCR_CKMODE_SHIFT)
#  define ADC_CCR_CKMODE_ASYCH       (0 << ADC_CCR_CKMODE_SHIFT) /* Asynchronous clock mode */
#  define ADC_CCR_CKMODE_SYNCH_DIV1  (1 << ADC_CCR_CKMODE_SHIFT) /* Synchronous clock mode divided by 1 */
#  define ADC_CCR_CKMODE_SYNCH_DIV2  (2 << ADC_CCR_CKMODE_SHIFT) /* Synchronous clock mode divided by 2 */
#  define ADC_CCR_CKMODE_SYNCH_DIV4  (3 << ADC_CCR_CKMODE_SHIFT) /* Synchronous clock mode divided by 4 */
#define ADC_CCR_PRESC_SHIFT          (18)                        /* Bits 18-21: ADC prescaler */
#define ADC_CCR_PRESC_MASK           (3 << ADC_CCR_PRESC_SHIFT)
#  define ADC_CCR_PRESC_NOT_DIV      (0 << ADC_CCR_PRESC_SHIFT)  /* Input ADC clock not divided */
#  define ADC_CCR_PRESC_DIV2         (1 << ADC_CCR_PRESC_SHIFT)  /* Input ADC clock divided by 2 */
#  define ADC_CCR_PRESC_DIV4         (2 << ADC_CCR_PRESC_SHIFT)  /* Input ADC clock divided by 4 */
#  define ADC_CCR_PRESC_DIV6         (3 << ADC_CCR_PRESC_SHIFT)  /* Input ADC clock divided by 6 */
#  define ADC_CCR_PRESC_DIV8         (4 << ADC_CCR_PRESC_SHIFT)  /* Input ADC clock divided by 8 */
#  define ADC_CCR_PRESC_DIV10        (5 << ADC_CCR_PRESC_SHIFT)  /* Input ADC clock divided by 10 */
#  define ADC_CCR_PRESC_DIV12        (6 << ADC_CCR_PRESC_SHIFT)  /* Input ADC clock divided by 12 */
#  define ADC_CCR_PRESC_DIV16        (7 << ADC_CCR_PRESC_SHIFT)  /* Input ADC clock divided by 16 */
#  define ADC_CCR_PRESC_DIV32        (8 << ADC_CCR_PRESC_SHIFT)  /* Input ADC clock divided by 32 */
#  define ADC_CCR_PRESC_DIV64        (9 << ADC_CCR_PRESC_SHIFT)  /* Input ADC clock divided by 64 */
#  define ADC_CCR_PRESC_DIV128       (10 << ADC_CCR_PRESC_SHIFT) /* Input ADC clock divided by 128 */
#  define ADC_CCR_PRESC_DIV256       (11 << ADC_CCR_PRESC_SHIFT) /* Input ADC clock divided by 256 */
#define ADC_CCR_VREFEN               (1 << 22)                   /* Bit 22: VREFINT enable */
#define ADC_CCR_TSEN                 (1 << 23)                   /* Bit 23: Temperature sensor enable */
#define ADC_CCR_VBATEN               (1 << 24)                   /* Bit 22: VBAT enable */

/* Common regular data register for dual mode */

#ifndef CONFIG_STM32L4_STM32L4X3
#  define ADC_CDR_RDATA_MST_SHIFT    (0)       /* Bits 0-15: Regular data of the master ADC */
#  define ADC_CDR_RDATA_MST_MASK     (0xffff << ADC_CDR_RDATA_MST_SHIFT)
#  define ADC_CDR_RDATA_SLV_SHIFT    (16)      /* Bits 16-31: Regular data of the slave ADC */
#  define ADC_CDR_RDATA_SLV_MASK     (0xffff << ADC_CDR_RDATA_SLV_SHIFT)
#endif

#endif /* __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_ADC_H */
