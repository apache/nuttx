/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_adc_v1l1.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_ADC_V1L1_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_ADC_V1L1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* This is implementation for STM32 ADC IPv1 modified for L1 */

#define HAVE_IP_ADC_V1
#undef HAVE_IP_ADC_V2           /* No ADC IPv2 */
#undef HAVE_BASIC_ADC           /* No basic ADC */
#undef HAVE_ADC_VBAT            /* No VBAT channel */

/* Some families use HSI as clock-source for the ADC (L0, L1).
 * NOTE: F0 use dedicated HSI14 clock which does not belong here.
 */

#ifdef CONFIG_STM32_STM32L15XX
#  define HAVE_ADC_CLOCK_HSI
#endif

/* Power down functions for ADC (L1) */

#ifdef CONFIG_STM32_STM32L15XX
#  define HAVE_ADC_POWERDOWN
#endif

/* Base addresses ***********************************************************/

#define STM32_ADC1_OFFSET            0x0000
#define STM32_ADC2_OFFSET            0x0100
#define STM32_ADC3_OFFSET            0x0200
#define STM32_ADC_CMN_OFFSET         0x0300

#define STM32_ADC1_BASE              (STM32_ADC1_OFFSET + STM32_ADC_BASE) /* ADC1 ADC */
#define STM32_ADC2_BASE              (STM32_ADC2_OFFSET + STM32_ADC_BASE) /* ADC2 ADC */
#define STM32_ADC3_BASE              (STM32_ADC3_OFFSET + STM32_ADC_BASE) /* ADC3 ADC */

#define STM32_ADCCMN_BASE            (STM32_ADC_CMN_OFFSET + STM32_ADC_BASE) /* ADC1, ADC2, ADC3 common */

/* Register Offsets *********************************************************/

#define STM32_ADC_SR_OFFSET          0x0000  /* ADC status register (32-bit) */
#define STM32_ADC_CR1_OFFSET         0x0004  /* ADC control register 1 (32-bit) */
#define STM32_ADC_CR2_OFFSET         0x0008  /* ADC control register 2 (32-bit) */
#define STM32_ADC_SMPR1_OFFSET       0x000c  /* ADC sample time register 1 (32-bit) */
#define STM32_ADC_SMPR2_OFFSET       0x0010  /* ADC sample time register 2 (32-bit) */
#define STM32_ADC_SMPR3_OFFSET       0x0014  /* ADC sample time register 3 (32-bit) */
#define STM32_ADC_JOFR1_OFFSET       0x0018  /* ADC injected channel data offset register 1 (32-bit) */
#define STM32_ADC_JOFR2_OFFSET       0x001c  /* ADC injected channel data offset register 2 (32-bit) */
#define STM32_ADC_JOFR3_OFFSET       0x0020  /* ADC injected channel data offset register 3 (32-bit) */
#define STM32_ADC_JOFR4_OFFSET       0x0024  /* ADC injected channel data offset register 4 (32-bit) */
#define STM32_ADC_HTR_OFFSET         0x0028  /* ADC watchdog high threshold register (32-bit) */
#define STM32_ADC_LTR_OFFSET         0x002c  /* ADC watchdog low threshold register (32-bit) */
#define STM32_ADC_SQR1_OFFSET        0x0030  /* ADC regular sequence register 1 (32-bit) */
#define STM32_ADC_SQR2_OFFSET        0x0034  /* ADC regular sequence register 2 (32-bit) */
#define STM32_ADC_SQR3_OFFSET        0x0038  /* ADC regular sequence register 3 (32-bit) */
#define STM32_ADC_SQR4_OFFSET        0x003c  /* ADC regular sequence register 4 (32-bit) */
#define STM32_ADC_SQR5_OFFSET        0x0040  /* ADC regular sequence register 5 (32-bit) */
#define STM32_ADC_JSQR_OFFSET        0x0044  /* ADC injected sequence register (32-bit) */
#define STM32_ADC_JDR1_OFFSET        0x0048  /* ADC injected data register 1 (32-bit) */
#define STM32_ADC_JDR2_OFFSET        0x004c  /* ADC injected data register 1 (32-bit) */
#define STM32_ADC_JDR3_OFFSET        0x0050  /* ADC injected data register 1 (32-bit) */
#define STM32_ADC_JDR4_OFFSET        0x0054  /* ADC injected data register 1 (32-bit) */
#define STM32_ADC_DR_OFFSET          0x0058  /* ADC regular data register (32-bit) */
#define STM32_ADC_SMPR0_OFFSET       0X005c  /* ADC sample time register 3 (32-bit) */

#define STM32_ADC_CSR_OFFSET         0x0000  /* Common status register */
#define STM32_ADC_CCR_OFFSET         0x0004  /* Common control register */

/* Register Addresses *******************************************************/

#if STM32_NADC > 0
#  define STM32_ADC1_SR              (STM32_ADC1_BASE + STM32_ADC_SR_OFFSET)
#  define STM32_ADC1_CR1             (STM32_ADC1_BASE + STM32_ADC_CR1_OFFSET)
#  define STM32_ADC1_CR2             (STM32_ADC1_BASE + STM32_ADC_CR2_OFFSET)
#  define STM32_ADC1_SMPR1           (STM32_ADC1_BASE + STM32_ADC_SMPR1_OFFSET)
#  define STM32_ADC1_SMPR2           (STM32_ADC1_BASE + STM32_ADC_SMPR2_OFFSET)
#  define STM32_ADC1_SMPR3           (STM32_ADC1_BASE + STM32_ADC_SMPR3_OFFSET)
#  define STM32_ADC1_JOFR1           (STM32_ADC1_BASE + STM32_ADC_JOFR1_OFFSET)
#  define STM32_ADC1_JOFR2           (STM32_ADC1_BASE + STM32_ADC_JOFR2_OFFSET)
#  define STM32_ADC1_JOFR3           (STM32_ADC1_BASE + STM32_ADC_JOFR3_OFFSET)
#  define STM32_ADC1_JOFR4           (STM32_ADC1_BASE + STM32_ADC_JOFR4_OFFSET)
#  define STM32_ADC1_HTR             (STM32_ADC1_BASE + STM32_ADC_HTR_OFFSET)
#  define STM32_ADC1_LTR             (STM32_ADC1_BASE + STM32_ADC_LTR_OFFSET)
#  define STM32_ADC1_SQR1            (STM32_ADC1_BASE + STM32_ADC_SQR1_OFFSET)
#  define STM32_ADC1_SQR2            (STM32_ADC1_BASE + STM32_ADC_SQR2_OFFSET)
#  define STM32_ADC1_SQR3            (STM32_ADC1_BASE + STM32_ADC_SQR3_OFFSET)
#  define STM32_ADC1_SQR4            (STM32_ADC1_BASE + STM32_ADC_SQR4_OFFSET)
#  define STM32_ADC1_SQR5            (STM32_ADC1_BASE + STM32_ADC_SQR5_OFFSET)
#  define STM32_ADC1_JSQR            (STM32_ADC1_BASE + STM32_ADC_JSQR_OFFSET)
#  define STM32_ADC1_JDR1            (STM32_ADC1_BASE + STM32_ADC_JDR1_OFFSET)
#  define STM32_ADC1_JDR2            (STM32_ADC1_BASE + STM32_ADC_JDR2_OFFSET)
#  define STM32_ADC1_JDR3            (STM32_ADC1_BASE + STM32_ADC_JDR3_OFFSET)
#  define STM32_ADC1_JDR4            (STM32_ADC1_BASE + STM32_ADC_JDR4_OFFSET)
#  define STM32_ADC1_DR              (STM32_ADC1_BASE + STM32_ADC_DR_OFFSET)
#  define STM32_ADC1_SMPR0           (STM32_ADC1_BASE + STM32_ADC_SMPR0_OFFSET)
#endif

#if STM32_NADC > 1
#  define STM32_ADC2_SR              (STM32_ADC2_BASE + STM32_ADC_SR_OFFSET)
#  define STM32_ADC2_CR1             (STM32_ADC2_BASE + STM32_ADC_CR1_OFFSET)
#  define STM32_ADC2_CR2             (STM32_ADC2_BASE + STM32_ADC_CR2_OFFSET)
#  define STM32_ADC2_SMPR1           (STM32_ADC2_BASE + STM32_ADC_SMPR1_OFFSET)
#  define STM32_ADC2_SMPR2           (STM32_ADC2_BASE + STM32_ADC_SMPR2_OFFSET)
#  define STM32_ADC2_JOFR1           (STM32_ADC2_BASE + STM32_ADC_JOFR1_OFFSET)
#  define STM32_ADC2_JOFR2           (STM32_ADC2_BASE + STM32_ADC_JOFR2_OFFSET)
#  define STM32_ADC2_JOFR3           (STM32_ADC2_BASE + STM32_ADC_JOFR3_OFFSET)
#  define STM32_ADC2_JOFR4           (STM32_ADC2_BASE + STM32_ADC_JOFR4_OFFSET)
#  define STM32_ADC2_HTR             (STM32_ADC2_BASE + STM32_ADC_HTR_OFFSET)
#  define STM32_ADC2_LTR             (STM32_ADC2_BASE + STM32_ADC_LTR_OFFSET)
#  define STM32_ADC2_SQR1            (STM32_ADC2_BASE + STM32_ADC_SQR1_OFFSET)
#  define STM32_ADC2_SQR2            (STM32_ADC2_BASE + STM32_ADC_SQR2_OFFSET)
#  define STM32_ADC2_SQR3            (STM32_ADC2_BASE + STM32_ADC_SQR3_OFFSET)
#  define STM32_ADC2_JSQR            (STM32_ADC2_BASE + STM32_ADC_JSQR_OFFSET)
#  define STM32_ADC2_JDR1            (STM32_ADC2_BASE + STM32_ADC_JDR1_OFFSET)
#  define STM32_ADC2_JDR2            (STM32_ADC2_BASE + STM32_ADC_JDR2_OFFSET)
#  define STM32_ADC2_JDR3            (STM32_ADC2_BASE + STM32_ADC_JDR3_OFFSET)
#  define STM32_ADC2_JDR4            (STM32_ADC2_BASE + STM32_ADC_JDR4_OFFSET)
#  define STM32_ADC2_DR              (STM32_ADC2_BASE + STM32_ADC_DR_OFFSET)
#endif

#if STM32_NADC > 2
#  define STM32_ADC3_SR              (STM32_ADC3_BASE + STM32_ADC_SR_OFFSET)
#  define STM32_ADC3_CR1             (STM32_ADC3_BASE + STM32_ADC_CR1_OFFSET)
#  define STM32_ADC3_CR2             (STM32_ADC3_BASE + STM32_ADC_CR2_OFFSET)
#  define STM32_ADC3_SMPR1           (STM32_ADC3_BASE + STM32_ADC_SMPR1_OFFSET)
#  define STM32_ADC3_SMPR2           (STM32_ADC3_BASE + STM32_ADC_SMPR2_OFFSET)
#  define STM32_ADC3_JOFR1           (STM32_ADC3_BASE + STM32_ADC_JOFR1_OFFSET)
#  define STM32_ADC3_JOFR2           (STM32_ADC3_BASE + STM32_ADC_JOFR2_OFFSET)
#  define STM32_ADC3_JOFR3           (STM32_ADC3_BASE + STM32_ADC_JOFR3_OFFSET)
#  define STM32_ADC3_JOFR4           (STM32_ADC3_BASE + STM32_ADC_JOFR4_OFFSET)
#  define STM32_ADC3_HTR             (STM32_ADC3_BASE + STM32_ADC_HTR_OFFSET)
#  define STM32_ADC3_LTR             (STM32_ADC3_BASE + STM32_ADC_LTR_OFFSET)
#  define STM32_ADC3_SQR1            (STM32_ADC3_BASE + STM32_ADC_SQR1_OFFSET)
#  define STM32_ADC3_SQR2            (STM32_ADC3_BASE + STM32_ADC_SQR2_OFFSET)
#  define STM32_ADC3_SQR3            (STM32_ADC3_BASE + STM32_ADC_SQR3_OFFSET)
#  define STM32_ADC3_JSQR            (STM32_ADC3_BASE + STM32_ADC_JSQR_OFFSET)
#  define STM32_ADC3_JDR1            (STM32_ADC3_BASE + STM32_ADC_JDR1_OFFSET)
#  define STM32_ADC3_JDR2            (STM32_ADC3_BASE + STM32_ADC_JDR2_OFFSET)
#  define STM32_ADC3_JDR3            (STM32_ADC3_BASE + STM32_ADC_JDR3_OFFSET)
#  define STM32_ADC3_JDR4            (STM32_ADC3_BASE + STM32_ADC_JDR4_OFFSET)
#  define STM32_ADC3_DR              (STM32_ADC3_BASE + STM32_ADC_DR_OFFSET)
#endif

#define STM32_ADC_CSR                (STM32_ADCCMN_BASE + STM32_ADC_CSR_OFFSET)
#define STM32_ADC_CCR                (STM32_ADCCMN_BASE + STM32_ADC_CCR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* ADC status register */

#define ADC_SR_AWD                   (1 << 0)  /* Bit 0 : Analog watchdog flag */
#define ADC_SR_EOC                   (1 << 1)  /* Bit 1 : End of conversion */
#define ADC_SR_JEOC                  (1 << 2)  /* Bit 2 : Injected channel end of conversion */
#define ADC_SR_JSTRT                 (1 << 3)  /* Bit 3 : Injected channel Start flag */
#define ADC_SR_STRT                  (1 << 4)  /* Bit 4 : Regular channel Start flag */
#define ADC_SR_OVR                   (1 << 5)  /* Bit 5 : Overrun */
#define ADC_SR_ADONS                 (1 << 6)  /* Bit 6 : ADC ON status. Set and cleared by HW */
#define ADC_SR_RCNR                  (1 << 8)  /* Bit 8 : Regular channel not ready. Set and cleared by HW */
#define ADC_SR_JCNR                  (1 << 9)  /* Bit 9 : Injected channel not ready. Set and cleared by HW */

/* ADC control register 1 */

#define ADC_CR1_AWDCH_SHIFT          (0)       /* Bits 4-0: Analog watchdog channel select bits */
#define ADC_CR1_AWDCH_MASK           (0x1f << ADC_CR1_AWDCH_SHIFT)

#define ADC_CR1_EOCIE                (1 << 5)  /* Bit 5: Interrupt enable for EOC */
#define ADC_CR1_AWDIE                (1 << 6)  /* Bit 6: Analog Watchdog interrupt enable */
#define ADC_CR1_JEOCIE               (1 << 7)  /* Bit 7: Interrupt enable for injected channels */
#define ADC_CR1_SCAN                 (1 << 8)  /* Bit 8: Scan mode */
#define ADC_CR1_AWDSGL               (1 << 9)  /* Bit 9: Enable the watchdog on a single channel in scan mode */
#define ADC_CR1_JAUTO                (1 << 10) /* Bit 10: Automatic Injected Group conversion */
#define ADC_CR1_DISCEN               (1 << 11) /* Bit 11: Discontinuous mode on regular channels */
#define ADC_CR1_JDISCEN              (1 << 12) /* Bit 12: Discontinuous mode on injected channels */

#define ADC_CR1_DISCNUM_SHIFT        (13)      /* Bits 15-13: Discontinuous mode channel count */
#define ADC_CR1_DISCNUM_MASK         (0x07 << ADC_CR1_DISCNUM_SHIFT)

#define ADC_CR1_PDD                  (1 << 16) /* Bit 16 : Power down during the delay phase. This bit must be written only when ADON=0 */
#define ADC_CR1_PDI                  (1 << 17) /* Bit 17 : Power down during the idle phase. This bit must
                                                *          be written only when ADON=0 */
#define ADC_CR1_JAWDEN               (1 << 22) /* Bit 22: Analog watchdog enable on injected channels */
#define ADC_CR1_AWDEN                (1 << 23) /* Bit 23: Analog watchdog enable on regular channels */

#define ADC_CR1_RES_SHIFT            (24)      /* Bits 24-25: Resolution */
#define ADC_CR1_RES_MASK             (3 << ADC_CR1_RES_SHIFT)
#  define ADC_CR1_RES_12BIT          (0 << ADC_CR1_RES_SHIFT) /* 12 ADCCLK cycles */
#  define ADC_CR1_RES_10BIT          (1 << ADC_CR1_RES_SHIFT) /* 11 ADCCLK cycles */
#  define ADC_CR1_RES_8BIT           (2 << ADC_CR1_RES_SHIFT) /* 9 ADCCLK cycles */
#  define ADC_CR1_RES_6BIT           (3 << ADC_CR1_RES_SHIFT) /* 7 ADCCLK cycles */

#define ADC_CR1_OVRIE                (1 << 26) /* Bit 26: Overrun interrupt enable */
#define ADC_CR1_RESERVED             (0xfb3f0000)

/* ADC control register 2 */

#define ADC_CR2_ADON                 (1 << 0)  /* Bit 0: A/D Converter ON / OFF */
#define ADC_CR2_CONT                 (1 << 1)  /* Bit 1: Continuous Conversion */
#define ADC_CR2_CFG                  (1 << 2)  /* Bit 2 : ADC configuration. This bit must be modified only when no
                                                * conversion is on going.  This bit is available in high and medium +
                                                * density devices only.
                                                */

#define ADC_CR2_DELS_SHIFT           (4)       /* Bits 2-0: Delay selection */
#define ADC_CR2_DELS_MASK            (0x07 << ADC_CR2_DELS_SHIFT)
#define ADC_CR2_DELS_NODEL           (0x0 << ADC_CR2_DELS_SHIFT)  /* No delay */
#define ADC_CR2_DELS_TILLRD          (0x01 << ADC_CR2_DELS_SHIFT) /* Until the converted data have been read */
#define ADC_CR2_DELS_APB7            (0x02 << ADC_CR2_DELS_SHIFT) /* 7 APB clock cycles after the end of conversion */
#define ADC_CR2_DELS_APB15           (0x03 << ADC_CR2_DELS_SHIFT) /* 15 APB clock cycles after the end of conversion */
#define ADC_CR2_DELS_APB31           (0x04 << ADC_CR2_DELS_SHIFT) /* 31 APB clock cycles after the end of conversion */
#define ADC_CR2_DELS_APB63           (0x05 << ADC_CR2_DELS_SHIFT) /* 63 APB clock cycles after the end of conversion */
#define ADC_CR2_DELS_APB127          (0x06 << ADC_CR2_DELS_SHIFT) /* 127 APB clock cycles after the end of conversion */
#define ADC_CR2_DELS_APB255          (0x07 << ADC_CR2_DELS_SHIFT) /* 255 APB clock cycles after the end of conversion */

#define ADC_CR2_DMA                  (1 << 8)  /* Bit 8: Direct Memory access mode */
#define ADC_CR2_DDS                  (1 << 9)  /* Bit 9: DMA disable selection (for single ADC mode) */
#define ADC_CR2_EOCS                 (1 << 10) /* Bit 10: End of conversion selection */
#define ADC_CR2_ALIGN                (1 << 11) /* Bit 11: Data Alignment */
                                               /* Bits 12-15: Reserved */
#define ADC_CR2_JEXTSEL_SHIFT        (16)      /* Bits 16-19: External event select for injected group */
#define ADC_CR2_JEXTSEL_MASK         (0x0F << ADC_CR2_JEXTSEL_SHIFT)
#  define ADC_CR2_JEXTSEL_T9CC1      (0x00 << ADC_CR2_JEXTSEL_SHIFT) /* 0000: Timer 9 CC1 event */
#  define ADC_CR2_JEXTSEL_T9TRGO     (0x01 << ADC_CR2_JEXTSEL_SHIFT) /* 0001: Timer 9 TRGO event */
#  define ADC_CR2_JEXTSEL_T2TRGO     (0x02 << ADC_CR2_JEXTSEL_SHIFT) /* 0010: Timer 2 TRGO event*/
#  define ADC_CR2_JEXTSEL_T2CC1      (0x03 << ADC_CR2_JEXTSEL_SHIFT) /* 0011: Timer 2 CC1 event */
#  define ADC_CR2_JEXTSEL_T3CC4      (0x04 << ADC_CR2_JEXTSEL_SHIFT) /* 0100: Timer 3 CC4 event */
#  define ADC_CR2_JEXTSEL_T4TRGO     (0x05 << ADC_CR2_JEXTSEL_SHIFT) /* 0101: Timer 4 TRGO event */
#  define ADC_CR2_JEXTSEL_T4CC1      (0x06 << ADC_CR2_JEXTSEL_SHIFT) /* 0110: Timer 4 CC1 event */
#  define ADC_CR2_JEXTSEL_T4CC2      (0x07 << ADC_CR2_JEXTSEL_SHIFT) /* 0111: Timer 4 CC2 event */
#  define ADC_CR2_JEXTSEL_T4CC3      (0x08 << ADC_CR2_JEXTSEL_SHIFT) /* 1000: Timer 4 CC3 event */
#  define ADC_CR2_JEXTSEL_T10CC1     (0x09 << ADC_CR2_JEXTSEL_SHIFT) /* 1001: Timer 10 CC1 event */
#  define ADC_CR2_JEXTSEL_T7TRGO     (0x0A << ADC_CR2_JEXTSEL_SHIFT) /* 1010: Timer 7 TRGO event */
#  define ADC_CR2_JEXTSEL_EXTI15     (0x0F << ADC_CR2_JEXTSEL_SHIFT) /* 1111: EXTI line 15 */

#define ADC_CR2_JEXTEN_SHIFT         (20)      /* Bits 20-21: External trigger enable for injected channels */
#define ADC_CR2_JEXTEN_MASK          (3 << ADC_CR2_JEXTEN_SHIFT)
#  define ADC_CR2_JEXTEN_NONE        (0 << ADC_CR2_JEXTEN_SHIFT) /* 00: Trigger detection disabled */
#  define ADC_CR2_JEXTEN_RISING      (1 << ADC_CR2_JEXTEN_SHIFT) /* 01: Trigger detection on the rising edge */
#  define ADC_CR2_JEXTEN_FALLING     (2 << ADC_CR2_JEXTEN_SHIFT) /* 10: Trigger detection on the falling edge */
#  define ADC_CR2_JEXTEN_BOTH        (3 << ADC_CR2_JEXTEN_SHIFT) /* 11: Trigger detection on both the rising and falling edges */

#define ADC_CR2_JSWSTART             (1 << 22) /* Bit 22: Start Conversion of injected channels */
                                               /* Bit 23: Reserved, must be kept at reset value. */
#define ADC_CR2_EXTSEL_SHIFT         (24)      /* Bits 24-27: External Event Select for regular group */
#define ADC_CR2_EXTSEL_MASK          (0x0F << ADC_CR2_EXTSEL_SHIFT)
#  define ADC_CR2_EXTSEL_T9CC2       (0x00 << ADC_CR2_EXTSEL_SHIFT) /* 0000: Timer 9 CC2 event */
#  define ADC_CR2_EXTSEL_T9TRGO      (0x01 << ADC_CR2_EXTSEL_SHIFT) /* 0001: Timer 9 TRGO event */
#  define ADC_CR2_EXTSEL_T2CC3       (0x02 << ADC_CR2_EXTSEL_SHIFT) /* 0010: Timer 2 CC3 event */
#  define ADC_CR2_EXTSEL_T2CC2       (0x03 << ADC_CR2_EXTSEL_SHIFT) /* 0011: Timer 2 CC2 event */
#  define ADC_CR2_EXTSEL_T3TRGO      (0x04 << ADC_CR2_EXTSEL_SHIFT) /* 0100: Timer 3 TRGO event */
#  define ADC_CR2_EXTSEL_T4CC4       (0x05 << ADC_CR2_EXTSEL_SHIFT) /* 0101: Timer 4 CC4 event */
#  define ADC_CR2_EXTSEL_T2TRGO      (0x06 << ADC_CR2_EXTSEL_SHIFT) /* 0110: Timer 2 TRGO event */
#  define ADC_CR2_EXTSEL_T3CC1       (0x07 << ADC_CR2_EXTSEL_SHIFT) /* 0111: Timer 3 CC1 event */
#  define ADC_CR2_EXTSEL_T3CC3       (0x08 << ADC_CR2_EXTSEL_SHIFT) /* 1000: Timer 3 CC3 event */
#  define ADC_CR2_EXTSEL_T4TRGO      (0x09 << ADC_CR2_EXTSEL_SHIFT) /* 1001: Timer 4 TRGO event */
#  define ADC_CR2_EXTSEL_T6TRGO      (0x0A << ADC_CR2_EXTSEL_SHIFT) /* 1010: Timer 6 TRGO event */
#  define ADC_CR2_EXTSEL_EXTI11      (0x0F << ADC_CR2_EXTSEL_SHIFT) /* 1111: EXTI line 11 */

#define ADC_CR2_EXTEN_SHIFT          (28)      /* Bits 28-29: External trigger enable for regular channels */
#define ADC_CR2_EXTEN_MASK           (3 << ADC_CR2_EXTEN_SHIFT)
#  define ADC_CR2_EXTEN_NONE         (0 << ADC_CR2_EXTEN_SHIFT) /* 00: Trigger detection disabled */
#  define ADC_CR2_EXTEN_RISING       (1 << ADC_CR2_EXTEN_SHIFT) /* 01: Trigger detection on the rising edge */
#  define ADC_CR2_EXTEN_FALLING      (2 << ADC_CR2_EXTEN_SHIFT) /* 10: Trigger detection on the falling edge */
#  define ADC_CR2_EXTEN_BOTH         (3 << ADC_CR2_EXTEN_SHIFT) /* 11: Trigger detection on both the rising and falling edges */

#define ADC_CR2_SWSTART              (1 << 30) /* Bit 30: Start Conversion of regular channels */
#define ADC_CR2_RESERVED             (0x8080f0fc)

/* ADC sample time register 1 */

#define ADC_SMPR_4                   0         /* 000: 3 cycles */
#define ADC_SMPR_9                   1         /* 001: 9 cycles */
#define ADC_SMPR_16                  2         /* 010: 16 cycles */
#define ADC_SMPR_24                  3         /* 011: 24 cycles */
#define ADC_SMPR_48                  4         /* 100: 48 cycles */
#define ADC_SMPR_96                  5         /* 101: 96 cycles */
#define ADC_SMPR_192                 6         /* 110: 192 cycles */
#define ADC_SMPR_384                 7         /* 111: 384 cycles */

#define ADC_SMPR1_SMP20_SHIFT        (0)       /* Bits 0-2: Channel 20 Sample time selection */
#define ADC_SMPR1_SMP20_MASK         (7 << ADC_SMPR1_SMP20_SHIFT)
#define ADC_SMPR1_SMP21_SHIFT        (3)       /* Bits 3-5: Channel 21 Sample time selection */
#define ADC_SMPR1_SMP21_MASK         (7 << ADC_SMPR1_SMP21_SHIFT)
#define ADC_SMPR1_SMP22_SHIFT        (6)       /* Bits 6-8: Channel 22 Sample time selection */
#define ADC_SMPR1_SMP22_MASK         (7 << ADC_SMPR1_SMP22_SHIFT)
#define ADC_SMPR1_SMP23_SHIFT        (9)       /* Bits 9-11: Channel 23 Sample time selection */
#define ADC_SMPR1_SMP23_MASK         (7 << ADC_SMPR1_SMP23_SHIFT)
#define ADC_SMPR1_SMP24_SHIFT        (12)      /* Bits 12-14: Channel 24 Sample time selection */
#define ADC_SMPR1_SMP24_MASK         (7 << ADC_SMPR1_SMP24_SHIFT)
#define ADC_SMPR1_SMP25_SHIFT        (15)      /* Bits 15-17: Channel 25 Sample time selection */
#define ADC_SMPR1_SMP25_MASK         (7 << ADC_SMPR1_SMP25_SHIFT)
#define ADC_SMPR1_SMP26_SHIFT        (18)      /* Bits 18-20: Channel 26 Sample time selection */
#define ADC_SMPR1_SMP26_MASK         (7 << ADC_SMPR1_SMP26_SHIFT)
#define ADC_SMPR1_SMP27_SHIFT        (21)      /* Bits 21-23: Channel 27 Sample time selection */
#define ADC_SMPR1_SMP27_MASK         (7 << ADC_SMPR1_SMP27_SHIFT)
#define ADC_SMPR1_SMP28_SHIFT        (24)      /* Bits 24-26: Channel 28 Sample time selection */
#define ADC_SMPR1_SMP28_MASK         (7 << ADC_SMPR1_SMP28_SHIFT)
#define ADC_SMPR1_SMP29_SHIFT        (27)      /* Bits 27-29: Channel 29 Sample time selection */
#define ADC_SMPR1_SMP29_MASK         (7 << ADC_SMPR1_SMP29_SHIFT)

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
#define ADC_SMPR2_SMP19_SHIFT        (27)      /* Bits 27-29: Channel 18 Sample time selection */
#define ADC_SMPR2_SMP19_MASK         (7 << ADC_SMPR2_SMP19_SHIFT)

/* ADC sample time register 3 */

#define ADC_SMPR3_SMP0_SHIFT         (0)       /* Bits 2-0: Channel 0 Sample time selection */
#define ADC_SMPR3_SMP0_MASK          (7 << ADC_SMPR3_SMP0_SHIFT)
#define ADC_SMPR3_SMP1_SHIFT         (3)       /* Bits 5-3: Channel 1 Sample time selection */
#define ADC_SMPR3_SMP1_MASK          (7 << ADC_SMPR3_SMP1_SHIFT)
#define ADC_SMPR3_SMP2_SHIFT         (6)       /* Bits 8-6: Channel 2 Sample time selection */
#define ADC_SMPR3_SMP2_MASK          (7 << ADC_SMPR3_SMP2_SHIFT)
#define ADC_SMPR3_SMP3_SHIFT         (9)       /* Bits 11-9: Channel 3 Sample time selection */
#define ADC_SMPR3_SMP3_MASK          (7 << ADC_SMPR3_SMP3_SHIFT)
#define ADC_SMPR3_SMP4_SHIFT         (12)      /* Bits 14-12: Channel 4 Sample time selection */
#define ADC_SMPR3_SMP4_MASK          (7 << ADC_SMPR3_SMP4_SHIFT)
#define ADC_SMPR3_SMP5_SHIFT         (15)      /* Bits 17-15: Channel 5 Sample time selection */
#define ADC_SMPR3_SMP5_MASK          (7 << ADC_SMPR3_SMP5_SHIFT)
#define ADC_SMPR3_SMP6_SHIFT         (18)      /* Bits 20-18: Channel 6 Sample time selection */
#define ADC_SMPR3_SMP6_MASK          (7 << ADC_SMPR3_SMP6_SHIFT)
#define ADC_SMPR3_SMP7_SHIFT         (21)      /* Bits 23-21: Channel 7 Sample time selection */
#define ADC_SMPR3_SMP7_MASK          (7 << ADC_SMPR3_SMP7_SHIFT)
#define ADC_SMPR3_SMP8_SHIFT         (24)      /* Bits 26-24: Channel 8 Sample time selection */
#define ADC_SMPR3_SMP8_MASK          (7 << ADC_SMPR3_SMP8_SHIFT)
#define ADC_SMPR3_SMP9_SHIFT         (27)      /* Bits 29-27: Channel 9 Sample time selection */
#define ADC_SMPR3_SMP9_MASK          (7 << ADC_SMPR3_SMP9_SHIFT)

/* ADC sample time register 0 */

#define ADC_SMPR0_SMP30_SHIFT        (0)       /* Bits 2-0: Channel 30 Sample time selection */
#define ADC_SMPR0_SMP30_MASK         (7 << ADC_SMPR3_SMP30_SHIFT)
#define ADC_SMPR0_SMP31_SHIFT        (3)       /* Bits 5-3: Channel 31 Sample time selection */
#define ADC_SMPR0_SMP31_MASK         (7 << ADC_SMPR3_SMP31_SHIFT)

/* ADC injected channel data offset register 1-4 */

#define ADC_JOFR_SHIFT               (0)       /* Bits 11-0: Data offset for injected channel x */
#define ADC_JOFR_MASK                (0x0fff << ADC_JOFR_SHIFT)

/* ADC watchdog high threshold register */

#define ADC_HTR_SHIFT                (0)       /* Bits 11-0: Analog watchdog high threshold */
#define ADC_HTR_MASK                 (0x0fff << ADC_HTR_SHIFT)

/* ADC watchdog low threshold register */

#define ADC_LTR_SHIFT                (0)       /* Bits 11-0: Analog watchdog low threshold */
#define ADC_LTR_MASK                 (0x0fff << ADC_LTR_SHIFT)

/* ADC regular sequence register 1 */

#define ADC_SQR1_SQ25_SHIFT          (0)       /* Bits 4-0: 25th conversion in regular sequence */
#define ADC_SQR1_SQ25_MASK           (0x1f << ADC_SQR1_SQ25_SHIFT)
#define ADC_SQR1_SQ26_SHIFT          (5)       /* Bits 9-5: 26th conversion in regular sequence */
#define ADC_SQR1_SQ26_MASK           (0x1f << ADC_SQR1_SQ26_SHIFT)
#define ADC_SQR1_SQ27_SHIFT          (10)      /* Bits 14-10: 27th conversion in regular sequence */
#define ADC_SQR1_SQ27_MASK           (0x1f << ADC_SQR1_SQ27_SHIFT)
#define ADC_SQR1_SQ28_SHIFT          (15)      /* Bits 19-15: 28th conversion in regular sequence */
#define ADC_SQR1_SQ28_MASK           (0x1f << ADC_SQR1_SQ28_SHIFT)
#define ADC_SQR1_L_SHIFT             (20)      /* Bits 24-20: Regular channel sequence length */
#define ADC_SQR1_L_MASK              (0x1f << ADC_SQR1_L_SHIFT)
#define ADC_SQR1_RESERVED            (0xff000000)
#define ADC_SQR1_FIRST               (25)
#define ADC_SQR1_LAST                (28)
#define ADC_SQR1_SQ_OFFSET           (0)

/* ADC regular sequence register 2 */

#define ADC_SQR2_SQ19_SHIFT          (0)       /* Bits 4-0: 19th conversion in regular sequence */
#define ADC_SQR2_SQ19_MASK           (0x1f << ADC_SQR2_SQ19_SHIFT)
#define ADC_SQR2_SQ20_SHIFT          (5)       /* Bits 9-5: 20th conversion in regular sequence */
#define ADC_SQR2_SQ20_MASK           (0x1f << ADC_SQR2_SQ20_SHIFT)
#define ADC_SQR2_SQ21_SHIFT          (10)      /* Bits 14-10: 21th conversion in regular sequence */
#define ADC_SQR2_SQ21_MASK           (0x1f << ADC_SQR2_SQ21_SHIFT)
#define ADC_SQR2_SQ22_SHIFT          (15)      /* Bits 19-15: 22th conversion in regular sequence */
#define ADC_SQR2_SQ22_MASK           (0x1f << ADC_SQR2_SQ22_SHIFT)
#define ADC_SQR2_SQ23_SHIFT          (20)      /* Bits 24-20: 23th conversion in regular sequence */
#define ADC_SQR2_SQ23_MASK           (0x1f << ADC_SQR2_SQ23_SHIFT )
#define ADC_SQR2_SQ24_SHIFT          (25)      /* Bits 29-25: 24th conversion in regular sequence */
#define ADC_SQR2_SQ24_MASK           (0x1f << ADC_SQR2_SQ24_SHIFT)
#define ADC_SQR2_RESERVED            (0xc0000000)
#define ADC_SQR2_FIRST               (19)
#define ADC_SQR2_LAST                (24)
#define ADC_SQR2_SQ_OFFSET           (0)

/* ADC regular sequence register 3 */

#define ADC_SQR3_SQ13_SHIFT          (0)       /* Bits 4-0: 13th conversion in regular sequence */
#define ADC_SQR3_SQ13_MASK           (0x1f << ADC_SQR3_SQ13_SHIFT)
#define ADC_SQR3_SQ14_SHIFT          (5)       /* Bits 9-5: 14th conversion in regular sequence */
#define ADC_SQR3_SQ14_MASK           (0x1f << ADC_SQR3_SQ14_SHIFT)
#define ADC_SQR3_SQ15_SHIFT          (10)      /* Bits 14-10: 15th conversion in regular sequence */
#define ADC_SQR3_SQ15_MASK           (0x1f << ADC_SQR3_SQ15_SHIFT)
#define ADC_SQR3_SQ16_SHIFT          (15)      /* Bits 19-15: 16th conversion in regular sequence */
#define ADC_SQR3_SQ16_MASK           (0x1f << ADC_SQR3_SQ16_SHIFT)
#define ADC_SQR3_SQ17_SHIFT          (20)      /* Bits 24-20: 17th conversion in regular sequence */
#define ADC_SQR3_SQ17_MASK           (0x1f << ADC_SQR3_SQ17_SHIFT )
#define ADC_SQR3_SQ18_SHIFT          (25)      /* Bits 29-25: 18th conversion in regular sequence */
#define ADC_SQR3_SQ18_MASK           (0x1f << ADC_SQR3_SQ18_SHIFT)
#define ADC_SQR3_RESERVED            (0xc0000000)
#define ADC_SQR3_FIRST               (13)
#define ADC_SQR3_LAST                (18)
#define ADC_SQR3_SQ_OFFSET           (0)

/* ADC regular sequence register 4 */

#define ADC_SQR4_SQ7_SHIFT           (0)       /* Bits 4-0: 7th conversion in regular sequence */
#define ADC_SQR4_SQ7_MASK            (0x1f << ADC_SQR4_SQ7_SHIFT)
#define ADC_SQR4_SQ8_SHIFT           (5)       /* Bits 9-5: 8th conversion in regular sequence */
#define ADC_SQR4_SQ8_MASK            (0x1f << ADC_SQR4_SQ8_SHIFT)
#define ADC_SQR4_SQ9_SHIFT           (10)      /* Bits 14-10: 9th conversion in regular sequence */
#define ADC_SQR4_SQ9_MASK            (0x1f << ADC_SQR4_SQ9_SHIFT)
#define ADC_SQR4_SQ10_SHIFT          (15)      /* Bits 19-15: 10th conversion in regular sequence */
#define ADC_SQR4_SQ10_MASK           (0x1f << ADC_SQR4_SQ10_SHIFT)
#define ADC_SQR4_SQ11_SHIFT          (20)      /* Bits 24-20: 11th conversion in regular sequence */
#define ADC_SQR4_SQ11_MASK           (0x1f << ADC_SQR4_SQ11_SHIFT )
#define ADC_SQR4_SQ12_SHIFT          (25)      /* Bits 29-25: 12th conversion in regular sequence */
#define ADC_SQR4_SQ12_MASK           (0x1f << ADC_SQR4_SQ12_SHIFT)
#define ADC_SQR4_RESERVED            (0xc0000000)
#define ADC_SQR4_FIRST               (7)
#define ADC_SQR4_LAST                (12)
#define ADC_SQR4_SQ_OFFSET           (0)

/* ADC regular sequence register 5 */

#define ADC_SQR5_SQ1_SHIFT           (0)       /* Bits 4-0: 1st conversion in regular sequence */
#define ADC_SQR5_SQ1_MASK            (0x1f << ADC_SQR5_SQ1_SHIFT)
#define ADC_SQR5_SQ2_SHIFT           (5)       /* Bits 9-5: 2nd conversion in regular sequence */
#define ADC_SQR5_SQ2_MASK            (0x1f << ADC_SQR5_SQ2_SHIFT)
#define ADC_SQR5_SQ3_SHIFT           (10)      /* Bits 14-10: 3rd conversion in regular sequence */
#define ADC_SQR5_SQ3_MASK            (0x1f << ADC_SQR5_SQ3_SHIFT)
#define ADC_SQR5_SQ4_SHIFT           (15)      /* Bits 19-15: 4th conversion in regular sequence */
#define ADC_SQR5_SQ4_MASK            (0x1f << ADC_SQR5_SQ4_SHIFT)
#define ADC_SQR5_SQ5_SHIFT           (20)      /* Bits 24-20: 5th conversion in regular sequence */
#define ADC_SQR5_SQ5_MASK            (0x1f << ADC_SQR5_SQ5_SHIFT )
#define ADC_SQR5_SQ6_SHIFT           (25)      /* Bits 29-25: 6th conversion in regular sequence */
#define ADC_SQR5_SQ6_MASK            (0x1f << ADC_SQR5_SQ6_SHIFT)
#define ADC_SQR5_RESERVED            (0xc0000000)
#define ADC_SQR5_FIRST               (1)
#define ADC_SQR5_LAST                (6)
#define ADC_SQR5_SQ_OFFSET           (0)

/* Offset between SQ bits */

#define ADC_SQ_OFFSET                (5)

/* ADC injected sequence register */

#define ADC_JSQR_JSQ1_SHIFT          (0)       /* Bits 4-0: 1st conversion in injected sequence */
#define ADC_JSQR_JSQ1_MASK           (0x1f << ADC_JSQR_JSQ1_SHIFT)
#define ADC_JSQR_JSQ2_SHIFT          (5)       /* Bits 9-5: 2nd conversion in injected sequence */
#define ADC_JSQR_JSQ2_MASK           (0x1f << ADC_JSQR_JSQ2_SHIFT)
#define ADC_JSQR_JSQ3_SHIFT          (10)      /* Bits 14-10: 3rd conversion in injected sequence */
#define ADC_JSQR_JSQ3_MASK           (0x1f << ADC_JSQR_JSQ3_SHIFT)
#define ADC_JSQR_JSQ4_SHIFT          (15)      /* Bits 19-15: 4th conversion in injected sequence */
#define ADC_JSQR_JSQ4_MASK           (0x1f << ADC_JSQR_JSQ4_SHIFT)
#define ADC_JSQR_JSQ_SHIFT           (5)       /* Shift between JSQx bits */
#define ADC_JSQR_JL_SHIFT            (20)      /* Bits 21-20: Injected Sequence length */
#define ADC_JSQR_JL_MASK             (3 << ADC_JSQR_JL_SHIFT)
#  define ADC_JSQR_JL(n)             (((n)-1) << ADC_JSQR_JL_SHIFT) /* n=1..4 */

/* ADC injected data register 1-4 */

#define ADC_JDR_JDATA_SHIFT          (0)       /* Bits 15-0: Injected data */
#define ADC_JDR_JDATA_MASK           (0xffff << ADC_JDR_JDATA_SHIFT)

/* ADC regular data register */

#define ADC_DR_RDATA_SHIFT           (0)       /* Bits 15-0 Regular data */
#define ADC_DR_RDATA_MASK            (0xffff << ADC_DR_RDATA_SHIFT)

/* Common status register */

#define ADC_CSR_AWD1                 (1 << 0)  /* Bit 0:  Analog watchdog flag of ADC1   (copy of AWD in ADC1_SR) */
#define ADC_CSR_EOC1                 (1 << 1)  /* Bit 1:  End of conversion of ADC1   (copy of EOC in ADC1_SR) */
#define ADC_CSR_JEOC1                (1 << 2)  /* Bit 2:  Injected channel end of conversion of ADC1   (copy of JEOC in ADC1_SR) */
#define ADC_CSR_JSTRT1               (1 << 3)  /* Bit 3:  Injected channel Start flag of ADC1   (copy of JSTRT in ADC1_SR) */
#define ADC_CSR_STRT1                (1 << 4)  /* Bit 4:  Regular channel Start flag of ADC1   (copy of STRT in ADC1_SR) */
#define ADC_CSR_OVR1                 (1 << 5)  /* Bit 5:  Overrun flag of ADC1   (copy of OVR in ADC1_SR) */
                                               /* Bits 6-7: Reserved, must be kept at reset value. */
#define ADC_CSR_ADONS1               (1 << 6)  /* Bit 6: ADON Status of ADC1. This bit is a copy of the ADONS bit in the ADC_SR register. */

/* Common control register */

/*                                                Bits 15-0: Reserved,
 * must be kept at reset value
 */

#define ADC_CCR_ADCPRE_SHIFT         (16)      /* Bits 16-17: ADC prescaler */
#define ADC_CCR_ADCPRE_MASK          (3 << ADC_CCR_ADCPRE_SHIFT)
#  define ADC_CCR_ADCPRE_DIV1        (0 << ADC_CCR_ADCPRE_SHIFT) /* HSI divided by 1 */
#  define ADC_CCR_ADCPRE_DIV2        (1 << ADC_CCR_ADCPRE_SHIFT) /* HSI divided by 2 */
#  define ADC_CCR_ADCPRE_DIV4        (2 << ADC_CCR_ADCPRE_SHIFT) /* HSI divided by 4 */
                                                                 /* 11: Reserved */

/*                                                Bits 22-18: Reserved,
 * must be kept at reset value
 */
#define ADC_CCR_TSVREFE              (1 << 23) /* Bit 23: Temperature sensor and VREFINT enable */
                                               /* Bits 31-24: Reserved, must be kept at reset value */

/* Data register for dual and triple modes
 *(32-bit data with no named fields)
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_ADC_V1L1_H */
