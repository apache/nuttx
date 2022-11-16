/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_adc.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_ADC_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* General definitions ******************************************************/

#if defined(ATSAMA5D3) || defined (ATSAMA5D2)
#  define SAM_ADC_NCHANNELS          12     /* 12 ADC Channels */
#elif defined(ATSAMA5D4)
#  define SAM_ADC_NCHANNELS          5      /* 5 ADC Channels */
#endif

#define SAM_ADC_MAXPERCLK          66000000 /* Maximum peripheral clock frequency */
#define SAM_ADC_CLOCKMAX           20000000 /* Maximum ADC Clock Frequency (Hz) */

/* ADC register offsets *****************************************************/

#define SAM_ADC_CR_OFFSET          0x0000 /* Control Register */
#define SAM_ADC_MR_OFFSET          0x0004 /* Mode Register */
#define SAM_ADC_SEQR1_OFFSET       0x0008 /* Channel Sequence Register 1 */
#define SAM_ADC_SEQR2_OFFSET       0x000c /* Channel Sequence Register 2 */
#define SAM_ADC_CHER_OFFSET        0x0010 /* Channel Enable Register */
#define SAM_ADC_CHDR_OFFSET        0x0014 /* Channel Disable Register */
#define SAM_ADC_CHSR_OFFSET        0x0018 /* Channel Status Register */
                                          /* 0x001c Reserved */
#define SAM_ADC_LCDR_OFFSET        0x0020 /* Last Converted Data Register */
#define SAM_ADC_IER_OFFSET         0x0024 /* Interrupt Enable Register */
#define SAM_ADC_IDR_OFFSET         0x0028 /* Interrupt Disable Register */
#define SAM_ADC_IMR_OFFSET         0x002c /* Interrupt Mask Register */
#define SAM_ADC_ISR_OFFSET         0x0030 /* Interrupt Status Register */
                                          /* 0x0034-38 Reserved */
#define SAM_ADC_OVER_OFFSET        0x003c /* Overrun Status Register */
#define SAM_ADC_EMR_OFFSET         0x0040 /* Extended Mode Register */
#define SAM_ADC_CWR_OFFSET         0x0044 /* Compare Window Register */

#ifdef ATSAMA5D3
#  define SAM_ADC_CGR_OFFSET       0x0048 /* Channel Gain Register */
#endif

#define SAM_ADC_COR_OFFSET         0x004c /* Channel Offset Register */

#define SAM_ADC_CDR_OFFSET(n)      (0x0050+((n)<<2))
#define SAM_ADC_CDR0_OFFSET        0x0050 /* Channel Data Register 0 */
#define SAM_ADC_CDR1_OFFSET        0x0054 /* Channel Data Register 1 */
#define SAM_ADC_CDR2_OFFSET        0x0058 /* Channel Data Register 2 */
#define SAM_ADC_CDR3_OFFSET        0x005c /* Channel Data Register 3 */
#define SAM_ADC_CDR4_OFFSET        0x0060 /* Channel Data Register 4 */

#if defined (ATSAMA5D3) || defined (ATSAMA5D2)
#  define SAM_ADC_CDR5_OFFSET      0x0064 /* Channel Data Register 5 */
#  define SAM_ADC_CDR6_OFFSET      0x0068 /* Channel Data Register 6 */
#  define SAM_ADC_CDR7_OFFSET      0x006c /* Channel Data Register 7 */
#  define SAM_ADC_CDR8_OFFSET      0x0070 /* Channel Data Register 8 */
#  define SAM_ADC_CDR9_OFFSET      0x0074 /* Channel Data Register 9 */
#  define SAM_ADC_CDR10_OFFSET     0x0078 /* Channel Data Register 10 */
#  define SAM_ADC_CDR11_OFFSET     0x007c /* Channel Data Register 11 */
#endif
                                          /* 0x0080-90 Reserved */
#define SAM_ADC_ACR_OFFSET         0x0094 /* Analog Control Register */
                                          /* 0x0098-ac Reserved */
#define SAM_ADC_TSMR_OFFSET        0x00b0 /* Touchscreen Mode Register */
#define SAM_ADC_XPOSR_OFFSET       0x00b4 /* Touchscreen X Position Register */
#define SAM_ADC_YPOSR_OFFSET       0x00b8 /* Touchscreen Y Position Register */
#define SAM_ADC_PRESSR_OFFSET      0x00bc /* Touchscreen Pressure Register */
#define SAM_ADC_TRGR_OFFSET        0x00c0 /* Trigger Register */
                                          /* 0x00c4-e0 Reserved */
#define SAM_ADC_WPMR_OFFSET        0x00e4 /* Write Protect Mode Register */
#define SAM_ADC_WPSR_OFFSET        0x00e8 /* Write Protect Status Register */
                                          /* 0x00ec-fc Reserved */

/* ADC register addresses ***************************************************/

#define SAM_ADC_CR                 (SAM_TSADC_VBASE+SAM_ADC_CR_OFFSET)
#define SAM_ADC_MR                 (SAM_TSADC_VBASE+SAM_ADC_MR_OFFSET)
#define SAM_ADC_SEQR1              (SAM_TSADC_VBASE+SAM_ADC_SEQR1_OFFSET)
#define SAM_ADC_SEQR2              (SAM_TSADC_VBASE+SAM_ADC_SEQR2_OFFSET)
#define SAM_ADC_CHER               (SAM_TSADC_VBASE+SAM_ADC_CHER_OFFSET)
#define SAM_ADC_CHDR               (SAM_TSADC_VBASE+SAM_ADC_CHDR_OFFSET)
#define SAM_ADC_CHSR               (SAM_TSADC_VBASE+SAM_ADC_CHSR_OFFSET)
#define SAM_ADC_LCDR               (SAM_TSADC_VBASE+SAM_ADC_LCDR_OFFSET)
#define SAM_ADC_IER                (SAM_TSADC_VBASE+SAM_ADC_IER_OFFSET)
#define SAM_ADC_IDR                (SAM_TSADC_VBASE+SAM_ADC_IDR_OFFSET)
#define SAM_ADC_IMR                (SAM_TSADC_VBASE+SAM_ADC_IMR_OFFSET)
#define SAM_ADC_ISR                (SAM_TSADC_VBASE+SAM_ADC_ISR_OFFSET)
#define SAM_ADC_OVER               (SAM_TSADC_VBASE+SAM_ADC_OVER_OFFSET)
#define SAM_ADC_EMR                (SAM_TSADC_VBASE+SAM_ADC_EMR_OFFSET)
#define SAM_ADC_CWR                (SAM_TSADC_VBASE+SAM_ADC_CWR_OFFSET)

#ifdef ATSAMA5D3
#  define SAM_ADC_CGR              (SAM_TSADC_VBASE+SAM_ADC_CGR_OFFSET)
#endif

#if defined (ATSAMA5D3) || defined (ATSAMA5D2)
#  define SAM_ADC_COR              (SAM_TSADC_VBASE+SAM_ADC_COR_OFFSET)
#endif

#define SAM_ADC_CDR(n)             (SAM_TSADC_VBASE+SAM_ADC_CDR_OFFSET(n))
#define SAM_ADC_CDR0               (SAM_TSADC_VBASE+SAM_ADC_CDR0_OFFSET)
#define SAM_ADC_CDR1               (SAM_TSADC_VBASE+SAM_ADC_CDR1_OFFSET)
#define SAM_ADC_CDR2               (SAM_TSADC_VBASE+SAM_ADC_CDR2_OFFSET)
#define SAM_ADC_CDR3               (SAM_TSADC_VBASE+SAM_ADC_CDR3_OFFSET)
#define SAM_ADC_CDR4               (SAM_TSADC_VBASE+SAM_ADC_CDR4_OFFSET)

#if defined (ATSAMA5D3) || defined (ATSAMA5D2)
#  define SAM_ADC_CDR5             (SAM_TSADC_VBASE+SAM_ADC_CDR5_OFFSET)
#  define SAM_ADC_CDR6             (SAM_TSADC_VBASE+SAM_ADC_CDR6_OFFSET)
#  define SAM_ADC_CDR7             (SAM_TSADC_VBASE+SAM_ADC_CDR7_OFFSET)
#  define SAM_ADC_CDR8             (SAM_TSADC_VBASE+SAM_ADC_CDR8_OFFSET)
#  define SAM_ADC_CDR9             (SAM_TSADC_VBASE+SAM_ADC_CDR9_OFFSET)
#  define SAM_ADC_CDR10            (SAM_TSADC_VBASE+SAM_ADC_CDR10_OFFSET)
#  define SAM_ADC_CDR11            (SAM_TSADC_VBASE+SAM_ADC_CDR11_OFFSET)
#endif

#define SAM_ADC_ACR                (SAM_TSADC_VBASE+SAM_ADC_ACR_OFFSET)
#define SAM_ADC_TSMR               (SAM_TSADC_VBASE+SAM_ADC_TSMR_OFFSET)
#define SAM_ADC_XPOSR              (SAM_TSADC_VBASE+SAM_ADC_XPOSR_OFFSET)
#define SAM_ADC_YPOSR              (SAM_TSADC_VBASE+SAM_ADC_YPOSR_OFFSET)
#define SAM_ADC_PRESSR             (SAM_TSADC_VBASE+SAM_ADC_PRESSR_OFFSET)
#define SAM_ADC_TRGR               (SAM_TSADC_VBASE+SAM_ADC_TRGR_OFFSET)
#define SAM_ADC_WPMR               (SAM_TSADC_VBASE+SAM_ADC_WPMR_OFFSET)
#define SAM_ADC_WPSR               (SAM_TSADC_VBASE+SAM_ADC_WPSR_OFFSET)

/* ADC register bit definitions *********************************************/

/* Control Register and ADC Control Register common bit-field definitions */

#define ADC_CR_SWRST               (1 << 0)  /* Bit 0:  Software Reset */
#define ADC_CR_START               (1 << 1)  /* Bit 1:  Start Conversion */
#define ADC_CR_TSCALIB             (1 << 2)  /* Bit 2:  Touchscreen Calibration */

#ifdef ATSAMA5D3
#  define ADC_CR_AUTOCAL           (1 << 3)  /* Bit 3:  Automatic Calibration of ADC */
#endif

/* Mode Register and ADC Mode Register common bit-field definitions */

#define ADC_MR_TRGSEL_SHIFT        (1)       /* Bits 1-3: Trigger Selection */
#define ADC_MR_TRGSEL_MASK         (7 << ADC_MR_TRGSEL_SHIFT)
#  define ADC_MR_TRGSEL_ADTRG      (0 << ADC_MR_TRGSEL_SHIFT) /* ADTRG */
#  define ADC_MR_TRGSEL_TIOA0      (1 << ADC_MR_TRGSEL_SHIFT) /* TIOA0 */
#  define ADC_MR_TRGSEL_TIOA1      (2 << ADC_MR_TRGSEL_SHIFT) /* TIOA1 */
#  define ADC_MR_TRGSEL_TIOA2      (3 << ADC_MR_TRGSEL_SHIFT) /* TIOA2 */
#  define ADC_MR_TRGSEL_PWM0       (4 << ADC_MR_TRGSEL_SHIFT) /* PWM Event Line 0 */
#  define ADC_MR_TRGSEL_PWM1       (5 << ADC_MR_TRGSEL_SHIFT) /* PWM Event Line 1 */

#ifdef ATSAMA5D4
#  define ADC_MR_LOWRES            (1 << 4)  /* Bit 4:  LOWRES: Resolution */
#endif

#define ADC_MR_SLEEP               (1 << 5)  /* Bit 5:  Sleep Mode */

#ifdef ATSAMA5D3
#  define ADC_MR_FWUP              (1 << 6)  /* Bit 6:  Fast Wake Up */
#endif

#define ADC_MR_PRESCAL_SHIFT       (8)       /* Bits 8-15: Prescaler Rate Selection */
#define ADC_MR_PRESCAL_MASK        (0xff << ADC_MR_PRESCAL_SHIFT)
#  define ADC_MR_PRESCAL(n)        ((uint32_t)(n) << ADC_MR_PRESCAL_SHIFT)
#define ADC_MR_STARTUP_SHIFT       (16)      /* Bits 16-19: Start Up Time */
#define ADC_MR_STARTUP_MASK        (15 << ADC_MR_STARTUP_SHIFT)
#  define ADC_MR_STARTUP_0         (0 << ADC_MR_STARTUP_SHIFT)  /* 0 periods of ADCClock   */
#  define ADC_MR_STARTUP_8         (1 << ADC_MR_STARTUP_SHIFT)  /* 8 periods of ADCClock   */
#  define ADC_MR_STARTUP_16        (2 << ADC_MR_STARTUP_SHIFT)  /* 16 periods of ADCClock  */
#  define ADC_MR_STARTUP_24        (3 << ADC_MR_STARTUP_SHIFT)  /* 24 periods of ADCClock  */
#  define ADC_MR_STARTUP_64        (4 << ADC_MR_STARTUP_SHIFT)  /* 64 periods of ADCClock  */
#  define ADC_MR_STARTUP_80        (5 << ADC_MR_STARTUP_SHIFT)  /* 80 periods of ADCClock  */
#  define ADC_MR_STARTUP_96        (6 << ADC_MR_STARTUP_SHIFT)  /* 96 periods of ADCClock  */
#  define ADC_MR_STARTUP_112       (7 << ADC_MR_STARTUP_SHIFT)  /* 112 periods of ADCClock */
#  define ADC_MR_STARTUP_512       (8 << ADC_MR_STARTUP_SHIFT)  /* 512 periods of ADCClock */
#  define ADC_MR_STARTUP_576       (9 << ADC_MR_STARTUP_SHIFT)  /* 576 periods of ADCClock */
#  define ADC_MR_STARTUP_640       (10 << ADC_MR_STARTUP_SHIFT) /* 640 periods of ADCClock */
#  define ADC_MR_STARTUP_704       (11 << ADC_MR_STARTUP_SHIFT) /* 704 periods of ADCClock */
#  define ADC_MR_STARTUP_768       (12 << ADC_MR_STARTUP_SHIFT) /* 768 periods of ADCClock */
#  define ADC_MR_STARTUP_832       (13 << ADC_MR_STARTUP_SHIFT) /* 832 periods of ADCClock */
#  define ADC_MR_STARTUP_896       (14 << ADC_MR_STARTUP_SHIFT) /* 896 periods of ADCClock */
#  define ADC_MR_STARTUP_960       (15 << ADC_MR_STARTUP_SHIFT) /* 960 periods of ADCClock */

#ifdef ATSAMA5D3
#  define ADC_MR_SETTLING_SHIFT    (20)      /* Bits 20-21: Analog Settling Time           */
#  define ADC_MR_SETTLING_MASK     (15 << ADC_MR_SETTLING_SHIFT)
#    define ADC_MR_SETTLING_3      (0 << ADC_MR_SETTLING_SHIFT) /* 3 periods of ADCClock   */
#    define ADC_MR_SETTLING_5      (1 << ADC_MR_SETTLING_SHIFT) /* 5 periods of ADCClock   */
#    define ADC_MR_SETTLING_9      (2 << ADC_MR_SETTLING_SHIFT) /* 9 periods of ADCClock   */
#    define ADC_MR_SETTLING_17     (3 << ADC_MR_SETTLING_SHIFT) /* 17 periods of ADCClock  */
#else
#  define ADC_MR_SETTLING_SHIFT    (20)      /* Not present in SAMA5D2 or SAMA5D4          */
#  define ADC_MR_SETTLING_MASK     (0)
#    define ADC_MR_SETTLING_3      (0 << ADC_MR_SETTLING_SHIFT) /* n/a periods of ADCClock */
#    define ADC_MR_SETTLING_5      (0 << ADC_MR_SETTLING_SHIFT) /* n/a periods of ADCClock */
#    define ADC_MR_SETTLING_9      (0 << ADC_MR_SETTLING_SHIFT) /* n/a periods of ADCClock */
#    define ADC_MR_SETTLING_17     (0 << ADC_MR_SETTLING_SHIFT) /* n/a periods of ADCClock */
#endif

#if defined (ATSAMA5D3) || defined (ATSAMA5D2)
#  define ADC_MR_ANACH             (1 << 23) /* Bit 23: Analog Change */
#endif

#define ADC_MR_TRACKTIM_SHIFT      (24)      /* Bits 24-27: Tracking Time */
#define ADC_MR_TRACKTIM_MASK       (15 << ADC_MR_TRACKTIM_SHIFT)
#  define ADC_MR_TRACKTIM(n)       ((uint32_t)(n) << ADC_MR_TRACKTIM_SHIFT)

#if defined (ATSAMA5D3) || defined (ATSAMA5D2)
#  define ADC_MR_TRANSFER_SHIFT    (28)      /* Bits 28-29: Transfer Period */
#  define ADC_MR_TRANSFER_MASK     (3 << ADC_MR_TRANSFER_SHIFT)
#    define ADC_MR_TRANSFER        (2 << ADC_MR_TRANSFER_SHIFT) /* Must be 2 */
#endif

#define ADC_MR_USEQ                (1 << 31) /* Bit 31: Use Sequence Enable */

/* Channel Sequence Register 1 */

#define ADC_SEQR1_USCH_SHIFT(n)    (((n)-1) << 2) /* n=1..8 */
#define ADC_SEQR1_USCH_MASK(n)     (15 << ADC_SEQR1_USCH_SHIFT(n))
#  define ADC_SEQR1_USCH(n,v)      ((uint32_t)(v) << ADC_SEQR1_USCH_SHIFT(n))
#define ADC_SEQR1_USCH1_SHIFT      (0) /* Bits 0-3: User sequence number 1 */
#define ADC_SEQR1_USCH1_MASK       (15 << ADC_SEQR1_USCH1_SHIFT)
#  define ADC_SEQR1_USCH1(v)       ((uint32_t)(v) << ADC_SEQR1_USCH1_SHIFT)
#define ADC_SEQR1_USCH2_SHIFT      (4) /* Bits 4-7: User sequence number 2 */
#define ADC_SEQR1_USCH2_MASK       (15 << ADC_SEQR1_USCH2_SHIFT)
#  define ADC_SEQR1_USCH2(v)       ((uint32_t)(v) << ADC_SEQR1_USCH2_SHIFT)
#define ADC_SEQR1_USCH3_SHIFT      (8) /* Bits 8-11: User sequence number 3 */
#define ADC_SEQR1_USCH3_MASK       (15 << ADC_SEQR1_USCH3_SHIFT)
#  define ADC_SEQR1_USCH3(v)       ((uint32_t)(v) << ADC_SEQR1_USCH3_SHIFT)
#define ADC_SEQR1_USCH4_SHIFT      (12) /* Bits 12-15: User sequence number 4 */
#define ADC_SEQR1_USCH4_MASK       (15 << ADC_SEQR1_USCH4_SHIFT)
#  define ADC_SEQR1_USCH4(v)       ((uint32_t)(v) << ADC_SEQR1_USCH4_SHIFT)

#ifdef ATSAMA5D3
#  define ADC_SEQR1_USCH5_SHIFT    (16) /* Bits 16-19: User sequence number 5 */
#  define ADC_SEQR1_USCH5_MASK     (15 << ADC_SEQR1_USCH5_SHIFT)
#    define ADC_SEQR1_USCH5(v)     ((uint32_t)(v) << ADC_SEQR1_USCH5_SHIFT)
#  define ADC_SEQR1_USCH6_SHIFT    (20) /* Bits 20-23: User sequence number 6 */
#  define ADC_SEQR1_USCH6_MASK     (15 << ADC_SEQR1_USCH6_SHIFT)
#    define ADC_SEQR1_USCH6(v)     ((uint32_t)(v) << ADC_SEQR1_USCH6_SHIFT)
#  define ADC_SEQR1_USCH7_SHIFT    (24) /* Bits 24-27: User sequence number 7 */
#  define ADC_SEQR1_USCH7_MASK     (15 << ADC_SEQR1_USCH7_SHIFT)
#    define ADC_SEQR1_USCH7(v)     ((uint32_t)(v) << ADC_SEQR1_USCH7_SHIFT)
#  define ADC_SEQR1_USCH8_SHIFT    (28) /* Bits 28-31: User sequence number 8 */
#  define ADC_SEQR1_USCH8_MASK     (15 << ADC_SEQR1_USCH8_SHIFT)
#    define ADC_SEQR1_USCH8(v)     ((uint32_t)(v) << ADC_SEQR1_USCH8_SHIFT)
#endif

#if defined (ATSAMA5D3) || defined (ATSAMA5D2)
/* Channel Sequence Register 2 */

#  define ADC_SEQR2_USCH_SHIFT(n)  (((n)-9) << 2) /* n=9..11 */
#  define ADC_SEQR2_USCH_MASK(n)   (15 << ADC_SEQR2_USCH_SHIFT(n))
#    define ADC_SEQR2_USCH(n,v)    ((uint32_t)(v) << ADC_SEQR2_USCH_SHIFT(n))
#  define ADC_SEQR2_USCH9_SHIFT    (0) /* Bits 0-3: User sequence number 9 */
#  define ADC_SEQR2_USCH9_MASK     (15 << ADC_SEQR2_USCH9_SHIFT)
#    define ADC_SEQR2_USCH9(v)     ((uint32_t)(v) << ADC_SEQR2_USCH9_SHIFT)
#  define ADC_SEQR2_USCH10_SHIFT   (4) /* Bits 4-7: User sequence number 10 */
#  define ADC_SEQR2_USCH10_MASK    (15 << ADC_SEQR2_USCH10_SHIFT)
#    define ADC_SEQR2_USCH10(v)    ((uint32_t)(v) << ADC_SEQR2_USCH10_SHIFT)
#  define ADC_SEQR2_USCH11_SHIFT   (8) /* Bits 8-11: User sequence number 11 */
#  define ADC_SEQR2_USCH11_MASK    (15 << ADC_SEQR2_USCH11_SHIFT)
#    define ADC_SEQR2_USCH11(v)    ((uint32_t)(v) << ADC_SEQR2_USCH11_SHIFT)
#endif

/* Channel Enable Register, Channel Disable Register, Channel
 * Status Register, ADC Channel Enable Register, ADC Channel Disable
 * Register, and ADC Channel Status Register common bit-field definitions
 */

#define ADC_CH(n)                  (1 << (n))
#define ADC_CH0                    (1 << 0)  /* Bit 0:  Channel 0 Enable */
#define ADC_CH1                    (1 << 1)  /* Bit 1:  Channel 1 Enable */
#define ADC_CH2                    (1 << 2)  /* Bit 2:  Channel 2 Enable */
#define ADC_CH3                    (1 << 3)  /* Bit 3:  Channel 3 Enable */
#define ADC_CH4                    (1 << 4)  /* Bit 4:  Channel 4 Enable */
#define ADC_CH5                    (1 << 5)  /* Bit 5:  Channel 5 Enable */

#if defined (ATSAMA5D3) || defined (ATSAMA5D2)
#  define ADC_CH6                  (1 << 6)  /* Bit 6:  Channel 6 Enable */
#  define ADC_CH7                  (1 << 7)  /* Bit 7:  Channel 7 Enable */
#  define ADC_CH8                  (1 << 8)  /* Bit 8:  Channel 8 Enable */
#  define ADC_CH9                  (1 << 9)  /* Bit 9:  Channel 9 Enable */
#  define ADC_CH10                 (1 << 10) /* Bit 10: Channel 10 Enable */
#  define ADC_CH11                 (1 << 11) /* Bit 11: Channel 11 Enable */
#endif

#define TSD_4WIRE_ALL              (0x0000000f)
#define TSD_5WIRE_ALL              (0x0000001f)

#if defined (ATSAMA5D3) || defined (ATSAMA5D2)
#  define ADC_CHALL                (0x00000fff)
#elif defined(ATSAMA5D4)
#  define ADC_CHALL                (0x0000001f)
#endif

/* Last Converted Data Register */

#define ADC_LCDR_DATA_SHIFT        (0)       /* Bits 0-11: Last Data Converted */
#define ADC_LCDR_DATA_MASK         (0xfff << ADC_LCDR_DATA_SHIFT)
#define ADC_LCDR_CHANB_SHIFT       (12)      /* Bits 12-15: Channel number */
#define ADC_LCDR_CHANB_MASK        (15 << ADC_LCDR_CHANB_SHIFT)

/* Status Register , Interrupt Enable Register, Interrupt
 * Disable Register, Interrupt Mask Register, ADC Status Register,
 * ADC Interrupt Enable Register, ADC Interrupt Disable Register, and
 * ADC Interrupt Mask Register common bit-field definitions
 */

#define ADC_INT_EOC(n)             (1 << (n))
#define ADC_INT_EOC0               (1 << 0)  /* Bit 0:  End of Conversion 0 */
#define ADC_INT_EOC1               (1 << 1)  /* Bit 1:  End of Conversion 1 */
#define ADC_INT_EOC2               (1 << 2)  /* Bit 2:  End of Conversion 2 */
#define ADC_INT_EOC3               (1 << 3)  /* Bit 3:  End of Conversion 3 */
#define ADC_INT_EOC4               (1 << 4)  /* Bit 4:  End of Conversion 4 */

#if defined (ATSAMA5D3) || defined (ATSAMA5D2)
#  define ADC_INT_EOC5             (1 << 5)  /* Bit 5:  End of Conversion 5 */
#  define ADC_INT_EOC6             (1 << 6)  /* Bit 6:  End of Conversion 6 */
#  define ADC_INT_EOC7             (1 << 7)  /* Bit 7:  End of Conversion 7 */
#  define ADC_INT_EOC8             (1 << 8)  /* Bit 8:  End of Conversion 8 */
#  define ADC_INT_EOC9             (1 << 9)  /* Bit 9:  End of Conversion 9 */
#  define ADC_INT_EOC10            (1 << 10) /* Bit 10: End of Conversion 10 */
#  define ADC_INT_EOC11            (1 << 11) /* Bit 11: End of Conversion 11 */
#  define ADC_INT_EOCALL           (0x00000fff)
#elif defined(ATSAMA5D4)
#  define ADC_INT_EOCALL           (0x0000001f)
#endif

#define ADC_INT_XRDY               (1 << 20) /* Bit 20: TS Measure XPOS Ready Interrupt */
#define ADC_INT_YRDY               (1 << 21) /* Bit 21: TS Measure YPOS Ready Interrupt */
#define ADC_INT_PRDY               (1 << 22) /* Bit 22: TS Measure Pressure Ready Interrupt */

#ifdef ATSAMA5D3
#  define ADC_INT_EOCAL            (1 << 23) /* Bit 23: End of Calibration Sequence */
#endif

#define ADC_INT_DRDY               (1 << 24) /* Bit 24: Data Ready Interrupt */
#define ADC_INT_GOVRE              (1 << 25) /* Bit 25: General Overrun Error */
#define ADC_INT_COMPE              (1 << 26) /* Bit 26: Comparison Event Interrupt */
#define ADC_INT_PEN                (1 << 29) /* Bit 29: Pen Contact Interrupt */
#define ADC_INT_NOPEN              (1 << 30) /* Bit 30: No Pen Contact Interrupt */
#define ADC_SR_PENS                (1 << 31) /* Bit 31: Pen detect Status (SR only) */

#if defined (ATSAMA5D3)
#  define ADC_INT_ALL              (0xe7f00fff)
#elif defined (ATSAMA5D2)
#  define ADC_INT_ALL              (0x67780fff)
#endif

/* Overrun Status Register */

#define ADC_OVER_OVRE(n)           (1 << (n))
#define ADC_OVER_OVRE0             (1 << 0)  /* Bit 0:  Overrun Error 0 */
#define ADC_OVER_OVRE1             (1 << 1)  /* Bit 1:  Overrun Error 1 */
#define ADC_OVER_OVRE2             (1 << 2)  /* Bit 2:  Overrun Error 2 */
#define ADC_OVER_OVRE3             (1 << 3)  /* Bit 3:  Overrun Error 3 */
#define ADC_OVER_OVRE4             (1 << 4)  /* Bit 4:  Overrun Error 4 */

#ifdef ATSAMA5D3
#  define ADC_OVER_OVRE5           (1 << 5)  /* Bit 5:  Overrun Error 5 */
#  define ADC_OVER_OVRE6           (1 << 6)  /* Bit 6:  Overrun Error 6 */
#  define ADC_OVER_OVRE7           (1 << 7)  /* Bit 7:  Overrun Error 7 */
#  define ADC_OVER_OVRE8           (1 << 8)  /* Bit 8:  Overrun Error 8 */
#  define ADC_OVER_OVRE9           (1 << 9)  /* Bit 9:  Overrun Error 9 */
#  define ADC_OVER_OVRE10          (1 << 10) /* Bit 10: Overrun Error 10 */
#  define ADC_OVER_OVRE11          (1 << 11) /* Bit 11: Overrun Error 11 */
#endif

/* Extended Mode Register */

#define ADC_EMR_CMPMODE_SHIFT      (0)      /* Bit 0-1: Comparison Mode */
#define ADC_EMR_CMPMODE_MASK       (3 << ADC_EMR_CMPMODE_SHIFT)
#  define ADC_EMR_CMPMODE_LOW      (0 << ADC_EMR_CMPMODE_SHIFT) /* Event when lower than low window threshold */
#  define ADC_EMR_CMPMODE_HIGH     (1 << ADC_EMR_CMPMODE_SHIFT) /* Event when higher than high window threshold */
#  define ADC_EMR_CMPMODE_IN       (2 << ADC_EMR_CMPMODE_SHIFT) /* Event when in comparison window */
#  define ADC_EMR_CMPMODE_OUT      (3 << ADC_EMR_CMPMODE_SHIFT) /* Event when out of comparison window */

#define ADC_EMR_CMPSEL_SHIFT       (4)       /* Bit 4-7: Comparison Selected Channel */
#define ADC_EMR_CMPSEL_MASK        (15 << ADC_EMR_CMPSEL_SHIFT)
#  define ADC_EMR_CMPSEL(n)        ((uint32_t)(n) << ADC_EMR_CMPSEL_SHIFT)
#define ADC_EMR_CMPALL             (1 << 9)  /* Bit 9:  Compare All Channels */
#define ADC_EMR_CMPFILTER_SHIFT    (12)      /* Bit 12-13: Compare Event Filtering */
#define ADC_EMR_CMPFILTER_MASK     (3 << ADC_EMR_CMPFILTER_SHIFT)
#  define ADC_EMR_CMPFILTER(n)     ((uint32_t)(n) << ADC_EMR_CMPFILTER_SHIFT)

#ifdef ATSAMA5D4
#  define ADC_EMR_OSR_SHIFT        (16)      /* Bit 16-17: Compare Event Filtering */
#  define ADC_EMR_OSR_MASK         (3 << ADC_EMR_OSR_SHIFT)
#    define ADC_EMR_OSR_NOAVG      (0 << ADC_EMR_OSR_SHIFT) /* No averaging */
#    define ADC_EMR_OSR_OSR4       (1 << ADC_EMR_OSR_SHIFT) /* 1-bit averaging. ADC sample rate / 4 */
#    define ADC_EMR_OSR_OSR16      (2 << ADC_EMR_OSR_SHIFT) /* 2-bit averaging. ADC sample rate / 16 */

#  define ADC_EMR_ASTE             (1 << 10) /* Bit 10: Averaging on Single Trigger Event */
#endif

#define ADC_EMR_TAG                (1 << 24) /* Bit 24: TAG of the ADC_LDCR register */

#ifdef ATSAMA5D3
/* Channel Gain Register */

#  define ADC_CGR_GAIN_SHIFT(n)    ((n) << 1) /* n=0..11 */
#  define ADC_CGR_GAIN_MASK(n)     (3 << ADC_CGR_GAIN_SHIFT(n))
#    define ADC_CGR_GAIN(n,v)      ((uint32_t)(v) << ADC_CGR_GAIN_SHIFT(n))
#  define ADC_CGR_GAIN0_SHIFT      (0)        /* Bits 0-1: Gain for channel 0 */
#  define ADC_CGR_GAIN0_MASK       (3 << ADC_CGR_GAIN0_SHIFT)
#    define ADC_CGR_GAIN0(v)       ((uint32_t)(v) << ADC_CGR_GAIN0_SHIFT)
#  define ADC_CGR_GAIN1_SHIFT      (2)        /* Bits 2-3: Gain for channel 1 */
#  define ADC_CGR_GAIN1_MASK       (3 << ADC_CGR_GAIN1_SHIFT)
#    define ADC_CGR_GAIN1(v)       ((uint32_t)(v) << ADC_CGR_GAIN1_SHIFT)
#  define ADC_CGR_GAIN2_SHIFT      (4)        /* Bits 4-5: Gain for channel 2 */
#  define ADC_CGR_GAIN2_MASK       (3 << ADC_CGR_GAIN2_SHIFT)
#    define ADC_CGR_GAIN2(v)       ((uint32_t)(v) << ADC_CGR_GAIN2_SHIFT)
#  define ADC_CGR_GAIN3_SHIFT      (6)        /* Bits 6-7: Gain for channel 3 */
#  define ADC_CGR_GAIN3_MASK       (3 << ADC_CGR_GAIN3_SHIFT)
#    define ADC_CGR_GAIN3(v)       ((uint32_t)(v) << ADC_CGR_GAIN3_SHIFT)
#  define ADC_CGR_GAIN4_SHIFT      (8)        /* Bits 8-9: Gain for channel 4 */
#  define ADC_CGR_GAIN4_MASK       (3 << ADC_CGR_GAIN4_SHIFT)
#    define ADC_CGR_GAIN4(v)       ((uint32_t)(v) << ADC_CGR_GAIN4_SHIFT)
#  define ADC_CGR_GAIN5_SHIFT      (10)       /* Bits 10-11: Gain for channel 5 */
#  define ADC_CGR_GAIN5_MASK       (3 << ADC_CGR_GAIN5_SHIFT)
#    define ADC_CGR_GAIN5(v)       ((uint32_t)(v) << ADC_CGR_GAIN5_SHIFT)
#  define ADC_CGR_GAIN6_SHIFT      (12)       /* Bits 12-13: Gain for channel 6 */
#  define ADC_CGR_GAIN6_MASK       (3 << ADC_CGR_GAIN6_SHIFT)
#    define ADC_CGR_GAIN6(v)       ((uint32_t)(v) << ADC_CGR_GAIN6_SHIFT)
#  define ADC_CGR_GAIN7_SHIFT      (14)       /* Bits 14-15: Gain for channel 7 */
#  define ADC_CGR_GAIN7_MASK       (3 << ADC_CGR_GAIN7_SHIFT)
#    define ADC_CGR_GAIN7(v)       ((uint32_t)(v) << ADC_CGR_GAIN7_SHIFT)
#  define ADC_CGR_GAIN8_SHIFT      (16)       /* Bits 16-17: Gain for channel 8 */
#  define ADC_CGR_GAIN8_MASK       (3 << ADC_CGR_GAIN8_SHIFT)
#    define ADC_CGR_GAIN8(v)       ((uint32_t)(v) << ADC_CGR_GAIN8_SHIFT)
#  define ADC_CGR_GAIN9_SHIFT      (18)       /* Bits 18-19: Gain for channel 9 */
#  define ADC_CGR_GAIN9_MASK       (3 << ADC_CGR_GAIN9_SHIFT)
#    define ADC_CGR_GAIN9(v)       ((uint32_t)(v) << ADC_CGR_GAIN9_SHIFT)
#  define ADC_CGR_GAIN10_SHIFT     (20)       /* Bits 20-21: Gain for channel 10 */
#  define ADC_CGR_GAIN10_MASK      (3 << ADC_CGR_GAIN10_SHIFT)
#    define ADC_CGR_GAIN10(v)      ((uint32_t)(v) << ADC_CGR_GAIN10_SHIFT)
#  define ADC_CGR_GAIN11_SHIFT     (22)       /* Bits 22-23: Gain for channel 11 */
#  define ADC_CGR_GAIN11_MASK      (3 << ADC_CGR_GAIN11_SHIFT)
#    define ADC_CGR_GAIN11(v)      ((uint32_t)(v) << ADC_CGR_GAIN11_SHIFT)
#endif

#ifdef ATSAMA5D3
/* Channel Offset Register */

#  define ADC_COR_OFF(n)           (1 << (n))
#  define ADC_COR_OFF0             (1 << 0)  /* Bit 0:  Offset for channel 0 */
#  define ADC_COR_OFF1             (1 << 1)  /* Bit 1:  Offset for channel 1 */
#  define ADC_COR_OFF2             (1 << 2)  /* Bit 2:  Offset for channel 2 */
#  define ADC_COR_OFF3             (1 << 3)  /* Bit 3:  Offset for channel 3 */
#  define ADC_COR_OFF4             (1 << 4)  /* Bit 4:  Offset for channel 4 */
#  define ADC_COR_OFF5             (1 << 5)  /* Bit 5:  Offset for channel 5 */
#  define ADC_COR_OFF6             (1 << 6)  /* Bit 6:  Offset for channel 6 */
#  define ADC_COR_OFF7             (1 << 7)  /* Bit 7:  Offset for channel 7 */
#  define ADC_COR_OFF8             (1 << 8)  /* Bit 8:  Offset for channel 8 */
#  define ADC_COR_OFF9             (1 << 9)  /* Bit 9:  Offset for channel 9 */
#  define ADC_COR_OFF10            (1 << 10) /* Bit 10: Offset for channel 10 */
#  define ADC_COR_OFF11            (1 << 11) /* Bit 11: Offset for channel 11 */

#  define ADC_COR_DIFF(n)          (1 << ((n)+16))
#  define ADC_COR_DIFF0            (1 << 16) /* Bit 16: Offset for channel 0 */
#  define ADC_COR_DIFF1            (1 << 17) /* Bit 17: Offset for channel 1 */
#  define ADC_COR_DIFF2            (1 << 18) /* Bit 18: Offset for channel 2 */
#  define ADC_COR_DIFF3            (1 << 19) /* Bit 19: Offset for channel 3 */
#  define ADC_COR_DIFF4            (1 << 20) /* Bit 20: Offset for channel 4 */
#  define ADC_COR_DIFF5            (1 << 21) /* Bit 21: Offset for channel 5 */
#  define ADC_COR_DIFF6            (1 << 22) /* Bit 22: Offset for channel 6 */
#  define ADC_COR_DIFF7            (1 << 23) /* Bit 23: Offset for channel 7 */
#  define ADC_COR_DIFF8            (1 << 24) /* Bit 24: Offset for channel 8 */
#  define ADC_COR_DIFF9            (1 << 25) /* Bit 25: Offset for channel 9 */
#  define ADC_COR_DIFF10           (1 << 26) /* Bit 26: Offset for channel 10 */
#  define ADC_COR_DIFF11           (1 << 27) /* Bit 27: Offset for channel 11 */
#endif

/* Channel Data Register */

#define ADC_CDR_DATA_SHIFT         (0)       /* Bits 0-11: Converted Data */

#if defined (ATSAMA5D2)
#  define ADC_CDR_DATA_MASK        (0x3fff << ADC_CDR_DATA_SHIFT)
#else
#  define ADC_CDR_DATA_MASK        (0xfff << ADC_CDR_DATA_SHIFT)
#endif

/* Compare Window Register */

#define ADC_CWR_LOWTHRES_SHIFT     (0)      /* Bit 0-11: Low Threshold */
#define ADC_CWR_LOWTHRES_MASK      (0xfff << ADC_CWR_LOWTHRES_SHIFT)
#  define ADC_CWR_LOWTHRES(n)      ((uint32_t)(n) << ADC_CWR_LOWTHRES_SHIFT)
#define ADC_CWR_HIGHTHRES_SHIFT    (16)     /* Bit 16-27: High Threshold */
#define ADC_CWR_HIGHTHRES_MASK     (0xfff << ADC_CWR_HIGHTHRES_SHIFT)
#  define ADC_CWR_HIGHTHRES(n)     ((uint32_t)(n) << ADC_CWR_HIGHTHRES_SHIFT)

/* Analog Control Register */

#define ADC_ACR_PENDETSENS_SHIFT   (0)       /* Bits 0-1: Pen Detection Sensitivity */
#define ADC_ACR_PENDETSENS_MASK    (3 << ADC_ACR_PENDETSENS_SHIFT)
#  define ADC_ACR_PENDETSENS(n)    ((uint32_t)(n) << ADC_ACR_PENDETSENS_SHIFT)

#if defined (ATSAMA5D2)
#  define ADC_ACR_IBTL_SHIFT       (8) /* Bits 8-9: ADC Bias Current Control */
#  define ADC_ACR_IBCTL_MASK       (3 << ADC_ACR_IBTL_SHIFT)
#  define ADC_ACR_IBCTL(n)         ((uint32_t)(n) << ADC_ACR_IBTL_SHIFT)
#endif

/* Touchscreen Mode Register */

#define ADC_TSMR_TSMODE_SHIFT      (0)       /* Bit 0-1: Touchscreen Mode */
#define ADC_TSMR_TSMODE_MASK       (3 << ADC_TSMR_TSMODE_SHIFT)
#  define ADC_TSMR_TSMODE_NONE     (0 << ADC_TSMR_TSMODE_SHIFT) /* No Touchscreen */
#  define ADC_TSMR_TSMODE_4WIRENPM (1 << ADC_TSMR_TSMODE_SHIFT) /* 4-wire TS w/o pressure measurement */
#  define ADC_TSMR_TSMODE_4WIRE    (2 << ADC_TSMR_TSMODE_SHIFT) /* 4-wire TS w/ pressure measurement */
#  define ADC_TSMR_TSMODE_5WIRE    (3 << ADC_TSMR_TSMODE_SHIFT) /* 5-wire Touchscreen */

#define ADC_TSMR_TSAV_SHIFT        (4)       /* Bit 4-5: Touchscreen Average */
#define ADC_TSMR_TSAV_MASK         (3 << ADC_TSMR_TSAV_SHIFT)
#  define ADC_TSMR_TSAV_NOFILTER   (0 << ADC_TSMR_TSAV_SHIFT) /* No Filtering */
#  define ADC_TSMR_TSAV_2CONV      (1 << ADC_TSMR_TSAV_SHIFT) /* Average 2 ADC conversions */
#  define ADC_TSMR_TSAV_4CONV      (2 << ADC_TSMR_TSAV_SHIFT) /* Average 4 ADC conversions */
#  define ADC_TSMR_TSAV_8CONV      (3 << ADC_TSMR_TSAV_SHIFT) /* Averages 8 ADC conversions */

#define ADC_TSMR_TSFREQ_SHIFT      (8)       /* Bit 8-11: Touchscreen Frequency */
#define ADC_TSMR_TSFREQ_MASK       (15 << ADC_TSMR_TSFREQ_SHIFT)
#  define ADC_TSMR_TSFREQ_DIV1     (0 << ADC_TSMR_TSFREQ_SHIFT) /* TS freq = trigger freq */
#  define ADC_TSMR_TSFREQ_DIV2     (1 << ADC_TSMR_TSFREQ_SHIFT) /* TS freq = trigger freq / 2 */
#  define ADC_TSMR_TSFREQ_DIV4     (2 << ADC_TSMR_TSFREQ_SHIFT) /* TS freq = trigger freq / 4 */
#  define ADC_TSMR_TSFREQ_DIV8     (3 << ADC_TSMR_TSFREQ_SHIFT) /* TS freq = trigger freq / 8 */
#  define ADC_TSMR_TSFREQ(n)       ((uint32_t)(n) << ADC_TSMR_TSFREQ_SHIFT)

#define ADC_TSMR_TSSCTIM_SHIFT     (16)      /* Bit 16-19: Touchscreen Switches Closure Time */
#define ADC_TSMR_TSSCTIM_MASK      (15 << ADC_TSMR_TSSCTIM_SHIFT)
#  define ADC_TSMR_TSSCTIM(n)      ((uint32_t)(n) << ADC_TSMR_TSSCTIM_SHIFT)
#define ADC_TSMR_NOTSDMA           (1 << 22) /* Bit 22: No TouchScreen DMA */
#define ADC_TSMR_PENDET            (1 << 24) /* Bit 24: Pen Contact Detection Enable */
#define ADC_TSMR_PENDBC_SHIFT      (28)      /* Bit 28-31: Pen Detect Debouncing Period */
#define ADC_TSMR_PENDBC_MASK       (15 << ADC_TSMR_PENDBC_SHIFT)
#  define ADC_TSMR_PENDBC(n)       ((uint32_t)(n) << ADC_TSMR_PENDBC_SHIFT)

/* Touchscreen X Position Register */

#define ADC_XPOSR_XPOS_SHIFT       (0)       /* Bit 0-11: X Position */
#define ADC_XPOSR_XPOS_MASK        (0xfff << ADC_XPOSR_XPOS_SHIFT)
#define ADC_XPOSR_XSCALE_SHIFT     (16)      /* Bit 16-27: Scale of XPOS */
#define ADC_XPOSR_XSCALE_MASK      (0xfff << ADC_XPOSR_XSCALE_SHIFT)

/* Touchscreen Y Position Register */

#define ADC_YPOSR_YPOS_SHIFT       (0)       /* Bit 0-11: Y Position */
#define ADC_YPOSR_YPOS_MASK        (0xfff << ADC_YPOSR_YPOS_SHIFT)
#define ADC_YPOSR_YSCALE_SHIFT     (16)      /* Bit 16-27: Scale of YPOS */
#define ADC_YPOSR_YSCALE_MASK      (0xfff << ADC_YPOSR_YSCALE_SHIFT)

/* Touchscreen Pressure Register */

#define ADC_PRESSR_Z1_SHIFT        (0)       /* Bit 0-11: Data of Z1 Measurement */
#define ADC_PRESSR_Z1_MASK         (0xfff << ADC_PRESSR_Z1_SHIFT)
#define ADC_PRESSR_Z2_SHIFT        (16)      /* Bit 16-27: Data of Z2 Measuremen */
#define ADC_PRESSR_Z2_MASK         (0xfff << ADC_PRESSR_Z2_SHIFT)

/* Trigger Register */

#define ADC_TRGR_TRGMOD_SHIFT      (0)       /* Bit 0-2: Trigger Mode */
#define ADC_TRGR_TRGMOD_MASK       (7 << ADC_TRGR_TRGMOD_SHIFT)
#  define ADC_TRGR_TRGMOD_NOTRIG   (0 << ADC_TRGR_TRGMOD_SHIFT) /* No trigger */
#  define ADC_TRGR_TRGMOD_EXTRISE  (1 << ADC_TRGR_TRGMOD_SHIFT) /* External Trigger Rising Edge */
#  define ADC_TRGR_TRGMOD_EXTFALL  (2 << ADC_TRGR_TRGMOD_SHIFT) /* External Trigger Falling Edge */
#  define ADC_TRGR_TRGMOD_EXTBOTH  (3 << ADC_TRGR_TRGMOD_SHIFT) /* External Trigger Any Edge */
#  define ADC_TRGR_TRGMOD_PEN      (4 << ADC_TRGR_TRGMOD_SHIFT) /* Pen Detect Trigger */
#  define ADC_TRGR_TRGMOD_PERIOD   (5 << ADC_TRGR_TRGMOD_SHIFT) /* Periodic Trigger */
#  define ADC_TRGR_TRGMOD_CONT     (6 << ADC_TRGR_TRGMOD_SHIFT) /* Continuous Mode */

#define ADC_TRGR_TRGPER_SHIFT      (16)      /* Bit 16-31: Trigger Period */
#define ADC_TRGR_TRGPER_MASK       (0xffff << ADC_TRGR_TRGPER_SHIFT)
#  define ADC_TRGR_TRGPER(n)       ((uint32_t)(n) << ADC_TRGR_TRGPER_SHIFT)

/* Write Protect Mode Register */

#define ADC_WPMR_WPEN              (1 << 0)  /* Bit 0:  Write Protect Enable */
#define ADC_WPMR_WPKEY_SHIFT       (8)       /* Bits 8-31: Write Protect KEY */
#define ADC_WPMR_WPKEY_MASK        (0x00ffffff << ADC_WPMR_WPKEY_SHIFT)
#  define ADC_WPMR_WPKEY           (0x00414443 << ADC_WPMR_WPKEY_SHIFT)

/* Write Protect Status Register */

#define ADC_WPSR_WPVS              (1 << 0)  /* Bit 0:  Write Protect Violation Status */
#define ADC_WPSR_WPVSRC_SHIFT      (8)       /* Bits 8-23: Write Protect Violation Source */
#define ADC_WPSR_WPVSRC_MASK       (0xffff << ADC_WPSR_WPVSRC_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_ADC_H */
