/****************************************************************************
 * arch/arm/src/samd2l2/hardware/samd_gclk.h
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

/* References:
 *   "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *   Datasheet", 42129J�SAM�12/2013
 *   "Atmel SAM D21E / SAM D21G / SAM D21J SMART ARM-Based Microcontroller
 *   Datasheet", Atmel-42181E�SAM-D21_Datasheet�02/2015
 */

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_GCLK_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_GCLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if defined(CONFIG_ARCH_FAMILY_SAMD20) || defined(CONFIG_ARCH_FAMILY_SAMD21)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GCLK register offsets ****************************************************/

#define SAM_GCLK_CTRL_OFFSET    0x0000 /* Control register */
#define SAM_GCLK_STATUS_OFFSET  0x0001 /* Status register */
#define SAM_GCLK_CLKCTRL_OFFSET 0x0002 /* Generic clock control register */
#define SAM_GCLK_GENCTRL_OFFSET 0x0004 /* Generic clock generator control register */
#define SAM_GCLK_GENDIV_OFFSET  0x0008 /* Generic clock generator division register */

/* GCLK register addresses **************************************************/

#define SAM_GCLK_CTRL           (SAM_GCLK_BASE+SAM_GCLK_CTRL_OFFSET)
#define SAM_GCLK_STATUS         (SAM_GCLK_BASE+SAM_GCLK_STATUS_OFFSET)
#define SAM_GCLK_CLKCTRL        (SAM_GCLK_BASE+SAM_GCLK_CLKCTRL_OFFSET)
#define SAM_GCLK_GENCTRL        (SAM_GCLK_BASE+SAM_GCLK_GENCTRL_OFFSET)
#define SAM_GCLK_GENDIV         (SAM_GCLK_BASE+SAM_GCLK_GENDIV_OFFSET)

/* GCLK register bit definitions ********************************************/

/* Control register */

#define GCLK_CTRL_SWRST         (1 << 0)  /* Bit 0:  Software Reset */

/* Status register */

#define GCLK_STATUS_SYNCBUSY    (1 << 7)  /* Bit 7: Synchronization Busy Status */

/* Generic clock control register */

#define GCLK_CLKCTRL_ID_SHIFT   (0)       /* Bits 0-5: Generic Clock Selection ID */
#define GCLK_CLKCTRL_ID_MASK    (0x3f << GCLK_CLKCTRL_ID_SHIFT)
#  define GCLK_CLKCTRL_ID(n)    ((n) << GCLK_CLKCTRL_ID_SHIFT)

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define GCLK_CLKCTRL_ID_DFLL48M    (0 << GCLK_CLKCTRL_ID_SHIFT)  /* DFLL48M Reference */
#  define GCLK_CLKCTRL_ID_WDT        (1 << GCLK_CLKCTRL_ID_SHIFT)  /* WDT */
#  define GCLK_CLKCTRL_ID_RTC        (2 << GCLK_CLKCTRL_ID_SHIFT)  /* RTC */
#  define GCLK_CLKCTRL_ID_EIC        (3 << GCLK_CLKCTRL_ID_SHIFT)  /* EIC */
#  define GCLK_CLKCTRL_ID_EVSYS(n)   (((n)+4) << GCLK_CLKCTRL_ID_SHIFT)
#  define GCLK_CLKCTRL_ID_EVSYS1     (5 << GCLK_CLKCTRL_ID_SHIFT)  /* EVSYS_CHANNEL_1 */
#  define GCLK_CLKCTRL_ID_EVSYS1     (5 << GCLK_CLKCTRL_ID_SHIFT)  /* EVSYS_CHANNEL_1 */
#  define GCLK_CLKCTRL_ID_EVSYS2     (6 << GCLK_CLKCTRL_ID_SHIFT)  /* EVSYS_CHANNEL_2 */
#  define GCLK_CLKCTRL_ID_EVSYS3     (7 << GCLK_CLKCTRL_ID_SHIFT)  /* EVSYS_CHANNEL_3 */
#  define GCLK_CLKCTRL_ID_EVSYS4     (8 << GCLK_CLKCTRL_ID_SHIFT)  /* EVSYS_CHANNEL_4 */
#  define GCLK_CLKCTRL_ID_EVSYS5     (9 << GCLK_CLKCTRL_ID_SHIFT)  /* EVSYS_CHANNEL_5 */
#  define GCLK_CLKCTRL_ID_EVSYS6     (10 << GCLK_CLKCTRL_ID_SHIFT) /* EVSYS_CHANNEL_6 */
#  define GCLK_CLKCTRL_ID_EVSYS7     (11 << GCLK_CLKCTRL_ID_SHIFT) /* EVSYS_CHANNEL_7 */
#  define GCLK_CLKCTRL_ID_SERCOMS    (12 << GCLK_CLKCTRL_ID_SHIFT) /* SERCOMx_SLOW */
#  define GCLK_CLKCTRL_ID_SERCOMC(n) (((n)+13) << GCLK_CLKCTRL_ID_SHIFT)
#  define GCLK_CLKCTRL_ID_SERCOM0C   (13 << GCLK_CLKCTRL_ID_SHIFT) /* SERCOM0_CORE */
#  define GCLK_CLKCTRL_ID_SERCOM1C   (14 << GCLK_CLKCTRL_ID_SHIFT) /* SERCOM1_CORE */
#  define GCLK_CLKCTRL_ID_SERCOM2C   (15 << GCLK_CLKCTRL_ID_SHIFT) /* SERCOM2_CORE */
#  define GCLK_CLKCTRL_ID_SERCOM3C   (16 << GCLK_CLKCTRL_ID_SHIFT) /* SERCOM3_CORE */
#  define GCLK_CLKCTRL_ID_SERCOM4C   (17 << GCLK_CLKCTRL_ID_SHIFT) /* SERCOM4_CORE */
#  define GCLK_CLKCTRL_ID_SERCOM5C   (18 << GCLK_CLKCTRL_ID_SHIFT) /* SERCOM5_CORE */
#  define GCLK_CLKCTRL_ID_TC01       (19 << GCLK_CLKCTRL_ID_SHIFT) /* TC0,TC1 */
#  define GCLK_CLKCTRL_ID_TC23       (20 << GCLK_CLKCTRL_ID_SHIFT) /* TC2,TC3 */
#  define GCLK_CLKCTRL_ID_TC45       (21 << GCLK_CLKCTRL_ID_SHIFT) /* TC4,TC5 */
#  define GCLK_CLKCTRL_ID_TC67       (22 << GCLK_CLKCTRL_ID_SHIFT) /* TC6,TC7 */
#  define GCLK_CLKCTRL_ID_ADC        (23 << GCLK_CLKCTRL_ID_SHIFT) /* ADC */
#  define GCLK_CLKCTRL_ID_ACDIG      (24 << GCLK_CLKCTRL_ID_SHIFT) /* AC_DIG */
#  define GCLK_CLKCTRL_ID_ACANA      (25 << GCLK_CLKCTRL_ID_SHIFT) /* AC_ANA */
#  define GCLK_CLKCTRL_ID_DAC        (26 << GCLK_CLKCTRL_ID_SHIFT) /* DAC */
#  define GCLK_CLKCTRL_ID_PTC        (27 << GCLK_CLKCTRL_ID_SHIFT) /* PTC */
#endif

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define GCLK_CLKCTRL_ID_DFLL48M    (0 << GCLK_CLKCTRL_ID_SHIFT)  /* DFLL48M Reference */
#  define GCLK_CLKCTRL_ID_DPLL       (1 << GCLK_CLKCTRL_ID_SHIFT)  /* FDPLL96M input clock source for reference */
#  define GCLK_CLKCTRL_ID_DPLL32K    (2 << GCLK_CLKCTRL_ID_SHIFT)  /* FDPLL96M 32kHz clock for FDPLL96M internal lock timer */
#  define GCLK_CLKCTRL_ID_WDT        (3 << GCLK_CLKCTRL_ID_SHIFT)  /* WDT */
#  define GCLK_CLKCTRL_ID_RTC        (4 << GCLK_CLKCTRL_ID_SHIFT)  /* RTC */
#  define GCLK_CLKCTRL_ID_EIC        (5 << GCLK_CLKCTRL_ID_SHIFT)  /* EIC */
#  define GCLK_CLKCTRL_ID_USB        (6 << GCLK_CLKCTRL_ID_SHIFT)  /* USB */
#  define GCLK_CLKCTRL_ID_EVSYS(n)   (((n)+7) << GCLK_CLKCTRL_ID_SHIFT)
#  define GCLK_CLKCTRL_ID_EVSYS0     (7 << GCLK_CLKCTRL_ID_SHIFT)  /* EVSYS_CHANNEL_0 */
#  define GCLK_CLKCTRL_ID_EVSYS1     (8 << GCLK_CLKCTRL_ID_SHIFT)  /* EVSYS_CHANNEL_1 */
#  define GCLK_CLKCTRL_ID_EVSYS2     (9 << GCLK_CLKCTRL_ID_SHIFT)  /* EVSYS_CHANNEL_2 */
#  define GCLK_CLKCTRL_ID_EVSYS3     (10 << GCLK_CLKCTRL_ID_SHIFT) /* EVSYS_CHANNEL_3 */
#  define GCLK_CLKCTRL_ID_EVSYS4     (11 << GCLK_CLKCTRL_ID_SHIFT) /* EVSYS_CHANNEL_4 */
#  define GCLK_CLKCTRL_ID_EVSYS5     (12 << GCLK_CLKCTRL_ID_SHIFT) /* EVSYS_CHANNEL_5 */
#  define GCLK_CLKCTRL_ID_EVSYS6     (13 << GCLK_CLKCTRL_ID_SHIFT) /* EVSYS_CHANNEL_6 */
#  define GCLK_CLKCTRL_ID_EVSYS7     (14 << GCLK_CLKCTRL_ID_SHIFT) /* EVSYS_CHANNEL_7 */
#  define GCLK_CLKCTRL_ID_EVSYS8     (15 << GCLK_CLKCTRL_ID_SHIFT) /* EVSYS_CHANNEL_8 */
#  define GCLK_CLKCTRL_ID_EVSYS9     (16 << GCLK_CLKCTRL_ID_SHIFT) /* EVSYS_CHANNEL_9 */
#  define GCLK_CLKCTRL_ID_EVSYS10    (17 << GCLK_CLKCTRL_ID_SHIFT) /* EVSYS_CHANNEL_10 */
#  define GCLK_CLKCTRL_ID_EVSYS11    (18 << GCLK_CLKCTRL_ID_SHIFT) /* EVSYS_CHANNEL_11 */
#  define GCLK_CLKCTRL_ID_SERCOMS    (19 << GCLK_CLKCTRL_ID_SHIFT) /* SERCOMx_SLOW */
#  define GCLK_CLKCTRL_ID_SERCOMC(n) (((n)+20) << GCLK_CLKCTRL_ID_SHIFT)
#  define GCLK_CLKCTRL_ID_SERCOM0C   (20 << GCLK_CLKCTRL_ID_SHIFT) /* SERCOM0_CORE */
#  define GCLK_CLKCTRL_ID_SERCOM1C   (21 << GCLK_CLKCTRL_ID_SHIFT) /* SERCOM1_CORE */
#  define GCLK_CLKCTRL_ID_SERCOM2C   (22 << GCLK_CLKCTRL_ID_SHIFT) /* SERCOM2_CORE */
#  define GCLK_CLKCTRL_ID_SERCOM3C   (23 << GCLK_CLKCTRL_ID_SHIFT) /* SERCOM3_CORE */
#  define GCLK_CLKCTRL_ID_SERCOM4C   (24 << GCLK_CLKCTRL_ID_SHIFT) /* SERCOM4_CORE */
#  define GCLK_CLKCTRL_ID_SERCOM5C   (25 << GCLK_CLKCTRL_ID_SHIFT) /* SERCOM5_CORE */
#  define GCLK_CLKCTRL_ID_TCC01      (26 << GCLK_CLKCTRL_ID_SHIFT) /* TCC0,TCC1 */
#  define GCLK_CLKCTRL_ID_TCC23      (27 << GCLK_CLKCTRL_ID_SHIFT) /* TCC2,TC3 */
#  define GCLK_CLKCTRL_ID_TC45       (28 << GCLK_CLKCTRL_ID_SHIFT) /* TC4,TC5 */
#  define GCLK_CLKCTRL_ID_TC67       (29 << GCLK_CLKCTRL_ID_SHIFT) /* TC6,TC7 */
#  define GCLK_CLKCTRL_ID_ADC        (30 << GCLK_CLKCTRL_ID_SHIFT) /* ADC */
#  define GCLK_CLKCTRL_ID_ACDIG      (31 << GCLK_CLKCTRL_ID_SHIFT) /* AC_DIG */
#  define GCLK_CLKCTRL_ID_ACANA      (32 << GCLK_CLKCTRL_ID_SHIFT) /* AC_ANA */
#  define GCLK_CLKCTRL_ID_DAC        (33 << GCLK_CLKCTRL_ID_SHIFT) /* DAC */
#  define GCLK_CLKCTRL_ID_PTC        (34 << GCLK_CLKCTRL_ID_SHIFT) /* PTC */
#  define GCLK_CLKCTRL_ID_I2S0       (35 << GCLK_CLKCTRL_ID_SHIFT) /* I2S0 */
#  define GCLK_CLKCTRL_ID_I2S1       (36 << GCLK_CLKCTRL_ID_SHIFT) /* I2S1 */
#endif

#define GCLK_CLKCTRL_GEN_SHIFT  (8)       /* Bits 8-11: Generic Clock Generator */
#define GCLK_CLKCTRL_GEN_MASK   (15 << GCLK_CLKCTRL_GEN_SHIFT)
#  define GCLK_CLKCTRL_GEN(n)   ((n) << GCLK_CLKCTRL_GEN_SHIFT) /* Generic clock generator n */
#  define GCLK_CLKCTRL_GEN0     (0 << GCLK_CLKCTRL_GEN_SHIFT)   /* Generic clock generator 0 */
#  define GCLK_CLKCTRL_GEN1     (1 << GCLK_CLKCTRL_GEN_SHIFT)   /* Generic clock generator 1 */
#  define GCLK_CLKCTRL_GEN2     (2 << GCLK_CLKCTRL_GEN_SHIFT)   /* Generic clock generator 2 */
#  define GCLK_CLKCTRL_GEN3     (3 << GCLK_CLKCTRL_GEN_SHIFT)   /* Generic clock generator 3 */
#  define GCLK_CLKCTRL_GEN4     (4 << GCLK_CLKCTRL_GEN_SHIFT)   /* Generic clock generator 4 */
#  define GCLK_CLKCTRL_GEN5     (5 << GCLK_CLKCTRL_GEN_SHIFT)   /* Generic clock generator 5 */
#  define GCLK_CLKCTRL_GEN6     (6 << GCLK_CLKCTRL_GEN_SHIFT)   /* Generic clock generator 6 */
#  define GCLK_CLKCTRL_GEN7     (7 << GCLK_CLKCTRL_GEN_SHIFT)   /* Generic clock generator 7 */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define GCLK_CLKCTRL_GEN8     (8 << GCLK_CLKCTRL_GEN_SHIFT)   /* Generic clock generator 8 */
#endif

#define GCLK_CLKCTRL_CLKEN      (1 << 14) /* Bit 14: Clock Enable */
#define GCLK_CLKCTRL_WRTLOCK    (1 << 15) /* Bit 15: Write Lock */

/* Generic clock generator control register */

#define GCLK_GENCTRL_ID_SHIFT   (0)       /* Bits 0-3: Generic Clock Generator Selection */
#define GCLK_GENCTRL_ID_MASK    (15 << GCLK_GENCTRL_ID_SHIFT)
#  define GCLK_GENCTRL_ID(n)    ((n) << GCLK_GENCTRL_ID_SHIFT) /* Generic clock generator n */
#  define GCLK_GENCTRL_ID0      (0 << GCLK_GENCTRL_ID_SHIFT)   /* Generic clock generator 0 */
#  define GCLK_GENCTRL_ID1      (1 << GCLK_GENCTRL_ID_SHIFT)   /* Generic clock generator 1 */
#  define GCLK_GENCTRL_ID2      (2 << GCLK_GENCTRL_ID_SHIFT)   /* Generic clock generator 2 */
#  define GCLK_GENCTRL_ID3      (3 << GCLK_GENCTRL_ID_SHIFT)   /* Generic clock generator 3 */
#  define GCLK_GENCTRL_ID4      (4 << GCLK_GENCTRL_ID_SHIFT)   /* Generic clock generator 4 */
#  define GCLK_GENCTRL_ID5      (5 << GCLK_GENCTRL_ID_SHIFT)   /* Generic clock generator 5 */
#  define GCLK_GENCTRL_ID6      (6 << GCLK_GENCTRL_ID_SHIFT)   /* Generic clock generator 6 */
#  define GCLK_GENCTRL_ID7      (7 << GCLK_GENCTRL_ID_SHIFT)   /* Generic clock generator 7 */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define GCLK_GENCTRL_ID8      (8 << GCLK_GENCTRL_ID_SHIFT)   /* Generic clock generator 8 */
#endif

#define GCLK_GENCTRL_SRC_SHIFT  (8)       /* Bits 8-12: Source Select */
#define GCLK_GENCTRL_SRC_MASK   (31 << GCLK_GENCTRL_SRC_SHIFT)
#  define GCLK_GENCTRL_SRC_XOSC      (0 << GCLK_GENCTRL_SRC_SHIFT) /* XOSC oscillator output */
#  define GCLK_GENCTRL_SRC_GCLKIN    (1 << GCLK_GENCTRL_SRC_SHIFT) /* Generator input pad */
#  define GCLK_GENCTRL_SRC_GCLKGEN1  (2 << GCLK_GENCTRL_SRC_SHIFT) /* Generic clock generator 1 output */
#  define GCLK_GENCTRL_SRC_OSCULP32K (3 << GCLK_GENCTRL_SRC_SHIFT) /* OSCULP32K oscillator output */
#  define GCLK_GENCTRL_SRC_OSC32K    (4 << GCLK_GENCTRL_SRC_SHIFT) /* OSC32K oscillator output */
#  define GCLK_GENCTRL_SRC_XOSC32K   (5 << GCLK_GENCTRL_SRC_SHIFT) /* XOSC32K oscillator output */
#  define GCLK_GENCTRL_SRC_OSC8M     (6 << GCLK_GENCTRL_SRC_SHIFT) /* OSC8M oscillator output */
#  define GCLK_GENCTRL_SRC_DFLL48M   (7 << GCLK_GENCTRL_SRC_SHIFT) /* DFLL48M output */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define GCLK_GENCTRL_SRC_FDPLL96M  (8 << GCLK_GENCTRL_SRC_SHIFT) /* FDPLL96M output */
#endif

#define GCLK_GENCTRL_GENEN      (1 << 16) /* Bit 16: Generic Clock Generator Enable */
#define GCLK_GENCTRL_IDC        (1 << 17) /* Bit 17: Improve Duty Cycle */
#define GCLK_GENCTRL_OOV        (1 << 18) /* Bit 18: Output Off Value */
#define GCLK_GENCTRL_OE         (1 << 19) /* Bit 19: Output Enable */
#define GCLK_GENCTRL_DIVSEL     (1 << 20) /* Bit 20: Divide Selection */
#define GCLK_GENCTRL_RUNSTDBY   (1 << 21) /* Bit 21: Run in Standby */

/* Generic clock generator division register */

#define GCLK_GENDIV_ID_SHIFT    (0)       /* Bits 0-3: Generic Clock Generator Selection */
#define GCLK_GENDIV_ID_MASK     (15 << GCLK_GENDIV_ID_SHIFT)
#  define GCLK_GENDIV_ID(n)     ((n) << GCLK_GENDIV_ID_SHIFT) /* Generic clock generator n */
#  define GCLK_GENDIV_ID0       (0 << GCLK_GENDIV_ID_SHIFT)   /* Generic clock generator 0 */
#  define GCLK_GENDIV_ID1       (1 << GCLK_GENDIV_ID_SHIFT)   /* Generic clock generator 1 */
#  define GCLK_GENDIV_ID2       (2 << GCLK_GENDIV_ID_SHIFT)   /* Generic clock generator 2 */
#  define GCLK_GENDIV_ID3       (3 << GCLK_GENDIV_ID_SHIFT)   /* Generic clock generator 3 */
#  define GCLK_GENDIV_ID4       (4 << GCLK_GENDIV_ID_SHIFT)   /* Generic clock generator 4 */
#  define GCLK_GENDIV_ID5       (5 << GCLK_GENDIV_ID_SHIFT)   /* Generic clock generator 5 */
#  define GCLK_GENDIV_ID6       (6 << GCLK_GENDIV_ID_SHIFT)   /* Generic clock generator 6 */
#  define GCLK_GENDIV_ID7       (7 << GCLK_GENDIV_ID_SHIFT)   /* Generic clock generator 7 */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define GCLK_GENDIV_ID8       (8 << GCLK_GENCTRL_ID_SHIFT)  /* Generic clock generator 8 */
#endif

#define GCLK_GENDIV_DIV_SHIFT   (8)       /* Bits 8-23: Division Factor */
#define GCLK_GENDIV_DIV_MASK    (0xffff << GCLK_GENDIV_DIV_SHIFT)
#  define GCLK_GENDIV_DIV(n)    ((n) << GCLK_GENDIV_DIV_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAMD20 || CONFIG_ARCH_FAMILY_SAMD21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_GCLK_H */
