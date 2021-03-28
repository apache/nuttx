/****************************************************************************
 * arch/arm/src/samd2l2/hardware/samd20_pinmap.h
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
 */

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD20_PINMAP_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD20_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO pin definitions *****************************************************/

/* Alternate Pin Functions.
 *
 * Alternative pin selections are provided with a numeric suffix like _1, _2,
 * etc. Drivers, however, will use the pin selection without the numeric
 * suffix. Additional definitions are required in the board.h file.
 * For example, if we wanted the SERCOM0 PAD0 on PA8, then the following
 * definition should appear in the board.h header file for that board:
 *
 * #define PORT_SERCOM0_PAD0 PORT_SERCOM0_PAD0_1
 *
 * The driver will then automatically configure PA8 as the SERCOM0 PAD0
 * pin.
 */

/* WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!!
 * Additional effort is required to select specific GPIO options such as
 * frequency, open-drain/push-pull, and pull-up/down!  Just the basics are
 * defined for most pins in this file.
 */

/* Analog comparator */

#define PORT_AC_CMP0_1       (PORT_FUNCH | PORTA | PORT_PIN12)
#define PORT_AC_CMP0_2       (PORT_FUNCH | PORTA | PORT_PIN18)
#define PORT_AC_CMP1_1       (PORT_FUNCH | PORTA | PORT_PIN13)
#define PORT_AC_CMP1_2       (PORT_FUNCH | PORTA | PORT_PIN19)

/* ADC voltage references */

#define PORT_ADC_VREFA       (PORT_FUNCB | PORTA | PORT_PIN3)
#define PORT_ADC_VREFB       (PORT_FUNCB | PORTA | PORT_PIN4)

#define PORT_AIN0_1          (PORT_FUNCB | PORTA | PORT_PIN2)
#define PORT_AIN0_2          (PORT_FUNCB | PORTA | PORT_PIN4)
#define PORT_AIN1_1          (PORT_FUNCB | PORTA | PORT_PIN3)
#define PORT_AIN1_2          (PORT_FUNCB | PORTA | PORT_PIN5)
#define PORT_AIN2_1          (PORT_FUNCB | PORTA | PORT_PIN6)
#define PORT_AIN2_2          (PORT_FUNCB | PORTB | PORT_PIN8)
#define PORT_AIN3_1          (PORT_FUNCB | PORTA | PORT_PIN7)
#define PORT_AIN3_2          (PORT_FUNCB | PORTB | PORT_PIN9)
#define PORT_AIN4            (PORT_FUNCB | PORTA | PORT_PIN4)
#define PORT_AIN5            (PORT_FUNCB | PORTA | PORT_PIN5)
#define PORT_AIN6            (PORT_FUNCB | PORTA | PORT_PIN6)
#define PORT_AIN7            (PORT_FUNCB | PORTA | PORT_PIN7)
#define PORT_AIN8            (PORT_FUNCB | PORTB | PORT_PIN0)
#define PORT_AIN9            (PORT_FUNCB | PORTB | PORT_PIN1)
#define PORT_AIN10           (PORT_FUNCB | PORTB | PORT_PIN2)
#define PORT_AIN11           (PORT_FUNCB | PORTB | PORT_PIN3)
#define PORT_AIN12           (PORT_FUNCB | PORTB | PORT_PIN4)
#define PORT_AIN13           (PORT_FUNCB | PORTB | PORT_PIN5)
#define PORT_AIN14           (PORT_FUNCB | PORTB | PORT_PIN6)
#define PORT_AIN15           (PORT_FUNCB | PORTB | PORT_PIN7)
#define PORT_AIN16           (PORT_FUNCB | PORTA | PORT_PIN8)
#define PORT_AIN17           (PORT_FUNCB | PORTA | PORT_PIN9)
#define PORT_AIN18           (PORT_FUNCB | PORTA | PORT_PIN10)
#define PORT_AIN19           (PORT_FUNCB | PORTA | PORT_PIN11)

/* DAC */

#define PORT_DAC_VREFA       (PORT_FUNCB | PORTA | PORT_PIN3)
#define PORT_DAC_VOUT        (PORT_FUNCB | PORTA | PORT_PIN2)

/* External interrupts */

#define PORT_EXTINT0_1       (PORT_FUNCA | PORTA | PORT_PIN0)
#define PORT_EXTINT0_2       (PORT_FUNCA | PORTA | PORT_PIN16)
#define PORT_EXTINT0_3       (PORT_FUNCA | PORTB | PORT_PIN0)
#define PORT_EXTINT0_4       (PORT_FUNCA | PORTB | PORT_PIN16)
#define PORT_EXTINT1_1       (PORT_FUNCA | PORTA | PORT_PIN1)
#define PORT_EXTINT1_2       (PORT_FUNCA | PORTA | PORT_PIN17)
#define PORT_EXTINT1_3       (PORT_FUNCA | PORTB | PORT_PIN1)
#define PORT_EXTINT1_4       (PORT_FUNCA | PORTB | PORT_PIN17)
#define PORT_EXTINT2_1       (PORT_FUNCA | PORTA | PORT_PIN18)
#define PORT_EXTINT2_2       (PORT_FUNCA | PORTA | PORT_PIN2)
#define PORT_EXTINT2_3       (PORT_FUNCA | PORTB | PORT_PIN2)
#define PORT_EXTINT3_1       (PORT_FUNCA | PORTA | PORT_PIN19)
#define PORT_EXTINT3_2       (PORT_FUNCA | PORTA | PORT_PIN3)
#define PORT_EXTINT3_3       (PORT_FUNCA | PORTB | PORT_PIN3)
#define PORT_EXTINT4_1       (PORT_FUNCA | PORTA | PORT_PIN20)
#define PORT_EXTINT4_2       (PORT_FUNCA | PORTA | PORT_PIN4)
#define PORT_EXTINT4_3       (PORT_FUNCA | PORTB | PORT_PIN4)
#define PORT_EXTINT5_1       (PORT_FUNCA | PORTA | PORT_PIN21)
#define PORT_EXTINT5_2       (PORT_FUNCA | PORTA | PORT_PIN5)
#define PORT_EXTINT5_3       (PORT_FUNCA | PORTB | PORT_PIN5)
#define PORT_EXTINT6_1       (PORT_FUNCA | PORTA | PORT_PIN22)
#define PORT_EXTINT6_2       (PORT_FUNCA | PORTA | PORT_PIN6)
#define PORT_EXTINT6_3       (PORT_FUNCA | PORTB | PORT_PIN22)
#define PORT_EXTINT6_4       (PORT_FUNCA | PORTB | PORT_PIN6)
#define PORT_EXTINT7_1       (PORT_FUNCA | PORTA | PORT_PIN23)
#define PORT_EXTINT7_2       (PORT_FUNCA | PORTA | PORT_PIN7)
#define PORT_EXTINT7_3       (PORT_FUNCA | PORTB | PORT_PIN23)
#define PORT_EXTINT7_4       (PORT_FUNCA | PORTB | PORT_PIN7)
#define PORT_EXTINT8_1       (PORT_FUNCA | PORTA | PORT_PIN28)
#define PORT_EXTINT8_2       (PORT_FUNCA | PORTB | PORT_PIN8)
#define PORT_EXTINT9_1       (PORT_FUNCA | PORTA | PORT_PIN9)
#define PORT_EXTINT9_2       (PORT_FUNCA | PORTB | PORT_PIN9)
#define PORT_EXTINT10_1      (PORT_FUNCA | PORTA | PORT_PIN10)
#define PORT_EXTINT10_2      (PORT_FUNCA | PORTA | PORT_PIN30)
#define PORT_EXTINT10_3      (PORT_FUNCA | PORTB | PORT_PIN10)
#define PORT_EXTINT11_1      (PORT_FUNCA | PORTA | PORT_PIN11)
#define PORT_EXTINT11_2      (PORT_FUNCA | PORTA | PORT_PIN31)
#define PORT_EXTINT11_3      (PORT_FUNCA | PORTB | PORT_PIN11)
#define PORT_EXTINT12_1      (PORT_FUNCA | PORTA | PORT_PIN12)
#define PORT_EXTINT12_2      (PORT_FUNCA | PORTA | PORT_PIN24)
#define PORT_EXTINT12_3      (PORT_FUNCA | PORTB | PORT_PIN12)
#define PORT_EXTINT13_1      (PORT_FUNCA | PORTA | PORT_PIN13)
#define PORT_EXTINT13_2      (PORT_FUNCA | PORTA | PORT_PIN25)
#define PORT_EXTINT13_3      (PORT_FUNCA | PORTB | PORT_PIN13)
#define PORT_EXTINT14_1      (PORT_FUNCA | PORTA | PORT_PIN14)
#define PORT_EXTINT14_2      (PORT_FUNCA | PORTB | PORT_PIN14)
#define PORT_EXTINT14_3      (PORT_FUNCA | PORTB | PORT_PIN30)
#define PORT_EXTINT15_1      (PORT_FUNCA | PORTA | PORT_PIN15)
#define PORT_EXTINT15_2      (PORT_FUNCA | PORTA | PORT_PIN27)
#define PORT_EXTINT15_3      (PORT_FUNCA | PORTB | PORT_PIN15)
#define PORT_EXTINT15_4      (PORT_FUNCA | PORTB | PORT_PIN31)

/* Generic clock controller I/O */

#define PORT_GCLK_IO0_1      (PORT_FUNCH | PORTA | PORT_PIN14)
#define PORT_GCLK_IO0_2      (PORT_FUNCH | PORTA | PORT_PIN27)
#define PORT_GCLK_IO0_3      (PORT_FUNCH | PORTA | PORT_PIN28)
#define PORT_GCLK_IO0_4      (PORT_FUNCH | PORTA | PORT_PIN30)
#define PORT_GCLK_IO0_5      (PORT_FUNCH | PORTB | PORT_PIN14)
#define PORT_GCLK_IO0_6      (PORT_FUNCH | PORTB | PORT_PIN22)
#define PORT_GCLK_IO1_1      (PORT_FUNCH | PORTA | PORT_PIN15)
#define PORT_GCLK_IO1_2      (PORT_FUNCH | PORTB | PORT_PIN15)
#define PORT_GCLK_IO1_3      (PORT_FUNCH | PORTB | PORT_PIN23)
#define PORT_GCLK_IO2_1      (PORT_FUNCH | PORTA | PORT_PIN16)
#define PORT_GCLK_IO2_2      (PORT_FUNCH | PORTB | PORT_PIN16)
#define PORT_GCLK_IO3_1      (PORT_FUNCH | PORTA | PORT_PIN17)
#define PORT_GCLK_IO3_2      (PORT_FUNCH | PORTB | PORT_PIN17)
#define PORT_GCLK_IO4_1      (PORT_FUNCH | PORTA | PORT_PIN10)
#define PORT_GCLK_IO4_2      (PORT_FUNCH | PORTA | PORT_PIN20)
#define PORT_GCLK_IO4_3      (PORT_FUNCH | PORTB | PORT_PIN10)
#define PORT_GCLK_IO5_1      (PORT_FUNCH | PORTA | PORT_PIN11)
#define PORT_GCLK_IO5_2      (PORT_FUNCH | PORTA | PORT_PIN21)
#define PORT_GCLK_IO5_3      (PORT_FUNCH | PORTB | PORT_PIN11)
#define PORT_GCLK_IO6_1      (PORT_FUNCH | PORTA | PORT_PIN22)
#define PORT_GCLK_IO6_2      (PORT_FUNCH | PORTB | PORT_PIN12)
#define PORT_GCLK_IO7_1      (PORT_FUNCH | PORTA | PORT_PIN23)
#define PORT_GCLK_IO7_2      (PORT_FUNCH | PORTB | PORT_PIN13)

/* Non maskable interrupt */

#define PORT_NMI             (PORT_FUNCA | PORTA | PORT_PIN8)

/* Serial communication interface (SERCOM) */

#define PORT_SERCOM0_PAD0_1  (PORT_FUNCC | PORTA | PORT_PIN8)
#define PORT_SERCOM0_PAD0_2  (PORT_FUNCD | PORTA | PORT_PIN4)
#define PORT_SERCOM0_PAD1_1  (PORT_FUNCC | PORTA | PORT_PIN9)
#define PORT_SERCOM0_PAD1_2  (PORT_FUNCD | PORTA | PORT_PIN5)
#define PORT_SERCOM0_PAD2_1  (PORT_FUNCC | PORTA | PORT_PIN10)
#define PORT_SERCOM0_PAD2_2  (PORT_FUNCD | PORTA | PORT_PIN6)
#define PORT_SERCOM0_PAD3_1  (PORT_FUNCC | PORTA | PORT_PIN11)
#define PORT_SERCOM0_PAD3_2  (PORT_FUNCD | PORTA | PORT_PIN7)
#define PORT_SERCOM1_PAD0_1  (PORT_FUNCC | PORTA | PORT_PIN16)
#define PORT_SERCOM1_PAD0_2  (PORT_FUNCD | PORTA | PORT_PIN0)
#define PORT_SERCOM1_PAD1_1  (PORT_FUNCC | PORTA | PORT_PIN17)
#define PORT_SERCOM1_PAD1_2  (PORT_FUNCD | PORTA | PORT_PIN1)
#define PORT_SERCOM1_PAD2_1  (PORT_FUNCC | PORTA | PORT_PIN18)
#define PORT_SERCOM1_PAD2_2  (PORT_FUNCD | PORTA | PORT_PIN30)
#define PORT_SERCOM1_PAD3_1  (PORT_FUNCC | PORTA | PORT_PIN19)
#define PORT_SERCOM1_PAD3_2  (PORT_FUNCD | PORTA | PORT_PIN31)
#define PORT_SERCOM2_PAD0_1  (PORT_FUNCC | PORTA | PORT_PIN12)
#define PORT_SERCOM2_PAD0_2  (PORT_FUNCD | PORTA | PORT_PIN8)
#define PORT_SERCOM2_PAD1_1  (PORT_FUNCC | PORTA | PORT_PIN13)
#define PORT_SERCOM2_PAD1_2  (PORT_FUNCD | PORTA | PORT_PIN9)
#define PORT_SERCOM2_PAD2_1  (PORT_FUNCC | PORTA | PORT_PIN14)
#define PORT_SERCOM2_PAD2_2  (PORT_FUNCD | PORTA | PORT_PIN10)
#define PORT_SERCOM2_PAD3_1  (PORT_FUNCC | PORTA | PORT_PIN15)
#define PORT_SERCOM2_PAD3_2  (PORT_FUNCD | PORTA | PORT_PIN11)
#define PORT_SERCOM3_PAD0_1  (PORT_FUNCC | PORTA | PORT_PIN22)
#define PORT_SERCOM3_PAD0_2  (PORT_FUNCD | PORTA | PORT_PIN16)
#define PORT_SERCOM3_PAD1_1  (PORT_FUNCC | PORTA | PORT_PIN23)
#define PORT_SERCOM3_PAD1_2  (PORT_FUNCD | PORTA | PORT_PIN17)
#define PORT_SERCOM3_PAD2_1  (PORT_FUNCC | PORTA | PORT_PIN24)
#define PORT_SERCOM3_PAD2_2  (PORT_FUNCD | PORTA | PORT_PIN18)
#define PORT_SERCOM3_PAD2_3  (PORT_FUNCD | PORTA | PORT_PIN20)
#define PORT_SERCOM3_PAD3_1  (PORT_FUNCC | PORTA | PORT_PIN25)
#define PORT_SERCOM3_PAD3_2  (PORT_FUNCD | PORTA | PORT_PIN19)
#define PORT_SERCOM3_PAD3_3  (PORT_FUNCD | PORTA | PORT_PIN21)
#define PORT_SERCOM4_PAD0_1  (PORT_FUNCC | PORTB | PORT_PIN12)
#define PORT_SERCOM4_PAD0_2  (PORT_FUNCD | PORTA | PORT_PIN12)
#define PORT_SERCOM4_PAD0_3  (PORT_FUNCD | PORTB | PORT_PIN8)
#define PORT_SERCOM4_PAD1_1  (PORT_FUNCC | PORTB | PORT_PIN13)
#define PORT_SERCOM4_PAD1_2  (PORT_FUNCD | PORTA | PORT_PIN13)
#define PORT_SERCOM4_PAD1_3  (PORT_FUNCD | PORTB | PORT_PIN9)
#define PORT_SERCOM4_PAD2_1  (PORT_FUNCC | PORTB | PORT_PIN14)
#define PORT_SERCOM4_PAD2_2  (PORT_FUNCD | PORTA | PORT_PIN14)
#define PORT_SERCOM4_PAD2_3  (PORT_FUNCD | PORTB | PORT_PIN10)
#define PORT_SERCOM4_PAD3_1  (PORT_FUNCC | PORTB | PORT_PIN15)
#define PORT_SERCOM4_PAD3_2  (PORT_FUNCD | PORTA | PORT_PIN15)
#define PORT_SERCOM4_PAD3_3  (PORT_FUNCD | PORTB | PORT_PIN11)
#define PORT_SERCOM5_PAD0_1  (PORT_FUNCC | PORTB | PORT_PIN16)
#define PORT_SERCOM5_PAD0_2  (PORT_FUNCD | PORTA | PORT_PIN22)
#define PORT_SERCOM5_PAD0_3  (PORT_FUNCD | PORTB | PORT_PIN2)
#define PORT_SERCOM5_PAD0_4  (PORT_FUNCD | PORTB | PORT_PIN30)
#define PORT_SERCOM5_PAD1_1  (PORT_FUNCC | PORTB | PORT_PIN17)
#define PORT_SERCOM5_PAD1_2  (PORT_FUNCD | PORTA | PORT_PIN23)
#define PORT_SERCOM5_PAD1_3  (PORT_FUNCD | PORTB | PORT_PIN3)
#define PORT_SERCOM5_PAD1_4  (PORT_FUNCD | PORTB | PORT_PIN31)
#define PORT_SERCOM5_PAD2_1  (PORT_FUNCC | PORTA | PORT_PIN20)
#define PORT_SERCOM5_PAD2_2  (PORT_FUNCD | PORTA | PORT_PIN24)
#define PORT_SERCOM5_PAD2_3  (PORT_FUNCD | PORTB | PORT_PIN0)
#define PORT_SERCOM5_PAD2_4  (PORT_FUNCD | PORTB | PORT_PIN22)
#define PORT_SERCOM5_PAD3_1  (PORT_FUNCC | PORTA | PORT_PIN21)
#define PORT_SERCOM5_PAD3_2  (PORT_FUNCD | PORTA | PORT_PIN25)
#define PORT_SERCOM5_PAD3_3  (PORT_FUNCD | PORTB | PORT_PIN1)
#define PORT_SERCOM5_PAD3_4  (PORT_FUNCD | PORTB | PORT_PIN23)

/* JTAG/SWI */

#define PORT_SWCLK           (PORT_FUNCG | PORTA | PORT_PIN30)
#define PORT_SWDIO           (PORT_FUNCG | PORTA | PORT_PIN31)

/* Timer/Counters */

#define PORT_TC0_WO0_1       (PORT_FUNCE | PORTA | PORT_PIN8)
#define PORT_TC0_WO0_2       (PORT_FUNCF | PORTA | PORT_PIN4)
#define PORT_TC0_WO0_3       (PORT_FUNCF | PORTB | PORT_PIN30)
#define PORT_TC0_WO1_1       (PORT_FUNCE | PORTA | PORT_PIN9)
#define PORT_TC0_WO1_2       (PORT_FUNCF | PORTA | PORT_PIN5)
#define PORT_TC0_WO1_3       (PORT_FUNCF | PORTB | PORT_PIN31)
#define PORT_TC1_WO0_1       (PORT_FUNCE | PORTA | PORT_PIN10)
#define PORT_TC1_WO0_2       (PORT_FUNCF | PORTA | PORT_PIN30)
#define PORT_TC1_WO0_3       (PORT_FUNCF | PORTA | PORT_PIN6)
#define PORT_TC1_WO1_1       (PORT_FUNCE | PORTA | PORT_PIN11)
#define PORT_TC1_WO1_2       (PORT_FUNCF | PORTA | PORT_PIN31)
#define PORT_TC1_WO1_3       (PORT_FUNCF | PORTA | PORT_PIN7)
#define PORT_TC2_WO0_1       (PORT_FUNCE | PORTA | PORT_PIN12)
#define PORT_TC2_WO0_2       (PORT_FUNCF | PORTA | PORT_PIN0)
#define PORT_TC2_WO0_3       (PORT_FUNCF | PORTA | PORT_PIN16)
#define PORT_TC2_WO1_1       (PORT_FUNCE | PORTA | PORT_PIN13)
#define PORT_TC2_WO1_2       (PORT_FUNCF | PORTA | PORT_PIN1)
#define PORT_TC2_WO1_3       (PORT_FUNCF | PORTA | PORT_PIN17)
#define PORT_TC3_WO0_1       (PORT_FUNCE | PORTA | PORT_PIN14)
#define PORT_TC3_WO0_2       (PORT_FUNCF | PORTA | PORT_PIN18)
#define PORT_TC3_WO1_1       (PORT_FUNCE | PORTA | PORT_PIN15)
#define PORT_TC3_WO1_2       (PORT_FUNCF | PORTA | PORT_PIN19)
#define PORT_TC4_WO0_1       (PORT_FUNCE | PORTB | PORT_PIN12)
#define PORT_TC4_WO0_2       (PORT_FUNCF | PORTA | PORT_PIN22)
#define PORT_TC4_WO0_3       (PORT_FUNCF | PORTB | PORT_PIN8)
#define PORT_TC4_WO1_1       (PORT_FUNCE | PORTB | PORT_PIN13)
#define PORT_TC4_WO1_2       (PORT_FUNCF | PORTA | PORT_PIN23)
#define PORT_TC4_WO1_3       (PORT_FUNCF | PORTB | PORT_PIN9)
#define PORT_TC5_WO0_1       (PORT_FUNCE | PORTB | PORT_PIN14)
#define PORT_TC5_WO0_2       (PORT_FUNCF | PORTA | PORT_PIN24)
#define PORT_TC5_WO0_3       (PORT_FUNCF | PORTB | PORT_PIN10)
#define PORT_TC5_WO1_1       (PORT_FUNCE | PORTB | PORT_PIN15)
#define PORT_TC5_WO1_2       (PORT_FUNCF | PORTA | PORT_PIN25)
#define PORT_TC5_WO1_3       (PORT_FUNCF | PORTB | PORT_PIN11)
#define PORT_TC6_WO0_1       (PORT_FUNCE | PORTB | PORT_PIN16)
#define PORT_TC6_WO0_2       (PORT_FUNCF | PORTB | PORT_PIN2)
#define PORT_TC6_WO1_1       (PORT_FUNCE | PORTB | PORT_PIN17)
#define PORT_TC6_WO1_2       (PORT_FUNCF | PORTB | PORT_PIN3)
#define PORT_TC7_WO0_1       (PORT_FUNCE | PORTA | PORT_PIN20)
#define PORT_TC7_WO0_2       (PORT_FUNCF | PORTB | PORT_PIN0)
#define PORT_TC7_WO0_3       (PORT_FUNCF | PORTB | PORT_PIN22)
#define PORT_TC7_WO1_1       (PORT_FUNCE | PORTA | PORT_PIN21)
#define PORT_TC7_WO1_2       (PORT_FUNCF | PORTB | PORT_PIN1)
#define PORT_TC7_WO1_3       (PORT_FUNCF | PORTB | PORT_PIN23)

/* Peripheral touch controller */

#define PORT_PTC_X0          (PORT_FUNCB | PORTA | PORT_PIN8)
#define PORT_PTC_X1          (PORT_FUNCB | PORTA | PORT_PIN9)
#define PORT_PTC_X2          (PORT_FUNCB | PORTA | PORT_PIN10)
#define PORT_PTC_X3          (PORT_FUNCB | PORTA | PORT_PIN11)
#define PORT_PTC_X4          (PORT_FUNCB | PORTA | PORT_PIN16)
#define PORT_PTC_X5          (PORT_FUNCB | PORTA | PORT_PIN17)
#define PORT_PTC_X6          (PORT_FUNCB | PORTA | PORT_PIN18)
#define PORT_PTC_X7          (PORT_FUNCB | PORTA | PORT_PIN19)
#define PORT_PTC_X8          (PORT_FUNCB | PORTA | PORT_PIN20)
#define PORT_PTC_X9          (PORT_FUNCB | PORTA | PORT_PIN21)
#define PORT_PTC_X10         (PORT_FUNCB | PORTA | PORT_PIN22)
#define PORT_PTC_X11         (PORT_FUNCB | PORTA | PORT_PIN23)
#define PORT_PTC_X12         (PORT_FUNCB | PORTB | PORT_PIN12)
#define PORT_PTC_X13         (PORT_FUNCB | PORTB | PORT_PIN13)
#define PORT_PTC_X14         (PORT_FUNCB | PORTB | PORT_PIN14)
#define PORT_PTC_X15         (PORT_FUNCB | PORTB | PORT_PIN15)

#define PORT_PTC_Y0          (PORT_FUNCB | PORTA | PORT_PIN2)
#define PORT_PTC_Y1          (PORT_FUNCB | PORTA | PORT_PIN3)
#define PORT_PTC_Y2          (PORT_FUNCB | PORTA | PORT_PIN4)
#define PORT_PTC_Y3          (PORT_FUNCB | PORTA | PORT_PIN5)
#define PORT_PTC_Y4          (PORT_FUNCB | PORTA | PORT_PIN6)
#define PORT_PTC_Y5          (PORT_FUNCB | PORTA | PORT_PIN7)
#define PORT_PTC_Y6          (PORT_FUNCB | PORTB | PORT_PIN0)
#define PORT_PTC_Y7          (PORT_FUNCB | PORTB | PORT_PIN1)
#define PORT_PTC_Y8          (PORT_FUNCB | PORTB | PORT_PIN2)
#define PORT_PTC_Y9          (PORT_FUNCB | PORTB | PORT_PIN3)
#define PORT_PTC_Y10         (PORT_FUNCB | PORTB | PORT_PIN4)
#define PORT_PTC_Y11         (PORT_FUNCB | PORTB | PORT_PIN5)
#define PORT_PTC_Y12         (PORT_FUNCB | PORTB | PORT_PIN6)
#define PORT_PTC_Y13         (PORT_FUNCB | PORTB | PORT_PIN7)
#define PORT_PTC_Y14         (PORT_FUNCB | PORTB | PORT_PIN8)
#define PORT_PTC_Y15         (PORT_FUNCB | PORTB | PORT_PIN9)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD20_PINMAP_H */
