/****************************************************************************
 * arch/arm/src/max326xx/hardware/max32660_gpio.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_GPIO_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX326_GPIO0_NPINS             14      /* Pins 0-13 (max) */
#define MAX326_GPIO0_ALLPINS           0x3fff  /* Set of all pins */

/* Register Offsets *********************************************************/

#define MAX326_GPIO_AF0SEL_OFFSET      0x0000  /* I/O and Alternate Function 1
                                                * Select Register */
#define MAX326_GPIO_OUTEN_OFFSET       0x000c  /* Output Enable Register */
#define MAX326_GPIO_OUT_OFFSET         0x0018  /* Output Register */
#define MAX326_GPIO_IN_OFFSET          0x0024  /* Input Register */
#define MAX326_GPIO_INTMODE_OFFSET     0x0028  /* Interrupt Mode Register */
#define MAX326_GPIO_INTPOL_OFFSET      0x002c  /* Interrupt Polarity Select
                                                * Register */
#define MAX326_GPIO_INTEN_OFFSET       0x0034  /* Interrupt Enable Register */
#define MAX326_GPIO_INTFL_OFFSET       0x0040  /* Interrupt Flag Register */
#define MAX326_GPIO_WAKEEN_OFFSET      0x004c  /* Wakeup Enable Register */
#define MAX326_GPIO_INTDUALEDGE_OFFSET 0x005c  /* Dual Edge Select Interrupt
                                                * Register */
#define MAX326_GPIO_PULLEN_OFFSET      0x0060  /* Input Pullup/Pulldown Select
                                                * Register */
#define MAX326_GPIO_AF1SEL_OFFSET      0x0068  /* Alternate Function 2/3 Select
                                                * Register */
#define MAX326_GPIO_INHYSEN_OFFSET     0x00a8  /* Input Hysteresis Enable Register */
#define MAX326_GPIO_SRSEL_OFFSET       0x00ac  /* Slew Rate Select Register */
#define MAX326_GPIO_DS0SEL_OFFSET      0x00b0  /* Drive Strength Select 0 Register */
#define MAX326_GPIO_DS1SEL_OFFSET      0x00b4  /* Drive Strength Select 1 Register */
#define MAX326_GPIO_PULLSEL_OFFSET     0x00b8  /* Pullup/Pulldown Enable Register */

/* Register Addresses *******************************************************/

#define MAX326_GPIO0_AF0SEL            (MAX326_GPIO_BASE + MAX326_GPIO_AF0SEL_OFFSET)
#define MAX326_GPIO0_OUTEN             (MAX326_GPIO_BASE + MAX326_GPIO_OUTEN_OFFSET)
#define MAX326_GPIO0_OUT               (MAX326_GPIO_BASE + MAX326_GPIO_OUT_OFFSET)
#define MAX326_GPIO0_IN                (MAX326_GPIO_BASE + MAX326_GPIO_IN_OFFSET)
#define MAX326_GPIO0_INTMODE           (MAX326_GPIO_BASE + MAX326_GPIO_INTMODE_OFFSET)
#define MAX326_GPIO0_INTPOL            (MAX326_GPIO_BASE + MAX326_GPIO_INTPOL_OFFSET)
#define MAX326_GPIO0_INTEN             (MAX326_GPIO_BASE + MAX326_GPIO_INTEN_OFFSET)
#define MAX326_GPIO0_INTFL             (MAX326_GPIO_BASE + MAX326_GPIO_INTFL_OFFSET)
#define MAX326_GPIO0_WAKEEN            (MAX326_GPIO_BASE + MAX326_GPIO_WAKEEN_OFFSET)
#define MAX326_GPIO0_INTDUALEDGE       (MAX326_GPIO_BASE + MAX326_GPIO_INTDUALEDGE_OFFSET)
#define MAX326_GPIO0_PULLEN            (MAX326_GPIO_BASE + MAX326_GPIO_PULLEN_OFFSET)
#define MAX326_GPIO0_AF1SEL            (MAX326_GPIO_BASE + MAX326_GPIO_AF1SEL_OFFSET)
#define MAX326_GPIO0_INHYSEN           (MAX326_GPIO_BASE + MAX326_GPIO_INHYSEN_OFFSET)
#define MAX326_GPIO0_SRSEL             (MAX326_GPIO_BASE + MAX326_GPIO_SRSEL_OFFSET)
#define MAX326_GPIO0_DS0SEL            (MAX326_GPIO_BASE + MAX326_GPIO_DS0SEL_OFFSET)
#define MAX326_GPIO0_DS1SEL            (MAX326_GPIO_BASE + MAX326_GPIO_DS1SEL_OFFSET)
#define MAX326_GPIO0_PULLSEL           (MAX326_GPIO_BASE + MAX326_GPIO_PULLSEL_OFFSET)

/* Register Bit-field Definitions *******************************************/

/* I/O and Alternate Function 1 Select Register */

#define GPIO_AF0SEL(n)                 (1 << (n))  /* Pin n GPIO instead of Default function */
#  define GPIO_AF0SEL_TDOGPIO          (1 << 0)    /* GPIO instead of JTAG TCK/SWCLK */
#  define GPIO_AF0SEL_TMSGPIO          (1 << 1)    /* GPIO instead of JTAG TMS/SWDIO */
#  define GPIO_AF0SEL_TCKGPIO          (1 << 2)    /* GPIO instead of JTAG TDO */

/* Output Enable Register */

#define GPIO_OUTEN(n)                  (1 << (n)) /* GPIO Pin n output mode enabled */

/* Output Register */

#define GPIO_OUT(n)                    (1 << (n)) /* GPIO Pin n output level */

/* Input Register */

#define GPIO_IN(n)                     (1 << (n)) /* GPIO Pin n input level */

/* Interrupt Mode Register */

#define GPIO_INTMODE_EDGE(n)           (1 << (n)) /* GPIO Pin n edge (vs level) triggered */

/* Interrupt Polarity Select Register */

#define GPIO_INTPOL_HIGH(n)            (1 << (n)) /* GPIO Pin n high level triggers */
#define GPIO_INTPOL_RISING(n)          (1 << (n)) /* GPIO Pin n rising edge triggers */

/* Interrupt Enable Register */

#define GPIO_INTEN(n)                  (1 << (n)) /* GPIO Pin n interrupt enabled */

/* Interrupt Flag Register */

#define GPIO_INTFL(n)                  (1 << (n)) /* GPIO Pin n interrupt pending */

/* Wakeup Enable Register */

#define GPIO_WAKEEN(n)                 (1 << (n)) /* GPIO Pin n wakeup enable */

/* Dual Edge Select Interrupt Register */

#define GPIO_INTDUALEDGE(n)            (1 << (n)) /* GPIO Pin n Dual edge detection
                                                   * mode interrupts enabled */

/* Input Pullup/Pulldown Select Register */

#define GPIO_PULLEN(n)                 (1 << (n)) /* GPIO Pin n:
                                                   * n=0-1, 4-7, 10-13: Pull up/down enable
                                                   * n=8-9: Pull down enable */

/* Alternate Function 2/3 Select Register */

#define GPIO_AF1SEL(n)                 (1 << (n)) /* GPIO Pin n alternate function 1
                                                   * mode select */

/* Input Hysteresis Enable Register */

#define GPIO_INHYSEN(n)                (1 << (n)) /* GPIO Pin n input hysteresis
                                                   * enable */

/* Slew Rate Select Register */

#define GPIO_SRSEL(n)                  (1 << (n)) /* GPIO Pin n Slew rate mode
                                                   * n=4-7, 10-13: Slow slew rate
                                                   * selected */

/* Drive Strength Select 0 Register */

#define GPIO_DS0SEL(n)                 (1 << (n)) /* GPIO Pin n Drive strength 0
                                                   * select:
                                                   * n=0-1, 4-7, 10-13: See DS1SEL
                                                   * n=2-3, 8-9:  High drive strength */

/* Drive Strength Select 1 Register */

#define GPIO_DS1SEL(n)                 (1 << (n)) /* GPIO Pin n Pullup/Pulldown
                                                   * Resistor Select:
                                                   * n=0-1, 4-7, 10-13: 2-bit drive
                                                   * strength */

/* Pullup/Pulldown Enable Register */

#define GPIO_PULLSEL(n)                (1 << (n)) /* GPIO Pin n Pullup/Pulldown
                                                   * Resistor Select:
                                                   * n=0-1, 4-7, 10-13: Pullup select */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_GPIO_H */
