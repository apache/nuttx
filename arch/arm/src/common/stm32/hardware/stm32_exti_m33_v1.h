/****************************************************************************
 * arch/arm/src/common/stm32/hardware/stm32_exti_m33_v1.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_EXTI_M33_V1_H
#define __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_EXTI_M33_V1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EXTI line inventory. */

#if defined(CONFIG_STM32_STM32L562XX)
#  define STM32_EXTI_NLINES  43
#elif defined(CONFIG_STM32_STM32U3C5XX)
#  define STM32_EXTI_NLINES  23
#elif defined(CONFIG_STM32_STM32U585XX)
#  define STM32_EXTI_NLINES  24
#elif defined(CONFIG_STM32_STM32U5A5XX)
#  define STM32_EXTI_NLINES  26
#elif defined(CONFIG_ARCH_CHIP_STM32C5)
#  define STM32_EXTI_NLINES  36
#elif defined(CONFIG_STM32_STM32H50XXX)
#  define STM32_EXTI_NLINES  54
#elif defined(CONFIG_STM32_STM32H53XXX)
#  define STM32_EXTI_NLINES  59
#elif defined(CONFIG_STM32_STM32H563XX)
#  define STM32_EXTI_NLINES  58
#else
#  error "Unsupported STM32 M33 EXTI line inventory"
#endif

/* Register availability and implemented lines depend on the chip.  Some
 * chips omit bank 2, security or lock registers.  The shared layout below
 * does not imply that every register or line is implemented.  In particular,
 * direct lines do not have trigger or pending bits.
 */

/* Register Offsets *********************************************************/

#define STM32_EXTI_RTSR1_OFFSET      0x0000  /* Rising Trigger Selection 1       */
#define STM32_EXTI_FTSR1_OFFSET      0x0004  /* Falling Trigger Selection 1      */
#define STM32_EXTI_SWIER1_OFFSET     0x0008  /* Software Interrupt Event 1       */
#define STM32_EXTI_RPR1_OFFSET       0x000c  /* Rising Edge Pending 1            */
#define STM32_EXTI_FPR1_OFFSET       0x0010  /* Falling Edge Pending 1           */
#define STM32_EXTI_SECCFGR1_OFFSET   0x0014  /* Security Configuration 1         */
#define STM32_EXTI_PRIVCFGR1_OFFSET  0x0018  /* Privilege Configuration 1        */
#define STM32_EXTI_RTSR2_OFFSET      0x0020  /* Rising Trigger Selection 2       */
#define STM32_EXTI_FTSR2_OFFSET      0x0024  /* Falling Trigger Selection 2      */
#define STM32_EXTI_SWIER2_OFFSET     0x0028  /* Software Interrupt Event 2       */
#define STM32_EXTI_RPR2_OFFSET       0x002c  /* Rising Edge Pending 2            */
#define STM32_EXTI_FPR2_OFFSET       0x0030  /* Falling Edge Pending 2           */
#define STM32_EXTI_SECCFGR2_OFFSET   0x0034  /* Security Configuration 2         */
#define STM32_EXTI_PRIVCFGR2_OFFSET  0x0038  /* Privilege Configuration 2        */
#define STM32_EXTI_EXTICR1_OFFSET    0x0060  /* External Interrupt Selection 1   */
#define STM32_EXTI_EXTICR2_OFFSET    0x0064  /* External Interrupt Selection 2   */
#define STM32_EXTI_EXTICR3_OFFSET    0x0068  /* External Interrupt Selection 3   */
#define STM32_EXTI_EXTICR4_OFFSET    0x006c  /* External Interrupt Selection 4   */
#define STM32_EXTI_LOCKR_OFFSET      0x0070  /* Lock                             */
#define STM32_EXTI_IMR1_OFFSET       0x0080  /* CPU Wakeup with Interrupt Mask 1 */
#define STM32_EXTI_EMR1_OFFSET       0x0084  /* CPU Wakeup with Event Mask 1     */
#define STM32_EXTI_IMR2_OFFSET       0x0090  /* CPU Wakeup with Interrupt Mask 2 */
#define STM32_EXTI_EMR2_OFFSET       0x0094  /* CPU Wakeup with Event Mask 2     */

/* Register Addresses *******************************************************/

#define STM32_EXTI_RTSR1      (STM32_EXTI_BASE + STM32_EXTI_RTSR1_OFFSET)
#define STM32_EXTI_FTSR1      (STM32_EXTI_BASE + STM32_EXTI_FTSR1_OFFSET)
#define STM32_EXTI_SWIER1     (STM32_EXTI_BASE + STM32_EXTI_SWIER1_OFFSET)
#define STM32_EXTI_RPR1       (STM32_EXTI_BASE + STM32_EXTI_RPR1_OFFSET)
#define STM32_EXTI_FPR1       (STM32_EXTI_BASE + STM32_EXTI_FPR1_OFFSET)
#define STM32_EXTI_SECCFGR1   (STM32_EXTI_BASE + STM32_EXTI_SECCFGR1_OFFSET)
#define STM32_EXTI_PRIVCFGR1  (STM32_EXTI_BASE + STM32_EXTI_PRIVCFGR1_OFFSET)
#define STM32_EXTI_RTSR2      (STM32_EXTI_BASE + STM32_EXTI_RTSR2_OFFSET)
#define STM32_EXTI_FTSR2      (STM32_EXTI_BASE + STM32_EXTI_FTSR2_OFFSET)
#define STM32_EXTI_SWIER2     (STM32_EXTI_BASE + STM32_EXTI_SWIER2_OFFSET)
#define STM32_EXTI_RPR2       (STM32_EXTI_BASE + STM32_EXTI_RPR2_OFFSET)
#define STM32_EXTI_FPR2       (STM32_EXTI_BASE + STM32_EXTI_FPR2_OFFSET)
#define STM32_EXTI_SECCFGR2   (STM32_EXTI_BASE + STM32_EXTI_SECCFGR2_OFFSET)
#define STM32_EXTI_PRIVCFGR2  (STM32_EXTI_BASE + STM32_EXTI_PRIVCFGR2_OFFSET)
#define STM32_EXTI_EXTICR1    (STM32_EXTI_BASE + STM32_EXTI_EXTICR1_OFFSET)
#define STM32_EXTI_EXTICR2    (STM32_EXTI_BASE + STM32_EXTI_EXTICR2_OFFSET)
#define STM32_EXTI_EXTICR3    (STM32_EXTI_BASE + STM32_EXTI_EXTICR3_OFFSET)
#define STM32_EXTI_EXTICR4    (STM32_EXTI_BASE + STM32_EXTI_EXTICR4_OFFSET)
#define STM32_EXTI_LOCKR      (STM32_EXTI_BASE + STM32_EXTI_LOCKR_OFFSET)
#define STM32_EXTI_IMR1       (STM32_EXTI_BASE + STM32_EXTI_IMR1_OFFSET)
#define STM32_EXTI_EMR1       (STM32_EXTI_BASE + STM32_EXTI_EMR1_OFFSET)
#define STM32_EXTI_IMR2       (STM32_EXTI_BASE + STM32_EXTI_IMR2_OFFSET)
#define STM32_EXTI_EMR2       (STM32_EXTI_BASE + STM32_EXTI_EMR2_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Each implemented EXTI line occupies one bit in its register bank.  Lines
 * 0-31 use bank 1 and lines 32-63 use bank 2.  Use only lines implemented in
 * the selected register; trigger/pending and mask registers can differ.
 */

#define STM32_EXTI_BIT(n)          (1u << ((n) & 31))

/* EXTI rising trigger selection register 1 */

#define EXTI_RTSR1_RT0             STM32_EXTI_BIT(0)
#define EXTI_RTSR1_RT1             STM32_EXTI_BIT(1)
#define EXTI_RTSR1_RT2             STM32_EXTI_BIT(2)
#define EXTI_RTSR1_RT3             STM32_EXTI_BIT(3)
#define EXTI_RTSR1_RT4             STM32_EXTI_BIT(4)
#define EXTI_RTSR1_RT5             STM32_EXTI_BIT(5)
#define EXTI_RTSR1_RT6             STM32_EXTI_BIT(6)
#define EXTI_RTSR1_RT7             STM32_EXTI_BIT(7)
#define EXTI_RTSR1_RT8             STM32_EXTI_BIT(8)
#define EXTI_RTSR1_RT9             STM32_EXTI_BIT(9)
#define EXTI_RTSR1_RT10            STM32_EXTI_BIT(10)
#define EXTI_RTSR1_RT11            STM32_EXTI_BIT(11)
#define EXTI_RTSR1_RT12            STM32_EXTI_BIT(12)
#define EXTI_RTSR1_RT13            STM32_EXTI_BIT(13)
#define EXTI_RTSR1_RT14            STM32_EXTI_BIT(14)
#define EXTI_RTSR1_RT15            STM32_EXTI_BIT(15)
#define EXTI_RTSR1_RT16            STM32_EXTI_BIT(16)
#define EXTI_RTSR1_RT17            STM32_EXTI_BIT(17)
#define EXTI_RTSR1_RT18            STM32_EXTI_BIT(18)
#define EXTI_RTSR1_RT19            STM32_EXTI_BIT(19)
#define EXTI_RTSR1_RT20            STM32_EXTI_BIT(20)
#define EXTI_RTSR1_RT21            STM32_EXTI_BIT(21)
#define EXTI_RTSR1_RT22            STM32_EXTI_BIT(22)
#define EXTI_RTSR1_RT23            STM32_EXTI_BIT(23)
#define EXTI_RTSR1_RT24            STM32_EXTI_BIT(24)
#define EXTI_RTSR1_RT25            STM32_EXTI_BIT(25)
#define EXTI_RTSR1_RT26            STM32_EXTI_BIT(26)
#define EXTI_RTSR1_RT27            STM32_EXTI_BIT(27)
#define EXTI_RTSR1_RT28            STM32_EXTI_BIT(28)
#define EXTI_RTSR1_RT29            STM32_EXTI_BIT(29)
#define EXTI_RTSR1_RT30            STM32_EXTI_BIT(30)
#define EXTI_RTSR1_RT31            STM32_EXTI_BIT(31)

/* EXTI rising trigger selection register 2 */

#define EXTI_RTSR2_RT32            STM32_EXTI_BIT(32)
#define EXTI_RTSR2_RT33            STM32_EXTI_BIT(33)
#define EXTI_RTSR2_RT34            STM32_EXTI_BIT(34)
#define EXTI_RTSR2_RT35            STM32_EXTI_BIT(35)
#define EXTI_RTSR2_RT36            STM32_EXTI_BIT(36)
#define EXTI_RTSR2_RT37            STM32_EXTI_BIT(37)
#define EXTI_RTSR2_RT38            STM32_EXTI_BIT(38)
#define EXTI_RTSR2_RT39            STM32_EXTI_BIT(39)
#define EXTI_RTSR2_RT40            STM32_EXTI_BIT(40)
#define EXTI_RTSR2_RT41            STM32_EXTI_BIT(41)
#define EXTI_RTSR2_RT42            STM32_EXTI_BIT(42)
#define EXTI_RTSR2_RT43            STM32_EXTI_BIT(43)
#define EXTI_RTSR2_RT44            STM32_EXTI_BIT(44)
#define EXTI_RTSR2_RT45            STM32_EXTI_BIT(45)
#define EXTI_RTSR2_RT46            STM32_EXTI_BIT(46)
#define EXTI_RTSR2_RT47            STM32_EXTI_BIT(47)
#define EXTI_RTSR2_RT48            STM32_EXTI_BIT(48)
#define EXTI_RTSR2_RT49            STM32_EXTI_BIT(49)
#define EXTI_RTSR2_RT50            STM32_EXTI_BIT(50)
#define EXTI_RTSR2_RT51            STM32_EXTI_BIT(51)
#define EXTI_RTSR2_RT52            STM32_EXTI_BIT(52)
#define EXTI_RTSR2_RT53            STM32_EXTI_BIT(53)
#define EXTI_RTSR2_RT54            STM32_EXTI_BIT(54)
#define EXTI_RTSR2_RT55            STM32_EXTI_BIT(55)
#define EXTI_RTSR2_RT56            STM32_EXTI_BIT(56)
#define EXTI_RTSR2_RT57            STM32_EXTI_BIT(57)
#define EXTI_RTSR2_RT58            STM32_EXTI_BIT(58)
#define EXTI_RTSR2_RT59            STM32_EXTI_BIT(59)
#define EXTI_RTSR2_RT60            STM32_EXTI_BIT(60)
#define EXTI_RTSR2_RT61            STM32_EXTI_BIT(61)
#define EXTI_RTSR2_RT62            STM32_EXTI_BIT(62)
#define EXTI_RTSR2_RT63            STM32_EXTI_BIT(63)

/* EXTI falling trigger selection register 1 */

#define EXTI_FTSR1_FT0             STM32_EXTI_BIT(0)
#define EXTI_FTSR1_FT1             STM32_EXTI_BIT(1)
#define EXTI_FTSR1_FT2             STM32_EXTI_BIT(2)
#define EXTI_FTSR1_FT3             STM32_EXTI_BIT(3)
#define EXTI_FTSR1_FT4             STM32_EXTI_BIT(4)
#define EXTI_FTSR1_FT5             STM32_EXTI_BIT(5)
#define EXTI_FTSR1_FT6             STM32_EXTI_BIT(6)
#define EXTI_FTSR1_FT7             STM32_EXTI_BIT(7)
#define EXTI_FTSR1_FT8             STM32_EXTI_BIT(8)
#define EXTI_FTSR1_FT9             STM32_EXTI_BIT(9)
#define EXTI_FTSR1_FT10            STM32_EXTI_BIT(10)
#define EXTI_FTSR1_FT11            STM32_EXTI_BIT(11)
#define EXTI_FTSR1_FT12            STM32_EXTI_BIT(12)
#define EXTI_FTSR1_FT13            STM32_EXTI_BIT(13)
#define EXTI_FTSR1_FT14            STM32_EXTI_BIT(14)
#define EXTI_FTSR1_FT15            STM32_EXTI_BIT(15)
#define EXTI_FTSR1_FT16            STM32_EXTI_BIT(16)
#define EXTI_FTSR1_FT17            STM32_EXTI_BIT(17)
#define EXTI_FTSR1_FT18            STM32_EXTI_BIT(18)
#define EXTI_FTSR1_FT19            STM32_EXTI_BIT(19)
#define EXTI_FTSR1_FT20            STM32_EXTI_BIT(20)
#define EXTI_FTSR1_FT21            STM32_EXTI_BIT(21)
#define EXTI_FTSR1_FT22            STM32_EXTI_BIT(22)
#define EXTI_FTSR1_FT23            STM32_EXTI_BIT(23)
#define EXTI_FTSR1_FT24            STM32_EXTI_BIT(24)
#define EXTI_FTSR1_FT25            STM32_EXTI_BIT(25)
#define EXTI_FTSR1_FT26            STM32_EXTI_BIT(26)
#define EXTI_FTSR1_FT27            STM32_EXTI_BIT(27)
#define EXTI_FTSR1_FT28            STM32_EXTI_BIT(28)
#define EXTI_FTSR1_FT29            STM32_EXTI_BIT(29)
#define EXTI_FTSR1_FT30            STM32_EXTI_BIT(30)
#define EXTI_FTSR1_FT31            STM32_EXTI_BIT(31)

/* EXTI falling trigger selection register 2 */

#define EXTI_FTSR2_FT32            STM32_EXTI_BIT(32)
#define EXTI_FTSR2_FT33            STM32_EXTI_BIT(33)
#define EXTI_FTSR2_FT34            STM32_EXTI_BIT(34)
#define EXTI_FTSR2_FT35            STM32_EXTI_BIT(35)
#define EXTI_FTSR2_FT36            STM32_EXTI_BIT(36)
#define EXTI_FTSR2_FT37            STM32_EXTI_BIT(37)
#define EXTI_FTSR2_FT38            STM32_EXTI_BIT(38)
#define EXTI_FTSR2_FT39            STM32_EXTI_BIT(39)
#define EXTI_FTSR2_FT40            STM32_EXTI_BIT(40)
#define EXTI_FTSR2_FT41            STM32_EXTI_BIT(41)
#define EXTI_FTSR2_FT42            STM32_EXTI_BIT(42)
#define EXTI_FTSR2_FT43            STM32_EXTI_BIT(43)
#define EXTI_FTSR2_FT44            STM32_EXTI_BIT(44)
#define EXTI_FTSR2_FT45            STM32_EXTI_BIT(45)
#define EXTI_FTSR2_FT46            STM32_EXTI_BIT(46)
#define EXTI_FTSR2_FT47            STM32_EXTI_BIT(47)
#define EXTI_FTSR2_FT48            STM32_EXTI_BIT(48)
#define EXTI_FTSR2_FT49            STM32_EXTI_BIT(49)
#define EXTI_FTSR2_FT50            STM32_EXTI_BIT(50)
#define EXTI_FTSR2_FT51            STM32_EXTI_BIT(51)
#define EXTI_FTSR2_FT52            STM32_EXTI_BIT(52)
#define EXTI_FTSR2_FT53            STM32_EXTI_BIT(53)
#define EXTI_FTSR2_FT54            STM32_EXTI_BIT(54)
#define EXTI_FTSR2_FT55            STM32_EXTI_BIT(55)
#define EXTI_FTSR2_FT56            STM32_EXTI_BIT(56)
#define EXTI_FTSR2_FT57            STM32_EXTI_BIT(57)
#define EXTI_FTSR2_FT58            STM32_EXTI_BIT(58)
#define EXTI_FTSR2_FT59            STM32_EXTI_BIT(59)
#define EXTI_FTSR2_FT60            STM32_EXTI_BIT(60)
#define EXTI_FTSR2_FT61            STM32_EXTI_BIT(61)
#define EXTI_FTSR2_FT62            STM32_EXTI_BIT(62)
#define EXTI_FTSR2_FT63            STM32_EXTI_BIT(63)

/* EXTI software interrupt event register 1 */

#define EXTI_SWIER1_SWI0           STM32_EXTI_BIT(0)
#define EXTI_SWIER1_SWI1           STM32_EXTI_BIT(1)
#define EXTI_SWIER1_SWI2           STM32_EXTI_BIT(2)
#define EXTI_SWIER1_SWI3           STM32_EXTI_BIT(3)
#define EXTI_SWIER1_SWI4           STM32_EXTI_BIT(4)
#define EXTI_SWIER1_SWI5           STM32_EXTI_BIT(5)
#define EXTI_SWIER1_SWI6           STM32_EXTI_BIT(6)
#define EXTI_SWIER1_SWI7           STM32_EXTI_BIT(7)
#define EXTI_SWIER1_SWI8           STM32_EXTI_BIT(8)
#define EXTI_SWIER1_SWI9           STM32_EXTI_BIT(9)
#define EXTI_SWIER1_SWI10          STM32_EXTI_BIT(10)
#define EXTI_SWIER1_SWI11          STM32_EXTI_BIT(11)
#define EXTI_SWIER1_SWI12          STM32_EXTI_BIT(12)
#define EXTI_SWIER1_SWI13          STM32_EXTI_BIT(13)
#define EXTI_SWIER1_SWI14          STM32_EXTI_BIT(14)
#define EXTI_SWIER1_SWI15          STM32_EXTI_BIT(15)
#define EXTI_SWIER1_SWI16          STM32_EXTI_BIT(16)
#define EXTI_SWIER1_SWI17          STM32_EXTI_BIT(17)
#define EXTI_SWIER1_SWI18          STM32_EXTI_BIT(18)
#define EXTI_SWIER1_SWI19          STM32_EXTI_BIT(19)
#define EXTI_SWIER1_SWI20          STM32_EXTI_BIT(20)
#define EXTI_SWIER1_SWI21          STM32_EXTI_BIT(21)
#define EXTI_SWIER1_SWI22          STM32_EXTI_BIT(22)
#define EXTI_SWIER1_SWI23          STM32_EXTI_BIT(23)
#define EXTI_SWIER1_SWI24          STM32_EXTI_BIT(24)
#define EXTI_SWIER1_SWI25          STM32_EXTI_BIT(25)
#define EXTI_SWIER1_SWI26          STM32_EXTI_BIT(26)
#define EXTI_SWIER1_SWI27          STM32_EXTI_BIT(27)
#define EXTI_SWIER1_SWI28          STM32_EXTI_BIT(28)
#define EXTI_SWIER1_SWI29          STM32_EXTI_BIT(29)
#define EXTI_SWIER1_SWI30          STM32_EXTI_BIT(30)
#define EXTI_SWIER1_SWI31          STM32_EXTI_BIT(31)

/* EXTI software interrupt event register 2 */

#define EXTI_SWIER2_SWI32          STM32_EXTI_BIT(32)
#define EXTI_SWIER2_SWI33          STM32_EXTI_BIT(33)
#define EXTI_SWIER2_SWI34          STM32_EXTI_BIT(34)
#define EXTI_SWIER2_SWI35          STM32_EXTI_BIT(35)
#define EXTI_SWIER2_SWI36          STM32_EXTI_BIT(36)
#define EXTI_SWIER2_SWI37          STM32_EXTI_BIT(37)
#define EXTI_SWIER2_SWI38          STM32_EXTI_BIT(38)
#define EXTI_SWIER2_SWI39          STM32_EXTI_BIT(39)
#define EXTI_SWIER2_SWI40          STM32_EXTI_BIT(40)
#define EXTI_SWIER2_SWI41          STM32_EXTI_BIT(41)
#define EXTI_SWIER2_SWI42          STM32_EXTI_BIT(42)
#define EXTI_SWIER2_SWI43          STM32_EXTI_BIT(43)
#define EXTI_SWIER2_SWI44          STM32_EXTI_BIT(44)
#define EXTI_SWIER2_SWI45          STM32_EXTI_BIT(45)
#define EXTI_SWIER2_SWI46          STM32_EXTI_BIT(46)
#define EXTI_SWIER2_SWI47          STM32_EXTI_BIT(47)
#define EXTI_SWIER2_SWI48          STM32_EXTI_BIT(48)
#define EXTI_SWIER2_SWI49          STM32_EXTI_BIT(49)
#define EXTI_SWIER2_SWI50          STM32_EXTI_BIT(50)
#define EXTI_SWIER2_SWI51          STM32_EXTI_BIT(51)
#define EXTI_SWIER2_SWI52          STM32_EXTI_BIT(52)
#define EXTI_SWIER2_SWI53          STM32_EXTI_BIT(53)
#define EXTI_SWIER2_SWI54          STM32_EXTI_BIT(54)
#define EXTI_SWIER2_SWI55          STM32_EXTI_BIT(55)
#define EXTI_SWIER2_SWI56          STM32_EXTI_BIT(56)
#define EXTI_SWIER2_SWI57          STM32_EXTI_BIT(57)
#define EXTI_SWIER2_SWI58          STM32_EXTI_BIT(58)
#define EXTI_SWIER2_SWI59          STM32_EXTI_BIT(59)
#define EXTI_SWIER2_SWI60          STM32_EXTI_BIT(60)
#define EXTI_SWIER2_SWI61          STM32_EXTI_BIT(61)
#define EXTI_SWIER2_SWI62          STM32_EXTI_BIT(62)
#define EXTI_SWIER2_SWI63          STM32_EXTI_BIT(63)

/* EXTI rising edge pending register 1 */

#define EXTI_RPR1_RPIF0            STM32_EXTI_BIT(0)
#define EXTI_RPR1_RPIF1            STM32_EXTI_BIT(1)
#define EXTI_RPR1_RPIF2            STM32_EXTI_BIT(2)
#define EXTI_RPR1_RPIF3            STM32_EXTI_BIT(3)
#define EXTI_RPR1_RPIF4            STM32_EXTI_BIT(4)
#define EXTI_RPR1_RPIF5            STM32_EXTI_BIT(5)
#define EXTI_RPR1_RPIF6            STM32_EXTI_BIT(6)
#define EXTI_RPR1_RPIF7            STM32_EXTI_BIT(7)
#define EXTI_RPR1_RPIF8            STM32_EXTI_BIT(8)
#define EXTI_RPR1_RPIF9            STM32_EXTI_BIT(9)
#define EXTI_RPR1_RPIF10           STM32_EXTI_BIT(10)
#define EXTI_RPR1_RPIF11           STM32_EXTI_BIT(11)
#define EXTI_RPR1_RPIF12           STM32_EXTI_BIT(12)
#define EXTI_RPR1_RPIF13           STM32_EXTI_BIT(13)
#define EXTI_RPR1_RPIF14           STM32_EXTI_BIT(14)
#define EXTI_RPR1_RPIF15           STM32_EXTI_BIT(15)
#define EXTI_RPR1_RPIF16           STM32_EXTI_BIT(16)
#define EXTI_RPR1_RPIF17           STM32_EXTI_BIT(17)
#define EXTI_RPR1_RPIF18           STM32_EXTI_BIT(18)
#define EXTI_RPR1_RPIF19           STM32_EXTI_BIT(19)
#define EXTI_RPR1_RPIF20           STM32_EXTI_BIT(20)
#define EXTI_RPR1_RPIF21           STM32_EXTI_BIT(21)
#define EXTI_RPR1_RPIF22           STM32_EXTI_BIT(22)
#define EXTI_RPR1_RPIF23           STM32_EXTI_BIT(23)
#define EXTI_RPR1_RPIF24           STM32_EXTI_BIT(24)
#define EXTI_RPR1_RPIF25           STM32_EXTI_BIT(25)
#define EXTI_RPR1_RPIF26           STM32_EXTI_BIT(26)
#define EXTI_RPR1_RPIF27           STM32_EXTI_BIT(27)
#define EXTI_RPR1_RPIF28           STM32_EXTI_BIT(28)
#define EXTI_RPR1_RPIF29           STM32_EXTI_BIT(29)
#define EXTI_RPR1_RPIF30           STM32_EXTI_BIT(30)
#define EXTI_RPR1_RPIF31           STM32_EXTI_BIT(31)

/* EXTI rising edge pending register 2 */

#define EXTI_RPR2_RPIF32           STM32_EXTI_BIT(32)
#define EXTI_RPR2_RPIF33           STM32_EXTI_BIT(33)
#define EXTI_RPR2_RPIF34           STM32_EXTI_BIT(34)
#define EXTI_RPR2_RPIF35           STM32_EXTI_BIT(35)
#define EXTI_RPR2_RPIF36           STM32_EXTI_BIT(36)
#define EXTI_RPR2_RPIF37           STM32_EXTI_BIT(37)
#define EXTI_RPR2_RPIF38           STM32_EXTI_BIT(38)
#define EXTI_RPR2_RPIF39           STM32_EXTI_BIT(39)
#define EXTI_RPR2_RPIF40           STM32_EXTI_BIT(40)
#define EXTI_RPR2_RPIF41           STM32_EXTI_BIT(41)
#define EXTI_RPR2_RPIF42           STM32_EXTI_BIT(42)
#define EXTI_RPR2_RPIF43           STM32_EXTI_BIT(43)
#define EXTI_RPR2_RPIF44           STM32_EXTI_BIT(44)
#define EXTI_RPR2_RPIF45           STM32_EXTI_BIT(45)
#define EXTI_RPR2_RPIF46           STM32_EXTI_BIT(46)
#define EXTI_RPR2_RPIF47           STM32_EXTI_BIT(47)
#define EXTI_RPR2_RPIF48           STM32_EXTI_BIT(48)
#define EXTI_RPR2_RPIF49           STM32_EXTI_BIT(49)
#define EXTI_RPR2_RPIF50           STM32_EXTI_BIT(50)
#define EXTI_RPR2_RPIF51           STM32_EXTI_BIT(51)
#define EXTI_RPR2_RPIF52           STM32_EXTI_BIT(52)
#define EXTI_RPR2_RPIF53           STM32_EXTI_BIT(53)
#define EXTI_RPR2_RPIF54           STM32_EXTI_BIT(54)
#define EXTI_RPR2_RPIF55           STM32_EXTI_BIT(55)
#define EXTI_RPR2_RPIF56           STM32_EXTI_BIT(56)
#define EXTI_RPR2_RPIF57           STM32_EXTI_BIT(57)
#define EXTI_RPR2_RPIF58           STM32_EXTI_BIT(58)
#define EXTI_RPR2_RPIF59           STM32_EXTI_BIT(59)
#define EXTI_RPR2_RPIF60           STM32_EXTI_BIT(60)
#define EXTI_RPR2_RPIF61           STM32_EXTI_BIT(61)
#define EXTI_RPR2_RPIF62           STM32_EXTI_BIT(62)
#define EXTI_RPR2_RPIF63           STM32_EXTI_BIT(63)

/* EXTI falling edge pending register 1 */

#define EXTI_FPR1_FPIF0            STM32_EXTI_BIT(0)
#define EXTI_FPR1_FPIF1            STM32_EXTI_BIT(1)
#define EXTI_FPR1_FPIF2            STM32_EXTI_BIT(2)
#define EXTI_FPR1_FPIF3            STM32_EXTI_BIT(3)
#define EXTI_FPR1_FPIF4            STM32_EXTI_BIT(4)
#define EXTI_FPR1_FPIF5            STM32_EXTI_BIT(5)
#define EXTI_FPR1_FPIF6            STM32_EXTI_BIT(6)
#define EXTI_FPR1_FPIF7            STM32_EXTI_BIT(7)
#define EXTI_FPR1_FPIF8            STM32_EXTI_BIT(8)
#define EXTI_FPR1_FPIF9            STM32_EXTI_BIT(9)
#define EXTI_FPR1_FPIF10           STM32_EXTI_BIT(10)
#define EXTI_FPR1_FPIF11           STM32_EXTI_BIT(11)
#define EXTI_FPR1_FPIF12           STM32_EXTI_BIT(12)
#define EXTI_FPR1_FPIF13           STM32_EXTI_BIT(13)
#define EXTI_FPR1_FPIF14           STM32_EXTI_BIT(14)
#define EXTI_FPR1_FPIF15           STM32_EXTI_BIT(15)
#define EXTI_FPR1_FPIF16           STM32_EXTI_BIT(16)
#define EXTI_FPR1_FPIF17           STM32_EXTI_BIT(17)
#define EXTI_FPR1_FPIF18           STM32_EXTI_BIT(18)
#define EXTI_FPR1_FPIF19           STM32_EXTI_BIT(19)
#define EXTI_FPR1_FPIF20           STM32_EXTI_BIT(20)
#define EXTI_FPR1_FPIF21           STM32_EXTI_BIT(21)
#define EXTI_FPR1_FPIF22           STM32_EXTI_BIT(22)
#define EXTI_FPR1_FPIF23           STM32_EXTI_BIT(23)
#define EXTI_FPR1_FPIF24           STM32_EXTI_BIT(24)
#define EXTI_FPR1_FPIF25           STM32_EXTI_BIT(25)
#define EXTI_FPR1_FPIF26           STM32_EXTI_BIT(26)
#define EXTI_FPR1_FPIF27           STM32_EXTI_BIT(27)
#define EXTI_FPR1_FPIF28           STM32_EXTI_BIT(28)
#define EXTI_FPR1_FPIF29           STM32_EXTI_BIT(29)
#define EXTI_FPR1_FPIF30           STM32_EXTI_BIT(30)
#define EXTI_FPR1_FPIF31           STM32_EXTI_BIT(31)

/* EXTI falling edge pending register 2 */

#define EXTI_FPR2_FPIF32           STM32_EXTI_BIT(32)
#define EXTI_FPR2_FPIF33           STM32_EXTI_BIT(33)
#define EXTI_FPR2_FPIF34           STM32_EXTI_BIT(34)
#define EXTI_FPR2_FPIF35           STM32_EXTI_BIT(35)
#define EXTI_FPR2_FPIF36           STM32_EXTI_BIT(36)
#define EXTI_FPR2_FPIF37           STM32_EXTI_BIT(37)
#define EXTI_FPR2_FPIF38           STM32_EXTI_BIT(38)
#define EXTI_FPR2_FPIF39           STM32_EXTI_BIT(39)
#define EXTI_FPR2_FPIF40           STM32_EXTI_BIT(40)
#define EXTI_FPR2_FPIF41           STM32_EXTI_BIT(41)
#define EXTI_FPR2_FPIF42           STM32_EXTI_BIT(42)
#define EXTI_FPR2_FPIF43           STM32_EXTI_BIT(43)
#define EXTI_FPR2_FPIF44           STM32_EXTI_BIT(44)
#define EXTI_FPR2_FPIF45           STM32_EXTI_BIT(45)
#define EXTI_FPR2_FPIF46           STM32_EXTI_BIT(46)
#define EXTI_FPR2_FPIF47           STM32_EXTI_BIT(47)
#define EXTI_FPR2_FPIF48           STM32_EXTI_BIT(48)
#define EXTI_FPR2_FPIF49           STM32_EXTI_BIT(49)
#define EXTI_FPR2_FPIF50           STM32_EXTI_BIT(50)
#define EXTI_FPR2_FPIF51           STM32_EXTI_BIT(51)
#define EXTI_FPR2_FPIF52           STM32_EXTI_BIT(52)
#define EXTI_FPR2_FPIF53           STM32_EXTI_BIT(53)
#define EXTI_FPR2_FPIF54           STM32_EXTI_BIT(54)
#define EXTI_FPR2_FPIF55           STM32_EXTI_BIT(55)
#define EXTI_FPR2_FPIF56           STM32_EXTI_BIT(56)
#define EXTI_FPR2_FPIF57           STM32_EXTI_BIT(57)
#define EXTI_FPR2_FPIF58           STM32_EXTI_BIT(58)
#define EXTI_FPR2_FPIF59           STM32_EXTI_BIT(59)
#define EXTI_FPR2_FPIF60           STM32_EXTI_BIT(60)
#define EXTI_FPR2_FPIF61           STM32_EXTI_BIT(61)
#define EXTI_FPR2_FPIF62           STM32_EXTI_BIT(62)
#define EXTI_FPR2_FPIF63           STM32_EXTI_BIT(63)

/* EXTI security configuration register 1 */

#define EXTI_SECCFGR1_SEC0         STM32_EXTI_BIT(0)
#define EXTI_SECCFGR1_SEC1         STM32_EXTI_BIT(1)
#define EXTI_SECCFGR1_SEC2         STM32_EXTI_BIT(2)
#define EXTI_SECCFGR1_SEC3         STM32_EXTI_BIT(3)
#define EXTI_SECCFGR1_SEC4         STM32_EXTI_BIT(4)
#define EXTI_SECCFGR1_SEC5         STM32_EXTI_BIT(5)
#define EXTI_SECCFGR1_SEC6         STM32_EXTI_BIT(6)
#define EXTI_SECCFGR1_SEC7         STM32_EXTI_BIT(7)
#define EXTI_SECCFGR1_SEC8         STM32_EXTI_BIT(8)
#define EXTI_SECCFGR1_SEC9         STM32_EXTI_BIT(9)
#define EXTI_SECCFGR1_SEC10        STM32_EXTI_BIT(10)
#define EXTI_SECCFGR1_SEC11        STM32_EXTI_BIT(11)
#define EXTI_SECCFGR1_SEC12        STM32_EXTI_BIT(12)
#define EXTI_SECCFGR1_SEC13        STM32_EXTI_BIT(13)
#define EXTI_SECCFGR1_SEC14        STM32_EXTI_BIT(14)
#define EXTI_SECCFGR1_SEC15        STM32_EXTI_BIT(15)
#define EXTI_SECCFGR1_SEC16        STM32_EXTI_BIT(16)
#define EXTI_SECCFGR1_SEC17        STM32_EXTI_BIT(17)
#define EXTI_SECCFGR1_SEC18        STM32_EXTI_BIT(18)
#define EXTI_SECCFGR1_SEC19        STM32_EXTI_BIT(19)
#define EXTI_SECCFGR1_SEC20        STM32_EXTI_BIT(20)
#define EXTI_SECCFGR1_SEC21        STM32_EXTI_BIT(21)
#define EXTI_SECCFGR1_SEC22        STM32_EXTI_BIT(22)
#define EXTI_SECCFGR1_SEC23        STM32_EXTI_BIT(23)
#define EXTI_SECCFGR1_SEC24        STM32_EXTI_BIT(24)
#define EXTI_SECCFGR1_SEC25        STM32_EXTI_BIT(25)
#define EXTI_SECCFGR1_SEC26        STM32_EXTI_BIT(26)
#define EXTI_SECCFGR1_SEC27        STM32_EXTI_BIT(27)
#define EXTI_SECCFGR1_SEC28        STM32_EXTI_BIT(28)
#define EXTI_SECCFGR1_SEC29        STM32_EXTI_BIT(29)
#define EXTI_SECCFGR1_SEC30        STM32_EXTI_BIT(30)
#define EXTI_SECCFGR1_SEC31        STM32_EXTI_BIT(31)

/* EXTI security configuration register 2 */

#define EXTI_SECCFGR2_SEC32        STM32_EXTI_BIT(32)
#define EXTI_SECCFGR2_SEC33        STM32_EXTI_BIT(33)
#define EXTI_SECCFGR2_SEC34        STM32_EXTI_BIT(34)
#define EXTI_SECCFGR2_SEC35        STM32_EXTI_BIT(35)
#define EXTI_SECCFGR2_SEC36        STM32_EXTI_BIT(36)
#define EXTI_SECCFGR2_SEC37        STM32_EXTI_BIT(37)
#define EXTI_SECCFGR2_SEC38        STM32_EXTI_BIT(38)
#define EXTI_SECCFGR2_SEC39        STM32_EXTI_BIT(39)
#define EXTI_SECCFGR2_SEC40        STM32_EXTI_BIT(40)
#define EXTI_SECCFGR2_SEC41        STM32_EXTI_BIT(41)
#define EXTI_SECCFGR2_SEC42        STM32_EXTI_BIT(42)
#define EXTI_SECCFGR2_SEC43        STM32_EXTI_BIT(43)
#define EXTI_SECCFGR2_SEC44        STM32_EXTI_BIT(44)
#define EXTI_SECCFGR2_SEC45        STM32_EXTI_BIT(45)
#define EXTI_SECCFGR2_SEC46        STM32_EXTI_BIT(46)
#define EXTI_SECCFGR2_SEC47        STM32_EXTI_BIT(47)
#define EXTI_SECCFGR2_SEC48        STM32_EXTI_BIT(48)
#define EXTI_SECCFGR2_SEC49        STM32_EXTI_BIT(49)
#define EXTI_SECCFGR2_SEC50        STM32_EXTI_BIT(50)
#define EXTI_SECCFGR2_SEC51        STM32_EXTI_BIT(51)
#define EXTI_SECCFGR2_SEC52        STM32_EXTI_BIT(52)
#define EXTI_SECCFGR2_SEC53        STM32_EXTI_BIT(53)
#define EXTI_SECCFGR2_SEC54        STM32_EXTI_BIT(54)
#define EXTI_SECCFGR2_SEC55        STM32_EXTI_BIT(55)
#define EXTI_SECCFGR2_SEC56        STM32_EXTI_BIT(56)
#define EXTI_SECCFGR2_SEC57        STM32_EXTI_BIT(57)
#define EXTI_SECCFGR2_SEC58        STM32_EXTI_BIT(58)
#define EXTI_SECCFGR2_SEC59        STM32_EXTI_BIT(59)
#define EXTI_SECCFGR2_SEC60        STM32_EXTI_BIT(60)
#define EXTI_SECCFGR2_SEC61        STM32_EXTI_BIT(61)
#define EXTI_SECCFGR2_SEC62        STM32_EXTI_BIT(62)
#define EXTI_SECCFGR2_SEC63        STM32_EXTI_BIT(63)

/* EXTI privilege configuration register 1 */

#define EXTI_PRIVCFGR1_PRIV0       STM32_EXTI_BIT(0)
#define EXTI_PRIVCFGR1_PRIV1       STM32_EXTI_BIT(1)
#define EXTI_PRIVCFGR1_PRIV2       STM32_EXTI_BIT(2)
#define EXTI_PRIVCFGR1_PRIV3       STM32_EXTI_BIT(3)
#define EXTI_PRIVCFGR1_PRIV4       STM32_EXTI_BIT(4)
#define EXTI_PRIVCFGR1_PRIV5       STM32_EXTI_BIT(5)
#define EXTI_PRIVCFGR1_PRIV6       STM32_EXTI_BIT(6)
#define EXTI_PRIVCFGR1_PRIV7       STM32_EXTI_BIT(7)
#define EXTI_PRIVCFGR1_PRIV8       STM32_EXTI_BIT(8)
#define EXTI_PRIVCFGR1_PRIV9       STM32_EXTI_BIT(9)
#define EXTI_PRIVCFGR1_PRIV10      STM32_EXTI_BIT(10)
#define EXTI_PRIVCFGR1_PRIV11      STM32_EXTI_BIT(11)
#define EXTI_PRIVCFGR1_PRIV12      STM32_EXTI_BIT(12)
#define EXTI_PRIVCFGR1_PRIV13      STM32_EXTI_BIT(13)
#define EXTI_PRIVCFGR1_PRIV14      STM32_EXTI_BIT(14)
#define EXTI_PRIVCFGR1_PRIV15      STM32_EXTI_BIT(15)
#define EXTI_PRIVCFGR1_PRIV16      STM32_EXTI_BIT(16)
#define EXTI_PRIVCFGR1_PRIV17      STM32_EXTI_BIT(17)
#define EXTI_PRIVCFGR1_PRIV18      STM32_EXTI_BIT(18)
#define EXTI_PRIVCFGR1_PRIV19      STM32_EXTI_BIT(19)
#define EXTI_PRIVCFGR1_PRIV20      STM32_EXTI_BIT(20)
#define EXTI_PRIVCFGR1_PRIV21      STM32_EXTI_BIT(21)
#define EXTI_PRIVCFGR1_PRIV22      STM32_EXTI_BIT(22)
#define EXTI_PRIVCFGR1_PRIV23      STM32_EXTI_BIT(23)
#define EXTI_PRIVCFGR1_PRIV24      STM32_EXTI_BIT(24)
#define EXTI_PRIVCFGR1_PRIV25      STM32_EXTI_BIT(25)
#define EXTI_PRIVCFGR1_PRIV26      STM32_EXTI_BIT(26)
#define EXTI_PRIVCFGR1_PRIV27      STM32_EXTI_BIT(27)
#define EXTI_PRIVCFGR1_PRIV28      STM32_EXTI_BIT(28)
#define EXTI_PRIVCFGR1_PRIV29      STM32_EXTI_BIT(29)
#define EXTI_PRIVCFGR1_PRIV30      STM32_EXTI_BIT(30)
#define EXTI_PRIVCFGR1_PRIV31      STM32_EXTI_BIT(31)

/* EXTI privilege configuration register 2 */

#define EXTI_PRIVCFGR2_PRIV32      STM32_EXTI_BIT(32)
#define EXTI_PRIVCFGR2_PRIV33      STM32_EXTI_BIT(33)
#define EXTI_PRIVCFGR2_PRIV34      STM32_EXTI_BIT(34)
#define EXTI_PRIVCFGR2_PRIV35      STM32_EXTI_BIT(35)
#define EXTI_PRIVCFGR2_PRIV36      STM32_EXTI_BIT(36)
#define EXTI_PRIVCFGR2_PRIV37      STM32_EXTI_BIT(37)
#define EXTI_PRIVCFGR2_PRIV38      STM32_EXTI_BIT(38)
#define EXTI_PRIVCFGR2_PRIV39      STM32_EXTI_BIT(39)
#define EXTI_PRIVCFGR2_PRIV40      STM32_EXTI_BIT(40)
#define EXTI_PRIVCFGR2_PRIV41      STM32_EXTI_BIT(41)
#define EXTI_PRIVCFGR2_PRIV42      STM32_EXTI_BIT(42)
#define EXTI_PRIVCFGR2_PRIV43      STM32_EXTI_BIT(43)
#define EXTI_PRIVCFGR2_PRIV44      STM32_EXTI_BIT(44)
#define EXTI_PRIVCFGR2_PRIV45      STM32_EXTI_BIT(45)
#define EXTI_PRIVCFGR2_PRIV46      STM32_EXTI_BIT(46)
#define EXTI_PRIVCFGR2_PRIV47      STM32_EXTI_BIT(47)
#define EXTI_PRIVCFGR2_PRIV48      STM32_EXTI_BIT(48)
#define EXTI_PRIVCFGR2_PRIV49      STM32_EXTI_BIT(49)
#define EXTI_PRIVCFGR2_PRIV50      STM32_EXTI_BIT(50)
#define EXTI_PRIVCFGR2_PRIV51      STM32_EXTI_BIT(51)
#define EXTI_PRIVCFGR2_PRIV52      STM32_EXTI_BIT(52)
#define EXTI_PRIVCFGR2_PRIV53      STM32_EXTI_BIT(53)
#define EXTI_PRIVCFGR2_PRIV54      STM32_EXTI_BIT(54)
#define EXTI_PRIVCFGR2_PRIV55      STM32_EXTI_BIT(55)
#define EXTI_PRIVCFGR2_PRIV56      STM32_EXTI_BIT(56)
#define EXTI_PRIVCFGR2_PRIV57      STM32_EXTI_BIT(57)
#define EXTI_PRIVCFGR2_PRIV58      STM32_EXTI_BIT(58)
#define EXTI_PRIVCFGR2_PRIV59      STM32_EXTI_BIT(59)
#define EXTI_PRIVCFGR2_PRIV60      STM32_EXTI_BIT(60)
#define EXTI_PRIVCFGR2_PRIV61      STM32_EXTI_BIT(61)
#define EXTI_PRIVCFGR2_PRIV62      STM32_EXTI_BIT(62)
#define EXTI_PRIVCFGR2_PRIV63      STM32_EXTI_BIT(63)

/* EXTI CPU wake-up with interrupt mask register 1 */

#define EXTI_IMR1_IM0              STM32_EXTI_BIT(0)
#define EXTI_IMR1_IM1              STM32_EXTI_BIT(1)
#define EXTI_IMR1_IM2              STM32_EXTI_BIT(2)
#define EXTI_IMR1_IM3              STM32_EXTI_BIT(3)
#define EXTI_IMR1_IM4              STM32_EXTI_BIT(4)
#define EXTI_IMR1_IM5              STM32_EXTI_BIT(5)
#define EXTI_IMR1_IM6              STM32_EXTI_BIT(6)
#define EXTI_IMR1_IM7              STM32_EXTI_BIT(7)
#define EXTI_IMR1_IM8              STM32_EXTI_BIT(8)
#define EXTI_IMR1_IM9              STM32_EXTI_BIT(9)
#define EXTI_IMR1_IM10             STM32_EXTI_BIT(10)
#define EXTI_IMR1_IM11             STM32_EXTI_BIT(11)
#define EXTI_IMR1_IM12             STM32_EXTI_BIT(12)
#define EXTI_IMR1_IM13             STM32_EXTI_BIT(13)
#define EXTI_IMR1_IM14             STM32_EXTI_BIT(14)
#define EXTI_IMR1_IM15             STM32_EXTI_BIT(15)
#define EXTI_IMR1_IM16             STM32_EXTI_BIT(16)
#define EXTI_IMR1_IM17             STM32_EXTI_BIT(17)
#define EXTI_IMR1_IM18             STM32_EXTI_BIT(18)
#define EXTI_IMR1_IM19             STM32_EXTI_BIT(19)
#define EXTI_IMR1_IM20             STM32_EXTI_BIT(20)
#define EXTI_IMR1_IM21             STM32_EXTI_BIT(21)
#define EXTI_IMR1_IM22             STM32_EXTI_BIT(22)
#define EXTI_IMR1_IM23             STM32_EXTI_BIT(23)
#define EXTI_IMR1_IM24             STM32_EXTI_BIT(24)
#define EXTI_IMR1_IM25             STM32_EXTI_BIT(25)
#define EXTI_IMR1_IM26             STM32_EXTI_BIT(26)
#define EXTI_IMR1_IM27             STM32_EXTI_BIT(27)
#define EXTI_IMR1_IM28             STM32_EXTI_BIT(28)
#define EXTI_IMR1_IM29             STM32_EXTI_BIT(29)
#define EXTI_IMR1_IM30             STM32_EXTI_BIT(30)
#define EXTI_IMR1_IM31             STM32_EXTI_BIT(31)

/* EXTI CPU wake-up with interrupt mask register 2 */

#define EXTI_IMR2_IM32             STM32_EXTI_BIT(32)
#define EXTI_IMR2_IM33             STM32_EXTI_BIT(33)
#define EXTI_IMR2_IM34             STM32_EXTI_BIT(34)
#define EXTI_IMR2_IM35             STM32_EXTI_BIT(35)
#define EXTI_IMR2_IM36             STM32_EXTI_BIT(36)
#define EXTI_IMR2_IM37             STM32_EXTI_BIT(37)
#define EXTI_IMR2_IM38             STM32_EXTI_BIT(38)
#define EXTI_IMR2_IM39             STM32_EXTI_BIT(39)
#define EXTI_IMR2_IM40             STM32_EXTI_BIT(40)
#define EXTI_IMR2_IM41             STM32_EXTI_BIT(41)
#define EXTI_IMR2_IM42             STM32_EXTI_BIT(42)
#define EXTI_IMR2_IM43             STM32_EXTI_BIT(43)
#define EXTI_IMR2_IM44             STM32_EXTI_BIT(44)
#define EXTI_IMR2_IM45             STM32_EXTI_BIT(45)
#define EXTI_IMR2_IM46             STM32_EXTI_BIT(46)
#define EXTI_IMR2_IM47             STM32_EXTI_BIT(47)
#define EXTI_IMR2_IM48             STM32_EXTI_BIT(48)
#define EXTI_IMR2_IM49             STM32_EXTI_BIT(49)
#define EXTI_IMR2_IM50             STM32_EXTI_BIT(50)
#define EXTI_IMR2_IM51             STM32_EXTI_BIT(51)
#define EXTI_IMR2_IM52             STM32_EXTI_BIT(52)
#define EXTI_IMR2_IM53             STM32_EXTI_BIT(53)
#define EXTI_IMR2_IM54             STM32_EXTI_BIT(54)
#define EXTI_IMR2_IM55             STM32_EXTI_BIT(55)
#define EXTI_IMR2_IM56             STM32_EXTI_BIT(56)
#define EXTI_IMR2_IM57             STM32_EXTI_BIT(57)
#define EXTI_IMR2_IM58             STM32_EXTI_BIT(58)
#define EXTI_IMR2_IM59             STM32_EXTI_BIT(59)
#define EXTI_IMR2_IM60             STM32_EXTI_BIT(60)
#define EXTI_IMR2_IM61             STM32_EXTI_BIT(61)
#define EXTI_IMR2_IM62             STM32_EXTI_BIT(62)
#define EXTI_IMR2_IM63             STM32_EXTI_BIT(63)

/* EXTI CPU wake-up with event mask register 1 */

#define EXTI_EMR1_EM0              STM32_EXTI_BIT(0)
#define EXTI_EMR1_EM1              STM32_EXTI_BIT(1)
#define EXTI_EMR1_EM2              STM32_EXTI_BIT(2)
#define EXTI_EMR1_EM3              STM32_EXTI_BIT(3)
#define EXTI_EMR1_EM4              STM32_EXTI_BIT(4)
#define EXTI_EMR1_EM5              STM32_EXTI_BIT(5)
#define EXTI_EMR1_EM6              STM32_EXTI_BIT(6)
#define EXTI_EMR1_EM7              STM32_EXTI_BIT(7)
#define EXTI_EMR1_EM8              STM32_EXTI_BIT(8)
#define EXTI_EMR1_EM9              STM32_EXTI_BIT(9)
#define EXTI_EMR1_EM10             STM32_EXTI_BIT(10)
#define EXTI_EMR1_EM11             STM32_EXTI_BIT(11)
#define EXTI_EMR1_EM12             STM32_EXTI_BIT(12)
#define EXTI_EMR1_EM13             STM32_EXTI_BIT(13)
#define EXTI_EMR1_EM14             STM32_EXTI_BIT(14)
#define EXTI_EMR1_EM15             STM32_EXTI_BIT(15)
#define EXTI_EMR1_EM16             STM32_EXTI_BIT(16)
#define EXTI_EMR1_EM17             STM32_EXTI_BIT(17)
#define EXTI_EMR1_EM18             STM32_EXTI_BIT(18)
#define EXTI_EMR1_EM19             STM32_EXTI_BIT(19)
#define EXTI_EMR1_EM20             STM32_EXTI_BIT(20)
#define EXTI_EMR1_EM21             STM32_EXTI_BIT(21)
#define EXTI_EMR1_EM22             STM32_EXTI_BIT(22)
#define EXTI_EMR1_EM23             STM32_EXTI_BIT(23)
#define EXTI_EMR1_EM24             STM32_EXTI_BIT(24)
#define EXTI_EMR1_EM25             STM32_EXTI_BIT(25)
#define EXTI_EMR1_EM26             STM32_EXTI_BIT(26)
#define EXTI_EMR1_EM27             STM32_EXTI_BIT(27)
#define EXTI_EMR1_EM28             STM32_EXTI_BIT(28)
#define EXTI_EMR1_EM29             STM32_EXTI_BIT(29)
#define EXTI_EMR1_EM30             STM32_EXTI_BIT(30)
#define EXTI_EMR1_EM31             STM32_EXTI_BIT(31)

/* EXTI CPU wake-up with event mask register 2 */

#define EXTI_EMR2_EM32             STM32_EXTI_BIT(32)
#define EXTI_EMR2_EM33             STM32_EXTI_BIT(33)
#define EXTI_EMR2_EM34             STM32_EXTI_BIT(34)
#define EXTI_EMR2_EM35             STM32_EXTI_BIT(35)
#define EXTI_EMR2_EM36             STM32_EXTI_BIT(36)
#define EXTI_EMR2_EM37             STM32_EXTI_BIT(37)
#define EXTI_EMR2_EM38             STM32_EXTI_BIT(38)
#define EXTI_EMR2_EM39             STM32_EXTI_BIT(39)
#define EXTI_EMR2_EM40             STM32_EXTI_BIT(40)
#define EXTI_EMR2_EM41             STM32_EXTI_BIT(41)
#define EXTI_EMR2_EM42             STM32_EXTI_BIT(42)
#define EXTI_EMR2_EM43             STM32_EXTI_BIT(43)
#define EXTI_EMR2_EM44             STM32_EXTI_BIT(44)
#define EXTI_EMR2_EM45             STM32_EXTI_BIT(45)
#define EXTI_EMR2_EM46             STM32_EXTI_BIT(46)
#define EXTI_EMR2_EM47             STM32_EXTI_BIT(47)
#define EXTI_EMR2_EM48             STM32_EXTI_BIT(48)
#define EXTI_EMR2_EM49             STM32_EXTI_BIT(49)
#define EXTI_EMR2_EM50             STM32_EXTI_BIT(50)
#define EXTI_EMR2_EM51             STM32_EXTI_BIT(51)
#define EXTI_EMR2_EM52             STM32_EXTI_BIT(52)
#define EXTI_EMR2_EM53             STM32_EXTI_BIT(53)
#define EXTI_EMR2_EM54             STM32_EXTI_BIT(54)
#define EXTI_EMR2_EM55             STM32_EXTI_BIT(55)
#define EXTI_EMR2_EM56             STM32_EXTI_BIT(56)
#define EXTI_EMR2_EM57             STM32_EXTI_BIT(57)
#define EXTI_EMR2_EM58             STM32_EXTI_BIT(58)
#define EXTI_EMR2_EM59             STM32_EXTI_BIT(59)
#define EXTI_EMR2_EM60             STM32_EXTI_BIT(60)
#define EXTI_EMR2_EM61             STM32_EXTI_BIT(61)
#define EXTI_EMR2_EM62             STM32_EXTI_BIT(62)
#define EXTI_EMR2_EM63             STM32_EXTI_BIT(63)

/* GPIO lines 0-15 have four port selections per EXTICR, eight bits apart.
 * Clear the whole selector byte.  Valid port encodings leave any unused
 * upper bits at their reset value of zero.
 */

#define STM32_EXTI_EXTICR(n)       (STM32_EXTI_EXTICR1 + (((n) >> 2) << 2))
#define EXTI_EXTICR_SHIFT(n)       (((n) & 3) << 3)
#define EXTI_EXTICR_MASK(n)        (0xffu << EXTI_EXTICR_SHIFT(n))
#define EXTI_EXTICR_PORT(n, p)     ((p) << EXTI_EXTICR_SHIFT(n))

/* EXTI external interrupt selection register 1 */

#define EXTI_EXTICR1_EXTI0_SHIFT    EXTI_EXTICR_SHIFT(0)
#define EXTI_EXTICR1_EXTI0_MASK     EXTI_EXTICR_MASK(0)
#define EXTI_EXTICR1_EXTI0(n)       EXTI_EXTICR_PORT(0, n)
#define EXTI_EXTICR1_EXTI1_SHIFT    EXTI_EXTICR_SHIFT(1)
#define EXTI_EXTICR1_EXTI1_MASK     EXTI_EXTICR_MASK(1)
#define EXTI_EXTICR1_EXTI1(n)       EXTI_EXTICR_PORT(1, n)
#define EXTI_EXTICR1_EXTI2_SHIFT    EXTI_EXTICR_SHIFT(2)
#define EXTI_EXTICR1_EXTI2_MASK     EXTI_EXTICR_MASK(2)
#define EXTI_EXTICR1_EXTI2(n)       EXTI_EXTICR_PORT(2, n)
#define EXTI_EXTICR1_EXTI3_SHIFT    EXTI_EXTICR_SHIFT(3)
#define EXTI_EXTICR1_EXTI3_MASK     EXTI_EXTICR_MASK(3)
#define EXTI_EXTICR1_EXTI3(n)       EXTI_EXTICR_PORT(3, n)

/* EXTI external interrupt selection register 2 */

#define EXTI_EXTICR2_EXTI4_SHIFT    EXTI_EXTICR_SHIFT(4)
#define EXTI_EXTICR2_EXTI4_MASK     EXTI_EXTICR_MASK(4)
#define EXTI_EXTICR2_EXTI4(n)       EXTI_EXTICR_PORT(4, n)
#define EXTI_EXTICR2_EXTI5_SHIFT    EXTI_EXTICR_SHIFT(5)
#define EXTI_EXTICR2_EXTI5_MASK     EXTI_EXTICR_MASK(5)
#define EXTI_EXTICR2_EXTI5(n)       EXTI_EXTICR_PORT(5, n)
#define EXTI_EXTICR2_EXTI6_SHIFT    EXTI_EXTICR_SHIFT(6)
#define EXTI_EXTICR2_EXTI6_MASK     EXTI_EXTICR_MASK(6)
#define EXTI_EXTICR2_EXTI6(n)       EXTI_EXTICR_PORT(6, n)
#define EXTI_EXTICR2_EXTI7_SHIFT    EXTI_EXTICR_SHIFT(7)
#define EXTI_EXTICR2_EXTI7_MASK     EXTI_EXTICR_MASK(7)
#define EXTI_EXTICR2_EXTI7(n)       EXTI_EXTICR_PORT(7, n)

/* EXTI external interrupt selection register 3 */

#define EXTI_EXTICR3_EXTI8_SHIFT    EXTI_EXTICR_SHIFT(8)
#define EXTI_EXTICR3_EXTI8_MASK     EXTI_EXTICR_MASK(8)
#define EXTI_EXTICR3_EXTI8(n)       EXTI_EXTICR_PORT(8, n)
#define EXTI_EXTICR3_EXTI9_SHIFT    EXTI_EXTICR_SHIFT(9)
#define EXTI_EXTICR3_EXTI9_MASK     EXTI_EXTICR_MASK(9)
#define EXTI_EXTICR3_EXTI9(n)       EXTI_EXTICR_PORT(9, n)
#define EXTI_EXTICR3_EXTI10_SHIFT   EXTI_EXTICR_SHIFT(10)
#define EXTI_EXTICR3_EXTI10_MASK    EXTI_EXTICR_MASK(10)
#define EXTI_EXTICR3_EXTI10(n)      EXTI_EXTICR_PORT(10, n)
#define EXTI_EXTICR3_EXTI11_SHIFT   EXTI_EXTICR_SHIFT(11)
#define EXTI_EXTICR3_EXTI11_MASK    EXTI_EXTICR_MASK(11)
#define EXTI_EXTICR3_EXTI11(n)      EXTI_EXTICR_PORT(11, n)

/* EXTI external interrupt selection register 4 */

#define EXTI_EXTICR4_EXTI12_SHIFT   EXTI_EXTICR_SHIFT(12)
#define EXTI_EXTICR4_EXTI12_MASK    EXTI_EXTICR_MASK(12)
#define EXTI_EXTICR4_EXTI12(n)      EXTI_EXTICR_PORT(12, n)
#define EXTI_EXTICR4_EXTI13_SHIFT   EXTI_EXTICR_SHIFT(13)
#define EXTI_EXTICR4_EXTI13_MASK    EXTI_EXTICR_MASK(13)
#define EXTI_EXTICR4_EXTI13(n)      EXTI_EXTICR_PORT(13, n)
#define EXTI_EXTICR4_EXTI14_SHIFT   EXTI_EXTICR_SHIFT(14)
#define EXTI_EXTICR4_EXTI14_MASK    EXTI_EXTICR_MASK(14)
#define EXTI_EXTICR4_EXTI14(n)      EXTI_EXTICR_PORT(14, n)
#define EXTI_EXTICR4_EXTI15_SHIFT   EXTI_EXTICR_SHIFT(15)
#define EXTI_EXTICR4_EXTI15_MASK    EXTI_EXTICR_MASK(15)
#define EXTI_EXTICR4_EXTI15(n)      EXTI_EXTICR_PORT(15, n)

/* EXTI lock register */

#define EXTI_LOCKR_LOCK           (1 << 0)

#endif /* __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_EXTI_M33_V1_H */
