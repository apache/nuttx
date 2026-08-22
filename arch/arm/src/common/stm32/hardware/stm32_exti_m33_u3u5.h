/****************************************************************************
 * arch/arm/src/common/stm32/hardware/stm32_exti_m33_u3u5.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_EXTI_M33_U3U5_H
#define __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_EXTI_M33_U3U5_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EXTI Line Inventory ******************************************************/

#if defined(CONFIG_STM32_STM32U3C5XX)
#  define STM32_EXTI_NLINES  23
#elif defined(CONFIG_STM32_STM32U585XX)
#  define STM32_EXTI_NLINES  24
#elif defined(CONFIG_STM32_STM32U5A5XX)
#  define STM32_EXTI_NLINES  26
#else
#  error "Unsupported STM32U3/U5 EXTI line inventory"
#endif

/* Register Offsets *********************************************************/

#define STM32_EXTI_RTSR1_OFFSET      0x0000  /* Rising Trigger Selection 1       */
#define STM32_EXTI_FTSR1_OFFSET      0x0004  /* Falling Trigger Selection 1      */
#define STM32_EXTI_SWIER1_OFFSET     0x0008  /* Software Interrupt Event 1       */
#define STM32_EXTI_RPR1_OFFSET       0x000c  /* Rising Edge Pending 1            */
#define STM32_EXTI_FPR1_OFFSET       0x0010  /* Falling Edge Pending 1           */
#define STM32_EXTI_SECCFGR1_OFFSET   0x0014  /* Security Configuration 1         */
#define STM32_EXTI_PRIVCFGR1_OFFSET  0x0018  /* Privilege Configuration 1        */
#define STM32_EXTI_EXTICR1_OFFSET    0x0060  /* External Interrupt Selection 1   */
#define STM32_EXTI_EXTICR2_OFFSET    0x0064  /* External Interrupt Selection 2   */
#define STM32_EXTI_EXTICR3_OFFSET    0x0068  /* External Interrupt Selection 3   */
#define STM32_EXTI_EXTICR4_OFFSET    0x006c  /* External Interrupt Selection 4   */
#define STM32_EXTI_LOCKR_OFFSET      0x0070  /* Lock                             */
#define STM32_EXTI_IMR1_OFFSET       0x0080  /* CPU Wakeup with Interrupt Mask 1 */
#define STM32_EXTI_EMR1_OFFSET       0x0084  /* CPU Wakeup with Event Mask 1     */

/* Register Addresses *******************************************************/

#define STM32_EXTI_RTSR1      (STM32_EXTI_BASE + STM32_EXTI_RTSR1_OFFSET)
#define STM32_EXTI_FTSR1      (STM32_EXTI_BASE + STM32_EXTI_FTSR1_OFFSET)
#define STM32_EXTI_SWIER1     (STM32_EXTI_BASE + STM32_EXTI_SWIER1_OFFSET)
#define STM32_EXTI_RPR1       (STM32_EXTI_BASE + STM32_EXTI_RPR1_OFFSET)
#define STM32_EXTI_FPR1       (STM32_EXTI_BASE + STM32_EXTI_FPR1_OFFSET)
#define STM32_EXTI_SECCFGR1   (STM32_EXTI_BASE + STM32_EXTI_SECCFGR1_OFFSET)
#define STM32_EXTI_PRIVCFGR1  (STM32_EXTI_BASE + STM32_EXTI_PRIVCFGR1_OFFSET)
#define STM32_EXTI_EXTICR1    (STM32_EXTI_BASE + STM32_EXTI_EXTICR1_OFFSET)
#define STM32_EXTI_EXTICR2    (STM32_EXTI_BASE + STM32_EXTI_EXTICR2_OFFSET)
#define STM32_EXTI_EXTICR3    (STM32_EXTI_BASE + STM32_EXTI_EXTICR3_OFFSET)
#define STM32_EXTI_EXTICR4    (STM32_EXTI_BASE + STM32_EXTI_EXTICR4_OFFSET)
#define STM32_EXTI_LOCKR      (STM32_EXTI_BASE + STM32_EXTI_LOCKR_OFFSET)
#define STM32_EXTI_IMR1       (STM32_EXTI_BASE + STM32_EXTI_IMR1_OFFSET)
#define STM32_EXTI_EMR1       (STM32_EXTI_BASE + STM32_EXTI_EMR1_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* EXTI rising trigger selection register */

#define EXTI_RTSR1_RT0            (1 << 0)
#define EXTI_RTSR1_RT1            (1 << 1)
#define EXTI_RTSR1_RT2            (1 << 2)
#define EXTI_RTSR1_RT3            (1 << 3)
#define EXTI_RTSR1_RT4            (1 << 4)
#define EXTI_RTSR1_RT5            (1 << 5)
#define EXTI_RTSR1_RT6            (1 << 6)
#define EXTI_RTSR1_RT7            (1 << 7)
#define EXTI_RTSR1_RT8            (1 << 8)
#define EXTI_RTSR1_RT9            (1 << 9)
#define EXTI_RTSR1_RT10           (1 << 10)
#define EXTI_RTSR1_RT11           (1 << 11)
#define EXTI_RTSR1_RT12           (1 << 12)
#define EXTI_RTSR1_RT13           (1 << 13)
#define EXTI_RTSR1_RT14           (1 << 14)
#define EXTI_RTSR1_RT15           (1 << 15)
#define EXTI_RTSR1_RT16           (1 << 16)
#define EXTI_RTSR1_RT17           (1 << 17)
#define EXTI_RTSR1_RT18           (1 << 18)
#define EXTI_RTSR1_RT19           (1 << 19)
#define EXTI_RTSR1_RT20           (1 << 20)
#define EXTI_RTSR1_RT21           (1 << 21)
#define EXTI_RTSR1_RT22           (1 << 22)
#if STM32_EXTI_NLINES > 23
#define EXTI_RTSR1_RT23            (1 << 23)
#endif
#if STM32_EXTI_NLINES > 24
#define EXTI_RTSR1_RT24            (1 << 24)
#endif
#if STM32_EXTI_NLINES > 25
#define EXTI_RTSR1_RT25            (1 << 25)
#endif

/* EXTI falling trigger selection register */

#define EXTI_FTSR1_FT0            (1 << 0)
#define EXTI_FTSR1_FT1            (1 << 1)
#define EXTI_FTSR1_FT2            (1 << 2)
#define EXTI_FTSR1_FT3            (1 << 3)
#define EXTI_FTSR1_FT4            (1 << 4)
#define EXTI_FTSR1_FT5            (1 << 5)
#define EXTI_FTSR1_FT6            (1 << 6)
#define EXTI_FTSR1_FT7            (1 << 7)
#define EXTI_FTSR1_FT8            (1 << 8)
#define EXTI_FTSR1_FT9            (1 << 9)
#define EXTI_FTSR1_FT10           (1 << 10)
#define EXTI_FTSR1_FT11           (1 << 11)
#define EXTI_FTSR1_FT12           (1 << 12)
#define EXTI_FTSR1_FT13           (1 << 13)
#define EXTI_FTSR1_FT14           (1 << 14)
#define EXTI_FTSR1_FT15           (1 << 15)
#define EXTI_FTSR1_FT16           (1 << 16)
#define EXTI_FTSR1_FT17           (1 << 17)
#define EXTI_FTSR1_FT18           (1 << 18)
#define EXTI_FTSR1_FT19           (1 << 19)
#define EXTI_FTSR1_FT20           (1 << 20)
#define EXTI_FTSR1_FT21           (1 << 21)
#define EXTI_FTSR1_FT22           (1 << 22)
#if STM32_EXTI_NLINES > 23
#define EXTI_FTSR1_FT23            (1 << 23)
#endif
#if STM32_EXTI_NLINES > 24
#define EXTI_FTSR1_FT24            (1 << 24)
#endif
#if STM32_EXTI_NLINES > 25
#define EXTI_FTSR1_FT25            (1 << 25)
#endif

/* EXTI software interrupt event register */

#define EXTI_SWIER1_SWI0          (1 << 0)
#define EXTI_SWIER1_SWI1          (1 << 1)
#define EXTI_SWIER1_SWI2          (1 << 2)
#define EXTI_SWIER1_SWI3          (1 << 3)
#define EXTI_SWIER1_SWI4          (1 << 4)
#define EXTI_SWIER1_SWI5          (1 << 5)
#define EXTI_SWIER1_SWI6          (1 << 6)
#define EXTI_SWIER1_SWI7          (1 << 7)
#define EXTI_SWIER1_SWI8          (1 << 8)
#define EXTI_SWIER1_SWI9          (1 << 9)
#define EXTI_SWIER1_SWI10         (1 << 10)
#define EXTI_SWIER1_SWI11         (1 << 11)
#define EXTI_SWIER1_SWI12         (1 << 12)
#define EXTI_SWIER1_SWI13         (1 << 13)
#define EXTI_SWIER1_SWI14         (1 << 14)
#define EXTI_SWIER1_SWI15         (1 << 15)
#define EXTI_SWIER1_SWI16         (1 << 16)
#define EXTI_SWIER1_SWI17         (1 << 17)
#define EXTI_SWIER1_SWI18         (1 << 18)
#define EXTI_SWIER1_SWI19         (1 << 19)
#define EXTI_SWIER1_SWI20         (1 << 20)
#define EXTI_SWIER1_SWI21         (1 << 21)
#define EXTI_SWIER1_SWI22         (1 << 22)
#if STM32_EXTI_NLINES > 23
#define EXTI_SWIER1_SWI23          (1 << 23)
#endif
#if STM32_EXTI_NLINES > 24
#define EXTI_SWIER1_SWI24          (1 << 24)
#endif
#if STM32_EXTI_NLINES > 25
#define EXTI_SWIER1_SWI25          (1 << 25)
#endif

/* EXTI rising edge pending register */

#define EXTI_RPR1_RPIF0           (1 << 0)
#define EXTI_RPR1_RPIF1           (1 << 1)
#define EXTI_RPR1_RPIF2           (1 << 2)
#define EXTI_RPR1_RPIF3           (1 << 3)
#define EXTI_RPR1_RPIF4           (1 << 4)
#define EXTI_RPR1_RPIF5           (1 << 5)
#define EXTI_RPR1_RPIF6           (1 << 6)
#define EXTI_RPR1_RPIF7           (1 << 7)
#define EXTI_RPR1_RPIF8           (1 << 8)
#define EXTI_RPR1_RPIF9           (1 << 9)
#define EXTI_RPR1_RPIF10          (1 << 10)
#define EXTI_RPR1_RPIF11          (1 << 11)
#define EXTI_RPR1_RPIF12          (1 << 12)
#define EXTI_RPR1_RPIF13          (1 << 13)
#define EXTI_RPR1_RPIF14          (1 << 14)
#define EXTI_RPR1_RPIF15          (1 << 15)
#define EXTI_RPR1_RPIF16          (1 << 16)
#define EXTI_RPR1_RPIF17          (1 << 17)
#define EXTI_RPR1_RPIF18          (1 << 18)
#define EXTI_RPR1_RPIF19          (1 << 19)
#define EXTI_RPR1_RPIF20          (1 << 20)
#define EXTI_RPR1_RPIF21          (1 << 21)
#define EXTI_RPR1_RPIF22          (1 << 22)
#if STM32_EXTI_NLINES > 23
#define EXTI_RPR1_RPIF23           (1 << 23)
#endif
#if STM32_EXTI_NLINES > 24
#define EXTI_RPR1_RPIF24           (1 << 24)
#endif
#if STM32_EXTI_NLINES > 25
#define EXTI_RPR1_RPIF25           (1 << 25)
#endif

/* EXTI falling edge pending register */

#define EXTI_FPR1_FPIF0           (1 << 0)
#define EXTI_FPR1_FPIF1           (1 << 1)
#define EXTI_FPR1_FPIF2           (1 << 2)
#define EXTI_FPR1_FPIF3           (1 << 3)
#define EXTI_FPR1_FPIF4           (1 << 4)
#define EXTI_FPR1_FPIF5           (1 << 5)
#define EXTI_FPR1_FPIF6           (1 << 6)
#define EXTI_FPR1_FPIF7           (1 << 7)
#define EXTI_FPR1_FPIF8           (1 << 8)
#define EXTI_FPR1_FPIF9           (1 << 9)
#define EXTI_FPR1_FPIF10          (1 << 10)
#define EXTI_FPR1_FPIF11          (1 << 11)
#define EXTI_FPR1_FPIF12          (1 << 12)
#define EXTI_FPR1_FPIF13          (1 << 13)
#define EXTI_FPR1_FPIF14          (1 << 14)
#define EXTI_FPR1_FPIF15          (1 << 15)
#define EXTI_FPR1_FPIF16          (1 << 16)
#define EXTI_FPR1_FPIF17          (1 << 17)
#define EXTI_FPR1_FPIF18          (1 << 18)
#define EXTI_FPR1_FPIF19          (1 << 19)
#define EXTI_FPR1_FPIF20          (1 << 20)
#define EXTI_FPR1_FPIF21          (1 << 21)
#define EXTI_FPR1_FPIF22          (1 << 22)
#if STM32_EXTI_NLINES > 23
#define EXTI_FPR1_FPIF23           (1 << 23)
#endif
#if STM32_EXTI_NLINES > 24
#define EXTI_FPR1_FPIF24           (1 << 24)
#endif
#if STM32_EXTI_NLINES > 25
#define EXTI_FPR1_FPIF25           (1 << 25)
#endif

/* EXTI security configuration register */

#define EXTI_SECCFGR1_SEC0        (1 << 0)
#define EXTI_SECCFGR1_SEC1        (1 << 1)
#define EXTI_SECCFGR1_SEC2        (1 << 2)
#define EXTI_SECCFGR1_SEC3        (1 << 3)
#define EXTI_SECCFGR1_SEC4        (1 << 4)
#define EXTI_SECCFGR1_SEC5        (1 << 5)
#define EXTI_SECCFGR1_SEC6        (1 << 6)
#define EXTI_SECCFGR1_SEC7        (1 << 7)
#define EXTI_SECCFGR1_SEC8        (1 << 8)
#define EXTI_SECCFGR1_SEC9        (1 << 9)
#define EXTI_SECCFGR1_SEC10       (1 << 10)
#define EXTI_SECCFGR1_SEC11       (1 << 11)
#define EXTI_SECCFGR1_SEC12       (1 << 12)
#define EXTI_SECCFGR1_SEC13       (1 << 13)
#define EXTI_SECCFGR1_SEC14       (1 << 14)
#define EXTI_SECCFGR1_SEC15       (1 << 15)
#define EXTI_SECCFGR1_SEC16       (1 << 16)
#define EXTI_SECCFGR1_SEC17       (1 << 17)
#define EXTI_SECCFGR1_SEC18       (1 << 18)
#define EXTI_SECCFGR1_SEC19       (1 << 19)
#define EXTI_SECCFGR1_SEC20       (1 << 20)
#define EXTI_SECCFGR1_SEC21       (1 << 21)
#define EXTI_SECCFGR1_SEC22       (1 << 22)
#if STM32_EXTI_NLINES > 23
#define EXTI_SECCFGR1_SEC23        (1 << 23)
#endif
#if STM32_EXTI_NLINES > 24
#define EXTI_SECCFGR1_SEC24        (1 << 24)
#endif
#if STM32_EXTI_NLINES > 25
#define EXTI_SECCFGR1_SEC25        (1 << 25)
#endif

/* EXTI privilege configuration register */

#define EXTI_PRIVCFGR1_PRIV0      (1 << 0)
#define EXTI_PRIVCFGR1_PRIV1      (1 << 1)
#define EXTI_PRIVCFGR1_PRIV2      (1 << 2)
#define EXTI_PRIVCFGR1_PRIV3      (1 << 3)
#define EXTI_PRIVCFGR1_PRIV4      (1 << 4)
#define EXTI_PRIVCFGR1_PRIV5      (1 << 5)
#define EXTI_PRIVCFGR1_PRIV6      (1 << 6)
#define EXTI_PRIVCFGR1_PRIV7      (1 << 7)
#define EXTI_PRIVCFGR1_PRIV8      (1 << 8)
#define EXTI_PRIVCFGR1_PRIV9      (1 << 9)
#define EXTI_PRIVCFGR1_PRIV10     (1 << 10)
#define EXTI_PRIVCFGR1_PRIV11     (1 << 11)
#define EXTI_PRIVCFGR1_PRIV12     (1 << 12)
#define EXTI_PRIVCFGR1_PRIV13     (1 << 13)
#define EXTI_PRIVCFGR1_PRIV14     (1 << 14)
#define EXTI_PRIVCFGR1_PRIV15     (1 << 15)
#define EXTI_PRIVCFGR1_PRIV16     (1 << 16)
#define EXTI_PRIVCFGR1_PRIV17     (1 << 17)
#define EXTI_PRIVCFGR1_PRIV18     (1 << 18)
#define EXTI_PRIVCFGR1_PRIV19     (1 << 19)
#define EXTI_PRIVCFGR1_PRIV20     (1 << 20)
#define EXTI_PRIVCFGR1_PRIV21     (1 << 21)
#define EXTI_PRIVCFGR1_PRIV22     (1 << 22)
#if STM32_EXTI_NLINES > 23
#define EXTI_PRIVCFGR1_PRIV23      (1 << 23)
#endif
#if STM32_EXTI_NLINES > 24
#define EXTI_PRIVCFGR1_PRIV24      (1 << 24)
#endif
#if STM32_EXTI_NLINES > 25
#define EXTI_PRIVCFGR1_PRIV25      (1 << 25)
#endif

/* EXTI external interrupt selection register */

#define EXTI_EXTICR1_EXTI0_SHIFT  (0)
#define EXTI_EXTICR1_EXTI0_MASK   (0xf << EXTI_EXTICR1_EXTI0_SHIFT)
#define EXTI_EXTICR1_EXTI0(n)     ((n) << EXTI_EXTICR1_EXTI0_SHIFT)
#define EXTI_EXTICR1_EXTI1_SHIFT  (8)
#define EXTI_EXTICR1_EXTI1_MASK   (0xf << EXTI_EXTICR1_EXTI1_SHIFT)
#define EXTI_EXTICR1_EXTI1(n)     ((n) << EXTI_EXTICR1_EXTI1_SHIFT)
#define EXTI_EXTICR1_EXTI2_SHIFT  (16)
#define EXTI_EXTICR1_EXTI2_MASK   (0xf << EXTI_EXTICR1_EXTI2_SHIFT)
#define EXTI_EXTICR1_EXTI2(n)     ((n) << EXTI_EXTICR1_EXTI2_SHIFT)
#define EXTI_EXTICR1_EXTI3_SHIFT  (24)
#define EXTI_EXTICR1_EXTI3_MASK   (0xf << EXTI_EXTICR1_EXTI3_SHIFT)
#define EXTI_EXTICR1_EXTI3(n)     ((n) << EXTI_EXTICR1_EXTI3_SHIFT)

/* EXTI external interrupt selection register */

#define EXTI_EXTICR2_EXTI4_SHIFT  (0)
#define EXTI_EXTICR2_EXTI4_MASK   (0xf << EXTI_EXTICR2_EXTI4_SHIFT)
#define EXTI_EXTICR2_EXTI4(n)     ((n) << EXTI_EXTICR2_EXTI4_SHIFT)
#define EXTI_EXTICR2_EXTI5_SHIFT  (8)
#define EXTI_EXTICR2_EXTI5_MASK   (0xf << EXTI_EXTICR2_EXTI5_SHIFT)
#define EXTI_EXTICR2_EXTI5(n)     ((n) << EXTI_EXTICR2_EXTI5_SHIFT)
#define EXTI_EXTICR2_EXTI6_SHIFT  (16)
#define EXTI_EXTICR2_EXTI6_MASK   (0xf << EXTI_EXTICR2_EXTI6_SHIFT)
#define EXTI_EXTICR2_EXTI6(n)     ((n) << EXTI_EXTICR2_EXTI6_SHIFT)
#define EXTI_EXTICR2_EXTI7_SHIFT  (24)
#define EXTI_EXTICR2_EXTI7_MASK   (0xf << EXTI_EXTICR2_EXTI7_SHIFT)
#define EXTI_EXTICR2_EXTI7(n)     ((n) << EXTI_EXTICR2_EXTI7_SHIFT)

/* EXTI external interrupt selection register */

#define EXTI_EXTICR3_EXTI8_SHIFT  (0)
#define EXTI_EXTICR3_EXTI8_MASK   (0xf << EXTI_EXTICR3_EXTI8_SHIFT)
#define EXTI_EXTICR3_EXTI8(n)     ((n) << EXTI_EXTICR3_EXTI8_SHIFT)
#define EXTI_EXTICR3_EXTI9_SHIFT  (8)
#define EXTI_EXTICR3_EXTI9_MASK   (0xf << EXTI_EXTICR3_EXTI9_SHIFT)
#define EXTI_EXTICR3_EXTI9(n)     ((n) << EXTI_EXTICR3_EXTI9_SHIFT)
#define EXTI_EXTICR3_EXTI10_SHIFT (16)
#define EXTI_EXTICR3_EXTI10_MASK  (0xf << EXTI_EXTICR3_EXTI10_SHIFT)
#define EXTI_EXTICR3_EXTI10(n)    ((n) << EXTI_EXTICR3_EXTI10_SHIFT)
#define EXTI_EXTICR3_EXTI11_SHIFT (24)
#define EXTI_EXTICR3_EXTI11_MASK  (0xf << EXTI_EXTICR3_EXTI11_SHIFT)
#define EXTI_EXTICR3_EXTI11(n)    ((n) << EXTI_EXTICR3_EXTI11_SHIFT)

/* EXTI external interrupt selection register */

#define EXTI_EXTICR4_EXTI12_SHIFT (0)
#define EXTI_EXTICR4_EXTI12_MASK  (0xf << EXTI_EXTICR4_EXTI12_SHIFT)
#define EXTI_EXTICR4_EXTI12(n)    ((n) << EXTI_EXTICR4_EXTI12_SHIFT)
#define EXTI_EXTICR4_EXTI13_SHIFT (8)
#define EXTI_EXTICR4_EXTI13_MASK  (0xf << EXTI_EXTICR4_EXTI13_SHIFT)
#define EXTI_EXTICR4_EXTI13(n)    ((n) << EXTI_EXTICR4_EXTI13_SHIFT)
#define EXTI_EXTICR4_EXTI14_SHIFT (16)
#define EXTI_EXTICR4_EXTI14_MASK  (0xf << EXTI_EXTICR4_EXTI14_SHIFT)
#define EXTI_EXTICR4_EXTI14(n)    ((n) << EXTI_EXTICR4_EXTI14_SHIFT)
#define EXTI_EXTICR4_EXTI15_SHIFT (24)
#define EXTI_EXTICR4_EXTI15_MASK  (0xf << EXTI_EXTICR4_EXTI15_SHIFT)
#define EXTI_EXTICR4_EXTI15(n)    ((n) << EXTI_EXTICR4_EXTI15_SHIFT)

/* EXTI lock register */

#define EXTI_LOCKR_LOCK           (1 << 0)

/* EXTI CPU wake-up with interrupt mask register */

#define EXTI_IMR1_IM0             (1 << 0)
#define EXTI_IMR1_IM1             (1 << 1)
#define EXTI_IMR1_IM2             (1 << 2)
#define EXTI_IMR1_IM3             (1 << 3)
#define EXTI_IMR1_IM4             (1 << 4)
#define EXTI_IMR1_IM5             (1 << 5)
#define EXTI_IMR1_IM6             (1 << 6)
#define EXTI_IMR1_IM7             (1 << 7)
#define EXTI_IMR1_IM8             (1 << 8)
#define EXTI_IMR1_IM9             (1 << 9)
#define EXTI_IMR1_IM10            (1 << 10)
#define EXTI_IMR1_IM11            (1 << 11)
#define EXTI_IMR1_IM12            (1 << 12)
#define EXTI_IMR1_IM13            (1 << 13)
#define EXTI_IMR1_IM14            (1 << 14)
#define EXTI_IMR1_IM15            (1 << 15)
#define EXTI_IMR1_IM16            (1 << 16)
#define EXTI_IMR1_IM17            (1 << 17)
#define EXTI_IMR1_IM18            (1 << 18)
#define EXTI_IMR1_IM19            (1 << 19)
#define EXTI_IMR1_IM20            (1 << 20)
#define EXTI_IMR1_IM21            (1 << 21)
#define EXTI_IMR1_IM22            (1 << 22)
#if STM32_EXTI_NLINES > 23
#define EXTI_IMR1_IM23             (1 << 23)
#endif
#if STM32_EXTI_NLINES > 24
#define EXTI_IMR1_IM24             (1 << 24)
#endif
#if STM32_EXTI_NLINES > 25
#define EXTI_IMR1_IM25             (1 << 25)
#endif

/* EXTI CPU wake-up with event mask register */

#define EXTI_EMR1_EM0             (1 << 0)
#define EXTI_EMR1_EM1             (1 << 1)
#define EXTI_EMR1_EM2             (1 << 2)
#define EXTI_EMR1_EM3             (1 << 3)
#define EXTI_EMR1_EM4             (1 << 4)
#define EXTI_EMR1_EM5             (1 << 5)
#define EXTI_EMR1_EM6             (1 << 6)
#define EXTI_EMR1_EM7             (1 << 7)
#define EXTI_EMR1_EM8             (1 << 8)
#define EXTI_EMR1_EM9             (1 << 9)
#define EXTI_EMR1_EM10            (1 << 10)
#define EXTI_EMR1_EM11            (1 << 11)
#define EXTI_EMR1_EM12            (1 << 12)
#define EXTI_EMR1_EM13            (1 << 13)
#define EXTI_EMR1_EM14            (1 << 14)
#define EXTI_EMR1_EM15            (1 << 15)
#define EXTI_EMR1_EM16            (1 << 16)
#define EXTI_EMR1_EM17            (1 << 17)
#define EXTI_EMR1_EM18            (1 << 18)
#define EXTI_EMR1_EM19            (1 << 19)
#define EXTI_EMR1_EM20            (1 << 20)
#define EXTI_EMR1_EM21            (1 << 21)
#define EXTI_EMR1_EM22            (1 << 22)
#if STM32_EXTI_NLINES > 23
#define EXTI_EMR1_EM23             (1 << 23)
#endif
#if STM32_EXTI_NLINES > 24
#define EXTI_EMR1_EM24             (1 << 24)
#endif
#if STM32_EXTI_NLINES > 25
#define EXTI_EMR1_EM25             (1 << 25)
#endif

/* GPIO port selection occupies four bits in each EXTICR field. */

#define STM32_EXTI_EXTICR_PORT_MASK  0x0f

#endif /* __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_EXTI_M33_U3U5_H */
