/****************************************************************************
 * arch/arm/src/common/stm32/hardware/stm32_gpio_m33_v1.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_GPIO_M33_V1_H
#define __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_GPIO_M33_V1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/chip/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_GPIO_MODER_OFFSET    0x0000 /* GPIO port mode register */
#define STM32_GPIO_OTYPER_OFFSET   0x0004 /* GPIO port output type register */
#define STM32_GPIO_OSPEED_OFFSET   0x0008 /* GPIO port output speed register */
#define STM32_GPIO_PUPDR_OFFSET    0x000c /* GPIO port pull-up/pull-down register */
#define STM32_GPIO_IDR_OFFSET      0x0010 /* GPIO port input data register */
#define STM32_GPIO_ODR_OFFSET      0x0014 /* GPIO port output data register */
#define STM32_GPIO_BSRR_OFFSET     0x0018 /* GPIO port bit set/reset register */
#define STM32_GPIO_LCKR_OFFSET     0x001c /* GPIO port configuration lock register */
#define STM32_GPIO_AFRL_OFFSET     0x0020 /* GPIO alternate function low register */
#define STM32_GPIO_AFRH_OFFSET     0x0024 /* GPIO alternate function high register */
#define STM32_GPIO_BRR_OFFSET      0x0028 /* GPIO port bit reset register */
#define STM32_GPIO_HSLVR_OFFSET    0x002c /* GPIO high-speed low-voltage register */
#define STM32_GPIO_SECCFGR_OFFSET  0x0030 /* GPIO secure configuration register */

/* Register Addresses *******************************************************/

#if STM32_NPORTS > 0
#  define STM32_GPIOA_MODER        (STM32_GPIOA_BASE + STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOA_OTYPER       (STM32_GPIOA_BASE + STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOA_OSPEED       (STM32_GPIOA_BASE + STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOA_PUPDR        (STM32_GPIOA_BASE + STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOA_IDR          (STM32_GPIOA_BASE + STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOA_ODR          (STM32_GPIOA_BASE + STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOA_BSRR         (STM32_GPIOA_BASE + STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOA_LCKR         (STM32_GPIOA_BASE + STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOA_AFRL         (STM32_GPIOA_BASE + STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOA_AFRH         (STM32_GPIOA_BASE + STM32_GPIO_AFRH_OFFSET)
#  define STM32_GPIOA_BRR          (STM32_GPIOA_BASE + STM32_GPIO_BRR_OFFSET)
#  define STM32_GPIOA_HSLVR        (STM32_GPIOA_BASE + STM32_GPIO_HSLVR_OFFSET)
#  define STM32_GPIOA_SECCFGR      (STM32_GPIOA_BASE + STM32_GPIO_SECCFGR_OFFSET)
#endif

#if STM32_NPORTS > 1
#  define STM32_GPIOB_MODER        (STM32_GPIOB_BASE + STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOB_OTYPER       (STM32_GPIOB_BASE + STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOB_OSPEED       (STM32_GPIOB_BASE + STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOB_PUPDR        (STM32_GPIOB_BASE + STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOB_IDR          (STM32_GPIOB_BASE + STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOB_ODR          (STM32_GPIOB_BASE + STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOB_BSRR         (STM32_GPIOB_BASE + STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOB_LCKR         (STM32_GPIOB_BASE + STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOB_AFRL         (STM32_GPIOB_BASE + STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOB_AFRH         (STM32_GPIOB_BASE + STM32_GPIO_AFRH_OFFSET)
#  define STM32_GPIOB_BRR          (STM32_GPIOB_BASE + STM32_GPIO_BRR_OFFSET)
#  define STM32_GPIOB_HSLVR        (STM32_GPIOB_BASE + STM32_GPIO_HSLVR_OFFSET)
#  define STM32_GPIOB_SECCFGR      (STM32_GPIOB_BASE + STM32_GPIO_SECCFGR_OFFSET)
#endif

#if STM32_NPORTS > 2
#  define STM32_GPIOC_MODER        (STM32_GPIOC_BASE + STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOC_OTYPER       (STM32_GPIOC_BASE + STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOC_OSPEED       (STM32_GPIOC_BASE + STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOC_PUPDR        (STM32_GPIOC_BASE + STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOC_IDR          (STM32_GPIOC_BASE + STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOC_ODR          (STM32_GPIOC_BASE + STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOC_BSRR         (STM32_GPIOC_BASE + STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOC_LCKR         (STM32_GPIOC_BASE + STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOC_AFRL         (STM32_GPIOC_BASE + STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOC_AFRH         (STM32_GPIOC_BASE + STM32_GPIO_AFRH_OFFSET)
#  define STM32_GPIOC_BRR          (STM32_GPIOC_BASE + STM32_GPIO_BRR_OFFSET)
#  define STM32_GPIOC_HSLVR        (STM32_GPIOC_BASE + STM32_GPIO_HSLVR_OFFSET)
#  define STM32_GPIOC_SECCFGR      (STM32_GPIOC_BASE + STM32_GPIO_SECCFGR_OFFSET)
#endif

#if STM32_NPORTS > 3
#  define STM32_GPIOD_MODER        (STM32_GPIOD_BASE + STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOD_OTYPER       (STM32_GPIOD_BASE + STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOD_OSPEED       (STM32_GPIOD_BASE + STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOD_PUPDR        (STM32_GPIOD_BASE + STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOD_IDR          (STM32_GPIOD_BASE + STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOD_ODR          (STM32_GPIOD_BASE + STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOD_BSRR         (STM32_GPIOD_BASE + STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOD_LCKR         (STM32_GPIOD_BASE + STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOD_AFRL         (STM32_GPIOD_BASE + STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOD_AFRH         (STM32_GPIOD_BASE + STM32_GPIO_AFRH_OFFSET)
#  define STM32_GPIOD_BRR          (STM32_GPIOD_BASE + STM32_GPIO_BRR_OFFSET)
#  define STM32_GPIOD_HSLVR        (STM32_GPIOD_BASE + STM32_GPIO_HSLVR_OFFSET)
#  define STM32_GPIOD_SECCFGR      (STM32_GPIOD_BASE + STM32_GPIO_SECCFGR_OFFSET)
#endif

#if STM32_NPORTS > 4
#  define STM32_GPIOE_MODER        (STM32_GPIOE_BASE + STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOE_OTYPER       (STM32_GPIOE_BASE + STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOE_OSPEED       (STM32_GPIOE_BASE + STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOE_PUPDR        (STM32_GPIOE_BASE + STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOE_IDR          (STM32_GPIOE_BASE + STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOE_ODR          (STM32_GPIOE_BASE + STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOE_BSRR         (STM32_GPIOE_BASE + STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOE_LCKR         (STM32_GPIOE_BASE + STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOE_AFRL         (STM32_GPIOE_BASE + STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOE_AFRH         (STM32_GPIOE_BASE + STM32_GPIO_AFRH_OFFSET)
#  define STM32_GPIOE_BRR          (STM32_GPIOE_BASE + STM32_GPIO_BRR_OFFSET)
#  define STM32_GPIOE_HSLVR        (STM32_GPIOE_BASE + STM32_GPIO_HSLVR_OFFSET)
#  define STM32_GPIOE_SECCFGR      (STM32_GPIOE_BASE + STM32_GPIO_SECCFGR_OFFSET)
#endif

#if STM32_NPORTS > 5
#  define STM32_GPIOF_MODER        (STM32_GPIOF_BASE + STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOF_OTYPER       (STM32_GPIOF_BASE + STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOF_OSPEED       (STM32_GPIOF_BASE + STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOF_PUPDR        (STM32_GPIOF_BASE + STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOF_IDR          (STM32_GPIOF_BASE + STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOF_ODR          (STM32_GPIOF_BASE + STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOF_BSRR         (STM32_GPIOF_BASE + STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOF_LCKR         (STM32_GPIOF_BASE + STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOF_AFRL         (STM32_GPIOF_BASE + STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOF_AFRH         (STM32_GPIOF_BASE + STM32_GPIO_AFRH_OFFSET)
#  define STM32_GPIOF_BRR          (STM32_GPIOF_BASE + STM32_GPIO_BRR_OFFSET)
#  define STM32_GPIOF_HSLVR        (STM32_GPIOF_BASE + STM32_GPIO_HSLVR_OFFSET)
#  define STM32_GPIOF_SECCFGR      (STM32_GPIOF_BASE + STM32_GPIO_SECCFGR_OFFSET)
#endif

#if STM32_NPORTS > 6
#  define STM32_GPIOG_MODER        (STM32_GPIOG_BASE + STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOG_OTYPER       (STM32_GPIOG_BASE + STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOG_OSPEED       (STM32_GPIOG_BASE + STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOG_PUPDR        (STM32_GPIOG_BASE + STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOG_IDR          (STM32_GPIOG_BASE + STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOG_ODR          (STM32_GPIOG_BASE + STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOG_BSRR         (STM32_GPIOG_BASE + STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOG_LCKR         (STM32_GPIOG_BASE + STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOG_AFRL         (STM32_GPIOG_BASE + STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOG_AFRH         (STM32_GPIOG_BASE + STM32_GPIO_AFRH_OFFSET)
#  define STM32_GPIOG_BRR          (STM32_GPIOG_BASE + STM32_GPIO_BRR_OFFSET)
#  define STM32_GPIOG_HSLVR        (STM32_GPIOG_BASE + STM32_GPIO_HSLVR_OFFSET)
#  define STM32_GPIOG_SECCFGR      (STM32_GPIOG_BASE + STM32_GPIO_SECCFGR_OFFSET)
#endif

#if STM32_NPORTS > 7
#  define STM32_GPIOH_MODER        (STM32_GPIOH_BASE + STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOH_OTYPER       (STM32_GPIOH_BASE + STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOH_OSPEED       (STM32_GPIOH_BASE + STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOH_PUPDR        (STM32_GPIOH_BASE + STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOH_IDR          (STM32_GPIOH_BASE + STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOH_ODR          (STM32_GPIOH_BASE + STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOH_BSRR         (STM32_GPIOH_BASE + STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOH_LCKR         (STM32_GPIOH_BASE + STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOH_AFRL         (STM32_GPIOH_BASE + STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOH_AFRH         (STM32_GPIOH_BASE + STM32_GPIO_AFRH_OFFSET)
#  define STM32_GPIOH_BRR          (STM32_GPIOH_BASE + STM32_GPIO_BRR_OFFSET)
#  define STM32_GPIOH_HSLVR        (STM32_GPIOH_BASE + STM32_GPIO_HSLVR_OFFSET)
#  define STM32_GPIOH_SECCFGR      (STM32_GPIOH_BASE + STM32_GPIO_SECCFGR_OFFSET)
#endif

#if STM32_NPORTS > 8
#  define STM32_GPIOI_MODER        (STM32_GPIOI_BASE + STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOI_OTYPER       (STM32_GPIOI_BASE + STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOI_OSPEED       (STM32_GPIOI_BASE + STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOI_PUPDR        (STM32_GPIOI_BASE + STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOI_IDR          (STM32_GPIOI_BASE + STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOI_ODR          (STM32_GPIOI_BASE + STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOI_BSRR         (STM32_GPIOI_BASE + STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOI_LCKR         (STM32_GPIOI_BASE + STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOI_AFRL         (STM32_GPIOI_BASE + STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOI_AFRH         (STM32_GPIOI_BASE + STM32_GPIO_AFRH_OFFSET)
#  define STM32_GPIOI_BRR          (STM32_GPIOI_BASE + STM32_GPIO_BRR_OFFSET)
#  define STM32_GPIOI_HSLVR        (STM32_GPIOI_BASE + STM32_GPIO_HSLVR_OFFSET)
#  define STM32_GPIOI_SECCFGR      (STM32_GPIOI_BASE + STM32_GPIO_SECCFGR_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* GPIO port mode register */

#define GPIO_MODER_MODE0_SHIFT      (0)
#define GPIO_MODER_MODE0_MASK       (0x3 << GPIO_MODER_MODE0_SHIFT)
#define GPIO_MODER_MODE0(n)         ((n) << GPIO_MODER_MODE0_SHIFT)
#define GPIO_MODER_MODE1_SHIFT      (2)
#define GPIO_MODER_MODE1_MASK       (0x3 << GPIO_MODER_MODE1_SHIFT)
#define GPIO_MODER_MODE1(n)         ((n) << GPIO_MODER_MODE1_SHIFT)
#define GPIO_MODER_MODE2_SHIFT      (4)
#define GPIO_MODER_MODE2_MASK       (0x3 << GPIO_MODER_MODE2_SHIFT)
#define GPIO_MODER_MODE2(n)         ((n) << GPIO_MODER_MODE2_SHIFT)
#define GPIO_MODER_MODE3_SHIFT      (6)
#define GPIO_MODER_MODE3_MASK       (0x3 << GPIO_MODER_MODE3_SHIFT)
#define GPIO_MODER_MODE3(n)         ((n) << GPIO_MODER_MODE3_SHIFT)
#define GPIO_MODER_MODE4_SHIFT      (8)
#define GPIO_MODER_MODE4_MASK       (0x3 << GPIO_MODER_MODE4_SHIFT)
#define GPIO_MODER_MODE4(n)         ((n) << GPIO_MODER_MODE4_SHIFT)
#define GPIO_MODER_MODE5_SHIFT      (10)
#define GPIO_MODER_MODE5_MASK       (0x3 << GPIO_MODER_MODE5_SHIFT)
#define GPIO_MODER_MODE5(n)         ((n) << GPIO_MODER_MODE5_SHIFT)
#define GPIO_MODER_MODE6_SHIFT      (12)
#define GPIO_MODER_MODE6_MASK       (0x3 << GPIO_MODER_MODE6_SHIFT)
#define GPIO_MODER_MODE6(n)         ((n) << GPIO_MODER_MODE6_SHIFT)
#define GPIO_MODER_MODE7_SHIFT      (14)
#define GPIO_MODER_MODE7_MASK       (0x3 << GPIO_MODER_MODE7_SHIFT)
#define GPIO_MODER_MODE7(n)         ((n) << GPIO_MODER_MODE7_SHIFT)
#define GPIO_MODER_MODE8_SHIFT      (16)
#define GPIO_MODER_MODE8_MASK       (0x3 << GPIO_MODER_MODE8_SHIFT)
#define GPIO_MODER_MODE8(n)         ((n) << GPIO_MODER_MODE8_SHIFT)
#define GPIO_MODER_MODE9_SHIFT      (18)
#define GPIO_MODER_MODE9_MASK       (0x3 << GPIO_MODER_MODE9_SHIFT)
#define GPIO_MODER_MODE9(n)         ((n) << GPIO_MODER_MODE9_SHIFT)
#define GPIO_MODER_MODE10_SHIFT     (20)
#define GPIO_MODER_MODE10_MASK      (0x3 << GPIO_MODER_MODE10_SHIFT)
#define GPIO_MODER_MODE10(n)        ((n) << GPIO_MODER_MODE10_SHIFT)
#define GPIO_MODER_MODE11_SHIFT     (22)
#define GPIO_MODER_MODE11_MASK      (0x3 << GPIO_MODER_MODE11_SHIFT)
#define GPIO_MODER_MODE11(n)        ((n) << GPIO_MODER_MODE11_SHIFT)
#define GPIO_MODER_MODE12_SHIFT     (24)
#define GPIO_MODER_MODE12_MASK      (0x3 << GPIO_MODER_MODE12_SHIFT)
#define GPIO_MODER_MODE12(n)        ((n) << GPIO_MODER_MODE12_SHIFT)
#define GPIO_MODER_MODE13_SHIFT     (26)
#define GPIO_MODER_MODE13_MASK      (0x3 << GPIO_MODER_MODE13_SHIFT)
#define GPIO_MODER_MODE13(n)        ((n) << GPIO_MODER_MODE13_SHIFT)
#define GPIO_MODER_MODE14_SHIFT     (28)
#define GPIO_MODER_MODE14_MASK      (0x3 << GPIO_MODER_MODE14_SHIFT)
#define GPIO_MODER_MODE14(n)        ((n) << GPIO_MODER_MODE14_SHIFT)
#define GPIO_MODER_MODE15_SHIFT     (30)
#define GPIO_MODER_MODE15_MASK      (0x3 << GPIO_MODER_MODE15_SHIFT)
#define GPIO_MODER_MODE15(n)        ((n) << GPIO_MODER_MODE15_SHIFT)

/* GPIO port output type register */

#define GPIO_OTYPER_OT0             (1 << 0)
#define GPIO_OTYPER_OT1             (1 << 1)
#define GPIO_OTYPER_OT2             (1 << 2)
#define GPIO_OTYPER_OT3             (1 << 3)
#define GPIO_OTYPER_OT4             (1 << 4)
#define GPIO_OTYPER_OT5             (1 << 5)
#define GPIO_OTYPER_OT6             (1 << 6)
#define GPIO_OTYPER_OT7             (1 << 7)
#define GPIO_OTYPER_OT8             (1 << 8)
#define GPIO_OTYPER_OT9             (1 << 9)
#define GPIO_OTYPER_OT10            (1 << 10)
#define GPIO_OTYPER_OT11            (1 << 11)
#define GPIO_OTYPER_OT12            (1 << 12)
#define GPIO_OTYPER_OT13            (1 << 13)
#define GPIO_OTYPER_OT14            (1 << 14)
#define GPIO_OTYPER_OT15            (1 << 15)

/* GPIO port output speed register */

#define GPIO_OSPEEDR_OSPEED0_SHIFT  (0)
#define GPIO_OSPEEDR_OSPEED0_MASK   (0x3 << GPIO_OSPEEDR_OSPEED0_SHIFT)
#define GPIO_OSPEEDR_OSPEED0(n)     ((n) << GPIO_OSPEEDR_OSPEED0_SHIFT)
#define GPIO_OSPEEDR_OSPEED1_SHIFT  (2)
#define GPIO_OSPEEDR_OSPEED1_MASK   (0x3 << GPIO_OSPEEDR_OSPEED1_SHIFT)
#define GPIO_OSPEEDR_OSPEED1(n)     ((n) << GPIO_OSPEEDR_OSPEED1_SHIFT)
#define GPIO_OSPEEDR_OSPEED2_SHIFT  (4)
#define GPIO_OSPEEDR_OSPEED2_MASK   (0x3 << GPIO_OSPEEDR_OSPEED2_SHIFT)
#define GPIO_OSPEEDR_OSPEED2(n)     ((n) << GPIO_OSPEEDR_OSPEED2_SHIFT)
#define GPIO_OSPEEDR_OSPEED3_SHIFT  (6)
#define GPIO_OSPEEDR_OSPEED3_MASK   (0x3 << GPIO_OSPEEDR_OSPEED3_SHIFT)
#define GPIO_OSPEEDR_OSPEED3(n)     ((n) << GPIO_OSPEEDR_OSPEED3_SHIFT)
#define GPIO_OSPEEDR_OSPEED4_SHIFT  (8)
#define GPIO_OSPEEDR_OSPEED4_MASK   (0x3 << GPIO_OSPEEDR_OSPEED4_SHIFT)
#define GPIO_OSPEEDR_OSPEED4(n)     ((n) << GPIO_OSPEEDR_OSPEED4_SHIFT)
#define GPIO_OSPEEDR_OSPEED5_SHIFT  (10)
#define GPIO_OSPEEDR_OSPEED5_MASK   (0x3 << GPIO_OSPEEDR_OSPEED5_SHIFT)
#define GPIO_OSPEEDR_OSPEED5(n)     ((n) << GPIO_OSPEEDR_OSPEED5_SHIFT)
#define GPIO_OSPEEDR_OSPEED6_SHIFT  (12)
#define GPIO_OSPEEDR_OSPEED6_MASK   (0x3 << GPIO_OSPEEDR_OSPEED6_SHIFT)
#define GPIO_OSPEEDR_OSPEED6(n)     ((n) << GPIO_OSPEEDR_OSPEED6_SHIFT)
#define GPIO_OSPEEDR_OSPEED7_SHIFT  (14)
#define GPIO_OSPEEDR_OSPEED7_MASK   (0x3 << GPIO_OSPEEDR_OSPEED7_SHIFT)
#define GPIO_OSPEEDR_OSPEED7(n)     ((n) << GPIO_OSPEEDR_OSPEED7_SHIFT)
#define GPIO_OSPEEDR_OSPEED8_SHIFT  (16)
#define GPIO_OSPEEDR_OSPEED8_MASK   (0x3 << GPIO_OSPEEDR_OSPEED8_SHIFT)
#define GPIO_OSPEEDR_OSPEED8(n)     ((n) << GPIO_OSPEEDR_OSPEED8_SHIFT)
#define GPIO_OSPEEDR_OSPEED9_SHIFT  (18)
#define GPIO_OSPEEDR_OSPEED9_MASK   (0x3 << GPIO_OSPEEDR_OSPEED9_SHIFT)
#define GPIO_OSPEEDR_OSPEED9(n)     ((n) << GPIO_OSPEEDR_OSPEED9_SHIFT)
#define GPIO_OSPEEDR_OSPEED10_SHIFT (20)
#define GPIO_OSPEEDR_OSPEED10_MASK  (0x3 << GPIO_OSPEEDR_OSPEED10_SHIFT)
#define GPIO_OSPEEDR_OSPEED10(n)    ((n) << GPIO_OSPEEDR_OSPEED10_SHIFT)
#define GPIO_OSPEEDR_OSPEED11_SHIFT (22)
#define GPIO_OSPEEDR_OSPEED11_MASK  (0x3 << GPIO_OSPEEDR_OSPEED11_SHIFT)
#define GPIO_OSPEEDR_OSPEED11(n)    ((n) << GPIO_OSPEEDR_OSPEED11_SHIFT)
#define GPIO_OSPEEDR_OSPEED12_SHIFT (24)
#define GPIO_OSPEEDR_OSPEED12_MASK  (0x3 << GPIO_OSPEEDR_OSPEED12_SHIFT)
#define GPIO_OSPEEDR_OSPEED12(n)    ((n) << GPIO_OSPEEDR_OSPEED12_SHIFT)
#define GPIO_OSPEEDR_OSPEED13_SHIFT (26)
#define GPIO_OSPEEDR_OSPEED13_MASK  (0x3 << GPIO_OSPEEDR_OSPEED13_SHIFT)
#define GPIO_OSPEEDR_OSPEED13(n)    ((n) << GPIO_OSPEEDR_OSPEED13_SHIFT)
#define GPIO_OSPEEDR_OSPEED14_SHIFT (28)
#define GPIO_OSPEEDR_OSPEED14_MASK  (0x3 << GPIO_OSPEEDR_OSPEED14_SHIFT)
#define GPIO_OSPEEDR_OSPEED14(n)    ((n) << GPIO_OSPEEDR_OSPEED14_SHIFT)
#define GPIO_OSPEEDR_OSPEED15_SHIFT (30)
#define GPIO_OSPEEDR_OSPEED15_MASK  (0x3 << GPIO_OSPEEDR_OSPEED15_SHIFT)
#define GPIO_OSPEEDR_OSPEED15(n)    ((n) << GPIO_OSPEEDR_OSPEED15_SHIFT)

/* GPIO port pull-up/pull-down register */

#define GPIO_PUPDR_PUPD0_SHIFT      (0)
#define GPIO_PUPDR_PUPD0_MASK       (0x3 << GPIO_PUPDR_PUPD0_SHIFT)
#define GPIO_PUPDR_PUPD0(n)         ((n) << GPIO_PUPDR_PUPD0_SHIFT)
#define GPIO_PUPDR_PUPD1_SHIFT      (2)
#define GPIO_PUPDR_PUPD1_MASK       (0x3 << GPIO_PUPDR_PUPD1_SHIFT)
#define GPIO_PUPDR_PUPD1(n)         ((n) << GPIO_PUPDR_PUPD1_SHIFT)
#define GPIO_PUPDR_PUPD2_SHIFT      (4)
#define GPIO_PUPDR_PUPD2_MASK       (0x3 << GPIO_PUPDR_PUPD2_SHIFT)
#define GPIO_PUPDR_PUPD2(n)         ((n) << GPIO_PUPDR_PUPD2_SHIFT)
#define GPIO_PUPDR_PUPD3_SHIFT      (6)
#define GPIO_PUPDR_PUPD3_MASK       (0x3 << GPIO_PUPDR_PUPD3_SHIFT)
#define GPIO_PUPDR_PUPD3(n)         ((n) << GPIO_PUPDR_PUPD3_SHIFT)
#define GPIO_PUPDR_PUPD4_SHIFT      (8)
#define GPIO_PUPDR_PUPD4_MASK       (0x3 << GPIO_PUPDR_PUPD4_SHIFT)
#define GPIO_PUPDR_PUPD4(n)         ((n) << GPIO_PUPDR_PUPD4_SHIFT)
#define GPIO_PUPDR_PUPD5_SHIFT      (10)
#define GPIO_PUPDR_PUPD5_MASK       (0x3 << GPIO_PUPDR_PUPD5_SHIFT)
#define GPIO_PUPDR_PUPD5(n)         ((n) << GPIO_PUPDR_PUPD5_SHIFT)
#define GPIO_PUPDR_PUPD6_SHIFT      (12)
#define GPIO_PUPDR_PUPD6_MASK       (0x3 << GPIO_PUPDR_PUPD6_SHIFT)
#define GPIO_PUPDR_PUPD6(n)         ((n) << GPIO_PUPDR_PUPD6_SHIFT)
#define GPIO_PUPDR_PUPD7_SHIFT      (14)
#define GPIO_PUPDR_PUPD7_MASK       (0x3 << GPIO_PUPDR_PUPD7_SHIFT)
#define GPIO_PUPDR_PUPD7(n)         ((n) << GPIO_PUPDR_PUPD7_SHIFT)
#define GPIO_PUPDR_PUPD8_SHIFT      (16)
#define GPIO_PUPDR_PUPD8_MASK       (0x3 << GPIO_PUPDR_PUPD8_SHIFT)
#define GPIO_PUPDR_PUPD8(n)         ((n) << GPIO_PUPDR_PUPD8_SHIFT)
#define GPIO_PUPDR_PUPD9_SHIFT      (18)
#define GPIO_PUPDR_PUPD9_MASK       (0x3 << GPIO_PUPDR_PUPD9_SHIFT)
#define GPIO_PUPDR_PUPD9(n)         ((n) << GPIO_PUPDR_PUPD9_SHIFT)
#define GPIO_PUPDR_PUPD10_SHIFT     (20)
#define GPIO_PUPDR_PUPD10_MASK      (0x3 << GPIO_PUPDR_PUPD10_SHIFT)
#define GPIO_PUPDR_PUPD10(n)        ((n) << GPIO_PUPDR_PUPD10_SHIFT)
#define GPIO_PUPDR_PUPD11_SHIFT     (22)
#define GPIO_PUPDR_PUPD11_MASK      (0x3 << GPIO_PUPDR_PUPD11_SHIFT)
#define GPIO_PUPDR_PUPD11(n)        ((n) << GPIO_PUPDR_PUPD11_SHIFT)
#define GPIO_PUPDR_PUPD12_SHIFT     (24)
#define GPIO_PUPDR_PUPD12_MASK      (0x3 << GPIO_PUPDR_PUPD12_SHIFT)
#define GPIO_PUPDR_PUPD12(n)        ((n) << GPIO_PUPDR_PUPD12_SHIFT)
#define GPIO_PUPDR_PUPD13_SHIFT     (26)
#define GPIO_PUPDR_PUPD13_MASK      (0x3 << GPIO_PUPDR_PUPD13_SHIFT)
#define GPIO_PUPDR_PUPD13(n)        ((n) << GPIO_PUPDR_PUPD13_SHIFT)
#define GPIO_PUPDR_PUPD14_SHIFT     (28)
#define GPIO_PUPDR_PUPD14_MASK      (0x3 << GPIO_PUPDR_PUPD14_SHIFT)
#define GPIO_PUPDR_PUPD14(n)        ((n) << GPIO_PUPDR_PUPD14_SHIFT)
#define GPIO_PUPDR_PUPD15_SHIFT     (30)
#define GPIO_PUPDR_PUPD15_MASK      (0x3 << GPIO_PUPDR_PUPD15_SHIFT)
#define GPIO_PUPDR_PUPD15(n)        ((n) << GPIO_PUPDR_PUPD15_SHIFT)

/* GPIO port input data register */

#define GPIO_IDR_ID0                (1 << 0)
#define GPIO_IDR_ID1                (1 << 1)
#define GPIO_IDR_ID2                (1 << 2)
#define GPIO_IDR_ID3                (1 << 3)
#define GPIO_IDR_ID4                (1 << 4)
#define GPIO_IDR_ID5                (1 << 5)
#define GPIO_IDR_ID6                (1 << 6)
#define GPIO_IDR_ID7                (1 << 7)
#define GPIO_IDR_ID8                (1 << 8)
#define GPIO_IDR_ID9                (1 << 9)
#define GPIO_IDR_ID10               (1 << 10)
#define GPIO_IDR_ID11               (1 << 11)
#define GPIO_IDR_ID12               (1 << 12)
#define GPIO_IDR_ID13               (1 << 13)
#define GPIO_IDR_ID14               (1 << 14)
#define GPIO_IDR_ID15               (1 << 15)

/* GPIO port output data register */

#define GPIO_ODR_OD0                (1 << 0)
#define GPIO_ODR_OD1                (1 << 1)
#define GPIO_ODR_OD2                (1 << 2)
#define GPIO_ODR_OD3                (1 << 3)
#define GPIO_ODR_OD4                (1 << 4)
#define GPIO_ODR_OD5                (1 << 5)
#define GPIO_ODR_OD6                (1 << 6)
#define GPIO_ODR_OD7                (1 << 7)
#define GPIO_ODR_OD8                (1 << 8)
#define GPIO_ODR_OD9                (1 << 9)
#define GPIO_ODR_OD10               (1 << 10)
#define GPIO_ODR_OD11               (1 << 11)
#define GPIO_ODR_OD12               (1 << 12)
#define GPIO_ODR_OD13               (1 << 13)
#define GPIO_ODR_OD14               (1 << 14)
#define GPIO_ODR_OD15               (1 << 15)

/* GPIO port bit set/reset register */

#define GPIO_BSRR_BS0               (1 << 0)
#define GPIO_BSRR_BS1               (1 << 1)
#define GPIO_BSRR_BS2               (1 << 2)
#define GPIO_BSRR_BS3               (1 << 3)
#define GPIO_BSRR_BS4               (1 << 4)
#define GPIO_BSRR_BS5               (1 << 5)
#define GPIO_BSRR_BS6               (1 << 6)
#define GPIO_BSRR_BS7               (1 << 7)
#define GPIO_BSRR_BS8               (1 << 8)
#define GPIO_BSRR_BS9               (1 << 9)
#define GPIO_BSRR_BS10              (1 << 10)
#define GPIO_BSRR_BS11              (1 << 11)
#define GPIO_BSRR_BS12              (1 << 12)
#define GPIO_BSRR_BS13              (1 << 13)
#define GPIO_BSRR_BS14              (1 << 14)
#define GPIO_BSRR_BS15              (1 << 15)
#define GPIO_BSRR_BR0               (1 << 16)
#define GPIO_BSRR_BR1               (1 << 17)
#define GPIO_BSRR_BR2               (1 << 18)
#define GPIO_BSRR_BR3               (1 << 19)
#define GPIO_BSRR_BR4               (1 << 20)
#define GPIO_BSRR_BR5               (1 << 21)
#define GPIO_BSRR_BR6               (1 << 22)
#define GPIO_BSRR_BR7               (1 << 23)
#define GPIO_BSRR_BR8               (1 << 24)
#define GPIO_BSRR_BR9               (1 << 25)
#define GPIO_BSRR_BR10              (1 << 26)
#define GPIO_BSRR_BR11              (1 << 27)
#define GPIO_BSRR_BR12              (1 << 28)
#define GPIO_BSRR_BR13              (1 << 29)
#define GPIO_BSRR_BR14              (1 << 30)
#define GPIO_BSRR_BR15              (1 << 31)

/* GPIO port configuration lock register */

#define GPIO_LCKR_LCK0              (1 << 0)
#define GPIO_LCKR_LCK1              (1 << 1)
#define GPIO_LCKR_LCK2              (1 << 2)
#define GPIO_LCKR_LCK3              (1 << 3)
#define GPIO_LCKR_LCK4              (1 << 4)
#define GPIO_LCKR_LCK5              (1 << 5)
#define GPIO_LCKR_LCK6              (1 << 6)
#define GPIO_LCKR_LCK7              (1 << 7)
#define GPIO_LCKR_LCK8              (1 << 8)
#define GPIO_LCKR_LCK9              (1 << 9)
#define GPIO_LCKR_LCK10             (1 << 10)
#define GPIO_LCKR_LCK11             (1 << 11)
#define GPIO_LCKR_LCK12             (1 << 12)
#define GPIO_LCKR_LCK13             (1 << 13)
#define GPIO_LCKR_LCK14             (1 << 14)
#define GPIO_LCKR_LCK15             (1 << 15)
#define GPIO_LCKR_LCKK              (1 << 16)

/* GPIO alternate function low register */

#define GPIO_AFRL_AFSEL0_SHIFT      (0)
#define GPIO_AFRL_AFSEL0_MASK       (0xf << GPIO_AFRL_AFSEL0_SHIFT)
#define GPIO_AFRL_AFSEL0(n)         ((n) << GPIO_AFRL_AFSEL0_SHIFT)
#define GPIO_AFRL_AFSEL1_SHIFT      (4)
#define GPIO_AFRL_AFSEL1_MASK       (0xf << GPIO_AFRL_AFSEL1_SHIFT)
#define GPIO_AFRL_AFSEL1(n)         ((n) << GPIO_AFRL_AFSEL1_SHIFT)
#define GPIO_AFRL_AFSEL2_SHIFT      (8)
#define GPIO_AFRL_AFSEL2_MASK       (0xf << GPIO_AFRL_AFSEL2_SHIFT)
#define GPIO_AFRL_AFSEL2(n)         ((n) << GPIO_AFRL_AFSEL2_SHIFT)
#define GPIO_AFRL_AFSEL3_SHIFT      (12)
#define GPIO_AFRL_AFSEL3_MASK       (0xf << GPIO_AFRL_AFSEL3_SHIFT)
#define GPIO_AFRL_AFSEL3(n)         ((n) << GPIO_AFRL_AFSEL3_SHIFT)
#define GPIO_AFRL_AFSEL4_SHIFT      (16)
#define GPIO_AFRL_AFSEL4_MASK       (0xf << GPIO_AFRL_AFSEL4_SHIFT)
#define GPIO_AFRL_AFSEL4(n)         ((n) << GPIO_AFRL_AFSEL4_SHIFT)
#define GPIO_AFRL_AFSEL5_SHIFT      (20)
#define GPIO_AFRL_AFSEL5_MASK       (0xf << GPIO_AFRL_AFSEL5_SHIFT)
#define GPIO_AFRL_AFSEL5(n)         ((n) << GPIO_AFRL_AFSEL5_SHIFT)
#define GPIO_AFRL_AFSEL6_SHIFT      (24)
#define GPIO_AFRL_AFSEL6_MASK       (0xf << GPIO_AFRL_AFSEL6_SHIFT)
#define GPIO_AFRL_AFSEL6(n)         ((n) << GPIO_AFRL_AFSEL6_SHIFT)
#define GPIO_AFRL_AFSEL7_SHIFT      (28)
#define GPIO_AFRL_AFSEL7_MASK       (0xf << GPIO_AFRL_AFSEL7_SHIFT)
#define GPIO_AFRL_AFSEL7(n)         ((n) << GPIO_AFRL_AFSEL7_SHIFT)

/* GPIO alternate function high register */

#define GPIO_AFRH_AFSEL8_SHIFT      (0)
#define GPIO_AFRH_AFSEL8_MASK       (0xf << GPIO_AFRH_AFSEL8_SHIFT)
#define GPIO_AFRH_AFSEL8(n)         ((n) << GPIO_AFRH_AFSEL8_SHIFT)
#define GPIO_AFRH_AFSEL9_SHIFT      (4)
#define GPIO_AFRH_AFSEL9_MASK       (0xf << GPIO_AFRH_AFSEL9_SHIFT)
#define GPIO_AFRH_AFSEL9(n)         ((n) << GPIO_AFRH_AFSEL9_SHIFT)
#define GPIO_AFRH_AFSEL10_SHIFT     (8)
#define GPIO_AFRH_AFSEL10_MASK      (0xf << GPIO_AFRH_AFSEL10_SHIFT)
#define GPIO_AFRH_AFSEL10(n)        ((n) << GPIO_AFRH_AFSEL10_SHIFT)
#define GPIO_AFRH_AFSEL11_SHIFT     (12)
#define GPIO_AFRH_AFSEL11_MASK      (0xf << GPIO_AFRH_AFSEL11_SHIFT)
#define GPIO_AFRH_AFSEL11(n)        ((n) << GPIO_AFRH_AFSEL11_SHIFT)
#define GPIO_AFRH_AFSEL12_SHIFT     (16)
#define GPIO_AFRH_AFSEL12_MASK      (0xf << GPIO_AFRH_AFSEL12_SHIFT)
#define GPIO_AFRH_AFSEL12(n)        ((n) << GPIO_AFRH_AFSEL12_SHIFT)
#define GPIO_AFRH_AFSEL13_SHIFT     (20)
#define GPIO_AFRH_AFSEL13_MASK      (0xf << GPIO_AFRH_AFSEL13_SHIFT)
#define GPIO_AFRH_AFSEL13(n)        ((n) << GPIO_AFRH_AFSEL13_SHIFT)
#define GPIO_AFRH_AFSEL14_SHIFT     (24)
#define GPIO_AFRH_AFSEL14_MASK      (0xf << GPIO_AFRH_AFSEL14_SHIFT)
#define GPIO_AFRH_AFSEL14(n)        ((n) << GPIO_AFRH_AFSEL14_SHIFT)
#define GPIO_AFRH_AFSEL15_SHIFT     (28)
#define GPIO_AFRH_AFSEL15_MASK      (0xf << GPIO_AFRH_AFSEL15_SHIFT)
#define GPIO_AFRH_AFSEL15(n)        ((n) << GPIO_AFRH_AFSEL15_SHIFT)

/* GPIO port bit reset register */

#define GPIO_BRR_BR0                (1 << 0)
#define GPIO_BRR_BR1                (1 << 1)
#define GPIO_BRR_BR2                (1 << 2)
#define GPIO_BRR_BR3                (1 << 3)
#define GPIO_BRR_BR4                (1 << 4)
#define GPIO_BRR_BR5                (1 << 5)
#define GPIO_BRR_BR6                (1 << 6)
#define GPIO_BRR_BR7                (1 << 7)
#define GPIO_BRR_BR8                (1 << 8)
#define GPIO_BRR_BR9                (1 << 9)
#define GPIO_BRR_BR10               (1 << 10)
#define GPIO_BRR_BR11               (1 << 11)
#define GPIO_BRR_BR12               (1 << 12)
#define GPIO_BRR_BR13               (1 << 13)
#define GPIO_BRR_BR14               (1 << 14)
#define GPIO_BRR_BR15               (1 << 15)

/* GPIO high-speed low-voltage register */

#define GPIO_HSLVR_HSLV0            (1 << 0)
#define GPIO_HSLVR_HSLV1            (1 << 1)
#define GPIO_HSLVR_HSLV2            (1 << 2)
#define GPIO_HSLVR_HSLV3            (1 << 3)
#define GPIO_HSLVR_HSLV4            (1 << 4)
#define GPIO_HSLVR_HSLV5            (1 << 5)
#define GPIO_HSLVR_HSLV6            (1 << 6)
#define GPIO_HSLVR_HSLV7            (1 << 7)
#define GPIO_HSLVR_HSLV8            (1 << 8)
#define GPIO_HSLVR_HSLV9            (1 << 9)
#define GPIO_HSLVR_HSLV10           (1 << 10)
#define GPIO_HSLVR_HSLV11           (1 << 11)
#define GPIO_HSLVR_HSLV12           (1 << 12)
#define GPIO_HSLVR_HSLV13           (1 << 13)
#define GPIO_HSLVR_HSLV14           (1 << 14)
#define GPIO_HSLVR_HSLV15           (1 << 15)

/* GPIO secure configuration register */

#define GPIO_SECCFGR_SEC0           (1 << 0)
#define GPIO_SECCFGR_SEC1           (1 << 1)
#define GPIO_SECCFGR_SEC2           (1 << 2)
#define GPIO_SECCFGR_SEC3           (1 << 3)
#define GPIO_SECCFGR_SEC4           (1 << 4)
#define GPIO_SECCFGR_SEC5           (1 << 5)
#define GPIO_SECCFGR_SEC6           (1 << 6)
#define GPIO_SECCFGR_SEC7           (1 << 7)
#define GPIO_SECCFGR_SEC8           (1 << 8)
#define GPIO_SECCFGR_SEC9           (1 << 9)
#define GPIO_SECCFGR_SEC10          (1 << 10)
#define GPIO_SECCFGR_SEC11          (1 << 11)
#define GPIO_SECCFGR_SEC12          (1 << 12)
#define GPIO_SECCFGR_SEC13          (1 << 13)
#define GPIO_SECCFGR_SEC14          (1 << 14)
#define GPIO_SECCFGR_SEC15          (1 << 15)

/* NuttX GPIO Configuration Definitions *************************************/

/* GPIO port mode register */

#define GPIO_MODER_INPUT              (0)
#define GPIO_MODER_OUTPUT             (1)
#define GPIO_MODER_ALT                (2)
#define GPIO_MODER_ANALOG             (3)
#define GPIO_MODER_SHIFT(n)           ((n) << 1)
#define GPIO_MODER_MASK(n)            (3 << GPIO_MODER_SHIFT(n))
#define GPIO_MODER0_SHIFT              GPIO_MODER_MODE0_SHIFT
#define GPIO_MODER0_MASK               GPIO_MODER_MODE0_MASK
#define GPIO_MODER1_SHIFT              GPIO_MODER_MODE1_SHIFT
#define GPIO_MODER1_MASK               GPIO_MODER_MODE1_MASK
#define GPIO_MODER2_SHIFT              GPIO_MODER_MODE2_SHIFT
#define GPIO_MODER2_MASK               GPIO_MODER_MODE2_MASK
#define GPIO_MODER3_SHIFT              GPIO_MODER_MODE3_SHIFT
#define GPIO_MODER3_MASK               GPIO_MODER_MODE3_MASK
#define GPIO_MODER4_SHIFT              GPIO_MODER_MODE4_SHIFT
#define GPIO_MODER4_MASK               GPIO_MODER_MODE4_MASK
#define GPIO_MODER5_SHIFT              GPIO_MODER_MODE5_SHIFT
#define GPIO_MODER5_MASK               GPIO_MODER_MODE5_MASK
#define GPIO_MODER6_SHIFT              GPIO_MODER_MODE6_SHIFT
#define GPIO_MODER6_MASK               GPIO_MODER_MODE6_MASK
#define GPIO_MODER7_SHIFT              GPIO_MODER_MODE7_SHIFT
#define GPIO_MODER7_MASK               GPIO_MODER_MODE7_MASK
#define GPIO_MODER8_SHIFT              GPIO_MODER_MODE8_SHIFT
#define GPIO_MODER8_MASK               GPIO_MODER_MODE8_MASK
#define GPIO_MODER9_SHIFT              GPIO_MODER_MODE9_SHIFT
#define GPIO_MODER9_MASK               GPIO_MODER_MODE9_MASK
#define GPIO_MODER10_SHIFT             GPIO_MODER_MODE10_SHIFT
#define GPIO_MODER10_MASK              GPIO_MODER_MODE10_MASK
#define GPIO_MODER11_SHIFT             GPIO_MODER_MODE11_SHIFT
#define GPIO_MODER11_MASK              GPIO_MODER_MODE11_MASK
#define GPIO_MODER12_SHIFT             GPIO_MODER_MODE12_SHIFT
#define GPIO_MODER12_MASK              GPIO_MODER_MODE12_MASK
#define GPIO_MODER13_SHIFT             GPIO_MODER_MODE13_SHIFT
#define GPIO_MODER13_MASK              GPIO_MODER_MODE13_MASK
#define GPIO_MODER14_SHIFT             GPIO_MODER_MODE14_SHIFT
#define GPIO_MODER14_MASK              GPIO_MODER_MODE14_MASK
#define GPIO_MODER15_SHIFT             GPIO_MODER_MODE15_SHIFT
#define GPIO_MODER15_MASK              GPIO_MODER_MODE15_MASK

/* GPIO port output type register */

#define GPIO_OTYPER_OD(n)             (1 << (n))
#define GPIO_OTYPER_PP(n)             (0)

/* GPIO port output speed register */

#define GPIO_OSPEED_2MHZ              (0)
#define GPIO_OSPEED_25MHZ             (1)
#define GPIO_OSPEED_50MHZ             (2)
#define GPIO_OSPEED_100MHZ            (3)
#define GPIO_OSPEED_SHIFT(n)          ((n) << 1)
#define GPIO_OSPEED_MASK(n)           (3 << GPIO_OSPEED_SHIFT(n))
#define GPIO_OSPEED0_SHIFT             GPIO_OSPEEDR_OSPEED0_SHIFT
#define GPIO_OSPEED0_MASK              GPIO_OSPEEDR_OSPEED0_MASK
#define GPIO_OSPEED1_SHIFT             GPIO_OSPEEDR_OSPEED1_SHIFT
#define GPIO_OSPEED1_MASK              GPIO_OSPEEDR_OSPEED1_MASK
#define GPIO_OSPEED2_SHIFT             GPIO_OSPEEDR_OSPEED2_SHIFT
#define GPIO_OSPEED2_MASK              GPIO_OSPEEDR_OSPEED2_MASK
#define GPIO_OSPEED3_SHIFT             GPIO_OSPEEDR_OSPEED3_SHIFT
#define GPIO_OSPEED3_MASK              GPIO_OSPEEDR_OSPEED3_MASK
#define GPIO_OSPEED4_SHIFT             GPIO_OSPEEDR_OSPEED4_SHIFT
#define GPIO_OSPEED4_MASK              GPIO_OSPEEDR_OSPEED4_MASK
#define GPIO_OSPEED5_SHIFT             GPIO_OSPEEDR_OSPEED5_SHIFT
#define GPIO_OSPEED5_MASK              GPIO_OSPEEDR_OSPEED5_MASK
#define GPIO_OSPEED6_SHIFT             GPIO_OSPEEDR_OSPEED6_SHIFT
#define GPIO_OSPEED6_MASK              GPIO_OSPEEDR_OSPEED6_MASK
#define GPIO_OSPEED7_SHIFT             GPIO_OSPEEDR_OSPEED7_SHIFT
#define GPIO_OSPEED7_MASK              GPIO_OSPEEDR_OSPEED7_MASK
#define GPIO_OSPEED8_SHIFT             GPIO_OSPEEDR_OSPEED8_SHIFT
#define GPIO_OSPEED8_MASK              GPIO_OSPEEDR_OSPEED8_MASK
#define GPIO_OSPEED9_SHIFT             GPIO_OSPEEDR_OSPEED9_SHIFT
#define GPIO_OSPEED9_MASK              GPIO_OSPEEDR_OSPEED9_MASK
#define GPIO_OSPEED10_SHIFT            GPIO_OSPEEDR_OSPEED10_SHIFT
#define GPIO_OSPEED10_MASK             GPIO_OSPEEDR_OSPEED10_MASK
#define GPIO_OSPEED11_SHIFT            GPIO_OSPEEDR_OSPEED11_SHIFT
#define GPIO_OSPEED11_MASK             GPIO_OSPEEDR_OSPEED11_MASK
#define GPIO_OSPEED12_SHIFT            GPIO_OSPEEDR_OSPEED12_SHIFT
#define GPIO_OSPEED12_MASK             GPIO_OSPEEDR_OSPEED12_MASK
#define GPIO_OSPEED13_SHIFT            GPIO_OSPEEDR_OSPEED13_SHIFT
#define GPIO_OSPEED13_MASK             GPIO_OSPEEDR_OSPEED13_MASK
#define GPIO_OSPEED14_SHIFT            GPIO_OSPEEDR_OSPEED14_SHIFT
#define GPIO_OSPEED14_MASK             GPIO_OSPEEDR_OSPEED14_MASK
#define GPIO_OSPEED15_SHIFT            GPIO_OSPEEDR_OSPEED15_SHIFT
#define GPIO_OSPEED15_MASK             GPIO_OSPEEDR_OSPEED15_MASK

/* GPIO port pull-up/pull-down register */

#define GPIO_PUPDR_NONE               (0)
#define GPIO_PUPDR_PULLUP             (1)
#define GPIO_PUPDR_PULLDOWN           (2)
#define GPIO_PUPDR_SHIFT(n)           ((n) << 1)
#define GPIO_PUPDR_MASK(n)            (3 << GPIO_PUPDR_SHIFT(n))
#define GPIO_PUPDR0_SHIFT              GPIO_PUPDR_PUPD0_SHIFT
#define GPIO_PUPDR0_MASK               GPIO_PUPDR_PUPD0_MASK
#define GPIO_PUPDR1_SHIFT              GPIO_PUPDR_PUPD1_SHIFT
#define GPIO_PUPDR1_MASK               GPIO_PUPDR_PUPD1_MASK
#define GPIO_PUPDR2_SHIFT              GPIO_PUPDR_PUPD2_SHIFT
#define GPIO_PUPDR2_MASK               GPIO_PUPDR_PUPD2_MASK
#define GPIO_PUPDR3_SHIFT              GPIO_PUPDR_PUPD3_SHIFT
#define GPIO_PUPDR3_MASK               GPIO_PUPDR_PUPD3_MASK
#define GPIO_PUPDR4_SHIFT              GPIO_PUPDR_PUPD4_SHIFT
#define GPIO_PUPDR4_MASK               GPIO_PUPDR_PUPD4_MASK
#define GPIO_PUPDR5_SHIFT              GPIO_PUPDR_PUPD5_SHIFT
#define GPIO_PUPDR5_MASK               GPIO_PUPDR_PUPD5_MASK
#define GPIO_PUPDR6_SHIFT              GPIO_PUPDR_PUPD6_SHIFT
#define GPIO_PUPDR6_MASK               GPIO_PUPDR_PUPD6_MASK
#define GPIO_PUPDR7_SHIFT              GPIO_PUPDR_PUPD7_SHIFT
#define GPIO_PUPDR7_MASK               GPIO_PUPDR_PUPD7_MASK
#define GPIO_PUPDR8_SHIFT              GPIO_PUPDR_PUPD8_SHIFT
#define GPIO_PUPDR8_MASK               GPIO_PUPDR_PUPD8_MASK
#define GPIO_PUPDR9_SHIFT              GPIO_PUPDR_PUPD9_SHIFT
#define GPIO_PUPDR9_MASK               GPIO_PUPDR_PUPD9_MASK
#define GPIO_PUPDR10_SHIFT             GPIO_PUPDR_PUPD10_SHIFT
#define GPIO_PUPDR10_MASK              GPIO_PUPDR_PUPD10_MASK
#define GPIO_PUPDR11_SHIFT             GPIO_PUPDR_PUPD11_SHIFT
#define GPIO_PUPDR11_MASK              GPIO_PUPDR_PUPD11_MASK
#define GPIO_PUPDR12_SHIFT             GPIO_PUPDR_PUPD12_SHIFT
#define GPIO_PUPDR12_MASK              GPIO_PUPDR_PUPD12_MASK
#define GPIO_PUPDR13_SHIFT             GPIO_PUPDR_PUPD13_SHIFT
#define GPIO_PUPDR13_MASK              GPIO_PUPDR_PUPD13_MASK
#define GPIO_PUPDR14_SHIFT             GPIO_PUPDR_PUPD14_SHIFT
#define GPIO_PUPDR14_MASK              GPIO_PUPDR_PUPD14_MASK
#define GPIO_PUPDR15_SHIFT             GPIO_PUPDR_PUPD15_SHIFT
#define GPIO_PUPDR15_MASK              GPIO_PUPDR_PUPD15_MASK

/* GPIO port input and output data registers */

#define GPIO_IDR(n)                   (1 << (n))
#define GPIO_ODR(n)                   (1 << (n))

/* GPIO port bit set/reset register */

#define GPIO_BSRR_SET(n)              (1 << (n))
#define GPIO_BSRR_RESET(n)            (1 << ((n) + 16))

/* GPIO port configuration lock register */

#define GPIO_LCKR(n)                  (1 << (n))
#define GPIO_LCKK                     GPIO_LCKR_LCKK

/* GPIO alternate function low/high registers */

#define GPIO_AFR_SHIFT(n)             ((n) << 2)
#define GPIO_AFR_MASK(n)              (15 << GPIO_AFR_SHIFT(n))
#define GPIO_AFRL0_SHIFT               GPIO_AFRL_AFSEL0_SHIFT
#define GPIO_AFRL0_MASK                GPIO_AFRL_AFSEL0_MASK
#define GPIO_AFRL1_SHIFT               GPIO_AFRL_AFSEL1_SHIFT
#define GPIO_AFRL1_MASK                GPIO_AFRL_AFSEL1_MASK
#define GPIO_AFRL2_SHIFT               GPIO_AFRL_AFSEL2_SHIFT
#define GPIO_AFRL2_MASK                GPIO_AFRL_AFSEL2_MASK
#define GPIO_AFRL3_SHIFT               GPIO_AFRL_AFSEL3_SHIFT
#define GPIO_AFRL3_MASK                GPIO_AFRL_AFSEL3_MASK
#define GPIO_AFRL4_SHIFT               GPIO_AFRL_AFSEL4_SHIFT
#define GPIO_AFRL4_MASK                GPIO_AFRL_AFSEL4_MASK
#define GPIO_AFRL5_SHIFT               GPIO_AFRL_AFSEL5_SHIFT
#define GPIO_AFRL5_MASK                GPIO_AFRL_AFSEL5_MASK
#define GPIO_AFRL6_SHIFT               GPIO_AFRL_AFSEL6_SHIFT
#define GPIO_AFRL6_MASK                GPIO_AFRL_AFSEL6_MASK
#define GPIO_AFRL7_SHIFT               GPIO_AFRL_AFSEL7_SHIFT
#define GPIO_AFRL7_MASK                GPIO_AFRL_AFSEL7_MASK
#define GPIO_AFRH8_SHIFT               GPIO_AFRH_AFSEL8_SHIFT
#define GPIO_AFRH8_MASK                GPIO_AFRH_AFSEL8_MASK
#define GPIO_AFRH9_SHIFT               GPIO_AFRH_AFSEL9_SHIFT
#define GPIO_AFRH9_MASK                GPIO_AFRH_AFSEL9_MASK
#define GPIO_AFRH10_SHIFT              GPIO_AFRH_AFSEL10_SHIFT
#define GPIO_AFRH10_MASK               GPIO_AFRH_AFSEL10_MASK
#define GPIO_AFRH11_SHIFT              GPIO_AFRH_AFSEL11_SHIFT
#define GPIO_AFRH11_MASK               GPIO_AFRH_AFSEL11_MASK
#define GPIO_AFRH12_SHIFT              GPIO_AFRH_AFSEL12_SHIFT
#define GPIO_AFRH12_MASK               GPIO_AFRH_AFSEL12_MASK
#define GPIO_AFRH13_SHIFT              GPIO_AFRH_AFSEL13_SHIFT
#define GPIO_AFRH13_MASK               GPIO_AFRH_AFSEL13_MASK
#define GPIO_AFRH14_SHIFT              GPIO_AFRH_AFSEL14_SHIFT
#define GPIO_AFRH14_MASK               GPIO_AFRH_AFSEL14_MASK
#define GPIO_AFRH15_SHIFT              GPIO_AFRH_AFSEL15_SHIFT
#define GPIO_AFRH15_MASK               GPIO_AFRH_AFSEL15_MASK

/* GPIO port bit reset register */

#define GPIO_BRR_SET(n)               (1 << (n))

/* GPIO high-speed low-voltage register */

#define GPIO_HSLVR_SET(n)             (1 << (n))

/* GPIO port secure configuration register */

#define GPIO_SECCFGR_SET(n)           (1 << (n))

#endif /* __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_GPIO_M33_V1_H */
