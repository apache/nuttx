/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f4xx_gpio.h
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

#ifndef __ARCH_ARM_SRC_GD32_HARDWARE_GD32F4XX_GPIO_H
#define __ARCH_ARM_SRC_GD32_HARDWARE_GD32F4XX_GPIO_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIOx(x=A,B,C,D,E,F,G,H,I) definitions ***********************************/
#define GD32_GPIOA_BASE                  (GD32_GPIO_BASE + 0x00000000)
#define GD32_GPIOB_BASE                  (GD32_GPIO_BASE + 0x00000400)
#define GD32_GPIOC_BASE                  (GD32_GPIO_BASE + 0x00000800)
#define GD32_GPIOD_BASE                  (GD32_GPIO_BASE + 0x00000c00)
#define GD32_GPIOE_BASE                  (GD32_GPIO_BASE + 0x00001000)
#define GD32_GPIOF_BASE                  (GD32_GPIO_BASE + 0x00001400)
#define GD32_GPIOG_BASE                  (GD32_GPIO_BASE + 0x00001800)
#define GD32_GPIOH_BASE                  (GD32_GPIO_BASE + 0x00001c00)
#define GD32_GPIOI_BASE                  (GD32_GPIO_BASE + 0x00002000)

/* Register Offsets *********************************************************/

#define GD32_GPIO_CTL_OFFSET             0x0000 /* GPIO port control register offfset */
#define GD32_GPIO_OMODE_OFFSET           0x0004 /* GPIO port output mode register offfset */
#define GD32_GGPIO_OSPD_OFFSET           0x0008 /* GPIO port output speed register offfset */
#define GD32_GPIO_PUD_OFFSET             0x000c /* GPIO port pull-up/pull-down register offfset */
#define GD32_GPIO_ISTAT_OFFSET           0x0010 /* GPIO port input status register offfset */
#define GD32_GPIO_OCTL_OFFSET            0x0014 /* GPIO port output control register offfset */
#define GD32_GPIO_BOP_OFFSET             0x0018 /* GPIO port bit operation register offfset */
#define GD32_GPIO_LOCK_OFFSET            0x001c /* GPIO port configuration lock register offfset */
#define GD32_GPIO_AFSEL0_OFFSET          0x0020 /* GPIO alternate function selected register 0 offfset */
#define GD32_GPIO_AFSEL1_OFFSET          0x0024 /* GPIO alternate function selected register 1 offfset */
#define GD32_GPIO_BC_OFFSET              0x0028 /* GPIO bit clear register offfset */
#define GD32_GPIO_TG_OFFSET              0x002c /* GPIO port bit toggle register offfset */

/* Register Addresses *******************************************************/

#define GD32_GPIOA                       GD32_GPIOA_BASE
#define GD32_GPIOB                       GD32_GPIOB_BASE
#define GD32_GPIOC                       GD32_GPIOC_BASE
#define GD32_GPIOD                       GD32_GPIOD_BASE
#define GD32_GPIOE                       GD32_GPIOE_BASE
#define GD32_GPIOF                       GD32_GPIOF_BASE
#define GD32_GPIOG                       GD32_GPIOG_BASE
#define GD32_GPIOH                       GD32_GPIOH_BASE
#define GD32_GPIOI                       GD32_GPIOI_BASE

#define GD32_GPIO_CTL(gpiox)            ((gpiox)+GD32_GPIO_CTL_OFFSET)    /* GPIO port control register */
#define GD32_GPIO_OMODE(gpiox)          ((gpiox)+GD32_GPIO_OMODE_OFFSET)  /* GPIO port output mode register */
#define GD32_GPIO_OSPD(gpiox)           ((gpiox)+GD32_GGPIO_OSPD_OFFSET)  /* GPIO port output speed register */
#define GD32_GPIO_PUD(gpiox)            ((gpiox)+GD32_GPIO_PUD_OFFSET)    /* GPIO port pull-up/pull-down register */
#define GD32_GPIO_ISTAT(gpiox)          ((gpiox)+GD32_GPIO_ISTAT_OFFSET)  /* GPIO port input status register */
#define GD32_GPIO_OCTL(gpiox)           ((gpiox)+GD32_GPIO_OCTL_OFFSET)   /* GPIO port output control register */
#define GD32_GPIO_BOP(gpiox)            ((gpiox)+GD32_GPIO_BOP_OFFSET)    /* GPIO port bit operation register */
#define GD32_GPIO_LOCK(gpiox)           ((gpiox)+GD32_GPIO_LOCK_OFFSET)   /* GPIO port configuration lock register */
#define GD32_GPIO_AFSEL0(gpiox)         ((gpiox)+GD32_GPIO_AFSEL0_OFFSET) /* GPIO alternate function selected register 0 */
#define GD32_GPIO_AFSEL1(gpiox)         ((gpiox)+GD32_GPIO_AFSEL1_OFFSET) /* GPIO alternate function selected register 1 */
#define GD32_GPIO_BC(gpiox)             ((gpiox)+GD32_GPIO_BC_OFFSET)     /* GPIO bit clear register */
#define GD32_GPIO_TG(gpiox)             ((gpiox)+GD32_GPIO_TG_OFFSET)     /* GPIO port bit toggle register */

/* Register Bitfield Definitions ********************************************/

/* GPIO port control register */

#define GPIO_MODE_INPUT                  (0) /* Input mode */
#define GPIO_MODE_OUTPUT                 (1) /* Output mode */
#define GPIO_MODE_AF                     (2) /* Alternate function mode */
#define GPIO_MODE_ANALOG                 (3) /* Analog mode */

#define GPIO_MODE_SHIFT(n)               ((n) << 1)
#define GPIO_MODE_MASK(n)                (3 << GPIO_MODE_SHIFT(n))

#define GPIO_MODE0_MASK                  (3 << GPIO_MODE_SHIFT(0))
#define GPIO_MODE1_MASK                  (3 << GPIO_MODE_SHIFT(1))
#define GPIO_MODE2_MASK                  (3 << GPIO_MODE_SHIFT(2))
#define GPIO_MODE3_MASK                  (3 << GPIO_MODE_SHIFT(3))
#define GPIO_MODE4_MASK                  (3 << GPIO_MODE_SHIFT(4))
#define GPIO_MODE5_MASK                  (3 << GPIO_MODE_SHIFT(5))
#define GPIO_MODE6_MASK                  (3 << GPIO_MODE_SHIFT(6))
#define GPIO_MODE7_MASK                  (3 << GPIO_MODE_SHIFT(7))
#define GPIO_MODE8_MASK                  (3 << GPIO_MODE_SHIFT(8))
#define GPIO_MODE9_MASK                  (3 << GPIO_MODE_SHIFT(9))
#define GPIO_MODE10_MASK                 (3 << GPIO_MODE_SHIFT(10))
#define GPIO_MODE11_MASK                 (3 << GPIO_MODE_SHIFT(11))
#define GPIO_MODE12_MASK                 (3 << GPIO_MODE_SHIFT(12))
#define GPIO_MODE13_MASK                 (3 << GPIO_MODE_SHIFT(13))
#define GPIO_MODE14_MASK                 (3 << GPIO_MODE_SHIFT(14))
#define GPIO_MODE15_MASK                 (3 << GPIO_MODE_SHIFT(15))

/* GPIO port output mode register */

#define GPIO_OTYPE_OD(n)                 (1 << (n)) /* 1=Output open drain mode */
#define GPIO_OTYPE_PP(n)                 (0)        /* 0=Output push pull mode */

/* GPIO port output speed register */

#define GPIO_OSPEED_2MHZ                 (0) /* Output max speed 2MHz */
#define GPIO_OSPEED_25MHZ                (1) /* Output max speed 25MHz */
#define GPIO_OSPEED_50MHZ                (2) /* Output max speed 50MHz */
#define GPIO_OSPEED_200MHZ               (3) /* Output max speed 200MHz */

#define GPIO_OSPEED_SHIFT(n)             ((n) << 1)
#define GPIO_OSPEED_MASK(n)              (3 << GPIO_OSPEED_SHIFT(n))

#define GPIO_OSPEED0_MASK                (3 << GPIO_OSPEED_SHIFT(0))
#define GPIO_OSPEED1_MASK                (3 << GPIO_OSPEED_SHIFT(1))
#define GPIO_OSPEED2_MASK                (3 << GPIO_OSPEED_SHIFT(2))
#define GPIO_OSPEED3_MASK                (3 << GPIO_OSPEED_SHIFT(3))
#define GPIO_OSPEED4_MASK                (3 << GPIO_OSPEED_SHIFT(4))
#define GPIO_OSPEED5_MASK                (3 << GPIO_OSPEED_SHIFT(5))
#define GPIO_OSPEED6_MASK                (3 << GPIO_OSPEED_SHIFT(6))
#define GPIO_OSPEED7_MASK                (3 << GPIO_OSPEED_SHIFT(7))
#define GPIO_OSPEED8_MASK                (3 << GPIO_OSPEED_SHIFT(8))
#define GPIO_OSPEED9_MASK                (3 << GPIO_OSPEED_SHIFT(9))
#define GPIO_OSPEED10_MASK               (3 << GPIO_OSPEED_SHIFT(10))
#define GPIO_OSPEED11_MASK               (3 << GPIO_OSPEED_SHIFT(11))
#define GPIO_OSPEED12_MASK               (3 << GPIO_OSPEED_SHIFT(12))
#define GPIO_OSPEED13_MASK               (3 << GPIO_OSPEED_SHIFT(13))
#define GPIO_OSPEED14_MASK               (3 << GPIO_OSPEED_SHIFT(14))
#define GPIO_OSPEED15_MASK               (3 << GPIO_OSPEED_SHIFT(15))

/* GPIO port pull-up/pull-down register */

#define GPIO_PUPD_NONE                   (0) /* Floating mode, no pull-up and pull-down resistors */
#define GPIO_PUPD_PULLUP                 (1) /* With pull-up resistor */
#define GPIO_PUPD_PULLDOWN               (2) /* With pull-up resistor */

#define GPIO_PUPD_SHIFT(n)               ((n) << 1)
#define GPIO_PUPD_MASK(n)                (3 << GPIO_PUPD_SHIFT(n))

#define GPIO_PUPD0_MASK                  (3 << GPIO_PUPD_SHIFT(0))
#define GPIO_PUPD1_MASK                  (3 << GPIO_PUPD_SHIFT(1))
#define GPIO_PUPD2_MASK                  (3 << GPIO_PUPD_SHIFT(2))
#define GPIO_PUPD3_MASK                  (3 << GPIO_PUPD_SHIFT(3))
#define GPIO_PUPD4_MASK                  (3 << GPIO_PUPD_SHIFT(4))
#define GPIO_PUPD5_MASK                  (3 << GPIO_PUPD_SHIFT(5))
#define GPIO_PUPD6_MASK                  (3 << GPIO_PUPD_SHIFT(6))
#define GPIO_PUPD7_MASK                  (3 << GPIO_PUPD_SHIFT(7))
#define GPIO_PUPD8_MASK                  (3 << GPIO_PUPD_SHIFT(8))
#define GPIO_PUPD9_MASK                  (3 << GPIO_PUPD_SHIFT(9))
#define GPIO_PUPD10_MASK                 (3 << GPIO_PUPD_SHIFT(10))
#define GPIO_PUPD11_MASK                 (3 << GPIO_PUPD_SHIFT(11))
#define GPIO_PUPD12_MASK                 (3 << GPIO_PUPD_SHIFT(12))
#define GPIO_PUPD13_MASK                 (3 << GPIO_PUPD_SHIFT(13))
#define GPIO_PUPD14_MASK                 (3 << GPIO_PUPD_SHIFT(14))
#define GPIO_PUPD15_MASK                 (3 << GPIO_PUPD_SHIFT(15))

/* GPIO port input status register */

#define GPIO_ISTAT(n)                    (1 << (n))

/* GPIO port output control register */

#define GPIO_OCTL(n)                     (1 << (n))

/* GPIO port bit operation register */

#define GPIO_BOP_SET(n)                  (1 << (n))
#define GPIO_BOP_CLEAR(n)                (1 << ((n)+16))

/* GPIO port configuration lock register */

#define GPIO_LOCK(n)                     (1 << (n))
#define GPIO_LOCK_LKK                    (1 << 16)   /* Lock key */

/* GPIO alternate function selected register 0/1 */

#define GPIO_AF_SHIFT(n)                 ((n) << 2)
#define GPIO_AF_MASK(n)                  (15 << GPIO_AF_SHIFT(n))

#define GPIO_AFSEL0_SEL0_MASK            (15 << GPIO_AF_SHIFT(0))
#define GPIO_AFSEL0_SEL1_MASK            (15 << GPIO_AF_SHIFT(1))
#define GPIO_AFSEL0_SEL2_MASK            (15 << GPIO_AF_SHIFT(2))
#define GPIO_AFSEL0_SEL3_MASK            (15 << GPIO_AF_SHIFT(3))
#define GPIO_AFSEL0_SEL4_MASK            (15 << GPIO_AF_SHIFT(4))
#define GPIO_AFSEL0_SEL5_MASK            (15 << GPIO_AF_SHIFT(5))
#define GPIO_AFSEL0_SEL6_MASK            (15 << GPIO_AF_SHIFT(6))
#define GPIO_AFSEL0_SEL7_MASK            (15 << GPIO_AF_SHIFT(7))

#define GPIO_AFSEL1_SEL8_MASK            (15 << GPIO_AF_SHIFT(0))
#define GPIO_AFSEL1_SEL9_MASK            (15 << GPIO_AF_SHIFT(1))
#define GPIO_AFSEL1_SEL10_MASK           (15 << GPIO_AF_SHIFT(2))
#define GPIO_AFSEL1_SEL11_MASK           (15 << GPIO_AF_SHIFT(3))
#define GPIO_AFSEL1_SEL12_MASK           (15 << GPIO_AF_SHIFT(4))
#define GPIO_AFSEL1_SEL13_MASK           (15 << GPIO_AF_SHIFT(5))
#define GPIO_AFSEL1_SEL14_MASK           (15 << GPIO_AF_SHIFT(6))
#define GPIO_AFSEL1_SEL15_MASK           (15 << GPIO_AF_SHIFT(7))

/* GPIO bit clear register */

#define GPIO_BC_SET(n)                   (1 << (n))

/* GPIO port bit toggle register */
#define GPIO_TG_SET(n)                   (1 << (n))

/* GPIO pin definitions */
#define GPIO_PIN(n)                      (1 << (n)) /* Bit n: Pin n, n=0-15 */

#endif /* __ARCH_ARM_SRC_GD32_HARDWARE_GD32F4XX_GPIO_H */
