/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32g4xxxx_gpio.h
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_GPIO_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_GPIO_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_NGPIO_PORTS              (7)            /* GPIOA-G */

/* Register Offsets *********************************************************/

#define STM32_GPIO_MODER_OFFSET        0x0000         /* GPIO port mode register */
#define STM32_GPIO_OTYPER_OFFSET       0x0004         /* GPIO port output type register */
#define STM32_GPIO_OSPEED_OFFSET       0x0008         /* GPIO port output speed register */
#define STM32_GPIO_PUPDR_OFFSET        0x000c         /* GPIO port pull-up/pull-down register */
#define STM32_GPIO_IDR_OFFSET          0x0010         /* GPIO port input data register */
#define STM32_GPIO_ODR_OFFSET          0x0014         /* GPIO port output data register */
#define STM32_GPIO_BSRR_OFFSET         0x0018         /* GPIO port bit set/reset register */
#define STM32_GPIO_LCKR_OFFSET         0x001c         /* GPIO port configuration lock register */
#define STM32_GPIO_AFRL_OFFSET         0x0020         /* GPIO alternate function low register */
#define STM32_GPIO_AFRH_OFFSET         0x0024         /* GPIO alternate function high register */
#define STM32_GPIO_BRR_OFFSET          0x0028         /* GPIO port bit reset register */

/* Register Addresses *******************************************************/

#if (STM32_NGPIO_PORTS > 0)
#  define STM32_GPIOA_MODER            (STM32_GPIOA_BASE + STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOA_OTYPER           (STM32_GPIOA_BASE + STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOA_OSPEED           (STM32_GPIOA_BASE + STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOA_PUPDR            (STM32_GPIOA_BASE + STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOA_IDR              (STM32_GPIOA_BASE + STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOA_ODR              (STM32_GPIOA_BASE + STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOA_BSRR             (STM32_GPIOA_BASE + STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOA_LCKR             (STM32_GPIOA_BASE + STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOA_AFRL             (STM32_GPIOA_BASE + STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOA_AFRH             (STM32_GPIOA_BASE + STM32_GPIO_AFRH_OFFSET)
#  define STM32_GPIOA_BRR              (STM32_GPIOA_BASE + STM32_GPIO_BRR_OFFSET)
#endif

#if (STM32_NGPIO_PORTS > 1)
#  define STM32_GPIOB_MODER            (STM32_GPIOB_BASE + STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOB_OTYPER           (STM32_GPIOB_BASE + STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOB_OSPEED           (STM32_GPIOB_BASE + STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOB_PUPDR            (STM32_GPIOB_BASE + STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOB_IDR              (STM32_GPIOB_BASE + STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOB_ODR              (STM32_GPIOB_BASE + STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOB_BSRR             (STM32_GPIOB_BASE + STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOB_LCKR             (STM32_GPIOB_BASE + STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOB_AFRL             (STM32_GPIOB_BASE + STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOB_AFRH             (STM32_GPIOB_BASE + STM32_GPIO_AFRH_OFFSET)
#  define STM32_GPIOB_BRR              (STM32_GPIOB_BASE + STM32_GPIO_BRR_OFFSET)
#endif

#if (STM32_NGPIO_PORTS > 2)
#  define STM32_GPIOC_MODER            (STM32_GPIOC_BASE + STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOC_OTYPER           (STM32_GPIOC_BASE + STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOC_OSPEED           (STM32_GPIOC_BASE + STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOC_PUPDR            (STM32_GPIOC_BASE + STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOC_IDR              (STM32_GPIOC_BASE + STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOC_ODR              (STM32_GPIOC_BASE + STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOC_BSRR             (STM32_GPIOC_BASE + STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOC_LCKR             (STM32_GPIOC_BASE + STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOC_AFRL             (STM32_GPIOC_BASE + STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOC_AFRH             (STM32_GPIOC_BASE + STM32_GPIO_AFRH_OFFSET)
#  define STM32_GPIOC_BRR              (STM32_GPIOC_BASE + STM32_GPIO_BRR_OFFSET)
#endif

#if (STM32_NGPIO_PORTS > 3)
#  define STM32_GPIOD_MODER            (STM32_GPIOD_BASE + STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOD_OTYPER           (STM32_GPIOD_BASE + STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOD_OSPEED           (STM32_GPIOD_BASE + STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOD_PUPDR            (STM32_GPIOD_BASE + STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOD_IDR              (STM32_GPIOD_BASE + STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOD_ODR              (STM32_GPIOD_BASE + STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOD_BSRR             (STM32_GPIOD_BASE + STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOD_LCKR             (STM32_GPIOD_BASE + STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOD_AFRL             (STM32_GPIOD_BASE + STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOD_AFRH             (STM32_GPIOD_BASE + STM32_GPIO_AFRH_OFFSET)
#  define STM32_GPIOD_BRR              (STM32_GPIOD_BASE + STM32_GPIO_BRR_OFFSET)
#endif

#if (STM32_NGPIO_PORTS > 4)
#  define STM32_GPIOE_MODER            (STM32_GPIOE_BASE + STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOE_OTYPER           (STM32_GPIOE_BASE + STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOE_OSPEED           (STM32_GPIOE_BASE + STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOE_PUPDR            (STM32_GPIOE_BASE + STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOE_IDR              (STM32_GPIOE_BASE + STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOE_ODR              (STM32_GPIOE_BASE + STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOE_BSRR             (STM32_GPIOE_BASE + STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOE_LCKR             (STM32_GPIOE_BASE + STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOE_AFRL             (STM32_GPIOE_BASE + STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOE_AFRH             (STM32_GPIOE_BASE + STM32_GPIO_AFRH_OFFSET)
#  define STM32_GPIOE_BRR              (STM32_GPIOE_BASE + STM32_GPIO_BRR_OFFSET)
#endif

#if (STM32_NGPIO_PORTS > 5)
#  define STM32_GPIOF_MODER            (STM32_GPIOF_BASE + STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOF_OTYPER           (STM32_GPIOF_BASE + STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOF_OSPEED           (STM32_GPIOF_BASE + STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOF_PUPDR            (STM32_GPIOF_BASE + STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOF_IDR              (STM32_GPIOF_BASE + STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOF_ODR              (STM32_GPIOF_BASE + STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOF_BSRR             (STM32_GPIOF_BASE + STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOF_LCKR             (STM32_GPIOF_BASE + STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOF_AFRL             (STM32_GPIOF_BASE + STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOF_AFRH             (STM32_GPIOF_BASE + STM32_GPIO_AFRH_OFFSET)
#  define STM32_GPIOF_BRR              (STM32_GPIOF_BASE + STM32_GPIO_BRR_OFFSET)
#endif

#if (STM32_NGPIO_PORTS > 6)
#  define STM32_GPIOG_MODER            (STM32_GPIOG_BASE + STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOG_OTYPER           (STM32_GPIOG_BASE + STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOG_OSPEED           (STM32_GPIOG_BASE + STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOG_PUPDR            (STM32_GPIOG_BASE + STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOG_IDR              (STM32_GPIOG_BASE + STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOG_ODR              (STM32_GPIOG_BASE + STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOG_BSRR             (STM32_GPIOG_BASE + STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOG_LCKR             (STM32_GPIOG_BASE + STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOG_AFRL             (STM32_GPIOG_BASE + STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOG_AFRH             (STM32_GPIOG_BASE + STM32_GPIO_AFRH_OFFSET)
#  define STM32_GPIOG_BRR              (STM32_GPIOG_BASE + STM32_GPIO_BRR_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* GPIO port mode register */

#define GPIO_MODER_INPUT               (0x0)          /* Input mode */
#define GPIO_MODER_OUTPUT              (0x1)          /* General purpose output mode */
#define GPIO_MODER_ALT                 (0x2)          /* Alternate mode */
#define GPIO_MODER_ANALOG              (0x3)          /* Analog mode (reset state) */

#define GPIO_MODER_SHIFT(n)            ((n) << 1)
#define GPIO_MODER_MASK(n)             (0x3 << GPIO_MODER_SHIFT(n))

#define GPIO_MODER0_SHIFT              (0)
#define GPIO_MODER0_MASK               (3 << GPIO_MODER0_SHIFT)
#define GPIO_MODER1_SHIFT              (2)
#define GPIO_MODER1_MASK               (3 << GPIO_MODER1_SHIFT)
#define GPIO_MODER2_SHIFT              (4)
#define GPIO_MODER2_MASK               (3 << GPIO_MODER2_SHIFT)
#define GPIO_MODER3_SHIFT              (6)
#define GPIO_MODER3_MASK               (3 << GPIO_MODER3_SHIFT)
#define GPIO_MODER4_SHIFT              (8)
#define GPIO_MODER4_MASK               (3 << GPIO_MODER4_SHIFT)
#define GPIO_MODER5_SHIFT              (10)
#define GPIO_MODER5_MASK               (3 << GPIO_MODER5_SHIFT)
#define GPIO_MODER6_SHIFT              (12)
#define GPIO_MODER6_MASK               (3 << GPIO_MODER6_SHIFT)
#define GPIO_MODER7_SHIFT              (14)
#define GPIO_MODER7_MASK               (3 << GPIO_MODER7_SHIFT)
#define GPIO_MODER8_SHIFT              (16)
#define GPIO_MODER8_MASK               (3 << GPIO_MODER8_SHIFT)
#define GPIO_MODER9_SHIFT              (18)
#define GPIO_MODER9_MASK               (3 << GPIO_MODER9_SHIFT)
#define GPIO_MODER10_SHIFT             (20)
#define GPIO_MODER10_MASK              (3 << GPIO_MODER10_SHIFT)
#define GPIO_MODER11_SHIFT             (22)
#define GPIO_MODER11_MASK              (3 << GPIO_MODER11_SHIFT)
#define GPIO_MODER12_SHIFT             (24)
#define GPIO_MODER12_MASK              (3 << GPIO_MODER12_SHIFT)
#define GPIO_MODER13_SHIFT             (26)
#define GPIO_MODER13_MASK              (3 << GPIO_MODER13_SHIFT)
#define GPIO_MODER14_SHIFT             (28)
#define GPIO_MODER14_MASK              (3 << GPIO_MODER14_SHIFT)
#define GPIO_MODER15_SHIFT             (30)
#define GPIO_MODER15_MASK              (3 << GPIO_MODER15_SHIFT)

/* GPIO port output type register */

#define GPIO_OTYPER_PP(n)              (0)            /* 0=Output push-pull (reset state) */
#define GPIO_OTYPER_OD(n)              (1 << (n))     /* 1=Output open-drain */

/* GPIO port output speed register */

#define GPIO_OSPEED_5MHz               (0x0)          /* 5 MHz Low speed output */
#define GPIO_OSPEED_25MHz              (0x1)          /* 25 MHz Medium speed output */
#define GPIO_OSPEED_50MHz              (0x2)          /* 50 MHz Fast speed output */
#define GPIO_OSPEED_120MHz             (0x3)          /* 120 MHz High speed output */

#define GPIO_OSPEED_SHIFT(n)           ((n) << 1)
#define GPIO_OSPEED_MASK(n)            (0x3 << GPIO_OSPEED_SHIFT(n))

#define GPIO_OSPEED0_SHIFT             (GPIO_OSPEED_SHIFT(0))
#define GPIO_OSPEED0_MASK              (GPIO_OSPEED_MASK(0))
#define GPIO_OSPEED1_SHIFT             (GPIO_OSPEED_SHIFT(1))
#define GPIO_OSPEED1_MASK              (GPIO_OSPEED_MASK(1))
#define GPIO_OSPEED2_SHIFT             (GPIO_OSPEED_SHIFT(2))
#define GPIO_OSPEED2_MASK              (GPIO_OSPEED_MASK(2))
#define GPIO_OSPEED3_SHIFT             (GPIO_OSPEED_SHIFT(3))
#define GPIO_OSPEED3_MASK              (GPIO_OSPEED_MASK(3))
#define GPIO_OSPEED4_SHIFT             (GPIO_OSPEED_SHIFT(4))
#define GPIO_OSPEED4_MASK              (GPIO_OSPEED_MASK(4))
#define GPIO_OSPEED5_SHIFT             (GPIO_OSPEED_SHIFT(5))
#define GPIO_OSPEED5_MASK              (GPIO_OSPEED_MASK(5))
#define GPIO_OSPEED6_SHIFT             (GPIO_OSPEED_SHIFT(6))
#define GPIO_OSPEED6_MASK              (GPIO_OSPEED_MASK(6))
#define GPIO_OSPEED7_SHIFT             (GPIO_OSPEED_SHIFT(7))
#define GPIO_OSPEED7_MASK              (GPIO_OSPEED_MASK(7))
#define GPIO_OSPEED8_SHIFT             (GPIO_OSPEED_SHIFT(8))
#define GPIO_OSPEED8_MASK              (GPIO_OSPEED_MASK(8))
#define GPIO_OSPEED9_SHIFT             (GPIO_OSPEED_SHIFT(9))
#define GPIO_OSPEED9_MASK              (GPIO_OSPEED_MASK(9))
#define GPIO_OSPEED10_SHIFT            (GPIO_OSPEED_SHIFT(10))
#define GPIO_OSPEED10_MASK             (GPIO_OSPEED_MASK(10))
#define GPIO_OSPEED11_SHIFT            (GPIO_OSPEED_SHIFT(11))
#define GPIO_OSPEED11_MASK             (GPIO_OSPEED_MASK(11))
#define GPIO_OSPEED12_SHIFT            (GPIO_OSPEED_SHIFT(12))
#define GPIO_OSPEED12_MASK             (GPIO_OSPEED_MASK(12))
#define GPIO_OSPEED13_SHIFT            (GPIO_OSPEED_SHIFT(13))
#define GPIO_OSPEED13_MASK             (GPIO_OSPEED_MASK(13))
#define GPIO_OSPEED14_SHIFT            (GPIO_OSPEED_SHIFT(14))
#define GPIO_OSPEED14_MASK             (GPIO_OSPEED_MASK(14))
#define GPIO_OSPEED15_SHIFT            (GPIO_OSPEED_SHIFT(15))
#define GPIO_OSPEED15_MASK             (GPIO_OSPEED_MASK(15))

/* GPIO port pull-up/pull-down register */

#define GPIO_PUPDR_NONE                (0x0)          /* No pull-up, pull-down */
#define GPIO_PUPDR_PULLUP              (0x1)          /* Pull-up */
#define GPIO_PUPDR_PULLDOWN            (0x2)          /* Pull-down */

#define GPIO_PUPDR_SHIFT(n)            ((n) << 1)
#define GPIO_PUPDR_MASK(n)             (0x3 << GPIO_PUPDR_SHIFT(n))

#define GPIO_PUPDR0_SHIFT              GPIO_PUPDR_SHIFT(0)
#define GPIO_PUPDR0_MASK               GPIO_PUPDR_MASK(0)
#define GPIO_PUPDR1_SHIFT              GPIO_PUPDR_SHIFT(1)
#define GPIO_PUPDR1_MASK               GPIO_PUPDR_MASK(1)
#define GPIO_PUPDR2_SHIFT              GPIO_PUPDR_SHIFT(2)
#define GPIO_PUPDR2_MASK               GPIO_PUPDR_MASK(2)
#define GPIO_PUPDR3_SHIFT              GPIO_PUPDR_SHIFT(3)
#define GPIO_PUPDR3_MASK               GPIO_PUPDR_MASK(3)
#define GPIO_PUPDR4_SHIFT              GPIO_PUPDR_SHIFT(4)
#define GPIO_PUPDR4_MASK               GPIO_PUPDR_MASK(4)
#define GPIO_PUPDR5_SHIFT              GPIO_PUPDR_SHIFT(5)
#define GPIO_PUPDR5_MASK               GPIO_PUPDR_MASK(5)
#define GPIO_PUPDR6_SHIFT              GPIO_PUPDR_SHIFT(6)
#define GPIO_PUPDR6_MASK               GPIO_PUPDR_MASK(6)
#define GPIO_PUPDR7_SHIFT              GPIO_PUPDR_SHIFT(7)
#define GPIO_PUPDR7_MASK               GPIO_PUPDR_MASK(7)
#define GPIO_PUPDR8_SHIFT              GPIO_PUPDR_SHIFT(8)
#define GPIO_PUPDR8_MASK               GPIO_PUPDR_MASK(8)
#define GPIO_PUPDR9_SHIFT              GPIO_PUPDR_SHIFT(9)
#define GPIO_PUPDR9_MASK               GPIO_PUPDR_MASK(9)
#define GPIO_PUPDR10_SHIFT             GPIO_PUPDR_SHIFT(10)
#define GPIO_PUPDR10_MASK              GPIO_PUPDR_MASK(10)
#define GPIO_PUPDR11_SHIFT             GPIO_PUPDR_SHIFT(11)
#define GPIO_PUPDR11_MASK              GPIO_PUPDR_MASK(11)
#define GPIO_PUPDR12_SHIFT             GPIO_PUPDR_SHIFT(12)
#define GPIO_PUPDR12_MASK              GPIO_PUPDR_MASK(12)
#define GPIO_PUPDR13_SHIFT             GPIO_PUPDR_SHIFT(13)
#define GPIO_PUPDR13_MASK              GPIO_PUPDR_MASK(13)
#define GPIO_PUPDR14_SHIFT             GPIO_PUPDR_SHIFT(14)
#define GPIO_PUPDR14_MASK              GPIO_PUPDR_MASK(14)
#define GPIO_PUPDR15_SHIFT             GPIO_PUPDR_SHIFT(15)
#define GPIO_PUPDR15_MASK              GPIO_PUPDR_MASK(15)

/* GPIO port input data register */

#define GPIO_IDR(n)                    (1 << (n))

/* GPIO port output data register */

#define GPIO_ODR(n)                    (1 << (n))

/* GPIO port bit set/reset register */

#define GPIO_BSRR_SET(n)               (1 << (n))
#define GPIO_BSRR_RESET(n)             (1 << ((n) + 16))

/* GPIO port configuration lock register */

#define GPIO_LCKR(n)                   (1 << (n))
#define GPIO_LCKK                      (1 << 16)      /* Lock key */

/* GPIO alternate function low/high register */

#define GPIO_AFR_SHIFT(n)              ((n) << 2)
#define GPIO_AFR_MASK(n)               (0xf << GPIO_AFR_SHIFT(n))

#define GPIO_AFRL0_SHIFT               (0)
#define GPIO_AFRL0_MASK                (0xf << GPIO_AFRL0_SHIFT)
#define GPIO_AFRL1_SHIFT               (4)
#define GPIO_AFRL1_MASK                (0xf << GPIO_AFRL1_SHIFT)
#define GPIO_AFRL2_SHIFT               (8)
#define GPIO_AFRL2_MASK                (0xf << GPIO_AFRL2_SHIFT)
#define GPIO_AFRL3_SHIFT               (12)
#define GPIO_AFRL3_MASK                (0xf << GPIO_AFRL3_SHIFT)
#define GPIO_AFRL4_SHIFT               (16)
#define GPIO_AFRL4_MASK                (0xf << GPIO_AFRL4_SHIFT)
#define GPIO_AFRL5_SHIFT               (20)
#define GPIO_AFRL5_MASK                (0xf << GPIO_AFRL5_SHIFT)
#define GPIO_AFRL6_SHIFT               (24)
#define GPIO_AFRL6_MASK                (0xf << GPIO_AFRL6_SHIFT)
#define GPIO_AFRL7_SHIFT               (28)
#define GPIO_AFRL7_MASK                (0xf << GPIO_AFRL7_SHIFT)

#define GPIO_AFRH8_SHIFT               (0)
#define GPIO_AFRH8_MASK                (0xf << GPIO_AFRH8_SHIFT)
#define GPIO_AFRH9_SHIFT               (4)
#define GPIO_AFRH9_MASK                (0xf << GPIO_AFRH9_SHIFT)
#define GPIO_AFRH10_SHIFT              (8)
#define GPIO_AFRH10_MASK               (0xf << GPIO_AFRH10_SHIFT)
#define GPIO_AFRH11_SHIFT              (12)
#define GPIO_AFRH11_MASK               (0xf << GPIO_AFRH11_SHIFT)
#define GPIO_AFRH12_SHIFT              (16)
#define GPIO_AFRH12_MASK               (0xf << GPIO_AFRH12_SHIFT)
#define GPIO_AFRH13_SHIFT              (20)
#define GPIO_AFRH13_MASK               (0xf << GPIO_AFRH13_SHIFT)
#define GPIO_AFRH14_SHIFT              (24)
#define GPIO_AFRH14_MASK               (0xf << GPIO_AFRH14_SHIFT)
#define GPIO_AFRH15_SHIFT              (28)
#define GPIO_AFRH15_MASK               (0xf << GPIO_AFRH15_SHIFT)

/* GPIO port bit reset register */

#define GPIO_BRR_RESET(n)              (1 << (n))

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_GPIO_H */
