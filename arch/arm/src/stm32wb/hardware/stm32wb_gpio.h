/****************************************************************************
 * arch/arm/src/stm32wb/hardware/stm32wb_gpio.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_GPIO_H
#define __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_GPIO_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32WB_GPIO_MODER_OFFSET        0x0000         /* GPIO port mode register */
#define STM32WB_GPIO_OTYPER_OFFSET       0x0004         /* GPIO port output type register */
#define STM32WB_GPIO_OSPEED_OFFSET       0x0008         /* GPIO port output speed register */
#define STM32WB_GPIO_PUPDR_OFFSET        0x000c         /* GPIO port pull-up/pull-down register */
#define STM32WB_GPIO_IDR_OFFSET          0x0010         /* GPIO port input data register */
#define STM32WB_GPIO_ODR_OFFSET          0x0014         /* GPIO port output data register */
#define STM32WB_GPIO_BSRR_OFFSET         0x0018         /* GPIO port bit set/reset register */
#define STM32WB_GPIO_LCKR_OFFSET         0x001c         /* GPIO port configuration lock register */
#define STM32WB_GPIO_AFRL_OFFSET         0x0020         /* GPIO alternate function low register */
#define STM32WB_GPIO_AFRH_OFFSET         0x0024         /* GPIO alternate function high register */
#define STM32WB_GPIO_BRR_OFFSET          0x0028         /* GPIO port bit reset register */

/* Register Addresses *******************************************************/

#define STM32WB_GPIOA_MODER              (STM32WB_GPIOA_BASE + STM32WB_GPIO_MODER_OFFSET)
#define STM32WB_GPIOA_OTYPER             (STM32WB_GPIOA_BASE + STM32WB_GPIO_OTYPER_OFFSET)
#define STM32WB_GPIOA_OSPEED             (STM32WB_GPIOA_BASE + STM32WB_GPIO_OSPEED_OFFSET)
#define STM32WB_GPIOA_PUPDR              (STM32WB_GPIOA_BASE + STM32WB_GPIO_PUPDR_OFFSET)
#define STM32WB_GPIOA_IDR                (STM32WB_GPIOA_BASE + STM32WB_GPIO_IDR_OFFSET)
#define STM32WB_GPIOA_ODR                (STM32WB_GPIOA_BASE + STM32WB_GPIO_ODR_OFFSET)
#define STM32WB_GPIOA_BSRR               (STM32WB_GPIOA_BASE + STM32WB_GPIO_BSRR_OFFSET)
#define STM32WB_GPIOA_LCKR               (STM32WB_GPIOA_BASE + STM32WB_GPIO_LCKR_OFFSET)
#define STM32WB_GPIOA_AFRL               (STM32WB_GPIOA_BASE + STM32WB_GPIO_AFRL_OFFSET)
#define STM32WB_GPIOA_AFRH               (STM32WB_GPIOA_BASE + STM32WB_GPIO_AFRH_OFFSET)
#define STM32WB_GPIOA_BRR                (STM32WB_GPIOA_BASE + STM32WB_GPIO_BRR_OFFSET)

#define STM32WB_GPIOB_MODER              (STM32WB_GPIOB_BASE + STM32WB_GPIO_MODER_OFFSET)
#define STM32WB_GPIOB_OTYPER             (STM32WB_GPIOB_BASE + STM32WB_GPIO_OTYPER_OFFSET)
#define STM32WB_GPIOB_OSPEED             (STM32WB_GPIOB_BASE + STM32WB_GPIO_OSPEED_OFFSET)
#define STM32WB_GPIOB_PUPDR              (STM32WB_GPIOB_BASE + STM32WB_GPIO_PUPDR_OFFSET)
#define STM32WB_GPIOB_IDR                (STM32WB_GPIOB_BASE + STM32WB_GPIO_IDR_OFFSET)
#define STM32WB_GPIOB_ODR                (STM32WB_GPIOB_BASE + STM32WB_GPIO_ODR_OFFSET)
#define STM32WB_GPIOB_BSRR               (STM32WB_GPIOB_BASE + STM32WB_GPIO_BSRR_OFFSET)
#define STM32WB_GPIOB_LCKR               (STM32WB_GPIOB_BASE + STM32WB_GPIO_LCKR_OFFSET)
#define STM32WB_GPIOB_AFRL               (STM32WB_GPIOB_BASE + STM32WB_GPIO_AFRL_OFFSET)
#define STM32WB_GPIOB_AFRH               (STM32WB_GPIOB_BASE + STM32WB_GPIO_AFRH_OFFSET)
#define STM32WB_GPIOB_BRR                (STM32WB_GPIOB_BASE + STM32WB_GPIO_BRR_OFFSET)

#define STM32WB_GPIOC_MODER              (STM32WB_GPIOC_BASE + STM32WB_GPIO_MODER_OFFSET)
#define STM32WB_GPIOC_OTYPER             (STM32WB_GPIOC_BASE + STM32WB_GPIO_OTYPER_OFFSET)
#define STM32WB_GPIOC_OSPEED             (STM32WB_GPIOC_BASE + STM32WB_GPIO_OSPEED_OFFSET)
#define STM32WB_GPIOC_PUPDR              (STM32WB_GPIOC_BASE + STM32WB_GPIO_PUPDR_OFFSET)
#define STM32WB_GPIOC_IDR                (STM32WB_GPIOC_BASE + STM32WB_GPIO_IDR_OFFSET)
#define STM32WB_GPIOC_ODR                (STM32WB_GPIOC_BASE + STM32WB_GPIO_ODR_OFFSET)
#define STM32WB_GPIOC_BSRR               (STM32WB_GPIOC_BASE + STM32WB_GPIO_BSRR_OFFSET)
#define STM32WB_GPIOC_LCKR               (STM32WB_GPIOC_BASE + STM32WB_GPIO_LCKR_OFFSET)
#define STM32WB_GPIOC_AFRL               (STM32WB_GPIOC_BASE + STM32WB_GPIO_AFRL_OFFSET)
#define STM32WB_GPIOC_AFRH               (STM32WB_GPIOC_BASE + STM32WB_GPIO_AFRH_OFFSET)
#define STM32WB_GPIOC_BRR                (STM32WB_GPIOC_BASE + STM32WB_GPIO_BRR_OFFSET)

#if defined(CONFIG_STM32WB_GPIO_HAVE_PORTD)
#  define STM32WB_GPIOD_MODER            (STM32WB_GPIOD_BASE + STM32WB_GPIO_MODER_OFFSET)
#  define STM32WB_GPIOD_OTYPER           (STM32WB_GPIOD_BASE + STM32WB_GPIO_OTYPER_OFFSET)
#  define STM32WB_GPIOD_OSPEED           (STM32WB_GPIOD_BASE + STM32WB_GPIO_OSPEED_OFFSET)
#  define STM32WB_GPIOD_PUPDR            (STM32WB_GPIOD_BASE + STM32WB_GPIO_PUPDR_OFFSET)
#  define STM32WB_GPIOD_IDR              (STM32WB_GPIOD_BASE + STM32WB_GPIO_IDR_OFFSET)
#  define STM32WB_GPIOD_ODR              (STM32WB_GPIOD_BASE + STM32WB_GPIO_ODR_OFFSET)
#  define STM32WB_GPIOD_BSRR             (STM32WB_GPIOD_BASE + STM32WB_GPIO_BSRR_OFFSET)
#  define STM32WB_GPIOD_LCKR             (STM32WB_GPIOD_BASE + STM32WB_GPIO_LCKR_OFFSET)
#  define STM32WB_GPIOD_AFRL             (STM32WB_GPIOD_BASE + STM32WB_GPIO_AFRL_OFFSET)
#  define STM32WB_GPIOD_AFRH             (STM32WB_GPIOD_BASE + STM32WB_GPIO_AFRH_OFFSET)
#  define STM32WB_GPIOD_BRR              (STM32WB_GPIOD_BASE + STM32WB_GPIO_BRR_OFFSET)
#endif

#if defined(CONFIG_STM32WB_GPIO_HAVE_PORTE)
#  define STM32WB_GPIOE_MODER            (STM32WB_GPIOE_BASE + STM32WB_GPIO_MODER_OFFSET)
#  define STM32WB_GPIOE_OTYPER           (STM32WB_GPIOE_BASE + STM32WB_GPIO_OTYPER_OFFSET)
#  define STM32WB_GPIOE_OSPEED           (STM32WB_GPIOE_BASE + STM32WB_GPIO_OSPEED_OFFSET)
#  define STM32WB_GPIOE_PUPDR            (STM32WB_GPIOE_BASE + STM32WB_GPIO_PUPDR_OFFSET)
#  define STM32WB_GPIOE_IDR              (STM32WB_GPIOE_BASE + STM32WB_GPIO_IDR_OFFSET)
#  define STM32WB_GPIOE_ODR              (STM32WB_GPIOE_BASE + STM32WB_GPIO_ODR_OFFSET)
#  define STM32WB_GPIOE_BSRR             (STM32WB_GPIOE_BASE + STM32WB_GPIO_BSRR_OFFSET)
#  define STM32WB_GPIOE_LCKR             (STM32WB_GPIOE_BASE + STM32WB_GPIO_LCKR_OFFSET)
#  define STM32WB_GPIOE_AFRL             (STM32WB_GPIOE_BASE + STM32WB_GPIO_AFRL_OFFSET)
#  define STM32WB_GPIOE_BRR              (STM32WB_GPIOE_BASE + STM32WB_GPIO_BRR_OFFSET)
#endif

#define STM32WB_GPIOH_MODER              (STM32WB_GPIOH_BASE + STM32WB_GPIO_MODER_OFFSET)
#define STM32WB_GPIOH_OTYPER             (STM32WB_GPIOH_BASE + STM32WB_GPIO_OTYPER_OFFSET)
#define STM32WB_GPIOH_OSPEED             (STM32WB_GPIOH_BASE + STM32WB_GPIO_OSPEED_OFFSET)
#define STM32WB_GPIOH_PUPDR              (STM32WB_GPIOH_BASE + STM32WB_GPIO_PUPDR_OFFSET)
#define STM32WB_GPIOH_IDR                (STM32WB_GPIOH_BASE + STM32WB_GPIO_IDR_OFFSET)
#define STM32WB_GPIOH_ODR                (STM32WB_GPIOH_BASE + STM32WB_GPIO_ODR_OFFSET)
#define STM32WB_GPIOH_BSRR               (STM32WB_GPIOH_BASE + STM32WB_GPIO_BSRR_OFFSET)
#define STM32WB_GPIOH_LCKR               (STM32WB_GPIOH_BASE + STM32WB_GPIO_LCKR_OFFSET)
#define STM32WB_GPIOH_AFRL               (STM32WB_GPIOH_BASE + STM32WB_GPIO_AFRL_OFFSET)
#define STM32WB_GPIOH_BRR                (STM32WB_GPIOH_BASE + STM32WB_GPIO_BRR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* GPIO port mode register */

#define GPIO_MODER_INPUT               (0x0)          /* Input mode */
#define GPIO_MODER_OUTPUT              (0x1)          /* General purpose output mode */
#define GPIO_MODER_ALT                 (0x2)          /* Alternate mode */
#define GPIO_MODER_ANALOG              (0x3)          /* Analog mode (reset state) */

#define GPIO_MODER_SHIFT(n)            ((n) << 1)
#define GPIO_MODER_MASK(n)             (0x3 << GPIO_MODER_SHIFT(n))

#define GPIO_MODER0_SHIFT              (0)
#define GPIO_MODER0_MASK               (0x3 << GPIO_MODER0_SHIFT)
#define GPIO_MODER1_SHIFT              (2)
#define GPIO_MODER1_MASK               (0x3 << GPIO_MODER1_SHIFT)
#define GPIO_MODER2_SHIFT              (4)
#define GPIO_MODER2_MASK               (0x3 << GPIO_MODER2_SHIFT)
#define GPIO_MODER3_SHIFT              (6)
#define GPIO_MODER3_MASK               (0x3 << GPIO_MODER3_SHIFT)
#define GPIO_MODER4_SHIFT              (8)
#define GPIO_MODER4_MASK               (0x3 << GPIO_MODER4_SHIFT)
#define GPIO_MODER5_SHIFT              (10)
#define GPIO_MODER5_MASK               (0x3 << GPIO_MODER5_SHIFT)
#define GPIO_MODER6_SHIFT              (12)
#define GPIO_MODER6_MASK               (0x3 << GPIO_MODER6_SHIFT)
#define GPIO_MODER7_SHIFT              (14)
#define GPIO_MODER7_MASK               (0x3 << GPIO_MODER7_SHIFT)
#define GPIO_MODER8_SHIFT              (16)
#define GPIO_MODER8_MASK               (0x3 << GPIO_MODER8_SHIFT)
#define GPIO_MODER9_SHIFT              (18)
#define GPIO_MODER9_MASK               (0x3 << GPIO_MODER9_SHIFT)
#define GPIO_MODER10_SHIFT             (20)
#define GPIO_MODER10_MASK              (0x3 << GPIO_MODER10_SHIFT)
#define GPIO_MODER11_SHIFT             (22)
#define GPIO_MODER11_MASK              (0x3 << GPIO_MODER11_SHIFT)
#define GPIO_MODER12_SHIFT             (24)
#define GPIO_MODER12_MASK              (0x3 << GPIO_MODER12_SHIFT)
#define GPIO_MODER13_SHIFT             (26)
#define GPIO_MODER13_MASK              (0x3 << GPIO_MODER13_SHIFT)
#define GPIO_MODER14_SHIFT             (28)
#define GPIO_MODER14_MASK              (0x3 << GPIO_MODER14_SHIFT)
#define GPIO_MODER15_SHIFT             (30)
#define GPIO_MODER15_MASK              (0x3 << GPIO_MODER15_SHIFT)

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

#define GPIO_OSPEED0_SHIFT             (0)
#define GPIO_OSPEED0_MASK              (0x3 << GPIO_OSPEED0_SHIFT)
#define GPIO_OSPEED1_SHIFT             (2)
#define GPIO_OSPEED1_MASK              (0x3 << GPIO_OSPEED1_SHIFT)
#define GPIO_OSPEED2_SHIFT             (4)
#define GPIO_OSPEED2_MASK              (0x3 << GPIO_OSPEED2_SHIFT)
#define GPIO_OSPEED3_SHIFT             (6)
#define GPIO_OSPEED3_MASK              (0x3 << GPIO_OSPEED3_SHIFT)
#define GPIO_OSPEED4_SHIFT             (8)
#define GPIO_OSPEED4_MASK              (0x3 << GPIO_OSPEED4_SHIFT)
#define GPIO_OSPEED5_SHIFT             (10)
#define GPIO_OSPEED5_MASK              (0x3 << GPIO_OSPEED5_SHIFT)
#define GPIO_OSPEED6_SHIFT             (12)
#define GPIO_OSPEED6_MASK              (0x3 << GPIO_OSPEED6_SHIFT)
#define GPIO_OSPEED7_SHIFT             (14)
#define GPIO_OSPEED7_MASK              (0x3 << GPIO_OSPEED7_SHIFT)
#define GPIO_OSPEED8_SHIFT             (16)
#define GPIO_OSPEED8_MASK              (0x3 << GPIO_OSPEED8_SHIFT)
#define GPIO_OSPEED9_SHIFT             (18)
#define GPIO_OSPEED9_MASK              (0x3 << GPIO_OSPEED9_SHIFT)
#define GPIO_OSPEED10_SHIFT            (20)
#define GPIO_OSPEED10_MASK             (0x3 << GPIO_OSPEED10_SHIFT)
#define GPIO_OSPEED11_SHIFT            (22)
#define GPIO_OSPEED11_MASK             (0x3 << GPIO_OSPEED11_SHIFT)
#define GPIO_OSPEED12_SHIFT            (24)
#define GPIO_OSPEED12_MASK             (0x3 << GPIO_OSPEED12_SHIFT)
#define GPIO_OSPEED13_SHIFT            (26)
#define GPIO_OSPEED13_MASK             (0x3 << GPIO_OSPEED13_SHIFT)
#define GPIO_OSPEED14_SHIFT            (28)
#define GPIO_OSPEED14_MASK             (0x3 << GPIO_OSPEED14_SHIFT)
#define GPIO_OSPEED15_SHIFT            (30)
#define GPIO_OSPEED15_MASK             (0x3 << GPIO_OSPEED15_SHIFT)

/* GPIO port pull-up/pull-down register */

#define GPIO_PUPDR_NONE                (0x0)          /* No pull-up, pull-down */
#define GPIO_PUPDR_PULLUP              (0x1)          /* Pull-up */
#define GPIO_PUPDR_PULLDOWN            (0x2)          /* Pull-down */

#define GPIO_PUPDR_SHIFT(n)            ((n) << 1)
#define GPIO_PUPDR_MASK(n)             (0x3 << GPIO_PUPDR_SHIFT(n))

#define GPIO_PUPDR0_SHIFT              (0)
#define GPIO_PUPDR0_MASK               (0x3 << GPIO_PUPDR0_SHIFT)
#define GPIO_PUPDR1_SHIFT              (2)
#define GPIO_PUPDR1_MASK               (0x3 << GPIO_PUPDR1_SHIFT)
#define GPIO_PUPDR2_SHIFT              (4)
#define GPIO_PUPDR2_MASK               (0x3 << GPIO_PUPDR2_SHIFT)
#define GPIO_PUPDR3_SHIFT              (6)
#define GPIO_PUPDR3_MASK               (0x3 << GPIO_PUPDR3_SHIFT)
#define GPIO_PUPDR4_SHIFT              (8)
#define GPIO_PUPDR4_MASK               (0x3 << GPIO_PUPDR4_SHIFT)
#define GPIO_PUPDR5_SHIFT              (10)
#define GPIO_PUPDR5_MASK               (0x3 << GPIO_PUPDR5_SHIFT)
#define GPIO_PUPDR6_SHIFT              (12)
#define GPIO_PUPDR6_MASK               (0x3 << GPIO_PUPDR6_SHIFT)
#define GPIO_PUPDR7_SHIFT              (14)
#define GPIO_PUPDR7_MASK               (0x3 << GPIO_PUPDR7_SHIFT)
#define GPIO_PUPDR8_SHIFT              (16)
#define GPIO_PUPDR8_MASK               (0x3 << GPIO_PUPDR8_SHIFT)
#define GPIO_PUPDR9_SHIFT              (18)
#define GPIO_PUPDR9_MASK               (0x3 << GPIO_PUPDR9_SHIFT)
#define GPIO_PUPDR10_SHIFT             (20)
#define GPIO_PUPDR10_MASK              (0x3 << GPIO_PUPDR10_SHIFT)
#define GPIO_PUPDR11_SHIFT             (22)
#define GPIO_PUPDR11_MASK              (0x3 << GPIO_PUPDR11_SHIFT)
#define GPIO_PUPDR12_SHIFT             (24)
#define GPIO_PUPDR12_MASK              (0x3 << GPIO_PUPDR12_SHIFT)
#define GPIO_PUPDR13_SHIFT             (26)
#define GPIO_PUPDR13_MASK              (0x3 << GPIO_PUPDR13_SHIFT)
#define GPIO_PUPDR14_SHIFT             (28)
#define GPIO_PUPDR14_MASK              (0x3 << GPIO_PUPDR14_SHIFT)
#define GPIO_PUPDR15_SHIFT             (30)
#define GPIO_PUPDR15_MASK              (0x3 << GPIO_PUPDR15_SHIFT)

/* GPIO port input data register */

#define GPIO_IDR(n)                    (1 << (n))

/* GPIO port output data register */

#define GPIO_ODR(n)                    (1 << (n))

/* GPIO port bit set/reset register */

#define GPIO_BSRR_SET(n)               (1 << (n))
#define GPIO_BSRR_RESET(n)             (1 << ((n) + 16))

/* GPIO port configuration lock register */

#define GPIO_LCKR(n)                   (1 << (n))
#define GPIO_LCKK                      (1 << 16)      /* Bit 16: Lock key */

/* GPIO alternate function low register */

#define GPIO_AFRL_AFSEL_SHIFT(n)        ((n) << 2)    /* Alt function selection for pins 0 to 7 */
#define GPIO_AFRL_AFSEL_MASK(n)         (0xf << GPIO_AFRL_AFSEL_SHIFT(n))

#define GPIO_AFRL_AFSEL0_SHIFT          (0)
#define GPIO_AFRL_AFSEL0_MASK           (0xf << GPIO_AFRL_AFSEL0_SHIFT)
#define GPIO_AFRL_AFSEL1_SHIFT          (4)
#define GPIO_AFRL_AFSEL1_MASK           (0xf << GPIO_AFRL_AFSEL1_SHIFT)
#define GPIO_AFRL_AFSEL2_SHIFT          (8)
#define GPIO_AFRL_AFSEL2_MASK           (0xf << GPIO_AFRL_AFSEL2_SHIFT)
#define GPIO_AFRL_AFSEL3_SHIFT          (12)
#define GPIO_AFRL_AFSEL3_MASK           (0xf << GPIO_AFRL_AFSEL3_SHIFT)
#define GPIO_AFRL_AFSEL4_SHIFT          (16)
#define GPIO_AFRL_AFSEL4_MASK           (0xf << GPIO_AFRL_AFSEL4_SHIFT)
#define GPIO_AFRL_AFSEL5_SHIFT          (20)
#define GPIO_AFRL_AFSEL5_MASK           (0xf << GPIO_AFRL_AFSEL5_SHIFT)
#define GPIO_AFRL_AFSEL6_SHIFT          (24)
#define GPIO_AFRL_AFSEL6_MASK           (0xf << GPIO_AFRL_AFSEL6_SHIFT)
#define GPIO_AFRL_AFSEL7_SHIFT          (28)
#define GPIO_AFRL_AFSEL7_MASK           (0xf << GPIO_AFRL_AFSEL7_SHIFT)

/* GPIO alternate function high register */

#define GPIO_AFRH_AFSEL_SHIFT(n)        (((n) - 8) << 2) /* Alt function selection for pins 8 to 15 */
#define GPIO_AFRH_AFSEL_MASK(n)         (0xf << GPIO_AFRH_AFSEL_SHIFT(n))

#define GPIO_AFRH_AFSEL8_SHIFT          (0)
#define GPIO_AFRH_AFSEL8_MASK           (0xf << GPIO_AFRH_AFSEL8_SHIFT)
#define GPIO_AFRH_AFSEL9_SHIFT          (4)
#define GPIO_AFRH_AFSEL9_MASK           (0xf << GPIO_AFRH_AFSEL9_SHIFT)
#define GPIO_AFRH_AFSEL10_SHIFT         (8)
#define GPIO_AFRH_AFSEL10_MASK          (0xf << GPIO_AFRH_AFSEL10_SHIFT)
#define GPIO_AFRH_AFSEL11_SHIFT         (12)
#define GPIO_AFRH_AFSEL11_MASK          (0xf << GPIO_AFRH_AFSEL11_SHIFT)
#define GPIO_AFRH_AFSEL12_SHIFT         (16)
#define GPIO_AFRH_AFSEL12_MASK          (0xf << GPIO_AFRH_AFSEL12_SHIFT)
#define GPIO_AFRH_AFSEL13_SHIFT         (20)
#define GPIO_AFRH_AFSEL13_MASK          (0xf << GPIO_AFRH_AFSEL13_SHIFT)
#define GPIO_AFRH_AFSEL14_SHIFT         (24)
#define GPIO_AFRH_AFSEL14_MASK          (0xf << GPIO_AFRH_AFSEL14_SHIFT)
#define GPIO_AFRH_AFSEL15_SHIFT         (28)
#define GPIO_AFRH_AFSEL15_MASK          (0xf << GPIO_AFRH_AFSEL15_SHIFT)

/* GPIO port bit reset register */

#define GPIO_BRR_RESET(n)               (1 << (n))

#endif /* __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_GPIO_H */
