/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32f30xxx_gpio.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32F30XXX_GPIO_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32F30XXX_GPIO_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Alther the number of GPIO pins varies from 37 to 87, there are always 6
 * GPIO ports in the STM32 F302/F303 parts.
 */

#define STM32_NGPIO_PORTS          (6)

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

/* Register Addresses *******************************************************/

#define STM32_GPIOA_MODER         (STM32_GPIOA_BASE+STM32_GPIO_MODER_OFFSET)
#define STM32_GPIOA_OTYPER        (STM32_GPIOA_BASE+STM32_GPIO_OTYPER_OFFSET)
#define STM32_GPIOA_OSPEED        (STM32_GPIOA_BASE+STM32_GPIO_OSPEED_OFFSET)
#define STM32_GPIOA_PUPDR         (STM32_GPIOA_BASE+STM32_GPIO_PUPDR_OFFSET)
#define STM32_GPIOA_IDR           (STM32_GPIOA_BASE+STM32_GPIO_IDR_OFFSET)
#define STM32_GPIOA_ODR           (STM32_GPIOA_BASE+STM32_GPIO_ODR_OFFSET)
#define STM32_GPIOA_BSRR          (STM32_GPIOA_BASE+STM32_GPIO_BSRR_OFFSET)
#define STM32_GPIOA_LCKR          (STM32_GPIOA_BASE+STM32_GPIO_LCKR_OFFSET)
#define STM32_GPIOA_AFRL          (STM32_GPIOA_BASE+STM32_GPIO_AFRL_OFFSET)
#define STM32_GPIOA_AFRH          (STM32_GPIOA_BASE+STM32_GPIO_AFRH_OFFSET)
#define STM32_GPIOA_BRR           (STM32_GPIOA_BASE+STM32_GPIO_BRR_OFFSET)

#define STM32_GPIOB_MODER         (STM32_GPIOB_BASE+STM32_GPIO_MODER_OFFSET)
#define STM32_GPIOB_OTYPER        (STM32_GPIOB_BASE+STM32_GPIO_OTYPER_OFFSET)
#define STM32_GPIOB_OSPEED        (STM32_GPIOB_BASE+STM32_GPIO_OSPEED_OFFSET)
#define STM32_GPIOB_PUPDR         (STM32_GPIOB_BASE+STM32_GPIO_PUPDR_OFFSET)
#define STM32_GPIOB_IDR           (STM32_GPIOB_BASE+STM32_GPIO_IDR_OFFSET)
#define STM32_GPIOB_ODR           (STM32_GPIOB_BASE+STM32_GPIO_ODR_OFFSET)
#define STM32_GPIOB_BSRR          (STM32_GPIOB_BASE+STM32_GPIO_BSRR_OFFSET)
#define STM32_GPIOB_LCKR          (STM32_GPIOB_BASE+STM32_GPIO_LCKR_OFFSET)
#define STM32_GPIOB_AFRL          (STM32_GPIOB_BASE+STM32_GPIO_AFRL_OFFSET)
#define STM32_GPIOB_AFRH          (STM32_GPIOB_BASE+STM32_GPIO_AFRH_OFFSET)
#define STM32_GPIOB_BRR           (STM32_GPIOB_BASE+STM32_GPIO_BRR_OFFSET)

#define STM32_GPIOC_MODER         (STM32_GPIOC_BASE+STM32_GPIO_MODER_OFFSET)
#define STM32_GPIOC_OTYPER        (STM32_GPIOC_BASE+STM32_GPIO_OTYPER_OFFSET)
#define STM32_GPIOC_OSPEED        (STM32_GPIOC_BASE+STM32_GPIO_OSPEED_OFFSET)
#define STM32_GPIOC_PUPDR         (STM32_GPIOC_BASE+STM32_GPIO_PUPDR_OFFSET)
#define STM32_GPIOC_IDR           (STM32_GPIOC_BASE+STM32_GPIO_IDR_OFFSET)
#define STM32_GPIOC_ODR           (STM32_GPIOC_BASE+STM32_GPIO_ODR_OFFSET)
#define STM32_GPIOC_BSRR          (STM32_GPIOC_BASE+STM32_GPIO_BSRR_OFFSET)
#define STM32_GPIOC_LCKR          (STM32_GPIOC_BASE+STM32_GPIO_LCKR_OFFSET)
#define STM32_GPIOC_AFRL          (STM32_GPIOC_BASE+STM32_GPIO_AFRL_OFFSET)
#define STM32_GPIOC_AFRH          (STM32_GPIOC_BASE+STM32_GPIO_AFRH_OFFSET)
#define STM32_GPIOC_BRR           (STM32_GPIOC_BASE+STM32_GPIO_BRR_OFFSET)

#define STM32_GPIOD_MODER         (STM32_GPIOD_BASE+STM32_GPIO_MODER_OFFSET)
#define STM32_GPIOD_OTYPER        (STM32_GPIOD_BASE+STM32_GPIO_OTYPER_OFFSET)
#define STM32_GPIOD_OSPEED        (STM32_GPIOD_BASE+STM32_GPIO_OSPEED_OFFSET)
#define STM32_GPIOD_PUPDR         (STM32_GPIOD_BASE+STM32_GPIO_PUPDR_OFFSET)
#define STM32_GPIOD_IDR           (STM32_GPIOD_BASE+STM32_GPIO_IDR_OFFSET)
#define STM32_GPIOD_ODR           (STM32_GPIOD_BASE+STM32_GPIO_ODR_OFFSET)
#define STM32_GPIOD_BSRR          (STM32_GPIOD_BASE+STM32_GPIO_BSRR_OFFSET)
#define STM32_GPIOD_LCKR          (STM32_GPIOD_BASE+STM32_GPIO_LCKR_OFFSET)
#define STM32_GPIOD_AFRL          (STM32_GPIOD_BASE+STM32_GPIO_AFRL_OFFSET)
#define STM32_GPIOD_AFRH          (STM32_GPIOD_BASE+STM32_GPIO_AFRH_OFFSET)
#define STM32_GPIOD_BRR           (STM32_GPIOD_BASE+STM32_GPIO_BRR_OFFSET)

#define STM32_GPIOE_MODER         (STM32_GPIOE_BASE+STM32_GPIO_MODER_OFFSET)
#define STM32_GPIOE_OTYPER        (STM32_GPIOE_BASE+STM32_GPIO_OTYPER_OFFSET)
#define STM32_GPIOE_OSPEED        (STM32_GPIOE_BASE+STM32_GPIO_OSPEED_OFFSET)
#define STM32_GPIOE_PUPDR         (STM32_GPIOE_BASE+STM32_GPIO_PUPDR_OFFSET)
#define STM32_GPIOE_IDR           (STM32_GPIOE_BASE+STM32_GPIO_IDR_OFFSET)
#define STM32_GPIOE_ODR           (STM32_GPIOE_BASE+STM32_GPIO_ODR_OFFSET)
#define STM32_GPIOE_BSRR          (STM32_GPIOE_BASE+STM32_GPIO_BSRR_OFFSET)
#define STM32_GPIOE_LCKR          (STM32_GPIOE_BASE+STM32_GPIO_LCKR_OFFSET)
#define STM32_GPIOE_AFRL          (STM32_GPIOE_BASE+STM32_GPIO_AFRL_OFFSET)
#define STM32_GPIOE_AFRH          (STM32_GPIOE_BASE+STM32_GPIO_AFRH_OFFSET)
#define STM32_GPIOE_BRR           (STM32_GPIOE_BASE+STM32_GPIO_BRR_OFFSET)

#define STM32_GPIOF_MODER         (STM32_GPIOF_BASE+STM32_GPIO_MODER_OFFSET)
#define STM32_GPIOF_OTYPER        (STM32_GPIOF_BASE+STM32_GPIO_OTYPER_OFFSET)
#define STM32_GPIOF_OSPEED        (STM32_GPIOF_BASE+STM32_GPIO_OSPEED_OFFSET)
#define STM32_GPIOF_PUPDR         (STM32_GPIOF_BASE+STM32_GPIO_PUPDR_OFFSET)
#define STM32_GPIOF_IDR           (STM32_GPIOF_BASE+STM32_GPIO_IDR_OFFSET)
#define STM32_GPIOF_ODR           (STM32_GPIOF_BASE+STM32_GPIO_ODR_OFFSET)
#define STM32_GPIOF_BSRR          (STM32_GPIOF_BASE+STM32_GPIO_BSRR_OFFSET)
#define STM32_GPIOF_LCKR          (STM32_GPIOF_BASE+STM32_GPIO_LCKR_OFFSET)
#define STM32_GPIOF_AFRL          (STM32_GPIOF_BASE+STM32_GPIO_AFRL_OFFSET)
#define STM32_GPIOF_AFRH          (STM32_GPIOF_BASE+STM32_GPIO_AFRH_OFFSET)
#define STM32_GPIOF_BRR           (STM32_GPIOF_BASE+STM32_GPIO_BRR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* GPIO port mode register */

#define GPIO_MODER_INPUT           (0) /* Input */
#define GPIO_MODER_OUTPUT          (1) /* General purpose output mode */
#define GPIO_MODER_ALT             (2) /* Alternate mode */
#define GPIO_MODER_ANALOG          (3) /* Analog mode */

#define GPIO_MODER_SHIFT(n)        ((n) << 1)
#define GPIO_MODER_MASK(n)         (3 << GPIO_MODER_SHIFT(n))

#define GPIO_MODER0_SHIFT          (0)
#define GPIO_MODER0_MASK           (3 << GPIO_MODER0_SHIFT)
#define GPIO_MODER1_SHIFT          (2)
#define GPIO_MODER1_MASK           (3 << GPIO_MODER1_SHIFT)
#define GPIO_MODER2_SHIFT          (4)
#define GPIO_MODER2_MASK           (3 << GPIO_MODER2_SHIFT)
#define GPIO_MODER3_SHIFT          (6)
#define GPIO_MODER3_MASK           (3 << GPIO_MODER3_SHIFT)
#define GPIO_MODER4_SHIFT          (8)
#define GPIO_MODER4_MASK           (3 << GPIO_MODER4_SHIFT)
#define GPIO_MODER5_SHIFT          (10)
#define GPIO_MODER5_MASK           (3 << GPIO_MODER5_SHIFT)
#define GPIO_MODER6_SHIFT          (12)
#define GPIO_MODER6_MASK           (3 << GPIO_MODER6_SHIFT)
#define GPIO_MODER7_SHIFT          (14)
#define GPIO_MODER7_MASK           (3 << GPIO_MODER7_SHIFT)
#define GPIO_MODER8_SHIFT          (16)
#define GPIO_MODER8_MASK           (3 << GPIO_MODER8_SHIFT)
#define GPIO_MODER9_SHIFT          (18)
#define GPIO_MODER9_MASK           (3 << GPIO_MODER9_SHIFT)
#define GPIO_MODER10_SHIFT         (20)
#define GPIO_MODER10_MASK          (3 << GPIO_MODER10_SHIFT)
#define GPIO_MODER11_SHIFT         (22)
#define GPIO_MODER11_MASK          (3 << GPIO_MODER11_SHIFT)
#define GPIO_MODER12_SHIFT         (24)
#define GPIO_MODER12_MASK          (3 << GPIO_MODER12_SHIFT)
#define GPIO_MODER13_SHIFT         (26)
#define GPIO_MODER13_MASK          (3 << GPIO_MODER13_SHIFT)
#define GPIO_MODER14_SHIFT         (28)
#define GPIO_MODER14_MASK          (3 << GPIO_MODER14_SHIFT)
#define GPIO_MODER15_SHIFT         (30)
#define GPIO_MODER15_MASK          (3 << GPIO_MODER15_SHIFT)

/* GPIO port output type register */

#define GPIO_OTYPER_OD(n)          (1 << (n)) /* 1=Output open-drain */
#define GPIO_OTYPER_PP(n)          (0)        /* 0=OuTput push-pull */

/* GPIO port output speed register */

#define GPIO_OSPEED_2MHz           (0) /* 2 MHz Low speed */
#define GPIO_OSPEED_25MHz          (1) /* 25 MHz Medium speed */
#define GPIO_OSPEED_50MHz          (3) /* 50 MHz High speed */

#define GPIO_OSPEED_SHIFT(n)       ((n) << 1)
#define GPIO_OSPEED_MASK(n)        (3 << GPIO_OSPEED_SHIFT(n))

#define GPIO_OSPEED0_SHIFT         (0)
#define GPIO_OSPEED0_MASK          (3 << GPIO_OSPEED0_SHIFT)
#define GPIO_OSPEED1_SHIFT         (2)
#define GPIO_OSPEED1_MASK          (3 << GPIO_OSPEED1_SHIFT)
#define GPIO_OSPEED2_SHIFT         (4)
#define GPIO_OSPEED2_MASK          (3 << GPIO_OSPEED2_SHIFT)
#define GPIO_OSPEED3_SHIFT         (6)
#define GPIO_OSPEED3_MASK          (3 << GPIO_OSPEED3_SHIFT)
#define GPIO_OSPEED4_SHIFT         (8)
#define GPIO_OSPEED4_MASK          (3 << GPIO_OSPEED4_SHIFT)
#define GPIO_OSPEED5_SHIFT         (10)
#define GPIO_OSPEED5_MASK          (3 << GPIO_OSPEED5_SHIFT)
#define GPIO_OSPEED6_SHIFT         (12)
#define GPIO_OSPEED6_MASK          (3 << GPIO_OSPEED6_SHIFT)
#define GPIO_OSPEED7_SHIFT         (14)
#define GPIO_OSPEED7_MASK          (3 << GPIO_OSPEED7_SHIFT)
#define GPIO_OSPEED8_SHIFT         (16)
#define GPIO_OSPEED8_MASK          (3 << GPIO_OSPEED8_SHIFT)
#define GPIO_OSPEED9_SHIFT         (18)
#define GPIO_OSPEED9_MASK          (3 << GPIO_OSPEED9_SHIFT)
#define GPIO_OSPEED10_SHIFT        (20)
#define GPIO_OSPEED10_MASK         (3 << GPIO_OSPEED10_SHIFT)
#define GPIO_OSPEED11_SHIFT        (22)
#define GPIO_OSPEED11_MASK         (3 << GPIO_OSPEED11_SHIFT)
#define GPIO_OSPEED12_SHIFT        (24)
#define GPIO_OSPEED12_MASK         (3 << GPIO_OSPEED12_SHIFT)
#define GPIO_OSPEED13_SHIFT        (26)
#define GPIO_OSPEED13_MASK         (3 << GPIO_OSPEED13_SHIFT)
#define GPIO_OSPEED14_SHIFT        (28)
#define GPIO_OSPEED14_MASK         (3 << GPIO_OSPEED14_SHIFT)
#define GPIO_OSPEED15_SHIFT        (30)
#define GPIO_OSPEED15_MASK         (3 << GPIO_OSPEED15_SHIFT)

/* GPIO port pull-up/pull-down register */

#define GPIO_PUPDR_NONE            (0) /* No pull-up, pull-down */
#define GPIO_PUPDR_PULLUP          (1) /* Pull-up */
#define GPIO_PUPDR_PULLDOWN        (2) /* Pull-down */

#define GPIO_PUPDR_SHIFT(n)        ((n) << 1)
#define GPIO_PUPDR_MASK(n)         (3 << GPIO_PUPDR_SHIFT(n))

#define GPIO_PUPDR0_SHIFT          (0)
#define GPIO_PUPDR0_MASK           (3 << GPIO_PUPDR0_SHIFT)
#define GPIO_PUPDR1_SHIFT          (2)
#define GPIO_PUPDR1_MASK           (3 << GPIO_PUPDR1_SHIFT)
#define GPIO_PUPDR2_SHIFT          (4)
#define GPIO_PUPDR2_MASK           (3 << GPIO_PUPDR2_SHIFT)
#define GPIO_PUPDR3_SHIFT          (6)
#define GPIO_PUPDR3_MASK           (3 << GPIO_PUPDR3_SHIFT)
#define GPIO_PUPDR4_SHIFT          (8)
#define GPIO_PUPDR4_MASK           (3 << GPIO_PUPDR4_SHIFT)
#define GPIO_PUPDR5_SHIFT          (10)
#define GPIO_PUPDR5_MASK           (3 << GPIO_PUPDR5_SHIFT)
#define GPIO_PUPDR6_SHIFT          (12)
#define GPIO_PUPDR6_MASK           (3 << GPIO_PUPDR6_SHIFT)
#define GPIO_PUPDR7_SHIFT          (14)
#define GPIO_PUPDR7_MASK           (3 << GPIO_PUPDR7_SHIFT)
#define GPIO_PUPDR8_SHIFT          (16)
#define GPIO_PUPDR8_MASK           (3 << GPIO_PUPDR8_SHIFT)
#define GPIO_PUPDR9_SHIFT          (18)
#define GPIO_PUPDR9_MASK           (3 << GPIO_PUPDR9_SHIFT)
#define GPIO_PUPDR10_SHIFT         (20)
#define GPIO_PUPDR10_MASK          (3 << GPIO_PUPDR10_SHIFT)
#define GPIO_PUPDR11_SHIFT         (22)
#define GPIO_PUPDR11_MASK          (3 << GPIO_PUPDR11_SHIFT)
#define GPIO_PUPDR12_SHIFT         (24)
#define GPIO_PUPDR12_MASK          (3 << GPIO_PUPDR12_SHIFT)
#define GPIO_PUPDR13_SHIFT         (26)
#define GPIO_PUPDR13_MASK          (3 << GPIO_PUPDR13_SHIFT)
#define GPIO_PUPDR14_SHIFT         (28)
#define GPIO_PUPDR14_MASK          (3 << GPIO_PUPDR14_SHIFT)
#define GPIO_PUPDR15_SHIFT         (30)
#define GPIO_PUPDR15_MASK          (3 << GPIO_PUPDR15_SHIFT)

/* GPIO port input data register */

#define GPIO_IDR(n)                (1 << (n))

/* GPIO port output data register */

#define GPIO_ODR(n)                (1 << (n))

/* GPIO port bit set/reset register */

#define GPIO_BSRR_SET(n)           (1 << (n))
#define GPIO_BSRR_RESET(n)         (1 << ((n)+16))

/* GPIO port configuration lock register */

#define GPIO_LCKR(n)               (1 << (n))
#define GPIO_LCKK                  (1 << 16)   /* Lock key */

/* GPIO alternate function low/high register */

#define GPIO_AFR_SHIFT(n)          ((n) << 2)
#define GPIO_AFR_MASK(n)           (15 << GPIO_AFR_SHIFT(n))

#define GPIO_AFRL0_SHIFT           (0)
#define GPIO_AFRL0_MASK            (15 << GPIO_AFRL0_SHIFT)
#define GPIO_AFRL1_SHIFT           (4)
#define GPIO_AFRL1_MASK            (15 << GPIO_AFRL1_SHIFT)
#define GPIO_AFRL2_SHIFT           (8)
#define GPIO_AFRL2_MASK            (15 << GPIO_AFRL2_SHIFT)
#define GPIO_AFRL3_SHIFT           (12)
#define GPIO_AFRL3_MASK            (15 << GPIO_AFRL3_SHIFT)
#define GPIO_AFRL4_SHIFT           (16)
#define GPIO_AFRL4_MASK            (15 << GPIO_AFRL4_SHIFT)
#define GPIO_AFRL5_SHIFT           (20)
#define GPIO_AFRL5_MASK            (15 << GPIO_AFRL5_SHIFT)
#define GPIO_AFRL6_SHIFT           (24)
#define GPIO_AFRL6_MASK            (15 << GPIO_AFRL6_SHIFT)
#define GPIO_AFRL7_SHIFT           (28)
#define GPIO_AFRL7_MASK            (15 << GPIO_AFRL7_SHIFT)

#define GPIO_AFRH8_SHIFT           (0)
#define GPIO_AFRH8_MASK            (15 << GPIO_AFRH8_SHIFT)
#define GPIO_AFRH9_SHIFT           (4)
#define GPIO_AFRH9_MASK            (15 << GPIO_AFRH9_SHIFT)
#define GPIO_AFRH10_SHIFT          (8)
#define GPIO_AFRH10_MASK           (15 << GPIO_AFRH10_SHIFT)
#define GPIO_AFRH11_SHIFT          (12)
#define GPIO_AFRH11_MASK           (15 << GPIO_AFRH11_SHIFT)
#define GPIO_AFRH12_SHIFT          (16)
#define GPIO_AFRH12_MASK           (15 << GPIO_AFRH12_SHIFT)
#define GPIO_AFRH13_SHIFT          (20)
#define GPIO_AFRH13_MASK           (15 << GPIO_AFRH13_SHIFT)
#define GPIO_AFRH14_SHIFT          (24)
#define GPIO_AFRH14_MASK           (15 << GPIO_AFRH14_SHIFT)
#define GPIO_AFRH15_SHIFT          (28)
#define GPIO_AFRH15_MASK           (15 << GPIO_AFRH15_SHIFT)

/* GPIO port bit reset register */

#define GPIO_BRR(n)                (1 << (n))

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32F30XXX_GPIO_H */
