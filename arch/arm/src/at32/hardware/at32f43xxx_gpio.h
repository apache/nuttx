/****************************************************************************
 * arch/arm/src/at32/hardware/at32f43xxx_gpio.h
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

#ifndef __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_GPIO_H
#define __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_GPIO_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AT32_NGPIO_PORTS          ((AT32_NGPIO + 15) >> 4)

/* Register Offsets *********************************************************/

#define AT32_GPIO_CFGR_OFFSET           (0x00) /* GPIO config register */
#define AT32_GPIO_OMODER_OFFSET         (0x04) /* GPIO output mode register */
#define AT32_GPIO_ODRVR_OFFSET          (0x08) /* GPIO drive capability */
#define AT32_GPIO_PULL_OFFSET           (0x0C) /* GPIO pull config register */
#define AT32_GPIO_IDT_OFFSET            (0x10) /* GPIO input data register */
#define AT32_GPIO_ODT_OFFSET            (0x14) /* GPIO output data register */
#define AT32_GPIO_SCR_OFFSET            (0x18) /* GPIO clear/set bit register */
#define AT32_GPIO_WPR_OFFSET            (0x1C) /* GPIO write protect register */
#define AT32_GPIO_MUXL_OFFSET           (0x20) /* GPIO muxing register (pin0~7) */
#define AT32_GPIO_MUXH_OFFSET           (0x24) /* GPIO muxing register (pin8~15)*/
#define AT32_GPIO_CLR_OFFSET            (0x28) /* GPIO clear bit */
#define AT32_GPIO_HDRV_OFFSET           (0x3C) /* GPIO high driver */

/* Register Addresses *******************************************************/

#if AT32_NGPIO_PORTS > 0
#  define AT32_GPIOA_CFGR               (AT32_GPIOA_BASE+AT32_GPIO_CFGR_OFFSET)
#  define AT32_GPIOA_OMODER             (AT32_GPIOA_BASE+AT32_GPIO_OMODER_OFFSET)
#  define AT32_GPIOA_ODRVR              (AT32_GPIOA_BASE+AT32_GPIO_ODRVR_OFFSET)
#  define AT32_GPIOA_PULL               (AT32_GPIOA_BASE+AT32_GPIO_PULL_OFFSET)
#  define AT32_GPIOA_IDT                (AT32_GPIOA_BASE+AT32_GPIO_IDT_OFFSET)
#  define AT32_GPIOA_ODT                (AT32_GPIOA_BASE+AT32_GPIO_ODT_OFFSET)
#  define AT32_GPIOA_SCR                (AT32_GPIOA_BASE+AT32_GPIO_SCR_OFFSET)
#  define AT32_GPIOA_WPR                (AT32_GPIOA_BASE+AT32_GPIO_WPR_OFFSET)
#  define AT32_GPIOA_MUXL               (AT32_GPIOA_BASE+AT32_GPIO_MUXL_OFFSET)
#  define AT32_GPIOA_MUXH               (AT32_GPIOA_BASE+AT32_GPIO_MUXH_OFFSET)
#  define AT32_GPIOA_CLR                (AT32_GPIOA_BASE+AT32_GPIO_CLR_OFFSET)
#  define AT32_GPIOA_HDRV               (AT32_GPIOA_BASE+AT32_GPIO_HDRV_OFFSET)
#endif

#if AT32_NGPIO_PORTS > 1
#  define AT32_GPIOB_CFGR               (AT32_GPIOB_BASE+AT32_GPIO_CFGR_OFFSET)
#  define AT32_GPIOB_OMODER             (AT32_GPIOB_BASE+AT32_GPIO_OMODER_OFFSET)
#  define AT32_GPIOB_ODRVR              (AT32_GPIOB_BASE+AT32_GPIO_ODRVR_OFFSET)
#  define AT32_GPIOB_PULL               (AT32_GPIOB_BASE+AT32_GPIO_PULL_OFFSET)
#  define AT32_GPIOB_IDT                (AT32_GPIOB_BASE+AT32_GPIO_IDT_OFFSET)
#  define AT32_GPIOB_ODT                (AT32_GPIOB_BASE+AT32_GPIO_ODT_OFFSET)
#  define AT32_GPIOB_SCR                (AT32_GPIOB_BASE+AT32_GPIO_SCR_OFFSET)
#  define AT32_GPIOB_WPR                (AT32_GPIOB_BASE+AT32_GPIO_WPR_OFFSET)
#  define AT32_GPIOB_MUXL               (AT32_GPIOB_BASE+AT32_GPIO_MUXL_OFFSET)
#  define AT32_GPIOB_MUXH               (AT32_GPIOB_BASE+AT32_GPIO_MUXH_OFFSET)
#  define AT32_GPIOB_CLR                (AT32_GPIOB_BASE+AT32_GPIO_CLR_OFFSET)
#  define AT32_GPIOB_HDRV               (AT32_GPIOB_BASE+AT32_GPIO_HDRV_OFFSET)
#endif

#if AT32_NGPIO_PORTS > 2
#  define AT32_GPIOC_CFGR               (AT32_GPIOC_BASE+AT32_GPIO_CFGR_OFFSET)
#  define AT32_GPIOC_OMODER             (AT32_GPIOC_BASE+AT32_GPIO_OMODER_OFFSET)
#  define AT32_GPIOC_ODRVR              (AT32_GPIOC_BASE+AT32_GPIO_ODRVR_OFFSET)
#  define AT32_GPIOC_PULL               (AT32_GPIOC_BASE+AT32_GPIO_PULL_OFFSET)
#  define AT32_GPIOC_IDT                (AT32_GPIOC_BASE+AT32_GPIO_IDT_OFFSET)
#  define AT32_GPIOC_ODT                (AT32_GPIOC_BASE+AT32_GPIO_ODT_OFFSET)
#  define AT32_GPIOC_SCR                (AT32_GPIOC_BASE+AT32_GPIO_SCR_OFFSET)
#  define AT32_GPIOC_WPR                (AT32_GPIOC_BASE+AT32_GPIO_WPR_OFFSET)
#  define AT32_GPIOC_MUXL               (AT32_GPIOC_BASE+AT32_GPIO_MUXL_OFFSET)
#  define AT32_GPIOC_MUXH               (AT32_GPIOC_BASE+AT32_GPIO_MUXH_OFFSET)
#  define AT32_GPIOC_CLR                (AT32_GPIOC_BASE+AT32_GPIO_CLR_OFFSET)
#  define AT32_GPIOC_HDRV               (AT32_GPIOC_BASE+AT32_GPIO_HDRV_OFFSET)
#endif

#if AT32_NGPIO_PORTS > 3
#  define AT32_GPIOD_CFGR               (AT32_GPIOD_BASE+AT32_GPIO_CFGR_OFFSET)
#  define AT32_GPIOD_OMODER             (AT32_GPIOD_BASE+AT32_GPIO_OMODER_OFFSET)
#  define AT32_GPIOD_ODRVR              (AT32_GPIOD_BASE+AT32_GPIO_ODRVR_OFFSET)
#  define AT32_GPIOD_PULL               (AT32_GPIOD_BASE+AT32_GPIO_PULL_OFFSET)
#  define AT32_GPIOD_IDT                (AT32_GPIOD_BASE+AT32_GPIO_IDT_OFFSET)
#  define AT32_GPIOD_ODT                (AT32_GPIOD_BASE+AT32_GPIO_ODT_OFFSET)
#  define AT32_GPIOD_SCR                (AT32_GPIOD_BASE+AT32_GPIO_SCR_OFFSET)
#  define AT32_GPIOD_WPR                (AT32_GPIOD_BASE+AT32_GPIO_WPR_OFFSET)
#  define AT32_GPIOD_MUXL               (AT32_GPIOD_BASE+AT32_GPIO_MUXL_OFFSET)
#  define AT32_GPIOD_MUXH               (AT32_GPIOD_BASE+AT32_GPIO_MUXH_OFFSET)
#  define AT32_GPIOD_CLR                (AT32_GPIOD_BASE+AT32_GPIO_CLR_OFFSET)
#  define AT32_GPIOD_HDRV               (AT32_GPIOD_BASE+AT32_GPIO_HDRV_OFFSET)
#endif

#if AT32_NGPIO_PORTS > 4
#  define AT32_GPIOE_CFGR               (AT32_GPIOE_BASE+AT32_GPIO_CFGR_OFFSET)
#  define AT32_GPIOE_OMODER             (AT32_GPIOE_BASE+AT32_GPIO_OMODER_OFFSET)
#  define AT32_GPIOE_ODRVR              (AT32_GPIOE_BASE+AT32_GPIO_ODRVR_OFFSET)
#  define AT32_GPIOE_PULL               (AT32_GPIOE_BASE+AT32_GPIO_PULL_OFFSET)
#  define AT32_GPIOE_IDT                (AT32_GPIO_BASE+AT32_GPIO_IDT_OFFSET)
#  define AT32_GPIOE_ODT                (AT32_GPIOE_BASE+AT32_GPIO_ODT_OFFSET)
#  define AT32_GPIOE_SCR                (AT32_GPIOE_BASE+AT32_GPIO_SCR_OFFSET)
#  define AT32_GPIOE_WPR                (AT32_GPIOE_BASE+AT32_GPIO_WPR_OFFSET)
#  define AT32_GPIOE_MUXL               (AT32_GPIOE_BASE+AT32_GPIO_MUXL_OFFSET)
#  define AT32_GPIOE_MUXH               (AT32_GPIOE_BASE+AT32_GPIO_MUXH_OFFSET)
#  define AT32_GPIOE_CLR                (AT32_GPIOE_BASE+AT32_GPIO_CLR_OFFSET)
#  define AT32_GPIOE_HDRV               (AT32_GPIOE_BASE+AT32_GPIO_HDRV_OFFSET)
#endif

#if AT32_NGPIO_PORTS > 5
#  define AT32_GPIOF_CFGR               (AT32_GPIOF_BASE+AT32_GPIO_CFGR_OFFSET)
#  define AT32_GPIOF_OMODER             (AT32_GPIOF_BASE+AT32_GPIO_OMODER_OFFSET)
#  define AT32_GPIOF_ODRVR              (AT32_GPIOF_BASE+AT32_GPIO_ODRVR_OFFSET)
#  define AT32_GPIOF_PULL               (AT32_GPIOF_BASE+AT32_GPIO_PULL_OFFSET)
#  define AT32_GPIOF_IDT                (AT32_GPIOF_BASE+AT32_GPIO_IDT_OFFSET)
#  define AT32_GPIOF_ODT                (AT32_GPIOF_BASE+AT32_GPIO_ODT_OFFSET)
#  define AT32_GPIOF_SCR                (AT32_GPIOF_BASE+AT32_GPIO_SCR_OFFSET)
#  define AT32_GPIOF_WPR                (AT32_GPIOF_BASE+AT32_GPIO_WPR_OFFSET)
#  define AT32_GPIOF_MUXL               (AT32_GPIOF_BASE+AT32_GPIO_MUXL_OFFSET)
#  define AT32_GPIOF_MUXH               (AT32_GPIOF_BASE+AT32_GPIO_MUXH_OFFSET)
#  define AT32_GPIOF_CLR                (AT32_GPIOF_BASE+AT32_GPIO_CLR_OFFSET)
#  define AT32_GPIOF_HDRV               (AT32_GPIOF_BASE+AT32_GPIO_HDRV_OFFSET)
#endif

#if AT32_NGPIO_PORTS > 6
#  define AT32_GPIOG_CFGR               (AT32_GPIOG_BASE+AT32_GPIO_CFGR_OFFSET)
#  define AT32_GPIOG_OMODER             (AT32_GPIOG_BASE+AT32_GPIO_OMODER_OFFSET)
#  define AT32_GPIOG_ODRVR              (AT32_GPIOG_BASE+AT32_GPIO_ODRVR_OFFSET)
#  define AT32_GPIOG_PULL               (AT32_GPIOG_BASE+AT32_GPIO_PULL_OFFSET)
#  define AT32_GPIOG_IDT                (AT32_GPIOG_BASE+AT32_GPIO_IDT_OFFSET)
#  define AT32_GPIOG_ODT                (AT32_GPIOG_BASE+AT32_GPIO_ODT_OFFSET)
#  define AT32_GPIOG_SCR                (AT32_GPIOG_BASE+AT32_GPIO_SCR_OFFSET)
#  define AT32_GPIOG_WPR                (AT32_GPIOG_BASE+AT32_GPIO_WPR_OFFSET)
#  define AT32_GPIOG_MUXL               (AT32_GPIOG_BASE+AT32_GPIO_MUXL_OFFSET)
#  define AT32_GPIOG_MUXH               (AT32_GPIOG_BASE+AT32_GPIO_MUXH_OFFSET)
#  define AT32_GPIOG_CLR                (AT32_GPIOG_BASE+AT32_GPIO_CLR_OFFSET)
#  define AT32_GPIOG_HDRV               (AT32_GPIOG_BASE+AT32_GPIO_HDRV_OFFSET)
#endif

#if AT32_NGPIO_PORTS > 7
#  define AT32_GPIOH_CFGR               (AT32_GPIOH_BASE+AT32_GPIO_CFGR_OFFSET)
#  define AT32_GPIOH_OMODER             (AT32_GPIOH_BASE+AT32_GPIO_OMODER_OFFSET)
#  define AT32_GPIOH_ODRVR              (AT32_GPIOH_BASE+AT32_GPIO_ODRVR_OFFSET)
#  define AT32_GPIOH_PULL               (AT32_GPIOH_BASE+AT32_GPIO_PULL_OFFSET)
#  define AT32_GPIOH_IDT                (AT32_GPIOH_BASE+AT32_GPIO_IDT_OFFSET)
#  define AT32_GPIOH_ODT                (AT32_GPIOH_BASE+AT32_GPIO_ODT_OFFSET)
#  define AT32_GPIOH_SCR                (AT32_GPIOH_BASE+AT32_GPIO_SCR_OFFSET)
#  define AT32_GPIOH_WPR                (AT32_GPIOH_BASE+AT32_GPIO_WPR_OFFSET)
#  define AT32_GPIOH_MUXL               (AT32_GPIOH_BASE+AT32_GPIO_MUXL_OFFSET)
#  define AT32_GPIOH_MUXH               (AT32_GPIOH_BASE+AT32_GPIO_MUXH_OFFSET)
#  define AT32_GPIOH_CLR                (AT32_GPIOH_BASE+AT32_GPIO_CLR_OFFSET)
#  define AT32_GPIOH_HDRV               (AT32_GPIOH_BASE+AT32_GPIO_HDRV_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* GPIO port config register */

#define GPIO_CFGR_INPUT                 (0) /* Input mode */
#define GPIO_CFGR_OUTPUT                (1) /* Output mode */
#define GPIO_CFGR_AF                    (2) /* Alternate mode */
#define GPIO_CFGR_ANALOG                (3) /* Analog mode */

#define GPIO_CFGR_SHIFT(n)              ((n) << 1)
#define GPIO_CFGR_MASK(n)               (3 << GPIO_CFGR_SHIFT(n))

#define GPIO_CFGR0_SHIFT                (0)
#define GPIO_CFGR0_MASK                 (3 << GPIO_CFGR0_SHIFT)
#define GPIO_CFGR1_SHIFT                (2)
#define GPIO_CFGR1_MASK                 (3 << GPIO_CFGR1_SHIFT)
#define GPIO_CFGR2_SHIFT                (4)
#define GPIO_CFGR2_MASK                 (3 << GPIO_CFGR2_SHIFT)
#define GPIO_CFGR3_SHIFT                (6)
#define GPIO_CFGR3_MASK                 (3 << GPIO_CFGR3_SHIFT)
#define GPIO_CFGR4_SHIFT                (8)
#define GPIO_CFGR4_MASK                 (3 << GPIO_CFGR4_SHIFT)
#define GPIO_CFGR5_SHIFT                (10)
#define GPIO_CFGR5_MASK                 (3 << GPIO_CFGR5_SHIFT)
#define GPIO_CFGR6_SHIFT                (12)
#define GPIO_CFGR6_MASK                 (3 << GPIO_CFGR6_SHIFT)
#define GPIO_CFGR7_SHIFT                (14)
#define GPIO_CFGR7_MASK                 (3 << GPIO_CFGR7_SHIFT)
#define GPIO_CFGR8_SHIFT                (16)
#define GPIO_CFGR8_MASK                 (3 << GPIO_CFGR8_SHIFT)
#define GPIO_CFGR9_SHIFT                (18)
#define GPIO_CFGR9_MASK                 (3 << GPIO_CFGR9_SHIFT)
#define GPIO_CFGR10_SHIFT               (20)
#define GPIO_CFGR10_MASK                (3 << GPIO_CFGR10_SHIFT)
#define GPIO_CFGR11_SHIFT               (22)
#define GPIO_CFGR11_MASK                (3 << GPIO_CFGR11_SHIFT)
#define GPIO_CFGR12_SHIFT               (24)
#define GPIO_CFGR12_MASK                (3 << GPIO_CFGR12_SHIFT)
#define GPIO_CFGR13_SHIFT               (26)
#define GPIO_CFGR13_MASK                (3 << GPIO_CFGR13_SHIFT)
#define GPIO_CFGR14_SHIFT               (28)
#define GPIO_CFGR14_MASK                (3 << GPIO_CFGR14_SHIFT)
#define GPIO_CFGR15_SHIFT               (30)
#define GPIO_CFGR15_MASK                (3 << GPIO_CFGR15_SHIFT)

/* GPIO port output type register */

#define GPIO_OMODER_OD(n)          (1 << (n)) /* 1=Output open-drain */
#define GPIO_OMODER_PP(n)          (0)        /* 0=Output push-pull */

/* GPIO output dirve capability */

#define GPIO_ODRVR_MODERATE             (0) /* moderate drive */
#define GPIO_ODRVR_STRONG               (1) /* strong drive */

#define GPIO_ODRVR_SHIFT(n)             ((n) << 1)
#define GPIO_ODRVR_MASK(n)              (3 << GPIO_ODRVR_SHIFT(n))

#define GPIO_ODRVR0_SHIFT               (0)
#define GPIO_ODRVR0_MASK                (3 << GPIO_ODRVR0_SHIFT)
#define GPIO_ODRVR1_SHIFT               (2)
#define GPIO_ODRVR1_MASK                (3 << GPIO_ODRVR1_SHIFT)
#define GPIO_ODRVR2_SHIFT               (4)
#define GPIO_ODRVR2_MASK                (3 << GPIO_ODRVR2_SHIFT)
#define GPIO_ODRVR3_SHIFT               (6)
#define GPIO_ODRVR3_MASK                (3 << GPIO_ODRVR3_SHIFT)
#define GPIO_ODRVR4_SHIFT               (8)
#define GPIO_ODRVR4_MASK                (3 << GPIO_ODRVR4_SHIFT)
#define GPIO_ODRVR5_SHIFT               (10)
#define GPIO_ODRVR5_MASK                (3 << GPIO_ODRVR5_SHIFT)
#define GPIO_ODRVR6_SHIFT               (12)
#define GPIO_ODRVR6_MASK                (3 << GPIO_ODRVR6_SHIFT)
#define GPIO_ODRVR7_SHIFT               (14)
#define GPIO_ODRVR7_MASK                (3 << GPIO_ODRVR7_SHIFT)
#define GPIO_ODRVR8_SHIFT               (16)
#define GPIO_ODRVR8_MASK                (3 << GPIO_ODRVR8_SHIFT)
#define GPIO_ODRVR9_SHIFT               (18)
#define GPIO_ODRVR9_MASK                (3 << GPIO_ODRVR9_SHIFT)
#define GPIO_ODRVR10_SHIFT              (20)
#define GPIO_ODRVR10_MASK               (3 << GPIO_ODRVR10_SHIFT)
#define GPIO_ODRVR11_SHIFT              (22)
#define GPIO_ODRVR11_MASK               (3 << GPIO_ODRVR11_SHIFT)
#define GPIO_ODRVR12_SHIFT              (24)
#define GPIO_ODRVR12_MASK               (3 << GPIO_ODRVR12_SHIFT)
#define GPIO_ODRVR13_SHIFT              (26)
#define GPIO_ODRVR13_MASK               (3 << GPIO_ODRVR13_SHIFT)
#define GPIO_ODRVR14_SHIFT              (28)
#define GPIO_ODRVR14_MASK               (3 << GPIO_ODRVR14_SHIFT)
#define GPIO_ODRVR15_SHIFT              (30)
#define GPIO_ODRVR15_MASK               (3 << GPIO_ODRVR15_SHIFT)

/* GPIO port pull-up/pull-down register */

#define GPIO_PULL_NONE                  (0) /* No pull-up, pull-down */
#define GPIO_PULL_PULLUP                (1) /* Pull-up */
#define GPIO_PULL_PULLDOWN              (2) /* Pull-down */

#define GPIO_PULL_SHIFT(n)              ((n) << 1)
#define GPIO_PULL_MASK(n)               (3 << GPIO_PULL_SHIFT(n))

#define GPIO_PULL0_SHIFT                (0)
#define GPIO_PULL0_MASK                 (3 << GPIO_PULL0_SHIFT)
#define GPIO_PULL1_SHIFT                (2)
#define GPIO_PULL1_MASK                 (3 << GPIO_PULL1_SHIFT)
#define GPIO_PULL2_SHIFT                (4)
#define GPIO_PULL2_MASK                 (3 << GPIO_PULL2_SHIFT)
#define GPIO_PULL3_SHIFT                (6)
#define GPIO_PULL3_MASK                 (3 << GPIO_PULL3_SHIFT)
#define GPIO_PULL4_SHIFT                (8)
#define GPIO_PULL4_MASK                 (3 << GPIO_PULL4_SHIFT)
#define GPIO_PULL5_SHIFT                (10)
#define GPIO_PULL5_MASK                 (3 << GPIO_PULL5_SHIFT)
#define GPIO_PULL6_SHIFT                (12)
#define GPIO_PULL6_MASK                 (3 << GPIO_PULL6_SHIFT)
#define GPIO_PULL7_SHIFT                (14)
#define GPIO_PULL7_MASK                 (3 << GPIO_PULL7_SHIFT)
#define GPIO_PULL8_SHIFT                (16)
#define GPIO_PULL8_MASK                 (3 << GPIO_PULL8_SHIFT)
#define GPIO_PULL9_SHIFT                (18)
#define GPIO_PULL9_MASK                 (3 << GPIO_PULL9_SHIFT)
#define GPIO_PULL10_SHIFT               (20)
#define GPIO_PULL10_MASK                (3 << GPIO_PULL10_SHIFT)
#define GPIO_PULL11_SHIFT               (22)
#define GPIO_PULL11_MASK                (3 << GPIO_PULL11_SHIFT)
#define GPIO_PULL12_SHIFT               (24)
#define GPIO_PULL12_MASK                (3 << GPIO_PULL12_SHIFT)
#define GPIO_PULL13_SHIFT               (26)
#define GPIO_PULL13_MASK                (3 << GPIO_PULL13_SHIFT)
#define GPIO_PULL14_SHIFT               (28)
#define GPIO_PULL14_MASK                (3 << GPIO_PULL14_SHIFT)
#define GPIO_PULL15_SHIFT               (30)
#define GPIO_PULL15_MASK                (3 << GPIO_PULL15_SHIFT)

/* GPIO port input data register */

#define GPIO_IDT(n)                     (1 << (n))

/* GPIO port output data register */

#define GPIO_ODT(n)                     (1 << (n))

/* GPIO port bit set/reset register */

#define GPIO_SCR_SET(n)                 (1 << (n))
#define GPIO_SCR_RESET(n)               (1 << ((n)+16))

/* GPIO port configuration protect register */

#define GPIO_PWR_WPEN(n)                (1 << (n))
#define GPIO_PWR_WPSEQ                  (1 << 16)   /* Lock key */

/* GPIO alternate function low/high register */

#define GPIO_MUX_SHIFT(n)               ((n) << 2)
#define GPIO_MUX_MASK(n)                (15 << GPIO_MUX_SHIFT(n))

#define GPIO_MUXL0_SHIFT                (0)
#define GPIO_MUXL0_MASK                 (15 << GPIO_MUXL0_SHIFT)
#define GPIO_MUXL1_SHIFT                (4)
#define GPIO_MUXL1_MASK                 (15 << GPIO_MUXL1_SHIFT)
#define GPIO_MUXL2_SHIFT                (8)
#define GPIO_MUXL2_MASK                 (15 << GPIO_MUXL2_SHIFT)
#define GPIO_MUXL3_SHIFT                (12)
#define GPIO_MUXL3_MASK                 (15 << GPIO_MUXL3_SHIFT)
#define GPIO_MUXL4_SHIFT                (16)
#define GPIO_MUXL4_MASK                 (15 << GPIO_MUXL4_SHIFT)
#define GPIO_MUXL5_SHIFT                (20)
#define GPIO_MUXL5_MASK                 (15 << GPIO_MUXL5_SHIFT)
#define GPIO_MUXL6_SHIFT                (24)
#define GPIO_MUXL6_MASK                 (15 << GPIO_MUXL6_SHIFT)
#define GPIO_MUXL7_SHIFT                (28)
#define GPIO_MUXL7_MASK                 (15 << GPIO_MUXL7_SHIFT)
#define GPIO_MUXH8_SHIFT                (0)
#define GPIO_MUXH8_MASK                 (15 << GPIO_MUXH8_SHIFT)
#define GPIO_MUXH9_SHIFT                (4)
#define GPIO_MUXH9_MASK                 (15 << GPIO_MUXH9_SHIFT)
#define GPIO_MUXH10_SHIFT               (8)
#define GPIO_MUXH10_MASK                (15 << GPIO_MUXH10_SHIFT)
#define GPIO_MUXH11_SHIFT               (12)
#define GPIO_MUXH11_MASK                (15 << GPIO_MUXH11_SHIFT)
#define GPIO_MUXH12_SHIFT               (16)
#define GPIO_MUXH12_MASK                (15 << GPIO_MUXH12_SHIFT)
#define GPIO_MUXH13_SHIFT               (20)
#define GPIO_MUXH13_MASK                (15 << GPIO_MUXH13_SHIFT)
#define GPIO_MUXH14_SHIFT               (24)
#define GPIO_MUXH14_MASK                (15 << GPIO_MUXH14_SHIFT)
#define GPIO_MUXH15_SHIFT               (28)
#define GPIO_MUXH15_MASK                (15 << GPIO_MUXH15_SHIFT)

#endif /* __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_GPIO_H */