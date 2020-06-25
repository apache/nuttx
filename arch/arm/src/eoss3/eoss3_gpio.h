/****************************************************************************
 * arch/arm/src/eoss3/eoss3_gpio.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_EOSS3_EOSS3_GPIO_H
#define __ARCH_ARM_SRC_EOSS3_EOSS3_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"
#include "hardware/eoss3_iomux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* pinset encoding
 *  3322 2222 2222 1111 1111 1100 0000 0000
 *  1098 7654 3210 9876 5432 1098 7654 3210
 *  GGGE SSSS SSII ICCC CCCC CCCC CCPP PPPP
 *
 *  G - GPIO Reg Bit
 *  E - GPIO Reg Enable
 *  S - Maps to input select
 *  I - Input Select Register
 *  C - Maps to packed PAD CTRL
 *  P - PAD Number
 */

#define GPIO_PIN_SHIFT             (0)     /* Bits 0-5: Pin number: 0-45 */
#define GPIO_PIN_MASK              (0x3f << GPIO_PIN_SHIFT)
#  define GPIO_PIN0                (0  << GPIO_PIN_SHIFT)
#  define GPIO_PIN1                (1  << GPIO_PIN_SHIFT)
#  define GPIO_PIN2                (2  << GPIO_PIN_SHIFT)
#  define GPIO_PIN3                (3  << GPIO_PIN_SHIFT)
#  define GPIO_PIN4                (4  << GPIO_PIN_SHIFT)
#  define GPIO_PIN5                (5  << GPIO_PIN_SHIFT)
#  define GPIO_PIN6                (6  << GPIO_PIN_SHIFT)
#  define GPIO_PIN7                (7  << GPIO_PIN_SHIFT)
#  define GPIO_PIN8                (8  << GPIO_PIN_SHIFT)
#  define GPIO_PIN9                (9  << GPIO_PIN_SHIFT)
#  define GPIO_PIN10               (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11               (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12               (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13               (13 << GPIO_PIN_SHIFT)
#  define GPIO_PIN14               (14 << GPIO_PIN_SHIFT)
#  define GPIO_PIN15               (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN16               (16 << GPIO_PIN_SHIFT)
#  define GPIO_PIN17               (17 << GPIO_PIN_SHIFT)
#  define GPIO_PIN18               (18 << GPIO_PIN_SHIFT)
#  define GPIO_PIN19               (19 << GPIO_PIN_SHIFT)
#  define GPIO_PIN20               (20 << GPIO_PIN_SHIFT)
#  define GPIO_PIN21               (21 << GPIO_PIN_SHIFT)
#  define GPIO_PIN22               (22 << GPIO_PIN_SHIFT)
#  define GPIO_PIN23               (23 << GPIO_PIN_SHIFT)
#  define GPIO_PIN24               (24 << GPIO_PIN_SHIFT)
#  define GPIO_PIN25               (25 << GPIO_PIN_SHIFT)
#  define GPIO_PIN26               (26 << GPIO_PIN_SHIFT)
#  define GPIO_PIN27               (27 << GPIO_PIN_SHIFT)
#  define GPIO_PIN28               (28 << GPIO_PIN_SHIFT)
#  define GPIO_PIN29               (29 << GPIO_PIN_SHIFT)
#  define GPIO_PIN30               (30 << GPIO_PIN_SHIFT)
#  define GPIO_PIN31               (31 << GPIO_PIN_SHIFT)
#  define GPIO_PIN32               (32 << GPIO_PIN_SHIFT)
#  define GPIO_PIN33               (33 << GPIO_PIN_SHIFT)
#  define GPIO_PIN34               (34 << GPIO_PIN_SHIFT)
#  define GPIO_PIN35               (35 << GPIO_PIN_SHIFT)
#  define GPIO_PIN36               (36 << GPIO_PIN_SHIFT)
#  define GPIO_PIN37               (37 << GPIO_PIN_SHIFT)
#  define GPIO_PIN38               (38 << GPIO_PIN_SHIFT)
#  define GPIO_PIN39               (39 << GPIO_PIN_SHIFT)
#  define GPIO_PIN40               (40 << GPIO_PIN_SHIFT)
#  define GPIO_PIN41               (41 << GPIO_PIN_SHIFT)
#  define GPIO_PIN42               (42 << GPIO_PIN_SHIFT)
#  define GPIO_PIN43               (43 << GPIO_PIN_SHIFT)
#  define GPIO_PIN44               (44 << GPIO_PIN_SHIFT)
#  define GPIO_PIN45               (45 << GPIO_PIN_SHIFT)

#define GPIO_CTRL_SHIFT            (6)     /* Bits 6-18 */
#define GPIO_CTRL_MASK             (0x1fff << GPIO_CTRL_SHIFT)
#define GPIO_INPUT_SEL_SHIFT       (19)    /* Bits 19-27 */
#define GPIO_INPUT_SEL_MASK        (0x1ff << GPIO_INPUT_SEL_SHIFT)

#define GPIO_REG_EN_SHIFT          (28)
#define GPIO_REG_EN_MASK           (1 << GPIO_REG_EN_SHIFT)

#define GPIO_REG_BIT_SHIFT         (29)
#define GPIO_REG_BIT_MASK          (0x7 << GPIO_REG_BIT_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Must be big enough to hold the 20-bit encoding */

typedef uint32_t gpio_pinset_t;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: eoss3_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   PIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_EOSS3_GPIO_IRQ
void eoss3_gpioirqinitialize(void);
#else
#  define eoss3_gpioirqinitialize()
#endif

/****************************************************************************
 * Name: eoss3_configgpio
 *
 * Description:
 *   Configure a PIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int eoss3_configgpio(gpio_pinset_t cfgset);

/****************************************************************************

 * Name: eoss3_unconfiggpio
 *
 * Description:
 *   UnConfigure a PIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int eoss3_unconfiggpio(gpio_pinset_t cfgset);

/****************************************************************************
 * Name: eoss3_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected PIO pin
 *
 ****************************************************************************/

void eoss3_gpiowrite(gpio_pinset_t pinset, bool value);

/****************************************************************************
 * Name: eoss3_gpioread
 *
 * Description:
 *   Read one or zero from the selected PIO pin
 *
 ****************************************************************************/

bool eoss3_gpioread(gpio_pinset_t pinset);

/****************************************************************************
 * Name: eoss3_gpioirq
 *
 * Description:
 *   Configure an interrupt for the specified PIO pin.
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_GPIO_IRQ
void eoss3_gpioirq(gpio_pinset_t pinset);
#else
#  define eoss3_gpioirq(pinset)
#endif

/****************************************************************************
 * Name: eoss3_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified PIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_GPIO_IRQ
void eoss3_gpioirqenable(int irq);
#else
#  define eoss3_gpioirqenable(irq)
#endif

/****************************************************************************
 * Name: eoss3_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified PIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_GPIO_IRQ
void eoss3_gpioirqdisable(int irq);
#else
#  define eoss3_gpioirqdisable(irq)
#endif

/****************************************************************************
 * Name: eoss3_gpioirqclear
 *
 * Description:
 *   clear the interrupt for specified PIO IRQ
 *
 ****************************************************************************/
#ifdef CONFIG_EOSS3_GPIO_IRQ
void eoss3_gpioirqclear(int irq);
#else
#  define eoss3_gpioirqclear(irq)
#endif

/****************************************************************************
 * Function:  eoss3_dumpgpio
 *
 * Description:
 *   Dump all PIO registers associated with the base address of the provided
 *   pinset.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int eoss3_dumpgpio(uint32_t pinset, const char *msg);
#else
#  define eoss3_dumpgpio(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_EOSS3_EOSS3_GPIO_H */
