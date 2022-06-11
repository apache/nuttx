/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_gpio.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_TLSR82_GPIO_H
#define __ARCH_ARM_SRC_TLSR82_TLSR82_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>
#include <stdint.h>
#include <stdbool.h>

#include "hardware/tlsr82_irq.h"
#include "hardware/tlsr82_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO definitions t = type, n = name, c = config, p = pinset */

#define GPIO_VAL(t, n)        (GPIO_##t##_##n >> GPIO_##t##_SHIFT)
#define GPIO_GET(t, c)        (((c) & GPIO_##t##_MASK) >> GPIO_##t##_SHIFT)
#define GPIO_IS(t, n, c)      (((c) & GPIO_##t##_MASK) == GPIO_##t##_##n)

#define GPIO_CFG2SDKPIN(p)    (((p) & GPIO_GROUP_MASK) | (1 << GPIO_GET(PIN, p)))
#define GPIO_CFG2PIN(p)       ((p) & (GPIO_GROUP_MASK | GPIO_PIN_MASK))
#define GPIO_PIN2NUM(p)       (GPIO_GET(PIN, p) + (GPIO_GET(GROUP, p) << 3))

#define GPIO_ALL_MASK         (GPIO_PIN_MASK | GPIO_GROUP_MASK | GPIO_AF_MASK |\
                               GPIO_PUPD_MASK | GPIO_DS_MASK | GPIO_IRQ_MASK |\
                               GPIO_POL_MASK)

#define GPIO_INVLD_CFG        (~GPIO_ALL_MASK)

#define GPIO_VALID(c)         (((c) & (~GPIO_ALL_MASK)) == 0)

/* Gpio pin option, each gpio group has 8 pin
 * Mask in pinset 0x0000 00FF
 */

#define GPIO_PIN_SHIFT                 0
#define GPIO_PIN_MASK                  (0xFF << GPIO_PIN_SHIFT)
#define GPIO_PIN(n)                    ((n) << GPIO_PIN_SHIFT)
#define GPIO_PIN0                      (0 << GPIO_PIN_SHIFT)
#define GPIO_PIN1                      (1 << GPIO_PIN_SHIFT)
#define GPIO_PIN2                      (2 << GPIO_PIN_SHIFT)
#define GPIO_PIN3                      (3 << GPIO_PIN_SHIFT)
#define GPIO_PIN4                      (4 << GPIO_PIN_SHIFT)
#define GPIO_PIN5                      (5 << GPIO_PIN_SHIFT)
#define GPIO_PIN6                      (6 << GPIO_PIN_SHIFT)
#define GPIO_PIN7                      (7 << GPIO_PIN_SHIFT)

/* Gpio group option
 * Mask in pinset 0x0000 0300
 */

#define GPIO_GROUP_SHIFT               8
#define GPIO_GROUP_MASK                (0x3 << GPIO_GROUP_SHIFT)
#define GPIO_GROUP_A                   (0 << GPIO_GROUP_SHIFT)
#define GPIO_GROUP_B                   (1 << GPIO_GROUP_SHIFT)
#define GPIO_GROUP_C                   (2 << GPIO_GROUP_SHIFT)
#define GPIO_GROUP_D                   (3 << GPIO_GROUP_SHIFT)

/* Gpio alternative function option
 * Mask in config 0x0000 7000
 */

#define GPIO_AF_SHIFT                  12
#define GPIO_AF_MASK                   (0x7 << GPIO_AF_SHIFT)
#define GPIO_AF_SHUTDOWN               (0 << GPIO_AF_SHIFT)
#define GPIO_AF_INPUT                  (1 << GPIO_AF_SHIFT)
#define GPIO_AF_OUTPUT                 (2 << GPIO_AF_SHIFT)
#define GPIO_AF_MUX0                   (3 << GPIO_AF_SHIFT)
#define GPIO_AF_MUX1                   (4 << GPIO_AF_SHIFT)
#define GPIO_AF_MUX2                   (5 << GPIO_AF_SHIFT)
#define GPIO_AF_MUX3                   (6 << GPIO_AF_SHIFT)

/* Pull-up and Pull-down option
 * Mask in config 0x0003 0000
 */

#define GPIO_PUPD_SHIFT                16
#define GPIO_PUPD_MASK                 (0x3 << GPIO_PUPD_SHIFT)
#define GPIO_PUPD_NONE                 (0 << GPIO_PUPD_SHIFT)
#define GPIO_PUPD_PU1M                 (1 << GPIO_PUPD_SHIFT)
#define GPIO_PUPD_PD100K               (2 << GPIO_PUPD_SHIFT)
#define GPIO_PUPD_PU10K                (3 << GPIO_PUPD_SHIFT)

/* Driver Strength option
 * Mask in config 0x0004 0000
 */

#define GPIO_DS_SHIFT                  18
#define GPIO_DS_MASK                   (0x1 << GPIO_DS_SHIFT)
#define GPIO_DS_LOW                    (0 << GPIO_DS_SHIFT)
#define GPIO_DS_HIGH                   (1 << GPIO_DS_SHIFT)

/* Gpio interrupt type option
 * Mask in config 0x0070 0000
 */

#define GPIO_IRQ_SHIFT                 20
#define GPIO_IRQ_MASK                  (0x7 << GPIO_IRQ_SHIFT)
#define GPIO_IRQ_DISABLE               (0 << GPIO_IRQ_SHIFT)
#define GPIO_IRQ_NORMAL                (1 << GPIO_IRQ_SHIFT)
#define GPIO_IRQ_M0                    (2 << GPIO_IRQ_SHIFT)
#define GPIO_IRQ_M1                    (3 << GPIO_IRQ_SHIFT)
#define GPIO_IRQ_M2                    (4 << GPIO_IRQ_SHIFT)
#define GPIO_IRQ_RISC0                 (5 << GPIO_IRQ_SHIFT)
#define GPIO_IRQ_RISC1                 (6 << GPIO_IRQ_SHIFT)
#define GPIO_IRQ_NORMAL_VAL            1
#define GPIO_IRQ_M0_VAL                2
#define GPIO_IRQ_M1_VAL                3
#define GPIO_IRQ_M2_VAL                4
#define GPIO_IRQ_RISC0_VAL             5
#define GPIO_IRQ_RISC1_VAL             6

/* Gpio interrupt polarity option
 * Mask in config 0x0080 0000
 */

#define GPIO_POL_SHIFT                 23
#define GPIO_POL_MASK                  (0x1 << GPIO_POL_SHIFT)
#define GPIO_POL_RISE                  (0 << GPIO_POL_SHIFT)
#define GPIO_POL_FALL                  (1 << GPIO_POL_SHIFT)

/* GPIO specific pin definitions */

#define GPIO_PIN_PA0                   (GPIO_GROUP_A | GPIO_PIN0)
#define GPIO_PIN_PA1                   (GPIO_GROUP_A | GPIO_PIN1)
#define GPIO_PIN_PA2                   (GPIO_GROUP_A | GPIO_PIN2)
#define GPIO_PIN_PA3                   (GPIO_GROUP_A | GPIO_PIN3)
#define GPIO_PIN_PA4                   (GPIO_GROUP_A | GPIO_PIN4)
#define GPIO_PIN_PA5                   (GPIO_GROUP_A | GPIO_PIN5)
#define GPIO_PIN_PA6                   (GPIO_GROUP_A | GPIO_PIN6)
#define GPIO_PIN_PA7                   (GPIO_GROUP_A | GPIO_PIN7)

#define GPIO_PIN_PB0                   (GPIO_GROUP_B | GPIO_PIN0)
#define GPIO_PIN_PB1                   (GPIO_GROUP_B | GPIO_PIN1)
#define GPIO_PIN_PB2                   (GPIO_GROUP_B | GPIO_PIN2)
#define GPIO_PIN_PB3                   (GPIO_GROUP_B | GPIO_PIN3)
#define GPIO_PIN_PB4                   (GPIO_GROUP_B | GPIO_PIN4)
#define GPIO_PIN_PB5                   (GPIO_GROUP_B | GPIO_PIN5)
#define GPIO_PIN_PB6                   (GPIO_GROUP_B | GPIO_PIN6)
#define GPIO_PIN_PB7                   (GPIO_GROUP_B | GPIO_PIN7)

#define GPIO_PIN_PC0                   (GPIO_GROUP_C | GPIO_PIN0)
#define GPIO_PIN_PC1                   (GPIO_GROUP_C | GPIO_PIN1)
#define GPIO_PIN_PC2                   (GPIO_GROUP_C | GPIO_PIN2)
#define GPIO_PIN_PC3                   (GPIO_GROUP_C | GPIO_PIN3)
#define GPIO_PIN_PC4                   (GPIO_GROUP_C | GPIO_PIN4)
#define GPIO_PIN_PC5                   (GPIO_GROUP_C | GPIO_PIN5)
#define GPIO_PIN_PC6                   (GPIO_GROUP_C | GPIO_PIN6)
#define GPIO_PIN_PC7                   (GPIO_GROUP_C | GPIO_PIN7)

#define GPIO_PIN_PD0                   (GPIO_GROUP_D | GPIO_PIN0)
#define GPIO_PIN_PD1                   (GPIO_GROUP_D | GPIO_PIN1)
#define GPIO_PIN_PD2                   (GPIO_GROUP_D | GPIO_PIN2)
#define GPIO_PIN_PD3                   (GPIO_GROUP_D | GPIO_PIN3)
#define GPIO_PIN_PD4                   (GPIO_GROUP_D | GPIO_PIN4)
#define GPIO_PIN_PD5                   (GPIO_GROUP_D | GPIO_PIN5)
#define GPIO_PIN_PD6                   (GPIO_GROUP_D | GPIO_PIN6)
#define GPIO_PIN_PD7                   (GPIO_GROUP_D | GPIO_PIN7)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Must be big enough to hold the above encodings */

typedef uint32_t gpio_cfg_t;

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

void tlsr82_gpio_as_gpio(gpio_cfg_t cfg, bool enable);
void tlsr82_gpio_input_ctrl(gpio_cfg_t cfg, bool enable);
void tlsr82_gpio_output_ctrl(gpio_cfg_t cfg, bool enable);
void tlsr82_gpio_pupd_ctrl(gpio_cfg_t cfg, uint8_t pupd);

/****************************************************************************
 * Name: tlsr82_gpioconfig
 *
 * Description:
 *   Configure a GPIO pin based on encoded pin config.
 *
 ****************************************************************************/

int tlsr82_gpioconfig(gpio_cfg_t cfg);

/****************************************************************************
 * Name: tlsr82_unconfiggpio
 *
 * Description:
 *   Un-configure a GPIO pin based on encoded pin config.
 *
 ****************************************************************************/

int tlsr82_gpiounconfig(gpio_cfg_t cfg);

/****************************************************************************
 * Name: tlsr82_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void tlsr82_gpiowrite(gpio_cfg_t cfg, bool value);

/****************************************************************************
 * Name: tlsr82_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool tlsr82_gpioread(gpio_cfg_t cfg);

/****************************************************************************
 * Name: tlsr82_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_IRQ
void tlsr82_gpioirqinitialize(void);
#else
#  define tlsr82_gpioirqinitialize()
#endif

#ifdef CONFIG_TLSR82_GPIO_IRQ
int tlsr82_gpioirqconfig(gpio_cfg_t cfg, xcpt_t func, void *arg);
#else
#  define tlsr82_gpioirqconfig(cfg, func, arg)
#endif

/****************************************************************************
 * Name: tlsr82_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_IRQ
void tlsr82_gpioirqenable(gpio_cfg_t cfg);
#else
#  define tlsr82_gpioirqenable(cfg)
#endif

/****************************************************************************
 * Name: tlsr82_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_IRQ
void tlsr82_gpioirqdisable(gpio_cfg_t cfg);
#else
#  define tlsr82_gpioirqdisable(cfg)
#endif

/****************************************************************************
 * Name: tlsr82_gpioirqdisable_all
 *
 * Description:
 *   Disable all the gpio interrupt (not clear gpio pin corresponding
 *   interrupt bit.)
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_IRQ
void tlsr82_gpioirqdisable_all(void);
#else
#  define tlsr82_gpioirqdisable_all()
#endif

/****************************************************************************
 * Name: tlsr82_gpioirqenable_all
 *
 * Description:
 *   Enable all the gpio interrupt (not set gpio pin corresponding
 *   interrupt bit.)
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_IRQ
void tlsr82_gpioirqenable_all(void);
#else
#  define tlsr82_gpioirqenable_all()
#endif

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_TLSR82_TLSR82_GPIO_H */
