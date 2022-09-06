/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_gpio.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_GD32F4XX_GPIO_H
#define __ARCH_ARM_SRC_GD32F4_GD32F4XX_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include <nuttx/irq.h>

#include "chip.h"

#include "hardware/gd32f4xx_gpio.h"

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/* Bit-encoded input to gd32_configgpio() */

#if defined(CONFIG_GD32F4_GD32F4XX)
/* Each pin of the general-purpose I/O (GPIO) ports can be individually
 * configured by software in several modes. The following definitions provide
 * the bit encoding that used to define the pin mode.
 *
 * 20-bit Encoding:       1111 1111 1100 0000 0000
 *                        9876 5432 1098 7654 3210
 * ENCODING               MMUU OVSS AAAA PPPP BBBB
 * GPIO_MODE_INPUT:       00UU .X.. .... PPPP BBBB
 * GPIO_MODE_OUTPUT:      01UU OVSS .... PPPP BBBB
 * GPIO_MODE_AF:          10UU O.SS AAAA PPPP BBBB
 * GPIO_MODE_ANALOG:      11UU .... .... PPPP BBBB
 */

/* Mode configuration:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * MM.. .... .... .... ....
 */

#define GPIO_CFG_MODE_SHIFT        (18)                          /* Bits 18-19: GPIO pin mode */
#define GPIO_CFG_MODE_MASK         (3 << GPIO_CFG_MODE_SHIFT)
#  define GPIO_CFG_MODE_INPUT      (0 << GPIO_CFG_MODE_SHIFT)    /* Input mode */
#  define GPIO_CFG_MODE_OUTPUT     (1 << GPIO_CFG_MODE_SHIFT)    /* General purpose output mode */
#  define GPIO_CFG_MODE_AF         (2 << GPIO_CFG_MODE_SHIFT)    /* Alternate function mode */
#  define GPIO_CFG_MODE_ANALOG     (3 << GPIO_CFG_MODE_SHIFT)    /* Analog mode */

/* Pull-up/ pull-down definitions:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * ..UU .... .... .... ....
 */

#define GPIO_CFG_PUPD_SHIFT        (16)                          /* Bits 16-17: Pull-up and pull-down resistors */
#define GPIO_CFG_PUPD_MASK         (3 << GPIO_CFG_PUPD_SHIFT)
#  define GPIO_CFG_PUPD_NONE       (0 << GPIO_CFG_PUPD_SHIFT)    /* No pull-up, pull-down */
#  define GPIO_CFG_PUPD_PULLUP     (1 << GPIO_CFG_PUPD_SHIFT)    /* With pull-up resistor */
#  define GPIO_CFG_PUPD_PULLDOWN   (2 << GPIO_CFG_PUPD_SHIFT)    /* With pull-down resistor */

/* GPIO output type:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... O... .... .... ....
 */

#define GPIO_CFG_ODPP_SHIFT        (15)                          /* Bit15: Open drain and push pull mode */
#define GPIO_CFG_ODPP_MASK         (1 << GPIO_CFG_ODPP_SHIFT)
#define GPIO_CFG_OD                (1 << GPIO_CFG_ODPP_SHIFT)    /* Open drain mode */
#define GPIO_CFG_PP                (0)                           /* Push pull mode */

/* If the pin is a GPIO digital output, then this identifies the initial
 * output value.
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .V.. .... .... ....
 */

#define GPIO_CFG_OUTPUT_SHIFT      (14)                          /* Bit 14: If output, output value */
#define GPIO_CFG_OUTPUT_MASK       (1 << GPIO_CFG_OUTPUT_SHIFT)
#define GPIO_CFG_OUTPUT_SET        (1 << GPIO_CFG_OUTPUT_SHIFT)
#define GPIO_CFG_OUTPUT_RESET      (0)

/* External interrupt selection (GPIO inputs only):
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .X.. .... .... ....
 */

#define GPIO_CFG_EXTI              (1 << 14)                     /* Bit 14: If input, configure as EXTI interrupt */

/* GPIO output max speed value:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... SS.. .... ....
 */

#define GPIO_CFG_SPEED_SHIFT       (12)                          /* Bits 12-13: GPIO speed selection */
#define GPIO_CFG_SPEED_MASK        (3 << GPIO_CFG_SPEED_SHIFT)
#  define GPIO_CFG_SPEED_2MHZ      (0 << GPIO_CFG_SPEED_SHIFT)   /* 2 MHZ Low speed output */
#  define GPIO_CFG_SPEED_25MHZ     (1 << GPIO_CFG_SPEED_SHIFT)   /* 25 MHZ Medium speed output */
#  define GPIO_CFG_SPEED_50MHZ     (2 << GPIO_CFG_SPEED_SHIFT)   /* 50 MHZ Fast speed output  */
#  define GPIO_CFG_SPEED_200MHZ    (3 << GPIO_CFG_SPEED_SHIFT)   /* 100 MHZ High speed output */

/* GPIO alternate function:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... AAAA .... ....
 */

#define GPIO_CFG_AF_SHIFT          (8)                           /* Bits 8-11: Alternate function */
#define GPIO_CFG_AF_MASK           (15 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF(n)           ((n) << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_0            (0 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_1            (1 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_2            (2 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_3            (3 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_4            (4 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_5            (5 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_6            (6 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_7            (7 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_8            (8 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_9            (9 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_10           (10 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_11           (11 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_12           (12 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_13           (13 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_14           (14 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_15           (15 << GPIO_CFG_AF_SHIFT)

/* This identifies the GPIO port:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... PPPP ....
 */

#define GPIO_CFG_PORT_SHIFT        (4)                           /* Bit 4-7:  Port number */
#define GPIO_CFG_PORT_MASK         (15 << GPIO_CFG_PORT_SHIFT)
#  define GPIO_CFG_PORT_A          (0 << GPIO_CFG_PORT_SHIFT)    /*   GPIOA */
#  define GPIO_CFG_PORT_B          (1 << GPIO_CFG_PORT_SHIFT)    /*   GPIOB */
#  define GPIO_CFG_PORT_C          (2 << GPIO_CFG_PORT_SHIFT)    /*   GPIOC */
#  define GPIO_CFG_PORT_D          (3 << GPIO_CFG_PORT_SHIFT)    /*   GPIOD */
#  define GPIO_CFG_PORT_E          (4 << GPIO_CFG_PORT_SHIFT)    /*   GPIOE */
#  define GPIO_CFG_PORT_F          (5 << GPIO_CFG_PORT_SHIFT)    /*   GPIOF */
#  define GPIO_CFG_PORT_G          (6 << GPIO_CFG_PORT_SHIFT)    /*   GPIOG */
#  define GPIO_CFG_PORT_H          (7 << GPIO_CFG_PORT_SHIFT)    /*   GPIOH */
#  define GPIO_CFG_PORT_I          (8 << GPIO_CFG_PORT_SHIFT)    /*   GPIOI */

/* This identifies the bit in the port:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... .... BBBB
 */

#define GPIO_CFG_PIN_SHIFT         (0)                           /* Bits 0-3: GPIO number: 0-15 */
#define GPIO_CFG_PIN_MASK          (15 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_0           (0 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_1           (1 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_2           (2 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_3           (3 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_4           (4 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_5           (5 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_6           (6 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_7           (7 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_8           (8 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_9           (9 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_10           (10 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_11           (11 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_12           (12 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_13           (13 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_14           (14 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_15           (15 << GPIO_CFG_PIN_SHIFT)

#else
#  error "Unknown GD32 chip"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Base addresses for each GPIO block */

EXTERN const uint32_t g_gpio_base[GD32_NGPIO_PORTS];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 * Return value:
 *   OK on success
 *   A negated errno value on invalid port or mode.
 ****************************************************************************/

int gd32_gpio_config(uint32_t cfgset);

/****************************************************************************
 * Name: gd32_gpio_unconfig
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set
 *   it into default HiZ state.
 *
 * Returned Value:
 *  OK on success
 *  A negated errno value on invalid port or mode
 ****************************************************************************/

int gd32_gpio_unconfig(uint32_t cfgset);

/****************************************************************************
 * Name: gd32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void gd32_gpio_write(uint32_t pinset, bool value);

/****************************************************************************
 * Name: gd32_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool gd32_gpio_read(uint32_t pinset);

/****************************************************************************
 * Function:  gd32_dump_gpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided
 *   pinset.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
int gd32_dump_gpio(uint32_t pinset, const char *msg);
#else
#  define gd32_dump_gpio(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32F4_GD32F4XX_GPIO_H */
