/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_gpio.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_GPIO_H
#define __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_GPIO_H

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
#include "hardware/gd32vw55x_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit-encoded input to gd32_gpio_config().
 *
 * Each pin of the general-purpose I/O (GPIO) ports can be individually
 * configured by software in several modes.  The following definitions
 * provide the bit encoding used to describe the pin configuration.  It is
 * the same encoding as the one used by the GD32F4xx port.
 *
 * 20-bit encoding:       1111 1111 1100 0000 0000
 *                        9876 5432 1098 7654 3210
 * ENCODING               MMUU OVSS AAAA PPPP BBBB
 * GPIO_CFG_MODE_INPUT:   00UU .X.. .... PPPP BBBB
 * GPIO_CFG_MODE_OUTPUT:  01UU OVSS .... PPPP BBBB
 * GPIO_CFG_MODE_AF:      10UU O.SS AAAA PPPP BBBB
 * GPIO_CFG_MODE_ANALOG:  11UU .... .... PPPP BBBB
 */

/* Mode configuration:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * MM.. .... .... .... ....
 */

/* Bits 18-19: GPIO pin mode */

#define GPIO_CFG_MODE_SHIFT      (18)
#define GPIO_CFG_MODE_MASK       (3 << GPIO_CFG_MODE_SHIFT)
#  define GPIO_CFG_MODE_INPUT    (0 << GPIO_CFG_MODE_SHIFT) /* Input */
#  define GPIO_CFG_MODE_OUTPUT   (1 << GPIO_CFG_MODE_SHIFT) /* Output */
#  define GPIO_CFG_MODE_AF       (2 << GPIO_CFG_MODE_SHIFT) /* Alt function */
#  define GPIO_CFG_MODE_ANALOG   (3 << GPIO_CFG_MODE_SHIFT) /* Analog */

/* Pull-up/pull-down definitions:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * ..UU .... .... .... ....
 */

/* Bits 16-17: Pull-up and pull-down resistors */

#define GPIO_CFG_PUPD_SHIFT      (16)
#define GPIO_CFG_PUPD_MASK       (3 << GPIO_CFG_PUPD_SHIFT)
#  define GPIO_CFG_PUPD_NONE     (0 << GPIO_CFG_PUPD_SHIFT) /* No pull */
#  define GPIO_CFG_PUPD_PULLUP   (1 << GPIO_CFG_PUPD_SHIFT) /* Pull-up */
#  define GPIO_CFG_PUPD_PULLDOWN (2 << GPIO_CFG_PUPD_SHIFT) /* Pull-down */

/* GPIO output type:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... O... .... .... ....
 */

/* Bit 15: Open-drain or push-pull output */

#define GPIO_CFG_ODPP_SHIFT      (15)
#define GPIO_CFG_ODPP_MASK       (1 << GPIO_CFG_ODPP_SHIFT)
#define GPIO_CFG_OD              (1 << GPIO_CFG_ODPP_SHIFT) /* Open drain */
#define GPIO_CFG_PP              (0)                        /* Push pull */

/* If the pin is a GPIO digital output, then this identifies the initial
 * output value:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .V.. .... .... ....
 */

/* Bit 14: If output, the initial output value */

#define GPIO_CFG_OUTPUT_SHIFT    (14)
#define GPIO_CFG_OUTPUT_MASK     (1 << GPIO_CFG_OUTPUT_SHIFT)
#define GPIO_CFG_OUTPUT_SET      (1 << GPIO_CFG_OUTPUT_SHIFT)
#define GPIO_CFG_OUTPUT_RESET    (0)

/* External interrupt selection (GPIO inputs only).  This reuses bit 14,
 * which is meaningless for an input pin:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .X.. .... .... ....
 */

#define GPIO_CFG_EXTI            (1 << 14)

/* GPIO output max speed value:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... SS.. .... ....
 */

/* Bits 12-13: GPIO output speed selection.  NOTE that the speed levels of
 * the GD32VW55x are not the same as the GD32F4xx ones.
 */

#define GPIO_CFG_SPEED_SHIFT     (12)
#define GPIO_CFG_SPEED_MASK      (3 << GPIO_CFG_SPEED_SHIFT)
#  define GPIO_CFG_SPEED_2MHZ    (0 << GPIO_CFG_SPEED_SHIFT) /* 2 MHz */
#  define GPIO_CFG_SPEED_10MHZ   (1 << GPIO_CFG_SPEED_SHIFT) /* 10 MHz */
#  define GPIO_CFG_SPEED_25MHZ   (2 << GPIO_CFG_SPEED_SHIFT) /* 25 MHz */
#  define GPIO_CFG_SPEED_MAX     (3 << GPIO_CFG_SPEED_SHIFT) /* Maximum */

/* GPIO alternate function:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... AAAA .... ....
 */

/* Bits 8-11: Alternate function */

#define GPIO_CFG_AF_SHIFT        (8)
#define GPIO_CFG_AF_MASK         (15 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF(n)         ((n) << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_0          (0 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_1          (1 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_2          (2 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_3          (3 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_4          (4 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_5          (5 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_6          (6 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_7          (7 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_8          (8 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_9          (9 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_10         (10 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_11         (11 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_12         (12 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_13         (13 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_14         (14 << GPIO_CFG_AF_SHIFT)
#  define GPIO_CFG_AF_15         (15 << GPIO_CFG_AF_SHIFT)

/* This identifies the GPIO port:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... PPPP ....
 */

/* Bits 4-7: Port number.  The GD32VW55x has GPIOA, GPIOB and GPIOC only */

#define GPIO_CFG_PORT_SHIFT      (4)
#define GPIO_CFG_PORT_MASK       (15 << GPIO_CFG_PORT_SHIFT)
#  define GPIO_CFG_PORT_A        (0 << GPIO_CFG_PORT_SHIFT)   /* GPIOA */
#  define GPIO_CFG_PORT_B        (1 << GPIO_CFG_PORT_SHIFT)   /* GPIOB */
#  define GPIO_CFG_PORT_C        (2 << GPIO_CFG_PORT_SHIFT)   /* GPIOC */

/* This identifies the bit in the port:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... .... BBBB
 */

/* Bits 0-3: GPIO pin number, 0-15 */

#define GPIO_CFG_PIN_SHIFT       (0)
#define GPIO_CFG_PIN_MASK        (15 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_0         (0 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_1         (1 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_2         (2 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_3         (3 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_4         (4 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_5         (5 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_6         (6 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_7         (7 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_8         (8 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_9         (9 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_10        (10 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_11        (11 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_12        (12 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_13        (13 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_14        (14 << GPIO_CFG_PIN_SHIFT)
#  define GPIO_CFG_PIN_15        (15 << GPIO_CFG_PIN_SHIFT)

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

/* Base addresses for each GPIO port */

EXTERN const uint32_t g_gpio_base[GD32VW55X_NGPIO_PORTS];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 * Input Parameters:
 *   cfgset - Bit-encoded description of the pin
 *
 * Returned Value:
 *   OK on success
 *   A negated errno value on invalid port or mode.
 *
 ****************************************************************************/

int gd32_gpio_config(uint32_t cfgset);

/****************************************************************************
 * Name: gd32_gpio_unconfig
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set
 *   it into default HiZ state.
 *
 * Input Parameters:
 *   cfgset - Bit-encoded description of the pin
 *
 * Returned Value:
 *   OK on success
 *   A negated errno value on invalid port or mode.
 *
 ****************************************************************************/

int gd32_gpio_unconfig(uint32_t cfgset);

/****************************************************************************
 * Name: gd32_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 * Input Parameters:
 *   pinset - Bit-encoded description of the pin
 *   value  - The value to write to the pin
 *
 ****************************************************************************/

void gd32_gpio_write(uint32_t pinset, bool value);

/****************************************************************************
 * Name: gd32_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 * Input Parameters:
 *   pinset - Bit-encoded description of the pin
 *
 * Returned Value:
 *   The input state of the pin
 *
 ****************************************************************************/

bool gd32_gpio_read(uint32_t pinset);

/****************************************************************************
 * Name: gd32_gpio_clock_enable
 *
 * Description:
 *   Enable the clock of the GPIO port that owns the given port base
 *   address.
 *
 * Input Parameters:
 *   port_base - The base address of the GPIO port
 *
 ****************************************************************************/

void gd32_gpio_clock_enable(uint32_t port_base);

/****************************************************************************
 * Name: gd32_dump_gpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the
 *   provided pinset.
 *
 * Input Parameters:
 *   pinset - Bit-encoded description of the pin
 *   msg    - A message to print with the register dump
 *
 * Returned Value:
 *   OK on success
 *   A negated errno value on an invalid port.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int gd32_dump_gpio(uint32_t pinset, const char *msg);
#else
#  define gd32_dump_gpio(p,m) (0)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_GPIO_H */
