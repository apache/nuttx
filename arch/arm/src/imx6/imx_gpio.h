/****************************************************************************
 * arch/arm/src/imx6/imx_gpio.h
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

#ifndef __ARCH_ARM_SRC_IMX6_IMX_GPIO_H
#define __ARCH_ARM_SRC_IMX6_IMX_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "hardware/imx_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 32-bit Encoding:
 *
 *   ENCODING    IIXX XXXX XXXX XXXX  MMMM MMMM MMMM MMMM
 *   GPIO INPUT  00.. .... GGGP PPPP  MMMM MMMM MMMM MMMM
 *   GPIO OUTPUT 01V. .... GGGP PPPP  MMMM MMMM MMMM MMMM
 *   PERIPHERAL  10AA A... IIII IIII  MMMM MMMM MMMM MMMM
 */

/* Input/Output Selection:
 *
 *   ENCODING    II.. .... .... ....  .... .... .... ....
 */

#define GPIO_MODE_SHIFT        (30)      /* Bits 30-31: Pin mode */
#define GPIO_MODE_MASK         (3u << GPIO_MODE_SHIFT)
#  define GPIO_INPUT           (0u << GPIO_MODE_SHIFT) /* GPIO input */
#  define GPIO_OUTPUT          (1u << GPIO_MODE_SHIFT) /* GPIO output */
#  define GPIO_PERIPH          (2u << GPIO_MODE_SHIFT) /* Peripheral */

/* Initial Output Value:
 *
 *   GPIO OUTPUT 01V. .... .... ....  .... .... .... ....
 */

#define GPIO_OUTPUT_ZERO       (0)       /* Bit 29: 0=Initial output is low */
#define GPIO_OUTPUT_ONE        (1 << 29) /* Bit 29: 1=Initial output is high */

/* GPIO Port Number
 *
 *   GPIO INPUT  00.. .... GGG. ....  .... .... .... ....
 *   GPIO OUTPUT 01.. .... GGG. ....  .... .... .... ....
 */

#define GPIO_PORT_SHIFT        (21)      /* Bits 21-23: GPIO port index */
#define GPIO_PORT_MASK         (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORT1           (0 << GPIO_PORT_SHIFT) /* GPIO1 */
#  define GPIO_PORT2           (1 << GPIO_PORT_SHIFT) /* GPIO2 */
#  define GPIO_PORT3           (2 << GPIO_PORT_SHIFT) /* GPIO3 */
#  define GPIO_PORT4           (3 << GPIO_PORT_SHIFT) /* GPIO4 */
#  define GPIO_PORT5           (4 << GPIO_PORT_SHIFT) /* GPIO5 */
#  define GPIO_PORT6           (5 << GPIO_PORT_SHIFT) /* GPIO6 */
#  define GPIO_PORT7           (6 << GPIO_PORT_SHIFT) /* GPIO7 */

/* GPIO Pin Number:
 *
 *   GPIO INPUT  00.. .... ...P PPPP  .... .... .... ....
 *   GPIO OUTPUT 01.. .... ...P PPPP  .... .... .... ....
 */

#define GPIO_PIN_SHIFT         (16)      /* Bits 16-20: GPIO pin number */
#define GPIO_PIN_MASK          (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN0            (0 << GPIO_PIN_SHIFT)  /* Pin  0 */
#  define GPIO_PIN1            (1 << GPIO_PIN_SHIFT)  /* Pin  1 */
#  define GPIO_PIN2            (2 << GPIO_PIN_SHIFT)  /* Pin  2 */
#  define GPIO_PIN3            (3 << GPIO_PIN_SHIFT)  /* Pin  3 */
#  define GPIO_PIN4            (4 << GPIO_PIN_SHIFT)  /* Pin  4 */
#  define GPIO_PIN5            (5 << GPIO_PIN_SHIFT)  /* Pin  5 */
#  define GPIO_PIN6            (6 << GPIO_PIN_SHIFT)  /* Pin  6 */
#  define GPIO_PIN7            (7 << GPIO_PIN_SHIFT)  /* Pin  7 */
#  define GPIO_PIN8            (8 << GPIO_PIN_SHIFT)  /* Pin  8 */
#  define GPIO_PIN9            (9 << GPIO_PIN_SHIFT)  /* Pin  9 */
#  define GPIO_PIN10           (10 << GPIO_PIN_SHIFT) /* Pin 10 */
#  define GPIO_PIN11           (11 << GPIO_PIN_SHIFT) /* Pin 11 */
#  define GPIO_PIN12           (12 << GPIO_PIN_SHIFT) /* Pin 12 */
#  define GPIO_PIN13           (13 << GPIO_PIN_SHIFT) /* Pin 13 */
#  define GPIO_PIN14           (14 << GPIO_PIN_SHIFT) /* Pin 14 */
#  define GPIO_PIN15           (15 << GPIO_PIN_SHIFT) /* Pin 15 */
#  define GPIO_PIN16           (16 << GPIO_PIN_SHIFT) /* Pin 16 */
#  define GPIO_PIN17           (17 << GPIO_PIN_SHIFT) /* Pin 17 */
#  define GPIO_PIN18           (18 << GPIO_PIN_SHIFT) /* Pin 18 */
#  define GPIO_PIN19           (19 << GPIO_PIN_SHIFT) /* Pin 19 */
#  define GPIO_PIN20           (20 << GPIO_PIN_SHIFT) /* Pin 20 */
#  define GPIO_PIN21           (21 << GPIO_PIN_SHIFT) /* Pin 21 */
#  define GPIO_PIN22           (22 << GPIO_PIN_SHIFT) /* Pin 22 */
#  define GPIO_PIN23           (23 << GPIO_PIN_SHIFT) /* Pin 23 */
#  define GPIO_PIN24           (24 << GPIO_PIN_SHIFT) /* Pin 24 */
#  define GPIO_PIN25           (25 << GPIO_PIN_SHIFT) /* Pin 25 */
#  define GPIO_PIN26           (26 << GPIO_PIN_SHIFT) /* Pin 26 */
#  define GPIO_PIN27           (27 << GPIO_PIN_SHIFT) /* Pin 27 */
#  define GPIO_PIN28           (28 << GPIO_PIN_SHIFT) /* Pin 28 */
#  define GPIO_PIN29           (29 << GPIO_PIN_SHIFT) /* Pin 29 */
#  define GPIO_PIN30           (30 << GPIO_PIN_SHIFT) /* Pin 30 */
#  define GPIO_PIN31           (31 << GPIO_PIN_SHIFT) /* Pin 31 */

/* Peripheral Alternate Function:
 *
 *   PERIPHERAL  10AA A... .... ....  MMMM MMMM MMMM MMMM
 */

#define GPIO_ALT_SHIFT         (27)      /* Bits 27-29: Peripheral alternate function */
#define GPIO_ALT_MASK          (15 << GPIO_ALT_SHIFT)
#  define GPIO_ALT0            (0 << GPIO_ALT_SHIFT)  /* Alternate function 1 */
#  define GPIO_ALT1            (1 << GPIO_ALT_SHIFT)  /* Alternate function 2 */
#  define GPIO_ALT2            (2 << GPIO_ALT_SHIFT)  /* Alternate function 3 */
#  define GPIO_ALT3            (3 << GPIO_ALT_SHIFT)  /* Alternate function 4 */
#  define GPIO_ALT4            (4 << GPIO_ALT_SHIFT)  /* Alternate function 5 */
                                                      /* Alternate function 5 is GPIO */
#  define GPIO_ALT6            (6 << GPIO_ALT_SHIFT)  /* Alternate function 1 */
#  define GPIO_ALT7            (7 << GPIO_ALT_SHIFT)  /* Alternate function 1 */

/* Pad Mux Register Index:
 *
 *   PERIPHERAL  10.. .... IIII IIII  MMMM MMMM MMMM MMMM
 */

#define GPIO_PADMUX_SHIFT      (16)      /* Bits 16-23: Peripheral alternate function */
#define GPIO_PADMUX_MASK       (0xff << GPIO_PADMUX_SHIFT)
#  define GPIO_PADMUX(n)       ((uint32_t)(n) << GPIO_PADMUX_SHIFT)

/* IOMUX Pin Configuration:
 *
 *   ENCODING    .... .... .... ....  MMMM MMMM MMMM MMMM
 *
 * See imx_iomuxc.h for detailed content.
 */

#define GPIO_IOMUX_SHIFT       (0)       /* Bits 0-15: IOMUX pin configuration */
#define GPIO_IOMUX_MASK        (0xffff << GPIO_IOMUX_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The smallest integer type that can hold the GPIO encoding */

typedef uint32_t gpio_pinset_t;

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
 * Name: imx_gpioirq_initialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_IMX6_GPIO_IRQ
void imx_gpioirq_initialize(void);
#else
#  define imx_gpio_irqinitialize()
#endif

/****************************************************************************
 * Name: imx_config_gpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int imx_config_gpio(gpio_pinset_t pinset);

/****************************************************************************
 * Name: imx_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void imx_gpio_write(gpio_pinset_t pinset, bool value);

/****************************************************************************
 * Name: imx_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool imx_gpio_read(gpio_pinset_t pinset);

/****************************************************************************
 * Name: imx_gpioirq
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ****************************************************************************/

#ifdef CONFIG_IMX6_GPIO_IRQ
void imx_gpioirq(gpio_pinset_t pinset);
#else
#  define imx_gpioirq(pinset)
#endif

/****************************************************************************
 * Name: imx_gpioirq_enable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_IMX6_GPIO_IRQ
void imx_gpioirq_enable(int irq);
#else
#  define imx_gpioirq_enable(irq)
#endif

/****************************************************************************
 * Name: imx_gpioirq_disable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_IMX6_GPIO_IRQ
void imx_gpioirq_disable(int irq);
#else
#  define imx_gpioirq_disable(irq)
#endif

/****************************************************************************
 * Function:  imx_dump_gpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided
 *   pinset.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int imx_dump_gpio(uint32_t pinset, const char *msg);
#else
#  define imx_dumpgpio(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ARCH_ARM_SRC_IMX6_IMX_GPIO_H */
