/****************************************************************************
 * arch/arm64/src/imx9/imx9_gpio.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_GPIO_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"
#include "hardware/imx9_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO pinset is a 16-bit word used to configure the GPIO settings. The
 * encoding is as follows...
 *
 *               1111 1100 0000 0000
 *               5432 1098 7654 3210
 *   ENCODING    MMVX BEEG GGGP PPPP
 *   GPIO INPUT  00.. BEEG GGGP PPPP
 *   INT INPUT   11.. BEEG GGGP PPPP
 *   GPIO OUTPUT 01V. ...G GGGP PPPP
 */

/* Input/Output Selection:
 *
 *               1111 1100 0000 0000
 *               5432 1098 7654 3210
 *   ENCODING    MM.. .... .... ....
 */

#define GPIO_MODE_SHIFT        (14)      /* Bits 14-15: Pin mode */
#define GPIO_MODE_MASK         (0x3 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT           (0 << GPIO_MODE_SHIFT) /* GPIO input */
#  define GPIO_OUTPUT          (1 << GPIO_MODE_SHIFT) /* GPIO output */
#  define GPIO_INTERRUPT       (2 << GPIO_MODE_SHIFT) /* Interrupt input */

/* Initial Output Value:
 *
 *               1111 1100 0000 0000
 *               5432 1098 7654 3210
 *   GPIO OUTPUT 01V. .... .... ....
 */

#define GPIO_OUTPUT_SHIFT      (13)      /* Bit 13: Initial output */
#define GPIO_OUTPUT_MASK       (0x1 << GPIO_OUTPUT_SHIFT)
#  define GPIO_OUTPUT_ZERO     (0 << GPIO_OUTPUT_SHIFT) /* Bit 29: 0=Initial output is low */
#  define GPIO_OUTPUT_ONE      (1 << GPIO_OUTPUT_SHIFT) /* Bit 29: 1=Initial output is high */

/* Interrupt on both edges configuration
 *
 *               1111 1100 0000 0000
 *               5432 1098 7654 3210
 *   INT INPUT   11.. B... .... ....
 */

#define GPIO_INTBOTHCFG_SHIFT  (11)      /* Bit 11: Interrupt both edges configuration */
#define GPIO_INTBOTHCFG_MASK   (1 << GPIO_INTBOTHCFG_SHIFT)
#  define GPIO_INTBOTH_EDGES   (1 << GPIO_INTBOTHCFG_SHIFT)

/* Interrupt edge/level configuration
 *
 *               1111 1100 0000 0000
 *               5432 1098 7654 3210
 *   INT INPUT   11.. .EE. .... ....
 */

#define GPIO_INTCFG_SHIFT      (9)      /* Bits 9-10: Interrupt edge/level configuration */
#define GPIO_INTCFG_MASK       (0x3 << GPIO_INTCFG_SHIFT)
#  define GPIO_INT_LOWLEVEL    (0 << GPIO_INTCFG_SHIFT)
#  define GPIO_INT_HIGHLEVEL   (1 << GPIO_INTCFG_SHIFT)
#  define GPIO_INT_RISINGEDGE  (2 << GPIO_INTCFG_SHIFT)
#  define GPIO_INT_FALLINGEDGE (3 << GPIO_INTCFG_SHIFT)

/* GPIO Port Number
 *
 *               1111 1100 0000 0000
 *               5432 1098 7654 3210
 *   GPIO IN/OUT .... ...G GGG. ....
 */

#define GPIO_PORT_SHIFT        (5)      /* Bits 5-8: GPIO port index */
#define GPIO_PORT_MASK         (0xf << GPIO_PORT_SHIFT)
#  define GPIO_PORT1           (GPIO1 << GPIO_PORT_SHIFT)  /* GPIO1 */
#  define GPIO_PORT2           (GPIO2 << GPIO_PORT_SHIFT)  /* GPIO2 */
#  define GPIO_PORT3           (GPIO3 << GPIO_PORT_SHIFT)  /* GPIO3 */
#  define GPIO_PORT4           (GPIO4 << GPIO_PORT_SHIFT)  /* GPIO4 */
#  define GPIO_PORT5           (GPIO5 << GPIO_PORT_SHIFT)  /* GPIO5 */
#  define GPIO_PORT6           (GPIO6 << GPIO_PORT_SHIFT)  /* GPIO6 */
#  define GPIO_PORT7           (GPIO7 << GPIO_PORT_SHIFT)  /* GPIO7 */
#  define GPIO_PORT8           (GPIO8 << GPIO_PORT_SHIFT)  /* GPIO8 */
#  define GPIO_PORT9           (GPIO9 << GPIO_PORT_SHIFT)  /* GPIO9 */
#  define GPIO_PORT10          (GPIO10 << GPIO_PORT_SHIFT) /* GPIO10 */
#  define GPIO_PORT11          (GPIO11 << GPIO_PORT_SHIFT) /* GPIO11 */
#  define GPIO_PORT12          (GPIO12 << GPIO_PORT_SHIFT) /* GPIO12 */
#  define GPIO_PORT13          (GPIO13 << GPIO_PORT_SHIFT) /* GPIO13 */

/* GPIO Pin Number:
 *
 *               1111 1100 0000 0000
 *               5432 1098 7654 3210
 *   GPIO IN/OUT .... .... ...P PPPP
 */

#define GPIO_PIN_SHIFT         (0)      /* Bits 0-4: GPIO pin number */
#define GPIO_PIN_MASK          (0x1f << GPIO_PIN_SHIFT)
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

/* Port access via global LUT */

#define IMX9_GPIO_BASE(n)      g_gpio_base[n]  /* Use GPIO1..GPIOn macros as indices */

#define IMX9_GPIO_VERID(n)     (IMX9_GPIO_BASE(n) + IMX9_GPIO_VERID_OFFSET)
#define IMX9_GPIO_PARAM(n)     (IMX9_GPIO_BASE(n) + IMX9_GPIO_PARAM_OFFSET)
#define IMX9_GPIO_LOCK(n)      (IMX9_GPIO_BASE(n) + IMX9_GPIO_LOCK_OFFSET)
#define IMX9_GPIO_PCNS(n)      (IMX9_GPIO_BASE(n) + IMX9_GPIO_PCNS_OFFSET)
#define IMX9_GPIO_ICNS(n)      (IMX9_GPIO_BASE(n) + IMX9_GPIO_ICNS_OFFSET)
#define IMX9_GPIO_PCNP(n)      (IMX9_GPIO_BASE(n) + IMX9_GPIO_PCNP_OFFSET)
#define IMX9_GPIO_ICNP(n)      (IMX9_GPIO_BASE(n) + IMX9_GPIO_ICNP_OFFSET)
#define IMX9_GPIO_PDOR(n)      (IMX9_GPIO_BASE(n) + IMX9_GPIO_PDOR_OFFSET)
#define IMX9_GPIO_PSOR(n)      (IMX9_GPIO_BASE(n) + IMX9_GPIO_PSOR_OFFSET)
#define IMX9_GPIO_PCOR(n)      (IMX9_GPIO_BASE(n) + IMX9_GPIO_PCOR_OFFSET)
#define IMX9_GPIO_PTOR(n)      (IMX9_GPIO_BASE(n) + IMX9_GPIO_PTOR_OFFSET)
#define IMX9_GPIO_PDIR(n)      (IMX9_GPIO_BASE(n) + IMX9_GPIO_PDIR_OFFSET)
#define IMX9_GPIO_PDDR(n)      (IMX9_GPIO_BASE(n) + IMX9_GPIO_PDDR_OFFSET)
#define IMX9_GPIO_PIDR(n)      (IMX9_GPIO_BASE(n) + IMX9_GPIO_PIDR_OFFSET)
#define IMX9_GPIO_GICLR(n)     (IMX9_GPIO_BASE(n) + IMX9_GPIO_GICLR_OFFSET)
#define IMX9_GPIO_GICHR(n)     (IMX9_GPIO_BASE(n) + IMX9_GPIO_GICHR_OFFSET)

/* Interrupt status flags, these have two channels. Channel is selected by
 * setting / clearing ICRN.IRQS bit.
 */

#define IMX9_GPIO_ISFR0(n)     (IMX9_GPIO_BASE(n) + IMX9_GPIO_ISFR0_OFFSET)
#define IMX9_GPIO_ISFR1(n)     (IMX9_GPIO_BASE(n) + IMX9_GPIO_ISFR1_OFFSET)

/* GPIO PIN[0...31] and ICR[0...31] */

#define IMX9_GPIO_P0DR(n)      (IMX9_GPIO_BASE(n) + IMX9_GPIO_P0DR_OFFSET)
#define IMX9_GPIO_PNDR(n, p)   (IMX9_GPIO_P0DR(n) + ((p) * 0x4))
#define IMX9_GPIO_ICR0(n)      (IMX9_GPIO_BASE(n) + IMX9_GPIO_ICR0_OFFSET)
#define IMX9_GPIO_ICRN(n, p)   (IMX9_GPIO_ICR0(n) + ((p) * 0x4))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The smallest integer type that can hold the GPIO encoding */

typedef uint16_t gpio_pinset_t;

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

/* Look-up table that maps GPIO1..GPIOn indexes into GPIO register base
 * addresses
 */

EXTERN const uintptr_t g_gpio_base[];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_gpioirq_initialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_GPIO_IRQ
void imx9_gpioirq_initialize(void);
#else
#  define imx9_gpioirq_initialize()
#endif

/****************************************************************************
 * Name: imx9_config_gpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int imx9_config_gpio(gpio_pinset_t pinset);

/****************************************************************************
 * Name: imx9_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void imx9_gpio_write(gpio_pinset_t pinset, bool value);

/****************************************************************************
 * Name: imx9_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool imx9_gpio_read(gpio_pinset_t pinset);

/****************************************************************************
 * Name: imx9_gpioirq_attach
 *
 * Description:
 *   Attach a pin interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_GPIO_IRQ
int imx9_gpioirq_attach(gpio_pinset_t pinset, xcpt_t isr, void *arg);
#else
#define imx9_gpioirq_attach(pinset, isr, arg) 0
#endif

/****************************************************************************
 * Name: imx9_gpioirq_configure
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_GPIO_IRQ
int imx9_gpioirq_configure(gpio_pinset_t pinset);
#else
#  define imx9_gpioirq_configure(pinset) 0
#endif

/****************************************************************************
 * Name: imx9_gpioirq_enable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_GPIO_IRQ
int imx9_gpioirq_enable(gpio_pinset_t pinset);
#else
#  define imx9_gpioirq_enable(pinset) 0
#endif

/****************************************************************************
 * Name: imx9_gpioirq_disable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_GPIO_IRQ
int imx9_gpioirq_disable(gpio_pinset_t pinset);
#else
#  define imx9_gpioirq_disable(pinset) 0
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_GPIO_H */
