/************************************************************************************
 * arch/arm/src/am335x/am335x_gpio.h
 *
 *   Copyright (C) 2018 Petro Karashchenko. All rights reserved.
 *   Author: Petro Karashchenko <petro.karashchneko@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_AM335X_AM335X_GPIO_H
#define __ARCH_ARM_SRC_AM335X_AM335X_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "hardware/am335x_scm.h"
#include "hardware/am335x_gpio.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* 32-bit Encoding:
 *
 *   ENCODING    IIXX XXXX XXXX XXXX  XXXX XXXX MMMM MMMM
 *   GPIO INPUT  00.. RRRR .GGP PPPP  .... .... MMMM MMMM
 *   GPIO OUTPUT 01V. .... .GGP PPPP  .... .... MMMM MMMM
 *   PERIPHERAL  10.. .... CCCC CCCC  .... .... MMMM MMMM
 */

/* Input/Output Selection:
 *
 *   ENCODING    II.. .... .... ....  .... .... .... ....
 */

#define GPIO_MODE_SHIFT        (30)      /* Bits 30-31: Pin mode */
#define GPIO_MODE_MASK         (3 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT           (0 << GPIO_MODE_SHIFT) /* GPIO input */
#  define GPIO_OUTPUT          (1 << GPIO_MODE_SHIFT) /* GPIO output */
#  define GPIO_PERIPH          (2 << GPIO_MODE_SHIFT) /* Peripheral */

/* Interrupt Detection Modes:
 *
 *  GPIO INPUT  00.. RRRR .GGP PPPP  .... .... MMMM MMMM
 */

#define GPIO_INT_SHIFT         (24)         /* Bits 24-27: GPIO interrupt bits */
#define GPIO_INT_MASK          (0xf << GPIO_INT_SHIFT)
#  define GPIO_INT_NONE        (0 << GPIO_INT_SHIFT)
#  define GPIO_INT_RISING      (1 << GPIO_INT_SHIFT)
#  define GPIO_INT_FALLING     (2 << GPIO_INT_SHIFT)
#  define GPIO_INT_BOTHEDGES   (GPIO_INT_RISING | GPIO_INT_FALLING)

#  define GPIO_INT_HIGHLEVEL   (4 << GPIO_INT_SHIFT)
#  define GPIO_INT_LOWLEVEL    (8 << GPIO_INT_SHIFT)
#  define GPIO_INT_LEVELBOTH   (GPIO_INT_HIGHLEVEL | GPIO_INT_LOWLEVEL)

/* Initial Output Value:
 *
 *   GPIO OUTPUT 01V. .... .... ....  .... .... .... ....
 */

#define GPIO_OUTPUT_ZERO       (0)       /* Bit 29: 0=Initial output is low */
#define GPIO_OUTPUT_ONE        (1 << 29) /* Bit 29: 1=Initial output is high */

/* GPIO Port Number
 *
 *   GPIO INPUT  00.. .... .GG. ....  .... .... .... ....
 *   GPIO OUTPUT 01.. .... .GG. ....  .... .... .... ....
 */

#define GPIO_PORT_SHIFT        (21)      /* Bits 21-22: GPIO port index */
#define GPIO_PORT_MASK         (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORT0           (0 << GPIO_PORT_SHIFT) /* GPIO0 */
#  define GPIO_PORT1           (1 << GPIO_PORT_SHIFT) /* GPIO1 */
#  define GPIO_PORT2           (2 << GPIO_PORT_SHIFT) /* GPIO2 */
#  define GPIO_PORT3           (3 << GPIO_PORT_SHIFT) /* GPIO3 */

/* GPIO Pin Number:
 *
 *   GPIO INPUT  00.. .... ...P PPPP  .... .... .... ....
 *   GPIO OUTPUT 01.. .... ...P PPPP  .... .... .... ....
 */

#define GPIO_PIN_SHIFT         (16)      /* Bits 16-20: GPIO pin number */
#define GPIO_PIN_MASK          (31 << GPIO_PIN_SHIFT)
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

/* Pad Control Register Index:
 *
 *   PERIPHERAL  10.. .... CCCC CCCC  .... .... MMMM MMMM
 */

#define GPIO_PADCTL_SHIFT      (16)      /* Bits 16-23: Peripheral control index */
#define GPIO_PADCTL_MASK       (0xff << GPIO_PADCTL_SHIFT)
#  define GPIO_PADCTL(n)       ((uint32_t)(n) << GPIO_PADCTL_SHIFT)

/* Pin Mux Configuration:
 *
 *   ENCODING    .... .... .... ....  .... .... MMMM MMMM
 *
 * See am335x_pinmux.h for detailed content.
 */

#define GPIO_PINMUX_SHIFT      (0)       /* Bits 0-8: PINMUX pin configuration */
#define GPIO_PINMUX_MASK       (0xff << GPIO_PINMUX_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* The smallest integer type that can hold the GPIO encoding */

typedef uint32_t gpio_pinset_t;

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

extern const uintptr_t g_gpiobase[AM335X_GPIO_NPORTS];
#define am335x_gpion_vbase(n) (g_gpiobase[(n)])

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: am335x_gpioirq_initialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for GPIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_AM335X_GPIO_IRQ
void am335x_gpioirq_initialize(void);
#else
#  define am335x_gpio_irqinitialize()
#endif

/************************************************************************************
 * Name: am335x_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int am335x_gpio_config(gpio_pinset_t pinset);

/************************************************************************************
 * Name: am335x_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void am335x_gpio_write(gpio_pinset_t pinset, bool value);

/************************************************************************************
 * Name: am335x_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool am335x_gpio_read(gpio_pinset_t pinset);

/************************************************************************************
 * Name: am335x_gpioirq
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ************************************************************************************/

#ifdef CONFIG_AM335X_GPIO_IRQ
void am335x_gpioirq(gpio_pinset_t pinset);
#else
#  define am335x_gpioirq(pinset)
#endif

/************************************************************************************
 * Name: am335x_gpioirq_enable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_AM335X_GPIO_IRQ
void am335x_gpioirq_enable(int irq);
#else
#  define am335x_gpioirq_enable(irq)
#endif

/************************************************************************************
 * Name: am335x_gpioirq_disable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_AM335X_GPIO_IRQ
void am335x_gpioirq_disable(int irq);
#else
#  define am335x_gpioirq_disable(irq)
#endif

/************************************************************************************
 * Function:  am335x_dump_gpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int am335x_dump_gpio(uint32_t pinset, const char *msg);
#else
#  define am335x_dump_gpio(p,m)
#endif

/************************************************************************************
 * Function:  am335x_periph_gpio
 *
 * Description:
 *   Return GPIO pinset that correspond to provided peripheral pinset.
 *
 ************************************************************************************/

gpio_pinset_t am335x_periph_gpio(gpio_pinset_t pinset);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ARCH_ARM_SRC_AM335X_AM335X_GPIO_H */
