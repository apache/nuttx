/************************************************************************************
 * arch/arm/src/imxrt/imxrt_gpio.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_GPIO_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"
#include "hardware/imxrt_gpio.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* 32-bit Encoding:
 *
 *               3322 2222 2222 1111  1111 1100 0000 0000
 *               1098 7654 3210 9876  5432 1098 7654 3210
 *   ENCODING    IIXX XXXX XXXX XXXX  MMMM MMMM MMMM MMMM
 *   GPIO INPUT  00.. BEEG GGGP PPPP  MMMM MMMM MMMM MMMM
 *   INT INPUT   11.. BEEG GGGP PPPP  MMMM MMMM MMMM MMMM
 *   GPIO OUTPUT 01V. ..SG GGGP PPPP  MMMM MMMM MMMM MMMM
 *   PERIPHERAL  10AA AAS. IIII IIII  MMMM MMMM MMMM MMMM
 */

/* Input/Output Selection:
 *
 *               3322 2222 2222 1111  1111 1100 0000 0000
 *               1098 7654 3210 9876  5432 1098 7654 3210
 *   ENCODING    II.. .... .... ....  .... .... .... ....
 */

#define GPIO_MODE_SHIFT        (30)      /* Bits 30-31: Pin mode */
#define GPIO_MODE_MASK         (3 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT           (0 << GPIO_MODE_SHIFT) /* GPIO input */
#  define GPIO_OUTPUT          (1 << GPIO_MODE_SHIFT) /* GPIO output */
#  define GPIO_PERIPH          (2 << GPIO_MODE_SHIFT) /* Peripheral */
#  define GPIO_INTERRUPT       (3 << GPIO_MODE_SHIFT) /* Interrupt input */

/* Initial Output Value:
 *
 *   GPIO OUTPUT 01V. .... .... ....  .... .... .... ....
 */

#define GPIO_OUTPUT_ZERO       (0)       /* Bit 29: 0=Initial output is low */
#define GPIO_OUTPUT_ONE        (1 << 29) /* Bit 29: 1=Initial output is high */

/* GPIO Port Number
 *
 *               3322 2222 2222 1111  1111 1100 0000 0000
 *               1098 7654 3210 9876  5432 1098 7654 3210
 *   GPIO INPUT  00.. ...G GGG. ....  .... .... .... ....
 *   GPIO OUTPUT 01.. ...G GGG. ....  .... .... .... ....
 */

#define GPIO_PORT_SHIFT        (21)      /* Bits 21-23: GPIO port index */
#define GPIO_PORT_MASK         (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORT1           (GPIO1 << GPIO_PORT_SHIFT) /* GPIO1 */
#  define GPIO_PORT2           (GPIO2 << GPIO_PORT_SHIFT) /* GPIO2 */
#  define GPIO_PORT3           (GPIO3 << GPIO_PORT_SHIFT) /* GPIO3 */
#  define GPIO_PORT4           (GPIO4 << GPIO_PORT_SHIFT) /* GPIO4 */
#  define GPIO_PORT5           (GPIO5 << GPIO_PORT_SHIFT) /* GPIO5 */
#if IMXRT_GPIO_NPORTS > 5
#  define GPIO_PORT6           (GPIO6 << GPIO_PORT_SHIFT) /* GPIO6 */
#  define GPIO_PORT7           (GPIO7 << GPIO_PORT_SHIFT) /* GPIO7 */
#  define GPIO_PORT8           (GPIO8 << GPIO_PORT_SHIFT) /* GPIO8 */
#  define GPIO_PORT9           (GPIO9 << GPIO_PORT_SHIFT) /* GPIO9 */
#endif
/* GPIO Pin Number:
 *
 *               3322 2222 2222 1111  1111 1100 0000 0000
 *               1098 7654 3210 9876  5432 1098 7654 3210
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

/* Peripheral Alternate Function:
 *
 *               3322 2222 2222 1111  1111 1100 0000 0000
 *               1098 7654 3210 9876  5432 1098 7654 3210
 *   PERIPHERAL  ..AA AA.. .... ....  .... .... .... ....
 */

#define GPIO_ALT_SHIFT         (26)      /* Bits 26-29: Peripheral alternate function */
#define GPIO_ALT_MASK          (0xf << GPIO_ALT_SHIFT)
#  define GPIO_ALT0            (0 << GPIO_ALT_SHIFT)  /* Alternate function 0 */
#  define GPIO_ALT1            (1 << GPIO_ALT_SHIFT)  /* Alternate function 1 */
#  define GPIO_ALT2            (2 << GPIO_ALT_SHIFT)  /* Alternate function 2 */
#  define GPIO_ALT3            (3 << GPIO_ALT_SHIFT)  /* Alternate function 3 */
#  define GPIO_ALT4            (4 << GPIO_ALT_SHIFT)  /* Alternate function 4 */
#  define GPIO_ALT5            (5 << GPIO_ALT_SHIFT)  /* Alternate function 5 is GPIO */
#  define GPIO_ALT6            (6 << GPIO_ALT_SHIFT)  /* Alternate function 6 */
#  define GPIO_ALT7            (7 << GPIO_ALT_SHIFT)  /* Alternate function 7 */
#  define GPIO_ALT8            (8 << GPIO_ALT_SHIFT)  /* Alternate function 8 */
#  define GPIO_ALT9            (9 << GPIO_ALT_SHIFT)  /* Alternate function 9 */

/* Peripheral Software Input On Field:
 *
 *               3322 2222 2222 1111  1111 1100 0000 0000
 *               1098 7654 3210 9876  5432 1098 7654 3210
 *   PERIPHERAL  .... ..S. .... ....  .... .... .... ....
 */

#define GPIO_SION_SHIFT        (25)      /* Bits 25: Peripheral SION function */
#define GPIO_SION_MASK         (1 << GPIO_SION_SHIFT)
#  define GPIO_SION_ENABLE     (1 << GPIO_SION_SHIFT)  /* enable SION */

/* Interrupt edge/level configuration
 *
 *               3322 2222 2222 1111  1111 1100 0000 0000
 *               1098 7654 3210 9876  5432 1098 7654 3210
 *   INT INPUT   11.. .EE. .... ....  .... .... .... ....
 */

#define GPIO_INTCFG_SHIFT      (25)      /* Bits 25-26: Interrupt edge/level configuration */
#define GPIO_INTCFG_MASK       (3 << GPIO_INTCFG_SHIFT)
#  define GPIO_INT_LOWLEVEL    (GPIO_ICR_LOWLEVEL << GPIO_INTCFG_SHIFT)
#  define GPIO_INT_HIGHLEVEL   (GPIO_ICR_HIGHLEVEL << GPIO_INTCFG_SHIFT)
#  define GPIO_INT_RISINGEDGE  (GPIO_ICR_RISINGEDGE << GPIO_INTCFG_SHIFT)
#  define GPIO_INT_FALLINGEDGE (GPIO_ICR_FALLINGEDGE << GPIO_INTCFG_SHIFT)

/* Interrupt on both edges configuration
 *
 *               3322 2222 2222 1111  1111 1100 0000 0000
 *               1098 7654 3210 9876  5432 1098 7654 3210
 *   INT INPUT   11.. B... .... ....  .... .... .... ....
 */

#define GPIO_INTBOTHCFG_SHIFT      (27)      /* Bit 27: Interrupt both edges configuration */
#define GPIO_INTBOTHCFG_MASK       (1 << GPIO_INTBOTHCFG_SHIFT)
#  define GPIO_INTBOTH_EDGES       (1 << GPIO_INTBOTHCFG_SHIFT)

/* Pad Mux Register Index:
 *
 *               3322 2222 2222 1111  1111 1100 0000 0000
 *               1098 7654 3210 9876  5432 1098 7654 3210
 *   PERIPHERAL  .... .... IIII IIII  .... .... .... ....
 */

#define GPIO_PADMUX_SHIFT      (16)      /* Bits 16-23: Peripheral alternate function */
#define GPIO_PADMUX_MASK       (0xff << GPIO_PADMUX_SHIFT)
#  define GPIO_PADMUX(n)       ((uint32_t)(n) << GPIO_PADMUX_SHIFT)
#define GPIO_PADMUX_GET(n)     ((n&GPIO_PADMUX_MASK)>>GPIO_PADMUX_SHIFT)

/* IOMUX Pin Configuration:
 *
 *               3322 2222 2222 1111  1111 1100 0000 0000
 *               1098 7654 3210 9876  5432 1098 7654 3210
 *   ENCODING    .... .... .... ....  MMMM MMMM MMMM MMMM
 *
 * See imxrt_iomuxc.h for detailed content.
 */

#define GPIO_IOMUX_SHIFT       (0)       /* Bits 0-15: IOMUX pin configuration */
#define GPIO_IOMUX_MASK        (0xffff << GPIO_IOMUX_SHIFT)

/* Helper addressing macros */

#define IMXRT_GPIO_BASE(n)       g_gpio_base[n]  /* Use GPIO1..GPIOn macros as indices */

#define IMXRT_GPIO_DR(n)         (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_DR_OFFSET)
#define IMXRT_GPIO_GDIR(n)       (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_GDIR_OFFSET)
#define IMXRT_GPIO_PSR(n)        (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_PSR_OFFSET)
#define IMXRT_GPIO_ICR1(n)       (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_ICR1_OFFSET)
#define IMXRT_GPIO_ICR2(n)       (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_ICR2_OFFSET)
#define IMXRT_GPIO_IMR(n)        (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_IMR_OFFSET)
#define IMXRT_GPIO_ISR(n)        (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_ISR_OFFSET)
#define IMXRT_GPIO_EDGE(n)       (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_EDGE_OFFSET)
#define IMXRT_GPIO_SET(n)        (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_SET_OFFSET)
#define IMXRT_GPIO_CLEAR(n)      (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_CLEAR_OFFSET)
#define IMXRT_GPIO_TOGGLE(n)     (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_TOGGLE_OFFSET)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* The smallest integer type that can hold the GPIO encoding */

typedef uint32_t gpio_pinset_t;

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Look-up table that maps GPIO1..GPIOn indexes into GPIO register base addresses */

EXTERN const uintptr_t g_gpio_base[IMXRT_GPIO_NPORTS];

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: imxrt_gpioirq_initialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for GPIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_IMXRT_GPIO_IRQ
void imxrt_gpioirq_initialize(void);
#else
#  define imxrt_gpioirq_initialize()
#endif

/************************************************************************************
 * Name: imxrt_config_gpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int imxrt_config_gpio(gpio_pinset_t pinset);

/************************************************************************************
 * Name: imxrt_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void imxrt_gpio_write(gpio_pinset_t pinset, bool value);

/************************************************************************************
 * Name: imxrt_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool imxrt_gpio_read(gpio_pinset_t pinset);

/************************************************************************************
 * Name: imxrt_gpioirq_configure
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ************************************************************************************/

#ifdef CONFIG_IMXRT_GPIO_IRQ
int imxrt_gpioirq_configure(gpio_pinset_t pinset);
#else
#  define imxrt_gpioirq_configure(pinset)
#endif

/************************************************************************************
 * Name: imxrt_gpioirq_enable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_IMXRT_GPIO_IRQ
int imxrt_gpioirq_enable(int irq);
#else
#  define imxrt_gpioirq_enable(irq)
#endif

/************************************************************************************
 * Name: imxrt_gpioirq_disable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_IMXRT_GPIO_IRQ
int imxrt_gpioirq_disable(int irq);
#else
#  define imxrt_gpioirq_disable(irq)
#endif

/************************************************************************************
 * Function:  imxrt_dump_gpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int imxrt_dump_gpio(uint32_t pinset, const char *msg);
#else
#  define imxrt_dumpgpio(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_GPIO_H */
