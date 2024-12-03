/****************************************************************************
 * arch/risc-v/src/hpm6000/hpm_gpio.h
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

#ifndef __ARCH_RISCV_SRC_HPM6000_HPM_GPIO_H
#define __ARCH_RISCV_SRC_HPM6000_HPM_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#include <stdint.h>
#include <stdbool.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "chip.h"
#include "hardware/hpm_gpio.h"

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

/* Loopback On Field:
 *
 *               3322 2222 2222 1111  1111 1100 0000 0000
 *               1098 7654 3210 9876  5432 1098 7654 3210
 *   PERIPHERAL  .... ..S. .... ....  .... .... .... ....
 */

#define GPIO_LOOP_SHIFT        (24)      /* Bits 24: Peripheral SION function */
#define GPIO_LOOP_MASK         (1 << GPIO_LOOP_SHIFT)
#  define GPIO_LOOP_ENABLE     (1 << GPIO_LOOP_SHIFT)  /* enable SION */

/* Interrupt edge/level configuration
 *
 *               3322 2222 2222 1111  1111 1100 0000 0000
 *               1098 7654 3210 9876  5432 1098 7654 3210
 *   INT INPUT   11.. .EE. .... ....  .... .... .... ....
 */

#define GPIO_INTCFG_SHIFT      (24)      /* Bits 24-25: Interrupt edge/level configuration */
#define GPIO_INTCFG_MASK       (3 << GPIO_INTCFG_SHIFT)
#  define GPIO_INT_LOWLEVEL    (GPIO_ICR_LOWLEVEL << GPIO_INTCFG_SHIFT)
#  define GPIO_INT_HIGHLEVEL   (GPIO_ICR_HIGHLEVEL << GPIO_INTCFG_SHIFT)
#  define GPIO_INT_RISINGEDGE  (GPIO_ICR_RISINGEDGE << GPIO_INTCFG_SHIFT)
#  define GPIO_INT_FALLINGEDGE (GPIO_ICR_FALLINGEDGE << GPIO_INTCFG_SHIFT)

/* GPIO Port Number
 *
 *               3322 2222 2222 1111  1111 1100 0000 0000
 *               1098 7654 3210 9876  5432 1098 7654 3210
 *   GPIO INPUT  00.. ...G GGG. ....  .... .... .... ....
 *   GPIO OUTPUT 01.. ...G GGG. ....  .... .... .... ....
 */

#define GPIO_PORT_SHIFT        (21)      /* Bits 21-23: GPIO port index */
#define GPIO_PORT_MASK         (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA           (GPIOA << GPIO_PORT_SHIFT) /* GPIO1 */
#  define GPIO_PORTB           (GPIOB << GPIO_PORT_SHIFT) /* GPIO2 */
#  define GPIO_PORTC           (GPIOC << GPIO_PORT_SHIFT) /* GPIO3 */
#  define GPIO_PORTD           (GPIOD << GPIO_PORT_SHIFT) /* GPIO3 */
#  define GPIO_PORTE           (GPIOE << GPIO_PORT_SHIFT) /* GPIO3 */
#  define GPIO_PORTF           (GPIOF << GPIO_PORT_SHIFT) /* GPIO3 */
#  define GPIO_PORTX           (GPIOX << GPIO_PORT_SHIFT) /* GPIO4 */
#  define GPIO_PORTY           (GPIOY << GPIO_PORT_SHIFT) /* GPIO5 */
#  define GPIO_PORTZ           (GPIOZ << GPIO_PORT_SHIFT) /* GPIOZ */

/* GPIO Pin Number:
 *
 *               3322 2222 2222 1111  1111 1100 0000 0000
 *               1098 7654 3210 9876  5432 1098 7654 3210
 *   GPIO INPUT  00.. .... ...P PPPP  .... .... .... ....
 *   GPIO OUTPUT 01.. .... ...P PPPP  .... .... .... ....
 */

#define GPIO_PIN_SHIFT (16) /* Bits 0-4: GPIO number: 0-31 */
#define GPIO_PIN_MASK  (0x1f << GPIO_PIN_SHIFT)
#define GPIO_PIN0      (0 << GPIO_PIN_SHIFT)
#define GPIO_PIN1      (1 << GPIO_PIN_SHIFT)
#define GPIO_PIN2      (2 << GPIO_PIN_SHIFT)
#define GPIO_PIN3      (3 << GPIO_PIN_SHIFT)
#define GPIO_PIN4      (4 << GPIO_PIN_SHIFT)
#define GPIO_PIN5      (5 << GPIO_PIN_SHIFT)
#define GPIO_PIN6      (6 << GPIO_PIN_SHIFT)
#define GPIO_PIN7      (7 << GPIO_PIN_SHIFT)
#define GPIO_PIN8      (8 << GPIO_PIN_SHIFT)
#define GPIO_PIN9      (9 << GPIO_PIN_SHIFT)
#define GPIO_PIN10     (10 << GPIO_PIN_SHIFT)
#define GPIO_PIN11     (11 << GPIO_PIN_SHIFT)
#define GPIO_PIN12     (12 << GPIO_PIN_SHIFT)
#define GPIO_PIN13     (13 << GPIO_PIN_SHIFT)
#define GPIO_PIN14     (14 << GPIO_PIN_SHIFT)
#define GPIO_PIN15     (15 << GPIO_PIN_SHIFT)
#define GPIO_PIN16     (16 << GPIO_PIN_SHIFT)
#define GPIO_PIN17     (17 << GPIO_PIN_SHIFT)
#define GPIO_PIN18     (18 << GPIO_PIN_SHIFT)
#define GPIO_PIN19     (19 << GPIO_PIN_SHIFT)
#define GPIO_PIN20     (20 << GPIO_PIN_SHIFT)
#define GPIO_PIN21     (21 << GPIO_PIN_SHIFT)
#define GPIO_PIN22     (22 << GPIO_PIN_SHIFT)
#define GPIO_PIN23     (23 << GPIO_PIN_SHIFT)
#define GPIO_PIN24     (24 << GPIO_PIN_SHIFT)
#define GPIO_PIN25     (25 << GPIO_PIN_SHIFT)
#define GPIO_PIN26     (26 << GPIO_PIN_SHIFT)
#define GPIO_PIN27     (27 << GPIO_PIN_SHIFT)
#define GPIO_PIN28     (28 << GPIO_PIN_SHIFT)
#define GPIO_PIN29     (29 << GPIO_PIN_SHIFT)
#define GPIO_PIN30     (30 << GPIO_PIN_SHIFT)
#define GPIO_PIN31     (31 << GPIO_PIN_SHIFT)

/* Pad Mux Register Index:
 *
 *               3322 2222 2222 1111  1111 1100 0000 0000
 *               1098 7654 3210 9876  5432 1098 7654 3210
 *   PERIPHERAL  .... .... IIII IIII  .... .... .... ....
 */

#define GPIO_PADMUX_SHIFT   (16)
#define GPIO_PADMUX_MASK    (0xfff << GPIO_PADMUX_SHIFT)
#  define GPIO_PADMUX(n)    ((uint32_t)(n) << GPIO_PADMUX_SHIFT)
#define GPIO_PADMUX_GET(n)  ((n&GPIO_PADMUX_MASK) >> GPIO_PADMUX_SHIFT)

/* IOC PAD CTL Configuration
 *
 *               3322 2222 2222 1111  1111 1100 0000 0000
 *               1098 7654 3210 9876  5432 1098 7654 3210
 *   ENCODING    .... .... .... ....  MMMM MMMM MMMM MMMM
 *
 */

#define GPIO_IOCPAD_SHIFT   (0)
#define GPIO_IOCPAD_MASK    (0xffff << GPIO_IOCPAD_SHIFT)

#define HPM_GPIO_GPIOA      (0)
#define HPM_GPIO_GPIOB      (1)
#define HPM_GPIO_GPIOC      (2)
#define HPM_GPIO_GPIOX      (3)
#define HPM_GPIO_GPIOY      (4)
#define HPM_GPIO_GPIOZ      (5)

#define HPM_GPIO_DI_VAL(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0000)
#define HPM_GPIO_DI_SET(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0004)
#define HPM_GPIO_DI_CLR(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0008)
#define HPM_GPIO_DI_TOG(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x000c)

#define HPM_GPIO_DO_VAL(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0100)
#define HPM_GPIO_DO_SET(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0104)
#define HPM_GPIO_DO_CLR(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0108)
#define HPM_GPIO_DO_TOG(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x010c)

#define HPM_GPIO_OE_VAL(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0200)
#define HPM_GPIO_OE_SET(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0204)
#define HPM_GPIO_OE_CLR(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0208)
#define HPM_GPIO_OE_TOG(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x020c)

#define HPM_GPIO_IF_VAL(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0300)
#define HPM_GPIO_IF_SET(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0304)
#define HPM_GPIO_IF_CLR(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0308)
#define HPM_GPIO_IF_TOG(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x030c)

#define HPM_GPIO_IE_VAL(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0400)
#define HPM_GPIO_IE_SET(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0404)
#define HPM_GPIO_IE_CLR(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0408)
#define HPM_GPIO_IE_TOG(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x040c)

#define HPM_GPIO_PL_VAL(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0500)
#define HPM_GPIO_PL_SET(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0504)
#define HPM_GPIO_PL_CLR(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0508)
#define HPM_GPIO_PL_TOG(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x050c)

#define HPM_GPIO_TP_VAL(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0600)
#define HPM_GPIO_TP_SET(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0604)
#define HPM_GPIO_TP_CLR(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0608)
#define HPM_GPIO_TP_TOG(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x060c)

#define HPM_GPIO_AS_VAL(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0700)
#define HPM_GPIO_AS_SET(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0704)
#define HPM_GPIO_AS_CLR(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x0708)
#define HPM_GPIO_AS_TOG(n)  (HPM_GPIO0_BASE + (n) * 0x10 + 0x070c)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The smallest integer type that can hold the GPIO encoding */

typedef uint32_t gpio_pinset_t;

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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port.
 *
 ****************************************************************************/

int hpm_gpio_config(gpio_pinset_t pinset);

/****************************************************************************
 * Name: mpfs_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void hpm_gpio_write(gpio_pinset_t pinset, bool value);

/****************************************************************************
 * Name: mpfs_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool hpm_gpio_read(gpio_pinset_t pinset);

/****************************************************************************
 * Name: mpfs_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - risingedge:  Enables interrupt on rising edges
 *  - fallingedge: Enables interrupt on falling edges
 *  - high:        Enables interrupt on level high
 *  - low:         Enables interrupt on level low
 *  - event:       Generate event when set
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int hpm_gpiosetevent(gpio_pinset_t pinset, bool risingedge,
                      bool fallingedge, bool high, bool low, bool event,
                      xcpt_t func, void *arg);

/****************************************************************************
 * Name: mpfs_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int hpm_gpio_initialize(void);

/****************************************************************************
 * Function:  mpfs_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int hpm_dumpgpio(gpio_pinset_t pinset, const char *msg);
#else
#define hpm_dumpgpio(p, m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_HPM6000_HPM_GPIO_H */
