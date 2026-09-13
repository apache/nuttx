/****************************************************************************
 * arch/arm/src/imxrt/imxrt_rgpio.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_RGPIO_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_RGPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"
#include "hardware/rt118x/imxrt118x_gpio.h"
#include "imxrt_iomuxc_ver3.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 16-bit gpio pinset bit layout (lives in bits 48..63 of imxrt_pinset_t):
 *
 *               1111 1100 0000 0000
 *               5432 1098 7654 3210
 *   ENCODING    MMVX BEEG GGGP PPPP
 *   GPIO INPUT  00.. BEEG GGGP PPPP
 *   INT INPUT   11.. BEEG GGGP PPPP
 *   GPIO OUTPUT 01V. ...G GGGP PPPP
 */

/* Input/Output Selection: bits 14-15 */

#define GPIO_MODE_SHIFT        (14)
#define GPIO_MODE_MASK         (0x3u << GPIO_MODE_SHIFT)
#  define GPIO_INPUT           (0u << GPIO_MODE_SHIFT)
#  define GPIO_OUTPUT          (1u << GPIO_MODE_SHIFT)
#  define GPIO_INTERRUPT       (2u << GPIO_MODE_SHIFT)

/* Initial Output Value: bit 13 (only meaningful for GPIO_OUTPUT) */

#define GPIO_OUTPUT_SHIFT      (13)
#define GPIO_OUTPUT_MASK       (0x1u << GPIO_OUTPUT_SHIFT)
#  define GPIO_OUTPUT_ZERO     (0u << GPIO_OUTPUT_SHIFT)
#  define GPIO_OUTPUT_ONE      (1u << GPIO_OUTPUT_SHIFT)

/* Interrupt "both edges" flag: bit 11 (only for GPIO_INTERRUPT) */

#define GPIO_INTBOTHCFG_SHIFT  (11)
#define GPIO_INTBOTHCFG_MASK   (1u << GPIO_INTBOTHCFG_SHIFT)
#  define GPIO_INTBOTH_EDGES   (1u << GPIO_INTBOTHCFG_SHIFT)

/* Interrupt edge/level configuration: bits 9-10 (only for GPIO_INTERRUPT) */

#define GPIO_INTCFG_SHIFT      (9)
#define GPIO_INTCFG_MASK       (0x3u << GPIO_INTCFG_SHIFT)
#  define GPIO_INT_LOWLEVEL    (0u << GPIO_INTCFG_SHIFT)
#  define GPIO_INT_HIGHLEVEL   (1u << GPIO_INTCFG_SHIFT)
#  define GPIO_INT_RISINGEDGE  (2u << GPIO_INTCFG_SHIFT)
#  define GPIO_INT_FALLINGEDGE (3u << GPIO_INTCFG_SHIFT)

/* GPIO Port Number: bits 5-8 (GPIO1..GPIO6) */

#define GPIO_PORT_SHIFT        (5)
#define GPIO_PORT_MASK         (0xfu << GPIO_PORT_SHIFT)
#  define GPIO_PORT1           (GPIO1 << GPIO_PORT_SHIFT)
#  define GPIO_PORT2           (GPIO2 << GPIO_PORT_SHIFT)
#  define GPIO_PORT3           (GPIO3 << GPIO_PORT_SHIFT)
#  define GPIO_PORT4           (GPIO4 << GPIO_PORT_SHIFT)
#  define GPIO_PORT5           (GPIO5 << GPIO_PORT_SHIFT)
#  define GPIO_PORT6           (GPIO6 << GPIO_PORT_SHIFT)

/* GPIO Pin Number: bits 0-4 */

#define GPIO_PIN_SHIFT         (0)
#define GPIO_PIN_MASK          (0x1fu << GPIO_PIN_SHIFT)
#  define GPIO_PIN0            (0u  << GPIO_PIN_SHIFT)
#  define GPIO_PIN1            (1u  << GPIO_PIN_SHIFT)
#  define GPIO_PIN2            (2u  << GPIO_PIN_SHIFT)
#  define GPIO_PIN3            (3u  << GPIO_PIN_SHIFT)
#  define GPIO_PIN4            (4u  << GPIO_PIN_SHIFT)
#  define GPIO_PIN5            (5u  << GPIO_PIN_SHIFT)
#  define GPIO_PIN6            (6u  << GPIO_PIN_SHIFT)
#  define GPIO_PIN7            (7u  << GPIO_PIN_SHIFT)
#  define GPIO_PIN8            (8u  << GPIO_PIN_SHIFT)
#  define GPIO_PIN9            (9u  << GPIO_PIN_SHIFT)
#  define GPIO_PIN10           (10u << GPIO_PIN_SHIFT)
#  define GPIO_PIN11           (11u << GPIO_PIN_SHIFT)
#  define GPIO_PIN12           (12u << GPIO_PIN_SHIFT)
#  define GPIO_PIN13           (13u << GPIO_PIN_SHIFT)
#  define GPIO_PIN14           (14u << GPIO_PIN_SHIFT)
#  define GPIO_PIN15           (15u << GPIO_PIN_SHIFT)
#  define GPIO_PIN16           (16u << GPIO_PIN_SHIFT)
#  define GPIO_PIN17           (17u << GPIO_PIN_SHIFT)
#  define GPIO_PIN18           (18u << GPIO_PIN_SHIFT)
#  define GPIO_PIN19           (19u << GPIO_PIN_SHIFT)
#  define GPIO_PIN20           (20u << GPIO_PIN_SHIFT)
#  define GPIO_PIN21           (21u << GPIO_PIN_SHIFT)
#  define GPIO_PIN22           (22u << GPIO_PIN_SHIFT)
#  define GPIO_PIN23           (23u << GPIO_PIN_SHIFT)
#  define GPIO_PIN24           (24u << GPIO_PIN_SHIFT)
#  define GPIO_PIN25           (25u << GPIO_PIN_SHIFT)
#  define GPIO_PIN26           (26u << GPIO_PIN_SHIFT)
#  define GPIO_PIN27           (27u << GPIO_PIN_SHIFT)
#  define GPIO_PIN28           (28u << GPIO_PIN_SHIFT)
#  define GPIO_PIN29           (29u << GPIO_PIN_SHIFT)
#  define GPIO_PIN30           (30u << GPIO_PIN_SHIFT)
#  define GPIO_PIN31           (31u << GPIO_PIN_SHIFT)

/* Port access via global LUT */

#define IMXRT_GPIO_BASE(n)     g_gpio_base[n]

#define IMXRT_GPIO_VERID(n)    (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_VERID_OFFSET)
#define IMXRT_GPIO_PARAM(n)    (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_PARAM_OFFSET)
#define IMXRT_GPIO_LOCK(n)     (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_LOCK_OFFSET)
#define IMXRT_GPIO_PCNS(n)     (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_PCNS_OFFSET)
#define IMXRT_GPIO_ICNS(n)     (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_ICNS_OFFSET)
#define IMXRT_GPIO_PCNP(n)     (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_PCNP_OFFSET)
#define IMXRT_GPIO_ICNP(n)     (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_ICNP_OFFSET)
#define IMXRT_GPIO_PDOR(n)     (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_PDOR_OFFSET)
#define IMXRT_GPIO_PSOR(n)     (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_PSOR_OFFSET)
#define IMXRT_GPIO_PCOR(n)     (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_PCOR_OFFSET)
#define IMXRT_GPIO_PTOR(n)     (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_PTOR_OFFSET)
#define IMXRT_GPIO_PDIR(n)     (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_PDIR_OFFSET)
#define IMXRT_GPIO_PDDR(n)     (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_PDDR_OFFSET)
#define IMXRT_GPIO_PIDR(n)     (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_PIDR_OFFSET)
#define IMXRT_GPIO_GICLR(n)    (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_GICLR_OFFSET)
#define IMXRT_GPIO_GICHR(n)    (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_GICHR_OFFSET)

/* Interrupt status flag registers (two channels).  Channel selected via
 * ICRN.IRQS.
 */

#define IMXRT_GPIO_ISFR0(n)    (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_ISFR0_OFFSET)
#define IMXRT_GPIO_ISFR1(n)    (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_ISFR1_OFFSET)

/* Per-pin ICR and PDR at n*4 offsets */

#define IMXRT_GPIO_P0DR(n)     (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_P0DR_OFFSET)
#define IMXRT_GPIO_PNDR(n, p)  (IMXRT_GPIO_P0DR(n) + ((p) * 0x4))
#define IMXRT_GPIO_ICR0(n)     (IMXRT_GPIO_BASE(n) + IMXRT_GPIO_ICR0_OFFSET)
#define IMXRT_GPIO_ICRN(n, p)  (IMXRT_GPIO_ICR0(n) + ((p) * 0x4))

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef imxrt_pinset_t gpio_pinset_t;

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

/* Look-up table that maps GPIO1..GPIOn indexes into RGPIO instance base
 * addresses.
 */

EXTERN const uintptr_t g_gpio_base[IMXRT_GPIO_NPORTS];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_gpioirq_initialize
 ****************************************************************************/

#ifdef CONFIG_IMXRT_GPIO_IRQ
void imxrt_gpioirq_initialize(void);
#else
#  define imxrt_gpioirq_initialize()
#endif

/****************************************************************************
 * Name: imxrt_config_gpio
 *
 * Description:
 *   Configure the pad routing and (for GPIO pins) the RGPIO direction,
 *   initial output value and interrupt configuration.
 *
 ****************************************************************************/

int imxrt_config_gpio(gpio_pinset_t pinset);

/****************************************************************************
 * Name: imxrt_gpio_write
 ****************************************************************************/

void imxrt_gpio_write(gpio_pinset_t pinset, bool value);

/****************************************************************************
 * Name: imxrt_gpio_read
 ****************************************************************************/

bool imxrt_gpio_read(gpio_pinset_t pinset);

/****************************************************************************
 * Name: imxrt_gpioirq_attach / imxrt_gpioirq_configure /
 *       imxrt_gpioirq_enable  / imxrt_gpioirq_disable
 ****************************************************************************/

#ifdef CONFIG_IMXRT_GPIO_IRQ
int imxrt_gpioirq_attach(gpio_pinset_t pinset, xcpt_t isr, void *arg);
int imxrt_gpioirq_configure(gpio_pinset_t pinset);
int imxrt_gpioirq_enable(gpio_pinset_t pinset);
int imxrt_gpioirq_disable(gpio_pinset_t pinset);
#else
#  define imxrt_gpioirq_attach(pinset, isr, arg) 0
#  define imxrt_gpioirq_configure(pinset)        0
#  define imxrt_gpioirq_enable(pinset)           0
#  define imxrt_gpioirq_disable(pinset)          0
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_RGPIO_H */
