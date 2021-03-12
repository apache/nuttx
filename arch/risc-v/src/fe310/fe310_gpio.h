/****************************************************************************
 * arch/risc-v/src/fe310/fe310_gpio.h
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

#ifndef __ARCH_RISCV_SRC_FE310_FE310_GPIO_H
#define __ARCH_RISCV_SRC_FE310_FE310_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include "riscv_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Input/Output mode
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * MM.. .... .... ....
 */

#define GPIO_MODE_SHIFT      (14)  /* Bits 14-15: Mode of the GPIO pin */
#define GPIO_MODE_MASK       (3 << GPIO_MODE_SHIFT)
#define GPIO_MODE_INPUT      (0 << GPIO_MODE_SHIFT) /* input */
#define GPIO_MODE_OUTPUT     (1 << GPIO_MODE_SHIFT) /* output */
#define GPIO_MODE_INIRQ      (2 << GPIO_MODE_SHIFT) /* input interrupt */
#define GPIO_MODE_OUTPUT_INV (3 << GPIO_MODE_SHIFT) /* output inverted */

/* Initial value (for GPIO outputs only)
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * ..V. .... .... ....
 */

#define GPIO_VALUE_SHIFT     (13)  /* Bits 13: Value of the GPIO pin */
#define GPIO_VALUE_MASK      (1 << GPIO_VALUE_SHIFT)
#define GPIO_VALUE_ZERO      (0 << GPIO_VALUE_SHIFT)
#define GPIO_VALUE_ONE       (1 << GPIO_VALUE_SHIFT)

/* GPIO IOF selection
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... XX.. .... ....
 */

#define GPIO_IOF_SHIFT       (10)  /* Bits 10-11: IOF_SEL */
#define GPIO_IOF_MASK        (3 << GPIO_IOF_SHIFT)
#define GPIO_IOF_GPIO        (0 << GPIO_IOF_SHIFT)  /* GPIO */
#define GPIO_IOF_0           (1 << GPIO_IOF_SHIFT)  /* IOF0 */
#define GPIO_IOF_1           (2 << GPIO_IOF_SHIFT)  /* IOF1 */

/* GPIO pull-up (NOTE: FE310 only supports pull-up)
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... ...U .... ....
 */

#define GPIO_PULL_SHIFT      (8)  /* Bits 8: Pull-up */
#define GPIO_PULL_MASK       (1 << GPIO_PULL_SHIFT)
#define GPIO_DEFAULT         (0 << GPIO_PULL_SHIFT)
#define GPIO_PULLUP          (1 << GPIO_PULL_SHIFT)

/* GPIO interrupt
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... III. ....
 */

#define GPIO_INT_SHIFT       (5)  /* Bits 5-7: Interrupt */
#define GPIO_INT_MASK        (7 << GPIO_INT_SHIFT)
#define GPIO_INT_RISE        (0 << GPIO_INT_SHIFT)
#define GPIO_INT_FALL        (1 << GPIO_INT_SHIFT)
#define GPIO_INT_BOTH        (2 << GPIO_INT_SHIFT) /* Both edges */
#define GPIO_INT_HIGH        (3 << GPIO_INT_SHIFT)
#define GPIO_INT_LOW         (4 << GPIO_INT_SHIFT)

/* GPIO Pin Number:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... ...P PPPP
 */

#define GPIO_PIN_SHIFT        (0)   /* Bits 0-4: Pin number */
#define GPIO_PIN_MASK         (31 << GPIO_PIN_SHIFT)
#define GPIO_PIN0             (0 << GPIO_PIN_SHIFT)
#define GPIO_PIN1             (1 << GPIO_PIN_SHIFT)
#define GPIO_PIN2             (2 << GPIO_PIN_SHIFT)
#define GPIO_PIN3             (3 << GPIO_PIN_SHIFT)
#define GPIO_PIN4             (4 << GPIO_PIN_SHIFT)
#define GPIO_PIN5             (5 << GPIO_PIN_SHIFT)
#define GPIO_PIN6             (6 << GPIO_PIN_SHIFT)
#define GPIO_PIN7             (7 << GPIO_PIN_SHIFT)
#define GPIO_PIN8             (8 << GPIO_PIN_SHIFT)
#define GPIO_PIN9             (9 << GPIO_PIN_SHIFT)
#define GPIO_PIN10            (10 << GPIO_PIN_SHIFT)
#define GPIO_PIN11            (11 << GPIO_PIN_SHIFT)
#define GPIO_PIN12            (12 << GPIO_PIN_SHIFT)
#define GPIO_PIN13            (13 << GPIO_PIN_SHIFT)
#define GPIO_PIN14            (14 << GPIO_PIN_SHIFT)
#define GPIO_PIN15            (15 << GPIO_PIN_SHIFT)
#define GPIO_PIN16            (16 << GPIO_PIN_SHIFT)
#define GPIO_PIN17            (17 << GPIO_PIN_SHIFT)
#define GPIO_PIN18            (18 << GPIO_PIN_SHIFT)
#define GPIO_PIN19            (19 << GPIO_PIN_SHIFT)
#define GPIO_PIN20            (20 << GPIO_PIN_SHIFT)
#define GPIO_PIN21            (21 << GPIO_PIN_SHIFT)
#define GPIO_PIN22            (22 << GPIO_PIN_SHIFT)
#define GPIO_PIN23            (23 << GPIO_PIN_SHIFT)

/****************************************************************************
 * Public Functions Prototypes
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
 * Name: fe310_gpio_config
 ****************************************************************************/

EXTERN int fe310_gpio_config(uint16_t gpiocfg);

/****************************************************************************
 * Name: fe310_gpio_write
 ****************************************************************************/

EXTERN void fe310_gpio_write(uint16_t gpiocfg, bool value);

/****************************************************************************
 * Name: fe310_gpio_read
 ****************************************************************************/

EXTERN bool fe310_gpio_read(uint16_t gpiocfg);

/****************************************************************************
 * Name: fe310_gpio_clearpending
 ****************************************************************************/

EXTERN void fe310_gpio_clearpending(uint32_t pin);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_FE310_FE310_GPIO_H */
