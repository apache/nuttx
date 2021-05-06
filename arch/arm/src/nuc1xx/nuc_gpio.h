/****************************************************************************
 * arch/arm/src/nuc1xx/nuc_gpio.h
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

#ifndef __ARCH_ARM_SRC_NUC1XX_NUC_GPIO_H
#define __ARCH_ARM_SRC_NUC1XX_NUC_GPIO_H

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
#include "hardware/nuc_gpio.h"

/****************************************************************************
 * Pre-processor Declarations
 ****************************************************************************/

/* Bit-encoded input to nuc_configgpio() */

/* 16-bit Encoding:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * MMAD III. VPPP BBBB
 */

/* GPIO mode:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * MM.. .... .... ....
 */

#define GPIO_MODE_SHIFT               (14)      /* Bits 14-15: GPIO mode */
#define GPIO_MODE_MASK                (3 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT                  (0 << GPIO_MODE_SHIFT) /* Input */
#  define GPIO_OUTPUT                 (1 << GPIO_MODE_SHIFT) /* Push-pull output */
#  define GPIO_OPENDRAIN              (2 << GPIO_MODE_SHIFT) /* Open drain output */
#  define GPIO_BIDI                   (3 << GPIO_MODE_SHIFT) /* Quasi bi-directional */

/* GPIO analog: If the pin is an analog input, then it would be necessary to
 * disable the digital input
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * ..A. .... .... ....
 */

#define GPIO_ANALOG                   (1 << 13) /* Bit 13: Disable digital input */

/* De-bounce enable:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * ...D .... .... ....
 */

#define GPIO_DEBOUNCE                 (1 << 12) /* Bit 12: Debounce enable */

/* Interrupt Controls:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... III. .... ....
 */

#define GPIO_INTERRUPT_SHIFT          (9)       /* Bits 9-11:  Interrupt controls */
#define GPIO_INTERRUPT_MASK           (7 << GPIO_INTERRUPT_SHIFT)
#  define GPIO_INTERRUPT_NONE         (0 << GPIO_INTERRUPT_SHIFT)
#  define GPIO_INTERRUPT_RISING_EDGE  (1 << GPIO_INTERRUPT_SHIFT)
#  define GPIO_INTERRUPT_FALLING_EDGE (2 << GPIO_INTERRUPT_SHIFT)
#  define GPIO_INTERRUPT_BOTH_EDGES   (3 << GPIO_INTERRUPT_SHIFT)
#  define GPIO_INTERRUPT_HIGH_LEVEL   (4 << GPIO_INTERRUPT_SHIFT)
#  define GPIO_INTERRUPT_LOW_LEVEL    (5 << GPIO_INTERRUPT_SHIFT)

/* If the pin is a GPIO digital output, then this identifies the initial
 * output value.
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... V... ....
 */

#define GPIO_OUTPUT_SET               (1 << 7)                   /* Bit 7: If output, initial value of output */
#define GPIO_OUTPUT_CLEAR             (0)

/* This identifies the GPIO port:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... .PPP ....
 */

#define GPIO_PORT_SHIFT               4                          /* Bit 4-6:  Port number */
#define GPIO_PORT_MASK                (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA                  (0 << GPIO_PORT_SHIFT)     /*   GPIOA */
#  define GPIO_PORTB                  (1 << GPIO_PORT_SHIFT)     /*   GPIOB */
#  define GPIO_PORTC                  (2 << GPIO_PORT_SHIFT)     /*   GPIOC */
#  define GPIO_PORTD                  (3 << GPIO_PORT_SHIFT)     /*   GPIOD */
#  define GPIO_PORTE                  (4 << GPIO_PORT_SHIFT)     /*   GPIOE */
#

/* This identifies the bit in the port:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... .... BBBB
 */

#define GPIO_PIN_SHIFT                0                          /* Bits 0-3: GPIO number: 0-15 */
#define GPIO_PIN_MASK                 (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN0                   (0 << GPIO_PIN_SHIFT)
#  define GPIO_PIN1                   (1 << GPIO_PIN_SHIFT)
#  define GPIO_PIN2                   (2 << GPIO_PIN_SHIFT)
#  define GPIO_PIN3                   (3 << GPIO_PIN_SHIFT)
#  define GPIO_PIN4                   (4 << GPIO_PIN_SHIFT)
#  define GPIO_PIN5                   (5 << GPIO_PIN_SHIFT)
#  define GPIO_PIN6                   (6 << GPIO_PIN_SHIFT)
#  define GPIO_PIN7                   (7 << GPIO_PIN_SHIFT)
#  define GPIO_PIN8                   (8 << GPIO_PIN_SHIFT)
#  define GPIO_PIN9                   (9 << GPIO_PIN_SHIFT)
#  define GPIO_PIN10                  (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11                  (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12                  (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13                  (13 << GPIO_PIN_SHIFT)
#  define GPIO_PIN14                  (14 << GPIO_PIN_SHIFT)
#  define GPIO_PIN15                  (15 << GPIO_PIN_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint16_t gpio_cfgset_t;

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
 * Name: nuc_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...)
 *   function, it must be unconfigured with nuc_unconfiggpio() with
 *   the same cfgset first before it can be set to non-alternative function.
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port, or when pin is locked as ALT function.
 *
 ****************************************************************************/

int nuc_configgpio(gpio_cfgset_t cfgset);

/****************************************************************************
 * Name: nuc_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void nuc_gpiowrite(gpio_cfgset_t pinset, bool value);

/****************************************************************************
 * Name: nuc_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool nuc_gpioread(gpio_cfgset_t pinset);

/****************************************************************************
 * Function:  nuc_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided pin description
 *   along with a descriptive message.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
void nuc_dumpgpio(gpio_cfgset_t pinset, const char *msg);
#else
#  define nuc_dumpgpio(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_NUC1XX_NUC_GPIO_H */
