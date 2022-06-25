/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_gpio.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_MPFS_GPIO_H
#define __ARCH_RISCV_SRC_MPFS_MPFS_GPIO_H

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
#include "hardware/mpfs_gpio.h"

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/* Bit-encoded input to mpfs_configgpio() */

/* Each port bit of the general-purpose I/O (GPIO) ports can be individually
 * configured by software in several modes:
 *
 *  - Input
 *  - Output
 *  - Output with buffer enable
 *  - Output with buffer disable
 *  - Input with irq level high
 *  - Input with irq level low
 *  - Input with irq edge positive
 *  - Input with irq edge negative
 *  - Alternate Function IO (pad) mux
 *
 * 16-bit Encoding:       1111 1100 0000 0000
 *                        5432 1098 7654 3210
 *                        ---- ---- ---- ----
 *                        MMBI III. .bbP PPPP
 */

/* Mode:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * MM.. .... .... ....
 */

#define GPIO_EC_SHIFT        (20) /* Bits 20-31 Electrical Configuration */
#define GPIO_EC_MASK         (0xFFF << GPIO_EC_SHIFT)
#define GPIO_EC_PUPD_SHIFT   (30) /* Bit 30-31 Electrical Configuration PUPD */
#define GPIO_EC_PUPD_MASK    (3 << GPIO_EC_PUPD_SHIFT)
#define GPIO_EC_LOCKDN_SHIFT (29) /* Bit 29 Electrical Configuration Lockdn */
#define GPIO_EC_LOCKDN_MASK  (1 << GPIO_EC_LOCKDN_SHIFT)
#define GPIO_EC_ENHYST_SHIFT (28) /* Bit 28 Electrical Configuration Hyst */
#define GPIO_EC_ENHYST_MASK  (1 << GPIO_EC_ENHYST_SHIFT)
#define GPIO_CLAMP_SHIFT     (27) /* Bit 27 Electrical Configuration Clamp */
#define GPIO_EC_CLAMP_MASK   (1 << GPIO_CLAMP_SHIFT)
#define GPIO_EC_DRVSTR_SHIFT (23) /* Bit 23-26 Electrical Configuration drive strength */
#define GPIO_EC_DRVSTR_MASK  (0xF << GPIO_EC_SHIFT)
#define GPIO_EC_BUFM_SHIFT   (20) /* Bit 20-22 Electrical Configuration Buffer Mode*/
#define GPIO_EC_BUFM_MASK    (0x7 << GPIO_EC_BUFM_SHIFT)
#define GPIO_AF_SHIFT        (16) /* Bit 16-19 Alternate Function */
#define GPIO_AF_MASK         (15 << GPIO_AF_SHIFT)
#define GPIO_MODE_SHIFT      (14) /* Bit 14-15: IO Mode */
#define GPIO_MODE_MASK       (3 << GPIO_MODE_SHIFT)
#  define GPIO_NOINOUT       (0 << GPIO_MODE_SHIFT)  /* No input or output */
#  define GPIO_INPUT         (1 << GPIO_MODE_SHIFT)  /* Input Enable */
#  define GPIO_OUTPUT        (2 << GPIO_MODE_SHIFT)  /* Output Enable */

/* Output buffer:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * ..B. .... .... ....
 */

#define GPIO_BUFFER_SHIFT   (13)  /* Bit 13: Output buffer enabled */
#define GPIO_BUFFER_MASK    (1 << GPIO_BUFFER_SHIFT)
#define GPIO_BUFFER_DISABLE (0 << GPIO_BUFFER_SHIFT) /* Disable Output buffer */
#define GPIO_BUFFER_ENABLE  (1 << GPIO_BUFFER_SHIFT) /* Enable Output buffer */

/* Irq:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * ...I III. .... ....
 */

#define GPIO_IRQ_SHIFT     (9) /* Bits 9-12: Irq Mode */
#define GPIO_IRQ_MASK      (15 << GPIO_IRQ_SHIFT)
#define GPIO_IRQ_HIGH      (0 << GPIO_IRQ_SHIFT)
#define GPIO_IRQ_LOW       (1 << GPIO_IRQ_SHIFT)
#define GPIO_IRQ_EDGE_POS  (2 << GPIO_IRQ_SHIFT)
#define GPIO_IRQ_EDGE_NEG  (3 << GPIO_IRQ_SHIFT)
#define GPIO_IRQ_EDGE_BOTH (4 << GPIO_IRQ_SHIFT)
#define GPIO_IRQ_ENABLE    (8 << GPIO_IRQ_SHIFT)
#define GPIO_IRQ_DISABLE   (0 << GPIO_IRQ_SHIFT)

/* Bank:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... .bb. ....
 */

#define GPIO_BANK_SHIFT    (5) /* Bits 5-6: Bank */
#define GPIO_BANK_MASK     (3 << GPIO_BANK_SHIFT)
#define GPIO_BANK0         (0 << GPIO_BANK_SHIFT)
#define GPIO_BANK1         (1 << GPIO_BANK_SHIFT)
#define GPIO_BANK2         (2 << GPIO_BANK_SHIFT)

#define GPIO_BANK0_NUM_PINS 14
#define GPIO_BANK1_NUM_PINS 24
#define GPIO_BANK2_NUM_PINS 32

/* This identifies the bit in the bank:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... ...P PPPP
 */

#define GPIO_PIN_SHIFT (0) /* Bits 0-4: GPIO number: 0-31 */
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

int mpfs_configgpio(gpio_pinset_t cfgset);

/****************************************************************************
 * Name: mpfs_gpio_deinit
 *
 * Description:
 *   Deinit a GPIO (Set GPIO to floating input state)
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port.
 *
 ****************************************************************************/

int mpfs_gpio_deinit(uint8_t pin);

/****************************************************************************
 * Name: mpfs_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void mpfs_gpiowrite(gpio_pinset_t pinset, bool value);

/****************************************************************************
 * Name: mpfs_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool mpfs_gpioread(gpio_pinset_t pinset);

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

int mpfs_gpiosetevent(gpio_pinset_t pinset, bool risingedge,
                      bool fallingedge, bool high, bool low, bool event,
                      xcpt_t func, void *arg);

/****************************************************************************
 * Name: mpfs_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int mpfs_gpio_initialize(void);

/****************************************************************************
 * Function:  mpfs_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int mpfs_dumpgpio(gpio_pinset_t pinset, const char *msg);
#else
#define mpfs_dumpgpio(p, m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_MPFS_MPFS_GPIO_H */
