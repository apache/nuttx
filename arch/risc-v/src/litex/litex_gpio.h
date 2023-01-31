/****************************************************************************
 * arch/risc-v/src/litex/litex_gpio.h
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

#ifndef __ARCH_RISCV_SRC_LITEX_LITEX_GPIO_H
#define __ARCH_RISCV_SRC_LITEX_LITEX_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Encoded pin attributes used with gpio_pinattr_t */
#define GPIO_PINMODE_INPUT      1   /* Pin is set to input mode. */
#define GPIO_PINMODE_OUTPUT     2   /* Pin is set to output mode. */

/* Encoded pin attributes used with gpio_intertype_t */
#define GPIO_ISR_NONE           1  /* Disables interrupts */
#define GPIO_ISR_RISING_EDGE    2  /* Interrupt generated on rising edge. */
#define GPIO_ISR_FALLING_EDGE   4  /* Interrupt generated on falling edge. */
#define GPIO_ISR_BOTH_EDGES     8  /* Interrupt generated on both edges. */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Describes attributes used to configure GPIO pins */

typedef uint32_t gpio_pinattr_t;

/* Describes attributes used to enable / disable GPIO interrupts. */

typedef uint32_t gpio_intrtype_t;

struct gpio_isr_config_s
{
    uint32_t port;          /* The GPIO port number to configure */
    uint32_t pin;           /* The pin on the port to configure */
    gpio_intrtype_t type;   /* The type of interrupt to use */
    xcpt_t handler;         /* The handler to call on interrupt event */
    void * userdata;        /* User data passed to the handler */
};

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
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: litex_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on encoded pin attributes. GPIO's currently
 *   only support a single attribute.
 *
 * Input Parameters:
 *   port      - The GPIO port number to read from.
 *   pin       - The pin, or bit offset inside the GPIO port.
 *   attr      - The pin attribute to apply to the pin.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int litex_gpio_config(uint32_t port, uint32_t pin, gpio_pinattr_t attr);

/****************************************************************************
 * Name: litex_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin.
 *
 * Input Parameters:
 *   port      - The GPIO port number to read from.
 *   pin       - The pin, or bit offset inside the GPIO port.
 *
 ****************************************************************************/

void litex_gpio_write(uint32_t port, uint32_t pin, bool value);

/****************************************************************************
 * Name: litex_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin.
 *
 * Input Parameters:
 *   port      - The GPIO port number to read from.
 *   pin       - The pin, or bit offset inside the GPIO port.
 *
 * Returned Value:
 *   True if the pin is logic high. False if the pin is logic low.
 *
 ****************************************************************************/

bool litex_gpio_read(uint32_t port, uint32_t pin);

/****************************************************************************
 * Name: litex_gpio_irq_enable
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins on the GPIO port.
 *
 * Input Parameters:
 *   irq       - The IRQ number to enable. Must be a hardware interrupt
 *               number as defined in irq.h.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_LITEX_GPIO_IRQ
int litex_gpio_irq_enable(int irq);
#else
#  define litex_gpio_irq_enable(irq)
#endif

/****************************************************************************
 * Name: litex_gpio_irq_configure
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ.
 *
 * Input Parameters:
 *   config      - The interrupt configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 ****************************************************************************/

#ifdef CONFIG_LITEX_GPIO_IRQ
int litex_gpio_irq_config(struct gpio_isr_config_s * config);
#else
#  define litex_gpio_irq_config(config)
#endif

/****************************************************************************
 * Name: litex_gpio_irq_disable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ port.
 *
 * Input Parameters:
 *   irq       - The IRQ number to disable. Must be a hardware interrupt
 *               number as defined in irq.h.
 *
 ****************************************************************************/

#ifdef CONFIG_LITEX_GPIO_IRQ
void litex_irq_disable(int irq);
#else
#  define litex_irq_disable(irq)
#endif

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_LITEX_LITEX_GPIO_H */