/****************************************************************************
 * arch/arm/src/rp2040/rp2040_gpio.h
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

#ifndef __ARCH_ARM_SRC_RP2040_RP2040_GPIO_H
#define __ARCH_ARM_SRC_RP2040_RP2040_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include "hardware/rp2040_sio.h"
#include "hardware/rp2040_io_bank0.h"
#include "hardware/rp2040_pads_bank0.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RP2040_GPIO_NUM    30       /* Number of GPIO pins */

/* GPIO function types ******************************************************/

#define RP2040_GPIO_FUNC_JTAG       RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_JTAG
#define RP2040_GPIO_FUNC_SPI        RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_SPI
#define RP2040_GPIO_FUNC_UART       RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_UART
#define RP2040_GPIO_FUNC_I2C        RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_I2C
#define RP2040_GPIO_FUNC_PWM        RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_PWM
#define RP2040_GPIO_FUNC_SIO        RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_SIO
#define RP2040_GPIO_FUNC_PIO0       RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_PIO0
#define RP2040_GPIO_FUNC_PIO1       RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_PIO1
#define RP2040_GPIO_FUNC_CLOCKS     RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_CLOCKS
#define RP2040_GPIO_FUNC_USB        RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_USB
#define RP2040_GPIO_FUNC_NULL       RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_NULL

/* GPIO interrupt modes *****************************************************/

#define RP2040_GPIO_INTR_LEVEL_LOW    0
#define RP2040_GPIO_INTR_LEVEL_HIGH   1
#define RP2040_GPIO_INTR_EDGE_LOW     2
#define RP2040_GPIO_INTR_EDGE_HIGH    3

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 * Inline Functions
 ****************************************************************************/

static inline void rp2040_gpio_put(uint32_t gpio, int set)
{
  uint32_t value = 1 << gpio;

  DEBUGASSERT(gpio < RP2040_GPIO_NUM);

  if (set)
    {
      putreg32(value, RP2040_SIO_GPIO_OUT_SET);
    }
  else
    {
      putreg32(value, RP2040_SIO_GPIO_OUT_CLR);
    }
}

static inline bool rp2040_gpio_get(uint32_t gpio)
{
  uint32_t value = 1 << gpio;

  DEBUGASSERT(gpio < RP2040_GPIO_NUM);

  return (getreg32(RP2040_SIO_GPIO_IN) & value) != 0;
}

static inline void rp2040_gpio_setdir(uint32_t gpio, int out)
{
  uint32_t value = 1 << gpio;

  DEBUGASSERT(gpio < RP2040_GPIO_NUM);

  if (out)
    {
      putreg32(value, RP2040_SIO_GPIO_OE_SET);
    }
  else
    {
      putreg32(value, RP2040_SIO_GPIO_OE_CLR);
    }
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: r2040_gpio_get_function_pin
 *
 * Description:
 *   Get the GPIO pin number to which the specified function is assigned
 *
 ****************************************************************************/

int rp2040_gpio_get_function_pin(uint32_t func, uint32_t port);

/****************************************************************************
 * Name: r2040_gpio_set_function
 *
 * Description:
 *   Assign functions to the specified GPIO pin
 *
 ****************************************************************************/

void rp2040_gpio_set_function(uint32_t gpio, uint32_t func);

/****************************************************************************
 * Name: r2040_gpio_set_pulls
 *
 * Description:
 *   Set pull-up or pull-down to the specified GPIO pin
 *
 ****************************************************************************/

void rp2040_gpio_set_pulls(uint32_t gpio, int up, int down);

/****************************************************************************
 * Name: r2040_gpio_init
 *
 * Description:
 *   Initialize software-controlled GPIO function
 *
 ****************************************************************************/

void rp2040_gpio_init(uint32_t gpio);

/****************************************************************************
 * Name: r2040_gpio_irq_attach
 *
 * Description:
 *   Configure the interrupt generated by the specified GPIO pin.
 *
 ****************************************************************************/

int rp2040_gpio_irq_attach(uint32_t gpio, uint32_t intrmode,
                           xcpt_t isr, void *arg);

/****************************************************************************
 * Name: rp2040_gpio_enable_irq
 *
 * Description:
 *   Enable the GPIO IRQ specified by 'gpio'
 *
 ****************************************************************************/

void rp2040_gpio_enable_irq(uint32_t gpio);

/****************************************************************************
 * Name: rp2040_gpio_disable_irq
 *
 * Description:
 *   Disable the GPIO IRQ specified by 'gpio'
 *
 ****************************************************************************/

void rp2040_gpio_disable_irq(uint32_t gpio);

/****************************************************************************
 * Name: r2040_gpio_initialize
 *
 * Description:
 *   Initialize GPIO function management
 *
 ****************************************************************************/

void rp2040_gpio_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP2040_RP2040_GPIO_H */
