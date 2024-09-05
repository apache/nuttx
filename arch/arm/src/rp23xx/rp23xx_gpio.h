/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_gpio.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_RP23XX_GPIO_H
#define __ARCH_ARM_SRC_RP23XX_RP23XX_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include "arm_internal.h"
#include "hardware/address_mapped.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/pads_bank0.h"
#include "hardware/structs/io_bank0.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if PICO_RP2350A
#define RP23XX_GPIO_NUM    30       /* Number of GPIO pins */
#else
#define RP23XX_GPIO_NUM    48       /* Number of GPIO pins */
#endif

/* GPIO function pins *******************************************************/

#define RP23XX_GPIO_PIN_CLK_GPOUT0  (21)
#define RP23XX_GPIO_PIN_CLK_GPOUT1  (23)
#define RP23XX_GPIO_PIN_CLK_GPOUT2  (24)
#define RP23XX_GPIO_PIN_CLK_GPOUT3  (25)

/* GPIO interrupt modes *****************************************************/

#define RP23XX_GPIO_INTR_LEVEL_LOW  0
#define RP23XX_GPIO_INTR_LEVEL_HIGH 1
#define RP23XX_GPIO_INTR_EDGE_LOW   2
#define RP23XX_GPIO_INTR_EDGE_HIGH  3

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

static inline void rp23xx_gpio_put(uint32_t gpio, int set)
{
  uint32_t value = 1 << gpio;

  DEBUGASSERT(gpio < RP23XX_GPIO_NUM);

  if (set)
    {
      putreg32(value, &sio_hw->gpio_set);
    }
  else
    {
      putreg32(value, &sio_hw->gpio_clr);
    }
}

static inline bool rp23xx_gpio_get(uint32_t gpio)
{
  uint32_t value = 1 << gpio;

  DEBUGASSERT(gpio < RP23XX_GPIO_NUM);

  return (getreg32(&sio_hw->gpio_in) & value) != 0;
}

static inline void rp23xx_gpio_setdir(uint32_t gpio, int out)
{
  uint32_t value = 1 << gpio;

  DEBUGASSERT(gpio < RP23XX_GPIO_NUM);

  if (out)
    {
      putreg32(value, &sio_hw->gpio_oe_set);
    }
  else
    {
      putreg32(value, &sio_hw->gpio_oe_clr);
    }
}

/****************************************************************************
 * Name: rp23xx_gpio_set_input_hysteresis_enabled
 *
 * Description:
 *   Set whether the pin's input hysteresis will be enabled.
 *
 ****************************************************************************/

static inline void rp23xx_gpio_set_input_hysteresis_enabled(uint32_t gpio,
                                                            bool enabled)
{
  DEBUGASSERT(gpio < RP23XX_GPIO_NUM);

  hw_write_masked(&pads_bank0_hw->io[gpio],
                  enabled ? PADS_BANK0_GPIO0_SCHMITT_BITS : 0,
                  PADS_BANK0_GPIO0_SCHMITT_BITS);
}

/****************************************************************************
 * Name: rp23xx_gpio_set_slew_fast
 *
 * Description:
 *   Set whether the pin's fast slew rate will be enabled.
 *
 ****************************************************************************/

static inline void rp23xx_gpio_set_slew_fast(uint32_t gpio,
                                             bool enabled)
{
  DEBUGASSERT(gpio < RP23XX_GPIO_NUM);

  hw_write_masked(&pads_bank0_hw->io[gpio],
                  enabled ? PADS_BANK0_GPIO0_SLEWFAST_BITS : 0,
                  PADS_BANK0_GPIO0_SLEWFAST_BITS);
}

/****************************************************************************
 * Name: rp23xx_gpio_set_drive_strength
 *
 * Description:
 *   Set the pin's drive strength.
 *
 ****************************************************************************/

static inline void rp23xx_gpio_set_drive_strength(uint32_t gpio,
                                                  uint32_t drive_strength)
{
  DEBUGASSERT(gpio < RP23XX_GPIO_NUM);

  hw_write_masked(&pads_bank0_hw->io[gpio],
                  drive_strength,
                  PADS_BANK0_GPIO0_DRIVE_BITS);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_gpio_get_function_pin
 *
 * Description:
 *   Get the GPIO pin number to which the specified function is assigned
 *
 ****************************************************************************/

int rp23xx_gpio_get_function_pin(uint32_t func, uint32_t port);

/****************************************************************************
 * Name: rp23xx_gpio_set_function
 *
 * Description:
 *   Assign functions to the specified GPIO pin
 *
 ****************************************************************************/

void rp23xx_gpio_set_function(uint32_t gpio, uint32_t func);

/****************************************************************************
 * Name: rp23xx_gpio_set_pulls
 *
 * Description:
 *   Set pull-up or pull-down to the specified GPIO pin
 *
 ****************************************************************************/

void rp23xx_gpio_set_pulls(uint32_t gpio, int up, int down);

/****************************************************************************
 * Name: rp23xx_gpio_init
 *
 * Description:
 *   Initialize software-controlled GPIO function
 *
 ****************************************************************************/

void rp23xx_gpio_init(uint32_t gpio);

/****************************************************************************
 * Name: rp23xx_gpio_irq_attach
 *
 * Description:
 *   Configure the interrupt generated by the specified GPIO pin.
 *
 ****************************************************************************/

int rp23xx_gpio_irq_attach(uint32_t gpio, uint32_t intrmode,
                           xcpt_t isr, void *arg);

/****************************************************************************
 * Name: rp23xx_gpio_enable_irq
 *
 * Description:
 *   Enable the GPIO IRQ specified by 'gpio'
 *
 ****************************************************************************/

void rp23xx_gpio_enable_irq(uint32_t gpio);

/****************************************************************************
 * Name: rp23xx_gpio_disable_irq
 *
 * Description:
 *   Disable the GPIO IRQ specified by 'gpio'
 *
 ****************************************************************************/

void rp23xx_gpio_disable_irq(uint32_t gpio);

/****************************************************************************
 * Name: rp23xx_gpio_clear_interrupt
 *
 * Description:
 *   Clear the interrupt flags for a gpio pin.
 *
 ****************************************************************************/

void rp23xx_gpio_clear_interrupt(uint32_t gpio,
                                 bool     edge_low,
                                 bool     edge_high);

/****************************************************************************
 * Name: rp23xx_gpio_initialize
 *
 * Description:
 *   Initialize GPIO function management
 *
 ****************************************************************************/

void rp23xx_gpio_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP23XX_RP23XX_GPIO_H */
