/****************************************************************************
 * arch/arm/src/nrf53/nrf53_gpiote.h
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

#ifndef __ARCH_ARM_SRC_NRF53_NRF53_GPIOTE_H
#define __ARCH_ARM_SRC_NRF53_NRF53_GPIOTE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>

#include "chip.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* GPIOTE OUT task configuration */

enum nrf53_gpiote_outcfg_e
{
  NRF53_GPIOTE_SET    = 0,
  NRF53_GPIOTE_CLEAR  = 1,
  NRF53_GPIOTE_TOGGLE = 2,
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_gpiote_set_ch_event
 *
 * Description:
 *   Configures a GPIOTE channel in EVENT mode, assigns it to a given pin
 *   and sets a handler for the corresponding channel events.
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - channel:     GPIOTE channel used to capture events
 *  - risingedge:  Enables interrupt on rising edges
 *  - fallingedge: Enables interrupt on falling edges
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 ****************************************************************************/

void nrf53_gpiote_set_ch_event(uint32_t pinset, int channel,
                               bool risingedge, bool fallingedge,
                               xcpt_t func, void *arg);

/****************************************************************************
 * Name: nrf53_gpiote_set_event
 *
 * Description:
 *   Configures a GPIOTE channel in EVENT mode, assigns it to a given pin
 *   and sets a handler for the first availalbe GPIOTE channel
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - risingedge:  Enables interrupt on rising edges
 *  - fallingedge: Enables interrupt on falling edges
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int nrf53_gpiote_set_event(uint32_t pinset,
                           bool risingedge, bool fallingedge,
                           xcpt_t func, void *arg);

#ifdef CONFIG_NRF53_PER_PIN_INTERRUPTS
/****************************************************************************
 * Name: nrf53_gpiote_set_pin_event
 *
 * Description:
 *   Sets/clears a handler for a given pin for the GPIO PORT event. This
 *   will mean edge-sensitive or level-sensitive according to GPIO detect
 *   mode configuration for the port (see nrf53_gpio_detectmode()). Pin
 *   will be sensitive to high/low according to GPIO_SENSE_LOW/HIGH
 *   (set via nrf53_gpio_config()).
 *
 *   The passed handler will be invoked from the main ISR for the PORT
 *   event and will take care of clearing the LATCH register.
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 ****************************************************************************/

void nrf53_gpiote_set_pin_event(uint32_t pinset, xcpt_t func, void *arg);
#else

/****************************************************************************
 * Name: nrf53_gpiote_set_port_event
 *
 * Description:
 *   Sets/clears the handler for the GPIO PORT event.
 *
 *   The passed handler will be invoked from the main ISR for the PORT
 *   event and will take care of clearing the LATCH register.
 *
 * Input Parameters:
 *  - pinset:      GPIO port will be extracted from this parameter
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 ****************************************************************************/

void nrf53_gpiote_set_port_event(uint32_t pinset, xcpt_t func, void *arg);

#endif

/****************************************************************************
 * Name: nrf53_gpio_set_task
 *
 * Description:
 *   Configure GPIO in TASK mode (to be controlled via tasks).
 *   Note that a pin can only be either in TASK or EVENT mode (set by
 *   nrf53_gpiosetevent with event set to true). Also, once set to TASK mode,
 *   pin control is only possible via tasks on the via nrf53_gpio_write and
 *   will automatically set the output mode.
 *   Finally, a given pin should only be assigned to a given channel.
 *
 * Input Parameters:
 *  - pinset:      gpio pin configuration (only port + pin is important here)
 *  - channel:     the GPIOTE channel used to control the given pin
 *  - output_high: set pin initially to output HIGH or LOW.
 *  - outcfg:      configure pin behavior one OUT task is triggered
 *
 ****************************************************************************/

void nrf53_gpio_set_task(uint32_t pinset, int channel,
                        bool output_high, enum nrf53_gpiote_outcfg_e outcfg);

/****************************************************************************
 * Name: nrf53_gpiote_init
 *
 * Description:
 *   Initialize GPIOTE
 *
 ****************************************************************************/

int nrf53_gpiote_init(void);

#endif /* __ARCH_ARM_SRC_NRF53_NRF53_GPIOTE_H */
