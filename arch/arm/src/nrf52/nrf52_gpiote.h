/****************************************************************************
 * arch/arm/src/nrf52/nrf52_gpiote.h
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_GPIOTE_H
#define __ARCH_ARM_SRC_NRF52_NRF52_GPIOTE_H

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

enum nrf52_gpiote_outcfg_e
{
  NRF52_GPIOTE_SET    = 0,
  NRF52_GPIOTE_CLEAR  = 1,
  NRF52_GPIOTE_TOGGLE = 2,
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  - pinset: gpio pin configuration
 *  - rising/falling edge: enables
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 *  - arg:    Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int nrf52_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
                       bool event, xcpt_t func, FAR void *arg);

/****************************************************************************
 * Name: nrf52_gpiotaskset
 *
 * Description:
 *   Configure GPIO in TASK mode (to be controlled via tasks).
 *   Note that a pin can only be either in TASK or EVENT mode (set by
 *   nrf52_gpiosetevent with event set to true). Also, once set to TASK mode,
 *   pin control is only possible via tasks on the via nrf52_gpio_write and
 *   will automatically set the output mode.
 *   Finally, a given pin should only be assigned to a given channel.
 *
 * Input Parameters:
 *  - pinset: gpio pin configuration (only port + pin is important here)
 *  - channel: the GPIOTE channel used to control the given pin
 *  - output_high: set pin initially to output HIGH or LOW.
 *  - outcfg: configure pin behavior one OUT task is triggered
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int nrf52_gpiotaskset(uint32_t pinset, int channel, bool output_high,
                      enum nrf52_gpiote_outcfg_e outcfg);

/****************************************************************************
 * Name: nrf52_gpiote_init
 *
 * Description:
 *   Initialize GPIOTE
 *
 ****************************************************************************/

int nrf52_gpiote_init(void);

#endif /* __ARCH_ARM_SRC_NRF52_NRF52_GPIOTE_H */
