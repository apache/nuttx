/****************************************************************************
 * arch/arm/src/nrf52/nrf52_wdt.h
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_WDT_H
#define __ARCH_ARM_SRC_NRF52_NRF52_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_wdt_int_enable
 *
 * Description:
 *   Enable watchdog interrupt.
 *
 * Input Parameter:
 *   int_mask - Interrupt mask
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void nrf52_wdt_int_enable(uint32_t int_mask);

/****************************************************************************
 * Name: nrf52_wdt_task_trigger
 *
 * Description:
 *   Starts the watchdog
 *
 * Input Parameter:
 *   None
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void nrf52_wdt_task_trigger(void);

/****************************************************************************
 * Name: nrf52_wdt_reload_register_enable
 *
 * Description:
 *   Enable/disable a given reload register
 *
 * Input Parameter:
 *   rr_register - Reload request register
 *   enable - true to enable, false to disable
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void nrf52_wdt_reload_register_enable(int rr_register, bool enable);

/****************************************************************************
 * Name: nrf52_wdt_reload_request_set
 *
 * Description:
 *   Setting a specific reload request register
 *
 * Input Parameter:
 *   rr_register - Reload request register to set
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void nrf52_wdt_reload_request_set(int rr_register);

/****************************************************************************
 * Name: nrf52_wdt_behaviour_set
 *
 * Description:
 *   Configure the watchdog behaviour when the CPU is sleeping or halted
 *
 * Input Parameter:
 *   behavior - The behaviour to be configured
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void nrf52_wdt_behaviour_set(int behaviour);

/****************************************************************************
 * Name: nrf52_wdt_reload_value_set
 *
 * Description:
 *   Setup the watchdog reload value
 *
 * Input Parameter:
 *   reload_value - Watchdog counter initial value
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void nrf52_wdt_reload_value_set(uint32_t reload_value);

/****************************************************************************
 * Name: nrf52_wdt_reload_value_get
 *
 * Description:
 *   Read the watchdog reload value
 *
 * Returned Values:
 *   Watchdog counter initial value
 *
 ****************************************************************************/

uint32_t nrf52_wdt_reload_value_get(void);

/****************************************************************************
 * Name: nrf52_wdt_running
 *
 * Description:
 *   Check if watchdog is running
 *
 * Returned Values:
 *   true if watchdog is running, false otherwise
 *
 ****************************************************************************/

bool nrf52_wdt_running(void);

#endif /* __ARCH_ARM_SRC_NRF52_NRF52_WDT_H */
