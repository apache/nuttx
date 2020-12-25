/****************************************************************************
 * arch/arm/src/nrf52/nrf52_wdt.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
