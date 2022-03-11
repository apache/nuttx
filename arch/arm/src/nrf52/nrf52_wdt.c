/****************************************************************************
 * arch/arm/src/nrf52/nrf52_wdt.c
 *
 *   Copyright (C) 2018 Zglue Inc. All rights reserved.
 *   Author: Levin Li <zhiqiang@zglue.com>
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "arm_internal.h"
#include "hardware/nrf52_wdt.h"
#include "nrf52_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
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

void nrf52_wdt_int_enable(uint32_t int_mask)
{
  putreg32(int_mask, NRF52_WDT_INTENSET);
}

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

void nrf52_wdt_task_trigger(void)
{
  putreg32(1, NRF52_WDT_TASKS_START);
}

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

void nrf52_wdt_reload_register_enable(int rr_register, bool enable)
{
  modifyreg32(NRF52_WDT_RREN,
              enable ? 0 : WDT_RREN_RR(rr_register),
              enable ? WDT_RREN_RR(rr_register) : 0);
}

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

void nrf52_wdt_reload_request_set(int rr_register)
{
  /* Each register is 32-bit (4 bytes), then multiply by 4 to get offset */

  putreg32(WDT_RR_VALUE, NRF52_WDT_RR0 + (4 * rr_register));
}

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

void nrf52_wdt_behaviour_set(int behaviour)
{
  putreg32(behaviour, NRF52_WDT_CONFIG);
}

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

void nrf52_wdt_reload_value_set(uint32_t reload_value)
{
  putreg32(reload_value, NRF52_WDT_CRV);
}

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

uint32_t nrf52_wdt_reload_value_get(void)
{
  return getreg32(NRF52_WDT_CRV);
}

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

bool nrf52_wdt_running(void)
{
  return getreg32(NRF52_WDT_RUNSTATUS);
}
