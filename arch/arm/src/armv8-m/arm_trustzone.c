/****************************************************************************
 * arch/arm/src/armv8-m/arm_trustzone.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <arch/irq.h>

#include "arm_arch.h"
#include "nvic.h"

#ifdef CONFIG_ARCH_HAVE_TRUSTZONE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_secure_set_nbanked_exception_state
 *
 * Description:
 *   Set target state for exceptions not banked between security states
 *   Function sets the security state (Secure or Non-Secure) target
 *   for ARMv8-M HardFault, NMI, and BusFault exception.
 *
 ****************************************************************************/

void up_secure_set_nbanked_exception_state(bool secure)
{
  uint32_t aircr = getreg32(NVIC_AIRCR) & (~(NVIC_AIRCR_VECTKEYSTAT_MASK));

  if (secure)
    {
      if (!(aircr & NVIC_AIRCR_BFHFNMINS))
        {
          return;
        }

      aircr &= ~(NVIC_AIRCR_BFHFNMINS);
    }
  else
    {
      if (aircr & NVIC_AIRCR_BFHFNMINS)
        {
          return;
        }

      aircr |= NVIC_AIRCR_BFHFNMINS;
    }

  putreg32(NVIC_AIRCR_VECTKEY | aircr, NVIC_AIRCR);
}

#endif /* CONFIG_ARCH_HAVE_TRUSTZONE */
