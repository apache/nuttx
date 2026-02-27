/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_xtensa_intr.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <stdbool.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include <arch/xtensa/xtensa_specregs.h>

#include "xtensa.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"
#include "platform/os.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef XCHAL_NUM_INTERRUPTS
#  define XCHAL_NUM_INTERRUPTS 32
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Handler table entry structure - must match HAL's xt_handler_table_entry */

typedef struct xt_handler_table_entry
{
  void *handler;
  void *arg;
} xt_handler_table_entry;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Interrupt handler table - exported as _xt_interrupt_table for HAL
 * compatibility. The HAL's xtensa_intr.c expects this to be defined
 * externally (originally in assembly).
 */

xt_handler_table_entry
    _xt_interrupt_table[XCHAL_NUM_INTERRUPTS * OS_PORT_NUM_PROCESSORS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xt_ints_on
 *
 * Description:
 *   Enables a set of interrupts.
 *
 * Input Parameters:
 *   mask - Bit mask of interrupts to be enabled.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void xt_ints_on(unsigned int mask)
{
  uint32_t intenable;

  __asm__ __volatile__("rsr %0, intenable" : "=r"(intenable));
  intenable |= mask;
  __asm__ __volatile__("wsr %0, intenable; rsync" :: "r"(intenable));
}

/****************************************************************************
 * Name: xt_ints_off
 *
 * Description:
 *   Disables a set of interrupts.
 *
 * Input Parameters:
 *   mask - Bit mask of interrupts to be disabled.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void xt_ints_off(unsigned int mask)
{
  uint32_t intenable;

  __asm__ __volatile__("rsr %0, intenable" : "=r"(intenable));
  intenable &= ~mask;
  __asm__ __volatile__("wsr %0, intenable; rsync" :: "r"(intenable));
}

/****************************************************************************
 * Name: esp_xtensa_intr_init
 *
 * Description:
 *   Initialize the interrupt handler table by setting all handlers to the
 *   default unhandled interrupt handler (provided by HAL's xtensa_intr.c).
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_xtensa_intr_init(void)
{
  int i;

  for (i = 0; i < (XCHAL_NUM_INTERRUPTS * OS_PORT_NUM_PROCESSORS); i++)
    {
      /* Calling xt_set_interrupt_handler with f=NULL sets the handler
       * to xt_unhandled_interrupt (defined in HAL's xtensa_intr.c)
       */

      xt_set_interrupt_handler(i, NULL, (void *)(uintptr_t)i);
    }
}
