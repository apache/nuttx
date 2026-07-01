/****************************************************************************
 * arch/arm/src/rtl8720f/ameba_ipc.c
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
 * RTL8720F km4tz<->km4ns IPC bring-up.
 *
 * The on-chip IPC links the AP (km4tz, runs NuttX) and the NP (km4ns, the
 * prebuilt device blob).  Several SDK fwlib subsystems use it, NOT just
 * WiFi:
 *
 *   - FLASH_Write_Lock()/Unlock() (ameba_flash_ram.c) IPC-pause the NP while
 *     the AP erases/programs flash.  Both cores XIP from the SAME SPI NOR
 *     (NP@0x02000000, AP@0x04000000), so the NP MUST be paused during a
 *     flash erase/program or its XIP fetch faults.  The lock sends
 *     IPC_A2N_FLASHPG_REQ and busy-waits for the NP to acknowledge; without
 *     IPC up the AP hangs.
 *   - The WHC host WiFi TRX channels.
 *
 * This must therefore be initialized before the flash filesystem mounts,
 * independent of whether WiFi is enabled.  It mirrors the SDK km4tz main():
 *
 *     ipc_table_init(IPCAP_DEV);
 *     InterruptRegister(IPC_INTHandler, IPC_KM4TZ_IRQ, IPCAP_DEV, ...);
 *     InterruptEn(IPC_KM4TZ_IRQ, ...);
 *
 * but using NuttX's irq_attach()/up_enable_irq() (NuttX owns the vector
 * table).  ipc_table_init() walks the .ipc.table.data section and registers
 * every channel callback the linked SDK objects contributed.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "ameba_irq.h"
#include "ameba_ipc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AP (km4tz) IPC device register base = IPCAP_DEV / IPC0_REG_BASE, from the
 * SDK RTL8720F hal_platform.h.  This matches the SDK km4tz main(), which
 * calls ipc_table_init(IPCAP_DEV) with the NON-secure alias 0x40804000
 * (the secure alias 0x50804000 is NOT what the AP uses).
 * RTL8720F_IRQ_IPC_KM4 maps to APIRQn IPC_KM4TZ_IRQ (external vector 3) --
 * see arch/.../irq.h.
 */

#define IPCAP_DEV  ((void *)0x40804000)

/****************************************************************************
 * External Function Prototypes
 ****************************************************************************/

/* SDK fwlib IPC (libameba_fwlib.a, compiled from SDK source). */

extern uint32_t IPC_INTHandler(void *data);
extern void     ipc_table_init(void *ipcx);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_ameba_ipc_inited;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_ipc_interrupt
 *
 * Description:
 *   NuttX IRQ handler shim for the km4tz IPC interrupt; forwards to the
 *   SDK's IPC_INTHandler, which dispatches to the registered channel
 *   callbacks.
 *
 ****************************************************************************/

static int ameba_ipc_interrupt(int irq, void *context, void *arg)
{
  UNUSED(irq);
  UNUSED(context);

  IPC_INTHandler(arg);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_ipc_initialize
 *
 * Description:
 *   Bring up the km4tz<->km4ns IPC transport once, after the scheduler is
 *   running.  Idempotent: safe to call from both the flash filesystem
 *   bring-up and the WiFi bring-up; only the first call wires the interrupt
 *   and walks the channel table.
 *
 ****************************************************************************/

void ameba_ipc_initialize(void)
{
  if (g_ameba_ipc_inited)
    {
      return;
    }

  g_ameba_ipc_inited = true;

  /* Register every channel the linked SDK objects contributed (flash sync,
   * and -- when linked -- the WHC WiFi TRX channels), then route the IPC
   * interrupt to the SDK dispatcher.  Order follows the SDK km4tz main():
   * table first, then unmask the NVIC line.
   */

  ipc_table_init(IPCAP_DEV);
  irq_attach(RTL8720F_IRQ_IPC_KM4, ameba_ipc_interrupt, IPCAP_DEV);
  up_enable_irq(RTL8720F_IRQ_IPC_KM4);
}
