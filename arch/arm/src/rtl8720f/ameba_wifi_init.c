/****************************************************************************
 * arch/arm/src/rtl8720f/ameba_wifi_init.c
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
 * NuttX side of the RTL8720F WiFi bring-up.
 *
 * The WHC host stack talks to the WiFi MAC/PHY (which runs on the NP network
 * processor, km4ns) over the on-chip AP<->NP IPC.  This file wires that IPC
 * into NuttX's interrupt system -- mirroring the SDK AP main():
 *
 *     InterruptRegister(IPC_INTHandler, IPC_KM4_IRQ, IPCKM4_DEV, ...);
 *     InterruptEn(IPC_KM4_IRQ, ...);
 *     ipc_table_init(IPCKM4_DEV);
 *
 * but using NuttX's irq_attach()/up_enable_irq() (NuttX owns the vector
 * table).  ipc_table_init() then walks the .ipc.table.data section and
 * registers every channel callback the linked SDK objects contributed,
 * including the WHC WiFi TRX channels.  Once IPC is live it spawns a task
 * that runs the (blocking) host bring-up in ameba_wifi_start() (SDK-header
 * side, libameba_wifi.a).
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <sched.h>
#include <syslog.h>

#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/arch.h>

#include "ameba_irq.h"
#include "ameba_ipc.h"
#ifdef CONFIG_NET
#  include "ameba_wlan.h"
#endif

/****************************************************************************
 * External Function Prototypes
 ****************************************************************************/

/* Host WiFi bring-up (libameba_wifi.a, SDK-header side). */

extern int      ameba_wifi_start(void);

/****************************************************************************
 * Name: ameba_wifi_start_task
 *
 * Description:
 *   Task entry running the blocking WHC host bring-up once the scheduler and
 *   IPC are up.
 *
 ****************************************************************************/

static int ameba_wifi_start_task(int argc, char *argv[])
{
  UNUSED(argc);
  UNUSED(argv);

  /* Power on the WHC host stack (blocks on the NP round-trip), then register
   * the wlan0 network device.
   */

  int ret = ameba_wifi_start();

  syslog(LOG_INFO, "[ameba-wifi] ameba_wifi_start -> %d\n", ret);
  if (ret == 0)
    {
#ifdef CONFIG_NET
      ret = ameba_wlan_initialize();
      syslog(LOG_INFO, "[ameba-wifi] ameba_wlan_initialize -> %d\n", ret);
#endif
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8720f_wifi_initialize
 *
 * Description:
 *   Bring up the KM4 IPC transport for WiFi and start the WHC host stack.
 *   Call from board bring-up after the scheduler is running.
 *
 * Returned Value:
 *   0 on success; a negated errno on failure to spawn the bring-up task.
 *
 ****************************************************************************/

int rtl8720f_wifi_initialize(void)
{
  int pid;

  /* Route the km4tz IPC interrupt to the SDK dispatcher and register every
   * channel the linked objects contributed (WHC WiFi TRX channels included).
   * Idempotent and shared with the flash filesystem bring-up.
   */

  ameba_ipc_initialize();

  /* wifi_on() blocks on the NP round-trip, so run it off the init path. */

  pid = kthread_create("ameba_wifi", SCHED_PRIORITY_DEFAULT, 8192,
                       ameba_wifi_start_task, NULL);
  return pid < 0 ? pid : 0;
}
