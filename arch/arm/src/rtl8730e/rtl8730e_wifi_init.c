/****************************************************************************
 * arch/arm/src/rtl8730e/rtl8730e_wifi_init.c
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
 * NuttX side of the RTL8730E (AmebaSmart CA32) WiFi bring-up.
 *
 * The WHC (WiFi Host Controller) stack talks to the WiFi MAC/PHY running on
 * the NP (KM4) over the on-chip AP<->NP IPC.  CA32 is the WHC HOST; KM4 is
 * the WHC DEVICE.
 *
 * The IPC_AP IRQ (GIC SPI 24) is owned by the LOGUART serial driver
 * (rtl8730e_serial.c).  This file does NOT call irq_attach for that IRQ;
 * instead it provides rtl8730e_wifi_ipc_dispatch() (a strong override of the
 * weak stub in rtl8730e_serial.c) which the LOGUART handler calls at the end
 * of every IPC_AP interrupt so that NP→AP WiFi channels are also serviced.
 *
 * ipc_table_init() walks .ipc.table.data and registers every WHC channel
 * callback into the SDK's IPC_IrqHandler[] table.  After IPC is live,
 * ameba_wifi_start() powers on the WHC host stack.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <syslog.h>

#ifdef CONFIG_NET
#  include "ameba_wlan.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CA32 AP IPC register base (IPCAP_REG_BASE from SDK hal_platform.h). */

#define IPCAP_DEV  ((void *)0x41000580)

/****************************************************************************
 * External Function Prototypes
 ****************************************************************************/

/* SDK fwlib IPC (libameba_fwlib.a, compiled from SDK source). */

extern uint32_t IPC_INTHandler(void *data);
extern void     ipc_table_init(void *ipcx);

/* Host WiFi bring-up (libameba_wifi.a, SDK-header side). */

extern int      ameba_wifi_start(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Strong implementation of the WiFi IPC dispatch hook declared weak in
 * rtl8730e_serial.c.  Called at the tail of every IPC_AP interrupt (which
 * is owned by the LOGUART serial driver) so that NP→AP WiFi channels
 * (bits 16/17) are processed without stealing the IRQ from LOGUART RX.
 */

void rtl8730e_wifi_ipc_dispatch(void *ipcx)
{
  IPC_INTHandler(ipcx);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8730e_wifi_initialize
 *
 * Description:
 *   Wire the AP IPC GIC interrupt, initialise the IPC channel table, start
 *   the WHC host stack and register the wlan0 network device.  Must be
 *   called from board bring-up after the scheduler is running.
 *
 *   wifi_on() blocks on a KM4 IPC round-trip.  KM4 is already running its
 *   WiFi firmware by this point, so the wait is bounded.
 *
 ****************************************************************************/

int rtl8730e_wifi_initialize(void)
{
  int ret;

  /* The IPC_AP IRQ (GIC SPI 24) is already owned by the LOGUART serial
   * driver (rtl8730e_serial.c loguart_attach).  We must NOT call irq_attach
   * here — that would overwrite the LOGUART handler and break console RX.
   * Instead, rtl8730e_wifi_ipc_dispatch() (above) is invoked by the LOGUART
   * handler at the tail of every IPC_AP interrupt to process WiFi channels.
   *
   * ipc_table_init() walks .ipc.table.data and registers WHC channel
   * callbacks into the SDK's IPC_IrqHandler[] table so IPC_INTHandler()
   * dispatches them correctly.
   */

  ipc_table_init(IPCAP_DEV);

  /* Power on the WHC host stack (blocks on the KM4 round-trip). */

  ret = ameba_wifi_start();
  syslog(LOG_INFO, "[ameba-wifi] ameba_wifi_start -> %d\n", ret);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_NET
  /* Register wlan0 before returning so netinit sees it at bring-up. */

  ret = ameba_wlan_initialize();
  syslog(LOG_INFO, "[ameba-wifi] ameba_wlan_initialize -> %d\n", ret);
#endif

  return ret;
}
