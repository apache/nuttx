/****************************************************************************
 * arch/arm/src/rm57/rm57_netinitialize.c
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

/* Single owner of arm_netinitialize() for this port.
 *
 * arm_internal.h declares arm_netinitialize() (and arm_initialize.c calls
 * it unconditionally from up_initialize()) whenever CONFIG_NET is set and
 * CONFIG_NETDEV_LATEINIT is NOT set; with CONFIG_NETDEV_LATEINIT, board
 * bring-up code registers network devices itself instead (see
 * boards/arm/rm57/rm57l843-launchxl2/src/rm57_can.c and
 * src/rm57_ethernet.c), typically because it wants to supply a real MAC
 * address rather than this file's placeholder one.
 *
 * Both of this port's netdev-capable drivers (SocketCAN over DCAN,
 * rm57_dcan_sock.c; and the EMAC, rm57_ethernet.c) previously each
 * defined arm_netinitialize() themselves, which only worked because at
 * most one was ever built at a time.  Enabling both together would fail
 * to link (duplicate symbol), so this symbol now has exactly one owner.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "rm57_dcan.h"
#include "rm57_ethernet.h"

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_netinitialize
 ****************************************************************************/

void arm_netinitialize(void)
{
#ifdef CONFIG_RM57_DCAN1_SOCKET
  rm57_dcan_sock_initialize(0);
#endif
#ifdef CONFIG_RM57_DCAN2_SOCKET
  rm57_dcan_sock_initialize(1);
#endif
#ifdef CONFIG_RM57_DCAN3_SOCKET
  rm57_dcan_sock_initialize(2);
#endif
#ifdef CONFIG_RM57_DCAN4_SOCKET
  rm57_dcan_sock_initialize(3);
#endif

#ifdef CONFIG_RM57_EMAC
  /* NULL: no board-supplied MAC address available in this path (that
   * requires CONFIG_NETDEV_LATEINIT so board bring-up can run first) -
   * rm57_ethinitialize() substitutes a fixed locally-administered
   * placeholder.
   */

  rm57_ethinitialize(0, NULL);
#endif
}

#endif /* CONFIG_NET && !CONFIG_NETDEV_LATEINIT */
