/****************************************************************************
 * net/can/can_sockif.c
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

#ifdef CONFIG_NET_CAN

#include <sys/socket.h>

#include "can/can.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_sockif
 *
 * Description:
 *   Return the socket interface of the requested CAN protocol.  CAN_RAW
 *   is the only CAN protocol for now; can_raw_setup() validates the socket
 *   type and protocol.
 *
 * Input Parameters:
 *   family   - Socket address family (always PF_CAN)
 *   type     - Socket type
 *   protocol - CAN protocol
 *
 * Returned Value:
 *   The socket interface of the CAN protocol.
 *
 ****************************************************************************/

FAR const struct sock_intf_s *
can_sockif(sa_family_t family, int type, int protocol)
{
  return &g_can_raw_sockif;
}

#endif /* CONFIG_NET_CAN */
