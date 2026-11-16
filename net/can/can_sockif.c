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
 *   Return the socket interface for the requested CAN protocol: the
 *   ISO-TP interface for CAN_ISOTP, the CAN_RAW interface otherwise.  The
 *   setup routine of the selected interface validates the socket type and
 *   protocol.
 *
 * Input Parameters:
 *   family   - Socket address family (always PF_CAN)
 *   type     - Socket type
 *   protocol - CAN protocol
 *
 * Returned Value:
 *   The socket interface for the CAN protocol.
 *
 ****************************************************************************/

FAR const struct sock_intf_s *
can_sockif(sa_family_t family, int type, int protocol)
{
#ifdef CONFIG_NET_CAN_ISOTP
  if (protocol == CAN_ISOTP)
    {
      return &g_can_isotp_sockif;
    }
#endif

  return &g_can_raw_sockif;
}

/****************************************************************************
 * Name: can_set_rx_callback
 *
 * Description:
 *   register rx sync callback to the can connection structures.
 *
 * Input Parameters:
 *   psock    - The socket structure of the CAN connection
 *   callback - The callback function to be called when a frame is received
 *   args     - The argument to be passed to the callback function
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

int can_set_rx_callback(FAR struct socket *psock, can_rx_cb_t callback,
                        FAR void *args)
{
  FAR struct can_conn_s *conn;

  if (psock == NULL || psock->s_conn == NULL || callback == NULL)
    {
      return -EBADF;
    }

  conn = psock->s_conn;

  conn->rxcb  = callback;
  conn->rxarg = args;

  return OK;
}

#endif /* CONFIG_NET_CAN */
