/****************************************************************************
 * net/inet/inet_txdrain.c
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

#include <sys/types.h>
#include <sys/socket.h>

#include <string.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/net/net.h>

#include "udp/udp.h"
#include "tcp/tcp.h"
#include "inet/inet.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_txdrain
 *
 * Description:
 *   Wait for any buffered Tx data to be sent from the socket.
 *
 * Parameters:
 *   psock   - Pointer to the socket structure instance
 *   timeout - The relative time when the timeout will occur
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated error value is returned on
 *   any failure.
 *
 ****************************************************************************/

int inet_txdrain(FAR struct socket *psock, unsigned int timeout)
{
  int ret = OK;

  DEBUGASSERT(psock != NULL);

  /* Draining depends on the socket family */

  switch (psock->s_type)
    {
#if defined(NET_TCP_HAVE_STACK)
      case SOCK_STREAM:
        {
          ret = tcp_txdrain(psock, timeout);
        }
        break;
#endif

#if defined(NET_UDP_HAVE_STACK)
      case SOCK_DGRAM:
        {
          ret = udp_txdrain(psock, timeout);
        }
        break;
#endif

     /* Other protocols do no support write buffering */

      default:
        break;
    }

  return ret;
}
