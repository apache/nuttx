/****************************************************************************
 * net/socket/net_close.c
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
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/net/net.h>

#include "socket/socket.h"

#ifdef CONFIG_NET

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_close
 *
 * Description:
 *   Performs the close operation on a socket instance
 *
 * Input Parameters:
 *   psock   Socket instance
 *
 * Returned Value:
 *  Returns zero (OK) on success.  On failure, it returns a negated errno
 *  value to indicate the nature of the error.
 *
 * Assumptions:
 *
 ****************************************************************************/

int psock_close(FAR struct socket *psock)
{
  int ret;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_crefs <= 0)
    {
      return -EBADF;
    }

  /* We perform the close operation only if this is the last count on
   * the socket. (actually, I think the socket crefs only takes the values
   * 0 and 1 right now).
   *
   * It is possible for a psock to have no connection, e.g. a TCP socket
   * waiting in accept.
   */

  if (psock->s_crefs <= 1 && psock->s_conn != NULL)
    {
      /* Assume that the socket close operation will be successful.  Save
       * the current flags and mark the socket uninitialized.  This avoids
       * race conditions in the SMP case.  We save the flags as a type
       * unsigned int in case the size of s_flags changes in the future
       * (currently uint8_t).
       */

      unsigned int saveflags = psock->s_flags;

      psock->s_flags &= ~_SF_INITD;

      /* Let the address family's close() method handle the operation */

      DEBUGASSERT(psock->s_sockif != NULL &&
                  psock->s_sockif->si_close != NULL);

      ret = psock->s_sockif->si_close(psock);

      /* Was the close successful */

      if (ret < 0)
        {
          /* No.. restore the socket flags */

          psock->s_flags = saveflags;
          return ret;
        }
    }

  /* Then release our reference on the socket structure containing the
   * connection.
   */

  psock_release(psock);
  return OK;
}

/****************************************************************************
 * Name: net_close
 *
 * Description:
 *   Performs the close operation on socket descriptors
 *
 * Input Parameters:
 *   sockfd   Socket descriptor of socket
 *
 * Returned Value:
 *  Returns zero (OK) on success.  On failure, it returns a negated errno
 *  value to indicate the nature of the error.
 *
 * Assumptions:
 *
 ****************************************************************************/

int net_close(int sockfd)
{
  return psock_close(sockfd_socket(sockfd));
}

#endif /* CONFIG_NET */
