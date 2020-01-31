/****************************************************************************
 * net/socket/net_dupsd2.c
 *
 *   Copyright (C) 2009, 2011, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/socket.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include "socket/socket.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_dupsd2
 *
 * Description:
 *   Clone a socket descriptor to an arbitray descriptor number.  If file
 *   descriptors are implemented, then this is called by dup2() for the case
 *   of socket file descriptors.  If file descriptors are not implemented,
 *   then this function IS dup2().
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On any error,
 *   a negated errno value is returned:.
 *
 ****************************************************************************/

int net_dupsd2(int sockfd1, int sockfd2)
{
  FAR struct socket *psock1;
  FAR struct socket *psock2;
  int ret;

  /* Lock the scheduler throughout the following */

  sched_lock();

  /* Get the socket structures underly both descriptors */

  psock1 = sockfd_socket(sockfd1);
  psock2 = sockfd_socket(sockfd2);

  /* Verify that the sockfd1 and sockfd2 both refer to valid socket
   * descriptors and that sockfd2 corresponds to an allocated socket
   */

  if (psock1 == NULL || psock2 == NULL || psock1->s_crefs <= 0)
    {
      ret = -EBADF;
      goto errout;
    }

  /* If sockfd2 also valid, allocated socket, then we will have to
   * close it!
   */

  if (psock2->s_crefs > 0)
    {
      net_close(sockfd2);
    }

  /* Duplicate the socket state */

  ret = net_clone(psock1, psock2);

errout:
  sched_unlock();
  return ret;
}
