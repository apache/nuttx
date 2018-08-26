/****************************************************************************
 * net/socket/net_dupsd.c
 *
 *   Copyright (C) 2009, 2017 Gregory Nutt. All rights reserved.
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

#include "socket/socket.h"

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_dupsd
 *
 * Description:
 *   Clone a socket descriptor to an arbitrary descriptor number.  If file
 *   descriptors are implemented, then this is called by dup() for the case
 *   of socket file descriptors.  If file descriptors are not implemented,
 *   then this function IS dup().
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On any error,
 *   a negated errno value is returned:.
 *
 ****************************************************************************/

int psock_dupsd(FAR struct socket *psock, int minsd)
{
  FAR struct socket *psock2;
  int sockfd2;
  int ret;

  /* Make sure that the minimum socket descriptor is within the legal range.
   * The minimum value we receive is relative to file descriptor 0;  we need
   * map it relative of the first socket descriptor.
   */

#if CONFIG_NFILE_DESCRIPTORS > 0
  if (minsd >= CONFIG_NFILE_DESCRIPTORS)
    {
      minsd -= CONFIG_NFILE_DESCRIPTORS;
    }
  else
    {
      minsd = 0;
    }
#endif

  /* Lock the scheduler throughout the following */

  sched_lock();

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      ret = -EBADF;
      goto errout;
    }

  /* Allocate a new socket descriptor */

  sockfd2 = sockfd_allocate(minsd);
  if (sockfd2 < 0)
    {
      ret = -ENFILE;
      goto errout;
    }

  /* Get the socket structure underlying the new descriptor */

  psock2 = sockfd_socket(sockfd2);
  if (!psock2)
    {
      ret = -ENOSYS; /* Should not happen */
      goto errout_with_sockfd;
    }

  /* Duplicate the socket state */

  ret = net_clone(psock, psock2);
  if (ret < 0)
    {
      goto errout_with_sockfd;
    }

  sched_unlock();
  return sockfd2;

errout_with_sockfd:
  sockfd_release(sockfd2);

errout:
  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: net_dupsd
 *
 * Description:
 *   Clone a socket descriptor to an arbitrary descriptor number.  If file
 *   descriptors are implemented, then this is called by dup() for the case
 *   of socket file descriptors.  If file descriptors are not implemented,
 *   then this function IS dup().
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On any error,
 *   a negated errno value is returned:.
 *
 ****************************************************************************/

int net_dupsd(int sockfd, int minsd)
{
  return psock_dupsd(sockfd_socket(sockfd), minsd);
}

#endif /* defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0 */
