/****************************************************************************
 * net/socket/net_dup.c
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

#include <sys/socket.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include "socket/socket.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_dup
 *
 * Description:
 *   Clone a socket descriptor to an arbitrary descriptor number.
 *
 * Returned Value:
 *   On success, returns the number of new socket.  On any error,
 *   a negated errno value is returned.
 *
 ****************************************************************************/

int psock_dup(FAR struct socket *psock, int minsd)
{
  FAR struct socket *psock2;
  int sockfd2;
  int ret;

  /* Make sure that the minimum socket descriptor is within the legal range.
   * The minimum value we receive is relative to file descriptor 0;  we need
   * map it relative of the first socket descriptor.
   */

  if (minsd >= CONFIG_NFILE_DESCRIPTORS)
    {
      minsd -= CONFIG_NFILE_DESCRIPTORS;
    }
  else
    {
      minsd = 0;
    }

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

  ret = psock_dup2(psock, psock2);
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
 * Name: net_dup
 *
 * Description:
 *   Clone a socket descriptor to an arbitrary descriptor number.
 *
 * Returned Value:
 *   On success, returns the number of new socket.  On any error,
 *   a negated errno value is returned.
 *
 ****************************************************************************/

int net_dup(int sockfd, int minsd)
{
  return psock_dup(sockfd_socket(sockfd), minsd);
}
