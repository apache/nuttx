/****************************************************************************
 * fs/socket/socketpair.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mm/mm.h>

#include <sys/socket.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include "inode/inode.h"
#include "fs_heap.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: socketpair
 *
 * Description:
 * Create an unbound pair of connected sockets in a specified domain, of a
 * specified type, under the protocol optionally specified by the protocol
 * argument. The two sockets shall be identical. The file descriptors used
 * in referencing the created sockets shall be returned in
 * sv[0] and sv[1].
 *
 * Input Parameters:
 *   domain   - (see sys/socket.h)
 *   type     - (see sys/socket.h)
 *   protocol - (see sys/socket.h)
 *   sv[2]    - The user provided array in which to catch the pair
 *              descriptors
 *
 ****************************************************************************/

int socketpair(int domain, int type, int protocol, int sv[2])
{
  FAR struct socket *psocks[2];
  int oflags = O_RDWR;
  int ret;
  int i;
  int j = 0;
  int k;

  if (sv == NULL)
    {
      ret = -EINVAL;
      goto errout;
    }

  for (k = 0; k < 2; k++)
    {
      psocks[k] = fs_heap_zalloc(sizeof(*psocks[k]));
      if (psocks[k] == NULL)
        {
          ret = -ENOMEM;
          goto errout_with_alloc;
        }
    }

  ret = psock_socketpair(domain, type, protocol, psocks);
  if (ret < 0)
    {
      goto errout_with_alloc;
    }

  if (type & SOCK_CLOEXEC)
    {
      oflags |= O_CLOEXEC;
    }

  if (type & SOCK_NONBLOCK)
    {
      oflags |= O_NONBLOCK;
    }

  /* Allocate a socket descriptor */

  for (; j < 2; j++)
    {
      sv[j] = sockfd_allocate(psocks[j], oflags);
      if (sv[j] < 0)
        {
          ret = sv[j];
          goto errout_with_psock;
        }
    }

  return OK;

errout_with_psock:
  for (i = 0; i < j; i++)
    {
      nx_close(sv[i]);
    }

  for (i = j; i < k; i++)
    {
      psock_close(psocks[i]);
    }

errout_with_alloc:
  for (i = j; i < k; i++)
    {
      fs_heap_free(psocks[i]);
    }

errout:
  set_errno(-ret);
  return ERROR;
}
