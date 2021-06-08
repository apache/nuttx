/****************************************************************************
 * net/socket/socketpair.c
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

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>

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
 *   psocks   - The array to catch the pair descriptors
 *
 ****************************************************************************/

int psock_socketpair(int domain, int type, int protocol,
                     FAR struct socket *psocks[2])
{
  int ret;

  /* Initialize the socket structure */

  ret = psock_socket(domain, type, protocol, psocks[0]);
  if (ret < 0)
    {
      return ret;
    }

  if (psocks[0]->s_sockif->si_socketpair == NULL)
    {
      ret = -EAFNOSUPPORT;
      goto errsock;
    }

  ret = psock_socket(domain, type, protocol, psocks[1]);
  if (ret < 0)
    {
      goto errsock;
    }

  /* Perform socketpair process */

  ret = psocks[0]->s_sockif->si_socketpair(psocks);
  if (ret == 0)
    {
      return ret;
    }

  psock_close(psocks[1]);
errsock:
  psock_close(psocks[0]);
  return ret;
}

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
                descriptors
 *
 ****************************************************************************/

int socketpair(int domain, int type, int protocol, int sv[2])
{
  FAR struct socket *psocks[2];
  int oflags = O_RDWR;
  int i = 0;
  int ret;

  if (sv == NULL)
    {
      ret = -EINVAL;
      goto errout;
    }

  for (i = 0; i < 2; i++)
    {
      psocks[i] = kmm_zalloc(sizeof(*psocks[i]));
      if (psocks[i] == NULL)
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

  /* Allocate a socket descriptor */

  for (i = 0; i < 2; i++)
    {
      sv[i] = sockfd_allocate(psocks[i], oflags);
      if (sv[i] < 0)
        {
          ret = sv[i];
          goto errout_with_psock;
        }
    }

  return OK;

errout_with_psock:
  while (i-- > 0)
    {
      nx_close(sv[i]);
    }

  i = 2;
errout_with_alloc:
  while (i-- > 0)
    {
      kmm_free(psocks[i]);
    }

errout:
  set_errno(-ret);
  return ERROR;
}
