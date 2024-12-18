/****************************************************************************
 * fs/socket/accept.c
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

#include <sys/types.h>
#include <sys/socket.h>

#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <fcntl.h>

#include <nuttx/cancelpt.h>
#include <nuttx/net/net.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mm/kmap.h>
#include <arch/irq.h>

#include "sched/sched.h"
#include "fs_heap.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: map_user_mem
 *
 * Description:
 *   Map userspace memory to continuous kernel virtual memory using
 *   `kmm_map_user()`.
 *
 * Input Parameters:
 *   uaddr    The user address to map. Can be NULL, in which case NULL will
 *            be returned through `p_kaddr`.
 *   size     The size of the mem region to map
 *   p_kaddr  Pointer to the kernel address to return. Cannot be NULL.
 *
 * Returned Value:
 *  Returns 0 (OK) on success, or:
 *    -ENOMEM
 *      If there is not enough free memory to create the mapping.
 *    -EFAULT
 *      If `uaddr` points to memory not belonging to the user address space.
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KMAP
static int map_user_mem(FAR void *uaddr, size_t size, void **p_kaddr)
{
  *p_kaddr = uaddr;

  if (uaddr)
    {
      *p_kaddr = kmm_map_user(this_task(), uaddr, size);

      if (*p_kaddr == NULL)
        {
          return -ENOMEM;
        }
      else if (*p_kaddr == uaddr)
        {
          return -EFAULT;
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: accept4
 *
 * Description:
 *   The accept4 function is used with connection-based socket types
 *   (SOCK_STREAM, SOCK_SEQPACKET and SOCK_RDM). It extracts the first
 *   connection request on the queue of pending connections, creates a new
 *   connected socket with mostly the same properties as 'sockfd', and
 *   allocates a new socket descriptor for the socket, which is returned. The
 *   newly created socket is no longer in the listening state. The original
 *   socket 'sockfd' is unaffected by this call.  Per file descriptor flags
 *   are not inherited across an accept.
 *
 *   The 'sockfd' argument is a socket descriptor that has been created with
 *   socket(), bound to a local address with bind(), and is listening for
 *   connections after a call to listen().
 *
 *   On return, the 'addr' structure is filled in with the address of the
 *   connecting entity. The 'addrlen' argument initially contains the size
 *   of the structure pointed to by 'addr'; on return it will contain the
 *   actual length of the address returned.
 *
 *   If no pending connections are present on the queue, and the socket is
 *   not marked as non-blocking, accept blocks the caller until a connection
 *   is present. If the socket is marked non-blocking and no pending
 *   connections are present on the queue, accept returns EAGAIN.
 *
 * Input Parameters:
 *   sockfd   The listening socket descriptor
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr',
 *            Return: returned size of 'addr'
 *   flags    The flags used for initialization
 *
 * Returned Value:
 *  Returns -1 on error. If it succeeds, it returns a non-negative integer
 *  that is a descriptor for the accepted socket.
 *
 * EAGAIN or EWOULDBLOCK
 *   The socket is marked non-blocking and no connections are present to
 *   be accepted.
 * EBADF
 *   The descriptor is invalid.
 * ENOTSOCK
 *  The descriptor references a file, not a socket.
 * EOPNOTSUPP
 *   The referenced socket is not of type SOCK_STREAM.
 * EINTR
 *   The system call was interrupted by a signal that was caught before
 *   a valid connection arrived.
 * ECONNABORTED
 *   A connection has been aborted.
 * EINVAL
 *   Socket is not listening for connections.
 * EMFILE
 *   The per-process limit of open file descriptors has been reached.
 * ENFILE
 *   The system maximum for file descriptors has been reached.
 * EFAULT
 *   The addr parameter is not in a writable part of the user address
 *   space.
 * ENOBUFS or ENOMEM
 *   Not enough free memory.
 * EPROTO
 *   Protocol error.
 * EPERM
 *   Firewall rules forbid connection.
 *
 ****************************************************************************/

int accept4(int sockfd, FAR struct sockaddr *addr, FAR socklen_t *addrlen,
            int flags)
{
  FAR struct socket *psock = NULL;
  FAR struct socket *newsock;
  FAR struct file *filep;
  struct sockaddr *kaddr;
  socklen_t *kaddrlen;
  int oflags = O_RDWR;
  int errcode;
  int newfd;
  int ret;

  /* accept4() is a cancellation point */

  enter_cancellation_point();

  if (flags & ~(SOCK_NONBLOCK | SOCK_CLOEXEC))
    {
      errcode = EINVAL;
      goto errout;
    }

  /* Map user addresses to kernel virtual memory and check they actually
   * belong to the user
   *
   * NOTE: also check if writeable?
   * NOTE: possible TOCTOU with contents of addrlen?
   */

#ifdef CONFIG_MM_KMAP
  ret = map_user_mem(addr, sizeof(*addr), (void **)&kaddr);
  if (ret != OK)
    {
      errcode = -ret;
      goto errout;
    }

  ret = map_user_mem(addrlen, sizeof(*addrlen), (void **)&kaddrlen);
  if (ret != OK)
    {
      kmm_unmap(kaddr);
      errcode = -ret;
      goto errout;
    }
#else
  kaddr = addr;
  kaddrlen = addrlen;
#endif

  /* Get the underlying socket structure */

  ret = sockfd_socket(sockfd, &filep, &psock);

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (ret < 0)
    {
      errcode = -ret;
      goto errout_with_kmap;
    }

  newsock = fs_heap_zalloc(sizeof(*newsock));
  if (newsock == NULL)
    {
      errcode = ENOMEM;
      goto errout_with_filep;
    }

  ret = psock_accept(psock, kaddr, kaddrlen, newsock, flags);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout_with_alloc;
    }

  /* Allocate a socket descriptor for the new connection now (so that it
   * cannot fail later)
   */

  if (flags & SOCK_CLOEXEC)
    {
      oflags |= O_CLOEXEC;
    }

  if (flags & SOCK_NONBLOCK)
    {
      oflags |= O_NONBLOCK;
    }

  newfd = sockfd_allocate(newsock, oflags);
  if (newfd < 0)
    {
      errcode = ENFILE;
      goto errout_with_psock;
    }

  fs_putfilep(filep);

#ifdef CONFIG_MM_KMAP
  kmm_unmap(kaddr);
  kmm_unmap(kaddrlen);
#endif

  leave_cancellation_point();
  return newfd;

errout_with_psock:
  psock_close(newsock);

errout_with_alloc:
  fs_heap_free(newsock);

errout_with_filep:
  fs_putfilep(filep);

errout_with_kmap:
#ifdef CONFIG_MM_KMAP
  kmm_unmap(kaddr);
  kmm_unmap(kaddrlen);
#endif

errout:
  leave_cancellation_point();

  set_errno(errcode);
  return ERROR;
}
