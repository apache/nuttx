/****************************************************************************
 * fs/socket/socket.c
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
#include <nuttx/fs/uio.h>
#include <nuttx/mm/mm.h>
#include <nuttx/mm/mempool.h>
#include <nuttx/mm/map.h>

#include <sys/socket.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <nuttx/debug.h>

#include "inode/inode.h"
#include "fs_heap.h"
#include "socket.h"

#if CONFIG_FS_PREALLOC_SOCKETS > 0 || CONFIG_FS_ALLOC_SOCKETS > 0
#  define SOCKET_POOL_BLOCKSIZE ALIGN_UP(sizeof(struct socket), MM_ALIGN)
#endif

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int sock_file_open(FAR struct file *filep);
static int sock_file_close(FAR struct file *filep);
static ssize_t sock_file_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static ssize_t sock_file_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen);
static int sock_file_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);
static int sock_file_mmap(FAR struct file *filep,
                          FAR struct mm_map_entry_s *map);
static int sock_file_poll(FAR struct file *filep, struct pollfd_s *fds,
                          bool setup);
static int sock_file_truncate(FAR struct file *filep, off_t length);
static ssize_t sock_file_readv(FAR struct file *filep,
                               FAR struct uio *uio);
static ssize_t sock_file_writev(FAR struct file *filep,
                                FAR struct uio *uio);

#if CONFIG_FS_PREALLOC_SOCKETS > 0 || CONFIG_FS_ALLOC_SOCKETS > 0
static FAR void *socket_pool_alloc(FAR struct mempool_s *pool, size_t size);
static void socket_pool_free(FAR struct mempool_s *pool, FAR void *addr);
static void socket_pool_check(FAR struct mempool_s *pool, FAR void *addr);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if CONFIG_FS_PREALLOC_SOCKETS > 0 || CONFIG_FS_ALLOC_SOCKETS > 0
static char g_socket_pool_buffer[CONFIG_FS_PREALLOC_SOCKETS *
  MEMPOOL_REALBLOCKSIZE(SOCKET_POOL_BLOCKSIZE) + MEMPOOL_HEADER_SIZE]
  aligned_data(MM_ALIGN);

static struct mempool_s g_socket_mempool =
{
  .initialbase = g_socket_pool_buffer,
  .initialsize = sizeof(g_socket_pool_buffer),
  .blocksize   = SOCKET_POOL_BLOCKSIZE,
  .expandsize  = CONFIG_FS_ALLOC_SOCKETS *
                 MEMPOOL_REALBLOCKSIZE(SOCKET_POOL_BLOCKSIZE) +
                 MEMPOOL_HEADER_SIZE,
  .name        = "socket",
  .alloc       = socket_pool_alloc,
  .free        = socket_pool_free,
  .check       = socket_pool_check,
};
#endif

static const struct file_operations g_sock_fileops =
{
  sock_file_open,     /* open */
  sock_file_close,    /* close */
  sock_file_read,     /* read */
  sock_file_write,    /* write */
  NULL,               /* seek */
  sock_file_ioctl,    /* ioctl */
  sock_file_mmap,     /* mmap */
  sock_file_truncate, /* truncate */
  sock_file_poll,     /* poll */
  sock_file_readv,    /* readv */
  sock_file_writev,   /* writev */
};

static struct inode g_sock_inode =
{
  NULL,                   /* i_parent */
  NULL,                   /* i_peer */
  NULL,                   /* i_child */
  1,                      /* i_crefs */
  FSNODEFLAG_TYPE_SOCKET, /* i_flags */
  {
    &g_sock_fileops       /* u */
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if CONFIG_FS_PREALLOC_SOCKETS > 0 || CONFIG_FS_ALLOC_SOCKETS > 0
/****************************************************************************
 * Name: socket_pool_alloc
 ****************************************************************************/

static FAR void *socket_pool_alloc(FAR struct mempool_s *pool, size_t size)
{
  return fs_heap_malloc(size);
}

/****************************************************************************
 * Name: socket_pool_free
 ****************************************************************************/

static void socket_pool_free(FAR struct mempool_s *pool, FAR void *addr)
{
  fs_heap_free(addr);
}

/****************************************************************************
 * Name: socket_pool_check
 ****************************************************************************/

static void socket_pool_check(FAR struct mempool_s *pool, FAR void *addr)
{
  DEBUGASSERT(fs_heap_member(addr));
}
#endif

static int sock_file_open(FAR struct file *filep)
{
  FAR struct socket *psock;
  int ret;

  psock = sock_alloc();
  if (psock == NULL)
    {
      return -ENOMEM;
    }

  ret = psock_dup2(filep->f_priv, psock);
  if (ret >= 0)
    {
      filep->f_priv = psock;
    }
  else
    {
      sock_free(psock);
    }

  return ret;
}

static int sock_file_close(FAR struct file *filep)
{
  psock_close(filep->f_priv);
  sock_free(filep->f_priv);
  return 0;
}

static ssize_t sock_file_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  return psock_recv(filep->f_priv, buffer, buflen, 0);
}

static ssize_t sock_file_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen)
{
  ssize_t ret;

  ret = psock_send(filep->f_priv, buffer, buflen, 0);
  psock_sigpipe(filep->f_priv, ret, 0);
  return ret;
}

static int sock_file_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  return psock_ioctl(filep->f_priv, cmd, arg);
}

static int sock_file_mmap(FAR struct file *filep,
                          FAR struct mm_map_entry_s *map)
{
  return psock_mmap(filep->f_priv, map);
}

static int sock_file_poll(FAR struct file *filep, FAR struct pollfd_s *fds,
                          bool setup)
{
  return psock_poll(filep->f_priv, fds, setup);
}

static int sock_file_truncate(FAR struct file *filep, off_t length)
{
  return -EINVAL;
}

static ssize_t sock_file_readv(FAR struct file *filep,
                               FAR struct uio *uio)
{
  struct msghdr msg;

  memset(&msg, 0, sizeof(struct msghdr));
  msg.msg_iov = (FAR struct iovec *)uio->uio_iov;
  msg.msg_iovlen = uio->uio_iovcnt;

  return psock_recvmsg(filep->f_priv, &msg, 0);
}

static ssize_t sock_file_writev(FAR struct file *filep,
                                FAR struct uio *uio)
{
  struct msghdr msg;
  ssize_t ret;

  memset(&msg, 0, sizeof(struct msghdr));
  msg.msg_iov = (FAR struct iovec *)uio->uio_iov;
  msg.msg_iovlen = uio->uio_iovcnt;

  ret = psock_sendmsg(filep->f_priv, &msg, 0);
  psock_sigpipe(filep->f_priv, ret, 0);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if CONFIG_FS_PREALLOC_SOCKETS > 0 || CONFIG_FS_ALLOC_SOCKETS > 0
/****************************************************************************
 * Name: sock_alloc
 ****************************************************************************/

FAR struct socket *sock_alloc(void)
{
  return mempool_zallocate(&g_socket_mempool, 0);
}

/****************************************************************************
 * Name: sock_free
 ****************************************************************************/

void sock_free(FAR struct socket *psock)
{
  mempool_release(&g_socket_mempool, psock);
}
#endif

/****************************************************************************
 * Name: sockfd_allocate
 *
 * Description:
 *   Allocate a socket descriptor
 *
 * Input Parameters:
 *   psock    A pointer to socket structure.
 *   oflags   Open mode flags.
 *
 * Returned Value:
 *   Allocate a struct files instance and associate it with an socket
 *   instance.  Returns the file descriptor == index into the files array.
 *
 ****************************************************************************/

int sockfd_allocate(FAR struct socket *psock, int oflags)
{
  return file_allocate_from_inode(&g_sock_inode, oflags, 0, psock, 0);
}

/****************************************************************************
 * Name: sockfd_socket
 *
 * Description:
 *   Given a socket descriptor, return the underlying socket structure.
 *
 * Input Parameters:
 *   sockfd - The socket descriptor index to use.
 *
 * Returns zero (OK) on success.  On failure, it returns a negated errno
 * value to indicate the nature of the error.
 *
 *    EBADF
 *      The file descriptor is not a valid index in the descriptor table.
 *    ENOTSOCK
 *      psock is a descriptor for a file, not a socket.
 *
 ****************************************************************************/

FAR struct socket *file_socket(FAR struct file *filep)
{
  if (filep != NULL && INODE_IS_SOCKET(filep->f_inode))
    {
      return filep->f_priv;
    }

  return NULL;
}

int sockfd_socket(int sockfd, FAR struct file **filep,
                  FAR struct socket **socketp)
{
  if (file_get(sockfd, filep) < 0)
    {
      *socketp = NULL;
      return -EBADF;
    }

  *socketp = file_socket(*filep);
  if (*socketp == NULL)
    {
      file_put(*filep);
    }

  return *socketp != NULL ? OK : -ENOTSOCK;
}

/****************************************************************************
 * Name: socket
 *
 * Description:
 *   socket() creates an endpoint for communication and returns a descriptor.
 *
 * Input Parameters:
 *   domain   (see sys/socket.h)
 *   type     (see sys/socket.h)
 *   protocol (see sys/socket.h)
 *
 * Returned Value:
 *   A non-negative socket descriptor on success; -1 on error with errno set
 *   appropriately.
 *
 *   EACCES
 *     Permission to create a socket of the specified type and/or protocol
 *     is denied.
 *   EAFNOSUPPORT
 *     The implementation does not support the specified address family.
 *   EINVAL
 *     Unknown protocol, or protocol family not available.
 *   EMFILE
 *     Process file table overflow.
 *   ENFILE
 *     The system limit on the total number of open files has been reached.
 *   ENOBUFS or ENOMEM
 *     Insufficient memory is available. The socket cannot be created until
 *     sufficient resources are freed.
 *   EPROTONOSUPPORT
 *     The protocol type or the specified protocol is not supported within
 *     this domain.
 *
 * Assumptions:
 *
 ****************************************************************************/

int socket(int domain, int type, int protocol)
{
  FAR struct socket *psock;
  int oflags = O_RDWR;
  int sockfd;
  int ret;

  if (type & SOCK_CLOEXEC)
    {
      oflags |= O_CLOEXEC;
    }

  if (type & SOCK_NONBLOCK)
    {
      oflags |= O_NONBLOCK;
    }

  psock = sock_alloc();
  if (psock == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* Initialize the socket structure */

  ret = psock_socket(domain, type, protocol, psock);
  if (ret < 0)
    {
      nerr("ERROR: psock_socket() failed: %d\n", ret);
      goto errout_with_alloc;
    }

  /* Allocate a socket descriptor */

  sockfd = sockfd_allocate(psock, oflags);
  if (sockfd < 0)
    {
      nerr("ERROR: Failed to allocate a socket descriptor\n");
      ret = sockfd;
      goto errout_with_psock;
    }

  return sockfd;

errout_with_psock:
  psock_close(psock);

errout_with_alloc:
  sock_free(psock);

errout:
  set_errno(-ret);
  return ERROR;
}
