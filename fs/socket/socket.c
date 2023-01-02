/****************************************************************************
 * fs/socket/socket.c
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
static int sock_file_poll(FAR struct file *filep, struct pollfd *fds,
                          bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_sock_fileops =
{
  sock_file_open,   /* open */
  sock_file_close,  /* close */
  sock_file_read,   /* read */
  sock_file_write,  /* write */
  NULL,             /* seek */
  sock_file_ioctl,  /* ioctl */
  NULL,             /* mmap */
  NULL,             /* truncate */
  sock_file_poll    /* poll */
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

static int sock_file_open(FAR struct file *filep)
{
  FAR struct socket *psock;
  int ret;

  psock = kmm_zalloc(sizeof(*psock));
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
      kmm_free(psock);
    }

  return ret;
}

static int sock_file_close(FAR struct file *filep)
{
  psock_close(filep->f_priv);
  kmm_free(filep->f_priv);
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
  return psock_send(filep->f_priv, buffer, buflen, 0);
}

static int sock_file_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  return psock_ioctl(filep->f_priv, cmd, arg);
}

static int sock_file_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup)
{
  return psock_poll(filep->f_priv, fds, setup);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
  return file_allocate(&g_sock_inode, oflags, 0, psock, 0, true);
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
  if (filep != NULL && filep->f_inode != NULL &&
      INODE_IS_SOCKET(filep->f_inode))
    {
      return filep->f_priv;
    }

  return NULL;
}

int sockfd_socket(int sockfd, FAR struct socket **socketp)
{
  FAR struct file *filep;

  if (fs_getfilep(sockfd, &filep) < 0)
    {
      *socketp = NULL;
      return -EBADF;
    }

  *socketp = file_socket(filep);

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

  psock = kmm_zalloc(sizeof(*psock));
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
  kmm_free(psock);

errout:
  set_errno(-ret);
  return ERROR;
}
