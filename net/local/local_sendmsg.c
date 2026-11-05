/****************************************************************************
 * net/local/local_sendmsg.c
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
#include <nuttx/debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>

#include "socket/socket.h"
#include "local/local.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_sendctl
 *
 * Description:
 *   Handle the socket message conntrol field
 *
 * Input Parameters:
 *   conn     Local connection instance
 *   msg      Message to send
 *
 * Returned Value:
 *  On any failure, a negated errno value is returned
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_SCM
static void local_freectl(FAR struct local_conn_s *conn, int count)
{
  FAR struct local_conn_s *peer = conn->lc_peer;

  if (peer == NULL)
    {
      peer = conn;
    }

  while (count-- > 0)
    {
      file_put(peer->lc_cfps[--peer->lc_cfpcount]);
      peer->lc_cfps[peer->lc_cfpcount] = NULL;
    }
}

static int local_sendctl(FAR struct local_conn_s *conn,
                         FAR const struct msghdr *msg)
{
  FAR struct local_conn_s *peer;
  FAR struct file *filep;
  FAR struct cmsghdr *cmsg;
  int count = 0;
  FAR int *fds;
  int ret;
  int i = 0;

  local_lock();
  peer = conn->lc_peer;
  if (peer == NULL)
    {
      peer = conn;
    }

  for_each_cmsghdr(cmsg, msg)
    {
      if (!CMSG_OK(msg, cmsg) ||
          cmsg->cmsg_level != SOL_SOCKET ||
          cmsg->cmsg_type != SCM_RIGHTS)
        {
          ret = -EOPNOTSUPP;
          goto fail;
        }

      fds = (FAR int *)CMSG_DATA(cmsg);
      count = (cmsg->cmsg_len - sizeof(struct cmsghdr)) / sizeof(int);

      if (count + peer->lc_cfpcount >= LOCAL_NCONTROLFDS)
        {
          ret = -EMFILE;
          goto fail;
        }

      for (i = 0; i < count; i++)
        {
          ret = file_get(fds[i], &filep);
          if (ret < 0)
            {
              goto fail;
            }

          peer->lc_cfps[peer->lc_cfpcount++] = filep;
        }
    }

  local_unlock();
  return count;

fail:
  local_freectl(conn, i);
  local_unlock();
  return ret;
}
#endif /* CONFIG_NET_LOCAL_SCM */

/****************************************************************************
 * Name: local_sendto
 *
 * Description:
 *   This function implements the Unix domain-specific logic of the
 *   standard sendto() socket operation.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 *   NOTE: All input parameters were verified by sendto() before this
 *   function was called.
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is returned.  See the description in
 *   net/socket/sendto.c for the list of appropriate return value.
 *
 ****************************************************************************/

static ssize_t local_sendto(FAR struct socket *psock,
                            FAR const struct iovec *buf,
                            size_t len, int flags,
                            FAR const struct sockaddr *to,
                            socklen_t tolen)
{
#ifdef CONFIG_NET_LOCAL_DGRAM
  FAR struct local_conn_s *conn = psock->s_conn;
  FAR struct local_conn_s *server;
  FAR const struct sockaddr_un *unaddr = (FAR const struct sockaddr_un *)to;
  FAR const char *path;
  ssize_t ret;

  /* Verify that a valid address has been provided */

  if (to->sa_family != AF_LOCAL || tolen < sizeof(sa_family_t))
    {
      nerr("ERROR: Unrecognized address family: %d\n",
           to->sa_family);
      return -EAFNOSUPPORT;
    }

  /* At present, only standard pathname type address are support */

  if (tolen < sizeof(sa_family_t) + 2)
    {
      /* EFAULT
       * - An invalid user space address was specified for a parameter
       */

      return -EFAULT;
    }

  /* If this is a connected socket, then return EISCONN */

  if (psock->s_type != SOCK_DGRAM)
    {
      nerr("ERROR: Connected socket\n");
      return -EISCONN;
    }

  /* Verify that this is not a connected peer socket.  It need not be
   * bound, however.  If unbound, recvfrom will see this as a nameless
   * connection.  A DGRAM default peer set by connect() is not a state of
   * its own, so it never reaches this check.  A socketpair() is
   * LOCAL_STATE_CONNECTED with its outgoing FIFO already open, which the
   * DEBUGASSERT below would catch, so refuse it here as master does.
   */

  if (conn->lc_state != LOCAL_STATE_UNBOUND &&
      conn->lc_state != LOCAL_STATE_BOUND)
    {
      /* Either not bound to address or it is connected */

      nerr("ERROR: Connected state\n");
      return -EISCONN;
    }

  local_lock();

  server = local_findconn(conn, unaddr);
  if (server == NULL)
    {
      local_unlock();
      nerr("ERROR: No such file or directory\n");
      return -ENOENT;
    }

#ifdef CONFIG_NET_LOCAL_DGRAM
  /* POSIX: for a datagram socket the peer address set by connect() "limits
   * the remote sender for subsequent recv() functions".  Refuse to deliver
   * to a receiver that has connected to somebody else.
   */

  if (server->lc_dconnected &&
      strncmp(server->lc_daddr.sun_path +
              (server->lc_daddr.sun_path[0] == '\0' ? 1 : 0),
              conn->lc_path, UNIX_PATH_MAX - 1) != 0)
    {
      local_unlock();
      nerr("ERROR: Receiver is connected to another peer\n");
      return -EPERM;
    }
#endif

  local_unlock();

  /* Make sure that dgram is sent safely */

  ret = nxmutex_lock(&conn->lc_sendlock);
  if (ret < 0)
    {
      /* May fail because the task was canceled. */

      nerr("ERROR: Failed to get localsocket sendlock: %zd\n", ret);
      return ret;
    }

  /* The outgoing FIFO should not be open */

  DEBUGASSERT(conn->lc_outfile.f_inode == NULL);

  /* An abstract address starts with a NUL, which local_findconn() skips.
   * Skip it here too so that the FIFO name matches the one the receiver
   * bound, instead of being the empty name.
   */

  path = &unaddr->sun_path[unaddr->sun_path[0] == '\0' ? 1 : 0];

  /* Make sure that half duplex FIFO has been created.
   * REVISIT:  Or should be just make sure that it already exists?
   */

  ret = local_create_halfduplex(conn, path, server->lc_rcvsize);
  if (ret < 0)
    {
      nerr("ERROR: Failed to create FIFO for %s: %zd\n", path, ret);
      goto errout_with_lock;
    }

  /* Open the sending side of the transfer */

  ret = local_open_sender(conn, path,
                          _SS_ISNONBLOCK(conn->lc_conn.s_flags) ||
                          (flags & MSG_DONTWAIT) != 0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to open FIFO for %s: %zd\n", path, ret);

      goto errout_with_halfduplex;
    }

  /* Send the preamble */

  ret = local_send_preamble(conn, &conn->lc_outfile, buf, len,
                            server->lc_rcvsize);
  if (ret < 0)
    {
      nerr("ERROR: Failed to send the preamble: %zd\n", ret);
      goto errout_with_sender;
    }

  /* Send the packet */

  ret = local_send_packet(&conn->lc_outfile, buf, len);
  if (ret < 0)
    {
      nerr("ERROR: Failed to send the packet: %zd\n", ret);
      goto errout_with_sender;
    }

errout_with_sender:

  /* Now we can close the write-only socket descriptor */

  file_close(&conn->lc_outfile);
  conn->lc_outfile.f_inode = NULL;

errout_with_halfduplex:

  /* Release our reference to the half duplex FIFO */

  local_release_halfduplex(conn);

errout_with_lock:

  /* Release localsocket sendlock */

  nxmutex_unlock(&conn->lc_sendlock);
  return ret;
#else
  return -EISCONN;
#endif /* CONFIG_NET_LOCAL_DGRAM */
}

/****************************************************************************
 * Name: local_send
 *
 * Description:
 *   Send a local packet as a stream.
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags (ignored for now)
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately (see send() for the
 *   list of errno numbers).
 *
 ****************************************************************************/

static ssize_t local_send(FAR struct socket *psock,
                          FAR const struct iovec *buf,
                          size_t len, int flags)
{
  ssize_t ret;

  switch (psock->s_type)
    {
      case SOCK_STREAM:
        {
          FAR struct local_conn_s *conn = psock->s_conn;

          DEBUGASSERT(buf);

          if (conn->lc_state != LOCAL_STATE_CONNECTED)
            {
              nerr("ERROR: not connected\n");
              return -ENOTCONN;
            }

          if (conn->lc_outfile.f_inode == NULL)
            {
              return -EPIPE;
            }

          /* Serialize concurrent senders on the same socket */

          ret = nxmutex_lock(&conn->lc_sendlock);
          if (ret < 0)
            {
              /* May fail because the task was canceled. */

              return ret;
            }

          ret = local_send_packet(&conn->lc_outfile, buf, len);
          nxmutex_unlock(&conn->lc_sendlock);
        }
        break;

#ifdef CONFIG_NET_LOCAL_DGRAM
      case SOCK_DGRAM:
        {
          FAR struct local_conn_s *conn = psock->s_conn;

          if (!conn->lc_dconnected)
            {
              return -EDESTADDRREQ;
            }

          ret = local_sendto(psock, buf, len, flags,
                  (FAR const struct sockaddr *)&conn->lc_daddr,
                  sizeof(conn->lc_daddr));

          /* connect() checked that the peer existed, so a peer that has
           * gone away since is a refused connection rather than a name
           * that was never there.
           */

          if (ret == -ENOENT)
            {
              ret = -ECONNREFUSED;
            }
        }
        break;
#endif /* CONFIG_NET_LOCAL_DGRAM */

      default:
        {
          ret = -EDESTADDRREQ;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_sendmsg
 *
 * Description:
 *   Implements the sendmsg() operation for the case of the local Unix socket
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      msg to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error, a negated
 *   errno value is returned (see sendmsg() for the list of appropriate error
 *   values.
 *
 ****************************************************************************/

ssize_t local_sendmsg(FAR struct socket *psock, FAR const struct msghdr *msg,
                      int flags)
{
  FAR const struct sockaddr *to = msg->msg_name;
  FAR const struct iovec *buf = msg->msg_iov;
  socklen_t tolen = msg->msg_namelen;
  size_t iovlen = msg->msg_iovlen;
  ssize_t ret;

#ifdef CONFIG_NET_LOCAL_SCM
  FAR struct local_conn_s *conn = psock->s_conn;
  int count = 0;

  if (msg->msg_control &&
      msg->msg_controllen > sizeof(struct cmsghdr))
    {
      /* Stream sockets require data to be sent with control messages */

      if (psock->s_type == SOCK_STREAM)
        {
          size_t total = 0;
          size_t i;

          for (i = 0; i < iovlen; i++)
            {
              total += buf[i].iov_len;
            }

          if (total == 0)
            {
              return -EINVAL;
            }
        }

      count = local_sendctl(conn, msg);
      if (count < 0)
        {
          return count;
        }
    }

  ret = to ? local_sendto(psock, buf, iovlen, flags, to, tolen) :
             local_send(psock, buf, iovlen, flags);

  if (ret < 0 && count > 0)
    {
      local_lock();
      local_freectl(conn, count);
      local_unlock();
    }
#else
  ret = to ? local_sendto(psock, buf, iovlen, flags, to, tolen) :
             local_send(psock, buf, iovlen, flags);
#endif

  return ret;
}
