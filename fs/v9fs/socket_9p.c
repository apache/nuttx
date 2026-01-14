/****************************************************************************
 * fs/v9fs/socket_9p.c
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

#include <sys/param.h>
#include <nuttx/kmalloc.h>
#include <arpa/inet.h>
#include <nuttx/net/net.h>
#include <nuttx/fs/fs.h>

#include "client.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define V9FS_HEADER_OFFSET 7
#define V9FS_DEAFULT_PORT ":563"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct socket_9p_priv_s
{
  struct v9fs_transport_s transport;
  struct socket psock;
  mutex_t lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int socket_9p_create(FAR struct v9fs_transport_s **transport,
                            FAR const char *args);
static int socket_9p_request(FAR struct v9fs_transport_s *transport,
                             FAR struct v9fs_payload_s *payload);
static void socket_9p_destroy(FAR struct v9fs_transport_s *transport);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct v9fs_transport_ops_s g_socket_9p_transport_ops =
{
  socket_9p_create,  /* create */
  socket_9p_request, /* request */
  socket_9p_destroy, /* close */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: socket_9p_create
 ****************************************************************************/

static int socket_9p_create(FAR struct v9fs_transport_s **transport,
                            FAR const char *args)
{
  FAR struct socket_9p_priv_s *priv;
  struct sockaddr_in sin;
  FAR const char *port;
  FAR const char *addr;
  int ret;

  /* Parse IP and port */

  addr = strstr(args, "tag=");
  if (addr == NULL)
    {
      return -EINVAL;
    }

  addr += 4;
  port = strchr(addr, ':');
  if (port == NULL)
    {
      /* set default port */

      port = V9FS_DEAFULT_PORT;
    }

  priv = kmm_zalloc(sizeof(struct socket_9p_priv_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  ret = psock_socket(PF_INET, SOCK_STREAM, IPPROTO_TCP, &priv->psock);
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  sin.sin_family = PF_INET;
  sin.sin_port = htons(atoi(port + 1));
  sin.sin_addr.s_addr = inet_addr(addr);
  ret = psock_connect(&priv->psock, (FAR const struct sockaddr *)&sin,
                      sizeof(sin));
  if (ret < 0)
    {
      goto out;
    }

  nxmutex_init(&priv->lock);
  priv->transport.ops = &g_socket_9p_transport_ops;
  *transport = &priv->transport;
  return 0;

out:
  psock_close(&priv->psock);
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: socket_9p_request
 ****************************************************************************/

static int socket_9p_request(FAR struct v9fs_transport_s *transport,
                             FAR struct v9fs_payload_s *payload)
{
  FAR struct socket_9p_priv_s *priv =
                              (FAR struct socket_9p_priv_s *)transport;
  struct msghdr msg;
  FAR char *ptr;
  size_t len;
  int ret;

  memset(&msg, 0, sizeof(struct msghdr));
  msg.msg_iov = payload->wiov;
  msg.msg_iovlen = payload->wcount;

  nxmutex_lock(&priv->lock);
  ret = psock_sendmsg(&priv->psock, &msg, 0);
  if (ret < 0)
    {
      goto out;
    }

  ptr = payload->riov[0].iov_base;
  ret = psock_recvfrom(&priv->psock, ptr, V9FS_HEADER_OFFSET,
                       MSG_WAITALL, NULL, NULL);
  if (ret < 0)
    {
      goto out;
    }

  len = v9fs_parse_size(ptr);
  len -= ret;

  if (len > 0)
    {
      /* There is still data left to process */

      int index;

      /* Skip the header */

      payload->riov[0].iov_base += V9FS_HEADER_OFFSET;
      payload->riov[0].iov_len -= V9FS_HEADER_OFFSET;

      for (index = 0; index < payload->rcount; index++)
        {
          payload->riov[index].iov_len =
            MIN(len, payload->riov[index].iov_len);
          len -= payload->riov[index].iov_len;
        }

      msg.msg_iov = payload->riov;
      msg.msg_iovlen = payload->rcount;

      ret = psock_recvmsg(&priv->psock, &msg, MSG_WAITALL);
      if (ret < 0)
        {
          goto out;
        }

      /* Restore the header */

      payload->riov[0].iov_base -= V9FS_HEADER_OFFSET;
      payload->riov[0].iov_len += V9FS_HEADER_OFFSET;
    }

  ret = 0;

out:
  nxmutex_unlock(&priv->lock);
  v9fs_transport_done(payload, ret);
  return 0;
}

/****************************************************************************
 * Name: socket_9p_destroy
 ****************************************************************************/

static void socket_9p_destroy(FAR struct v9fs_transport_s *transport)
{
  FAR struct socket_9p_priv_s *priv =
                              (FAR struct socket_9p_priv_s *)transport;

  psock_close(&priv->psock);
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
}
