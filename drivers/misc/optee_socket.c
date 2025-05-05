/****************************************************************************
 * drivers/misc/optee_socket.c
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

#include <nuttx/arch.h>
#include <nuttx/net/net.h>
#include <nuttx/kmalloc.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/param.h>
#include <sys/un.h>
#include <netpacket/rpmsg.h>

#include "optee.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OPTEE_SOCKET_MAX_IOVEC_NUM            7

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct optee_socket_priv_data
{
  struct optee_priv_data base;
  struct socket socket;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int optee_socket_recv(FAR struct socket *psock, FAR void *msg,
                             size_t size)
{
  while (size > 0)
    {
      ssize_t n = psock_recv(psock, msg, size, 0);
      if (n <= 0)
        {
          return n < 0 ? n : -EIO;
        }

      size -= n;
      msg = (FAR char *)msg + n;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: optee_transport_init
 *
 * Description:
 *   Perform any initialization actions specific to the transport used
 *   right before the driver is registered.
 *
 * Returned Values:
 *   0 on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int optee_transport_init(void)
{
  return 0;
}

/****************************************************************************
 * Name: optee_transport_open
 *
 * Description:
 *   Perform any transport-specific actions upon driver character device
 *   open. In this case, open the socket and connect to it.
 *
 * Parameters:
 *   priv_  - the optee_priv_data struct to allocate and return by
 *            reference.
 *
 * Returned Values:
 *   0 on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int optee_transport_open(FAR struct optee_priv_data **priv_)
{
  FAR struct optee_socket_priv_data *priv;
#ifdef CONFIG_DEV_OPTEE_LOCAL
  struct sockaddr_un addr;
#else
  struct sockaddr_rpmsg addr;
#endif
  int ret;

  priv = kmm_zalloc(sizeof(struct optee_socket_priv_data));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

#ifdef CONFIG_DEV_OPTEE_LOCAL
  ret = psock_socket(AF_UNIX, SOCK_STREAM, 0, &priv->socket);
#else
  ret = psock_socket(AF_RPMSG, SOCK_STREAM, 0, &priv->socket);
#endif
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  memset(&addr, 0, sizeof(addr));

#ifdef CONFIG_DEV_OPTEE_LOCAL
  addr.sun_family = AF_UNIX;
  strlcpy(addr.sun_path, OPTEE_SERVER_PATH, sizeof(addr.sun_path));
#else
  addr.rp_family = AF_RPMSG;
  strlcpy(addr.rp_name, OPTEE_SERVER_PATH, sizeof(addr.rp_name));
  strlcpy(addr.rp_cpu, CONFIG_OPTEE_REMOTE_CPU_NAME, sizeof(addr.rp_cpu));
#endif

  ret = psock_connect(&priv->socket, (FAR const struct sockaddr *)&addr,
                      sizeof(addr));
  if (ret < 0)
    {
      psock_close(&priv->socket);
      kmm_free(priv);
      return ret;
    }

  priv->base.alignment = 0;
  *priv_ = (FAR struct optee_priv_data *)priv;
  return 0;
}

/****************************************************************************
 * Name: optee_transport_close
 *
 * Description:
 *   Perform any transport-specific actions upon driver character device
 *   close.
 *
 * Parameters:
 *   priv_  - the optee_priv_data struct to close and de-allocate.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void optee_transport_close(FAR struct optee_priv_data *priv_)
{
  FAR struct optee_socket_priv_data *priv =
                  (FAR struct optee_socket_priv_data *)priv_;

  psock_close(&priv->socket);
  kmm_free(priv);
}

/****************************************************************************
 * Name: optee_transport_call
 *
 * Description:
 *   Call OP-TEE OS through the RPMsg/local socket.
 *
 * Parameters:
 *   priv_  - the optee_priv_data struct to use
 *
 * Returned Values:
 *   0 on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int optee_transport_call(FAR struct optee_priv_data *priv_,
                         FAR struct optee_msg_arg *arg)
{
  /* iov[0]: struct opteee_msg_arg + struct optee_msg_param[n]
   * iov[1 - n+1]: shm_mem
   * 0 <= n <= 6
   */

  FAR struct optee_socket_priv_data *priv =
                  (FAR struct optee_socket_priv_data *)priv_;
  size_t arg_size = OPTEE_MSG_GET_ARG_SIZE(arg->num_params);
  size_t shm_size[OPTEE_MAX_PARAM_NUM];
  size_t shm_addr[OPTEE_MAX_PARAM_NUM];
  struct iovec iov[OPTEE_SOCKET_MAX_IOVEC_NUM];
  struct msghdr msghdr;
  unsigned long iovlen = 1;
  unsigned long i;
  int ret;

  memset(iov, 0, sizeof(iov));
  memset(shm_size, 0, sizeof(shm_size));

  iov[0].iov_base = arg;
  iov[0].iov_len = arg_size;

  for (i = 0; i < arg->num_params; i++)
    {
      if (arg->params[i].attr == OPTEE_MSG_ATTR_TYPE_RMEM_INPUT ||
          arg->params[i].attr == OPTEE_MSG_ATTR_TYPE_RMEM_INOUT)
        {
          iov[iovlen].iov_base =
            (FAR void *)(uintptr_t)arg->params[i].u.rmem.shm_ref;
          iov[iovlen].iov_len = arg->params[i].u.rmem.size;
          shm_size[i] = arg->params[i].u.rmem.size;
          shm_addr[i] = arg->params[i].u.rmem.shm_ref;
          iovlen++;
        }
      else if (arg->params[i].attr == OPTEE_MSG_ATTR_TYPE_RMEM_OUTPUT)
        {
          shm_size[i] = arg->params[i].u.rmem.size;
          shm_addr[i] = arg->params[i].u.rmem.shm_ref;
        }
    }

  memset(&msghdr, 0, sizeof(struct msghdr));
  msghdr.msg_iov = iov;
  msghdr.msg_iovlen = iovlen;

  ret = psock_sendmsg(&priv->socket, &msghdr, 0);
  if (ret < 0)
    {
      return ret;
    }

  ret = optee_socket_recv(&priv->socket, arg, arg_size);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < arg->num_params; i++)
    {
      if (arg->params[i].attr == OPTEE_MSG_ATTR_TYPE_RMEM_OUTPUT ||
          arg->params[i].attr == OPTEE_MSG_ATTR_TYPE_RMEM_INOUT)
        {
          size_t size = MIN(arg->params[i].u.rmem.size, shm_size[i]);
          arg->params[i].u.rmem.shm_ref = shm_addr[i];
          ret = optee_socket_recv(&priv->socket,
                                  (FAR void *)(uintptr_t)
                                  arg->params[i].u.rmem.shm_ref, size);
          if (ret < 0)
            {
              return ret;
            }
        }
    }

  return 0;
}
