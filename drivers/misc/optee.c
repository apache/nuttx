/****************************************************************************
 * drivers/misc/optee.c
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

#include <nuttx/tee.h>

#include <fcntl.h>
#include <netpacket/rpmsg.h>
#include <nuttx/drivers/optee.h>
#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/un.h>

#include "optee_msg.h"

/****************************************************************************
 *   The driver's main purpose is to support the porting of the open source
 * component optee_client (https://github.com/OP-TEE/optee_client) to NuttX.
 *   The basic function of the driver module is to convert
 * the REE application layer data and send it to the TEE through rpmsg.
 * TEE implementation is optee_os(https://github.com/OP-TEE/optee_os).
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some GlobalPlatform error codes used in this driver */

#define TEE_SUCCESS                    0x00000000
#define TEE_ERROR_BAD_PARAMETERS       0xFFFF0006
#define TEE_ERROR_NOT_SUPPORTED        0xFFFF000A
#define TEE_ERROR_COMMUNICATION        0xFFFF000E
#define TEE_ERROR_OUT_OF_MEMORY        0xFFFF000C
#define TEE_ERROR_BUSY                 0xFFFF000D
#define TEE_ERROR_SHORT_BUFFER         0xFFFF0010

#define TEE_ORIGIN_COMMS               0x00000002

#define TEE_IOCTL_PARAM_SIZE(x)        (sizeof(struct tee_ioctl_param) * (x))

#define OPTEE_MAX_IOVEC_NUM             7
#define OPTEE_MAX_PARAM_NUM             6

#define OPTEE_SERVER_PATH               "optee"
#define OPTEE_DEV_PATH                  "/dev/tee0"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* The file operation functions */

static int optee_open(FAR struct file *filep);
static int optee_close(FAR struct file *filep);
static int optee_ioctl(FAR struct file *filep, int cmd,
                       unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* File operations */

static const struct file_operations g_optee_ops =
{
  optee_open,     /* open */
  optee_close,    /* close */
  NULL,           /* read */
  NULL,           /* write */
  NULL,           /* seek */
  optee_ioctl,    /* ioctl */
  NULL,           /* mmap */
  NULL,           /* truncate */
  NULL            /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: optee_open
 *
 * Description:
 *   optee open operation
 *
 * Parameters:
 *   filep  - the file instance
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int optee_open(FAR struct file *filep)
{
  FAR struct socket *psock;
#ifdef CONFIG_DEV_OPTEE_LOCAL
  struct sockaddr_un addr;
#else
  struct sockaddr_rpmsg addr;
#endif
  int ret;

  psock = (FAR struct socket *)kmm_zalloc(sizeof(struct socket));
  if (psock == NULL)
    {
      return -ENOMEM;
    }

#ifdef CONFIG_DEV_OPTEE_LOCAL
  ret = psock_socket(AF_UNIX, SOCK_STREAM, 0, psock);
#else
  ret = psock_socket(AF_RPMSG, SOCK_STREAM, 0, psock);
#endif
  if (ret < 0)
    {
      kmm_free(psock);
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

  ret = psock_connect(psock, (FAR const struct sockaddr *)&addr,
                      sizeof(addr));
  if (ret < 0)
    {
      psock_close(psock);
      kmm_free(psock);
      return ret;
    }

  filep->f_priv = psock;
  return 0;
}

/****************************************************************************
 * Name: optee_close
 *
 * Description:
 *   optee close operation
 *
 * Parameters:
 *   filep  - the file instance
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int optee_close(FAR struct file *filep)
{
  FAR struct socket *psock = filep->f_priv;

  psock_close(psock);
  kmm_free(psock);
  return 0;
}

static int optee_to_msg_param(FAR struct optee_msg_param *mparams,
                              size_t num_params,
                              FAR const struct tee_ioctl_param *params)
{
  size_t n;

  for (n = 0; n < num_params; n++)
    {
      FAR const struct tee_ioctl_param *p = params + n;
      FAR struct optee_msg_param *mp = mparams + n;

      if (p->attr & ~TEE_IOCTL_PARAM_ATTR_MASK)
        {
          return -EINVAL;
        }

      switch (p->attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK)
        {
          case TEE_IOCTL_PARAM_ATTR_TYPE_NONE:
            mp->attr = OPTEE_MSG_ATTR_TYPE_NONE;
            break;
          case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT:
          case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT:
          case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT:
            mp->attr = OPTEE_MSG_ATTR_TYPE_VALUE_INPUT + p->attr -
                       TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
            mp->u.value.a = p->a;
            mp->u.value.b = p->b;
            mp->u.value.c = p->c;
            break;
          case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT:
          case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT:
          case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT:
            mp->attr = OPTEE_MSG_ATTR_TYPE_RMEM_INPUT + p->attr -
                       TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
             if (p->c != TEE_MEMREF_NULL)
              {
                mp->u.rmem.shm_ref = p->c;
              }
            else
              {
                mp->u.rmem.shm_ref = 0;
              }

            mp->u.rmem.size = p->b;
            mp->u.rmem.offs = p->a;
            break;
          default:
            return -EINVAL;
        }
    }

  return 0;
}

static int optee_from_msg_param(FAR struct tee_ioctl_param *params,
                                size_t num_params,
                                FAR const struct optee_msg_param *mparams)
{
  size_t n;

  for (n = 0; n < num_params; n++)
    {
      FAR const struct optee_msg_param *mp = mparams + n;
      FAR struct tee_ioctl_param *p = params + n;

      switch (mp->attr & OPTEE_MSG_ATTR_TYPE_MASK)
        {
          case OPTEE_MSG_ATTR_TYPE_NONE:
            p->attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
            p->a = 0;
            p->b = 0;
            p->c = 0;
            break;
          case OPTEE_MSG_ATTR_TYPE_VALUE_INPUT:
          case OPTEE_MSG_ATTR_TYPE_VALUE_OUTPUT:
          case OPTEE_MSG_ATTR_TYPE_VALUE_INOUT:
            p->attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT +
                      mp->attr - OPTEE_MSG_ATTR_TYPE_VALUE_INPUT;
            p->a = mp->u.value.a;
            p->b = mp->u.value.b;
            p->c = mp->u.value.c;
            break;
          case OPTEE_MSG_ATTR_TYPE_RMEM_INPUT:
          case OPTEE_MSG_ATTR_TYPE_RMEM_OUTPUT:
          case OPTEE_MSG_ATTR_TYPE_RMEM_INOUT:
            p->attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT +
                      mp->attr - OPTEE_MSG_ATTR_TYPE_RMEM_INPUT;
            p->b = mp->u.rmem.size;
            break;
          default:
            return -EINVAL;
        }
    }

  return 0;
}

static int optee_recv(FAR struct socket *psock, FAR void *msg, size_t size)
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

static int optee_send_recv(FAR struct socket *psocket,
                           FAR struct optee_msg_arg *arg)
{
  /* iov[0]: struct opteee_msg_arg + struct optee_msg_param[n]
   * iov[1 - n+1]: shm_mem
   * 0 <= n <= 6
   */

  size_t arg_size = OPTEE_MSG_GET_ARG_SIZE(arg->num_params);
  size_t shm_size[OPTEE_MAX_PARAM_NUM];
  size_t shm_addr[OPTEE_MAX_PARAM_NUM];
  struct iovec iov[OPTEE_MAX_IOVEC_NUM];
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

  ret = psock_sendmsg(psocket, &msghdr, 0);
  if (ret < 0)
    {
      return ret;
    }

  ret = optee_recv(psocket, arg, arg_size);
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
          ret = optee_recv(psocket,
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

static int optee_ioctl_open_session(FAR struct socket *psocket,
                                    FAR struct tee_ioctl_buf_data *buf)
{
  char msg_buf[OPTEE_MSG_GET_ARG_SIZE(OPTEE_MAX_PARAM_NUM)];
  FAR struct tee_ioctl_open_session_arg *arg;
  FAR struct optee_msg_arg *msg;
  int ret;

  if (buf->buf_len > TEE_MAX_ARG_SIZE ||
      buf->buf_len < sizeof(struct tee_ioctl_open_session_arg))
    {
      return -EINVAL;
    }

  arg = (FAR struct tee_ioctl_open_session_arg *)(uintptr_t)buf->buf_ptr;

  if (sizeof(*arg) + TEE_IOCTL_PARAM_SIZE(arg->num_params) !=
      buf->buf_len)
    {
      return -EINVAL;
    }

  if (arg->num_params + 2 > OPTEE_MAX_PARAM_NUM)
    {
      return -EINVAL;
    }

  if (arg->clnt_login >= TEE_IOCTL_LOGIN_REE_KERNEL_MIN &&
      arg->clnt_login <= TEE_IOCTL_LOGIN_REE_KERNEL_MAX)
    {
      return -EPERM;
    }

  arg->ret = TEE_ERROR_COMMUNICATION;
  arg->ret_origin = TEE_ORIGIN_COMMS;

  memset(msg_buf, 0, sizeof(msg_buf));
  msg = (FAR struct optee_msg_arg *)&msg_buf[0];

  msg->cmd = OPTEE_MSG_CMD_OPEN_SESSION;
  msg->cancel_id = arg->cancel_id;
  msg->num_params = arg->num_params + 2;

  /* Initialize and add the meta parameters needed when opening a
   * session.
   */

  msg->params[0].attr = OPTEE_MSG_ATTR_TYPE_VALUE_INPUT |
                        OPTEE_MSG_ATTR_META;
  msg->params[1].attr = OPTEE_MSG_ATTR_TYPE_VALUE_INPUT |
                        OPTEE_MSG_ATTR_META;
  memcpy(&msg->params[0].u.value, arg->uuid, sizeof(arg->uuid));
  msg->params[1].u.value.c = arg->clnt_login;

  ret = optee_to_msg_param(msg->params + 2, arg->num_params, arg->params);
  if (ret < 0)
    {
      return ret;
    }

  ret = optee_send_recv(psocket, msg);
  if (ret < 0)
    {
      return ret;
    }

  ret = optee_from_msg_param(arg->params, arg->num_params,
                             msg->params + 2);
  if (ret < 0)
    {
      return ret;
    }

  arg->session = msg->session;
  arg->ret = msg->ret;
  arg->ret_origin = msg->ret_origin;

  return ret;
}

static int optee_ioctl_invoke(FAR struct socket *psocket,
                              FAR struct tee_ioctl_buf_data *buf)
{
  char msg_buf[OPTEE_MSG_GET_ARG_SIZE(OPTEE_MAX_PARAM_NUM)];
  FAR struct tee_ioctl_invoke_arg *arg;
  FAR struct optee_msg_arg *msg;
  int ret;

  if (buf->buf_len > TEE_MAX_ARG_SIZE ||
      buf->buf_len < sizeof(struct tee_ioctl_invoke_arg))
    {
      return -EINVAL;
    }

  arg = (FAR struct tee_ioctl_invoke_arg *)(uintptr_t)buf->buf_ptr;

  if (sizeof(*arg) + TEE_IOCTL_PARAM_SIZE(arg->num_params) !=
      buf->buf_len)
    {
      return -EINVAL;
    }

  if (arg->num_params > OPTEE_MAX_PARAM_NUM)
    {
      return -EINVAL;
    }

  arg->ret = TEE_ERROR_COMMUNICATION;
  arg->ret_origin = TEE_ORIGIN_COMMS;

  memset(msg_buf, 0, sizeof(msg_buf));
  msg = (FAR struct optee_msg_arg *)&msg_buf[0];

  msg->cmd = OPTEE_MSG_CMD_INVOKE_COMMAND;
  msg->func = arg->func;
  msg->session = arg->session;
  msg->cancel_id = arg->cancel_id;
  msg->num_params = arg->num_params;

  ret = optee_to_msg_param(msg->params, arg->num_params, arg->params);
  if (ret < 0)
    {
      return ret;
    }

  ret = optee_send_recv(psocket, msg);
  if (ret < 0)
    {
      return ret;
    }

  ret = optee_from_msg_param(arg->params, arg->num_params, msg->params);
  if (ret < 0)
    {
      return ret;
    }

  arg->ret = msg->ret;
  arg->ret_origin = msg->ret_origin;

  return ret;
}

static int
optee_ioctl_close_session(FAR struct socket *psocket,
                          FAR struct tee_ioctl_close_session_arg *arg)
{
  struct optee_msg_arg msg;

  memset(&msg, 0, sizeof(struct optee_msg_arg));
  msg.cmd = OPTEE_MSG_CMD_CLOSE_SESSION;
  msg.session = arg->session;
  msg.num_params = 0;

  return optee_send_recv(psocket, &msg);
}

static int optee_ioctl_version(FAR struct tee_ioctl_version_data *vers)
{
  vers->impl_id = TEE_IMPL_ID_OPTEE;
  vers->impl_caps = TEE_OPTEE_CAP_TZ;
  vers->gen_caps = TEE_GEN_CAP_GP | TEE_GEN_CAP_MEMREF_NULL;
  return 0;
}

static int optee_ioctl_cancel(FAR struct socket *psocket,
                              FAR struct tee_ioctl_cancel_arg *arg)
{
  struct optee_msg_arg msg;

  memset(&msg, 0, sizeof(struct optee_msg_arg));
  msg.cmd = OPTEE_MSG_CMD_CANCEL;
  msg.session = arg->session;
  msg.cancel_id = arg->cancel_id;
  return optee_send_recv(psocket, &msg);
}

static int
optee_ioctl_shm_alloc(FAR struct tee_ioctl_shm_alloc_data *data)
{
  int memfd = memfd_create(OPTEE_SERVER_PATH, O_CREAT | O_CLOEXEC);

  if (memfd < 0)
    {
      return get_errno();
    }

  if (ftruncate(memfd, data->size) < 0)
    {
      close(memfd);
      return get_errno();
    }

  data->id = (uintptr_t)mmap(NULL, data->size, PROT_READ | PROT_WRITE,
                             MAP_SHARED, memfd, 0);
  if (data->id == (uintptr_t)MAP_FAILED)
    {
      return get_errno();
    }

  return memfd;
}

/****************************************************************************
 * Name: optee_ioctl
 *
 * Description:
 *   optee ioctl operation
 *
 * Parameters:
 *   filep  - the file instance
 *   cmd    - the ioctl command
 *   arg    - the ioctl arguments
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int optee_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct socket *psock = filep->f_priv;
  FAR void *parg = (FAR void *)arg;

  switch (cmd)
    {
      case TEE_IOC_VERSION:
        return optee_ioctl_version(parg);
      case TEE_IOC_OPEN_SESSION:
        return optee_ioctl_open_session(psock, parg);
      case TEE_IOC_INVOKE:
        return optee_ioctl_invoke(psock, parg);
      case TEE_IOC_CLOSE_SESSION:
        return optee_ioctl_close_session(psock, parg);
      case TEE_IOC_CANCEL:
        return optee_ioctl_cancel(psock, parg);
      case TEE_IOC_SHM_ALLOC:
        return optee_ioctl_shm_alloc(parg);
      default:
        return -ENOTTY;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: optee_register
 *
 * Description:
 *   optee client initialize function, the client cpu should call
 *   this function in the board initialize process.
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int optee_register(void)
{
  return register_driver(OPTEE_DEV_PATH, &g_optee_ops, 0666, NULL);
}
