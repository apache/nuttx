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

#include <fcntl.h>
#include <nuttx/tee.h>
#include <nuttx/nuttx.h>
#include <nuttx/drivers/optee.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/lib/math32.h>
#include <sys/mman.h>
#include <sys/param.h>

#ifdef CONFIG_ARCH_ADDRENV
#  include <nuttx/pgalloc.h>
#  include <nuttx/sched.h>
#  include <nuttx/arch.h>
#endif

#include "optee.h"

/****************************************************************************
 * The driver's main purpose is to support the porting of the open source
 * component optee_client (https://github.com/OP-TEE/optee_client) to NuttX.
 * The basic function of the driver module is to convert the REE application
 * layer data and send it to the TEE through rpmsg or SMCs. TEE
 * implementation is optee_os(https://github.com/OP-TEE/optee_os).
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some GlobalPlatform error codes used in this driver */

#define TEE_SUCCESS                    0x00000000
#define TEE_ERROR_ACCESS_DENIED        0xFFFF0001
#define TEE_ERROR_BAD_FORMAT           0xFFFF0005
#define TEE_ERROR_BAD_PARAMETERS       0xFFFF0006
#define TEE_ERROR_NOT_SUPPORTED        0xFFFF000A
#define TEE_ERROR_OUT_OF_MEMORY        0xFFFF000C
#define TEE_ERROR_BUSY                 0xFFFF000D
#define TEE_ERROR_COMMUNICATION        0xFFFF000E
#define TEE_ERROR_SECURITY             0xFFFF000F
#define TEE_ERROR_SHORT_BUFFER         0xFFFF0010
#define TEE_ERROR_TIMEOUT              0xFFFF3001

#define TEE_ORIGIN_COMMS               0x00000002

#define OPTEE_DEV_PATH                 "/dev/tee0"

/* According to optee_msg.h#OPTEE_MSG_ATTR_NONCONTIG */

#define OPTEE_PAGES_ARRAY_LEN \
        ((OPTEE_MSG_NONCONTIG_PAGE_SIZE / sizeof(uint64_t)) - 1)

/* Name: optee_msg_alloc
 *
 * Description:
 *   Allocate OP-TEE page-aligned memory for use as arguments to OP-TEE
 *   calls, large enough to fit `numparams` parameters. Initialize the
 *   buffer to 0, and set the `.num_params` field to the specified value.
 *
 *   Use `optee_msg_free()` to free the memory returned.
 *
 * Parameters:
 *   priv       - pointer to the driver's optee_priv_data struct
 *   numparams  - the number of arguments to allocate shared memory for.
 *                Can be zero.
 *   msh        - pointer to a struct optee_msg_arg to receive the result.
 *                Will be set to NULL on failure.
 */

#define optee_msg_alloc(priv, numparams, msg) \
  do \
    { \
      if ((priv)->alignment > sizeof(uintptr_t)) \
        { \
          (msg) = kmm_memalign((priv)->alignment, \
                               OPTEE_MSG_GET_ARG_SIZE(numparams)); \
        } \
      else \
        { \
          (msg) = alloca(OPTEE_MSG_GET_ARG_SIZE(numparams)); \
        } \
      \
      if (msg) \
        { \
          memset(msg, 0, OPTEE_MSG_GET_ARG_SIZE(numparams)); \
          (msg)->num_params = numparams; \
        } \
    } \
  while (0)

#define optee_msg_free(priv, msg) \
  if ((priv)->alignment > sizeof(uintptr_t)) \
    { \
      kmm_free(msg); \
    }

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* According to optee_msg.h#OPTEE_MSG_ATTR_NONCONTIG documentation */

struct optee_page_list_entry
{
  uint64_t pages_array[OPTEE_PAGES_ARRAY_LEN];
  uint64_t next_entry;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* The file operation functions */

static int optee_open(FAR struct file *filep);
static int optee_close(FAR struct file *filep);
static int optee_ioctl(FAR struct file *filep, int cmd,
                       unsigned long arg);

static int optee_shm_close(FAR struct file *filep);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* File operations */

static const struct file_operations g_optee_ops =
{
  optee_open,      /* open */
  optee_close,     /* close */
  NULL,            /* read */
  NULL,            /* write */
  NULL,            /* seek */
  optee_ioctl,     /* ioctl */
  NULL,            /* mmap */
  NULL,            /* truncate */
  NULL             /* poll */
};

static const struct file_operations g_optee_shm_ops =
{
  NULL,            /* open */
  optee_shm_close, /* close */
  NULL,            /* read */
  NULL,            /* write */
  NULL,            /* seek */
  NULL,            /* ioctl */
  NULL,            /* mmap */
  NULL,            /* truncate */
  NULL             /* poll */
};

static struct inode g_optee_shm_inode =
{
  .i_flags = FSNODEFLAG_TYPE_SHM, /* could also be DRIVER; doesn't matter */
  .i_crefs = 1,                   /* inode never to be released */
  .u.i_ops = &g_optee_shm_ops
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int optee_convert_error(uint32_t oterr)
{
  switch (oterr)
    {
      case TEE_SUCCESS:
        return 0;
      case TEE_ERROR_ACCESS_DENIED:
      case TEE_ERROR_SECURITY:
        return -EACCES;
      case TEE_ERROR_BAD_FORMAT:
      case TEE_ERROR_BAD_PARAMETERS:
        return -EINVAL;
      case TEE_ERROR_NOT_SUPPORTED:
        return -EOPNOTSUPP;
      case TEE_ERROR_OUT_OF_MEMORY:
        return -ENOMEM;
      case TEE_ERROR_BUSY:
        return -EBUSY;
      case TEE_ERROR_COMMUNICATION:
        return -ECOMM;
      case TEE_ERROR_SHORT_BUFFER:
        return -ENOBUFS;
      case TEE_ERROR_TIMEOUT:
        return -ETIMEDOUT;
      default:
        return -EIO;
    }
}

/****************************************************************************
 * Name: optee_is_valid_range
 *
 * Description:
 *   Check whether provided virtual address is not NULL and the address
 *   range belongs to user-owned memory. If this function is called from a
 *   kernel thread, it returns true. If this function is called in a build
 *   without CONFIG_ARCH_ADDRENV it always returns true.
 *
 * Parameters:
 *   va    - Beginning of address range to check.
 *   size  - Size of memory to check.
 *
 * Returned Values:
 *   True if the provided address range is not NULL and belongs to the user
 *   or the caller is a kernel thread. False otherwise.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
static bool optee_is_valid_range(FAR const void *va, size_t size)
{
  FAR struct tcb_s *tcb;
  uint8_t ttype;

  if (va == NULL)
    {
      return false;
    }

  tcb = nxsched_self();
  ttype = tcb->flags & TCB_FLAG_TTYPE_MASK;

  if (ttype == TCB_FLAG_TTYPE_KERNEL)
    {
      return true;
    }

  if (up_addrenv_user_vaddr((uintptr_t)va) &&
      up_addrenv_user_vaddr((uintptr_t)va + size - 1))
    {
      return true;
    }

  return false;
}
#else
#  define optee_is_valid_range(addr, size) (true)
#endif

/****************************************************************************
 * Name: optee_shm_to_page_list
 *
 * Description:
 *   Provide a page list of a shared memory buffer. Secure world doesn't
 *   know about the address environment mapping of NuttX, so physical
 *   pointers are needed when sharing memory. This implementation enables
 *   sharing of physically non-contiguous buffers according to
 *   optee_msg.h#OPTEE_MSG_ATTR_NONCONTIG.
 *   Each entry in the generated page list is an array of the physical,
 *   potentially non-contiguous pages making up the actual buffer to
 *   represent. Note that this representation does not account for the page
 *   offset of the shared memory buffer. The offset is encoded in the
 *   physical address returned in `list_pa`.
 *
 * Parameters:
 *   shm     - Shared memory object to create a page list for.
 *   list_pa - If not NULL, will be set to the page list's physical address
 *             (which is aligned to OPTEE_MSG_NONCONTIG_PAGE_SIZE) added
 *             with shared memory page offset.
 *
 * Returned Values:
 *   A pointer to the kernel virtual address of the page list on success.
 *   NULL on failure. Caller responsible to free the returned list using
 *   `kmm_free()`.
 *
 ****************************************************************************/

static FAR void *
optee_shm_to_page_list(FAR struct optee_shm *shm, FAR uintptr_t *list_pa)
{
  FAR struct optee_page_list_entry *list_entry;
  size_t pgsize = OPTEE_MSG_NONCONTIG_PAGE_SIZE;
  uintptr_t pgoff;
  uintptr_t page;
  FAR void *list;
  uint32_t total_pages;
  uint32_t list_size;
  uint32_t i = 0;

  pgoff = shm->addr & (pgsize - 1);
  total_pages = (uint32_t)div_round_up(pgoff + shm->length, pgsize);
  list_size = div_round_up(total_pages, OPTEE_PAGES_ARRAY_LEN)
              * sizeof(struct optee_page_list_entry);

  /* Page list's address should be page aligned, conveniently leaving
   * log2(<page size>) zero least-significant bits to use as the page
   * offset of the shm buffer (added last before return below).
   */

  list = kmm_memalign(pgsize, list_size);
  if (!list)
    {
      return NULL;
    }

  list_entry = (FAR struct optee_page_list_entry *)list;
  page = ALIGN_DOWN(shm->addr, pgsize);
  while (total_pages)
    {
      list_entry->pages_array[i++] = optee_va_to_pa((FAR const void *)page);
      page += pgsize;
      total_pages--;

      if (i == OPTEE_PAGES_ARRAY_LEN)
        {
          list_entry->next_entry = optee_va_to_pa(list_entry + 1);
          list_entry++;
          i = 0;
        }
    }

  if (list_pa)
    {
      *list_pa = optee_va_to_pa(list) | pgoff;
    }

  return list;
}

/****************************************************************************
 * Name: optee_shm_register
 *
 * Description:
 *   Register specified shared memory object with OP-TEE.
 *
 * Parameters:
 *   priv - The driver's private data structure
 *   shm  - Pointer to shared memory object to register. Neither the shm
 *          object, nor the referenced shared buffer pointer can be NULL.
 *
 * Returned Values:
 *   0 on success, negative error code otherwise.
 *
 ****************************************************************************/

static int optee_shm_register(FAR struct optee_priv_data *priv,
                              FAR struct optee_shm *shm)
{
  FAR struct optee_msg_arg *msg;
  uintptr_t page_list_pa;
  FAR void *page_list;
  int ret = -ENOMEM;

  optee_msg_alloc(priv, 1, msg);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  page_list = optee_shm_to_page_list(shm, &page_list_pa);
  if (page_list == NULL)
    {
      goto errout_with_msg;
    }

  msg->cmd = OPTEE_MSG_CMD_REGISTER_SHM;
  msg->params[0].attr = OPTEE_MSG_ATTR_TYPE_TMEM_OUTPUT |
                        OPTEE_MSG_ATTR_NONCONTIG;
  msg->params[0].u.tmem.buf_ptr = page_list_pa;
  msg->params[0].u.tmem.shm_ref = (uintptr_t)shm;
  msg->params[0].u.tmem.size = shm->length;

  ret = optee_transport_call(priv, msg);
  if (ret < 0)
    {
      goto errout_with_list;
    }

  if (msg->ret)
    {
      ret = optee_convert_error(msg->ret);
    }

errout_with_list:
  kmm_free(page_list);
errout_with_msg:
  optee_msg_free(priv, msg);
  return ret;
}

/****************************************************************************
 * Name: optee_shm_unregister
 *
 * Description:
 *   Unregister specified shared memory object from OP-TEE.
 *
 * Parameters:
 *   priv - the driver's private data structure
 *   shm  - Pointer to shared memory object to unregister. The shared memory
 *          entry must have been previously registered previously with the
 *          OP-TEE OS and cannot be NULL.
 *
 * Returned Values:
 *   0 on success, negative error code otherwise.
 *
 ****************************************************************************/

static int optee_shm_unregister(FAR struct optee_priv_data *priv,
                                FAR struct optee_shm *shm)
{
  FAR struct optee_msg_arg *msg;
  int ret = 0;

  optee_msg_alloc(priv, 1, msg);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  msg->cmd = OPTEE_MSG_CMD_UNREGISTER_SHM;
  msg->params[0].attr = OPTEE_MSG_ATTR_TYPE_RMEM_INPUT;
  msg->params[0].u.rmem.shm_ref = (uintptr_t)shm;

  ret = optee_transport_call(priv, msg);
  if (ret < 0)
    {
      goto errout_with_msg;
    }

  if (msg->ret)
    {
      ret = optee_convert_error(msg->ret);
    }

errout_with_msg:
  optee_msg_free(priv, msg);
  return ret;
}

/****************************************************************************
 * Name: optee_shm_close
 *
 * Description:
 *   shm close operation
 *
 * Parameters:
 *   filep  - the file instance
 *
 * Returned Values:
 *   Always OK.
 *
 ****************************************************************************/

static int optee_shm_close(FAR struct file *filep)
{
  FAR struct optee_shm *shm = filep->f_priv;

  if (shm != NULL && shm->id > -1)
    {
      filep->f_priv = NULL;
      shm->fd = -1;
      optee_shm_free(shm);
    }

  return 0;
}

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
  FAR struct optee_priv_data *priv;
  int ret;

  ret = optee_transport_open(&priv);
  if (ret < 0)
    {
      return ret;
    }

  priv->shms = idr_init();
  filep->f_priv = priv;
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
 *   Always OK.
 *
 ****************************************************************************/

static int optee_close(FAR struct file *filep)
{
  FAR struct optee_priv_data *priv = filep->f_priv;
  FAR struct optee_shm *shm;
  FAR struct file *shm_filep;
  int id = 0;

  idr_for_each_entry(priv->shms, shm, id)
    {
      if (shm->fd > -1 && file_get(shm->fd, &shm_filep) >= 0)
        {
          /* The user did not call close(), prevent vfs auto-close from
           * double-freeing our SHM
           */

          shm_filep->f_priv = NULL;
          file_put(shm_filep);
        }

      optee_shm_free(shm);
    }

  idr_destroy(priv->shms);
  optee_transport_close(priv);
  return 0;
}

static int optee_memref_to_msg_param(FAR struct optee_priv_data *priv,
                                     FAR struct optee_msg_param *mp,
                                     FAR const struct tee_ioctl_param *p)
{
  FAR struct optee_shm *shm;
  uintptr_t page_list_pa;

  if (p->c == TEE_MEMREF_NULL)
    {
      mp->attr = OPTEE_MSG_ATTR_TYPE_TMEM_INPUT + p->attr -
                 TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
      mp->u.tmem.buf_ptr = 0;
      mp->u.tmem.shm_ref = 0;
      mp->u.tmem.size = p->b;
      return 0;
    }

  shm = idr_find(priv->shms, p->c);
  if (shm == NULL)
    {
      return -EINVAL;
    }

  if (shm->flags & TEE_SHM_REGISTER)
    {
      /* registered memory */

      mp->attr = OPTEE_MSG_ATTR_TYPE_RMEM_INPUT + p->attr -
                 TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
      mp->u.rmem.offs = p->a;
      mp->u.rmem.size = p->b;
      mp->u.rmem.shm_ref = (uintptr_t)shm;
    }
  else
    {
      /* non-registered memory (temporary) */

      mp->attr = OPTEE_MSG_ATTR_TYPE_TMEM_INPUT + p->attr -
                 TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
      mp->attr |= OPTEE_MSG_ATTR_NONCONTIG;

      shm->page_list = optee_shm_to_page_list(shm, &page_list_pa);
      if (shm->page_list == NULL)
        {
          return -ENOMEM;
        }

      mp->u.tmem.buf_ptr = page_list_pa;
      mp->u.tmem.shm_ref = (uintptr_t)shm;
      mp->u.tmem.size = shm->length;
    }

  return 0;
}

static int optee_to_msg_param(FAR struct optee_priv_data *priv,
                              FAR struct optee_msg_param *mparams,
                              size_t num_params,
                              FAR const struct tee_ioctl_param *params)
{
  size_t n;
  int ret;

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
            ret = optee_memref_to_msg_param(priv, mp, p);
            if (ret < 0)
              {
                return ret;
              }
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
      FAR struct optee_shm *shm;

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
          case OPTEE_MSG_ATTR_TYPE_TMEM_INPUT:
          case OPTEE_MSG_ATTR_TYPE_TMEM_OUTPUT:
          case OPTEE_MSG_ATTR_TYPE_TMEM_INOUT:
            p->attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT +
                      mp->attr - OPTEE_MSG_ATTR_TYPE_TMEM_INPUT;
            p->b = mp->u.tmem.size;

            shm = (FAR struct optee_shm *)(uintptr_t)mp->u.tmem.shm_ref;
            if (shm && shm->page_list)
              {
                kmm_free(shm->page_list);
                shm->page_list = NULL;
              }
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

static int optee_close_session(FAR struct optee_priv_data *priv,
                               uint32_t session)
{
  FAR struct optee_msg_arg *msg;
  int ret;

  optee_msg_alloc(priv, 0, msg);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  msg->cmd = OPTEE_MSG_CMD_CLOSE_SESSION;
  msg->session = session;

  ret = optee_transport_call(priv, msg);

  optee_msg_free(priv, msg);
  return ret;
}

static int optee_ioctl_open_session(FAR struct optee_priv_data *priv,
                                    FAR struct tee_ioctl_buf_data *buf)
{
  FAR struct tee_ioctl_open_session_arg *arg;
  FAR struct optee_msg_arg *msg;
  int ret;

  if (!optee_is_valid_range(buf, sizeof(*buf)))
    {
      return -EFAULT;
    }

  if (buf->buf_len > TEE_MAX_ARG_SIZE ||
      buf->buf_len < sizeof(struct tee_ioctl_open_session_arg))
    {
      return -EINVAL;
    }

  arg = (FAR struct tee_ioctl_open_session_arg *)(uintptr_t)buf->buf_ptr;

  if (!optee_is_valid_range(arg, buf->buf_len))
    {
      return -EFAULT;
    }

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

  optee_msg_alloc(priv, arg->num_params + 2, msg);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  msg->cmd = OPTEE_MSG_CMD_OPEN_SESSION;
  msg->cancel_id = arg->cancel_id;

  /* Initialize and add the meta parameters needed when opening a
   * session.
   */

  msg->params[0].attr = OPTEE_MSG_ATTR_TYPE_VALUE_INPUT |
                        OPTEE_MSG_ATTR_META;
  msg->params[1].attr = OPTEE_MSG_ATTR_TYPE_VALUE_INPUT |
                        OPTEE_MSG_ATTR_META;
  memcpy(&msg->params[0].u.value, arg->uuid, sizeof(arg->uuid));
  msg->params[1].u.value.c = arg->clnt_login;

  ret = optee_to_msg_param(priv, msg->params + 2, arg->num_params,
                           arg->params);
  if (ret < 0)
    {
      goto errout_with_msg;
    }

  ret = optee_transport_call(priv, msg);
  if (ret < 0)
    {
      goto errout_with_msg;
    }

  ret = optee_from_msg_param(arg->params, arg->num_params, msg->params + 2);
  if (ret < 0)
    {
      optee_close_session(priv, msg->session);
      goto errout_with_msg;
    }

  arg->session = msg->session;
  arg->ret = msg->ret;
  arg->ret_origin = msg->ret_origin;

errout_with_msg:
  optee_msg_free(priv, msg);
  return ret;
}

static int optee_ioctl_invoke(FAR struct optee_priv_data *priv,
                              FAR struct tee_ioctl_buf_data *buf)
{
  FAR struct tee_ioctl_invoke_arg *arg;
  FAR struct optee_msg_arg *msg;
  int ret;

  if (!optee_is_valid_range(buf, sizeof(*buf)))
    {
      return -EFAULT;
    }

  if (buf->buf_len > TEE_MAX_ARG_SIZE ||
      buf->buf_len < sizeof(struct tee_ioctl_invoke_arg))
    {
      return -EINVAL;
    }

  arg = (FAR struct tee_ioctl_invoke_arg *)(uintptr_t)buf->buf_ptr;

  if (!optee_is_valid_range(arg, buf->buf_len))
    {
      return -EFAULT;
    }

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

  optee_msg_alloc(priv, arg->num_params, msg);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  msg->cmd = OPTEE_MSG_CMD_INVOKE_COMMAND;
  msg->func = arg->func;
  msg->session = arg->session;
  msg->cancel_id = arg->cancel_id;

  ret = optee_to_msg_param(priv, msg->params, arg->num_params, arg->params);
  if (ret < 0)
    {
      goto errout_with_msg;
    }

  ret = optee_transport_call(priv, msg);
  if (ret < 0)
    {
      goto errout_with_msg;
    }

  ret = optee_from_msg_param(arg->params, arg->num_params, msg->params);
  if (ret < 0)
    {
      goto errout_with_msg;
    }

  arg->ret = msg->ret;
  arg->ret_origin = msg->ret_origin;

errout_with_msg:
  optee_msg_free(priv, msg);
  return ret;
}

static int
optee_ioctl_close_session(FAR struct optee_priv_data *priv,
                          FAR struct tee_ioctl_close_session_arg *arg)
{
  if (!optee_is_valid_range(arg, sizeof(*arg)))
    {
      return -EFAULT;
    }

  return optee_close_session(priv, arg->session);
}

static int optee_ioctl_version(FAR struct tee_ioctl_version_data *vers)
{
  vers->impl_id = TEE_IMPL_ID_OPTEE;
  vers->impl_caps = TEE_OPTEE_CAP_TZ;
  vers->gen_caps = TEE_GEN_CAP_GP |
                   TEE_GEN_CAP_MEMREF_NULL |
                   TEE_GEN_CAP_REG_MEM;
  return 0;
}

static int optee_ioctl_cancel(FAR struct optee_priv_data *priv,
                              FAR struct tee_ioctl_cancel_arg *arg)
{
  FAR struct optee_msg_arg *msg;
  int ret;

  if (!optee_is_valid_range(arg, sizeof(*arg)))
    {
      return -EFAULT;
    }

  optee_msg_alloc(priv, 0, msg);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  msg->cmd = OPTEE_MSG_CMD_CANCEL;
  msg->session = arg->session;
  msg->cancel_id = arg->cancel_id;
  ret = optee_transport_call(priv, msg);

  optee_msg_free(priv, msg);
  return ret;
}

static int
optee_ioctl_shm_alloc(FAR struct optee_priv_data *priv,
                      FAR struct tee_ioctl_shm_alloc_data *data)
{
  FAR struct optee_shm *shm;
  FAR void *addr;
  int memfd;
  int ret;

  if (!optee_is_valid_range(data, sizeof(*data)))
    {
      return -EFAULT;
    }

  memfd = memfd_create(OPTEE_SERVER_PATH, O_CREAT | O_CLOEXEC);
  if (memfd < 0)
    {
      return get_errno();
    }

  if (ftruncate(memfd, data->size) < 0)
    {
      ret = get_errno();
      goto err;
    }

  addr = mmap(NULL, data->size, PROT_READ | PROT_WRITE, MAP_SHARED,
              memfd, 0);
  if (addr == MAP_FAILED)
    {
      ret = get_errno();
      goto err;
    }

  ret = optee_shm_alloc(priv, addr, data->size, 0, &shm);
  if (ret < 0)
    {
      goto err_with_mmap;
    }

  data->id = shm->id;
  return memfd;

err_with_mmap:
  munmap(addr, data->size);
err:
  close(memfd);
  return ret;
}

static int
optee_ioctl_shm_register(FAR struct optee_priv_data *priv,
                         FAR struct tee_ioctl_shm_register_data *rdata)
{
  FAR struct optee_shm *shm;
  int ret;

  if (!optee_is_valid_range(rdata, sizeof(*rdata)))
    {
      return -EFAULT;
    }

  if (!optee_is_valid_range((FAR void *)(uintptr_t)
                            rdata->addr, rdata->length))
    {
      return -EFAULT;
    }

  if (rdata->flags)
    {
      return -EINVAL;
    }

  ret = optee_shm_alloc(priv, (FAR void *)(uintptr_t)rdata->addr,
                        rdata->length, TEE_SHM_REGISTER, &shm);
  if (ret < 0)
    {
      return ret;
    }

  ret = file_allocate_from_inode(&g_optee_shm_inode, O_CLOEXEC, 0, shm, 0);
  if (ret < 0)
    {
      optee_shm_free(shm);
      return ret;
    }

  shm->fd = ret;
  rdata->id = shm->id;
  return ret;
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
  FAR struct optee_priv_data *priv = filep->f_priv;
  FAR void *parg = (FAR void *)arg;

  switch (cmd)
    {
      case TEE_IOC_VERSION:
        return optee_ioctl_version(parg);
      case TEE_IOC_OPEN_SESSION:
        return optee_ioctl_open_session(priv, parg);
      case TEE_IOC_INVOKE:
        return optee_ioctl_invoke(priv, parg);
      case TEE_IOC_CLOSE_SESSION:
        return optee_ioctl_close_session(priv, parg);
      case TEE_IOC_CANCEL:
        return optee_ioctl_cancel(priv, parg);
      case TEE_IOC_SHM_ALLOC:
        return optee_ioctl_shm_alloc(priv, parg);
      case TEE_IOC_SHM_REGISTER:
        return optee_ioctl_shm_register(priv, parg);
      default:
        return -ENOTTY;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: optee_va_to_pa
 *
 * Description:
 *   Convert the specified virtual address to a physical address. If the
 *   virtual address does not belong to the user, it is assumed to be a
 *   kernel virtual address with a 1-1 mapping and the VA is returned as-is.
 *   The VA is also returned as-is if this is a build without
 *   CONFIG_ARCH_ADDRENV.
 *
 * Parameters:
 *   va    - The virtual address to convert.
 *
 * Returned Values:
 *   The physical address corresponding to the specified VA.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
uintptr_t optee_va_to_pa(FAR const void *va)
{
  FAR arch_addrenv_t *addrenv;
  FAR struct tcb_s *tcb;
  uintptr_t page;

  tcb = nxsched_self();
  addrenv = &tcb->addrenv_curr->addrenv;

  page = up_addrenv_find_page(addrenv, (uintptr_t)va);
  if (page == 0)
    {
      return (uintptr_t)va;
    }

  return page | ((uintptr_t)va & MM_PGMASK);
}
#endif

/****************************************************************************
 * Name: optee_shm_alloc
 *
 * Description:
 *   Allocate and/or register shared memory with the OP-TEE OS.
 *   This function always allocates a shared memory object and adds it
 *   to the tree maintained by the driver, regardless of flags. The rest of
 *   this function's behaviour is determined by `flags`:
 *   - If `TEE_SHM_ALLOC` is specified, then a buffer of length `size` will
 *     be allocated. In this case `addr` will be ignored. The allocated
 *     buffer will be aligned to `priv->alignment`. `TEE_SHM_ALLOC` flag
 *     is reserved for kernel use only.
 *   - If `TEE_SHM_REGISTER` is specified, then the memory specified by
 *     `addr` or allocated through `TEE_SHM_ALLOC`, will be registered with
 *     OP-TEE as dynamic shared memory.
 *
 *   Use `optee_shm_free()` to undo this operation, i.e. to remove the
 *   shared memory boject from driver, and/or free, and/or unregister it
 *   from the OP-TEE OS.
 *
 * Parameters:
 *   priv  - The driver's private data structure
 *   addr  - The address of the shared memory to register with OP-TEE and/or
 *           add to the driver's linked list of shared memory chunks.
 *   size  - The size of the shared memory buffer to allocate/add/register.
 *   flags - Flags specifying the behaviour of this function. Supports
 *           combinations of `TEE_SHM_{ALLOC,REGISTER,SEC_REGISTER}`.
 *   shmp  - Pass-by-reference pointer to return the shared memory object
 *           allocated. Cannot be NULL.
 *
 * Returned Values:
 *   0 on success, negative error code otherwise.
 *
 ****************************************************************************/

int optee_shm_alloc(FAR struct optee_priv_data *priv, FAR void *addr,
                    size_t size, uint32_t flags,
                    FAR struct optee_shm **shmp)
{
  FAR struct optee_shm *shm;
  FAR void *ptr;
  int ret = -ENOMEM;

  shm = kmm_zalloc(sizeof(struct optee_shm));
  if (shm == NULL)
    {
      return ret;
    }

  if (flags & TEE_SHM_ALLOC)
    {
      if (priv->alignment)
        {
          ptr = kmm_memalign(priv->alignment, size);
        }
      else
        {
          ptr = kmm_malloc(size);
        }
    }
  else
    {
      ptr = addr;
    }

  if (ptr == NULL)
    {
      goto err;
    }

  shm->fd = -1;
  shm->priv = priv;
  shm->addr = (uintptr_t)ptr;
  shm->length = size;
  shm->flags = flags;
  shm->id = idr_alloc(priv->shms, shm, 0, 0);

  if (shm->id < 0)
    {
      goto err;
    }

  if (flags & TEE_SHM_REGISTER)
    {
      ret = optee_shm_register(priv, shm);
      if (ret < 0)
        {
          goto err_with_idr;
        }
    }

  *shmp = shm;
  return 0;

err_with_idr:
  idr_remove(priv->shms, shm->id);
err:
  kmm_free(shm);
  if (flags & TEE_SHM_ALLOC)
    {
      kmm_free(ptr);
    }

  return ret;
}

/****************************************************************************
 * Name: optee_shm_free
 *
 * Description:
 *   Free and/or unregister shared memory allocated by `optee_shm_alloc()`.
 *
 * Parameters:
 *   shm  - Pointer to shared memory object to free. Can be NULL, in which
 *          case, this is a no-op.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void optee_shm_free(FAR struct optee_shm *shm)
{
  if (!shm || !shm->priv)
    {
      return;
    }

  if (shm->flags & TEE_SHM_REGISTER)
    {
      optee_shm_unregister(shm->priv, shm);
    }

  if (shm->flags & TEE_SHM_ALLOC)
    {
      kmm_free((FAR void *)(uintptr_t)shm->addr);
    }

  if (!(shm->flags & (TEE_SHM_ALLOC | TEE_SHM_REGISTER)))
    {
      /* allocated by optee_ioctl_shm_alloc(), need to unmap */

      munmap((FAR void *)(uintptr_t)shm->addr, shm->length);
    }

  idr_remove(shm->priv->shms, shm->id);
  kmm_free(shm);
}

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
  int ret;

  ret = optee_transport_init();
  if (ret < 0)
    {
      return ret;
    }

  return register_driver(OPTEE_DEV_PATH, &g_optee_ops, 0666, NULL);
}
