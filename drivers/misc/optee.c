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
#include <nuttx/cache.h>
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

#ifdef CONFIG_DEV_OPTEE_SUPPLICANT
#  include "optee_supplicant.h"
#endif

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

#define OPTEE_DEV_PATH                 "/dev/tee0"
#define OPTEE_SUPPLICANT_DEV_PATH      "/dev/teepriv0"

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

/* File operation functions for /dev/tee* */

static int optee_open(FAR struct file *filep);
static int optee_close(FAR struct file *filep);
static int optee_ioctl(FAR struct file *filep, int cmd,
                       unsigned long arg);

/* File operation functions for shm fds. */

static int optee_shm_close(FAR struct file *filep);
static int optee_shm_mmap(FAR struct file *filep,
                          FAR struct mm_map_entry_s *map);

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
  .close = optee_shm_close,
  .mmap = optee_shm_mmap,
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

  pgoff = shm->vaddr & (pgsize - 1);
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
  page = ALIGN_DOWN(shm->vaddr, pgsize);
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

#ifndef CONFIG_ARCH_USE_MMU
  up_clean_dcache((uintptr_t)list, (uintptr_t)list + list_size);
#endif

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
      ret = optee_convert_to_errno(msg->ret);
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
      ret = optee_convert_to_errno(msg->ret);
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
      optee_shm_free(shm);
    }

  return 0;
}

/****************************************************************************
 * Name: optee_shm_mmap
 *
 * Description:
 *   shm mmap operation
 *
 * Parameters:
 *   filep  - the file instance
 *   map    - Filled by the userspace, with the mapping parameters.
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int optee_shm_mmap(FAR struct file *filep,
                          FAR struct mm_map_entry_s *map)
{
  FAR struct optee_shm *shm = filep->f_priv;
  int32_t ret = OK;

  if ((map->flags & MAP_PRIVATE) && (map->flags & MAP_SHARED))
    {
      return -EINVAL;
    }

  /* Forbid multiple mmaps of the same fd. */

  if (shm->vaddr != 0)
    {
      return -EBUSY;
    }

  /* Forbid allocations with bigger size than what registered with
   * with optee_ioctl_shm_alloc().
   */

  if (shm->length < map->length)
    {
      return -EINVAL;
    }

  ret = map_anonymous(map, false);

  if (ret == OK)
    {
      DEBUGASSERT(map->vaddr != NULL);
      shm->vaddr = (uint64_t)map->vaddr;
      shm->paddr = optee_va_to_pa(map->vaddr);
    }

  return ret;
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
  enum optee_role_e role = (uintptr_t)filep->f_inode->i_private;
  int ret;

  ret = optee_transport_open(&priv);
  if (ret < 0)
    {
      return ret;
    }

  priv->role = role;

  if (role == OPTEE_ROLE_CA)
    {
      priv->shms = idr_init();
    }
#ifdef CONFIG_DEV_OPTEE_SUPPLICANT
  else if (role == OPTEE_ROLE_SUPPLICANT)
    {
      /* Allow only one process to open the device. */

      if (filep->f_inode->i_crefs > 2)
        {
          return -EBUSY;
        }

      optee_supplicant_init(&priv->shms);
    }
#endif
  else
    {
      return -EOPNOTSUPP;
    }

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
  int id = 0;

  idr_for_each_entry(priv->shms, shm, id)
    {
      /* Here, we only free, unfreed kernel allocations, the rest will be
       * done by optee_shm_close().
       */

      if (shm->fd == -1)
        {
          optee_shm_free(shm);
        }
    }

  idr_destroy(priv->shms);
  optee_transport_close(priv);
#ifdef CONFIG_DEV_OPTEE_SUPPLICANT
  if (priv->role == OPTEE_ROLE_SUPPLICANT)
    {
      optee_supplicant_uninit();
    }
#endif

  return 0;
}

static int optee_memref_to_msg_param(FAR struct optee_priv_data *priv,
                                     FAR struct optee_msg_param *mp,
                                     FAR const struct tee_ioctl_param *p)
{
  FAR struct optee_shm *shm;
  uintptr_t page_list_pa;
  bool external_vm_context = false;

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
#ifdef CONFIG_DEV_OPTEE_SUPPLICANT
      /* Search also the shared memory registered by the supplicant. */

      shm = optee_supplicant_search_shm(p->c);
      external_vm_context = true;

      if (shm == NULL)
#endif
        {
          return -EINVAL;
        }
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

      /* This shouldn't happen, we can't translate vmas from
       * another vm context.
       */

      if (external_vm_context)
        {
          return -EPROTO;
        }

      shm->page_list = optee_shm_to_page_list(shm, &page_list_pa);
      if (shm->page_list == NULL)
        {
          return -ENOMEM;
        }

      mp->u.tmem.buf_ptr = page_list_pa;
      mp->u.tmem.shm_ref = (uintptr_t)shm;
      mp->u.tmem.size = p->b;
    }

#ifndef CONFIG_ARCH_USE_MMU
  up_clean_dcache(shm->vaddr, shm->vaddr + shm->length);
#endif

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
  int ret;

  if (!optee_is_valid_range(data, sizeof(*data)))
    {
      return -EFAULT;
    }

  ret = optee_shm_alloc(priv, NULL, data->size, TEE_SHM_USER_MAP, &shm);
  if (ret < 0)
    {
      return ret;
    }

  ret = file_allocate_from_inode(&g_optee_shm_inode,
                                 O_CLOEXEC | O_RDOK, 0, shm, 0);

  if (ret < 0)
    {
      optee_shm_free(shm);
      return ret;
    }

  shm->fd = ret;

  data->id = shm->id;
  return shm->fd;
}

#ifdef CONFIG_DEV_OPTEE_SUPPLICANT
static int
optee_shm_register_supplicant(FAR struct optee_priv_data *priv,
                              uintptr_t addr, uint64_t length,
                              FAR struct optee_shm **shmp)
{
  FAR struct optee_shm *shm;
  uintptr_t page_list_pa;
  int ret = 0;

  shm = kmm_zalloc(sizeof(struct optee_shm));
  *shmp = shm;
  if (shm == NULL)
    {
      return -ENOMEM;
    }

  shm->fd = -1;
  shm->priv = priv;
  shm->vaddr = addr;
  shm->length = length;
  shm->flags = TEE_SHM_REGISTER | TEE_SHM_SUPP;
  shm->page_list = optee_shm_to_page_list(shm, &page_list_pa);
  shm->paddr = page_list_pa;
  shm->id = idr_alloc(priv->shms, shm, 0, 0);

  if (shm->id < 0)
    {
      kmm_free(shm->page_list);
      kmm_free(shm);
      return -ENOMEM;
    }

  return ret;
}
#endif

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

  if (priv->role == OPTEE_ROLE_CA)
    {
      ret = optee_shm_alloc(priv, (FAR void *)(uintptr_t)rdata->addr,
                            rdata->length, TEE_SHM_REGISTER, &shm);
    }

#ifdef CONFIG_DEV_OPTEE_SUPPLICANT
  else if (priv->role == OPTEE_ROLE_SUPPLICANT)
    {
      ret = optee_shm_register_supplicant(priv, (uintptr_t)rdata->addr,
                                          rdata->length, &shm);
      rdata->flags = shm->flags;
    }
#endif
  else
    {
      return -ENODEV;
    }

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

#ifdef CONFIG_DEV_OPTEE_SUPPLICANT
static int
optee_ioctl_supplicant_recv(FAR struct optee_priv_data *priv,
                            FAR struct tee_ioctl_buf_data *data)
{
  int n;
  int ret;
  FAR struct tee_iocl_supp_recv_arg *arg;

  if (!optee_is_valid_range(data, sizeof(*data)))
    {
      return -EFAULT;
    }

  if (!optee_is_valid_range((FAR void *)data->buf_ptr, data->buf_len))
    {
      return -EFAULT;
    }

  if (data->buf_len > TEE_MAX_ARG_SIZE ||
      data->buf_len < sizeof(struct tee_iocl_supp_recv_arg))
    {
      return -EINVAL;
    }

  arg = (FAR struct tee_iocl_supp_recv_arg *)(uintptr_t)data->buf_ptr;

  if (sizeof(*arg) + TEE_IOCTL_PARAM_SIZE(arg->num_params) !=
      data->buf_len)
    {
      return -EINVAL;
    }

  if (arg->num_params > OPTEE_MAX_PARAM_NUM)
    {
      return -EINVAL;
    }

  ret = optee_supplicant_recv(&arg->func, &arg->num_params, arg->params);

  if (ret < 0)
    {
      return ret;
    }

  for (n = 0; n < arg->num_params; n++)
    {
      FAR struct tee_ioctl_param *p = arg->params + n;

      switch (p->attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK)
        {
          case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT:
          case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT:
          case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT:
            if (!p->b)
              {
                p->a = 0;
                p->c = (uint64_t)-1; /* Invalid shm id. */
                break;
              }
          break;
        default:
          break;
        }
    }

  return 0;
}

static int
optee_ioctl_supplicant_send(FAR struct optee_priv_data *priv,
                            FAR struct tee_ioctl_buf_data *data)
{
  FAR struct tee_iocl_supp_send_arg *arg;

  if (!optee_is_valid_range(data, sizeof(*data)))
    {
      return -EFAULT;
    }

  if (!optee_is_valid_range((FAR void *)data->buf_ptr, data->buf_len))
    {
      return -EFAULT;
    }

  if (data->buf_len > TEE_MAX_ARG_SIZE ||
      data->buf_len < sizeof(struct tee_iocl_supp_send_arg))
    {
      return -EINVAL;
    }

  arg = (FAR struct tee_iocl_supp_send_arg *)(uintptr_t)data->buf_ptr;

  if (sizeof(*arg) + TEE_IOCTL_PARAM_SIZE(arg->num_params) !=
      data->buf_len)
    {
      return -EINVAL;
    }

  if (arg->num_params > OPTEE_MAX_PARAM_NUM)
    {
      return -EINVAL;
    }

  return optee_supplicant_send(arg->ret, arg->num_params, arg->params);
}
#endif

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
#ifdef CONFIG_DEV_OPTEE_SUPPLICANT
      case TEE_IOC_SUPPL_RECV:
        return optee_ioctl_supplicant_recv(priv, parg);
      case TEE_IOC_SUPPL_SEND:
        return optee_ioctl_supplicant_send(priv, parg);
#endif
      default:
        return -ENOTTY;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: optee_convert_to_errno
 *
 * Description:
 *   Convert TEE errors to errno values
 *
 * Parameters:
 *   oterr - TEE error code.
 *
 * Returned Values:
 *   The converted errno value.
 *
 ****************************************************************************/

int optee_convert_to_errno(uint32_t oterr)
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
 * Name: optee_convert_from_errno
 *
 * Description:
 *   Convert errno values to TEE errors.
 *
 * Parameters:
 *   err - errno value (negative).
 *
 * Returned Values:
 *   The converted TEE error code.
 *
 ****************************************************************************/

uint32_t optee_convert_from_errno(int err)
{
  /* Make sure we handle negative errno values */

  switch (-err)
    {
      case 0:
        return TEE_SUCCESS;
      case EACCES:
        return TEE_ERROR_ACCESS_DENIED;
      case EINVAL:
        return TEE_ERROR_BAD_PARAMETERS;
      case ENOTSUP:
      case EOPNOTSUPP:
        return TEE_ERROR_NOT_SUPPORTED;
      case ENOMEM:
        return TEE_ERROR_OUT_OF_MEMORY;
      case EBUSY:
        return TEE_ERROR_BUSY;
      case ECOMM:
      case EPROTO:
        return TEE_ERROR_COMMUNICATION;
      case ENOBUFS:
        return TEE_ERROR_SHORT_BUFFER;
      case ETIMEDOUT:
        return TEE_ERROR_TIMEOUT;
      default:
        return TEE_ERROR_GENERIC;
    }
}

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

  if (!(flags & TEE_SHM_USER_MAP) && ptr == NULL)
    {
      return -EINVAL;
    }

  shm->fd = -1;
  shm->priv = priv;
  shm->vaddr = (uintptr_t)ptr;
  shm->paddr = shm->vaddr ? optee_va_to_pa((FAR void *)shm->vaddr) : 0;
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

  if (!(shm->flags & TEE_SHM_SUPP) && (shm->flags & TEE_SHM_REGISTER))
    {
      optee_shm_unregister(shm->priv, shm);
    }

  if (shm->flags & TEE_SHM_ALLOC)
    {
      kmm_free((FAR void *)(uintptr_t)shm->vaddr);
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

#ifdef CONFIG_DEV_OPTEE_SUPPLICANT
  ret = register_driver(OPTEE_SUPPLICANT_DEV_PATH, &g_optee_ops, 0666,
                        (FAR void *)OPTEE_ROLE_SUPPLICANT);
  if (ret < 0)
    {
      return ret;
    }
#endif

  return register_driver(OPTEE_DEV_PATH, &g_optee_ops, 0666,
                         (FAR void *)OPTEE_ROLE_CA);
}

/****************************************************************************
 * Name: optee_from_msg_param
 *
 * Description:
 *   Converts and copies the message parameters received by OP-TEE to buffer
 *   for processing by nuttx.
 *
 * Parameters:
 *   params - Pointer, to copy the received parameters after some processing.
 *   num_params - Number of these parameters.
 *   mparams - Pointer to the message parameters received by OP-TEE.
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int optee_from_msg_param(FAR struct tee_ioctl_param *params,
                         size_t num_params,
                         FAR const struct optee_msg_param *mparams)
{
  size_t n;

  for (n = 0; n < num_params; n++)
    {
      FAR const struct optee_msg_param *mp = mparams + n;
      FAR struct tee_ioctl_param *p = params + n;
      FAR struct optee_shm *shm = NULL;

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
                p->c = shm->id;
              }
            else
              {
                p->c = TEE_MEMREF_NULL;
              }
            break;
          case OPTEE_MSG_ATTR_TYPE_RMEM_INPUT:
          case OPTEE_MSG_ATTR_TYPE_RMEM_OUTPUT:
          case OPTEE_MSG_ATTR_TYPE_RMEM_INOUT:
            p->attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT +
                      mp->attr - OPTEE_MSG_ATTR_TYPE_RMEM_INPUT;
            p->b = mp->u.rmem.size;
            p->a = mp->u.rmem.offs;
            shm = (FAR struct optee_shm *)(uintptr_t)mp->u.tmem.shm_ref;
            if (shm)
              {
                p->c = shm->id;
              }
            else
              {
                p->c = TEE_MEMREF_NULL;
              }
            break;
          default:
            return -EINVAL;
        }

#ifndef CONFIG_ARCH_USE_MMU
          if (shm)
            {
              up_invalidate_dcache(shm->vaddr, shm->vaddr + shm->length);
            }
#endif
    }

  return 0;
}

/****************************************************************************
 * Name: optee_to_msg_param
 *
 * Description:
 *   Converts and copies the processed by nuttx parameters to the shared
 *   memory area containing the message to/from the OP-TEE.
 *
 * Parameters:
 *   priv - pointer to the driver's optee_priv_data struct
 *   mparams - Pointer to the message parameters provided by OP-TEE.
 *   params - Pointer, of the processed by nuttx parameters containing the
 *            response.
 *   num_params - Number of these parameters.
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int optee_to_msg_param(FAR struct optee_priv_data *priv,
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

