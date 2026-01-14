/****************************************************************************
 * drivers/misc/optee_rpc.c
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

#include <nuttx/signal.h>
#include <nuttx/kmalloc.h>
#include <stdint.h>

#include "optee.h"
#include "optee_msg.h"
#include "optee_rpc.h"

#ifdef CONFIG_DEV_OPTEE_SUPPLICANT
#  include "optee_supplicant.h"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: optee_rpc_cmd_get_time
 *
 * Description:
 *   Return REE wall-clock time (seconds + nanoseconds) to secure world.
 *
 * Parameters:
 *   arg - Pointer to the RPC message argument located in shared memory by
 *         the secure world. The answer will be placed in the same argument.
 *
 * Returned Value:
 *   The time is written to:
 *     arg->params[0].u.value.a   containing seconds since epoch
 *     arg->params[0].u.value.b   containing nanoseconds
 *   Result code is written to arg->ret.
 *
 ****************************************************************************/

static void optee_rpc_cmd_get_time(FAR struct optee_msg_arg *arg)
{
  struct timespec ts;

  /* OP-TEE parameter validation. */

  if (arg->num_params != 1 ||
      (arg->params[0].attr & OPTEE_MSG_ATTR_TYPE_MASK)
      != OPTEE_MSG_ATTR_TYPE_VALUE_OUTPUT)
    {
      arg->ret = TEE_ERROR_BAD_PARAMETERS;
      return;
    }

  if (clock_gettime(CLOCK_REALTIME, &ts) < 0)
    {
      /* Should not happen unless the RTC driver is missing */

      arg->ret = TEE_ERROR_GENERIC;
      return;
    }

  arg->params[0].u.value.a = (uint32_t)ts.tv_sec;   /* Seconds since epoch. */
  arg->params[0].u.value.b = (uint32_t)ts.tv_nsec;  /* Nanoseconds.         */

  arg->ret = TEE_SUCCESS;
}

/****************************************************************************
 * Name: optee_rpc_cmd_suspend
 *
 * Description:
 *   Request from OP-TEE to suspend the current nuttx process.
 *
 * Parameters:
 *   arg - Pointer to the RPC message argument, located in a shared page, by
 *         the secure world, containing the time in msec to sleep.
 *
 * Returned Value:
 *   None.  Result codes are written into arg->ret.
 *
 ****************************************************************************/

static void optee_rpc_cmd_suspend(FAR struct optee_msg_arg *arg)
{
  useconds_t usec_to_wait;

  /* OP-TEE parameter validation. */

  if (arg->num_params != 1 ||
      (arg->params[0].attr & OPTEE_MSG_ATTR_TYPE_MASK) !=
      OPTEE_MSG_ATTR_TYPE_VALUE_INPUT)
    {
      arg->ret = TEE_ERROR_BAD_PARAMETERS;
      return;
    }

  usec_to_wait = arg->params[0].u.value.a * 1000;

  if (usec_to_wait)
    {
      nxsched_usleep(usec_to_wait);
    }

  arg->ret = TEE_SUCCESS;
}

/****************************************************************************
 * Name: optee_rpc_cmd_shm_alloc
 *
 * Description:
 *   Handle OP-TEE's RPC to allocate shared memory.
 *
 * Parameters:
 *   priv - Pointer to the driver's optee_priv_data struct.
 *   arg - Pointer to the RPC message argument, located in a shared page
 *         by the secure world. A copy of this message might be sent to the
 *         supplicant process that runs in userspace for further processing.
 *   last_page_list - Passes by reference a pointer that will be updated with
 *                    the virtual address of the page list.
 *
 * Returned Value:
 *   None.  Result codes are written into arg->ret.
 *   Information about the shared memory is passed through arg->params
 *
 ****************************************************************************/

static void optee_rpc_cmd_shm_alloc(FAR struct optee_priv_data *priv,
                                    FAR struct optee_msg_arg *arg,
                                    FAR void **last_page_list)
{
  FAR struct optee_shm *shm;
  size_t n;
  size_t size;
  int32_t ret = -EINVAL;

  arg->ret_origin = TEE_ORIGIN_COMMS;

  /* OP-TEE parameter validation. */

  if (arg->num_params == 0 ||
      arg->params[0].attr != OPTEE_MSG_ATTR_TYPE_VALUE_INPUT)
    {
      arg->ret = TEE_ERROR_BAD_PARAMETERS;
      return;
    }

  for (n = 1; n < arg->num_params; n++)
    {
      if (arg->params[n].attr != OPTEE_MSG_ATTR_TYPE_NONE)
        {
          arg->ret = TEE_ERROR_BAD_PARAMETERS;
          return;
        }
    }

  size = arg->params[0].u.value.b;
  switch (arg->params[0].u.value.a)
    {
      case OPTEE_MSG_RPC_SHM_TYPE_APPL:
#ifdef CONFIG_DEV_OPTEE_SUPPLICANT
        ret = optee_supplicant_alloc(priv, size, &shm);
        break;
#else
        ret = -ENOTSUP;
#endif
      case OPTEE_MSG_RPC_SHM_TYPE_KERNEL:
        ret = optee_shm_alloc(priv, NULL, size, TEE_SHM_ALLOC, &shm);
        break;
    }

  arg->ret = optee_convert_from_errno(ret);
  if (arg->ret != TEE_SUCCESS)
    {
      return;
    }

  if (shm->flags & TEE_SHM_REGISTER)
    {
      arg->params[0].attr = OPTEE_MSG_ATTR_TYPE_TMEM_OUTPUT |
                            OPTEE_MSG_ATTR_NONCONTIG;
      arg->params[0].u.tmem.buf_ptr = shm->paddr;
      arg->params[0].u.tmem.size = shm->length;
      arg->params[0].u.tmem.shm_ref = (unsigned long)shm;
      *last_page_list = shm->page_list;
    }
  else
    {
      arg->params[0].attr = OPTEE_MSG_ATTR_TYPE_TMEM_OUTPUT;
      arg->params[0].u.tmem.buf_ptr = shm->paddr;
      arg->params[0].u.tmem.size = size;
      arg->params[0].u.tmem.shm_ref = (unsigned long)shm;
    }
}

/****************************************************************************
 * Name: optee_rpc_cmd_shm_free
 *
 * Description:
 *   RPC Request from OP-TEE to free shared memory allocated by nuttx.
 *
 * Parameters:
 *   priv - Pointer to the driver's optee_priv_data struct.
 *   arg  - Pointer to the RPC message argument, located in a shared page
 *          by the secure world. A copy of this message might be sent to the
 *          supplicant process that runs in userspace for further processing.
 *
 * Returned Value:
 *   None.  Result codes are written into arg->ret.
 *
 ****************************************************************************/

static void optee_rpc_cmd_shm_free(FAR struct optee_priv_data *priv,
                                   FAR struct optee_msg_arg *arg)
{
  FAR struct optee_shm *shm;

  arg->ret_origin = TEE_ORIGIN_COMMS;

  /* OP-TEE parameter validation. */

  if (arg->num_params != 1 ||
      arg->params[0].attr != OPTEE_MSG_ATTR_TYPE_VALUE_INPUT)
    {
      arg->ret = TEE_ERROR_BAD_PARAMETERS;
      return;
    }

  shm = (FAR struct optee_shm *)(unsigned long)arg->params[0].u.value.b;

  switch (arg->params[0].u.value.a)
    {
      case OPTEE_MSG_RPC_SHM_TYPE_APPL:
#ifdef CONFIG_DEV_OPTEE_SUPPLICANT
        arg->ret = optee_supplicant_free(shm->id);
        if (arg->ret)
            {
              /* The supplicant either failed or isn't running. */

              return;
            }
#else
        arg->ret = TEE_ERROR_NOT_SUPPORTED;
        return;
#endif
        break;
      case OPTEE_MSG_RPC_SHM_TYPE_KERNEL:
        optee_shm_free(shm);
        arg->ret = TEE_SUCCESS;
        break;
      default:
        arg->ret = TEE_ERROR_BAD_PARAMETERS;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: optee_rpc_handle_cmd
 *
 * Description:
 *   Handler of RPC requests.
 *
 * Parameters:
 *   priv - Pointer to the driver's optee_priv_data struct.
 *   shm  - Contains a pointer to the RPC message argument, located in a
 *          shared page, by the secure world. A copy of this message
 *          might be sent to the supplicant process that runs in userspace
 *          for further processing.
 *   last_page_list - Passes by reference a pointer to the virtual address
 *                    of a page list. The page list can be freed by a
 *                    caller or by this function, depending on the response
 *                    of the OP-TEE to the next SMC.
 *
 * Returned Value:
 *   None. The response to OP-TEE will passed through the shared memory.
 *
 ****************************************************************************/

void optee_rpc_handle_cmd(FAR struct optee_priv_data *priv,
                          FAR struct optee_shm *shm,
                          FAR void **last_page_list)
{
  FAR struct optee_msg_arg *arg;

  DEBUGASSERT(shm != NULL);

  arg = (FAR struct optee_msg_arg *)shm->vaddr;

  switch (arg->cmd)
    {
      case OPTEE_MSG_RPC_CMD_GET_TIME:
        optee_rpc_cmd_get_time(arg);
        break;
      case OPTEE_MSG_RPC_CMD_SUSPEND:
        optee_rpc_cmd_suspend(arg);
        break;
      case OPTEE_MSG_RPC_CMD_SHM_ALLOC:
        kmm_free(*last_page_list);
        *last_page_list = NULL;
        optee_rpc_cmd_shm_alloc(priv, arg, last_page_list);
        break;
      case OPTEE_MSG_RPC_CMD_SHM_FREE:
        optee_rpc_cmd_shm_free(priv, arg);
        break;
      default:
#ifdef CONFIG_DEV_OPTEE_SUPPLICANT
        optee_supplicant_cmd(priv, arg);
#else
        arg->ret = TEE_ERROR_NOT_SUPPORTED;
#endif
    }
}
