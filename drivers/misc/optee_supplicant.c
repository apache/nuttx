/****************************************************************************
 * drivers/misc/optee_supplicant.c
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

#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>
#include <nuttx/idr.h>
#include <string.h>

#include "optee.h"
#include "optee_supplicant.h"
#include "optee_msg.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Request structure for RPCs serviced by the supplicant. */

struct optee_supplicant_req
{
  sq_entry_t                  link;
  uint32_t                    func;
  uint32_t                    ret;
  uint32_t                    num_params;
  FAR struct tee_ioctl_param *params;
  sem_t                       c;
};

struct optee_supplicant
{
  mutex_t           mutex;
  int               req_id;
  struct sq_queue_s reqs;
  FAR struct idr_s *idr;
  FAR struct idr_s *shm_idr;
  sem_t             reqs_c;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct optee_supplicant g_optee_supp;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pop_entry
 *
 * Description:
 *   Pop the first request from the request queue, and create unique id.
 *
 * Parameters:
 *   num_params - Number of parameters passed.
 *   id         - Pointer to the unique request id.
 *
 * Returned Value:
 *   A pointer to the request on success or NULL.
 *
 ****************************************************************************/

static FAR struct optee_supplicant_req *pop_entry(uint32_t num_params,
                                                  FAR int *id)
{
  FAR struct optee_supplicant_req *req;

  if (g_optee_supp.req_id != -1)
    {
      /* Mixing sync/async not supported */

      return NULL;
    }

  if (sq_empty(&g_optee_supp.reqs))
    {
      return NULL;
    }

  req = (FAR struct optee_supplicant_req *)sq_remfirst(&g_optee_supp.reqs);

  /* The request can't fit in the supplicant's supplied parameter buffer. */

  if (num_params < req->num_params)
    {
      return NULL;
    }

  *id = idr_alloc(g_optee_supp.idr, req, 0, INT32_MAX);
  if (*id < 0)
    {
      return NULL;
    }

  return req;
}

/****************************************************************************
 * Name: optee_supplicant_request
 *
 * Description:
 *   Create a request with the received parameters from the OP-TEE, add it to
 *   the supplicant's request queue and then wait on the request's semaphore
 *   until it is serviced.
 *
 * Parameters:
 *   func - Requested function for the supplicant to perform
 *   params - Pointer pointer to parameter data.
 *   num_params - Number of parameters passed.
 *
 * Returned Value:
 *   TEE_SUCCESS on success or a global platform api error code on failure.
 *
 ****************************************************************************/

static uint32_t optee_supplicant_request(uint32_t func, uint32_t num_params,
                                         FAR struct tee_ioctl_param *params)
{
  FAR struct optee_supplicant_req req;
  uint32_t ret;

  /* The supplicant isn't running if the shm_idr == NULL. */

  if (NULL == g_optee_supp.shm_idr)
    {
      return TEE_ERROR_COMMUNICATION;
    }

  sem_init(&req.c, 0, 0);
  req.func = func;
  req.num_params = num_params;
  req.params = params;

  nxmutex_lock(&g_optee_supp.mutex);
  sq_addlast(&req.link, &g_optee_supp.reqs);
  nxmutex_unlock(&g_optee_supp.mutex);

  /* Wake supplicant receiver */

  sem_post(&g_optee_supp.reqs_c);

  /* Wait for completion if supplicant is running. */

  while (sem_wait(&req.c) < 0)
    {
    }

  ret = req.ret;
  sem_destroy(&req.c);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: optee_supplicant_init
 *
 * Description:
 *   Initialize supplicant data.
 *
 * Parameters:
 *   shm_idr - A pointer, passed by reference, to the optee driver's shm_idr.
 *             The destruction of the shm_idr will be handled by
 *             optee_close(), so we only need to initialize it in this
 *             context.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void optee_supplicant_init(FAR struct idr_s **shm_idr)
{
  memset(&g_optee_supp, 0, sizeof(g_optee_supp));
  nxmutex_init(&g_optee_supp.mutex);
  nxsem_init(&g_optee_supp.reqs_c, 0, 0);
  sq_init(&g_optee_supp.reqs);
  g_optee_supp.idr = idr_init();
  g_optee_supp.shm_idr = idr_init();
  g_optee_supp.req_id = -1;

  /* Pass .shm_idr to the caller. */

  *shm_idr = g_optee_supp.shm_idr;
}

/****************************************************************************
 * Name: optee_supplicant_uninit
 *
 * Description:
 *   Uninitialize supplicant data.
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void optee_supplicant_uninit(void)
{
  /* The destruction of the idr_s is performed by optee_close(). */

  g_optee_supp.shm_idr = NULL;
  nxmutex_destroy(&g_optee_supp.mutex);
  nxsem_destroy(&g_optee_supp.reqs_c);
  idr_destroy(g_optee_supp.idr);
}

/****************************************************************************
 * Name: optee_supplicant_recv
 *
 * Description:
 *   This function is invoked by an ioctl, used only by the supplicant. It
 *   will obtain the function to be performed and the parameters passed by
 *   OP-TEE.
 *
 * Parameters:
 *   func - Pointer to obtain the function to be performed by the supplicant.
 *   params - Pointer to the parameter data passed to supplicant.
 *   num_params - Pointer with the number of parameters the supplicant can
 *                process, and later updated with the number of parameters
 *                of the OP-TEE's RPC request.
 *
 * Returned Value:
 *   0 on success, a negated errno on failure.
 *
 ****************************************************************************/

int optee_supplicant_recv(FAR uint32_t *func, FAR uint32_t *num_params,
                          FAR struct tee_ioctl_param *params)
{
  FAR struct optee_supplicant_req *req = NULL;
  int id;
  int n;
  uint8_t num_meta = (params->attr == TEE_IOCTL_PARAM_ATTR_META);

  if (0 == num_params)
    {
      return -EINVAL;
    }

  for (n = 0; n < *num_params; n++)
    {
      if (params[n].attr &&
          params[n].attr != TEE_IOCTL_PARAM_ATTR_META)
        {
          return -EINVAL;
        }
    }

  for (; ; )
    {
      nxmutex_lock(&g_optee_supp.mutex);
      req = pop_entry(*num_params - num_meta, &id);
      nxmutex_unlock(&g_optee_supp.mutex);

      if (req)
        {
          break;
        }

      if (sem_wait(&g_optee_supp.reqs_c) < 0)
        {
          return -EINTR;
        }
    }

  if (num_meta)
    {
      params->attr |= TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
      params->a = id;
      params->b = 0;
      params->c = 0;
    }
  else
    {
      nxmutex_lock(&g_optee_supp.mutex);
      g_optee_supp.req_id = id;
      nxmutex_unlock(&g_optee_supp.mutex);
    }

  /* Setup parameters */

  *func = req->func;
  *num_params = req->num_params + num_meta;

  memcpy(params + num_meta, req->params,
         req->num_params * sizeof(params[0]));

  return OK;
}

/****************************************************************************
 * Name: optee_supplicant_send
 *
 * Description:
 *   This function is invoked by an ioctl, used only by the supplicant. It
 *   will update the parameters of the OP-TEE request with the response from
 *   the supplicant.
 *
 * Parameters:
 *   ret - The return value to send to OP-TEE.
 *   params - Contains the response parameters from nuttx.
 *   num_params - Number of parameters.
 *
 * Returned Value:
 *   0 on success, a negated errno on failure.
 *
 ****************************************************************************/

int optee_supplicant_send(uint32_t ret, uint32_t num_params,
                          FAR struct tee_ioctl_param *params)
{
  FAR struct optee_supplicant_req *req;
  int id;
  uint8_t num_meta = 0;
  const uint32_t async_attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT |
                              TEE_IOCTL_PARAM_ATTR_META;

  nxmutex_lock(&g_optee_supp.mutex);

  /* Check the parameters and obtain the request from the idr. */

  if (!num_params)
    {
      nxmutex_unlock(&g_optee_supp.mutex);
      return -EINVAL;
    }

  /* Async. */

  if (g_optee_supp.req_id == -1)
    {
      if (params->attr != async_attr)
        {
          nxmutex_unlock(&g_optee_supp.mutex);
          return -EINVAL;
        }

      id = params->a;
      num_meta = 1;
    }
  else
    {
      /* Sync. */

      id = g_optee_supp.req_id;
      num_meta = 0;
    }

  req = idr_find(g_optee_supp.idr, id);
  if (!req)
    {
      nxmutex_unlock(&g_optee_supp.mutex);
      return -ENOENT;
    }

  if ((num_params - num_meta) != req->num_params)
    {
      nxmutex_unlock(&g_optee_supp.mutex);
      return -EINVAL;
    }

  idr_remove(g_optee_supp.idr, id);
  g_optee_supp.req_id = -1;

  nxmutex_unlock(&g_optee_supp.mutex);

  if (!req)
    {
      return -EINVAL;
    }

  /* Update output and in/out parameters. */

  for (size_t n = 0; n < req->num_params; n++)
    {
      FAR struct tee_ioctl_param *p = &req->params[n];
      FAR struct tee_ioctl_param *r = &params[n + num_meta];

      switch (p->attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK)
        {
          case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT:
          case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT:
            p->a = r->a;
            p->b = r->b;
            p->c = r->c;
            break;

          case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT:
          case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT:
            p->b = r->b;
            break;

          default:
            break;
        }
    }

  req->ret = ret;
  sem_post(&req->c);

  return OK;
}

/****************************************************************************
 * Name: optee_supplicant_alloc
 *
 * Description:
 *   Prepares and creates a request for userspace memory allocation that was
 *   requested by the OP-TEE through an RPC.
 *
 * Parameters:
 *   priv - Pointer to the driver's optee_priv_data struct.
 *   size - Size to allocate.
 *   shm - Passed by reference pointer to shared memory. On success it will
 *         be updated with the shared memory the supplicant allocated.
 *
 * Returned Value:
 *   0 on success, a negated errno on failure.
 *
 ****************************************************************************/

int optee_supplicant_alloc(FAR struct optee_priv_data *priv,
                           size_t size, FAR struct optee_shm **shm)
{
  uint32_t ret;
  struct tee_ioctl_param param;

  param.attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
  param.a = OPTEE_MSG_RPC_SHM_TYPE_APPL;
  param.b = size;
  param.c = 0;

  ret = optee_supplicant_request(OPTEE_MSG_RPC_CMD_SHM_ALLOC, 1, &param);
  if (ret)
    {
      return optee_convert_to_errno(ret);
    }

  if (NULL == g_optee_supp.shm_idr)
    {
      return -ECOMM;
    }

  *shm = idr_find(g_optee_supp.shm_idr, param.c);

  if (NULL == *shm)
    {
      return -ENOENT;
    }

  return OK;
}

/****************************************************************************
 * Name: optee_supplicant_free
 *
 * Description:
 *   Handles freeing of shared memory allocated by the TEE supplicant.
 *
 * Parameters:
 *   shm_id - The id of the shared memory to be freed.
 *
 * Returned Value:
 *   TEE_SUCCESS on success or a global platform api error code on failure.
 *
 ****************************************************************************/

uint32_t optee_supplicant_free(int32_t shm_id)
{
  struct tee_ioctl_param param;
  uint32_t ret;

  param.attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
  param.a = OPTEE_MSG_RPC_SHM_TYPE_APPL;
  param.b = shm_id;
  param.c = 0;

  ret = optee_supplicant_request(OPTEE_MSG_RPC_CMD_SHM_FREE, 1, &param);

  if (ret != TEE_SUCCESS)
    {
      return ret;
    }

  if (NULL == g_optee_supp.shm_idr)
    {
      ret = TEE_ERROR_COMMUNICATION;
      return ret;
    }

  idr_remove(g_optee_supp.shm_idr, shm_id);
  return ret;
}

/****************************************************************************
 * Name: optee_supplicant_cmd
 *
 * Description:
 *   Copy the parameters from OP-TEE's message and then create a request to
 *   supplicant to service this RPC request.
 *
 * Parameters:
 *   priv - Pointer to the driver's optee_priv_data struct.
 *   arg  - Pointer to the RPC message argument, located in a shared page, by
 *          by the secure world. A copy of this message will be sent to the
 *          supplicant process that runs in userspace for further processing.
 *
 * Returned Value:
 *   None.  Result codes are written into arg->ret.
 *
 ****************************************************************************/

void optee_supplicant_cmd(FAR struct optee_priv_data *priv,
                          FAR struct optee_msg_arg *arg)
{
  FAR struct tee_ioctl_param *params;
  int ret = 0;

  arg->ret_origin = TEE_ORIGIN_COMMS;

  params = kmm_zalloc(TEE_IOCTL_PARAM_SIZE(arg->num_params));
  if (!params)
    {
      arg->ret = TEE_ERROR_OUT_OF_MEMORY;
      return;
    }

  if ((ret = optee_from_msg_param(params, arg->num_params, arg->params))
      != 0)
    {
      arg->ret = optee_convert_from_errno(ret);
      goto out;
    }

  arg->ret = optee_supplicant_request(arg->cmd, arg->num_params, params);
  if (arg->ret != TEE_SUCCESS)
    {
      goto out;
    }

  ret = optee_to_msg_param(priv, arg->params, arg->num_params, params);
  arg->ret = optee_convert_from_errno(ret);

out:
  kmm_free(params);
}

/****************************************************************************
 * Name: optee_supplicant_search_shm
 *
 * Description:
 *   Search the supplicant's idr_s for a specific id and return a pointer to
 *   that shared memory if found.
 *
 * Parameters:
 *   id - Shared memory id to search for, in the supplicant's idr_s.
 *
 * Returned Value:
 *   Pointer to the shm with the specified id or NULL.
 *
 ****************************************************************************/

FAR struct optee_shm *optee_supplicant_search_shm(uint32_t id)
{
  return idr_find(g_optee_supp.shm_idr, id);
}
