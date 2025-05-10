/****************************************************************************
 * drivers/misc/optee_smc.c
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

#include <nuttx/kmalloc.h>
#include <errno.h>
#include <syslog.h>
#include <string.h>

#include "optee.h"
#include "optee_smc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ARCH_ARM)
#  define smccc_smc                  arm_smccc_smc
#  define smccc_hvc                  arm_smccc_hvc
#  define smccc_res_t                arm_smccc_res_t
#elif defined(CONFIG_ARCH_ARM64)
#  define smccc_smc                  arm64_smccc_smc
#  define smccc_hvc                  arm64_smccc_hvc
#  define smccc_res_t                arm64_smccc_res_t
#else
#  error "CONFIG_DEV_OPTEE_SMC is only supported on arm and arm64"
#endif

#ifdef CONFIG_DEV_OPTEE_SMC_CONDUIT_SMC
#  define smc_conduit                smccc_smc
#else
#  define smc_conduit                smccc_hvc
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void (*optee_smc_fn)(unsigned long, unsigned long, unsigned long,
                             unsigned long, unsigned long, unsigned long,
                             unsigned long, unsigned long,
                             FAR smccc_res_t *);

struct optee_smc_priv_data
{
  struct optee_priv_data base;
  optee_smc_fn smc_fn;
};

union optee_smc_os_revision
{
  smccc_res_t smccc;
  struct optee_smc_call_get_os_revision_result result;
};

union optee_smc_calls_revision
{
  smccc_res_t smccc;
  struct optee_smc_calls_revision_result result;
};

union optee_smc_exchg_caps
{
  smccc_res_t smccc;
  struct optee_smc_exchange_capabilities_result result;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline FAR void *reg_pair_to_ptr(uint64_t reg0, uint64_t reg1)
{
  return (FAR void *)(uintptr_t)(reg0 << 32 | (reg1 & UINT32_MAX));
}

static inline void reg_pair_from_64(uint64_t val, uint64_t *reg0,
                                    uint64_t *reg1)
{
  *reg0 = val >> 32;
  *reg1 = val & UINT32_MAX;
}

static bool optee_smc_is_compatible(optee_smc_fn smc_fn)
{
  union optee_smc_os_revision osrev;
  union optee_smc_calls_revision callsrev;
  union optee_smc_exchg_caps xchgcaps;
  smccc_res_t callsuid;

  /* Print the OS revision and build ID (if reported) */

  osrev.result.build_id = 0;

  smc_fn(OPTEE_SMC_CALL_GET_OS_REVISION, 0, 0, 0, 0, 0, 0, 0, &osrev.smccc);

  if (osrev.result.build_id)
    {
      syslog(LOG_INFO, "OP-TEE: OS revision %lu.%lu (%08lx)\n",
                       osrev.result.major, osrev.result.minor,
                       osrev.result.build_id);
    }
  else
    {
      syslog(LOG_INFO, "OP-TEE: OS revision %lu.%lu\n",
                       osrev.result.major, osrev.result.minor);
    }

  /* Check the API UID */

  smc_fn(OPTEE_SMC_CALLS_UID, 0, 0, 0, 0, 0, 0, 0, &callsuid);

  if (callsuid.a0 != OPTEE_MSG_UID_0 || callsuid.a1 != OPTEE_MSG_UID_1 ||
      callsuid.a2 != OPTEE_MSG_UID_2 || callsuid.a3 != OPTEE_MSG_UID_3)
    {
      syslog(LOG_ERR, "OP-TEE: API UID mismatch\n");
      return false;
    }

  /* Check the API revision */

  smc_fn(OPTEE_SMC_CALLS_REVISION, 0, 0, 0, 0, 0, 0, 0, &callsrev.smccc);

  if (callsrev.result.major != OPTEE_MSG_REVISION_MAJOR ||
      callsrev.result.minor < OPTEE_MSG_REVISION_MINOR)
    {
      syslog(LOG_ERR, "OP-TEE: API revision incompatible\n");
      return false;
    }

  /* Check the capabilities */

  smc_fn(OPTEE_SMC_EXCHANGE_CAPABILITIES, OPTEE_SMC_NSEC_CAP_UNIPROCESSOR,
         0, 0, 0, 0, 0, 0, &xchgcaps.smccc);

  if (xchgcaps.result.status != OPTEE_SMC_RETURN_OK)
    {
      syslog(LOG_ERR, "OP-TEE: Failed to exchange capabilities\n");
      return false;
    }

  if (!(xchgcaps.result.capabilities & OPTEE_SMC_SEC_CAP_DYNAMIC_SHM))
    {
      syslog(LOG_ERR, "OP-TEE: Does not support dynamic shared mem\n");
      return false;
    }

  return true;
}

static void optee_smc_handle_rpc(FAR struct optee_priv_data *priv_,
                                 FAR smccc_res_t *par)
{
  FAR struct optee_shm *shm;
  uintptr_t shm_pa;
  uint32_t rpc_func;

  rpc_func = OPTEE_SMC_RETURN_GET_RPC_FUNC(par->a0);
  par->a0 = OPTEE_SMC_CALL_RETURN_FROM_RPC;

  switch (rpc_func)
    {
      case OPTEE_SMC_RPC_FUNC_ALLOC:
        if (!optee_shm_alloc(priv_, NULL, par->a1, TEE_SHM_ALLOC, &shm))
          {
            shm_pa = optee_va_to_pa((FAR void *)(uintptr_t)shm->addr);
            reg_pair_from_64(shm_pa, &par->a1, &par->a2);
            reg_pair_from_64((uintptr_t)shm, &par->a4, &par->a5);
          }
        else
          {
            reg_pair_from_64(0, &par->a1, &par->a2);
            reg_pair_from_64(0, &par->a4, &par->a5);
          }
        break;

      case OPTEE_SMC_RPC_FUNC_FREE:
        shm = reg_pair_to_ptr(par->a1, par->a2);
        optee_shm_free(shm);
        break;

      case OPTEE_SMC_RPC_FUNC_FOREIGN_INTR:
        break;

      default:
        syslog(LOG_ERR, "OP-TEE: RPC 0x%04x not implemented\n", rpc_func);
        break;
    }
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
  if (!optee_smc_is_compatible(smc_conduit))
    {
      return -ENOENT;
    }

  return 0;
}

/****************************************************************************
 * Name: optee_transport_open
 *
 * Description:
 *   Perform any transport-specific actions upon driver character device
 *   open.
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
  FAR struct optee_smc_priv_data *priv;

  priv = kmm_zalloc(sizeof(struct optee_smc_priv_data));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->base.alignment = OPTEE_MSG_NONCONTIG_PAGE_SIZE;
  priv->smc_fn = smc_conduit;
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
 *  None
 *
 ****************************************************************************/

void optee_transport_close(FAR struct optee_priv_data *priv_)
{
  kmm_free(priv_);
}

/****************************************************************************
 * Name: optee_transport_call
 *
 * Description:
 *   Call OP-TEE OS using SMCs.
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
  FAR struct optee_smc_priv_data *priv =
                  (FAR struct optee_smc_priv_data *)priv_;
  smccc_res_t res;
  smccc_res_t par;
  uintptr_t arg_pa;

  memset(&par, 0, sizeof(smccc_res_t));

  par.a0 = OPTEE_SMC_CALL_WITH_ARG;
  arg_pa = optee_va_to_pa(arg);
  reg_pair_from_64(arg_pa, &par.a1, &par.a2);

  for (; ; )
    {
      memset(&res, 0, sizeof(smccc_res_t));

      priv->smc_fn(par.a0, par.a1, par.a2, par.a3,
                   par.a4, par.a5, par.a6, par.a7, &res);

      if (OPTEE_SMC_RETURN_IS_RPC(res.a0))
        {
          memcpy(&par, &res, 4 * sizeof(unsigned long));
          optee_smc_handle_rpc(priv_, &par);
        }
      else
        {
          return (int)res.a0;
        }
    }
}
