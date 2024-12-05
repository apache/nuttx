/***************************************************************************
 * arch/arm/src/armv7-a/arm_cpu_psci.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include <debug.h>
#include <assert.h>
#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/chip/chip.h>
#include <nuttx/spinlock.h>

#include "arch/syscall.h"

#include "arm_cpu_psci.h"

/***************************************************************************
 * Private Data
 ***************************************************************************/

static struct psci_interface psci_data;

/***************************************************************************
 * Private Functions
 ***************************************************************************/

static int psci_to_dev_err(int ret)
{
  switch (ret)
    {
    case PSCI_RET_SUCCESS:
      {
        return 0;
      }

    case PSCI_RET_NOT_SUPPORTED:
      {
        return -ENOTSUP;
      }

    case PSCI_RET_INVALID_PARAMS:
    case PSCI_RET_INVALID_ADDRESS:
      {
        return -EINVAL;
      }

    case PSCI_RET_DENIED:
      {
        return -EPERM;
      }
    }

  return -EINVAL;
}

static unsigned long __invoke_psci_fn_hvc(unsigned long function_id,
                                          unsigned long arg0,
                                          unsigned long arg1,
                                          unsigned long arg2)
{
  struct arm_smccc_res res;

  arm_smccc_hvc(function_id, arg0, arg1, arg2, 0, 0, 0, 0, &res);
  return res.a0;
}

static unsigned long __invoke_psci_fn_smc(unsigned long function_id,
                                          unsigned long arg0,
                                          unsigned long arg1,
                                          unsigned long arg2)
{
  struct arm_smccc_res res;

  arm_smccc_smc(function_id, arg0, arg1, arg2, 0, 0, 0, 0, &res);
  return res.a0;
}

static uint32_t psci_get_version(void)
{
  return psci_data.invoke_psci_fn(PSCI_0_2_FN_PSCI_VERSION, 0, 0, 0);
}

static int set_conduit_method(const char *method)
{
  if (!strcmp("hvc", method))
    {
      psci_data.conduit = SMCCC_CONDUIT_HVC;
      psci_data.invoke_psci_fn = __invoke_psci_fn_hvc;
    }
  else if (!strcmp("smc", method))
    {
      psci_data.conduit = SMCCC_CONDUIT_SMC;
      psci_data.invoke_psci_fn = __invoke_psci_fn_smc;
    }
  else
    {
      serr("Invalid conduit method");
      return -EINVAL;
    }

  return 0;
}

static int psci_detect(void)
{
  uint32_t ver = psci_get_version();

  sinfo("Detected PSCI v%ld.%ld\n",
        PSCI_VERSION_MAJOR(ver), PSCI_VERSION_MINOR(ver));

  if (PSCI_VERSION_MAJOR(ver) == 0 && PSCI_VERSION_MINOR(ver) < 2)
    {
      serr("PSCI unsupported version");
      return -ENOTSUP;
    }

  psci_data.version = ver;

  return 0;
}

/***************************************************************************
 * Public Functions
 ***************************************************************************/

uint32_t psci_version(void)
{
  return psci_data.version;
}

int psci_cpu_off(void)
{
  int ret;

  if (psci_data.conduit == SMCCC_CONDUIT_NONE)
    {
      return -EINVAL;
    }

  ret = psci_data.invoke_psci_fn(PSCI_0_2_FN_CPU_OFF, 0, 0, 0);

  return psci_to_dev_err(ret);
}

int psci_cpu_on(unsigned long cpuid, uintptr_t entry_point)
{
  int ret;

  if (psci_data.conduit == SMCCC_CONDUIT_NONE)
    {
      return -EINVAL;
    }

  ret = psci_data.invoke_psci_fn(PSCI_0_2_FN_CPU_ON,
                                 cpuid, (unsigned long)entry_point, 0);

  return psci_to_dev_err(ret);
}

int psci_sys_reset(void)
{
  int ret;

  if (psci_data.conduit == SMCCC_CONDUIT_NONE)
    {
      return -EINVAL;
    }

  ret = psci_data.invoke_psci_fn(PSCI_0_2_FN_SYSTEM_RESET, 0, 0, 0);

  return psci_to_dev_err(ret);
}

int psci_sys_poweroff(void)
{
  int ret;

  if (psci_data.conduit == SMCCC_CONDUIT_NONE)
    {
      return -EINVAL;
    }

  ret = psci_data.invoke_psci_fn(PSCI_0_2_FN_SYSTEM_OFF, 0, 0, 0);

  return psci_to_dev_err(ret);
}

int arm_psci_init(const char *method)
{
  psci_data.conduit = SMCCC_CONDUIT_NONE;

  if (set_conduit_method(method))
    {
      return -ENOTSUP;
    }

  return psci_detect();
}

/***************************************************************************
 * Name: up_systempoweroff
 *
 * Description:
 *   Internal, arm poweroff logic.
 *
 ***************************************************************************/

void up_systempoweroff(void)
{
  int ret;

  /* Set up for the system poweroff */

  ret = psci_sys_poweroff();
  if (ret)
    {
      sinfo("Failed to power off CPU, error code: %d\n", ret);
    }

  /* Wait for power off */

  for (; ; );
}

/***************************************************************************
 * Name: up_systemreset
 *
 * Description:
 *   Internal, arm reset logic.
 *
 ***************************************************************************/

void up_systemreset(void)
{
  int ret;

  /* Set up for the system reset */

  ret = psci_sys_reset();
  if (ret)
    {
      sinfo("Failed to reset CPU, error code: %d\n", ret);
    }

  /* Wait for the reset */

  for (; ; );
}
