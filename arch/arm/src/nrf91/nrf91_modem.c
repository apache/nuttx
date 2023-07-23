/****************************************************************************
 * arch/arm/src/nrf91/nrf91_modem.c
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

#include <debug.h>
#include <assert.h>

#include "chip.h"

#include "nrf_modem.h"

#include "nrf91_modem.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ARCH_TRUSTZONE_NONSECURE
#  error NRF91 modem works only in non-secure environment
#endif

#ifndef CONFIG_NRF91_LFCLK_XTAL
#  error NRF91 modem requires using LFXO as the LFCLK source
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nrf91_modem_fault_handler(struct nrf_modem_fault_info *info);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Modem library parameters - must be allocated in data segment */

static const struct nrf_modem_init_params g_init_params =
{
  .shmem =
  {
    .ctrl =
    {
      .base = NRF91_SHMEM_CTRL_BASE,
      .size = NRF91_SHMEM_CTRL_SIZE
    },
    .tx =
    {
      .base = NRF91_SHMEM_TX_BASE,
      .size = NRF91_SHMEM_TX_SIZE
    },
    .rx =
    {
      .base = NRF91_SHMEM_RX_BASE,
      .size = NRF91_SHMEM_RX_SIZE
    },
    .trace =
    {
      .base = NRF91_SHMEM_TRACE_BASE,
      .size = NRF91_SHMEM_TRACE_SIZE
    }
  },
  .ipc_irq_prio  = NVIC_SYSH_PRIORITY_DEFAULT,
  .fault_handler = nrf91_modem_fault_handler
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_modem_fault_handler
 ****************************************************************************/

static void nrf91_modem_fault_handler(struct nrf_modem_fault_info *info)
{
  nerr("Modem hard fault");
  ASSERT(0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_modem_initialize
 ****************************************************************************/

int nrf91_modem_initialize(void)
{
  int ret = OK;

  /* Initialize modem */

  ret = nrf_modem_init(&g_init_params);
  if (ret < 0)
    {
      nerr("nrf_modem_init failed %d\n", ret);
    }

  return ret;
}
