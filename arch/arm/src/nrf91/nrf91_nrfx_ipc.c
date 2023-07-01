/****************************************************************************
 * arch/arm/src/nrf91/nrf91_nrfx_ipc.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "arm_internal.h"

#include "hardware/nrf91_ipc.h"

#include "nrf91_nrfx_ipc.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static nrfx_ipc_handler_t g_nrfx_ipc_handler = NULL;
static void *g_nrfx_ipc_context = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrfx_ipc_irq_handler
 ****************************************************************************/

static int nrfx_ipc_irq_handler(int irq, void *context, void *arg)
{
  uint32_t regval = 0;
  int      i      = 0;

  regval = getreg32(NRF91_IPC_INTPEND);

  for (i = 0; i < NRF91_IPC_CHANS; i += 1)
    {
      if (regval & IPC_CHAN_ID(i))
        {
          /* Clear EVENT */

          putreg32(0, NRF91_IPC_EVENTS_RECEIVE(i));

          /* Handle event */

          g_nrfx_ipc_handler(i, g_nrfx_ipc_context);
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrfx_ipc_init
 ****************************************************************************/

nrfx_err_t nrfx_ipc_init(uint8_t irq_priority, nrfx_ipc_handler_t handler,
                         void *context)
{
  /* Store arguments */

  g_nrfx_ipc_handler = handler;
  g_nrfx_ipc_context = context;

  /* Attach modem IPC handler and set priority */

  irq_attach(NRF91_IRQ_IPC, nrfx_ipc_irq_handler, 0);
  up_prioritize_irq(NRF91_IRQ_IPC, irq_priority);

#if 0
  /* IPC interrupts non-secure */

  up_secure_irq(NRF91_IRQ_IPC, false);
#endif

  /* Enable IPC interrupts */

  up_enable_irq(NRF91_IRQ_IPC);

  return 0;
}

/****************************************************************************
 * Name: nrfx_ipc_uninit
 ****************************************************************************/

void nrfx_ipc_uninit(void)
{
  up_disable_irq(NRF91_IRQ_IPC);
}

/****************************************************************************
 * Name: nrfx_ipc_config_load
 ****************************************************************************/

void nrfx_ipc_config_load(const nrfx_ipc_config_t *config)
{
  int i = 0;

  /* Configure send events */

  for (i = 0; i < IPC_CONF_NUM; i++)
    {
      putreg32(config->send_task_config[i],
               NRF91_IPC_SEND_CNF(i));
    }

  /* Configure receive events */

  for (i = 0; i < IPC_CONF_NUM; i++)
    {
      putreg32(config->receive_event_config[i],
               NRF91_IPC_RECEIVE_CNF(i));
    }

  /* Enable interrupts for a given channel */

  putreg32(config->receive_events_enabled,
           NRF91_IPC_INTENSET);
}

/****************************************************************************
 * Name: nrfx_ipc_receive_event_enable
 ****************************************************************************/

void nrfx_ipc_receive_event_enable(uint8_t event_index)
{
  /* Enable interrupts for a given channel */

  putreg32(IPC_CHAN_ID(event_index), NRF91_IPC_INTENSET);
}

/****************************************************************************
 * Name: nrfx_ipc_receive_event_disable
 ****************************************************************************/

void nrfx_ipc_receive_event_disable(uint8_t event_index)
{
  /* Disable interrupts for a given channel */

  putreg32(IPC_CHAN_ID(event_index), NRF91_IPC_INTENCLR);
}
