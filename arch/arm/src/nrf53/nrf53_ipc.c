/****************************************************************************
 * arch/arm/src/nrf53/nrf53_ipc.c
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

#include <debug.h>
#include <inttypes.h>
#include <stdint.h>

#include <nuttx/arch.h>

#include "arm_internal.h"
#include "nrf53_ipc.h"
#include "hardware/nrf53_ipc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* IPC receive channel configuration */

struct nrf53_ipc_recv_s
{
  ipc_callback_t callback;
  void *args;
};

/* IPC device */

struct nrf53_ipc_s
{
  struct nrf53_ipc_recv_s recv[NRF53_IPC_CHANS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct nrf53_ipc_s g_nrf53_ipc;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_ipc_interrupt
 ****************************************************************************/

static int nrf53_ipc_interrupt(int irq, void *context, void *args)
{
  struct nrf53_ipc_s *dev    = args;
  uint32_t            regval = 0;
  int                 i      = 0;

  regval = getreg32(NRF53_IPC_INTPEND);

  _info("IPC interrupt 0x%" PRIx32 "\n", regval);

  for (i = 0; i < NRF53_IPC_CHANS; i += 1)
    {
      if (regval & IPC_CHAN_ID(i))
        {
          if (dev->recv[i].callback)
            {
              dev->recv[i].callback(i, dev->recv[i].args);
            }

          /* Clear EVENT */

          putreg32(0, NRF53_IPC_EVENTS_RECEIVE(i));
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_ipc_subscribe
 ****************************************************************************/

void nrf53_ipc_subscribe(int id, ipc_callback_t callback, void *args)
{
  struct nrf53_ipc_s *dev = &g_nrf53_ipc;

  DEBUGASSERT(id < NRF53_IPC_CHANS);

  _info("IPC subscribe %d\n", id);

  /* Register callaback */

  dev->recv[id].callback = callback;
  dev->recv[id].args = args;

  if (callback)
    {
      /* Register as receive channel and enable interrupts */

      putreg32(IPC_CHAN_ID(id), NRF53_IPC_RECEIVE_CNF(id));
      putreg32(IPC_CHAN_ID(id), NRF53_IPC_INTENSET);
    }
  else
    {
      /* Disable interrupts */

      putreg32(IPC_CHAN_ID(id), NRF53_IPC_INTENCLR);
    }
}

/****************************************************************************
 * Name: nrf53_ipc_signal
 ****************************************************************************/

void nrf53_ipc_signal(int id)
{
  DEBUGASSERT(id < NRF53_IPC_CHANS);

  _info("IPC signal %d\n", id);

  putreg32(1, NRF53_IPC_TASKS_SEND(id));
}

/****************************************************************************
 * Name: nrf53_ipc_send_cfg
 ****************************************************************************/

void nrf53_ipc_send_cfg(int id)
{
  DEBUGASSERT(id < NRF53_IPC_CHANS);

  _info("IPC send cfg %d\n", id);

  /* Enable send event on a single IPC channel */

  putreg32(IPC_CHAN_ID(id), NRF53_IPC_SEND_CNF(id));
}

/****************************************************************************
 * Name: nrf53_ipc_init
 ****************************************************************************/

void nrf53_ipc_init(void)
{
  struct nrf53_ipc_s *dev = &g_nrf53_ipc;

  /* Reset device */

  memset(dev, 0, sizeof(struct nrf53_ipc_s));

  /* Attach and enable the IRQ */

  irq_attach(NRF53_IRQ_IPC, nrf53_ipc_interrupt, dev);

  up_enable_irq(NRF53_IRQ_IPC);
}
