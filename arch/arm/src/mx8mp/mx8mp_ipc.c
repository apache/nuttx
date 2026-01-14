/****************************************************************************
 * arch/arm/src/mx8mp/mx8mp_ipc.c
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

#include "mx8mp_ipc.h"

#include <debug.h>
#include <inttypes.h>
#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <stdint.h>

#include "arm_internal.h"
#include "hardware/mx8mp_mu.h"
#include "hardware/mx8mp_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MX8MP_IPC_CHANS   8
#define RPMSG_MU_CHANNEL  1

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* IPC receive channel configuration */

struct mx8mp_ipc_recv_s
{
  ipc_callback_t callback;
  void *args;
};

/* IPC device */

struct mx8mp_ipc_s
{
  struct mx8mp_ipc_recv_s recv[MX8MP_IPC_CHANS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct mx8mp_ipc_s g_mx8mp_ipc;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_ipc_disable_all_interrupts
 ****************************************************************************/

static void mx8mp_ipc_disable_all_interrupts(void)
{
  uint32_t reg = getreg32(MX8M_MUB + MU_CR_OFFSET);
  putreg32(reg & ~(MU_CR_GIE_MASK | MU_CR_RIE_MASK | MU_CR_TIE_MASK),
           MX8M_MUB + MU_CR_OFFSET);
}

/****************************************************************************
 * Name: mx8mp_ipc_mu_enable_interrupt
 ****************************************************************************/

static void mx8mp_ipc_mu_enable_interrupt(uint32_t index)
{
  uint32_t reg = getreg32(MX8M_MUB + MU_CR_OFFSET);

  /* GIR bits must be masked! */

  putreg32((reg & ~MU_CR_GIR_MASK) | (1UL << MU_CR_RIE0_SHIFT) >> index,
           MX8M_MUB + MU_CR_OFFSET);
}

/****************************************************************************
 * Name: mx8mp_ipc_mu_data_ready
 ****************************************************************************/

static bool mx8mp_ipc_mu_data_ready(int id)
{
  return (((1UL << MU_SR_RF0_SHIFT) >> id)
            & getreg32(MX8M_MUB + MU_SR_OFFSET)) != 0UL;
}

/****************************************************************************
 * Name: mx8mp_ipc_mu_tx_empty
 ****************************************************************************/

static bool mx8mp_ipc_mu_tx_empty(int index)
{
  return (getreg32(MX8M_MUB + MU_SR_OFFSET)
             & ((1UL << MU_SR_TE0_SHIFT) >> index)) != 0UL;
}

/****************************************************************************
 * Name: mx8mp_ipc_mu_send_msg
 ****************************************************************************/

static void mx8mp_ipc_mu_send_msg(uint32_t index, uint32_t msg)
{
  putreg32(msg, MX8M_MUB + MU_TR_OFFSET + (index * sizeof(uint32_t)));
}

/****************************************************************************
 * Name: mx8mp_ipc_interrupt
 ****************************************************************************/

static int mx8mp_ipc_interrupt(int irq, void *context, void *args)
{
  struct mx8mp_ipc_s *dev = args;
  if (!mx8mp_ipc_mu_data_ready(RPMSG_MU_CHANNEL))
    {
      return 0;
    }

  uint32_t channel = getreg32(MX8M_MUB + MU_RR_OFFSET
                               + (RPMSG_MU_CHANNEL * sizeof(uint32_t)))
                     >> 16;
  if (channel >= MX8MP_IPC_CHANS)
    {
      return 0;
    }

  if (dev->recv[channel].callback)
    {
      dev->recv[channel].callback(channel, dev->recv[channel].args);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_ipc_subscribe
 ****************************************************************************/

void mx8mp_ipc_subscribe(int id, ipc_callback_t callback, void *args)
{
  struct mx8mp_ipc_s *dev = &g_mx8mp_ipc;

  DEBUGASSERT(id < MX8MP_IPC_CHANS);

  ipcinfo("IPC subscribe %d\n", id);

  /* Register callback */

  dev->recv[id].callback = callback;
  dev->recv[id].args = args;
}

/****************************************************************************
 * Name: mx8mp_ipc_signal
 ****************************************************************************/

void mx8mp_ipc_signal(int id)
{
  DEBUGASSERT(id < MX8MP_IPC_CHANS);

  ipcinfo("IPC signal %d\n", id);

  /* Wait TX register to be empty. */

  while (!mx8mp_ipc_mu_tx_empty(id))
    {
    }

  mx8mp_ipc_mu_send_msg(1, id);
}

/****************************************************************************
 * Name: mx8mp_ipc_init
 ****************************************************************************/

void mx8mp_ipc_init(void)
{
  struct mx8mp_ipc_s *dev = &g_mx8mp_ipc;

  /* Reset device */

  memset(dev, 0, sizeof(struct mx8mp_ipc_s));

  mx8mp_ipc_disable_all_interrupts();

  /* Attach and enable the IRQ */

  irq_attach(MX8MP_IRQ_MU1_M7, mx8mp_ipc_interrupt, dev);
  up_enable_irq(MX8MP_IRQ_MU1_M7);

  mx8mp_ipc_mu_enable_interrupt(RPMSG_MU_CHANNEL);
}
