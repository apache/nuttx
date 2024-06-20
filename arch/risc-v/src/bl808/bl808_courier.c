/****************************************************************************
 * arch/risc-v/src/bl808/bl808_courier.c
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
#include <nuttx/arch.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "hardware/bl808_ipc.h"
#include "riscv_internal.h"
#include "chip.h"
#include "bl808_courier.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: courier_interrupt
 *
 * Description:
 *   Interrupt handler for IPC. Reads the IPC message, gets the interrupt
 *   number and dispatches the appropriate handler.
 *
 ****************************************************************************/

static int __courier_interrupt(int irq, void *context, void *arg)
{
  uint32_t msg = getreg32(IPC2_MSG_READ);
  int m0_extirq = msg & BL808_COURIER_IRQN_MASK;
  int irqn = m0_extirq + BL808_M0_IRQ_OFFSET + RISCV_IRQ_SEXT;

  irq_dispatch(irqn, NULL);

  bl808_courier_req_irq_enable(m0_extirq);

  putreg32(msg, IPC2_MSG_ACK);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl808_courier_req_irq_disable
 *
 * Description:
 *   Sends an IPC message to M0 core to enable m0_extirq.
 *
 ****************************************************************************/

void bl808_courier_req_irq_enable(int m0_extirq)
{
  putreg32((m0_extirq & BL808_COURIER_IRQN_MASK)
           | (1 << BL808_INT_SIG_SHIFT)
           | (1 << BL808_INT_EN_SHIFT),
           IPC0_MSG_SEND);
}

/****************************************************************************
 * Name: bl808_courier_req_irq_disable
 *
 * Description:
 *   Sends an IPC message to M0 core to disable m0_extirq.
 *
 ****************************************************************************/

void bl808_courier_req_irq_disable(int m0_extirq)
{
  putreg32((m0_extirq & BL808_COURIER_IRQN_MASK)
           | (1 << BL808_INT_SIG_SHIFT),
           IPC0_MSG_SEND);
}

/****************************************************************************
 * Name: bl808_courier_init
 *
 * Description:
 *   Enables the IPC interrupt on D0 core and attaches its handler.
 *
 ****************************************************************************/

int bl808_courier_init(void)
{
  putreg32((1 << BL808_INT_SIG_SHIFT), IPC2_INT_UNMASK);

  int ret = irq_attach(BL808_IRQ_D0_IPC, __courier_interrupt, NULL);
  if (ret == OK)
    {
      up_enable_irq(BL808_IRQ_D0_IPC);
    }

  return ret;
}
