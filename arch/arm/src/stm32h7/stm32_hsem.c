/****************************************************************************
 * arch/arm/src/stm32h7/stm32_hsem.c
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

#include <nuttx/config.h>

#include <debug.h>

#include "nvic.h"

#include "arm_internal.h"
#include "stm32_hsem.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* HSEM receive channel configuration */

struct stm32_hsem_recv_s
{
  hsem_callback_t  callback;
  void            *args;
};

/* HSEM device */

struct stm32_hsem_s
{
  struct stm32_hsem_recv_s recv[STM32_HSEM_CHANS];
  uint32_t                 irq;
  uint8_t                  block;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct stm32_hsem_s g_stm32_hsem;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_hsem_interrupt
 ****************************************************************************/

static int stm32_hsem_interrupt(int irq, void *context, void *args)
{
  struct stm32_hsem_s *dev    = args;
  uint32_t             regval = 0;
  int                  i      = 0;

  regval = getreg32(STM32_HSEM_CXMISR(dev->block));

  ipcinfo("HSEM interrupt 0x%" PRIx32 "\n", regval);

  for (i = 0; i < STM32_HSEM_CHANS; i++)
    {
      if (regval & HSEM_CHAN_ID(i))
        {
          if (dev->recv[i].callback)
            {
              dev->recv[i].callback(i, dev->recv[i].args);
            }
        }
    }

  /* Clear events */

  putreg32(regval, STM32_HSEM_CXICR(dev->block));

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_hsem_subscribe
 *
 * Description:
 *   Subscribe to event on a given semaphore
 *
 ****************************************************************************/

void stm32_hsem_subscribe(uint8_t id, hsem_callback_t callback, void *args)
{
  struct stm32_hsem_s *dev = &g_stm32_hsem;

  DEBUGASSERT(id < STM32_HSEM_CHANS);

  ipcinfo("HSEM subscribe %d\n", id);

  /* Register callaback */

  dev->recv[id].callback = callback;
  dev->recv[id].args = args;

  if (callback)
    {
      /* Clear pending */

      putreg32(HSEM_CHAN_ID(id), STM32_HSEM_CXICR(dev->block));

      /* Enable interrupt */

      modifyreg32(STM32_HSEM_CXIER(dev->block), 0, HSEM_CHAN_ID(id));
    }
  else
    {
      /* Disable interrupts */

      modifyreg32(STM32_HSEM_CXIER(dev->block), HSEM_CHAN_ID(id), 0);
    }
}

/****************************************************************************
 * Name: stm32_hsem_signal
 *
 * Description:
 *   Signal on a semaphore (free and lock again)
 *
 ****************************************************************************/

void stm32_hsem_signal(uint8_t id)
{
  DEBUGASSERT(id < STM32_HSEM_CHANS);

  ipcinfo("HSEM signal %d\n", id);

  /* Lock semaphore */

  while (stm32_hsem_take(id) == 0);

  /* Free semaphore to signal event */

  stm32_hsem_free(id);
}

/****************************************************************************
 * Name: stm32_hsem_busywait_lock
 *
 * Description:
 *   Busy wait for a semaphore to be locked
 *
 ****************************************************************************/

void stm32_hsem_busywait_lock(uint8_t id)
{
  DEBUGASSERT(id < STM32_HSEM_CHANS);

  ipcinfo("HSEM busywait lock %d\n", id);

  /* Wait for semaphore lock */

  while ((getreg32(STM32_HSEM_RX(id)) & HSEM_SEMX_LOCK) == 0);
}

/****************************************************************************
 * Name: stm32_hsem_busywait_free
 *
 * Description:
 *   Busy wait for a semaphore to be free
 *
 ****************************************************************************/

void stm32_hsem_busywait_free(uint8_t id)
{
  DEBUGASSERT(id < STM32_HSEM_CHANS);

  ipcinfo("HSEM busywait free %d\n", id);

  /* Wait for semaphore free */

  while ((getreg32(STM32_HSEM_RX(id)) & HSEM_SEMX_LOCK) != 0);
}

/****************************************************************************
 * Name: stm32_hsem_wait_take
 *
 * Description:
 *   Take a semaphore (1-step lock)
 *
 ****************************************************************************/

void stm32_hsem_wait_take(uint8_t id)
{
  DEBUGASSERT(id < STM32_HSEM_CHANS);

  ipcinfo("HSEM wait take %d\n", id);

  stm32_hsem_busywait_free(id);
  while (stm32_hsem_take(id) == 0);
}

/****************************************************************************
 * Name: stm32_hsem_take
 *
 * Description:
 *   Take a semaphore (1-step lock)
 *
 ****************************************************************************/

bool stm32_hsem_take(uint8_t id)
{
  DEBUGASSERT(id < STM32_HSEM_CHANS);

  ipcinfo("HSEM take %d\n", id);

  /* Take semaphore */

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
  return (getreg32(STM32_HSEM_RLRX(id)) ==
          ((HSEM_COREID_CPU1 << HSEM_SEMX_COREID_SHIFT) | HSEM_SEMX_LOCK));
#else
  return (getreg32(STM32_HSEM_RLRX(id)) ==
          ((HSEM_COREID_CPU2 << HSEM_SEMX_COREID_SHIFT) | HSEM_SEMX_LOCK));
#endif
}

/****************************************************************************
 * Name: stm32_hsem_free
 *
 * Description:
 *   Free a semaphore
 *
 ****************************************************************************/

void stm32_hsem_free(uint8_t id)
{
#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
  putreg32((HSEM_COREID_CPU1 << HSEM_CR_COREID_SHIFT) , STM32_HSEM_RX(id));
#else
  putreg32((HSEM_COREID_CPU2 << HSEM_CR_COREID_SHIFT) , STM32_HSEM_RX(id));
#endif
}

/****************************************************************************
 * Name: stm32_hsem_init
 *
 * Description:
 *   Initialize the hardware semaphore driver
 *
 ****************************************************************************/

void stm32_hsem_init(void)
{
  struct stm32_hsem_s *dev = &g_stm32_hsem;

  /* Reset device */

  memset(dev, 0, sizeof(struct stm32_hsem_s));

  /* Set block id */

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
  dev->block = 0;
  dev->irq   = STM32_IRQ_HSEM0;
#else
  dev->block = 1;
  dev->irq   = STM32_IRQ_HSEM1;
#endif

  /* Attach and enable the IRQ */

  irq_attach(dev->irq, stm32_hsem_interrupt, dev);
  up_enable_irq(dev->irq);
}
