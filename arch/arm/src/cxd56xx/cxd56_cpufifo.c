/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cpufifo.c
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
#include <nuttx/queue.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/cxd56_cpufifo.h"
#include "cxd56_cpufifo.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#ifdef CONFIG_CXD56_CPUFIFO_ENTRIES
#define NR_PUSHBUFENTRIES   CONFIG_CXD56_CPUFIFO_ENTRIES
#else
#define NR_PUSHBUFENTRIES   8
#endif

#define MSGFROM(x)       (((x)[0] >> 28) & 0xf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cfpushdata_s
{
  sq_entry_t entry;
  uint32_t data[2];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  cpufifo_txhandler(int irq, void *context, void *arg);
static int  cpufifo_rxhandler(int irq, void *context, void *arg);
static int  cpufifo_trypush(uint32_t data[2]);
static int  cpufifo_reserve(uint32_t data[2]);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Only for SYS, GNSS CPUs */

static sq_queue_t          g_pushqueue;
static sq_queue_t          g_emptyqueue;
static struct cfpushdata_s g_pushbuffer[NR_PUSHBUFENTRIES];
static cpufifo_handler_t   g_cfrxhandler;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int cpufifo_txhandler(int irq, void *context, void *arg)
{
  struct cfpushdata_s *pd;

  pd = (struct cfpushdata_s *)sq_remfirst(&g_pushqueue);
  if (pd)
    {
      /* Ignore error because always FIFO is not full at here */

      cpufifo_trypush(pd->data);
      sq_addlast(&pd->entry, &g_emptyqueue);
    }

  if (sq_empty(&g_pushqueue))
    {
      up_disable_irq(CXD56_IRQ_FIFO_TO);
    }

  return OK;
}

static int cpufifo_rxhandler(int irq, void *context, void *arg)
{
  uint32_t word[2] =
                     {
                      0
                     };

  int cpuid;

  /* Drain from PULL FIFO. But not all data because this handler
   * will be re-entered when data remaining in PULL FIFO.
   */

  cxd56_cfpull(word);
  cpuid = MSGFROM(word);

  DEBUGASSERT(cpuid >= 0 && cpuid < 8);

  if (g_cfrxhandler)
    {
      g_cfrxhandler(cpuid, word);
    }

  return OK;
}

static int cpufifo_trypush(uint32_t data[2])
{
  if (getreg32(CXD56_FIF_PUSH_FULL))
    {
      return -1;
    }

  putreg32(data[0], CXD56_FIF_PUSH_WRD0);
  putreg32(data[1], CXD56_FIF_PUSH_WRD1);
  putreg32(1, CXD56_FIF_PUSH_CMP);

  return OK;
}

static int cpufifo_reserve(uint32_t data[2])
{
  struct cfpushdata_s *pd;

  pd = (struct cfpushdata_s *)sq_remfirst(&g_emptyqueue);

  /* This error indicates that need more sending buffer, it can be
   * configured by CONFIG_CXD56_CPUFIFO_ENTRIES.
   */

  if (!pd)
    {
      return -EAGAIN;
    }

  pd->data[0] = data[0];
  pd->data[1] = data[1];
  sq_addlast(&pd->entry, &g_pushqueue);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cxd56_cfpush(uint32_t data[2])
{
  irqstate_t flags;
  int        ret;

  flags = enter_critical_section();
  if (!sq_empty(&g_pushqueue))
    {
      ret = cpufifo_reserve(data);
      leave_critical_section(flags);
      return ret;
    }

  ret = cpufifo_trypush(data);
  if (ret < 0)
    {
      cpufifo_reserve(data);
      up_enable_irq(CXD56_IRQ_FIFO_TO);
    }

  leave_critical_section(flags);

  return OK;
}

int cxd56_cfpull(uint32_t data[2])
{
  if (getreg32(CXD56_FIF_PULL_EMP))
    {
      return -1;
    }

  data[0] = getreg32(CXD56_FIF_PULL_WRD0);
  data[1] = getreg32(CXD56_FIF_PULL_WRD1);
  putreg32(1, CXD56_FIF_PULL_CMP);

  return 0;
}

int cxd56_cfregrxhandler(cpufifo_handler_t handler)
{
  irqstate_t flags;
  int ret = OK;

  flags = enter_critical_section();
  if (g_cfrxhandler)
    {
      ret = -1;
    }
  else
    {
      g_cfrxhandler = handler;
    }

  leave_critical_section(flags);
  return ret;
}

void cxd56_cfunregrxhandler(void)
{
  irqstate_t flags;
  flags = enter_critical_section();
  g_cfrxhandler = NULL;
  leave_critical_section(flags);
}

int cxd56_cfinitialize(void)
{
  int i;

  /* Setup IRQ handlers. Enable only FROM (RX) interrupt because TO (TX)
   * interrupt for retry sending when FIFO is full.
   */

  irq_attach(CXD56_IRQ_FIFO_FROM, cpufifo_rxhandler, NULL);
  irq_attach(CXD56_IRQ_FIFO_TO, cpufifo_txhandler, NULL);
  up_enable_irq(CXD56_IRQ_FIFO_FROM);

  /* Initialize push buffer. */

  sq_init(&g_pushqueue);
  sq_init(&g_emptyqueue);

  for (i = 0; i < NR_PUSHBUFENTRIES; i++)
    {
      sq_addlast((sq_entry_t *)&g_pushbuffer[i], &g_emptyqueue);
    }

  /* Clear user defined receive handler. */

  g_cfrxhandler = NULL;

  return OK;
}
