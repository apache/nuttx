/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cpufifo.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <queue.h>
#include <debug.h>
#include <errno.h>

#include "arm_arch.h"

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
  FAR sq_entry_t entry;
  uint32_t data[2];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  cpufifo_txhandler(int irq, FAR void *context, FAR void *arg);
static int  cpufifo_rxhandler(int irq, FAR void *context, FAR void *arg);
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

static int cpufifo_txhandler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct cfpushdata_s *pd;

  pd = (FAR struct cfpushdata_s *)sq_remfirst(&g_pushqueue);
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

static int cpufifo_rxhandler(int irq, FAR void *context, FAR void *arg)
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
  FAR struct cfpushdata_s *pd;

  pd = (FAR struct cfpushdata_s *)sq_remfirst(&g_emptyqueue);

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
      sq_addlast((FAR sq_entry_t *)&g_pushbuffer[i], &g_emptyqueue);
    }

  /* Clear user defined receive handler. */

  g_cfrxhandler = NULL;

  return OK;
}
