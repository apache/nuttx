/****************************************************************************
 * arch/risc-v/src/k230/k230_ipi.c
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

#if !defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_NUTTSBI)

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>

#include "riscv_internal.h"
#include "chip.h"
#include "k230_ipi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define K230_IPI_IRQ(n)        (K230_IRQ_IPI0 + n)
#define K230_SOC_RESET_ADDR    0x91101020ul
#define K230_MBOX_RESET_BIT    17
#define K230_RESET_DELAY_US    100

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct k230_ipi_entry_s
{
  uint16_t        role;        /* role of this node */
  uint16_t        mask;        /* allowed line masks */
  ipi_callback_t  cbfn;        /* the callback function */
  void           *args;        /* argument for callback */
};

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static mutex_t g_ipi_lock = NXMUTEX_INITIALIZER;

static struct k230_ipi_entry_s  g_ipi_confs[K230_IPI_DEVN_MAX + 1] =
{
  0
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int k230_ipi_isr(int irq, void *context, void *args)
{
  uint32_t  devn = (uintptr_t)args;
  bool remote = (g_ipi_confs[devn].role == IPI_ROLE_REMOTE);
  uintptr_t regc = remote ? K230_IPI_M2R_INTCLR(devn):
                            K230_IPI_R2M_INTCLR(devn);
  uintptr_t regs = remote ? K230_IPI_M2R_INTSTS(devn):
                            K230_IPI_R2M_INTSTS(devn);
  uint32_t  stat = getreg32(regs);
  for (uint32_t i = 0; i <= K230_IPI_LINE_MAX; i++)
    {
      if (stat & (3 << (i << 1)))
        {
          g_ipi_confs[devn].cbfn(IPI_COMB(devn, i), g_ipi_confs[devn].args);
          putreg32(i, regc);
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int k230_ipi_init(uintptr_t devn, uint16_t mask, uint16_t role,
                  ipi_callback_t ipcb, void *args)
{
  int ret = -EBUSY;
  bool master = (role == IPI_ROLE_MASTER);
#ifdef CONFIG_K230_IPI_RESET_ON_INIT
  static bool mbox_reset = false;
#endif

  DEBUGASSERT(devn <= K230_IPI_DEVN_MAX);
  DEBUGASSERT(mask);
  DEBUGASSERT(ipcb);
  DEBUGASSERT(role == IPI_ROLE_MASTER || role == IPI_ROLE_REMOTE);

  nxmutex_lock(&g_ipi_lock);
  if (0 == g_ipi_confs[devn].role)
    {
      g_ipi_confs[devn].mask = mask;
      g_ipi_confs[devn].cbfn = ipcb;
      g_ipi_confs[devn].args = args;
      g_ipi_confs[devn].role = role;
      ret = 0;
    }

  nxmutex_unlock(&g_ipi_lock);

  if (0 == ret)
    {
#ifdef CONFIG_K230_IPI_RESET_UPON_INIT
      /* Reset whole mailbox device */

      if (master && !mbox_reset)
        {
          uint32_t val = getreg32(K230_SOC_RESET_ADDR);
          val &= ~(1 << K230_MBOX_RESET_BIT);
          putreg32(val, K230_SOC_RESET_ADDR);
          up_udelay(K230_RESET_DELAY_US);
          val |= (1 << K230_MBOX_RESET_BIT);
          putreg32(val, K230_SOC_RESET_ADDR);
          up_udelay(K230_RESET_DELAY_US);
          mbox_reset = true;
          sinfo("mbox reset @ %lx\n", K230_SOC_RESET_ADDR);
        }

#endif
      /* Attach and enable the IRQ */

      ret = irq_attach(K230_IPI_IRQ(devn), k230_ipi_isr, (void *)devn);
      up_enable_irq(K230_IPI_IRQ(devn));

      /* enable IPI device like RTT/Linux does */

      uint32_t v = (master ? 3u : 1u) | (mask << 16);
      putreg32(v, K230_IPI_R2M_INTEN(devn));
      putreg32(v, K230_IPI_M2R_INTEN(devn));
    }

  sinfo("devn=%ld,lmsk=%x,role=%c,r2me=%x,m2re=%x,ret=%d\n",
        devn, mask, master ? 'M' : 'R',
        getreg32(K230_IPI_R2M_INTEN(devn)),
        getreg32(K230_IPI_M2R_INTEN(devn)), ret);

  return ret;
}

void k230_ipi_notify(uint8_t devn, uint8_t line)
{
  DEBUGASSERT(devn <= K230_IPI_DEVN_MAX);
  DEBUGASSERT(line <= K230_IPI_LINE_MAX);

  nxmutex_lock(&g_ipi_lock);
  uint16_t role = g_ipi_confs[devn].role;
  uint16_t mask = g_ipi_confs[devn].mask;
  nxmutex_unlock(&g_ipi_lock);

  if ((1 << line) & mask)
    {
      bool master = (role == IPI_ROLE_MASTER);
      uintptr_t regs = master ? K230_IPI_M2R_INTSET(devn):
                                K230_IPI_R2M_INTSET(devn);
      putreg32(line, regs);
    }
}

void k230_ipi_finish(uint8_t devn, uint16_t mask)
{
  DEBUGASSERT(devn <= K230_IPI_DEVN_MAX);
  DEBUGASSERT(g_ipi_confs[devn].mask == mask);
  uint16_t role = g_ipi_confs[devn].role;
  DEBUGASSERT(role == IPI_ROLE_MASTER || role == IPI_ROLE_MASTER);

  /* reset device and disable interrupts */

  bool remote = (role == IPI_ROLE_REMOTE);
  putreg32(2u | mask << 16, remote ? K230_IPI_M2R_INTEN(devn):
                                     K230_IPI_R2M_INTEN(devn));
  up_disable_irq(K230_IPI_IRQ(devn));
  irq_detach(K230_IPI_IRQ(devn));

  nxmutex_lock(&g_ipi_lock);
  memset(g_ipi_confs + devn, 0, sizeof(g_ipi_confs[0]));
  nxmutex_unlock(&g_ipi_lock);
  sinfo("devn=%d,mask=%x\n", devn, mask);
}

#endif /* !defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_NUTTSBI) */
