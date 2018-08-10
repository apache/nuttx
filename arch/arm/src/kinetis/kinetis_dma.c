/****************************************************************************
 *  arch/arm/src/kinetis/kinetis_dma.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *            Jan Okle <jan@leitwert.ch>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <sys/types.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "kinetis_config.h"
#include "chip.h"
#include "kinetis_dma.h"
#include "chip/kinetis_dmamux.h"
#include "chip/kinetis_sim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef DMA_CHN_PER_GROUP
#   define DMA_CHN_PER_GROUP KINETIS_NDMACH /* Number of channels per group */
#endif

#ifndef CONFIG_DMA_PRI
#  define CONFIG_DMA_PRI NVIC_SYSH_PRIORITY_DEFAULT
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

 struct kinetis_dma_ch
 {
    bool used;
    uint8_t ind;
    uint8_t irq;
    enum kinetis_dma_direction_e dir;
    enum kinetis_dma_data_sz_e data_sz;
    dma_callback_t callback;
    void *arg;
 };

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct kinetis_dma_ch g_channels[KINETIS_NDMACH];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int kinetis_dmainterrupt_int(int irq, void *context,
                                    struct kinetis_dma_ch *ch)
{
  /* Clear bit in the interrupt */

  putreg8(ch->ind, KINETIS_DMA_CINT);

  /* Invoke the callback */

  if (ch->callback)
    {
      ch->callback((DMA_HANDLE)&ch, ch->arg, 0);
    }

  return OK;
}

static int kinetis_dmainterrupt(int irq, void *context, void *arg)
{
  uint8_t irq_int = *(uint8_t *)arg;
  uint32_t regval;
  regval = getreg32(KINETIS_DMA_INT);

  /* Channel irq_int and irq_int + DMA_CHN_PER_GROUP use the same arg. Check
   * which one requested an interrupt
   */

  if ((regval & (1 << irq_int)) != 0)
    {
      kinetis_dmainterrupt_int(irq, context, &g_channels[irq_int]);
    }

  if ((regval & (1 << (irq_int + DMA_CHN_PER_GROUP))) != 0)
    {
      kinetis_dmainterrupt_int(irq, context,
                               &g_channels[irq_int + DMA_CHN_PER_GROUP]);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

size_t kinetis_dmaresidual(DMA_HANDLE handle)
{
  struct kinetis_dma_ch *ch = (struct kinetis_dma_ch *)handle;

  /* Channel Linking Disabled */

  return ((getreg16(KINETIS_DMA_TCD_CITER(ch->ind)) >> DMA_TCD_CITER2_SHIFT) &
          DMA_TCD_CITER2_MASK);
}

/****************************************************************************
 * Name: kinetis_dmainitialize
 *
 * Description:
 *   Initialize the DMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function up_dmainitialize(void)
{
  int i;
  uint32_t regval;
  int ret;

  for (i = KINETIS_NDMACH - 1; i >= 0; i--)
    {
      g_channels[i].ind = i;
      g_channels[i].used = false;
      g_channels[i].irq = KINETIS_IRQ_FIRST + (i % DMA_CHN_PER_GROUP);

      if (i < DMA_CHN_PER_GROUP)
        {
#ifdef CONFIG_ARCH_IRQPRIO
          /* Set up the interrupt priority */

          up_prioritize_irq(g_channels[i].irq, CONFIG_DMA_PRI);
#endif

          /* Attach DMA interrupt */

          ret = irq_attach(g_channels[i].irq, kinetis_dmainterrupt,
                           (void *)&g_channels[i].ind);

          if (ret == OK)
            {
              /* Enable the IRQ at the NVIC (still disabled at the DMA
               * controller)
               */

              up_enable_irq(g_channels[i].irq);
            }
         else
            {
              g_channels[i].used = true;
              g_channels[i + DMA_CHN_PER_GROUP].used = true;
            }
        }
    }

  /* Enable clocking for DMA */

  regval  = getreg32(KINETIS_SIM_SCGC7);
  regval |= SIM_SCGC7_DMA;
  putreg32(regval, KINETIS_SIM_SCGC7);

  /* Configure DMA for round robin arbitration */

  regval  = 0;
  regval |= DMA_CR_ERCA | DMA_CR_ERGA;
  putreg32(regval, KINETIS_DMA_CR);

  /* Enable clocking for the DMA mux*/

  regval  = getreg32(KINETIS_SIM_SCGC6);
  regval |= SIM_SCGC6_DMAMUX0;
  putreg32(regval, KINETIS_SIM_SCGC6);
}

/****************************************************************************
 * Name: kinetis_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   One success, this function returns a non-NULL, void* DMA channel
 *   handle.  NULL is returned on any failure.  This function can fail only
 *   if no DMA channel is available.
 *
 ****************************************************************************/

DMA_HANDLE kinetis_dmachannel(uint8_t src, uint32_t per_addr,
                              enum kinetis_dma_data_sz_e per_data_sz,
                              enum kinetis_dma_direction_e dir)
{
  int i;
  int ch_ind;
  uint8_t regval8;
  uint16_t regval16;
  irqstate_t flags;
  struct kinetis_dma_ch *ch;

  /* Find available channel */

  ch_ind = -1;
  flags = enter_critical_section();
  for (i = 0; i < KINETIS_NDMACH; i++)
    {
      if (!g_channels[i].used)
        {
          ch_ind = i;
          g_channels[ch_ind].used = true;
          break;
        }
    }

  leave_critical_section(flags);

  if (ch_ind == -1)
    {
      /* No available channel */

      return NULL;
    }

  ch = &g_channels[ch_ind];

  /* Copy arguments */

  ch->dir = dir;
  ch->data_sz = per_data_sz;

  /* DMAMUX Set DMA channel source and enable it */

  regval8 = ((((uint8_t)src) << DMAMUX_CHCFG_SOURCE_SHIFT) &
             DMAMUX_CHCFG_SOURCE_MASK) | DMAMUX_CHCFG_ENBL;
  putreg8(regval8, KINETIS_DMAMUX_CHCFG(ch_ind));

  /* DMA  Set peripheral address in TCD */

  if (ch->dir == KINETIS_DMA_DIRECTION_PERIPHERAL_TO_MEMORY)
    {
      putreg32(per_addr, KINETIS_DMA_TCD_SADDR(ch->ind));
      putreg16(0, KINETIS_DMA_TCD_SOFF(ch->ind));
      putreg32(0, KINETIS_DMA_TCD_SLAST(ch->ind));
    }
  else if (ch->dir == KINETIS_DMA_DIRECTION_MEMORY_TO_PERIPHERAL)
    {
      putreg32(per_addr, KINETIS_DMA_TCD_DADDR(ch->ind));
      putreg16(0, KINETIS_DMA_TCD_DOFF(ch->ind));
      putreg32(0, KINETIS_DMA_TCD_DLASTSGA(ch->ind));
    }
  else
    {
      ch->used = false;
      return NULL;
    }

  /* Set data sizes */

  regval16 = (DMA_TCD_ATTR_SSIZE_MASK & ((uint16_t)per_data_sz) <<
             DMA_TCD_ATTR_SSIZE_SHIFT);
  regval16 |= (DMA_TCD_ATTR_DSIZE_MASK & ((uint16_t)per_data_sz) <<
               DMA_TCD_ATTR_DSIZE_SHIFT);
  putreg16(regval16, KINETIS_DMA_TCD_ATTR(ch->ind));

  /* Set minor loop count */

  putreg32(1 << (uint8_t)per_data_sz, KINETIS_DMA_TCD_NBYTES(ch->ind));
  return (DMA_HANDLE)ch;
}

/****************************************************************************
 * Name: kinetis_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until kinetis_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kinetis_dmafree(DMA_HANDLE handle)
{
  struct kinetis_dma_ch *ch = (struct kinetis_dma_ch *)handle;
  irqstate_t flags;

  DEBUGASSERT(handle != NULL);

  /* Disable DMA channel in the dmamux */

  putreg8(0, KINETIS_DMAMUX_CHCFG(ch->ind));

  flags = enter_critical_section();
  ch->used = false;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: kinetis_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

int kinetis_dmasetup(DMA_HANDLE handle, uint32_t mem_addr, size_t ntransfers,
                     uint16_t control)
{
  struct kinetis_dma_ch *ch = (struct kinetis_dma_ch *)handle;
  uint16_t regval = 0;
  uint32_t nbytes;

  if (ntransfers > (DMA_TCD_CITER2_MASK >> DMA_TCD_CITER2_SHIFT))
    {
      return -EINVAL;
    }

  DEBUGASSERT(handle != NULL);

  nbytes = (uint32_t)ntransfers * (uint32_t)(1 << (uint8_t)ch->data_sz);

  if (ch->dir == KINETIS_DMA_DIRECTION_PERIPHERAL_TO_MEMORY)
    {
      putreg32(mem_addr, KINETIS_DMA_TCD_DADDR(ch->ind));
      putreg16(1 << (uint8_t)ch->data_sz, KINETIS_DMA_TCD_DOFF(ch->ind));
      putreg32(-nbytes, KINETIS_DMA_TCD_DLASTSGA(ch->ind));
    }
  else if (ch->dir == KINETIS_DMA_DIRECTION_MEMORY_TO_PERIPHERAL)
    {
      putreg32(mem_addr, KINETIS_DMA_TCD_SADDR(ch->ind));
      putreg16(1 << (uint8_t)ch->data_sz, KINETIS_DMA_TCD_SOFF(ch->ind));
      putreg32(-nbytes, KINETIS_DMA_TCD_SLAST(ch->ind));
    }
  else
    {
      return -EINVAL;
    }

  /* Set up channel with control word */

  regval =  (control & DMA_TCD_CSR_MAJORELINK) ? ch->ind : 0;
  regval <<= DMA_TCD_CSR_MAJORLINKCH_SHIFT;
  regval &= DMA_TCD_CSR_MAJORLINKCH_MASK;
  regval |= (DMA_TCD_CSR_INTMAJOR |
            (control & (DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_MAJORELINK)));
  putreg16(regval, KINETIS_DMA_TCD_CSR(ch->ind));

  /* Set major loop count */

  putreg16(ntransfers, KINETIS_DMA_TCD_BITER(ch->ind));
  putreg16(ntransfers, KINETIS_DMA_TCD_CITER(ch->ind));

  return OK;
}

/****************************************************************************
 * Name: kinetis_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int kinetis_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct kinetis_dma_ch *ch = (struct kinetis_dma_ch *)handle;

  DEBUGASSERT(handle != NULL);

  ch->callback = callback;
  ch->arg = arg;

  /* Enable request register for this channel */

  putreg8(ch->ind, KINETIS_DMA_SERQ);

  return OK;
}

/****************************************************************************
 * Name: kinetis_dmastop
 *
 * Description:
 *   Cancel the DMA.  After kinetis_dmastop() is called, the DMA channel is
 *   reset and kinetis_dmasetup() must be called before kinetis_dmastart()
 *   can be called again
 *
 ****************************************************************************/

void kinetis_dmastop(DMA_HANDLE handle)
{
  struct kinetis_dma_ch *ch = (struct kinetis_dma_ch *)handle;

  DEBUGASSERT(handle != NULL);

  putreg8(ch->ind, KINETIS_DMA_CERQ);
}

/****************************************************************************
 * Name: kinetis_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void kinetis_dmasample(DMA_HANDLE handle, struct kinetis_dmaregs_s *regs)
{
  DEBUGASSERT(handle != NULL);
}
#endif

/****************************************************************************
 * Name: kinetis_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void kinetis_dmadump(DMA_HANDLE handle, const struct kinetis_dmaregs_s *regs,
                const char *msg)
{
  DEBUGASSERT(handle != NULL);
}
#endif
