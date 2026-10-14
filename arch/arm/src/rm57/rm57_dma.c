/****************************************************************************
 * arch/arm/src/rm57/rm57_dma.c
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

/* Driver for the RM57L843 DMA controller (TRM SPNU562A chapter 20):
 * 32 channels, 48 hardware request lines, 2 ports.  Unlike TMS570's
 * smaller instance, and unlike stm32h7's per-controller dispatch table
 * (multiple DMA/BDMA/MDMA controllers), this device has exactly one DMA
 * controller, so the implementation here is a single flat channel table.
 *
 * Only the interrupt types actually used by this port's serial DMA are
 * wired up: HBC ("first half of block complete") and BTC ("block
 * transfer complete").  FTC/LFS would fire once per element/frame - once
 * per byte for the byte-granularity transfers used by the serial driver
 * - and are intentionally left disabled.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <strings.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "arm_internal.h"
#include "hardware/rm57_dma.h"
#include "hardware/rm57_sys.h"
#include "rm57_dma.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rm57_dmach_s
{
  uint8_t         chan;      /* Channel number, 0..RM57_DMA_NCHANNELS-1 */
  bool            inuse;     /* True: channel has been allocated */
  uint16_t        nelems;    /* Elements/frame, set by rm57_dmasetup() */
  dma_callback_t  callback;  /* Callback registered by rm57_dmastart() */
  void           *arg;       /* Callback argument */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rm57_dmach_s g_dmach[RM57_DMA_NCHANNELS];
static bool g_dma_initialized;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_dma_pcp_base / rm57_dma_wcp_base
 *
 * Description:
 *   Return the base address of a channel's primary/working control
 *   packet in the DMA RAM.
 *
 ****************************************************************************/

static inline uintptr_t rm57_dma_pcp_base(uint8_t chan)
{
  return RM57_DMARAM_BASE + (uintptr_t)chan * RM57_DMA_PCP_STRIDE;
}

static inline uintptr_t rm57_dma_wcp_base(uint8_t chan)
{
  return RM57_DMARAM_BASE + RM57_DMA_WORKING_CP_OFFSET +
         (uintptr_t)chan * RM57_DMA_WCP_STRIDE;
}

/****************************************************************************
 * Name: rm57_dma_ctcount
 *
 * Description:
 *   Return a channel's current transfer count, preferring the live value
 *   over the copy held in the working control packet.
 *
 *   The working control packet is only refreshed when the channel is
 *   arbitrated out of the priority queue (TRM Table 20-108), so between
 *   writebacks it reads several elements behind what the channel has
 *   really transferred - measured on RM57L843 as two elements behind for a
 *   byte-at-a-time SCI receive, which is enough to strand the last byte of
 *   every received burst in the ring buffer.
 *
 *   In fact the packet may never be written back at all: a channel that is
 *   simply left armed - an SCI receive ring, for one - can sit there having
 *   taken dozens of bytes with its shadow still all zeroes.  Measured on a
 *   DMA console: 57 bytes received, CTCOUNT 0x00000000.  So the shadow on
 *   its own cannot drive a receive ring.
 *
 *   The FIFO A/B active channel registers carry the live count instead.
 *   FIFOASTAT/FIFOBSTAT names the channel each FIFO is processing right now
 *   (TRM 20.3.1.86/87), and a FIFO that is processing nothing keeps
 *   describing whichever channel last ran there - still useful, since a
 *   mostly idle port is rarely "currently processing" when the poll asks.
 *   Such a leftover is claimed by matching its retained source address.
 *
 *   The catch is that with several ports on DMA a channel bounces between
 *   the two FIFOs, so both can hold a matching snapshot and one of them is
 *   the older.  Taking the first match found reports a position that has
 *   moved *backwards*, and a ring-buffer reader takes that for a nearly
 *   full buffer of data it has already consumed - on the console it came
 *   out as whole runs of replayed characters.  Every candidate is therefore
 *   weighed and the furthest along wins; a smaller count is further along,
 *   since CFTCOUNT sits in the high bits of the register.
 *
 ****************************************************************************/

static uint32_t rm57_dma_ctcount(struct rm57_dmach_s *priv)
{
  uint32_t chbit  = (uint32_t)1 << priv->chan;
  uint32_t isaddr = getreg32(rm57_dma_pcp_base(priv->chan) +
                             RM57_DMA_PCP_ISADDR_OFFSET);
  uint32_t best   = 0;
  int i;

  for (i = 0; i < 2; i++)
    {
      uint32_t statoff  = i == 0 ? RM57_DMA_FIFOASTATREG_OFFSET :
                                   RM57_DMA_FIFOBSTATREG_OFFSET;
      uint32_t saddroff = i == 0 ? RM57_DMA_FAACSADDR_OFFSET :
                                   RM57_DMA_FBACSADDR_OFFSET;
      uint32_t tcoff    = i == 0 ? RM57_DMA_FAACTC_OFFSET :
                                   RM57_DMA_FBACTC_OFFSET;
      uint32_t stat     = getreg32(RM57_DMA_BASE + statoff);
      uint32_t tc;

      /* Processing this channel right now: live and authoritative. */

      if ((stat & chbit) != 0)
        {
          return getreg32(RM57_DMA_BASE + tcoff);
        }

      if (stat != 0 || getreg32(RM57_DMA_BASE + saddroff) != isaddr)
        {
          continue;
        }

      tc = getreg32(RM57_DMA_BASE + tcoff);
      if (tc != 0 && (best == 0 || tc < best))
        {
          best = tc;
        }
    }

  /* A zero count means "nothing recorded here yet" rather than "finished",
   * for the shadow and for a FIFO that has not run this channel.
   */

  if (best == 0)
    {
      best = getreg32(rm57_dma_wcp_base(priv->chan) +
                      RM57_DMA_WCP_CTCOUNT_OFFSET);
    }

  return best;
}

/****************************************************************************
 * Name: rm57_dma_dispatch
 *
 * Description:
 *   Common HBCA/BTCA interrupt bottom half: read-and-clear the pending
 *   channel bits from the given flag register, then invoke the callback
 *   for each in-use channel that asked for this interrupt.
 *
 ****************************************************************************/

static void rm57_dma_dispatch(uint32_t flagoffset, uint32_t enaoffset,
                              uint8_t status)
{
  uint32_t pending;
  uint32_t bit;
  int chan;

  pending = getreg32(RM57_DMA_BASE + flagoffset);
  if (pending == 0)
    {
      return;
    }

  /* FTCFLAG/LFSFLAG/HBCFLAG/BTCFLAG are write-1-to-clear (TRM 20.3.1.38-
   * 41); clear exactly the bits we observed as pending before running any
   * callback, so a callback that immediately re-arms the channel is not
   * lost.
   */

  putreg32(pending, RM57_DMA_BASE + flagoffset);

  /* The hardware raises a channel's flag whenever that channel reaches the
   * half-block or block boundary, whether or not the channel enabled the
   * matching interrupt - xxINTENAS only gates who gets to raise the ARM
   * interrupt, not who sets the flag.  Dispatching on the raw flags
   * therefore delivered an HBC callback to channels that had only asked
   * for BTC: a serial TX channel would get a "transfer done" callback at
   * its halfway point, shut the SCI's TX DMA request off there and stall
   * the second half of every transfer.  Only call back the channels that
   * actually subscribed to this interrupt.
   */

  pending &= getreg32(RM57_DMA_BASE + enaoffset);

  while (pending != 0)
    {
      chan     = ffs((int)pending) - 1;
      bit      = (uint32_t)1 << chan;
      pending &= ~bit;

      if (g_dmach[chan].inuse && g_dmach[chan].callback != NULL)
        {
          g_dmach[chan].callback(&g_dmach[chan], status,
                                  g_dmach[chan].arg);
        }
    }
}

/****************************************************************************
 * Name: rm57_dma_hbca_interrupt / rm57_dma_btca_interrupt
 ****************************************************************************/

static int rm57_dma_hbca_interrupt(int irq, void *context, void *arg)
{
  rm57_dma_dispatch(RM57_DMA_HBCFLAG_OFFSET, RM57_DMA_HBCINTENAS_OFFSET,
                    RM57_DMA_STATUS_HBC);
  return OK;
}

static int rm57_dma_btca_interrupt(int irq, void *context, void *arg)
{
  rm57_dma_dispatch(RM57_DMA_BTCFLAG_OFFSET, RM57_DMA_BTCINTENAS_OFFSET,
                    RM57_DMA_STATUS_BTC);
  return OK;
}

/****************************************************************************
 * Name: rm57_dma_initialize
 *
 * Description:
 *   One-time DMA controller bring-up: hardware-initialize (ECC-clear) the
 *   control packet RAM, enable the DMA controller, and attach the HBCA/
 *   BTCA interrupt handlers used to dispatch per-channel callbacks.
 *   Lazily invoked from rm57_dmachannel() rather than from arm_boot(), so
 *   that irq_attach()/up_enable_irq() always run after up_irqinitialize()
 *   has completed (up_irqinitialize() unconditionally masks all VIM
 *   channels, which would otherwise clobber an enable done earlier).
 *
 ****************************************************************************/

static void rm57_dma_initialize(void)
{
  int i;

  memset(g_dmach, 0, sizeof(g_dmach));
  for (i = 0; i < RM57_DMA_NCHANNELS; i++)
    {
      g_dmach[i].chan = (uint8_t)i;
    }

  /* Hardware-initialize (ECC-clear) the DMA control packet RAM before any
   * channel is used, using the same MINITGCR/MSINENA protocol as
   * rm57_memory_initialize() in rm57_boot.c (TRM section 2.2.4.2,
   * Table 2-7: DMA RAM is selected by MSINENA bit 1).
   */

  putreg32(SYS_MINITGCR_ENABLE, RM57_SYS_MINITGCR);
  putreg32(1 << 1, RM57_SYS_MSINENA);

  while ((getreg32(RM57_SYS_MSTCGSTAT) & SYS_MSTCGSTAT_MINIDONE) == 0)
    {
    }

  putreg32(SYS_MINITGCR_DISABLE, RM57_SYS_MINITGCR);

  /* FTCMAP/LFSMAP/HBCMAP/BTCMAP reset to 0 for every channel, i.e. every
   * channel's interrupts already route to Group A (FTCA/LFSA/HBCA/BTCA),
   * the only group wired to the ARM CPU on this device (TRM 20.2.9) - no
   * explicit mapping write is needed.
   */

  putreg32(DMA_GCTRL_DMA_EN, RM57_DMA_BASE + RM57_DMA_GCTRL_OFFSET);

  irq_attach(RM57_REQ_DMAHBCA, rm57_dma_hbca_interrupt, NULL);
  irq_attach(RM57_REQ_DMABTCA, rm57_dma_btca_interrupt, NULL);
  up_enable_irq(RM57_REQ_DMAHBCA);
  up_enable_irq(RM57_REQ_DMABTCA);

  g_dma_initialized = true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_dmachannel
 ****************************************************************************/

DMA_HANDLE rm57_dmachannel(unsigned int req)
{
  irqstate_t flags;
  int chan = -1;
  int i;

  DEBUGASSERT(req < RM57_DMA_NREQUESTS);

  flags = enter_critical_section();

  if (!g_dma_initialized)
    {
      rm57_dma_initialize();
    }

  for (i = 0; i < RM57_DMA_NCHANNELS; i++)
    {
      if (!g_dmach[i].inuse)
        {
          g_dmach[i].inuse    = true;
          g_dmach[i].callback = NULL;
          g_dmach[i].arg      = NULL;
          g_dmach[i].nelems   = 0;
          chan = i;
          break;
        }
    }

  leave_critical_section(flags);

  if (chan < 0)
    {
      return NULL;
    }

  /* Assign the requested hardware DMA request line to this channel via
   * its DREQASI field (TRM 20.3.1.13/14, Figure 20-31/20-32).
   */

  uintptr_t dreqasi = RM57_DMA_BASE + RM57_DMA_DREQASI_OFFSET(chan >> 2);
  int shift = RM57_DMA_DREQASI_CHSHIFT(chan);
  uint32_t regval = getreg32(dreqasi);

  regval &= ~((uint32_t)DMA_DREQASI_FIELD_MASK << shift);
  regval |= ((uint32_t)req & DMA_DREQASI_FIELD_MASK) << shift;
  putreg32(regval, dreqasi);

  return &g_dmach[chan];
}

/****************************************************************************
 * Name: rm57_dmafree
 ****************************************************************************/

void rm57_dmafree(DMA_HANDLE handle)
{
  struct rm57_dmach_s *priv = (struct rm57_dmach_s *)handle;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL && priv->inuse);

  flags = enter_critical_section();
  priv->inuse    = false;
  priv->callback = NULL;
  priv->arg      = NULL;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rm57_dmasetup
 ****************************************************************************/

void rm57_dmasetup(DMA_HANDLE handle, const struct rm57_dmacfg_s *cfg)
{
  struct rm57_dmach_s *priv = (struct rm57_dmach_s *)handle;
  uintptr_t pcp;
  uint32_t itcount;

  DEBUGASSERT(priv != NULL && priv->inuse && cfg != NULL);

  pcp = rm57_dma_pcp_base(priv->chan);

  putreg32(cfg->saddr, pcp + RM57_DMA_PCP_ISADDR_OFFSET);
  putreg32(cfg->daddr, pcp + RM57_DMA_PCP_IDADDR_OFFSET);

  itcount = DMA_TCOUNT_FTCOUNT(cfg->nframes) |
            DMA_TCOUNT_ETCOUNT(cfg->nelems);
  putreg32(itcount, pcp + RM57_DMA_PCP_ITCOUNT_OFFSET);

  putreg32(cfg->chctrl, pcp + RM57_DMA_PCP_CHCTRL_OFFSET);
  putreg32(0, pcp + RM57_DMA_PCP_EIOFF_OFFSET);
  putreg32(0, pcp + RM57_DMA_PCP_FIOFF_OFFSET);

  /* Port assignment: which port(s) this channel uses for its read/write
   * accesses (TRM 20.3.1.21-24, Figure 20-39..20-42).
   */

  uintptr_t par = RM57_DMA_BASE + RM57_DMA_PAR_OFFSET(priv->chan >> 3);
  int parshift = RM57_DMA_PAR_CHSHIFT(priv->chan);
  uint32_t parval = getreg32(par);

  parval &= ~((uint32_t)DMA_PAR_FIELD_MASK << parshift);
  parval |= ((uint32_t)cfg->parassign & DMA_PAR_FIELD_MASK) << parshift;
  putreg32(parval, par);

  priv->nelems = cfg->nelems;
}

/****************************************************************************
 * Name: rm57_dmastart
 ****************************************************************************/

void rm57_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                    bool half)
{
  struct rm57_dmach_s *priv = (struct rm57_dmach_s *)handle;
  uint32_t bit;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL && priv->inuse);

  bit = (uint32_t)1 << priv->chan;

  flags = enter_critical_section();

  priv->callback = callback;
  priv->arg      = arg;

  /* Clear any stale pending flags for this channel before arming it. */

  putreg32(bit, RM57_DMA_BASE + RM57_DMA_HBCFLAG_OFFSET);
  putreg32(bit, RM57_DMA_BASE + RM57_DMA_BTCFLAG_OFFSET);

  putreg32(bit, RM57_DMA_BASE + RM57_DMA_BTCINTENAS_OFFSET);
  if (half)
    {
      putreg32(bit, RM57_DMA_BASE + RM57_DMA_HBCINTENAS_OFFSET);
    }

  putreg32(bit, RM57_DMA_BASE + RM57_DMA_GCHIENAS_OFFSET);
  putreg32(bit, RM57_DMA_BASE + RM57_DMA_HWCHENAS_OFFSET);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rm57_dmastop
 ****************************************************************************/

void rm57_dmastop(DMA_HANDLE handle)
{
  struct rm57_dmach_s *priv = (struct rm57_dmach_s *)handle;
  uint32_t bit;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL && priv->inuse);

  bit = (uint32_t)1 << priv->chan;

  flags = enter_critical_section();

  putreg32(bit, RM57_DMA_BASE + RM57_DMA_HWCHENAR_OFFSET);
  putreg32(bit, RM57_DMA_BASE + RM57_DMA_GCHIENAR_OFFSET);
  putreg32(bit, RM57_DMA_BASE + RM57_DMA_HBCINTENAR_OFFSET);
  putreg32(bit, RM57_DMA_BASE + RM57_DMA_BTCINTENAR_OFFSET);
  putreg32(bit, RM57_DMA_BASE + RM57_DMA_HBCFLAG_OFFSET);
  putreg32(bit, RM57_DMA_BASE + RM57_DMA_BTCFLAG_OFFSET);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rm57_dmaresidual
 ****************************************************************************/

size_t rm57_dmaresidual(DMA_HANDLE handle)
{
  struct rm57_dmach_s *priv = (struct rm57_dmach_s *)handle;
  uint32_t ctcount;
  uint32_t cftcount;
  uint32_t cetcount;

  DEBUGASSERT(priv != NULL && priv->inuse);

  ctcount = rm57_dma_ctcount(priv);

  cftcount = (ctcount & DMA_TCOUNT_FTCOUNT_MASK) >> DMA_TCOUNT_FTCOUNT_SHIFT;
  cetcount = (ctcount & DMA_TCOUNT_ETCOUNT_MASK) >> DMA_TCOUNT_ETCOUNT_SHIFT;

  /* CFTCOUNT counts the frames still to be transferred *including* the one
   * in progress, and CETCOUNT counts the elements still to go within that
   * in-progress frame - so the in-progress frame must be counted once, not
   * twice.  Counting it twice made an idle channel (CFTCOUNT = FTCOUNT,
   * CETCOUNT = ETCOUNT) report one element more than it had ever been
   * programmed to transfer.
   */

  if (cftcount == 0)
    {
      return (size_t)cetcount;
    }

  return (size_t)(cftcount - 1) * priv->nelems + cetcount;
}
