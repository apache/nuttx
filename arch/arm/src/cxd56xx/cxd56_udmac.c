/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_udmac.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "cxd56_clock.h"
#include "hardware/cxd56_udmac.h"
#include "cxd56_udmac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALIGN_MASK(s)    ((1 << s) - 1)
#define ALIGN_DOWN(v, m) ((v) & ~m)
#define ALIGN_UP(v, m)   (((v) + (m)) & ~m)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one DMA channel */

struct dma_channel_s
{
  uint8_t        chan;          /* DMA channel number (0-CXD56_DMA_NCHANNELS) */
  bool           inuse;         /* TRUE: The DMA channel is in use */
  dma_config_t   config;        /* Current configuration */
  dma_callback_t callback;      /* Callback invoked when the DMA completes */
  void          *arg;           /* Argument passed to callback function */
};

/* This structure describes the state of the DMA controller */

struct dma_controller_s
{
  sem_t exclsem; /* Protects channel table */
  sem_t chansem; /* Count of free channels */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the overall state of the DMA controller */

static struct dma_controller_s g_dmac;

/* This is the array of all DMA channels */

static struct dma_channel_s g_dmach[CXD56_DMA_NCHANNELS];

/* This array describes the available channel control data structures.  Each
 * structure must be aligned to a 256 address boundary.  The last 8 or 9
 * bits of address are provided by the DMA controller:
 *
 * 8-Channels:
 *   Bit 7:    Selects the alternate descriptor list
 *   Bits 4-6: The DMA channel (0-7)
 *   Bits 2-3: Selects the descriptor field
 *   Bits 0-1: Always zero
 *
 * 12-Channels:
 *   Bit 8:    Selects the alternate descriptor list
 *   Bits 4-7: The DMA channel (0-11)
 *   Bits 2-3: Selects the descriptor field
 *   Bits 0-1: Always zero
 */

#if CXD56_DMA_NCHANNELS <= 8
#define DESC_TABLE_SIZE  8
#define DESC_TABLE_ALIGN 256 /* 2*8*16 */
#elif CXD56_DMA_NCHANNELS <= 16
#define DESC_TABLE_SIZE  16
#define DESC_TABLE_ALIGN 512 /* 2*16*16 */
#elif CXD56_DMA_NCHANNELS <= 32
#define DESC_TABLE_SIZE  32
#define DESC_TABLE_ALIGN 1024 /* 2*16*16 */
#else
#  error Unknown descriptor table size
#endif

static struct dma_descriptor_s g_descriptors[CXD56_DMA_NCHANNELS]
  aligned_data(DESC_TABLE_ALIGN);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_align_shift
 *
 * Description:
 *  Set the channel control register
 *
 ****************************************************************************/

static inline unsigned int cxd56_align_shift(dma_config_t config)
{
  unsigned int shift;

  shift = (config.channel_cfg & CXD56_UDMA_XFERSIZE_MASK) >>
           CXD56_UDMA_XFERSIZE_SHIFT;
  DEBUGASSERT(shift != 3);
  return shift;
}

/****************************************************************************
 * Name: cxd56_get_descriptor
 *
 * Description:
 *  Get the address of the primary or alternate descriptor assigned to the
 *  DMA channel
 *
 ****************************************************************************/

static inline struct dma_descriptor_s *cxd56_get_descriptor(
  struct dma_channel_s *dmach, bool alt)
{
  uintptr_t base;

  base = alt ?
           getreg32(CXD56_DMA_ALTCTRLBASE) : getreg32(CXD56_DMA_CTRLBASE);
  return ((struct dma_descriptor_s *)base) + dmach->chan;
}

/****************************************************************************
 * Name: cxd56_dmac_interrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int cxd56_dmac_interrupt(int irq, void *context, void *arg)
{
  struct dma_channel_s *dmach;
  unsigned int chndx;
  int done;
  int err;
  int mask;
  int result = OK;

  chndx = irq - CXD56_IRQ_DMA_A_0;

  mask = 1 << chndx;
  done = getreg32(CXD56_DMA_DONE);
  err  = getreg32(CXD56_DMA_ERR);
  if (err & mask)
    {
      putreg32(mask, CXD56_DMA_ERR);
      result = EIO;
    }

  if (done & mask)
    {
      /* Clear DMA done status */

      putreg32(mask, CXD56_DMA_DONE);
    }

  dmach = &g_dmach[chndx];

  /* Put the DMA channel in the stopped state */

  cxd56_udmastop((DMA_HANDLE)dmach);

  /* Call the DMA completion callback */

  if (dmach->callback)
    {
      dmach->callback((DMA_HANDLE)dmach, result, dmach->arg);
      dmach->callback = NULL;
    }

  dmach->arg = NULL;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_udmainitialize
 *
 * Description:
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cxd56_udmainitialize(void)
{
  int i;

  dmainfo("Initialize uDMAC\n");

  /* Initialize the channel list  */

  nxsem_init(&g_dmac.exclsem, 0, 1);
  nxsem_init(&g_dmac.chansem, 0, CXD56_DMA_NCHANNELS);

  for (i = 0; i < CXD56_DMA_NCHANNELS; i++)
    {
      g_dmach[i].chan = i;
    }

  /* Enable clocking to the DMA module. */

  cxd56_udmac_clock_enable();

  /* Set the control base addresses.  Note: CXD56_DMA_ALTCTRLBASE
   * is a read-only register that provides the location where hardware
   * will obtain the alternative descriptors.
   */

  putreg32(CXD56_PHYSADDR(g_descriptors), CXD56_DMA_CTRLBASE);

  /* Enable the DMA controller */

  putreg32(DMA_CFG_EN, CXD56_DMA_CFG);
}

/****************************************************************************
 * Name: cxd56_udmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to a DMA channel.
 *
 *   If no DMA channel is available, then cxd56_udmachannel() will wait
 *   until the holder of a channel relinquishes the channel by calling
 *   cxd56_dmafree().
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   This function ALWAYS returns a non-NULL, void* DMA channel handle.
 *
 * Assumptions:
 *   - The caller can wait for a DMA channel to be freed if it is not
 *     available.
 *
 ****************************************************************************/

DMA_HANDLE cxd56_udmachannel(void)
{
  struct dma_channel_s *dmach;
  unsigned int ch;
  uint32_t bit;
  int ret;

  /* Take a count from the channel counting semaphore.  We may block
   * if there are no free channels.  When we get the count, then we can
   * be assured that a channel is available in the channel list and is
   * reserved for us.
   */

  ret = nxsem_wait_uninterruptible(&g_dmac.chansem);
  if (ret < 0)
    {
      return NULL;
    }

  /* Get exclusive access to the DMA channel list */

  ret = nxsem_wait_uninterruptible(&g_dmac.exclsem);
  if (ret < 0)
    {
      nxsem_post(&g_dmac.chansem);
      return NULL;
    }

  /* Search for an available DMA channel */

  for (ch = 0, dmach = NULL; ch < CXD56_DMA_NCHANNELS; ch++)
    {
      struct dma_channel_s *candidate = &g_dmach[ch];
      if (!candidate->inuse)
        {
          dmach        = candidate;
          dmach->inuse = true;

          bit = 1 << ch;

          /* Disable the channel */

          putreg32(bit, CXD56_DMA_CHENC);
          break;
        }
    }

  nxsem_post(&g_dmac.exclsem);

  /* Attach DMA interrupt vector */

  irq_attach(CXD56_IRQ_DMA_A_0 + ch, cxd56_dmac_interrupt, NULL);

  /* Enable the IRQ at the AIC (still disabled at the DMA controller) */

  up_enable_irq(CXD56_IRQ_DMA_A_0 + ch);

  /* Since we have reserved a DMA descriptor by taking a count from chansem,
   * it would be a serious logic failure if we could not find a free channel
   * for our use.
   */

  DEBUGASSERT(dmach);
  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: cxd56_udmafree
 *
 * Description:
 *   Release a DMA channel.  If another thread is waiting for this DMA
 *   channel in a call to cxd56_udmachannel, then this function will
 *   re-assign the DMA channel to that thread and wake it up.  NOTE:  The
 *   'handle' used in this argument must NEVER be used again until
 *   cxd56_udmachannel() is called again to re-gain access to the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void cxd56_udmafree(DMA_HANDLE handle)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;

  DEBUGASSERT(dmach != NULL && dmach->inuse);
  dmainfo("DMA channel %d\n", dmach->chan);

  /* Disable the channel */

  putreg32(1 << dmach->chan, CXD56_DMA_CHENC);

  /* Mark the channel no longer in use.  Clearing the in-use flag is an
   * atomic operation and so should be safe.
   */

  dmach->inuse = false;

  /* And increment the count of free channels... possibly waking up a
   * thread that may be waiting for a channel.
   */

  nxsem_post(&g_dmac.chansem);
}

/****************************************************************************
 * Name: cxd56_rxudmasetup
 *
 * Description:
 *   Configure an RX (peripheral-to-memory) DMA before starting the transfer.
 *
 * Input Parameters:
 *   paddr  - Peripheral address (source)
 *   maddr  - Memory address (destination)
 *   nbytes - Number of bytes to transfer.  Must be an even multiple of the
 *            configured transfer size.
 *   config - Channel configuration selections
 *
 ****************************************************************************/

void cxd56_rxudmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
                       size_t nbytes, dma_config_t config)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;
  struct dma_descriptor_s *desc;
  unsigned int xfersize;
  unsigned int shift;
  uint32_t regval;
  uint32_t incr;
  uint32_t mask;

  DEBUGASSERT(dmach != NULL && dmach->inuse);

  /* Get the properly alignment shift and mask */

  shift = cxd56_align_shift(config);
  mask  = ALIGN_MASK(shift);

  /* Make sure that the number of bytes we are asked to transfer is a
   * multiple of the transfer size.
   */

  xfersize = (1 << shift);
  nbytes   = ALIGN_DOWN(nbytes, mask);
  DEBUGASSERT(nbytes > 0);

  /* Save the configuration (for cxd56_udmastart()). */

  dmach->config = config;

  /* Configure the primary channel descriptor */

  desc         = cxd56_get_descriptor(dmach, false);
  desc->srcend = paddr;
  desc->dstend = CXD56_PHYSADDR(maddr + nbytes - xfersize);

  /* No source increment, destination increments according to transfer size.
   * No privileges.  Arbitrate after each transfer. Default priority.
   */

  regval = DMA_CTRL_SRC_INC_NONE | DMA_CTRL_DST_PROT_NON_PRIVILEGED |
           DMA_CTRL_SRC_PROT_NON_PRIVILEGED | DMA_CTRL_R_POWER_2 |
           DMA_CTRL_CYCLE_CTRL_AUTO;

  switch (shift)
    {
      default:
      case 0: /* Byte transfer */
        regval |= DMA_CTRL_DST_SIZE_BYTE | DMA_CTRL_SRC_SIZE_BYTE;
        incr = DMA_CTRL_DST_INC_BYTE;
        break;

      case 1: /* Half word transfer */
        regval |= DMA_CTRL_DST_SIZE_HALFWORD | DMA_CTRL_SRC_SIZE_HALFWORD;
        incr = DMA_CTRL_DST_INC_HALFWORD;
        break;

      case 2: /* Word transfer */
        regval |= DMA_CTRL_DST_SIZE_WORD | DMA_CTRL_SRC_SIZE_WORD;
        incr = DMA_CTRL_DST_INC_WORD;
        break;
    }

  /* Do we need to increment the memory address? */

  if ((config.channel_cfg & CXD56_UDMA_MEMINCR_MASK) == CXD56_UDMA_MEMINCR)
    {
      regval |= incr;
    }
  else
    {
      regval |= DMA_CTRL_DST_INC_NONE;
    }

  /* Set the number of transfers (minus 1) */

  DEBUGASSERT((nbytes >> shift) < 1024);
  regval |= DMA_CTRL_N_MINUS_1((nbytes >> shift) - 1);
  desc->ctrl = regval;
  desc->user = 0;
}

/****************************************************************************
 * Name: cxd56_txudmasetup
 *
 * Description:
 *   Configure an TX (memory-to-memory) DMA before starting the transfer.
 *
 * Input Parameters:
 *   paddr  - Peripheral address (destination)
 *   maddr  - Memory address (source)
 *   nbytes - Number of bytes to transfer.  Must be an even multiple of the
 *            configured transfer size.
 *   config - Channel configuration selections
 *
 ****************************************************************************/

void cxd56_txudmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
                       size_t nbytes, dma_config_t config)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;
  struct dma_descriptor_s *desc;
  unsigned int xfersize;
  unsigned int shift;
  uint32_t regval;
  uint32_t incr;
  uint32_t mask;

  DEBUGASSERT(dmach != NULL && dmach->inuse);

  /* Get the properly alignment shift and mask */

  shift = cxd56_align_shift(config);
  mask  = ALIGN_MASK(shift);

  /* Make sure that the number of bytes we are asked to transfer is a
   * multiple of the transfer size.
   */

  xfersize = (1 << shift);
  nbytes   = ALIGN_DOWN(nbytes, mask);
  DEBUGASSERT(nbytes > 0);

  /* Save the configuration (for cxd56_udmastart()). */

  dmach->config = config;

  /* Configure the primary channel descriptor */

  desc         = cxd56_get_descriptor(dmach, false);
  desc->srcend = CXD56_PHYSADDR(maddr + nbytes - xfersize);
  desc->dstend = paddr;

  /* No destination increment, source increments according to transfer size.
   * No privileges.  Arbitrate after each transfer. Default priority.
   */

  regval = DMA_CTRL_DST_INC_NONE | DMA_CTRL_DST_PROT_NON_PRIVILEGED |
           DMA_CTRL_SRC_PROT_NON_PRIVILEGED | DMA_CTRL_R_POWER_2 |
           DMA_CTRL_CYCLE_CTRL_AUTO;

  switch (shift)
    {
      default:
      case 0: /* Byte transfer */
        regval |= DMA_CTRL_DST_SIZE_BYTE | DMA_CTRL_SRC_SIZE_BYTE;
        incr = DMA_CTRL_SRC_INC_BYTE;
        break;

      case 1: /* Half word transfer */
        regval |= DMA_CTRL_DST_SIZE_HALFWORD | DMA_CTRL_SRC_SIZE_HALFWORD;
        incr = DMA_CTRL_SRC_INC_HALFWORD;
        break;

      case 2: /* Word transfer */
        regval |= DMA_CTRL_DST_SIZE_WORD | DMA_CTRL_SRC_SIZE_WORD;
        incr = DMA_CTRL_SRC_INC_WORD;
        break;
    }

  /* Do we need to increment the memory address? */

  if ((config.channel_cfg & CXD56_UDMA_MEMINCR_MASK) == CXD56_UDMA_MEMINCR)
    {
      regval |= incr;
    }

  /* Set the number of transfers (minus 1) */

  DEBUGASSERT((nbytes >> shift) < 1024);
  regval |= DMA_CTRL_N_MINUS_1((nbytes >> shift) - 1);
  desc->ctrl = regval;
  desc->user = 0;
}

/****************************************************************************
 * Name: cxd56_udmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by cxd56_udmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void cxd56_udmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;
  irqstate_t flags;
  uint32_t bit;

  DEBUGASSERT(dmach && dmach->inuse);

  /* Save the DMA complete callback info */

  dmach->callback = callback;
  dmach->arg      = arg;

  /* Finish configuring the channel */

  bit = 1 << dmach->chan;
  if ((dmach->config.channel_cfg & CXD56_UDMA_SINGLE_MASK) ==
       CXD56_UDMA_BUFFER_FULL)
    {
      /* Disable the single requests for the channel
       * (i.e. do not react to data
       * available, wait for buffer full)
       */

      putreg32(bit, CXD56_DMA_CHUSEBURSTS);

      /* Enable buffer-full requests for the channel */

      putreg32(bit, CXD56_DMA_CHREQMASKC);
    }
  else
    {
      /* Enable the single requests for the channel */

      putreg32(bit, CXD56_DMA_CHUSEBURSTC);

      /* Disable buffer-full requests for the channel */

      putreg32(bit, CXD56_DMA_CHREQMASKS);
    }

  /* Use the primary data structure for the channel */

  putreg32(bit, CXD56_DMA_CHALTC);

  flags = enter_critical_section();

  /* Enable the channel */

  putreg32(bit, CXD56_DMA_CHENS);

  /* Generate a software DMA request */

  putreg32(bit, CXD56_DMA_CHSWREQUEST);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: cxd56_udmastop
 *
 * Description:
 *   Cancel the DMA.  After cxd56_udmastop() is called, the DMA channel is
 *   reset and cxd56_udmasetup() must be called before cxd56_udmastart()
 *   can be called again
 *
 * Assumptions:
 *   - DMA handle allocated by cxd56_udmachannel()
 *
 ****************************************************************************/

void cxd56_udmastop(DMA_HANDLE handle)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;
  irqstate_t flags;
  uint32_t bit;

  DEBUGASSERT(dmach);
  bit = 1 << dmach->chan;

  /* Disable the channel */

  flags = enter_critical_section();
  putreg32(bit, CXD56_DMA_CHENC);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: cxd56_udmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by cxd56_udmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void cxd56_udmasample(DMA_HANDLE handle, struct cxd56_udmaregs_s *regs)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;
  struct dma_descriptor_s *desc;
  irqstate_t flags;

  /* Sample DMA registers. */

  flags = enter_critical_section();

  regs->status       = getreg32(CXD56_DMA_STATUS);
  regs->ctrlbase     = getreg32(CXD56_DMA_CTRLBASE);
  regs->altctrlbase  = getreg32(CXD56_DMA_ALTCTRLBASE);
  regs->chwaitstatus = getreg32(CXD56_DMA_CHWAITSTATUS);
  regs->chusebursts  = getreg32(CXD56_DMA_CHUSEBURSTS);
  regs->chreqmasks   = getreg32(CXD56_DMA_CHREQMASKS);
  regs->chens        = getreg32(CXD56_DMA_CHENS);
  regs->chalts       = getreg32(CXD56_DMA_CHALTS);
  regs->chpris       = getreg32(CXD56_DMA_CHPRIS);
  regs->errorc       = getreg32(CXD56_DMA_ERRORC);
  regs->done         = getreg32(CXD56_DMA_DONE);
  regs->err          = getreg32(CXD56_DMA_ERR);

  desc = cxd56_get_descriptor(dmach, false);

  regs->srcend = (uint32_t)(uintptr_t)desc->srcend;
  regs->dstend = (uint32_t)(uintptr_t)desc->dstend;
  regs->ctrl   = desc->ctrl;

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: cxd56_udmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by cxd56_udmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void cxd56_udmadump(DMA_HANDLE handle, const struct cxd56_udmaregs_s *regs,
                    const char *msg)
{
  dmainfo("%s\n", msg);
  dmainfo("  DMA Registers:\n");
  dmainfo("        STATUS: %08x\n", regs->status);
  dmainfo("      CTRLBASE: %08x\n", regs->ctrlbase);
  dmainfo("   ALTCTRLBASE: %08x\n", regs->altctrlbase);
  dmainfo("  CHWAITSTATUS: %08x\n", regs->chwaitstatus);
  dmainfo("   CHUSEBURSTS: %08x\n", regs->chusebursts);
  dmainfo("    CHREQMASKS: %08x\n", regs->chreqmasks);
  dmainfo("         CHENS: %08x\n", regs->chens);
  dmainfo("        CHALTS: %08x\n", regs->chalts);
  dmainfo("        CHPRIS: %08x\n", regs->chpris);
  dmainfo("        ERRORC: %08x\n", regs->errorc);
  dmainfo("          DONE: %08x\n", regs->done);
  dmainfo("           ERR: %08x\n", regs->err);
  dmainfo("  Descriptors:\n");
  dmainfo("        SRCEND: %08x\n", regs->srcend);
  dmainfo("        DSTEND: %08x\n", regs->dstend);
  dmainfo("          CTRL: %08x\n", regs->ctrl);
}
#endif
