/****************************************************************************
 * arch/arm/src/efm32/efm32_dma.c
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
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "hardware/efm32_cmu.h"
#include "hardware/efm32_dma.h"
#include "efm32_dma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALIGN_MASK(s)   ((1 << s) - 1)
#define ALIGN_DOWN(v,m) ((v) & ~m)
#define ALIGN_UP(v,m)   (((v) + (m)) & ~m)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one DMA channel */

struct dma_channel_s
{
  uint8_t chan;                  /* DMA channel number (0-EFM32_DMA_NCHANNELS) */
  bool inuse;                    /* TRUE: The DMA channel is in use */
  dma_config_t config;           /* Current configuration */
  dma_callback_t callback;       /* Callback invoked when the DMA completes */
  void *arg;                     /* Argument passed to callback function */
};

/* This structure describes the state of the DMA controller */

struct dma_controller_s
{
  mutex_t lock;                  /* Protects channel table */
  sem_t chansem;                 /* Count of free channels */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the overall state of the DMA controller */

static struct dma_controller_s g_dmac;

/* This is the array of all DMA channels */

static struct dma_channel_s g_dmach[EFM32_DMA_NCHANNELS];

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

#if EFM32_DMA_NCHANNELS <= 8
#  define DESC_TABLE_SIZE  8
#  define DESC_TABLE_ALIGN 256 /* 2*8*16 */
#elif EFM32_DMA_NCHANNELS <= 16
#  define DESC_TABLE_SIZE  16
#  define DESC_TABLE_ALIGN 512 /* 2*16*16 */
#else
#  error Unknown descriptor table size
#endif

#ifdef CONFIG_EFM32_DMA_ALTDSEC
static struct dma_descriptor_s
  g_descriptors[DESC_TABLE_SIZE + EFM32_DMA_NCHANNELS]
  aligned_data(DESC_TABLE_ALIGN);
#else
static struct dma_descriptor_s g_descriptors[EFM32_DMA_NCHANNELS]
  aligned_data(DESC_TABLE_ALIGN);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_set_chctrl
 *
 * Description:
 *  Set the channel control register
 *
 ****************************************************************************/

static void efm32_set_chctrl(struct dma_channel_s *dmach,
                             dma_config_t config)
{
  uintptr_t regaddr;
  uint32_t decoded;
  uint32_t regval;

  decoded  = (uint32_t)(config & EFM32_DMA_SIGSEL_MASK) >>
              EFM32_DMA_SIGSEL_SHIFT;
  regval   = (decoded << _DMA_CH_CTRL_SIGSEL_SHIFT);
  decoded  = (uint32_t)(config & EFM32_DMA_SOURCSEL_MASK) >>
              EFM32_DMA_SOURCSEL_SHIFT;
  regval  |= (decoded << _DMA_CH_CTRL_SOURCESEL_SHIFT);

  regaddr = EFM32_DMA_CHN_CTRL(dmach->chan);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: efm32_align_shift
 *
 * Description:
 *  Set the channel control register
 *
 ****************************************************************************/

static inline unsigned int efm32_align_shift(dma_config_t config)
{
  unsigned int shift;

  shift = (config & EFM32_DMA_XFERSIZE_MASK) >> EFM32_DMA_XFERSIZE_SHIFT;
  DEBUGASSERT(shift != 3);
  return shift;
}

/****************************************************************************
 * Name: efm32_get_descriptor
 *
 * Description:
 *  Get the address of the primary or alternate descriptor assigned to the
 *  DMA channel
 *
 ****************************************************************************/

static inline struct dma_descriptor_s *
efm32_get_descriptor(struct dma_channel_s *dmach, bool alt)
{
  uintptr_t base = alt ? getreg32(EFM32_DMA_ALTCTRLBASE) :
                         getreg32(EFM32_DMA_CTRLBASE);
  return ((struct dma_descriptor_s *)base) + dmach->chan;
}

/****************************************************************************
 * Name: efm32_dmac_interrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int efm32_dmac_interrupt(int irq, void *context, void *arg)
{
  struct dma_channel_s *dmach;
  unsigned int chndx;
  uint32_t pending;
  uint32_t bit;

  /* Get the set of pending, unmasked global XDMAC interrupts */

  pending = getreg32(EFM32_DMA_IF) & getreg32(EFM32_DMA_IEN);
  putreg32(pending, EFM32_DMA_IFC);

  /* Check each bit to see which channel(s) have interrupted */

  for (chndx = 0; chndx < EFM32_DMA_NCHANNELS && pending != 0; chndx++)
    {
      /* Are any interrupts pending for this channel? */

      bit = 1 << chndx;
      if ((pending & bit) != 0)
        {
          dmach = &g_dmach[chndx];

          /* Put the DMA channel in the stopped state */

          efm32_dmastop((DMA_HANDLE)dmach);

          /* Call the DMA completion callback */

          if (dmach->callback)
            {
              dmach->callback((DMA_HANDLE)dmach, OK, dmach->arg);
              dmach->callback = NULL;
            }

          dmach->arg = NULL;

          /* Clear the bit in the sampled set of pending interrupts */

          pending &= !bit;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_dma_initialize
 *
 * Description:
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function arm_dma_initialize(void)
{
  uint32_t regval;
  int i;

  dmainfo("Initialize XDMAC0\n");

  /* Initialize the channel list  */

  nxmutex_init(&g_dmac.lock);
  nxsem_init(&g_dmac.chansem, 0, EFM32_DMA_NCHANNELS);

  for (i = 0; i < EFM32_DMA_NCHANNELS; i++)
    {
      g_dmach[i].chan = i;
    }

  /* Enable clocking to the DMA module.  DMA is clocked by HFCORECLK. */

  regval  = getreg32(EFM32_CMU_HFCORECLKEN0);
  regval |= CMU_HFCORECLKEN0_DMA;
  putreg32(regval, EFM32_CMU_HFCORECLKEN0);

  /* Set the control base addresses.  Note: EFM32_DMA_ALTCTRLBASE
   * is a read-only register that provides the location where hardware
   * will obtain the alternative descriptors.
   */

  putreg32((uint32_t)g_descriptors, EFM32_DMA_CTRLBASE);

  /* Attach DMA interrupt vector */

  irq_attach(EFM32_IRQ_DMA, efm32_dmac_interrupt, NULL);

  /* Enable the DMA controller */

  putreg32(DMA_CONFIG_EN, EFM32_DMA_CONFIG);

  /* Enable the IRQ at the AIC (still disabled at the DMA controller) */

  up_enable_irq(EFM32_IRQ_DMA);
}

/****************************************************************************
 * Name: efm32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to a DMA channel.
 *
 *   If no DMA channel is available, then efm32_dmachannel() will wait
 *   until the holder of a channel relinquishes the channel by calling
 *   efm32_dmafree().
 *
 * Input Parameters:
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

DMA_HANDLE efm32_dmachannel(void)
{
  struct dma_channel_s *dmach;
  unsigned int chndx;
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

  ret = nxmutex_lock(&g_dmac.lock);
  if (ret < 0)
    {
      nxsem_post(&g_dmac.chansem);
      return NULL;
    }

  /* Search for an available DMA channel */

  for (chndx = 0, dmach = NULL; chndx < EFM32_DMA_NCHANNELS; chndx++)
    {
      struct dma_channel_s *candidate = &g_dmach[chndx];
      if (!candidate->inuse)
        {
          dmach         = candidate;
          dmach->inuse  = true;

          /* Clear any pending channel interrupts */

          bit = 1 << chndx;
          putreg32(bit, EFM32_DMA_IFC);

          /* Disable the channel */

          putreg32(bit, EFM32_DMA_CHENC);
          break;
        }
    }

  nxmutex_unlock(&g_dmac.lock);

  /* Since we have reserved a DMA descriptor by taking a count from chansem,
   * it would be a serious logic failure if we could not find a free channel
   * for our use.
   */

  DEBUGASSERT(dmach);
  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: efm32_dmafree
 *
 * Description:
 *   Release a DMA channel. If another thread is waiting for this DMA channel
 *   in a call to efm32_dmachannel, then this function will re-assign the
 *   DMA channel to that thread and wake it up.  NOTE:  The 'handle' used
 *   in this argument must NEVER be used again until efm32_dmachannel() is
 *   called again to re-gain access to the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void efm32_dmafree(DMA_HANDLE handle)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;

  DEBUGASSERT(dmach != NULL && dmach->inuse);
  dmainfo("DMA channel %d\n", dmach->chan);

  /* Disable the channel */

  putreg32(1 << dmach->chan, EFM32_DMA_CHENC);

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
 * Name: efm32_rxdmasetup
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

void efm32_rxdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
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

  shift = efm32_align_shift(config);
  mask  = ALIGN_MASK(shift);

  /* Make sure that the number of bytes we are asked to transfer is a
   * multiple of the transfer size.
   */

  xfersize = (1 << shift);
  nbytes   = ALIGN_DOWN(nbytes, mask);
  DEBUGASSERT(nbytes > 0);

  /* Save the configuration (for efm32_dmastart()). */

  dmach->config = config;

  /* Configure for the selected peripheral */

  efm32_set_chctrl(dmach, config);

  /* Configure the primary channel descriptor */

  desc         = efm32_get_descriptor(dmach, false);
  desc->srcend = (uint32_t *)paddr;
  desc->dstend = (uint32_t *)(maddr + nbytes - xfersize);

  /* No source increment, destination increments according to transfer size.
   * No privileges.  Arbitrate after each transfer. Default priority.
   */

  regval = DMA_CTRL_SRC_INC_NONE | DMA_CTRL_DST_PROT_NON_PRIVILEGED |
           DMA_CTRL_SRC_PROT_NON_PRIVILEGED | DMA_CTRL_R_POWER_1 |
           (0 << _DMA_CTRL_NEXT_USEBURST_SHIFT) | _DMA_CTRL_CYCLE_CTRL_BASIC;

  switch (shift)
    {
    default:
    case 0: /* Byte transfer */
      regval |= DMA_CTRL_DST_SIZE_BYTE | DMA_CTRL_SRC_SIZE_BYTE;
      incr    = DMA_CTRL_DST_INC_BYTE;
      break;

    case 1: /* Half word transfer */
      regval |= DMA_CTRL_DST_SIZE_HALFWORD | DMA_CTRL_SRC_SIZE_HALFWORD;
      incr    = DMA_CTRL_DST_INC_HALFWORD;
      break;

    case 2: /* Word transfer */
      regval |=  DMA_CTRL_DST_SIZE_WORD | DMA_CTRL_SRC_SIZE_WORD;
      incr    = DMA_CTRL_DST_INC_WORD;
      break;
    }

  /* Do we need to increment the memory address? */

  if ((config & EFM32_DMA_MEMINCR_MASK) == EFM32_DMA_MEMINCR)
    {
      regval |= incr;
    }

  /* Set the number of transfers (minus 1) */

  DEBUGASSERT((nbytes >> shift) < 1024);
  regval |= ((nbytes >> shift) - 1) << _DMA_CTRL_N_MINUS_1_SHIFT;
  desc->ctrl = regval;
  desc->user = 0;
}

/****************************************************************************
 * Name: efm32_txdmasetup
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

void efm32_txdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
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

  shift = efm32_align_shift(config);
  mask  = ALIGN_MASK(shift);

  /* Make sure that the number of bytes we are asked to transfer is a
   * multiple of the transfer size.
   */

  xfersize = (1 << shift);
  nbytes   = ALIGN_DOWN(nbytes, mask);
  DEBUGASSERT(nbytes > 0);

  /* Save the configuration (for efm32_dmastart()). */

  dmach->config = config;

  /* Configure for the selected peripheral */

  efm32_set_chctrl(dmach, config);

  /* Configure the primary channel descriptor */

  desc         = efm32_get_descriptor(dmach, false);
  desc->srcend = (uint32_t *)(maddr + nbytes - xfersize);
  desc->dstend = (uint32_t *)paddr;

  /* No destination increment, source increments according to transfer size.
   * No privileges.  Arbitrate after each transfer. Default priority.
   */

  regval = DMA_CTRL_DST_INC_NONE | DMA_CTRL_DST_PROT_NON_PRIVILEGED |
           DMA_CTRL_SRC_PROT_NON_PRIVILEGED | DMA_CTRL_R_POWER_1 |
           (0 << _DMA_CTRL_NEXT_USEBURST_SHIFT) | _DMA_CTRL_CYCLE_CTRL_BASIC;

  switch (shift)
    {
    default:
    case 0: /* Byte transfer */
      regval |= DMA_CTRL_DST_SIZE_BYTE | DMA_CTRL_SRC_SIZE_BYTE;
      incr    = DMA_CTRL_SRC_INC_BYTE;
      break;

    case 1: /* Half word transfer */
      regval |= DMA_CTRL_DST_SIZE_HALFWORD | DMA_CTRL_SRC_SIZE_HALFWORD;
      incr    = DMA_CTRL_SRC_INC_HALFWORD;
      break;

    case 2: /* Word transfer */
      regval |= DMA_CTRL_DST_SIZE_WORD | DMA_CTRL_SRC_SIZE_WORD;
      incr    = DMA_CTRL_SRC_INC_WORD;
      break;
    }

  /* Do we need to increment the memory address? */

  if ((config & EFM32_DMA_MEMINCR_MASK) == EFM32_DMA_MEMINCR)
    {
      regval |= incr;
    }

  /* Set the number of transfers (minus 1) */

  DEBUGASSERT((nbytes >> shift) < 1024);
  regval |= ((nbytes >> shift) - 1) << _DMA_CTRL_N_MINUS_1_SHIFT;
  desc->ctrl = regval;
  desc->user = 0;
}

/****************************************************************************
 * Name: efm32_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void efm32_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;
  irqstate_t flags;
  uint32_t regval;
  uint32_t bit;

  DEBUGASSERT(dmach && dmach->inuse && dmach->desc);

  /* Save the DMA complete callback info */

  dmach->callback = callback;
  dmach->arg       = arg;

  /* Finish configuring the channel */

  bit = 1 << dmach->chan;
  if ((dmach->config & EFM32_DMA_SINGLE_MASK) == EFM32_DMA_BUFFER_FULL)
    {
      /* Disable the single requests for the channel (i.e. do not react to
       * data available, wait for buffer full)
       */

      putreg32(bit, EFM32_DMA_CHUSEBURSTS);

      /* Enable buffer-full requests for the channel */

      putreg32(bit, EFM32_DMA_CHREQMASKC);
    }
  else
    {
      /* Enable the single requests for the channel */

      putreg32(bit, EFM32_DMA_CHUSEBURSTC);

      /* Disable buffer-full requests for the channel */

      putreg32(bit, EFM32_DMA_CHREQMASKS);
    }

  /* Use the primary data structure for the channel */

  putreg32(bit, EFM32_DMA_CHALTC);

  /* Enable DMA completion interrupts */

  flags   = enter_critical_section();
  regval  = getreg32(EFM32_DMA_IEN);
  regval |= bit;
  putreg32(regval, EFM32_DMA_IEN);

  /* Enable the channel */

  putreg32(bit, EFM32_DMA_CHENS);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: efm32_dmastop
 *
 * Description:
 *   Cancel the DMA.  After efm32_dmastop() is called, the DMA channel is
 *   reset and efm32_dmasetup() must be called before efm32_dmastart() can be
 *   called again
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *
 ****************************************************************************/

void efm32_dmastop(DMA_HANDLE handle)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;
  irqstate_t flags;
  uint32_t regval;
  uint32_t bit;

  DEBUGASSERT(dmach);
  bit = 1 << dmach->chan;

  /* Disable the channel */

  flags = enter_critical_section();
  putreg32(bit, EFM32_DMA_CHENC);

  /* Disable Channel interrupts */

  regval  = getreg32(EFM32_DMA_IEN);
  regval |= bit;
  putreg32(regval, EFM32_DMA_IEN);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: efm32_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void efm32_dmasample(DMA_HANDLE handle, struct efm32_dmaregs_s *regs)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;
  uintptr_t regaddr;
  irqstate_t flags;

  /* Sample DMA registers. */

  flags              = enter_critical_section();

  regs->status       = getreg32(EFM32_DMA_STATUS);
  regs->ctrlbase     = getreg32(EFM32_DMA_CTRLBASE);
  regs->altctrlbase  = getreg32(EFM32_DMA_ALTCTRLBASE);
  regs->chwaitstatus = getreg32(EFM32_DMA_CHWAITSTATUS);
  regs->chusebursts  = getreg32(EFM32_DMA_CHUSEBURSTS);
  regs->chreqmasks   = getreg32(EFM32_DMA_CHREQMASKS);
  regs->chens        = getreg32(EFM32_DMA_CHENS);
  regs->chalts       = getreg32(EFM32_DMA_CHALTS);
  regs->chpris       = getreg32(EFM32_DMA_CHPRIS);
  regs->errorc       = getreg32(EFM32_DMA_ERRORC);
  regs->chreqstatus  = getreg32(EFM32_DMA_CHREQSTATUS);
  regs->chsreqstatus = getreg32(EFM32_DMA_CHSREQSTATUS);
  regs->ifr          = getreg32(EFM32_DMA_IF);
  regs->ien          = getreg32(EFM32_DMA_IEN);
#if defined(CONFIG_EFM32_EFM32GG)
  regs->ctrl         = getreg32(EFM32_DMA_CTRL);
  regs->rds          = getreg32(EFM32_DMA_RDS);
  regs->loop0        = getreg32(EFM32_DMA_LOOP0);
  regs->loop1        = getreg32(EFM32_DMA_LOOP1);
  regs->rect0        = getreg32(EFM32_DMA_RECT0);
#endif

  /* Sample channel control register */

  regaddr            = EFM32_DMA_CHN_CTRL(dmach->chan)
  regs->chnctrl      = getreg32(regaddr);

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: efm32_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void efm32_dmadump(DMA_HANDLE handle, const struct efm32_dmaregs_s *regs,
                   const char *msg)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;

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
  dmainfo("   CHREQSTATUS: %08x\n", regs->chreqstatus);
  dmainfo("  CHSREQSTATUS: %08x\n", regs->chsreqstatus);
  dmainfo("           IEN: %08x\n", regs->ien);
#if defined(CONFIG_EFM32_EFM32GG)
  dmainfo("          CTRL: %08x\n", regs->ctrl);
  dmainfo("           RDS: %08x\n", regs->rds);
  dmainfo("         LOOP0: %08x\n", regs->loop0);
  dmainfo("         LOOP1: %08x\n", regs->loop1);
  dmainfo("         RECT0: %08x\n", regs->rect0);
#endif
  dmainfo("  DMA Channel %d Registers:\n", dmach->chan);
  dmainfo("        CHCTRL: %08x\n", regs->chnctrl);
}
#endif
