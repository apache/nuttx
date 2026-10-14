/****************************************************************************
 * arch/arm/src/rm57/rm57_dcan.c
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

/* Framework-agnostic core driver for the four Bosch D_CAN controllers of
 * the RM57L843 (TRM SPNU562A chapter 26).  The character-device and
 * SocketCAN frontends bind to this core through struct rm57can_cbs_s.
 *
 * Mailbox partitioning (64 message objects, 1-based):
 *  - Objects 1 .. 64-TXFIFO_DEPTH: receive filters.  Each acceptance
 *    filter is a hardware FIFO of RXFIFO_DEPTH chained objects (EoB set
 *    only on the last).  A catch-all filter is installed while no user
 *    filter is active.  Low numbers = high acceptance priority, and the
 *    hardware stores a frame in the first matching object, so RX always
 *    wins over the TX objects at the top.
 *  - Objects 64-TXFIFO_DEPTH+1 .. 64: transmit FIFO.  The hardware
 *    transmits the lowest-numbered pending object first, and the driver
 *    fills them in ascending order, so completion order equals
 *    submission order.  The window is reused only after it fully drains
 *    to keep priority and order intact.
 *
 * Interface register discipline: IF1 is used by thread context only
 * (under iflock), IF2 by interrupt handlers only, IF3 by the optional
 * auto-update RX paths.  Message-object interrupt flags can only be
 * cleared through an IFx ClrIntPnd transfer (as on Tiva - the INT/ES
 * registers do not clear them).
 *
 * Interrupt routing: all RX message objects are muxed to interrupt
 * line 1 (VIM channel RM57_REQ_CANnLOW) via INTMUX, so INT.Int1ID
 * always yields the lowest pending RX object directly.  TX objects and
 * the status/error interrupt (hardwired to line 0) use line 0
 * (RM57_REQ_CANnHIGH).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/rm57_dcan.h"
#include "hardware/rm57_sys.h"
#include "rm57_dcan.h"

#ifdef RM57_DCAN_HAVE_IF3DMA
#include <nuttx/cache.h>
#include <nuttx/clock.h>
#include "hardware/rm57_dma.h"
#include "rm57_dma.h"
#endif

#ifdef CONFIG_RM57_DCAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CAN_CLK is VCLKA1, mapped to VCLK by rm57_clockconfig.c */

#define RM57_DCAN_CLOCK       BOARD_VCLK_FREQUENCY

/* Target sample point: 75.0% of the bit time, expressed in tenths of a
 * percent.  This reproduces TI's HALCoGen defaults (e.g. 500 kbit/s at
 * a 75 MHz CAN_CLK -> BRP=10, TSEG1=10, TSEG2=4).
 */

#define RM57_DCAN_SAMPLEPOINT 750

/* The configured SECDED mode, as a CTL.PMD field value.  Every write of
 * the whole CTL register has to carry this: the field is 4 bits wide and
 * "disabled" is the single value 0x5, so leaving it out of a write does
 * not mean "unchanged" or even "off" - it writes 0x0, which the hardware
 * reads as yet another "enabled" encoding.
 */

#ifdef CONFIG_RM57_DCAN_SECDED
#  define RM57_DCAN_CTL_PMD   DCAN_CTL_PMD_ENABLE
#else
#  define RM57_DCAN_CTL_PMD   DCAN_CTL_PMD_DISABLE
#endif

/* Defensive bound for the ISR drain loops */

#define RM57_DCAN_ISR_BOUND   (RM57_DCAN_NMSGOBJ * 2)

/* IF3 DMA stuck-request watchdog period */

#define RM57_DCAN_IF3WD_TICKS MSEC2TICK(100)

#if CONFIG_RM57_DCAN_RXFIFO_DEPTH > RM57_DCAN_NRXMBOX
#  error "RM57_DCAN_RXFIFO_DEPTH exceeds the available RX mailboxes"
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void rm57can_meminit(struct rm57can_s *priv);
static void rm57can_if_wait(struct rm57can_s *priv, uint32_t ifbase);
static void rm57can_if_exec(struct rm57can_s *priv, uint32_t ifbase,
                            uint32_t cmd, int mbox);
static void rm57can_disable_mbox(struct rm57can_s *priv, uint32_t ifbase,
                                 int mbox);
static void rm57can_initfilter(struct rm57can_s *priv, uint64_t bitmap,
                               bool ext, uint32_t id, uint32_t mask);
static int  rm57can_alloc_fifo(struct rm57can_s *priv, uint64_t *bitmap);
static void rm57can_free_fifo(struct rm57can_s *priv, uint64_t bitmap);
static void rm57can_install_catchall(struct rm57can_s *priv);
static void rm57can_remove_catchall(struct rm57can_s *priv);
static void rm57can_rxframe_if2(struct rm57can_s *priv, int mbox);
static int  rm57can_isr0(int irq, void *context, void *arg);
static int  rm57can_isr1(int irq, void *context, void *arg);
static int  rm57can_isrif3(int irq, void *context, void *arg);
#ifdef RM57_DCAN_HAVE_IF3DMA
static void rm57can_if3_read(struct rm57can_s *priv);
static void rm57can_if3dma_callback(DMA_HANDLE handle, uint8_t status,
                                    void *arg);
static void rm57can_if3wd_expiry(wdparm_t arg);
static int  rm57can_if3dma_setup(struct rm57can_s *priv);
static void rm57can_if3dma_teardown(struct rm57can_s *priv);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Serializes SYS memory hardware initialization across instances */

static mutex_t g_meminit_lock = NXMUTEX_INITIALIZER;

/* IF3 DMA landing buffers: one D-cache line (32 bytes) each so cache
 * invalidation cannot touch neighbouring data.
 */

#ifdef CONFIG_RM57_DCAN1_RX_IF3DMA
static uint32_t g_dcan1_if3buf[8] aligned_data(32);
#endif
#ifdef CONFIG_RM57_DCAN2_RX_IF3DMA
static uint32_t g_dcan2_if3buf[8] aligned_data(32);
#endif
#ifdef CONFIG_RM57_DCAN3_RX_IF3DMA
static uint32_t g_dcan3_if3buf[8] aligned_data(32);
#endif
#ifdef CONFIG_RM57_DCAN4_RX_IF3DMA
static uint32_t g_dcan4_if3buf[8] aligned_data(32);
#endif

#ifdef CONFIG_RM57_DCAN1
static struct rm57can_s g_dcan1 =
{
  .base         = RM57_DCAN1_BASE,
  .port         = 1,
  .irq0         = RM57_REQ_CAN1HIGH,
  .irq1         = RM57_REQ_CAN1LOW,
  .irqif3       = RM57_REQ_CAN1IF3,
  .msinena      = 5,
#if defined(CONFIG_RM57_DCAN1_RX_IF3DMA)
  .rxpath       = RM57_DCAN_RXPATH_IF3DMA,
  .dmareq       = DMA_REQ_DCAN1_IF3,
  .if3buf       = g_dcan1_if3buf,
#elif defined(CONFIG_RM57_DCAN1_RX_IF3IRQ)
  .rxpath       = RM57_DCAN_RXPATH_IF3IRQ,
#else
  .rxpath       = RM57_DCAN_RXPATH_PIO,
#endif
  .initial_baud = CONFIG_RM57_DCAN1_BAUD,
  .iflock       = NXMUTEX_INITIALIZER,
};
#endif

#ifdef CONFIG_RM57_DCAN2
static struct rm57can_s g_dcan2 =
{
  .base         = RM57_DCAN2_BASE,
  .port         = 2,
  .irq0         = RM57_REQ_CAN2HIGH,
  .irq1         = RM57_REQ_CAN2LOW,
  .irqif3       = RM57_REQ_CAN2IF3,
  .msinena      = 6,
#if defined(CONFIG_RM57_DCAN2_RX_IF3DMA)
  .rxpath       = RM57_DCAN_RXPATH_IF3DMA,
  .dmareq       = DMA_REQ_DCAN2_IF3,
  .if3buf       = g_dcan2_if3buf,
#elif defined(CONFIG_RM57_DCAN2_RX_IF3IRQ)
  .rxpath       = RM57_DCAN_RXPATH_IF3IRQ,
#else
  .rxpath       = RM57_DCAN_RXPATH_PIO,
#endif
  .initial_baud = CONFIG_RM57_DCAN2_BAUD,
  .iflock       = NXMUTEX_INITIALIZER,
};
#endif

#ifdef CONFIG_RM57_DCAN3
static struct rm57can_s g_dcan3 =
{
  .base         = RM57_DCAN3_BASE,
  .port         = 3,
  .irq0         = RM57_REQ_CAN3HIGH,
  .irq1         = RM57_REQ_CAN3LOW,
  .irqif3       = RM57_REQ_CAN3IF3,
  .msinena      = 10,
#if defined(CONFIG_RM57_DCAN3_RX_IF3DMA)
  .rxpath       = RM57_DCAN_RXPATH_IF3DMA,
  .dmareq       = DMA_REQ_DCAN3_IF3,
  .if3buf       = g_dcan3_if3buf,
#elif defined(CONFIG_RM57_DCAN3_RX_IF3IRQ)
  .rxpath       = RM57_DCAN_RXPATH_IF3IRQ,
#else
  .rxpath       = RM57_DCAN_RXPATH_PIO,
#endif
  .initial_baud = CONFIG_RM57_DCAN3_BAUD,
  .iflock       = NXMUTEX_INITIALIZER,
};
#endif

#ifdef CONFIG_RM57_DCAN4
static struct rm57can_s g_dcan4 =
{
  .base         = RM57_DCAN4_BASE,
  .port         = 4,
  .irq0         = RM57_REQ_CAN4HIGH,
  .irq1         = RM57_REQ_CAN4LOW,
  .irqif3       = RM57_REQ_CAN4IF3,
  .msinena      = 20,
#if defined(CONFIG_RM57_DCAN4_RX_IF3DMA)
  .rxpath       = RM57_DCAN_RXPATH_IF3DMA,
  .dmareq       = DMA_REQ_DCAN4_IF3,
  .if3buf       = g_dcan4_if3buf,
#elif defined(CONFIG_RM57_DCAN4_RX_IF3IRQ)
  .rxpath       = RM57_DCAN_RXPATH_IF3IRQ,
#else
  .rxpath       = RM57_DCAN_RXPATH_PIO,
#endif
  .initial_baud = CONFIG_RM57_DCAN4_BAUD,
  .iflock       = NXMUTEX_INITIALIZER,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57can_meminit
 *
 * Description:
 *   Hardware-initialize (ECC-clear) this instance's message RAM through
 *   the SYS memory init controller, as required before first use.  Same
 *   sequence as rm57_memory_initialize() in rm57_boot.c but for the
 *   DCAN RAM select bit.
 *
 ****************************************************************************/

static void rm57can_meminit(struct rm57can_s *priv)
{
  nxmutex_lock(&g_meminit_lock);

  putreg32(SYS_MINITGCR_ENABLE, RM57_SYS_MINITGCR);
  putreg32(1u << priv->msinena, RM57_SYS_MSINENA);

  while ((getreg32(RM57_SYS_MSTCGSTAT) & SYS_MSTCGSTAT_MINIDONE) == 0)
    {
    }

  putreg32(SYS_MINITGCR_DISABLE, RM57_SYS_MINITGCR);

  nxmutex_unlock(&g_meminit_lock);
}

/****************************************************************************
 * Name: rm57can_if_wait / rm57can_if_exec
 *
 * Description:
 *   IF transfer primitives.  A transfer between an IF register set and
 *   the message RAM takes 4..14 VBUS clocks; the IF set is
 *   write-protected while BUSY.  rm57can_if_exec() starts a transfer by
 *   writing the command and message number in one 32-bit access, then
 *   waits for completion.
 *
 ****************************************************************************/

static void rm57can_if_wait(struct rm57can_s *priv, uint32_t ifbase)
{
  while ((getreg32(priv->base + RM57_DCAN_IFCMD_OFFSET(ifbase)) &
          DCAN_IFCMD_BUSY) != 0)
    {
    }
}

static void rm57can_if_exec(struct rm57can_s *priv, uint32_t ifbase,
                            uint32_t cmd, int mbox)
{
  putreg32(cmd | DCAN_IFCMD_MSGNUM(mbox),
           priv->base + RM57_DCAN_IFCMD_OFFSET(ifbase));
  rm57can_if_wait(priv, ifbase);
}

/****************************************************************************
 * Name: rm57can_disable_mbox
 *
 * Description:
 *   Clear MsgVal of a message object and confirm through the MSGVALx
 *   mirror that the write took effect.  A write can be silently lost
 *   when it collides with a message-handler read-modify-write for an
 *   in-flight frame (same hazard as on Tiva's C_CAN, see TI E2E thread
 *   916169), hence the read-back retry loop.
 *
 ****************************************************************************/

static void rm57can_disable_mbox(struct rm57can_s *priv, uint32_t ifbase,
                                 int mbox)
{
  uint32_t regval;

  do
    {
      rm57can_if_wait(priv, ifbase);
      putreg32(0, priv->base + RM57_DCAN_IFARB_OFFSET(ifbase));
      rm57can_if_exec(priv, ifbase, DCAN_IFCMD_WR | DCAN_IFCMD_ARB, mbox);

      regval = getreg32(priv->base +
                        RM57_DCAN_MSGVAL_OFFSET(RM57_DCAN_MBOX_GROUP(mbox)));
    }
  while ((regval & RM57_DCAN_MBOX_BIT(mbox)) != 0);
}

/****************************************************************************
 * Name: rm57can_initfilter
 *
 * Description:
 *   Program the message objects in 'bitmap' as one receive FIFO for the
 *   given identifier/mask.  Objects are written in descending order so
 *   that the FIFO's end marker (EoB on the highest object) is in place
 *   before the lower objects become valid - the FIFO only goes live
 *   with the final (lowest, first-matched) object.
 *
 *   Caller must hold iflock (IF1) or be in setup context.
 *
 ****************************************************************************/

static void rm57can_initfilter(struct rm57can_s *priv, uint64_t bitmap,
                               bool ext, uint32_t id, uint32_t mask)
{
  uint32_t base = priv->base;
  uint32_t mskval;
  uint32_t arbval;
  uint32_t mctl;
  bool eob = true;
  int mbox;

  if (ext)
    {
      mskval = (mask & DCAN_IFMSK_MSK_MASK) | DCAN_IFMSK_MXTD;
      arbval = (id & DCAN_IFARB_ID_MASK) | DCAN_IFARB_XTD |
               DCAN_IFARB_MSGVAL;
    }
  else
    {
      mskval = ((mask & 0x7ff) << DCAN_IFMSK_MSK_STD_SHIFT) |
               DCAN_IFMSK_MXTD;
      arbval = ((id & 0x7ff) << DCAN_IFARB_ID_STD_SHIFT) |
               DCAN_IFARB_MSGVAL;
    }

  /* Dir = 0 (receive data frames).  MDir is deliberately clear.
   *
   * Note: received remote frames are only stored in objects configured
   * with Dir = 1, so this RX path (like Tiva's) does not deliver
   * incoming RTR frames to the application.
   */

  mctl = DCAN_IFMCTL_UMASK | DCAN_IFMCTL_DLC_MASK;

  if (priv->rxpath == RM57_DCAN_RXPATH_PIO)
    {
      mctl |= DCAN_IFMCTL_RXIE;
    }

  for (mbox = RM57_DCAN_NRXMBOX; mbox >= 1; mbox--)
    {
      if ((bitmap & (1ull << (mbox - 1))) == 0)
        {
          continue;
        }

      rm57can_disable_mbox(priv, RM57_DCAN_IF1, mbox);

      putreg32(mskval, base + RM57_DCAN_IFMSK_OFFSET(RM57_DCAN_IF1));
      putreg32(arbval, base + RM57_DCAN_IFARB_OFFSET(RM57_DCAN_IF1));
      putreg32(mctl | (eob ? DCAN_IFMCTL_EOB : 0),
               base + RM57_DCAN_IFMCTL_OFFSET(RM57_DCAN_IF1));
      rm57can_if_exec(priv, RM57_DCAN_IF1,
                      DCAN_IFCMD_WR | DCAN_IFCMD_MASK | DCAN_IFCMD_ARB |
                      DCAN_IFCMD_CONTROL, mbox);

      eob = false;
    }

  /* Enable IF3 auto-update for these objects on the IF3 RX paths */

  if (priv->rxpath != RM57_DCAN_RXPATH_PIO)
    {
      int g;

      for (g = 0; g < 2; g++)
        {
          uint32_t bits = (uint32_t)(bitmap >> (32 * g));

          if (bits != 0)
            {
              modifyreg32(base + RM57_DCAN_IF3UPD_OFFSET(g), 0, bits);
            }
        }
    }
}

/****************************************************************************
 * Name: rm57can_alloc_fifo / rm57can_free_fifo
 *
 * Description:
 *   Allocate/release a contiguous run of RXFIFO_DEPTH mailboxes from
 *   the RX region.  Contiguity keeps FIFO order equal to acceptance
 *   priority order.
 *
 ****************************************************************************/

static int rm57can_alloc_fifo(struct rm57can_s *priv, uint64_t *bitmap)
{
  uint64_t run = (CONFIG_RM57_DCAN_RXFIFO_DEPTH >= 64) ? ~0ull :
                 ((1ull << CONFIG_RM57_DCAN_RXFIFO_DEPTH) - 1);
  int first;

  for (first = 0;
       first + CONFIG_RM57_DCAN_RXFIFO_DEPTH <= RM57_DCAN_NRXMBOX;
       first++)
    {
      if ((priv->mbox_used & (run << first)) == 0)
        {
          priv->mbox_used |= run << first;
          *bitmap = run << first;
          return OK;
        }
    }

  return -ENOSPC;
}

static void rm57can_free_fifo(struct rm57can_s *priv, uint64_t bitmap)
{
  int mbox;

  for (mbox = 1; mbox <= RM57_DCAN_NRXMBOX; mbox++)
    {
      if ((bitmap & (1ull << (mbox - 1))) != 0)
        {
          rm57can_disable_mbox(priv, RM57_DCAN_IF1, mbox);
        }
    }

  if (priv->rxpath != RM57_DCAN_RXPATH_PIO)
    {
      int g;

      for (g = 0; g < 2; g++)
        {
          uint32_t bits = (uint32_t)(bitmap >> (32 * g));

          if (bits != 0)
            {
              modifyreg32(priv->base + RM57_DCAN_IF3UPD_OFFSET(g), bits, 0);
            }
        }
    }

  priv->mbox_used &= ~bitmap;
}

/****************************************************************************
 * Name: rm57can_install_catchall / rm57can_remove_catchall
 *
 * Description:
 *   Manage the default accept-everything filter (mask = 0, MXtd clear
 *   so both standard and extended frames match) used while no user
 *   filter is active.
 *
 ****************************************************************************/

static void rm57can_install_catchall(struct rm57can_s *priv)
{
  uint64_t bitmap;

  if (!priv->catchall && rm57can_alloc_fifo(priv, &bitmap) == OK)
    {
      /* Extended object with a zero mask and MXtd/MDir clear accepts
       * every data frame.  rm57can_initfilter() sets MXtd, so build
       * the same FIFO here with the MXtd override.
       */

      uint32_t base = priv->base;
      uint32_t mctl = DCAN_IFMCTL_UMASK | DCAN_IFMCTL_DLC_MASK;
      bool eob = true;
      int mbox;

      if (priv->rxpath == RM57_DCAN_RXPATH_PIO)
        {
          mctl |= DCAN_IFMCTL_RXIE;
        }

      for (mbox = RM57_DCAN_NRXMBOX; mbox >= 1; mbox--)
        {
          if ((bitmap & (1ull << (mbox - 1))) == 0)
            {
              continue;
            }

          rm57can_disable_mbox(priv, RM57_DCAN_IF1, mbox);

          putreg32(0, base + RM57_DCAN_IFMSK_OFFSET(RM57_DCAN_IF1));
          putreg32(DCAN_IFARB_MSGVAL | DCAN_IFARB_XTD,
                   base + RM57_DCAN_IFARB_OFFSET(RM57_DCAN_IF1));
          putreg32(mctl | (eob ? DCAN_IFMCTL_EOB : 0),
                   base + RM57_DCAN_IFMCTL_OFFSET(RM57_DCAN_IF1));
          rm57can_if_exec(priv, RM57_DCAN_IF1,
                          DCAN_IFCMD_WR | DCAN_IFCMD_MASK |
                          DCAN_IFCMD_ARB | DCAN_IFCMD_CONTROL, mbox);

          eob = false;
        }

      if (priv->rxpath != RM57_DCAN_RXPATH_PIO)
        {
          int g;

          for (g = 0; g < 2; g++)
            {
              uint32_t bits = (uint32_t)(bitmap >> (32 * g));

              if (bits != 0)
                {
                  modifyreg32(base + RM57_DCAN_IF3UPD_OFFSET(g), 0, bits);
                }
            }
        }

      priv->catchall_map = bitmap;
      priv->catchall = true;
    }
}

static void rm57can_remove_catchall(struct rm57can_s *priv)
{
  if (priv->catchall)
    {
      rm57can_free_fifo(priv, priv->catchall_map);
      priv->catchall_map = 0;
      priv->catchall = false;
    }
}

/****************************************************************************
 * Name: rm57can_rxframe_if2
 *
 * Description:
 *   Read one received message object through IF2 (interrupt context),
 *   clearing NewDat and IntPnd in the same transfer, and hand the raw
 *   register images to the frontend.
 *
 ****************************************************************************/

static void rm57can_note_rxlost(struct rm57can_s *priv, int mbox,
                                uint32_t mctl)
{
  priv->rxlost++;

  /* MsgLst is sticky: TRM SPNU562A section 26.17 defines it as "no
   * message was lost since the last time when this bit was reset by the
   * CPU".  Left alone it stays set forever and every later read of the
   * same object counts another phantom overflow, so write the control
   * section back with it cleared.  NewDat and IntPnd have to be masked
   * out of the write as well: the IF register image still holds the
   * values from before the read transfer cleared them in the message
   * RAM, and writing them back would resurrect the frame we just took.
   *
   * mbox 0 means the caller does not know which object this came from -
   * the IF3 auto-copy paths only see the register image, never the
   * object number - so there the overflow is counted and reported but
   * cannot be cleared.
   */

  if (mbox > 0)
    {
      putreg32(mctl & ~(DCAN_IFMCTL_MSGLST | DCAN_IFMCTL_INTPND |
                        DCAN_IFMCTL_NEWDAT),
               priv->base + RM57_DCAN_IFMCTL_OFFSET(RM57_DCAN_IF2));
      rm57can_if_exec(priv, RM57_DCAN_IF2,
                      DCAN_IFCMD_WR | DCAN_IFCMD_CONTROL, mbox);
    }

  if (priv->cbs->rxoverflow != NULL)
    {
      priv->cbs->rxoverflow(priv->ctx);
    }
}

static void rm57can_rxframe_if2(struct rm57can_s *priv, int mbox)
{
  uint32_t base = priv->base;
  uint32_t arb;
  uint32_t mctl;
  uint32_t da;
  uint32_t db;

  rm57can_if_exec(priv, RM57_DCAN_IF2,
                  DCAN_IFCMD_ARB | DCAN_IFCMD_CONTROL | DCAN_IFCMD_DATAA |
                  DCAN_IFCMD_DATAB | DCAN_IFCMD_TXRQST |
                  DCAN_IFCMD_CLRINTPND, mbox);

  arb  = getreg32(base + RM57_DCAN_IFARB_OFFSET(RM57_DCAN_IF2));
  mctl = getreg32(base + RM57_DCAN_IFMCTL_OFFSET(RM57_DCAN_IF2));
  da   = getreg32(base + RM57_DCAN_IFDATA_OFFSET(RM57_DCAN_IF2));
  db   = getreg32(base + RM57_DCAN_IFDATB_OFFSET(RM57_DCAN_IF2));

  if ((mctl & DCAN_IFMCTL_MSGLST) != 0)
    {
      rm57can_note_rxlost(priv, mbox, mctl);
    }

  if (priv->cbs->rxframe != NULL)
    {
      priv->cbs->rxframe(priv->ctx, arb, mctl, da, db);
    }
}

/****************************************************************************
 * Name: rm57can_isr0
 *
 * Description:
 *   Interrupt line 0: status/error interrupt (Int0ID == 0x8000) and
 *   TX mailbox completions.
 *
 ****************************************************************************/

static int rm57can_isr0(int irq, void *context, void *arg)
{
  struct rm57can_s *priv = (struct rm57can_s *)arg;
  uint32_t intid;
  int bound;

  for (bound = 0; bound < RM57_DCAN_ISR_BOUND; bound++)
    {
      intid = getreg32(priv->base + RM57_DCAN_INT_OFFSET) &
              DCAN_INT_INT0ID_MASK;
      if (intid == 0)
        {
          break;
        }

      if (intid == DCAN_INT_INT0ID_STATUS)
        {
          /* One read services (and clears) the status flags */

          uint32_t es   = getreg32(priv->base + RM57_DCAN_ES_OFFSET);
          uint32_t errc = getreg32(priv->base + RM57_DCAN_ERRC_OFFSET);

          if ((es & DCAN_ES_BOFF) != 0 && !priv->boff)
            {
              /* Bus-off entry: the hardware has set CTL.Init and every
               * in-flight transmission is lost.  Reset the TX window;
               * recovery is via CTL.ABO or rm57can_busoff_recovery().
               */

              priv->boff        = true;
              priv->tx_used     = 0;
              priv->tx_inflight = 0;

              if (priv->cbs->txempty != NULL)
                {
                  priv->cbs->txempty(priv->ctx);
                }
            }
          else if ((es & DCAN_ES_BOFF) == 0 && priv->boff)
            {
              priv->boff = false;
            }

          if (priv->cbs->errevent != NULL)
            {
              priv->cbs->errevent(priv->ctx, es, errc);
            }

          continue;
        }

      if (intid >= 1 && intid <= RM57_DCAN_NMSGOBJ)
        {
          int mbox = (int)intid;

          if (mbox >= RM57_DCAN_TXMBOX_FIRST)
            {
              /* TX completion: clear IntPnd through IF2 */

              rm57can_if_exec(priv, RM57_DCAN_IF2, DCAN_IFCMD_CLRINTPND,
                              mbox);

              if (priv->tx_inflight > 0)
                {
                  priv->tx_inflight--;
                }

              if (priv->cbs->txdone != NULL)
                {
                  priv->cbs->txdone(priv->ctx);
                }

              if (priv->tx_inflight == 0)
                {
                  priv->tx_used = 0;

                  if (priv->cbs->txempty != NULL)
                    {
                      priv->cbs->txempty(priv->ctx);
                    }
                }
            }
          else
            {
              /* An RX object on line 0 - not expected with the INTMUX
               * routing, but drain it anyway.
               */

              rm57can_rxframe_if2(priv, mbox);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: rm57can_isr1
 *
 * Description:
 *   Interrupt line 1: RX mailbox interrupts (PIO path).  Int1ID always
 *   holds the lowest pending RX object.
 *
 ****************************************************************************/

static int rm57can_isr1(int irq, void *context, void *arg)
{
  struct rm57can_s *priv = (struct rm57can_s *)arg;
  uint32_t intid;
  int bound;

  for (bound = 0; bound < RM57_DCAN_ISR_BOUND; bound++)
    {
      intid = (getreg32(priv->base + RM57_DCAN_INT_OFFSET) &
               DCAN_INT_INT1ID_MASK) >> DCAN_INT_INT1ID_SHIFT;
      if (intid == 0)
        {
          break;
        }

      /* Int1ID is an 8-bit field but only 1..64 are message objects.
       * Anything else would index the message RAM out of bounds.
       */

      if (intid > RM57_DCAN_NMSGOBJ)
        {
          canerr("ERROR: DCAN%d bogus Int1ID %" PRIu32 "\n",
                 priv->port, intid);
          break;
        }

      rm57can_rxframe_if2(priv, (int)intid);
    }

  return OK;
}

/****************************************************************************
 * Name: rm57can_isrif3
 *
 * Description:
 *   IF3 interrupt: the message handler has auto-copied a received
 *   object into the IF3 register set.  Reading the observation-flagged
 *   sections (Arb/Ctrl/DataA/DataB) completes the cycle and re-arms
 *   IF3 with the next pending object.
 *
 ****************************************************************************/

static int rm57can_isrif3(int irq, void *context, void *arg)
{
  struct rm57can_s *priv = (struct rm57can_s *)arg;
  uint32_t base = priv->base;
  int bound;

  for (bound = 0; bound < RM57_DCAN_ISR_BOUND; bound++)
    {
      uint32_t arb;
      uint32_t mctl;
      uint32_t da;
      uint32_t db;

      if ((getreg32(base + RM57_DCAN_IF3OBS_OFFSET) & DCAN_IF3OBS_UPD) == 0)
        {
          break;
        }

      arb  = getreg32(base + RM57_DCAN_IF3ARB_OFFSET);
      mctl = getreg32(base + RM57_DCAN_IF3MCTL_OFFSET);
      da   = getreg32(base + RM57_DCAN_IF3DATA_OFFSET);
      db   = getreg32(base + RM57_DCAN_IF3DATB_OFFSET);

      if ((mctl & DCAN_IFMCTL_MSGLST) != 0)
        {
          rm57can_note_rxlost(priv, 0, mctl);
        }

      if (priv->cbs->rxframe != NULL)
        {
          priv->cbs->rxframe(priv->ctx, arb, mctl, da, db);
        }
    }

  return OK;
}

#ifdef RM57_DCAN_HAVE_IF3DMA

/****************************************************************************
 * Name: rm57can_if3_read
 *
 * Description:
 *   CPU drain of one IF3 update.  Reading the four observation-flagged
 *   registers completes the cycle, re-arms IF3 and clears its pending
 *   DMA request line.  Used by the stuck-request watchdog.
 *
 ****************************************************************************/

static void rm57can_if3_read(struct rm57can_s *priv)
{
  uint32_t base = priv->base;
  uint32_t arb  = getreg32(base + RM57_DCAN_IF3ARB_OFFSET);
  uint32_t mctl = getreg32(base + RM57_DCAN_IF3MCTL_OFFSET);
  uint32_t da   = getreg32(base + RM57_DCAN_IF3DATA_OFFSET);
  uint32_t db   = getreg32(base + RM57_DCAN_IF3DATB_OFFSET);

  if ((mctl & DCAN_IFMCTL_MSGLST) != 0)
    {
      rm57can_note_rxlost(priv, 0, mctl);
    }

  if (priv->cbs->rxframe != NULL)
    {
      priv->cbs->rxframe(priv->ctx, arb, mctl, da, db);
    }
}

/****************************************************************************
 * Name: rm57can_if3dma_callback
 *
 * Description:
 *   Block-transfer-complete callback: the DMA moved one auto-updated
 *   IF3 message object (Arb/Ctrl/DataA/DataB) into the landing buffer.
 *   The AIM bit reloaded the channel from the control packet, so it is
 *   already armed for the next request - dispatch the frame.
 *
 ****************************************************************************/

static void rm57can_if3dma_callback(DMA_HANDLE handle, uint8_t status,
                                    void *arg)
{
  struct rm57can_s *priv = (struct rm57can_s *)arg;
  uint32_t arb;
  uint32_t mctl;

  up_invalidate_dcache((uintptr_t)priv->if3buf,
                       (uintptr_t)priv->if3buf + 32);

  arb  = priv->if3buf[0];
  mctl = priv->if3buf[1];

  if ((mctl & DCAN_IFMCTL_MSGLST) != 0)
    {
      rm57can_note_rxlost(priv, 0, mctl);
    }

  priv->if3stuck = false;

  if (priv->cbs->rxframe != NULL)
    {
      priv->cbs->rxframe(priv->ctx, arb, mctl,
                         priv->if3buf[2], priv->if3buf[3]);
    }
}

/****************************************************************************
 * Name: rm57can_if3wd_expiry
 *
 * Description:
 *   Watchdog against a wedged IF3 request: if an IF3 update has been
 *   pending across two consecutive periods without the DMA draining it
 *   (a request pulse lost while the controller re-armed), read the IF3
 *   registers directly - that completes the cycle and unblocks the
 *   auto-update pipeline.
 *
 ****************************************************************************/

static void rm57can_if3wd_expiry(wdparm_t arg)
{
  struct rm57can_s *priv = (struct rm57can_s *)(uintptr_t)arg;
  int bound;

  if (!priv->setup)
    {
      return;
    }

  if ((getreg32(priv->base + RM57_DCAN_IF3OBS_OFFSET) &
       DCAN_IF3OBS_UPD) != 0)
    {
      if (priv->if3stuck)
        {
          canerr("DCAN%d: stuck IF3 update, draining by CPU\n",
                 priv->port);

          for (bound = 0; bound < RM57_DCAN_ISR_BOUND; bound++)
            {
              if ((getreg32(priv->base + RM57_DCAN_IF3OBS_OFFSET) &
                   DCAN_IF3OBS_UPD) == 0)
                {
                  break;
                }

              rm57can_if3_read(priv);
            }

          priv->if3stuck = false;
        }
      else
        {
          priv->if3stuck = true;
        }
    }
  else
    {
      priv->if3stuck = false;
    }

  wd_start(&priv->if3wd, RM57_DCAN_IF3WD_TICKS,
           rm57can_if3wd_expiry, arg);
}

/****************************************************************************
 * Name: rm57can_if3dma_setup / rm57can_if3dma_teardown
 *
 * Description:
 *   Arm the IF3 DMA channel: one frame of four 32-bit elements from
 *   the contiguous IF3ARB..IF3DATB registers into the landing buffer,
 *   frame-triggered by the IF3 request line, with AIM so the channel
 *   re-initializes itself after every block (no re-arm window).
 *
 ****************************************************************************/

static int rm57can_if3dma_setup(struct rm57can_s *priv)
{
  struct rm57_dmacfg_s cfg;

  priv->dma = rm57_dmachannel(priv->dmareq);
  if (priv->dma == NULL)
    {
      canerr("DCAN%d: no DMA channel for IF3 request %u\n",
             priv->port, priv->dmareq);
      return -EBUSY;
    }

  memset(&cfg, 0, sizeof(cfg));
  cfg.saddr     = priv->base + RM57_DCAN_IF3ARB_OFFSET;
  cfg.daddr     = (uint32_t)(uintptr_t)priv->if3buf;
  cfg.nframes   = 1;
  cfg.nelems    = 4;
  cfg.chctrl    = DMA_CHCTRL_RES_32BIT | DMA_CHCTRL_WES_32BIT |
                  DMA_CHCTRL_ADDMR_INCR | DMA_CHCTRL_ADDMW_INCR |
                  DMA_CHCTRL_AIM;
  cfg.parassign = DMA_PAR_AB_B_RD_A_WR;

  rm57_dmasetup(priv->dma, &cfg);

  up_invalidate_dcache((uintptr_t)priv->if3buf,
                       (uintptr_t)priv->if3buf + 32);

  rm57_dmastart(priv->dma, rm57can_if3dma_callback, priv, false);

  priv->if3stuck = false;
  wd_start(&priv->if3wd, RM57_DCAN_IF3WD_TICKS,
           rm57can_if3wd_expiry, (wdparm_t)(uintptr_t)priv);

  return OK;
}

static void rm57can_if3dma_teardown(struct rm57can_s *priv)
{
  wd_cancel(&priv->if3wd);

  if (priv->dma != NULL)
    {
      rm57_dmastop(priv->dma);
      rm57_dmafree(priv->dma);
      priv->dma = NULL;
    }
}

#endif /* RM57_DCAN_HAVE_IF3DMA */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57can_instance
 ****************************************************************************/

struct rm57can_s *rm57can_instance(int n)
{
  switch (n)
    {
#ifdef CONFIG_RM57_DCAN1
      case 0:
        return &g_dcan1;
#endif
#ifdef CONFIG_RM57_DCAN2
      case 1:
        return &g_dcan2;
#endif
#ifdef CONFIG_RM57_DCAN3
      case 2:
        return &g_dcan3;
#endif
#ifdef CONFIG_RM57_DCAN4
      case 3:
        return &g_dcan4;
#endif
      default:
        return NULL;
    }
}

/****************************************************************************
 * Name: rm57can_init
 ****************************************************************************/

int rm57can_init(struct rm57can_s *priv,
                 const struct rm57can_cbs_s *cbs, void *ctx)
{
  DEBUGASSERT(priv != NULL && cbs != NULL);

  priv->cbs         = cbs;
  priv->ctx         = ctx;
  priv->mbox_used   = 0;
  priv->catchall    = false;
  priv->catchall_map = 0;
  priv->nfilters    = 0;
  priv->tx_used     = 0;
  priv->tx_inflight = 0;
  priv->setup       = false;
  priv->boff        = false;
  priv->rxlost      = 0;
  memset(priv->filters, 0, sizeof(priv->filters));

#ifdef CONFIG_RM57_DCAN_LOOPBACK
  priv->loopback    = true;
#else
  priv->loopback    = false;
#endif
  priv->silent      = false;
  priv->nart        = false;
  priv->abom        = false;

  return rm57can_timing_from_baud(priv->initial_baud, &priv->timing);
}

/****************************************************************************
 * Name: rm57can_timing_from_baud
 *
 * Description:
 *   Derive brp/tseg1/tseg2/sjw for a bit rate at the fixed CAN_CLK.
 *   Prefers exact rate divisors and, among those, the tq count whose
 *   sample point is closest to RM57_DCAN_SAMPLEPOINT.
 *
 ****************************************************************************/

int rm57can_timing_from_baud(uint32_t baud, struct rm57can_timing_s *timing)
{
  uint32_t bestscore = UINT32_MAX;
  int ntq;

  if (baud == 0 || baud > 1000000)
    {
      return -EINVAL;
    }

  for (ntq = 8; ntq <= 25; ntq++)
    {
      uint32_t divisor = baud * ntq;
      uint32_t brp     = (RM57_DCAN_CLOCK + divisor / 2) / divisor;
      uint32_t actual;
      uint32_t rate_err;
      uint32_t sp;
      uint32_t sp_err;
      uint32_t score;
      int tseg1;
      int tseg2;

      if (brp < 1 || brp > 1024)
        {
          continue;
        }

      actual   = RM57_DCAN_CLOCK / (brp * ntq);
      rate_err = (actual > baud) ? actual - baud : baud - actual;

      /* Split the bit time at the target sample point */

      tseg1 = ((ntq * RM57_DCAN_SAMPLEPOINT) + 500) / 1000 - 1;
      if (tseg1 < 2)
        {
          tseg1 = 2;
        }
      else if (tseg1 > 16)
        {
          tseg1 = 16;
        }

      tseg2 = ntq - 1 - tseg1;
      if (tseg2 < 1 || tseg2 > 8)
        {
          if (tseg2 < 1)
            {
              tseg2 = 1;
            }
          else
            {
              tseg2 = 8;
            }

          tseg1 = ntq - 1 - tseg2;
          if (tseg1 < 2 || tseg1 > 16)
            {
              continue;
            }
        }

      sp     = ((1 + tseg1) * 1000) / ntq;
      sp_err = (sp > RM57_DCAN_SAMPLEPOINT) ?
               sp - RM57_DCAN_SAMPLEPOINT : RM57_DCAN_SAMPLEPOINT - sp;

      /* Exact bit rates dominate; sample point breaks ties */

      score = rate_err * 1000 + sp_err;

      if (score < bestscore)
        {
          bestscore     = score;
          timing->baud  = baud;
          timing->brp   = (uint16_t)brp;
          timing->tseg1 = (uint8_t)tseg1;
          timing->tseg2 = (uint8_t)tseg2;
          timing->sjw   = (tseg2 < 4) ? (uint8_t)tseg2 : 4;
        }
    }

  return (bestscore == UINT32_MAX) ? -EINVAL : OK;
}

/****************************************************************************
 * Name: rm57can_set_timing
 *
 * Description:
 *   Program the BTR.  Requires an Init+CCE round trip, which briefly
 *   removes the node from the bus (rejoin after 129 x 11 recessive
 *   bits when Init is cleared again).
 *
 ****************************************************************************/

int rm57can_set_timing(struct rm57can_s *priv,
                       const struct rm57can_timing_s *timing)
{
  uint32_t base = priv->base;
  uint32_t btr;
  uint32_t ctl;
  int ntq = 1 + timing->tseg1 + timing->tseg2;

  if (timing->brp < 1 || timing->brp > 1024 ||
      timing->tseg1 < 2 || timing->tseg1 > 16 ||
      timing->tseg2 < 1 || timing->tseg2 > 8 ||
      timing->sjw < 1 || timing->sjw > 4 || timing->sjw > timing->tseg2 ||
      ntq < 8 || ntq > 25)
    {
      return -EINVAL;
    }

  btr = DCAN_BTR_BRP(timing->brp - 1) |
        DCAN_BTR_BRPE((timing->brp - 1) >> 6) |
        DCAN_BTR_SJW(timing->sjw - 1) |
        DCAN_BTR_TSEG1(timing->tseg1 - 1) |
        DCAN_BTR_TSEG2(timing->tseg2 - 1);

  nxmutex_lock(&priv->iflock);

  ctl = getreg32(base + RM57_DCAN_CTL_OFFSET);
  putreg32(ctl | DCAN_CTL_INIT | DCAN_CTL_CCE, base + RM57_DCAN_CTL_OFFSET);
  putreg32(btr, base + RM57_DCAN_BTR_OFFSET);
  putreg32(ctl & ~(DCAN_CTL_INIT | DCAN_CTL_CCE),
           base + RM57_DCAN_CTL_OFFSET);

  priv->timing = *timing;

  nxmutex_unlock(&priv->iflock);
  return OK;
}

/****************************************************************************
 * Name: rm57can_setup
 ****************************************************************************/

int rm57can_setup(struct rm57can_s *priv)
{
  uint32_t base = priv->base;
  uint32_t ctl;
  uint32_t rxbits;
  int mbox;
  int g;

  DEBUGASSERT(priv->cbs != NULL);

  nxmutex_lock(&priv->iflock);

  /* Software-reset the module first: SWR restores CTL to its reset
   * value, which would undo the PMD setting that has to be in place
   * before the message RAM is initialized.
   */

  putreg32(DCAN_CTL_INIT | RM57_DCAN_CTL_PMD, base + RM57_DCAN_CTL_OFFSET);
  putreg32(DCAN_CTL_INIT | RM57_DCAN_CTL_PMD | DCAN_CTL_SWR,
           base + RM57_DCAN_CTL_OFFSET);
  while ((getreg32(base + RM57_DCAN_CTL_OFFSET) & DCAN_CTL_SWR) != 0)
    {
    }

  /* Select the SECDED mode *before* the message RAM is auto-initialized.
   * The checkbits are only generated during auto-initialization if PMD
   * is already set (TRM SPNU562A section 26.15: "During RAM
   * initialization, no ECC check will be done, but if the PMD bit is
   * set, the ECC bits will be generated", and section 2.2.4.2: "The ECC
   * or parity should be enabled on the RAMs before hardware
   * auto-initialization starts").  Initializing with SECDED still off -
   * PMD reads 0x5, "disabled", out of reset - leaves the checkbit RAM
   * holding garbage, so the first read-modify-write of any message
   * object raises a double-bit error: ES.PER, reported to the
   * application as a spurious CAN_ERROR_INTERNAL, and a message object
   * whose MsgVal the message handler clears out from under us.
   *
   * PMD: 0x5 disables SECDED, any other value enables it.
   */

  putreg32(DCAN_CTL_INIT | RM57_DCAN_CTL_PMD, base + RM57_DCAN_CTL_OFFSET);

  rm57can_meminit(priv);

  /* Enter configuration mode */

  ctl = DCAN_CTL_INIT | DCAN_CTL_CCE | RM57_DCAN_CTL_PMD;
  putreg32(ctl, base + RM57_DCAN_CTL_OFFSET);

  /* Program the bit timing */

  putreg32(DCAN_BTR_BRP(priv->timing.brp - 1) |
           DCAN_BTR_BRPE((priv->timing.brp - 1) >> 6) |
           DCAN_BTR_SJW(priv->timing.sjw - 1) |
           DCAN_BTR_TSEG1(priv->timing.tseg1 - 1) |
           DCAN_BTR_TSEG2(priv->timing.tseg2 - 1),
           base + RM57_DCAN_BTR_OFFSET);

  /* Auto-bus-on timer: resume immediately when ABO is enabled */

  putreg32(0, base + RM57_DCAN_ABOTR_OFFSET);

  /* Put the CAN pins in functional mode (writable while Init is set) */

  putreg32(DCAN_IOC_PU | DCAN_IOC_FUNC, base + RM57_DCAN_TIOC_OFFSET);
  putreg32(DCAN_IOC_PU | DCAN_IOC_FUNC, base + RM57_DCAN_RIOC_OFFSET);

  /* Invalidate every message object before use */

  for (mbox = 1; mbox <= RM57_DCAN_NMSGOBJ; mbox++)
    {
      putreg32(0, base + RM57_DCAN_IFARB_OFFSET(RM57_DCAN_IF1));
      rm57can_if_exec(priv, RM57_DCAN_IF1,
                      DCAN_IFCMD_WR | DCAN_IFCMD_ARB, mbox);
    }

  /* Route all RX objects to interrupt line 1, TX objects to line 0.
   * Only INTMUX groups 0 and 1 cover the 64 implemented objects; the
   * remaining groups are cleared for good measure.
   */

  for (g = 0; g < 4; g++)
    {
      uint64_t rxmask = (RM57_DCAN_NRXMBOX >= 64) ? ~0ull :
                        ((1ull << RM57_DCAN_NRXMBOX) - 1);

      rxbits = (g < 2) ? (uint32_t)(rxmask >> (32 * g)) : 0;
      putreg32(rxbits, base + RM57_DCAN_INTMUX_OFFSET(g));
    }

  /* No IF3 auto-update until filters claim mailboxes */

  for (g = 0; g < 2; g++)
    {
      putreg32(0, base + RM57_DCAN_IF3UPD_OFFSET(g));
    }

  /* TX mailboxes: exact-ID acceptance masks with MDir and MXtd so no
   * incoming data frame can ever be stored in a TX object (data frames
   * only match direction = receive when MDir is set).
   */

  for (mbox = RM57_DCAN_TXMBOX_FIRST; mbox <= RM57_DCAN_NMSGOBJ; mbox++)
    {
      putreg32(DCAN_IFMSK_MSK_MASK | DCAN_IFMSK_MDIR | DCAN_IFMSK_MXTD,
               base + RM57_DCAN_IFMSK_OFFSET(RM57_DCAN_IF1));
      rm57can_if_exec(priv, RM57_DCAN_IF1,
                      DCAN_IFCMD_WR | DCAN_IFCMD_MASK, mbox);
    }

  /* Reinstate acceptance filtering state */

  priv->mbox_used = 0;
  priv->catchall  = false;

  if (priv->nfilters == 0)
    {
      rm57can_install_catchall(priv);
    }
  else
    {
      /* Re-setup with user filters active is handled by the frontend
       * re-adding them; start from the catch-all.
       */

      priv->nfilters = 0;
      memset(priv->filters, 0, sizeof(priv->filters));
      rm57can_install_catchall(priv);
    }

  /* IF3 RX paths: observation flags select Arb+Ctrl+DataA+DataB as the
   * sections that complete a read cycle (Mask excluded - four 32-bit
   * reads at 0x148..0x154 re-arm IF3).
   */

  if (priv->rxpath != RM57_DCAN_RXPATH_PIO)
    {
      putreg32(DCAN_IF3OBS_ARB_OBS | DCAN_IF3OBS_CTRL_OBS |
               DCAN_IF3OBS_DATAA_OBS | DCAN_IF3OBS_DATAB_OBS,
               base + RM57_DCAN_IF3OBS_OFFSET);
    }

  /* Attach and enable interrupts */

  irq_attach(priv->irq0, rm57can_isr0, priv);
  up_enable_irq(priv->irq0);

  if (priv->rxpath == RM57_DCAN_RXPATH_PIO)
    {
      irq_attach(priv->irq1, rm57can_isr1, priv);
      up_enable_irq(priv->irq1);
    }
  else if (priv->rxpath == RM57_DCAN_RXPATH_IF3IRQ)
    {
      irq_attach(priv->irqif3, rm57can_isrif3, priv);
      up_enable_irq(priv->irqif3);
    }
#ifdef RM57_DCAN_HAVE_IF3DMA
  else
    {
      int ret = rm57can_if3dma_setup(priv);

      if (ret < 0)
        {
          up_disable_irq(priv->irq0);
          irq_detach(priv->irq0);
          putreg32(DCAN_CTL_INIT | RM57_DCAN_CTL_PMD,
                   base + RM57_DCAN_CTL_OFFSET);
          nxmutex_unlock(&priv->iflock);
          return ret;
        }
    }
#endif

  /* Optional test modes must be configured while CTL.Test is set */

  if (priv->loopback || priv->silent)
    {
      putreg32(ctl | DCAN_CTL_TEST, base + RM57_DCAN_CTL_OFFSET);
      putreg32((priv->loopback ? DCAN_TEST_LBACK : 0) |
               (priv->silent ? DCAN_TEST_SILENT : 0),
               base + RM57_DCAN_TEST_OFFSET);
    }

  /* Leave Init: interrupts on both lines, error interrupts, status
   * change interrupts when error reporting is configured, IF3 DMA/IRQ
   * request when an IF3 RX path is active.
   */

  ctl = DCAN_CTL_IE0 | DCAN_CTL_IE1 | DCAN_CTL_EIE | RM57_DCAN_CTL_PMD;

#if defined(CONFIG_CAN_ERRORS) || defined(CONFIG_NET_CAN_ERRORS)
  ctl |= DCAN_CTL_SIE;
#endif

  if (priv->rxpath != RM57_DCAN_RXPATH_PIO)
    {
      ctl |= DCAN_CTL_DE3;
    }

  if (priv->nart)
    {
      ctl |= DCAN_CTL_DAR;
    }

  if (priv->abom)
    {
      ctl |= DCAN_CTL_ABO;
    }

  if (priv->loopback || priv->silent)
    {
      ctl |= DCAN_CTL_TEST;
    }

  /* Discard everything latched while the mailboxes were being
   * programmed, so that the first status interrupt reports what happens
   * on the bus rather than the tail of our own setup.  Reading ES clears
   * PER/RxOK/TxOK/WakeUpPnd; the single-bit error flag in ECC_CS is
   * write-1-to-clear and PERR is read-only diagnostic state that only
   * updates on the next error.  This has to be the last thing before
   * interrupts are armed.
   */

  getreg32(base + RM57_DCAN_ES_OFFSET);
  putreg32(getreg32(base + RM57_DCAN_ECCCS_OFFSET) | DCAN_ECCCS_SBERR,
           base + RM57_DCAN_ECCCS_OFFSET);

  putreg32(ctl, base + RM57_DCAN_CTL_OFFSET);

  priv->tx_used     = 0;
  priv->tx_inflight = 0;
  priv->boff        = false;
  priv->setup       = true;

  nxmutex_unlock(&priv->iflock);

  caninfo("DCAN%d: baud %lu brp %u tseg1 %u tseg2 %u sjw %u rxpath %u\n",
          priv->port, (unsigned long)priv->timing.baud, priv->timing.brp,
          priv->timing.tseg1, priv->timing.tseg2, priv->timing.sjw,
          priv->rxpath);

  return OK;
}

/****************************************************************************
 * Name: rm57can_shutdown
 ****************************************************************************/

void rm57can_shutdown(struct rm57can_s *priv)
{
  if (!priv->setup)
    {
      return;
    }

  up_disable_irq(priv->irq0);
  irq_detach(priv->irq0);

  if (priv->rxpath == RM57_DCAN_RXPATH_PIO)
    {
      up_disable_irq(priv->irq1);
      irq_detach(priv->irq1);
    }
  else if (priv->rxpath == RM57_DCAN_RXPATH_IF3IRQ)
    {
      up_disable_irq(priv->irqif3);
      irq_detach(priv->irqif3);
    }
#ifdef RM57_DCAN_HAVE_IF3DMA
  else
    {
      rm57can_if3dma_teardown(priv);
    }
#endif

  /* Stop all bus activity */

  putreg32(DCAN_CTL_INIT | RM57_DCAN_CTL_PMD,
           priv->base + RM57_DCAN_CTL_OFFSET);

  priv->setup       = false;
  priv->tx_used     = 0;
  priv->tx_inflight = 0;
}

/****************************************************************************
 * Name: rm57can_send
 *
 * Description:
 *   Queue one frame in the next TX mailbox.  Thread context; grabs
 *   iflock for the IF1 transfer.  rtr transmits a remote frame (Dir=0
 *   with TxRqst).
 *
 ****************************************************************************/

int rm57can_send(struct rm57can_s *priv, bool extid, bool rtr,
                 uint32_t id, uint8_t dlc, const uint8_t *data)
{
  uint32_t base = priv->base;
  irqstate_t flags;
  uint32_t arbval;
  uint32_t da = 0;
  uint32_t db = 0;
  int mbox;
  int i;

  if (dlc > 8)
    {
      dlc = 8;
    }

  nxmutex_lock(&priv->iflock);

  if (priv->boff || !priv->setup)
    {
      nxmutex_unlock(&priv->iflock);
      return -ENETDOWN;
    }

  if (priv->tx_used >= RM57_DCAN_NTXMBOX)
    {
      nxmutex_unlock(&priv->iflock);
      return -EBUSY;
    }

  mbox = RM57_DCAN_TXMBOX_FIRST + priv->tx_used;

  /* Invalidate the object first: guards the IF write against a
   * concurrent message-handler access (see rm57can_disable_mbox).
   */

  rm57can_disable_mbox(priv, RM57_DCAN_IF1, mbox);

  if (extid)
    {
      arbval = (id & DCAN_IFARB_ID_MASK) | DCAN_IFARB_XTD |
               DCAN_IFARB_MSGVAL;
    }
  else
    {
      arbval = ((id & 0x7ff) << DCAN_IFARB_ID_STD_SHIFT) |
               DCAN_IFARB_MSGVAL;
    }

  if (!rtr)
    {
      arbval |= DCAN_IFARB_DIR;
    }

  for (i = 0; i < dlc && i < 4; i++)
    {
      da |= (uint32_t)data[i] << (8 * i);
    }

  for (i = 4; i < dlc; i++)
    {
      db |= (uint32_t)data[i] << (8 * (i - 4));
    }

  putreg32(arbval, base + RM57_DCAN_IFARB_OFFSET(RM57_DCAN_IF1));
  putreg32((dlc << DCAN_IFMCTL_DLC_SHIFT) | DCAN_IFMCTL_EOB |
           DCAN_IFMCTL_TXIE | DCAN_IFMCTL_UMASK,
           base + RM57_DCAN_IFMCTL_OFFSET(RM57_DCAN_IF1));
  putreg32(da, base + RM57_DCAN_IFDATA_OFFSET(RM57_DCAN_IF1));
  putreg32(db, base + RM57_DCAN_IFDATB_OFFSET(RM57_DCAN_IF1));

  /* The TxRqst command flag sets TxRqst+NewDat in the object atomically
   * with the transfer, starting the transmission.
   */

  rm57can_if_exec(priv, RM57_DCAN_IF1,
                  DCAN_IFCMD_WR | DCAN_IFCMD_ARB | DCAN_IFCMD_CONTROL |
                  DCAN_IFCMD_DATAA | DCAN_IFCMD_DATAB | DCAN_IFCMD_TXRQST,
                  mbox);

  flags = enter_critical_section();
  priv->tx_used++;
  priv->tx_inflight++;
  leave_critical_section(flags);

  nxmutex_unlock(&priv->iflock);
  return OK;
}

/****************************************************************************
 * Name: rm57can_txready / rm57can_txempty
 ****************************************************************************/

bool rm57can_txready(struct rm57can_s *priv)
{
  return priv->setup && !priv->boff &&
         priv->tx_used < RM57_DCAN_NTXMBOX;
}

bool rm57can_txempty(struct rm57can_s *priv)
{
  return priv->tx_inflight == 0;
}

/****************************************************************************
 * Name: rm57can_add_filter / rm57can_del_filter
 ****************************************************************************/

int rm57can_add_filter(struct rm57can_s *priv, bool ext,
                       uint32_t id, uint32_t mask)
{
  uint64_t bitmap;
  int slot;
  int ret;

  nxmutex_lock(&priv->iflock);

  for (slot = 0; slot < CONFIG_RM57_DCAN_NFILTERS_MAX; slot++)
    {
      if (priv->filters[slot] == 0)
        {
          break;
        }
    }

  if (slot >= CONFIG_RM57_DCAN_NFILTERS_MAX)
    {
      nxmutex_unlock(&priv->iflock);
      return -ENOMEM;
    }

  /* The first user filter replaces the catch-all */

  rm57can_remove_catchall(priv);

  ret = rm57can_alloc_fifo(priv, &bitmap);
  if (ret < 0)
    {
      if (priv->nfilters == 0)
        {
          rm57can_install_catchall(priv);
        }

      nxmutex_unlock(&priv->iflock);
      return ret;
    }

  rm57can_initfilter(priv, bitmap, ext, id, mask);

  priv->filters[slot]    = bitmap;
  priv->filter_ext[slot] = ext;
  priv->nfilters++;

  nxmutex_unlock(&priv->iflock);
  return slot;
}

int rm57can_del_filter(struct rm57can_s *priv, int handle)
{
  nxmutex_lock(&priv->iflock);

  if (handle < 0 || handle >= CONFIG_RM57_DCAN_NFILTERS_MAX ||
      priv->filters[handle] == 0)
    {
      nxmutex_unlock(&priv->iflock);
      return -EINVAL;
    }

  rm57can_free_fifo(priv, priv->filters[handle]);
  priv->filters[handle] = 0;
  priv->nfilters--;

  if (priv->nfilters == 0)
    {
      rm57can_install_catchall(priv);
    }

  nxmutex_unlock(&priv->iflock);
  return OK;
}

/****************************************************************************
 * Name: rm57can_connmodes
 *
 * Description:
 *   Configure loopback (TEST.LBack) and silent/bus-monitor
 *   (TEST.Silent) modes.  TEST is writable only while CTL.Test is set.
 *
 ****************************************************************************/

int rm57can_connmodes(struct rm57can_s *priv, bool loopback, bool silent)
{
  uint32_t base = priv->base;
  uint32_t ctl;

  nxmutex_lock(&priv->iflock);

  priv->loopback = loopback;
  priv->silent   = silent;

  if (priv->setup)
    {
      ctl = getreg32(base + RM57_DCAN_CTL_OFFSET);

      if (loopback || silent)
        {
          putreg32(ctl | DCAN_CTL_TEST, base + RM57_DCAN_CTL_OFFSET);
          putreg32((loopback ? DCAN_TEST_LBACK : 0) |
                   (silent ? DCAN_TEST_SILENT : 0),
                   base + RM57_DCAN_TEST_OFFSET);
        }
      else
        {
          putreg32(0, base + RM57_DCAN_TEST_OFFSET);
          putreg32(ctl & ~DCAN_CTL_TEST, base + RM57_DCAN_CTL_OFFSET);
        }
    }

  nxmutex_unlock(&priv->iflock);
  return OK;
}

/****************************************************************************
 * Name: rm57can_set_nart / rm57can_set_abom / rm57can_busoff_recovery
 ****************************************************************************/

int rm57can_set_nart(struct rm57can_s *priv, bool nart)
{
  nxmutex_lock(&priv->iflock);
  priv->nart = nart;

  if (priv->setup)
    {
      modifyreg32(priv->base + RM57_DCAN_CTL_OFFSET,
                  nart ? 0 : DCAN_CTL_DAR, nart ? DCAN_CTL_DAR : 0);
    }

  nxmutex_unlock(&priv->iflock);
  return OK;
}

int rm57can_set_abom(struct rm57can_s *priv, bool abom)
{
  nxmutex_lock(&priv->iflock);
  priv->abom = abom;

  if (priv->setup)
    {
      modifyreg32(priv->base + RM57_DCAN_CTL_OFFSET,
                  abom ? 0 : DCAN_CTL_ABO, abom ? DCAN_CTL_ABO : 0);
    }

  nxmutex_unlock(&priv->iflock);
  return OK;
}

int rm57can_busoff_recovery(struct rm57can_s *priv)
{
  if (!priv->boff)
    {
      return -EAGAIN;
    }

  /* The hardware set CTL.Init on bus-off entry; clearing it starts the
   * bus-off recovery sequence (129 x 11 recessive bits).  ES.BOff
   * clears when the sequence completes - tracked by the status ISR.
   */

  /* iflock also serializes the read-modify-writes of CTL that
   * rm57can_set_nart()/set_abom()/set_connmodes() perform.
   */

  nxmutex_lock(&priv->iflock);
  modifyreg32(priv->base + RM57_DCAN_CTL_OFFSET, DCAN_CTL_INIT, 0);
  nxmutex_unlock(&priv->iflock);
  return OK;
}

/****************************************************************************
 * Name: rm57can_rxint / rm57can_txint
 ****************************************************************************/

void rm57can_rxint(struct rm57can_s *priv, bool enable)
{
  int irq = (priv->rxpath == RM57_DCAN_RXPATH_PIO) ?
            priv->irq1 : priv->irqif3;

  if (!priv->setup || priv->rxpath == RM57_DCAN_RXPATH_IF3DMA)
    {
      /* The DMA RX path has no VIM interrupt to gate */

      return;
    }

  if (enable)
    {
      up_enable_irq(irq);
    }
  else
    {
      up_disable_irq(irq);
    }
}

void rm57can_txint(struct rm57can_s *priv, bool enable)
{
  /* Interrupt line 0 also carries the status/error interrupt, which
   * must stay live for bus-off tracking - TX interrupts are left
   * permanently enabled (the upper halves tolerate this; same approach
   * as Tiva).
   */
}

#endif /* CONFIG_RM57_DCAN */
