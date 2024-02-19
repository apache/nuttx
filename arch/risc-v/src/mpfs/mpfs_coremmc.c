/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_coremmc.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>
#include <nuttx/mmcsd.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "mpfs_coremmc.h"
#include "riscv_internal.h"
#include "hardware/mpfs_coremmc.h"

#include "mpfs_sdio_dev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_COREMMC_SR        (priv->hw_base + MPFS_COREMMC_SR_OFFSET)
#define MPFS_COREMMC_VR        (priv->hw_base + MPFS_COREMMC_VR_OFFSET)
#define MPFS_COREMMC_MJVR      (priv->hw_base + MPFS_COREMMC_MJVR_OFFSET)
#define MPFS_COREMMC_MIVR      (priv->hw_base + MPFS_COREMMC_MIVR_OFFSET)

#define MPFS_COREMMC_CMDX      (priv->hw_base + MPFS_COREMMC_CMDX_OFFSET)
#define MPFS_COREMMC_ARG1      (priv->hw_base + MPFS_COREMMC_ARG1_OFFSET)
#define MPFS_COREMMC_ARG2      (priv->hw_base + MPFS_COREMMC_ARG2_OFFSET)
#define MPFS_COREMMC_ARG3      (priv->hw_base + MPFS_COREMMC_ARG3_OFFSET)
#define MPFS_COREMMC_ARG4      (priv->hw_base + MPFS_COREMMC_ARG4_OFFSET)

#define MPFS_COREMMC_RR0       (priv->hw_base + MPFS_COREMMC_RR0_OFFSET)
#define MPFS_COREMMC_RR1       (priv->hw_base + MPFS_COREMMC_RR1_OFFSET)
#define MPFS_COREMMC_RR2       (priv->hw_base + MPFS_COREMMC_RR2_OFFSET)
#define MPFS_COREMMC_RR3       (priv->hw_base + MPFS_COREMMC_RR3_OFFSET)
#define MPFS_COREMMC_RR4       (priv->hw_base + MPFS_COREMMC_RR4_OFFSET)
#define MPFS_COREMMC_RR5       (priv->hw_base + MPFS_COREMMC_RR5_OFFSET)
#define MPFS_COREMMC_RR6       (priv->hw_base + MPFS_COREMMC_RR6_OFFSET)
#define MPFS_COREMMC_RR7       (priv->hw_base + MPFS_COREMMC_RR7_OFFSET)
#define MPFS_COREMMC_RR8       (priv->hw_base + MPFS_COREMMC_RR8_OFFSET)
#define MPFS_COREMMC_RR9       (priv->hw_base + MPFS_COREMMC_RR9_OFFSET)
#define MPFS_COREMMC_RR10      (priv->hw_base + MPFS_COREMMC_RR10_OFFSET)
#define MPFS_COREMMC_RR11      (priv->hw_base + MPFS_COREMMC_RR11_OFFSET)
#define MPFS_COREMMC_RR12      (priv->hw_base + MPFS_COREMMC_RR12_OFFSET)
#define MPFS_COREMMC_RR13      (priv->hw_base + MPFS_COREMMC_RR13_OFFSET)
#define MPFS_COREMMC_RR14      (priv->hw_base + MPFS_COREMMC_RR14_OFFSET)
#define MPFS_COREMMC_RR15      (priv->hw_base + MPFS_COREMMC_RR15_OFFSET)

#define MPFS_COREMMC_WDR       (priv->hw_base + MPFS_COREMMC_WDR_OFFSET)
#define MPFS_COREMMC_RDR       (priv->hw_base + MPFS_COREMMC_RDR_OFFSET)
#define MPFS_COREMMC_IMR       (priv->hw_base + MPFS_COREMMC_IMR_OFFSET)
#define MPFS_COREMMC_SBIMR     (priv->hw_base + MPFS_COREMMC_SBIMR_OFFSET)
#define MPFS_COREMMC_MBIMR     (priv->hw_base + MPFS_COREMMC_MBIMR_OFFSET)
#define MPFS_COREMMC_ISR       (priv->hw_base + MPFS_COREMMC_ISR_OFFSET)
#define MPFS_COREMMC_SBISR     (priv->hw_base + MPFS_COREMMC_SBISR_OFFSET)
#define MPFS_COREMMC_MBISR     (priv->hw_base + MPFS_COREMMC_MBISR_OFFSET)
#define MPFS_COREMMC_ICR       (priv->hw_base + MPFS_COREMMC_ICR_OFFSET)
#define MPFS_COREMMC_SBICR     (priv->hw_base + MPFS_COREMMC_SBICR_OFFSET)
#define MPFS_COREMMC_MBICR     (priv->hw_base + MPFS_COREMMC_MBICR_OFFSET)
#define MPFS_COREMMC_CTRL      (priv->hw_base + MPFS_COREMMC_CTRL_OFFSET)
#define MPFS_COREMMC_SBCSR     (priv->hw_base + MPFS_COREMMC_SBCSR_OFFSET)
#define MPFS_COREMMC_MBCSR     (priv->hw_base + MPFS_COREMMC_MBCSR_OFFSET)

#define MPFS_COREMMC_RSPTO     (priv->hw_base + MPFS_COREMMC_RSPTO_OFFSET)
#define MPFS_COREMMC_DATATO    (priv->hw_base + MPFS_COREMMC_DATATO_OFFSET)
#define MPFS_COREMMC_BLR       (priv->hw_base + MPFS_COREMMC_BLR_OFFSET)
#define MPFS_COREMMC_DCTRL     (priv->hw_base + MPFS_COREMMC_DCTRL_OFFSET)
#define MPFS_COREMMC_CLKR      (priv->hw_base + MPFS_COREMMC_CLKR_OFFSET)
#define MPFS_COREMMC_BCR       (priv->hw_base + MPFS_COREMMC_BCR_OFFSET)

/* Clocks and reset */

#define MPFS_SYSREG_SOFT_RESET_CR     (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SOFT_RESET_CR_OFFSET)

#define MPFS_SYSREG_SUBBLK_CLOCK_CR   (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET)

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE"
#endif

#ifndef CONFIG_SDIO_BLOCKSETUP
#  error "This requires CONFIG_SDIO_BLOCKSETUP"
#endif

/* Clocks and timing */

#define MPFS_FPGA_FIC0_CLK                 (50000000)

#define COREMMC_CMDTIMEOUT                 (100000)
#define COREMMC_LONGTIMEOUT                (100000000)
#define COREMMC_DATA_TIMEOUT               (500000)

#define MPFS_MMC_CLOCK_50MHZ               (50000000)
#define MPFS_MMC_CLOCK_400KHZ              (400000)

/* Event wait interrupt status and clear bits */

#define COREMMC_ALL_ICR       (COREMMC_ICR_CLRCSI | \
                               COREMMC_ICR_CLRRRI | \
                               COREMMC_ICR_ERROR)

/* Event wait interrupt mask bits */

#define COREMMC_RECV_MASK     (COREMMC_IMR_ERROR)

#define COREMMC_RECV_SB_MASK  (COREMMC_SBIMR_ERROR | \
                               COREMMC_SBIMR_RDONE)

#define COREMMC_RECV_MB_MASK  (COREMMC_MBIMR_ERROR | \
                               COREMMC_MBIMR_RDONE)

#define COREMMC_SEND_MASK     (COREMMC_IMR_ERROR)

#define COREMMC_SEND_SB_MASK  (COREMMC_SBIMR_ERROR | \
                               COREMMC_SBIMR_WDONE)

#define COREMMC_SEND_MB_MASK  (COREMMC_MBIMR_ERROR | \
                               COREMMC_MBIMR_WDONE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

union
{
  uint32_t w;
  uint8_t  b[4];
} data;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Initialization/setup */

static void mpfs_reset(struct sdio_dev_s *dev);
static sdio_capset_t mpfs_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t mpfs_status(struct sdio_dev_s *dev);
static void mpfs_widebus(struct sdio_dev_s *dev, bool enable);
static void mpfs_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate);
static int  mpfs_attach(struct sdio_dev_s *dev);

/* Command / Status / Data Transfer */

static int  mpfs_sendcmd(struct sdio_dev_s *dev, uint32_t cmd, uint32_t arg);
static void mpfs_blocksetup(struct sdio_dev_s *dev, unsigned int blocksize,
                            unsigned int nblocks);
static int  mpfs_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                           size_t nbytes);
static int  mpfs_sendsetup(struct sdio_dev_s *dev, const uint8_t *buffer,
                           size_t nbytes);
static int  mpfs_cancel(struct sdio_dev_s *dev);
static int  mpfs_waitresponse(struct sdio_dev_s *dev, uint32_t cmd);
static int  mpfs_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                              uint32_t *rshort);
static int  mpfs_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                          uint32_t rlong[4]);
static int  mpfs_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t *rshort);

/* Event handler */

static void mpfs_waitenable(struct sdio_dev_s *dev,
                            sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t mpfs_eventwait(struct sdio_dev_s *dev);
static void mpfs_callbackenable(struct sdio_dev_s *dev,
                                sdio_eventset_t eventset);
static int  mpfs_registercallback(struct sdio_dev_s *dev,
                                  worker_t callback, void *arg);
static void mpfs_callback(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct mpfs_dev_s g_coremmc_dev =
{
  .dev =
  {
    .reset            = mpfs_reset,
    .capabilities     = mpfs_capabilities,
    .status           = mpfs_status,
    .widebus          = mpfs_widebus,
    .clock            = mpfs_clock,
    .attach           = mpfs_attach,
    .sendcmd          = mpfs_sendcmd,
    .blocksetup       = mpfs_blocksetup,
    .recvsetup        = mpfs_recvsetup,
    .sendsetup        = mpfs_sendsetup,
    .cancel           = mpfs_cancel,
    .waitresponse     = mpfs_waitresponse,
    .recv_r1          = mpfs_recvshortcrc,
    .recv_r2          = mpfs_recvlong,
    .recv_r3          = mpfs_recvshort,
    .recv_r4          = mpfs_recvshort,
    .recv_r5          = mpfs_recvshortcrc,
    .recv_r6          = mpfs_recvshortcrc,
    .recv_r7          = mpfs_recvshort,
    .waitenable       = mpfs_waitenable,
    .eventwait        = mpfs_eventwait,
    .callbackenable   = mpfs_callbackenable,
    .registercallback = mpfs_registercallback,
  },
  .hw_base           = CONFIG_MPFS_COREMMC_BASE,
  .plic_irq          = MPFS_IRQ_FABRIC_F2H_0 + CONFIG_MPFS_COREMMC_IRQNUM,
  .blocksize         = 512,
  .fifo_depth        = 0,
  .onebit            = false,
  .polltransfer      = true,
  .multiblock        = false,
  .waitsem           = SEM_INITIALIZER(0),
};

/* Not all requests are 32-bit aligned, use a spare buffer workaround */

static uint32_t g_aligned_buffer[512 / 4];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_modifyreg8
 *
 * Description:
 *   Atomically modify the specified bits in a memory mapped register
 *
 * Input Parameters:
 *   addr       - address to use
 *   clearbits  - bit clear mask
 *   setbits    - set bit mask
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_modifyreg8(uintptr_t addr, uint8_t clearbits,
                            uint8_t setbits)
{
  irqstate_t flags;
  uint8_t    regval;

  flags   = spin_lock_irqsave(NULL);
  regval  = getreg8(addr);
  regval &= ~clearbits;
  regval |= setbits;
  putreg8(regval, addr);
  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name: mpfs_putreg32
 *
 * Description:
 *   Set the contents of a 32-bit register.  This helper wrapper checks
 *   the range before accessing the address which prevents thinkos.
 *
 * Input Parameters:
 *   regval     - Value to store
 *   regaddr    - Address where the regval is stored
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void mpfs_putreg32(struct mpfs_dev_s *priv, uint32_t regval,
                                 uintptr_t regaddr)
{
  DEBUGASSERT((regaddr >= priv->hw_base) && regaddr < (priv->hw_base +
               MPFS_COREMMC_MAX_OFFSET));

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: mpfs_putreg8
 *
 * Description:
 *   Set the contents of an 8-bit register.  This helper wrapper checks
 *   the range before accessing the address which prevents thinkos.
 *
 * Input Parameters:
 *   regval     - Value to store
 *   regaddr    - Address where the regval is stored
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void mpfs_putreg8(struct mpfs_dev_s *priv, uint8_t regval,
                                uintptr_t regaddr)
{
  DEBUGASSERT((regaddr >= priv->hw_base) && regaddr < (priv->hw_base +
               MPFS_COREMMC_MAX_OFFSET));

  putreg8(regval, regaddr);
}

/****************************************************************************
 * Name: mpfs_reset_lines
 *
 * Description:
 *   Resets the DAT and CMD lines.
 *
 * Input Parameters:
 *   priv  - Instance of the CoreMMC private state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_reset_lines(struct mpfs_dev_s *priv)
{
  mpfs_modifyreg8(MPFS_COREMMC_CTRL, 0, COREMMC_CTRL_FIFORESET);
}

/****************************************************************************
 * Name: mpfs_check_lines_busy
 *
 * Description:
 *   Verifies the DAT and CMD lines are available, not busy.
 *
 * Input Parameters:
 *   priv  - Instance of the CoreMMC private state structure.
 *
 * Returned Value:
 *   true if busy, false if available
 *
 ****************************************************************************/

static bool mpfs_check_lines_busy(struct mpfs_dev_s *priv)
{
  uint32_t retries = COREMMC_LONGTIMEOUT;
  uint8_t  ctrl;

  do
    {
      ctrl = getreg8(MPFS_COREMMC_CTRL);
    }
  while ((ctrl & COREMMC_CTRL_BUSY) && --retries);

  if (retries == 0)
    {
      mcerr("Lines are still busy!\n");
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: mpfs_setclkrate
 *
 * Description:
 *   Set the clock rate. Disables the SD clock for the time changing the
 *   settings, if already enabled.
 *
 * Input Parameters:
 *   priv  - Instance of the CoreMMC private state structure.
 *   clkcr - New clock rate.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_setclkrate(struct mpfs_dev_s *priv, uint32_t clkrate)
{
  uint8_t divider;

  mcinfo("clkrate: %08" PRIx32 "\n", clkrate);

  if (clkrate == 0)
    {
      mpfs_modifyreg8(MPFS_COREMMC_CTRL, COREMMC_CTRL_CLKOE, 0);
      priv->clk_enabled = false;
      return;
    }

  /* Disable clock if enabled */

  if (priv->clk_enabled)
    {
      mpfs_modifyreg8(MPFS_COREMMC_CTRL, COREMMC_CTRL_CLKOE, 0);
    }

  /* Datasheet has misleading clk equation, correct is:
   * Fclk = Hclk / (2 * (CLKHP - 1)), CLKHP = 50000000 / (Fclk * 2) + 1
   * except for the higher rates.
   */

  divider = MPFS_FPGA_FIC0_CLK / (clkrate * 2) + 1;

  mcinfo("divider: %02" PRIx8 "\n", divider);

  mpfs_modifyreg8(MPFS_COREMMC_CLKR, 0xff, divider);

  /* Apply new settings */

  priv->clk_enabled = true;
  mpfs_modifyreg8(MPFS_COREMMC_CTRL, 0, COREMMC_CTRL_CLKOE);
}

/****************************************************************************
 * Name: mpfs_configwaitints
 *
 * Description:
 *   Enable/disable SDIO interrupts needed to support the wait function
 *
 * Input Parameters:
 *   priv       - Instance of the CoreMMC private state structure.
 *   waitmask   - The set of bits in the SDIO MASK register to set
 *   waitevents - Waited for events
 *   wkupevent  - Wake-up events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_configwaitints(struct mpfs_dev_s *priv, uint32_t waitmask,
                                sdio_eventset_t waitevents,
                                sdio_eventset_t wkupevent)
{
  irqstate_t flags;

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags = enter_critical_section();

  priv->waitevents = waitevents;
  priv->wkupevent  = wkupevent;
  priv->waitmask   = waitmask;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: mpfs_configxfrints
 *
 * Description:
 *   Enable SDIO interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv        - Instance of the CoreMMC private state structure
 *   xfrmask     - The set of bits in the IMR register to set
 *   xfr_blkmask - The set of bits in the SB/MB IMR register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_configxfrints(struct mpfs_dev_s *priv, uint32_t xfrmask,
                               uint32_t xfr_blkmask)
{
  irqstate_t flags;

  flags = enter_critical_section();
  priv->xfrmask = xfrmask;
  priv->xfr_blkmask = xfr_blkmask;

  mcinfo("Mask: %08" PRIx32 "\n", priv->xfrmask | priv->waitmask);
  mcinfo("blkmask: %08" PRIx32 "\n", priv->xfr_blkmask);

  mpfs_putreg8(priv, priv->xfrmask | priv->waitmask, MPFS_COREMMC_IMR);

  if (priv->multiblock)
    {
      mpfs_putreg8(priv, priv->xfr_blkmask, MPFS_COREMMC_MBIMR);
    }
  else
    {
      mpfs_putreg8(priv, priv->xfr_blkmask, MPFS_COREMMC_SBIMR);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: mpfs_sendfifo
 *
 * Description:
 *   Send single block or multiblock data in interrupt mode
 *
 * Input Parameters:
 *   priv  - Instance of the CoreMMC private state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_sendfifo(struct mpfs_dev_s *priv)
{
  /* Loop while there is more data to be sent and the TX FIFO is not full */

  while (priv->remaining > 0)
    {
      /* Is there a full word remaining in the user buffer? */

      if (priv->remaining >= sizeof(uint32_t))
        {
          /* Yes, transfer the word to the TX FIFO */

          data.w           = *priv->buffer++;
          priv->remaining -= sizeof(uint32_t);
        }
      else
        {
          /* No.. transfer just the bytes remaining in the user buffer,
           * padding with zero as necessary to extend to a full word.
           */

          uint8_t *ptr = (uint8_t *)priv->remaining;
          size_t i;

          data.w = 0;
          for (i = 0; i < priv->remaining; i++)
            {
              data.b[i] = *ptr++;
            }

          /* Now the transfer is finished */

          priv->remaining = 0;
        }

      /* Put the word in the FIFO - needs to be 32-bit write  */

      mpfs_putreg32(priv, data.w, MPFS_COREMMC_WDR);
    }

  mcinfo("Wrote all\n");
}

/****************************************************************************
 * Name: mpfs_recvfifo
 *
 * Description:
 *   Receive mmc data in single block or multiblock interrupt mode
 *
 * Input Parameters:
 *   priv  - Instance of the mmc private state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_recvfifo(struct mpfs_dev_s *priv)
{
  mcinfo("Reading: %lu bytes\n", priv->remaining);

  /* Loop while there is space to store the data and there is more
   * data available in the RX FIFO.
   */

  while (priv->remaining > 0)
    {
      /* Read the next word from the RX FIFO */

      data.w = getreg32(MPFS_COREMMC_RDR);

      if (priv->remaining >= sizeof(uint32_t))
        {
          /* Transfer the whole word to the user buffer */

          *priv->buffer++  = data.w;
          priv->remaining -= sizeof(uint32_t);
        }
      else
        {
          /* Transfer any trailing fractional word */

          uint8_t *ptr = (uint8_t *)priv->buffer;
          size_t i;

          for (i = 0; i < priv->remaining; i++)
            {
              *ptr++ = data.b[i];
            }

          /* Now the transfer is finished */

          priv->remaining = 0;
        }
    }

    mcinfo("Read all\n");
}

/****************************************************************************
 * Name: mpfs_endwait
 *
 * Description:
 *   Wake up a waiting thread if the waited-for event has occurred.
 *
 * Input Parameters:
 *   priv  - Instance of the CoreMMC private state structure.
 *   wkupevent - The event that caused the wait to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void mpfs_endwait(struct mpfs_dev_s *priv,
                         sdio_eventset_t wkupevent)
{
  mcinfo("wkupevent: %u\n", wkupevent);

  /* Cancel the watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Disable event-related interrupts */

  mpfs_configwaitints(priv, 0, 0, wkupevent);

  /* Wake up the waiting thread */

  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: mpfs_eventtimeout
 *
 * Description:
 *   The watchdog timeout setup when the event wait start has expired without
 *   any other waited-for event occurring.
 *
 * Input Parameters:
 *   arg    - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void mpfs_eventtimeout(wdparm_t arg)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)arg;

  /* There is always race conditions with timer expirations. */

  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0 ||
               priv->wkupevent != 0);

  mcinfo("sta: %08" PRIx32 " enabled irq: %08" PRIx32 "\n",
         getreg8(MPFS_COREMMC_ISR),
         getreg8(MPFS_COREMMC_IMR));

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. wake up any waiting threads */

      mpfs_endwait(priv, SDIOWAIT_TIMEOUT);
      mcerr("Timeout: remaining: %lu\n", priv->remaining);
    }
}

/****************************************************************************
 * Name: mpfs_endtransfer
 *
 * Description:
 *   Terminate a transfer with the provided status.  This function is called
 *   only from the interrupt handler when end-of-transfer conditions are
 *   detected.
 *
 * Input Parameters:
 *   priv  - Instance of the CoreMMC private state structure.
 *   wkupevent - The event that caused the transfer to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void mpfs_endtransfer(struct mpfs_dev_s *priv,
                             sdio_eventset_t wkupevent)
{
  /* Disable all transfer related interrupts */

  mpfs_configxfrints(priv, 0, 0);

  /* If there were errors, reset lines */

  if ((wkupevent & (~SDIOWAIT_TRANSFERDONE)) != 0)
    {
      mpfs_reset_lines(priv);
    }

  /* Clear Read Done (RDONE), Write Done (WDONE) */

  if (priv->multiblock)
    {
      mpfs_putreg8(priv, COREMMC_MBICR_RDONE | COREMMC_MBICR_WDONE,
                   MPFS_COREMMC_MBICR);
    }
  else
    {
      mpfs_putreg8(priv, COREMMC_SBICR_RDONE | COREMMC_SBICR_WDONE,
                   MPFS_COREMMC_SBICR);
    }

  /* Mark the transfer finished */

  priv->remaining = 0;

  /* Is a thread wait for these data transfer complete events? */

  if ((priv->waitevents & wkupevent) != 0)
    {
      /* Yes.. wake up any waiting threads */

      mpfs_endwait(priv, wkupevent);
    }
}

/****************************************************************************
 * Name: mpfs_coremmc_interrupt
 *
 * Description:
 *   coremmc interrupt handler
 *
 * Input Parameters:
 *   priv  - Instance of the coremmc private state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int mpfs_coremmc_interrupt(int irq, void *context, void *arg)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)arg;
  uint8_t status;
  uint8_t status_xb;

  DEBUGASSERT(priv != NULL);

  status = getreg8(MPFS_COREMMC_ISR);

  if (priv->multiblock)
    {
      status_xb = getreg8(MPFS_COREMMC_MBISR);
    }
  else
    {
      status_xb = getreg8(MPFS_COREMMC_SBISR);
    }

  mcinfo("status: %02" PRIx8 " mb: %02" PRIx8 "\n", status, status_xb);

  /* Check for errors first */

  if (status & COREMMC_ISR_ERROR)
    {
      mcerr("ISR ERROR: %08" PRIx8 "\n", status);
      mpfs_endtransfer(priv, SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
      return OK;
    }

  if (status_xb & COREMMC_XBISR_ERROR)
    {
      mcerr("XBISR ERROR: %08" PRIx8 "\n", status_xb);
      mpfs_endtransfer(priv, SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
      return OK;
    }

  /* Check single / multiple block activities */

  if (status_xb)
    {
      if (status_xb & COREMMC_XBISR_RDONE)
        {
          mpfs_recvfifo(priv);
          if (priv->multiblock)
            {
              mpfs_putreg8(priv, COREMMC_MBICR_RDONE, MPFS_COREMMC_MBICR);
            }
          else
            {
              mpfs_putreg8(priv, COREMMC_SBICR_RDONE, MPFS_COREMMC_SBICR);
            }

          if (priv->remaining == 0)
            {
              mpfs_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
            }
        }

      if (status_xb & COREMMC_XBISR_WDONE)
        {
          if (priv->multiblock)
            {
              mpfs_putreg8(priv, COREMMC_MBICR_WDONE, MPFS_COREMMC_MBICR);
            }
          else
            {
              mpfs_putreg8(priv, COREMMC_SBICR_WDONE, MPFS_COREMMC_SBICR);
            }

          mpfs_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
        }
    }

  mcinfo("done\n");

  return OK;
}

/****************************************************************************
 * Name: mpfs_lock
 *
 * Description:
 *   Locks the bus. Function calls low-level multiplexed bus routines to
 *   resolve bus requests and acknowledgment issues.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   lock   - TRUE to lock, FALSE to unlock.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#if defined(CONFIG_SDIO_MUXBUS)
static int mpfs_lock(struct sdio_dev_s *dev, bool lock)
{
  /* The multiplex bus is part of board support package. */

  mpfs_muxbus_sdio_lock(dev, lock);

  return OK;
}
#endif

/****************************************************************************
 * Name: mpfs_set_data_timeout
 *
 * Description:
 *   Sets the cycles for determining the data line timeout.
 *
 * Input Parameters:
 *   priv        - Instance of the CoreMMC private state structure.
 *   timeout_us  - Requested timeout in microseconds
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_set_data_timeout(struct mpfs_dev_s *priv,
                                  uint32_t timeout_us)
{
  uint32_t timeout;

  /* MPFS_FPGA_FIC0_CLK = 125 Mhz:
   * uS = 0.000001 s, clk tick =  1 / 125 MHz = 0.008 uS
   */

  timeout = (MPFS_FPGA_FIC0_CLK / USEC_PER_SEC) * timeout_us;

  mpfs_putreg32(priv, timeout, MPFS_COREMMC_DATATO);

  /* Response timeout */

  mpfs_putreg8(priv, 0xc0, MPFS_COREMMC_RSPTO);
}

/****************************************************************************
 * Name: mpfs_device_reset
 *
 * Description:
 *   Reset the SDIO controller. Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *
 * Returned Value:
 *   true on success, false otherwise
 *
 ****************************************************************************/

static bool mpfs_device_reset(struct sdio_dev_s *dev)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  uint8_t fifo_size;
  bool retval = true;

  /* CoreMMC uses FIC0 clock at FPGA. Don't perform resets on it as
   * many other drivers may be using the same CLK provider. Just
   * make sure the FPGA is out of reset and clock is on.
   */

  modifyreg32(MPFS_SYSREG_SOFT_RESET_CR, SYSREG_SOFT_RESET_CR_FPGA |
              SYSREG_SOFT_RESET_CR_FIC0, 0);

  modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR, 0, SYSREG_SUBBLK_CLOCK_CR_FIC0);

  nxsig_usleep(100);

  /* Perform module and slave reset */

  mpfs_modifyreg8(MPFS_COREMMC_CTRL, 0, COREMMC_CTRL_SLRST |
                  COREMMC_CTRL_SWRST);

  nxsig_usleep(100);

  mpfs_modifyreg8(MPFS_COREMMC_CTRL, COREMMC_CTRL_SLRST | COREMMC_CTRL_SWRST,
                  0);

  nxsig_usleep(100);

  /* Clear interrupt status and disable interrupts */

  mpfs_putreg8(priv, COREMMC_ALL_ICR, MPFS_COREMMC_ICR);
  mpfs_putreg8(priv, 0, MPFS_COREMMC_IMR);

  mpfs_set_data_timeout(priv, COREMMC_DATA_TIMEOUT);

  /* Set 1-bit bus mode */

  mpfs_modifyreg8(MPFS_COREMMC_DCTRL, COREMMC_DCTRL_DSIZE,
                  COREMMC_DCTRL_DSIZE_1BIT);

  mpfs_setclkrate(priv, MPFS_MMC_CLOCK_400KHZ);

  nxsig_usleep(100);

  /* Store fifo size for later to check no fifo overruns occur */

  fifo_size = ((getreg8(MPFS_COREMMC_VR) >> 2) & 0x3);
  if (fifo_size == 0)
    {
      priv->fifo_depth = 512;
    }
  else if (fifo_size == 1)
    {
      priv->fifo_depth = 1024 * 4;
    }
  else if (fifo_size == 2)
    {
      priv->fifo_depth = 1024 * 16;
    }
  else
    {
      priv->fifo_depth = 1024 * 32;
    }

  /* Reset data */

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitmask   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */

  wd_cancel(&priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->buffer     = 0;      /* Address of current R/W buffer */
  priv->remaining  = 0;      /* Number of bytes remaining in the transfer */
  priv->xfrmask    = 0;      /* Interrupt enables for data transfer */

  priv->widebus    = false;

  mpfs_reset_lines(priv);

  return retval;
}

/****************************************************************************
 * Name: mpfs_reset
 *
 * Description:
 *   Reset the SDIO controller via mpfs_device_reset. This is a wrapper
 *   function only.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_reset(struct sdio_dev_s *dev)
{
  mpfs_device_reset(dev);
}

/****************************************************************************
 * Name: mpfs_capabilities
 *
 * Description:
 *   Get capabilities (and limitations) of the SDIO driver (optional)
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see SDIO_CAPS_* defines)
 *
 ****************************************************************************/

static sdio_capset_t mpfs_capabilities(struct sdio_dev_s *dev)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  sdio_capset_t caps = 0;

  if (priv->onebit)
    {
      caps |= SDIO_CAPS_1BIT_ONLY;
    }

  return caps;
}

/****************************************************************************
 * Name: mpfs_status
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see mpfs_status_* defines)
 *
 ****************************************************************************/

static sdio_statset_t mpfs_status(struct sdio_dev_s *dev)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  return priv->cdstatus;
}

/****************************************************************************
 * Name: mpfs_widebus
 *
 * Description:
 *   Called after change in Bus width has been selected (via ACMD6).  Most
 *   controllers will need to perform some special operations to work
 *   correctly in the new bus mode.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   wide - true: wide bus (4-bit) bus mode enabled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_widebus(struct sdio_dev_s *dev, bool wide)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  priv->widebus = wide;

  mcinfo("wide: %d\n", wide);
}

/****************************************************************************
 * Name: mpfs_set_hs_mode
 *
 * Description:
 *   Sets the device in HS mode via CMD6.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_set_hs_mode(struct sdio_dev_s *dev)
{
  uint32_t r1;
  int ret;

  if ((ret = mpfs_sendcmd(dev, MMCSD_CMD6, 0x03b90100u)) == OK)
    {
      if ((ret == mpfs_waitresponse(dev, MMCSD_CMD6)) == OK)
        {
          ret = mpfs_recvshortcrc(dev, MMCSD_CMD6, &r1);
        }
    }

  if (ret < 0)
    {
      mcerr("Failed to set high speed mode\n");
      return;
    }
}

/****************************************************************************
 * Name: mpfs_set_4bit_mode
 *
 * Description:
 *   Sets the device for 4-bit data width via CMD6.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_set_4bit_mode(struct sdio_dev_s *dev)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  uint32_t r1;
  uint8_t dw;
  int ret;

  if ((ret = mpfs_sendcmd(dev, MMCSD_CMD6, 0x03b70100u)) == OK)
    {
      if ((ret == mpfs_waitresponse(dev, MMCSD_CMD6)) == OK)
        {
          ret = mpfs_recvshortcrc(dev, MMCSD_CMD6, &r1);
        }
    }

  if (ret < 0)
    {
      mcerr("Failed to set 4-bit mode\n");
    }
  else
    {
      dw = (getreg8(MPFS_COREMMC_VR) >> 2) & 3;

      if (dw == 0)
        {
          mcerr("HW doesn't support 4-bit data width\n");
        }

      /* CoreMMC 4-bit mode */

      mpfs_modifyreg8(MPFS_COREMMC_DCTRL, COREMMC_DCTRL_DSIZE,
                      COREMMC_DCTRL_DSIZE_4BIT);
    }
}

/****************************************************************************
 * Name: mpfs_clock
 *
 * Description:
 *   Enable/disable SDIO clocking. Only up to 25 Mhz is supported now. 50 Mhz
 *   may work with some cards.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   rate - Specifies the clocking to use (see enum sdio_clock_e)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  uint32_t clckr;

  switch (rate)
  {
    /* Disable clock */

    default:
    case CLOCK_SDIO_DISABLED:
      clckr = 0;
      break;

    /* Enable in initial ID mode clocking (400KHz) */

    case CLOCK_IDMODE:
      clckr = MPFS_MMC_CLOCK_400KHZ;
      break;

    /* Enable normal MMC operation clocking */

    case CLOCK_MMC_TRANSFER:
      clckr = MPFS_MMC_CLOCK_50MHZ;
      break;

    case CLOCK_SD_TRANSFER_4BIT:
    case CLOCK_SD_TRANSFER_1BIT:
      clckr = MPFS_MMC_CLOCK_50MHZ;
      break;
  }

  if (clckr == MPFS_MMC_CLOCK_50MHZ)
    {
      mpfs_set_hs_mode(dev);
    }

  /* Set the new clock frequency */

  mpfs_setclkrate(priv, clckr);

  if (rate == CLOCK_SD_TRANSFER_4BIT)
    {
      /* Need to settle a bit before issuing this CMD6 after clk change */

      nxsig_usleep(100);

      mpfs_set_4bit_mode(dev);
    }
}

/****************************************************************************
 * Name: mpfs_attach
 *
 * Description:
 *   Attach and prepare interrupts
 *
 * Input Parameters:
 *   dev - An instance of the mmc device interface
 *
 * Returned Value:
 *   OK on success; A negated errno on failure.
 *
 ****************************************************************************/

static int mpfs_attach(struct sdio_dev_s *dev)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  int ret;

  /* Attach the mmc interrupt handler */

  ret = irq_attach(priv->plic_irq, mpfs_coremmc_interrupt, priv);
  if (ret == OK)
    {
      /* Disable all interrupts and clear interrupt status
       * registers.
       */

      mpfs_putreg8(priv, 0x00, MPFS_COREMMC_IMR);
      mpfs_putreg8(priv, 0xff, MPFS_COREMMC_ICR);

      mpfs_putreg8(priv, 0x00, MPFS_COREMMC_SBIMR);
      mpfs_putreg8(priv, 0xff, MPFS_COREMMC_SBICR);

      mpfs_putreg8(priv, 0x00, MPFS_COREMMC_MBIMR);
      mpfs_putreg8(priv, 0xff, MPFS_COREMMC_MBICR);

      up_enable_irq(priv->plic_irq);
    }

  mcinfo("attach: %d\n", ret);

  return ret;
}

/****************************************************************************
 * Name: mpfs_sendcmd
 *
 * Description:
 *   Send the SDIO command
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command to send (32-bits, encoded)
 *   arg  - 32-bit argument required with some commands
 *
 * Returned Value:
 *   OK if no errors, an error otherwise
 *
 ****************************************************************************/

static int mpfs_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                        uint32_t arg)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  uint32_t cmdidx;

  mpfs_reset_lines(priv);

  /* Check if command / data lines are busy */

  if (mpfs_check_lines_busy(priv))
    {
      mcerr("Busy!\n");
      return -EBUSY;
    }

  /* Clear all status interrupts */

  mpfs_putreg8(priv, COREMMC_ALL_ICR, MPFS_COREMMC_ICR);

  cmdidx = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;

  /* Arg must be the last one */

  mpfs_putreg8(priv, (uint8_t)cmdidx, MPFS_COREMMC_CMDX);

  /* The argument needs endianness swapped */

  mpfs_putreg8(priv, (arg >> 24), MPFS_COREMMC_ARG1);
  mpfs_putreg8(priv, (arg >> 16) & 0xff, MPFS_COREMMC_ARG2);
  mpfs_putreg8(priv, (arg >> 8) & 0xff, MPFS_COREMMC_ARG3);
  mpfs_putreg8(priv, (arg & 0xff), MPFS_COREMMC_ARG4);

  mcinfo(" cmd: %08" PRIx32 " arg: %08" PRIx32 " cmdidx: %08" PRIx32 "\n",
         cmd, arg, cmdidx);

  return OK;
}

/****************************************************************************
 * Name: mpfs_blocksetup
 *
 * Description:
 *   Configure block size and the number of blocks for next transfer.
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO device interface.
 *   blocksize - The selected block size.
 *   nblocks   - The number of blocks to transfer.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_blocksetup(struct sdio_dev_s *dev, unsigned int blocksize,
                            unsigned int nblocks)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;

  mcinfo("nblocks: %u, blksize: %u\n", nblocks, blocksize);

  priv->multiblock = (nblocks > 1);
  priv->blocksize  = blocksize;

  /* Block Count (size) */

  putreg16(nblocks, MPFS_COREMMC_BCR);

  /* Block Length */

  putreg16(blocksize, MPFS_COREMMC_BLR);
}

/****************************************************************************
 * Name: mpfs_recvsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card in non-DMA
 *   (interrupt driven mode).  This method will do whatever controller setup
 *   is necessary.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - Address of the buffer in which to receive the data
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int mpfs_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                          size_t nbytes)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;

  mcinfo("Receive: %zu bytes\n", nbytes);

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uintptr_t)buffer & 3) == 0);
  if (nbytes > priv->fifo_depth)
    {
      mcerr("Request doesn't fit the fifo!\n");
    }

  DEBUGASSERT(priv->fifo_depth >= nbytes);

  priv->buffer       = (uint32_t *)buffer;
  priv->remaining    = nbytes;
  priv->receivecnt   = nbytes;
  priv->polltransfer = true;

  mpfs_check_lines_busy(priv);

  /* Set up the SDIO data path, reset DAT and CMD lines */

  mpfs_reset_lines(priv);

  if (priv->multiblock)
    {
      mpfs_putreg8(priv, COREMMC_MBICR_RDONE | COREMMC_MBICR_ERROR,
                   MPFS_COREMMC_MBICR);
    }
  else
    {
      mpfs_putreg8(priv, COREMMC_SBICR_RDONE | COREMMC_SBICR_ERROR,
                   MPFS_COREMMC_SBICR);
    }

  mpfs_putreg8(priv, COREMMC_ALL_ICR, MPFS_COREMMC_ICR);

  /* Enable interrupts and issue the start of operation */

  if (priv->multiblock)
    {
      mpfs_configxfrints(priv, COREMMC_RECV_MASK, COREMMC_RECV_MB_MASK);
      mpfs_modifyreg8(MPFS_COREMMC_MBCSR, 0, COREMMC_MBCSR_RSTRT);
    }
  else
    {
      mpfs_configxfrints(priv, COREMMC_RECV_MASK, COREMMC_RECV_SB_MASK);
      mpfs_modifyreg8(MPFS_COREMMC_SBCSR, 0, COREMMC_SBCSR_RSTRT);
    }

  return OK;
}

/****************************************************************************
 * Name: mpfs_sendsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card.  This
 *   method will do whatever controller setup is necessary.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - Address of the buffer containing the data to send
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int mpfs_sendsetup(struct sdio_dev_s *dev, const
                          uint8_t *buffer, size_t nbytes)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;

  mcinfo("Send: %zu bytes\n", nbytes);

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  if (nbytes > priv->fifo_depth)
    {
      mcerr("Request doesn't fit the fifo!\n");
    }

  DEBUGASSERT(priv->fifo_depth >= nbytes);

  mpfs_reset_lines(priv);
  mpfs_check_lines_busy(priv);

  /* Unaligned (not 32-bit aligned) writes seem to be possible.  Copy data to
   * an aligned buffer in this case.
   */

  if ((uintptr_t)buffer & 3)
    {
      if (nbytes <= sizeof(g_aligned_buffer))
        {
          memcpy((uint8_t *)g_aligned_buffer, buffer, nbytes);
          priv->buffer = g_aligned_buffer;
        }
      else
        {
          DEBUGPANIC();
        }
    }
  else
    {
      priv->buffer = (uint32_t *)buffer;
    }

  priv->remaining    = nbytes;
  priv->receivecnt   = 0;
  priv->polltransfer = true;

  /* Clear interrupt status bits via interrupt clear registers */

  if (priv->multiblock)
    {
      mpfs_putreg8(priv, COREMMC_MBICR_WDONE | COREMMC_MBICR_ERROR,
                   MPFS_COREMMC_MBICR);
    }
  else
    {
      mpfs_putreg8(priv, COREMMC_SBICR_WDONE | COREMMC_SBICR_ERROR,
                   MPFS_COREMMC_SBICR);
    }

  mpfs_putreg8(priv, COREMMC_ALL_ICR, MPFS_COREMMC_ICR);

  /* Enable required interrupts */

  if (priv->multiblock)
    {
      mpfs_configxfrints(priv, COREMMC_SEND_MASK, COREMMC_SEND_MB_MASK);
      mpfs_modifyreg8(MPFS_COREMMC_MBCSR, 0, COREMMC_MBCSR_WSTRT);
    }
  else
    {
      mpfs_configxfrints(priv, COREMMC_SEND_MASK, COREMMC_SEND_SB_MASK);
      mpfs_modifyreg8(MPFS_COREMMC_SBCSR, 0, COREMMC_SBCSR_WSTRT);
    }

  /* Write data into the fifo */

  mpfs_sendfifo(priv);

  return OK;
}

/****************************************************************************
 * Name: mpfs_cancel
 *
 * Description:
 *   Cancel the data transfer setup of various send / receive attempts. This
 *   must be called to cancel the data transfer setup if, for some reason,
 *   you cannot perform the transfer.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int mpfs_cancel(struct sdio_dev_s *dev)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;

  /* Disable all transfer- and event- related interrupts */

  mpfs_configxfrints(priv, 0, 0);
  mpfs_configwaitints(priv, 0, 0, 0);

  /* Clearing pending interrupt status on all transfer- and event- related
   * interrupts
   */

  mpfs_putreg8(priv, COREMMC_ALL_ICR, MPFS_COREMMC_ICR);
  mpfs_putreg8(priv, MPFS_COREMMC_SR_EBOD | MPFS_COREMMC_SR_EBUD |
               MPFS_COREMMC_SR_ECRD, MPFS_COREMMC_SR);

  if (priv->multiblock)
    {
      mpfs_putreg8(priv, COREMMC_MBICR_ERROR | COREMMC_MBICR_RDONE |
                   COREMMC_MBICR_WDONE, MPFS_COREMMC_MBICR);
    }
  else
    {
      mpfs_putreg8(priv, COREMMC_SBICR_ERROR | COREMMC_SBICR_RDONE |
                   COREMMC_SBICR_WDONE, MPFS_COREMMC_SBICR);
    }

  /* Cancel any watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Mark no transfer in progress */

  priv->remaining = 0;
  return OK;
}

/****************************************************************************
 * Name: mpfs_waitresponse
 *
 * Description:
 *   Poll-wait for the response to the last command to be ready.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command that was sent.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int mpfs_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  uint8_t status;
  int32_t timeout;

  mcinfo("cmd: %08" PRIx32 "\n", cmd);

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
      case MMCSD_NO_RESPONSE:
        timeout = COREMMC_CMDTIMEOUT;
        break;

      case MMCSD_R1_RESPONSE:
      case MMCSD_R1B_RESPONSE:
      case MMCSD_R2_RESPONSE:
      case MMCSD_R4_RESPONSE:
      case MMCSD_R5_RESPONSE:
      case MMCSD_R6_RESPONSE:
        timeout = COREMMC_LONGTIMEOUT;
        break;

      case MMCSD_R3_RESPONSE:
      case MMCSD_R7_RESPONSE:
        timeout = COREMMC_CMDTIMEOUT;
        break;

      default:
        mcerr("Unknown command\n");
        return -EINVAL;
    }

  /* Wait for the response (or timeout) */

  do
    {
      status = getreg8(MPFS_COREMMC_ISR);
    }
  while (!(status & (COREMMC_ISR_RRI | COREMMC_ISR_ERROR))
         && --timeout);

  if (timeout == 0 || (status & COREMMC_ISR_ERROR))
    {
      mcerr("ERROR: Timeout cmd: %08" PRIx32 " stat: %02" PRIx8 "\n", cmd,
            status);
      return -EBUSY;
    }

  return OK;
}

/****************************************************************************
 * Name: mpfs_check_recverror
 *
 * Description:
 *   Receive response to SDIO command.
 *
 * Input Parameters:
 *   priv    - Instance of the CoreMMC private state structure.
 *
 * Returned Value:
 *   OK on success; a negated errno if error detected.
 *
 ****************************************************************************/

static int mpfs_check_recverror(struct mpfs_dev_s *priv)
{
  uint32_t timeout = COREMMC_CMDTIMEOUT;
  uint8_t regval;
  int ret = OK;

  regval = getreg8(MPFS_COREMMC_ISR);

  if (regval & COREMMC_ISR_ERROR)
    {
      if (regval & COREMMC_ISR_SBI)
        {
          mcerr("ERROR: Command timeout: %02" PRIx8 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & COREMMC_ISR_TBI)
        {
          mcerr("ERROR: Stop bit error: %02" PRIx8 "\n", regval);
          ret = -EIO;
        }
      else
        {
          mcerr("ERROR: %02" PRIx8 "\n", regval);
          ret = -EIO;
        }
    }
  else if (!(regval & COREMMC_ISR_RRI))
    {
      mcerr("ERROR: Command not completed: %02" PRIx8 "\n", regval);
      ret = -EIO;
    }

  /* Make sure response data ready is set (RDRE) */

  do
    {
      regval = getreg8(MPFS_COREMMC_SR);
    }
  while (!(regval & (MPFS_COREMMC_SR_RDRE))
         && --timeout);

  if (timeout == 0)
    {
      mcerr("ERROR: Response not ready: %02" PRIx8 "\n", regval);
      ret = -ETIMEDOUT;
    }

  if (ret)
    {
      /* With an error, reset DAT and CMD lines. Otherwise the next command
       * will fail as well.
       */

      mpfs_reset_lines(priv);

      /* Clear all status interrupts */

      mpfs_putreg8(priv, COREMMC_ALL_ICR, MPFS_COREMMC_ICR);
    }

  return ret;
}

/****************************************************************************
 * Name: mpfs_recvshortcrc
 *
 * Description:
 *   Receive response to SDIO command.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   cmd    - Command
 *   rshort - Buffer for reveiving the response
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 ****************************************************************************/

static int mpfs_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t *rshort)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  uint32_t response;
  int ret = OK;

  /* Check if a timeout or CRC error occurred */

  if (!(cmd & MMCSD_DATAXFR_MASK))
    {
      ret = mpfs_check_recverror(priv);
    }

  if (rshort)
    {
      response  = getreg8(MPFS_COREMMC_RR1) << 24;
      response |= getreg8(MPFS_COREMMC_RR2) << 16;
      response |= getreg8(MPFS_COREMMC_RR3) << 8;
      response |= getreg8(MPFS_COREMMC_RR4) << 0;
      *rshort = response;

      mcinfo("recv: %08" PRIx32 "\n", *rshort);
    }

  return ret;
}

/****************************************************************************
 * Name: mpfs_recvshort
 *
 * Description:
 *   Receive response to SDIO command.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   cmd    - Command
 *   rshort - Buffer for reveiving the response
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 ****************************************************************************/

static int mpfs_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                          uint32_t *rshort)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  uint32_t response;
  int ret = OK;

  /* Check if a timeout or CRC error occurred */

  if (!(cmd & MMCSD_DATAXFR_MASK))
    {
      ret = mpfs_check_recverror(priv);
    }

  if (rshort)
    {
      response  = getreg8(MPFS_COREMMC_RR1) << 24;
      response |= getreg8(MPFS_COREMMC_RR2) << 16;
      response |= getreg8(MPFS_COREMMC_RR3) << 8;
      response |= getreg8(MPFS_COREMMC_RR4) << 0;

      *rshort = response;
      mcinfo("recv: %08" PRIx32 "\n", *rshort);
    }

  return ret;
}

/****************************************************************************
 * Name: mpfs_recvlong
 *
 * Description:
 *   Receive response to SDIO command.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   cmd    - Command
 *   rlong  - Buffer for reveiving the response
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 ****************************************************************************/

static int mpfs_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                         uint32_t rlong[4])
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  int ret;

  ret = mpfs_check_recverror(priv);

  /* Return the long response */

  if (rlong)
    {
      rlong[0] = getreg8(MPFS_COREMMC_RR0) << 24 |
                 getreg8(MPFS_COREMMC_RR1) << 16 |
                 getreg8(MPFS_COREMMC_RR2) << 8  |
                 getreg8(MPFS_COREMMC_RR3);
      rlong[1] = getreg8(MPFS_COREMMC_RR4) << 24 |
                 getreg8(MPFS_COREMMC_RR5) << 16 |
                 getreg8(MPFS_COREMMC_RR6) << 8  |
                 getreg8(MPFS_COREMMC_RR7);
      rlong[2] = getreg8(MPFS_COREMMC_RR8)  << 24 |
                 getreg8(MPFS_COREMMC_RR9) << 16  |
                 getreg8(MPFS_COREMMC_RR10) << 8  |
                 getreg8(MPFS_COREMMC_RR11);
      rlong[3] = getreg8(MPFS_COREMMC_RR12) << 24 |
                 getreg8(MPFS_COREMMC_RR13) << 16 |
                 getreg8(MPFS_COREMMC_RR14) << 8  |
                 getreg8(MPFS_COREMMC_RR15);

      mcinfo("recv: %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08"
             PRIx32"\n", rlong[0], rlong[1], rlong[2], rlong[3]);
    }

  return ret;
}

/****************************************************************************
 * Name: mpfs_waitenable
 *
 * Description:
 *   Enable / disable of a set of SDIO wait events.  This is part of the
 *   the EMMCSD_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling mpfs_eventwait.
 *
 *   The enabled events persist until either (1) EMMCSD_WAITENABLE is called
 *   again specifying a different set of wait events, or (2) EMMCSD_EVENTWAIT
 *   returns.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOWAIT_*
 *              definitions). 0=disable; 1=enable.
 *   timeout  - SDIO timeout
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_waitenable(struct sdio_dev_s *dev, sdio_eventset_t eventset,
                            uint32_t timeout)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  uint32_t waitmask = 0;

  mcinfo("eventset: %02" PRIx8 "\n", eventset);

  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  mpfs_configwaitints(priv, 0, 0, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

  if ((eventset & SDIOWAIT_CMDDONE) != 0)
    {
      waitmask |= COREMMC_ISR_CSI;
    }

  if ((eventset & SDIOWAIT_RESPONSEDONE) != 0)
    {
      waitmask |= COREMMC_ISR_RRI;
    }

  /* Enable event-related interrupts */

  mpfs_configwaitints(priv, waitmask, eventset, 0);

  /* Check if the timeout event is specified in the event set */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      int delay;
      int ret;

      /* Yes.. Handle a cornercase: The user request a timeout event but
       * with timeout == 0?
       */

      if (!timeout)
        {
          priv->wkupevent = SDIOWAIT_TIMEOUT;
          return;
        }

      /* Start the watchdog timer */

      delay = MSEC2TICK(timeout);
      ret   = wd_start(&priv->waitwdog, delay,
                       mpfs_eventtimeout, (wdparm_t)priv);
      if (ret < OK)
        {
          mcerr("ERROR: wd_start failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: mpfs_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by EMMCSD_WAITEVENTS are disabled when mpfs_eventwait
 *   returns.  EMMCSD_WAITEVENTS must be called again before mpfs_eventwait
 *   can be used again.
 *
 * Input Parameters:
 *   dev     - An instance of the SDIO device interface
 *   timeout - Maximum time in milliseconds to wait.  Zero means immediate
 *             timeout with no wait.  The timeout value is ignored if
 *             SDIOWAIT_TIMEOUT is not included in the waited-for eventset.
 *
 * Returned Value:
 *   Event set containing the event(s) that ended the wait.  Should always
 *   be non-zero.  All events are disabled after the wait concludes.
 *
 ****************************************************************************/

static sdio_eventset_t mpfs_eventwait(struct sdio_dev_s *dev)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  sdio_eventset_t wkupevent = 0;
  int ret;

  mcinfo("wait\n");

  DEBUGASSERT(priv->waitevents != 0 || priv->wkupevent != 0);

  /* Loop until the event (or the timeout occurs). Race conditions are
   * avoided by calling mpfs_waitenable prior to triggering the logic that
   * will cause the wait to terminate.  Under certain race conditions, the
   * waited-for may have already occurred before this function was called!
   */

  for (; ; )
    {
      /* Wait for an event in event set to occur.  If this the event has
       * already occurred, then the semaphore will already have been
       * incremented and there will be no wait.
       */

      ret = nxsem_wait_uninterruptible(&priv->waitsem);
      if (ret < 0)
        {
          /* Task canceled.  Cancel the wdog (assuming it was started) and
           * return an SDIO error.
           */

          wd_cancel(&priv->waitwdog);
          wkupevent = SDIOWAIT_ERROR;
          goto errout_with_waitints;
        }

      wkupevent = priv->wkupevent;

      /* Check if the event has occurred.  When the event has occurred, then
       * evenset will be set to 0 and wkupevent will be set to a nonzero
       * value.
       */

      if (wkupevent != 0)
        {
          /* Yes... break out of the loop with wkupevent non-zero */

          break;
        }
    }

  /* Disable event-related interrupts */

errout_with_waitints:
  mpfs_configwaitints(priv, 0, 0, 0);

  return wkupevent;
}

/****************************************************************************
 * Name: mpfs_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events. This is part of the
 *   the SDIO callback sequence. The set of events is configured to enabled
 *   callbacks to the function provided in mpfs_registercallback.
 *
 *   Events are automatically disabled once the callback is performed and no
 *   further callback events will occur until they are again enabled by
 *   calling this method.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOMEDIA_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_callbackenable(struct sdio_dev_s *dev,
                                sdio_eventset_t eventset)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;

  mcinfo("eventset: %02" PRIx8 "\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  mpfs_callback(priv);
}

/****************************************************************************
 * Name: mpfs_registercallback
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change. When this method is called, all callbacks should be disabled
 *   until they are enabled via a call to EMMCSD_CALLBACKENABLE
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   callback - The function to call on the media change
 *   arg      - A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

static int mpfs_registercallback(struct sdio_dev_s *dev,
                                 worker_t callback, void *arg)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;

  /* Disable callbacks and register this callback and is argument */

  mcinfo("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;

  return OK;
}

/****************************************************************************
 * Name: mpfs_callback
 *
 * Description:
 *   Perform callback.
 *
 * Assumptions:
 *   This function does not execute in the context of an interrupt handler.
 *   It may be invoked on any user thread or scheduled on the work thread
 *   from an interrupt handler.
 *
 ****************************************************************************/

static void mpfs_callback(void *arg)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)arg;

  /* Is a callback registered? */

  DEBUGASSERT(priv != NULL);

  mcinfo("Callback %p(%p) cbevents: %02" PRIx8 " cdstatus: %02" PRIx8 "\n",
         priv->callback, priv->cbarg, priv->cbevents, priv->cdstatus);

  if (priv->callback)
    {
      /* Yes.. Check for enabled callback events */

      if ((priv->cdstatus & SDIO_STATUS_PRESENT) != 0)
        {
          /* Media is present.  Is the media inserted event enabled? */

          if ((priv->cbevents & SDIOMEDIA_INSERTED) == 0)
            {
              /* No... return without performing the callback */

              return;
            }
        }
      else
        {
          /* Media is not present.  Is the media eject event enabled? */

          if ((priv->cbevents & SDIOMEDIA_EJECTED) == 0)
            {
              /* No... return without performing the callback */

              return;
            }
        }

      /* Perform the callback, disabling further callbacks.  Of course, the
       * the callback can (and probably should) re-enable callbacks.
       */

      priv->cbevents = 0;

      /* Callbacks cannot be performed in the context of an interrupt
       * handler.  If we are in an interrupt handler, then queue the
       * callback to be performed later on the work thread.
       */

      if (up_interrupt_context())
        {
          /* Yes.. queue it */

          mcinfo("Queuing callback to %p(%p)\n",
                 priv->callback, priv->cbarg);

          work_queue(HPWORK, &priv->cbwork, priv->callback,
                     priv->cbarg, 0);
        }
      else
        {
          /* No.. then just call the callback here */

          mcinfo("Callback to %p(%p)\n", priv->callback, priv->cbarg);
          priv->callback(priv->cbarg);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_coremmc_sdio_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Values:
 *   A reference to an SDIO interface structure.  NULL is returned on
 *   failures.
 *
 ****************************************************************************/

struct sdio_dev_s *mpfs_coremmc_sdio_initialize(int slotno)
{
  struct mpfs_dev_s *priv = &g_coremmc_dev;

  /* Reset the card and assure that it is in the initial, unconfigured
   * state.
   */

  if (!mpfs_device_reset(&priv->dev))
    {
      return NULL;
    }

  return &priv->dev;
}

/****************************************************************************
 * Name: mpfs_coremmc_sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic -- possible from an interrupt handler --
 *   in order to signal to the driver that a card has been inserted or
 *   removed from the slot
 *
 * Input Parameters:
 *   dev        - An instance of the SDIO driver device state structure.
 *   cardinslot - true is a card has been detected in the slot; false if a
 *                card has been removed from the slot.  Only transitions
 *                (inserted->removed or removed->inserted should be reported)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpfs_coremmc_sdio_mediachange(struct sdio_dev_s *dev, bool cardinslot)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  sdio_statset_t cdstatus;
  irqstate_t flags;

  /* Update card status */

  flags = enter_critical_section();
  cdstatus = priv->cdstatus;
  if (cardinslot)
    {
      priv->cdstatus |= SDIO_STATUS_PRESENT;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_PRESENT;
    }

  leave_critical_section(flags);

  mcinfo("cdstatus OLD: %02" PRIx8 " NEW: %02" PRIx8 "\n",
         cdstatus, priv->cdstatus);

  /* Perform any requested callback if the status has changed */

  if (cdstatus != priv->cdstatus)
    {
      mpfs_callback(priv);
    }
}

/****************************************************************************
 * Name: mpfs_coremmc_sdio_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report if the card in the slot is
 *   mechanically write protected.
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO driver device state structure.
 *   wrprotect - true is a card is writeprotected.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpfs_coremmc_sdio_wrprotect(struct sdio_dev_s *dev, bool wrprotect)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  irqstate_t flags;

  /* Update card status */

  flags = enter_critical_section();
  if (wrprotect)
    {
      priv->cdstatus |= SDIO_STATUS_WRPROTECTED;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_WRPROTECTED;
    }

  mcinfo("cdstatus: %02" PRIx8 "\n", priv->cdstatus);

  leave_critical_section(flags);
}
