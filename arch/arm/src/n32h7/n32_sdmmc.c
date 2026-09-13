/****************************************************************************
 * arch/arm/src/n32h7/n32_sdmmc.c
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

#include <inttypes.h>
#include <nuttx/sched.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/compiler.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/mmcsd.h>
#include <nuttx/irq.h>
#include <nuttx/cache.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "hardware/n32h7_sdmmc.h"
#include "n32_gpio.h"
#include "n32_sdmmc.h"
#include "hardware/n32h76x_pwr.h"
#include "hardware/n32h76x_rcc.h"

#if defined(CONFIG_N32H7_SDMMC1) || defined(CONFIG_N32H7_SDMMC2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Workqueue support required */

#if !defined(CONFIG_SCHED_WORKQUEUE) || !defined(CONFIG_SCHED_HPWORK)
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE and CONFIG_SCHED_HPWORK"
#endif

/* DMA support (ADMA2) is optional but recommended */

#ifdef CONFIG_N32H7_SDMMC_ADMA
#  define HAVE_ADMA2 1
#else
#  define HAVE_ADMA2 0
#endif

/* FIFO Size (4KB Block Buffer) */

#define N32_SDMMC_FIFO_SIZE         (4096)

/* Max Block Size (512 bytes for SD, 2048 for eMMC) */

#define N32_SDMMC_MAX_BLOCK_SIZE    (512)

/* Default Data Timeout (250 ms) */

#define N32_SDMMC_DATATIMEOUT_MS    (250)

/* Command Timeout Loops */

#define N32_SDMMC_CMDTIMEOUT        (1000000)

/* Event wait masks */

#define N32_WAIT_CMDDONE            (N32_SDHOST_INTSTS_CMDC)
#define N32_WAIT_RESPDONE           (N32_SDHOST_INTSTS_CMDC | N32_SDHOST_INTSTS_CTERR | N32_SDHOST_INTSTS_CCRCERR)
#define N32_WAIT_XFRDONE            (N32_SDHOST_INTSTS_TC | N32_SDHOST_INTSTS_DTERR | N32_SDHOST_INTSTS_DCRERR | N32_SDHOST_INTSTS_ADMAERR)

#define N32_WAIT_ALL                (N32_SDHOST_INTSTS_CMDC | N32_SDHOST_INTSTS_TC | N32_SDHOST_INTSTS_BLKGAPE | N32_SDHOST_INTSTS_DMAINT |\
                                     N32_SDHOST_INTSTS_BUFWRDY | N32_SDHOST_INTSTS_BUFRRDY | N32_SDHOST_INTSTS_CINS | N32_SDHOST_INTSTS_CRMV |\
                                     N32_SDHOST_INTSTS_CINT | N32_SDHOST_INTSTS_RETUNE | N32_SDHOST_INTSTS_BOOTACKR | N32_SDHOST_INTSTS_BOOTTER |\
                                     N32_SDHOST_INTSTS_CTERR | N32_SDHOST_INTSTS_CCRCERR | N32_SDHOST_INTSTS_CENDBERR | N32_SDHOST_INTSTS_CINXERR |\
                                     N32_SDHOST_INTSTS_DTERR | N32_SDHOST_INTSTS_DCRERR | N32_SDHOST_INTSTS_DENDERR | N32_SDHOST_INTSTS_ACMDERR |\
                                     N32_SDHOST_INTSTS_ADMAERR | N32_SDHOST_INTSTS_TRGRERR)

/* Helper: Get CFG and HOST base addresses */

#define N32_CFG_BASE(priv)           ((priv)->cfg_base)
#define N32_HOST_BASE(priv)          ((priv)->host_base)
#define N32_PUT_CFG(priv, off, val)  putreg32((val), (priv)->cfg_base + (off))
#define N32_GET_CFG(priv, off)       getreg32((priv)->cfg_base + (off))
#define N32_PUT_HOST(priv, off, val) putreg32((val), (priv)->host_base + (off))
#define N32_GET_HOST(priv, off)      getreg32((priv)->host_base + (off))

/* Clock calculation: SDCLK = BCLKF / (2 * SDCLKSEL), BCLKF in MHz */

#define N32_BCLKF_FREQ              (N32_PLL1B_FREQUENCY / 1000000) /* 100 MHz */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ADMA2 Descriptor List (for scatter-gather) */

struct n32_sdmmc_adma_list_s
{
  struct n32_sdmmc_adma_desc_s *desc;   /* Pointer to descriptor array */
  int                           count;  /* Number of descriptors */
  uint32_t                      total_len;
};

/* N32 SDMMC Device State */

struct n32_sdmmc_s
{
  struct sdio_dev_s  dev;             /* Standard SDIO interface */

  uint32_t           cfg_base;        /* SDMMC_CFG base address */
  uint32_t           host_base;       /* SDHOST base address */
  int                irq;             /* Interrupt number */

  /* Clock */

  uint32_t           clock;           /* Current SDCLK frequency in Hz */

  /* Event support */

  sem_t              waitsem;         /* Event wait semaphore */
  sdio_eventset_t    waitevents;      /* Events to wait for */
  uint32_t           waitmask;        /* Interrupt mask for waiting */
  volatile sdio_eventset_t wkupevent; /* Wakeup event */
  struct wdog_s      waitwdog;        /* Timeout watchdog */

  /* Callback support */

  sdio_statset_t     cdstatus;        /* Card status */
  sdio_eventset_t    cbevents;        /* Callback events */
  worker_t           callback;        /* Callback function */
  void              *cbarg;           /* Callback argument */
  struct work_s      cbwork;          /* Callback work */

  /* Transfer state (PIO / DMA) */

  uint32_t          *buffer;          /* Current buffer pointer */
  size_t             remaining;       /* Bytes remaining */
  uint32_t           blocksize;       /* Current block size */
  uint32_t           nblocks;         /* Number of blocks */
  bool               is_read;         /* true=read, false=write */
  bool               using_dma;       /* true if using ADMA */

  /* ADMA2 state */

#if HAVE_ADMA2
  struct n32_sdmmc_adma_list_s adma_list;
  bool                         adma_in_progress;
#endif

  /* Misc */

  bool               widebus;         /* 4-bit bus mode */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register access helpers */

static inline void n32_sdmmc_putreg(struct n32_sdmmc_s *priv,
                                    uint32_t value, int offset);
static inline uint32_t n32_sdmmc_getreg(struct n32_sdmmc_s *priv,
                                        int offset);
static inline void n32_sdmmc_modifyreg(struct n32_sdmmc_s *priv,
                                       int offset, uint32_t clr,
                                       uint32_t set);

/* Interrupt control */

static void n32_sdmmc_configints(struct n32_sdmmc_s *priv);
static void n32_sdmmc_configsts(struct n32_sdmmc_s *priv, uint32_t mask);

/* Clock / Power */

static int n32_sdmmc_enable_clock(struct n32_sdmmc_s *priv,
                                  uint32_t freq_hz);
static void n32_sdmmc_power_on(struct n32_sdmmc_s *priv);
static void n32_sdmmc_power_off(struct n32_sdmmc_s *priv);

/* Command & Response */

static int n32_sdmmc_send_cmd(struct n32_sdmmc_s *priv,
                              uint32_t cmd, uint32_t arg);
static int n32_sdmmc_wait_cmd_done(struct n32_sdmmc_s *priv,
                                   uint32_t cmd);
static void n32_sdmmc_get_response(struct n32_sdmmc_s *priv,
                                   uint32_t cmd, uint32_t *resp);

/* Data transfer (PIO) */

static void n32_sdmmc_setup_data(struct n32_sdmmc_s *priv);
static void n32_sdmmc_send_fifo(struct n32_sdmmc_s *priv);
static void n32_sdmmc_recv_fifo(struct n32_sdmmc_s *priv);
static void n32_sdmmc_datadisable(struct n32_sdmmc_s *priv);

/* ADMA2 */

#if HAVE_ADMA2
static int n32_sdmmc_build_adma_desc(struct n32_sdmmc_s *priv,
                                     uint8_t *buffer, size_t len);
static void n32_sdmmc_start_adma(struct n32_sdmmc_s *priv);
static void n32_sdmmc_stop_adma(struct n32_sdmmc_s *priv);
#endif

/* Event handling */

static void n32_sdmmc_endwait(struct n32_sdmmc_s *priv,
                              sdio_eventset_t wkupevent);
static void n32_sdmmc_eventtimeout(wdparm_t arg);

/* Interrupt handler */

static int n32_sdmmc_interrupt(int irq, void *context, void *arg);

/* SDIO interface methods */

#ifdef CONFIG_SDIO_MUXBUS
static int n32_sdmmc_lock(struct sdio_dev_s *dev, bool lock);
#endif /* CONFIG_SDIO_MUXBUS */
static void n32_sdmmc_reset(struct sdio_dev_s *dev);
static sdio_capset_t n32_sdmmc_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t n32_sdmmc_status(struct sdio_dev_s *dev);
static void n32_sdmmc_widebus(struct sdio_dev_s *dev, bool enable);
static void n32_sdmmc_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate);
static int n32_sdmmc_attach(struct sdio_dev_s *dev);
static int n32_sdmmc_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t arg);
static void n32_sdmmc_blocksetup(struct sdio_dev_s *dev,
                                 unsigned int blocksize,
                                 unsigned int nblocks);
static int n32_sdmmc_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                               size_t nbytes);
static int n32_sdmmc_sendsetup(struct sdio_dev_s *dev,
                               const uint8_t *buffer, size_t nbytes);
static int n32_sdmmc_cancel(struct sdio_dev_s *dev);
static int n32_sdmmc_waitresponse(struct sdio_dev_s *dev, uint32_t cmd);
static int n32_sdmmc_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                                  uint32_t *rshort);
static int n32_sdmmc_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                              uint32_t rlong[4]);
static int n32_sdmmc_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                               uint32_t *rshort);
static void n32_sdmmc_waitenable(struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset,
                                 uint32_t timeout);
static sdio_eventset_t n32_sdmmc_eventwait(struct sdio_dev_s *dev);
static void n32_sdmmc_callbackenable(struct sdio_dev_s *dev,
                                     sdio_eventset_t eventset);
static int n32_sdmmc_registercallback(struct sdio_dev_s *dev,
                                      worker_t callback, void *arg);
#ifdef CONFIG_SDIO_DMA
#  if HAVE_ADMA2
static int n32_sdmmc_dmarecvsetup(struct sdio_dev_s *dev,
                                  uint8_t *buffer, size_t buflen);
static int n32_sdmmc_dmasendsetup(struct sdio_dev_s *dev,
                                  const uint8_t *buffer, size_t buflen);
#  else
#    error "CONFIG_SDIO_DMA requires HAVE_ADMA2"
#  endif /* HAVE_ADMA2 */
#endif /* CONFIG_SDIO_DMA */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SDMMC1 and SDMMC2 device instances */

#if defined(CONFIG_N32H7_SDMMC1)
static struct n32_sdmmc_s g_sdmmc1_dev =
{
  .dev =
  {
#ifdef CONFIG_SDIO_MUXBUS
    .lock             = n32_sdmmc_lock,
#endif /* CONFIG_SDIO_MUXBUS */
    .reset            = n32_sdmmc_reset,
    .capabilities     = n32_sdmmc_capabilities,
    .status           = n32_sdmmc_status,
    .widebus          = n32_sdmmc_widebus,
    .clock            = n32_sdmmc_clock,
    .attach           = n32_sdmmc_attach,
    .sendcmd          = n32_sdmmc_sendcmd,
    .blocksetup       = n32_sdmmc_blocksetup,
    .recvsetup        = n32_sdmmc_recvsetup,
    .sendsetup        = n32_sdmmc_sendsetup,
    .cancel           = n32_sdmmc_cancel,
    .waitresponse     = n32_sdmmc_waitresponse,
    .recv_r1          = n32_sdmmc_recvshortcrc,
    .recv_r2          = n32_sdmmc_recvlong,
    .recv_r3          = n32_sdmmc_recvshort,
    .recv_r4          = n32_sdmmc_recvshort,
    .recv_r5          = n32_sdmmc_recvshortcrc,
    .recv_r6          = n32_sdmmc_recvshortcrc,
    .recv_r7          = n32_sdmmc_recvshort,
    .waitenable       = n32_sdmmc_waitenable,
    .eventwait        = n32_sdmmc_eventwait,
    .callbackenable   = n32_sdmmc_callbackenable,
    .registercallback = n32_sdmmc_registercallback,
#ifdef CONFIG_SDIO_DMA
#  if HAVE_ADMA2
    .dmarecvsetup     = n32_sdmmc_dmarecvsetup,
    .dmasendsetup     = n32_sdmmc_dmasendsetup,
#  else
#    error "CONFIG_SDIO_DMA requires HAVE_ADMA2"
#  endif /* HAVE_ADMA2 */
#endif /* CONFIG_SDIO_DMA */
  },
  .cfg_base = N32_SDMMC1_CFG_BASE,
  .host_base = N32_SDHOST1_BASE,
  .irq = N32_IRQ_SDMMC1,
  .waitsem = SEM_INITIALIZER(0),
};
#endif

#if defined(CONFIG_N32H7_SDMMC2)
static struct n32_sdmmc_s g_sdmmc2_dev =
{
  .dev =
  {
#ifdef CONFIG_SDIO_MUXBUS
    .lock             = n32_sdmmc_lock,
#endif /* CONFIG_SDIO_MUXBUS */
    .reset            = n32_sdmmc_reset,
    .capabilities     = n32_sdmmc_capabilities,
    .status           = n32_sdmmc_status,
    .widebus          = n32_sdmmc_widebus,
    .clock            = n32_sdmmc_clock,
    .attach           = n32_sdmmc_attach,
    .sendcmd          = n32_sdmmc_sendcmd,
    .blocksetup       = n32_sdmmc_blocksetup,
    .recvsetup        = n32_sdmmc_recvsetup,
    .sendsetup        = n32_sdmmc_sendsetup,
    .cancel           = n32_sdmmc_cancel,
    .waitresponse     = n32_sdmmc_waitresponse,
    .recv_r1          = n32_sdmmc_recvshortcrc,
    .recv_r2          = n32_sdmmc_recvlong,
    .recv_r3          = n32_sdmmc_recvshort,
    .recv_r4          = n32_sdmmc_recvshort,
    .recv_r5          = n32_sdmmc_recvshortcrc,
    .recv_r6          = n32_sdmmc_recvshortcrc,
    .recv_r7          = n32_sdmmc_recvshort,
    .waitenable       = n32_sdmmc_waitenable,
    .eventwait        = n32_sdmmc_eventwait,
    .callbackenable   = n32_sdmmc_callbackenable,
    .registercallback = n32_sdmmc_registercallback,
#ifdef CONFIG_SDIO_DMA
#  if HAVE_ADMA2
    .dmarecvsetup     = n32_sdmmc_dmarecvsetup,
    .dmasendsetup     = n32_sdmmc_dmasendsetup,
#  else
#    error "CONFIG_SDIO_DMA requires HAVE_ADMA2"
#  endif /* HAVE_ADMA2 */
#endif /* CONFIG_SDIO_DMA */
  },
  .cfg_base = N32_SDMMC2_CFG_BASE,
  .host_base = N32_SDHOST2_BASE,
  .irq = N32_IRQ_SDMMC2,
  .waitsem = SEM_INITIALIZER(0),
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* ==========================================================================
 * Register Access Helpers
 * ==========================================================================
 */

/****************************************************************************
 * Name: n32_sdmmc_putreg
 *
 * Description:
 *   Write a 32-bit value to a SDHOST register.
 *
 * Input Parameters:
 *   priv   - SDMMC device instance
 *   value  - Value to write
 *   offset - Register offset from SDHOST base
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void n32_sdmmc_putreg(struct n32_sdmmc_s *priv,
                                    uint32_t value, int offset)
{
  putreg32(value, priv->host_base + offset);
}

/****************************************************************************
 * Name: n32_sdmmc_getreg
 *
 * Description:
 *   Read a 32-bit value from a SDHOST register.
 *
 * Input Parameters:
 *   priv   - SDMMC device instance
 *   offset - Register offset from SDHOST base
 *
 * Returned Value:
 *   Register value
 *
 ****************************************************************************/

static inline uint32_t n32_sdmmc_getreg(struct n32_sdmmc_s *priv,
                                        int offset)
{
  return getreg32(priv->host_base + offset);
}

/****************************************************************************
 * Name: n32_sdmmc_modifyreg
 *
 * Description:
 *   Atomically modify a SDHOST register (clear and set bits).
 *
 * Input Parameters:
 *   priv - SDMMC device instance
 *   offset - Register offset
 *   clr - Bits to clear
 *   set - Bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void n32_sdmmc_modifyreg(struct n32_sdmmc_s *priv,
                                       int offset, uint32_t clr,
                                       uint32_t set)
{
  uint32_t reg = n32_sdmmc_getreg(priv, offset);

  reg &= ~clr;
  reg |= set;
  n32_sdmmc_putreg(priv, reg, offset);
}

/* ==========================================================================
 * Interrupt Configuration
 * ==========================================================================
 */

/****************************************************************************
 * Name: n32_sdmmc_configints
 *
 * Description:
 *   Enable/disable SDHOST interrupts and set waiting events.
 *
 * Input Parameters:
 *   priv   - SDMMC device instance
 *   mask   - Interrupt mask to write to ISE
 *   events - Events to be waited for (SDIOWAIT_*)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_configints(struct n32_sdmmc_s *priv)
{
  irqstate_t flags = enter_critical_section();

  /* Enable interrupts at the signal level (ISE) */

  n32_sdmmc_putreg(priv, priv->waitmask, N32_SDHOST_ISE_OFFSET);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: n32_sdmmc_configsts
 *
 * Description:
 *   Enable/disable SDHOST status signal and set waiting events.
 *
 * Input Parameters:
 *   priv   - SDMMC device instance
 *   mask   - Interrupt mask to write to IE
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_configsts(struct n32_sdmmc_s *priv, uint32_t mask)
{
  irqstate_t flags = enter_critical_section();

  /* Enable interrupts at the SDHOST level (IE) */

  n32_sdmmc_putreg(priv, mask, N32_SDHOST_IE_OFFSET);

  leave_critical_section(flags);
}

/* ==========================================================================
 * Clock & Power Control
 * ==========================================================================
 */

/****************************************************************************
 * Name: n32_sdmmc_enable_clock
 *
 * Description:
 *   Enable or disable SD clock with the desired frequency.
 *
 * Input Parameters:
 *   priv    - SDMMC device instance
 *   freq_hz - Desired SD clock frequency in Hz (0 to disable)
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

static int n32_sdmmc_enable_clock(struct n32_sdmmc_s *priv, uint32_t freq_hz)
{
  uint32_t regval;
  uint32_t div;
  int timeout = 10000;

  /* If freq_hz is 0, disable clock */

  if (freq_hz == 0)
    {
      n32_sdmmc_modifyreg(priv, N32_SDHOST_CTRL2_OFFSET,
                          N32_SDHOST_CTRL2_INCLKE |
                              N32_SDHOST_CTRL2_SDCLKE,
                          0);
      priv->clock = 0;
      return OK;
    }

  /* Calculate SDCLKSEL divider: freq = BCLKF / (2 * div) */

  /* BCLKF is in MHz, freq_hz in Hz */

  uint32_t bclk_hz = N32_BCLKF_FREQ * 1000000UL;

  div = bclk_hz / (2 * freq_hz);

  if (div > 0x3ff)
    {
      div = 0x3ff; /* Max divider */
    }

  priv->clock = freq_hz;

  /* 1. Disable clock */

  n32_sdmmc_modifyreg(priv, N32_SDHOST_CTRL2_OFFSET,
                    N32_SDHOST_CTRL2_INCLKE | N32_SDHOST_CTRL2_SDCLKE, 0);

  /* 2. Set divider */

  regval = n32_sdmmc_getreg(priv, N32_SDHOST_CTRL2_OFFSET);
  regval &= ~(N32_SDHOST_CTRL2_SDCLKSEL70_MASK |
              N32_SDHOST_CTRL2_SDCLKSEL98_MASK);
  regval |= N32_SDHOST_CTRL2_SDCLKSEL(div);
  n32_sdmmc_putreg(priv, regval, N32_SDHOST_CTRL2_OFFSET);

  /* 3. Enable clock */

  n32_sdmmc_modifyreg(priv, N32_SDHOST_CTRL2_OFFSET, 0,
                      N32_SDHOST_CTRL2_INCLKE | N32_SDHOST_CTRL2_SDCLKE);

  /* 4. Wait for stable */

  do
    {
      regval = n32_sdmmc_getreg(priv, N32_SDHOST_CTRL2_OFFSET);
      if (regval & N32_SDHOST_CTRL2_INCLKSTS)
        {
          break;
        }

      up_udelay(10);
    }
  while (--timeout > 0);

  if (timeout == 0)
    {
      mcerr("ERROR: Internal clock not stable\n");
      return -ETIMEDOUT;
    }

  mcinfo("SDCLK enabled: freq=%lu Hz, div=%lu\n", (unsigned long)freq_hz,
         (unsigned long)div);
  return OK;
}

/****************************************************************************
 * Name: n32_sdmmc_power_on
 *
 * Description:
 *   Turn on SD bus power (3.3V).
 *
 * Input Parameters:
 *   priv - SDMMC device instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_power_on(struct n32_sdmmc_s *priv)
{
  uint32_t regval;

  /* Set voltage to 3.3V and enable power */

  regval = n32_sdmmc_getreg(priv, N32_SDHOST_CTRL1_OFFSET);
  regval &= ~(N32_SDHOST_CTRL1_SDBVSEL_MASK | N32_SDHOST_CTRL1_SDPWR);
  regval |= N32_SDHOST_CTRL1_SDBVSEL_3V3;
  n32_sdmmc_putreg(priv, regval, N32_SDHOST_CTRL1_OFFSET);

  /* Power up delay */

  nxsched_msleep(10);
  regval |= N32_SDHOST_CTRL1_SDPWR;
  n32_sdmmc_putreg(priv, regval, N32_SDHOST_CTRL1_OFFSET);
}

/****************************************************************************
 * Name: n32_sdmmc_power_off
 *
 * Description:
 *   Turn off SD bus power.
 *
 * Input Parameters:
 *   priv - SDMMC device instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_power_off(struct n32_sdmmc_s *priv)
{
  n32_sdmmc_modifyreg(priv, N32_SDHOST_CTRL1_OFFSET,
                      N32_SDHOST_CTRL1_SDPWR, 0);
}

/* ==========================================================================
 * Command & Response
 * ==========================================================================
 */

/****************************************************************************
 * Name: n32_sdmmc_send_cmd
 *
 * Description:
 *   Format and send an SD command.
 *
 * Input Parameters:
 *   priv - SDMMC device instance
 *   cmd  - Command word (including flags)
 *   arg  - 32-bit argument
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

static int n32_sdmmc_send_cmd(struct n32_sdmmc_s *priv,
                              uint32_t cmd, uint32_t arg)
{
  uint32_t regval = 0;
  uint32_t cmdidx;

  /* Clear previous command status */

  n32_sdmmc_putreg(priv, N32_SDHOST_INTSTS_CMDC | N32_SDHOST_INTSTS_CTERR |
                      N32_SDHOST_INTSTS_CCRCERR | N32_SDHOST_INTSTS_CINXERR |
                      N32_SDHOST_INTSTS_CENDBERR,
                   N32_SDHOST_INTSTS_OFFSET);

  /* Set argument */

  n32_sdmmc_putreg(priv, arg, N32_SDHOST_CMDARG1_OFFSET);

  /* Build TMODE register */

  cmdidx = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval |= (cmdidx << N32_SDHOST_TMODE_CMDINDEX_SHIFT);

  /* Response type */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
      case MMCSD_NO_RESPONSE:
        regval |= N32_SDHOST_TMODE_RTYPES_NONE;
        break;
      case MMCSD_R1_RESPONSE:
      case MMCSD_R5_RESPONSE:
      case MMCSD_R6_RESPONSE:
      case MMCSD_R7_RESPONSE:
        regval |= N32_SDHOST_TMODE_RTYPES_48 | N32_SDHOST_TMODE_CRCCK |
                  N32_SDHOST_TMODE_CMDXCK;
        break;
      case MMCSD_R1B_RESPONSE:
        regval |= N32_SDHOST_TMODE_RTYPES_48_BUSY | N32_SDHOST_TMODE_CRCCK |
                  N32_SDHOST_TMODE_CMDXCK;
        break;
      case MMCSD_R2_RESPONSE:
        regval |= N32_SDHOST_TMODE_RTYPES_136 | N32_SDHOST_TMODE_CRCCK;
        break;
      case MMCSD_R3_RESPONSE:
      case MMCSD_R4_RESPONSE:
        regval |= N32_SDHOST_TMODE_RTYPES_48;
        break;
      default:
        return -EINVAL;
    }

  /* Data transfer */

  if ((cmd & MMCSD_DATAXFR_MASK) != MMCSD_NODATAXFR)
    {
      regval |= N32_SDHOST_TMODE_DPSEL;
      if ((cmd & MMCSD_DATAXFR_MASK) == MMCSD_RDDATAXFR)
        {
          regval |= N32_SDHOST_TMODE_DATDIR; /* Read */
          priv->is_read = true;
        }
      else
        {
          priv->is_read = false; /* Write */
        }

      n32_sdmmc_setup_data(priv);
    }

  /* Multi-block */

  if (priv->nblocks > 1)
    {
      regval |= N32_SDHOST_TMODE_BLKSEL | N32_SDHOST_TMODE_BCNTE;
    }

  /* DMA enable if using ADMA */

  if (priv->using_dma)
    {
      regval |= N32_SDHOST_TMODE_DMAE;
    }

  /* Write TMODE to trigger command */

  n32_sdmmc_putreg(priv, regval, N32_SDHOST_TMODE_OFFSET);

  mcinfo("CMD: 0x%08lx ARG: 0x%08lx TMODE: 0x%08lx\n",
         (unsigned long)cmd, (unsigned long)arg,
         (unsigned long)regval);

  return OK;
}

/****************************************************************************
 * Name: n32_sdmmc_wait_cmd_done
 *
 * Description:
 *   Poll-wait for command completion or error.
 *
 * Input Parameters:
 *   priv - SDMMC device instance
 *   cmd  - Command word (for debugging)
 *
 * Returned Value:
 *   OK on success, -ETIMEDOUT or -EIO on error.
 *
 ****************************************************************************/

static int n32_sdmmc_wait_cmd_done(struct n32_sdmmc_s *priv, uint32_t cmd)
{
  uint32_t regval;
  int32_t timeout = N32_SDMMC_CMDTIMEOUT;

  /* Wait for command complete or error */

  while (--timeout > 0)
    {
      regval = n32_sdmmc_getreg(priv, N32_SDHOST_INTSTS_OFFSET);
      if (regval & N32_SDHOST_INTSTS_CMDC)
        {
          /* Clear it */

          n32_sdmmc_putreg(priv, N32_SDHOST_INTSTS_CMDC,
                           N32_SDHOST_INTSTS_OFFSET);
          return OK;
        }

      if (regval & (N32_SDHOST_INTSTS_CTERR | N32_SDHOST_INTSTS_CCRCERR))
        {
          mcerr("CMD error: Istatus=0x%08lx\n", (unsigned long)regval);

          /* Reset command line */

          regval = n32_sdmmc_getreg(priv, N32_SDHOST_CTRL2_OFFSET);
          regval |= N32_SDHOST_CTRL2_SWRSTCMD;
          n32_sdmmc_putreg(priv, regval, N32_SDHOST_CTRL2_OFFSET);
          return -EIO;
        }

      up_udelay(1);
    }

  mcerr("CMD timeout: status=0x%08lx\n",
        (unsigned long)n32_sdmmc_getreg(priv, N32_SDHOST_INTSTS_OFFSET));
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: n32_sdmmc_get_response
 *
 * Description:
 *   Read command response from response registers.
 *
 * Input Parameters:
 *   priv - SDMMC device instance
 *   cmd  - Command word (to determine response length)
 *   resp - Pointer to store response (if R2, points to 4-word array)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_get_response(struct n32_sdmmc_s *priv,
                                   uint32_t cmd, uint32_t *resp)
{
  int rtype = cmd & MMCSD_RESPONSE_MASK;

  if (rtype == MMCSD_R2_RESPONSE)
    {
      if (resp)
        {
          resp[3] = n32_sdmmc_getreg(priv, N32_SDHOST_CMDRSP0_OFFSET) << 8;
          resp[2] = (n32_sdmmc_getreg(priv, N32_SDHOST_CMDRSP1_OFFSET) << 8)
           | (n32_sdmmc_getreg(priv, N32_SDHOST_CMDRSP0_OFFSET) >> 24);
          resp[1] = (n32_sdmmc_getreg(priv, N32_SDHOST_CMDRSP2_OFFSET) << 8)
           | (n32_sdmmc_getreg(priv, N32_SDHOST_CMDRSP1_OFFSET) >> 24);
          resp[0] = (n32_sdmmc_getreg(priv, N32_SDHOST_CMDRSP3_OFFSET) << 8)
           | (n32_sdmmc_getreg(priv, N32_SDHOST_CMDRSP2_OFFSET) >> 24);
        }
    }
  else
    {
      if (resp)
        {
          *resp = n32_sdmmc_getreg(priv, N32_SDHOST_CMDRSP0_OFFSET);
        }
    }
}

/* ==========================================================================
 * Data Transfer (PIO)
 * ==========================================================================
 */

/****************************************************************************
 * Name: n32_sdmmc_setup_data
 *
 * Description:
 *   Configure BLKCFG for a data transfer.
 *
 * Input Parameters:
 *   priv      - SDMMC device instance
 *   blocksize - Block size in bytes
 *   nblocks   - Number of blocks
 *   read      - true for read, false for write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_setup_data(struct n32_sdmmc_s *priv)
{
  uint32_t regval;
  uint32_t total_len = priv->blocksize * priv->nblocks;

  /* Set BLKCFG */

  regval = ((priv->nblocks & 0xffff) << N32_SDHOST_BLKCFG_CNT_SHIFT) |
           (priv->blocksize & N32_SDHOST_BLKCFG_SIZE_MASK);
  n32_sdmmc_putreg(priv, regval, N32_SDHOST_BLKCFG_OFFSET);

  up_udelay(10);

  mcinfo("DATA: size=%lu, blocks=%lu, total=%lu, dir=%s\n",
         (unsigned long)priv->blocksize, (unsigned long)priv->nblocks,
         (unsigned long)total_len, priv->is_read ? "READ" : "WRITE");
}

/****************************************************************************
 * Name: n32_sdmmc_send_fifo
 *
 * Description:
 *   Write data from buffer to SDHOST FIFO (PIO write).
 *
 * Input Parameters:
 *   priv - SDMMC device instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_send_fifo(struct n32_sdmmc_s *priv)
{
  uint32_t data;
  uint32_t status;

  while (priv->remaining > 0)
    {
      status = n32_sdmmc_getreg(priv, N32_SDHOST_PRESTS_OFFSET);
      if (!(status & N32_SDHOST_PRESTS_BUFW))
        {
          break; /* FIFO full */
        }

      if (priv->remaining >= 4)
        {
          data = *priv->buffer++;
          priv->remaining -= 4;
        }
      else
        {
          /* Last fractional word */

          uint8_t *ptr = (uint8_t *)priv->buffer;

          data = 0;
          for (int i = 0; i < priv->remaining; i++)
            {
              ((uint8_t *)&data)[i] = ptr[i];
            }

          priv->remaining = 0;
        }

      n32_sdmmc_putreg(priv, data, N32_SDHOST_BUFDAT_OFFSET);
    }
}

/****************************************************************************
 * Name: n32_sdmmc_recv_fifo
 *
 * Description:
 *   Read data from SDHOST FIFO to buffer (PIO read).
 *
 * Input Parameters:
 *   priv - SDMMC device instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_recv_fifo(struct n32_sdmmc_s *priv)
{
  uint32_t data;
  uint32_t status;

  while (priv->remaining > 0)
    {
      status = n32_sdmmc_getreg(priv, N32_SDHOST_PRESTS_OFFSET);
      if (!(status & N32_SDHOST_PRESTS_BUFR))
        {
          break; /* No data */
        }

      data = n32_sdmmc_getreg(priv, N32_SDHOST_BUFDAT_OFFSET);

      if (priv->remaining >= 4)
        {
          *priv->buffer++ = data;
          priv->remaining -= 4;
        }
      else
        {
          uint8_t *ptr = (uint8_t *)priv->buffer;

          for (int i = 0; i < priv->remaining; i++)
            {
              ptr[i] = ((uint8_t *)&data)[i];
            }

          priv->remaining = 0;
        }
    }
}

/****************************************************************************
 * Name: n32_sdmmc_datadisable
 *
 * Description:
 *   Disable data path, stop DMA, and reset data state.
 *
 * Input Parameters:
 *   priv - SDMMC device instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_datadisable(struct n32_sdmmc_s *priv)
{
  /* Clear BLKCFG to prevent residual data */

  n32_sdmmc_putreg(priv, 0, N32_SDHOST_BLKCFG_OFFSET);

  /* If in DMA mode, stop ADMA */

#if HAVE_ADMA2
  if (priv->adma_in_progress)
    {
      n32_sdmmc_stop_adma(priv);
    }
#endif

  /* Software reset data line to flush FIFO */

  n32_sdmmc_modifyreg(priv, N32_SDHOST_CTRL2_OFFSET,
                      0, N32_SDHOST_CTRL2_SWRSTDAT);
  up_udelay(10);
  n32_sdmmc_modifyreg(priv, N32_SDHOST_CTRL2_OFFSET,
                      N32_SDHOST_CTRL2_SWRSTDAT, 0);

  /* Clear all data-related interrupt pending bits */

  n32_sdmmc_putreg(priv, N32_SDHOST_INTSTS_TC | N32_SDHOST_INTSTS_BUFWRDY |
                      N32_SDHOST_INTSTS_BUFRRDY | N32_SDHOST_INTSTS_DMAINT |
                      N32_SDHOST_INTSTS_DTERR | N32_SDHOST_INTSTS_DCRERR |
                      N32_SDHOST_INTSTS_ADMAERR,
                   N32_SDHOST_INTSTS_OFFSET);

  priv->remaining = 0;
  priv->using_dma = false;
}

/* ==========================================================================
 * ADMA2 Support
 * ==========================================================================
 */

#if HAVE_ADMA2

/****************************************************************************
 * Name: n32_sdmmc_build_adma_desc
 *
 * Description:
 *   Build an ADMA2 descriptor table in system memory.
 *
 * Input Parameters:
 *   priv   - SDMMC device instance
 *   buffer - Data buffer address
 *   len    - Total data length in bytes
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

/* ADMA2 descriptor table in None-TCM memory */
#ifdef CONFIG_N32H7_SDMMC1
static struct n32_sdmmc_adma_desc_s locate_data(".axi_ram")
  n32_sdmmc1_desc[CONFIG_N32H7_SDMMC_MAX_DESCS] aligned_data(4);
#endif /* CONFIG_N32H7_SDMMC1 */
#ifdef CONFIG_N32H7_SDMMC2
static struct n32_sdmmc_adma_desc_s locate_data(".axi_ram")
  n32_sdmmc2_desc[CONFIG_N32H7_SDMMC_MAX_DESCS] aligned_data(4);
#endif /* CONFIG_N32H7_SDMMC2 */
#if CONFIG_MMCSD_MULTIBLOCK_LIMIT <= 16
#  ifdef CONFIG_N32H7_SDMMC1
static uint8_t locate_data(".axi_ram")
  n32_sdmmc1_blk_buf
  [N32_SDMMC_MAX_BLOCK_SIZE * CONFIG_MMCSD_MULTIBLOCK_LIMIT]
  aligned_data(4);
#  endif /* CONFIG_N32H7_SDMMC1 */
#  ifdef CONFIG_N32H7_SDMMC2
static uint8_t locate_data(".axi_ram")
  n32_sdmmc2_blk_buf
  [N32_SDMMC_MAX_BLOCK_SIZE * CONFIG_MMCSD_MULTIBLOCK_LIMIT]
  aligned_data(4);
#  endif /* CONFIG_N32H7_SDMMC2 */
#else
#  error "CONFIG_MMCSD_MULTIBLOCK_LIMIT must be less than or equal to 16"
#endif /* CONFIG_MMCSD_MULTIBLOCK_LIMIT */

static int n32_sdmmc_build_adma_desc(struct n32_sdmmc_s *priv,
                                     uint8_t *buffer, size_t len)
{
  uint32_t total_len = len;
  uint32_t offset = 0;
  uint32_t max_chunk = 65535; /* 64KB - 1 */
  struct n32_sdmmc_adma_desc_s *desc = NULL;
  uint8_t desc_count = 0;
  uint8_t *blk_buf = NULL;

#ifdef CONFIG_N32H7_SDMMC1
  if (priv->irq == N32_IRQ_SDMMC1)
    {
      desc = n32_sdmmc1_desc;
      blk_buf = n32_sdmmc1_blk_buf;
    }
#endif /* CONFIG_N32H7_SDMMC1 */

#ifdef CONFIG_N32H7_SDMMC2
  if (priv->irq == N32_IRQ_SDMMC2)
    {
      desc = n32_sdmmc2_desc;
      blk_buf = n32_sdmmc2_blk_buf;
    }
#endif /* CONFIG_N32H7_SDMMC2 */

  /* Calculate number of descriptors needed */

  while (len > 0)
    {
      uint32_t chunk = (len > max_chunk) ? max_chunk : len;

      desc_count++;
      len -= chunk;
    }

  if (desc_count == 0 || desc_count > CONFIG_N32H7_SDMMC_MAX_DESCS)
    {
      return -EINVAL;
    }

  priv->adma_list.desc = desc;
  priv->adma_list.count = desc_count;
  priv->adma_list.total_len = total_len;

  /* Fill descriptors */

  len = total_len;
  offset = 0;
  for (uint8_t i = 0; i < desc_count; i++)
    {
      uint32_t chunk = (len > max_chunk) ? max_chunk : len;

      desc[i].addr = (uint32_t *)(blk_buf + offset);
      desc[i].len = (uint16_t)chunk;
      desc[i].attr = N32_ADMA_ENTRY_VALID | N32_ADMA_ACT_TRAN;

      /* Prepare block buffer in None-TCM memory */

      if (!priv->is_read)
        {
          memcpy(desc[i].addr, buffer + offset, chunk);
        }

      /* Invalidate D-Cache before DMA read from memory */

      up_invalidate_dcache((uintptr_t)desc[i].addr,
                           (uintptr_t)desc[i].addr + chunk);

      if (i == desc_count - 1)
        {
          desc[i].attr |= N32_ADMA_ENTRY_END;
        }

      /* Optionally set INT for every descriptor or at end */

      /* desc[i].attr |= N32_ADMA_ENTRY_INT; */

      offset += chunk;
      len -= chunk;
    }

  mcinfo("ADMA desc: count=%d, total_len=%lu\n",
         desc_count, (unsigned long)total_len);
  return OK;
}

/****************************************************************************
 * Name: n32_sdmmc_start_adma
 *
 * Description:
 *   Start ADMA2 transfer by setting descriptor address and enabling ADMA.
 *
 * Input Parameters:
 *   priv - SDMMC device instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_start_adma(struct n32_sdmmc_s *priv)
{
  /* Set ADMA descriptor address */

  n32_sdmmc_putreg(priv, (uint32_t)priv->adma_list.desc,
                   N32_SDHOST_ASADD0_OFFSET);

  /* High 32 bits = 0 (32-bit addressing) */

  n32_sdmmc_putreg(priv, 0, N32_SDHOST_ASADD1_OFFSET);

  /* Select ADMA2 in CTRL1 */

  n32_sdmmc_modifyreg(priv, N32_SDHOST_CTRL1_OFFSET,
                      N32_SDHOST_CTRL1_DMASEL_MASK,
                      N32_SDHOST_CTRL1_DMASEL_ADMA2);

  priv->adma_in_progress = true;
  priv->using_dma = true;

  mcinfo("ADMA started, desc addr=0x%08lx\n",
         (unsigned long)priv->adma_list.desc);
}

/****************************************************************************
 * Name: n32_sdmmc_stop_adma
 *
 * Description:
 *   Stop ADMA2 and free descriptor table.
 *
 * Input Parameters:
 *   priv - SDMMC device instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_stop_adma(struct n32_sdmmc_s *priv)
{
  priv->adma_in_progress = false;
  priv->using_dma = false;

  /* Disable ADMA */

  n32_sdmmc_modifyreg(priv, N32_SDHOST_CTRL1_OFFSET,
                      N32_SDHOST_CTRL1_DMASEL_MASK, 0);

  /* Free descriptor table */

  if (priv->adma_list.desc)
    {
      priv->adma_list.desc = NULL;
      priv->adma_list.count = 0;
      priv->adma_list.total_len = 0;
    }

  mcinfo("ADMA stopped\n");
}

#endif /* HAVE_ADMA2 */

/* ==========================================================================
 * Event Handling
 * ==========================================================================
 */

/****************************************************************************
 * Name: n32_sdmmc_endwait
 *
 * Description:
 *   Wake up a waiting thread, cancel timeout, and clear events.
 *
 * Input Parameters:
 *   priv      - SDMMC device instance
 *   wkupevent - Event that caused the wakeup
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_endwait(struct n32_sdmmc_s *priv,
                              sdio_eventset_t wkupevent)
{
  /* Cancel the watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Disable all waiting-related interrupts */

  priv->waitmask = 0;
  priv->waitevents = 0;
  n32_sdmmc_configints(priv);

  /* Set wakeup event and post semaphore */

  priv->wkupevent = wkupevent;
  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: n32_sdmmc_eventtimeout
 *
 * Description:
 *   Watchdog timeout handler for event wait.
 *
 * Input Parameters:
 *   arg - Pointer to SDMMC device instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_eventtimeout(wdparm_t arg)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)arg;

  if (priv->waitevents & SDIOWAIT_TIMEOUT)
    {
      n32_sdmmc_endwait(priv, SDIOWAIT_TIMEOUT);
    }
}

/* ==========================================================================
 * Interrupt Handler
 * ==========================================================================
 */

/****************************************************************************
 * Name: n32_sdmmc_interrupt
 *
 * Description:
 *   SDHOST interrupt handler.
 *
 * Input Parameters:
 *   irq     - IRQ number
 *   context - Interrupt context
 *   arg     - Pointer to SDMMC device instance
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

static int n32_sdmmc_interrupt(int irq, void *context, void *arg)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)arg;
  uint32_t status;
  uint32_t enabled;
  uint32_t error_mask;

  status = n32_sdmmc_getreg(priv, N32_SDHOST_INTSTS_OFFSET);
  enabled = status & n32_sdmmc_getreg(priv, N32_SDHOST_ISE_OFFSET);

  if (enabled == 0)
    {
      return OK;
    }

  /* Handle errors (all error bits) */

  error_mask = enabled & (N32_SDHOST_INTSTS_CTERR    |
                          N32_SDHOST_INTSTS_CCRCERR  |
                          N32_SDHOST_INTSTS_CENDBERR |
                          N32_SDHOST_INTSTS_CINXERR  |
                          N32_SDHOST_INTSTS_DTERR    |
                          N32_SDHOST_INTSTS_DCRERR   |
                          N32_SDHOST_INTSTS_DENDERR  |
                          N32_SDHOST_INTSTS_ADMAERR  |
                          N32_SDHOST_INTSTS_ACMDERR);

  if (error_mask)
    {
      mcerr("ERROR: status=0x%08lx mask=0x%08lx\n",
            (unsigned long)status, (unsigned long)error_mask);
      n32_sdmmc_putreg(priv, error_mask, N32_SDHOST_INTSTS_OFFSET);

      /* Disable data path on data errors */

      if (error_mask & (N32_SDHOST_INTSTS_DTERR   |
                        N32_SDHOST_INTSTS_DCRERR  |
                        N32_SDHOST_INTSTS_DENDERR |
                        N32_SDHOST_INTSTS_ADMAERR))
        {
          n32_sdmmc_datadisable(priv);
        }

      if (priv->waitevents & SDIOWAIT_ERROR)
        {
          n32_sdmmc_endwait(priv, SDIOWAIT_ERROR);
        }

      return OK;
    }

  /* Command Complete */

  if (enabled & N32_SDHOST_INTSTS_CMDC)
    {
      n32_sdmmc_putreg(priv, N32_SDHOST_INTSTS_CMDC,
                       N32_SDHOST_INTSTS_OFFSET);
      if (priv->waitevents & SDIOWAIT_CMDDONE)
        {
          n32_sdmmc_endwait(priv, SDIOWAIT_CMDDONE);
        }
    }

  /* Transfer Complete */

  if (enabled & N32_SDHOST_INTSTS_TC)
    {
      n32_sdmmc_putreg(priv, N32_SDHOST_INTSTS_TC,
                       N32_SDHOST_INTSTS_OFFSET);

      if (priv->waitevents & SDIOWAIT_TRANSFERDONE)
        {
          if (priv->using_dma)
            {
              priv->adma_in_progress = false;
              if (priv->is_read)
                {
                  uint32_t offset = 0;

                  for (uint8_t i = 0; i < priv->adma_list.count; i++)
                    {
                      /* Invalidate D-Cache after DMA write to memory */

                      up_invalidate_dcache((uintptr_t)
                       priv->adma_list.desc[i].addr,
                       (uintptr_t)priv->adma_list.desc[i].addr +
                       priv->adma_list.desc[i].len);

                      /* Copy data from None-TCM memory to buffer */

                      memcpy(priv->buffer + offset,
                       priv->adma_list.desc[i].addr,
                       priv->adma_list.desc[i].len);
                      priv->remaining -= priv->adma_list.desc[i].len;
                      offset += priv->adma_list.desc[i].len;
                      if (priv->adma_list.desc[i].attr & N32_ADMA_ENTRY_END)
                        {
                          break;
                        }
                    }
                }
              else
                {
                  priv->remaining = 0;
                }
            }

          if (priv->remaining == 0)
            {
              n32_sdmmc_endwait(priv, SDIOWAIT_TRANSFERDONE);
            }
        }
    }

  /* Buffer Write Ready (PIO) */

  if (enabled & N32_SDHOST_INTSTS_BUFWRDY)
    {
      n32_sdmmc_putreg(priv, N32_SDHOST_INTSTS_BUFWRDY,
                       N32_SDHOST_INTSTS_OFFSET);
      if (!priv->using_dma && !priv->is_read)
        {
          n32_sdmmc_send_fifo(priv);
          if (priv->remaining == 0)
            {
              /* All data written, disable BUFWRDY interrupt */

              priv->waitmask &= ~N32_SDHOST_INTSTS_BUFWRDY;
              n32_sdmmc_configints(priv);
            }
        }
    }

  /* Buffer Read Ready (PIO) */

  if (enabled & N32_SDHOST_INTSTS_BUFRRDY)
    {
      n32_sdmmc_putreg(priv, N32_SDHOST_INTSTS_BUFRRDY,
                       N32_SDHOST_INTSTS_OFFSET);
      if (!priv->using_dma && priv->is_read)
        {
          n32_sdmmc_recv_fifo(priv);
          if (priv->remaining == 0)
            {
              /* All data read, disable BUFRRDY interrupt */

              priv->waitmask &= ~N32_SDHOST_INTSTS_BUFRRDY;
              n32_sdmmc_configints(priv);
            }
        }
    }

  /* DMA Interrupt (for SDMA, not used by ADMA) */

  if (enabled & N32_SDHOST_INTSTS_DMAINT)
    {
      n32_sdmmc_putreg(priv, N32_SDHOST_INTSTS_DMAINT,
                       N32_SDHOST_INTSTS_OFFSET);
    }

  return OK;
}

/* ==========================================================================
 * SDIO Interface Methods
 * ==========================================================================
 */

/****************************************************************************
 * Name: n32_sdmmc_lock
 *
 * Description:
 *   Lock/unlock the SD bus (not implemented for single slot).
 *
 * Input Parameters:
 *   dev  - SDIO device instance
 *   lock - true to lock, false to unlock
 *
 * Returned Value:
 *   Always OK.
 *
 ****************************************************************************/
#ifdef CONFIG_SDIO_MUXBUS
static int n32_sdmmc_lock(struct sdio_dev_s *dev, bool lock)
{
  /* Not implemented for single slot */

  return OK;
}
#endif /* CONFIG_SDIO_MUXBUS */

/****************************************************************************
 * Name: n32_sdmmc_reset
 *
 * Description:
 *   Reset the SDMMC controller to initial state.
 *
 * Input Parameters:
 *   dev - SDIO device instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_reset(struct sdio_dev_s *dev)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  uint32_t regval;
  uint16_t timeout = 10000;

  /* 1. Software Reset All */

  n32_sdmmc_modifyreg(priv, N32_SDHOST_CTRL2_OFFSET,
                      0, N32_SDHOST_CTRL2_SWRSTALL);

  /* Wait for reset to clear */

  do
    {
      regval = n32_sdmmc_getreg(priv, N32_SDHOST_CTRL2_OFFSET);
      if ((regval & N32_SDHOST_CTRL2_SWRSTALL) == 0)
        {
          break;
        }

      up_udelay(10);
    }
  while (--timeout > 0);

  /* Disable all status word */

  priv->waitmask = 0;
  priv->waitevents = 0;
  n32_sdmmc_configints(priv);
  n32_sdmmc_configsts(priv, 0);

  /* 2. Soft CD enable */

  regval = n32_sdmmc_getreg(priv, N32_SDHOST_CTRL1_OFFSET);
  regval |= N32_SDHOST_CTRL1_CDSD;
  n32_sdmmc_putreg(priv, regval, N32_SDHOST_CTRL1_OFFSET);

  /* 3. Configure CFG1 (BCLKF = 100 MHz, TCLKU = KHz) */

  regval = (N32_BCLKF_FREQ << N32_SDMMC_CFG1_BCLKF_SHIFT) | 0; /* TCLKU=0 */
  N32_PUT_CFG(priv, N32_SDMMC_CFG1_OFFSET, regval);

  /* 4. Set capabilities (hardcoded) */

  regval = N32_SDMMC_CFG2_VS33 | N32_SDMMC_CFG2_HS | N32_SDMMC_CFG2_ADMA2;
  N32_PUT_CFG(priv, N32_SDMMC_CFG2_OFFSET, regval);

  /* 5. Reset state */

  priv->clock = 0;
  priv->remaining = 0;
  priv->using_dma = false;
  priv->widebus = false;
  priv->waitevents = 0;
  priv->wkupevent = 0;
  priv->cdstatus = 0;

  wd_cancel(&priv->waitwdog);

  mcinfo("SDMMC reset completed\n");
}

/****************************************************************************
 * Name: n32_sdmmc_capabilities
 *
 * Description:
 *   Return SDIO capabilities (DMA supported if ADMA enabled).
 *
 * Input Parameters:
 *   dev - SDIO device instance
 *
 * Returned Value:
 *   Bitmask of capabilities (SDIO_CAPS_*).
 *
 ****************************************************************************/

static sdio_capset_t n32_sdmmc_capabilities(struct sdio_dev_s *dev)
{
  sdio_capset_t caps = SDIO_CAPS_4BIT | SDIO_CAPS_DMABEFOREWRITE;
#if HAVE_ADMA2
  caps |= (SDIO_CAPS_DMASUPPORTED/* | SDIO_CAPS_SD_HS_MODE */);
#endif
  return caps;
}

/****************************************************************************
 * Name: n32_sdmmc_status
 *
 * Description:
 *   Return current card status (present, write protected).
 *
 * Input Parameters:
 *   dev - SDIO device instance
 *
 * Returned Value:
 *   Bitmask of status flags (SDIO_STATUS_*).
 *
 ****************************************************************************/

static sdio_statset_t n32_sdmmc_status(struct sdio_dev_s *dev)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  return priv->cdstatus;
}

/****************************************************************************
 * Name: n32_sdmmc_widebus
 *
 * Description:
 *   Enable/disable 4-bit wide bus mode.
 *
 * Input Parameters:
 *   dev  - SDIO device instance
 *   enable - true for 4-bit, false for 1-bit
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_widebus(struct sdio_dev_s *dev, bool enable)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  priv->widebus = enable;

  if (enable)
    {
      n32_sdmmc_modifyreg(priv, N32_SDHOST_CTRL1_OFFSET,
                          0, N32_SDHOST_CTRL1_DTWIDTH);
    }
  else
    {
      n32_sdmmc_modifyreg(priv, N32_SDHOST_CTRL1_OFFSET,
                          N32_SDHOST_CTRL1_DTWIDTH, 0);
    }
}

/****************************************************************************
 * Name: n32_sdmmc_clock
 *
 * Description:
 *   Set SD clock frequency based on the requested mode.
 *
 * Input Parameters:
 *   dev  - SDIO device instance
 *   rate - Clock mode (CLOCK_*)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  uint32_t freq = 0, regval;

  switch (rate)
    {
      case CLOCK_SDIO_DISABLED:
        freq = 0;
        break;
      case CLOCK_IDMODE:
        freq = 400000;
        break;
      case CLOCK_MMC_TRANSFER:
      case CLOCK_MMC_TRANSFER_4BIT:
        freq = 25000000;
        break;
      case CLOCK_SD_TRANSFER_1BIT:
      case CLOCK_SD_TRANSFER_4BIT:
        freq = 25000000;
        break;
      case CLOCK_SD_TRANSFER_4BIT_HS:
        regval = n32_sdmmc_getreg(priv, N32_SDHOST_CTRL1_OFFSET);
        regval |= N32_SDHOST_CTRL1_HSEN;
        n32_sdmmc_putreg(priv, regval, N32_SDHOST_CTRL1_OFFSET);
        freq = 50000000;
        break;
      default:
        freq = 400000;
        break;
    }

  n32_sdmmc_enable_clock(priv, freq);
}

/****************************************************************************
 * Name: n32_sdmmc_attach
 *
 * Description:
 *   Attach interrupt handler and enable NVIC.
 *
 * Input Parameters:
 *   dev - SDIO device instance
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

static int n32_sdmmc_attach(struct sdio_dev_s *dev)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  int ret;

  ret = irq_attach(priv->irq, n32_sdmmc_interrupt, priv);
  if (ret == OK)
    {
      /* Disable all interrupts */

      n32_sdmmc_putreg(priv, 0, N32_SDHOST_IE_OFFSET);
      n32_sdmmc_putreg(priv, 0, N32_SDHOST_ISE_OFFSET);

      /* Clear pending */

      n32_sdmmc_putreg(priv, 0xffffffff, N32_SDHOST_INTSTS_OFFSET);
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: n32_sdmmc_sendcmd
 *
 * Description:
 *   Send an SD command (wrapper for n32_sdmmc_send_cmd).
 *
 * Input Parameters:
 *   dev - SDIO device instance
 *   cmd - Command word
 *   arg - 32-bit argument
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

static int n32_sdmmc_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t arg)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  return n32_sdmmc_send_cmd(priv, cmd, arg);
}

/****************************************************************************
 * Name: n32_sdmmc_blocksetup
 *
 * Description:
 *   Configure block size and number of blocks for next transfer.
 *
 * Input Parameters:
 *   dev       - SDIO device instance
 *   blocksize - Block size in bytes
 *   nblocks   - Number of blocks
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_blocksetup(struct sdio_dev_s *dev,
                                 unsigned int blocksize,
                                 unsigned int nblocks)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;

  priv->blocksize = blocksize;
  priv->nblocks = nblocks;
}

/****************************************************************************
 * Name: n32_sdmmc_recvsetup
 *
 * Description:
 *   Setup for PIO read transfer.
 *
 * Input Parameters:
 *   dev    - SDIO device instance
 *   buffer - Data buffer
 *   nbytes - Number of bytes to read
 *
 * Returned Value:
 *   OK on success.
 *
 ****************************************************************************/

static int n32_sdmmc_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                               size_t nbytes)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;

  /* Reset data path state */

  n32_sdmmc_datadisable(priv);

  priv->buffer = (uint32_t *)buffer;
  priv->remaining = nbytes;
  priv->is_read = true;
  priv->using_dma = false;

  /* Enable buffer read ready interrupt */

  n32_sdmmc_putreg(priv, N32_SDHOST_INTSTS_BUFRRDY,
                   N32_SDHOST_INTSTS_OFFSET);
  priv->waitmask |= N32_SDHOST_INTSTS_BUFRRDY;
  n32_sdmmc_configints(priv);

  mcinfo("PIO recv setup: buffer=%p, len=%zu\n", buffer, nbytes);
  return OK;
}

/****************************************************************************
 * Name: n32_sdmmc_sendsetup
 *
 * Description:
 *   Setup for PIO write transfer.
 *
 * Input Parameters:
 *   dev    - SDIO device instance
 *   buffer - Data buffer
 *   nbytes - Number of bytes to write
 *
 * Returned Value:
 *   OK on success.
 *
 ****************************************************************************/

static int n32_sdmmc_sendsetup(struct sdio_dev_s *dev,
                               const uint8_t *buffer, size_t nbytes)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;

  /* Reset data path state */

  n32_sdmmc_datadisable(priv);

  priv->buffer = (uint32_t *)buffer;
  priv->remaining = nbytes;
  priv->is_read = false;
  priv->using_dma = false;

  /* Enable buffer write ready interrupt */

  n32_sdmmc_putreg(priv, N32_SDHOST_INTSTS_BUFWRDY,
                   N32_SDHOST_INTSTS_OFFSET);
  priv->waitmask |= N32_SDHOST_INTSTS_BUFWRDY;
  n32_sdmmc_configints(priv);

  mcinfo("PIO send setup: buffer=%p, len=%zu\n", buffer, nbytes);
  return OK;
}

/****************************************************************************
 * Name: n32_sdmmc_cancel
 *
 * Description:
 *   Cancel any ongoing data transfer and reset controller state.
 *
 * Input Parameters:
 *   dev - SDIO device instance
 *
 * Returned Value:
 *   OK on success.
 *
 ****************************************************************************/

static int n32_sdmmc_cancel(struct sdio_dev_s *dev)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;

  /* Disable interrupts */

  priv->waitmask = 0;
  priv->waitevents = 0;
  n32_sdmmc_configints(priv);
  wd_cancel(&priv->waitwdog);

  /* Disable data path */

  n32_sdmmc_datadisable(priv);

  /* Soft reset command line */

  n32_sdmmc_modifyreg(priv, N32_SDHOST_CTRL2_OFFSET,
                      0, N32_SDHOST_CTRL2_SWRSTCMD);
  up_udelay(10);
  n32_sdmmc_modifyreg(priv, N32_SDHOST_CTRL2_OFFSET,
                      N32_SDHOST_CTRL2_SWRSTCMD, 0);

  return OK;
}

/****************************************************************************
 * Name: n32_sdmmc_waitresponse
 *
 * Description:
 *   Poll-wait for command response.
 *
 * Input Parameters:
 *   dev - SDIO device instance
 *   cmd - Command word
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

static int n32_sdmmc_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  return n32_sdmmc_wait_cmd_done(priv, cmd);
}

/****************************************************************************
 * Name: n32_sdmmc_recvshortcrc
 *
 * Description:
 *   Receive short response with CRC (R1, R5, R6).
 *
 * Input Parameters:
 *   dev    - SDIO device instance
 *   cmd    - Command word
 *   rshort - Pointer to store response
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

static int n32_sdmmc_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                                  uint32_t *rshort)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  uint32_t status;

  status = n32_sdmmc_getreg(priv, N32_SDHOST_INTSTS_OFFSET);
  if (status & N32_SDHOST_INTSTS_CTERR)
    {
      n32_sdmmc_putreg(priv, N32_SDHOST_INTSTS_CTERR,
                       N32_SDHOST_INTSTS_OFFSET);
      return -ETIMEDOUT;
    }

  if (status & N32_SDHOST_INTSTS_CCRCERR)
    {
      n32_sdmmc_putreg(priv, N32_SDHOST_INTSTS_CCRCERR,
                       N32_SDHOST_INTSTS_OFFSET);
      return -EIO;
    }

  n32_sdmmc_get_response(priv, cmd, rshort);
  return OK;
}

/****************************************************************************
 * Name: n32_sdmmc_recvlong
 *
 * Description:
 *   Receive long response (R2).
 *
 * Input Parameters:
 *   dev   - SDIO device instance
 *   cmd   - Command word
 *   rlong - Pointer to 4-word array for response
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

static int n32_sdmmc_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                              uint32_t rlong[4])
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  uint32_t status;

  status = n32_sdmmc_getreg(priv, N32_SDHOST_INTSTS_OFFSET);
  if (status & N32_SDHOST_INTSTS_CTERR)
    {
      n32_sdmmc_putreg(priv, N32_SDHOST_INTSTS_CTERR,
                       N32_SDHOST_INTSTS_OFFSET);
      return -ETIMEDOUT;
    }

  if (status & N32_SDHOST_INTSTS_CCRCERR)
    {
      n32_sdmmc_putreg(priv, N32_SDHOST_INTSTS_CCRCERR,
                       N32_SDHOST_INTSTS_OFFSET);
      return -EIO;
    }

  n32_sdmmc_get_response(priv, cmd, rlong);
  return OK;
}

/****************************************************************************
 * Name: n32_sdmmc_recvshort
 *
 * Description:
 *   Receive short response without CRC (R3, R4, R7).
 *
 * Input Parameters:
 *   dev    - SDIO device instance
 *   cmd    - Command word
 *   rshort - Pointer to store response
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

static int n32_sdmmc_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                               uint32_t *rshort)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  uint32_t status;

  status = n32_sdmmc_getreg(priv, N32_SDHOST_INTSTS_OFFSET);
  if (status & N32_SDHOST_INTSTS_CTERR)
    {
      n32_sdmmc_putreg(priv, N32_SDHOST_INTSTS_CTERR,
                       N32_SDHOST_INTSTS_OFFSET);
      return -ETIMEDOUT;
    }

  n32_sdmmc_get_response(priv, cmd, rshort);
  return OK;
}

/****************************************************************************
 * Name: n32_sdmmc_waitenable
 *
 * Description:
 *   Enable waiting for a set of events.
 *
 * Input Parameters:
 *   dev      - SDIO device instance
 *   eventset - Events to wait for (SDIOWAIT_*)
 *   timeout  - Timeout in milliseconds
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_waitenable(struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset,
                                 uint32_t timeout)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  uint32_t mask = priv->waitmask;

  if (eventset & SDIOWAIT_CMDDONE)
    {
      mask |= N32_SDHOST_INTSTS_CMDC;
    }

  if (eventset & SDIOWAIT_TRANSFERDONE)
    {
      mask |= N32_SDHOST_INTSTS_TC | N32_SDHOST_INTSTS_DTERR |
              N32_SDHOST_INTSTS_DCRERR;
    }

  if (eventset & SDIOWAIT_ERROR)
    {
#if HAVE_ADMA2
      mask |= N32_SDHOST_INTSTS_CTERR | N32_SDHOST_INTSTS_CCRCERR |
              N32_SDHOST_INTSTS_ADMAERR;
#else
      mask |= N32_SDHOST_INTSTS_CTERR | N32_SDHOST_INTSTS_CCRCERR;
#endif
    }

  /* Clear interrupt status register */

  n32_sdmmc_putreg(priv, mask, N32_SDHOST_INTSTS_OFFSET);

  /* Enable interrupts */

  priv->waitmask = mask;
  priv->waitevents = eventset;
  n32_sdmmc_configints(priv);

  if (eventset & SDIOWAIT_TIMEOUT)
    {
      int delay = MSEC2TICK(timeout);
      wd_start(&priv->waitwdog, delay, n32_sdmmc_eventtimeout,
               (wdparm_t)priv);
    }
}

/****************************************************************************
 * Name: n32_sdmmc_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur.
 *
 * Input Parameters:
 *   dev - SDIO device instance
 *
 * Returned Value:
 *   The event that occurred.
 *
 ****************************************************************************/

static sdio_eventset_t n32_sdmmc_eventwait(struct sdio_dev_s *dev)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  sdio_eventset_t wkupevent = 0;
  int ret;

  while (1)
    {
      ret = nxsem_wait_uninterruptible(&priv->waitsem);
      if (ret < 0)
        {
          wd_cancel(&priv->waitwdog);
          return SDIOWAIT_ERROR;
        }

      wkupevent = priv->wkupevent;
      if (wkupevent != 0)
        break;
    }

  /* Disable interrupts after event */

  priv->waitmask = 0;
  priv->waitevents = 0;
  n32_sdmmc_configints(priv);
  return wkupevent;
}

/****************************************************************************
 * Name: n32_sdmmc_callbackenable
 *
 * Description:
 *   Enable callback events (not fully implemented).
 *
 * Input Parameters:
 *   dev      - SDIO device instance
 *   eventset - Events to enable for callback
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void n32_sdmmc_callbackenable(struct sdio_dev_s *dev,
                                     sdio_eventset_t eventset)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  priv->cbevents = eventset;
}

/****************************************************************************
 * Name: n32_sdmmc_registercallback
 *
 * Description:
 *   Register a callback function for card status changes.
 *
 * Input Parameters:
 *   dev      - SDIO device instance
 *   callback - Callback function
 *   arg      - Argument for callback
 *
 * Returned Value:
 *   OK on success.
 *
 ****************************************************************************/

static int n32_sdmmc_registercallback(struct sdio_dev_s *dev,
                                      worker_t callback, void *arg)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  priv->callback = callback;
  priv->cbarg = arg;
  priv->cbevents = 0;
  return OK;
}
#ifdef CONFIG_SDIO_DMA
#  if HAVE_ADMA2

/****************************************************************************
 * Name: n32_sdmmc_dmarecvsetup
 *
 * Description:
 *   Setup for ADMA read transfer (with cache invalidation).
 *
 * Input Parameters:
 *   dev    - SDIO device instance
 *   buffer - Data buffer
 *   buflen - Number of bytes to read
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

static int n32_sdmmc_dmarecvsetup(struct sdio_dev_s *dev,
                                  uint8_t *buffer, size_t buflen)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  int ret;

  /* Reset data path state */

  n32_sdmmc_datadisable(priv);

  priv->buffer = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->is_read = true;
  priv->using_dma = true;

  ret = n32_sdmmc_build_adma_desc(priv, buffer, buflen);
  if (ret < 0)
    {
      return ret;
    }

  n32_sdmmc_start_adma(priv);

  return OK;
}

/****************************************************************************
 * Name: n32_sdmmc_dmasendsetup
 *
 * Description:
 *   Setup for ADMA write transfer (with cache clean).
 *
 * Input Parameters:
 *   dev    - SDIO device instance
 *   buffer - Data buffer
 *   buflen - Number of bytes to write
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

static int n32_sdmmc_dmasendsetup(struct sdio_dev_s *dev,
                                  const uint8_t *buffer, size_t buflen)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  int ret;

  /* Reset data path state */

  n32_sdmmc_datadisable(priv);

  priv->buffer = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->is_read = false;
  priv->using_dma = true;

  ret = n32_sdmmc_build_adma_desc(priv, (uint8_t *)buffer, buflen);
  if (ret < 0)
    {
      return ret;
    }

  n32_sdmmc_start_adma(priv);

  return OK;
}
#  else
    error "CONFIG_SDIO_DMA requires HAVE_ADMA2"
#  endif /* HAVE_ADMA2 */
#endif /* CONFIG_SDIO_DMA */

/****************************************************************************
 * Name: n32_sdmmc_callback
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

static void n32_sdmmc_callback(void *arg)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)arg;
  uint32_t regval;
  uint8_t timeout = 100;

  /* Is a callback registered? */

  DEBUGASSERT(priv != NULL);

  /* Wait for CD to settle */

  if (priv->cdstatus & SDIO_STATUS_PRESENT)
    {
      regval = n32_sdmmc_getreg(priv, N32_SDHOST_CTRL1_OFFSET);
      regval |= N32_SDHOST_CTRL1_CDTL;
      n32_sdmmc_putreg(priv, regval, N32_SDHOST_CTRL1_OFFSET);

      do
        {
          nxsched_msleep(10);
          regval = n32_sdmmc_getreg(priv, N32_SDHOST_PRESTS_OFFSET);
          if ((regval & N32_SDHOST_PRESTS_CSTSL) == N32_SDHOST_PRESTS_CSTSL)
            break;
        }
      while (--timeout > 0);

      n32_sdmmc_power_on(priv);
      n32_sdmmc_configsts(priv, N32_WAIT_ALL);
    }
  else
    {
      regval = n32_sdmmc_getreg(priv, N32_SDHOST_CTRL1_OFFSET);
      regval &= ~N32_SDHOST_CTRL1_CDTL;
      n32_sdmmc_putreg(priv, regval, N32_SDHOST_CTRL1_OFFSET);

      do
        {
          nxsched_msleep(10);
          regval = n32_sdmmc_getreg(priv, N32_SDHOST_PRESTS_OFFSET);
          if ((regval & N32_SDHOST_PRESTS_CSTSL) == N32_SDHOST_PRESTS_CSTSL)
            break;
        }
      while (--timeout > 0);

      n32_sdmmc_power_off(priv);
      n32_sdmmc_configsts(priv, 0);
    }

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

      mcinfo("Callback to %p(%p)\n", priv->callback, priv->cbarg);
      priv->callback(priv->cbarg);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - 0 for SDMMC1, 1 for SDMMC2 (depending on config).
 *
 * Returned Values:
 *   A reference to an SDIO interface structure. NULL is returned on failure.
 *
 ****************************************************************************/

struct sdio_dev_s *sdio_initialize(int slotno)
{
  struct n32_sdmmc_s *priv = NULL;
  uint32_t regval;

#if defined(CONFIG_N32H7_SDMMC1)
  if (slotno == 0)
    {
      priv = &g_sdmmc1_dev;

      /* Enable the HSC1 Power Gate */

      regval = getreg32(N32_PWR_SYS_PWR_CR3);
      regval |= PWR_SYSCTRL3_HSC1_PGEN;
      putreg32(regval, N32_PWR_SYS_PWR_CR3);

      while ((getreg32(N32_PWR_SYS_PWR_CR3) &
              PWR_SYSCTRL3_HSC1_PWRRDY) == 0);

      regval = getreg32(N32_PWR_SYS_PWR_CR3);
      regval |= (PWR_SYSCTRL3_HSC1_FUCEN | PWR_SYSCTRL3_HSC1_ISNEN);
      putreg32(regval, N32_PWR_SYS_PWR_CR3);

      /* Enable the SDMMC1 peripheral clock */

      regval  = getreg32(N32_RCC_AXIEN1);
      regval |= RCC_AXIEN1_M7SDMMC1EN;
      putreg32(regval, N32_RCC_AXIEN1);

      regval  = getreg32(N32_PWR_IP_MEMPWR_CR);
      regval &= ~PWR_IPMEMCTRL_SDMMC1_PGEN;
      putreg32(regval, N32_PWR_IP_MEMPWR_CR);

      while ((getreg32(N32_PWR_IP_MEMPWR_CSR) &
              PWR_IPMEMSTS_SDMMC1_PRDY) == 0);

      /* Configure sdmmc1_clk source is PLL1B */

      regval  = getreg32(N32_RCC_AXISEL1);
      regval &= ~RCC_AXISEL1_SDMMC1KERSEL_MASK;
      regval |= RCC_AXISEL1_SDMMC1KERSEL_PLL1B;
      putreg32(regval, N32_RCC_AXISEL1);

      /* Enable clock and reset */

      regval = getreg32(N32_RCC_AXIEN1);
      regval |= RCC_AXIEN1_M7SDMMC1EN;
      putreg32(regval, N32_RCC_AXIEN1);
      regval = getreg32(N32_RCC_AXIRST1);
      regval |= (RCC_AXIRST1_SDMMC1RST | RCC_AXIRST1_SDHOST1RST);
      putreg32(regval, N32_RCC_AXIRST1);
      up_udelay(10);
      regval &= ~(RCC_AXIRST1_SDMMC1RST | RCC_AXIRST1_SDHOST1RST);
      putreg32(regval, N32_RCC_AXIRST1);

      /* Configure GPIOs (must be defined in board.h) */

#ifndef CONFIG_SDIO_MUXBUS
      n32_configgpio(GPIO_SDMMC1_CK);
      n32_configgpio(GPIO_SDMMC1_CMD);
      n32_configgpio(GPIO_SDMMC1_D0);
#  ifndef CONFIG_SDMMC1_WIDTH_D1_ONLY
      n32_configgpio(GPIO_SDMMC1_D1);
      n32_configgpio(GPIO_SDMMC1_D2);
      n32_configgpio(GPIO_SDMMC1_D3);
#  endif

      /* Enable internal clock feedback */

      regval  = getreg32(N32_AFIO_RMP_CFG);
      regval |= N32_AFIO_RMP_CFG_SDMMC1_CLKFB;
      putreg32(regval, N32_AFIO_RMP_CFG);
#endif
    }
  else
#endif
#if defined(CONFIG_N32H7_SDMMC2)
  if (slotno == 1)
    {
      priv = &g_sdmmc2_dev;

      /* Enable the HSC2 Power Gate */

      regval = getreg32(N32_PWR_SYS_PWR_CR3);
      regval |= PWR_SYSCTRL3_HSC2_PGEN;
      putreg32(regval, N32_PWR_SYS_PWR_CR3);

      while ((getreg32(N32_PWR_SYS_PWR_CR3) &
              PWR_SYSCTRL3_HSC2_PWRRDY) == 0);

      regval = getreg32(N32_PWR_SYS_PWR_CR3);
      regval |= (PWR_SYSCTRL3_HSC2_FUCEN | PWR_SYSCTRL3_HSC2_ISNEN);
      putreg32(regval, N32_PWR_SYS_PWR_CR3);

      /* Enable the USBHS2 peripheral clock */

      regval  = getreg32(N32_RCC_AHB1EN1);
      regval |= RCC_AHB1EN1_M7SDMMC2EN;
      putreg32(regval, N32_RCC_AHB1EN1);

      regval  = getreg32(N32_PWR_IP_MEMPWR_CR);
      regval &= ~PWR_IPMEMCTRL_SDMMC2_PGEN;
      putreg32(regval, N32_PWR_IP_MEMPWR_CR);

      while ((getreg32(N32_PWR_IP_MEMPWR_CSR) &
              PWR_IPMEMSTS_SDMMC2_PRDY) == 0);

      /* Configure sdmmc2_clk source is PLL1B */

      regval  = getreg32(N32_RCC_AHB1SEL1);
      regval &= ~RCC_AHB1SEL1_SDMMC2KERSEL_MASK;
      regval |= RCC_AHB1SEL1_SDMMC2KERSEL_PLL1B;
      putreg32(regval, N32_RCC_AHB1SEL1);

      /* Enable clock and reset */

      regval = getreg32(N32_RCC_AHB1EN1);
      regval |= RCC_AHB1EN1_M7SDMMC2EN;
      putreg32(regval, N32_RCC_AHB1EN1);
      regval = getreg32(N32_RCC_AHB1RST1);
      regval |= (RCC_AHB1RST1_SDMMC2RST | RCC_AHB1RST1_SDHOST2RST);
      putreg32(regval, N32_RCC_AHB1RST1);
      up_udelay(10);
      regval &= ~(RCC_AHB1RST1_SDMMC2RST | RCC_AHB1RST1_SDHOST2RST);
      putreg32(regval, N32_RCC_AHB1RST1);

#ifndef CONFIG_SDIO_MUXBUS
      n32_configgpio(GPIO_SDMMC2_CK);
      n32_configgpio(GPIO_SDMMC2_CMD);
      n32_configgpio(GPIO_SDMMC2_D0);
#  ifndef CONFIG_SDMMC2_WIDTH_D1_ONLY
      n32_configgpio(GPIO_SDMMC2_D1);
      n32_configgpio(GPIO_SDMMC2_D2);
      n32_configgpio(GPIO_SDMMC2_D3);
#  endif

      /* Enable internal clock feedback */

      regval  = getreg32(N32_AFIO_RMP_CFG);
      regval |= N32_AFIO_RMP_CFG_SDMMC2_CLKFB;
      putreg32(regval, N32_AFIO_RMP_CFG);
#endif
    }
  else
#endif
    {
      mcerr("ERROR: Unsupported SDMMC slot: %d\n", slotno);
      return NULL;
    }

  if (priv)
    {
      n32_sdmmc_reset(&priv->dev);
    }

  return &priv->dev;
}

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic to signal card insertion/removal.
 *
 * Input Parameters:
 *   dev        - SDIO device instance
 *   cardinslot - true if card inserted, false otherwise
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sdio_mediachange(struct sdio_dev_s *dev, bool cardinslot)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  sdio_statset_t oldstatus;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Save the old status for comparison */

  oldstatus = priv->cdstatus;

  /* Update the card status */

  if (cardinslot)
    {
      priv->cdstatus |= SDIO_STATUS_PRESENT;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_PRESENT;
    }

  leave_critical_section(flags);

  /* If the status actually changed, schedule the callback to notify
   * the upper MMC/SD layer. Note: This may be called from an interrupt
   * context (EXTI), so we must use the high-priority work queue.
   */

  if (oldstatus != priv->cdstatus)
    {
      mcinfo("Media change: old=0x%02x, new=0x%02x, scheduling callback\n",
             oldstatus, priv->cdstatus);

      /* Callbacks cannot be performed in the context of an interrupt
       * handler.  If we are in an interrupt handler, then queue the
       * callback to be performed later on the work thread.
       */

      if (up_interrupt_context())
        {
          /* Yes.. queue it */

          work_queue(HPWORK, &priv->cbwork, n32_sdmmc_callback,
                        priv, 0);
        }

      else
        {
          /* No.. then just call the callback here */

          n32_sdmmc_callback(priv);
        }
    }
}

/****************************************************************************
 * Name: sdio_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report write protect status.
 *
 * Input Parameters:
 *   dev       - SDIO device instance
 *   wrprotect - true if write protected, false otherwise
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sdio_wrprotect(struct sdio_dev_s *dev, bool wrprotect)
{
  struct n32_sdmmc_s *priv = (struct n32_sdmmc_s *)dev;
  irqstate_t flags = enter_critical_section();

  if (wrprotect)
    priv->cdstatus |= SDIO_STATUS_WRPROTECTED;
  else
    priv->cdstatus &= ~SDIO_STATUS_WRPROTECTED;

  leave_critical_section(flags);
}

#endif /* CONFIG_N32H7_SDMMC1 || CONFIG_N32H7_SDMMC2 */
