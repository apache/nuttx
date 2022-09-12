/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_emmcsd.c
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
#include <nuttx/mmcsd.h>
#include <nuttx/irq.h>
#include <nuttx/cache.h>

#include <arch/board/board.h>

#include "mpfs_emmcsd.h"
#include "riscv_internal.h"
#include "hardware/mpfs_emmcsd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_EMMCSD_HRS00      (priv->hw_base + MPFS_EMMCSD_HRS00_OFFSET)
#define MPFS_EMMCSD_HRS01      (priv->hw_base + MPFS_EMMCSD_HRS01_OFFSET)
#define MPFS_EMMCSD_HRS04      (priv->hw_base + MPFS_EMMCSD_HRS04_OFFSET)
#define MPFS_EMMCSD_HRS06      (priv->hw_base + MPFS_EMMCSD_HRS06_OFFSET)

#define MPFS_EMMCSD_SRS01      (priv->hw_base + MPFS_EMMCSD_SRS01_OFFSET)
#define MPFS_EMMCSD_SRS02      (priv->hw_base + MPFS_EMMCSD_SRS02_OFFSET)
#define MPFS_EMMCSD_SRS03      (priv->hw_base + MPFS_EMMCSD_SRS03_OFFSET)
#define MPFS_EMMCSD_SRS04      (priv->hw_base + MPFS_EMMCSD_SRS04_OFFSET)
#define MPFS_EMMCSD_SRS05      (priv->hw_base + MPFS_EMMCSD_SRS05_OFFSET)
#define MPFS_EMMCSD_SRS06      (priv->hw_base + MPFS_EMMCSD_SRS06_OFFSET)
#define MPFS_EMMCSD_SRS07      (priv->hw_base + MPFS_EMMCSD_SRS07_OFFSET)
#define MPFS_EMMCSD_SRS08      (priv->hw_base + MPFS_EMMCSD_SRS08_OFFSET)
#define MPFS_EMMCSD_SRS09      (priv->hw_base + MPFS_EMMCSD_SRS09_OFFSET)
#define MPFS_EMMCSD_SRS10      (priv->hw_base + MPFS_EMMCSD_SRS10_OFFSET)
#define MPFS_EMMCSD_SRS11      (priv->hw_base + MPFS_EMMCSD_SRS11_OFFSET)
#define MPFS_EMMCSD_SRS12      (priv->hw_base + MPFS_EMMCSD_SRS12_OFFSET)
#define MPFS_EMMCSD_SRS13      (priv->hw_base + MPFS_EMMCSD_SRS13_OFFSET)
#define MPFS_EMMCSD_SRS14      (priv->hw_base + MPFS_EMMCSD_SRS14_OFFSET)
#define MPFS_EMMCSD_SRS15      (priv->hw_base + MPFS_EMMCSD_SRS15_OFFSET)
#define MPFS_EMMCSD_SRS16      (priv->hw_base + MPFS_EMMCSD_SRS16_OFFSET)
#define MPFS_EMMCSD_SRS21      (priv->hw_base + MPFS_EMMCSD_SRS21_OFFSET)
#define MPFS_EMMCSD_SRS22      (priv->hw_base + MPFS_EMMCSD_SRS22_OFFSET)
#define MPFS_EMMCSD_SRS23      (priv->hw_base + MPFS_EMMCSD_SRS23_OFFSET)

#define MPFS_SYSREG_SOFT_RESET_CR     (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SOFT_RESET_CR_OFFSET)
#define MPFS_SYSREG_SUBBLK_CLOCK_CR   (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET)

#define MPFS_PMPCFG_MMC_0             (MPFS_MPUCFG_BASE + 0x700)
#define MPFS_PMPCFG_MMC_1             (MPFS_MPUCFG_BASE + 0x708)
#define MPFS_PMPCFG_MMC_2             (MPFS_MPUCFG_BASE + 0x710)
#define MPFS_PMPCFG_MMC_3             (MPFS_MPUCFG_BASE + 0x718)

#define MPFS_MMC_CLOCK_400KHZ              400u
#define MPFS_MMC_CLOCK_12_5MHZ             12500u
#define MPFS_MMC_CLOCK_25MHZ               25000u
#define MPFS_MMC_CLOCK_26MHZ               26000u
#define MPFS_MMC_CLOCK_50MHZ               50000u
#define MPFS_MMC_CLOCK_100MHZ              100000u
#define MPFS_MMC_CLOCK_200MHZ              200000u

#define MPFS_EMMCSD_DEBOUNCE_TIME          0x300000u
#define MPFS_EMMCSD_DATA_TIMEOUT           500000

#define MPFS_EMMCSD_SRS10_3_3V_BUS_VOLTAGE (0x7 << 9)
#define MPFS_EMMCSD_SRS10_3_0V_BUS_VOLTAGE (0x6 << 9)
#define MPFS_EMMCSD_SRS10_1_8V_BUS_VOLTAGE (0x5 << 9)

#define MPFS_EMMCSD_1_8V_BUS_VOLTAGE       18
#define MPFS_EMMCSD_3_3V_BUS_VOLTAGE       33

#define MPFS_EMMCSD_INITIALIZED            0x00
#define MPFS_EMMCSD_NOT_INITIALIZED        0x01

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE"
#endif

/* High-speed single data rate supports clock frequency up to 52 MHz and data
 * bus width of 1 bit, 4 bits, and 8 bits.
 */

#define MPFS_EMMCSD_MODE_SDR               0x2u

/* High speed double data rate supports clock frequency up to 52 MHz and data
 * bus width of 4 bits and 8 bits.
 */

#define MPFS_EMMCSD_MODE_DDR               0x3u

/* SDR data sampling supports clock frequency up to 200 MHz and data bus
 * width of 4 bits and 8 bits.
 */

#define MPFS_EMMCSD_MODE_HS200             0x4u

/* DDR data sampling supports clock frequency up to 200 MHz and data bus
 * width of 8 bits.
 */

#define MPFS_EMMCSD_MODE_HS400             0x5u

/* HS400 mode with Enhanced Strobe */

#define MPFS_EMMCSD_MODE_HS400_ES          0x6u

/* Backwards compatibility with legacy MMC card supports clock frequency up
 * to 26MHz and data bus width of 1 bit, 4 bits, and 8 bits.
 */

#define MPFS_EMMCSD_MODE_LEGACY            0x7u

/* Define the Hardware FIFO size */

#define FIFO_SIZE_IN_BYTES                 64

/* Timing */

#define EMMCSD_CMDTIMEOUT                  (100000)
#define EMMCSD_LONGTIMEOUT                 (100000000)

/* Event waiting interrupt mask bits */

#define MPFS_EMMCSD_CARD_INTS     (MPFS_EMMCSD_SRS14_CINT_IE   | \
                                   MPFS_EMMCSD_SRS14_CIN_IE    | \
                                   MPFS_EMMCSD_SRS14_CR_IE)

#define MPFS_EMMCSD_CMDDONE_STA   (MPFS_EMMCSD_SRS12_CC)

#define MPFS_EMMCSD_RESPDONE_STA  (MPFS_EMMCSD_SRS12_ECT       | \
                                   MPFS_EMMCSD_SRS12_ECCRC)

#define MPFS_EMMCSD_XFRDONE_STA   (MPFS_EMMCSD_SRS12_TC)

#define MPFS_EMMCSD_CMDDONE_MASK  (MPFS_EMMCSD_CARD_INTS       | \
                                   MPFS_EMMCSD_SRS14_CC_IE)

#define MPFS_EMMCSD_RESPDONE_MASK (MPFS_EMMCSD_CARD_INTS       | \
                                   MPFS_EMMCSD_SRS14_ECCRC_IE  | \
                                   MPFS_EMMCSD_SRS14_ECT_IE)

#define MPFS_EMMCSD_XFRDONE_MASK  (MPFS_EMMCSD_CARD_INTS       | \
                                   MPFS_EMMCSD_SRS14_TC_IE)

#define MPFS_EMMCSD_CMDDONE_ICR   (MPFS_EMMCSD_SRS12_CC)

#define MPFS_EMMCSD_RESPDONE_ICR  (MPFS_EMMCSD_SRS12_ECT       | \
                                   MPFS_EMMCSD_SRS12_ECCRC)

#define MPFS_EMMCSD_XFRDONE_ICR   (MPFS_EMMCSD_SRS12_EDCRC     | \
                                   MPFS_EMMCSD_SRS12_EDT       | \
                                   MPFS_EMMCSD_SRS12_TC)

#define MPFS_EMMCSD_WAITALL_ICR   (MPFS_EMMCSD_CMDDONE_ICR     | \
                                   MPFS_EMMCSD_RESPDONE_ICR    | \
                                   MPFS_EMMCSD_XFRDONE_ICR)

#define MPFS_EMMCSD_RECV_MASKDMA  (MPFS_EMMCSD_CARD_INTS       | \
                                   MPFS_EMMCSD_SRS14_EADMA_IE  | \
                                   MPFS_EMMCSD_SRS14_DMAINT_IE | \
                                   MPFS_EMMCSD_SRS14_EDT_IE    | \
                                   MPFS_EMMCSD_SRS14_ECT_IE    | \
                                   MPFS_EMMCSD_SRS14_BRR_IE    | \
                                   MPFS_EMMCSD_SRS14_TC_IE)

#define MPFS_EMMCSD_RECV_MASK     (MPFS_EMMCSD_CARD_INTS       | \
                                   MPFS_EMMCSD_SRS14_EDT_IE    | \
                                   MPFS_EMMCSD_SRS14_ECT_IE    | \
                                   MPFS_EMMCSD_SRS14_BRR_IE    | \
                                   MPFS_EMMCSD_SRS14_TC_IE)

#define MPFS_EMMCSD_SEND_MASKDMA  (MPFS_EMMCSD_CARD_INTS       | \
                                   MPFS_EMMCSD_SRS14_EADMA_IE  | \
                                   MPFS_EMMCSD_SRS14_DMAINT_IE | \
                                   MPFS_EMMCSD_SRS14_EDT_IE    | \
                                   MPFS_EMMCSD_SRS14_ECT_IE    | \
                                   MPFS_EMMCSD_SRS14_BWR_IE    | \
                                   MPFS_EMMCSD_SRS14_TC_IE)

#define MPFS_EMMCSD_SEND_MASK     (MPFS_EMMCSD_CARD_INTS       | \
                                   MPFS_EMMCSD_SRS14_EDT_IE    | \
                                   MPFS_EMMCSD_SRS14_ECT_IE    | \
                                   MPFS_EMMCSD_SRS14_BWR_IE    | \
                                   MPFS_EMMCSD_SRS14_TC_IE)

/* SD-Card IOMUX */
#define LIBERO_SETTING_IOMUX1_CR_SD                     0x00000000UL
#ifdef CONFIG_MPFS_EMMCSD_MUX_GPIO
#define LIBERO_SETTING_IOMUX2_CR_SD   0X00BB0000UL
#else
#define LIBERO_SETTING_IOMUX2_CR_SD   0x00000000UL
#endif
#define LIBERO_SETTING_IOMUX6_CR_SD   0X0000001DUL
#define LIBERO_SETTING_MSSIO_BANK4_CFG_CR_SD            0x00080907UL
#define LIBERO_SETTING_MSSIO_BANK4_IO_CFG_0_1_CR_SD     0x08290829UL
#define LIBERO_SETTING_MSSIO_BANK4_IO_CFG_2_3_CR_SD     0x08290829UL
#define LIBERO_SETTING_MSSIO_BANK4_IO_CFG_4_5_CR_SD     0x08290829UL
#define LIBERO_SETTING_MSSIO_BANK4_IO_CFG_6_7_CR_SD     0x08290829UL
#define LIBERO_SETTING_MSSIO_BANK4_IO_CFG_8_9_CR_SD     0x08290829UL
#define LIBERO_SETTING_MSSIO_BANK4_IO_CFG_10_11_CR_SD   0x08290829UL
#define LIBERO_SETTING_MSSIO_BANK4_IO_CFG_12_13_CR_SD   0x08290829UL

/* eMMC IOMUX settings */

#define LIBERO_SETTING_IOMUX1_CR_EMMC                   0x11111111UL
#define LIBERO_SETTING_IOMUX2_CR_EMMC                   0x00FF1111UL
#define LIBERO_SETTING_IOMUX6_CR_EMMC                   0x00000000UL
#define LIBERO_SETTING_MSSIO_BANK4_CFG_CR_EMMC          0x00040A0DUL
#define LIBERO_SETTING_MSSIO_BANK4_IO_CFG_0_1_CR_EMMC   0x09280928UL
#define LIBERO_SETTING_MSSIO_BANK4_IO_CFG_2_3_CR_EMMC   0x09280928UL
#define LIBERO_SETTING_MSSIO_BANK4_IO_CFG_4_5_CR_EMMC   0x09280928UL
#define LIBERO_SETTING_MSSIO_BANK4_IO_CFG_6_7_CR_EMMC   0x09280928UL
#define LIBERO_SETTING_MSSIO_BANK4_IO_CFG_8_9_CR_EMMC   0x09280928UL
#define LIBERO_SETTING_MSSIO_BANK4_IO_CFG_10_11_CR_EMMC 0x09280928UL
#define LIBERO_SETTING_MSSIO_BANK4_IO_CFG_12_13_CR_EMMC 0x09280928UL

/* eMMC / SD switch address */

#define SDIO_REGISTER_ADDRESS                           0x4f000000

#define MPFS_SYSREG_IOMUX1   (MPFS_SYSREG_BASE + \
                              MPFS_SYSREG_IOMUX1_CR_OFFSET)
#define MPFS_SYSREG_IOMUX2   (MPFS_SYSREG_BASE + \
                              MPFS_SYSREG_IOMUX2_CR_OFFSET)
#define MPFS_SYSREG_IOMUX6   (MPFS_SYSREG_BASE + \
                              MPFS_SYSREG_IOMUX6_CR_OFFSET)
#define MPFS_SYSREG_B4_CFG   (MPFS_SYSREG_BASE + \
                              MPFS_SYSREG_MSSIO_BANK4_CFG_CR)
#define MPFS_SYSREG_B4_0_1   (MPFS_SYSREG_BASE + \
                              MPFS_SYSREG_MSSIO_BANK4_IO_CFG_0_1_CR_OFFSET)
#define MPFS_SYSREG_B4_2_3   (MPFS_SYSREG_BASE + \
                              MPFS_SYSREG_MSSIO_BANK4_IO_CFG_2_3_CR_OFFSET)
#define MPFS_SYSREG_B4_4_5   (MPFS_SYSREG_BASE + \
                              MPFS_SYSREG_MSSIO_BANK4_IO_CFG_4_5_CR_OFFSET)
#define MPFS_SYSREG_B4_6_7   (MPFS_SYSREG_BASE + \
                              MPFS_SYSREG_MSSIO_BANK4_IO_CFG_6_7_CR_OFFSET)
#define MPFS_SYSREG_B4_8_9   (MPFS_SYSREG_BASE + \
                              MPFS_SYSREG_MSSIO_BANK4_IO_CFG_8_9_CR_OFFSET)
#define MPFS_SYSREG_B4_10_11 (MPFS_SYSREG_BASE + \
                              MPFS_SYSREG_MSSIO_BANK4_IO_CFG_10_11_CR_OFFSET)
#define MPFS_SYSREG_4_12_13  (MPFS_SYSREG_BASE + \
                              MPFS_SYSREG_MSSIO_BANK4_IO_CFG_12_13_CR_OFFSET)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the state of the MPFS eMMCSD interface */

struct mpfs_dev_s
{
  struct sdio_dev_s  dev;             /* Standard, base SDIO interface */

  const uintptr_t    hw_base;         /* Base address */
  const int          plic_irq;        /* PLIC interrupt */
  bool               clk_enabled;     /* Clk state */

  /* eMMC / SD and HW parameters */

  const bool         emmc;            /* eMMC or SD */
  int                bus_voltage;     /* Bus voltage */
  int                bus_speed;       /* eMMC Bus speed */
  bool               jumpers_3v3;     /* Jumper settings: 1v8 or 3v3 */

  /* Event support */

  sem_t              waitsem;         /* Implements event waiting */
  sdio_eventset_t    waitevents;      /* Set of events to be waited for */
  uint32_t           waitmask;        /* Interrupt enables for event waiting */
  volatile sdio_eventset_t wkupevent; /* The event that caused the wakeup */
  struct wdog_s      waitwdog;        /* Watchdog that handles event timeouts */

  /* Callback support */

  sdio_statset_t     cdstatus;        /* Card status */
  sdio_eventset_t    cbevents;        /* Set of events to be cause callbacks */
  worker_t           callback;        /* Registered callback function */
  void              *cbarg;           /* Registered callback argument */
  struct work_s      cbwork;          /* Callback work queue structure */

  /* Interrupt mode data transfer support */

  uint32_t          *buffer;          /* Address of current R/W buffer */
  size_t             remaining;       /* Number of bytes remaining in the transfer */
  size_t             receivecnt;      /* Real count to receive */
  uint32_t           xfrmask;         /* Interrupt enables for data transfer */

  bool               widebus;         /* Required for DMA support */
  bool               onebit;          /* true: Only 1-bit transfers are supported */

  /* DMA data transfer support */

  bool               polltransfer;    /* Indicate a poll transfer, no DMA */

  /* Misc */

  uint32_t           blocksize;       /* Current block size */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helper  ********************************************************/

#define     mpfs_givesem(priv) (nxsem_post(&priv->waitsem))

/* Mutual exclusion */

#if defined(CONFIG_SDIO_MUXBUS)
static int  mpfs_lock(struct sdio_dev_s *dev, bool lock);
#endif

/* Initialization/setup */

static void mpfs_reset(struct sdio_dev_s *dev);
static sdio_capset_t mpfs_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t mpfs_status(struct sdio_dev_s *dev);
static void mpfs_widebus(struct sdio_dev_s *dev, bool enable);
static void mpfs_clock(struct sdio_dev_s *dev,
                       enum sdio_clock_e rate);
static int  mpfs_attach(struct sdio_dev_s *dev);

/* Command / Status / Data Transfer */

static int  mpfs_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                         uint32_t arg);
#ifdef CONFIG_SDIO_BLOCKSETUP
static void mpfs_blocksetup(struct sdio_dev_s *dev,
              unsigned int blocksize, unsigned int nblocks);
#endif
static int  mpfs_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                           size_t nbytes);
static int  mpfs_sendsetup(struct sdio_dev_s *dev,
                           const uint8_t *buffer, size_t nbytes);
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

/* DMA */

#if defined(CONFIG_SDIO_DMA)
static int  mpfs_dmarecvsetup(struct sdio_dev_s *dev,
                              uint8_t *buffer, size_t buflen);
static int  mpfs_dmasendsetup(struct sdio_dev_s *dev,
                              const uint8_t *buffer, size_t buflen);
#endif

/* Initialization/uninitialization/reset ************************************/

static void mpfs_callback(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct mpfs_dev_s g_emmcsd_dev =
{
  .dev =
  {
#if defined(CONFIG_SDIO_MUXBUS)
    .lock             = mpfs_lock,
#endif
    .reset            = mpfs_reset,
    .capabilities     = mpfs_capabilities,
    .status           = mpfs_status,
    .widebus          = mpfs_widebus,
    .clock            = mpfs_clock,
    .attach           = mpfs_attach,
    .sendcmd          = mpfs_sendcmd,
#ifdef CONFIG_SDIO_BLOCKSETUP
    .blocksetup       = mpfs_blocksetup,
#endif
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
#if defined(CONFIG_SDIO_DMA)
    .dmarecvsetup     = mpfs_dmarecvsetup,
    .dmasendsetup     = mpfs_dmasendsetup,
#endif
  },
  .hw_base           = MPFS_EMMC_SD_BASE,
  .plic_irq          = MPFS_IRQ_MMC_MAIN,
#ifdef CONFIG_MPFS_EMMCSD_MUX_EMMC
  .emmc              = true,
#else
  .emmc              = false,
#endif
  .blocksize         = 512,
  .onebit            = false,
  .polltransfer      = true,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_takesem
 *
 * Description:
 *   Take the wait semaphore (handling false alarm wakeups due to the receipt
 *   of signals).
 *
 * Input Parameters:
 *   priv  - Instance of the EMMCSD private state structure.
 *
 * Returned Value:
 *   Normally OK, but may return -ECANCELED in the rare event that the task
 *   has been canceled.
 *
 ****************************************************************************/

static int mpfs_takesem(struct mpfs_dev_s *priv)
{
  return nxsem_wait_uninterruptible(&priv->waitsem);
}

/****************************************************************************
 * Name: mpfs_reset_lines
 *
 * Description:
 *   Resets the DAT and CMD lines and waits until they are cleared.
 *
 * Input Parameters:
 *   priv  - Instance of the EMMCSD private state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_reset_lines(struct mpfs_dev_s *priv)
{
  uint32_t retries = EMMCSD_CMDTIMEOUT;
  uint32_t status;

  /* Software Reset For DAT Line and CMD Line */

  modifyreg32(MPFS_EMMCSD_SRS11, 0, MPFS_EMMCSD_SRS11_SRDAT |
              MPFS_EMMCSD_SRS11_SRCMD);

  do
    {
      status = getreg32(MPFS_EMMCSD_SRS11);
    }
  while ((status & (MPFS_EMMCSD_SRS11_SRDAT | MPFS_EMMCSD_SRS11_SRCMD)) &&
         --retries);

  if (retries == 0)
    {
      mcerr("Timeout waiting line resets!\n");
    }
}

/****************************************************************************
 * Name: mpfs_check_lines_busy
 *
 * Description:
 *   Verifies the DAT and CMD lines are available, not busy.
 *
 * Input Parameters:
 *   priv  - Instance of the EMMCSD private state structure.
 *
 * Returned Value:
 *   true if busy, false if available
 *
 ****************************************************************************/

static bool mpfs_check_lines_busy(struct mpfs_dev_s *priv)
{
  uint32_t retries = EMMCSD_LONGTIMEOUT;
  uint32_t srs9;

  do
    {
      srs9 = getreg32(MPFS_EMMCSD_SRS09);
    }
  while (srs9 & (MPFS_EMMCSD_SRS09_CICMD | MPFS_EMMCSD_SRS09_CIDAT) &&
         --retries);

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
 *   priv  - Instance of the EMMCSD private state structure.
 *   clkcr - New clock rate.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_setclkrate(struct mpfs_dev_s *priv, uint32_t clkrate)
{
  uint32_t baseclk;
  uint32_t settings;
  uint32_t divider;
  uint32_t clkstable;
  uint32_t retries = EMMCSD_CMDTIMEOUT;
  int i;

  mcinfo("clkrate: %08" PRIx32 "\n", clkrate);

  if (clkrate == 0)
    {
      modifyreg32(MPFS_EMMCSD_SRS11, MPFS_EMMCSD_SRS11_SDCE, 0);
      priv->clk_enabled = false;
      return;
    }

  baseclk = (getreg32(MPFS_EMMCSD_SRS16) & 0xff00) >> 8;
  baseclk *= 1000;

  DEBUGASSERT(baseclk != 0);

  /* 10-bit divider, search for match (N*2) */

  for (i = 1; i < 2046; i++)
    {
      if (((baseclk / i) < clkrate) || (((baseclk / i) == clkrate) &&
          ((baseclk % i) == 0u)))
        {
          break;
        }
    }

  divider = ((i / 2) << 8);

  /* Set SDCLK Frequency Select and Internal Clock Enable */

  settings = (divider & 0xff00u) | ((divider & 0x30000u) >> 10) |
              MPFS_EMMCSD_SRS11_ICE;

  /* Disable SD clock if enabled */

  if (priv->clk_enabled)
    {
      modifyreg32(MPFS_EMMCSD_SRS11, MPFS_EMMCSD_SRS11_SDCE, 0);
    }

  /* Apply new settings */

  modifyreg32(MPFS_EMMCSD_SRS11, MPFS_EMMCSD_SRS11_SDCFSL |
              MPFS_EMMCSD_SRS11_SDCFSH | MPFS_EMMCSD_SRS11_ICE,
              settings);

  /* Wait for stable clock */

  clkstable = getreg32(MPFS_EMMCSD_SRS11);
  while (!(clkstable & MPFS_EMMCSD_SRS11_ICS) && --retries)
    {
      clkstable = getreg32(MPFS_EMMCSD_SRS11);
    }

  if (retries == 0)
    {
      mcwarn("Clock didn't get stable!\n");
      DEBUGPANIC();
    }

  /* HSE bit enabled if clk greater than 25 Mhz */

  if (clkrate > MPFS_MMC_CLOCK_25MHZ)
    {
      modifyreg32(MPFS_EMMCSD_SRS10, 0, MPFS_EMMCSD_SRS10_HSE);
    }

  priv->clk_enabled = true;
  modifyreg32(MPFS_EMMCSD_SRS11, 0, MPFS_EMMCSD_SRS11_SDCE);

  mcinfo("SRS11 now: %08" PRIx32 "\n", getreg32(MPFS_EMMCSD_SRS11));
}

/****************************************************************************
 * Name: mpfs_configwaitints
 *
 * Description:
 *   Enable/disable SDIO interrupts needed to support the wait function
 *
 * Input Parameters:
 *   priv       - Instance of the EMMCSD private state structure.
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
 *   priv    - Instance of the EMMCSD private state structure.
 *   xfrmask - The set of bits in the SDIO MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_configxfrints(struct mpfs_dev_s *priv, uint32_t xfrmask)
{
  irqstate_t flags;

  flags = enter_critical_section();
  priv->xfrmask = xfrmask;

  mcinfo("Mask: %08" PRIx32 "\n", priv->xfrmask | priv->waitmask);

  putreg32(priv->xfrmask | priv->waitmask, MPFS_EMMCSD_SRS14);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: mpfs_sendfifo
 *
 * Description:
 *   Send SDIO data in interrupt mode
 *
 * Input Parameters:
 *   priv  - Instance of the EMMCSD private state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_sendfifo(struct mpfs_dev_s *priv)
{
  union
  {
    uint32_t w;
    uint8_t  b[4];
  } data;

  /* Disable Buffer Write Ready interrupt */

  modifyreg32(MPFS_EMMCSD_SRS14, MPFS_EMMCSD_SRS14_BWR_IE, 0);

  putreg32(MPFS_EMMCSD_SRS12_TC, MPFS_EMMCSD_SRS12);

  /* We might get extra interrupts */

  if (priv->remaining == 0)
    {
      return;
    }

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
          int i;

          data.w = 0;
          for (i = 0; i < (int)priv->remaining; i++)
            {
              data.b[i] = *ptr++;
            }

          /* Now the transfer is finished */

          priv->remaining = 0;
        }

      /* Put the word in the FIFO */

      putreg32(data.w, MPFS_EMMCSD_SRS08);

      /* Multi-block writes may hit the wall - stop for now,
       * continue later. Enable BWR interrupt, clear status and
       * come back when we're good to write again.
       */

      if (priv->remaining && (!(getreg32(MPFS_EMMCSD_SRS09) &
          MPFS_EMMCSD_SRS09_BWE)))
        {
          modifyreg32(MPFS_EMMCSD_SRS14, 0, MPFS_EMMCSD_SRS14_BWR_IE);
          putreg32(MPFS_EMMCSD_SRS12_BWR, MPFS_EMMCSD_SRS12);
          return;
        }
    }

  mcinfo("Wrote all\n");
}

/****************************************************************************
 * Name: mpfs_recvfifo
 *
 * Description:
 *   Receive SDIO data in interrupt mode
 *
 * Input Parameters:
 *   priv  - Instance of the EMMCSD private state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_recvfifo(struct mpfs_dev_s *priv)
{
  union
  {
    uint32_t w;
    uint8_t  b[4];
  } data;

  mcinfo("Reading: %lu bytes\n", priv->remaining);

  /* Disable Buffer Read Ready interrupt */

  modifyreg32(MPFS_EMMCSD_SRS14, MPFS_EMMCSD_SRS14_BRR_IE, 0);

  if (!priv->remaining)
    {
      return;
    }

  /* Loop while there is space to store the data and there is more
   * data available in the RX FIFO.
   */

  while (priv->remaining > 0)
    {
      /* Multi-block reads may hit the wall - stop for now,
       * continue later. Enable BRR interrupt, clear status and
       * come back when we're good to read more.
       */

      if (!(getreg32(MPFS_EMMCSD_SRS09) & MPFS_EMMCSD_SRS09_BRE))
        {
          modifyreg32(MPFS_EMMCSD_SRS14, 0, MPFS_EMMCSD_SRS14_BRR_IE);
          putreg32(MPFS_EMMCSD_SRS12_BRR, MPFS_EMMCSD_SRS12);
          return;
        }

      /* Read the next word from the RX FIFO */

      data.w = getreg32(MPFS_EMMCSD_SRS08);
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
          int i;

          for (i = 0; i < (int)priv->remaining; i++)
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
 *   priv  - Instance of the EMMCSD private state structure.
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

  mpfs_givesem(priv);
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
         getreg32(MPFS_EMMCSD_SRS12),
         getreg32(MPFS_EMMCSD_SRS13));

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
 *   only from the SDIO interrupt handler when end-of-transfer conditions
 *   are detected.
 *
 * Input Parameters:
 *   priv  - Instance of the EMMCSD private state structure.
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

  mpfs_configxfrints(priv, 0);

  /* If there were errors, reset lines */

  if ((wkupevent & (~SDIOWAIT_TRANSFERDONE)) != 0)
    {
      mpfs_reset_lines(priv);
    }

  /* Clear Buffer Read Ready (BRR), BWR and DMA statuses */

  putreg32(MPFS_EMMCSD_SRS12_BRR | MPFS_EMMCSD_SRS12_BWR |
           MPFS_EMMCSD_SRS12_DMAINT, MPFS_EMMCSD_SRS12);

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
 * Name: mpfs_emmcsd_interrupt
 *
 * Description:
 *   eMMCSD interrupt handler
 *
 * Input Parameters:
 *   priv  - Instance of the eMMCSD private state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int mpfs_emmcsd_interrupt(int irq, void *context, void *arg)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)arg;
  uint32_t status;

  DEBUGASSERT(priv != NULL);

  status = getreg32(MPFS_EMMCSD_SRS12);

  mcinfo("status: %08" PRIx32 "\n", status);

  if (status & MPFS_EMMCSD_SRS12_EINT)
    {
      if (status & MPFS_EMMCSD_SRS12_EDCRC)
        {
          /* Handle data block send/receive CRC failure */

          mcerr("ERROR: Data block CRC failure, remaining: %lu\n",
                priv->remaining);

          mpfs_endtransfer(priv, SDIOWAIT_TRANSFERDONE |
                                 SDIOWAIT_ERROR);
        }
      else if (status & MPFS_EMMCSD_SRS12_EDT)
        {
          /* Handle data timeout error */

          mcerr("ERROR: Data timeout: %08" PRIx32 " remaining: %lu\n",
                status, priv->remaining);

          mpfs_endtransfer(priv, SDIOWAIT_TRANSFERDONE |
                                 SDIOWAIT_TIMEOUT);
        }
      else if (status & MPFS_EMMCSD_SRS12_EADMA)
        {
          /* Handle DMA error */

          mcerr("ERROR: DMA error: %08" PRIx32 " SRS21: %08" PRIx32 "\n",
                status, getreg32(MPFS_EMMCSD_SRS21));

          mpfs_endtransfer(priv, SDIOWAIT_TRANSFERDONE |
                                 SDIOWAIT_ERROR);
        }
      else
        {
          /* Generic error, not specified above */

          mcerr("ERROR: %08" PRIx32 "\n", status);
          mpfs_endtransfer(priv, SDIOWAIT_TRANSFERDONE |
                                 SDIOWAIT_ERROR);
        }
    }
  else
    {
      /* Handle wait events */

      if (status & MPFS_EMMCSD_SRS12_DMAINT)
        {
          /* Very large transfers may end up here.
           * They are not tested at all.
           */

          mcerr("DMAINT not expected, TC instead!\n");

          mpfs_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
        }
      else if (status & MPFS_EMMCSD_SRS12_BRR)
        {
          if (priv->polltransfer)
            {
              mpfs_recvfifo(priv);
              if (!priv->remaining)
                {
                  if (status & MPFS_EMMCSD_SRS12_TC)
                    {
                      mpfs_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
                    }
                }
            }
          else
            {
              mpfs_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
            }
        }
      else if (status & MPFS_EMMCSD_SRS12_BWR)
        {
          if (priv->polltransfer)
            {
              mpfs_sendfifo(priv);
              if (status & MPFS_EMMCSD_SRS12_TC)
                {
                  mpfs_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
                }
            }
        }
      else if (status & MPFS_EMMCSD_SRS12_TC)
        {
          putreg32(MPFS_EMMCSD_SRS12_TC, MPFS_EMMCSD_SRS12);
          if (priv->polltransfer && priv->receivecnt)
            {
              mcerr("Unexpected Transfer Complete!\n");
            }
          else
            {
              mpfs_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
            }
        }
      else if (status & MPFS_EMMCSD_SRS12_CC)
        {
          /* We don't handle Command Completes here! */

          DEBUGPANIC();
        }
      else if (status & MPFS_EMMCSD_SRS12_CIN)
        {
          mcinfo("Card inserted!\n");

          sdio_mediachange((struct sdio_dev_s *)priv, true);
          putreg32(MPFS_EMMCSD_SRS12_CIN, MPFS_EMMCSD_SRS12);
        }
      else if (status & MPFS_EMMCSD_SRS12_CR)
        {
          mcinfo("Card removed!\n");

          sdio_mediachange((struct sdio_dev_s *)priv, false);
          putreg32(MPFS_EMMCSD_SRS12_CR, MPFS_EMMCSD_SRS12);
        }
      else
        {
          mcerr("Status not handled: %08" PRIx32 "\n", status);
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
 *   priv        - Instance of the eMMCSD private state structure.
 *   timeout_us  - Requested timeout in microseconds
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_set_data_timeout(struct mpfs_dev_s *priv,
                                  uint32_t timeout_us)
{
  uint32_t timeout_interval;
  uint32_t sdmclk_khz;
  uint32_t sdmclk_mhz;
  uint32_t sdmclk;
  uint32_t timeout;
  uint32_t temp;
  uint8_t j;

  temp = getreg32(MPFS_EMMCSD_SRS16);

  /* 0x4u; - 0x4 is dummy -> 50Mhz * 4 = 200Mhz */

  sdmclk_khz = (temp & 0x0000003fu);

  DEBUGASSERT(sdmclk_khz);

  if (!(temp & MPFS_EMMCSD_SRS16_TCU))
    {
      DEBUGASSERT(timeout_us >= 1000);
    }
  else
    {
      sdmclk_khz *= 1000;
    }

  sdmclk_mhz = sdmclk_khz / 1000;

  if (sdmclk_mhz == 0)
    {
      sdmclk = sdmclk_khz;
      timeout = timeout_us / 1000u;
    }
  else
    {
      sdmclk = sdmclk_mhz;
      timeout = timeout_us;
    }

  /* calculate data timeout counter value */

  timeout_interval = 8192; /* 2^13 */
  for (j = 0; j < 15; j++)
    {
      if (timeout < (timeout_interval / sdmclk))
        {
          break;
        }

        timeout_interval *= 2;
    }

  timeout_interval = (uint32_t)j << 16;

  mcinfo("Data timeout: %08" PRIx32 "\n", timeout_interval);

  modifyreg32(MPFS_EMMCSD_SRS11, MPFS_EMMCSD_SRS11_DTCV, timeout_interval);
}

/****************************************************************************
 * Name: mpfs_set_sdhost_power
 *
 * Description:
 *   Sets the requested SD bus voltage.
 *
 * Input Parameters:
 *   priv     - Instance of the eMMCSD private state structure.
 *   voltage  - Requested voltage: 0v, 3v3, 3v0 or 1v8
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_set_sdhost_power(struct mpfs_dev_s *priv, uint32_t voltage)
{
  uint32_t srs16;

  /* Disable SD bus power */

  modifyreg32(MPFS_EMMCSD_SRS10, MPFS_EMMCSD_SRS10_BP, 0);

  /* Clear current voltage settings */

  modifyreg32(MPFS_EMMCSD_SRS10, MPFS_EMMCSD_SRS10_BVS, 0);

  if (!voltage)
    {
      return;
    }

  srs16 = getreg32(MPFS_EMMCSD_SRS16);

  switch (voltage)
    {
      case MPFS_EMMCSD_SRS10_3_3V_BUS_VOLTAGE:
        DEBUGASSERT(srs16 & MPFS_EMMCSD_SRS16_VS33);
        modifyreg32(MPFS_EMMCSD_SRS10,
                    MPFS_EMMCSD_SRS10_BVS,
                    MPFS_EMMCSD_SRS10_3_3V_BUS_VOLTAGE |
                    MPFS_EMMCSD_SRS10_BP);
        break;
      case MPFS_EMMCSD_SRS10_3_0V_BUS_VOLTAGE:
        DEBUGASSERT(srs16 & MPFS_EMMCSD_SRS16_VS30);
        modifyreg32(MPFS_EMMCSD_SRS10,
                    MPFS_EMMCSD_SRS10_BVS,
                    MPFS_EMMCSD_SRS10_3_0V_BUS_VOLTAGE |
                    MPFS_EMMCSD_SRS10_BP);
        break;
      case MPFS_EMMCSD_SRS10_1_8V_BUS_VOLTAGE:
        DEBUGASSERT(srs16 & MPFS_EMMCSD_SRS16_VS18);
        modifyreg32(MPFS_EMMCSD_SRS10,
                    MPFS_EMMCSD_SRS10_BVS,
                    MPFS_EMMCSD_SRS10_1_8V_BUS_VOLTAGE |
                    MPFS_EMMCSD_SRS10_BP);
        break;
      default:
        DEBUGPANIC();
    }

  nxsig_usleep(1000);
}

/****************************************************************************
 * Name: mpfs_sdcard_init
 *
 * Description:
 *   Switches to use to SD-card instead of eMMC. Call only if the SD-card
 *   is used, not eMMC. SDIO_REGISTER_ADDRESS is the switch itself: 0
 *   means eMMC and 1 is for SD. Also the IOMUX settings are applied
 *   properly.
 *
 * Input Parameters:
 *   priv     - Instance of the eMMCSD private state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_sdcard_init(struct mpfs_dev_s *priv)
{
  mcinfo("Init SD-card. Old FPGA will crash here!\n");

  putreg32(LIBERO_SETTING_IOMUX1_CR_SD, MPFS_SYSREG_IOMUX1);
  putreg32(LIBERO_SETTING_IOMUX2_CR_SD, MPFS_SYSREG_IOMUX2);
  putreg32(LIBERO_SETTING_IOMUX6_CR_SD, MPFS_SYSREG_IOMUX6);

#ifdef CONFIG_MPFS_EMMCSD_MUX_GPIO
  /* Select SD-card */

  mcinfo("Selecting SD card\n");
  mpfs_gpiowrite(MPFS_EMMCSD_GPIO, true);

#else /* CONFIG_ARCH_BOARD_ICICLE_MPFS */
  /* With 3.3v we exit from here */

  if (priv->jumpers_3v3)
    {
      putreg32(1, SDIO_REGISTER_ADDRESS);
      return;
    }

  putreg32(LIBERO_SETTING_MSSIO_BANK4_CFG_CR_SD, MPFS_SYSREG_B4_CFG);
  putreg32(LIBERO_SETTING_MSSIO_BANK4_IO_CFG_0_1_CR_SD, MPFS_SYSREG_B4_0_1);
  putreg32(LIBERO_SETTING_MSSIO_BANK4_IO_CFG_2_3_CR_SD, MPFS_SYSREG_B4_2_3);
  putreg32(LIBERO_SETTING_MSSIO_BANK4_IO_CFG_4_5_CR_SD, MPFS_SYSREG_B4_4_5);
  putreg32(LIBERO_SETTING_MSSIO_BANK4_IO_CFG_6_7_CR_SD, MPFS_SYSREG_B4_6_7);
  putreg32(LIBERO_SETTING_MSSIO_BANK4_IO_CFG_8_9_CR_SD, MPFS_SYSREG_B4_8_9);
  putreg32(LIBERO_SETTING_MSSIO_BANK4_IO_CFG_10_11_CR_SD,
           MPFS_SYSREG_B4_10_11);
  putreg32(LIBERO_SETTING_MSSIO_BANK4_IO_CFG_12_13_CR_SD,
          MPFS_SYSREG_4_12_13);

  putreg32(1, SDIO_REGISTER_ADDRESS);
#endif
}

/****************************************************************************
 * Name: mpfs_emmc_card_init
 *
 * Description:
 *   Sets the IOMUX values properly for the internal eMMC. No need to access
 *   the SDIO_REGISTER for switching the eMMC/SD because it's zero by
 *   default.
 *
 * Input Parameters:
 *   priv     - Instance of the eMMCSD private state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_emmc_card_init(struct mpfs_dev_s *priv)
{
  putreg32(LIBERO_SETTING_IOMUX1_CR_EMMC, MPFS_SYSREG_IOMUX1);
  putreg32(LIBERO_SETTING_IOMUX2_CR_EMMC, MPFS_SYSREG_IOMUX2);
  putreg32(LIBERO_SETTING_IOMUX6_CR_EMMC, MPFS_SYSREG_IOMUX6);
  putreg32(LIBERO_SETTING_MSSIO_BANK4_CFG_CR_EMMC, MPFS_SYSREG_B4_CFG);
  putreg32(LIBERO_SETTING_MSSIO_BANK4_IO_CFG_0_1_CR_EMMC,
           MPFS_SYSREG_B4_0_1);
  putreg32(LIBERO_SETTING_MSSIO_BANK4_IO_CFG_2_3_CR_EMMC,
           MPFS_SYSREG_B4_2_3);
  putreg32(LIBERO_SETTING_MSSIO_BANK4_IO_CFG_4_5_CR_EMMC,
           MPFS_SYSREG_B4_4_5);
  putreg32(LIBERO_SETTING_MSSIO_BANK4_IO_CFG_6_7_CR_EMMC,
           MPFS_SYSREG_B4_6_7);
  putreg32(LIBERO_SETTING_MSSIO_BANK4_IO_CFG_8_9_CR_EMMC,
           MPFS_SYSREG_B4_8_9);
  putreg32(LIBERO_SETTING_MSSIO_BANK4_IO_CFG_10_11_CR_EMMC,
           MPFS_SYSREG_B4_10_11);
  putreg32(LIBERO_SETTING_MSSIO_BANK4_IO_CFG_12_13_CR_EMMC,
           MPFS_SYSREG_4_12_13);

#ifdef CONFIG_MPFS_EMMCSD_MUX_GPIO
  /* Select eMMC-card */

  mcinfo("Selecting eMMC card\n");
  mpfs_gpiowrite(MPFS_EMMCSD_GPIO, false);

#else
  putreg32(0, SDIO_REGISTER_ADDRESS);
#endif
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
  irqstate_t flags;
  uint32_t regval;
  uint32_t cap;
  uint32_t srs09;
  bool retval = true;
  int status = MPFS_EMMCSD_INITIALIZED;

  flags = enter_critical_section();

  up_disable_irq(priv->plic_irq);

  /* SD card needs FPGA out of reset and FIC3 clks for the eMMC / SD
   * switch.  It's OK if these are already out of reset or clk applied.
   * Also, switching back from SD card to eMMC needs these clocks.
   */

  modifyreg32(MPFS_SYSREG_SOFT_RESET_CR,
              SYSREG_SOFT_RESET_CR_FPGA |
              SYSREG_SOFT_RESET_CR_FIC3,
              0);

  modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR, 0,
              SYSREG_SUBBLK_CLOCK_CR_FIC3);

  if (!priv->emmc)
    {
      /* Apply default HW settings */

      priv->bus_voltage = MPFS_EMMCSD_3_3V_BUS_VOLTAGE;
      priv->bus_speed   = MPFS_EMMCSD_MODE_SDR;
      priv->jumpers_3v3 = true;

      mpfs_sdcard_init(priv);
    }
  else
    {
      /* For the eMMC, use these default values */

      priv->bus_voltage = MPFS_EMMCSD_1_8V_BUS_VOLTAGE;
      priv->jumpers_3v3 = false;

      /* The following defines come from the board.h file */

#if defined(MPFS_EMMC_CLK_200MHZ)
      /* MMCSD_CMD6 with 0x03B90200u isn't used here.  It's just the clk. */

      priv->bus_speed   = MPFS_EMMCSD_MODE_HS200;
#elif defined(MPFS_EMMC_CLK_50MHZ)
      /* MMCSD_CMD6 with argument 0x03B90100u isn't used here. */

      priv->bus_speed   = MPFS_EMMCSD_MODE_SDR;
#else
      priv->bus_speed   = MPFS_EMMCSD_MODE_LEGACY;
#endif

      /* Apply proper IOMUX values for the eMMC. This is required especially
       * if this NuttX works as the system bootloader. Otherwise, it's
       * possible the bootloader has already applied the proper IOMUX values.
       */

      mpfs_emmc_card_init(priv);
    }

  /* Perform system-level reset */

  modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR, 0,
              SYSREG_SUBBLK_CLOCK_CR_MMC);

  modifyreg32(MPFS_SYSREG_SOFT_RESET_CR, 0,
              SYSREG_SOFT_RESET_CR_MMC);

  modifyreg32(MPFS_SYSREG_SOFT_RESET_CR,
              SYSREG_SOFT_RESET_CR_MMC, 0);

  nxsig_sleep(1);

  /* Perform module-level reset */

  modifyreg32(MPFS_EMMCSD_HRS00, 0, MPFS_EMMCSD_HRS00_SWR);

  nxsig_usleep(1000);

  do
    {
      regval = getreg32(MPFS_EMMCSD_HRS00);
    }
  while (regval & MPFS_EMMCSD_HRS00_SWR);

  putreg32(MPFS_EMMCSD_DEBOUNCE_TIME, MPFS_EMMCSD_HRS01);

  modifyreg32(MPFS_EMMCSD_HRS06, MPFS_EMMCSD_HRS06_EMM, 0);

  /* eMMC, not SD */

  if (priv->emmc)
    {
      modifyreg32(MPFS_EMMCSD_HRS06, 0, MPFS_EMMCSD_MODE_LEGACY);
    }

  putreg32(MPFS_EMMCSD_SRS12_STAT_CLEAR, MPFS_EMMCSD_SRS12);

  cap = getreg32(MPFS_EMMCSD_SRS16);

  /* DMA 64 bit support? */

  if (cap & MPFS_EMMCSD_SRS16_A64S)
    {
      modifyreg32(MPFS_EMMCSD_SRS15, 0, MPFS_EMMCSD_SRS15_A64B |
                  MPFS_EMMCSD_SRS15_HV4E);
    }

#ifdef CONFIG_SDIO_DMA
  /* Check if SDMA is supported */

  if (!(cap & MPFS_EMMCSD_SRS16_DMAS))
    {
      mcerr("DMA not supported!\n");
      DEBUGPANIC();
    }

  uint64_t pmpcfg_mmc_x;

  /* DMA will not work with the power-on default PMPCFG values.
   * Check that the HSS or envm bootloader has applied the
   * following values below, provide info if not.
   */

  pmpcfg_mmc_x = getreg64(MPFS_PMPCFG_MMC_0);
  if ((pmpcfg_mmc_x & 0x1ffffff000000000) != 0x1f00000000000000)
    {
      mcinfo("Please check PMPCFG_MMC0 register.\n");
    }

  pmpcfg_mmc_x = getreg64(MPFS_PMPCFG_MMC_1);
  if ((pmpcfg_mmc_x & 0x1ffffff000000000) != 0x1f00000000000000)
    {
      mcinfo("Please check PMPCFG_MMC1 register.\n");
    }

  pmpcfg_mmc_x = getreg64(MPFS_PMPCFG_MMC_2);
  if ((pmpcfg_mmc_x & 0x1ffffff000000000) != 0x1f00000000000000)
    {
      mcinfo("Please check PMPCFG_MMC2 register.\n");
    }

  pmpcfg_mmc_x = getreg64(MPFS_PMPCFG_MMC_3);
  if ((pmpcfg_mmc_x & 0x1ffffff000000000) != 0x1f00000000000000)
    {
      mcinfo("Please check PMPCFG_MMC3 register.\n");
    }

#endif

  /* Clear interrupt status and disable interrupts */

  putreg32(MPFS_EMMCSD_SRS13_STATUS_EN, MPFS_EMMCSD_SRS13);
  putreg32(0, MPFS_EMMCSD_SRS14);

  mpfs_set_data_timeout(priv, MPFS_EMMCSD_DATA_TIMEOUT);

  /* Turn off host controller power */

  mpfs_set_sdhost_power(priv, 0);

  /* Card state stable */

  srs09 = getreg32(MPFS_EMMCSD_SRS09);
  DEBUGASSERT(srs09 & MPFS_EMMCSD_SRS09_CSS);

  if (!priv->emmc)
    {
      if (!(srs09 & MPFS_EMMCSD_SRS09_CI))
        {
          mcerr("Please insert the SD card!\n");

          /* No card detected, cannot communicate with it */

          retval = false;
        }
    }

  /* Set 1-bit bus mode */

  modifyreg32(MPFS_EMMCSD_SRS10,
              MPFS_EMMCSD_SRS10_DTW | MPFS_EMMCSD_SRS10_EDTW,
              0);

  if (priv->emmc)
    {
      switch (priv->bus_voltage)
        {
          case MPFS_EMMCSD_1_8V_BUS_VOLTAGE:
            mpfs_set_sdhost_power(priv, MPFS_EMMCSD_SRS10_1_8V_BUS_VOLTAGE);
            break;

          case MPFS_EMMCSD_3_3V_BUS_VOLTAGE:
            if ((priv->bus_speed != MPFS_EMMCSD_MODE_HS200) &&
                (priv->bus_speed != MPFS_EMMCSD_MODE_HS400_ES) &&
                (priv->bus_speed != MPFS_EMMCSD_MODE_HS400))
              {
                mpfs_set_sdhost_power(priv,
                                      MPFS_EMMCSD_SRS10_3_3V_BUS_VOLTAGE);
              }
              else
              {
                status = MPFS_EMMCSD_NOT_INITIALIZED;
                mcerr("Voltage / mode combination not supported!\n");
              }
            break;

          default:
            DEBUGPANIC();
        }
    }
  else
    {
      /* SD-card support currently only 3.3v */

       mpfs_set_sdhost_power(priv, MPFS_EMMCSD_SRS10_3_3V_BUS_VOLTAGE);
    }

  if (status == MPFS_EMMCSD_INITIALIZED)
    {
      mpfs_setclkrate(priv, MPFS_MMC_CLOCK_400KHZ);
    }

  nxsig_usleep(1000);

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

  leave_critical_section(flags);

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

  /* This reverses the logic for mmcsd_writesingle(): setup first, then
   * the command is sent. Normally the command is sent first and the setup
   * via SDIO_SENDSETUP().
   */

  caps |= SDIO_CAPS_DMABEFOREWRITE;

#ifdef CONFIG_SDIO_DMA
  caps |= SDIO_CAPS_DMASUPPORTED;
#endif

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

  if (wide)
    {
      modifyreg32(MPFS_EMMCSD_SRS10, 0, MPFS_EMMCSD_SRS10_DTW);
    }
  else
    {
      modifyreg32(MPFS_EMMCSD_SRS10, MPFS_EMMCSD_SRS10_DTW, 0);
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
      if (priv->bus_speed == MPFS_EMMCSD_MODE_HS200)
        {
          clckr = MPFS_MMC_CLOCK_200MHZ;
        }
      else if (priv->bus_speed == MPFS_EMMCSD_MODE_SDR)
        {
          clckr = MPFS_MMC_CLOCK_50MHZ;
        }
      else
        {
          clckr = MPFS_MMC_CLOCK_26MHZ;
        }
      break;

    /* SD normal operation clocking (wide 4-bit mode) */

    case CLOCK_SD_TRANSFER_4BIT:
      clckr = MPFS_MMC_CLOCK_25MHZ;
      break;

    /* SD normal operation clocking (narrow 1-bit mode) */

    case CLOCK_SD_TRANSFER_1BIT:
      clckr = MPFS_MMC_CLOCK_25MHZ;
      break;
  }

  /* Set the new clock frequency */

  mpfs_setclkrate(priv, clckr);
}

/****************************************************************************
 * Name: mpfs_attach
 *
 * Description:
 *   Attach and prepare interrupts
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK on success; A negated errno on failure.
 *
 ****************************************************************************/

static int mpfs_attach(struct sdio_dev_s *dev)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
  int ret;

  /* Attach the SDIO interrupt handler */

  ret = irq_attach(priv->plic_irq, mpfs_emmcsd_interrupt, priv);
  if (ret == OK)
    {
      /* Disable all interrupts at the SDIO controller and clear
       * interrupt flags, except current limit error, card interrupt,
       * card removal and card insertion
       */

      modifyreg32(MPFS_EMMCSD_SRS12, MPFS_EMMCSD_SRS12_ECL |
                                     MPFS_EMMCSD_SRS12_CINT |
                                     MPFS_EMMCSD_SRS12_CR |
                                     MPFS_EMMCSD_SRS12_CIN,
                                     0);

      /* Enable SDIO interrupts. They can now be enabled at the
       * SDIO controller as needed.
       */

      up_enable_irq(priv->plic_irq);

      /* Enable card insertion and removal interrupts */

      putreg32(MPFS_EMMCSD_CARD_INTS, MPFS_EMMCSD_SRS14);
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
  uint32_t command_information;
  uint32_t cmdidx;

  /* Check if command / data lines are busy */

  if (mpfs_check_lines_busy(priv))
    {
      mcerr("Busy!\n");
      return -EBUSY;
    }

  /* Clear all status interrupts */

  putreg32(MPFS_EMMCSD_SRS12_STAT_CLEAR, MPFS_EMMCSD_SRS12);

  /* Check response type */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
      case MMCSD_R2_RESPONSE:
        command_information = MPFS_EMMCSD_SRS03_RESP_L136 |
                              MPFS_EMMCSD_SRS03_CRCCE;
        break;
      case MMCSD_R3_RESPONSE:
      case MMCSD_R4_RESPONSE:
        command_information = MPFS_EMMCSD_SRS03_RESP_L48;
        break;
      case MMCSD_R1_RESPONSE:
      case MMCSD_R5_RESPONSE:
      case MMCSD_R6_RESPONSE:
      case MMCSD_R7_RESPONSE:
        command_information = MPFS_EMMCSD_SRS03_RESP_L48 |
                              MPFS_EMMCSD_SRS03_CRCCE |
                              MPFS_EMMCSD_SRS03_CICE;
        break;
      case MMCSD_R1B_RESPONSE:
        command_information = MPFS_EMMCSD_SRS03_RESP_L48B |
                              MPFS_EMMCSD_SRS03_CRCCE |
                              MPFS_EMMCSD_SRS03_CICE;
        break;
      case MMCSD_NO_RESPONSE:
        command_information = MPFS_EMMCSD_SRS03_NO_RESPONSE;
        break;
      default:
        return -EINVAL;
    }

  putreg32(arg, MPFS_EMMCSD_SRS02);

  cmdidx = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;

  if (cmd & MMCSD_DATAXFR_MASK)
    {
      command_information |= MPFS_EMMCSD_SRS03_DPS |
                             MPFS_EMMCSD_SRS03_BCE |
                             MPFS_EMMCSD_SRS03_RECE;

#ifdef CONFIG_SDIO_DMA
      if (!priv->polltransfer)
        {
          command_information |= MPFS_EMMCSD_SRS03_DMAE;
        }
#endif

      if ((cmd & MMCSD_DATAXFR_MASK) == MMCSD_RDDATAXFR)
        {
          command_information |= MPFS_EMMCSD_SRS03_DTDS;
          mcinfo("cmd & MMCSD_RDDATAXFR\n");
        }

      if (cmd & MMCSD_MULTIBLOCK)
        {
          command_information |= MPFS_EMMCSD_SRS03_MSBS;
        }

      mcinfo("cmd & MMCSD_DATAXFR_MASK\n");
    }

  putreg32((((cmdidx << 24) & MPFS_EMMCSD_SRS03_CIDX) | command_information),
           MPFS_EMMCSD_SRS03);

  mcinfo("sendcmd: %08" PRIx32 " cmd: %08" PRIx32 " arg: %08" PRIx32
         " cmdidx: %08" PRIx32 "\n",
         command_information, cmd, arg, cmdidx);

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

#ifdef CONFIG_SDIO_BLOCKSETUP
static void mpfs_blocksetup(struct sdio_dev_s *dev,
                             unsigned int blocksize, unsigned int nblocks)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;

  priv->blocksize = blocksize;

  putreg32(priv->blocksize | (nblocks << 16) |
           MPFS_EMMCSD_SRS01_DMA_SZ_512KB, MPFS_EMMCSD_SRS01);
}
#endif

/****************************************************************************
 * Name: mpfs_recvsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card in non-DMA
 *   (interrupt driven mode).  This method will do whatever controller setup
 *   is necessary.  This would be called for SD memory just BEFORE sending
 *   CMD13 (SEND_STATUS), CMD17 (READ_SINGLE_BLOCK), CMD18
 *   (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR), etc. Normally,
 *   EMMCSD_WAITEVENT will be called to receive the indication that the
 *   transfer is complete.
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

  priv->buffer       = (uint32_t *)buffer;
  priv->remaining    = nbytes;
  priv->receivecnt   = nbytes;
  priv->polltransfer = true;

  mpfs_check_lines_busy(priv);

  /* Set up the SDIO data path, reset DAT and CMD lines */

  mpfs_reset_lines(priv);

#ifndef CONFIG_SDIO_BLOCKSETUP
  uint32_t blockcount = ((nbytes - 1) / priv->blocksize) + 1;
  putreg32(priv->blocksize | (blockcount << 16), MPFS_EMMCSD_SRS01);
#endif

  putreg32(MPFS_EMMCSD_SRS13_STATUS_EN, MPFS_EMMCSD_SRS13);
  putreg32(MPFS_EMMCSD_SRS12_STAT_CLEAR, MPFS_EMMCSD_SRS12);

  /* Enable interrupts */

  mpfs_configxfrints(priv, MPFS_EMMCSD_RECV_MASK);

  return OK;
}

/****************************************************************************
 * Name: mpfs_sendsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card.  This
 *   method will do whatever controller setup is necessary. This would be
 *   called for SD memory just BEFORE sending CMD24 (WRITE_BLOCK), CMD25
 *   (WRITE_MULTIPLE_BLOCK).
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
  DEBUGASSERT(((uintptr_t)buffer & 3) == 0);

  mpfs_check_lines_busy(priv);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer       = (uint32_t *)buffer;
  priv->remaining    = nbytes;
  priv->receivecnt   = 0;
  priv->polltransfer = true;

#ifndef CONFIG_SDIO_BLOCKSETUP
  uint32_t blockcount = ((nbytes - 1) / priv->blocksize) + 1;
  putreg32(priv->blocksize | (blockcount << 16), MPFS_EMMCSD_SRS01);
#endif

  putreg32(MPFS_EMMCSD_SRS13_STATUS_EN, MPFS_EMMCSD_SRS13);
  putreg32(~(MPFS_EMMCSD_SRS12_BWR), MPFS_EMMCSD_SRS12);

  /* Enable interrupts */

  mpfs_configxfrints(priv, MPFS_EMMCSD_SEND_MASK);

  return OK;
}

/****************************************************************************
 * Name: mpfs_dmarecvsetup
 *
 * Description:
 *   Setup to perform a read DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent. For read transfers this may mean
 *   invalidating the data cache. No cache support is currently implemented.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA from
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static int mpfs_dmarecvsetup(struct sdio_dev_s *dev,
                              uint8_t *buffer, size_t buflen)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
#ifndef CONFIG_SDIO_BLOCKSETUP
  uint32_t blockcount;
#endif

  mcinfo("Receive: %zu bytes\n", buflen);

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uintptr_t)buffer & 3) == 0);

  priv->buffer       = (uint32_t *)buffer;
  priv->remaining    = buflen;
  priv->receivecnt   = buflen;
  priv->polltransfer = false;

  /* Set up the SDIO data path, reset DAT and CMD lines */

  mpfs_reset_lines(priv);

  modifyreg32(MPFS_EMMCSD_SRS10, MPFS_EMMCSD_SRS10_DMASEL, 0);

  putreg32((uintptr_t)buffer, MPFS_EMMCSD_SRS22);
  putreg32((uintptr_t)((uint64_t)buffer >> 32), MPFS_EMMCSD_SRS23);
#ifndef CONFIG_SDIO_BLOCKSETUP
  blockcount = ((buflen - 1) / priv->blocksize) + 1;

  putreg32((priv->blocksize | (blockcount << 16) |
           MPFS_EMMCSD_SRS01_DMA_SZ_512KB), MPFS_EMMCSD_SRS01);
#endif

  /* Clear interrupt status */

  putreg32(MPFS_EMMCSD_SRS12_STAT_CLEAR, MPFS_EMMCSD_SRS12);

  mpfs_configxfrints(priv, MPFS_EMMCSD_RECV_MASKDMA);

  return OK;
}
#endif

/****************************************************************************
 * Name: mpfs_dmasendsetup
 *
 * Description:
 *   Setup to perform a write DMA. No cache support is currently implemented.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA into
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static int mpfs_dmasendsetup(struct sdio_dev_s *dev,
                              const uint8_t *buffer, size_t buflen)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;
#ifndef CONFIG_SDIO_BLOCKSETUP
  uint32_t blockcount;
#endif

  mcinfo("Send: %zu bytes\n", buflen);

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uintptr_t)buffer & 3) == 0);

  /* DMA send doesn't work in 0x08xxxxxxx address range. Default to IRQ mode
   * in this special case.
   */

  if (((uintptr_t)buffer & 0xff000000) == 0x08000000)
    {
      return mpfs_sendsetup(dev, buffer, buflen);
    }

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer       = (uint32_t *)buffer;
  priv->remaining    = buflen;
  priv->receivecnt   = 0;
  priv->polltransfer = false;

  modifyreg32(MPFS_EMMCSD_SRS10, MPFS_EMMCSD_SRS10_DMASEL, 0);

  putreg32((uintptr_t)buffer, MPFS_EMMCSD_SRS22);
  putreg32((uintptr_t)((uint64_t)buffer >> 32), MPFS_EMMCSD_SRS23);

#ifndef CONFIG_SDIO_BLOCKSETUP
  blockcount = ((buflen - 1) / priv->blocksize) + 1;

  putreg32((priv->blocksize | (blockcount << 16) |
            MPFS_EMMCSD_SRS01_DMA_SZ_512KB),
            MPFS_EMMCSD_SRS01);
#endif

  /* Clear interrupt status */

  putreg32(MPFS_EMMCSD_SRS12_STAT_CLEAR, MPFS_EMMCSD_SRS12);

  mpfs_configxfrints(priv, MPFS_EMMCSD_SEND_MASKDMA);

  return OK;
}
#endif

/****************************************************************************
 * Name: mpfs_cancel
 *
 * Description:
 *   Cancel the data transfer setup of EMMCSD_RECVSETUP, EMMCSD_SENDSETUP,
 *   EMMCSD_DMARECVSETUP or EMMCSD_DMASENDSETUP.  This must be called to
 *   cancel the data transfer setup if, for some reason, you cannot perform
 *   the transfer.
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

  mpfs_configxfrints(priv, 0);
  mpfs_configwaitints(priv, 0, 0, 0);

  /* If this was a DMA transfer, make sure that DMA is stopped */

  modifyreg32(MPFS_EMMCSD_SRS03, MPFS_EMMCSD_SRS03_DMAE, 0);

  /* Clearing pending interrupt status on all transfer- and event- related
   * interrupts
   */

  putreg32(MPFS_EMMCSD_WAITALL_ICR, MPFS_EMMCSD_SRS12);

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
  uint32_t status;
  int32_t timeout;

  mcinfo("cmd: %08" PRIx32 "\n", cmd);

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
      case MMCSD_NO_RESPONSE:
        timeout = EMMCSD_CMDTIMEOUT;
        break;

      case MMCSD_R1_RESPONSE:
      case MMCSD_R1B_RESPONSE:
      case MMCSD_R2_RESPONSE:
      case MMCSD_R4_RESPONSE:
      case MMCSD_R5_RESPONSE:
      case MMCSD_R6_RESPONSE:
        timeout = EMMCSD_LONGTIMEOUT;
        break;

      case MMCSD_R3_RESPONSE:
      case MMCSD_R7_RESPONSE:
        timeout = EMMCSD_CMDTIMEOUT;
        break;

      default:
        mcerr("Unknown command\n");
        return -EINVAL;
    }

  /* Then wait for the response (or timeout) */

  do
    {
      status = getreg32(MPFS_EMMCSD_SRS12);
    }
  while (!(status & (MPFS_EMMCSD_SRS12_CC | MPFS_EMMCSD_SRS12_EINT))
         && --timeout);

  if (timeout == 0 || (status & MPFS_EMMCSD_SRS12_ECT))
    {
      mcerr("ERROR: Timeout cmd: %08" PRIx32 " stat: %08" PRIx32 "\n", cmd,
            status);
      return -EBUSY;
    }

  mcinfo("status: %08" PRIx32 "\n", status);

  return OK;
}

/****************************************************************************
 * Name: mpfs_check_recverror
 *
 * Description:
 *   Receive response to SDIO command.
 *
 * Input Parameters:
 *   priv    - Instance of the EMMCSD private state structure.
 *
 * Returned Value:
 *   OK on success; a negated errno if error detected.
 *
 ****************************************************************************/

static int mpfs_check_recverror(struct mpfs_dev_s *priv)
{
  uint32_t regval;
  int ret = OK;

  regval = getreg32(MPFS_EMMCSD_SRS12);

  if (regval & MPFS_EMMCSD_SRS12_EINT)
    {
      if (regval & (MPFS_EMMCSD_SRS12_ECT | MPFS_EMMCSD_SRS12_EDT))
        {
          mcerr("ERROR: Command timeout: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & MPFS_EMMCSD_SRS12_EDCRC)
        {
          mcerr("ERROR: CRC failure: %08" PRIx32 "\n", regval);
          ret = -EIO;
        }
      else
        {
          mcerr("ERROR: %08" PRIx32 "\n", regval);
          ret = -EIO;
        }
    }
  else if (!(regval & MPFS_EMMCSD_SRS12_CC))
    {
      mcerr("ERROR: Command not completed: %08" PRIx32 "\n", regval);
      ret = -EIO;
    }

  if (ret)
    {
      /* With an error, reset DAT and CMD lines. Otherwise the next command
       * will fail as well.
       */

      mpfs_reset_lines(priv);

      /* Clear all status interrupts */

      putreg32(MPFS_EMMCSD_SRS12_STAT_CLEAR, MPFS_EMMCSD_SRS12);
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
  int ret = OK;

  /* Check if a timeout or CRC error occurred */

  if (!(cmd & MMCSD_DATAXFR_MASK)) /* TBD: Fix this bypass! */
    {
      ret = mpfs_check_recverror(priv);
    }

  if (rshort)
    {
      *rshort = getreg32(MPFS_EMMCSD_SRS04);
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
  int ret = OK;

  if (!(cmd & MMCSD_DATAXFR_MASK))
    {
      ret = mpfs_check_recverror(priv);
    }

  if (rshort)
    {
      *rshort = getreg32(MPFS_EMMCSD_SRS04);
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
      /* Last 8-bits are missing, see SRS04 documemntation, RESP3[23:0]
       * has only 24 bits unlike RESP2, RESP1 and RESP0 that have 32 bits.
       * We have to shift left 8 bits to match the proper long response.
       */

      rlong[3] = getreg32(MPFS_EMMCSD_SRS04) << 8;
      rlong[2] = getreg32(MPFS_EMMCSD_SRS05) << 8;
      rlong[1] = getreg32(MPFS_EMMCSD_SRS06) << 8;
      rlong[0] = getreg32(MPFS_EMMCSD_SRS07) << 8;

      mcinfo("recv: %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" \
             PRIx32"\n", rlong[0], rlong[1], rlong[2], rlong[3]);
    }

  return ret;
}

/****************************************************************************
 * Name: mpfs_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the EMMCSD_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling mpfs_eventwait.  This is done in this way
 *   to help the driver to eliminate race conditions between the command
 *   setup and the subsequent events.
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

static void mpfs_waitenable(struct sdio_dev_s *dev,
                            sdio_eventset_t eventset, uint32_t timeout)
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
      waitmask |= MPFS_EMMCSD_CMDDONE_MASK;
    }

  if ((eventset & SDIOWAIT_RESPONSEDONE) != 0)
    {
      waitmask |= MPFS_EMMCSD_RESPDONE_MASK;
    }

  if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
    {
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
  irqstate_t flags;
  int ret;

  mcinfo("wait\n");

  /* There is a race condition here... the event may have completed before
   * we get here.  In this case waitevents will be zero, but wkupevents will
   * be non-zero (and, hopefully, the semaphore count will also be non-zero.
   */

  flags = enter_critical_section();

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

      ret = mpfs_takesem(priv);
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

  leave_critical_section(flags);
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
 *   change.  Callbacks should not be made from interrupt handlers, rather
 *   interrupt level events should be handled by calling back on the work
 *   thread.
 *
 *   When this method is called, all callbacks should be disabled until they
 *   are enabled via a call to EMMCSD_CALLBACKENABLE
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
 * Name: sdio_initialize
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

struct sdio_dev_s *sdio_initialize(int slotno)
{
  struct mpfs_dev_s *priv = NULL;
  priv = &g_emmcsd_dev;

  /* Initialize semaphores */

  nxsem_init(&priv->waitsem, 0, 0);

  /* The waitsem semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_set_protocol(&priv->waitsem, SEM_PRIO_NONE);

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
 * Name: sdio_mediachange
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

void sdio_mediachange(struct sdio_dev_s *dev, bool cardinslot)
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
 * Name: sdio_wrprotect
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

void sdio_wrprotect(struct sdio_dev_s *dev, bool wrprotect)
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
