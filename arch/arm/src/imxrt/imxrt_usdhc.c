/****************************************************************************
 * arch/arm/src/imxrt/imxrt_usdhc.c
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

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/mmcsd.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "imxrt_config.h"
#include "imxrt_gpio.h"
#include "hardware/imxrt_pinmux.h"
#include "hardware/imxrt_ccm.h"
#include "imxrt_periphclks.h"
#include "hardware/imxrt_usdhc.h"

#ifdef CONFIG_IMXRT_USDHC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if ((defined(CONFIG_IMXRT_USDHC1) && !defined(CONFIG_IMXRT_USDHC2)) || \
     (defined(CONFIG_IMXRT_USDHC2) && !defined(CONFIG_IMXRT_USDHC1)))
#  define  IMXRT_MAX_SDHC_DEV_SLOTS  1
#elif (defined(CONFIG_IMXRT_USDHC1) && defined(CONFIG_IMXRT_USDHC2))
#  define  IMXRT_MAX_SDHC_DEV_SLOTS  2
#else
#error Unrecognised number of SDHC slots
#endif

#if !defined(CONFIG_IMXRT_USDHC_DMA)
#  warning "Large Non-DMA transfer may result in RX overrun failures"
#elif !defined(CONFIG_SDIO_DMA)
#  warning CONFIG_SDIO_DMA should be defined with CONFIG_IMXRT_USDHC_DMA
#endif

#if !defined(CONFIG_SCHED_WORKQUEUE) || !defined(CONFIG_SCHED_HPWORK)
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE and CONFIG_SCHED_HPWORK"
#endif

#if !defined(CONFIG_SDIO_BLOCKSETUP)
#  error "CONFIG_SDIO_BLOCKSETUP is mandatory for this driver"
#endif

#ifndef CONFIG_DEBUG_MEMCARD_INFO
#  undef CONFIG_SDIO_XFRDEBUG
#endif

/* Timing in ms for commands wait response */

#define USDHC_CMDTIMEOUT            MSEC2TICK(100)
#define USDHC_LONGTIMEOUT           MSEC2TICK(500)

/* Big DTOCV setting.  Range is 0 = SDCLK * 2^13 through 15 = SDCLK * 2^29 */

#define USDHC_DTOCV_MAXTIMEOUT      (15)

/* Maximum watermark value */

#define USDHC_MAX_WATERMARK         128

/* Block size for multi-block transfers */

#define SDMMC_MAX_BLOCK_SIZE        (512)

/* Data transfer / Event waiting interrupt mask bits */

#define USDHC_RESPERR_INTS          (USDHC_INT_CCE | USDHC_INT_CTOE | \
                                     USDHC_INT_CEBE | USDHC_INT_CIE)
#define USDHC_RESPDONE_INTS         (USDHC_RESPERR_INTS | USDHC_INT_CC)

#define USDHC_XFRERR_INTS           (USDHC_INT_DCE | USDHC_INT_DTOE | \
                                     USDHC_INT_DEBE)
#define USDHC_RCVDONE_INTS          (USDHC_XFRERR_INTS | USDHC_INT_BRR | \
                                     USDHC_INT_TC)
#define USDHC_SNDDONE_INTS          (USDHC_XFRERR_INTS | USDHC_INT_BWR | \
                                     USDHC_INT_TC)
#define USDHC_XFRDONE_INTS          (USDHC_XFRERR_INTS | USDHC_INT_BRR | \
                                     USDHC_INT_BWR | USDHC_INT_TC)

/* CD Detect Types */

/* For DMA operations DINT is not interesting TC will indicate completions */

#define USDHC_DMAERR_INTS           (USDHC_XFRERR_INTS | USDHC_INT_DMAE)
#define USDHC_DMADONE_INTS          (USDHC_DMAERR_INTS | USDHC_INT_TC)

#define USDHC_WAITALL_INTS          (USDHC_RESPDONE_INTS | \
                                     USDHC_XFRDONE_INTS |  \
                                     USDHC_DMADONE_INTS)

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
#  define DBG_BASE_ADDR             IMXRT_USDHC1_BASE
#  define SAMPLENDX_BEFORE_SETUP    0
#  define SAMPLENDX_AFTER_SETUP     1
#  define SAMPLENDX_END_TRANSFER    2
#  define DEBUG_NSAMPLES            3
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the state of the Imxrt SDIO interface */

struct imxrt_dev_s
{
  struct sdio_dev_s dev;              /* Standard, base SDIO interface */

  /* Imxrt-specific extensions */

  /* Event support */

  sem_t waitsem;                      /* Implements event waiting */
  sdio_eventset_t waitevents;         /* Set of events to be waited for */
  uint32_t waitints;                  /* Interrupt enables for event waiting */
  volatile sdio_eventset_t wkupevent; /* The event that caused the wakeup */
  struct wdog_s waitwdog;             /* Watchdog that handles event timeouts */

  /* Callback support */

  sdio_statset_t cdstatus;            /* Card status */
  sdio_eventset_t cbevents;           /* Set of events to be cause callbacks */
  worker_t callback;                  /* Registered callback function */
  void *cbarg;                        /* Registered callback argument */
  struct work_s cbwork;               /* Callback work queue structure */

  /* Interrupt mode data transfer support */

  uint32_t *buffer;                   /* Address of current R/W buffer */
  size_t remaining;                   /* Number of bytes remaining in the
                                       * transfer */
  uint32_t xfrints;                   /* Interrupt enables for data transfer */

#ifdef CONFIG_IMXRT_USDHC_DMA
  /* DMA data transfer support */

  volatile uint8_t xfrflags;          /* Used to synchronize SDIO and DMA
                                       * completion */
                                      /* DMA buffer for unaligned transfers */
#if defined(CONFIG_ARMV7M_DCACHE)
  uint32_t blocksize;                 /* Current block size */
  uint8_t  rxbuffer[SDMMC_MAX_BLOCK_SIZE]
                   __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
  bool     unaligned_rx;              /* buffer is not cache-line aligned */
#endif
#endif

  /* Card interrupt support for SDIO */

  uint32_t cintints;                  /* Interrupt enables for card ints */
  int (*do_sdio_card)(void *);        /* SDIO card ISR */
  void *do_sdio_arg;                  /* arg for SDIO card ISR */

  uint32_t addr;                      /* Base address of this instances */
  uint32_t sw_cd_gpio;                /* If a non USDHCx CD pin is used,
                                       * this is its GPIO */
  uint32_t cd_invert;                 /* If true invert the CD pin */
};

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
struct imxrt_sdhcregs_s
{
  /* All read-able USDHC registers */

  uint32_t dsaddr;               /* DMA System Address Register */
  uint32_t blkattr;              /* Block Attributes Register */
  uint32_t cmdarg;               /* Command Argument Register */
  uint32_t xferty;               /* Transfer Type Register */
  uint32_t cmdrsp0;              /* Command Response 0 */
  uint32_t cmdrsp1;              /* Command Response 1 */
  uint32_t cmdrsp2;              /* Command Response 2 */
  uint32_t cmdrsp3;              /* Command Response 3 */
  uint32_t dbap;                 /* Data buffer access port */
  uint32_t prsstat;              /* Present State Register */
  uint32_t proctl;               /* Protocol Control Register */
  uint32_t sysctl;               /* System Control Register */
  uint32_t irqstat;              /* Interrupt Status Register */
  uint32_t irqstaten;            /* Interrupt Status Enable Register */
  uint32_t irqsigen;             /* Interrupt Signal Enable Register */
  uint32_t ac12err;              /* Auto CMD12 Error Status Register */
  uint32_t htcapblt;             /* Host Controller Capabilities */
  uint32_t wml;                  /* Watermark Level Register */
  uint32_t mixctrl;              /* Mixer Control */
  uint32_t fevent;               /* Force Event */
  uint32_t admaes;               /* ADMA Error Status Register */
  uint32_t adsaddr;              /* ADMA System Address Register */
  uint32_t dllctrl;              /* Delay line control */
  uint32_t dllstat;              /* Delay line status */
  uint32_t clktune;              /* Clock tune and control */
  uint32_t vendor;               /* Vendor Specific Register */
  uint32_t mmcboot;              /* MMC Boot Register */
  uint32_t vendor2;              /* Vendor Specific Register 2 */
  uint32_t tuningctrl;           /* Tuning Control */
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static void imxrt_configwaitints(struct imxrt_dev_s *priv, uint32_t waitints,
              sdio_eventset_t waitevents, sdio_eventset_t wkupevents);
static void imxrt_configxfrints(struct imxrt_dev_s *priv, uint32_t xfrints);

/* DMA Helpers **************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void imxrt_sampleinit(void);
static void imxrt_sdhcsample(struct imxrt_sdhcregs_s *regs);
static void imxrt_sample(struct imxrt_dev_s *priv, int index);
static void imxrt_dumpsample(struct imxrt_dev_s *priv,
              struct imxrt_sdhcregs_s *regs, const char *msg);
static void imxrt_dumpsamples(struct imxrt_dev_s *priv);
static void imxrt_showregs(struct imxrt_dev_s *priv, const char *msg);

#else
#  define   imxrt_sampleinit()
#  define   imxrt_sample(priv, index)
#  define   imxrt_dumpsamples(priv)
#  define   imxrt_showregs(priv, msg)
#endif

/* Data Transfer Helpers ****************************************************/

static void imxrt_dataconfig(struct imxrt_dev_s *priv, bool bwrite,
              unsigned int datalen, unsigned int timeout);

#ifndef CONFIG_IMXRT_USDHC_DMA
static void imxrt_transmit(struct imxrt_dev_s *priv);
static void imxrt_receive(struct imxrt_dev_s *priv);
#if defined(CONFIG_ARMV7M_DCACHE)
static void imxrt_recvdma(struct imxrt_dev_s *priv);
#endif
#endif

static void imxrt_eventtimeout(wdparm_t arg);
static void imxrt_endwait(struct imxrt_dev_s *priv,
              sdio_eventset_t wkupevent);
static void imxrt_endtransfer(struct imxrt_dev_s *priv,
              sdio_eventset_t wkupevent);

/* Interrupt Handling *******************************************************/

static int  imxrt_interrupt(int irq, void *context, void *arg);

/* SDIO interface methods ***************************************************/

/* Mutual exclusion */

#ifdef CONFIG_SDIO_MUXBUS
static int imxrt_lock(struct sdio_dev_s *dev, bool lock);
#endif

/* Initialization/setup */

static void imxrt_reset(struct sdio_dev_s *dev);
static sdio_capset_t imxrt_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t imxrt_status(struct sdio_dev_s *dev);
static void imxrt_widebus(struct sdio_dev_s *dev, bool enable);

#ifdef CONFIG_IMXRT_USDHC_ABSFREQ
static void imxrt_frequency(struct sdio_dev_s *dev, uint32_t frequency);
#endif

static void imxrt_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate);
static int  imxrt_attach(struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int  imxrt_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t arg);

#ifdef CONFIG_SDIO_BLOCKSETUP
static void imxrt_blocksetup(struct sdio_dev_s *dev,
              unsigned int blocklen, unsigned int nblocks);
#endif

#ifndef CONFIG_IMXRT_USDHC_DMA
static int  imxrt_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
              size_t nbytes);
static int  imxrt_sendsetup(struct sdio_dev_s *dev,
              const uint8_t *buffer, size_t nbytes);
#endif

static int  imxrt_cancel(struct sdio_dev_s *dev);
static int  imxrt_waitresponse(struct sdio_dev_s *dev, uint32_t cmd);
static int  imxrt_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  imxrt_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t rlong[4]);
static int  imxrt_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);

/* EVENT handler */

static void imxrt_waitenable(struct sdio_dev_s *dev,
              sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t imxrt_eventwait(struct sdio_dev_s *dev);
static void imxrt_callbackenable(struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static int  imxrt_registercallback(struct sdio_dev_s *dev,
              worker_t callback, void *arg);

/* DMA */

#ifdef CONFIG_IMXRT_USDHC_DMA
static int  imxrt_dmarecvsetup(struct sdio_dev_s *dev,
              uint8_t *buffer, size_t buflen);
static int  imxrt_dmasendsetup(struct sdio_dev_s *dev,
              const uint8_t *buffer, size_t buflen);
#endif

/* Initialization/uninitialization/reset ************************************/

static void imxrt_callback(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct imxrt_dev_s g_sdhcdev[IMXRT_MAX_SDHC_DEV_SLOTS] =
{
#ifdef CONFIG_IMXRT_USDHC1
  {
    .addr               = IMXRT_USDHC1_BASE,
#if defined(PIN_USDHC1_CD_GPIO)
    .sw_cd_gpio         = PIN_USDHC1_CD_GPIO,
#endif
#if defined(CONFIG_IMXRT_USDHC1_INVERT_CD)
    .cd_invert         = true,
#endif
    .dev                =
    {
#ifdef CONFIG_SDIO_MUXBUS
      .lock             = imxrt_lock,
#endif
      .reset            = imxrt_reset,
      .capabilities     = imxrt_capabilities,
      .status           = imxrt_status,
      .widebus          = imxrt_widebus,
      .clock            = imxrt_clock,
      .attach           = imxrt_attach,
      .sendcmd          = imxrt_sendcmd,
#ifdef CONFIG_SDIO_BLOCKSETUP
      .blocksetup       = imxrt_blocksetup,
#endif

#ifndef CONFIG_IMXRT_USDHC_DMA
      .recvsetup        = imxrt_recvsetup,
      .sendsetup        = imxrt_sendsetup,
#else
      .recvsetup        = imxrt_dmarecvsetup,
      .sendsetup        = imxrt_dmasendsetup,
#endif
      .cancel           = imxrt_cancel,
      .waitresponse     = imxrt_waitresponse,
      .recv_r1          = imxrt_recvshortcrc,
      .recv_r2          = imxrt_recvlong,
      .recv_r3          = imxrt_recvshort,
      .recv_r4          = imxrt_recvshort,
      .recv_r5          = imxrt_recvshortcrc,
      .recv_r6          = imxrt_recvshortcrc,
      .recv_r7          = imxrt_recvshort,
      .waitenable       = imxrt_waitenable,
      .eventwait        = imxrt_eventwait,
      .callbackenable   = imxrt_callbackenable,
      .registercallback = imxrt_registercallback,
#ifdef CONFIG_SDIO_DMA
#ifdef CONFIG_IMXRT_USDHC_DMA
      .dmarecvsetup     = imxrt_dmarecvsetup,
      .dmasendsetup     = imxrt_dmasendsetup,
#else
      .dmarecvsetup     = imxrt_recvsetup,
      .dmasendsetup     = imxrt_sendsetup,
#endif
#endif
    },
    .waitsem = SEM_INITIALIZER(0),
  },
#endif

#ifdef CONFIG_IMXRT_USDHC2
  {
    .addr               = IMXRT_USDHC2_BASE,
#if defined(PIN_USDHC2_CD_GPIO)
    .sw_cd_gpio         = PIN_USDHC2_CD_GPIO,
#endif
#if defined(CONFIG_IMXRT_USDHC2_INVERT_CD)
    .cd_invert         = true,
#endif
    .dev                =
    {
#ifdef CONFIG_SDIO_MUXBUS
      .lock             = imxrt_lock,
#endif
      .reset            = imxrt_reset,
      .capabilities     = imxrt_capabilities,
      .status           = imxrt_status,
      .widebus          = imxrt_widebus,
      .clock            = imxrt_clock,
      .attach           = imxrt_attach,
      .sendcmd          = imxrt_sendcmd,
#ifdef CONFIG_SDIO_BLOCKSETUP
      .blocksetup       = imxrt_blocksetup,
#endif

#ifndef CONFIG_IMXRT_USDHC_DMA
      .recvsetup        = imxrt_recvsetup,
      .sendsetup        = imxrt_sendsetup,
#else
      .recvsetup        = imxrt_dmarecvsetup,
      .sendsetup        = imxrt_dmasendsetup,
#endif
      .cancel           = imxrt_cancel,
      .waitresponse     = imxrt_waitresponse,
      .recv_r1          = imxrt_recvshortcrc,
      .recv_r2          = imxrt_recvlong,
      .recv_r3          = imxrt_recvshort,
      .recv_r4          = imxrt_recvshort,
      .recv_r5          = imxrt_recvshortcrc,
      .recv_r6          = imxrt_recvshortcrc,
      .recv_r7          = imxrt_recvshort,
      .waitenable       = imxrt_waitenable,
      .eventwait        = imxrt_eventwait,
      .callbackenable   = imxrt_callbackenable,
      .registercallback = imxrt_registercallback,
#ifdef CONFIG_SDIO_DMA
#ifdef CONFIG_IMXRT_USDHC_DMA
      .dmarecvsetup     = imxrt_dmarecvsetup,
      .dmasendsetup     = imxrt_dmasendsetup,
#else
      .dmarecvsetup     = imxrt_recvsetup,
      .dmasendsetup     = imxrt_sendsetup,
#endif
    },
    .waitsem = SEM_INITIALIZER(0),
  }
#endif
#endif
};

#ifdef CONFIG_SDIO_XFRDEBUG
/* Register logging support */

static struct imxrt_sdhcregs_s g_sampleregs[DEBUG_NSAMPLES];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_configwaitints
 *
 * Description:
 *   Enable/disable SDIO interrupts needed to support the wait function
 *
 * Input Parameters:
 *   priv       - A reference to the SDIO device state structure
 *   waitints   - The set of bits in the SDIO MASK register to set
 *   waitevents - Waited for events
 *   wkupevent  - Wake-up events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imxrt_configwaitints(struct imxrt_dev_s *priv, uint32_t waitints,
                                 sdio_eventset_t waitevents,
                                 sdio_eventset_t wkupevent)
{
  irqstate_t flags;

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags            = enter_critical_section();
  priv->waitevents = waitevents;
  priv->wkupevent  = wkupevent;
  priv->waitints   = waitints;

#ifdef CONFIG_IMXRT_USDHC_DMA
  priv->xfrflags   = 0;
#endif
  putreg32(priv->xfrints | priv->waitints | priv->cintints,
           priv->addr + IMXRT_USDHC_IRQSIGEN_OFFSET);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: imxrt_configxfrints
 *
 * Description:
 *   Enable SDIO interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the SDIO device state structure
 *   xfrints - The set of bits in the SDIO MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imxrt_configxfrints(struct imxrt_dev_s *priv, uint32_t xfrints)
{
  irqstate_t flags;

  flags = enter_critical_section();
  priv->xfrints = xfrints;
  putreg32(priv->xfrints | priv->waitints | priv->cintints,
           priv->addr + IMXRT_USDHC_IRQSIGEN_OFFSET);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: imxrt_sampleinit
 *
 * Description:
 *   Setup prior to collecting DMA samples
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void imxrt_sampleinit(void)
{
  memset(g_sampleregs, 0xff,
         DEBUG_NSAMPLES * sizeof(struct imxrt_sdhcregs_s));
}
#endif

/****************************************************************************
 * Name: imxrt_sdhcsample
 *
 * Description:
 *   Sample SDIO registers
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void imxrt_sdhcsample(struct imxrt_sdhcregs_s *regs)
{
  regs->dsaddr    = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_DSADDRL_OFFSET);
  regs->blkattr   = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_BLKATTRL_OFFSET);
  regs->cmdarg    = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_CMDARGL_OFFSET);
  regs->xferty    = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_XFERTYPL_OFFSET);
  regs->cmdrsp0   = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_CMDRSP0L_OFFSET);
  regs->cmdrsp1   = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_CMDRSP1L_OFFSET);
  regs->cmdrsp2   = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_CMDRSP2L_OFFSET);
  regs->cmdrsp3   = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_CMDRSP3L_OFFSET);
  regs->prsstat   = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_PRSSTATL_OFFSET);
  regs->proctl    = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_PROCTLL_OFFSET);
  regs->sysctl    = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_SYSCTLL_OFFSET);
  regs->irqstat   = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_IRQSTATL_OFFSET);
  regs->irqstaten = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_IRQSTATENL_OFFSET);
  regs->irqsigen  = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_IRQSIGENL_OFFSET);
  regs->ac12err   = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_AC12ERRL_OFFSET);
  regs->htcapblt  = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_HTCAPBLTL_OFFSET);
  regs->wml       = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_WMLL_OFFSET);
  regs->admaes    = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_ADMAESL_OFFSET);
  regs->adsaddr   = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_ADSADDRL_OFFSET);
  regs->vendor    = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_VENDORL_OFFSET);
  regs->mmcboot   = getreg32(DBG_BASE_ADDR + IMXRT_USDHC_MMCBOOTL_OFFSET);
}
#endif

/****************************************************************************
 * Name: imxrt_sample
 *
 * Description:
 *   Sample SDIO/DMA registers
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void imxrt_sample(struct imxrt_dev_s *priv, int index)
{
  if (priv->addr == DBG_BASE_ADDR)
    {
      imxrt_sdhcsample(&g_sampleregs[index]);
    }
}
#endif

/****************************************************************************
 * Name: imxrt_dumpsample
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void imxrt_dumpsample(struct imxrt_dev_s *priv,
                             struct imxrt_sdhcregs_s *regs, const char *msg)
{
  mcinfo("USDHC Registers: %s\n", msg);
  mcinfo("   DSADDR[%08x]: %08x\n",
         IMXRT_USDHC_DSADDR_OFFSET, regs->dsaddr);
  mcinfo("  BLKATTR[%08x]: %08x\n",
         IMXRT_USDHC_BLKATTR_OFFSET, regs->blkattr);
  mcinfo("   CMDARG[%08x]: %08x\n",
         IMXRT_USDHC_CMDARG_OFFSET, regs->cmdarg);
  mcinfo("   XFERTY[%08x]: %08x\n",
         IMXRT_USDHC_XFERTYP_OFFSET, regs->xferty);
  mcinfo("  CMDRSP0[%08x]: %08x\n",
         IMXRT_USDHC_CMDRSP0_OFFSET, regs->cmdrsp0);
  mcinfo("  CMDRSP1[%08x]: %08x\n",
         IMXRT_USDHC_CMDRSP1_OFFSET, regs->cmdrsp1);
  mcinfo("  CMDRSP2[%08x]: %08x\n",
         IMXRT_USDHC_CMDRSP2_OFFSET, regs->cmdrsp2);
  mcinfo("  CMDRSP3[%08x]: %08x\n",
         IMXRT_USDHC_CMDRSP3_OFFSET, regs->cmdrsp3);
  mcinfo("  PRSSTAT[%08x]: %08x\n",
         IMXRT_USDHC_PRSSTAT_OFFSET, regs->prsstat);
  mcinfo("   PROCTL[%08x]: %08x\n",
         IMXRT_USDHC_PROCTL_OFFSET, regs->proctl);
  mcinfo("   SYSCTL[%08x]: %08x\n",
         IMXRT_USDHC_SYSCTL_OFFSET, regs->sysctl);
  mcinfo("  IRQSTAT[%08x]: %08x\n",
         IMXRT_USDHC_IRQSTAT_OFFSET, regs->irqstat);
  mcinfo("IRQSTATEN[%08x]: %08x\n",
         IMXRT_USDHC_IRQSTATEN_OFFSET, regs->irqstaten);
  mcinfo(" IRQSIGEN[%08x]: %08x\n",
         IMXRT_USDHC_IRQSIGEN_OFFSET, regs->irqsigen);
  mcinfo("  AC12ERR[%08x]: %08x\n",
         IMXRT_USDHC_AC12ERR_OFFSET, regs->ac12err);
  mcinfo(" HTCAPBLT[%08x]: %08x\n",
         IMXRT_USDHC_HTCAPBLT_OFFSET, regs->htcapblt);
  mcinfo("      WML[%08x]: %08x\n",
         IMXRT_USDHC_WML_OFFSET, regs->wml);
  mcinfo("   ADMAES[%08x]: %08x\n",
         IMXRT_USDHC_ADMAES_OFFSET, regs->admaes);
  mcinfo("  ADSADDR[%08x]: %08x\n",
         IMXRT_USDHC_ADSADDR_OFFSET, regs->adsaddr);
  mcinfo("   VENDOR[%08x]: %08x\n",
         IMXRT_USDHC_VENDOR_OFFSET, regs->vendor);
  mcinfo("  MMCBOOT[%08x]: %08x\n",
         IMXRT_USDHC_MMCBOOT_OFFSET, regs->mmcboot);
}
#endif

/****************************************************************************
 * Name: imxrt_dumpsamples
 *
 * Description:
 *   Dump all sampled register data
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void imxrt_dumpsamples(struct imxrt_dev_s *priv)
{
  imxrt_dumpsample(priv, &g_sampleregs[SAMPLENDX_BEFORE_SETUP],
                   "Before setup");
  imxrt_dumpsample(priv, &g_sampleregs[SAMPLENDX_AFTER_SETUP],
                   "After setup");
  imxrt_dumpsample(priv, &g_sampleregs[SAMPLENDX_END_TRANSFER],
                   "End of transfer");
}
#endif

/****************************************************************************
 * Name: imxrt_showregs
 *
 * Description:
 *   Dump the current state of all registers
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void imxrt_showregs(struct imxrt_dev_s *priv, const char *msg)
{
  struct imxrt_sdhcregs_s regs;

  imxrt_sdhcsample(&regs);
  imxrt_dumpsample(priv, &regs, msg);
}
#endif

/****************************************************************************
 * Data Transfer Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_dataconfig
 *
 * Description:
 *   Configure the SDIO data path for the next data transfer
 *
 ****************************************************************************/

static void imxrt_dataconfig(struct imxrt_dev_s *priv, bool bwrite,
                             unsigned int datalen, unsigned int timeout)
{
  unsigned int watermark;
  uint32_t regval = 0;

  /* Set the data timeout value in the USDHC_SYSCTL field to the selected
   * value.
   */

  regval  = getreg32(priv->addr + IMXRT_USDHC_SYSCTL_OFFSET);
  regval &= ~USDHC_SYSCTL_DTOCV_MASK;
  regval |= timeout << USDHC_SYSCTL_DTOCV_SHIFT;
  putreg32(regval, priv->addr + IMXRT_USDHC_SYSCTL_OFFSET);

#if defined(CONFIG_IMXRT_USDHC_DMA) && defined(CONFIG_ARMV7M_DCACHE)
      /* If cache is enabled, and this is an unaligned receive,
       * receive one block at a time to the internal buffer
       */

      if (!bwrite && priv->unaligned_rx)
        {
          DEBUGASSERT(priv->blocksize <= sizeof(priv->rxbuffer));
          datalen = priv->blocksize;
        }
#endif

  /* Set the watermark level */

  /* Set the Read Watermark Level to the datalen to be read (limited to half
   * of the maximum watermark value).  BRR will be set when the number of
   * queued words is greater than or equal to this value.
   */

  watermark = (datalen + 3) >> 2;
  if (watermark > (USDHC_MAX_WATERMARK / 2))
    {
      watermark = (USDHC_MAX_WATERMARK / 2);
    }

  /* When the watermark level requirement is met in data transfer, and the
   * internal DMA is enabled, the data buffer block sends a DMA request to
   * the crossbar switch interface.
   */

  if (bwrite)
    {
      /* The USDHC will not start data transmission until the number of
       * words set in the WML register can be held in the buffer. If the
       * buffer is empty and the host system does not write data in time,
       * the USDHC will stop the SD_CLK to avoid the data buffer under-run
       * situation.
       */

      putreg32(watermark << USDHC_WML_WR_SHIFT,
               priv->addr + IMXRT_USDHC_WML_OFFSET);
    }
  else
    {
      /* The USDHC will not start data transmission until the number of words
       * set in the WML register are in the buffer. If the buffer is full and
       * the Host System does not read data in time, the USDHC will stop the
       * USDHC_DCLK to avoid the data buffer over-run situation.
       */

      putreg32(watermark << USDHC_WML_RD_SHIFT,
               priv->addr + IMXRT_USDHC_WML_OFFSET);
    }
}

/****************************************************************************
 * Name: imxrt_transmit
 *
 * Description:
 *   Send SDIO data in interrupt mode
 *
 * Input Parameters:
 *   priv - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_IMXRT_USDHC_DMA
static void imxrt_transmit(struct imxrt_dev_s *priv)
{
  union
  {
    uint32_t w;
    uint8_t  b[4];
  } data;

  /* Loop while there is more data to be sent, while buffer write enable
   * (PRSSTAT.BWEN)
   */

  mcinfo("Entry: remaining: %d IRQSTAT: %08x\n", priv->remaining,
         getreg32(priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET));

  while (priv->remaining > 0 &&
         (getreg32(priv->addr + IMXRT_USDHC_PRSSTAT_OFFSET) &
          USDHC_PRSSTAT_BWEN) != 0)
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
          for (i = 0; i < priv->remaining; i++)
            {
              data.b[i] = *ptr++;
            }

          /* Now the transfer is finished */

          priv->remaining = 0;
        }

      /* Put the word in the FIFO */

      putreg32(data.w, priv->addr + IMXRT_USDHC_DATAPORT_OFFSET);
    }

  /* Clear BWR.  If there is more data in the buffer, writing to the buffer
   * should reset BWR.
   */

  putreg32(USDHC_INT_BWR, priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET);

  mcinfo("Exit: remaining: %d IRQSTAT: %08x\n", priv->remaining,
         getreg32(priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET));
}
#endif

/****************************************************************************
 * Name: imxrt_receive
 *
 * Description:
 *   Receive SDIO data in interrupt mode
 *
 * Input Parameters:
 *   priv - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_IMXRT_USDHC_DMA
static void imxrt_receive(struct imxrt_dev_s *priv)
{
  unsigned int watermark;
  union
  {
    uint32_t w;
    uint8_t  b[4];
  } data;

  /* Set the Read Watermark Level to 1: BRR will be set when the number of
   * queued words is greater than or equal to 1.
   */

  putreg32(1 << USDHC_WML_RD_SHIFT, priv->addr + IMXRT_USDHC_WML_OFFSET);

  /* Loop while there is space to store the data, waiting for buffer
   * read ready (BRR)
   */

  mcinfo("Entry: remaining: %d IRQSTAT: %08x\n", priv->remaining,
         getreg32(priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET));

  while (priv->remaining > 0 &&
         (getreg32(priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET) &
          USDHC_INT_BRR) != 0)
    {
      /* Clear BRR.  If there is more data in the buffer, reading from the
       * buffer should reset BRR.
       */

      putreg32(USDHC_INT_BRR, priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET);

      /* Read the next word from the RX buffer */

      data.w = getreg32(priv->addr + IMXRT_USDHC_DATAPORT_OFFSET);
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

          for (i = 0; i < priv->remaining; i++)
            {
              *ptr++ = data.b[i];
            }

          /* Now the transfer is finished */

          priv->remaining = 0;
        }
    }

  /* Set the Read Watermark Level either the number of remaining words to be
   * read (limited to half of the maximum watermark value)
   */

  watermark = ((priv->remaining + 3) >> 2);
  if (watermark > (USDHC_MAX_WATERMARK / 2))
    {
      watermark = (USDHC_MAX_WATERMARK / 2);
    }

  putreg32(watermark << USDHC_WML_RD_SHIFT,
           priv->addr + IMXRT_USDHC_WML_OFFSET);

  mcinfo("Exit: remaining: %d IRQSTAT: %08x WML: %08x\n", priv->remaining,
         getreg32(priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET),
         getreg32(priv->addr + IMXRT_USDHC_WML_OFFSET));
}
#endif

/****************************************************************************
 * Name: imxrt_recvdma
 *
 * Description:
 *   Receive SDIO data in dma mode
 *
 * Input Parameters:
 *   priv  - Instance of the SDMMC private state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_IMXRT_USDHC_DMA) && defined(CONFIG_ARMV7M_DCACHE)
static void imxrt_recvdma(struct imxrt_dev_s *priv)
{
  unsigned int watermark;

  if (priv->unaligned_rx)
    {
      /* If we are receiving multiple blocks to an unaligned buffers,
       * we receive them one-by-one
       */

      /* Copy the received data to client buffer */

      memcpy(priv->buffer, priv->rxbuffer, priv->blocksize);

      /* Invalidate the cache before receiving next block */

      up_invalidate_dcache((uintptr_t)priv->rxbuffer,
                           (uintptr_t)priv->rxbuffer + priv->blocksize);

      /* Update how much there is left to receive */

      priv->remaining -= priv->blocksize;
    }
  else
    {
      /* In an aligned case, we have always received all blocks */

      priv->remaining = 0;
    }

  if (priv->remaining == 0)
    {
      /* no data remaining, end the transfer */

      imxrt_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
    }
  else
    {
      /* We end up here only in unaligned rx-buffers case, and are receiving
       * the data one block at a time
       */

      /* Update where to receive the following block */

      priv->buffer = (uint32_t *)((uintptr_t)priv->buffer + priv->blocksize);

      watermark = (priv->blocksize + 3) >> 2;
      if (watermark > (USDHC_MAX_WATERMARK / 2))
        {
          watermark = (USDHC_MAX_WATERMARK / 2);
        }

      /* Re-enable datapath and wait for next block */

      putreg32(watermark << USDHC_WML_RD_SHIFT,
               priv->addr + IMXRT_USDHC_WML_OFFSET);
    }
}

#endif
/****************************************************************************
 * Name: imxrt_eventtimeout
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

static void imxrt_eventtimeout(wdparm_t arg)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)arg;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0);

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. Sample registers at the time of the timeout */

      imxrt_sample(priv, SAMPLENDX_END_TRANSFER);

      /* Wake up any waiting threads */

      imxrt_endwait(priv, SDIOWAIT_TIMEOUT);
      mcerr("ERROR: Timeout: remaining: %d\n", priv->remaining);
    }
}

/****************************************************************************
 * Name: imxrt_endwait
 *
 * Description:
 *   Wake up a waiting thread if the waited-for event has occurred.
 *
 * Input Parameters:
 *   priv      - An instance of the SDIO device interface
 *   wkupevent - The event that caused the wait to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void imxrt_endwait(struct imxrt_dev_s *priv,
                          sdio_eventset_t wkupevent)
{
  /* Cancel the watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Disable event-related interrupts */

  imxrt_configwaitints(priv, 0, 0, wkupevent);

  /* Wake up the waiting thread */

  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: imxrt_endtransfer
 *
 * Description:
 *   Terminate a transfer with the provided status.  This function is called
 *   only from the SDIO interrupt handler when end-of-transfer conditions
 *   are detected.
 *
 * Input Parameters:
 *   priv      - An instance of the SDIO device interface
 *   wkupevent - The event that caused the transfer to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void imxrt_endtransfer(struct imxrt_dev_s *priv,
                              sdio_eventset_t wkupevent)
{
  /* Disable all transfer related interrupts */

  imxrt_configxfrints(priv, 0);

  /* Clearing pending interrupt status on all transfer related interrupts */

  putreg32(USDHC_XFRDONE_INTS | USDHC_DMADONE_INTS,
           priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET);

  /* Mark the transfer finished */

  priv->remaining = 0;

  /* Debug instrumentation */

  imxrt_sample(priv, SAMPLENDX_END_TRANSFER);

  /* Is a thread wait for these data transfer complete events? */

  if ((priv->waitevents & wkupevent) != 0)
    {
      /* Yes.. wake up any waiting threads */

      imxrt_endwait(priv, wkupevent);
    }
}

/****************************************************************************
 * Name: imxrt_interrupt
 *
 * Description:
 *   SDIO interrupt handler
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int imxrt_interrupt(int irq, void *context, void *arg)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)arg;
  uint32_t enabled;
  uint32_t pending;
  uint32_t regval;

  /* Check the USDHC IRQSTAT register.  Mask out all bits that don't
   * correspond to enabled interrupts.  (This depends on the fact that bits
   * are ordered the same in both the IRQSTAT and IRQSIGEN registers).
   * If there are non-zero bits remaining, then we have work to do here.
   */

  regval  = getreg32(priv->addr + IMXRT_USDHC_IRQSIGEN_OFFSET);
  enabled = getreg32(priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET) & regval;

  mcinfo("IRQSTAT: %08" PRIx32 " IRQSIGEN %08" PRIx32
         " enabled: %08" PRIx32 "\n",
         getreg32(priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET), regval, enabled);

  /* Clear all pending interrupts */

  putreg32(enabled, priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET);

  /* Handle in progress, interrupt driven data transfers ********************/

  pending = enabled & priv->xfrints;
  if (pending != 0)
    {
#ifndef CONFIG_IMXRT_USDHC_DMA
      /* Is the RX buffer read ready? Is so then we must be processing a
       * non-DMA receive transaction.
       */

      if ((pending & USDHC_INT_BRR) != 0)
        {
          /* Receive data from the RX buffer */

          imxrt_receive(priv);
        }

      /* Otherwise, Is the TX buffer write ready? If so we must be processing
       * non-DMA send transaction.  NOTE: We can't be processing both!
       */

      else if ((pending & USDHC_INT_BWR) != 0)
        {
          /* Send data via the TX FIFO */

          imxrt_transmit(priv);
        }
#endif

      /* ... transfer complete events */

      if ((pending & USDHC_INT_TC) != 0)
        {
          /* Terminate the transfer */
#if defined(CONFIG_IMXRT_USDHC_DMA) && defined(CONFIG_ARMV7M_DCACHE)
          imxrt_recvdma(priv);
#else
          imxrt_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
#endif
        }

      /* ... data block send/receive CRC failure */

      else if ((pending & USDHC_INT_DCE) != 0)
        {
          /* Terminate the transfer with an error */

          mcerr("ERROR: Data block CRC failure, remaining: %d\n",
                priv->remaining);
          imxrt_endtransfer(priv, SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
        }

      /* ... data timeout error */

      else if ((pending & USDHC_INT_DTOE) != 0)
        {
          /* Terminate the transfer with an error */

          mcerr("ERROR: Data timeout, remaining: %d\n", priv->remaining);
          imxrt_endtransfer(priv, SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT);
        }
    }

  /* Handle Card interrupt events *******************************************/

  pending = enabled & priv->cintints;
  if ((pending & USDHC_INT_CINT) != 0)
    {
      if (priv->do_sdio_card)
        {
          (priv->do_sdio_card)(priv->do_sdio_arg);
        }

      /* We don't want any more ints now, so switch it off */

      regval         &= ~USDHC_INT_CINT;
      priv->cintints  = regval;
      putreg32(regval, priv->addr + IMXRT_USDHC_IRQSIGEN_OFFSET);
    }

  if ((pending & USDHC_INT_CINS) != 0 || (pending & USDHC_INT_CRM) != 0)
    {
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

  /* Handle wait events *****************************************************/

  pending = enabled & priv->waitints;
  if (pending != 0)
    {
      /* Is this a response completion event? */

      if ((pending & USDHC_RESPDONE_INTS) != 0)
        {
          /* Yes.. Is there a thread waiting for response done? */

          if ((priv->waitevents &
               (SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE)) != 0)
            {
              /* Yes.. mask further interrupts and wake the thread up */

              regval  = getreg32(priv->addr + IMXRT_USDHC_IRQSIGEN_OFFSET);
              regval &= ~USDHC_RESPDONE_INTS;
              putreg32(regval, priv->addr + IMXRT_USDHC_IRQSIGEN_OFFSET);

              imxrt_endwait(priv, SDIOWAIT_RESPONSEDONE);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * SDIO Interface Methods
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_lock
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

#ifdef CONFIG_SDIO_MUXBUS
static int imxrt_lock(struct sdio_dev_s *dev, bool lock)
{
  /* The multiplex bus is part of board support package. */

  /* FIXME: Implement the below function to support bus share:
   *
   * imxrt_muxbus_sdio_lock((dev - g_sdhcdev) /
   *                        sizeof(struct imxrt_dev_s), lock);
   */

  return OK;
}
#endif

/****************************************************************************
 * Name: imxrt_reset
 *
 * Description:
 *   Reset the SDIO controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imxrt_reset(struct sdio_dev_s *dev)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;

  /* Disable all interrupts so that nothing interferes with the following. */

  putreg32(0, priv->addr + IMXRT_USDHC_IRQSIGEN_OFFSET);

  /* Reset the USDHC block, putting registers in their default, reset state.
   * Initiate the reset by setting the RSTA bit in the SYSCTL register.
   */

  modifyreg32(priv->addr + IMXRT_USDHC_VENDOR2_OFFSET,
              0, USDHC_VS2_BUSRESET);
  modifyreg32(priv->addr + IMXRT_USDHC_SYSCTL_OFFSET,
              0, USDHC_SYSCTL_RSTA);
  modifyreg32(priv->addr + IMXRT_USDHC_VENDOR2_OFFSET,
              USDHC_VS2_BUSRESET | USDHC_VS2_ACMD23ARGU2, 0);

  /* The USDHC will reset the RSTA bit to 0 when the capabilities registers
   * are valid and the host driver can read them.
   */

  while ((getreg32(priv->addr + IMXRT_USDHC_SYSCTL_OFFSET) &
          USDHC_SYSCTL_RSTA) != 0)
    {
    }

  mcinfo("Reset complete\n");

  /* Make sure that all clocking is disabled */

  imxrt_clock(dev, CLOCK_SDIO_DISABLED);

  /* Enable all status bits (these could not all be potential sources of
   * interrupts.
   */

  putreg32(USDHC_INT_ALL, priv->addr + IMXRT_USDHC_IRQSTATEN_OFFSET);

  mcinfo("SYSCTL: %08" PRIx32 " PRSSTAT: %08" PRIx32
         " IRQSTATEN: %08" PRIx32 "\n",
         getreg32(priv->addr + IMXRT_USDHC_SYSCTL_OFFSET),
         getreg32(priv->addr + IMXRT_USDHC_PRSSTAT_OFFSET),
         getreg32(priv->addr + IMXRT_USDHC_IRQSTATEN_OFFSET));

  /* The next phase of the hardware reset would be to set the SYSCTRL INITA
   * bit to send 80 clock ticks for card to power up and then reset the card
   * with CMD0.  This is done elsewhere.
   */

  /* Reset state data */

  priv->waitevents = 0;         /* Set of events to be waited for */
  priv->waitints   = 0;         /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;         /* The event that caused the wakeup */
#ifdef CONFIG_IMXRT_USDHC_DMA
  priv->xfrflags   = 0;         /* Used to synchronize SDIO and DMA completion */
#endif

  wd_cancel(&priv->waitwdog);   /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->buffer     = 0;         /* Address of current R/W buffer */
  priv->remaining  = 0;         /* Number of bytes remaining in the transfer */
  priv->xfrints    = 0;         /* Interrupt enables for data transfer */
}

/****************************************************************************
 * Name: imxrt_capabilities
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

static sdio_capset_t imxrt_capabilities(struct sdio_dev_s *dev)
{
  sdio_capset_t caps = 0;
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;

  switch (priv->addr)
    {
      case IMXRT_USDHC1_BASE:
#ifdef CONFIG_IMXRT_USDHC1_WIDTH_D1_ONLY
        caps |= SDIO_CAPS_1BIT_ONLY;
#endif
#ifdef CONFIG_IMXRT_USDHC1_WIDTH_D1_D4
        caps |= SDIO_CAPS_4BIT;
#endif
        break;

      case IMXRT_USDHC2_BASE:
#ifdef CONFIG_IMXRT_USDHC2_WIDTH_D1_ONLY
        caps |= SDIO_CAPS_1BIT_ONLY;
#endif
#ifdef CONFIG_IMXRT_USDHC2_WIDTH_D1_D4
        caps |= SDIO_CAPS_4BIT;
#endif
#ifdef CONFIG_IMXRT_USDHC2_WIDTH_D1_D8
        caps |= SDIO_CAPS_8BIT;
#endif
        break;

      default:
        break;
    }

#ifdef CONFIG_IMXRT_USDHC_DMA
  caps |= SDIO_CAPS_DMASUPPORTED;
#endif
  caps |= SDIO_CAPS_DMABEFOREWRITE;

  return caps;
}

/****************************************************************************
 * Name: imxrt_status
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see imxrt_status_* defines)
 *
 ****************************************************************************/

static sdio_statset_t imxrt_status(struct sdio_dev_s *dev)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;
  bool present = false;

  /* Board did not use one of the GPIO_USDHCn_CD pins
   * but instead a GPIO was used and defined in board.h
   * as PIN_USDHCx_CD_GPIO
   */

  if (priv->sw_cd_gpio != 0)
    {
      present = priv->cd_invert ^ !imxrt_gpio_read(priv->sw_cd_gpio);
    }
  else
    {
  /* This register reflects the state of CD no matter if it's a separate pin
   * or DAT3
   */

      present = ((getreg32(priv->addr + IMXRT_USDHC_PRSSTAT_OFFSET) &
                USDHC_PRSSTAT_CINS) != 0) ^ priv->cd_invert;
    }

  if (present)
    {
      priv->cdstatus |= SDIO_STATUS_PRESENT;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_PRESENT;
    }

  mcinfo("cdstatus=%02x\n", priv->cdstatus);

  return priv->cdstatus;
}

/****************************************************************************
 * Name: imxrt_widebus
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

static void imxrt_widebus(struct sdio_dev_s *dev, bool wide)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;
  uint32_t regval;

  /* Set the Data Transfer Width (DTW) field in the PROCTL register. */

  regval = getreg32(priv->addr + IMXRT_USDHC_PROCTL_OFFSET);
  regval &= ~USDHC_PROCTL_DTW_MASK;
  if (wide)
    {
      regval |= USDHC_PROCTL_DTW_4BIT;
    }
  else
    {
      regval |= USDHC_PROCTL_DTW_1BIT;
    }

  putreg32(regval, priv->addr + IMXRT_USDHC_PROCTL_OFFSET);
}

/****************************************************************************
 * Name: imxrt_frequency
 *
 * Description:
 *   Set the SD clock frequency
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO device interface
 *   frequency - The frequency to use
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_USDHC_ABSFREQ
static void imxrt_frequency(struct sdio_dev_s *dev, uint32_t frequency)
{
  uint32_t sdclkfs;
  uint32_t prescaled;
  uint32_t regval;
  unsigned int prescaler;
  unsigned int divisor;

  /* The SDCLK frequency is determined by
   *  (1) the frequency of the base clock that was selected as the
   *      input clock, and
   *  (2) by a prescaler and a divisor that are selected here:
   *
   *  SDCLK frequency = (base clock) / (prescaler * divisor)
   *
   * The prescaler is available only for the values: 2, 4, 8, 16, 32,
   * 64, 128, and 256.  Pick the smallest value of SDCLKFS that would
   * result in an in-range frequency. For example, if the base clock
   * frequency is 96 MHz, and the target frequency is 25 MHz, the
   * following logic will select prescaler:
   *
   *  96MHz / 2 <= 25MHz <= 96MHz / 2 /16 -- YES, prescaler == 2
   *
   * If the target frequency is 400 kHz, the following logic will
   * select prescaler:
   *
   *  96MHz / 2 <= 400KHz <= 96MHz / 2 / 16 -- NO
   *  96MHz / 4 <= 400KHz <= 96MHz / 4 / 16 -- NO
   *  96MHz / 8 <= 400KHz <= 96MHz / 8 / 16 -- NO
   *  96MHz / 16 <=400KHz <= 96MHz / 16 / 16 -- YES, prescaler == 16
   */

  if (/* frequency >= (BOARD_CORECLK_FREQ / 2) && */
       frequency <= (BOARD_CORECLK_FREQ / 2 / 16))
    {
      sdclkfs   = USDHC_SYSCTL_SDCLKFS_DIV2;
      prescaler = 2;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 4) &&
           frequency <= (BOARD_CORECLK_FREQ / 4 / 16))
    {
      sdclkfs   = USDHC_SYSCTL_SDCLKFS_DIV4;
      prescaler = 4;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 8) &&
           frequency <= (BOARD_CORECLK_FREQ / 8 / 16))
    {
      sdclkfs   = USDHC_SYSCTL_SDCLKFS_DIV8;
      prescaler = 8;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 16) &&
           frequency <= (BOARD_CORECLK_FREQ / 16 / 16))
    {
      sdclkfs   = USDHC_SYSCTL_SDCLKFS_DIV16;
      prescaler = 16;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 32) &&
           frequency <= (BOARD_CORECLK_FREQ / 32 / 16))
    {
      sdclkfs   = USDHC_SYSCTL_SDCLKFS_DIV32;
      prescaler = 32;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 64) &&
           frequency <= (BOARD_CORECLK_FREQ / 64 / 16))
    {
      sdclkfs   = USDHC_SYSCTL_SDCLKFS_DIV64;
      prescaler = 64;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 128) &&
           frequency <= (BOARD_CORECLK_FREQ / 128 / 16))
    {
      sdclkfs   = USDHC_SYSCTL_SDCLKFS_DIV128;
      prescaler = 128;
    }
  else
    {
      sdclkfs   = USDHC_SYSCTL_SDCLKFS_DIV256;
      prescaler = 256;
    }

  /* The optimal divider can than be calculated. For example, if the base
   * clock frequency is 96 MHz, the target frequency is 25 MHz, and the
   * selected prescaler value is 2, then
   *
   *   prescaled = 96MHz / 2 = 48MHz
   *   divisor = (48MHz + 12.5HMz/ 25MHz = 2
   *
   * And the resulting frequency will be 24MHz. Or, for example, if the
   * target frequency is 400 kHz and the selected prescaler is 16, the
   * following logic will select prescaler:
   *
   *   prescaled = 96MHz / 16 = 6MHz
   *   divisor = (6MHz + 200KHz) / 400KHz = 15
   *
   * And the resulting frequency will be exactly 400KHz.
   */

  prescaled = frequency / prescaler;
  divisor   = (prescaled + (frequency >> 1)) / frequency;

  /* Set the new divisor information and enable all clocks in the SYSCTRL
   * register. TODO: Investigate using the automatically gated clocks to
   * reduce power consumption.
   */

  regval  = getreg32(priv->addr + IMXRT_USDHC_SYSCTL_OFFSET);
  regval &= ~(USDHC_SYSCTL_SDCLKFS_MASK | USDHC_SYSCTL_DVS_MASK);
  regval |= (sdclkfs | USDHC_SYSCTL_DVS_DIV(divisor));
  regval |= (USDHC_SYSCTL_SDCLKEN | USDHC_SYSCTL_PEREN |
             USDHC_SYSCTL_HCKEN | USDHC_SYSCTL_IPGEN);

  putreg32(regval, priv->addr + IMXRT_USDHC_SYSCTL_OFFSET);

  mcinfo("SYSCTRL: %08x\n",
         getreg32(priv->addr + IMXRT_USDHC_SYSCTL_OFFSET));
}
#endif

/****************************************************************************
 * Name: imxrt_clock
 *
 * Description:
 *   Enable/disable SDIO clocking
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   rate - Specifies the clocking to use (see enum sdio_clock_e)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imxrt_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  struct imxrt_dev_s *priv =
    (struct imxrt_dev_s *)dev; uint32_t regval;

  /* Clear the old prescaler and divisor values so that new ones can be
   * ORed in.
   */

  regval  = getreg32(priv->addr + IMXRT_USDHC_SYSCTL_OFFSET);
  regval &= ~(USDHC_SYSCTL_SDCLKFS_MASK | USDHC_SYSCTL_DVS_MASK);

  /* Select the new prescaler and divisor values based on the requested
   * mode and the settings from the board.h file. Clocks are automatically
   * gated by the driver when not needed.
   */

  switch (rate)
    {
    default:
    case CLOCK_SDIO_DISABLED:      /* Clock is disabled */
      {
        /* Clear the prescaler and divisor settings */

        putreg32(regval, priv->addr + IMXRT_USDHC_SYSCTL_OFFSET);
        mcinfo("DISABLED, SYSCTRL: %08" PRIx32 "\n",
               getreg32(priv->addr + IMXRT_USDHC_SYSCTL_OFFSET)); return;
      }
      break;

    case CLOCK_IDMODE:
      {
        /* Initial ID mode clocking (<400KHz) */

        mcinfo("IDMODE\n");

        /* Put out an additional 80 clocks in case this is a power-up
         * sequence.
         */

        regval |= (BOARD_USDHC_IDMODE_PRESCALER |
                   BOARD_USDHC_IDMODE_DIVISOR |
                   USDHC_SYSCTL_INITA);
      }
      break;

    case CLOCK_MMC_TRANSFER:
      {
        /* MMC normal operation clocking */

        mcinfo("MMCTRANSFER\n");
        regval |= (BOARD_USDHC_MMCMODE_PRESCALER |
                   BOARD_USDHC_MMCMODE_DIVISOR);
      }
      break;

    case CLOCK_SD_TRANSFER_1BIT:
      {
#ifndef CONFIG_IMXRT_USDHC_WIDTH_D1_ONLY
        /* SD normal operation clocking (narrow 1-bit mode) */

        mcinfo("1BITTRANSFER\n");
        regval |= (BOARD_USDHC_SD1MODE_PRESCALER |
                   BOARD_USDHC_SD1MODE_DIVISOR);
      }
      break;
#endif

    case CLOCK_SD_TRANSFER_4BIT:
      {
        /* SD normal operation clocking (wide 4-bit mode) */

        mcinfo("4BITTRANSFER\n");
        regval |= (BOARD_USDHC_SD4MODE_PRESCALER |
                  BOARD_USDHC_SD4MODE_DIVISOR);
      }
      break;
    }

  putreg32(regval, priv->addr + IMXRT_USDHC_SYSCTL_OFFSET);

  /* Wait for clock to become stable */

  while ((getreg32(priv->addr + IMXRT_USDHC_PRSSTAT_OFFSET) &
          USDHC_PRSSTAT_SDSTB) == 0)
    {
    }

  mcinfo("SYSCTRL: %08" PRIx32 "\n",
         getreg32(priv->addr + IMXRT_USDHC_SYSCTL_OFFSET));
}

/****************************************************************************
 * Name: imxrt_attach
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

static int imxrt_attach(struct sdio_dev_s *dev)
{
  int ret;
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;

  /* Attach the SDIO interrupt handler */

  if (priv->addr == IMXRT_USDHC1_BASE)
    {
      ret = irq_attach(IMXRT_IRQ_USDHC1, imxrt_interrupt, &g_sdhcdev[0]);
    }
  else
    {
      if (priv->addr == IMXRT_USDHC2_BASE)
        {
          ret = irq_attach(IMXRT_IRQ_USDHC2, imxrt_interrupt, &g_sdhcdev[1]);
        }
      else
        {
          PANIC();
        }
    }

  if (ret == OK)
    {
      /* Disable all interrupts at the SDIO controller and clear all pending
       * interrupts.
       */

      putreg32(0, priv->addr + IMXRT_USDHC_IRQSIGEN_OFFSET);
      putreg32(USDHC_INT_ALL, priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET);

      /* Enable SDIO interrupts at the NVIC.  They can now be enabled at the
       * SDIO controller as needed.
       */

      if (priv->addr == IMXRT_USDHC1_BASE)
        {
          up_enable_irq(IMXRT_IRQ_USDHC1);
        }
      else if (priv->addr == IMXRT_USDHC2_BASE)
        {
          up_enable_irq(IMXRT_IRQ_USDHC2);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: imxrt_sendcmd
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
 *   None
 *
 ****************************************************************************/

static int imxrt_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                         uint32_t arg)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  uint32_t regval;
  uint32_t mcrregval;
  uint32_t cmdidx;

  /* Initialize the command index */

  cmdidx    = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval    = cmdidx << USDHC_XFERTYP_CMDINX_SHIFT;
  mcrregval = USDHC_MC_DEFAULTVAL;

  if (cmdidx == SD_ACMDIDX53)
    {
      /* Dynamically set parameters for ACMD53 because it can accommodate
       * different transmission characteristics (single and multi-block,
       * rx & tx).
       */

      if (arg & (1 << 31))
        {
          /* Transmit mode */

          cmd |= MMCSD_WRDATAXFR;
        }
      else
        {
          /* Receive mode */

          cmd |= MMCSD_RDDATAXFR;
        }

      if (arg & (1 << 27))
        {
          /* In block mode */

          cmd |= SDIO_MULTIBLOCK;
        }
    }

  /* Check if a data transfer accompanies the command */

  switch (cmd & MMCSD_DATAXFR_MASK)
    {
    default:
    case MMCSD_NODATAXFR:
      {
        /* No.. no data transfer */
      }
      break;

      /* The following two cases are probably missing some setup logic */

    case MMCSD_RDSTREAM:
      {
        /* Yes.. streaming read data transfer */

        regval |= USDHC_XFERTYP_DPSEL;
        mcrregval |= USDHC_MC_DTDSEL;
      }
      break;

    case MMCSD_WRSTREAM:
      {
        /* Yes.. streaming write data transfer */

       regval |= USDHC_XFERTYP_DPSEL;
      }
      break;

    case MMCSD_RDDATAXFR:
      {
        /* Yes.. normal read data transfer */

        regval |= USDHC_XFERTYP_DPSEL;
        mcrregval |= USDHC_MC_DTDSEL;
      }
      break;

    case MMCSD_WRDATAXFR:
      {
        /* Yes.. normal write data transfer */

        regval |= USDHC_XFERTYP_DPSEL;
      }
      break;
    }

  /* Is it a multi-block transfer? */

  if ((cmd & (MMCSD_MULTIBLOCK | SDIO_MULTIBLOCK)) != 0)
    {
      mcrregval |= USDHC_MC_MSBSEL;

      /* Yes.. should the transfer be stopped with ACMD12? */

      if (((cmd & MMCSD_MULTIBLOCK) != 0) &&
          ((cmd & MMCSD_STOPXFR) != 0))
        {
          /* Yes.. Indefinite block transfer (not SDIO) */

            mcrregval |= USDHC_MC_AC12EN;
        }
      else
        {
          /* No.. Fixed block transfer */

          mcrregval |= USDHC_MC_BCEN;
        }
    }

  /* Configure response type bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      {
        /* No response */

        regval |= USDHC_XFERTYP_RSPTYP_NONE;
      }
      break;

    case MMCSD_R1B_RESPONSE:
      {
        /* Response length 48, check busy & cmdindex */

        regval |=
          (USDHC_XFERTYP_RSPTYP_LEN48BSY | USDHC_XFERTYP_CICEN |
           USDHC_XFERTYP_CCCEN);
      }
      break;

    case MMCSD_R1_RESPONSE: /* Response length 48, check cmdindex */
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
      {
        regval |=
          (USDHC_XFERTYP_RSPTYP_LEN48 | USDHC_XFERTYP_CICEN |
           USDHC_XFERTYP_CCCEN);
      }
      break;

    case MMCSD_R2_RESPONSE:
      {
        /* Response length 136, check CRC */

        regval |= (USDHC_XFERTYP_RSPTYP_LEN136 | USDHC_XFERTYP_CCCEN);
      }
      break;

    case MMCSD_R3_RESPONSE: /* Response length 48 */
    case MMCSD_R4_RESPONSE:
    case MMCSD_R7_RESPONSE:
      {
        regval |= USDHC_XFERTYP_RSPTYP_LEN48;
      }
      break;
    }

#ifdef CONFIG_IMXRT_USDHC_DMA
  /* Enable DMA */

  /* Internal DMA is used */

  mcrregval |= USDHC_MC_DMAEN;
#endif

  /* Check for abort. TODO: Check Suspend/Resume bits too in
   * XFR_TYP::CMDTYP.
   */

  if (cmd & MMCSD_STOPXFR)
    {
      regval |= USDHC_XFERTYP_CMDTYP_ABORT;
    }

  mcinfo("cmd: %08" PRIx32 " arg: %08" PRIx32
         " regval: %08" PRIx32 " mcrval: %08" PRIx32 "\n", cmd, arg,
         regval, mcrregval);

  /* If there has been a response error then perform a reset and wait for it
   * to complete.
   */

  if ((getreg32(priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET) &
       USDHC_RESPERR_INTS) != 0)
    {
      modifyreg32(priv->addr + IMXRT_USDHC_SYSCTL_OFFSET, 0,
                  USDHC_SYSCTL_RSTC);
      while ((getreg32(priv->addr + IMXRT_USDHC_SYSCTL_OFFSET) &
             USDHC_SYSCTL_RSTC) != 0)
        {
        }
    }

  /* The Command Inhibit (CIHB) bit is set in the PRSSTAT bit immediately
   * after the transfer type register is written.  This bit is cleared
   * when the command response is received.  If this status bit is 0, it
   * indicates that the CMD line is not in use and the USDHC can issue a
   * SD/MMC Command using the CMD line. CIHB should always be clear before
   * this function is called, but this check is performed here to provide
   * overlap and maximum performance.
   */

  timeout = USDHC_CMDTIMEOUT;
  start   = clock_systime_ticks();
  while ((getreg32(priv->addr + IMXRT_USDHC_PRSSTAT_OFFSET) &
          USDHC_PRSSTAT_CIHB) != 0)
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;
      if (elapsed >= timeout)
        {
          mcerr("ERROR: Timeout (waiting CIHB) cmd: %08" PRIx32
                " PRSSTAT: %08" PRIx32 "\n",
                cmd, getreg32(priv->addr + IMXRT_USDHC_PRSSTAT_OFFSET));
          return -EBUSY;
        }
    }

  /* Set the USDHC Argument value */

  putreg32(arg, priv->addr + IMXRT_USDHC_CMDARG_OFFSET);

  /* Clear interrupt status and write the USDHC CMD */

  putreg32(USDHC_RESPDONE_INTS, priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET);
  putreg32(mcrregval, priv->addr + IMXRT_USDHC_MIX_OFFSET);
  putreg32(regval, priv->addr + IMXRT_USDHC_XFERTYP_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: imxrt_blocksetup
 *
 * Description:
 *   Configure block size and the number of blocks for next transfer
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO device interface
 *   blocklen  - The selected block size.
 *   nblocklen - The number of blocks to transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_BLOCKSETUP
static void imxrt_blocksetup(struct sdio_dev_s *dev,
                             unsigned int blocklen,
                             unsigned int nblocks)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;

  mcinfo("blocklen=%d, total transfer=%d (%d blocks)\n", blocklen,
         blocklen * nblocks, nblocks);

  /* Configure block size for next transfer */

#if defined(CONFIG_ARMV7M_DCACHE)
  priv->blocksize = blocklen;
#endif

  putreg32(USDHC_BLKATTR_SIZE(blocklen) | USDHC_BLKATTR_CNT(nblocks),
           priv->addr + IMXRT_USDHC_BLKATTR_OFFSET);
}
#endif

/****************************************************************************
 * Name: imxrt_recvsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card in non-
 *   DMA (interrupt driven mode).  This method will do whatever controller
 *   setup is necessary.  This would be called for SD memory just BEFORE
 *   sending CMD13 (SEND_STATUS), CMD17 (READ_SINGLE_BLOCK), CMD18
 *   (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR), etc.  Normally,
 *   SDIO_WAITEVENT will be called to receive the indication that the
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

#ifndef CONFIG_IMXRT_USDHC_DMA
static int imxrt_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                           size_t nbytes)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;
  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t) buffer & 3) == 0);

  /* Reset the DPSM configuration */

  imxrt_sampleinit();
  imxrt_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt
   * handler and DMA memory invalidation.
   */

  priv->buffer = (uint32_t *)buffer;
  priv->remaining = nbytes;

  /* Then set up the SDIO data path */

  imxrt_dataconfig(priv, false, nbytes, USDHC_DTOCV_MAXTIMEOUT);

  /* And enable interrupts */

  imxrt_configxfrints(priv, USDHC_RCVDONE_INTS);
  imxrt_sample(priv, SAMPLENDX_AFTER_SETUP); return OK;
}
#endif

/****************************************************************************
 * Name: imxrt_sendsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card.  This
 *   method will do whatever controller setup is necessary.  This would be
 *   called for SD memory just AFTER sending CMD24 (WRITE_BLOCK), CMD25
 *   (WRITE_MULTIPLE_BLOCK), ... and before SDIO_SENDDATA is called.
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

#ifndef CONFIG_IMXRT_USDHC_DMA
static int imxrt_sendsetup(struct sdio_dev_s *dev,
                           const uint8_t *buffer,
                           size_t nbytes)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;
  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t) buffer & 3) == 0);

  /* Reset the DPSM configuration */

  imxrt_sampleinit();
  imxrt_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer = (uint32_t *)buffer;
  priv->remaining = nbytes;

  /* Then set up the SDIO data path */

  imxrt_dataconfig(priv, true, nbytes, USDHC_DTOCV_MAXTIMEOUT);

  /* Enable TX interrupts */

  imxrt_configxfrints(priv, USDHC_SNDDONE_INTS);
  imxrt_sample(priv, SAMPLENDX_AFTER_SETUP); return OK;
}
#endif

/****************************************************************************
 * Name: imxrt_cancel
 *
 * Description:
 *   Cancel the data transfer setup of SDIO_RECVSETUP, SDIO_SENDSETUP,
 *   SDIO_DMARECVSETUP or SDIO_DMASENDSETUP.  This must be called to cancel
 *   the data transfer setup if, for some reason, you cannot perform the
 *   transfer.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int imxrt_cancel(struct sdio_dev_s *dev)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;

#ifdef CONFIG_IMXRT_USDHC_DMA
  uint32_t regval;
#endif

  /* Disable all transfer- and event- related interrupts */

  imxrt_configxfrints(priv, 0); imxrt_configwaitints(priv, 0, 0, 0);

  /* Clearing pending interrupt status on all transfer- and event- related
   * interrupts
   */

  putreg32(USDHC_WAITALL_INTS, priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET);

  /* Cancel any watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_IMXRT_USDHC_DMA

  /* Stop the DMA by resetting the data path */

  regval = getreg32(priv->addr + IMXRT_USDHC_SYSCTL_OFFSET);
  regval |= USDHC_SYSCTL_RSTD;
  putreg32(regval, priv->addr + IMXRT_USDHC_SYSCTL_OFFSET);
#endif

  /* Mark no transfer in progress */

  priv->remaining = 0; return OK;
}

/****************************************************************************
 * Name: imxrt_waitresponse
 *
 * Description:
 *   Poll-wait for the response to the last command to be ready.  This
 *   function should be called even after sending commands that have no
 *   response (such as CMD0) to make sure that the hardware is ready to
 *   receive the next command.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command that was sent.  See 32-bit command definitions above.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int imxrt_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  uint32_t errors;
  uint32_t enerrors;

  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;
  int ret = OK;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      timeout = USDHC_CMDTIMEOUT;
      errors  = 0;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
      {
        timeout = USDHC_LONGTIMEOUT;
        errors  = USDHC_RESPERR_INTS;
      }
      break;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      {
        timeout = USDHC_CMDTIMEOUT;
        errors  = USDHC_RESPERR_INTS;
      }
      break;

    default:
      return -EINVAL;
    }

  /* Then wait for the Command Complete (CC) indication (or timeout).  The
   * CC bit is set when the end bit of the command response is received
   * (except Auto CMD12).
   */

  start = clock_systime_ticks();
  while ((getreg32(priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET) &
          USDHC_INT_CC) == 0)
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;
      if (elapsed >= timeout)
        {
          mcerr("ERROR: Timeout cmd: %08" PRIx32
                " IRQSTAT: %08" PRIx32 "\n", cmd,
                getreg32(priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET));
          ret = -ETIMEDOUT;
          break;
        }
    }

  /* Check for hardware detected errors */

  enerrors = getreg32(priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET) & errors;
  if (enerrors != 0)
    {
      mcerr("ERROR: cmd: %08" PRIx32 " errors: %08" PRIx32 ", "
            "fired %08" PRIx32 " IRQSTAT: %08" PRIx32 "\n",
            cmd, errors, enerrors,
            getreg32(priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET));

      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: imxrt_recv*
 *
 * Description:
 *   Receive response to SDIO command.  Only the critical payload is
 *   returned -- that is 32 bits for 48 bit status and 128 bits for 136 bit
 *   status.  The driver implementation should verify the correctness of
 *   the remaining, non-returned bits (CRCs, CMD index, etc.).
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   Rx - Buffer in which to receive the response
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure.  Here a
 *   failure means only a failure to obtain the requested response (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intact and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int imxrt_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                              uint32_t *rshort)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;
  uint32_t regval;
  int ret = OK;

  /* R1 Command response (48-bit)
   *   47    0           Start bit
   *   46    0           Transmission bit (0=from card)
   *   45:40 bit5 - bit0 Command index (0-63)
   *   39:8  bit31- bit0 32-bit card status
   *    7:1  bit6 - bit0 CRC7
   *      0  1           End bit
   *
   * R1b Identical to R1 with the additional busy signalling via the data
   *     line.
   * R6  Published RCA Response (48-bit, SD card only)
   *   47    0           Start bit
   *   46    0           Transmission bit (0=from card)
   *   45:40 bit5 - bit0 Command index (0-63)
   *   39:8 bit31 - bit0 32-bit Argument Field, consisting of:
   *                        [31:16] New published RCA of card
   *                        [15:0]  Card status bits
   *   {23,22,19,12:0}
   *   7:1 bit6 - bit0   CRC7
   *     0 1             End bit
   */

#ifdef CONFIG_DEBUG_FEATURES
  if (!rshort)
    {
      mcerr("ERROR: rshort=NULL\n");
      ret = -EINVAL;
    }

  /* Check that this is the correct response to this command */

  else if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1B_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R5_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R6_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08" PRIx32 "\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = getreg32(priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET);
      if ((regval & USDHC_INT_CTOE) != 0)
        {
          mcerr("ERROR: Command timeout: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & USDHC_INT_CCE) != 0)
        {
          mcerr("ERROR: CRC failure: %08" PRIx32 "\n", regval); ret = -EIO;
        }
    }

  /* Return the R1/R1b/R6 response.  These responses are returned in
   * CDMRSP0.  NOTE: This is not true for R1b (Auto CMD12 response) which is
   * returned in CMDRSP3.
   */

  *rshort = getreg32(priv->addr + IMXRT_USDHC_CMDRSP0_OFFSET);
  return ret;
}

static int imxrt_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                          uint32_t rlong[4])
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;
  uint32_t regval;
  int ret = OK;

  /* R2 CID, CSD register (136-bit)
   *  135     0             Start bit
   *  134     0             Transmission bit (0=from card)
   *  133:128 bit5 - bit0   Reserved
   *  127:1   bit127 - bit1 127-bit CID or CSD register (including int. CRC)
   *    0     1             End bit
   */

#ifdef CONFIG_DEBUG_FEATURES
  /* Check that R1 is the correct response to this command */

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R2_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08" PRIx32 "\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = getreg32(priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET);
      if (regval & USDHC_INT_CTOE)
        {
          mcerr("ERROR: Timeout IRQSTAT: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & USDHC_INT_CCE)
        {
          mcerr("ERROR: CRC fail IRQSTAT: %08" PRIx32 "\n", regval);
          ret = -EIO;
        }
    }

  /* Return the long response in CMDRSP3..0 */

  if (rlong)
    {
      uint32_t rsp3 = getreg32(priv->addr + IMXRT_USDHC_CMDRSP3_OFFSET);
      uint32_t rsp2 = getreg32(priv->addr + IMXRT_USDHC_CMDRSP2_OFFSET);
      uint32_t rsp1 = getreg32(priv->addr + IMXRT_USDHC_CMDRSP1_OFFSET);
      uint32_t rsp0 = getreg32(priv->addr + IMXRT_USDHC_CMDRSP0_OFFSET);

      rlong[0] = rsp3 << 8 | rsp2 >> 24;
      rlong[1] = rsp2 << 8 | rsp1 >> 24;
      rlong[2] = rsp1 << 8 | rsp0 >> 24; rlong[3] = rsp0 << 8;
    }

  return ret;
}

static int imxrt_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t *rshort)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;
  uint32_t regval;
  int ret = OK;

  /* R3 OCR (48-bit)
   *    47    0           Start bit
   *    46    0           Transmission bit (0=from card)
   *    45:40 bit5 - bit0 Reserved
   *    39:8  bit31- bit0 32-bit OCR register
   *     7:1  bit6 - bit0 Reserved
   *     0    1           End bit
   */

  /* Check that this is the correct response to this command */

#ifdef CONFIG_DEBUG_FEATURES
  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R4_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08" PRIx32 "\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout occurred (Apparently a CRC error can terminate a
       * good response)
       */

      regval = getreg32(priv->addr + IMXRT_USDHC_IRQSTAT_OFFSET);
      if (regval & USDHC_INT_CTOE)
        {
          mcerr("ERROR: Timeout IRQSTAT: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
    }

  /* Return the short response in CMDRSP0 */

  if (rshort)
    {
      *rshort = getreg32(priv->addr + IMXRT_USDHC_CMDRSP0_OFFSET);
    }

  return ret;
}

/****************************************************************************
 * Name: imxrt_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the SDIO_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling imxrt_eventwait.  This is done in this way
 *   to help the driver to eliminate race conditions between the command
 *   setup and the subsequent events.
 *
 *   The enabled events persist until either (1) SDIO_WAITENABLE is called
 *   again specifying a different set of wait events, or (2) SDIO_EVENTWAIT
 *   returns.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOWAIT_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imxrt_waitenable(struct sdio_dev_s *dev,
                             sdio_eventset_t eventset, uint32_t timeout)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;
  uint32_t waitints;

  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  imxrt_configwaitints(priv, 0, 0, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

  waitints = 0;
  if ((eventset & (SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE)) != 0)
    {
      waitints |= USDHC_RESPDONE_INTS;
    }

  if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
    {
#ifdef CONFIG_IMXRT_USDHC_DMA
      waitints |= USDHC_DMADONE_INTS;
#else
      waitints |= USDHC_XFRDONE_INTS;
#endif
    }

  /* Enable event-related interrupts */

  imxrt_configwaitints(priv, waitints, eventset, 0);

  /* Check if the timeout event is specified in the event set */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      int delay;
      int ret;

      /* Yes.. Handle a corner case */

      if (!timeout)
        {
          priv->wkupevent = SDIOWAIT_TIMEOUT;
          return;
        }

      /* Start the watchdog timer */

      delay = MSEC2TICK(timeout);
      ret = wd_start(&priv->waitwdog, delay,
                     imxrt_eventtimeout, (wdparm_t)priv);

      if (ret < 0)
        {
          mcerr("ERROR: wd_start failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: imxrt_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDIO_WAITEVENTS are disabled when imxrt_eventwait
 *   returns.  SDIO_WAITEVENTS must be called again before imxrt_eventwait
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

static sdio_eventset_t imxrt_eventwait(struct sdio_dev_s *dev)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;
  sdio_eventset_t wkupevent = 0; int ret;

  /* There is a race condition here... the event may have completed before
   * we get here.  In this case waitevents will be zero, but wkupevents
   * will be non-zero (and, hopefully, the semaphore count will also be
   * non-zero.
   */

  DEBUGASSERT((priv->waitevents != 0 && priv->wkupevent == 0) ||
              (priv->waitevents == 0 && priv->wkupevent != 0));

  /* Loop until the event (or the timeout occurs). Race conditions are
   * avoided by calling imxrt_waitenable prior to triggering the logic
   * that will cause the wait to terminate.  Under certain race
   * conditions, the waited-for may have already occurred before this
   * function was called!
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
          return SDIOWAIT_ERROR;
        }

      wkupevent = priv->wkupevent;

      /* Check if the event has occurred.  When the event has occurred, then
       * evenset will be set to 0 and wkupevent will be set to a non-zero
       * value.
       */

      if (wkupevent != 0)
        {
          /* Yes... break out of the loop with wkupevent non-zero */

          break;
        }
    }

  /* Disable event-related interrupts */

  imxrt_configwaitints(priv, 0, 0, 0);
#ifdef CONFIG_IMXRT_USDHC_DMA
  priv->xfrflags = 0;
#endif
  imxrt_dumpsamples(priv);
  return wkupevent;
}

/****************************************************************************
 * Name: imxrt_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events.  This is part of the
 *   the SDIO callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in imxrt_registercallback.
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

static void imxrt_callbackenable(struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;

  mcinfo("eventset: %02x\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  imxrt_callback(priv);
}

/****************************************************************************
 * Name: imxrt_registercallback
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change.  Callbacks should not be made from interrupt handlers, rather
 *   interrupt level events should be handled by calling back on the work
 *   thread.
 *
 *   When this method is called, all callbacks should be disabled until they
 *   are enabled via a call to SDIO_CALLBACKENABLE
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

static int imxrt_registercallback(struct sdio_dev_s *dev,
                                  worker_t callback, void *arg)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;

  /* Disable callbacks and register this callback and is argument */

  mcinfo("Register %p(%p)\n", callback, arg);

  DEBUGASSERT(priv != NULL);
  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: imxrt_dmarecvsetup
 *
 * Description:
 *   Setup to perform a read DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For read transfers this may mean
 *   invalidating the data cache.
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

#ifdef CONFIG_IMXRT_USDHC_DMA
static int imxrt_dmarecvsetup(struct sdio_dev_s *dev,
                              uint8_t *buffer, size_t buflen)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;
  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t) buffer & 3) == 0);

  /* Begin sampling register values */

  imxrt_sampleinit();
  imxrt_sample(priv, SAMPLENDX_BEFORE_SETUP);

#if defined(CONFIG_ARMV7M_DCACHE)
  if (((uintptr_t)buffer & (ARMV7M_DCACHE_LINESIZE - 1)) != 0 ||
       (buflen & (ARMV7M_DCACHE_LINESIZE - 1)) != 0)
    {
      /* The read buffer is not cache-line aligned. Read to an internal
       * buffer instead.
       */

      up_invalidate_dcache((uintptr_t)priv->rxbuffer,
                           (uintptr_t)priv->rxbuffer + priv->blocksize);

      priv->unaligned_rx = true;
    }
  else
    {
      up_invalidate_dcache((uintptr_t)buffer,
                           (uintptr_t)buffer + buflen);

      priv->unaligned_rx = false;
    }
#endif

  /* Save the destination buffer information for use by the interrupt
   * handler
   */

  priv->buffer = (uint32_t *)buffer;
  priv->remaining = buflen;

  /* Then set up the SDIO data path */

  imxrt_dataconfig(priv, false, buflen, USDHC_DTOCV_MAXTIMEOUT);

  /* Configure the RX DMA */

  imxrt_configxfrints(priv, USDHC_DMADONE_INTS);
#if defined(CONFIG_ARMV7M_DCACHE)
  if (priv->unaligned_rx)
    {
      putreg32((uint32_t) priv->rxbuffer,
               priv->addr + IMXRT_USDHC_DSADDR_OFFSET);
    }
  else
#endif
    {
      putreg32((uint32_t) priv->buffer,
               priv->addr + IMXRT_USDHC_DSADDR_OFFSET);
    }

  /* Sample the register state */

  imxrt_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}
#endif

/****************************************************************************
 * Name: imxrt_dmasendsetup
 *
 * Description:
 *   Setup to perform a write DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For write transfers, this may mean
 *   flushing the data cache.
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

#ifdef CONFIG_IMXRT_USDHC_DMA
static int imxrt_dmasendsetup(struct sdio_dev_s *dev,
                              const uint8_t *buffer, size_t buflen)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;
  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t) buffer & 3) == 0);

  /* Begin sampling register values */

  imxrt_sampleinit();
  imxrt_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

#if defined(CONFIG_ARMV7M_DCACHE)
  priv->unaligned_rx = false;

  /* Flush cache to physical memory when not in DTCM memory */

#  if !defined(CONFIG_ARMV7M_DCACHE_WRITETHROUGH)
    {
      up_clean_dcache((uintptr_t)buffer, (uintptr_t)buffer + buflen);
    }

#  endif
#endif
  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;

  /* Then set up the SDIO data path */

  imxrt_dataconfig(priv, true, buflen, USDHC_DTOCV_MAXTIMEOUT);

  /* Configure the TX DMA */

  putreg32((uint32_t) buffer, priv->addr + IMXRT_USDHC_DSADDR_OFFSET);

  /* Sample the register state */

  imxrt_sample(priv, SAMPLENDX_AFTER_SETUP);

  /* Enable TX interrupts */

  imxrt_configxfrints(priv, USDHC_DMADONE_INTS);
  return OK;
}
#endif

/****************************************************************************
 * Name: imxrt_callback
 *
 *  Description:
 *   Perform callback.
 *
 * Assumptions:
 *   This function does not execute in the context of an interrupt handler.
 *   It may be invoked on any user thread or scheduled on the work thread
 *   from an interrupt handler.
 *
 ****************************************************************************/

static void imxrt_callback(void *arg)
{
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)arg;

  /* Is a callback registered? */

  DEBUGASSERT(priv != NULL);
  mcinfo("Callback %p(%p) cbevents: %02x cdstatus: %02x\n", priv->callback,
         priv->cbarg, priv->cbevents, priv->cdstatus);

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

          mcinfo("Callback to %p(%p)\n",
                 priv->callback, priv->cbarg);

          priv->callback(priv->cbarg);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_usdhc_set_sdio_card_isr
 *
 * Description:
 *   SDIO card generates interrupt via SDIO_DATA_1 pin.
 *   Called by board-specific logic to register an ISR for SDIO card.
 *
 * Input Parameters:
 *   func  - callback function.
 *   arg   - arg to be passed to the function.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imxrt_usdhc_set_sdio_card_isr(struct sdio_dev_s *dev,
                                   int (*func)(void *), void *arg)
{
  irqstate_t flags;
  uint32_t regval;
  struct imxrt_dev_s *priv = (struct imxrt_dev_s *)dev;

  priv->do_sdio_card = func;
  priv->do_sdio_arg = arg;

  if (priv->do_sdio_card != NULL)
    {
      priv->cintints = USDHC_INT_CINT;
    }
  else
    {
      priv->cintints = 0;
    }

#if defined(CONFIG_MMCSD_HAVE_CARDDETECT)
  if (priv->sw_cd_gpio == 0)
    {
      priv->cintints |= USDHC_INT_CINS | USDHC_INT_CRM;
    }
#endif

  flags  = enter_critical_section();
  regval = getreg32(priv->addr + IMXRT_USDHC_IRQSIGEN_OFFSET);
  regval = (regval & ~USDHC_INT_CINT) | priv->cintints;
  putreg32(regval, priv->addr + IMXRT_USDHC_IRQSIGEN_OFFSET);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: imxrt_sdhc_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Slot to be used
 *
 * Returned Value:
 *   A reference to an SDIO interface structure.
 *   NULL is returned on failures.
 *
 ****************************************************************************/

struct sdio_dev_s *imxrt_usdhc_initialize(int slotno)
{
  DEBUGASSERT(slotno < IMXRT_MAX_SDHC_DEV_SLOTS);
  struct imxrt_dev_s *priv = &g_sdhcdev[slotno];

  /* Initialize the USDHC slot structure data structure */

  switch (priv->addr)
    {
    case IMXRT_USDHC1_BASE:
      /* Configure pins for 1 or 4-bit, wide-bus operation (the chip is
       * capable of 8-bit wide bus operation but D4-D7 are not configured).
       * If bus is multiplexed then there is a custom bus configuration
       * utility in the scope of the board support package.
       */

#ifndef CONFIG_SDIO_MUXBUS
#if defined(CONFIG_IMXRT_USDHC1_WIDTH_D1_D4)
      imxrt_config_gpio(PIN_USDHC1_D1);
      imxrt_config_gpio(PIN_USDHC1_D2);
      imxrt_config_gpio(PIN_USDHC1_D3);
#endif
      /* Clocking and CMD pins (all data widths) */

      imxrt_config_gpio(PIN_USDHC1_D0);
      imxrt_config_gpio(PIN_USDHC1_DCLK);
      imxrt_config_gpio(PIN_USDHC1_CMD);
#endif

#if defined(CONFIG_MMCSD_HAVE_CARDDETECT)
#  if defined(PIN_USDHC1_CD)
      imxrt_config_gpio(PIN_USDHC1_CD);
#  else
      if (priv->sw_cd_gpio != 0)
        {
          imxrt_config_gpio(priv->sw_cd_gpio);
        }
#  endif
#endif

      imxrt_clockall_usdhc1();
      break;

#if defined(CONFIG_IMXRT_USDHC2)
    case IMXRT_USDHC2_BASE:
      imxrt_config_gpio(PIN_USDHC2_D0);
      imxrt_config_gpio(PIN_USDHC2_DCLK);
      imxrt_config_gpio(PIN_USDHC2_CMD);

#  if defined(CONFIG_IMXRT_USDHC2_WIDTH_D1_D4) || defined(CONFIG_IMXRT_USDHC2_WIDTH_D1_D8)
      imxrt_config_gpio(PIN_USDHC2_D1);
      imxrt_config_gpio(PIN_USDHC2_D2);
      imxrt_config_gpio(PIN_USDHC2_D3);
#  endif

#  if defined(CONFIG_IMXRT_USDHC2_WIDTH_D1_D8)
      imxrt_config_gpio(PIN_USDHC2_D4);
      imxrt_config_gpio(PIN_USDHC2_D5);
      imxrt_config_gpio(PIN_USDHC2_D6);
      imxrt_config_gpio(PIN_USDHC2_D7);
#  endif

#  if defined(CONFIG_MMCSD_HAVE_CARDDETECT)
#    if defined(PIN_USDHC2_CD)
      imxrt_config_gpio(PIN_USDHC2_CD);
#    else
      if (priv->sw_cd_gpio != 0)
        {
          imxrt_config_gpio(priv->sw_cd_gpio);
        }
#    endif
#  endif

      imxrt_clockall_usdhc2();
      break;
#endif
    default:
      return NULL;
    }

  imxrt_reset(&priv->dev);
  imxrt_showregs(priv, "After reset");

  return &g_sdhcdev[slotno].dev;
}

#endif /* CONFIG_IMXRT_USDHC */
