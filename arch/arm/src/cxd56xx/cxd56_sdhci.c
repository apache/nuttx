/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_sdhci.c
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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/param.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/mmcsd.h>
#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "cxd56_sdhci.h"
#include "cxd56_clock.h"
#include "cxd56_pinconfig.h"

#ifdef CONFIG_CXD56_SDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_CXD56_SDIO_DMA
#  warning "Large Non-DMA transfer may result in RX overrun failures"
#endif

#if !defined(CONFIG_SCHED_WORKQUEUE) || !defined(CONFIG_SCHED_HPWORK)
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE and CONFIG_SCHED_HPWORK"
#endif

#if (CONFIG_MMCSD_MULTIBLOCK_LIMIT != 1) && !defined(CONFIG_SDIO_BLOCKSETUP)
#  error "This driver requires CONFIG_SDIO_BLOCKSETUP"
#endif

#ifndef CONFIG_CXD56_SDHCI_PRIO
#  define CONFIG_CXD56_SDHCI_PRIO NVIC_SYSH_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_CXD56_SDHCI_DMAPRIO
#  define CONFIG_CXD56_SDHCI_DMAPRIO DMA_CCR_PRIMED
#endif

#ifndef CONFIG_DEBUG_MEMCARD_INFO
#  undef CONFIG_SDIO_XFRDEBUG
#endif

/* SDCLK frequencies corresponding to various modes of operation.  These
 * values may be provided in either the NuttX configuration file or in
 * the board.h file
 *
 * NOTE:  These settings are not currently used.  Since there are only four
 * frequencies, it makes more sense to just "can" the fixed frequency
 * prescaler and divider values.
 */

#ifndef CONFIG_CXD56_IDMODE_FREQ
#  define CONFIG_CXD56_IDMODE_FREQ 400000    /* 400 KHz, ID mode */
#endif
#ifndef CONFIG_CXD56_MMCXFR_FREQ
#  define CONFIG_CXD56_MMCXFR_FREQ 20000000  /* 20MHz MMC, normal clocking */
#endif
#ifndef CONFIG_CXD56_SD1BIT_FREQ
#  define CONFIG_CXD56_SD1BIT_FREQ 20000000  /* 20MHz SD 1-bit, normal clocking */
#endif
#ifndef CONFIG_CXD56_SD4BIT_FREQ
#  define CONFIG_CXD56_SD4BIT_FREQ 50000000  /* SDR25 SD 4-bit, normal clocking */
#endif

/* Timing */

#define SDHCI_CARDSTATETIMEOUT   (2000000)
#define SDHCI_CMDTIMEOUT         (100000)
#define SDHCI_LONGTIMEOUT        (1000000)

#define SDHCI_WAIT_POWERON      MSEC2TICK(252)
#define SDHCI_WAIT_POWEROFF     MSEC2TICK(300)

/* Big DVS setting.  Range is 0=SDCLK*213 through 14=SDCLK*227 */

#define SDHCI_DTOCV_MAXTIMEOUT     (0xF-0x1)
#define SDHCI_DTOCV_DATATIMEOUT    (0XF-0x1)

/* Data transfer / Event waiting interrupt mask bits */

#define SDHCI_RESPERR_INTS  (SDHCI_INT_CCE|SDHCI_INT_CTOE|SDHCI_INT_CEBE|SDHCI_INT_CIE)
#define SDHCI_RESPDONE_INTS (SDHCI_RESPERR_INTS|SDHCI_INT_CC)

#define SDHCI_XFRERR_INTS   (SDHCI_INT_DCE|SDHCI_INT_DTOE|SDHCI_INT_DEBE)
#define SDHCI_RCVDONE_INTS  (SDHCI_XFRERR_INTS|SDHCI_INT_BRR|SDHCI_INT_TC)
#define SDHCI_SNDDONE_INTS  (SDHCI_XFRERR_INTS|SDHCI_INT_BWR|SDHCI_INT_TC)
#define SDHCI_XFRDONE_INTS  (SDHCI_XFRERR_INTS|SDHCI_INT_BRR|SDHCI_INT_BWR|SDHCI_INT_TC)

#define SDHCI_DMAERR_INTS   (SDHCI_INT_DCE|SDHCI_INT_DTOE|SDHCI_INT_DEBE|SDHCI_INT_DMAE)
#define SDHCI_DMADONE_INTS  (SDHCI_DMAERR_INTS|SDHCI_INT_DINT|SDHCI_INT_TC)

#define SDHCI_WAITALL_INTS  (SDHCI_RESPDONE_INTS|SDHCI_XFRDONE_INTS|SDHCI_DMADONE_INTS)

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
#  define SAMPLENDX_BEFORE_SETUP  0
#  define SAMPLENDX_AFTER_SETUP   1
#  define SAMPLENDX_END_TRANSFER  2
#  define DEBUG_NSAMPLES          3
#endif

/* DMA */

#define CXD56_SDHCI_BUF_SIZE      (2048)
#define SDHCI_MAX_BLOCK_COUNT     (0xffffffff)
#define SDHCI_MAX_ADMA_TRANS_SIZE (0xffff+1)
#ifndef CONFIG_CXD56_SDIO_MAX_LEN_ADMA_DSCR
#  define CXD56_SDIO_MAX_LEN_ADMA_DSCR (16)
#else
#  define CXD56_SDIO_MAX_LEN_ADMA_DSCR (CONFIG_CXD56_SDIO_MAX_LEN_ADMA_DSCR)
#endif

#define SDHCI_ADMA_DSCR_L_LEN_MASK  (0xffff0000)
#define SDHCI_ADMA_DSCR_L_LEN_SHIFT (16)
#define SDHCI_ADMA_DSCR_L_ACT_SHIFT (4)
#define SDHCI_ADMA_DSCR_L_ACT_NOP   (0<<4)
#define SDHCI_ADMA_DSCR_L_ACT_RSV   (1<<4)
#define SDHCI_ADMA_DSCR_L_ACT_TRAN  (2<<4)
#define SDHCI_ADMA_DSCR_L_ACT_LNK   (3<<4)
#define SDHCI_ADMA_DSCR_L_INT       (0x00000004)
#define SDHCI_ADMA_DSCR_L_END       (0x00000002)
#define SDHCI_ADMA_DSCR_L_VALID     (0x00000001)
#define SDHCI_ADMA_DSCR_H_ADDR_MASK (0xffffffff)

#define CXD56_SDHCI_ADSADDR_H (CXD56_SDHCI_ADSADDR+0x4)

/* Card Common Control Registers (CCCR) */

#define SDIO_CCCR_SIZE          0x100

#define SDIO_CMD5253_READ       (0<<31)
#define SDIO_CMD5253_WRITE      (1<<31)
#define SDIO_CMD5253_FUNC_SHIFT (28)

#define SDIO_CMD52_EXCHANGE     (1<<27)
#define SDIO_CMD52_REG_SHIFT    (9)
#define SDIO_CMD52_DATA_MASK    0xff

#define CMD52_RESP_OK(resp)     (0 == (resp&0xCB00))

/* For Function Basic Registers */

#define SDIO_FBR_START          0x100

/* Card Information Structure (CIS) */

#define SDIO_CIS_START          0x1000
#define SDIO_CIS_END            (SDIO_CIS_START+0x17000-0x10)

/* CIS tuple codes (based on PC Card 16) */

#define SDIO_CISTPL_NULL        0x00
#define SDIO_CISTPL_VERS_1      0x15
#define SDIO_CISTPL_MANFID      0x20
#define SDIO_CISTPL_FUNCID      0x21
#define SDIO_CISTPL_FUNCE       0x22
#define SDIO_CISTPL_END         0xff

/* CISTPL_FUNCID codes */

#define TPLFID_FUNC_SDIO        0x0c

#define SDIO_THREAD_DEFPRIO     50
#define SDIO_THREAD_STACKSIZE   1024

#define SDIO_OCR_NUM_FUNCTIONS(ocr) (((ocr) >> 28) & 0x7)
#define SDIO_FUNC_NUM_MAX       (7)

#define SDIO_BLOCK_TIMEOUT      200
#define SDIO_BLOCK_SIZE         512

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the state of the CXD56xx SDIO interface */

struct cxd56_sdiodev_s
{
  struct sdio_dev_s  dev;             /* Standard, base SDIO interface */

  /* CXD56xx-specific extensions */

  /* Event support */

  sem_t              waitsem;         /* Implements event waiting */
  sdio_eventset_t    waitevents;      /* Set of events to be waited for */
  uint32_t           waitints;        /* Interrupt enables for event waiting */
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
  size_t             remaining;       /* Number of bytes remaining in the
                                       * transfer */
  uint32_t           xfrints;         /* Interrupt enables for data transfer */

  /* DMA data transfer support */

#ifdef CONFIG_SDIO_DMA
  volatile uint8_t   xfrflags;        /* Used to synchronize SDIO and DMA
                                       * completion events */
  bool usedma;
  bool dmasend_prepare;
  size_t   receive_size;
  uint8_t *aligned_buffer;            /* Used to buffer alignment */
  uint8_t *receive_buffer;            /* Used to keep receive buffer address */
  uint32_t dma_cmd;
  uint32_t dmasend_cmd;
  uint32_t dmasend_regcmd;
#endif

  /* Parameters */

  uint16_t blocksize;
};

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
struct cxd56_sdhcregs_s
{
  /* All read-able SDHC registers */

  uint32_t dsaddr;    /* DMA System Address Register */
  uint32_t blkattr;   /* Block Attributes Register */
  uint32_t cmdarg;    /* Command Argument Register */
  uint32_t xferty;    /* Transfer Type Register */
  uint32_t cmdrsp0;   /* Command Response 0 */
  uint32_t cmdrsp1;   /* Command Response 1 */
  uint32_t cmdrsp2;   /* Command Response 2 */
  uint32_t cmdrsp3;   /* Command Response 3 */
  uint32_t prsstat;   /* Present State Register */
  uint32_t proctl;    /* Protocol Control Register */
  uint32_t sysctl;    /* System Control Register */
  uint32_t irqstat;   /* Interrupt Status Register */
  uint32_t irqstaten; /* Interrupt Status Enable Register */
  uint32_t irqsigen;  /* Interrupt Signal Enable Register */
  uint32_t ac12err;   /* Auto CMD12 Error Status Register */
  uint32_t htcapblt;  /* Host Controller Capabilities */
  uint32_t admaes;    /* ADMA Error Status Register */
  uint32_t adsaddr;   /* ADMA System Address Register */
  uint32_t vendspec;  /* Vendor Specific Register */
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static void cxd56_configwaitints(struct cxd56_sdiodev_s *priv,
              uint32_t waitints, sdio_eventset_t waitevents,
              sdio_eventset_t wkupevents);
static void cxd56_configxfrints(struct cxd56_sdiodev_s *priv,
              uint32_t xfrints);

/* DMA Helpers **************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void cxd56_sampleinit(void);
static void cxd56_sdhcsample(struct cxd56_sdhcregs_s *regs);
static void cxd56_sample(struct cxd56_sdiodev_s *priv, int index);
static void cxd56_dumpsample(struct cxd56_sdiodev_s *priv,
              struct cxd56_sdhcregs_s *regs, const char *msg);
static void cxd56_dumpsamples(struct cxd56_sdiodev_s *priv);
static void cxd56_showregs(struct cxd56_sdiodev_s *priv, const char *msg);
#else
#  define   cxd56_sampleinit()
#  define   cxd56_sample(priv,index)
#  define   cxd56_dumpsamples(priv)
#  define   cxd56_showregs(priv,msg)
#endif

/* Data Transfer Helpers ****************************************************/

static void cxd56_dataconfig(struct cxd56_sdiodev_s *priv, bool bwrite,
                               unsigned int blocksize, unsigned int nblocks,
                               unsigned int timeout);
static void cxd56_datadisable(void);
#ifndef CONFIG_SDIO_DMA
static void cxd56_transmit(struct cxd56_sdiodev_s *priv);
static void cxd56_receive(struct cxd56_sdiodev_s *priv);
#endif
static void cxd56_eventtimeout(wdparm_t arg);
static void cxd56_endwait(struct cxd56_sdiodev_s *priv,
              sdio_eventset_t wkupevent);
static void cxd56_endtransfer(struct cxd56_sdiodev_s *priv,
              sdio_eventset_t wkupevent);

/* Interrupt Handling *******************************************************/

static int cxd56_interrupt(int irq, void *context, void *arg);

/* SDIO interface methods ***************************************************/

/* Mutual exclusion */

#ifdef CONFIG_SDIO_MUXBUS
static int cxd56_sdio_lock(struct sdio_dev_s *dev, bool lock);
#endif

/* Initialization/setup */

static void cxd56_sdio_sdhci_reset(struct sdio_dev_s *dev);
static sdio_capset_t cxd56_sdio_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t cxd56_sdio_status(struct sdio_dev_s *dev);
static void cxd56_sdio_widebus(struct sdio_dev_s *dev, bool enable);
static void cxd56_sdio_frequency(uint32_t frequency);
static void cxd56_sdio_clock(struct sdio_dev_s *dev,
              enum sdio_clock_e rate);
static int  cxd56_sdio_attach(struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int  cxd56_sdio_sendcmd(struct sdio_dev_s *dev,
              uint32_t cmd, uint32_t arg);
static void cxd56_blocksetup(struct sdio_dev_s *dev,
              unsigned int blocklen, unsigned int nblocks);
#ifndef CONFIG_SDIO_DMA
static int  cxd56_sdio_recvsetup(struct sdio_dev_s *dev,
              uint8_t *buffer, size_t nbytes);
static int  cxd56_sdio_sendsetup(struct sdio_dev_s *dev,
              const uint8_t *buffer, uint32_t nbytes);
#endif
static int  cxd56_sdio_cancel(struct sdio_dev_s *dev);

static int  cxd56_sdio_waitresponse(struct sdio_dev_s *dev,
              uint32_t cmd);
static int  cxd56_sdio_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  cxd56_sdio_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t rlong[4]);
static int  cxd56_sdio_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);

/* EVENT handler */

static void cxd56_sdio_waitenable(struct sdio_dev_s *dev,
              sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t cxd56_sdio_eventwait(struct sdio_dev_s *dev);
static void cxd56_sdio_callbackenable(struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static int  cxd56_sdio_registercallback(struct sdio_dev_s *dev,
              worker_t callback, void *arg);

/* DMA */

#ifdef CONFIG_SDIO_DMA
static int  cxd56_sdio_admasetup(const uint8_t *buffer, size_t buflen);
static int  cxd56_sdio_dmarecvsetup(struct sdio_dev_s *dev,
              uint8_t *buffer, size_t buflen);
static int  cxd56_sdio_dmasendsetup(struct sdio_dev_s *dev,
              const uint8_t *buffer, size_t buflen);
#endif

/* Initialization/uninitialization/reset ************************************/

static void cxd56_sdio_callback(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct cxd56_sdiodev_s g_sdhcdev =
{
  .dev =
    {
#ifdef CONFIG_SDIO_MUXBUS
      .lock             = cxd56_sdio_lock,
#endif
      .reset            = cxd56_sdio_sdhci_reset,
      .capabilities     = cxd56_sdio_capabilities,
      .status           = cxd56_sdio_status,
      .widebus          = cxd56_sdio_widebus,
      .clock            = cxd56_sdio_clock,
      .attach           = cxd56_sdio_attach,
      .sendcmd          = cxd56_sdio_sendcmd,
      .blocksetup       = cxd56_blocksetup,
#ifndef CONFIG_SDIO_DMA
      .recvsetup        = cxd56_sdio_recvsetup,
      .sendsetup        = cxd56_sdio_sendsetup,
#else
      .recvsetup        = cxd56_sdio_dmarecvsetup,
      .sendsetup        = cxd56_sdio_dmasendsetup,
#endif
      .cancel           = cxd56_sdio_cancel,
      .waitresponse     = cxd56_sdio_waitresponse,
      .recv_r1          = cxd56_sdio_recvshortcrc,
      .recv_r2          = cxd56_sdio_recvlong,
      .recv_r3          = cxd56_sdio_recvshort,
      .recv_r4          = cxd56_sdio_recvshort,
      .recv_r5          = cxd56_sdio_recvshort,
      .recv_r6          = cxd56_sdio_recvshortcrc,
      .recv_r7          = cxd56_sdio_recvshort,
      .waitenable       = cxd56_sdio_waitenable,
      .eventwait        = cxd56_sdio_eventwait,
      .callbackenable   = cxd56_sdio_callbackenable,
      .registercallback = cxd56_sdio_registercallback,
#ifdef CONFIG_SDIO_DMA
      .dmarecvsetup     = cxd56_sdio_dmarecvsetup,
      .dmasendsetup     = cxd56_sdio_dmasendsetup,
#else
      .dmarecvsetup     = cxd56_sdio_recvsetup,
      .dmasendsetup     = cxd56_sdio_sendsetup,
#endif
    },
  .waitsem = SEM_INITIALIZER(0),
};

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
static struct cxd56_sdhcregs_s g_sampleregs[DEBUG_NSAMPLES];
#endif

/* DMA */

#ifdef CONFIG_SDIO_DMA
static uint32_t cxd56_sdhci_adma_dscr[CXD56_SDIO_MAX_LEN_ADMA_DSCR * 2];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_configwaitints
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

static void cxd56_configwaitints(struct cxd56_sdiodev_s *priv,
                                 uint32_t waitints,
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
  priv->waitints   = waitints;
#ifdef CONFIG_SDIO_DMA
  priv->xfrflags   = 0;
#endif
  putreg32(priv->xfrints | priv->waitints | SDHCI_INT_CINT,
           CXD56_SDHCI_IRQSIGEN);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: cxd56_configxfrints
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

static void cxd56_configxfrints(struct cxd56_sdiodev_s *priv,
            uint32_t xfrints)
{
  irqstate_t flags;
  flags = enter_critical_section();
  priv->xfrints = xfrints;
  putreg32(priv->xfrints | priv->waitints | SDHCI_INT_CINT,
           CXD56_SDHCI_IRQSIGEN);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: cxd56_sampleinit
 *
 * Description:
 *   Setup prior to collecting DMA samples
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void cxd56_sampleinit(void)
{
  memset(g_sampleregs, 0xff, DEBUG_NSAMPLES *
         sizeof(struct cxd56_sdhcregs_s));
}
#endif

/****************************************************************************
 * Name: cxd56_sdhcsample
 *
 * Description:
 *   Sample SDIO registers
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void cxd56_sdhcsample(struct cxd56_sdhcregs_s *regs)
{
  regs->dsaddr    = getreg32(CXD56_SDHCI_DSADDR);    /* DMA System Address Register */
  regs->blkattr   = getreg32(CXD56_SDHCI_BLKATTR);   /* Block Attributes Register */
  regs->cmdarg    = getreg32(CXD56_SDHCI_CMDARG);    /* Command Argument Register */
  regs->xferty    = getreg32(CXD56_SDHCI_XFERTYP);   /* Transfer Type Register */
  regs->cmdrsp0   = getreg32(CXD56_SDHCI_CMDRSP0);   /* Command Response 0 */
  regs->cmdrsp1   = getreg32(CXD56_SDHCI_CMDRSP1);   /* Command Response 1 */
  regs->cmdrsp2   = getreg32(CXD56_SDHCI_CMDRSP2);   /* Command Response 2 */
  regs->cmdrsp3   = getreg32(CXD56_SDHCI_CMDRSP3);   /* Command Response 3 */
  regs->prsstat   = getreg32(CXD56_SDHCI_PRSSTAT);   /* Present State Register */
  regs->proctl    = getreg32(CXD56_SDHCI_PROCTL);    /* Protocol Control Register */
  regs->sysctl    = getreg32(CXD56_SDHCI_SYSCTL);    /* System Control Register */
  regs->irqstat   = getreg32(CXD56_SDHCI_IRQSTAT);   /* Interrupt Status Register */
  regs->irqstaten = getreg32(CXD56_SDHCI_IRQSTATEN); /* Interrupt Status Enable Register */
  regs->irqsigen  = getreg32(CXD56_SDHCI_IRQSIGEN);  /* Interrupt Signal Enable Register */
  regs->ac12err   = getreg32(CXD56_SDHCI_AC12ERR);   /* Auto CMD12 Error Status Register */
  regs->htcapblt  = getreg32(CXD56_SDHCI_HTCAPBLT);  /* Host Controller Capabilities */
  regs->admaes    = getreg32(CXD56_SDHCI_ADMAES);    /* ADMA Error Status Register */
  regs->adsaddr   = getreg32(CXD56_SDHCI_ADSADDR);   /* ADMA System Address Register */
  regs->vendspec  = getreg32(CXD56_SDHCI_VENDSPEC);  /* Vendor Specific Register */
}
#endif

/****************************************************************************
 * Name: cxd56_sample
 *
 * Description:
 *   Sample SDIO/DMA registers
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void cxd56_sample(struct cxd56_sdiodev_s *priv, int index)
{
  cxd56_sdhcsample(&g_sampleregs[index]);
}
#endif

/****************************************************************************
 * Name: cxd56_dumpsample
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void cxd56_dumpsample(struct cxd56_sdiodev_s *priv,
                             struct cxd56_sdhcregs_s *regs,
                             const char *msg)
{
  mcinfo("SDHC Registers: %s\n", msg);
  mcinfo("   DSADDR[%08x]: %08x\n", CXD56_SDHCI_DSADDR,    regs->dsaddr);
  mcinfo("  BLKATTR[%08x]: %08x\n", CXD56_SDHCI_BLKATTR,   regs->blkattr);
  mcinfo("   CMDARG[%08x]: %08x\n", CXD56_SDHCI_CMDARG,    regs->cmdarg);
  mcinfo("  COMMAND[%08x]: %08x\n", CXD56_SDHCI_XFERTYP,   regs->xferty);
  mcinfo("  CMDRSP0[%08x]: %08x\n", CXD56_SDHCI_CMDRSP0,   regs->cmdrsp0);
  mcinfo("  CMDRSP1[%08x]: %08x\n", CXD56_SDHCI_CMDRSP1,   regs->cmdrsp1);
  mcinfo("  CMDRSP2[%08x]: %08x\n", CXD56_SDHCI_CMDRSP2,   regs->cmdrsp2);
  mcinfo("  CMDRSP3[%08x]: %08x\n", CXD56_SDHCI_CMDRSP3,   regs->cmdrsp3);
  mcinfo("  PRSSTAT[%08x]: %08x\n", CXD56_SDHCI_PRSSTAT,   regs->prsstat);
  mcinfo("   PROCTL[%08x]: %08x\n", CXD56_SDHCI_PROCTL,    regs->proctl);
  mcinfo("  HOSTCTL[%08x]: %08x\n", CXD56_SDHCI_SYSCTL,    regs->sysctl);
  mcinfo("  IRQSTAT[%08x]: %08x\n", CXD56_SDHCI_IRQSTAT,   regs->irqstat);
  mcinfo("IRQSTATEN[%08x]: %08x\n", CXD56_SDHCI_IRQSTATEN, regs->irqstaten);
  mcinfo(" IRQSIGEN[%08x]: %08x\n", CXD56_SDHCI_IRQSIGEN,  regs->irqsigen);
  mcinfo("  AC12ERR[%08x]: %08x\n", CXD56_SDHCI_AC12ERR,   regs->ac12err);
  mcinfo(" HTCAPBLT[%08x]: %08x\n", CXD56_SDHCI_HTCAPBLT,  regs->htcapblt);
  mcinfo("   ADMAES[%08x]: %08x\n", CXD56_SDHCI_ADMAES,    regs->admaes);
  mcinfo("  ADSADDR[%08x]: %08x\n", CXD56_SDHCI_ADSADDR,   regs->adsaddr);
  mcinfo(" VENDSPEC[%08x]: %08x\n", CXD56_SDHCI_VENDSPEC,  regs->vendspec);
}
#endif

/****************************************************************************
 * Name: cxd56_dumpsamples
 *
 * Description:
 *   Dump all sampled register data
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void  cxd56_dumpsamples(struct cxd56_sdiodev_s *priv)
{
  cxd56_dumpsample(priv, &g_sampleregs[SAMPLENDX_BEFORE_SETUP],
                   "Before setup");
  cxd56_dumpsample(priv, &g_sampleregs[SAMPLENDX_AFTER_SETUP],
                   "After setup");
  cxd56_dumpsample(priv, &g_sampleregs[SAMPLENDX_END_TRANSFER],
                   "End of transfer");
}
#endif

/****************************************************************************
 * Name: cxd56_showregs
 *
 * Description:
 *   Dump the current state of all registers
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void cxd56_showregs(struct cxd56_sdiodev_s *priv, const char *msg)
{
  struct cxd56_sdhcregs_s regs;

  cxd56_sdhcsample(&regs);
  cxd56_dumpsample(priv, &regs, msg);
}
#endif

/****************************************************************************
 * Name: cxd56_dataconfig
 *
 * Description:
 *   Configure the SDIO data path for the next data transfer
 *
 ****************************************************************************/

static void cxd56_dataconfig(struct cxd56_sdiodev_s *priv, bool bwrite,
                               unsigned int blocksize, unsigned int nblocks,
                               unsigned int timeout)
{
  uint32_t regval = 0;

  /* Set the data timeout value in the SDHCI_SYSCTL field to the selected
   * value.
   */

  regval  = getreg32(CXD56_SDHCI_SYSCTL);
  regval &= ~SDHCI_SYSCTL_DTOCV_MASK;
  regval |= timeout << SDHCI_SYSCTL_DTOCV_SHIFT;
  putreg32(regval, CXD56_SDHCI_SYSCTL);

  /* Set the block size and count in the SDHCI_BLKATTR register.  The block
   * size is only valid for multiple block transfers.
   */

  priv->blocksize = blocksize;

  regval = blocksize << SDHCI_BLKATTR_SIZE_SHIFT |
           nblocks   << SDHCI_BLKATTR_CNT_SHIFT;
  putreg32(regval, CXD56_SDHCI_BLKATTR);
}

/****************************************************************************
 * Name: cxd56_datadisable
 *
 * Description:
 *   Disable the SDIO data path setup by cxd56_dataconfig() and
 *   disable DMA.
 *
 ****************************************************************************/

static void cxd56_datadisable(void)
{
  uint32_t regval;

  /* Set the data timeout value in the SDHCI_SYSCTL field to the maximum
   * value.
   */

  regval  = getreg32(CXD56_SDHCI_SYSCTL);
  regval &= ~SDHCI_SYSCTL_DTOCV_MASK;
  regval |= SDHCI_DTOCV_MAXTIMEOUT << SDHCI_SYSCTL_DTOCV_SHIFT;
  putreg32(regval, CXD56_SDHCI_SYSCTL);

  /* Set the block size to zero (no transfer) */

  putreg32(0, CXD56_SDHCI_BLKATTR);
}

/****************************************************************************
 * Name: cxd56_transmit
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

#ifndef CONFIG_SDIO_DMA
static void cxd56_transmit(struct cxd56_sdiodev_s *priv)
{
  union
    {
      uint32_t w;
      uint8_t  b[4];
    } data;

  /* Loop while there is more data to be sent, waiting for buffer write
   * ready (BWR)
   */

  if (priv->buffer == 0)
    {
      return;
    }

  mcinfo("Entry: remaining: %d IRQSTAT: %08x\n",
          priv->remaining, getreg32(CXD56_SDHCI_IRQSTAT));

  while (priv->remaining > 0 &&
         (getreg32(CXD56_SDHCI_IRQSTAT) & SDHCI_INT_BWR) != 0)
    {
      /* Clear BWR.  If there is more data in the buffer, writing to the
       * buffer should reset BRR.
       */

      putreg32(SDHCI_INT_BWR, CXD56_SDHCI_IRQSTAT);

      while (priv->remaining > 0 &&
            (getreg32(CXD56_SDHCI_PRSSTAT) & SDHCI_PRSSTAT_BWEN) != 0)
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

              uint8_t *ptr = (uint8_t *)priv->buffer;
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

          putreg32(data.w, CXD56_SDHCI_DATPORT);
        }
    }

  mcinfo("Exit: remaining: %d IRQSTAT: %08x\n",
          priv->remaining, getreg32(CXD56_SDHCI_IRQSTAT));
}
#endif

/****************************************************************************
 * Name: cxd56_receive
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

#ifndef CONFIG_SDIO_DMA
static void cxd56_receive(struct cxd56_sdiodev_s *priv)
{
  union
    {
      uint32_t w;
      uint8_t  b[4];
    } data;

  /* Loop while there is space to store the data, waiting for buffer read
   * ready (BRR)
   */

  if (priv->buffer == 0)
    {
      return;
    }

  mcinfo("Entry: remaining: %d IRQSTAT: %08x\n",
          priv->remaining, getreg32(CXD56_SDHCI_IRQSTAT));

  while (priv->remaining > 0 &&
         (getreg32(CXD56_SDHCI_IRQSTAT) & SDHCI_INT_BRR) != 0)
    {
      /* Clear BRR.  If there is more data in the buffer, reading from the
       * buffer should reset BRR.
       */

      putreg32(SDHCI_INT_BRR, CXD56_SDHCI_IRQSTAT);

      while (priv->remaining > 0 &&
            (getreg32(CXD56_SDHCI_PRSSTAT) & SDHCI_PRSSTAT_BREN) != 0)
        {
          /* Read the next word from the RX buffer */

          data.w = getreg32(CXD56_SDHCI_DATPORT);
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
    }
}
#endif

/****************************************************************************
 * Name: cxd56_eventtimeout
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

static void cxd56_eventtimeout(wdparm_t arg)
{
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)arg;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0);

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. Sample registers at the time of the timeout */

      cxd56_sample(priv, SAMPLENDX_END_TRANSFER);

      /* Wake up any waiting threads */

      cxd56_endwait(priv, SDIOWAIT_TIMEOUT);
      mcwarn("Timeout: remaining: %d\n", priv->remaining);
    }
}

/****************************************************************************
 * Name: cxd56_endwait
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

static void cxd56_endwait(struct cxd56_sdiodev_s *priv,
                          sdio_eventset_t wkupevent)
{
  /* Cancel the watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Disable event-related interrupts */

  cxd56_configwaitints(priv, 0, 0, wkupevent);

  /* Wake up the waiting thread */

  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: cxd56_endtransfer
 *
 * Description:
 *   Terminate a transfer with the provided status.  This function is called
 *   only from the SDIO interrupt handler when end-of-transfer conditions
 *   are detected.
 *
 * Input Parameters:
 *   priv   - An instance of the SDIO device interface
 *   wkupevent - The event that caused the transfer to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void cxd56_endtransfer(struct cxd56_sdiodev_s *priv,
                              sdio_eventset_t wkupevent)
{
#ifdef CONFIG_SDIO_DMA
  uint32_t regval;
#endif

  if (priv->buffer == 0)
    {
      return;
    }

  /* Disable all transfer related interrupts */

  cxd56_configxfrints(priv, 0);

  /* Clearing pending interrupt status on all transfer related interrupts */

  putreg32(SDHCI_XFRDONE_INTS, CXD56_SDHCI_IRQSTAT);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_SDIO_DMA
  /* Stop the DMA by resetting the data path */

  regval = getreg32(CXD56_SDHCI_SYSCTL);
  regval |= SDHCI_SYSCTL_RSTD;
  putreg32(regval, CXD56_SDHCI_SYSCTL);
  cxd56_sdhci_adma_dscr[0] = 0;
  cxd56_sdhci_adma_dscr[1] = 0;
  putreg32(CXD56_PHYSADDR(cxd56_sdhci_adma_dscr), CXD56_SDHCI_ADSADDR);
  putreg32(0, CXD56_SDHCI_ADSADDR_H);
  priv->usedma = false;
  priv->dmasend_prepare = false;
  priv->dmasend_cmd = 0;
  priv->dmasend_regcmd = 0;
#endif

  /* Mark the transfer finished */

  if ((priv->waitevents & wkupevent &
      (SDIOWAIT_TRANSFERDONE | SDIOWAIT_RESPONSEDONE)) == 0)
    {
      priv->remaining = 0;
    }

  priv->buffer = 0;

  /* Debug instrumentation */

  cxd56_sample(priv, SAMPLENDX_END_TRANSFER);

  /* Is a thread wait for these data transfer complete events? */

  if ((priv->waitevents & wkupevent) != 0)
    {
      /* Yes.. wake up any waiting threads */

      cxd56_endwait(priv, wkupevent);
    }
}

/****************************************************************************
 * Name: cxd56_interrupt
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

static int cxd56_interrupt(int irq, void *context, void *arg)
{
  struct cxd56_sdiodev_s *priv = &g_sdhcdev;
  uint32_t enabled;
  uint32_t pending;
  uint32_t regval;

  /* Check the SDHC IRQSTAT register.  Mask out all bits that don't
   * correspond to enabled interrupts.  (This depends on the fact that bits
   * are ordered the same in both the IRQSTAT and IRQSIGEN registers).  If
   * there are non-zero bits remaining, then we have work to do here.
   */

  regval  = getreg32(CXD56_SDHCI_IRQSIGEN);
  enabled = getreg32(CXD56_SDHCI_IRQSTAT) & regval;
  mcinfo("IRQSTAT: %08" PRIx32 " IRQSIGEN %08" PRIx32
         " enabled: %08" PRIx32 "\n",
         getreg32(CXD56_SDHCI_IRQSTAT), regval, enabled);

  /* Disable card interrupts to clear the card interrupt to the host
   * system.
   */

  regval &= ~SDHCI_INT_CINT;
  putreg32(regval, CXD56_SDHCI_IRQSIGEN);

  /* Clear all pending interrupts */

  putreg32(enabled, CXD56_SDHCI_IRQSTAT);

  /* Handle in progress, interrupt driven data transfers ********************/

  pending  = enabled & priv->xfrints;
  if (pending != 0)
    {
#ifndef CONFIG_SDIO_DMA
      /* Is the RX buffer read ready?  Is so then we must be processing a
       * non-DMA receive transaction.
       */

      if ((pending & SDHCI_INT_BRR) != 0)
        {
          /* Receive data from the RX buffer */

          cxd56_receive(priv);
        }

      /* Otherwise, Is the TX buffer write ready? If so we must
       * be processing a non-DMA send transaction.  NOTE:  We can't be
       * processing both!
       */

      else if ((pending & SDHCI_INT_BWR) != 0)
        {
          /* Send data via the TX FIFO */

          cxd56_transmit(priv);
        }
#endif

      /* Handle transfer complete events */

      if ((pending & SDHCI_INT_TC) != 0)
        {
          /* Terminate the transfer */

          cxd56_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
        }

      /* Handle data block send/receive CRC failure */

      else if ((pending & SDHCI_INT_DCE) != 0)
        {
          /* Terminate the transfer with an error */

          mcerr("ERROR: Data block CRC failure, remaining: %d\n",
                priv->remaining);
          cxd56_endtransfer(priv, SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
        }

      /* Handle data timeout error */

      else if ((pending & SDHCI_INT_DTOE) != 0)
        {
          /* Terminate the transfer with an error */

          mcerr("ERROR: Data timeout, remaining: %d\n", priv->remaining);
          cxd56_endtransfer(priv, SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT);
        }
    }

  /* Handle error interrupts ************************************************/

  if ((getreg32(CXD56_SDHCI_IRQSTAT) & SDHCI_INT_EINT) != 0)
    {
      /* Clear error interrupts */

      mcerr("ERROR: Occur error interrupts: %08" PRIx32 "\n", enabled);
      putreg32(enabled & SDHCI_EINT_MASK, CXD56_SDHCI_IRQSTAT);
    }

  /* Handle wait events *****************************************************/

  pending  = enabled & priv->waitints;
  if (pending != 0)
    {
      /* Is this a response completion event? */

      if ((pending & SDHCI_RESPDONE_INTS) != 0)
        {
          /* Yes.. Is their a thread waiting for response done? */

          if ((priv->waitevents &
              (SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE)) != 0)
            {
              /* Yes.. mask further interrupts and wake the thread up */

              regval = getreg32(CXD56_SDHCI_IRQSIGEN);
              regval &= ~SDHCI_RESPDONE_INTS;
              putreg32(regval, CXD56_SDHCI_IRQSIGEN);

              cxd56_endwait(priv, SDIOWAIT_RESPONSEDONE);
            }
        }
    }

  /* Re-enable card interrupts */

  regval  = getreg32(CXD56_SDHCI_IRQSIGEN);
  regval |= SDHCI_INT_CINT;
  putreg32(regval, CXD56_SDHCI_IRQSIGEN);

  return OK;
}

/****************************************************************************
 * Name: cxd56_sdio_lock
 *
 * Description:
 *   Locks the bus. Function calls low-level multiplexed bus routines to
 *   resolve bus requests and acknowledgement issues.
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
static int cxd56_sdio_lock(struct sdio_dev_s *dev, bool lock)
{
  /* Enable SD clock only while accessing to the SDIO. */

  if (lock)
    {
      modifyreg32(CXD56_SDHCI_SYSCTL, 0, SDHCI_SYSCTL_SDCLKEN);
    }
  else
    {
      modifyreg32(CXD56_SDHCI_SYSCTL, SDHCI_SYSCTL_SDCLKEN, 0);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: cxd56_sdio_sdhci_reset
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

static void cxd56_sdio_sdhci_reset(struct sdio_dev_s *dev)
{
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;
  uint32_t regval;
  int32_t timeout = 100;

  /* Disable all interrupts so that nothing interferes with the following. */

  putreg32(0, CXD56_SDHCI_IRQSIGEN);

  /* Reset the SDHC block, putting registers in their default, reset state.
   * Initiate the reset by setting the RSTA bit in the SYSCTL register.
   */

  regval  = getreg32(CXD56_SDHCI_SYSCTL);
  regval |= SDHCI_SYSCTL_RSTA;
  putreg32(regval, CXD56_SDHCI_SYSCTL);

  /* The SDHC will reset the RSTA bit to 0 when the capabilities
   * registers are valid and the host driver can read them.
   */

  while ((getreg32(CXD56_SDHCI_SYSCTL) & SDHCI_SYSCTL_RSTA) != 0)
    {
      timeout--;
      if (timeout < 1)
        {
          break;
        }

      up_mdelay(30);
    }

  /* Make sure that all clocking is disabled */

  cxd56_sdio_clock(dev, CLOCK_SDIO_DISABLED);

  /* Enable all status bits (these could not all be potential sources of
   * interrupts.
   */

  putreg32(SDHCI_INT_ALL & (~SDHCI_INT_CINT), CXD56_SDHCI_IRQSTATEN);

  mcinfo("SYSCTL: %08" PRIx32 " PRSSTAT: %08" PRIx32
         " IRQSTATEN: %08" PRIx32 "\n",
         getreg32(CXD56_SDHCI_SYSCTL), getreg32(CXD56_SDHCI_PRSSTAT),
         getreg32(CXD56_SDHCI_IRQSTATEN));

  /* Initialize the SDHC slot structure data structure */

  /* The next phase of the hardware reset would be to set the SYSCTRL INITA
   * bit to send 80 clock ticks for card to power up and then reset the card
   * with CMD0.  This is done elsewhere.
   */

  /* Reset state data */

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitints   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */
#ifdef CONFIG_SDIO_DMA
  priv->xfrflags   = 0;      /* Used to synchronize SDIO and DMA completion events */
#endif

  wd_cancel(&priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->buffer     = 0;      /* Address of current R/W buffer */
  priv->remaining  = 0;      /* Number of bytes remaining in the transfer */
  priv->xfrints    = 0;      /* Interrupt enables for data transfer */

  priv->blocksize = CXD56_SDHCI_BUF_SIZE;
}

/****************************************************************************
 * Name: cxd56_sdio_capabilities
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

static sdio_capset_t cxd56_sdio_capabilities(struct sdio_dev_s *dev)
{
  sdio_capset_t caps = 0;

#ifdef CONFIG_CXD56_SDIO_WIDTH_D1_ONLY
  caps |= SDIO_CAPS_1BIT_ONLY;
#endif
#ifdef CONFIG_CXD56_SDIO_DMA
  caps |= SDIO_CAPS_DMASUPPORTED;
#endif
#ifndef CONFIG_CXD56_SDIO_DMA
  /* In case of non-DMA, add below capability to change the write single
   * sequence with sending CMD24. If not, write is not completed.
   */

  caps |= SDIO_CAPS_DMABEFOREWRITE;
#endif

  return caps;
}

/****************************************************************************
 * Name: cxd56_sdio_status
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see cxd56_status_* defines)
 *
 ****************************************************************************/

static sdio_statset_t cxd56_sdio_status(struct sdio_dev_s *dev)
{
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;
  return priv->cdstatus;
}

/****************************************************************************
 * Name: cxd56_sdio_widebus
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

static void cxd56_sdio_widebus(struct sdio_dev_s *dev, bool wide)
{
  uint32_t regval;

  /* Set the Data Transfer Width (DTW) field in the PROCTL register */

  regval = getreg32(CXD56_SDHCI_PROCTL);
  regval &= ~SDHCI_PROCTL_DTW_MASK;
  if (wide)
    {
      regval |= SDHCI_PROCTL_DTW_4BIT;
    }
  else
    {
      regval |= SDHCI_PROCTL_DTW_1BIT;
    }

  putreg32(regval, CXD56_SDHCI_PROCTL);
}

/****************************************************************************
 * Name: cxd56_sdio_frequency
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

static void cxd56_sdio_frequency(uint32_t frequency)
{
  uint32_t baseclk;
  uint16_t i;
  uint32_t regval;
  uint16_t divisor;

  baseclk = cxd56_get_sdio_baseclock();
  if (frequency >= baseclk)
    {
      divisor = 0;
    }
  else
    {
      for (i = 1; i < 0x3ff; i++)
        {
          if (baseclk / (2 * i) < frequency)
            {
              break;
            }
        }

      divisor = i;
    }

  regval  = getreg32(CXD56_SDHCI_SYSCTL);
  regval &= ~SDHCI_SYSCTL_GENSEL;
  regval &= ~(SDHCI_SYSCTL_SDCLKFS_MASK | SDHCI_SYSCTL_SDCLKFSUP_MASK);
  regval |= (divisor << SDHCI_SYSCTL_SDCLKFS_SHIFT) &
             SDHCI_SYSCTL_SDCLKFS_MASK;
  regval |= ((divisor >> 8) << SDHCI_SYSCTL_SDCLKFSUP_SHIFT) &
              SDHCI_SYSCTL_SDCLKFSUP_MASK;
  putreg32(regval, CXD56_SDHCI_SYSCTL);
}

static void cxd56_sdio_clock(struct sdio_dev_s *dev,
                             enum sdio_clock_e rate)
{
  uint32_t regval;
  uint32_t frequency = 0;
  uint32_t i;

  /* The SDCLK must be disabled before its frequency can be changed: "SDCLK
   * frequency can be changed when this bit is 0. Then, the host controller
   * shall maintain the same clock frequency until SDCLK is stopped (stop at
   * SDCLK = 0).
   */

  regval  = getreg32(CXD56_SDHCI_SYSCTL);
  regval &= ~SDHCI_SYSCTL_SDCLKEN;
  putreg32(regval, CXD56_SDHCI_SYSCTL);
  mcinfo("SYSCTRL: %08" PRIx32 "\n", getreg32(CXD56_SDHCI_SYSCTL));

  /* sel_ttclk bit[16] */

  if (cxd56_get_sdio_baseclock() < 48 * 1000 * 1000)
    {
      putreg32(getreg32(CXD56_SDHCI_USERDEF2CTL) | (0x1 << 16),
               CXD56_SDHCI_USERDEF2CTL);
    }
  else
    {
      putreg32(getreg32(CXD56_SDHCI_USERDEF2CTL) & ~(0x1 << 16),
               CXD56_SDHCI_USERDEF2CTL);
    }

  /* HS_SYNC_RISE bit[16] */

  putreg32(0x01010100, CXD56_SDHCI_OTHERIOLL);

  /* sdclk_dly_sel */

  if (rate <= CLOCK_SD_TRANSFER_4BIT)
    putreg32((getreg32(CXD56_SDHCI_USERDEF2CTL) & ~(0x7)) | 0x1,
              CXD56_SDHCI_USERDEF2CTL);
  else
    putreg32((getreg32(CXD56_SDHCI_USERDEF2CTL) & ~(0x7)) | 0x0,
              CXD56_SDHCI_USERDEF2CTL);

  /* Select the new prescaler and divisor values based on the requested mode
   * and the settings from the board.h file.
   *
   * TODO:  Investigate using the automatically gated clocks to reduce power
   *        consumption.
   */

  switch (rate)
    {
    default:
    case CLOCK_SDIO_DISABLED :     /* Clock is disabled */
      {
        /* Clear the prescaler and divisor settings and other clock
         * enables as well.
         */

        regval &= ~(SDHCI_SYSCTL_SDCLKFS_MASK | SDHCI_SYSCTL_SDCLKFSUP_MASK);
        putreg32(regval, CXD56_SDHCI_SYSCTL);
        cxd56_sdio_frequency(CONFIG_CXD56_IDMODE_FREQ);
        mcinfo("SYSCTRL: %08" PRIx32 "\n", getreg32(CXD56_SDHCI_SYSCTL));
        return;
      }

    case CLOCK_IDMODE :            /* Initial ID mode clocking (<400KHz) */
      frequency = CONFIG_CXD56_IDMODE_FREQ;
      break;

    case CLOCK_MMC_TRANSFER :      /* MMC normal operation clocking */
      frequency = CONFIG_CXD56_MMCXFR_FREQ;
      break;

    case CLOCK_SD_TRANSFER_1BIT :  /* SD normal operation clocking (narrow 1-bit mode) */
#ifndef CONFIG_CXD56_SDIO_WIDTH_D1_ONLY
      frequency = CONFIG_CXD56_SD1BIT_FREQ;
      break;
#endif

    case CLOCK_SD_TRANSFER_4BIT :  /* SD normal operation clocking (wide 4-bit mode) */
      frequency = CONFIG_CXD56_SD4BIT_FREQ;
      break;
    }

  cxd56_sdio_frequency(frequency);

  putreg32(getreg32(CXD56_SDHCI_SYSCTL) | SDHCI_SYSCTL_ICLKEN,
           CXD56_SDHCI_SYSCTL);
  for (i = 0; i < 20; i++)
    {
      up_mdelay(50);
      regval  = getreg32(CXD56_SDHCI_SYSCTL);
      if (regval & SDHCI_SYSCTL_ICLKSTA)
        {
          break;
        }
    }

  do
    {
      putreg32(regval | SDHCI_SYSCTL_SDCLKEN, CXD56_SDHCI_SYSCTL);
    }
  while ((getreg32(CXD56_SDHCI_SYSCTL) & SDHCI_SYSCTL_SDCLKEN) == 0);
  mcinfo("SYSCTRL: %08" PRIx32 "\n", getreg32(CXD56_SDHCI_SYSCTL));
}

/****************************************************************************
 * Name: cxd56_sdio_attach
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

static int cxd56_sdio_attach(struct sdio_dev_s *dev)
{
  int ret;

  /* Attach the SDIO interrupt handler */

  ret = irq_attach(CXD56_IRQ_SDIO, cxd56_interrupt, NULL);
  if (ret == OK)
    {
      /* Disable all interrupts at the SDIO controller and clear all pending
       * interrupts.
       */

      putreg32(0,            CXD56_SDHCI_IRQSIGEN);
      putreg32(SDHCI_INT_ALL, CXD56_SDHCI_IRQSTAT);

#ifdef CONFIG_ARCH_IRQPRIO
      /* Set the interrupt priority */

      up_prioritize_irq(CXD56_IRQ_SDIO, CONFIG_CXD56_SDHCI_PRIO);
#endif

      /* Enable SDIO interrupts at the NVIC.  They can now be enabled at
       * the SDIO controller as needed.
       */

      up_enable_irq(CXD56_IRQ_SDIO);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd56_sdio_sendcmd
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

static int cxd56_sdio_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                              uint32_t arg)
{
#ifdef CONFIG_SDIO_DMA
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;
#endif
  uint32_t regval;
  uint32_t cmdidx;
  int32_t  timeout;

  /* Initialize the command index */

  cmdidx = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval = cmdidx << SDHCI_XFERTYP_CMDINX_SHIFT;

  /* Does a data transfer accompany the command? */

  if ((cmd & MMCSD_DATAXFR) != 0)
    {
      /* Yes.. Configure the data transfer */

      switch (cmd & MMCSD_DATAXFR_MASK)
        {
        default:
        case MMCSD_NODATAXFR : /* No.. no data transfer */
          break;

          /* The following two cases are probably missing some setup logic */

        case MMCSD_RDSTREAM :  /* Yes.. streaming read data transfer */
          regval |= (SDHCI_XFERTYP_DPSEL | SDHCI_XFERTYP_DTDSEL);
          break;

        case MMCSD_WRSTREAM :  /* Yes.. streaming write data transfer */
          regval |= SDHCI_XFERTYP_DPSEL;
          break;

        case MMCSD_RDDATAXFR : /* Yes.. normal read data transfer */
          regval |= (SDHCI_XFERTYP_DPSEL | SDHCI_XFERTYP_DTDSEL);
          break;

        case MMCSD_WRDATAXFR : /* Yes.. normal write data transfer */
          regval |= SDHCI_XFERTYP_DPSEL;
          break;
        }

      /* Is it a multi-block transfer? */

      if ((cmd & MMCSD_MULTIBLOCK) != 0)
        {
          /* Yes.. should the transfer be stopped with ACMD12? */

          if ((cmd & MMCSD_STOPXFR) != 0)
            {
              /* Yes.. Indefinite block transfer */

              regval |= (SDHCI_XFERTYP_MSBSEL | SDHCI_XFERTYP_AC12EN);
            }
          else
            {
              /* No.. Fixed block transfer */

              regval |= (SDHCI_XFERTYP_MSBSEL | SDHCI_XFERTYP_BCEN);
            }
        }
    }

  /* Configure response type bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:                /* No response */
      regval |= SDHCI_XFERTYP_RSPTYP_NONE;
      break;

    case MMCSD_R1B_RESPONSE:              /* Response length 48, check busy & cmdindex */
      regval |= (SDHCI_XFERTYP_RSPTYP_LEN48BSY | SDHCI_XFERTYP_CICEN |
                 SDHCI_XFERTYP_CCCEN);
      break;

    case MMCSD_R1_RESPONSE:              /* Response length 48, check cmdindex */
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
      regval |= (SDHCI_XFERTYP_RSPTYP_LEN48 | SDHCI_XFERTYP_CICEN |
                 SDHCI_XFERTYP_CCCEN);
      break;

    case MMCSD_R2_RESPONSE:              /* Response length 136, check CRC */
      regval |= (SDHCI_XFERTYP_RSPTYP_LEN136 | SDHCI_XFERTYP_CCCEN);
      break;

    case MMCSD_R3_RESPONSE:              /* Response length 48 */
    case MMCSD_R4_RESPONSE:
    case MMCSD_R7_RESPONSE:
      regval |= SDHCI_XFERTYP_RSPTYP_LEN48;
      break;
    }

  /* Enable DMA */

#ifdef CONFIG_SDIO_DMA

  /* Internal DMA is used */

  priv->dmasend_prepare = false;
  priv->dmasend_cmd = 0;
  priv->dmasend_regcmd = 0;
  if (((cmd & MMCSD_DATAXFR_MASK) != MMCSD_NODATAXFR) && priv->usedma)
    {
      regval |= SDHCI_XFERTYP_DMAEN;
    }
  else if (cmdidx == MMCSD_CMDIDX24 || cmdidx == MMCSD_CMDIDX25)
    {
      regval |= SDHCI_XFERTYP_DMAEN;
      priv->usedma = true;
      priv->dmasend_prepare = true;
    }
#endif

  /* Other bits? What about CMDTYP? */

  mcinfo("cmd: %08" PRIx32 " arg: %08" PRIx32 " regval: %08" PRIx32 "\n",
         cmd, arg, regval);

  /* The Command Inhibit (CIHB) bit is set in the PRSSTAT bit immediately
   * after the transfer type register is written.  This bit is cleared when
   * the command response is received.  If this status bit is 0, it
   * indicates that the CMD line is not in use and the SDHC can issue a
   * SD/MMC Command using the CMD line.
   *
   * CIHB should always be set when this function is called.
   */

  timeout = SDHCI_CMDTIMEOUT;
  while ((getreg32(CXD56_SDHCI_PRSSTAT) & SDHCI_PRSSTAT_CIHB) != 0)
    {
      if (--timeout <= 0)
        {
          mcerr("ERROR: Timeout cmd: %08" PRIx32 " PRSSTAT: %08" PRIx32 "\n",
                cmd, getreg32(CXD56_SDHCI_PRSSTAT));

          return -EBUSY;
        }
    }

  if ((cmd & MMCSD_DATAXFR_MASK) != MMCSD_NODATAXFR)
    {
      timeout = SDHCI_CMDTIMEOUT;
      while ((getreg32(CXD56_SDHCI_PRSSTAT) & SDHCI_PRSSTAT_CDIHB) != 0)
        {
          if (--timeout <= 0)
            {
              mcerr("ERROR: Timeout cmd data: %08" PRIx32
                    " PRSSTAT: %08" PRIx32 "\n",
                    cmd, getreg32(CXD56_SDHCI_PRSSTAT));

              return -EBUSY;
            }
        }
    }

  /* Set the SDHC Argument value */

  putreg32(arg, CXD56_SDHCI_CMDARG);

  /* Clear interrupt status and write the SDHC CMD */

  putreg32(SDHCI_RESPDONE_INTS, CXD56_SDHCI_IRQSTAT);
#ifdef CONFIG_SDIO_DMA
  priv->dma_cmd = cmd;
  if (priv->dmasend_prepare)
    {
      priv->dmasend_regcmd = regval;
      priv->dmasend_cmd = cmd;
    }
  else
#endif
    {
      putreg32(regval, CXD56_SDHCI_XFERTYP);
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_blocksetup
 *
 * Description:
 *   Some hardware needs to be informed of the selected blocksize.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   blocklen - The selected block size.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void cxd56_blocksetup(struct sdio_dev_s *dev,
                             unsigned int blocklen, unsigned int nblocks)
{
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;
  uint32_t regval;

  DEBUGASSERT(dev != NULL && nblocks > 0 && nblocks < 65535);
  DEBUGASSERT(blocklen < 65535);

  priv->blocksize = blocklen;

  /* Set the block size and count */

  regval = (blocklen << SDHCI_BLKATTR_SIZE_SHIFT) |
           (nblocks  << SDHCI_BLKATTR_CNT_SHIFT);
  putreg32(regval, CXD56_SDHCI_BLKATTR);
}

/****************************************************************************
 * Name: cxd56_sdio_recvsetup
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

#ifndef CONFIG_SDIO_DMA
static int cxd56_sdio_recvsetup(struct sdio_dev_s *dev,
                                uint8_t *buffer, size_t nbytes)
{
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

#ifdef CONFIG_SDIO_DMA
  priv->usedma = false;
#endif

  /* Reset the DPSM configuration */

  cxd56_datadisable();
  cxd56_sampleinit();
  cxd56_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;

  /* Then set up the SDIO data path */

  cxd56_dataconfig(priv, false, nbytes, 1, SDHCI_DTOCV_DATATIMEOUT);

  /* And enable interrupts */

  cxd56_configxfrints(priv, SDHCI_RCVDONE_INTS);
  cxd56_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}
#endif

/****************************************************************************
 * Name: cxd56_sdio_sendsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card.
 *   This method will do whatever controller setup is necessary.
 *   This would be called for SD memory just AFTER sending
 *   CMD24 (WRITE_BLOCK), CMD25 (WRITE_MULTIPLE_BLOCK), ...
 *   and before SDIO_SENDDATA is called.
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

#ifndef CONFIG_SDIO_DMA
static int cxd56_sdio_sendsetup(struct sdio_dev_s *dev,
                                const uint8_t *buffer, size_t nbytes)
{
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

#ifdef CONFIG_SDIO_DMA
  priv->usedma = false;
#endif

  /* Reset the DPSM configuration */

  cxd56_datadisable();
  cxd56_sampleinit();
  cxd56_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;

  /* Then set up the SDIO data path */

  cxd56_dataconfig(priv, true, nbytes, 1, SDHCI_DTOCV_DATATIMEOUT);

  /* Enable TX interrupts */

  cxd56_configxfrints(priv, SDHCI_SNDDONE_INTS);
  cxd56_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}
#endif

/****************************************************************************
 * Name: cxd56_sdio_cancel
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

static int cxd56_sdio_cancel(struct sdio_dev_s *dev)
{
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;
  uint32_t regval;

#ifdef CONFIG_SDIO_DMA
  /* Release allocated buffer */

  if (priv->aligned_buffer)
    {
      /* Free aligned buffer */

      kmm_free(priv->aligned_buffer);

      priv->aligned_buffer = NULL;
    }
#endif

  /* Disable all transfer- and event- related interrupts */

  cxd56_configxfrints(priv, 0);
  cxd56_configwaitints(priv, 0, 0, 0);

  /* Clearing pending interrupt status on all transfer- and event- related
   * interrupts
   */

  putreg32(SDHCI_WAITALL_INTS, CXD56_SDHCI_IRQSTAT);

  /* Cancel any watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_SDIO_DMA
  /* Stop the DMA by resetting the data path */

  regval = getreg32(CXD56_SDHCI_SYSCTL);
  regval |= SDHCI_SYSCTL_RSTD;
  putreg32(regval, CXD56_SDHCI_SYSCTL);
  priv->usedma = false;
  priv->dmasend_prepare = false;
  priv->dmasend_cmd = 0;
  priv->dmasend_regcmd = 0;
  cxd56_sdhci_adma_dscr[0] = 0;
  cxd56_sdhci_adma_dscr[1] = 0;
  putreg32(CXD56_PHYSADDR(cxd56_sdhci_adma_dscr), CXD56_SDHCI_ADSADDR);
  putreg32(0, CXD56_SDHCI_ADSADDR_H);
#endif
  regval  = getreg32(CXD56_SDHCI_SYSCTL);
  regval |= SDHCI_SYSCTL_RSTC;
  putreg32(regval, CXD56_SDHCI_SYSCTL);
  regval = getreg32(CXD56_SDHCI_SYSCTL);
  regval |= SDHCI_SYSCTL_RSTD;
  putreg32(regval, CXD56_SDHCI_SYSCTL);

  while ((getreg32(CXD56_SDHCI_SYSCTL) & SDHCI_SYSCTL_RSTC) != 0);
  while ((getreg32(CXD56_SDHCI_SYSCTL) & SDHCI_SYSCTL_RSTD) != 0);

  /* Mark no transfer in progress */

  priv->remaining = 0;

  return OK;
}

/****************************************************************************
 * Name: cxd56_sdio_waitresponse
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

static int cxd56_sdio_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  uint32_t errors;
  int32_t  timeout = SDHCI_CMDTIMEOUT;
  int      ret = OK;

#ifdef CONFIG_SDIO_DMA
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;

  if (priv->dmasend_prepare)
    {
      return OK;
    }
#endif

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      timeout = SDHCI_CMDTIMEOUT;
      errors  = 0;
      return OK;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R6_RESPONSE:
      timeout = SDHCI_LONGTIMEOUT;
      errors  = SDHCI_RESPERR_INTS;
      break;

    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      timeout = SDHCI_CMDTIMEOUT;
      errors  = SDHCI_RESPERR_INTS;
      break;

    default:
      return -EINVAL;
    }

  /* Then wait for the Command Complete (CC) indication (or timeout).  The
   * CC bit is set when the end bit of the command response is received
   * (except Auto CMD12).
   */

  while ((getreg32(CXD56_SDHCI_IRQSTAT) & SDHCI_INT_CC) == 0)
    {
      timeout -= 1;
      if (timeout <= 0)
        {
          mcerr("ERROR: Timeout cmd: %08" PRIx32 " IRQSTAT: %08" PRIx32 "\n",
               cmd, getreg32(CXD56_SDHCI_IRQSTAT));
          putreg32(0, CXD56_SDHCI_IRQSIGEN);

          return -ETIMEDOUT;
        }
    }

  /* Check for hardware detected errors */

  if ((getreg32(CXD56_SDHCI_IRQSTAT) & errors) != 0)
    {
      mcerr("ERROR: cmd: %08" PRIx32 " errors: %08" PRIx32
            " IRQSTAT: %08" PRIx32 "\n",
            cmd, errors, getreg32(CXD56_SDHCI_IRQSTAT));
      ret = -EIO;
    }

  /* Clear the response wait status bits */

  if ((cmd & MMCSD_DATAXFR_MASK) == MMCSD_NODATAXFR)
    {
      putreg32((SDHCI_INT_TC & getreg32(CXD56_SDHCI_IRQSTAT)) |
                SDHCI_RESPDONE_INTS, CXD56_SDHCI_IRQSTAT);
    }
  else
    {
      putreg32(SDHCI_RESPDONE_INTS, CXD56_SDHCI_IRQSTAT);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd56_sdio_recv*
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
 *   failure means only a faiure to obtain the requested response (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intacta and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int cxd56_sdio_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                                uint32_t *rshort)
{
  uint32_t regval;
  int ret = OK;
#ifdef CONFIG_SDIO_DMA
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;
#endif

  /* R1  Command response (48-bit)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Command index (0-63)
   *     39:8      bit31  - bit0   32-bit card status
   *     7:1       bit6   - bit0   CRC7
   *     0         1               End bit
   *
   * R1b Identical to R1 with the additional busy signalling via the data
   *     line.
   *
   * R6  Published RCA Response (48-bit, SD card only)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Command index (0-63)
   *     39:8      bit31  - bit0   32-bit Argument Field, consisting of:
   *                               [31:16] New published RCA of card
   *                               [15:0]  Card status bits {23,22,19,12:0}
   *     7:1       bit6   - bit0   CRC7
   *     0         1               End bit
   */

  if (rshort)
    {
      *rshort = 0;
    }

#ifdef CONFIG_SDIO_DMA
  if (priv->dmasend_prepare)
    {
      return OK;
    }

#endif
#ifdef CONFIG_DEBUG_FEATURES
  if (!rshort)
    {
      mcerr("ERROR: rshort=NULL\n");
      ret = -EINVAL;
    }

  /* Check that this is the correct response to this command */

  else if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1B_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R6_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08" PRIx32 "\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = getreg32(CXD56_SDHCI_IRQSTAT);
      if ((regval & SDHCI_INT_CTOE) != 0)
        {
          mcerr("ERROR: Command timeout: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & SDHCI_INT_CCE) != 0)
        {
          mcerr("ERROR: CRC failure: %08" PRIx32 "\n", regval);
          ret = -EIO;
        }
    }

  /* Return the R1/R1b/R6 response.  These responses are returned in
   * CDMRSP0.  NOTE: This is not true for R1b (Auto CMD12 response) which
   * is returned in CMDRSP3.
   */

  *rshort = getreg32(CXD56_SDHCI_CMDRSP0);

  return ret;
}

static int cxd56_sdio_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                               uint32_t rlong[4])
{
  uint32_t regval;
  int ret = OK;

  /* R2  CID, CSD register (136-bit)
   *     135       0               Start bit
   *     134       0               Transmission bit (0=from card)
   *     133:128   bit5   - bit0   Reserved
   *     127:1     bit127 - bit1   127-bit CID or CSD register
   *                               (including internal CRC)
   *     0         1               End bit
   */

  if (rlong)
    {
      rlong[0] = 0;
      rlong[1] = 0;
      rlong[2] = 0;
      rlong[3] = 0;
    }

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

      regval = getreg32(CXD56_SDHCI_IRQSTAT);
      if (regval & SDHCI_INT_CTOE)
        {
          mcerr("ERROR: Timeout IRQSTAT: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & SDHCI_INT_CCE)
        {
          mcerr("ERROR: CRC fail IRQSTAT: %08" PRIx32 "\n", regval);
          ret = -EIO;
        }
    }

  /* Return the long response in CMDRSP3..0 */

  if (rlong)
    {
      rlong[0] = getreg32(CXD56_SDHCI_CMDRSP3);
      rlong[1] = getreg32(CXD56_SDHCI_CMDRSP2);
      rlong[2] = getreg32(CXD56_SDHCI_CMDRSP1);
      rlong[3] = getreg32(CXD56_SDHCI_CMDRSP0);
    }

  if (1)
    {
      rlong[0] = ((rlong[0] << 8) & 0xffffff00) |
                 ((rlong[1] >> 24) & 0x000000ff);
      rlong[1] = ((rlong[1] << 8) & 0xffffff00) |
                 ((rlong[2] >> 24) & 0x000000ff);
      rlong[2] = ((rlong[2] << 8) & 0xffffff00) |
                 ((rlong[3] >> 24) & 0x000000ff);
      rlong[3] = (rlong[3] << 8)  & 0xffffff00;
    }

  return ret;
}

static int cxd56_sdio_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                                uint32_t *rshort)
{
  uint32_t regval;
  int ret = OK;

  /* R3  OCR (48-bit)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Reserved
   *     39:8      bit31  - bit0   32-bit OCR register
   *     7:1       bit6   - bit0   Reserved
   *     0         1               End bit
   */

  /* R4  Response (48-bit)
   *     47        0               Start bit
   *     46        0               Direction bit(0=card to host)
   *     45:40     bit5   - bit0   Reserved
   *     39        1               Set to 1 if Card is
   *                               ready to operate after initialization
   *     38:36     bit2   - bit0   Number of I/O functions
   *     35        1               Memory Present
   *     34:32     bit2   - bit0   Stuff Bits
   *     31:8      bit23  - bit0   I/O OCR
   *     7:1       bit6   - bit0   Reserved
   *     0         1               End bit
   */

  /* R5  Response (48-bit)
   *     47        0               Start bit
   *     46        0               Direction bit(0=card to host)
   *     45:40     bit5   - bit0   Command Index
   *     39:24     bit15  - bit0   16-bit Stuff Bits
   *     23:16     bit7   - bit0   Response Flags
   *     15:8      bit7   - bit0   Read or Write Data
   *     7:1       bit6   - bit0   CRC
   *     0         1               End bit
   */

  if (!rshort)
    {
      *rshort = 0;
    }

  /* Check that this is the correct response to this command */

#ifdef CONFIG_DEBUG_FEATURES
  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R4_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R5_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08" PRIx32 "\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout occurred (Apparently a CRC error can terminate
       * a good response)
       */

      regval = getreg32(CXD56_SDHCI_IRQSTAT);
      if (regval & SDHCI_INT_CTOE)
        {
          mcerr("ERROR: Timeout IRQSTAT: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
    }

  /* Return the short response in CMDRSP0 */

  if (rshort)
    {
      *rshort = getreg32(CXD56_SDHCI_CMDRSP0);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd56_sdio_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the SDIO_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling cxd56_eventwait.  This is done in this way
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

static void cxd56_sdio_waitenable(struct sdio_dev_s *dev,
                             sdio_eventset_t eventset, uint32_t timeout)
{
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;
  uint32_t waitints;

  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  cxd56_configwaitints(priv, 0, 0, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

  waitints = 0;
  if ((eventset & (SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE)) != 0)
    {
      waitints |= SDHCI_RESPDONE_INTS;
    }

  if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
    {
      waitints |= SDHCI_XFRDONE_INTS;
    }

  /* Enable event-related interrupts */

  cxd56_configwaitints(priv, waitints, eventset, 0);

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
      ret   = wd_start(&priv->waitwdog, delay,
                       cxd56_eventtimeout, (wdparm_t)priv);
      if (ret != OK)
        {
          mcerr("ERROR: wd_start failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: cxd56_sdio_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDIO_WAITEVENTS are disabled when cxd56_eventwait
 *   returns.  SDIO_WAITEVENTS must be called again before cxd56_eventwait
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

static sdio_eventset_t cxd56_sdio_eventwait(struct sdio_dev_s *dev)
{
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;
  sdio_eventset_t wkupevent = 0;
  int ret;

  /* There is a race condition here... the event may have completed before
   * we get here.  In this case waitevents will be zero, but wkupevents will
   * be non-zero (and, hopefully, the semaphore count will also be non-zero.
   */

  DEBUGASSERT((priv->waitevents != 0 && priv->wkupevent == 0) ||
              (priv->waitevents == 0 && priv->wkupevent != 0));

  /* Loop until the event (or the timeout occurs). Race conditions are
   * avoided by calling cxd56_waitenable prior to triggering the logic that
   * will cause the wait to terminate.  Under certain race conditions, the
   * waited-for may have already occurred before this function was called!
   */

  for (; ; )
    {
      /* Wait for an event in event set to occur.
       * If this the event has already occurred,
       * then the semaphore will already have been incremented and
       * there will be no wait.
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

      /* Check if the event has occurred
       * When the event has occurred, then evenset will be set to 0 and
       * wkupevent will be set to a non-zero value.
       */

      if (wkupevent != 0)
        {
          /* Yes... break out of the loop with wkupevent non-zero */

          if (wkupevent & (SDIOWAIT_RESPONSEDONE | SDIOWAIT_TRANSFERDONE))
            {
              if (priv->remaining > 0)
                {
                  priv->remaining = 0;
                }
            }

          if (wkupevent & (SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR))
            {
              cxd56_sdio_cancel(&(priv->dev));
            }
          break;
        }
    }

  /* Disable event-related interrupts */

  cxd56_configwaitints(priv, 0, 0, 0);
#ifdef CONFIG_SDIO_DMA
  priv->xfrflags = 0;
  if (priv->aligned_buffer)
    {
      if (priv->dma_cmd == MMCSD_CMD17 || priv->dma_cmd == MMCSD_CMD18)
        {
          /* Copy receive buffer from aligned address */

          memcpy(priv->receive_buffer, priv->aligned_buffer,
                 priv->receive_size);
        }

      /* Free aligned buffer */

      kmm_free(priv->aligned_buffer);
      priv->aligned_buffer = NULL;
    }
#endif

  cxd56_dumpsamples(priv);
  return wkupevent;
}

/****************************************************************************
 * Name: cxd56_sdio_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events.  This is part of the
 *   the SDIO callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in cxd56_registercallback.
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

static void cxd56_sdio_callbackenable(struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset)
{
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;

  mcinfo("eventset: %02x\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  cxd56_sdio_callback(priv);
}

/****************************************************************************
 * Name: cxd56_sdio_registercallback
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
 *   dev -      Device-specific state data
 *   callback - The function to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

static int cxd56_sdio_registercallback(struct sdio_dev_s *dev,
                                  worker_t callback, void *arg)
{
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;

  /* Disable callbacks and register this callback and is argument */

  mcinfo("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: cxd56_sdio_admasetup
 *
 * Description:
 *   Setup to perform ADMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.
 *
 * Input Parameters:
 *   buffer - The memory to/from DMA
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static int cxd56_sdio_admasetup(const uint8_t *buffer, size_t buflen)
{
  uint32_t dscr_top = CXD56_PHYSADDR(cxd56_sdhci_adma_dscr);
  uint32_t dscr_l;
  uint32_t i;
  uint32_t remaining;
  uint32_t len;
  uint32_t data_addr = CXD56_PHYSADDR(buffer);
  remaining = buflen;

  putreg32(0x0, CXD56_SDHCI_ADSADDR_H);
  putreg32(dscr_top, CXD56_SDHCI_ADSADDR);
  for (i = 0; i < CXD56_SDIO_MAX_LEN_ADMA_DSCR; i++)
    {
      cxd56_sdhci_adma_dscr[i * 2 + 1] = data_addr;
      dscr_l = SDHCI_ADMA_DSCR_L_ACT_TRAN;
      dscr_l |= SDHCI_ADMA_DSCR_L_VALID;
      if (remaining < SDHCI_MAX_ADMA_TRANS_SIZE)
        {
          len = remaining;
          dscr_l |= (len << SDHCI_ADMA_DSCR_L_LEN_SHIFT) &
                     SDHCI_ADMA_DSCR_L_LEN_MASK;
        }
      else
        {
          len = SDHCI_MAX_ADMA_TRANS_SIZE;
          data_addr += len;
        }

      cxd56_sdhci_adma_dscr[i * 2] = dscr_l;
      remaining -= len;
      if (remaining < 1)
        {
          cxd56_sdhci_adma_dscr[i * 2] |= SDHCI_ADMA_DSCR_L_END;
          break;
        }
    }

  if (remaining > 0)
    {
      return -EIO;
    }
  else
    {
      return OK;
    }
}
#endif

/****************************************************************************
 * Name: cxd56_sdio_dmarecvsetup
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

#ifdef CONFIG_SDIO_DMA
static int cxd56_sdio_dmarecvsetup(struct sdio_dev_s *dev,
                                   uint8_t *buffer, size_t buflen)
{
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;
  unsigned int blocksize;
  int ret = OK;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  if ((uint32_t)buffer & 3)
    {
      if (priv->aligned_buffer)
        {
          /* If buffer not freed,  free it */

          kmm_free(priv->aligned_buffer);

          priv->aligned_buffer = NULL;
        }

      /* Allocate aligned buffer */

      priv->aligned_buffer = (uint8_t *)
        kmm_malloc(sizeof(uint8_t) * buflen);

      /* Keep receive buffer address */

      priv->receive_buffer = buffer;

      /* Keep receive data size */

      priv->receive_size = buflen;

      /* Switch to aligned buffer */

      buffer = priv->aligned_buffer;
    }

  /* Reset the DPSM configuration */

  cxd56_datadisable();

  /* Begin sampling register values */

  cxd56_sampleinit();
  cxd56_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;

  /* Then set up the SDIO data path */

  blocksize = getreg32(CXD56_SDHCI_BLKATTR) & SDHCI_BLKATTR_SIZE_MASK;
  if (blocksize == 0)
    {
      if (priv->blocksize != 0)
        {
          blocksize = priv->blocksize;
        }
      else
        {
          ret = -EIO;
          goto error;
        }
    }

  cxd56_dataconfig(priv, false,  blocksize, buflen / blocksize,
                   SDHCI_DTOCV_DATATIMEOUT);

  /* Configure the RX DMA */

  cxd56_sdio_admasetup(buffer,  buflen);
  priv->usedma = true;

  cxd56_configxfrints(priv, SDHCI_DMADONE_INTS);
  putreg32(CXD56_PHYSADDR(buffer), CXD56_SDHCI_DSADDR);

  /* Sample the register state */

  cxd56_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;

error:

  /* Free allocated align buffer */

  kmm_free(priv->aligned_buffer);

  priv->aligned_buffer = NULL;
  return ret;
}
#endif

/****************************************************************************
 * Name: cxd56_sdio_dmasendsetup
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

#ifdef CONFIG_SDIO_DMA
static int cxd56_sdio_dmasendsetup(struct sdio_dev_s *dev,
                              const uint8_t *buffer, size_t buflen)
{
  uint32_t r1;
  int      ret = OK;
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;
  unsigned int blocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  if ((uint32_t)buffer & 3)
    {
      if (priv->aligned_buffer)
        {
          /* If buffer not freed,  free it */

          kmm_free(priv->aligned_buffer);

          priv->aligned_buffer = NULL;
        }

      /* Allocate aligned buffer */

      priv->aligned_buffer = (uint8_t *)
        kmm_malloc(sizeof(uint8_t) * buflen);

      /* Copy buffer to aligned address */

      memcpy(priv->aligned_buffer, buffer, buflen);

      /* Switch to aligned buffer */

      buffer = priv->aligned_buffer;
    }

  /* Reset the DPSM configuration */

  cxd56_datadisable();

  /* Begin sampling register values */

  cxd56_sampleinit();
  cxd56_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;

  /* Then set up the SDIO data path */

  blocksize = getreg32(CXD56_SDHCI_BLKATTR) & SDHCI_BLKATTR_SIZE_MASK;
  if (blocksize == 0)
    {
      if (priv->blocksize != 0)
        {
          blocksize = priv->blocksize;
        }
      else
        {
          ret = -EIO;
          goto error;
        }
    }

  cxd56_dataconfig(priv, true, blocksize, buflen / blocksize,
                   SDHCI_DTOCV_DATATIMEOUT);

  /* Configure the TX DMA */

  cxd56_sdio_admasetup(buffer, buflen);
  priv->usedma = true;
  if (priv->dmasend_prepare)
    {
      putreg32(priv->dmasend_regcmd, CXD56_SDHCI_XFERTYP);
      priv->dmasend_prepare = false;
      cxd56_sdio_waitresponse(dev, priv->dmasend_cmd);
      ret = cxd56_sdio_recvshortcrc(dev, priv->dmasend_cmd, &r1);
      if (ret != OK)
        {
          goto error;
        }
    }

  /* Sample the register state */

  cxd56_sample(priv, SAMPLENDX_AFTER_SETUP);

  /* Enable TX interrupts */

  cxd56_configxfrints(priv, SDHCI_DMADONE_INTS);

  return OK;

error:

  /* Free allocated align buffer */

  kmm_free(priv->aligned_buffer);

  priv->aligned_buffer = NULL;
  return ret;
}
#endif

/****************************************************************************
 * Initialization/uninitialization/reset
 ****************************************************************************/

static inline void cxd56_sdio_poweron(void *arg)
{
  uint32_t regval;

  /* Power ON for SDCARD */

  regval = getreg32(CXD56_SDHCI_PROCTL);
  regval |= 0xf << 8;
  putreg32(regval, CXD56_SDHCI_PROCTL);

  board_sdcard_pin_enable();
}

static inline void cxd56_sdio_poweroff(void *arg)
{
  uint32_t regval;

  board_sdcard_pin_disable();

  /* Power OFF for SDCARD */

  regval = getreg32(CXD56_SDHCI_PROCTL);
  regval &= ~(0x1 << 8);
  putreg32(regval, CXD56_SDHCI_PROCTL);
}

/****************************************************************************
 * Name: cxd56_sdio_callback
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

static void cxd56_sdio_callback(void *arg)
{
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)arg;
  uint32_t delay = 0;
  irqstate_t flags;

  /* Is a callback registered? */

  DEBUGASSERT(priv != NULL);
  mcinfo("Callback %p(%p) cbevents: %02x cdstatus: %02x\n",
        priv->callback, priv->cbarg, priv->cbevents, priv->cdstatus);

  flags = enter_critical_section();
  if (priv->callback)
    {
      /* Yes.. Check for enabled callback events */

      if ((priv->cdstatus & SDIO_STATUS_PRESENT) != 0)
        {
          /* Media is present.  Is the media inserted event enabled? */

          if ((priv->cbevents & SDIOMEDIA_INSERTED) == 0)
            {
              /* No... return without performing the callback */

              leave_critical_section(flags);
              return;
            }

          /* Power ON for SDCARD */

          cxd56_sdio_poweron(priv);
          putreg32(SDHCI_INT_CINS, CXD56_SDHCI_IRQSTAT);
          delay = SDHCI_WAIT_POWERON;
        }
      else
        {
          /* Media is not present.  Is the media eject event enabled? */

          if ((priv->cbevents & SDIOMEDIA_EJECTED) == 0)
            {
              /* No... return without performing the callback */

              leave_critical_section(flags);
              return;
            }

          /* Power OFF for SDCARD */

          cxd56_sdio_poweroff(arg);
          putreg32(SDHCI_INT_CRM | SDHCI_INT_CINT, CXD56_SDHCI_IRQSTAT);
          delay = SDHCI_WAIT_POWEROFF;
        }

      /* Perform the callback, disabling further callbacks.  Of course, the
       * the callback can (and probably should) re-enable callbacks.
       */

      priv->cbevents = 0;
      leave_critical_section(flags);

      /* Callbacks cannot be performed in the context of an interrupt
       * handler.  If we are in an interrupt handler, then queue the
       * callback to be performed later on the work thread.
       */

      if (up_interrupt_context())/* (1) */
        {
          /* Yes.. queue it */

          work_cancel(HPWORK, &priv->cbwork);

          mcinfo("Queuing callback to %p(%p)\n",
                 priv->callback, priv->cbarg);

          work_queue(HPWORK, &priv->cbwork, priv->callback,
                     priv->cbarg, delay);
        }
      else
        {
          /* No.. then just call the callback here */

          up_mdelay(delay);

          mcinfo("Callback to %p(%p)\n", priv->callback, priv->cbarg);

          priv->callback(priv->cbarg);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_sdhci_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Values:
 *   A reference to an SDIO interface structure. NULL is returned on
 *   failures.
 *
 ****************************************************************************/

struct sdio_dev_s *cxd56_sdhci_initialize(int slotno)
{
  uint32_t regval;
#ifdef CONFIG_SDIO_DMA
  uint32_t i;
#endif

  /* There is only one slot */

  struct cxd56_sdiodev_s *priv = &g_sdhcdev;
  DEBUGASSERT(slotno == 0);

  /* Initialize the pins */

  board_sdcard_pin_initialize();

  /* Enable clocking to the SDHC module.
   * Clocking is still disabled in the SYSCTRL register.
   */

  cxd56_sdio_clock_enable();

  putreg32(getreg32(CXD56_SDHCI_SYSCTL) | SDHCI_SYSCTL_ICLKEN,
           CXD56_SDHCI_SYSCTL);

  /* Command Line Pre Drive Enable */

  regval = getreg32(CXD56_SDHCI_VENDSPEC);
  putreg32(regval | 0x00000040, CXD56_SDHCI_VENDSPEC);

  /* Configure the pins */

  board_sdcard_pin_configuraton();

  /* Software reset */

  regval = getreg32(CXD56_SDHCI_SYSCTL);
  putreg32(regval | SDHCI_SYSCTL_RSTA, CXD56_SDHCI_SYSCTL);
  while ((getreg32(CXD56_SDHCI_SYSCTL) & SDHCI_SYSCTL_RSTA) != 0);

  putreg32(0xffffffff, CXD56_SDHCI_IRQSTATEN);

  cxd56_sdio_sdhci_reset(&(priv->dev));

#ifdef CONFIG_SDIO_DMA
  for (i = 0;
       i < sizeof(cxd56_sdhci_adma_dscr) / sizeof(cxd56_sdhci_adma_dscr[0]);
       i++)
    {
      cxd56_sdhci_adma_dscr[i] = 0;
    }

  putreg32(CXD56_PHYSADDR(cxd56_sdhci_adma_dscr), CXD56_SDHCI_ADSADDR);
  putreg32(0, CXD56_SDHCI_ADSADDR_H);
  putreg32(SDHCI_PROCTL_DMAS_ADMA2 |
          (getreg32(CXD56_SDHCI_PROCTL) & ~SDHCI_PROCTL_DMAS_MASK),
           CXD56_SDHCI_PROCTL);
  priv->usedma = false;
  priv->dmasend_prepare = false;
  priv->dmasend_cmd = 0;
  priv->dmasend_regcmd = 0;
#endif

  /* In addition to the system clock, the SDHC module needs a clock for the
   * base for the external card clock.  There are four possible sources for
   * this clock, selected by the SIM's SOPT2 register:
   *
   * - Core/system clock
   * - MCGPLLCLK/MCGFLLCLK clock
   * - OSCERCLK EXTAL clock
   * - External bypass clock from off-chip (SCHC0_CLKINB)
   */

  return &g_sdhcdev.dev;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_sdhci_finalize
 *
 * Description:
 *   Finalize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Values:
 *   A reference to an SDIO interface structure. NULL is returned on
 *   failures.
 *
 ****************************************************************************/

struct sdio_dev_s *cxd56_sdhci_finalize(int slotno)
{
  uint32_t regval;

  /* There is only one slot */

  struct cxd56_sdiodev_s *priv = &g_sdhcdev;
  DEBUGASSERT(slotno == 0);

  /* Enable clocking to the SDHC module.  Clocking is still disabled in
   * the SYSCTRL register.
   */

  /* SD clock disable */

  cxd56_sdio_clock(&priv->dev, CLOCK_SDIO_DISABLED);

  /* Power OFF for SDIO */

  regval = getreg32(CXD56_SDHCI_PROCTL);
  regval &= ~(0xf << 8);
  putreg32(regval, CXD56_SDHCI_PROCTL);

  /* Disable Internal Clock */

  putreg32(getreg32(CXD56_SDHCI_SYSCTL) & ~SDHCI_SYSCTL_ICLKEN,
           CXD56_SDHCI_SYSCTL);

  /* Command Line Pre Drive Disable */

  regval = getreg32(CXD56_SDHCI_VENDSPEC);
  putreg32(regval & ~0x00000040, CXD56_SDHCI_VENDSPEC);

  /* SDIO Clock Disable */

  cxd56_sdio_clock_disable();

  return &g_sdhcdev.dev;
}

/****************************************************************************
 * Name: cxd56_sdhci_mediachange
 *
 * Description:
 *   Called by board-specific logic -- possible from an interrupt handler --
 *   in order to signal to the driver that a card has been inserted or
 *   removed from the slot
 *
 * Input Parameters:
 *   dev        - An instance of the SDIO driver device state structure.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void cxd56_sdhci_mediachange(struct sdio_dev_s *dev)
{
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;
  sdio_statset_t cdstatus;
  irqstate_t flags;
  uint8_t mediachange = 0;
  int32_t timeout = SDHCI_CARDSTATETIMEOUT;

  /* Update card status */

  if (getreg32(CXD56_SDHCI_PRSSTAT) & SDHCI_PRSSTAT_SDCD)
    {
      while ((getreg32(CXD56_SDHCI_PRSSTAT) & SDHCI_PRSSTAT_CSTS) == 0)
        {
          if (timeout < 1)
            {
              break;
            }

          nxsig_usleep(100000);
          timeout -= 100000;
        }
    }

  flags = enter_critical_section();
  cdstatus = priv->cdstatus;

  if (getreg32(CXD56_SDHCI_PRSSTAT) & SDHCI_PRSSTAT_CINS)
    {
      priv->cdstatus |= SDIO_STATUS_PRESENT;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_PRESENT;
    }

  mcinfo("cdstatus OLD: %02x NEW: %02x\n", cdstatus, priv->cdstatus);

  /* Perform any requested callback if the status has changed */

  if (cdstatus != priv->cdstatus)
    {
      mediachange = 1;
    }

  leave_critical_section(flags);
  if (mediachange)
    {
      cxd56_sdio_callback(priv);
    }
}

/****************************************************************************
 * Name: cxd56_sdhci_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report if the card in the slot is
 *   mechanically write protected.
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO driver device state structure.
 *   wrprotect - true is a card is writeprotected.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void cxd56_sdhci_wrprotect(struct sdio_dev_s *dev, bool wrprotect)
{
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;
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

  mcinfo("cdstatus: %02x\n", priv->cdstatus);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: cxd56_sdio_resetstatus
 *
 * Description:
 *   Reset SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 ****************************************************************************/

void cxd56_sdio_resetstatus(struct sdio_dev_s *dev)
{
  struct cxd56_sdiodev_s *priv = (struct cxd56_sdiodev_s *)dev;
  priv->cdstatus = 0;
  priv->cbevents = SDIOMEDIA_INSERTED;
}

#endif /* CONFIG_CXD56_SDIO */
