/****************************************************************************
 * arch/arm/src/sam34/sam_hsmci.c
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
#include <nuttx/mmcsd.h>

#include <nuttx/irq.h>
#include <nuttx/signal.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "sam_gpio.h"
#include "sam_cmcc.h"
#include "sam_hsmci.h"
#include "sam_periphclks.h"

#ifdef CONFIG_SAM34_DMAC
#  include "sam_dmac.h"
#endif

#include "hardware/sam_pmc.h"
#include "hardware/sam_hsmci.h"
#include "hardware/sam_pinmap.h"

#ifdef CONFIG_SAM34_DMAC
#  include "hardware/sam_dmac.h"
#endif
#ifdef CONFIG_SAM34_PDCA
#  include "hardware/sam_pdc.h"
#endif

#ifdef CONFIG_SAM34_HSMCI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if !defined(CONFIG_SAM34_HSMCI_DMA)
#  warning "Large Non-DMA transfer may result in RX overrun failures"
#elif !defined(CONFIG_SDIO_DMA)
#  warning CONFIG_SDIO_DMA should be defined with CONFIG_SAM34_HSMCI_DMA
#endif

#if !(defined(CONFIG_SAM34_DMAC0) || defined(CONFIG_SAM34_PDCA))
#  warning "HSMCI driver requires CONFIG_SAM34_DMAC0 or CONFIG_SAM34_PDCA"
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE"
#endif

#ifndef CONFIG_SDIO_BLOCKSETUP
#  error "This driver requires CONFIG_SDIO_BLOCKSETUP"
#endif

#ifndef CONFIG_DEBUG_MEMCARD_INFO
#  undef CONFIG_SAM34_HSMCI_CMDDEBUG
#  undef CONFIG_SAM34_HSMCI_XFRDEBUG
#endif

#ifdef CONFIG_SAM34_HSMCI_RDPROOF
#  ifdef CONFIG_SAM34_HSMCI_WRPROOF
#    define HSMCU_PROOF_BITS (HSMCI_MR_RDPROOF | HSMCI_MR_WRPROOF)
#  else
#    define HSMCU_PROOF_BITS HSMCI_MR_RDPROOF
#  endif
#else
#  ifdef CONFIG_SAM34_HSMCI_WRPROOF
#    define HSMCU_PROOF_BITS HSMCI_MR_WRPROOF
#  else
#    define HSMCU_PROOF_BITS (0)
#  endif
#endif

/* Timing */

#define HSMCI_CMDTIMEOUT         (100000)
#define HSMCI_LONGTIMEOUT        (0x7fffffff)

/* Big DTIMER setting */

#define HSMCI_DTIMER_DATATIMEOUT (0x000fffff)

#ifdef CONFIG_SAM34_DMAC0
/* DMA configuration flags */

#define DMA_FLAGS \
  (DMACH_FLAG_FIFO_8BYTES | DMACH_FLAG_FIFOCFG_LARGEST | \
  (DMACHAN_INTF_HSMCI0 << DMACH_FLAG_PERIPHPID_SHIFT) | \
   DMACH_FLAG_PERIPHH2SEL | DMACH_FLAG_PERIPHISPERIPH |  \
   DMACH_FLAG_PERIPHWIDTH_32BITS | DMACH_FLAG_PERIPHCHUNKSIZE_1 | \
   DMACH_FLAG_MEMWIDTH_32BITS | DMACH_FLAG_MEMINCREMENT | \
   DMACH_FLAG_MEMCHUNKSIZE_4)
#endif

/* Status errors:
 *
 *   HSMCI_INT_UNRE          Data transmit underrun
 *   HSMCI_INT_OVRE          Data receive overrun
 *   HSMCI_INT_BLKOVRE       DMA receive block overrun error
 *   HSMCI_INT_CSTOE         Completion signal time-out error
 *                           (see HSMCI_CSTOR)
 *   HSMCI_INT_DTOE          Data time-out error (see HSMCI_DTOR)
 *   HSMCI_INT_DCRCE         Data CRC Error
 *   HSMCI_INT_RTOE          Response Time-out
 *   HSMCI_INT_RENDE         Response End Bit Error
 *   HSMCI_INT_RCRCE         Response CRC Error
 *   HSMCI_INT_RDIRE         Response Direction Error
 *   HSMCI_INT_RINDE         Response Index Error
 */

#define HSMCI_STATUS_ERRORS \
  (HSMCI_INT_UNRE  | HSMCI_INT_OVRE  | HSMCI_INT_BLKOVRE | HSMCI_INT_CSTOE | \
   HSMCI_INT_DTOE  | HSMCI_INT_DCRCE | HSMCI_INT_RTOE    | HSMCI_INT_RENDE | \
   HSMCI_INT_RCRCE | HSMCI_INT_RDIRE | HSMCI_INT_RINDE)

/* Response errors:
 *
 *   HSMCI_INT_CSTOE         Completion signal time-out error
 *                           (see HSMCI_CSTOR)
 *   HSMCI_INT_RTOE          Response Time-out
 *   HSMCI_INT_RENDE         Response End Bit Error
 *   HSMCI_INT_RCRCE         Response CRC Error
 *   HSMCI_INT_RDIRE         Response Direction Error
 *   HSMCI_INT_RINDE         Response Index Error
 */

#define HSMCI_RESPONSE_ERRORS \
  (HSMCI_INT_CSTOE | HSMCI_INT_RTOE  | HSMCI_INT_RENDE   | HSMCI_INT_RCRCE | \
   HSMCI_INT_RDIRE | HSMCI_INT_RINDE)

#define HSMCI_RESPONSE_NOCRC_ERRORS \
  (HSMCI_INT_CSTOE | HSMCI_INT_RTOE  | HSMCI_INT_RENDE   | HSMCI_INT_RDIRE | \
   HSMCI_INT_RINDE)

#define HSMCI_RESPONSE_TIMEOUT_ERRORS \
  (HSMCI_INT_CSTOE | HSMCI_INT_RTOE)

/* Data transfer errors:
 *
 *   HSMCI_INT_UNRE          Data transmit underrun
 *   HSMCI_INT_OVRE          Data receive overrun
 *   HSMCI_INT_BLKOVRE       DMA receive block overrun error
 *   HSMCI_INT_CSTOE         Completion signal time-out error
 *                           (see HSMCI_CSTOR)
 *   HSMCI_INT_DTOE          Data time-out error (see HSMCI_DTOR)
 *   HSMCI_INT_DCRCE         Data CRC Error
 */

#ifdef CONFIG_SAM34_DMAC0
#  if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X)
#    define HSMCI_DATA_ERRORS \
       (HSMCI_INT_UNRE | HSMCI_INT_OVRE  | HSMCI_INT_BLKOVRE | HSMCI_INT_CSTOE | \
        HSMCI_INT_DTOE | HSMCI_INT_DCRCE)
#  else
#    define HSMCI_DATA_ERRORS \
       (HSMCI_INT_UNRE | HSMCI_INT_OVRE | HSMCI_INT_CSTOE | HSMCI_INT_DTOE | \
        HSMCI_INT_DCRCE)
#  endif
#else
  #define HSMCI_DATA_ERRORS \
    (HSMCI_INT_UNRE | HSMCI_INT_OVRE | HSMCI_INT_CSTOE | HSMCI_INT_DTOE | \
     HSMCI_INT_DCRCE)
#endif

#define HSMCI_DATA_TIMEOUT_ERRORS \
  (HSMCI_INT_CSTOE | HSMCI_INT_DTOE)

#ifdef CONFIG_SAM34_DMAC0
#  if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X)
#    define HSMCI_DATA_DMARECV_ERRORS \
      (HSMCI_INT_OVRE  | HSMCI_INT_BLKOVRE | HSMCI_INT_CSTOE | HSMCI_INT_DTOE | \
       HSMCI_INT_DCRCE)
#  else
#    define HSMCI_DATA_DMARECV_ERRORS \
      (HSMCI_INT_OVRE  | HSMCI_INT_CSTOE | HSMCI_INT_DTOE | HSMCI_INT_DCRCE)
#  endif
#endif

#ifdef CONFIG_SAM34_PDCA
#  define HSMCI_DATA_DMARECV_ERRORS \
    (HSMCI_INT_OVRE | HSMCI_INT_CSTOE | HSMCI_INT_DTOE | HSMCI_INT_DCRCE)
#endif

#define HSMCI_DATA_DMASEND_ERRORS \
  (HSMCI_INT_UNRE | HSMCI_INT_CSTOE | HSMCI_INT_DTOE | HSMCI_INT_DCRCE)

/* Data transfer status and interrupt mask bits.
 *
 * The XFRDONE flag in the HSMCI_SR indicates exactly when the read or
 * write sequence is finished.
 *
 *   0: A transfer is in progress.
 *   1: Command register is ready to operate and the data bus is in the idle
 *      state.
 *
 * DMADONE: DMA Transfer done
 *
 *   0: DMA buffer transfer has not completed since the last read of
 *      HSMCI_SR register.
 *   1: DMA buffer transfer has completed.
 */

#ifdef CONFIG_SAM34_DMAC0
#  define HSMCI_DMARECV_INTS \
    (HSMCI_DATA_DMARECV_ERRORS | HSMCI_INT_XFRDONE /* | HSMCI_INT_DMADONE */)
#  define HSMCI_DMASEND_INTS \
    (HSMCI_DATA_DMASEND_ERRORS | HSMCI_INT_XFRDONE /* | HSMCI_INT_DMADONE */)
#endif

#ifdef CONFIG_SAM34_PDCA
  #define HSMCI_DMARECV_INTS \
    (HSMCI_DATA_DMARECV_ERRORS | HSMCI_INT_ENDRX)
  #define HSMCI_DMASEND_INTS \
    (HSMCI_DATA_DMASEND_ERRORS | HSMCI_INT_ENDTX)
#endif

/* Event waiting interrupt mask bits.
 *
 * CMDRDY (Command Ready):
 *
 *   0: A command is in progress
 *   1: The last command has been sent.  The CMDRDY flag is released 8 bits
 *      after the end of the card response. Cleared when writing in the
 *      HSMCI_CMDR
 */

#define HSMCI_CMDRESP_INTS \
  (HSMCI_RESPONSE_ERRORS | HSMCI_INT_CMDRDY)
#define HSMCI_CMDRESP_NOCRC_INTS \
  (HSMCI_RESPONSE_NOCRC_ERRORS | HSMCI_INT_CMDRDY)

/* Register logging support */

#ifdef CONFIG_SAM34_HSMCI_XFRDEBUG
#  ifdef CONFIG_DEBUG_DMA
#    define SAMPLENDX_BEFORE_SETUP  0
#    define SAMPLENDX_BEFORE_ENABLE 1
#    define SAMPLENDX_AFTER_SETUP   2
#    define SAMPLENDX_END_TRANSFER  3
#    define SAMPLENDX_DMA_CALLBACK  4
#    define DEBUG_NDMASAMPLES       5
#  else
#    define SAMPLENDX_BEFORE_SETUP  0
#    define SAMPLENDX_AFTER_SETUP   1
#    define SAMPLENDX_END_TRANSFER  2
#    define DEBUG_NDMASAMPLES       3
#  endif
#endif

#ifdef CONFIG_SAM34_HSMCI_CMDDEBUG
#  define SAMPLENDX_AFTER_CMDR      0
#  define SAMPLENDX_AT_WAKEUP       1
#  define DEBUG_NCMDSAMPLES         2
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the state of the SAM3/4 HSMCI interface */

struct sam_dev_s
{
  struct sdio_dev_s  dev;             /* Standard, base SDIO interface */

  /* SAM3/4-specific extensions */

  /* Event support */

  sem_t              waitsem;         /* Implements event waiting */
  sdio_eventset_t    waitevents;      /* Set of events to be waited for */
  uint32_t           waitmask;        /* Interrupt enables for event waiting */
  uint32_t           cmdrmask;        /* Interrupt enables for this
                                       * particular cmd/response */
  volatile sdio_eventset_t wkupevent; /* The event that caused the wakeup */
  struct wdog_s      waitwdog;        /* Watchdog that handles event timeouts */
#ifdef CONFIG_SAM34_DMAC0
  bool               dmabusy;         /* TRUE: DMA is in progress */
#endif

  /* Callback support */

  sdio_statset_t     cdstatus;        /* Card status */
  sdio_eventset_t    cbevents;        /* Set of events to be cause callbacks */
  worker_t           callback;        /* Registered callback function */
  void              *cbarg;           /* Registered callback argument */
  struct work_s      cbwork;          /* Callback work queue structure */

  /* Interrupt mode data transfer support */

  uint32_t           xfrmask;         /* Interrupt enables for data transfer */

  /* DMA data transfer support */

  bool               widebus;         /* Required for DMA support */
#ifdef CONFIG_SAM34_DMAC0
  DMA_HANDLE         dma;             /* Handle for DMA channel */
#endif
};

/* Register logging support */

#if defined(CONFIG_SAM34_HSMCI_XFRDEBUG) || defined(CONFIG_SAM34_HSMCI_CMDDEBUG)
struct sam_hsmciregs_s
{
  uint32_t mr;       /* Mode Register */
  uint32_t dtor;     /* Data Timeout Register */
  uint32_t sdcr;     /* SD/SDIO Card Register */
  uint32_t argr;     /* Argument Register */
  uint32_t blkr;     /* Block Register */
  uint32_t cstor;    /* Completion Signal Timeout Register */
  uint32_t rsp0;     /* Response Register 0 */
  uint32_t rsp1;     /* Response Register 1 */
  uint32_t rsp2;     /* Response Register 2 */
  uint32_t rsp3;     /* Response Register 3 */
  uint32_t sr;       /* Status Register */
  uint32_t imr;      /* Interrupt Mask Register */
#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X)
  uint32_t dma;      /* DMA Configuration Register */
#endif
  uint32_t cfg;      /* Configuration Register */
  uint32_t wpmr;     /* Write Protection Mode Register */
  uint32_t wpsr;     /* Write Protection Status Register */

#ifdef CONFIG_SAM34_PDCA
  uint32_t pdc_rpr;  /* Receive Pointer Register */
  uint32_t pdc_rcr;  /* Receive Counter Register */
  uint32_t pdc_tpr;  /* Transmit Pointer Register */
  uint32_t pdc_tcr;  /* Transmit Counter Register */
  uint32_t pdc_rnpr; /* Receive Next Pointer Register */
  uint32_t pdc_rncr; /* Receive Next Counter Register */
  uint32_t pdc_tnpr; /* Transmit Next Pointer Register */
  uint32_t pdc_tncr; /* Transmit Next Counter Register */
#if 0
  uint32_t pdc_ptcr; /* Transfer Control Register */
#endif
  uint32_t pdc_ptsr; /* Transfer Status Register */
#endif
};
#endif

#ifdef CONFIG_SAM34_HSMCI_XFRDEBUG
struct sam_xfrregs_s
{
  struct sam_hsmciregs_s hsmci;
#if defined(CONFIG_DEBUG_DMA) && defined(CONFIG_SAM34_DMAC0)
  struct sam_dmaregs_s  dma;
#endif
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static void sam_configwaitints(struct sam_dev_s *priv, uint32_t waitmask,
              sdio_eventset_t waitevents);
static void sam_disablewaitints(struct sam_dev_s *priv,
              sdio_eventset_t wkupevents);
static inline void sam_configxfrints(struct sam_dev_s *priv,
              uint32_t xfrmask);
static void sam_disablexfrints(struct sam_dev_s *priv);
static void sam_enableints(struct sam_dev_s *priv);

static inline void sam_disable(void);
static inline void sam_enable(void);

/* Register Sampling ********************************************************/

#if defined(CONFIG_SAM34_HSMCI_XFRDEBUG) || defined(CONFIG_SAM34_HSMCI_CMDDEBUG)
static void sam_hsmcisample(struct sam_hsmciregs_s *regs);
static void sam_hsmcidump(struct sam_hsmciregs_s *regs, const char *msg);
#endif

#ifdef CONFIG_SAM34_HSMCI_XFRDEBUG
static void sam_xfrsampleinit(void);
static void sam_xfrsample(struct sam_dev_s *priv, int index);
static void sam_xfrdumpone(struct sam_dev_s *priv,
              struct sam_xfrregs_s *regs, const char *msg);
static void sam_xfrdump(struct sam_dev_s *priv);
#else
#  define   sam_xfrsampleinit()
#  define   sam_xfrsample(priv,index)
#  define   sam_xfrdump(priv)
#endif

#ifdef CONFIG_SAM34_HSMCI_CMDDEBUG
static void sam_cmdsampleinit(void);
static inline void sam_cmdsample1(int index3);
static inline void sam_cmdsample2(int index, uint32_t sr);
static void sam_cmddump(void);
#else
#  define   sam_cmdsampleinit()
#  define   sam_cmdsample1(index)
#  define   sam_cmdsample2(index,sr)
#  define   sam_cmddump()
#endif

#ifdef CONFIG_SAM34_DMAC0
/* DMA Helpers **************************************************************/

static void sam_dmacallback(DMA_HANDLE handle, void *arg, int result);
#endif

/* Data Transfer Helpers ****************************************************/

static void sam_eventtimeout(wdparm_t arg);
static void sam_endwait(struct sam_dev_s *priv, sdio_eventset_t wkupevent);
static void sam_endtransfer(struct sam_dev_s *priv,
              sdio_eventset_t wkupevent);
static void sam_notransfer(struct sam_dev_s *priv);

/* Interrupt Handling *******************************************************/

static int  sam_interrupt(int irq, void *context, void *arg);

/* SDIO interface methods ***************************************************/

/* Initialization/setup */

static void sam_reset(struct sdio_dev_s *dev);
static sdio_capset_t sam_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t sam_status(struct sdio_dev_s *dev);
static void sam_widebus(struct sdio_dev_s *dev, bool enable);
static void sam_clock(struct sdio_dev_s *dev,
              enum sdio_clock_e rate);
static int  sam_attach(struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int  sam_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t arg);
static void sam_blocksetup(struct sdio_dev_s *dev, unsigned int blocklen,
              unsigned int nblocks);
static int  sam_cancel(struct sdio_dev_s *dev);
static int  sam_waitresponse(struct sdio_dev_s *dev, uint32_t cmd);
static int  sam_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  sam_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t rlong[4]);
static int  sam_recvnotimpl(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rnotimpl);

/* EVENT handler */

static void sam_waitenable(struct sdio_dev_s *dev,
              sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t sam_eventwait(struct sdio_dev_s *dev);
static void sam_callbackenable(struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static int  sam_registercallback(struct sdio_dev_s *dev,
              worker_t callback, void *arg);

/* DMA */

#ifdef CONFIG_SAM34_HSMCI_DMA
static int  sam_dmarecvsetup(struct sdio_dev_s *dev,
              uint8_t *buffer, size_t buflen);
static int  sam_dmasendsetup(struct sdio_dev_s *dev,
              const uint8_t *buffer, size_t buflen);
#endif

/* Initialization/uninitialization/reset ************************************/

static void sam_callback(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct sam_dev_s g_sdiodev =
{
  .dev =
  {
    .reset            = sam_reset,
    .capabilities     = sam_capabilities,
    .status           = sam_status,
    .widebus          = sam_widebus,
    .clock            = sam_clock,
    .attach           = sam_attach,
    .sendcmd          = sam_sendcmd,
    .blocksetup       = sam_blocksetup,
    .recvsetup        = sam_dmarecvsetup,
    .sendsetup        = sam_dmasendsetup,
    .cancel           = sam_cancel,
    .waitresponse     = sam_waitresponse,
    .recv_r1          = sam_recvshort,
    .recv_r2          = sam_recvlong,
    .recv_r3          = sam_recvshort,
    .recv_r4          = sam_recvnotimpl,
    .recv_r5          = sam_recvnotimpl,
    .recv_r6          = sam_recvshort,
    .recv_r7          = sam_recvshort,
    .waitenable       = sam_waitenable,
    .eventwait        = sam_eventwait,
    .callbackenable   = sam_callbackenable,
    .registercallback = sam_registercallback,
#ifdef CONFIG_SDIO_DMA
    .dmarecvsetup     = sam_dmarecvsetup,
    .dmasendsetup     = sam_dmasendsetup,
#endif
  },
};

/* Register logging support */

#ifdef CONFIG_SAM34_HSMCI_XFRDEBUG
static struct sam_xfrregs_s   g_xfrsamples[DEBUG_NDMASAMPLES];
#endif
#ifdef CONFIG_SAM34_HSMCI_CMDDEBUG
static struct sam_hsmciregs_s g_cmdsamples[DEBUG_NCMDSAMPLES];
#endif
#if defined(CONFIG_SAM34_HSMCI_XFRDEBUG) && defined(CONFIG_SAM34_HSMCI_CMDDEBUG)
static bool                     g_xfrinitialized;
static bool                     g_cmdinitialized;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_configwaitints
 *
 * Description:
 *   Configure HSMCI interrupts needed to support the wait function.  Wait
 *   interrupts are configured here, but not enabled until
 *   sam_enableints() is called.  Why?  Because the XFRDONE interrupt
 *   is always pending until start the data transfer.
 *
 * Input Parameters:
 *   priv       - A reference to the HSMCI device state structure
 *   waitmask   - The set of bits in the HSMCI MASK register to set
 *   waitevents - Waited for events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_configwaitints(struct sam_dev_s *priv, uint32_t waitmask,
                               sdio_eventset_t waitevents)
{
  irqstate_t flags;

  /* Save all of the data in one, atomic operation. */

  flags = enter_critical_section();
  priv->waitevents = waitevents;
  priv->wkupevent  = 0;
  priv->waitmask   = waitmask;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_disablewaitints
 *
 * Description:
 *   Disable HSMCI interrupts and save wakeup event.  Called
 *
 * Input Parameters:
 *   priv       - A reference to the HSMCI device state structure
 *   wkupevent  - Wake-up event(s)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_disablewaitints(struct sam_dev_s *priv,
                                sdio_eventset_t wkupevent)
{
  irqstate_t flags;

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags = enter_critical_section();
  priv->waitevents = 0;
  priv->wkupevent  = wkupevent;
  priv->waitmask   = 0;
  putreg32(~priv->xfrmask, SAM_HSMCI_IDR);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_configxfrints
 *
 * Description:
 *   Configure HSMCI interrupts needed to support the data transfer.  Data
 *   transfer interrupts are configured here, but not enabled until
 *   sam_enableints() is called.  Why?  Because the XFRDONE interrupt
 *   is always pending until start the data transfer.
 *
 * Input Parameters:
 *   priv    - A reference to the HSMCI device state structure
 *   xfrmask - The set of bits in the HSMCI MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sam_configxfrints(struct sam_dev_s *priv,
                                     uint32_t xfrmask)
{
  priv->xfrmask = xfrmask;
}

/****************************************************************************
 * Name: sam_disablexfrints
 *
 * Description:
 *   Disable HSMCI interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the HSMCI device state structure
 *   xfrmask - The set of bits in the HSMCI MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_disablexfrints(struct sam_dev_s *priv)
{
  irqstate_t flags = enter_critical_section();
  priv->xfrmask = 0;
  putreg32(~priv->waitmask, SAM_HSMCI_IDR);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_enableints
 *
 * Description:
 *   Enable the previously configured HSMCI interrupts needed to support the
 *   wait and transfer functions.
 *
 * Input Parameters:
 *   priv - A reference to the HSMCI device state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sam_enableints(struct sam_dev_s *priv)
{
  /* Enable all interrupts associated with the waited-for event */

  putreg32(priv->xfrmask | priv->waitmask, SAM_HSMCI_IER);
}

/****************************************************************************
 * Name: sam_disable
 *
 * Description:
 *   Disable the HSMCI
 *
 ****************************************************************************/

static inline void sam_disable(void)
{
  /* Disable the MCI peripheral clock */

  sam_hsmci_disableclk();

  /* Disable the MCI */

  putreg32(HSMCI_CR_MCIDIS, SAM_HSMCI_CR);

  /* Disable all the interrupts */

  putreg32(0xffffffff, SAM_HSMCI_IDR);
}

/****************************************************************************
 * Name: sam_enable
 *
 * Description:
 *   Enable the HSMCI
 *
 ****************************************************************************/

static inline void sam_enable(void)
{
  /* Enable the MCI peripheral clock */

  sam_hsmci_enableclk();

  /* Enable the MCI and the Power Saving */

  putreg32(HSMCI_CR_MCIEN, SAM_HSMCI_CR);
}

/****************************************************************************
 * Name: sam_hsmcisample
 *
 * Description:
 *   Sample HSMCI registers
 *
 ****************************************************************************/

#if defined(CONFIG_SAM34_HSMCI_XFRDEBUG) || defined(CONFIG_SAM34_HSMCI_CMDDEBUG)
static void sam_hsmcisample(struct sam_hsmciregs_s *regs)
{
  regs->mr       = getreg32(SAM_HSMCI_MR);
  regs->dtor     = getreg32(SAM_HSMCI_DTOR);
  regs->sdcr     = getreg32(SAM_HSMCI_SDCR);
  regs->argr     = getreg32(SAM_HSMCI_ARGR);
  regs->blkr     = getreg32(SAM_HSMCI_BLKR);
  regs->cstor    = getreg32(SAM_HSMCI_CSTOR);
  regs->rsp0     = getreg32(SAM_HSMCI_RSPR0);
  regs->rsp1     = getreg32(SAM_HSMCI_RSPR1);
  regs->rsp2     = getreg32(SAM_HSMCI_RSPR2);
  regs->rsp3     = getreg32(SAM_HSMCI_RSPR3);
  regs->sr       = getreg32(SAM_HSMCI_SR);
  regs->imr      = getreg32(SAM_HSMCI_IMR);
#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X)
  regs->dma      = getreg32(SAM_HSMCI_DMA);
#endif
  regs->cfg      = getreg32(SAM_HSMCI_CFG);
  regs->wpmr     = getreg32(SAM_HSMCI_WPMR);
  regs->wpsr     = getreg32(SAM_HSMCI_WPSR);

#ifdef CONFIG_SAM34_PDCA
  regs->pdc_rpr  = getreg32(SAM_HSMCI_PDC_RPR);
  regs->pdc_rcr  = getreg32(SAM_HSMCI_PDC_RCR);
  regs->pdc_tpr  = getreg32(SAM_HSMCI_PDC_TPR);
  regs->pdc_tcr  = getreg32(SAM_HSMCI_PDC_TCR);
  regs->pdc_rnpr = getreg32(SAM_HSMCI_PDC_RNPR);
  regs->pdc_rncr = getreg32(SAM_HSMCI_PDC_RNCR);
  regs->pdc_tnpr = getreg32(SAM_HSMCI_PDC_TNPR);
  regs->pdc_tncr = getreg32(SAM_HSMCI_PDC_TNCR);
#if 0
  regs->pdc_ptcr = getreg32(SAM_HSMCI_PDC_PTCR);
#endif
  regs->pdc_ptsr = getreg32(SAM_HSMCI_PDC_PTSR);
#endif
}
#endif

/****************************************************************************
 * Name: sam_hsmcidump
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#if defined(CONFIG_SAM34_HSMCI_XFRDEBUG) || defined(CONFIG_SAM34_HSMCI_CMDDEBUG)
static void sam_hsmcidump(struct sam_hsmciregs_s *regs, const char *msg)
{
  mcinfo("HSMCI Registers: %s\n", msg);
  mcinfo("     MR[%08x]: %08x\n",
         SAM_HSMCI_MR, regs->mr);
  mcinfo("   DTOR[%08x]: %08x\n",
         SAM_HSMCI_DTOR, regs->dtor);
  mcinfo("   SDCR[%08x]: %08x\n",
         SAM_HSMCI_SDCR, regs->sdcr);
  mcinfo("   ARGR[%08x]: %08x\n",
         SAM_HSMCI_ARGR, regs->argr);
  mcinfo("   BLKR[%08x]: %08x\n",
         SAM_HSMCI_BLKR, regs->blkr);
  mcinfo("  CSTOR[%08x]: %08x\n",
         SAM_HSMCI_CSTOR, regs->cstor);
  mcinfo("  RSPR0[%08x]: %08x\n",
         SAM_HSMCI_RSPR0, regs->rsp0);
  mcinfo("  RSPR1[%08x]: %08x\n",
         SAM_HSMCI_RSPR1, regs->rsp1);
  mcinfo("  RSPR2[%08x]: %08x\n",
         SAM_HSMCI_RSPR2, regs->rsp2);
  mcinfo("  RSPR3[%08x]: %08x\n",
         SAM_HSMCI_RSPR3, regs->rsp3);
  mcinfo("     SR[%08x]: %08x\n",
         SAM_HSMCI_SR, regs->sr);
  mcinfo("    IMR[%08x]: %08x\n",
         SAM_HSMCI_IMR, regs->imr);
#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X)
  mcinfo("    DMA[%08x]: %08x\n",
         SAM_HSMCI_DMA, regs->dma);
#endif
  mcinfo("    CFG[%08x]: %08x\n",
         SAM_HSMCI_CFG, regs->cfg);
  mcinfo("   WPMR[%08x]: %08x\n",
         SAM_HSMCI_WPMR, regs->wpmr);
  mcinfo("   WPSR[%08x]: %08x\n",
         SAM_HSMCI_WPSR, regs->wpsr);

#ifdef CONFIG_SAM34_PDCA
  mcinfo("HSMCI PDC Registers:\n");
  mcinfo("    RPR[%08x]: %08x\n",
         SAM_HSMCI_PDC_RPR, regs->pdc_rpr);
  mcinfo("    RCR[%08x]: %08x\n",
         SAM_HSMCI_PDC_RCR, regs->pdc_rcr);
  mcinfo("    TPR[%08x]: %08x\n",
         SAM_HSMCI_PDC_TPR, regs->pdc_tpr);
  mcinfo("    TCR[%08x]: %08x\n",
         SAM_HSMCI_PDC_TCR, regs->pdc_tcr);
  mcinfo("   RNPR[%08x]: %08x\n",
         SAM_HSMCI_PDC_RNPR, regs->pdc_rnpr);
  mcinfo("   RNCR[%08x]: %08x\n",
         SAM_HSMCI_PDC_RNCR, regs->pdc_rncr);
  mcinfo("   TNPR[%08x]: %08x\n",
         SAM_HSMCI_PDC_TNPR, regs->pdc_tnpr);
  mcinfo("   TNCR[%08x]: %08x\n",
         SAM_HSMCI_PDC_TNCR, regs->pdc_tncr);
#if 0
  mcinfo("    TCR[%08x]: %08x\n",
         SAM_HSMCI_PDC_PTCR, regs->pdc_ptcr);
#endif
  mcinfo("   PTSR[%08x]: %08x\n",
         SAM_HSMCI_PDC_PTSR, regs->pdc_ptsr);
#endif
}
#endif

/****************************************************************************
 * Name: sam_xfrsample
 *
 * Description:
 *   Sample HSMCI/DMA registers
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_HSMCI_XFRDEBUG
static void sam_xfrsample(struct sam_dev_s *priv, int index)
{
  struct sam_xfrregs_s *regs = &g_xfrsamples[index];
#if defined(CONFIG_DEBUG_DMA) && defined(CONFIG_SAM34_DMAC0)
  sam_dmasample(priv->dma, &regs->dma);
#endif
  sam_hsmcisample(&regs->hsmci);
}
#endif

/****************************************************************************
 * Name: sam_xfrsampleinit
 *
 * Description:
 *   Setup prior to collecting transfer samples
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_HSMCI_XFRDEBUG
static void sam_xfrsampleinit(void)
{
  memset(g_xfrsamples, 0xff,
         DEBUG_NDMASAMPLES * sizeof(struct sam_xfrregs_s));

#ifdef CONFIG_SAM34_HSMCI_CMDDEBUG
  g_xfrinitialized = true;
#endif
}
#endif

/****************************************************************************
 * Name: sam_xfrdumpone
 *
 * Description:
 *   Dump one transfer register sample
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_HSMCI_XFRDEBUG
static void sam_xfrdumpone(struct sam_dev_s *priv,
                           struct sam_xfrregs_s *regs, const char *msg)
{
#if defined(CONFIG_DEBUG_DMA) && defined(CONFIG_SAM34_DMAC0)
  sam_dmadump(priv->dma, &regs->dma, msg);
#endif
  sam_hsmcidump(&regs->hsmci, msg);
}
#endif

/****************************************************************************
 * Name: sam_xfrdump
 *
 * Description:
 *   Dump all transfer-related, sampled register data
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_HSMCI_XFRDEBUG
static void  sam_xfrdump(struct sam_dev_s *priv)
{
#ifdef CONFIG_SAM34_HSMCI_CMDDEBUG
  if (g_xfrinitialized)
#endif
    {
      sam_xfrdumpone(priv, &g_xfrsamples[SAMPLENDX_BEFORE_SETUP],
                     "Before setup");
#ifdef CONFIG_DEBUG_DMA
      sam_xfrdumpone(priv, &g_xfrsamples[SAMPLENDX_BEFORE_ENABLE],
                     "Before DMA enable");
#endif
      sam_xfrdumpone(priv, &g_xfrsamples[SAMPLENDX_AFTER_SETUP],
                     "After setup");
      sam_xfrdumpone(priv, &g_xfrsamples[SAMPLENDX_END_TRANSFER],
                     "End of transfer");
#if defined(CONFIG_DEBUG_DMA) && defined(CONFIG_SAM34_DMAC0)
      sam_xfrdumpone(priv, &g_xfrsamples[SAMPLENDX_DMA_CALLBACK],
                     "DMA Callback");
#endif
#ifdef CONFIG_SAM34_HSMCI_CMDDEBUG
      g_xfrinitialized = false;
#endif
    }
}
#endif

/****************************************************************************
 * Name: sam_cmdsampleinit
 *
 * Description:
 *   Setup prior to collecting command/response samples
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_HSMCI_CMDDEBUG
static void sam_cmdsampleinit(void)
{
  memset(g_cmdsamples, 0xff,
         DEBUG_NCMDSAMPLES * sizeof(struct sam_hsmciregs_s));

#ifdef CONFIG_SAM34_HSMCI_XFRDEBUG
  g_cmdinitialized = true;
#endif
}
#endif

/****************************************************************************
 * Name: sam_cmdsample1 & 2
 *
 * Description:
 *   Sample command/response registers
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_HSMCI_CMDDEBUG
static inline void sam_cmdsample1(int index)
{
  sam_hsmcisample(&g_cmdsamples[index]);
}

static inline void sam_cmdsample2(int index, uint32_t sr)
{
  sam_hsmcisample(&g_cmdsamples[index]);
  g_cmdsamples[index].sr = sr;
}
#endif

/****************************************************************************
 * Name: sam_cmddump
 *
 * Description:
 *   Dump all command/response register data
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_HSMCI_CMDDEBUG
static void sam_cmddump(void)
{
#ifdef CONFIG_SAM34_HSMCI_XFRDEBUG
  if (g_cmdinitialized)
#endif
    {
      sam_hsmcidump(&g_cmdsamples[SAMPLENDX_AFTER_CMDR],
                    "After command setup");
      sam_hsmcidump(&g_cmdsamples[SAMPLENDX_AT_WAKEUP],
                    "After wakeup");
#ifdef CONFIG_SAM34_HSMCI_XFRDEBUG
      g_cmdinitialized = false;
#endif
    }
}
#endif

/****************************************************************************
 * Name: sam_dmacallback
 *
 * Description:
 *   Called when HSMCI DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_DMAC0
static void sam_dmacallback(DMA_HANDLE handle, void *arg, int result)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)arg;

  /* We don't really do anything at the completion of DMA.  The termination
   * of the transfer is driven by the HSMCI interrupts.
   *
   * Mark the DMA not busy.
   */

  priv->dmabusy = false;

  sam_xfrsample((struct sam_dev_s *)arg, SAMPLENDX_DMA_CALLBACK);
}
#endif

/****************************************************************************
 * Name: sam_eventtimeout
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

static void sam_eventtimeout(wdparm_t arg)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)arg;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0);

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. wake up any waiting threads */

      sam_endwait(priv, SDIOWAIT_TIMEOUT);
      mcerr("ERROR: Timeout\n");
    }
}

/****************************************************************************
 * Name: sam_endwait
 *
 * Description:
 *   Wake up a waiting thread if the waited-for event has occurred.
 *
 * Input Parameters:
 *   priv      - An instance of the HSMCI device interface
 *   wkupevent - The event that caused the wait to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void sam_endwait(struct sam_dev_s *priv, sdio_eventset_t wkupevent)
{
  /* Cancel the watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Disable event-related interrupts and save wakeup event */

  sam_disablewaitints(priv, wkupevent);

  /* Wake up the waiting thread */

  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: sam_endtransfer
 *
 * Description:
 *   Terminate a transfer with the provided status.  This function is called
 *   only from the HSMCI interrupt handler when end-of-transfer conditions
 *   are detected.
 *
 * Input Parameters:
 *   priv   - An instance of the HSMCI device interface
 *   wkupevent - The event that caused the transfer to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void sam_endtransfer(struct sam_dev_s *priv,
                            sdio_eventset_t wkupevent)
{
  /* Disable all transfer related interrupts */

  sam_disablexfrints(priv);

  /* No data transfer */

  sam_notransfer(priv);

  /* DMA debug instrumentation */

  sam_xfrsample(priv, SAMPLENDX_END_TRANSFER);

#ifdef CONFIG_SAM34_DMAC0
  /* Make sure that the DMA is stopped (it will be stopped automatically
   * on normal transfers, but not necessarily when the transfer terminates
   * on an error condition.
   */

  sam_dmastop(priv->dma);
  priv->dmabusy = false;

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X)
  /* Disable the DMA handshaking */

  putreg32(0, SAM_HSMCI_DMA);
#endif
#endif

#ifdef CONFIG_SAM34_PDCA
  putreg32(PDC_PTCR_RXTDIS | PDC_PTCR_TXTDIS, SAM_HSMCI_PDC_PTCR);
#endif

  /* Is a thread wait for these data transfer complete events? */

  if ((priv->waitevents & wkupevent) != 0)
    {
      /* Yes.. wake up any waiting threads */

      sam_endwait(priv, wkupevent);
    }
}

/****************************************************************************
 * Name: sam_notransfer
 *
 * Description:
 *   Setup for no transfer.  This is the default setup that is overridden
 *   by sam_dmarecvsetup or sam_dmasendsetup
 *
 * Input Parameters:
 *   priv   - An instance of the HSMCI device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_notransfer(struct sam_dev_s *priv)
{
#ifdef CONFIG_SAM34_DMAC0
  uint32_t regval;

  regval = getreg32(SAM_HSMCI_MR);

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X)
  regval &= ~(HSMCI_MR_RDPROOF | HSMCI_MR_WRPROOF | HSMCI_MR_BLKLEN_MASK);
#else
  regval &= ~(HSMCI_MR_RDPROOF | HSMCI_MR_WRPROOF);
#endif

  putreg32(regval, SAM_HSMCI_MR);
#endif

#ifdef CONFIG_SAM34_PDCA
  modifyreg32(SAM_HSMCI_MR, HSMCI_MR_RDPROOF | HSMCI_MR_WRPROOF |
              HSMCI_MR_PDCMODE, 0);
#endif
}

/****************************************************************************
 * Name: sam_interrupt
 *
 * Description:
 *   HSMCI interrupt handler
 *
 * Input Parameters:
 *   irq - IRQ number of the interrupts
 *   context - Saved machine context at the time of the interrupt
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sam_interrupt(int irq, void *context, void *arg)
{
  struct sam_dev_s *priv = &g_sdiodev;
  uint32_t sr;
  uint32_t enabled;
  uint32_t pending;

  /* Loop while there are pending interrupts. */

  for (; ; )
    {
      /* Check the HSMCI status register.  Mask out all bits that don't
       * correspond to enabled interrupts.  (This depends on the fact that
       * bits are ordered the same in both the SR and IMR registers).  If
       * there are non-zero bits remaining, then we have work to do here.
       */

      sr      = getreg32(SAM_HSMCI_SR);
      enabled = sr & getreg32(SAM_HSMCI_IMR);

      if (enabled == 0)
        {
          break;
        }

      /* Handle in progress, interrupt driven data transfers ****************/

      /* Do any of these interrupts signal the end a data transfer? */

      pending = enabled & priv->xfrmask;
      if (pending != 0)
        {
          /* Yes.. the transfer is complete.  Did it complete with an
           * error?
           */

          if ((pending & HSMCI_DATA_ERRORS) != 0)
            {
              /* Yes.. Was it some kind of timeout error? */

              mcerr("ERROR: enabled: %08" PRIx32 " pending: %08" PRIx32 "\n",
                    enabled, pending);

              if ((pending & HSMCI_DATA_TIMEOUT_ERRORS) != 0)
                {
                  /* Yes.. Terminate with a timeout. */

                  sam_endtransfer(priv,
                                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT);
                }
              else
                {
                  /* No..  Terminate with an I/O error. */

                  sam_endtransfer(priv,
                                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
                }
            }
          else
            {
              /* No.. Then the transfer must have completed successfully */

              sam_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
            }
        }

      /* Handle wait events *************************************************/

      /* Do any of these interrupts signal wakeup event? */

      pending = enabled & priv->waitmask;
      if (pending != 0)
        {
          sdio_eventset_t wkupevent = 0;

          /* Is this a Command-Response sequence completion event? */

          if ((pending & priv->cmdrmask) != 0)
            {
              sam_cmdsample2(SAMPLENDX_AT_WAKEUP, sr);

              /* Yes.. Did the Command-Response sequence end with an error? */

              if ((pending & HSMCI_RESPONSE_ERRORS) != 0)
                {
                  /* Yes.. Was the error some kind of timeout? */

                  mcerr("ERROR: events: %08" PRIx32 " SR: %08" PRIx32 "\n",
                        priv->cmdrmask, enabled);

                  if ((pending & HSMCI_RESPONSE_TIMEOUT_ERRORS) != 0)
                    {
                      /* Yes.. signal a timeout error */

                      wkupevent = SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE |
                                  SDIOWAIT_TIMEOUT;
                    }
                  else
                    {
                      /* No.. signal some generic I/O error */

                      wkupevent = SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE |
                                  SDIOWAIT_ERROR;
                    }
                }
              else
                {
                  /* The Command-Response sequence ended with no error */

                  wkupevent = SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE;
                }

              /* Yes.. Is there a thread waiting for this event set? */

              wkupevent &= priv->waitevents;
              if (wkupevent != 0)
                {
                  /* Yes.. wake the thread up */

                  sam_endwait(priv, wkupevent);
                }
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: sam_reset
 *
 * Description:
 *   Reset the HSMCI controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_reset(struct sdio_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  irqstate_t flags;

  /* Enable the MCI clock */

  flags = enter_critical_section();
  sam_hsmci_enableclk();

  /* Reset the MCI */

  putreg32(HSMCI_CR_SWRST, SAM_HSMCI_CR);

  /* Disable the MCI */

  putreg32(HSMCI_CR_MCIDIS | HSMCI_CR_PWSDIS, SAM_HSMCI_CR);

  /* Disable all the interrupts */

  putreg32(0xffffffff, SAM_HSMCI_IDR);

  /* Set the Data Timeout Register */

  putreg32(HSMCI_DTOR_DTOCYC_MAX | HSMCI_DTOR_DTOMUL_MAX, SAM_HSMCI_DTOR);

  /* Set the Mode Register for ID mode frequency (probably 400KHz) */

  sam_clock(dev, CLOCK_IDMODE);

  /* Set the SDCard Register */

  putreg32(HSMCI_SDCR_SDCSEL_SLOTA | HSMCI_SDCR_SDCBUS_1BIT, SAM_HSMCI_SDCR);

  /* Enable the MCI controller */

  putreg32(HSMCI_CR_MCIEN, SAM_HSMCI_CR);

  /* Disable the DMA interface */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X)
  putreg32(0, SAM_HSMCI_DMA);
#endif

#ifdef CONFIG_SAM34_PDCA
  putreg32(PDC_PTCR_RXTDIS | PDC_PTCR_TXTDIS, SAM_HSMCI_PDC_PTCR);
#endif

  /* Configure MCI */

  putreg32(HSMCI_CFG_FIFOMODE, SAM_HSMCI_CFG);

  /* No data transfer */

  sam_notransfer(priv);

  /* Reset data */

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitmask   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */
#ifdef CONFIG_SAM34_DMAC0
  priv->dmabusy    = false;  /* No DMA in progress */
#endif

  wd_cancel(&priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->xfrmask    = 0;      /* Interrupt enables for data transfer */

  /* DMA data transfer support */

  priv->widebus    = false;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_capabilities
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

static sdio_capset_t sam_capabilities(struct sdio_dev_s *dev)
{
  sdio_capset_t caps = 0;

#ifdef CONFIG_SAM34_HSMCI_DMA
  caps |= SDIO_CAPS_DMASUPPORTED;
  caps |= SDIO_CAPS_DMABEFOREWRITE;
#endif

  return caps;
}

/****************************************************************************
 * Name: sam_status
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see sam_status_* defines)
 *
 ****************************************************************************/

static sdio_statset_t sam_status(struct sdio_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  return priv->cdstatus;
}

/****************************************************************************
 * Name: sam_widebus
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

static void sam_widebus(struct sdio_dev_s *dev, bool wide)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t regval;

  /* Set 1-bit or 4-bit bus by configuring the SDCBUS field of the SDCR
   * register.
   */

  regval  = getreg32(SAM_HSMCI_SDCR);
  regval &= ~HSMCI_SDCR_SDCBUS_MASK;
  regval |= wide ? HSMCI_SDCR_SDCBUS_4BIT : HSMCI_SDCR_SDCBUS_1BIT;
  putreg32(regval, SAM_HSMCI_SDCR);

  /* Remember the setting */

  priv->widebus = wide;
}

/****************************************************************************
 * Name: sam_clock
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

static void sam_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  uint32_t regval;
  bool enable = true;

  /* Fetch the current mode register and mask out the clkdiv (and pwsdiv) */

  regval = getreg32(SAM_HSMCI_MR);

  /* Does this HSMCI support the CLOCKODD bit? */

#ifdef HSMCI_MR_CLKODD
  regval &= ~(HSMCI_MR_CLKDIV_MASK | HSMCI_MR_PWSDIV_MASK | HSMCI_MR_CLKODD);
#else
  regval &= ~(HSMCI_MR_CLKDIV_MASK | HSMCI_MR_PWSDIV_MASK);
#endif

  /* These clock devisor values that must be defined in the board-specific
   * board.h header file: HSMCI_INIT_CLKDIV, HSMCI_MMCXFR_CLKDIV,
   * HSMCI_SDXFR_CLKDIV, and HSMCI_SDWIDEXFR_CLKDIV.
   */

  switch (rate)
    {
    default:
    case CLOCK_SDIO_DISABLED:     /* Clock is disabled */
      regval |= HSMCI_INIT_CLKDIV | HSMCI_MR_PWSDIV_MAX;
      enable = false;
      return;

    case CLOCK_IDMODE:            /* Initial ID mode clocking (<400KHz) */
      regval |= HSMCI_INIT_CLKDIV | HSMCI_MR_PWSDIV_MAX;
      break;

    case CLOCK_MMC_TRANSFER:      /* MMC normal operation clocking */
      regval |= HSMCI_MMCXFR_CLKDIV | HSMCI_MR_PWSDIV_MAX;
      break;

    case CLOCK_SD_TRANSFER_1BIT:  /* SD normal operation clocking (narrow 1-bit mode) */
      regval |= HSMCI_SDXFR_CLKDIV | HSMCI_MR_PWSDIV_MAX;
      break;

    case CLOCK_SD_TRANSFER_4BIT:  /* SD normal operation clocking (wide 4-bit mode) */
      regval |= HSMCI_SDWIDEXFR_CLKDIV | HSMCI_MR_PWSDIV_MAX;
      break;
    };

  /* Set the new clock  diver and make sure that the clock is enabled or
   * disabled, whichever the case.
   */

  putreg32(regval, SAM_HSMCI_MR);
  if (enable)
    {
      sam_enable();
    }
  else
    {
      sam_disable();
    }
}

/****************************************************************************
 * Name: sam_attach
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

static int sam_attach(struct sdio_dev_s *dev)
{
  int ret;

  /* Attach the HSMCI interrupt handler */

  ret = irq_attach(SAM_IRQ_HSMCI, sam_interrupt, NULL);
  if (ret == OK)
    {
      /* Disable all interrupts at the HSMCI controller and clear (most)
       * static interrupt flags by reading the status register.
       */

      putreg32(0xffffffff, SAM_HSMCI_IDR);
      getreg32(SAM_HSMCI_SR);

      /* Enable HSMCI interrupts at the NVIC.  They can now be enabled at
       * the HSMCI controller as needed.
       */

      up_enable_irq(SAM_IRQ_HSMCI);
    }

  return ret;
}

/****************************************************************************
 * Name: sam_sendcmd
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

static int sam_sendcmd(struct sdio_dev_s *dev,
                       uint32_t cmd, uint32_t arg)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t regval;
  uint32_t cmdidx;

  sam_cmdsampleinit();

  /* Set the HSMCI Argument value */

  putreg32(arg, SAM_HSMCI_ARGR);

  /* Construct the command valid, starting with the command index */

  cmdidx = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval  = cmdidx << HSMCI_CMDR_CMDNB_SHIFT;

  /* 'OR' in response related bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    /* No response */

    case MMCSD_NO_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_INTS;
      regval |= HSMCI_CMDR_RSPTYP_NONE;

      break;

    /* 48-bit response with CRC */

    case MMCSD_R1_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_INTS;
      regval |= HSMCI_CMDR_RSPTYP_48BIT | HSMCI_CMDR_MAXLAT;
      break;

    case MMCSD_R1B_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_INTS;
      regval |= HSMCI_CMDR_RSPTYP_R1B | HSMCI_CMDR_MAXLAT;
      break;

    /* 48-bit response without CRC */

    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_NOCRC_INTS;
      regval |= HSMCI_CMDR_RSPTYP_48BIT | HSMCI_CMDR_MAXLAT;
      break;

    /* 136-bit response with CRC */

    case MMCSD_R2_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_INTS;
      regval |= HSMCI_CMDR_RSPTYP_136BIT | HSMCI_CMDR_MAXLAT;
      break;
    }

  /* 'OR' in data transfer related bits */

  switch (cmd & MMCSD_DATAXFR_MASK)
    {
#if 0 /* No MMC support */
    case MMCSD_RDSTREAM: /* MMC Read stream */
      regval |= HSMCI_CMDR_TRCMD_START | HSMCI_CMDR_TRTYP_STREAM |
                HSMCI_CMDR_TRDIR_READ;
      break;

    case MMCSD_WRSTREAM: /* MMC Write stream */
      regval |= HSMCI_CMDR_TRCMD_START | HSMCI_CMDR_TRTYP_STREAM |
                HSMCI_CMDR_TRDIR_WRITE;
      break;
#endif

    case MMCSD_RDDATAXFR: /* Read block transfer */
      regval |= HSMCI_CMDR_TRCMD_START | HSMCI_CMDR_TRDIR_READ;
      regval |= (cmd & MMCSD_MULTIBLOCK) ?
                HSMCI_CMDR_TRTYP_MULTI : HSMCI_CMDR_TRTYP_SINGLE;
      break;

    case MMCSD_WRDATAXFR: /* Write block transfer */
      regval |= HSMCI_CMDR_TRCMD_START | HSMCI_CMDR_TRDIR_WRITE;
      regval |= (cmd & MMCSD_MULTIBLOCK) ?
                HSMCI_CMDR_TRTYP_MULTI : HSMCI_CMDR_TRTYP_SINGLE;
      break;

    case MMCSD_NODATAXFR:
    default:
      if ((cmd & MMCSD_STOPXFR) != 0)
        {
          regval |= HSMCI_CMDR_TRCMD_STOP;
        }
      break;
    }

  /* 'OR' in Open Drain option */

#if 0 /* No MMC support */
  if ((cmd & MMCSD_OPENDRAIN) != 0)
    {
      regval |= HSMCI_CMDR_OPDCMD;
    }
#endif

  /* Write the fully decorated command to CMDR */

  mcinfo("cmd: %08" PRIx32 " arg: %08" PRIx32 " regval: %08" PRIx32 "\n",
         cmd, arg, regval);
  putreg32(regval, SAM_HSMCI_CMDR);
  sam_cmdsample1(SAMPLENDX_AFTER_CMDR);

  /* Card initialisation is unsuccessful without the following delay.
   *
   * It appears the timing from writing SAM_HSMCI_CMDR to calling
   * sam_waitresponse is too short.
   *
   * For now the simplest solution is to add this delay.
   * Further investigation is required to find the root cause and
   * correct solution.
   */

  nxsig_usleep(10);

  return OK;
}

/****************************************************************************
 * Name: sam_blocksetup
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

static void sam_blocksetup(struct sdio_dev_s *dev, unsigned int blocklen,
                           unsigned int nblocks)
{
  uint32_t regval;

  DEBUGASSERT(dev != NULL && nblocks > 0 && nblocks < 65535 &&
              blocklen < 65535);

  /* When TRTYP - Single or Multi, blocklen must be 1-511, 0-512 */

  DEBUGASSERT(blocklen <= 512);

  /* Set the block size. Clear bits followed by set */

  regval = getreg32(SAM_HSMCI_MR);

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X)
  regval &= ~(HSMCI_MR_RDPROOF | HSMCI_MR_WRPROOF | HSMCI_MR_BLKLEN_MASK);
  regval |= HSMCU_PROOF_BITS;
  regval |= (blocklen << HSMCI_MR_BLKLEN_SHIFT);
#else
  regval &= ~(HSMCI_MR_RDPROOF | HSMCI_MR_WRPROOF);
  regval |= HSMCU_PROOF_BITS;
#endif

  putreg32(regval, SAM_HSMCI_MR);

  /* Set the block count register */

  regval  = HSMCI_BLKR_BLKLEN(blocklen) | HSMCI_BLKR_BCNT(nblocks);
  putreg32(regval, SAM_HSMCI_BLKR);
}

/****************************************************************************
 * Name: sam_cancel
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

static int sam_cancel(struct sdio_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  /* Disable all transfer- and event- related interrupts */

  sam_disablexfrints(priv);
  sam_disablewaitints(priv, 0);

  /* No data transfer */

  sam_notransfer(priv);

  /* Clearing (most) pending interrupt status by reading the status
   * register.
   */

  getreg32(SAM_HSMCI_SR);

  /* Cancel any watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Make sure that the DMA is stopped (it will be stopped automatically
   * on normal transfers, but not necessarily when the transfer terminates
   * on an error condition.
   */

#ifdef CONFIG_SAM34_DMAC0
  sam_dmastop(priv->dma);
  priv->dmabusy = false;
#endif

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X)
  /* Disable the DMA handshaking */

  putreg32(0, SAM_HSMCI_DMA);
#endif

#ifdef CONFIG_SAM34_PDCA
  putreg32(PDC_PTCR_RXTDIS | PDC_PTCR_TXTDIS, SAM_HSMCI_PDC_PTCR);
#endif

  return OK;
}

/****************************************************************************
 * Name: sam_waitresponse
 *
 * Description:
 *   Poll-wait for the response to the last command to be ready.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command that was sent.  See 32-bit command definitions above.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int sam_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t sr;
  uint32_t pending;
  int32_t  timeout;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R6_RESPONSE:
      timeout = HSMCI_LONGTIMEOUT;
      break;

    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
      return -ENOSYS;

    case MMCSD_NO_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      timeout = HSMCI_CMDTIMEOUT;
      break;

    default:
      return -EINVAL;
    }

  /* Then wait for the response (or timeout) */

  for (; ; )
    {
      /* Did a Command-Response sequence termination event occur? */

      sr      = getreg32(SAM_HSMCI_SR);
      pending = sr & priv->cmdrmask;

      if (pending != 0)
        {
          sam_cmdsample2(SAMPLENDX_AT_WAKEUP, sr);
          sam_cmddump();

          /* Yes.. Did the Command-Response sequence end with an error? */

          if ((pending & HSMCI_RESPONSE_ERRORS) != 0)
            {
              /* Yes.. Was the error some kind of timeout? */

              mcerr("ERROR: cmd: %08" PRIx32 " events: %08" PRIx32
                    " SR: %08" PRIx32 "\n",
                    cmd, priv->cmdrmask, sr);

              if ((pending & HSMCI_RESPONSE_TIMEOUT_ERRORS) != 0)
                {
                  /* Yes.. return a timeout error */

                  priv->wkupevent = SDIOWAIT_CMDDONE |
                                    SDIOWAIT_RESPONSEDONE |
                                    SDIOWAIT_TIMEOUT;
                  return -ETIMEDOUT;
                }
              else
                {
                  /* No.. return some generic I/O error */

                  priv->wkupevent = SDIOWAIT_CMDDONE |
                                    SDIOWAIT_RESPONSEDONE |
                                    SDIOWAIT_ERROR;
                  return -EIO;
                }
            }
          else
            {
              /* The Command-Response sequence ended with no error */

              priv->wkupevent = SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE;
              return OK;
            }
       }
      else if (--timeout <= 0)
        {
          mcerr("ERROR: Timeout cmd: %08" PRIx32 " events: %08" PRIx32
                " SR: %08" PRIx32 "\n",
                cmd, priv->cmdrmask, sr);

          priv->wkupevent = SDIOWAIT_TIMEOUT;
          return -ETIMEDOUT;
        }
    }
}

/****************************************************************************
 * Name: sam_recv*
 *
 * Description:
 *   Receive response to SDIO command.  Only the critical payload is
 *   returned -- that is 32 bits for 48 bit status and 128 bits for 136 bit
 *   status.  The driver implementation should verify the correctness of
 *   the remaining, non-returned bits (CRCs, CMD index, etc.).
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *   Rx  - Buffer in which to receive the response
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure.  Here a
 *   failure means only a failure to obtain the requested response (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intact and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int sam_recvshort(struct sdio_dev_s *dev,
                         uint32_t cmd, uint32_t *rshort)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  int ret = OK;

  /* These responses could have CRC errors:
   *
   * R1  Command response (48-bit)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Command index (0-63)
   *     39:8      bit31  - bit0   32-bit card status
   *     7:1       bit6   - bit0   CRC7
   *     0         1               End bit
   *
   * R1b Identical to R1 with the additional busy signaling via the data
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
   *
   * But there is no parity on the R3 response and parity errors should
   * be ignored.
   *
   * R3  OCR (48-bit)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Reserved
   *     39:8      bit31  - bit0   32-bit OCR register
   *     7:1       bit6   - bit0   Reserved
   *     0         1               End bit
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
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R6_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif

  /* Check for timeout errors */

  if ((priv->wkupevent & SDIOWAIT_TIMEOUT) != 0)
    {
      ret = -EINVAL;
    }

  /* Check for other errors */

  else if ((priv->wkupevent & SDIOWAIT_ERROR) != 0)
    {
      ret = -EIO;
    }

  /* Return the R1/R6 response */

  else if (rshort)
    {
      *rshort = getreg32(SAM_HSMCI_RSPR0);
    }

  priv->wkupevent = 0;
  return ret;
}

static int sam_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                        uint32_t rlong[4])
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  int ret = OK;

  /* R2  CID, CSD register (136-bit)
   *     135       0               Start bit
   *     134       0               Transmission bit (0=from card)
   *     133:128   bit5   - bit0   Reserved
   *     127:1     bit127 - bit1   127-bit CID or CSD register
   *                               (including internal CRC)
   *     0         1               End bit
   */

#ifdef CONFIG_DEBUG_FEATURES
  /* Check that R1 is the correct response to this command */

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R2_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif

  /* Check for timeout errors */

  if ((priv->wkupevent & SDIOWAIT_TIMEOUT) != 0)
    {
      ret = -EINVAL;
    }

  /* Check for other errors */

  else if ((priv->wkupevent & SDIOWAIT_ERROR) != 0)
    {
      ret = -EIO;
    }

  /* Return the long response */

  else if (rlong)
    {
      rlong[0] = getreg32(SAM_HSMCI_RSPR0);
      rlong[1] = getreg32(SAM_HSMCI_RSPR1);
      rlong[2] = getreg32(SAM_HSMCI_RSPR2);
      rlong[3] = getreg32(SAM_HSMCI_RSPR3);
    }

  priv->wkupevent = 0;
  return ret;
}

/* MMC responses not supported */

static int sam_recvnotimpl(struct sdio_dev_s *dev,
                           uint32_t cmd, uint32_t *rnotimpl)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  priv->wkupevent = 0;
  return -ENOSYS;
}

/****************************************************************************
 * Name: sam_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the SDIO_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling either calling SDIO_DMARECVSETUP,
 *   SDIO_DMASENDSETUP, or or SDIO_WAITEVENT.  This is the recommended
 *   ordering:
 *
 *     SDIO_WAITENABLE:    Discard any pending interrupts, enable event(s)
 *                         of interest
 *     SDIO_DMARECVSETUP/
 *     SDIO_DMASENDSETUP:  Setup the logic that will trigger the event the
 *                         event(s) of interest
 *     SDIO_WAITEVENT:     Wait for the event of interest (which might
 *                         already have occurred)
 *
 *   This sequence should eliminate race conditions between the command/
 *   transfer setup and the subsequent events.
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

static void sam_waitenable(struct sdio_dev_s *dev,
                           sdio_eventset_t eventset, uint32_t timeout)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t waitmask;

  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  sam_disablewaitints(priv, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

  waitmask = 0;
  if ((eventset & (SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE)) != 0)
    {
      waitmask |= priv->cmdrmask;
    }

  /* Clear (most) pending interrupts by reading the status register.
   * No interrupts should be lost (assuming that interrupts were enabled
   * before sam_waitenable() was called).  Any interrupts that become
   * pending after this point must be valid event indications.
   */

  getreg32(SAM_HSMCI_SR);

  /* Wait interrupts are configured here, but not enabled until
   * sam_eventwait() is called.  Why?  Because the XFRDONE interrupt is
   * always pending until start the data transfer.
   */

  sam_configwaitints(priv, waitmask, eventset);

  /* Check if the timeout event is specified in the event set */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      int delay;
      int ret;

      /* Yes.. Handle a cornercase */

      if (!timeout)
        {
          priv->wkupevent = SDIOWAIT_TIMEOUT;
          return;
        }

      /* Start the watchdog timer */

      delay = MSEC2TICK(timeout);
      ret   = wd_start(&priv->waitwdog, delay,
                       sam_eventtimeout, (wdparm_t)priv);
      if (ret < 0)
        {
          mcerr("ERROR: wd_start failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: sam_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDIO_WAITEVENTS are disabled when sam_eventwait
 *   returns.  SDIO_WAITEVENTS must be called again before sam_eventwait
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

static sdio_eventset_t sam_eventwait(struct sdio_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  sdio_eventset_t wkupevent = 0;
  int ret;

  /* Since interrupts not been enabled to this point, any relevant events
   * are pending and should not yet have occurred.
   */

  DEBUGASSERT(priv->waitevents != 0 && priv->wkupevent == 0);

  /* Now enable event-related interrupts. If the events are pending, they
   * may happen immediately here before entering the loop.
   */

  sam_enableints(priv);

  /* Loop until the event (or the timeout occurs). Race conditions are
   * avoided by calling sam_waitenable prior to triggering the logic that
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
          /* Task canceled.  Cancel the wdog (assuming it was started),
           * disable all event, and return an SDIO error.
           */

          wd_cancel(&priv->waitwdog);
          sam_disablexfrints(priv);
          sam_disablewaitints(priv, SDIOWAIT_ERROR);
          return SDIOWAIT_ERROR;
        }

      wkupevent = priv->wkupevent;

      /* Check if the event has occurred.  When the event has occurred, then
       * evenset will be set to 0 and wkupevent will be set to a nonzero
       * value.  When wkupevent becomes non-zero, further interrupts will
       * have already been disabled.
       */

      if (wkupevent != 0)
        {
          /* Yes... break out of the loop with wkupevent non-zero */

          break;
        }
    }

  sam_cmddump();
  sam_xfrdump(priv);
  return wkupevent;
}

/****************************************************************************
 * Name: sam_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events.  This is part of the
 *   the SDIO callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in sam_registercallback.
 *
 *   Events are automatically disabled once the callback is performed and no
 *   further callback events will occur until they are again enabled by
 *   calling this methods.
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

static void sam_callbackenable(struct sdio_dev_s *dev,
                               sdio_eventset_t eventset)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  mcinfo("eventset: %02x\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  sam_callback(priv);
}

/****************************************************************************
 * Name: sam_registercallback
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change.  Callbacks should not be made from interrupt handlers, rather
 *   interrupt level events should be handled by calling back on the work
 *   thread.
 *
 *   When this method is called, all callbacks should be disabled until they
 *   are enabled via a call to SDIO_CALLBACKENABLE.
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

static int sam_registercallback(struct sdio_dev_s *dev,
                                worker_t callback, void *arg)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  /* Disable callbacks and register this callback and is argument */

  mcinfo("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: sam_dmarecvsetup
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

static int sam_dmarecvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                          size_t buflen)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Setup register sampling */

  sam_xfrsampleinit();
  sam_xfrsample(priv, SAMPLENDX_BEFORE_SETUP);

#ifdef CONFIG_SAM34_DMAC0
  /* Configure the RX DMA */

  sam_dmarxsetup(priv->dma, SAM_HSMCI_RDR, (uint32_t)buffer, buflen);

  /* Invalidate the buffer memory to force re-fetching from RAM when the DMA
   * completes
   */

  sam_cmcc_invalidate((uintptr_t)buffer, (uintptr_t)buffer + buflen);

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X)
  /* Enable DMA handshaking */

  putreg32(HSMCI_DMA_DMAEN, SAM_HSMCI_DMA);
#endif
  sam_xfrsample(priv, SAMPLENDX_BEFORE_ENABLE);

  /* Start the DMA */

  priv->dmabusy = true;
  sam_dmastart(priv->dma, sam_dmacallback, priv);
#endif

#ifdef CONFIG_SAM34_PDCA
  modifyreg32(SAM_HSMCI_MR, 0, HSMCI_MR_PDCMODE);
  mcinfo("SAM_HSMCI_MR = 0x%08" PRIX32 "\n", getreg32(SAM_HSMCI_MR));

  putreg32((uint32_t)buffer, SAM_HSMCI_PDC_RPR);
  putreg32(buflen / 4, SAM_HSMCI_PDC_RCR);
  putreg32(PDC_PTCR_RXTEN, SAM_HSMCI_PDC_PTCR);
  sam_xfrsample(priv, SAMPLENDX_BEFORE_ENABLE);
#endif

  /* Configure transfer-related interrupts.  Transfer interrupts are not
   * enabled until after the transfer is start with an SD command (i.e.,
   * at the beginning of sam_eventwait().
   */

  sam_xfrsample(priv, SAMPLENDX_AFTER_SETUP);
  sam_configxfrints(priv, HSMCI_DMARECV_INTS);
  return OK;
}

/****************************************************************************
 * Name: sam_dmasendsetup
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

static int sam_dmasendsetup(struct sdio_dev_s *dev,
                            const uint8_t *buffer, size_t buflen)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Setup register sampling */

  sam_xfrsampleinit();
  sam_xfrsample(priv, SAMPLENDX_BEFORE_SETUP);

#ifdef CONFIG_SAM34_DMAC0
  /* Configure the TX DMA */

  sam_dmatxsetup(priv->dma, SAM_HSMCI_TDR, (uint32_t)buffer, buflen);

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X)
  /* Enable DMA handshaking */

  putreg32(HSMCI_DMA_DMAEN, SAM_HSMCI_DMA);
#endif
  sam_xfrsample(priv, SAMPLENDX_BEFORE_ENABLE);

  /* Start the DMA */

  priv->dmabusy = true;
  sam_dmastart(priv->dma, sam_dmacallback, priv);
#endif

#ifdef CONFIG_SAM34_PDCA
  modifyreg32(SAM_HSMCI_MR, 0, HSMCI_MR_PDCMODE);
  mcinfo("SAM_HSMCI_MR = 0x%08" PRIX32 "\n", getreg32(SAM_HSMCI_MR));

  putreg32((uint32_t)buffer, SAM_HSMCI_PDC_TPR);
  putreg32(buflen / 4, SAM_HSMCI_PDC_TCR);
  putreg32(PDC_PTCR_TXTEN, SAM_HSMCI_PDC_PTCR);
  sam_xfrsample(priv, SAMPLENDX_BEFORE_ENABLE);
#endif

  /* Configure transfer-related interrupts.  Transfer interrupts are not
   * enabled until after the transfer is start with an SD command (i.e.,
   * at the beginning of sam_eventwait().
   */

  sam_xfrsample(priv, SAMPLENDX_AFTER_SETUP);
  sam_configxfrints(priv, HSMCI_DMASEND_INTS);
  return OK;
}

/****************************************************************************
 * Name: sam_callback
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

static void sam_callback(void *arg)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)arg;

  /* Is a callback registered? */

  DEBUGASSERT(priv != NULL);
  mcinfo("Callback %p(%p) cbevents: %02x cdstatus: %02x\n",
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

          work_queue(LPWORK, &priv->cbwork,
                    priv->callback, priv->cbarg, 0);
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
 *   Initialize SD for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Value:
 *   A reference to an SDIO interface structure.  NULL is returned on
 *   failures.
 *
 ****************************************************************************/

struct sdio_dev_s *sdio_initialize(int slotno)
{
  /* There is only one slot */

  struct sam_dev_s *priv = &g_sdiodev;

  mcinfo("slotno: %d\n", slotno);

  /* Initialize the HSMCI slot structure */

  /* Initialize semaphores */

  nxsem_init(&priv->waitsem, 0, 0);
#ifdef CONFIG_SAM34_DMAC0
  /* Allocate a DMA channel.  A FIFO size of 8 is sufficient. */

  priv->dma = sam_dmachannel(DMA_FLAGS);
  DEBUGASSERT(priv->dma);
#endif

  /* Configure GPIOs for 4-bit, wide-bus operation.  NOTE: (1) the chip is
   * capable of 8-bit wide bus operation but D4-D7 are not configured, (2)
   * any card detection GPIOs must be set up in board-specific logic.
   */

#if defined(CONFIG_ARCH_CHIP_SAM3X)
  sam_configgpio(GPIO_HSMCIA_DAT0);   /* Data 0 of Slot A - PA21 */
  sam_configgpio(GPIO_HSMCIA_DAT1);   /* Data 1 of Slot A - PA22 */
  sam_configgpio(GPIO_HSMCIA_DAT2);   /* Data 2 of Slot A - PA23 */
  sam_configgpio(GPIO_HSMCIA_DAT3);   /* Data 3 of Slot A - PA24 */
  sam_configgpio(GPIO_HSMCI_CK);      /* SD clock - PA19 */
  sam_configgpio(GPIO_HSMCIA_CD);     /* Command/Response - PA20 */

#ifdef CONFIG_DEBUG_FS
  sam_dumpgpio(GPIO_PORT_PIOA, "Pins: 19-24");
#endif
#else
  sam_configgpio(GPIO_HSMCI_DAT0);   /* Data 0 of Slot A */
  sam_configgpio(GPIO_HSMCI_DAT1);   /* Data 1 of Slot A */
  sam_configgpio(GPIO_HSMCI_DAT2);   /* Data 2 of Slot A */
  sam_configgpio(GPIO_HSMCI_DAT3);   /* Data 3 of Slot A */
  sam_configgpio(GPIO_HSMCI_CK);     /* SD clock */
  sam_configgpio(GPIO_HSMCI_DA);     /* Command/Response */

#ifdef CONFIG_DEBUG_FS
  sam_dumpgpio(GPIO_PORT_PIOA, "Pins: 3-8");
  sam_dumpgpio(GPIO_PORT_PIOB, "Pins: 28-31");
#endif
#endif

  /* Reset the card and assure that it is in the initial, unconfigured
   * state.
   */

  sam_reset(&priv->dev);
  return &g_sdiodev.dev;
}

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic -- possibly from an interrupt handler --
 *   in order to signal to the driver that a card has been inserted or
 *   removed from the slot.
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
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
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

  mcinfo("cdstatus OLD: %02x NEW: %02x\n", cdstatus, priv->cdstatus);

  /* Perform any requested callback if the status has changed */

  if (cdstatus != priv->cdstatus)
    {
      sam_callback(priv);
    }

  leave_critical_section(flags);
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
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
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
#endif /* CONFIG_SAM34_HSMCI */
