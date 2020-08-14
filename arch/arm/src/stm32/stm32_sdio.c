/****************************************************************************
 * arch/arm/src/stm32/stm32_sdio.c
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
#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"

#include "stm32.h"
#include "stm32_dma.h"
#include "stm32_sdio.h"

#ifdef CONFIG_STM32_SDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Required system configuration options:
 *
 *   CONFIG_ARCH_DMA - Enable architecture-specific DMA subsystem
 *     initialization.  Required if CONFIG_STM32_SDIO_DMA is enabled.
 *   CONFIG_STM32_DMA2 - Enable STM32 DMA2 support.  Required if
 *     CONFIG_STM32_SDIO_DMA is enabled
 *   CONFIG_SCHED_WORKQUEUE -- Callback support requires work queue support.
 *
 * Driver-specific configuration options:
 *
 *   CONFIG_SDIO_MUXBUS - Setting this configuration enables some locking
 *     APIs to manage concurrent accesses on the SDIO bus.  This is not
 *     needed for the simple case of a single SD card, for example.
 *   CONFIG_STM32_SDIO_DMA - Enable SDIO.  This is a marginally optional.
 *     For most usages, SDIO will cause data overruns if used without DMA.
 *     NOTE the above system DMA configuration options.
 *   CONFIG_STM32_SDIO_WIDTH_D1_ONLY - This may be selected to force the
 *     driver operate with only a single data line (the default is to use
 *     all 4 SD data lines).
 *   CONFIG_SDM_DMAPRIO - SDIO DMA priority.  This can be selected if
 *     CONFIG_STM32_SDIO_DMA is enabled.
 *   CONFIG_SDIO_XFRDEBUG - Enables some very low-level debug output
 *     This also requires CONFIG_DEBUG_FS and CONFIG_DEBUG_INFO
 */

#if !defined(CONFIG_STM32_SDIO_DMA)
#  warning "Large Non-DMA transfer may result in RX overrun failures"
#else
#  ifndef CONFIG_STM32_DMA2
#    error "CONFIG_STM32_SDIO_DMA support requires CONFIG_STM32_DMA2"
#  endif
#  ifndef CONFIG_SDIO_DMA
#    error CONFIG_SDIO_DMA must be defined with CONFIG_STM32_SDIO_DMA
#  endif
#endif

#ifndef CONFIG_STM32_SDIO_DMA
#  warning "Large Non-DMA transfer may result in RX overrun failures"
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE"
#endif

#ifdef CONFIG_STM32_SDIO_DMA
#  ifndef CONFIG_STM32_SDIO_DMAPRIO
#    if defined(CONFIG_STM32_STM32F10XX)
#      define CONFIG_STM32_SDIO_DMAPRIO  DMA_CCR_PRIMED
#    elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#      define CONFIG_STM32_SDIO_DMAPRIO  DMA_SCR_PRIVERYHI
#    else
#      error "Unknown STM32 DMA"
#    endif
#  endif
#  if defined(CONFIG_STM32_STM32F10XX)
#    if (CONFIG_STM32_SDIO_DMAPRIO & ~DMA_CCR_PL_MASK) != 0
#      error "Illegal value for CONFIG_STM32_SDIO_DMAPRIO"
#    endif
#  elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#    if (CONFIG_STM32_SDIO_DMAPRIO & ~DMA_SCR_PL_MASK) != 0
#      error "Illegal value for CONFIG_STM32_SDIO_DMAPRIO"
#    endif
#  else
#    error "Unknown STM32 DMA"
#  endif
#else
#  undef CONFIG_STM32_SDIO_DMAPRIO
#endif

#ifndef CONFIG_DEBUG_MEMCARD_INFO
#  undef CONFIG_SDIO_XFRDEBUG
#endif

/* Enable the SDIO pull-up resistors if needed */

#ifdef CONFIG_STM32_SDIO_PULLUP
#  define SDIO_PULLUP_ENABLE GPIO_PULLUP
#else
#  define SDIO_PULLUP_ENABLE 0
#endif

/* Friendly CLKCR bit re-definitions ****************************************/

#define SDIO_CLKCR_RISINGEDGE    (0)
#define SDIO_CLKCR_FALLINGEDGE   SDIO_CLKCR_NEGEDGE

/* Use the default of the rising edge but allow a configuration,
 * that does not have the errata, to override the edge the SDIO
 * command and data is changed on.
 */

#if !defined(SDIO_CLKCR_EDGE)
#  define SDIO_CLKCR_EDGE SDIO_CLKCR_RISINGEDGE
#endif

/* Mode dependent settings.  These depend on clock devisor settings that must
 * be defined in the board-specific board.h header file: SDIO_INIT_CLKDIV,
 * SDIO_MMCXFR_CLKDIV, and SDIO_SDXFR_CLKDIV.
 */

#define STM32_CLCKCR_INIT        (SDIO_INIT_CLKDIV | SDIO_CLKCR_EDGE | \
                                  SDIO_CLKCR_WIDBUS_D1)
#define SDIO_CLKCR_MMCXFR        (SDIO_MMCXFR_CLKDIV | SDIO_CLKCR_EDGE | \
                                  SDIO_CLKCR_WIDBUS_D1)
#define SDIO_CLCKR_SDXFR         (SDIO_SDXFR_CLKDIV | SDIO_CLKCR_EDGE | \
                                  SDIO_CLKCR_WIDBUS_D1)
#define SDIO_CLCKR_SDWIDEXFR     (SDIO_SDXFR_CLKDIV | SDIO_CLKCR_EDGE | \
                                  SDIO_CLKCR_WIDBUS_D4)

/* Timing */

#define SDIO_CMDTIMEOUT          (100000)
#define SDIO_LONGTIMEOUT         (0x7fffffff)

/* Big DTIMER setting */

#define SDIO_DTIMER_DATATIMEOUT  (0x000fffff)

/* DMA channel/stream configuration register settings.  The following
 * must be selected.  The DMA driver will select the remaining fields.
 *
 * - 32-bit DMA
 * - Memory increment
 * - Direction (memory-to-peripheral, peripheral-to-memory)
 * - Memory burst size (F4 only)
 */

/* STM32 F1 channel configuration register (CCR) settings */

#if defined(CONFIG_STM32_STM32F10XX)
#  define SDIO_RXDMA32_CONFIG    (CONFIG_STM32_SDIO_DMAPRIO | DMA_CCR_MSIZE_32BITS | \
                                  DMA_CCR_PSIZE_32BITS | DMA_CCR_MINC)
#  define SDIO_TXDMA32_CONFIG    (CONFIG_STM32_SDIO_DMAPRIO | DMA_CCR_MSIZE_32BITS | \
                                  DMA_CCR_PSIZE_32BITS | DMA_CCR_MINC | DMA_CCR_DIR)

/* STM32 F4 stream configuration register (SCR) settings. */

#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#  define SDIO_RXDMA32_CONFIG    (DMA_SCR_PFCTRL | DMA_SCR_DIR_P2M|DMA_SCR_MINC | \
                                  DMA_SCR_PSIZE_32BITS | DMA_SCR_MSIZE_32BITS | \
                                  CONFIG_STM32_SDIO_DMAPRIO | DMA_SCR_PBURST_INCR4 | \
                                  DMA_SCR_MBURST_INCR4)
#  define SDIO_TXDMA32_CONFIG    (DMA_SCR_PFCTRL | DMA_SCR_DIR_M2P | DMA_SCR_MINC | \
                                  DMA_SCR_PSIZE_32BITS | DMA_SCR_MSIZE_32BITS | \
                                  CONFIG_STM32_SDIO_DMAPRIO | DMA_SCR_PBURST_INCR4 | \
                                  DMA_SCR_MBURST_INCR4)
#else
#  error "Unknown STM32 DMA"
#endif

/* SDIO DMA Channel/Stream selection.  For the case of the STM32 F4, there
 * are multiple DMA stream options that must be dis-ambiguated in the board.h
 * file.
 */

#if defined(CONFIG_STM32_STM32F10XX)
#  define SDIO_DMACHAN           DMACHAN_SDIO
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#  define SDIO_DMACHAN           DMAMAP_SDIO
#else
#  error "Unknown STM32 DMA"
#endif

/* FIFO sizes */

#define SDIO_HALFFIFO_WORDS      (8)
#define SDIO_HALFFIFO_BYTES      (8*4)

/* Data transfer interrupt mask bits */

#define SDIO_RECV_MASK     (SDIO_MASK_DCRCFAILIE | SDIO_MASK_DTIMEOUTIE | \
                            SDIO_MASK_DATAENDIE | SDIO_MASK_RXOVERRIE | \
                            SDIO_MASK_RXFIFOHFIE | SDIO_MASK_STBITERRIE)
#define SDIO_SEND_MASK     (SDIO_MASK_DCRCFAILIE | SDIO_MASK_DTIMEOUTIE | \
                            SDIO_MASK_DATAENDIE | SDIO_MASK_TXUNDERRIE | \
                            SDIO_MASK_TXFIFOHEIE | SDIO_MASK_STBITERRIE)
#define SDIO_DMARECV_MASK  (SDIO_MASK_DCRCFAILIE | SDIO_MASK_DTIMEOUTIE | \
                            SDIO_MASK_DATAENDIE | SDIO_MASK_RXOVERRIE | \
                            SDIO_MASK_STBITERRIE)
#define SDIO_DMASEND_MASK  (SDIO_MASK_DCRCFAILIE | SDIO_MASK_DTIMEOUTIE | \
                            SDIO_MASK_DATAENDIE | SDIO_MASK_TXUNDERRIE | \
                            SDIO_MASK_STBITERRIE)

/* Event waiting interrupt mask bits */

#define SDIO_CMDDONE_STA   (SDIO_STA_CMDSENT)
#define SDIO_RESPDONE_STA  (SDIO_STA_CTIMEOUT | SDIO_STA_CCRCFAIL | \
                            SDIO_STA_CMDREND)
#define SDIO_XFRDONE_STA   (0)

#define SDIO_CMDDONE_MASK  (SDIO_MASK_CMDSENTIE)
#define SDIO_RESPDONE_MASK (SDIO_MASK_CCRCFAILIE | SDIO_MASK_CTIMEOUTIE | \
                            SDIO_MASK_CMDRENDIE)
#define SDIO_XFRDONE_MASK  (0)

#define SDIO_CMDDONE_ICR   (SDIO_ICR_CMDSENTC | SDIO_ICR_DBCKENDC)
#define SDIO_RESPDONE_ICR  (SDIO_ICR_CTIMEOUTC | SDIO_ICR_CCRCFAILC | \
                            SDIO_ICR_CMDRENDC | SDIO_ICR_DBCKENDC)
#define SDIO_XFRDONE_ICR   (SDIO_ICR_DATAENDC | SDIO_ICR_DCRCFAILC | \
                            SDIO_ICR_DTIMEOUTC | SDIO_ICR_RXOVERRC | \
                            SDIO_ICR_TXUNDERRC | SDIO_ICR_STBITERRC | \
                            SDIO_ICR_DBCKENDC)

#define SDIO_WAITALL_ICR   (SDIO_CMDDONE_ICR | SDIO_RESPDONE_ICR | \
                            SDIO_XFRDONE_ICR | SDIO_ICR_DBCKENDC)

/* Let's wait until we have both SDIO transfer complete and DMA complete. */

#define SDIO_XFRDONE_FLAG  (1)
#define SDIO_DMADONE_FLAG  (2)
#define SDIO_ALLDONE       (3)

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
#  ifdef CONFIG_STM32_SDIO_DMA
#    define SAMPLENDX_BEFORE_SETUP  0
#    define SAMPLENDX_BEFORE_ENABLE 1
#    define SAMPLENDX_AFTER_SETUP   2
#    define SAMPLENDX_END_TRANSFER  3
#    define SAMPLENDX_DMA_CALLBACK  4
#    define DEBUG_NSAMPLES          5
#  else
#    define SAMPLENDX_BEFORE_SETUP  0
#    define SAMPLENDX_AFTER_SETUP   1
#    define SAMPLENDX_END_TRANSFER  2
#    define DEBUG_NSAMPLES          3
#  endif
#endif

#define STM32_SDIO_USE_DEFAULT_BLOCKSIZE ((uint8_t)-1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the state of the STM32 SDIO interface */

struct stm32_dev_s
{
  struct sdio_dev_s  dev;             /* Standard, base SDIO interface */

  /* STM32-specific extensions */

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
  uint32_t           xfrmask;         /* Interrupt enables for data transfer */

#ifdef CONFIG_STM32_SDIO_CARD
  /* Interrupt at SDIO_D1 pin, only for SDIO cards */

  uint32_t           sdiointmask;            /* STM32 SDIO register mask */
  int               (*do_sdio_card)(void *); /* SDIO card ISR */
  void               *do_sdio_arg;           /* arg for SDIO card ISR */
#endif

  /* Fixed transfer block size support */

#ifdef CONFIG_SDIO_BLOCKSETUP
  uint8_t            block_size;
#endif

  /* DMA data transfer support */

  bool               widebus;         /* Required for DMA support */
#ifdef CONFIG_STM32_SDIO_DMA
  volatile uint8_t   xfrflags;        /* Used to synchronize SDIO and
                                       * DMA completion events */
  bool               dmamode;         /* true: DMA mode transfer */
  DMA_HANDLE         dma;             /* Handle for DMA channel */
#endif
};

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
struct stm32_sdioregs_s
{
  uint8_t  power;
  uint16_t clkcr;
  uint16_t dctrl;
  uint32_t dtimer;
  uint32_t dlen;
  uint32_t dcount;
  uint32_t sta;
  uint32_t mask;
  uint32_t fifocnt;
};

struct stm32_sampleregs_s
{
  struct stm32_sdioregs_s sdio;
#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_STM32_SDIO_DMA)
  struct stm32_dmaregs_s  dma;
#endif
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static int  stm32_takesem(struct stm32_dev_s *priv);
#define     stm32_givesem(priv) (nxsem_post(&priv->waitsem))
static inline void stm32_setclkcr(uint32_t clkcr);
static void stm32_configwaitints(struct stm32_dev_s *priv, uint32_t waitmask,
              sdio_eventset_t waitevents, sdio_eventset_t wkupevents);
static void stm32_configxfrints(struct stm32_dev_s *priv, uint32_t xfrmask);
static void stm32_setpwrctrl(uint32_t pwrctrl);
static inline uint32_t stm32_getpwrctrl(void);

/* DMA Helpers **************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void stm32_sampleinit(void);
static void stm32_sdiosample(struct stm32_sdioregs_s *regs);
static void stm32_sample(struct stm32_dev_s *priv, int index);
static void stm32_sdiodump(struct stm32_sdioregs_s *regs, const char *msg);
static void stm32_dumpsample(struct stm32_dev_s *priv,
              struct stm32_sampleregs_s *regs, const char *msg);
static void stm32_dumpsamples(struct stm32_dev_s *priv);
#else
#  define   stm32_sampleinit()
#  define   stm32_sample(priv,index)
#  define   stm32_dumpsamples(priv)
#endif

#ifdef CONFIG_STM32_SDIO_DMA
static void stm32_dmacallback(DMA_HANDLE handle, uint8_t status, void *arg);
#endif

/* Data Transfer Helpers ****************************************************/

static uint8_t stm32_log2(uint16_t value);
static void stm32_dataconfig(uint32_t timeout, uint32_t dlen,
                             uint32_t dctrl);
static void stm32_datadisable(void);
static void stm32_sendfifo(struct stm32_dev_s *priv);
static void stm32_recvfifo(struct stm32_dev_s *priv);
static void stm32_eventtimeout(wdparm_t arg);
static void stm32_endwait(struct stm32_dev_s *priv,
                          sdio_eventset_t wkupevent);
static void stm32_endtransfer(struct stm32_dev_s *priv,
                              sdio_eventset_t wkupevent);

/* Interrupt Handling *******************************************************/

static int  stm32_interrupt(int irq, void *context, void *arg);
#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
static int  stm32_rdyinterrupt(int irq, void *context, void *arg);
#endif

/* SDIO interface methods ***************************************************/

/* Mutual exclusion */

#ifdef CONFIG_SDIO_MUXBUS
static int stm32_lock(FAR struct sdio_dev_s *dev, bool lock);
#endif

/* Initialization/setup */

static void stm32_reset(FAR struct sdio_dev_s *dev);
static sdio_capset_t stm32_capabilities(FAR struct sdio_dev_s *dev);
static sdio_statset_t stm32_status(FAR struct sdio_dev_s *dev);
static void stm32_widebus(FAR struct sdio_dev_s *dev, bool enable);
static void stm32_clock(FAR struct sdio_dev_s *dev,
              enum sdio_clock_e rate);
static int  stm32_attach(FAR struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int  stm32_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t arg);
#ifdef CONFIG_SDIO_BLOCKSETUP
static void stm32_blocksetup(FAR struct sdio_dev_s *dev,
              unsigned int blocklen, unsigned int nblocks);
#endif
static int  stm32_recvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
              size_t nbytes);
static int  stm32_sendsetup(FAR struct sdio_dev_s *dev,
              FAR const uint8_t *buffer, uint32_t nbytes);
static int  stm32_cancel(FAR struct sdio_dev_s *dev);

static int  stm32_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd);
static int  stm32_recvshortcrc(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  stm32_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t rlong[4]);
static int  stm32_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);

/* EVENT handler */

static void stm32_waitenable(FAR struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static sdio_eventset_t
            stm32_eventwait(FAR struct sdio_dev_s *dev, uint32_t timeout);
static void stm32_callbackenable(FAR struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static int  stm32_registercallback(FAR struct sdio_dev_s *dev,
              worker_t callback, void *arg);

/* DMA */

#ifdef CONFIG_STM32_SDIO_DMA
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
static int  stm32_dmapreflight(FAR struct sdio_dev_s *dev,
              FAR const uint8_t *buffer, size_t buflen);
#endif
static int  stm32_dmarecvsetup(FAR struct sdio_dev_s *dev,
              FAR uint8_t *buffer, size_t buflen);
static int  stm32_dmasendsetup(FAR struct sdio_dev_s *dev,
              FAR const uint8_t *buffer, size_t buflen);
#endif

/* Initialization/uninitialization/reset ************************************/

static void stm32_callback(void *arg);
static void stm32_default(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct stm32_dev_s g_sdiodev =
{
  .dev =
  {
#ifdef CONFIG_SDIO_MUXBUS
    .lock             = stm32_lock,
#endif
    .reset            = stm32_reset,
    .capabilities     = stm32_capabilities,
    .status           = stm32_status,
    .widebus          = stm32_widebus,
    .clock            = stm32_clock,
    .attach           = stm32_attach,
    .sendcmd          = stm32_sendcmd,
#ifdef CONFIG_SDIO_BLOCKSETUP
    .blocksetup       = stm32_blocksetup,
#endif
    .recvsetup        = stm32_recvsetup,
    .sendsetup        = stm32_sendsetup,
    .cancel           = stm32_cancel,
    .waitresponse     = stm32_waitresponse,
    .recv_r1          = stm32_recvshortcrc,
    .recv_r2          = stm32_recvlong,
    .recv_r3          = stm32_recvshort,
    .recv_r4          = stm32_recvshort,
    .recv_r5          = stm32_recvshortcrc,
    .recv_r6          = stm32_recvshortcrc,
    .recv_r7          = stm32_recvshort,
    .waitenable       = stm32_waitenable,
    .eventwait        = stm32_eventwait,
    .callbackenable   = stm32_callbackenable,
    .registercallback = stm32_registercallback,
#ifdef CONFIG_SDIO_DMA
#ifdef CONFIG_STM32_SDIO_DMA
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
    .dmapreflight     = stm32_dmapreflight,
#endif
    .dmarecvsetup     = stm32_dmarecvsetup,
    .dmasendsetup     = stm32_dmasendsetup,
#else
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
    .dmapreflight     = NULL,
#endif
    .dmarecvsetup     = stm32_recvsetup,
    .dmasendsetup     = stm32_sendsetup,
#endif
#endif
  },
};

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
static struct stm32_sampleregs_s g_sampleregs[DEBUG_NSAMPLES];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_takesem
 *
 * Description:
 *   Take the wait semaphore (handling false alarm wakeups due to the receipt
 *   of signals).
 *
 * Input Parameters:
 *   dev - Instance of the SDIO device driver state structure.
 *
 * Returned Value:
 *   Normally OK, but may return -ECANCELED in the rare event that the task
 *   has been canceled.
 *
 ****************************************************************************/

static int stm32_takesem(struct stm32_dev_s *priv)
{
  return nxsem_wait_uninterruptible(&priv->waitsem);
}

/****************************************************************************
 * Name: stm32_setclkcr
 *
 * Description:
 *   Modify oft-changed bits in the CLKCR register.  Only the following bit-
 *   fields are changed:
 *
 *   CLKDIV, PWRSAV, BYPASS, WIDBUS, NEGEDGE, and HWFC_EN
 *
 * Input Parameters:
 *   clkcr - A new CLKCR setting for the above mentions bits (other bits
 *           are ignored.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_setclkcr(uint32_t clkcr)
{
  uint32_t regval = getreg32(STM32_SDIO_CLKCR);

  /* Clear CLKDIV, PWRSAV, BYPASS, WIDBUS, NEGEDGE, HWFC_EN bits */

  regval &= ~(SDIO_CLKCR_CLKDIV_MASK | SDIO_CLKCR_PWRSAV |
              SDIO_CLKCR_BYPASS | SDIO_CLKCR_WIDBUS_MASK |
              SDIO_CLKCR_NEGEDGE | SDIO_CLKCR_HWFC_EN |
              SDIO_CLKCR_CLKEN);

  /* Replace with user provided settings */

  clkcr  &=  (SDIO_CLKCR_CLKDIV_MASK | SDIO_CLKCR_PWRSAV |
              SDIO_CLKCR_BYPASS | SDIO_CLKCR_WIDBUS_MASK |
              SDIO_CLKCR_NEGEDGE | SDIO_CLKCR_HWFC_EN |
              SDIO_CLKCR_CLKEN);

  regval |=  clkcr;
  putreg32(regval, STM32_SDIO_CLKCR);

  mcinfo("CLKCR: %08x PWR: %08x\n",
        getreg32(STM32_SDIO_CLKCR), getreg32(STM32_SDIO_POWER));
}

/****************************************************************************
 * Name: stm32_configwaitints
 *
 * Description:
 *   Enable/disable SDIO interrupts needed to support the wait function
 *
 * Input Parameters:
 *   priv       - A reference to the SDIO device state structure
 *   waitmask   - The set of bits in the SDIO MASK register to set
 *   waitevents - Waited for events
 *   wkupevent  - Wake-up events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_configwaitints(struct stm32_dev_s *priv, uint32_t waitmask,
                                 sdio_eventset_t waitevents,
                                 sdio_eventset_t wkupevent)
{
  irqstate_t flags;
#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
  int pinset;
#endif

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags = enter_critical_section();

#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
  if ((waitmask & SDIOWAIT_WRCOMPLETE) != 0)
    {
      /* Do not use this in STM32_SDIO_MASK register */

      waitmask &= ~SDIOWAIT_WRCOMPLETE;

      pinset = GPIO_SDIO_D0 & (GPIO_PORT_MASK | GPIO_PIN_MASK);
      pinset |= (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI);

      /* Arm the SDIO_D Ready and install Isr */

      stm32_gpiosetevent(pinset, true, false, false,
                         stm32_rdyinterrupt, priv);
    }

  /* Disarm SDIO_D ready */

  if ((wkupevent & SDIOWAIT_WRCOMPLETE) != 0)
    {
      stm32_gpiosetevent(GPIO_SDIO_D0, false, false, false,
                         NULL, NULL);
      stm32_configgpio(GPIO_SDIO_D0);
    }
#endif

  priv->waitevents = waitevents;
  priv->wkupevent  = wkupevent;
  priv->waitmask   = waitmask;
#ifdef CONFIG_STM32_SDIO_DMA
  priv->xfrflags   = 0;
#endif

#ifdef CONFIG_STM32_SDIO_CARD
  putreg32(priv->xfrmask | priv->waitmask | priv->sdiointmask,
           STM32_SDIO_MASK);
#else
  putreg32(priv->xfrmask | priv->waitmask, STM32_SDIO_MASK);
#endif

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32_configxfrints
 *
 * Description:
 *   Enable SDIO interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the SDIO device state structure
 *   xfrmask - The set of bits in the SDIO MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_configxfrints(struct stm32_dev_s *priv, uint32_t xfrmask)
{
  irqstate_t flags;

  flags = enter_critical_section();
  priv->xfrmask = xfrmask;
#ifdef CONFIG_STM32_SDIO_CARD
  putreg32(priv->xfrmask | priv->waitmask | priv->sdiointmask,
           STM32_SDIO_MASK);
#else
  putreg32(priv->xfrmask | priv->waitmask, STM32_SDIO_MASK);
#endif
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32_setpwrctrl
 *
 * Description:
 *   Change the PWRCTRL field of the SDIO POWER register to turn the SDIO
 *   ON or OFF
 *
 * Input Parameters:
 *   clkcr - A new PWRCTRL setting
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_setpwrctrl(uint32_t pwrctrl)
{
  uint32_t regval;

  regval  = getreg32(STM32_SDIO_POWER);
  regval &= ~SDIO_POWER_PWRCTRL_MASK;
  regval |= pwrctrl;
  putreg32(regval, STM32_SDIO_POWER);
}

/****************************************************************************
 * Name: stm32_getpwrctrl
 *
 * Description:
 *   Return the current value of the  the PWRCTRL field of the SDIO POWER
 *   register.  This function can be used to see if the SDIO is powered ON
 *   or OFF
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current value of the  the PWRCTRL field of the SDIO POWER register.
 *
 ****************************************************************************/

static inline uint32_t stm32_getpwrctrl(void)
{
  return getreg32(STM32_SDIO_POWER) & SDIO_POWER_PWRCTRL_MASK;
}

/****************************************************************************
 * Name: stm32_sampleinit
 *
 * Description:
 *   Setup prior to collecting DMA samples
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void stm32_sampleinit(void)
{
  memset(g_sampleregs, 0xff,
         DEBUG_NSAMPLES * sizeof(struct stm32_sampleregs_s));
}
#endif

/****************************************************************************
 * Name: stm32_sdiosample
 *
 * Description:
 *   Sample SDIO registers
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void stm32_sdiosample(struct stm32_sdioregs_s *regs)
{
  regs->power   = (uint8_t)getreg32(STM32_SDIO_POWER);
  regs->clkcr   = (uint16_t)getreg32(STM32_SDIO_CLKCR);
  regs->dctrl   = (uint16_t)getreg32(STM32_SDIO_DCTRL);
  regs->dtimer  = getreg32(STM32_SDIO_DTIMER);
  regs->dlen    = getreg32(STM32_SDIO_DLEN);
  regs->dcount  = getreg32(STM32_SDIO_DCOUNT);
  regs->sta     = getreg32(STM32_SDIO_STA);
  regs->mask    = getreg32(STM32_SDIO_MASK);
  regs->fifocnt = getreg32(STM32_SDIO_FIFOCNT);
}
#endif

/****************************************************************************
 * Name: stm32_sample
 *
 * Description:
 *   Sample SDIO/DMA registers
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void stm32_sample(struct stm32_dev_s *priv, int index)
{
  struct stm32_sampleregs_s *regs = &g_sampleregs[index];

#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_STM32_SDIO_DMA)
  if (priv->dmamode)
    {
      stm32_dmasample(priv->dma, &regs->dma);
    }
#endif

  stm32_sdiosample(&regs->sdio);
}
#endif

/****************************************************************************
 * Name: stm32_sdiodump
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void stm32_sdiodump(struct stm32_sdioregs_s *regs, const char *msg)
{
  mcinfo("SDIO Registers: %s\n", msg);
  mcinfo("  POWER[%08x]: %08x\n", STM32_SDIO_POWER,   regs->power);
  mcinfo("  CLKCR[%08x]: %08x\n", STM32_SDIO_CLKCR,   regs->clkcr);
  mcinfo("  DCTRL[%08x]: %08x\n", STM32_SDIO_DCTRL,   regs->dctrl);
  mcinfo(" DTIMER[%08x]: %08x\n", STM32_SDIO_DTIMER,  regs->dtimer);
  mcinfo("   DLEN[%08x]: %08x\n", STM32_SDIO_DLEN,    regs->dlen);
  mcinfo(" DCOUNT[%08x]: %08x\n", STM32_SDIO_DCOUNT,  regs->dcount);
  mcinfo("    STA[%08x]: %08x\n", STM32_SDIO_STA,     regs->sta);
  mcinfo("   MASK[%08x]: %08x\n", STM32_SDIO_MASK,    regs->mask);
  mcinfo("FIFOCNT[%08x]: %08x\n", STM32_SDIO_FIFOCNT, regs->fifocnt);
}
#endif

/****************************************************************************
 * Name: stm32_dumpsample
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void stm32_dumpsample(struct stm32_dev_s *priv,
                             struct stm32_sampleregs_s *regs,
                             const char *msg)
{
#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_STM32_SDIO_DMA)
  if (priv->dmamode)
    {
      stm32_dmadump(priv->dma, &regs->dma, msg);
    }
#endif

  stm32_sdiodump(&regs->sdio, msg);
}
#endif

/****************************************************************************
 * Name: stm32_dumpsamples
 *
 * Description:
 *   Dump all sampled register data
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void stm32_dumpsamples(struct stm32_dev_s *priv)
{
  stm32_dumpsample(priv, &g_sampleregs[SAMPLENDX_BEFORE_SETUP],
                   "Before setup");

#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_STM32_SDIO_DMA)
  if (priv->dmamode)
    {
      stm32_dumpsample(priv, &g_sampleregs[SAMPLENDX_BEFORE_ENABLE],
                       "Before DMA enable");
    }
#endif

  stm32_dumpsample(priv, &g_sampleregs[SAMPLENDX_AFTER_SETUP],
                   "After setup");
  stm32_dumpsample(priv, &g_sampleregs[SAMPLENDX_END_TRANSFER],
                   "End of transfer");

#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_STM32_SDIO_DMA)
  if (priv->dmamode)
    {
      stm32_dumpsample(priv, &g_sampleregs[SAMPLENDX_DMA_CALLBACK],
                       "DMA Callback");
    }
#endif
}
#endif

/****************************************************************************
 * Name: stm32_dmacallback
 *
 * Description:
 *   Called when SDIO DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SDIO_DMA
static void stm32_dmacallback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)arg;
  DEBUGASSERT(priv->dmamode);
  sdio_eventset_t result;

  /* In the normal case, SDIO appears to handle the End-Of-Transfer interrupt
   * first with the End-Of-DMA event occurring significantly later.  On
   * transfer errors, however, the DMA error will occur before the End-of-
   * Transfer.
   */

  stm32_sample((struct stm32_dev_s *)arg, SAMPLENDX_DMA_CALLBACK);

  /* Get the result of the DMA transfer */

  if ((status & DMA_STATUS_ERROR) != 0)
    {
      mcerr("ERROR: DMA error %02x, remaining: %d\n",
            status, priv->remaining);
      result = SDIOWAIT_ERROR;
    }
  else
    {
      result = SDIOWAIT_TRANSFERDONE;
    }

  /* Then terminate the transfer if this completes all of the steps in the
   * transfer OR if a DMA error occurred.  In the non-error case, we should
   * already have the SDIO transfer done interrupt.  If not, the transfer
   * will appropriately time out.
   */

  priv->xfrflags |= SDIO_DMADONE_FLAG;
  if (priv->xfrflags == SDIO_ALLDONE || result == SDIOWAIT_ERROR)
    {
      stm32_endtransfer(priv, result);
    }
}
#endif

/****************************************************************************
 * Name: stm32_log2
 *
 * Description:
 *   Take (approximate) log base 2 of the provided number (Only works if the
 *   provided number is a power of 2).
 *
 ****************************************************************************/

static uint8_t stm32_log2(uint16_t value)
{
  uint8_t log2 = 0;

  /* 0000 0000 0000 0001 -> return 0,
   * 0000 0000 0000 001x -> return 1,
   * 0000 0000 0000 01xx -> return 2,
   * 0000 0000 0000 1xxx -> return 3,
   * ...
   * 1xxx xxxx xxxx xxxx -> return 15,
   */

  DEBUGASSERT(value > 0);
  while (value != 1)
    {
      value >>= 1;
      log2++;
    }

  return log2;
}

/****************************************************************************
 * Name: stm32_dataconfig
 *
 * Description:
 *   Configure the SDIO data path for the next data transfer
 *
 ****************************************************************************/

static void stm32_dataconfig(uint32_t timeout, uint32_t dlen, uint32_t dctrl)
{
  uint32_t regval = 0;

  /* Enable data path */

  putreg32(timeout, STM32_SDIO_DTIMER); /* Set DTIMER */
  putreg32(dlen,    STM32_SDIO_DLEN);   /* Set DLEN */

  /* Configure DCTRL DTDIR, DTMODE, and DBLOCKSIZE fields and set the DTEN
   * field
   */

  regval  =  getreg32(STM32_SDIO_DCTRL);
  regval &= ~(SDIO_DCTRL_DTDIR | SDIO_DCTRL_DTMODE |
              SDIO_DCTRL_DBLOCKSIZE_MASK);
  dctrl  &=  (SDIO_DCTRL_DTDIR | SDIO_DCTRL_DTMODE |
              SDIO_DCTRL_DBLOCKSIZE_MASK);
  regval |=  (dctrl | SDIO_DCTRL_DTEN | SDIO_DCTRL_SDIOEN);
  putreg32(regval, STM32_SDIO_DCTRL);
}

/****************************************************************************
 * Name: stm32_datadisable
 *
 * Description:
 *   Disable the SDIO data path setup by stm32_dataconfig() and
 *   disable DMA.
 *
 ****************************************************************************/

static void stm32_datadisable(void)
{
  uint32_t regval;

  /* Disable the data path */

  putreg32(SDIO_DTIMER_DATATIMEOUT, STM32_SDIO_DTIMER); /* Reset DTIMER */
  putreg32(0,                       STM32_SDIO_DLEN);   /* Reset DLEN */

  /* Reset DCTRL DTEN, DTDIR, DTMODE, DMAEN, and DBLOCKSIZE fields */

  regval  = getreg32(STM32_SDIO_DCTRL);
  regval &= ~(SDIO_DCTRL_DTEN | SDIO_DCTRL_DTDIR | SDIO_DCTRL_DTMODE |
              SDIO_DCTRL_DMAEN | SDIO_DCTRL_DBLOCKSIZE_MASK);
  putreg32(regval, STM32_SDIO_DCTRL);
}

/****************************************************************************
 * Name: stm32_sendfifo
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

static void stm32_sendfifo(struct stm32_dev_s *priv)
{
  union
  {
    uint32_t w;
    uint8_t  b[4];
  } data;

  /* Loop while there is more data to be sent and the RX FIFO is not full */

  while (priv->remaining > 0 &&
         (getreg32(STM32_SDIO_STA) & SDIO_STA_TXFIFOF) == 0)
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

      putreg32(data.w, STM32_SDIO_FIFO);
    }
}

/****************************************************************************
 * Name: stm32_recvfifo
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

static void stm32_recvfifo(struct stm32_dev_s *priv)
{
  union
  {
    uint32_t w;
    uint8_t  b[4];
  } data;

  /* Loop while there is space to store the data and there is more
   * data available in the RX FIFO.
   */

  while (priv->remaining > 0 &&
         (getreg32(STM32_SDIO_STA) & SDIO_STA_RXDAVL) != 0)
    {
      /* Read the next word from the RX FIFO */

      data.w = getreg32(STM32_SDIO_FIFO);
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
}

/****************************************************************************
 * Name: stm32_eventtimeout
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

static void stm32_eventtimeout(wdparm_t arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)arg;

  /* There is always race conditions with timer expirations. */

  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0 ||
              priv->wkupevent != 0);

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. wake up any waiting threads */

      stm32_endwait(priv, SDIOWAIT_TIMEOUT);
      mcerr("ERROR: Timeout, remaining: %d\n", priv->remaining);
    }
}

/****************************************************************************
 * Name: stm32_endwait
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

static void stm32_endwait(struct stm32_dev_s *priv,
                          sdio_eventset_t wkupevent)
{
  /* Cancel the watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Disable event-related interrupts */

  stm32_configwaitints(priv, 0, 0, wkupevent);

  /* Wake up the waiting thread */

  stm32_givesem(priv);
}

/****************************************************************************
 * Name: stm32_endtransfer
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

static void stm32_endtransfer(struct stm32_dev_s *priv,
                              sdio_eventset_t wkupevent)
{
  /* Disable all transfer related interrupts */

  stm32_configxfrints(priv, 0);

  /* Clearing pending interrupt status on all transfer related interrupts */

  putreg32(SDIO_XFRDONE_ICR, STM32_SDIO_ICR);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_STM32_SDIO_DMA
  if (priv->dmamode)
    {
      /* DMA debug instrumentation */

      stm32_sample(priv, SAMPLENDX_END_TRANSFER);

      /* Make sure that the DMA is stopped (it will be stopped automatically
       * on normal transfers, but not necessarily when the transfer
       * terminates on an error condition).
       */

      stm32_dmastop(priv->dma);
    }
#endif

  /* Mark the transfer finished */

  priv->remaining = 0;

  /* Is a thread wait for these data transfer complete events? */

  if ((priv->waitevents & wkupevent) != 0)
    {
      /* Yes.. wake up any waiting threads */

      stm32_endwait(priv, wkupevent);
    }
}

/****************************************************************************
 * Name: stm32_rdyinterrupt
 *
 * Description:
 *   SDIO ready interrupt handler
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
static int stm32_rdyinterrupt(int irq, void *context, FAR void *arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)arg;
  stm32_endwait(priv, SDIOWAIT_WRCOMPLETE);
  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_interrupt
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

static int stm32_interrupt(int irq, void *context, FAR void *arg)
{
  struct stm32_dev_s *priv = &g_sdiodev;
  uint32_t enabled;
  uint32_t pending;

  /* Loop while there are pending interrupts.  Check the SDIO status
   * register.  Mask out all bits that don't correspond to enabled
   * interrupts.  (This depends on the fact that bits are ordered
   * the same in both the STA and MASK register).  If there are non-zero
   * bits remaining, then we have work to do here.
   */

  while ((enabled = getreg32(STM32_SDIO_STA) &
                    getreg32(STM32_SDIO_MASK)) != 0)
    {
      /* Handle in progress, interrupt driven data transfers ****************/

      pending  = enabled & priv->xfrmask;
      if (pending != 0)
        {
#ifdef CONFIG_STM32_SDIO_DMA
          if (!priv->dmamode)
#endif
            {
              /* Is the RX FIFO half full or more?  Is so then we must be
               * processing a receive transaction.
               */

              if ((pending & SDIO_STA_RXFIFOHF) != 0)
                {
                  /* Receive data from the RX FIFO */

                  stm32_recvfifo(priv);
                }

              /* Otherwise, Is the transmit FIFO half empty or less?  If so
               * we must be processing a send transaction.  NOTE:  We can't
               * be processing both!
               */

              else if ((pending & SDIO_STA_TXFIFOHE) != 0)
                {
                  /* Send data via the TX FIFO */

                  stm32_sendfifo(priv);
                }
            }

          /* Handle data end events */

          if ((pending & SDIO_STA_DATAEND) != 0)
            {
              /* Handle any data remaining the RX FIFO.  If the RX FIFO is
               * less than half full at the end of the transfer, then no
               * half-full interrupt will be received.
               */

              /* Was this transfer performed in DMA mode? */

#ifdef CONFIG_STM32_SDIO_DMA
              if (priv->dmamode)
                {
                  /* Yes.. Terminate the transfers only if the DMA has also
                   * finished.
                   */

                  priv->xfrflags |= SDIO_XFRDONE_FLAG;
                  if (priv->xfrflags == SDIO_ALLDONE)
                    {
                      stm32_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
                    }

                  /* Otherwise, just disable further transfer interrupts and
                   * wait for the DMA complete event.
                   */

                  else
                    {
                      stm32_configxfrints(priv, 0);
                    }
                }
              else
#endif
                {
                  /* Receive data from the RX FIFO */

                  stm32_recvfifo(priv);

                  /* Then terminate the transfer */

                  stm32_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
                }
            }

          /* Handle data block send/receive CRC failure */

          else if ((pending & SDIO_STA_DCRCFAIL) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Data block CRC failure, remaining: %d\n",
                    priv->remaining);
              stm32_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle data timeout error */

          else if ((pending & SDIO_STA_DTIMEOUT) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Data timeout, remaining: %d\n",
                     priv->remaining);
              stm32_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT);
            }

          /* Handle RX FIFO overrun error */

          else if ((pending & SDIO_STA_RXOVERR) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: RX FIFO overrun, remaining: %d\n",
                    priv->remaining);
              stm32_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle TX FIFO underrun error */

          else if ((pending & SDIO_STA_TXUNDERR) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: TX FIFO underrun, remaining: %d\n",
                    priv->remaining);
              stm32_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle start bit error */

          else if ((pending & SDIO_STA_STBITERR) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Start bit, remaining: %d\n",
                    priv->remaining);
              stm32_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }
        }

      /* Handle wait events *************************************************/

      pending  = enabled & priv->waitmask;
      if (pending != 0)
        {
          /* Is this a response completion event? */

          if ((pending & SDIO_RESPDONE_STA) != 0)
            {
              /* Yes.. Is their a thread waiting for response done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  putreg32(SDIO_RESPDONE_ICR | SDIO_CMDDONE_ICR,
                           STM32_SDIO_ICR);
                  stm32_endwait(priv, SDIOWAIT_RESPONSEDONE);
                }
            }

          /* Is this a command completion event? */

          if ((pending & SDIO_CMDDONE_STA) != 0)
            {
              /* Yes.. Is their a thread waiting for command done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  putreg32(SDIO_CMDDONE_ICR, STM32_SDIO_ICR);
                  stm32_endwait(priv, SDIOWAIT_CMDDONE);
                }
            }
        }

#ifdef CONFIG_STM32_SDIO_CARD
      /* Handle SDIO card interrupt */

      pending = enabled & priv->sdiointmask;
      if (pending != 0)
        {
          putreg32(SDIO_STA_SDIOIT, STM32_SDIO_ICR);

          /* Perform callback */

          if (priv->do_sdio_card)
            {
              priv->do_sdio_card(priv->do_sdio_arg);
            }
        }
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_lock
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
static int stm32_lock(FAR struct sdio_dev_s *dev, bool lock)
{
  /* Single SDIO instance so there is only one possibility.  The multiplex
   * bus is part of board support package.
   */

  stm32_muxbus_sdio_lock(lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_reset
 *
 * Description:
 *   Reset the SDIO controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_reset(FAR struct sdio_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;
  irqstate_t flags;

  /* Disable clocking */

  flags = enter_critical_section();
  putreg32(0, SDIO_CLKCR_CLKEN_BB);
  stm32_setpwrctrl(SDIO_POWER_PWRCTRL_OFF);

  /* Put SDIO registers in their default, reset state */

  stm32_default();

  /* Reset data */

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitmask   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */
#ifdef CONFIG_STM32_SDIO_DMA
  priv->xfrflags   = 0;      /* Used to synchronize SDIO and DMA completion events */
#endif

  wd_cancel(&priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->buffer     = 0;      /* Address of current R/W buffer */
  priv->remaining  = 0;      /* Number of bytes remaining in the transfer */
  priv->xfrmask    = 0;      /* Interrupt enables for data transfer */

#ifdef CONFIG_STM32_SDIO_CARD
  priv->sdiointmask = 0;     /* SDIO card in-band interrupt mask */
#endif

  /* DMA data transfer support */

  priv->widebus    = false;  /* Required for DMA support */
#ifdef CONFIG_STM32_SDIO_DMA
  priv->dmamode    = false;  /* true: DMA mode transfer */
#endif

  /* Configure the SDIO peripheral */

  stm32_setclkcr(STM32_CLCKCR_INIT | SDIO_CLKCR_CLKEN);
  stm32_setpwrctrl(SDIO_POWER_PWRCTRL_ON);
  leave_critical_section(flags);

  mcinfo("CLCKR: %08x POWER: %08x\n",
         getreg32(STM32_SDIO_CLKCR), getreg32(STM32_SDIO_POWER));
}

/****************************************************************************
 * Name: stm32_capabilities
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

static sdio_capset_t stm32_capabilities(FAR struct sdio_dev_s *dev)
{
  sdio_capset_t caps = 0;

#ifdef CONFIG_STM32_SDIO_WIDTH_D1_ONLY
  caps |= SDIO_CAPS_1BIT_ONLY;
#endif
#ifdef CONFIG_STM32_SDIO_DMA
  caps |= SDIO_CAPS_DMASUPPORTED;
#endif

  return caps;
}

/****************************************************************************
 * Name: stm32_status
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see stm32_status_* defines)
 *
 ****************************************************************************/

static sdio_statset_t stm32_status(FAR struct sdio_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  return priv->cdstatus;
}

/****************************************************************************
 * Name: stm32_widebus
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

static void stm32_widebus(FAR struct sdio_dev_s *dev, bool wide)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  priv->widebus = wide;
}

/****************************************************************************
 * Name: stm32_clock
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

static void stm32_clock(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  uint32_t clckr;

  switch (rate)
    {
      /* Disable clocking (with default ID mode divisor) */

      default:
      case CLOCK_SDIO_DISABLED:
        clckr = STM32_CLCKCR_INIT;
        return;

      /* Enable in initial ID mode clocking (<400KHz) */

      case CLOCK_IDMODE:
        clckr = (STM32_CLCKCR_INIT | SDIO_CLKCR_CLKEN);
        break;

      /* Enable in MMC normal operation clocking */

      case CLOCK_MMC_TRANSFER:
        clckr = (SDIO_CLKCR_MMCXFR | SDIO_CLKCR_CLKEN);
        break;

      /* SD normal operation clocking (wide 4-bit mode) */

      case CLOCK_SD_TRANSFER_4BIT:
#ifndef CONFIG_STM32_SDIO_WIDTH_D1_ONLY
        clckr = (SDIO_CLCKR_SDWIDEXFR | SDIO_CLKCR_CLKEN);
        break;
#endif

      /* SD normal operation clocking (narrow 1-bit mode) */

      case CLOCK_SD_TRANSFER_1BIT:
        clckr = (SDIO_CLCKR_SDXFR | SDIO_CLKCR_CLKEN);
        break;
    }

  /* Set the new clock frequency along with the clock enable/disable bit */

  stm32_setclkcr(clckr);
}

/****************************************************************************
 * Name: stm32_attach
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

static int stm32_attach(FAR struct sdio_dev_s *dev)
{
  int ret;

  /* Attach the SDIO interrupt handler */

  ret = irq_attach(STM32_IRQ_SDIO, stm32_interrupt, NULL);
  if (ret == OK)
    {
      /* Disable all interrupts at the SDIO controller and clear static
       * interrupt flags
       */

      putreg32(SDIO_MASK_RESET,      STM32_SDIO_MASK);
      putreg32(SDIO_ICR_STATICFLAGS, STM32_SDIO_ICR);

      /* Enable SDIO interrupts at the NVIC.  They can now be enabled at
       * the SDIO controller as needed.
       */

      up_enable_irq(STM32_IRQ_SDIO);
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_sendcmd
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

static int stm32_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd,
                         uint32_t arg)
{
  uint32_t regval;
  uint32_t cmdidx;

  /* Set the SDIO Argument value */

  putreg32(arg, STM32_SDIO_ARG);

  /* Clear CMDINDEX, WAITRESP, WAITINT, WAITPEND, and CPSMEN bits */

  regval = getreg32(STM32_SDIO_CMD);
  regval &= ~(SDIO_CMD_CMDINDEX_MASK | SDIO_CMD_WAITRESP_MASK |
              SDIO_CMD_WAITINT | SDIO_CMD_WAITPEND | SDIO_CMD_CPSMEN);

  /* Set WAITRESP bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      regval |= SDIO_CMD_NORESPONSE;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
    case MMCSD_R7_RESPONSE:
      regval |= SDIO_CMD_SHORTRESPONSE;
      break;

    case MMCSD_R2_RESPONSE:
      regval |= SDIO_CMD_LONGRESPONSE;
      break;
    }

  /* Set CPSMEN and the command index */

  cmdidx  = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval |= cmdidx | SDIO_CMD_CPSMEN;

  mcinfo("cmd: %08x arg: %08x regval: %08x\n", cmd, arg, regval);

  /* Write the SDIO CMD */

  putreg32(SDIO_RESPDONE_ICR | SDIO_CMDDONE_ICR, STM32_SDIO_ICR);
  putreg32(regval, STM32_SDIO_CMD);
  return OK;
}

/****************************************************************************
 * Name: stm32_blocksetup
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
static void stm32_blocksetup(FAR struct sdio_dev_s *dev,
                             unsigned int blocklen, unsigned int nblocks)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  /* Configure block size for next transfer */

  priv->block_size = stm32_log2(blocklen);
}
#endif

/****************************************************************************
 * Name: stm32_recvsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card in non-DMA
 *   (interrupt driven mode).  This method will do whatever controller setup
 *   is necessary.  This would be called for SD memory just BEFORE sending
 *   CMD13 (SEND_STATUS), CMD17 (READ_SINGLE_BLOCK), CMD18
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

static int stm32_recvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
                           size_t nbytes)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  stm32_datadisable();
  stm32_sampleinit();
  stm32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
#ifdef CONFIG_STM32_STM32_SDIO_DMA
  priv->dmamode   = false;
#endif

  /* Then set up the SDIO data path */

#ifdef CONFIG_SDIO_BLOCKSETUP
  if (priv->block_size != STM32_SDIO_USE_DEFAULT_BLOCKSIZE)
    {
      dblocksize = priv->block_size << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
    }
  else
#endif
    {
      dblocksize = stm32_log2(nbytes) << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
    }

  stm32_dataconfig(SDIO_DTIMER_DATATIMEOUT, nbytes,
                   dblocksize | SDIO_DCTRL_DTDIR);

  /* And enable interrupts */

  stm32_configxfrints(priv, SDIO_RECV_MASK);
  stm32_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}

/****************************************************************************
 * Name: stm32_sendsetup
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

static int stm32_sendsetup(FAR struct sdio_dev_s *dev,
                           FAR const uint8_t *buffer, size_t nbytes)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  stm32_datadisable();
  stm32_sampleinit();
  stm32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
#ifdef CONFIG_STM32_STM32_SDIO_DMA
  priv->dmamode   = false;
#endif

  /* Then set up the SDIO data path */

#ifdef CONFIG_SDIO_BLOCKSETUP
  if (priv->block_size != STM32_SDIO_USE_DEFAULT_BLOCKSIZE)
    {
      dblocksize = priv->block_size << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
    }
  else
#endif
    {
      dblocksize = stm32_log2(nbytes) << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
    }

  stm32_dataconfig(SDIO_DTIMER_DATATIMEOUT, nbytes, dblocksize);

  /* Enable TX interrupts */

  stm32_configxfrints(priv, SDIO_SEND_MASK);
  stm32_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}

/****************************************************************************
 * Name: stm32_cancel
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

static int stm32_cancel(FAR struct sdio_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  /* Disable all transfer- and event- related interrupts */

  stm32_configxfrints(priv, 0);
  stm32_configwaitints(priv, 0, 0, 0);

  /* Clearing pending interrupt status on all transfer- and event- related
   * interrupts
   */

  putreg32(SDIO_WAITALL_ICR, STM32_SDIO_ICR);

  /* Cancel any watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_STM32_SDIO_DMA
  if (priv->dmamode)
    {
      /* Make sure that the DMA is stopped (it will be stopped automatically
       * on normal transfers, but not necessarily when the transfer
       * terminates on an error condition.
       */

      stm32_dmastop(priv->dma);
    }
#endif

  /* Mark no transfer in progress */

  priv->remaining = 0;
  return OK;
}

/****************************************************************************
 * Name: stm32_waitresponse
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

static int stm32_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd)
{
  int32_t timeout;
  uint32_t events;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      events  = SDIO_CMDDONE_STA;
      timeout = SDIO_CMDTIMEOUT;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
      events  = SDIO_RESPDONE_STA;
      timeout = SDIO_LONGTIMEOUT;
      break;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      events  = SDIO_RESPDONE_STA;
      timeout = SDIO_CMDTIMEOUT;
      break;

    default:
      return -EINVAL;
    }

  /* Then wait for the response (or timeout) */

  while ((getreg32(STM32_SDIO_STA) & events) == 0)
    {
      if (--timeout <= 0)
        {
          mcerr("ERROR: Timeout cmd: %08x events: %08x STA: %08x\n",
               cmd, events, getreg32(STM32_SDIO_STA));

          return -ETIMEDOUT;
        }
    }

  putreg32(SDIO_CMDDONE_ICR, STM32_SDIO_ICR);
  return OK;
}

/****************************************************************************
 * Name: stm32_recv*
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

static int stm32_recvshortcrc(FAR struct sdio_dev_s *dev, uint32_t cmd,
                              uint32_t *rshort)
{
#ifdef CONFIG_DEBUG_MEMCARD_INFO
  uint32_t respcmd;
#endif
  uint32_t regval;
  int ret = OK;

  /* R1  Command response (48-bit)
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
   */

#ifdef CONFIG_DEBUG_MEMCARD_INFO
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
      mcerr("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = getreg32(STM32_SDIO_STA);
      if ((regval & SDIO_STA_CTIMEOUT) != 0)
        {
          mcerr("ERROR: Command timeout: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & SDIO_STA_CCRCFAIL) != 0)
        {
          mcerr("ERROR: CRC failure: %08x\n", regval);
          ret = -EIO;
        }
#ifdef CONFIG_DEBUG_MEMCARD_INFO
      else
        {
          /* Check response received is of desired command */

          respcmd = getreg32(STM32_SDIO_RESPCMD);
          if ((uint8_t)(respcmd & SDIO_RESPCMD_MASK) !=
              (cmd & MMCSD_CMDIDX_MASK))
            {
              mcerr("ERROR: RESCMD=%02x CMD=%08x\n", respcmd, cmd);
              ret = -EINVAL;
            }
        }
#endif
    }

  /* Clear all pending message completion events and return the R1/R6
   * response.
   */

  putreg32(SDIO_RESPDONE_ICR | SDIO_CMDDONE_ICR, STM32_SDIO_ICR);
  *rshort = getreg32(STM32_SDIO_RESP1);
  return ret;
}

static int stm32_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd,
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

#ifdef CONFIG_DEBUG_MEMCARD_INFO
  /* Check that R1 is the correct response to this command */

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R2_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = getreg32(STM32_SDIO_STA);
      if (regval & SDIO_STA_CTIMEOUT)
        {
          mcerr("ERROR: Timeout STA: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & SDIO_STA_CCRCFAIL)
        {
          mcerr("ERROR: CRC fail STA: %08x\n", regval);
          ret = -EIO;
        }
    }

  /* Return the long response */

  putreg32(SDIO_RESPDONE_ICR | SDIO_CMDDONE_ICR, STM32_SDIO_ICR);
  if (rlong)
    {
      rlong[0] = getreg32(STM32_SDIO_RESP1);
      rlong[1] = getreg32(STM32_SDIO_RESP2);
      rlong[2] = getreg32(STM32_SDIO_RESP3);
      rlong[3] = getreg32(STM32_SDIO_RESP4);
    }

  return ret;
}

static int stm32_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd,
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

  /* Check that this is the correct response to this command */

#ifdef CONFIG_DEBUG_MEMCARD_INFO
  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R4_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout occurred (Apparently a CRC error can terminate
       * a good response)
       */

      regval = getreg32(STM32_SDIO_STA);
      if (regval & SDIO_STA_CTIMEOUT)
        {
          mcerr("ERROR: Timeout STA: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
    }

  putreg32(SDIO_RESPDONE_ICR | SDIO_CMDDONE_ICR, STM32_SDIO_ICR);
  if (rshort)
    {
      *rshort = getreg32(STM32_SDIO_RESP1);
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the SDIO_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling stm32_eventwait.  This is done in this way
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

static void stm32_waitenable(FAR struct sdio_dev_s *dev,
                             sdio_eventset_t eventset)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t waitmask;

  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  stm32_configwaitints(priv, 0, 0, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  if ((eventset & SDIOWAIT_WRCOMPLETE) != 0)
    {
      waitmask = SDIOWAIT_WRCOMPLETE;
    }
  else
#endif
    {
      waitmask = 0;
      if ((eventset & SDIOWAIT_CMDDONE) != 0)
        {
          waitmask |= SDIO_CMDDONE_MASK;
        }

      if ((eventset & SDIOWAIT_RESPONSEDONE) != 0)
        {
          waitmask |= SDIO_RESPDONE_MASK;
        }

      if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
        {
          waitmask |= SDIO_XFRDONE_MASK;
        }

      /* Enable event-related interrupts */

      putreg32(SDIO_WAITALL_ICR, STM32_SDIO_ICR);
    }

  stm32_configwaitints(priv, waitmask, eventset, 0);
}

/****************************************************************************
 * Name: stm32_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDIO_WAITEVENTS are disabled when stm32_eventwait
 *   returns.  SDIO_WAITEVENTS must be called again before stm32_eventwait
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

static sdio_eventset_t stm32_eventwait(FAR struct sdio_dev_s *dev,
                                       uint32_t timeout)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  sdio_eventset_t wkupevent = 0;
  irqstate_t flags;
  int ret;

  /* There is a race condition here... the event may have completed before
   * we get here.  In this case waitevents will be zero, but wkupevents will
   * be non-zero (and, hopefully, the semaphore count will also be non-zero.
   */

  flags = enter_critical_section();

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  /* A card ejected while in SDIOWAIT_WRCOMPLETE can lead to a
   * condition where there is no waitevents set and no wkupevent
   */

  if (priv->waitevents == 0 && priv->wkupevent == 0)
    {
      wkupevent = SDIOWAIT_ERROR;
      goto errout_with_waitints;
    }

#else
  DEBUGASSERT(priv->waitevents != 0 || priv->wkupevent != 0);
#endif

  /* Check if the timeout event is specified in the event set */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      int delay;

      /* Yes.. Handle a cornercase: The user request a timeout event but
       * with timeout == 0?
       */

      if (!timeout)
        {
          /* Then just tell the caller that we already timed out */

          wkupevent = SDIOWAIT_TIMEOUT;
          goto errout;
        }

      /* Start the watchdog timer */

      delay = MSEC2TICK(timeout);
      ret   = wd_start(&priv->waitwdog, delay,
                       stm32_eventtimeout, (wdparm_t)priv);
      if (ret < 0)
        {
          mcerr("ERROR: wd_start failed: %d\n", ret);
        }
    }

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  if ((priv->waitevents & SDIOWAIT_WRCOMPLETE) != 0)
    {
      /* Atomically read pin to see if ready (true) and determine if ISR
       * fired.  If Pin is ready and if ISR did NOT fire end the wait here.
       */

      if (stm32_gpioread(GPIO_SDIO_D0) &&
         (priv->wkupevent & SDIOWAIT_WRCOMPLETE) == 0)
        {
          stm32_endwait(priv, SDIOWAIT_WRCOMPLETE);
        }
    }
#endif

  /* Loop until the event (or the timeout occurs). Race conditions are
   * avoided by calling stm32_waitenable prior to triggering the logic that
   * will cause the wait to terminate.  Under certain race conditions, the
   * waited-for may have already occurred before this function was called!
   */

  for (; ; )
    {
      /* Wait for an event in event set to occur.  If this the event has
       * already occurred, then the semaphore will already have been
       * incremented and there will be no wait.
       */

      ret = stm32_takesem(priv);
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

  stm32_configwaitints(priv, 0, 0, 0);
#ifdef CONFIG_STM32_SDIO_DMA
  priv->xfrflags   = 0;
#endif

errout:
  leave_critical_section(flags);
  stm32_dumpsamples(priv);
  return wkupevent;
}

/****************************************************************************
 * Name: stm32_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events.  This is part of the
 *   the SDIO callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in stm32_registercallback.
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

static void stm32_callbackenable(FAR struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  mcinfo("eventset: %02x\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  stm32_callback(priv);
}

/****************************************************************************
 * Name: stm32_registercallback
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

static int stm32_registercallback(FAR struct sdio_dev_s *dev,
                                  worker_t callback, void *arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  /* Disable callbacks and register this callback and is argument */

  mcinfo("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: stm32_dmapreflight
 *
 * Description:
 *   Preflight an SDIO DMA operation.  If the buffer is not well-formed for
 *   SDIO DMA transfer (alignment, size, etc.) returns an error.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA to/from
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 ****************************************************************************/

#if defined(CONFIG_STM32_SDIO_DMA) && defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
static int stm32_dmapreflight(FAR struct sdio_dev_s *dev,
                              FAR const uint8_t *buffer, size_t buflen)
{
#if !defined(CONFIG_STM32_STM32F4XXX)
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);

  /* Wide bus operation is required for DMA */

  if (!priv->widebus)
    {
      return -EINVAL;
    }
#endif

  /* DMA must be possible to the buffer */

  if (!stm32_dmacapable((uintptr_t)buffer, (buflen + 3) >> 2,
                        SDIO_RXDMA32_CONFIG))
    {
      return -EFAULT;
    }

  return 0;
}
#endif

/****************************************************************************
 * Name: stm32_dmarecvsetup
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

#ifdef CONFIG_STM32_SDIO_DMA
static int stm32_dmarecvsetup(FAR struct sdio_dev_s *dev,
                              FAR uint8_t *buffer, size_t buflen)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
  DEBUGASSERT(stm32_dmapreflight(dev, buffer, buflen) == 0);
#endif

  /* Reset the DPSM configuration */

  stm32_datadisable();

  /* Initialize register sampling */

  stm32_sampleinit();
  stm32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->dmamode   = true;

  /* Then set up the SDIO data path */

#ifdef CONFIG_SDIO_BLOCKSETUP
  if (priv->block_size != STM32_SDIO_USE_DEFAULT_BLOCKSIZE)
    {
      dblocksize = priv->block_size << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
    }
  else
#endif
    {
      dblocksize = stm32_log2(buflen) << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
    }

  stm32_dataconfig(SDIO_DTIMER_DATATIMEOUT, buflen,
                   dblocksize | SDIO_DCTRL_DTDIR);

  /* Configure the RX DMA */

  stm32_configxfrints(priv, SDIO_DMARECV_MASK);

  putreg32(1, SDIO_DCTRL_DMAEN_BB);
  stm32_dmasetup(priv->dma, STM32_SDIO_FIFO, (uint32_t)buffer,
                 (buflen + 3) >> 2, SDIO_RXDMA32_CONFIG);

  /* Start the DMA */

  stm32_sample(priv, SAMPLENDX_BEFORE_ENABLE);
  stm32_dmastart(priv->dma, stm32_dmacallback, priv, false);
  stm32_sample(priv, SAMPLENDX_AFTER_SETUP);

  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_dmasendsetup
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

#ifdef CONFIG_STM32_SDIO_DMA
static int stm32_dmasendsetup(FAR struct sdio_dev_s *dev,
                              FAR const uint8_t *buffer, size_t buflen)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
  DEBUGASSERT(stm32_dmapreflight(dev, buffer, buflen) == 0);
#endif

  /* Reset the DPSM configuration */

  stm32_datadisable();

  /* Initialize register sampling */

  stm32_sampleinit();
  stm32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->dmamode   = true;

  /* Then set up the SDIO data path */

#ifdef CONFIG_SDIO_BLOCKSETUP
  if (priv->block_size != STM32_SDIO_USE_DEFAULT_BLOCKSIZE)
    {
      dblocksize = priv->block_size << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
    }
  else
#endif
    {
      dblocksize = stm32_log2(buflen) << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
    }

  stm32_dataconfig(SDIO_DTIMER_DATATIMEOUT, buflen, dblocksize);

  /* Configure the TX DMA */

  stm32_dmasetup(priv->dma, STM32_SDIO_FIFO, (uint32_t)buffer,
                 (buflen + 3) >> 2, SDIO_TXDMA32_CONFIG);

  stm32_sample(priv, SAMPLENDX_BEFORE_ENABLE);
  putreg32(1, SDIO_DCTRL_DMAEN_BB);

  /* Start the DMA */

  stm32_dmastart(priv->dma, stm32_dmacallback, priv, false);
  stm32_sample(priv, SAMPLENDX_AFTER_SETUP);

  /* Enable TX interrupts */

  stm32_configxfrints(priv, SDIO_DMASEND_MASK);

  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_callback
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

static void stm32_callback(void *arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)arg;

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
           work_queue(HPWORK, &priv->cbwork, (worker_t)priv->callback,
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
 * Name: stm32_default
 *
 * Description:
 *   Restore SDIO registers to their default, reset values
 *
 ****************************************************************************/

static void stm32_default(void)
{
  putreg32(SDIO_POWER_RESET,  STM32_SDIO_POWER);
  putreg32(SDIO_CLKCR_RESET,  STM32_SDIO_CLKCR);
  putreg32(SDIO_ARG_RESET,    STM32_SDIO_ARG);
  putreg32(SDIO_CMD_RESET,    STM32_SDIO_CMD);
  putreg32(SDIO_DTIMER_RESET, STM32_SDIO_DTIMER);
  putreg32(SDIO_DLEN_RESET,   STM32_SDIO_DLEN);
  putreg32(SDIO_DCTRL_RESET,  STM32_SDIO_DCTRL);
  putreg32(SDIO_ICR_RESET,    STM32_SDIO_ICR);
  putreg32(SDIO_MASK_RESET,   STM32_SDIO_MASK);
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
 * Returned Value:
 *   A reference to an SDIO interface structure.  NULL is returned on
 *   failures.
 *
 ****************************************************************************/

FAR struct sdio_dev_s *sdio_initialize(int slotno)
{
  /* There is only one slot */

  struct stm32_dev_s *priv = &g_sdiodev;

  /* Initialize the SDIO slot structure */

  /* Initialize semaphores */

  nxsem_init(&priv->waitsem, 0, 0);

  /* The waitsem semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_set_protocol(&priv->waitsem, SEM_PRIO_NONE);

  /* Allocate a DMA channel */

#ifdef CONFIG_STM32_SDIO_DMA
  priv->dma = stm32_dmachannel(SDIO_DMACHAN);
  DEBUGASSERT(priv->dma);
#endif

  /* Configure GPIOs for 4-bit, wide-bus operation (the chip is capable of
   * 8-bit wide bus operation but D4-D7 are not configured).
   *
   * If bus is multiplexed then there is a custom bus configuration utility
   * in the scope of the board support package.
   */

#ifndef CONFIG_SDIO_MUXBUS
  stm32_configgpio(GPIO_SDIO_D0 | SDIO_PULLUP_ENABLE);
#ifndef CONFIG_STM32_SDIO_WIDTH_D1_ONLY
  stm32_configgpio(GPIO_SDIO_D1 | SDIO_PULLUP_ENABLE);
  stm32_configgpio(GPIO_SDIO_D2 | SDIO_PULLUP_ENABLE);
  stm32_configgpio(GPIO_SDIO_D3 | SDIO_PULLUP_ENABLE);
#endif
  stm32_configgpio(GPIO_SDIO_CK | SDIO_PULLUP_ENABLE);
  stm32_configgpio(GPIO_SDIO_CMD | SDIO_PULLUP_ENABLE);
#endif

  /* Reset the card and assure that it is in the initial, unconfigured
   * state.
   */

  stm32_reset(&priv->dev);
  return &g_sdiodev.dev;
}

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic -- possibly from an interrupt handler --
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

void sdio_mediachange(FAR struct sdio_dev_s *dev, bool cardinslot)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
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

  mcinfo("cdstatus OLD: %02x NEW: %02x\n", cdstatus, priv->cdstatus);

  /* Perform any requested callback if the status has changed */

  if (cdstatus != priv->cdstatus)
    {
      stm32_callback(priv);
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

void sdio_wrprotect(FAR struct sdio_dev_s *dev, bool wrprotect)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
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
 * Name: sdio_set_sdio_card_isr
 *
 * Description:
 *   SDIO card generates interrupt via SDIO_DATA_1 pin.
 *   Called by board-specific logic to register an ISR for SDIO card.
 *
 * Input Parameters:
 *   func      - callback function.
 *   arg       - arg to be passed to the function.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SDIO_CARD
void sdio_set_sdio_card_isr(FAR struct sdio_dev_s *dev,
                            int (*func)(void *), void *arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  priv->do_sdio_card = func;

  if (func != NULL)
    {
      priv->sdiointmask = SDIO_STA_SDIOIT;
      priv->do_sdio_arg = arg;
    }
  else
    {
      priv->sdiointmask = 0;
    }

  putreg32(priv->xfrmask | priv->waitmask | priv->sdiointmask,
           STM32_SDIO_MASK);
}
#endif

#endif /* CONFIG_STM32_SDIO */
