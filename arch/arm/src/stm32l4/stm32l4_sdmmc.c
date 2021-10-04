/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_sdmmc.c
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

#include "stm32l4_dma.h"
#include "stm32l4_gpio.h"
#include "stm32l4_sdmmc.h"

#if defined(CONFIG_STM32L4_SDMMC1) || defined(CONFIG_STM32L4_SDMMC2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Required system configuration options:
 *
 *   CONFIG_ARCH_DMA - Enable architecture-specific DMA subsystem
 *     initialization.  Required if CONFIG_SDMMC[1|2]_DMA is enabled.
 *   CONFIG_STM32L4_DMA2 - Enable STM32 DMA2 support.  Required if
 *     CONFIG_SDMMC[1|2]_DMA  is enabled
 *   CONFIG_SCHED_WORKQUEUE -- Callback support requires work queue support.
 *
 * Driver-specific configuration options:
 *
 *   CONFIG_SDIO_MUXBUS - Setting this configuration enables some locking
 *     APIs to manage concurrent accesses on the SDMMC bus.  This is not
 *     needed for the simple case of a single SD card, for example.
 *   CONFIG_STM32L4_SDMMC_DMA - Enable SDMMC.  This is a marginally
 *    optional.  For most usages, SDMMC will cause data overruns if used
 *    without DMA.  NOTE the above system DMA configuration options.
 *   CONFIG_SDMMC1/2_WIDTH_D1_ONLY - This may be selected to force the driver
 *     operate with only a single data line (the default is to use all
 *     4 SD data lines).
 *   CONFIG_SDMMC_DMAPRIO - SDMMC DMA priority.  This can be selected if
 *     CONFIG_STM32L4_SDMMC_DMA is enabled.
 *   CONFIG_CONFIG_STM32L4_SDMMC_XFRDEBUG - Enables some very low-level
 *     debug output.  This also requires CONFIG_DEBUG_FS and
 *     CONFIG_DEBUG_INFO
 */

#ifndef CONFIG_STM32L4_SDMMC_DMA
#  warning "Large Non-DMA transfer may result in RX overrun failures"
#else
#  if !defined(CONFIG_STM32L4_DMA2) && !defined(CONFIG_STM32L4_DMAMUX)
#    error "CONFIG_STM32L4_SDMMC_DMA support requires CONFIG_STM32L4_DMA2"
#  endif
#  ifndef CONFIG_SDIO_DMA
#    error CONFIG_SDIO_DMA must be defined with CONFIG_STM32L4_SDMMC_DMA
#  endif
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE"
#endif

#ifdef CONFIG_STM32L4_SDMMC1
#  ifdef CONFIG_STM32L4_SDMMC_DMA
#    ifndef CONFIG_STM32L4_SDMMC1_DMAPRIO
#        define CONFIG_STM32L4_SDMMC1_DMAPRIO DMA_SCR_PRIVERYHI
#    endif
#    if (CONFIG_STM32L4_SDMMC1_DMAPRIO & ~DMA_CCR_PL_MASK) != 0
#      error "Illegal value for CONFIG_STM32L4_SDMMC1_DMAPRIO"
#    endif
#  else
#    undef CONFIG_STM32L4_SDMMC1_DMAPRIO
#  endif
#endif

#ifdef CONFIG_STM32L4_SDMMC2
#  ifdef CONFIG_STM32L4_SDMMC_DMA
#    ifndef CONFIG_STM32L4_SDMMC2_DMAPRIO
#        define CONFIG_STM32L4_SDMMC2_DMAPRIO DMA_SCR_PRIVERYHI
#    endif
#    if (CONFIG_STM32L4_SDMMC2_DMAPRIO & ~DMA_CCR_PL_MASK) != 0
#      error "Illegal value for CONFIG_STM32L4_SDMMC2_DMAPRIO"
#    endif
#  else
#    undef CONFIG_STM32L4_SDMMC2_DMAPRIO
#  endif
#endif

#if !defined(CONFIG_DEBUG_FS) || !defined(CONFIG_DEBUG_FEATURES)
#  undef CONFIG_CONFIG_STM32L4_SDMMC_XFRDEBUG
#endif

/* Friendly CLKCR bit re-definitions ****************************************/

#define STM32_CLKCR_RISINGEDGE    (0)
#define STM32_CLKCR_FALLINGEDGE   STM32_SDMMC_CLKCR_NEGEDGE

/* Use the default of the rising edge but allow a configuration,
 * that does not have the errata, to override the edge the SDIO
 * command and data is changed on.
 */

#if !defined(STM32_SDMMC_CLKCR_EDGE)
#  define STM32_SDMMC_CLKCR_EDGE STM32_CLKCR_RISINGEDGE
#endif

/* Mode dependent settings.  These depend on clock divisor settings that must
 * be defined in the board-specific board.h header file:
 * STM32_SDMMC_INIT_CLKDIV, STM32_SDMMC_MMCXFR_CLKDIV, and
 * STM32_SDMMC_SDXFR_CLKDIV.
 */

#define STM32_CLCKCR_INIT           (STM32_SDMMC_INIT_CLKDIV      | \
                                     STM32_SDMMC_CLKCR_EDGE | \
                                     STM32_SDMMC_CLKCR_WIDBUS_D1)
#define STM32_SDMMC_CLKCR_MMCXFR    (STM32_SDMMC_MMCXFR_CLKDIV    | \
                                     STM32_SDMMC_CLKCR_EDGE | \
                                     STM32_SDMMC_CLKCR_WIDBUS_D1)
#define STM32_SDMMC_CLCKR_SDXFR     (STM32_SDMMC_SDXFR_CLKDIV     | \
                                     STM32_SDMMC_CLKCR_EDGE | \
                                     STM32_SDMMC_CLKCR_WIDBUS_D1)
#define STM32_SDMMC_CLCKR_SDWIDEXFR (STM32_SDMMC_SDXFR_CLKDIV     | \
                                     STM32_SDMMC_CLKCR_EDGE | \
                                     STM32_SDMMC_CLKCR_WIDBUS_D4)

/* Timing */

#define SDMMC_CMDTIMEOUT         (100000)
#define SDMMC_LONGTIMEOUT        (0x7fffffff)

/* Big DTIMER setting */

#define SDMMC_DTIMER_DATATIMEOUT (0x000fffff)

/* DMA channel/stream configuration register settings.  The following
 * must be selected.  The DMA driver will select the remaining fields.
 *
 * - 32-bit DMA
 * - Memory increment
 * - Direction (memory-to-peripheral, peripheral-to-memory)
 * - Memory burst size (F4 only)
 */

/* STM32 stream configuration register (SCR) settings base settings sans
 * priority.
 */

#define SDMMC_RXDMA32_CONFIG     (DMA_CCR_MINC | \
                                  DMA_CCR_PSIZE_32BITS | DMA_CCR_MSIZE_32BITS)
#define SDMMC_TXDMA32_CONFIG     (DMA_CCR_DIR | DMA_CCR_MINC | \
                                  DMA_CCR_PSIZE_32BITS | DMA_CCR_MSIZE_32BITS)

#ifdef DMAMAP_SDMMC1

/* SDMMC DMA Channel/Stream selection.  There
 * are multiple DMA stream options that must be dis-ambiguated in the board.h
 * file.
 */

#  define SDMMC1_DMACHAN          DMAMAP_SDMMC1
#endif

#ifdef DMAMAP_SDMMC2

/* SDMMC DMA Channel/Stream selection.  There
 * are multiple DMA stream options that must be dis-ambiguated in the board.h
 * file.
 */

#  define SDMMC2_DMACHAN           DMAMAP_SDMMC2
#endif

/* FIFO sizes */

#define SDMMC_HALFFIFO_WORDS      (8)
#define SDMMC_HALFFIFO_BYTES      (8*4)

/* Data transfer interrupt mask bits */

#define STM32_SDMMC_RECV_MASK     (STM32_SDMMC_MASK_DCRCFAILIE | \
                                   STM32_SDMMC_MASK_DTIMEOUTIE | \
                                   STM32_SDMMC_MASK_DATAENDIE  | \
                                   STM32_SDMMC_MASK_RXOVERRIE  | \
                                   STM32_SDMMC_MASK_RXFIFOHFIE | \
                                   STM32_SDMMC_MASK_STBITERRIE)

#define STM32_SDMMC_SEND_MASK     (STM32_SDMMC_MASK_DCRCFAILIE | \
                                   STM32_SDMMC_MASK_DTIMEOUTIE | \
                                   STM32_SDMMC_MASK_DATAENDIE  | \
                                   STM32_SDMMC_MASK_TXUNDERRIE | \
                                   STM32_SDMMC_MASK_TXFIFOHEIE | \
                                   STM32_SDMMC_MASK_STBITERRIE)

#define STM32_SDMMC_DMARECV_MASK  (STM32_SDMMC_MASK_DCRCFAILIE | \
                                   STM32_SDMMC_MASK_DTIMEOUTIE | \
                                   STM32_SDMMC_MASK_DATAENDIE  | \
                                   STM32_SDMMC_MASK_RXOVERRIE | \
                                   STM32_SDMMC_MASK_STBITERRIE)

#define STM32_SDMMC_DMASEND_MASK  (STM32_SDMMC_MASK_DCRCFAILIE | \
                                   STM32_SDMMC_MASK_DTIMEOUTIE | \
                                   STM32_SDMMC_MASK_DATAENDIE  | \
                                   STM32_SDMMC_MASK_TXUNDERRIE | \
                                   STM32_SDMMC_MASK_STBITERRIE)

/* Event waiting interrupt mask bits */

#define STM32_SDMMC_CMDDONE_STA   (STM32_SDMMC_STA_CMDSENT)

#define STM32_SDMMC_RESPDONE_STA  (STM32_SDMMC_STA_CTIMEOUT | \
                                   STM32_SDMMC_STA_CCRCFAIL | \
                                   STM32_SDMMC_STA_CMDREND)

#define STM32_SDMMC_XFRDONE_STA   (0)

#define STM32_SDMMC_CMDDONE_MASK  (STM32_SDMMC_MASK_CMDSENTIE)

#define STM32_SDMMC_RESPDONE_MASK (STM32_SDMMC_MASK_CCRCFAILIE | \
                                   STM32_SDMMC_MASK_CTIMEOUTIE | \
                                   STM32_SDMMC_MASK_CMDRENDIE)

#define STM32_SDMMC_XFRDONE_MASK  (0)

#define STM32_SDMMC_CMDDONE_ICR   (STM32_SDMMC_ICR_CMDSENTC | \
                                   STM32_SDMMC_ICR_DBCKENDC)

#define STM32_SDMMC_RESPDONE_ICR  (STM32_SDMMC_ICR_CTIMEOUTC | \
                                   STM32_SDMMC_ICR_CCRCFAILC | \
                                   STM32_SDMMC_ICR_CMDRENDC  | \
                                   STM32_SDMMC_ICR_DBCKENDC)

#define STM32_SDMMC_XFRDONE_ICR   (STM32_SDMMC_ICR_DATAENDC  | \
                                   STM32_SDMMC_ICR_DCRCFAILC | \
                                   STM32_SDMMC_ICR_DTIMEOUTC | \
                                   STM32_SDMMC_ICR_RXOVERRC  | \
                                   STM32_SDMMC_ICR_TXUNDERRC | \
                                   STM32_SDMMC_ICR_DBCKENDC)

#define STM32_SDMMC_WAITALL_ICR   (STM32_SDMMC_CMDDONE_ICR   | \
                                   STM32_SDMMC_RESPDONE_ICR  | \
                                   STM32_SDMMC_XFRDONE_ICR   | \
                                   STM32_SDMMC_ICR_DBCKENDC)

/* Let's wait until we have both SDIO transfer complete and DMA complete. */

#define SDMMC_XFRDONE_FLAG  (1)
#define SDMMC_DMADONE_FLAG  (2)
#define SDMMC_ALLDONE       (3)

/* Register logging support */

#ifdef CONFIG_STM32L4_SDMMC_XFRDEBUG
#  ifdef CONFIG_STM32L4_SDMMC_DMA
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

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the state of the STM32 SDMMC interface */

struct stm32_dev_s
{
  struct sdio_dev_s  dev;             /* Standard, base SDIO interface */

  /* STM32-specific extensions */

  uint32_t          base;
  int               nirq;
#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
  uint32_t          d0_gpio;
#endif
#ifdef CONFIG_STM32L4_SDMMC_DMA
  uint32_t          dmapri;
#endif

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

  /* DMA data transfer support */

  bool               widebus;         /* Required for DMA support */
  bool               onebit;          /* true: Only 1-bit transfers are supported */
#ifdef CONFIG_STM32L4_SDMMC_DMA
  volatile uint8_t   xfrflags;        /* Used to synchronize SDMMC and DMA completion events */
  bool               dmamode;         /* true: DMA mode transfer */
  DMA_HANDLE         dma;             /* Handle for DMA channel */
#endif
};

/* Register logging support */

#ifdef CONFIG_STM32L4_SDMMC_XFRDEBUG
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
#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_STM32L4_SDMMC_DMA)
  struct stm32_dmaregs_s  dma;
#endif
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static inline void sdmmc_putreg32(struct stm32_dev_s *priv, uint32_t value,
                                  int offset);
static inline uint32_t sdmmc_getreg32(struct stm32_dev_s *priv, int offset);
static int  stm32_takesem(struct stm32_dev_s *priv);
#define     stm32_givesem(priv) (nxsem_post(&priv->waitsem))
static inline void stm32_setclkcr(struct stm32_dev_s *priv, uint32_t clkcr);
static void stm32_configwaitints(struct stm32_dev_s *priv, uint32_t waitmask,
              sdio_eventset_t waitevents, sdio_eventset_t wkupevents);
static void stm32_configxfrints(struct stm32_dev_s *priv, uint32_t xfrmask);
static void stm32_setpwrctrl(struct stm32_dev_s *priv, uint32_t pwrctrl);
static inline uint32_t stm32_getpwrctrl(struct stm32_dev_s *priv);

/* DMA Helpers **************************************************************/

#ifdef CONFIG_STM32L4_SDMMC_XFRDEBUG
static void stm32_sampleinit(void);
static void stm32_sdiosample(struct stm32_dev_s *priv,
              struct stm32_sdioregs_s *regs);
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

#ifdef CONFIG_STM32L4_SDMMC_DMA
static void stm32_dmacallback(DMA_HANDLE handle, uint8_t status, void *arg);
#endif

/* Data Transfer Helpers ****************************************************/

static uint8_t stm32_log2(uint16_t value);
static void stm32_dataconfig(struct stm32_dev_s *priv, uint32_t timeout,
              uint32_t dlen, uint32_t dctrl);
static void stm32_datadisable(struct stm32_dev_s *priv);
static void stm32_sendfifo(struct stm32_dev_s *priv);
static void stm32_recvfifo(struct stm32_dev_s *priv);
static void stm32_eventtimeout(wdparm_t arg);
static void stm32_endwait(struct stm32_dev_s *priv,
              sdio_eventset_t wkupevent);
static void stm32_endtransfer(struct stm32_dev_s *priv,
              sdio_eventset_t wkupevent);

/* Interrupt Handling *******************************************************/

static int  stm32_sdmmc_interrupt(int irq, void *context, void *arg);
#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
static int  stm32_sdmmc_rdyinterrupt(int irq, void *context, void *arg);
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
static int  stm32_recvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
              size_t nbytes);
static int  stm32_sendsetup(FAR struct sdio_dev_s *dev,
              FAR const uint8_t *buffer, size_t nbytes);
static int  stm32_cancel(FAR struct sdio_dev_s *dev);

static int  stm32_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd);
static int  stm32_recvshortcrc(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  stm32_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t rlong[4]);
static int  stm32_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  stm32_recvnotimpl(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rnotimpl);

/* EVENT handler */

static void stm32_waitenable(FAR struct sdio_dev_s *dev,
              sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t stm32_eventwait(FAR struct sdio_dev_s *dev);
static void stm32_callbackenable(FAR struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static int  stm32_registercallback(FAR struct sdio_dev_s *dev,
              worker_t callback, void *arg);

/* DMA */

#ifdef CONFIG_STM32L4_SDMMC_DMA
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
static void stm32_default(struct stm32_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32L4_SDMMC1
struct stm32_dev_s g_sdmmcdev1 =
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
    .blocksetup       = stm32_blocksetup, /* Not implemented yet */
#endif
    .recvsetup        = stm32_recvsetup,
    .sendsetup        = stm32_sendsetup,
    .cancel           = stm32_cancel,
    .waitresponse     = stm32_waitresponse,
    .recv_r1          = stm32_recvshortcrc,
    .recv_r2          = stm32_recvlong,
    .recv_r3          = stm32_recvshort,
    .recv_r4          = stm32_recvnotimpl,
    .recv_r5          = stm32_recvnotimpl,
    .recv_r6          = stm32_recvshortcrc,
    .recv_r7          = stm32_recvshort,
    .waitenable       = stm32_waitenable,
    .eventwait        = stm32_eventwait,
    .callbackenable   = stm32_callbackenable,
#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
    .registercallback = stm32_registercallback,
#endif
#ifdef CONFIG_SDIO_DMA
#ifdef CONFIG_STM32L4_SDMMC_DMA
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
  .base              = STM32L4_SDMMC1_BASE,
  .nirq              = STM32L4_IRQ_SDMMC1,
#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
  .d0_gpio           = GPIO_SDMMC1_D0,
#endif
#ifdef CONFIG_STM32L4_SDMMC1_DMAPRIO
  .dmapri            = CONFIG_STM32L4_SDMMC1_DMAPRIO,
#endif
};
#endif
#ifdef CONFIG_STM32L4_SDMMC2
struct stm32_dev_s g_sdmmcdev2 =
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
    .blocksetup       = stm32_blocksetup, /* Not implemented yet */
#endif
    .recvsetup        = stm32_recvsetup,
    .sendsetup        = stm32_sendsetup,
    .cancel           = stm32_cancel,
    .waitresponse     = stm32_waitresponse,
    .recv_r1          = stm32_recvshortcrc,
    .recv_r2          = stm32_recvlong,
    .recv_r3          = stm32_recvshort,
    .recv_r4          = stm32_recvnotimpl,
    .recv_r5          = stm32_recvnotimpl,
    .recv_r6          = stm32_recvshortcrc,
    .recv_r7          = stm32_recvshort,
    .waitenable       = stm32_waitenable,
    .eventwait        = stm32_eventwait,
    .callbackenable   = stm32_callbackenable,
#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
    .registercallback = stm32_registercallback,
#endif
#ifdef CONFIG_SDIO_DMA
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
    .dmapreflight     = stm32_dmapreflight,
#endif
    .dmarecvsetup     = stm32_dmarecvsetup,
    .dmasendsetup     = stm32_dmasendsetup,
#endif
  },
  .base              = STM32_SDMMC2_BASE,
  .nirq              = STM32_IRQ_SDMMC2,
#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
  .d0_gpio           = GPIO_SDMMC2_D0,
#endif
#ifdef CONFIG_STM32L4_SDMMC2_DMAPRIO
  .dmapri            = CONFIG_STM32L4_SDMMC2_DMAPRIO,
#endif
};
#endif

/* Register logging support */

#ifdef CONFIG_STM32L4_SDMMC_XFRDEBUG
static struct stm32_sampleregs_s g_sampleregs[DEBUG_NSAMPLES];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sdmmc_putreg32
 ****************************************************************************/

static inline void sdmmc_putreg32(struct stm32_dev_s *priv, uint32_t value,
                                  int offset)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: sdmmc_gettreg32
 ****************************************************************************/

static inline uint32_t sdmmc_getreg32(struct stm32_dev_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: sdmmc_modifyreg32
 ****************************************************************************/

static inline void sdmmc_modifyreg32(struct stm32_dev_s *priv, int offset,
                                     uint32_t clearbits, uint32_t setbits)
{
  irqstate_t flags;
  int32_t   regval;

  flags   = enter_critical_section();
  regval  = getreg32(priv->base + offset);
  regval &= ~clearbits;
  regval |= setbits;
  putreg32(regval, priv->base + offset);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32_takesem
 *
 * Description:
 *   Take the wait semaphore (handling false alarm wakeups due to the receipt
 *   of signals).
 *
 * Input Parameters:
 *   priv  - Instance of the SDMMC private state structure.
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
 *   priv       - Instance of the SDMMC private state structure.
 *   clkcr - A new CLKCR setting for the above mentions bits (other bits
 *           are ignored.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_setclkcr(struct stm32_dev_s *priv, uint32_t clkcr)
{
  uint32_t regval = sdmmc_getreg32(priv, STM32_SDMMC_CLKCR_OFFSET);

  /* Clear CLKDIV, PWRSAV, BYPASS, WIDBUS, NEGEDGE, HWFC_EN bits */

  regval &= ~(STM32_SDMMC_CLKCR_CLKDIV_MASK | STM32_SDMMC_CLKCR_PWRSAV      |
              STM32_SDMMC_CLKCR_BYPASS      | STM32_SDMMC_CLKCR_WIDBUS_MASK |
              STM32_SDMMC_CLKCR_NEGEDGE     | STM32_SDMMC_CLKCR_HWFC_EN     |
              STM32_SDMMC_CLKCR_CLKEN);

  /* Replace with user provided settings */

  clkcr  &=  (STM32_SDMMC_CLKCR_CLKDIV_MASK | STM32_SDMMC_CLKCR_PWRSAV      |
              STM32_SDMMC_CLKCR_BYPASS      | STM32_SDMMC_CLKCR_WIDBUS_MASK |
              STM32_SDMMC_CLKCR_NEGEDGE     | STM32_SDMMC_CLKCR_HWFC_EN     |
              STM32_SDMMC_CLKCR_CLKEN);

  regval |=  clkcr;
  sdmmc_putreg32(priv, regval, STM32_SDMMC_CLKCR_OFFSET);

  mcinfo("CLKCR: %08x PWR: %08x\n",
        sdmmc_getreg32(priv, STM32_SDMMC_CLKCR_OFFSET),
        sdmmc_getreg32(priv, STM32_SDMMC_POWER_OFFSET));
}

/****************************************************************************
 * Name: stm32_configwaitints
 *
 * Description:
 *   Enable/disable SDIO interrupts needed to support the wait function
 *
 * Input Parameters:
 *   priv       - Instance of the SDMMC private state structure.
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
      /* Do not use this in STM32_SDMMC_MASK register */

      waitmask &= ~SDIOWAIT_WRCOMPLETE;

      pinset = priv->d0_gpio & (GPIO_PORT_MASK | GPIO_PIN_MASK);
      pinset |= (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI);

      /* Arm the SDMMC_D0 Ready and install ISR */

      stm32_gpiosetevent(pinset, true, false, false,
                         stm32_sdmmc_rdyinterrupt, priv);
    }

  /* Disarm SDMMC_D0 ready and return it to SDMMC D0 */

  if ((wkupevent & SDIOWAIT_WRCOMPLETE) != 0)
    {
      stm32_gpiosetevent(priv->d0_gpio, false, false, false,
                         NULL, NULL);
    }
#endif

  priv->waitevents = waitevents;
  priv->wkupevent  = wkupevent;
  priv->waitmask   = waitmask;
#ifdef CONFIG_STM32L4_SDMMC_DMA
  priv->xfrflags   = 0;
#endif
  sdmmc_putreg32(priv, priv->xfrmask | priv->waitmask,
                 STM32_SDMMC_MASK_OFFSET);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32_configxfrints
 *
 * Description:
 *   Enable SDIO interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - Instance of the SDMMC private state structure.
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
  sdmmc_putreg32(priv, priv->xfrmask | priv->waitmask,
                 STM32_SDMMC_MASK_OFFSET);
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
 *   priv  - Instance of the SDMMC private state structure.
 *   clkcr - A new PWRCTRL setting
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_setpwrctrl(struct stm32_dev_s *priv, uint32_t pwrctrl)
{
  uint32_t regval;

  regval  = sdmmc_getreg32(priv, STM32_SDMMC_POWER_OFFSET);
  regval &= ~STM32_SDMMC_POWER_PWRCTRL_MASK;
  regval |= pwrctrl;
  sdmmc_putreg32(priv, regval, STM32_SDMMC_POWER_OFFSET);
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
 *   priv  - Instance of the SDMMC private state structure.
 *
 * Returned Value:
 *   The current value of the  the PWRCTRL field of the SDIO POWER register.
 *
 ****************************************************************************/

static inline uint32_t stm32_getpwrctrl(struct stm32_dev_s *priv)
{
  return sdmmc_getreg32(priv, STM32_SDMMC_POWER_OFFSET) &
                        STM32_SDMMC_POWER_PWRCTRL_MASK;
}

/****************************************************************************
 * Name: stm32_sampleinit
 *
 * Description:
 *   Setup prior to collecting DMA samples
 *
 ****************************************************************************/

#ifdef CONFIG_STM32L4_SDMMC_XFRDEBUG
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

#ifdef CONFIG_STM32L4_SDMMC_XFRDEBUG
static void stm32_sdiosample(struct stm32_dev_s *priv,
                             struct stm32_sdioregs_s *regs)
{
  regs->power   = (uint8_t)sdmmc_getreg32(priv, STM32_SDMMC_POWER_OFFSET);
  regs->clkcr   = (uint16_t)sdmmc_getreg32(priv, STM32_SDMMC_CLKCR_OFFSET);
  regs->dctrl   = (uint16_t)sdmmc_getreg32(priv, STM32_SDMMC_DCTRL_OFFSET);
  regs->dtimer  = sdmmc_getreg32(priv, STM32_SDMMC_DTIMER_OFFSET);
  regs->dlen    = sdmmc_getreg32(priv, STM32_SDMMC_DLEN_OFFSET);
  regs->dcount  = sdmmc_getreg32(priv, STM32_SDMMC_DCOUNT_OFFSET);
  regs->sta     = sdmmc_getreg32(priv, STM32_SDMMC_STA_OFFSET);
  regs->mask    = sdmmc_getreg32(priv, STM32_SDMMC_MASK_OFFSET);
  regs->fifocnt = sdmmc_getreg32(priv, STM32_SDMMC_FIFOCNT_OFFSET);
}
#endif

/****************************************************************************
 * Name: stm32_sample
 *
 * Description:
 *   Sample SDIO/DMA registers
 *
 ****************************************************************************/

#ifdef CONFIG_STM32L4_SDMMC_XFRDEBUG
static void stm32_sample(struct stm32_dev_s *priv, int index)
{
  struct stm32_sampleregs_s *regs = &g_sampleregs[index];

#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_STM32L4_SDMMC_DMA)
  if (priv->dmamode)
    {
      stm32_dmasample(priv->dma, &regs->dma);
    }
#endif

  stm32_sdiosample(priv, &regs->sdio);
}
#endif

/****************************************************************************
 * Name: stm32_sdiodump
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#ifdef CONFIG_STM32L4_SDMMC_XFRDEBUG
static void stm32_sdiodump(struct stm32_sdioregs_s *regs, const char *msg)
{
  mcinfo("SDIO Registers: %s\n", msg);
  mcinfo("  POWER[%08x]: %08x\n", STM32_SDMMC_POWER_OFFSET,   regs->power);
  mcinfo("  CLKCR[%08x]: %08x\n", STM32_SDMMC_CLKCR_OFFSET,   regs->clkcr);
  mcinfo("  DCTRL[%08x]: %08x\n", STM32_SDMMC_DCTRL_OFFSET,   regs->dctrl);
  mcinfo(" DTIMER[%08x]: %08x\n", STM32_SDMMC_DTIMER_OFFSET,  regs->dtimer);
  mcinfo("   DLEN[%08x]: %08x\n", STM32_SDMMC_DLEN_OFFSET,    regs->dlen);
  mcinfo(" DCOUNT[%08x]: %08x\n", STM32_SDMMC_DCOUNT_OFFSET,  regs->dcount);
  mcinfo("    STA[%08x]: %08x\n", STM32_SDMMC_STA_OFFSET,     regs->sta);
  mcinfo("   MASK[%08x]: %08x\n", STM32_SDMMC_MASK_OFFSET,    regs->mask);
  mcinfo("FIFOCNT[%08x]: %08x\n", STM32_SDMMC_FIFOCNT_OFFSET, regs->fifocnt);
}
#endif

/****************************************************************************
 * Name: stm32_dumpsample
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#ifdef CONFIG_STM32L4_SDMMC_XFRDEBUG
static void stm32_dumpsample(struct stm32_dev_s *priv,
                             struct stm32_sampleregs_s *regs,
                             const char *msg)
{
#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_STM32L4_SDMMC_DMA)
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

#ifdef CONFIG_STM32L4_SDMMC_XFRDEBUG
static void stm32_dumpsamples(struct stm32_dev_s *priv)
{
  stm32_dumpsample(priv, &g_sampleregs[SAMPLENDX_BEFORE_SETUP],
                   "Before setup");

#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_STM32L4_SDMMC_DMA)
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

#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_STM32L4_SDMMC_DMA)
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

#ifdef CONFIG_STM32L4_SDMMC_DMA
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
      mcerr("DMA error %02x, remaining: %d\n", status, priv->remaining);
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

  priv->xfrflags |= SDMMC_DMADONE_FLAG;
  if (priv->xfrflags == SDMMC_ALLDONE || result == SDIOWAIT_ERROR)
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

static void stm32_dataconfig(struct stm32_dev_s *priv, uint32_t timeout,
                             uint32_t dlen, uint32_t dctrl)
{
  uint32_t regval = 0;

  /* Enable data path */

  sdmmc_putreg32(priv, timeout, STM32_SDMMC_DTIMER_OFFSET); /* Set DTIMER */
  sdmmc_putreg32(priv, dlen,    STM32_SDMMC_DLEN_OFFSET);   /* Set DLEN */

  /* Configure DCTRL DTDIR, DTMODE, and DBLOCKSIZE fields and set the DTEN
   * field
   */

  regval  =  sdmmc_getreg32(priv, STM32_SDMMC_DCTRL_OFFSET);
  regval &= ~(STM32_SDMMC_DCTRL_DTDIR | STM32_SDMMC_DCTRL_DTMODE |
              STM32_SDMMC_DCTRL_DBLOCKSIZE_MASK);
  dctrl  &=  (STM32_SDMMC_DCTRL_DTDIR | STM32_SDMMC_DCTRL_DTMODE |
              STM32_SDMMC_DCTRL_DBLOCKSIZE_MASK);
  regval |=  (dctrl | STM32_SDMMC_DCTRL_DTEN);
  sdmmc_putreg32(priv, regval, STM32_SDMMC_DCTRL_OFFSET);
}

/****************************************************************************
 * Name: stm32_datadisable
 *
 * Description:
 *   Disable the SDIO data path setup by stm32_dataconfig() and
 *   disable DMA.
 *
 ****************************************************************************/

static void stm32_datadisable(struct stm32_dev_s *priv)
{
  uint32_t regval;

  /* Disable the data path */

  /* Reset DTIMER */

  sdmmc_putreg32(priv, SDMMC_DTIMER_DATATIMEOUT, STM32_SDMMC_DTIMER_OFFSET);
  sdmmc_putreg32(priv, 0, STM32_SDMMC_DLEN_OFFSET);   /* Reset DLEN */

  /* Reset DCTRL DTEN, DTDIR, DTMODE, DMAEN, and DBLOCKSIZE fields */

  regval  = sdmmc_getreg32(priv, STM32_SDMMC_DCTRL_OFFSET);
  regval &= ~(STM32_SDMMC_DCTRL_DTEN   | STM32_SDMMC_DCTRL_DTDIR |
              STM32_SDMMC_DCTRL_DTMODE | STM32_SDMMC_DCTRL_DMAEN |
              STM32_SDMMC_DCTRL_DBLOCKSIZE_MASK);
  sdmmc_putreg32(priv, regval, STM32_SDMMC_DCTRL_OFFSET);
}

/****************************************************************************
 * Name: stm32_sendfifo
 *
 * Description:
 *   Send SDIO data in interrupt mode
 *
 * Input Parameters:
 *   priv  - Instance of the SDMMC private state structure.
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
         (sdmmc_getreg32(priv, STM32_SDMMC_STA_OFFSET) &
          STM32_SDMMC_STA_TXFIFOF) == 0)
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

     sdmmc_putreg32(priv, data.w, STM32_SDMMC_FIFO_OFFSET);
    }
}

/****************************************************************************
 * Name: stm32_recvfifo
 *
 * Description:
 *   Receive SDIO data in interrupt mode
 *
 * Input Parameters:
 *   priv  - Instance of the SDMMC private state structure.
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
         (sdmmc_getreg32(priv, STM32_SDMMC_STA_OFFSET) &
          STM32_SDMMC_STA_RXDAVL) != 0)
    {
      /* Read the next word from the RX FIFO */

      data.w = sdmmc_getreg32(priv, STM32_SDMMC_FIFO_OFFSET);
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
      mcerr("Timeout: remaining: %d\n", priv->remaining);
    }
}

/****************************************************************************
 * Name: stm32_endwait
 *
 * Description:
 *   Wake up a waiting thread if the waited-for event has occurred.
 *
 * Input Parameters:
 *   priv  - Instance of the SDMMC private state structure.
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
 *   priv  - Instance of the SDMMC private state structure.
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

  sdmmc_putreg32(priv, STM32_SDMMC_XFRDONE_ICR, STM32_SDMMC_ICR_OFFSET);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_STM32L4_SDMMC_DMA
  if (priv->dmamode)
    {
      /* DMA debug instrumentation */

      stm32_sample(priv, SAMPLENDX_END_TRANSFER);

      /* Make sure that the DMA is stopped (it will be stopped automatically
       * on normal transfers, but not necessarily when the transfer
       * terminates on an error condition).
       */

      stm32l4_dmastop(priv->dma);
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
 * Name: stm32_sdmmc[1|2]_rdyinterrupt
 *
 * Description:
 *   SDMMC ready interrupt handler
 *
 * Input Parameters:
 *   irq     - not used
 *   context - not used
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
static int stm32_sdmmc_rdyinterrupt(int irq, void *context, void *arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)arg;

  stm32_endwait(priv, SDIOWAIT_WRCOMPLETE);
  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_sdmmc_interrupt
 *
 * Description:
 *   SDMMC interrupt handler
 *
 * Input Parameters:
 *   priv  - Instance of the SDMMC private state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int stm32_sdmmc_interrupt(int irq, void *context, void *arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)arg;
  uint32_t enabled;
  uint32_t pending;

  DEBUGASSERT(priv != NULL);

  /* Loop while there are pending interrupts.  Check the SDIO status
   * register.  Mask out all bits that don't correspond to enabled
   * interrupts.  (This depends on the fact that bits are ordered
   * the same in both the STA and MASK register).  If there are non-zero
   * bits remaining, then we have work to do here.
   */

  while ((enabled = sdmmc_getreg32(priv, STM32_SDMMC_STA_OFFSET) &
          sdmmc_getreg32(priv, STM32_SDMMC_MASK_OFFSET)) != 0)
    {
      /* Handle in progress, interrupt driven data transfers ****************/

      pending  = enabled & priv->xfrmask;
      if (pending != 0)
        {
#ifdef CONFIG_STM32L4_SDMMC_DMA
          if (!priv->dmamode)
#endif
            {
              /* Is the RX FIFO half full or more?  Is so then we must be
               * processing a receive transaction.
               */

              if ((pending & STM32_SDMMC_STA_RXFIFOHF) != 0)
                {
                  /* Receive data from the RX FIFO */

                  stm32_recvfifo(priv);
                }

              /* Otherwise, Is the transmit FIFO half empty or less?  If so
               * we must be processing a send transaction.  NOTE:  We can't
               * be processing both!
               */

              else if ((pending & STM32_SDMMC_STA_TXFIFOHE) != 0)
                {
                  /* Send data via the TX FIFO */

                  stm32_sendfifo(priv);
                }
            }

          /* Handle data end events */

          if ((pending & STM32_SDMMC_STA_DATAEND) != 0)
            {
              /* Handle any data remaining the RX FIFO.  If the RX FIFO is
               * less than half full at the end of the transfer, then no
               * half-full interrupt will be received.
               */

              /* Was this transfer performed in DMA mode? */

#ifdef CONFIG_STM32L4_SDMMC_DMA
              if (priv->dmamode)
                {
                  /* Yes.. Terminate the transfers only if the DMA has also
                   * finished.
                   */

                  priv->xfrflags |= SDMMC_XFRDONE_FLAG;
                  if (priv->xfrflags == SDMMC_ALLDONE)
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

          else if ((pending & STM32_SDMMC_STA_DCRCFAIL) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Data block CRC failure, remaining: %d\n",
                    priv->remaining);
              stm32_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle data timeout error */

          else if ((pending & STM32_SDMMC_STA_DTIMEOUT) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Data timeout, remaining: %d\n",
                    priv->remaining);
              stm32_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT);
            }

          /* Handle RX FIFO overrun error */

          else if ((pending & STM32_SDMMC_STA_RXOVERR) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: RX FIFO overrun, remaining: %d\n",
                    priv->remaining);
              stm32_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle TX FIFO underrun error */

          else if ((pending & STM32_SDMMC_STA_TXUNDERR) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: TX FIFO underrun, remaining: %d\n",
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

          if ((pending & STM32_SDMMC_RESPDONE_STA) != 0)
            {
              /* Yes.. Is their a thread waiting for response done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                 sdmmc_putreg32(priv, STM32_SDMMC_RESPDONE_ICR |
                                STM32_SDMMC_CMDDONE_ICR,
                                STM32_SDMMC_ICR_OFFSET);
                  stm32_endwait(priv, SDIOWAIT_RESPONSEDONE);
                }
            }

          /* Is this a command completion event? */

          if ((pending & STM32_SDMMC_CMDDONE_STA) != 0)
            {
              /* Yes.. Is their a thread waiting for command done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                 sdmmc_putreg32(priv, STM32_SDMMC_CMDDONE_ICR,
                                STM32_SDMMC_ICR_OFFSET);
                  stm32_endwait(priv, SDIOWAIT_CMDDONE);
                }
            }
        }
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
  /* The multiplex bus is part of board support package. */

  stm32_muxbus_sdio_lock(dev, lock);
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
  sdmmc_modifyreg32(priv, STM32_SDMMC_CLKCR_OFFSET,
                    STM32_SDMMC_CLKCR_CLKEN, 0);
  stm32_setpwrctrl(priv, STM32_SDMMC_POWER_PWRCTRL_OFF);

  /* Put SDIO registers in their default, reset state */

  stm32_default(priv);

  /* Reset data */

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitmask   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */
#ifdef CONFIG_STM32L4_SDMMC_DMA
  priv->xfrflags   = 0;      /* Used to synchronize SDIO and DMA
                              * completion events */
#endif

  wd_cancel(&priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->buffer     = 0;      /* Address of current R/W buffer */
  priv->remaining  = 0;      /* Number of bytes remaining in the transfer */
  priv->xfrmask    = 0;      /* Interrupt enables for data transfer */

  /* DMA data transfer support */

  priv->widebus    = false;  /* Required for DMA support */
#ifdef CONFIG_STM32L4_SDMMC_DMA
  priv->dmamode    = false;  /* true: DMA mode transfer */
#endif

  /* Configure the SDIO peripheral */

  stm32_setclkcr(priv, STM32_CLCKCR_INIT | STM32_SDMMC_CLKCR_CLKEN);
  stm32_setpwrctrl(priv, STM32_SDMMC_POWER_PWRCTRL_ON);
  leave_critical_section(flags);

  mcinfo("CLCKR: %08x POWER: %08x\n",
        sdmmc_getreg32(priv, STM32_SDMMC_CLKCR_OFFSET),
        sdmmc_getreg32(priv, STM32_SDMMC_POWER_OFFSET));
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
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  sdio_capset_t caps = 0;

  if (priv->onebit)
    {
      caps |= SDIO_CAPS_1BIT_ONLY;
    }

#ifdef CONFIG_STM32L4_SDMMC_DMA
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
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
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
        clckr = (STM32_CLCKCR_INIT | STM32_SDMMC_CLKCR_CLKEN);
        break;

      /* Enable in MMC normal operation clocking */

      case CLOCK_MMC_TRANSFER:
        clckr = (STM32_SDMMC_CLKCR_MMCXFR | STM32_SDMMC_CLKCR_CLKEN);
        break;

      /* SD normal operation clocking (wide 4-bit mode) */

      case CLOCK_SD_TRANSFER_4BIT:
        if (!priv->onebit)
          {
            clckr = (STM32_SDMMC_CLCKR_SDWIDEXFR | STM32_SDMMC_CLKCR_CLKEN);
            break;
          }

      /* SD normal operation clocking (narrow 1-bit mode) */

      case CLOCK_SD_TRANSFER_1BIT:
        clckr = (STM32_SDMMC_CLCKR_SDXFR | STM32_SDMMC_CLKCR_CLKEN);
        break;
    }

  /* Set the new clock frequency along with the clock enable/disable bit */

  stm32_setclkcr(priv, clckr);
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
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  int ret;

  /* Attach the SDIO interrupt handler */

  ret = irq_attach(priv->nirq, stm32_sdmmc_interrupt, priv);
  if (ret == OK)
    {
      /* Disable all interrupts at the SDIO controller and clear static
       * interrupt flags
       */

     sdmmc_putreg32(priv, STM32_SDMMC_MASK_RESET,
                    STM32_SDMMC_MASK_OFFSET);
     sdmmc_putreg32(priv, STM32_SDMMC_ICR_STATICFLAGS,
                    STM32_SDMMC_ICR_OFFSET);

      /* Enable SDIO interrupts at the NVIC.  They can now be enabled at
       * the SDIO controller as needed.
       */

      up_enable_irq(priv->nirq);
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
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t regval;
  uint32_t cmdidx;

  /* Set the SDIO Argument value */

  sdmmc_putreg32(priv, arg, STM32_SDMMC_ARG_OFFSET);

  /* Clear CMDINDEX, WAITRESP, WAITINT, WAITPEND, and CPSMEN bits */

  regval = sdmmc_getreg32(priv, STM32_SDMMC_CMD_OFFSET);
  regval &= ~(STM32_SDMMC_CMD_CMDINDEX_MASK | STM32_SDMMC_CMD_WAITRESP_MASK |
              STM32_SDMMC_CMD_WAITINT       | STM32_SDMMC_CMD_WAITPEND      |
              STM32_SDMMC_CMD_CPSMEN);

  /* Set WAITRESP bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      regval |= STM32_SDMMC_CMD_NORESPONSE;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
    case MMCSD_R7_RESPONSE:
      regval |= STM32_SDMMC_CMD_SHORTRESPONSE;
      break;

    case MMCSD_R2_RESPONSE:
      regval |= STM32_SDMMC_CMD_LONGRESPONSE;
      break;
    }

  /* Set CPSMEN and the command index */

  cmdidx  = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval |= cmdidx | STM32_SDMMC_CMD_CPSMEN;

  mcinfo("cmd: %08x arg: %08x regval: %08x\n", cmd, arg, regval);

  /* Write the SDIO CMD */

  sdmmc_putreg32(priv, STM32_SDMMC_RESPDONE_ICR | STM32_SDMMC_CMDDONE_ICR,
                 STM32_SDMMC_ICR_OFFSET);
  sdmmc_putreg32(priv, regval, STM32_SDMMC_CMD_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: stm32_recvsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card in non-DMA
 *   (interrupt driven mode).  This method will do whatever controller setup
 *   is necessary.  This would be called for SD memory just BEFORE sending
 *   CMD13 (SEND_STATUS), CMD17 (READ_SINGLE_BLOCK), CMD18
 *   (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR), etc.  Normally,
 *   SDMMC_WAITEVENT will be called to receive the indication that the
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

  stm32_datadisable(priv);
  stm32_sampleinit();
  stm32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
#ifdef CONFIG_STM32L4_SDMMC_DMA
  priv->dmamode   = false;
#endif

  /* Then set up the SDIO data path */

  dblocksize = stm32_log2(nbytes) << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT;
  stm32_dataconfig(priv, SDMMC_DTIMER_DATATIMEOUT, nbytes, dblocksize |
                   STM32_SDMMC_DCTRL_DTDIR);

  /* And enable interrupts */

  stm32_configxfrints(priv, STM32_SDMMC_RECV_MASK);
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
 *   (WRITE_MULTIPLE_BLOCK), ... and before SDMMC_SENDDATA is called.
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

static int stm32_sendsetup(FAR struct sdio_dev_s *dev, FAR const
                           uint8_t *buffer, size_t nbytes)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  stm32_datadisable(priv);
  stm32_sampleinit();
  stm32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
#ifdef CONFIG_STM32L4_SDMMC_DMA
  priv->dmamode   = false;
#endif

  /* Then set up the SDIO data path */

  dblocksize = stm32_log2(nbytes) << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT;
  stm32_dataconfig(priv, SDMMC_DTIMER_DATATIMEOUT, nbytes, dblocksize);

  /* Enable TX interrupts */

  stm32_configxfrints(priv, STM32_SDMMC_SEND_MASK);
  stm32_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}

/****************************************************************************
 * Name: stm32_cancel
 *
 * Description:
 *   Cancel the data transfer setup of SDMMC_RECVSETUP, SDMMC_SENDSETUP,
 *   SDMMC_DMARECVSETUP or SDMMC_DMASENDSETUP.  This must be called to cancel
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

  sdmmc_putreg32(priv, STM32_SDMMC_WAITALL_ICR, STM32_SDMMC_ICR_OFFSET);

  /* Cancel any watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_STM32L4_SDMMC_DMA
  if (priv->dmamode)
    {
      /* Make sure that the DMA is stopped (it will be stopped automatically
       * on normal transfers, but not necessarily when the transfer
       * terminates on an error condition.
       */

      stm32l4_dmastop(priv->dma);
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
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  int32_t timeout;
  uint32_t events;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      events  = STM32_SDMMC_CMDDONE_STA;
      timeout = SDMMC_CMDTIMEOUT;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R6_RESPONSE:
      events  = STM32_SDMMC_RESPDONE_STA;
      timeout = SDMMC_LONGTIMEOUT;
      break;

    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
      return -ENOSYS;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      events  = STM32_SDMMC_RESPDONE_STA;
      timeout = SDMMC_CMDTIMEOUT;
      break;

    default:
      return -EINVAL;
    }

  /* Then wait for the response (or timeout) */

  while ((sdmmc_getreg32(priv, STM32_SDMMC_STA_OFFSET) & events) == 0)
    {
      if (--timeout <= 0)
        {
           mcerr("ERROR: Timeout cmd: %08x events: %08x STA: %08x\n",
               cmd, events, sdmmc_getreg32(priv, STM32_SDMMC_STA_OFFSET));

          return -ETIMEDOUT;
        }
    }

  sdmmc_putreg32(priv, STM32_SDMMC_CMDDONE_ICR, STM32_SDMMC_ICR_OFFSET);
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
 *   assures that the response is returned intact and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int stm32_recvshortcrc(FAR struct sdio_dev_s *dev, uint32_t cmd,
                              uint32_t *rshort)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
#ifdef CONFIG_DEBUG_FEATURES
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
      mcerr("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = sdmmc_getreg32(priv, STM32_SDMMC_STA_OFFSET);
      if ((regval & STM32_SDMMC_STA_CTIMEOUT) != 0)
        {
          mcerr("ERROR: Command timeout: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & STM32_SDMMC_STA_CCRCFAIL) != 0)
        {
          mcerr("ERROR: CRC failure: %08x\n", regval);
          ret = -EIO;
        }
#ifdef CONFIG_DEBUG_FEATURES
      else
        {
          /* Check response received is of desired command */

          respcmd = sdmmc_getreg32(priv, STM32_SDMMC_RESPCMD_OFFSET);
          if ((uint8_t)(respcmd & STM32_SDMMC_RESPCMD_MASK) !=
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

  sdmmc_putreg32(priv, STM32_SDMMC_RESPDONE_ICR | STM32_SDMMC_CMDDONE_ICR,
                 STM32_SDMMC_ICR_OFFSET);
  *rshort = sdmmc_getreg32(priv, STM32_SDMMC_RESP1_OFFSET);
  return ret;
}

static int stm32_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd,
                          uint32_t rlong[4])
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
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

#ifdef CONFIG_DEBUG_FEATURES
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

      regval = sdmmc_getreg32(priv, STM32_SDMMC_STA_OFFSET);
      if (regval & STM32_SDMMC_STA_CTIMEOUT)
        {
          mcerr("ERROR: Timeout STA: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & STM32_SDMMC_STA_CCRCFAIL)
        {
          mcerr("ERROR: CRC fail STA: %08x\n", regval);
          ret = -EIO;
        }
    }

  /* Return the long response */

  sdmmc_putreg32(priv, STM32_SDMMC_RESPDONE_ICR | STM32_SDMMC_CMDDONE_ICR,
                 STM32_SDMMC_ICR_OFFSET);
  if (rlong)
    {
      rlong[0] = sdmmc_getreg32(priv, STM32_SDMMC_RESP1_OFFSET);
      rlong[1] = sdmmc_getreg32(priv, STM32_SDMMC_RESP2_OFFSET);
      rlong[2] = sdmmc_getreg32(priv, STM32_SDMMC_RESP3_OFFSET);
      rlong[3] = sdmmc_getreg32(priv, STM32_SDMMC_RESP4_OFFSET);
    }

  return ret;
}

static int stm32_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t *rshort)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
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

#ifdef CONFIG_DEBUG_FEATURES
  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
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

      regval = sdmmc_getreg32(priv, STM32_SDMMC_STA_OFFSET);
      if (regval & STM32_SDMMC_STA_CTIMEOUT)
        {
          mcerr("ERROR: Timeout STA: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
    }

  sdmmc_putreg32(priv, STM32_SDMMC_RESPDONE_ICR | STM32_SDMMC_CMDDONE_ICR,
                 STM32_SDMMC_ICR_OFFSET);
  if (rshort)
    {
      *rshort = sdmmc_getreg32(priv, STM32_SDMMC_RESP1_OFFSET);
    }

  return ret;
}

/* MMC responses not supported */

static int stm32_recvnotimpl(FAR struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t *rnotimpl)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  sdmmc_putreg32(priv, STM32_SDMMC_RESPDONE_ICR | STM32_SDMMC_CMDDONE_ICR,
                 STM32_SDMMC_ICR_OFFSET);
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the SDMMC_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling stm32_eventwait.  This is done in this way
 *   to help the driver to eliminate race conditions between the command
 *   setup and the subsequent events.
 *
 *   The enabled events persist until either (1) SDMMC_WAITENABLE is called
 *   again specifying a different set of wait events, or (2) SDMMC_EVENTWAIT
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
                             sdio_eventset_t eventset, uint32_t timeout)
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
          waitmask |= STM32_SDMMC_CMDDONE_MASK;
        }

      if ((eventset & SDIOWAIT_RESPONSEDONE) != 0)
        {
          waitmask |= STM32_SDMMC_RESPDONE_MASK;
        }

      if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
        {
          waitmask |= STM32_SDMMC_XFRDONE_MASK;
        }

      /* Enable event-related interrupts */

     sdmmc_putreg32(priv, STM32_SDMMC_WAITALL_ICR, STM32_SDMMC_ICR_OFFSET);
    }

  stm32_configwaitints(priv, waitmask, eventset, 0);

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
                       stm32_eventtimeout, (wdparm_t)priv);
      if (ret < 0)
        {
          mcerr("ERROR: wd_start failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: stm32_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDMMC_WAITEVENTS are disabled when stm32_eventwait
 *   returns.  SDMMC_WAITEVENTS must be called again before stm32_eventwait
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

static sdio_eventset_t stm32_eventwait(FAR struct sdio_dev_s *dev)
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

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  if ((priv->waitevents & SDIOWAIT_WRCOMPLETE) != 0)
    {
      /* Atomically read pin to see if ready (true) and determine if ISR
       * fired.  If Pin is ready and if ISR did NOT fire end the wait here
       */

      if (stm32_gpioread(priv->d0_gpio) &&
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
#ifdef CONFIG_STM32L4_SDMMC_DMA
  priv->xfrflags   = 0;
#endif

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
 *   callbacks to the function provided in stm32_callback.
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
 *   are enabled via a call to SDMMC_CALLBACKENABLE
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

#if defined(CONFIG_STM32L4_SDMMC_DMA) && defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
static int stm32_dmapreflight(FAR struct sdio_dev_s *dev,
                              FAR const uint8_t *buffer, size_t buflen)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);

  /* DMA must be possible to the buffer */

  if (!stm32l4_dmacapable((uintptr_t)buffer, (buflen + 3) >> 2,
                        SDMMC_RXDMA32_CONFIG | priv->dmapri))
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
 *   Setup to perform a read DMA.
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

#ifdef CONFIG_STM32L4_SDMMC_DMA
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

  stm32_datadisable(priv);

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

  dblocksize = stm32_log2(buflen) << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT;
  stm32_dataconfig(priv, SDMMC_DTIMER_DATATIMEOUT, buflen, dblocksize |
                   STM32_SDMMC_DCTRL_DTDIR);

  /* Configure the RX DMA */

  stm32_configxfrints(priv, STM32_SDMMC_DMARECV_MASK);

  sdmmc_modifyreg32(priv, STM32_SDMMC_DCTRL_OFFSET, 0,
                    STM32_SDMMC_DCTRL_DMAEN);
  stm32l4_dmasetup(priv->dma, priv->base + STM32_SDMMC_FIFO_OFFSET,
                   (uint32_t)buffer, (buflen + 3) >> 2,
                   SDMMC_RXDMA32_CONFIG | priv->dmapri);

  /* Start the DMA */

  stm32_sample(priv, SAMPLENDX_BEFORE_ENABLE);
  stm32l4_dmastart(priv->dma, stm32_dmacallback, priv, false);
  stm32_sample(priv, SAMPLENDX_AFTER_SETUP);

  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_dmasendsetup
 *
 * Description:
 *   Setup to perform a write DMA.
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

#ifdef CONFIG_STM32L4_SDMMC_DMA
static int stm32_dmasendsetup(FAR struct sdio_dev_s *dev,
                              FAR const uint8_t *buffer, size_t buflen)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
  DEBUGASSERT(stm32_dmapreflight(dev, buffer, buflen) == 0);
#else
#endif

  /* Reset the DPSM configuration */

  stm32_datadisable(priv);

  /* Initialize register sampling */

  stm32_sampleinit();
  stm32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->dmamode   = true;

  /* Then set up the SDIO data path */

  dblocksize = stm32_log2(buflen) << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT;
  stm32_dataconfig(priv, SDMMC_DTIMER_DATATIMEOUT, buflen, dblocksize);

  /* Configure the TX DMA */

  stm32l4_dmasetup(priv->dma, priv->base + STM32_SDMMC_FIFO_OFFSET,
                   (uint32_t)buffer, (buflen + 3) >> 2,
                   SDMMC_TXDMA32_CONFIG | priv->dmapri);

  sdmmc_modifyreg32(priv, STM32_SDMMC_DCTRL_OFFSET, 0,
                    STM32_SDMMC_DCTRL_DMAEN);
  stm32_sample(priv, SAMPLENDX_BEFORE_ENABLE);

  /* Start the DMA */

  stm32l4_dmastart(priv->dma, stm32_dmacallback, priv, false);
  stm32_sample(priv, SAMPLENDX_AFTER_SETUP);

  /* Enable TX interrupts */

  stm32_configxfrints(priv, STM32_SDMMC_DMASEND_MASK);

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

          mcinfo("Callback to %p(%p)\n",
                 priv->callback, priv->cbarg);
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

static void stm32_default(struct stm32_dev_s *priv)
{
  sdmmc_putreg32(priv, STM32_SDMMC_POWER_RESET,  STM32_SDMMC_POWER_OFFSET);
  sdmmc_putreg32(priv, STM32_SDMMC_CLKCR_RESET,  STM32_SDMMC_CLKCR_OFFSET);
  sdmmc_putreg32(priv, STM32_SDMMC_ARG_RESET,    STM32_SDMMC_ARG_OFFSET);
  sdmmc_putreg32(priv, STM32_SDMMC_CMD_RESET,    STM32_SDMMC_CMD_OFFSET);
  sdmmc_putreg32(priv, STM32_SDMMC_DTIMER_RESET, STM32_SDMMC_DTIMER_OFFSET);
  sdmmc_putreg32(priv, STM32_SDMMC_DLEN_RESET,   STM32_SDMMC_DLEN_OFFSET);
  sdmmc_putreg32(priv, STM32_SDMMC_DCTRL_RESET,  STM32_SDMMC_DCTRL_OFFSET);
  sdmmc_putreg32(priv, STM32_SDMMC_ICR_RESET,    STM32_SDMMC_ICR_OFFSET);
  sdmmc_putreg32(priv, STM32_SDMMC_MASK_RESET,   STM32_SDMMC_MASK_OFFSET);
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
  struct stm32_dev_s *priv = NULL;
#ifdef CONFIG_STM32L4_SDMMC_DMA
  unsigned int dmachan;
#endif

#ifdef CONFIG_STM32L4_SDMMC1
  if (slotno == 0)
    {
      /* Select SDMMC 1 */

      priv = &g_sdmmcdev1;

#ifdef CONFIG_STM32L4_SDMMC_DMA
      dmachan = SDMMC1_DMACHAN;
#endif

#ifdef CONFIG_SDMMC1_WIDTH_D1_ONLY
      priv->onebit = true;
#else
      priv->onebit = false;
#endif

      /* Configure GPIOs for 4-bit, wide-bus operation (the chip is capable
       * of 8-bit wide bus operation but D4-D7 are not configured).
       *
       * If bus is multiplexed then there is a custom bus configuration
       * utility in the scope of the board support package.
       */
#ifndef CONFIG_SDIO_MUXBUS
      stm32l4_configgpio(GPIO_SDMMC1_D0);
#ifndef CONFIG_SDMMC1_WIDTH_D1_ONLY
      stm32l4_configgpio(GPIO_SDMMC1_D1);
      stm32l4_configgpio(GPIO_SDMMC1_D2);
      stm32l4_configgpio(GPIO_SDMMC1_D3);
#endif
      stm32l4_configgpio(GPIO_SDMMC1_CK);
      stm32l4_configgpio(GPIO_SDMMC1_CMD);
#endif
    }
  else
#endif
#ifdef CONFIG_STM32L4_SDMMC2
  if (slotno == 1)
    {
      /* Select SDMMC 2 */

      priv = &g_sdmmcdev2;

#ifdef CONFIG_STM32L4_SDMMC_DMA
      dmachan = SDMMC2_DMACHAN;
#endif

#ifdef CONFIG_SDMMC2_WIDTH_D1_ONLY
      priv->onebit = true;
#else
      priv->onebit = false;
#endif

      /* Configure GPIOs for 4-bit, wide-bus operation (the chip is capable
       * of 8-bit wide bus operation but D4-D7 are not configured).
       *
       * If bus is multiplexed then there is a custom bus configuration
       * utility in the scope of the board support package.
       */

#ifndef CONFIG_SDIO_MUXBUS
      stm32_configgpio(GPIO_SDMMC2_D0);
#ifndef CONFIG_SDMMC2_WIDTH_D1_ONLY
      stm32_configgpio(GPIO_SDMMC2_D1);
      stm32_configgpio(GPIO_SDMMC2_D2);
      stm32_configgpio(GPIO_SDMMC2_D3);
#endif
      stm32_configgpio(GPIO_SDMMC2_CK);
      stm32_configgpio(GPIO_SDMMC2_CMD);
#endif
    }
  else
#endif
    {
      mcerr("ERROR: Unsupported SDMMC slot: %d\n", slotno);
      return NULL;
    }

  /* Initialize the SDIO slot structure */

  /* Initialize semaphores */

  nxsem_init(&priv->waitsem, 0, 0);

  /* The waitsem semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_set_protocol(&priv->waitsem, SEM_PRIO_NONE);

#ifdef CONFIG_STM32L4_SDMMC_DMA
  /* Allocate a DMA channel */

  priv->dma = stm32l4_dmachannel(dmachan);
  DEBUGASSERT(priv->dma);
#endif

  /* Reset the card and assure that it is in the initial, unconfigured
   * state.
   */

  stm32_reset(&priv->dev);
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
#endif /* CONFIG_STM32L4_SDMMC1 || CONFIG_STM32L4_SDMMC2 */
