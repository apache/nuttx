/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_sdcard.c
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
#include "lpc17_40_gpdma.h"
#include "lpc17_40_gpio.h"
#include "lpc17_40_sdcard.h"

#include "hardware/lpc17_40_syscon.h"
#include "hardware/lpc17_40_pinconfig.h"

#ifdef CONFIG_LPC17_40_SDCARD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Required system configuration options:
 *
 *   CONFIG_ARCH_DMA - Enable architecture-specific DMA subsystem
 *     initialization.  Required if CONFIG_LPC17_40_SDCARD_DMA is enabled.
 *   CONFIG_LPC17_40_GPDMA - Enable LPC17XX_40XX GPDMA support.  Required if
 *     CONFIG_LPC17_40_SDCARD_DMA is enabled
 *   CONFIG_SCHED_WORKQUEUE -- Callback support requires work queue support.
 *
 * Driver-specific configuration options:
 *
 *   CONFIG_SDIO_MUXBUS - Setting this configuration enables some locking
 *     APIs to manage concurrent accesses on the SD card bus.  This is not
 *     needed for the simple case of a single SD card, for example.
 *   CONFIG_LPC17_40_SDCARD_DMA - Enable SD card DMA.  This is a marginally
 *   optional.
 *     For most usages, SD accesses will cause data overruns if used without
 *     DMA. NOTE the above system DMA configuration options.
 *   CONFIG_LPC17_40_SDCARD_WIDTH_D1_ONLY - This may be selected to force the
 *     driver operate with only a single data line (the default is to use
 *     all 4 SD data lines).
 *   CONFIG_DEBUG_MEMCARD_* - Enables some very low-level debug output
 *     This also requires CONFIG_DEBUG_FS and CONFIG_DEBUG_INFO
 */

#ifndef CONFIG_LPC17_40_SDCARD_DMA
#  warning "Large Non-DMA transfer may result in RX overrun failures"
#else
#  ifndef CONFIG_LPC17_40_GPDMA
#    error "CONFIG_LPC17_40_SDCARD_DMA support requires CONFIG_LPC17_40_GPDMA"
#  endif
#  ifndef CONFIG_SDIO_DMA
#    error CONFIG_SDIO_DMA must be defined with CONFIG_LPC17_40_SDCARD_DMA
#  endif
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE"
#endif

/* Friendly CLKCR bit re-definitions ****************************************/

/* Mode dependent settings.  These depend on clock devisor settings that must
 * be defined in the board-specific board.h header file: SDCARD_INIT_CLKDIV,
 * SDCARD_MMCXFR_CLKDIV, and SDCARD_SDXFR_CLKDIV.
 */

#define LPC17_40_CLCKCR_INIT         (SDCARD_INIT_CLKDIV | SDCARD_CLOCK_WIDBUS_D1)
#define SDCARD_CLOCK_MMCXFR       (SDCARD_MMCXFR_CLKDIV | SDCARD_CLOCK_WIDBUS_D1)
#define SDCARD_CLOCK_SDXFR        (SDCARD_SDXFR_CLKDIV | SDCARD_CLOCK_WIDBUS_D1)
#define SDCARD_CLOCK_SDWIDEXFR    (SDCARD_SDXFR_CLKDIV | SDCARD_CLOCK_WIDBUS_D4)

/* Timing */

#define SDCARD_CMDTIMEOUT         (100000)
#define SDCARD_LONGTIMEOUT        (0x7fffffff)

/* Big DTIMER setting */

#define SDCARD_DTIMER_DATATIMEOUT (0x000fffff)

/* DMA channel/stream configuration register settings.  The following
 * must be selected.  The DMA driver will select the remaining fields.
 *
 * - 32-bit DMA
 * - Memory increment
 * - Direction (memory-to-peripheral, peripheral-to-memory)
 * - Memory burst size (F4 only)
 */

/* DMA control register settings.  All CONTROL register fields need to be
 * specified except for the transfer size which is passed as a separate
 * parameter and for the terminal count interrupt enable bit which is
 * controlled by the driver.
 */

#define SDCARD_RXDMA32_CONTROL    (DMACH_CONTROL_SBSIZE_8 | DMACH_CONTROL_DBSIZE_8 | \
                                   DMACH_CONTROL_SWIDTH_32BIT | DMACH_CONTROL_DWIDTH_32BIT | \
                                   DMACH_CONTROL_DI)
#define SDCARD_TXDMA32_CONTROL    (DMACH_CONTROL_SBSIZE_8 | DMACH_CONTROL_DBSIZE_8 | \
                                   DMACH_CONTROL_SWIDTH_32BIT | DMACH_CONTROL_DWIDTH_32BIT | \
                                   DMACH_CONTROL_SI)

/* DMA configuration register settings.  Only the SRCPER, DSTPER, and
 * XFRTTYPE fields of the CONFIG register need be specified.
 */

#define SDCARD_RXDMA32_CONFIG     (DMACH_CONFIG_SRCPER_SDCARD | DMACH_CONFIG_XFRTYPE_P2M_SC)
#define SDCARD_TXDMA32_CONFIG     (DMACH_CONFIG_DSTPER_SDCARD | DMACH_CONFIG_XFRTYPE_M2P_DC)

/* FIFO sizes */

#define SDCARD_HALFFIFO_WORDS     (8)
#define SDCARD_HALFFIFO_BYTES     (8*4)

/* Data transfer interrupt mask bits */

#define SDCARD_RECV_MASK     (SDCARD_MASK0_DCRCFAILIE | SDCARD_MASK0_DTIMEOUTIE | \
                              SDCARD_MASK0_DATAENDIE | SDCARD_MASK0_RXOVERRIE | \
                              SDCARD_MASK0_RXFIFOHFIE | SDCARD_MASK0_STBITERRIE)
#define SDCARD_SEND_MASK     (SDCARD_MASK0_DCRCFAILIE | SDCARD_MASK0_DTIMEOUTIE | \
                              SDCARD_MASK0_DATAENDIE | SDCARD_MASK0_TXUNDERRIE | \
                              SDCARD_MASK0_TXFIFOHEIE | SDCARD_MASK0_STBITERRIE)
#define SDCARD_DMARECV_MASK  (SDCARD_MASK0_DCRCFAILIE | SDCARD_MASK0_DTIMEOUTIE | \
                              SDCARD_MASK0_DATAENDIE | SDCARD_MASK0_RXOVERRIE | \
                              SDCARD_MASK0_STBITERRIE)
#define SDCARD_DMASEND_MASK  (SDCARD_MASK0_DCRCFAILIE | SDCARD_MASK0_DTIMEOUTIE | \
                              SDCARD_MASK0_DATAENDIE | SDCARD_MASK0_TXUNDERRIE | \
                              SDCARD_MASK0_STBITERRIE)

/* Event waiting interrupt mask bits */

#define SDCARD_CMDDONE_STA   (SDCARD_STATUS_CMDSENT)
#define SDCARD_RESPDONE_STA  (SDCARD_STATUS_CTIMEOUT | SDCARD_STATUS_CCRCFAIL | \
                              SDCARD_STATUS_CMDREND)
#define SDCARD_XFRDONE_STA   (0)

#define SDCARD_CMDDONE_MASK  (SDCARD_MASK0_CMDSENTIE)
#define SDCARD_RESPDONE_MASK (SDCARD_MASK0_CCRCFAILIE | SDCARD_MASK0_CTIMEOUTIE | \
                              SDCARD_MASK0_CMDRENDIE)
#define SDCARD_XFRDONE_MASK  (0)

#define SDCARD_CMDDONE_ICR   (SDCARD_CLEAR_CMDSENTC)
#define SDCARD_RESPDONE_ICR  (SDCARD_CLEAR_CTIMEOUTC | SDCARD_CLEAR_CCRCFAILC | \
                              SDCARD_CLEAR_CMDRENDC)
#define SDCARD_XFRDONE_ICR   (SDCARD_CLEAR_DATAENDC | SDCARD_CLEAR_DCRCFAILC | \
                              SDCARD_CLEAR_DTIMEOUTC | SDCARD_CLEAR_RXOVERRC | \
                              SDCARD_CLEAR_TXUNDERRC | SDCARD_CLEAR_STBITERRC)

#define SDCARD_WAITALL_ICR   (SDCARD_CMDDONE_ICR | SDCARD_RESPDONE_ICR | \
                              SDCARD_XFRDONE_ICR)

/* Let's wait until we have both SD card transfer complete and DMA
 * complete.
 */

#define SDCARD_XFRDONE_FLAG  (1)
#define SDCARD_DMADONE_FLAG  (2)
#define SDCARD_ALLDONE       (3)

/* Register logging support */

#ifdef CONFIG_DEBUG_MEMCARD_INFO
#  ifdef CONFIG_LPC17_40_SDCARD_DMA
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

/* This structure defines the state of the LPC17XX_40XX SD card interface */

struct lpc17_40_dev_s
{
  struct sdio_dev_s  dev;        /* Standard, base SD card interface */

  /* LPC17XX_40XX-specific extensions */

  /* Event support */

  sem_t              waitsem;         /* Implements event waiting */
  sdio_eventset_t    waitevents;      /* Set of events to be waited for */
  uint32_t           waitmask;        /* Interrupt enables for event waiting */
  volatile sdio_eventset_t wkupevent; /* The event that caused the wakeup */
  struct wdog_s      waitwdog;        /* Watchdog that handles event timeouts */

  /* Callback support */

  sdio_statset_t     cdstatus;   /* Card status */
  sdio_eventset_t    cbevents;   /* Set of events to be cause callbacks */
  worker_t           callback;   /* Registered callback function */
  void              *cbarg;      /* Registered callback argument */
  struct work_s      cbwork;     /* Callback work queue structure */

  /* Interrupt mode data transfer support */

  uint32_t          *buffer;     /* Address of current R/W buffer */
  size_t             remaining;  /* Number of bytes remaining in the transfer */
  uint32_t           xfrmask;    /* Interrupt enables for data transfer */

  /* DMA data transfer support */

  bool               widebus;    /* Required for DMA support */
#ifdef CONFIG_LPC17_40_SDCARD_DMA
  volatile uint8_t   xfrflags;   /* Used to synchronize SD card and DMA completion events */
  bool               dmamode;    /* true: DMA mode transfer */
  DMA_HANDLE         dma;        /* Handle for DMA channel */
#endif
};

/* Register logging support */

#ifdef CONFIG_DEBUG_MEMCARD_INFO
struct lpc17_40_sdcard_regs_s
{
  uint8_t  pwr;
  uint16_t clkcr;
  uint16_t dctrl;
  uint32_t dtimer;
  uint32_t dlen;
  uint32_t dcount;
  uint32_t sta;
  uint32_t mask;
  uint32_t fifocnt;
};

struct lpc17_40_sampleregs_s
{
  struct lpc17_40_sdcard_regs_s sdcard;
#if defined(CONFIG_DEBUG_DMA) && defined(CONFIG_LPC17_40_SDCARD_DMA)
  struct lpc17_40_dmaregs_s  dma;
#endif
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static int lpc17_40_takesem(struct lpc17_40_dev_s *priv);
#define     lpc17_40_givesem(priv) (nxsem_post(&priv->waitsem))
static inline void lpc17_40_setclock(uint32_t clkcr);
static void lpc17_40_configwaitints(struct lpc17_40_dev_s *priv,
              uint32_t waitmask, sdio_eventset_t waitevents,
              sdio_eventset_t wkupevents);
static void lpc17_40_configxfrints(struct lpc17_40_dev_s *priv,
              uint32_t xfrmask);
static void lpc17_40_setpwrctrl(uint32_t pwrctrl);
static inline uint32_t lpc17_40_getpwrctrl(void);

/* DMA Helpers **************************************************************/

#ifdef CONFIG_DEBUG_MEMCARD_INFO
static void lpc17_40_sampleinit(void);
static void lpc17_40_sdcard_sample(struct lpc17_40_sdcard_regs_s *regs);
static void lpc17_40_sample(struct lpc17_40_dev_s *priv, int index);
static void lpc17_40_sdcard_dump(struct lpc17_40_sdcard_regs_s *regs,
              const char *msg);
static void lpc17_40_dumpsample(struct lpc17_40_dev_s *priv,
              struct lpc17_40_sampleregs_s *regs, const char *msg);
static void lpc17_40_dumpsamples(struct lpc17_40_dev_s *priv);
#else
#  define   lpc17_40_sampleinit()
#  define   lpc17_40_sample(priv,index)
#  define   lpc17_40_dumpsamples(priv)
#endif

#ifdef CONFIG_LPC17_40_SDCARD_DMA
static void lpc17_40_dmacallback(DMA_HANDLE handle, void *arg, int status);
#endif

/* Data Transfer Helpers ****************************************************/

static uint8_t lpc17_40_log2(uint16_t value);
static void lpc17_40_dataconfig(uint32_t timeout, uint32_t dlen,
              uint32_t dctrl);
static void lpc17_40_datadisable(void);
static void lpc17_40_sendfifo(struct lpc17_40_dev_s *priv);
static void lpc17_40_recvfifo(struct lpc17_40_dev_s *priv);
static void lpc17_40_eventtimeout(wdparm_t arg);
static void lpc17_40_endwait(struct lpc17_40_dev_s *priv,
              sdio_eventset_t wkupevent);
static void lpc17_40_endtransfer(struct lpc17_40_dev_s *priv,
              sdio_eventset_t wkupevent);

/* Interrupt Handling *******************************************************/

static int  lpc17_40_interrupt(int irq, void *context, void *arg);

/* SD Card Interface Methods ************************************************/

/* Mutual exclusion */

#ifdef CONFIG_SDIO_MUXBUS
static int lpc17_40_lock(struct sdio_dev_s *dev, bool lock);
#endif

/* Initialization/setup */

static void lpc17_40_reset(struct sdio_dev_s *dev);
static sdio_capset_t lpc17_40_capabilities(struct sdio_dev_s *dev);
static uint8_t lpc17_40_status(struct sdio_dev_s *dev);
static void lpc17_40_widebus(struct sdio_dev_s *dev, bool enable);
static void lpc17_40_clock(struct sdio_dev_s *dev,
              enum sdio_clock_e rate);
static int  lpc17_40_attach(struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int  lpc17_40_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t arg);
static int  lpc17_40_recvsetup(struct sdio_dev_s *dev,
              uint8_t *buffer, size_t nbytes);
static int  lpc17_40_sendsetup(struct sdio_dev_s *dev,
              const uint8_t *buffer, size_t nbytes);
static int  lpc17_40_cancel(struct sdio_dev_s *dev);

static int  lpc17_40_waitresponse(struct sdio_dev_s *dev, uint32_t cmd);
static int  lpc17_40_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  lpc17_40_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t rlong[4]);
static int  lpc17_40_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  lpc17_40_recvnotimpl(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rnotimpl);

/* EVENT handler */

static void lpc17_40_waitenable(struct sdio_dev_s *dev,
              sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t lpc17_40_eventwait(struct sdio_dev_s *dev);
static void lpc17_40_callbackenable(struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static int  lpc17_40_registercallback(struct sdio_dev_s *dev,
              worker_t callback, void *arg);

/* DMA */

#ifdef CONFIG_LPC17_40_SDCARD_DMA
static int  lpc17_40_dmarecvsetup(struct sdio_dev_s *dev,
              uint8_t *buffer, size_t buflen);
static int  lpc17_40_dmasendsetup(struct sdio_dev_s *dev,
              const uint8_t *buffer, size_t buflen);
#endif

/* Initialization/uninitialization/reset ************************************/

static void lpc17_40_callback(void *arg);
static void lpc17_40_default(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct lpc17_40_dev_s g_scard_dev =
{
  .dev =
  {
#ifdef CONFIG_SDIO_MUXBUS
    .lock             = lpc17_40_lock,
#endif
    .reset            = lpc17_40_reset,
    .capabilities     = lpc17_40_capabilities,
    .status           = lpc17_40_status,
    .widebus          = lpc17_40_widebus,
    .clock            = lpc17_40_clock,
    .attach           = lpc17_40_attach,
    .sendcmd          = lpc17_40_sendcmd,
    .recvsetup        = lpc17_40_recvsetup,
    .sendsetup        = lpc17_40_sendsetup,
    .cancel           = lpc17_40_cancel,
    .waitresponse     = lpc17_40_waitresponse,
    .recv_r1          = lpc17_40_recvshortcrc,
    .recv_r2          = lpc17_40_recvlong,
    .recv_r3          = lpc17_40_recvshort,
    .recv_r4          = lpc17_40_recvnotimpl,
    .recv_r5          = lpc17_40_recvnotimpl,
    .recv_r6          = lpc17_40_recvshortcrc,
    .recv_r7          = lpc17_40_recvshort,
    .waitenable       = lpc17_40_waitenable,
    .eventwait        = lpc17_40_eventwait,
    .callbackenable   = lpc17_40_callbackenable,
    .registercallback = lpc17_40_registercallback,
#ifdef CONFIG_SDIO_DMA
#ifdef CONFIG_LPC17_40_SDCARD_DMA
    .dmarecvsetup     = lpc17_40_dmarecvsetup,
    .dmasendsetup     = lpc17_40_dmasendsetup,
#else
    .dmarecvsetup     = lpc17_40_recvsetup,
    .dmasendsetup     = lpc17_40_sendsetup,
#endif
#endif
  },
};

/* Register logging support */

#ifdef CONFIG_DEBUG_MEMCARD_INFO
static struct lpc17_40_sampleregs_s g_sampleregs[DEBUG_NSAMPLES];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Low-level Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_takesem
 *
 * Description:
 *   Take the wait semaphore (handling false alarm wakeups due to the receipt
 *   of signals).
 *
 * Input Parameters:
 *   dev - Instance of the SD card device driver state structure.
 *
 * Returned Value:
 *   Normally OK, but may return -ECANCELED in the rare event that the task
 *   has been canceled.
 *
 ****************************************************************************/

static int lpc17_40_takesem(struct lpc17_40_dev_s *priv)
{
  return nxsem_wait_uninterruptible(&priv->waitsem);
}

/****************************************************************************
 * Name: lpc17_40_setclock
 *
 * Description:
 *   Modify oft-changed bits in the CLKCR register.  Only the following bit-
 *   fields are changed:
 *
 *   CLKDIV, PWRSAV, BYPASS, and WIDBUS
 *
 * Input Parameters:
 *   clkcr - A new CLKCR setting for the above mentions bits (other bits
 *           are ignored.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void lpc17_40_setclock(uint32_t clkcr)
{
  uint32_t regval = getreg32(LPC17_40_SDCARD_CLOCK);

  /* Clear CLKDIV, PWRSAV, BYPASS, and WIDBUS bits */

  regval &= ~(SDCARD_CLOCK_CLKDIV_MASK | SDCARD_CLOCK_PWRSAV |
              SDCARD_CLOCK_BYPASS | SDCARD_CLOCK_WIDBUS |
              SDCARD_CLOCK_CLKEN);

  /* Replace with user provided settings */

  clkcr  &=  (SDCARD_CLOCK_CLKDIV_MASK | SDCARD_CLOCK_PWRSAV |
              SDCARD_CLOCK_BYPASS | SDCARD_CLOCK_WIDBUS |
              SDCARD_CLOCK_CLKEN);

  regval |=  clkcr;
  putreg32(regval, LPC17_40_SDCARD_CLOCK);

  mcinfo("CLKCR: %08" PRIx32 " PWR: %08" PRIx32 "\n",
         getreg32(LPC17_40_SDCARD_CLOCK), getreg32(LPC17_40_SDCARD_PWR));
}

/****************************************************************************
 * Name: lpc17_40_configwaitints
 *
 * Description:
 *   Enable/disable SD card interrupts needed to support the wait function
 *
 * Input Parameters:
 *   priv       - A reference to the SD card device state structure
 *   waitmask   - The set of bits in the SD card MASK register to set
 *   waitevents - Waited for events
 *   wkupevent  - Wake-up events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc17_40_configwaitints(struct lpc17_40_dev_s *priv,
                                    uint32_t waitmask,
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
#ifdef CONFIG_LPC17_40_SDCARD_DMA
  priv->xfrflags   = 0;
#endif
  putreg32(priv->xfrmask | priv->waitmask, LPC17_40_SDCARD_MASK0);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc17_40_configxfrints
 *
 * Description:
 *   Enable SD card interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the SD card device state structure
 *   xfrmask - The set of bits in the SD card MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc17_40_configxfrints(struct lpc17_40_dev_s *priv,
                                   uint32_t xfrmask)
{
  irqstate_t flags;
  flags = enter_critical_section();
  priv->xfrmask = xfrmask;
  putreg32(priv->xfrmask | priv->waitmask, LPC17_40_SDCARD_MASK0);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc17_40_setpwrctrl
 *
 * Description:
 *   Change the PWRCTRL field of the SD card POWER register to turn the
 *   SD card ON or OFF
 *
 * Input Parameters:
 *   clkcr - A new PWRCTRL setting
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc17_40_setpwrctrl(uint32_t pwrctrl)
{
  uint32_t regval;

  /* Set the new value of the PWRCTRL field of the PWR register.  Also, as a
   * side-effect, clear the OPENDRAIN and ROD bits as well.
   */

  regval  = getreg32(LPC17_40_SDCARD_PWR);
  regval &= ~(SDCARD_PWR_CTRL_MASK | SDCARD_PWR_OPENDRAIN | SDCARD_PWR_ROD);
  regval |= pwrctrl;
  putreg32(regval, LPC17_40_SDCARD_PWR);
}

/****************************************************************************
 * Name: lpc17_40_getpwrctrl
 *
 * Description:
 *   Return the current value of the  the PWRCTRL field of the SD card P
 *   register.  This function can be used to see if the SD card is powered ON
 *   or OFF
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current value of the  the PWRCTRL field of the SD card PWR register.
 *
 ****************************************************************************/

static inline uint32_t lpc17_40_getpwrctrl(void)
{
  /* Extract and return the PWRCTRL field of the PWR register. */

  return getreg32(LPC17_40_SDCARD_PWR) & SDCARD_PWR_CTRL_MASK;
}

/****************************************************************************
 * DMA Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_sampleinit
 *
 * Description:
 *   Setup prior to collecting DMA samples
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_MEMCARD_INFO
static void lpc17_40_sampleinit(void)
{
  memset(g_sampleregs, 0xff, DEBUG_NSAMPLES *
         sizeof(struct lpc17_40_sampleregs_s));
}
#endif

/****************************************************************************
 * Name: lpc17_40_sdcard_sample
 *
 * Description:
 *   Sample SD card registers
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_MEMCARD_INFO
static void lpc17_40_sdcard_sample(struct lpc17_40_sdcard_regs_s *regs)
{
  regs->pwr     = (uint8_t)getreg32(LPC17_40_SDCARD_PWR);
  regs->clkcr   = (uint16_t)getreg32(LPC17_40_SDCARD_CLOCK);
  regs->dctrl   = (uint16_t)getreg32(LPC17_40_SDCARD_DCTRL);
  regs->dtimer  = getreg32(LPC17_40_SDCARD_DTIMER);
  regs->dlen    = getreg32(LPC17_40_SDCARD_DLEN);
  regs->dcount  = getreg32(LPC17_40_SDCARD_DCOUNT);
  regs->sta     = getreg32(LPC17_40_SDCARD_STATUS);
  regs->mask    = getreg32(LPC17_40_SDCARD_MASK0);
  regs->fifocnt = getreg32(LPC17_40_SDCARD_FIFOCNT);
}
#endif

/****************************************************************************
 * Name: lpc17_40_sample
 *
 * Description:
 *   Sample SD card/DMA registers
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_MEMCARD_INFO
static void lpc17_40_sample(struct lpc17_40_dev_s *priv, int index)
{
  struct lpc17_40_sampleregs_s *regs = &g_sampleregs[index];
#if defined(CONFIG_DEBUG_DMA) && defined(CONFIG_LPC17_40_SDCARD_DMA)
  if (priv->dmamode)
    {
      lpc17_40_dmasample(priv->dma, &regs->dma);
    }
#endif

  lpc17_40_sdcard_sample(&regs->sdcard);
}
#endif

/****************************************************************************
 * Name: lpc17_40_sdcard_dump
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_MEMCARD_INFO
static void lpc17_40_sdcard_dump(struct lpc17_40_sdcard_regs_s *regs,
                                 const char *msg)
{
  mcinfo("SD Card Registers: %s\n", msg);
  mcinfo("  POWER[%08x]: %08x\n", LPC17_40_SDCARD_PWR,     regs->pwr);
  mcinfo("  CLKCR[%08x]: %08x\n", LPC17_40_SDCARD_CLOCK,   regs->clkcr);
  mcinfo("  DCTRL[%08x]: %08x\n", LPC17_40_SDCARD_DCTRL,   regs->dctrl);
  mcinfo(" DTIMER[%08x]: %08x\n", LPC17_40_SDCARD_DTIMER,  regs->dtimer);
  mcinfo("   DLEN[%08x]: %08x\n", LPC17_40_SDCARD_DLEN,    regs->dlen);
  mcinfo(" DCOUNT[%08x]: %08x\n", LPC17_40_SDCARD_DCOUNT,  regs->dcount);
  mcinfo("    STA[%08x]: %08x\n", LPC17_40_SDCARD_STATUS,  regs->sta);
  mcinfo("   MASK[%08x]: %08x\n", LPC17_40_SDCARD_MASK0,   regs->mask);
  mcinfo("FIFOCNT[%08x]: %08x\n", LPC17_40_SDCARD_FIFOCNT, regs->fifocnt);
}
#endif

/****************************************************************************
 * Name: lpc17_40_dumpsample
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_MEMCARD_INFO
static void lpc17_40_dumpsample(struct lpc17_40_dev_s *priv,
                                struct lpc17_40_sampleregs_s *regs,
                                const char *msg)
{
#if defined(CONFIG_DEBUG_DMA) && defined(CONFIG_LPC17_40_SDCARD_DMA)
  if (priv->dmamode)
    {
      lpc17_40_dmadump(priv->dma, &regs->dma, msg);
    }
#endif

  lpc17_40_sdcard_dump(&regs->sdcard, msg);
}
#endif

/****************************************************************************
 * Name: lpc17_40_dumpsamples
 *
 * Description:
 *   Dump all sampled register data
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_MEMCARD_INFO
static void lpc17_40_dumpsamples(struct lpc17_40_dev_s *priv)
{
  lpc17_40_dumpsample(priv, &g_sampleregs[SAMPLENDX_BEFORE_SETUP],
                      "Before setup");
#if defined(CONFIG_DEBUG_DMA) && defined(CONFIG_LPC17_40_SDCARD_DMA)
  if (priv->dmamode)
    {
      lpc17_40_dumpsample(priv, &g_sampleregs[SAMPLENDX_BEFORE_ENABLE],
                          "Before DMA enable");
    }
#endif

  lpc17_40_dumpsample(priv, &g_sampleregs[SAMPLENDX_AFTER_SETUP],
                      "After setup");
  lpc17_40_dumpsample(priv, &g_sampleregs[SAMPLENDX_END_TRANSFER],
                      "End of transfer");
#if defined(CONFIG_DEBUG_DMA) && defined(CONFIG_LPC17_40_SDCARD_DMA)
  if (priv->dmamode)
    {
      lpc17_40_dumpsample(priv, &g_sampleregs[SAMPLENDX_DMA_CALLBACK],
                          "DMA Callback");
    }
#endif
}
#endif

/****************************************************************************
 * Name: lpc17_40_dmacallback
 *
 * Description:
 *   Called when SD card DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_SDCARD_DMA
static void lpc17_40_dmacallback(DMA_HANDLE handle, void *arg, int status)
{
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)arg;
  DEBUGASSERT(priv->dmamode);
  sdio_eventset_t result;

  /* In the normal case, SD card appears to handle the End-Of-Transfer
   * interrupt first with the End-Of-DMA event occurring significantly later.
   * On transfer errors, however, the DMA error will occur before the End-of-
   * Transfer.
   */

  lpc17_40_sample((struct lpc17_40_dev_s *)arg, SAMPLENDX_DMA_CALLBACK);

  /* Get the result of the DMA transfer */

  if (status < 0)
    {
      dmaerr("ERROR: DMA error %d, remaining: %d\n", status,
             priv->remaining);
      result = SDIOWAIT_ERROR;
    }
  else
    {
      result = SDIOWAIT_TRANSFERDONE;
    }

  /* Then terminate the transfer if this completes all of the steps in the
   * transfer OR if a DMA error occurred.  In the non-error case, we should
   * already have the SD card transfer done interrupt.  If not, the transfer
   * will appropriately time out.
   */

  priv->xfrflags |= SDCARD_DMADONE_FLAG;
  if (priv->xfrflags == SDCARD_ALLDONE || result == SDIOWAIT_ERROR)
    {
      lpc17_40_endtransfer(priv, result);
    }
}
#endif

/****************************************************************************
 * Data Transfer Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_log2
 *
 * Description:
 *   Take (approximate) log base 2 of the provided number (Only works if the
 *   provided number is a power of 2).
 *
 ****************************************************************************/

static uint8_t lpc17_40_log2(uint16_t value)
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
 * Name: lpc17_40_dataconfig
 *
 * Description:
 *   Configure the SD card data path for the next data transfer
 *
 ****************************************************************************/

static void lpc17_40_dataconfig(uint32_t timeout, uint32_t dlen,
                                uint32_t dctrl)
{
  uint32_t regval = 0;

  /* Enable data path */

  putreg32(timeout, LPC17_40_SDCARD_DTIMER); /* Set DTIMER */
  putreg32(dlen,    LPC17_40_SDCARD_DLEN);   /* Set DLEN */

  /* Configure DCTRL DTDIR, DTMODE, and DBLOCKSIZE fields and set the DTEN
   * field
   */

  regval  =  getreg32(LPC17_40_SDCARD_DCTRL);
  regval &= ~(SDCARD_DCTRL_DTDIR | SDCARD_DCTRL_DTMODE |
              SDCARD_DCTRL_DBLOCKSIZE_MASK);
  dctrl  &=  (SDCARD_DCTRL_DTDIR | SDCARD_DCTRL_DTMODE |
              SDCARD_DCTRL_DBLOCKSIZE_MASK);
  regval |=  (dctrl | SDCARD_DCTRL_DTEN);
  putreg32(regval, LPC17_40_SDCARD_DCTRL);
}

/****************************************************************************
 * Name: lpc17_40_datadisable
 *
 * Description:
 *   Disable the SD card data path setup by lpc17_40_dataconfig() and
 *   disable DMA.
 *
 ****************************************************************************/

static void lpc17_40_datadisable(void)
{
  uint32_t regval;

  /* Disable the data path */

  putreg32(SDCARD_DTIMER_DATATIMEOUT, LPC17_40_SDCARD_DTIMER); /* Reset DTIMER */
  putreg32(0,                         LPC17_40_SDCARD_DLEN);   /* Reset DLEN */

  /* Reset DCTRL DTEN, DTDIR, DTMODE, DMAEN, and DBLOCKSIZE fields */

  regval  = getreg32(LPC17_40_SDCARD_DCTRL);
  regval &= ~(SDCARD_DCTRL_DTEN | SDCARD_DCTRL_DTDIR | SDCARD_DCTRL_DTMODE |
              SDCARD_DCTRL_DMAEN | SDCARD_DCTRL_DBLOCKSIZE_MASK);
  putreg32(regval, LPC17_40_SDCARD_DCTRL);
}

/****************************************************************************
 * Name: lpc17_40_sendfifo
 *
 * Description:
 *   Send SD card data in interrupt mode
 *
 * Input Parameters:
 *   priv - An instance of the SD card device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc17_40_sendfifo(struct lpc17_40_dev_s *priv)
{
  union
  {
    uint32_t w;
    uint8_t  b[4];
  } data;

  /* Loop while there is more data to be sent and the RX FIFO is not full */

  while (priv->remaining > 0 &&
         (getreg32(LPC17_40_SDCARD_STATUS) & SDCARD_STATUS_TXFIFOF) == 0)
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

      putreg32(data.w, LPC17_40_SDCARD_FIFO);
    }
}

/****************************************************************************
 * Name: lpc17_40_recvfifo
 *
 * Description:
 *   Receive SD card data in interrupt mode
 *
 * Input Parameters:
 *   priv - An instance of the SD card device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc17_40_recvfifo(struct lpc17_40_dev_s *priv)
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
         (getreg32(LPC17_40_SDCARD_STATUS) & SDCARD_STATUS_RXDAVL) != 0)
    {
      /* Read the next word from the RX FIFO */

      data.w = getreg32(LPC17_40_SDCARD_FIFO);
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

/****************************************************************************
 * Name: lpc17_40_eventtimeout
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

static void lpc17_40_eventtimeout(wdparm_t arg)
{
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)arg;

  /* There is always race conditions with timer expirations. */

  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0 ||
              priv->wkupevent != 0);

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. wake up any waiting threads */

      lpc17_40_endwait(priv, SDIOWAIT_TIMEOUT);
      mcerr("ERROR: Timeout: remaining: %d\n", priv->remaining);
    }
}

/****************************************************************************
 * Name: lpc17_40_endwait
 *
 * Description:
 *   Wake up a waiting thread if the waited-for event has occurred.
 *
 * Input Parameters:
 *   priv      - An instance of the SD card device interface
 *   wkupevent - The event that caused the wait to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void lpc17_40_endwait(struct lpc17_40_dev_s *priv,
                             sdio_eventset_t wkupevent)
{
  /* Cancel the watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Disable event-related interrupts */

  lpc17_40_configwaitints(priv, 0, 0, wkupevent);

  /* Wake up the waiting thread */

  lpc17_40_givesem(priv);
}

/****************************************************************************
 * Name: lpc17_40_endtransfer
 *
 * Description:
 *   Terminate a transfer with the provided status.  This function is called
 *   only from the SD card interrupt handler when end-of-transfer conditions
 *   are detected.
 *
 * Input Parameters:
 *   priv   - An instance of the SD card device interface
 *   wkupevent - The event that caused the transfer to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void lpc17_40_endtransfer(struct lpc17_40_dev_s *priv,
                                 sdio_eventset_t wkupevent)
{
  /* Disable all transfer related interrupts */

  lpc17_40_configxfrints(priv, 0);

  /* Clearing pending interrupt status on all transfer related interrupts */

  putreg32(SDCARD_XFRDONE_ICR, LPC17_40_SDCARD_CLEAR);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_LPC17_40_SDCARD_DMA
  if (priv->dmamode)
    {
      /* DMA debug instrumentation */

      lpc17_40_sample(priv, SAMPLENDX_END_TRANSFER);

      /* Make sure that the DMA is stopped (it will be stopped automatically
       * on normal transfers, but not necessarily when the transfer
       * terminates on an error condition).
       */

      lpc17_40_dmastop(priv->dma);
    }
#endif

  /* Mark the transfer finished */

  priv->remaining = 0;

  /* Is a thread wait for these data transfer complete events? */

  if ((priv->waitevents & wkupevent) != 0)
    {
      /* Yes.. wake up any waiting threads */

      lpc17_40_endwait(priv, wkupevent);
    }
}

/****************************************************************************
 * Interrupt Handling
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_interrupt
 *
 * Description:
 *   SD card interrupt handler
 *
 * Input Parameters:
 *   dev - An instance of the SD card device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int lpc17_40_interrupt(int irq, void *context, void *arg)
{
  struct lpc17_40_dev_s *priv = &g_scard_dev;
  uint32_t enabled;
  uint32_t pending;

  /* Loop while there are pending interrupts.  Check the SD card status
   * register.  Mask out all bits that don't correspond to enabled
   * interrupts.  (This depends on the fact that bits are ordered
   * the same in both the STA and MASK register).  If there are non-zero
   * bits remaining, then we have work to do here.
   */

  while ((enabled = getreg32(LPC17_40_SDCARD_STATUS) &
         getreg32(LPC17_40_SDCARD_MASK0)) != 0)
    {
      /* Handle in progress, interrupt driven data transfers ****************/

      pending  = enabled & priv->xfrmask;
      if (pending != 0)
        {
#ifdef CONFIG_LPC17_40_SDCARD_DMA
          if (!priv->dmamode)
#endif
            {
              /* Is the RX FIFO half full or more?  Is so then we must be
               * processing a receive transaction.
               */

              if ((pending & SDCARD_STATUS_RXFIFOHF) != 0)
                {
                  /* Receive data from the RX FIFO */

                  lpc17_40_recvfifo(priv);
                }

              /* Otherwise, Is the transmit FIFO half empty or less?  If so
               * we must be processing a send transaction. NOTE: We can't be
               * processing both!
               */

              else if ((pending & SDCARD_STATUS_TXFIFOHE) != 0)
                {
                  /* Send data via the TX FIFO */

                  lpc17_40_sendfifo(priv);
                }
            }

          /* Handle data end events */

          if ((pending & SDCARD_STATUS_DATAEND) != 0)
            {
              /* Handle any data remaining the RX FIFO.  If the RX FIFO is
               * less than half full at the end of the transfer, then no
               * half-full interrupt will be received.
               */

              /* Was this transfer performed in DMA mode? */

#ifdef CONFIG_LPC17_40_SDCARD_DMA
              if (priv->dmamode)
                {
                  /* Yes.. Terminate the transfers only if the DMA has also
                   * finished.
                   */

                  priv->xfrflags |= SDCARD_XFRDONE_FLAG;
                  if (priv->xfrflags == SDCARD_ALLDONE)
                    {
                      lpc17_40_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
                    }

                  /* Otherwise, just disable further transfer interrupts and
                   * wait for the DMA complete event.
                   */

                  else
                    {
                      lpc17_40_configxfrints(priv, 0);
                    }
                }
              else
#endif
                {
                  /* Receive data from the RX FIFO */

                  lpc17_40_recvfifo(priv);

                  /* Then terminate the transfer */

                  lpc17_40_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
                }
            }

          /* Handle data block send/receive CRC failure */

          else if ((pending & SDCARD_STATUS_DCRCFAIL) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Data block CRC failure, remaining: %d\n",
                    priv->remaining);
              lpc17_40_endtransfer(priv, SDIOWAIT_TRANSFERDONE |
                                         SDIOWAIT_ERROR);
            }

          /* Handle data timeout error */

          else if ((pending & SDCARD_STATUS_DTIMEOUT) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Data timeout, remaining: %d\n", priv->remaining);
              lpc17_40_endtransfer(priv, SDIOWAIT_TRANSFERDONE |
                                         SDIOWAIT_TIMEOUT);
            }

          /* Handle RX FIFO overrun error */

          else if ((pending & SDCARD_STATUS_RXOVERR) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: RX FIFO overrun, remaining: %d\n",
                    priv->remaining);
              lpc17_40_endtransfer(priv, SDIOWAIT_TRANSFERDONE |
                                         SDIOWAIT_ERROR);
            }

          /* Handle TX FIFO underrun error */

          else if ((pending & SDCARD_STATUS_TXUNDERR) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: TX FIFO underrun, remaining: %d\n",
                    priv->remaining);
              lpc17_40_endtransfer(priv, SDIOWAIT_TRANSFERDONE |
                                         SDIOWAIT_ERROR);
            }

          /* Handle start bit error */

          else if ((pending & SDCARD_STATUS_STBITERR) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Start bit, remaining: %d\n", priv->remaining);
              lpc17_40_endtransfer(priv, SDIOWAIT_TRANSFERDONE |
                                         SDIOWAIT_ERROR);
            }
        }

      /* Handle wait events *************************************************/

      pending  = enabled & priv->waitmask;
      if (pending != 0)
        {
          /* Is this a response completion event? */

          if ((pending & SDCARD_RESPDONE_STA) != 0)
            {
              /* Yes.. Is their a thread waiting for response done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  putreg32(SDCARD_RESPDONE_ICR | SDCARD_CMDDONE_ICR,
                           LPC17_40_SDCARD_CLEAR);
                  lpc17_40_endwait(priv, SDIOWAIT_RESPONSEDONE);
                }
            }

          /* Is this a command completion event? */

          if ((pending & SDCARD_CMDDONE_STA) != 0)
            {
              /* Yes.. Is their a thread waiting for command done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  putreg32(SDCARD_CMDDONE_ICR, LPC17_40_SDCARD_CLEAR);
                  lpc17_40_endwait(priv, SDIOWAIT_CMDDONE);
                }
            }
        }
    }

  return OK;
}

/****************************************************************************
 * SD card Interface Methods
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_lock
 *
 * Description:
 *   Locks the bus. Function calls low-level multiplexed bus routines to
 *   resolve bus requests and acknowledgment issues.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *   lock   - TRUE to lock, FALSE to unlock.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_MUXBUS
static int lpc17_40_lock(struct sdio_dev_s *dev, bool lock)
{
  /* Single SD card instance so there is only one possibility.  The multiplex
   * bus is part of board support package.
   */

  lpc17_40_muxbus_sdio_lock(lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: lpc17_40_reset
 *
 * Description:
 *   Reset the SD card controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc17_40_reset(struct sdio_dev_s *dev)
{
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)dev;
  irqstate_t flags;
  uint32_t regval;

  /* Disable clocking */

  flags = enter_critical_section();

  /* Disable the SD Interface */

  regval = getreg32(LPC17_40_SDCARD_CLOCK);
  regval &= ~SDCARD_CLOCK_CLKEN;
  putreg32(regval, LPC17_40_SDCARD_CLOCK);

  lpc17_40_setpwrctrl(SDCARD_PWR_CTRL_OFF);

  /* Put SD card registers in their default, reset state */

  lpc17_40_default();

  /* Reset data */

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitmask   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */
#ifdef CONFIG_LPC17_40_SDCARD_DMA
  priv->xfrflags   = 0;      /* Used to synchronize SD card and DMA completion events */
#endif

  wd_cancel(&priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->buffer     = 0;      /* Address of current R/W buffer */
  priv->remaining  = 0;      /* Number of bytes remaining in the transfer */
  priv->xfrmask    = 0;      /* Interrupt enables for data transfer */

  /* DMA data transfer support */

  priv->widebus    = false;  /* Required for DMA support */
#ifdef CONFIG_LPC17_40_SDCARD_DMA
  priv->dmamode    = false;  /* true: DMA mode transfer */
#endif

  /* Configure and enable the SD card peripheral */

  lpc17_40_setclock(LPC17_40_CLCKCR_INIT | SDCARD_CLOCK_CLKEN);
  lpc17_40_setpwrctrl(SDCARD_PWR_CTRL_ON);
  leave_critical_section(flags);

  mcinfo("CLCKR: %08" PRIx32 " POWER: %08" PRIx32 "\n",
         getreg32(LPC17_40_SDCARD_CLOCK), getreg32(LPC17_40_SDCARD_PWR));
}

/****************************************************************************
 * Name: lpc17_40_capabilities
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

static sdio_capset_t lpc17_40_capabilities(struct sdio_dev_s *dev)
{
  sdio_capset_t caps = 0;

#ifdef CONFIG_LPC17_40_SDCARD_WIDTH_D1_ONLY
  caps |= SDIO_CAPS_1BIT_ONLY;
#endif
#ifdef CONFIG_LPC17_40_SDCARD_DMA
  caps |= SDIO_CAPS_DMASUPPORTED;
#endif

  return caps;
}

/****************************************************************************
 * Name: lpc17_40_status
 *
 * Description:
 *   Get SD card status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see lpc17_40_status_* defines)
 *
 ****************************************************************************/

static sdio_statset_t lpc17_40_status(struct sdio_dev_s *dev)
{
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)dev;
  return priv->cdstatus;
}

/****************************************************************************
 * Name: lpc17_40_widebus
 *
 * Description:
 *   Called after change in Bus width has been selected (via ACMD6).  Most
 *   controllers will need to perform some special operations to work
 *   correctly in the new bus mode.
 *
 * Input Parameters:
 *   dev  - An instance of the SD card device interface
 *   wide - true: wide bus (4-bit) bus mode enabled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc17_40_widebus(struct sdio_dev_s *dev, bool wide)
{
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)dev;
  priv->widebus = wide;
}

/****************************************************************************
 * Name: lpc17_40_clock
 *
 * Description:
 *   Enable/disable SD card clocking
 *
 * Input Parameters:
 *   dev  - An instance of the SD card device interface
 *   rate - Specifies the clocking to use (see enum sdio_clock_e)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc17_40_clock(struct sdio_dev_s *dev,
                           enum sdio_clock_e rate)
{
  uint32_t clkcr;

  switch (rate)
    {
      /* Disable clocking (with default ID mode divisor) */

      default:
      case CLOCK_SDIO_DISABLED:
        clkcr = LPC17_40_CLCKCR_INIT;
        return;

      /* Enable in initial ID mode clocking (<400KHz) */

      case CLOCK_IDMODE:
        clkcr = (LPC17_40_CLCKCR_INIT | SDCARD_CLOCK_CLKEN);
        break;

      /* Enable in MMC normal operation clocking */

      case CLOCK_MMC_TRANSFER:
        clkcr = (SDCARD_CLOCK_MMCXFR | SDCARD_CLOCK_CLKEN);
        lpc17_40_setpwrctrl(SDCARD_PWR_OPENDRAIN);
        break;

      /* SD normal operation clocking (wide 4-bit mode) */

      case CLOCK_SD_TRANSFER_4BIT:
#ifndef CONFIG_LPC17_40_SDCARD_WIDTH_D1_ONLY
        clkcr = (SDCARD_CLOCK_SDWIDEXFR | SDCARD_CLOCK_CLKEN);
        break;
#endif

      /* SD normal operation clocking (narrow 1-bit mode) */

      case CLOCK_SD_TRANSFER_1BIT:
        clkcr = (SDCARD_CLOCK_SDXFR | SDCARD_CLOCK_CLKEN);
        break;
    }

  /* Set the new clock frequency along with the clock enable/disable bit */

  lpc17_40_setclock(clkcr);
}

/****************************************************************************
 * Name: lpc17_40_attach
 *
 * Description:
 *   Attach and prepare interrupts
 *
 * Input Parameters:
 *   dev - An instance of the SD card device interface
 *
 * Returned Value:
 *   OK on success; A negated errno on failure.
 *
 ****************************************************************************/

static int lpc17_40_attach(struct sdio_dev_s *dev)
{
  int ret;

  /* Attach the SD card interrupt handler */

  ret = irq_attach(LPC17_40_IRQ_MCI, lpc17_40_interrupt, NULL);
  if (ret == OK)
    {
      /* Disable all interrupts at the SD card controller and clear static
       * interrupt flags
       */

      putreg32(SDCARD_MASK0_RESET,       LPC17_40_SDCARD_MASK0);
      putreg32(SDCARD_CLEAR_STATICFLAGS, LPC17_40_SDCARD_CLEAR);

      /* Enable SD card interrupts at the NVIC.  They can now be enabled at
       * the SD card controller as needed.
       */

      up_enable_irq(LPC17_40_IRQ_MCI);
    }

  return ret;
}

/****************************************************************************
 * Name: lpc17_40_sendcmd
 *
 * Description:
 *   Send the SD card command
 *
 * Input Parameters:
 *   dev  - An instance of the SD card device interface
 *   cmd  - The command to send (32-bits, encoded)
 *   arg  - 32-bit argument required with some commands
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int lpc17_40_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                            uint32_t arg)
{
  uint32_t regval;
  uint32_t cmdidx;

  /* Set the SD card Argument value */

  putreg32(arg, LPC17_40_SDCARD_ARG);

  /* Clear CMDINDEX, WAITRESP, WAITINT, WAITPEND, and CPSMEN bits */

  regval = getreg32(LPC17_40_SDCARD_CMD);
  regval &= ~(SDCARD_CMD_INDEX_MASK | SDCARD_CMD_WAITRESP_MASK |
              SDCARD_CMD_WAITINT | SDCARD_CMD_WAITPEND |
              SDCARD_CMD_CPSMEN);

  /* Set WAITRESP bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      regval |= SDCARD_CMD_NORESPONSE;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
    case MMCSD_R7_RESPONSE:
      regval |= SDCARD_CMD_SHORTRESPONSE;
      break;

    case MMCSD_R2_RESPONSE:
      regval |= SDCARD_CMD_LONGRESPONSE;
      break;
    }

  /* Set CPSMEN and the command index */

  cmdidx  = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval |= cmdidx | SDCARD_CMD_CPSMEN;

  mcinfo("cmd: %08" PRIx32 " arg: %08" PRIx32 " regval: %08" PRIx32 "\n",
         cmd, arg, regval);

  /* Write the SD card CMD */

  putreg32(SDCARD_RESPDONE_ICR | SDCARD_CMDDONE_ICR, LPC17_40_SDCARD_CLEAR);
  putreg32(regval, LPC17_40_SDCARD_CMD);
  return OK;
}

/****************************************************************************
 * Name: lpc17_40_recvsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card in non-DMA
 *   (interrupt driven mode).  This method will do whatever controller setup
 *   is necessary.  This would be called for SD memory just BEFORE sending
 *   CMD13 (SEND_STATUS), CMD17 (READ_SINGLE_BLOCK), CMD18
 *   (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR), etc.  Normally,
 *   SDCARD_WAITEVENT will be called to receive the indication that the
 *   transfer is complete.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *   buffer - Address of the buffer in which to receive the data
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc17_40_recvsetup(struct sdio_dev_s *dev,
                              uint8_t *buffer,
                              size_t nbytes)
{
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)dev;
  uint32_t dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  lpc17_40_datadisable();
  lpc17_40_sampleinit();
  lpc17_40_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
#ifdef CONFIG_LPC17_40_SDCARD_DMA
  priv->dmamode   = false;
#endif

  /* Then set up the SD card data path */

  dblocksize = lpc17_40_log2(nbytes) << SDCARD_DCTRL_DBLOCKSIZE_SHIFT;
  lpc17_40_dataconfig(SDCARD_DTIMER_DATATIMEOUT, nbytes,
                   dblocksize | SDCARD_DCTRL_DTDIR);

  /* And enable interrupts */

  lpc17_40_configxfrints(priv, SDCARD_RECV_MASK);
  lpc17_40_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}

/****************************************************************************
 * Name: lpc17_40_sendsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card.
 *   This method will do whatever controller setup is necessary.
 *   This would be called for SD memory just AFTER sending
 *   CMD24 (WRITE_BLOCK), CMD25 (WRITE_MULTIPLE_BLOCK), ... and before
 *   SDCARD_SENDDATA is called.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *   buffer - Address of the buffer containing the data to send
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc17_40_sendsetup(struct sdio_dev_s *dev,
                              const uint8_t *buffer,
                              size_t nbytes)
{
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)dev;
  uint32_t dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  lpc17_40_datadisable();
  lpc17_40_sampleinit();
  lpc17_40_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
#ifdef CONFIG_LPC17_40_SDCARD_DMA
  priv->dmamode   = false;
#endif

  /* Then set up the SD card data path */

  dblocksize = lpc17_40_log2(nbytes) << SDCARD_DCTRL_DBLOCKSIZE_SHIFT;
  lpc17_40_dataconfig(SDCARD_DTIMER_DATATIMEOUT, nbytes, dblocksize);

  /* Enable TX interrupts */

  lpc17_40_configxfrints(priv, SDCARD_SEND_MASK);
  lpc17_40_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}

/****************************************************************************
 * Name: lpc17_40_cancel
 *
 * Description:
 *   Cancel the data transfer setup of SDCARD_RECVSETUP, SDCARD_SENDSETUP,
 *   SDCARD_DMARECVSETUP or SDCARD_DMASENDSETUP.  This must be called to
 *   cancel the data transfer setup if, for some reason, you cannot perform
 *   the transfer.
 *
 * Input Parameters:
 *   dev  - An instance of the SD card device interface
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc17_40_cancel(struct sdio_dev_s *dev)
{
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)dev;

  /* Disable all transfer- and event- related interrupts */

  lpc17_40_configxfrints(priv, 0);
  lpc17_40_configwaitints(priv, 0, 0, 0);

  /* Clearing pending interrupt status on all transfer- and event- related
   * interrupts
   */

  putreg32(SDCARD_WAITALL_ICR, LPC17_40_SDCARD_CLEAR);

  /* Cancel any watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_LPC17_40_SDCARD_DMA
  if (priv->dmamode)
    {
      /* Make sure that the DMA is stopped (it will be stopped automatically
       * on normal transfers, but not necessarily when the transfer
       * terminates on an error condition.
       */

      lpc17_40_dmastop(priv->dma);
    }
#endif

  /* Mark no transfer in progress */

  priv->remaining = 0;
  return OK;
}

/****************************************************************************
 * Name: lpc17_40_waitresponse
 *
 * Description:
 *   Poll-wait for the response to the last command to be ready.
 *
 * Input Parameters:
 *   dev  - An instance of the SD card device interface
 *   cmd  - The command that was sent.  See 32-bit command definitions above.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc17_40_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  int32_t timeout;
  uint32_t events;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      events  = SDCARD_CMDDONE_STA;
      timeout = SDCARD_CMDTIMEOUT;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R6_RESPONSE:
      events  = SDCARD_RESPDONE_STA;
      timeout = SDCARD_LONGTIMEOUT;
      break;

    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
      return -ENOSYS;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      events  = SDCARD_RESPDONE_STA;
      timeout = SDCARD_CMDTIMEOUT;
      break;

    default:
      return -EINVAL;
    }

  /* Then wait for the response (or timeout) */

  while ((getreg32(LPC17_40_SDCARD_STATUS) & events) == 0)
    {
      if (--timeout <= 0)
        {
          mcerr("ERROR: Timeout cmd: %08" PRIx32
                " events: %08" PRIx32 " STA: %08" PRIx32 "\n",
                cmd, events, getreg32(LPC17_40_SDCARD_STATUS));

          return -ETIMEDOUT;
        }
    }

  putreg32(SDCARD_CMDDONE_ICR, LPC17_40_SDCARD_CLEAR);
  return OK;
}

/****************************************************************************
 * Name: lpc17_40_recv*
 *
 * Description:
 *   Receive response to SD card command.  Only the critical payload is
 *   returned -- that is 32 bits for 48 bit status and 128 bits for 136 bit
 *   status.  The driver implementation should verify the correctness of
 *   the remaining, non-returned bits (CRCs, CMD index, etc.).
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
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

static int lpc17_40_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                                 uint32_t *rshort)
{
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
      mcerr("ERROR: Wrong response CMD=%08" PRIx32 "\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = getreg32(LPC17_40_SDCARD_STATUS);
      if ((regval & SDCARD_STATUS_CTIMEOUT) != 0)
        {
          mcerr("ERROR: Command timeout: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & SDCARD_STATUS_CCRCFAIL) != 0)
        {
          mcerr("ERROR: CRC failure: %08" PRIx32 "\n", regval);
          ret = -EIO;
        }
#ifdef CONFIG_DEBUG_FEATURES
      else
        {
          /* Check response received is of desired command */

          respcmd = getreg32(LPC17_40_SDCARD_RESPCMD);
          if ((uint8_t)(respcmd & SDCARD_RESPCMD_MASK) !=
              (cmd & MMCSD_CMDIDX_MASK))
            {
              mcerr("ERROR: RESCMD=%02" PRIx32 " CMD=%08" PRIx32 "\n",
                    respcmd, cmd);
              ret = -EINVAL;
            }
        }
#endif
    }

  /* Clear all pending message completion events and return the R1/R6
   * response.
   */

  putreg32(SDCARD_RESPDONE_ICR | SDCARD_CMDDONE_ICR, LPC17_40_SDCARD_CLEAR);
  *rshort = getreg32(LPC17_40_SDCARD_RESP0);
  return ret;
}

static int lpc17_40_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
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

      regval = getreg32(LPC17_40_SDCARD_STATUS);
      if (regval & SDCARD_STATUS_CTIMEOUT)
        {
          mcerr("ERROR: Timeout STA: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & SDCARD_STATUS_CCRCFAIL)
        {
          mcerr("ERROR: CRC fail STA: %08" PRIx32 "\n", regval);
          ret = -EIO;
        }
    }

  /* Return the long response */

  putreg32(SDCARD_RESPDONE_ICR | SDCARD_CMDDONE_ICR, LPC17_40_SDCARD_CLEAR);
  if (rlong)
    {
      rlong[0] = getreg32(LPC17_40_SDCARD_RESP0);
      rlong[1] = getreg32(LPC17_40_SDCARD_RESP1);
      rlong[2] = getreg32(LPC17_40_SDCARD_RESP2);
      rlong[3] = getreg32(LPC17_40_SDCARD_RESP3);
    }

  return ret;
}

static int lpc17_40_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
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

#ifdef CONFIG_DEBUG_FEATURES
  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
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

      regval = getreg32(LPC17_40_SDCARD_STATUS);
      if (regval & SDCARD_STATUS_CTIMEOUT)
        {
          mcerr("ERROR: Timeout STA: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
    }

  putreg32(SDCARD_RESPDONE_ICR | SDCARD_CMDDONE_ICR, LPC17_40_SDCARD_CLEAR);
  if (rshort)
    {
      *rshort = getreg32(LPC17_40_SDCARD_RESP0);
    }

  return ret;
}

/* MMC responses not supported */

static int lpc17_40_recvnotimpl(struct sdio_dev_s *dev, uint32_t cmd,
                                uint32_t *rnotimpl)
{
  putreg32(SDCARD_RESPDONE_ICR | SDCARD_CMDDONE_ICR, LPC17_40_SDCARD_CLEAR);
  return -ENOSYS;
}

/****************************************************************************
 * Name: lpc17_40_waitenable
 *
 * Description:
 *   Enable/disable of a set of SD card wait events.  This is part of the
 *   the SDCARD_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling lpc17_40_eventwait.  This is done in this way
 *   to help the driver to eliminate race conditions between the command
 *   setup and the subsequent events.
 *
 *   The enabled events persist until either (1) SDCARD_WAITENABLE is called
 *   again specifying a different set of wait events, or (2) SDCARD_EVENTWAIT
 *   returns.
 *
 * Input Parameters:
 *   dev      - An instance of the SD card device interface
 *   eventset - A bitset of events to enable or disable (see SDIOWAIT_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc17_40_waitenable(struct sdio_dev_s *dev,
                             sdio_eventset_t eventset, uint32_t timeout)
{
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)dev;
  uint32_t waitmask;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  lpc17_40_configwaitints(priv, 0, 0, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

  waitmask = 0;
  if ((eventset & SDIOWAIT_CMDDONE) != 0)
    {
      waitmask |= SDCARD_CMDDONE_MASK;
    }

  if ((eventset & SDIOWAIT_RESPONSEDONE) != 0)
    {
      waitmask |= SDCARD_RESPDONE_MASK;
    }

  if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
    {
      waitmask |= SDCARD_XFRDONE_MASK;
    }

  /* Enable event-related interrupts */

  putreg32(SDCARD_WAITALL_ICR, LPC17_40_SDCARD_CLEAR);
  lpc17_40_configwaitints(priv, waitmask, eventset, 0);

  /* Check if the timeout event is specified in the event set */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      int delay;

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
                       lpc17_40_eventtimeout, (wdparm_t)priv);
      if (ret < 0)
        {
          mcerr("ERROR: wd_start failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: lpc17_40_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDCARD_WAITEVENTS are disabled when
 *   lpc17_40_eventwait returns.  SDCARD_WAITEVENTS must be called again
 *   before lpc17_40_eventwait can be used again.
 *
 * Input Parameters:
 *   dev     - An instance of the SD card device interface
 *   timeout - Maximum time in milliseconds to wait.  Zero means immediate
 *             timeout with no wait.  The timeout value is ignored if
 *             SDIOWAIT_TIMEOUT is not included in the waited-for eventset.
 *
 * Returned Value:
 *   Event set containing the event(s) that ended the wait.  Should always
 *   be non-zero.  All events are disabled after the wait concludes.
 *
 ****************************************************************************/

static sdio_eventset_t lpc17_40_eventwait(struct sdio_dev_s *dev)
{
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)dev;
  sdio_eventset_t wkupevent = 0;
  irqstate_t flags;
  int ret;

  /* There is a race condition here... the event may have completed before
   * we get here.  In this case waitevents will be zero, but wkupevents will
   * be non-zero (and, hopefully, the semaphore count will also be non-zero.
   */

  flags = enter_critical_section();
  DEBUGASSERT(priv->waitevents != 0 || priv->wkupevent != 0);

  /* Loop until the event (or the timeout occurs). Race conditions are
   * avoided by calling lpc17_40_waitenable prior to triggering the logic
   * that will cause the wait to terminate.  Under certain race conditions,
   * the waited-for may have already occurred before this function was
   * called!
   */

  for (; ; )
    {
      /* Wait for an event in event set to occur.  If this the event has
       * already occurred, then the semaphore will already have been
       * incremented and there will be no wait.
       */

      ret = lpc17_40_takesem(priv);
      if (ret < 0)
        {
          /* Task canceled.  Cancel the wdog (assuming it was started) and
           * return an SDIO error.
           */

          wd_cancel(&priv->waitwdog);
          leave_critical_section(flags);
          return SDIOWAIT_ERROR;
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

  lpc17_40_configwaitints(priv, 0, 0, 0);
#ifdef CONFIG_LPC17_40_SDCARD_DMA
  priv->xfrflags   = 0;
#endif

  leave_critical_section(flags);
  lpc17_40_dumpsamples(priv);
  return wkupevent;
}

/****************************************************************************
 * Name: lpc17_40_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SD card callback events.  This is part of the
 *   the SD card callback sequence.  The set of events is configured to
 *   enabled callbacks to the function provided in lpc17_40_registercallback.
 *
 *   Events are automatically disabled once the callback is performed and no
 *   further callback events will occur until they are again enabled by
 *   calling this method.
 *
 * Input Parameters:
 *   dev      - An instance of the SD card device interface
 *   eventset - A bitset of events to enable or disable (see SDIOMEDIA_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc17_40_callbackenable(struct sdio_dev_s *dev,
                                    sdio_eventset_t eventset)
{
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)dev;

  mcinfo("eventset: %02x\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  lpc17_40_callback(priv);
}

/****************************************************************************
 * Name: lpc17_40_registercallback
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change.  Callbacks should not be made from interrupt handlers, rather
 *   interrupt level events should be handled by calling back on the work
 *   thread.
 *
 *   When this method is called, all callbacks should be disabled until they
 *   are enabled via a call to SDCARD_CALLBACKENABLE
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

static int lpc17_40_registercallback(struct sdio_dev_s *dev,
                                     worker_t callback, void *arg)
{
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)dev;

  /* Disable callbacks and register this callback and is argument */

  mcinfo("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: lpc17_40_dmarecvsetup
 *
 * Description:
 *   Setup to perform a read DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For read transfers this may mean
 *   invalidating the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *   buffer - The memory to DMA from
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_SDCARD_DMA
static int lpc17_40_dmarecvsetup(struct sdio_dev_s *dev,
                                 uint8_t *buffer,
                                 size_t buflen)
{
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)dev;
  uint32_t dblocksize;
  uint32_t regval;
  int ret = -EINVAL;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  lpc17_40_datadisable();

  /* Wide bus operation is required for DMA */

  if (priv->widebus)
    {
      lpc17_40_sampleinit();
      lpc17_40_sample(priv, SAMPLENDX_BEFORE_SETUP);

      /* Save the destination buffer information for use by the interrupt
       * handler.
       */

      priv->buffer    = (uint32_t *)buffer;
      priv->remaining = buflen;
      priv->dmamode   = true;

      /* Then set up the SD card data path */

      dblocksize = lpc17_40_log2(buflen) << SDCARD_DCTRL_DBLOCKSIZE_SHIFT;
      lpc17_40_dataconfig(SDCARD_DTIMER_DATATIMEOUT, buflen,
                       dblocksize | SDCARD_DCTRL_DTDIR);

      /* Configure the RX DMA */

      lpc17_40_configxfrints(priv, SDCARD_DMARECV_MASK);

      regval  = getreg32(LPC17_40_SDCARD_DCTRL);
      regval |= SDCARD_DCTRL_DMAEN;
      putreg32(regval, LPC17_40_SDCARD_DCTRL);

      ret = lpc17_40_dmasetup(priv->dma, SDCARD_RXDMA32_CONTROL,
                           SDCARD_RXDMA32_CONFIG, LPC17_40_SDCARD_FIFO,
                           (uint32_t)buffer, (buflen + 3) >> 2);
      if (ret == OK)
        {
          /* Start the DMA */

          lpc17_40_sample(priv, SAMPLENDX_BEFORE_ENABLE);
          lpc17_40_dmastart(priv->dma, lpc17_40_dmacallback, priv);
          lpc17_40_sample(priv, SAMPLENDX_AFTER_SETUP);
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: lpc17_40_dmasendsetup
 *
 * Description:
 *   Setup to perform a write DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For write transfers, this may mean
 *   flushing the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *   buffer - The memory to DMA into
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_SDCARD_DMA
static int lpc17_40_dmasendsetup(struct sdio_dev_s *dev,
                                 const uint8_t *buffer, size_t buflen)
{
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)dev;
  uint32_t dblocksize;
  uint32_t regval;
  int ret = -EINVAL;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  lpc17_40_datadisable();

  /* Wide bus operation is required for DMA */

  if (priv->widebus)
    {
      lpc17_40_sampleinit();
      lpc17_40_sample(priv, SAMPLENDX_BEFORE_SETUP);

      /* Save the source buffer information for use by the interrupt
       * handler.
       */

      priv->buffer    = (uint32_t *)buffer;
      priv->remaining = buflen;
      priv->dmamode   = true;

      /* Then set up the SD card data path */

      dblocksize = lpc17_40_log2(buflen) << SDCARD_DCTRL_DBLOCKSIZE_SHIFT;
      lpc17_40_dataconfig(SDCARD_DTIMER_DATATIMEOUT, buflen, dblocksize);

      /* Configure the TX DMA */

      ret = lpc17_40_dmasetup(priv->dma, SDCARD_TXDMA32_CONTROL,
                           SDCARD_TXDMA32_CONFIG, (uint32_t)buffer,
                           LPC17_40_SDCARD_FIFO, (buflen + 3) >> 2);
      if (ret == OK)
        {
          lpc17_40_sample(priv, SAMPLENDX_BEFORE_ENABLE);

          regval  = getreg32(LPC17_40_SDCARD_DCTRL);
          regval |= SDCARD_DCTRL_DMAEN;
          putreg32(regval, LPC17_40_SDCARD_DCTRL);

          /* Start the DMA */

          lpc17_40_dmastart(priv->dma, lpc17_40_dmacallback, priv);
          lpc17_40_sample(priv, SAMPLENDX_AFTER_SETUP);

          /* Enable TX interrupts */

          lpc17_40_configxfrints(priv, SDCARD_DMASEND_MASK);
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Initialization/uninitialization/reset
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_callback
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

static void lpc17_40_callback(void *arg)
{
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)arg;

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
       * handler. If we are in an interrupt handler, then queue the
       * callback to be performed later on the work thread.
       */

      if (up_interrupt_context())
        {
          /* Yes.. queue it */

           mcinfo("Queuing callback to %p(%p)\n", priv->callback,
                                                  priv->cbarg);
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
 * Name: lpc17_40_default
 *
 * Description:
 *   Restore SD card registers to their default, reset values
 *
 ****************************************************************************/

static void lpc17_40_default(void)
{
  putreg32(SDCARD_PWR_RESET,    LPC17_40_SDCARD_PWR);
  putreg32(SDCARD_CLOCK_RESET,  LPC17_40_SDCARD_CLOCK);
  putreg32(SDCARD_ARG_RESET,    LPC17_40_SDCARD_ARG);
  putreg32(SDCARD_CMD_RESET,    LPC17_40_SDCARD_CMD);
  putreg32(SDCARD_DTIMER_RESET, LPC17_40_SDCARD_DTIMER);
  putreg32(SDCARD_DLEN_RESET,   LPC17_40_SDCARD_DLEN);
  putreg32(SDCARD_DCTRL_RESET,  LPC17_40_SDCARD_DCTRL);
  putreg32(SDCARD_CLEAR_RESET,  LPC17_40_SDCARD_CLEAR);
  putreg32(SDCARD_MASK0_RESET,  LPC17_40_SDCARD_MASK0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SD card for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Value:
 *   A reference to an SD card interface structure.  NULL is returned on
 *   failures.
 *
 ****************************************************************************/

struct sdio_dev_s *sdio_initialize(int slotno)
{
  uint32_t   regval;

  /* There is only one slot */

  struct lpc17_40_dev_s *priv = &g_scard_dev;

  /* Enable power on SD Interface */

  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCSDC;
  putreg32(regval, LPC17_40_SYSCON_PCONP);

  /* Initialize the SD card slot structure */

  /* Initialize semaphores */

  nxsem_init(&priv->waitsem, 0, 0);

  /* The waitsem semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_set_protocol(&priv->waitsem, SEM_PRIO_NONE);

#ifdef CONFIG_LPC17_40_SDCARD_DMA
  /* Configure the SDCARD DMA request */

  lpc17_40_dmaconfigure(DMA_REQ_SDCARD, DMA_DMASEL_SDCARD);

  /* Allocate a DMA channel for SDCARD DMA */

  priv->dma = lpc17_40_dmachannel();
  DEBUGASSERT(priv->dma);
#endif

  /* Configure GPIOs for 4-bit, wide-bus operation.
   *
   * If bus is multiplexed then there is a custom bus configuration utility
   * in the scope of the board support package.
   */

#ifndef CONFIG_SDIO_MUXBUS
  lpc17_40_configgpio(GPIO_SD_DAT0);
#ifndef CONFIG_LPC17_40_SDCARD_WIDTH_D1_ONLY
  lpc17_40_configgpio(GPIO_SD_DAT1);
  lpc17_40_configgpio(GPIO_SD_DAT2);
  lpc17_40_configgpio(GPIO_SD_DAT3);
#endif
  lpc17_40_configgpio(GPIO_SD_CLK);
  lpc17_40_configgpio(GPIO_SD_CMD);
#endif

  /* Reset the card and assure that it is in the initial, unconfigured
   * state.
   */

  lpc17_40_reset(&priv->dev);

  return &g_scard_dev.dev;
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
 *   dev        - An instance of the SD card driver device state structure.
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
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)dev;
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
      lpc17_40_callback(priv);
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
 *   dev       - An instance of the SD card driver device state structure.
 *   wrprotect - true is a card is writeprotected.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sdio_wrprotect(struct sdio_dev_s *dev, bool wrprotect)
{
  struct lpc17_40_dev_s *priv = (struct lpc17_40_dev_s *)dev;
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
#endif /* CONFIG_LPC17_40_SDCARD */
