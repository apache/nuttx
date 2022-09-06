/****************************************************************************
 * arch/arm/src/stm32h7/stm32_sdmmc.c
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
#include <nuttx/compiler.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/mmcsd.h>
#include <nuttx/irq.h>
#include <nuttx/cache.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_dtcm.h"
#include "stm32_dma.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "stm32_sdmmc.h"

#if defined(CONFIG_STM32H7_SDMMC1) || defined(CONFIG_STM32H7_SDMMC2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* HW Issues when not using IDMA
 *
 *    The FIFO lacks a FIFO data available interrupt. It only has a
 * FIFO Half Full and Full interrupt. Given that the FIFO size is 64
 * bytes (16 uint 32), this creates an issue for commands that read
 * less than 32 bytes hereafter referred to as a Lame FIFO.
 *
 * The DATAEND interrupt is only issued on the FIFO being emptied and it
 * appears the DTIMER does not start running again until the FIFO is read
 * so that DTIMEOUT interrupt can not be used to end the read operation.
 *
 * The only workaround is to use the IDMA to service the FIFO and to monitor
 * for a remainder in the FIFO that is less than FIFO_SIZE_IN_BYTES / 2.
 *
 * Therefore even when IDMA is not used, the IDMA still has to be used to
 * service the FIFO to receive less than FIFO_SIZE_IN_BYTES / 2 to a local
 * buffer. When the FIFO Half Full and Full interrupt do fire the FIFO will
 * be monitored off the an HP work thread for a residual of less than
 * FIFO_SIZE_IN_BYTES / 2.
 *
 * HW Issues when using IDMA
 *
 *    The DMA buffer must be located in a zone accessible via IDMA.
 * For SDMMC1, IDMA cannot access SRAM123 or SRAM4. Refer to ST AN5200.
 * Buffer validity is checked when CONFIG_ARCH_HAVE_SDIO_PREFLIGHT is set.
 *
 * MDMA is only available on for SDMMC1 and Not supported at this time.
 *
 * Required system configuration options:
 *
 *   CONFIG_SCHED_WORKQUEUE -- Callback support requires work queue support.
 *
 * Driver-specific configuration options:
 *
 *   CONFIG_SDIO_MUXBUS - Setting this configuration enables some locking
 *     APIs to manage concurrent accesses on the SDMMC bus.  This is not
 *     needed for the simple case of a single SD card, for example.
 *   CONFIG_STM32H7_SDMMC_IDMA - Enable SDMMC IDMA.
 *     DMA support for SDMMC. If disabled, the SDMMC will work in
 *     interrupt mode and still use the IDMA to a local buffer for data
 *     lengths less the 32 bytes due to the FIFO limitations.
 *   CONFIG_SDMMC1/2_WIDTH_D1_ONLY - This may be selected to force the driver
 *     operate with only a single data line (the default is to use all
 *     4 SD data lines).
 *   CONFIG_STM32H7_SDMMC_XFRDEBUG - Enables some very low-level debug
 *     output This also requires CONFIG_DEBUG_FS and CONFIG_DEBUG_INFO
 *   CONFIG_SDMMC1/2_SDIO_MODE
 *     Build ins additional support needed only for SDIO cards (vs. SD memory
 *     cards)
 *   CONFIG_SDMMC1/2_SDIO_PULLUP
 *      If you are using an external SDCard module that does not have the
 *      pull-up resistors for the SDIO interface (like the Gadgeteer SD Card
 *      Module) then enable this option to activate the internal pull-up
 *      resistors.
 */

/* If there are 2 SDMMC enabled, then Slot 0 is SDMMC1, and Slot 1 is SDMMC2
 * If there is only 1 SDMMC, then Slot 0 is assigned to the defined SDMMC
 * hence, if only SDMMC2 is defined it will be slot 0.
 */

#if !defined(CONFIG_STM32H7_SDMMC1)
#  define SDMMC2_SLOT  0
#else
#  define SDMMC2_SLOT  1
#endif

#if !defined(CONFIG_STM32H7_SDMMC_IDMA)
#  warning "Large Non-DMA transfer may result in RX overrun failures"
#elif defined(CONFIG_STM32H7_SDMMC1)
#  define SRAM123_START STM32_SRAM123_BASE
#  define SRAM123_END   (SRAM123_START + STM32H7_SRAM123_SIZE)
#  define SRAM4_START   STM32_SRAM4_BASE
#  define SRAM4_END     (SRAM4_START + STM32H7_SRAM4_SIZE)
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE"
#endif

#undef HAVE_SDMMC_SDIO_MODE
#if defined(CONFIG_SDMMC1_SDIO_MODE) || defined(CONFIG_SDMMC2_SDIO_MODE)
#  define HAVE_SDMMC_SDIO_MODE
#endif

#if !defined(CONFIG_DEBUG_FS) || !defined(CONFIG_DEBUG_FEATURES)
#  undef CONFIG_STM32H7_SDMMC_XFRDEBUG
#endif

#ifdef CONFIG_SDMMC1_SDIO_PULLUP
#  define SDMMC1_SDIO_PULL(g)  (((g) & ~GPIO_PUPD_MASK) | GPIO_PULLUP)
#else
#  define SDMMC1_SDIO_PULL(g)  (((g) & ~GPIO_PUPD_MASK) | GPIO_FLOAT)
#endif

#ifdef CONFIG_SDMMC2_SDIO_PULLUP
#  define SDMMC2_SDIO_PULL(g)  (((g) & ~GPIO_PUPD_MASK) | GPIO_PULLUP)
#else
#  define SDMMC2_SDIO_PULL(g)  (((g) & ~GPIO_PUPD_MASK) | GPIO_FLOAT)
#endif

/* Define the Hardware FIFO size */

#define FIFO_SIZE_IN_BYTES        64

/* Friendly Clock source & CLKCR bit re-definitions *************************/

/* If not set in board use default pll1_q_ck clock is selected as
 * kernel peripheral clock (default after reset)
 */

#if !defined(STM32_RCC_D1CCIPR_SDMMCSEL)
#  define STM32_RCC_D1CCIPR_SDMMCSEL  RCC_D1CCIPR_SDMMC_PLL1
#endif

#if STM32_RCC_D1CCIPR_SDMMCSEL  == RCC_D1CCIPR_SDMMC_PLL1
#  define STM32_SDMMC_CLK  STM32_PLL1Q_FREQUENCY
#else
#  define STM32_SDMMC_CLK  STM32_PLL2R_FREQUENCY
#endif

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

#define STM32_CLCKCR_INIT           (STM32_SDMMC_INIT_CLKDIV      |     \
                                     STM32_SDMMC_CLKCR_EDGE       |     \
                                     STM32_SDMMC_CLKCR_WIDBUS_D1)
#define STM32_SDMMC_CLKCR_MMCXFR    (STM32_SDMMC_MMCXFR_CLKDIV    |     \
                                     STM32_SDMMC_CLKCR_EDGE       |     \
                                     STM32_SDMMC_CLKCR_PWRSAV     |     \
                                     STM32_SDMMC_CLKCR_WIDBUS_D1)
#define STM32_SDMMC_CLCKR_SDXFR     (STM32_SDMMC_SDXFR_CLKDIV     |     \
                                     STM32_SDMMC_CLKCR_EDGE       |     \
                                     STM32_SDMMC_CLKCR_PWRSAV     |     \
                                     STM32_SDMMC_CLKCR_WIDBUS_D1)
#ifdef HAVE_SDMMC_SDIO_MODE
/* Do not enable power saving configuration bit (in SD 4-bit mode) because
 * the SDIO clock is not enabled when the bus goes to the idle state.
 * This condition breaks interrupts delivering mechanism over DAT[1]/IRQ
 * SDIO line to the host.
 */
#  define STM32_SDMMC_CLCKR_SDWIDEXFR (STM32_SDMMC_SDXFR_CLKDIV     |     \
                                       STM32_SDMMC_CLKCR_EDGE       |     \
                                       STM32_SDMMC_CLKCR_WIDBUS_D4)
#else
#  define STM32_SDMMC_CLCKR_SDWIDEXFR (STM32_SDMMC_SDXFR_CLKDIV     |     \
                                       STM32_SDMMC_CLKCR_EDGE       |     \
                                       STM32_SDMMC_CLKCR_PWRSAV     |     \
                                       STM32_SDMMC_CLKCR_WIDBUS_D4)
#endif

/* Timing */

#define SDMMC_CMDTIMEOUT         (100000)
#define SDMMC_LONGTIMEOUT        (0x7fffffff)

/* DTIMER setting */

#define SDMMC_DTIMER_DATATIMEOUT_MS  250

/* Block size for multi-block transfers */

#define SDMMC_MAX_BLOCK_SIZE          (512)

/* Data transfer interrupt mask bits */

/* DMA interrupts */

#define STM32_SDMMC_DMARECV_MASK  (STM32_SDMMC_MASK_DCRCFAILIE |        \
                                   STM32_SDMMC_MASK_DTIMEOUTIE |        \
                                   STM32_SDMMC_MASK_DATAENDIE  |        \
                                   STM32_SDMMC_MASK_RXOVERRIE)

#define STM32_SDMMC_DMASEND_MASK  (STM32_SDMMC_MASK_DCRCFAILIE |        \
                                   STM32_SDMMC_MASK_DTIMEOUTIE |        \
                                   STM32_SDMMC_MASK_DATAENDIE  |        \
                                   STM32_SDMMC_MASK_TXUNDERRIE)
#define STM32_SDMMC_XFRDONE_MASK  (0)

/* Interrupt mode */

#define STM32_SDMMC_RECV_MASK     (STM32_SDMMC_MASK_DCRCFAILIE |        \
                                   STM32_SDMMC_MASK_DTIMEOUTIE |        \
                                   STM32_SDMMC_MASK_DATAENDIE  |        \
                                   STM32_SDMMC_MASK_RXOVERRIE  |        \
                                   STM32_SDMMC_MASK_RXFIFOHFIE)

#define STM32_SDMMC_SEND_MASK     (STM32_SDMMC_MASK_DCRCFAILIE |        \
                                   STM32_SDMMC_MASK_DTIMEOUTIE |        \
                                   STM32_SDMMC_MASK_DATAENDIE  |        \
                                   STM32_SDMMC_MASK_TXFIFOHEIE |        \
                                   STM32_SDMMC_MASK_TXUNDERRIE)

/* Event waiting interrupt mask bits */

#define STM32_SDMMC_CMDDONE_STA   (STM32_SDMMC_STA_CMDSENT)

#define STM32_SDMMC_RESPDONE_STA  (STM32_SDMMC_STA_CTIMEOUT |    \
                                   STM32_SDMMC_STA_CCRCFAIL |    \
                                   STM32_SDMMC_STA_CMDREND)

#define STM32_SDMMC_CMDDONE_MASK  (STM32_SDMMC_MASK_CMDSENTIE)

#define STM32_SDMMC_RESPDONE_MASK (STM32_SDMMC_MASK_CCRCFAILIE | \
                                   STM32_SDMMC_MASK_CTIMEOUTIE | \
                                   STM32_SDMMC_MASK_CMDRENDIE)

#define STM32_SDMMC_XFRDONE_MASK  (0)

#define STM32_SDMMC_CMDDONE_ICR   (STM32_SDMMC_ICR_CMDSENTC |    \
                                   STM32_SDMMC_ICR_DBCKENDC)

#define STM32_SDMMC_RESPDONE_ICR  (STM32_SDMMC_ICR_CTIMEOUTC |   \
                                   STM32_SDMMC_ICR_CCRCFAILC |   \
                                   STM32_SDMMC_ICR_CMDRENDC  |   \
                                   STM32_SDMMC_ICR_DBCKENDC)

#define STM32_SDMMC_XFRDONE_ICR   (STM32_SDMMC_ICR_DATAENDC  |   \
                                   STM32_SDMMC_ICR_DCRCFAILC |   \
                                   STM32_SDMMC_ICR_DTIMEOUTC |   \
                                   STM32_SDMMC_ICR_RXOVERRC  |   \
                                   STM32_SDMMC_ICR_TXUNDERRC |   \
                                   STM32_SDMMC_ICR_DBCKENDC)

#define STM32_SDMMC_WAITALL_ICR   (STM32_SDMMC_CMDDONE_ICR   |   \
                                   STM32_SDMMC_RESPDONE_ICR  |   \
                                   STM32_SDMMC_XFRDONE_ICR   |   \
                                   STM32_SDMMC_ICR_DBCKENDC)

/* Let's wait until we have both SDIO transfer complete and DMA complete. */

#define SDMMC_XFRDONE_FLAG  (1)
#define SDMMC_DMADONE_FLAG  (2)
#define SDMMC_ALLDONE       (3)

/* Register logging support */

#if defined(CONFIG_STM32H7_SDMMC_XFRDEBUG)
#  define SAMPLENDX_BEFORE_SETUP  0
#  define SAMPLENDX_AFTER_SETUP   1
#  define SAMPLENDX_END_TRANSFER  2
#  define DEBUG_NSAMPLES          3
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
#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  uint32_t          d0_gpio;
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

#if defined(HAVE_SDMMC_SDIO_MODE)
  /* Interrupt at SDIO_D1 pin, only for SDIO cards */

  uint32_t           sdiointmask;            /* STM32 SDIO register mask */
  int               (*do_sdio_card)(void *); /* SDIO card ISR */
  void               *do_sdio_arg;           /* arg for SDIO card ISR */
  bool               sdiomode;               /* True: in SDIO mode */
#endif

  /* Misc */

  uint32_t           blocksize;       /* Current block size */
  uint32_t           receivecnt;      /* Real count to receive */
#if !defined(CONFIG_STM32H7_SDMMC_IDMA)
  struct work_s      cbfifo;          /* Monitor for Lame FIFO */
#endif
  uint8_t            rxfifo[FIFO_SIZE_IN_BYTES] /* To offload with IDMA and support un-alinged buffers */
                     aligned_data(ARMV7M_DCACHE_LINESIZE);
  bool               unaligned_rx; /* read buffer is not cache-line or 32 bit aligned */

  /* Input dma buffer for unaligned transfers */
#if defined(CONFIG_STM32H7_SDMMC_IDMA)
  uint8_t sdmmc_rxbuffer[SDMMC_MAX_BLOCK_SIZE]
          aligned_data(ARMV7M_DCACHE_LINESIZE);
#endif
};

/* Register logging support */

#if defined(CONFIG_STM32H7_SDMMC_XFRDEBUG)
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
};

struct stm32_sampleregs_s
{
  struct stm32_sdioregs_s sdio;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static inline void sdmmc_putreg32(struct stm32_dev_s *priv, uint32_t value,
                                  int offset);
static inline uint32_t sdmmc_getreg32(struct stm32_dev_s *priv, int offset);
static inline void stm32_setclkcr(struct stm32_dev_s *priv, uint32_t clkcr);
static void stm32_configwaitints(struct stm32_dev_s *priv, uint32_t waitmask,
                                 sdio_eventset_t waitevents,
                                 sdio_eventset_t wkupevents);
static void stm32_configxfrints(struct stm32_dev_s *priv, uint32_t xfrmask);
static void stm32_setpwrctrl(struct stm32_dev_s *priv, uint32_t pwrctrl);

/* Debug Helpers ************************************************************/

#if defined(CONFIG_STM32H7_SDMMC_XFRDEBUG)
static void stm32_sampleinit(void);
static void stm32_sdiosample(struct stm32_dev_s *priv,
                             struct stm32_sdioregs_s *regs);
static void stm32_sample(struct stm32_dev_s *priv, int index);
static void stm32_sdiodump(struct stm32_sdioregs_s *regs, const char *msg);
static void stm32_dumpsample(struct stm32_dev_s *priv,
                             struct stm32_sampleregs_s *regs,
                             const char *msg);
static void stm32_dumpsamples(struct stm32_dev_s *priv);
#else
#  define   stm32_sampleinit()
#  define   stm32_sample(priv,index)
#  define   stm32_dumpsamples(priv)
#endif

/* Data Transfer Helpers ****************************************************/

static uint8_t stm32_log2(uint16_t value);
static void stm32_dataconfig(struct stm32_dev_s *priv, uint32_t timeout,
                             uint32_t dlen, bool receive);
static void stm32_datadisable(struct stm32_dev_s *priv);
#ifndef CONFIG_STM32H7_SDMMC_IDMA
static void stm32_sendfifo(struct stm32_dev_s *priv);
static void stm32_recvfifo(struct stm32_dev_s *priv);
#else
static void stm32_recvdma(struct stm32_dev_s *priv);
#endif
static void stm32_eventtimeout(wdparm_t arg);
static void stm32_endwait(struct stm32_dev_s *priv,
                          sdio_eventset_t wkupevent);
static void stm32_endtransfer(struct stm32_dev_s *priv,
                              sdio_eventset_t wkupevent);

/* Interrupt Handling *******************************************************/

static int  stm32_sdmmc_interrupt(int irq, void *context, void *arg);
#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
static int  stm32_sdmmc_rdyinterrupt(int irq, void *context, void *arg);
#endif

/* SDIO interface methods ***************************************************/

/* Mutual exclusion */

#if defined(CONFIG_SDIO_MUXBUS)
static int stm32_lock(struct sdio_dev_s *dev, bool lock);
#endif

/* Initialization/setup */

static void stm32_reset(struct sdio_dev_s *dev);
static sdio_capset_t stm32_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t stm32_status(struct sdio_dev_s *dev);
static void stm32_widebus(struct sdio_dev_s *dev, bool enable);
static void stm32_clock(struct sdio_dev_s *dev,
                        enum sdio_clock_e rate);
static int  stm32_attach(struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int  stm32_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                          uint32_t arg);
static void stm32_blocksetup(struct sdio_dev_s *dev,
              unsigned int blocksize, unsigned int nblocks);
#ifndef CONFIG_STM32H7_SDMMC_IDMA
static int  stm32_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                            size_t nbytes);
static int  stm32_sendsetup(struct sdio_dev_s *dev,
                            const uint8_t *buffer, size_t nbytes);
#endif
static int  stm32_cancel(struct sdio_dev_s *dev);

static int  stm32_waitresponse(struct sdio_dev_s *dev, uint32_t cmd);
static int  stm32_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                               uint32_t *rshort);
static int  stm32_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t rlong[4]);
static int  stm32_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                            uint32_t *rshort);

/* EVENT handler */

static void stm32_waitenable(struct sdio_dev_s *dev,
                             sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t stm32_eventwait(struct sdio_dev_s *dev);
static void stm32_callbackenable(struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset);
static int  stm32_registercallback(struct sdio_dev_s *dev,
                                   worker_t callback, void *arg);

/* DMA */

#if defined(CONFIG_STM32H7_SDMMC_IDMA)
#  if defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
static int  stm32_dmapreflight(struct sdio_dev_s *dev,
                               const uint8_t *buffer, size_t buflen);
#  endif
static int  stm32_dmarecvsetup(struct sdio_dev_s *dev,
                               uint8_t *buffer, size_t buflen);
static int  stm32_dmasendsetup(struct sdio_dev_s *dev,
                               const uint8_t *buffer, size_t buflen);
#endif

/* Initialization/uninitialization/reset ************************************/

static void stm32_callback(void *arg);
static void stm32_default(struct stm32_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/
#if defined(CONFIG_STM32H7_SDMMC1)
struct stm32_dev_s g_sdmmcdev1 =
{
  .dev =
  {
#if defined(CONFIG_SDIO_MUXBUS)
    .lock             = stm32_lock,
#endif
    .reset            = stm32_reset,
    .capabilities     = stm32_capabilities,
    .status           = stm32_status,
    .widebus          = stm32_widebus,
    .clock            = stm32_clock,
    .attach           = stm32_attach,
    .sendcmd          = stm32_sendcmd,
    .blocksetup       = stm32_blocksetup,
#if defined(CONFIG_STM32H7_SDMMC_IDMA)
    .recvsetup        = stm32_dmarecvsetup,
    .sendsetup        = stm32_dmasendsetup,
#else
    .recvsetup        = stm32_recvsetup,
    .sendsetup        = stm32_sendsetup,
#endif
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
#if defined(CONFIG_STM32H7_SDMMC_IDMA)
#  if defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
    .dmapreflight     = stm32_dmapreflight,
#  endif
    .dmarecvsetup     = stm32_dmarecvsetup,
    .dmasendsetup     = stm32_dmasendsetup,
#endif
  },
  .base              = STM32_SDMMC1_BASE,
  .nirq              = STM32_IRQ_SDMMC1,
#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  .d0_gpio           = SDMMC1_SDIO_PULL(GPIO_SDMMC1_D0),
#endif
  .waitsem            = SEM_INITIALIZER(0),
#if defined(HAVE_SDMMC_SDIO_MODE) && defined(CONFIG_SDMMC1_SDIO_MODE)
  .sdiomode          = true,
#endif
};
#endif
#if defined(CONFIG_STM32H7_SDMMC2)
struct stm32_dev_s g_sdmmcdev2 =
{
  .dev =
  {
#if defined(CONFIG_SDIO_MUXBUS)
    .lock             = stm32_lock,
#endif
    .reset            = stm32_reset,
    .capabilities     = stm32_capabilities,
    .status           = stm32_status,
    .widebus          = stm32_widebus,
    .clock            = stm32_clock,
    .attach           = stm32_attach,
    .sendcmd          = stm32_sendcmd,
    .blocksetup       = stm32_blocksetup,
#if defined(CONFIG_STM32H7_SDMMC_IDMA)
    .recvsetup        = stm32_dmarecvsetup,
    .sendsetup        = stm32_dmasendsetup,
#else
    .recvsetup        = stm32_recvsetup,
    .sendsetup        = stm32_sendsetup,
#endif
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
#if defined(CONFIG_STM32H7_SDMMC_IDMA)
#  if defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
    .dmapreflight     = stm32_dmapreflight,
#  endif
    .dmarecvsetup     = stm32_dmarecvsetup,
    .dmasendsetup     = stm32_dmasendsetup,
#endif
  },
  .base              = STM32_SDMMC2_BASE,
  .nirq              = STM32_IRQ_SDMMC2,
#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  .d0_gpio           = SDMMC2_SDIO_PULL(GPIO_SDMMC2_D0),
#endif
  .waitsem            = SEM_INITIALIZER(0),
#if defined(HAVE_SDMMC_SDIO_MODE) && defined(CONFIG_SDMMC2_SDIO_MODE)
  .sdiomode          = true,
#endif
};
#endif
/* Register logging support */

#if defined(CONFIG_STM32H7_SDMMC_XFRDEBUG)
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

  /* Clear CLKDIV, PWRSAV, WIDBUS, NEGEDGE, HWFC_EN bits */

  regval &= ~(STM32_SDMMC_CLKCR_CLKDIV_MASK | STM32_SDMMC_CLKCR_PWRSAV      |
              STM32_SDMMC_CLKCR_WIDBUS_MASK | STM32_SDMMC_CLKCR_NEGEDGE     |
              STM32_SDMMC_CLKCR_HWFC_EN     | STM32_SDMMC_CLKCR_DDR         |
              STM32_SDMMC_CLKCR_BUS_SPEED   |
              STM32_SDMMC_CLKCR_SELCLKRX_MASK);

  /* Replace with user provided settings */

  clkcr  &=  (STM32_SDMMC_CLKCR_CLKDIV_MASK | STM32_SDMMC_CLKCR_PWRSAV      |
              STM32_SDMMC_CLKCR_WIDBUS_MASK | STM32_SDMMC_CLKCR_NEGEDGE     |
              STM32_SDMMC_CLKCR_HWFC_EN     | STM32_SDMMC_CLKCR_DDR         |
              STM32_SDMMC_CLKCR_BUS_SPEED   |
              STM32_SDMMC_CLKCR_SELCLKRX_MASK);

  regval |=  clkcr | STM32_SDMMC_CLKCR_HWFC_EN;

  sdmmc_putreg32(priv, regval, STM32_SDMMC_CLKCR_OFFSET);

  mcinfo("CLKCR: %08" PRIx32 " PWR: %08" PRIx32 "\n",
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
#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  int pinset;
#endif

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags = enter_critical_section();

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  if ((waitevents & SDIOWAIT_WRCOMPLETE) != 0)
    {
      pinset = priv->d0_gpio & (GPIO_PORT_MASK | GPIO_PIN_MASK | \
                                GPIO_PUPD_MASK);
      pinset |= (GPIO_INPUT | GPIO_EXTI);

      /* Arm the SDMMC_D0 Ready and install Isr */

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

#if defined(HAVE_SDMMC_SDIO_MODE)
  if (priv->sdiomode == true)
    {
      sdmmc_putreg32(priv,
                     priv->xfrmask | priv->waitmask | priv->sdiointmask,
                     STM32_SDMMC_MASK_OFFSET);
    }
  else
#endif
    {
      sdmmc_putreg32(priv, priv->xfrmask | priv->waitmask,
                     STM32_SDMMC_MASK_OFFSET);
    }

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

#if defined(HAVE_SDMMC_SDIO_MODE)
  if (priv->sdiomode == true)
    {
      sdmmc_putreg32(priv,
                     priv->xfrmask | priv->waitmask | priv->sdiointmask,
                     STM32_SDMMC_MASK_OFFSET);
    }
  else
#endif
    {
      sdmmc_putreg32(priv, priv->xfrmask | priv->waitmask,
                     STM32_SDMMC_MASK_OFFSET);
    }

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
 * Name: stm32_sampleinit
 *
 * Description:
 *   Setup prior to collecting register samples
 *
 ****************************************************************************/

#if defined(CONFIG_STM32H7_SDMMC_XFRDEBUG)
static void stm32_sampleinit(void)
{
  memset(g_sampleregs, 0xff, DEBUG_NSAMPLES *
         sizeof(struct stm32_sampleregs_s));
}
#endif

/****************************************************************************
 * Name: stm32_sdiosample
 *
 * Description:
 *   Sample SDIO registers
 *
 ****************************************************************************/

#if defined(CONFIG_STM32H7_SDMMC_XFRDEBUG)
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
}
#endif

/****************************************************************************
 * Name: stm32_sample
 *
 * Description:
 *   Sample SDIO registers
 *
 ****************************************************************************/

#if defined(CONFIG_STM32H7_SDMMC_XFRDEBUG)
static void stm32_sample(struct stm32_dev_s *priv, int index)
{
  struct stm32_sampleregs_s *regs = &g_sampleregs[index];
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

#if defined(CONFIG_STM32H7_SDMMC_XFRDEBUG)
static void stm32_sdiodump(struct stm32_sdioregs_s *regs, const char *msg)
{
  mcinfo("SDIO Registers: %s\n", msg);
  mcinfo("  POWER[%08x]: %08" PRIx8 "\n", STM32_SDMMC_POWER_OFFSET,
         regs->power);
  mcinfo("  CLKCR[%08x]: %08" PRIx16 "\n", STM32_SDMMC_CLKCR_OFFSET,
         regs->clkcr);
  mcinfo("  DCTRL[%08x]: %08" PRIx16 "\n", STM32_SDMMC_DCTRL_OFFSET,
         regs->dctrl);
  mcinfo(" DTIMER[%08x]: %08" PRIx32 "\n", STM32_SDMMC_DTIMER_OFFSET,
         regs->dtimer);
  mcinfo("   DLEN[%08x]: %08" PRIx32 "\n", STM32_SDMMC_DLEN_OFFSET,
         regs->dlen);
  mcinfo(" DCOUNT[%08x]: %08" PRIx32 "\n", STM32_SDMMC_DCOUNT_OFFSET,
         regs->dcount);
  mcinfo("    STA[%08x]: %08" PRIx32 "\n", STM32_SDMMC_STA_OFFSET,
         regs->sta);
  mcinfo("   MASK[%08x]: %08" PRIx32 "\n", STM32_SDMMC_MASK_OFFSET,
         regs->mask);
}
#endif

/****************************************************************************
 * Name: stm32_dumpsample
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#if defined(CONFIG_STM32H7_SDMMC_XFRDEBUG)
static void stm32_dumpsample(struct stm32_dev_s *priv,
                             struct stm32_sampleregs_s *regs,
                             const char *msg)
{
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

#if defined(CONFIG_STM32H7_SDMMC_XFRDEBUG)
static void stm32_dumpsamples(struct stm32_dev_s *priv)
{
  stm32_dumpsample(priv, &g_sampleregs[SAMPLENDX_BEFORE_SETUP],
                   "Before setup");
  stm32_dumpsample(priv, &g_sampleregs[SAMPLENDX_AFTER_SETUP],
                   "After setup");
  stm32_dumpsample(priv, &g_sampleregs[SAMPLENDX_END_TRANSFER],
                   "End of transfer");
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
                             uint32_t dlen, bool receive)
{
  uint32_t clkdiv;
  uint32_t regval;
  uint32_t dctrl;
  uint32_t sdio_clk = STM32_SDMMC_CLK;

  DEBUGASSERT((sdmmc_getreg32(priv, STM32_SDMMC_IDMACTRLR_OFFSET) &
               STM32_SDMMC_IDMACTRLR_IDMAEN) == 0);
  DEBUGASSERT((sdmmc_getreg32(priv, STM32_SDMMC_STA_OFFSET) &
               STM32_SDMMC_STA_DPSMACT) == 0);

  /* Configure DCTRL DTDIR, DTMODE, and DBLOCKSIZE fields.
   * Note: the DTEN is not used, DPSM, and CPSM are used.
   */

  dctrl  =  sdmmc_getreg32(priv, STM32_SDMMC_DCTRL_OFFSET);
  dctrl  &= ~(STM32_SDMMC_DCTRL_DTDIR | STM32_SDMMC_DCTRL_DTMODE_MASK |
              STM32_SDMMC_DCTRL_DBLOCKSIZE_MASK | STM32_SDMMC_DCTRL_DTEN |
              STM32_SDMMC_DCTRL_BOOTACKEN);

  dctrl  &= (STM32_SDMMC_DCTRL_DTDIR | STM32_SDMMC_DCTRL_DTMODE_MASK |
             STM32_SDMMC_DCTRL_DBLOCKSIZE_MASK);

  /* Configure the data direction */

  if (receive)
    {
      dctrl |= STM32_SDMMC_DCTRL_DTDIR;
    }

  /* Set SDIO_MODE */

#if defined(HAVE_SDMMC_SDIO_MODE)
  if (priv->sdiomode == true)
    {
      dctrl |= STM32_SDMMC_DCTRL_SDIOEN;
    }
#endif

  dctrl |= STM32_SDMMC_DCTRL_DTMODE_BLOCK;

  /* if dlen > priv->blocksize we assume that this is a multi-block transfer
   * and that the len is multiple of priv->blocksize.
   */

  if (dlen > priv->blocksize)
    {
      DEBUGASSERT((dlen % priv->blocksize) == 0);

#if defined(CONFIG_STM32H7_SDMMC_IDMA)
      /* If this is an unaligned receive, then receive one block at a
       * time to the internal buffer
       */

      if (priv->unaligned_rx)
        {
          DEBUGASSERT(priv->blocksize <= sizeof(priv->sdmmc_rxbuffer));
          dlen = priv->blocksize;
        }
#endif
    }

  dctrl |= stm32_log2(priv->blocksize) << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT;

  /* Enable data path */

  /* Set DTIMER
   *
   * Enable data path using a timeout scaled to the SD_CLOCK (the card
   * clock).
   */

  regval = sdmmc_getreg32(priv, STM32_SDMMC_CLKCR_OFFSET);
  clkdiv = (regval & STM32_SDMMC_CLKCR_CLKDIV_MASK) >>
            STM32_SDMMC_CLKCR_CLKDIV_SHIFT;

  /* CLKDIV_ of 0x000: is Bypass */

  if (clkdiv != 0)
    {
      sdio_clk = sdio_clk / (2 * clkdiv);
    }

  /*  Convert Timeout in Ms to SD_CLK counts */

  timeout  = timeout * (sdio_clk / 1000);

  sdmmc_putreg32(priv, timeout, STM32_SDMMC_DTIMER_OFFSET);

  /* Set DLEN */

  sdmmc_putreg32(priv, dlen, STM32_SDMMC_DLEN_OFFSET);

  /* Set DCTRL */

  sdmmc_putreg32(priv, dctrl, STM32_SDMMC_DCTRL_OFFSET);
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

  /* Disable the IDMA */

  sdmmc_putreg32(priv, 0, STM32_SDMMC_IDMACTRLR_OFFSET);

  /* Disable the data path */

  /* Reset DTIMER */

  sdmmc_putreg32(priv, UINT32_MAX, STM32_SDMMC_DTIMER_OFFSET);

  /* Reset DLEN */

  sdmmc_putreg32(priv,  0, STM32_SDMMC_DLEN_OFFSET);

  /* Reset DCTRL DTEN, DTDIR, DTMODE, and DBLOCKSIZE fields */

  regval  = sdmmc_getreg32(priv, STM32_SDMMC_DCTRL_OFFSET);
  regval &= ~(STM32_SDMMC_DCTRL_DTEN  | STM32_SDMMC_DCTRL_DBLOCKSIZE_MASK |
              STM32_SDMMC_DCTRL_DTDIR | STM32_SDMMC_DCTRL_DTMODE_MASK);
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

#if !defined(CONFIG_STM32H7_SDMMC_IDMA)
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
#endif

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

#if !defined(CONFIG_STM32H7_SDMMC_IDMA)
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
          STM32_SDMMC_STA_RXFIFOE) == 0)
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
#endif

/****************************************************************************
 * Name: stm32_recvdma
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

#if defined (CONFIG_STM32H7_SDMMC_IDMA)
static void stm32_recvdma(struct stm32_dev_s *priv)
{
  uint32_t dctrl;

  if (priv->unaligned_rx)
    {
      /* If we are receiving multiple blocks to an unaligned buffers,
       * we receive them one-by-one
       */

      /* Copy the received data to client buffer */

      memcpy(priv->buffer, priv->sdmmc_rxbuffer, priv->blocksize);

      /* Invalidate the cache before receiving next block */

      up_invalidate_dcache((uintptr_t)priv->sdmmc_rxbuffer,
                           (uintptr_t)priv->sdmmc_rxbuffer +
                           priv->blocksize);

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

      stm32_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
    }
  else
    {
      /* We end up here only in unaligned rx-buffers case, and are receiving
       * the data one block at a time
       */

      /* Update where to receive the following block */

      priv->buffer = (uint32_t *)((uintptr_t)priv->buffer + priv->blocksize);

      /* Clear pending interrupt status */

      sdmmc_putreg32(priv, STM32_SDMMC_XFRDONE_ICR, STM32_SDMMC_ICR_OFFSET);

      /* Re-enable datapath and wait for next block */

      dctrl = sdmmc_getreg32(priv, STM32_SDMMC_DCTRL_OFFSET);
      dctrl |= STM32_SDMMC_DCTRL_DTEN;
      sdmmc_putreg32(priv, dctrl, STM32_SDMMC_DCTRL_OFFSET);
    }
}
#endif

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

  mcinfo("sta: %08" PRIx32 " enabled irq: %08" PRIx32 "\n",
         sdmmc_getreg32(priv, STM32_SDMMC_STA_OFFSET),
         sdmmc_getreg32(priv, STM32_SDMMC_MASK_OFFSET));

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. wake up any waiting threads */

#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
      stm32_endwait(priv, SDIOWAIT_TIMEOUT |
                    (priv->waitevents & SDIOWAIT_WRCOMPLETE));
#else
      stm32_endwait(priv, SDIOWAIT_TIMEOUT);
#endif
      mcerr("Timeout: remaining: %zu\n", priv->remaining);
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

  nxsem_post(&priv->waitsem);
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

  /* If there were errors, send a stop command to DPSM */

  if ((wkupevent & (~SDIOWAIT_TRANSFERDONE)) != 0)
    {
      sdmmc_putreg32(priv, STM32_SDMMC_CMD_CMDSTOP,
                     STM32_SDMMC_CMD_OFFSET);
    }
  else
    {
      sdmmc_modifyreg32(priv, STM32_SDMMC_CMD_OFFSET,
                        STM32_SDMMC_CMD_CPSMEN, 0);
    }

  /* Clearing pending interrupt status on all transfer related interrupts */

  sdmmc_putreg32(priv, STM32_SDMMC_XFRDONE_ICR, STM32_SDMMC_ICR_OFFSET);

  /* DMA debug instrumentation */

  stm32_sample(priv, SAMPLENDX_END_TRANSFER);

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
 * Name: stm32_sdmmc_fifo_monitor
 *
 * Description:
 *   SDMMC FIFO monitor is used to detect the FIFO is stalled with less
 *   than FIFO_SIZE_IN_BYTES /2 that will trigger an interrupt.
 *
 * Input Parameters:
 *   arg - is the stm32_dev_s
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if !defined(CONFIG_STM32H7_SDMMC_IDMA)
static void stm32_sdmmc_fifo_monitor(void *arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)arg;

  if (priv->receivecnt && priv->remaining &&
      priv->remaining < FIFO_SIZE_IN_BYTES / 2)
    {
      /* Manually Receive data from the lame RX FIFO */

      stm32_recvfifo(priv);
    }

  /* Check for the lame FIFO condition */

  if (sdmmc_getreg32(priv, STM32_SDMMC_DCOUNT_OFFSET) != 0 &&
      sdmmc_getreg32(priv, STM32_SDMMC_STA_OFFSET) ==
      STM32_SDMMC_STA_DPSMACT)
    {
      work_queue(HPWORK, &priv->cbfifo,
                 stm32_sdmmc_fifo_monitor, arg, 1);
    }
}
#endif

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

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
static int stm32_sdmmc_rdyinterrupt(int irq, void *context, void *arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)arg;

  /* Avoid noise, check the state */

  if (stm32_gpioread(priv->d0_gpio))
    {
      stm32_endwait(priv, SDIOWAIT_WRCOMPLETE);
    }

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
#if defined(HAVE_SDMMC_SDIO_MODE)
  uint32_t mask;
#endif

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
#ifndef CONFIG_STM32H7_SDMMC_IDMA
          /* Is the RX FIFO half full or more?  Is so then we must be
           * processing a receive transaction.
           */

          if ((pending & STM32_SDMMC_STA_RXFIFOHF) != 0)
            {
              /* Receive data from the RX FIFO */

              stm32_recvfifo(priv);
              work_queue(HPWORK, &priv->cbfifo,
                         stm32_sdmmc_fifo_monitor, arg, 1);
            }

          /* Otherwise, Is the transmit FIFO half empty or less?  If so
           * we must be processing a send transaction.  NOTE:  We can't
           * be processing both!
           */

          else if ((pending & STM32_SDMMC_STA_TXFIFOHE) != 0)
            {
              /* Send data via the TX FIFO */

              stm32_sendfifo(priv);

              /* If we are done shutdown the TXFIFOHE interrupt */

              if (priv->remaining == 0)
                {
                  sdmmc_putreg32(priv, ~STM32_SDMMC_MASK_TXFIFOHEIE &
                                 sdmmc_getreg32(priv,
                                                STM32_SDMMC_MASK_OFFSET),
                                 STM32_SDMMC_MASK_OFFSET);
                }
            }
#endif

          /* Handle data end events */

          if ((pending & STM32_SDMMC_STA_DATAEND) != 0)
            {
              /* Handle any data remaining the RX FIFO.  If the RX FIFO is
               * less than half full at the end of the transfer, then no
               * half-full interrupt will be received.
               */

#ifndef CONFIG_STM32H7_SDMMC_IDMA

              /* If the transfer would not trigger fifo half full
               * we used IDMA to manage the lame fifo
               */

              if (priv->remaining && priv->remaining <
                  FIFO_SIZE_IN_BYTES / 2)
                {
                  memcpy(priv->buffer, priv->rxfifo, priv->remaining);
                }
#else
              if (priv->receivecnt)
                {
                  /* Invalidate dcache, and copy the received data into
                   * client buffers in unaligned case
                   */

                  stm32_recvdma(priv);
                }
              else
#endif
                {
                  /* Then terminate the transfer.
                   * Sets STM32_SDMMC_ICR_DATAENDC
                   */

                  stm32_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
                }
            }

          /* Handle data block send/receive CRC failure */

          else if ((pending & STM32_SDMMC_STA_DCRCFAIL) != 0)
            {
              /* Terminate the transfer with an error.
               *  Sets STM32_SDMMC_ICR_DCRCFAILC
               */

              mcerr("ERROR: Data block CRC failure, remaining: %u\n",
                    priv->remaining);

              stm32_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle data timeout error */

          else if ((pending & STM32_SDMMC_STA_DTIMEOUT) != 0)
            {
              /* Terminate the transfer with an error
               *  Sets STM32_SDMMC_ICR_DTIMEOUTC
               */

              mcerr("ERROR: Data timeout, remaining: %u\n",
                    priv->remaining);

              stm32_endtransfer(priv, SDIOWAIT_TRANSFERDONE |
                                      SDIOWAIT_TIMEOUT);
            }

          /* Handle RX FIFO overrun error */

          else if ((pending & STM32_SDMMC_STA_RXOVERR) != 0)
            {
              /* Terminate the transfer with an error
               * Sets STM32_SDMMC_ICR_RXOVERRC
               */

              mcerr("ERROR: RX FIFO overrun, remaining: %u\n",
                    priv->remaining);

              stm32_endtransfer(priv, SDIOWAIT_TRANSFERDONE |
                                      SDIOWAIT_ERROR);
            }

          /* Handle TX FIFO underrun error */

          else if ((pending & STM32_SDMMC_STA_TXUNDERR) != 0)
            {
              /* Terminate the transfer with an error
               * Sets STM32_SDMMC_ICR_TXUNDERRC
               */

              mcerr("ERROR: TX FIFO underrun, remaining: %u\n",
                    priv->remaining);

              stm32_endtransfer(priv, SDIOWAIT_TRANSFERDONE |
                                      SDIOWAIT_ERROR);
            }
        }

      /* Handle wait events *************************************************/

      pending  = enabled & priv->waitmask;
      if (pending != 0)
        {
          /* Is this a response completion event? */

          if ((pending & STM32_SDMMC_RESPDONE_STA) != 0)
            {
              sdmmc_putreg32(priv, STM32_SDMMC_RESPDONE_STA,
                             STM32_SDMMC_ICR_OFFSET);

              /* Yes.. Is there a thread waiting for response done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                   stm32_endwait(priv, SDIOWAIT_RESPONSEDONE);
                }
            }

          /* Is this a command completion event? */

          if ((pending & STM32_SDMMC_CMDDONE_STA) != 0)
            {
              sdmmc_putreg32(priv, STM32_SDMMC_CMDDONE_STA,
                             STM32_SDMMC_ICR_OFFSET);

              /* Yes.. Is there a thread waiting for command done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  stm32_endwait(priv, SDIOWAIT_CMDDONE);
                }
            }
        }

#if defined(HAVE_SDMMC_SDIO_MODE)
      if (priv->sdiomode == true)
        {
          pending = enabled & priv->sdiointmask;
          if (pending != 0)
            {
              mask = sdmmc_getreg32(priv, STM32_SDMMC_MASK_OFFSET);

              /* Clear the mask so we don't get call'd again */

              sdmmc_putreg32(priv, mask & ~STM32_SDMMC_MASK_SDIOITIE,
                             STM32_SDMMC_MASK_OFFSET);

              /* Now clear the interruption */

              sdmmc_putreg32(priv, STM32_SDMMC_ICR_SDIOITC,
                             STM32_SDMMC_ICR_OFFSET);

              /* Call the ISR that has been registered */

              if (priv->do_sdio_card)
                {
                  priv->do_sdio_card(priv->do_sdio_arg);
                }
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

#if defined(CONFIG_SDIO_MUXBUS)
static int stm32_lock(struct sdio_dev_s *dev, bool lock)
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

static void stm32_reset(struct sdio_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  irqstate_t flags;
  uint32_t regval;
  uint32_t regaddress;
  uint32_t restval;

  /* Disable clocking */

  flags = enter_critical_section();

#if defined(CONFIG_STM32H7_SDMMC1)
  if (priv->base == STM32_SDMMC1_BASE)
    {
      regaddress = STM32_RCC_AHB3RSTR;
      restval    = RCC_AHB3RSTR_SDMMC1RST;
    }
#endif

#if defined CONFIG_STM32H7_SDMMC2
  if (priv->base == STM32_SDMMC2_BASE)
    {
      regaddress = STM32_RCC_AHB2RSTR;
      restval    = RCC_AHB2RSTR_SDMMC2RST;
    }
#endif

  regval = getreg32(regaddress);
  putreg32(regval | restval, regaddress);
  nxsig_usleep(2);
  putreg32(regval, regaddress);

  stm32_setpwrctrl(priv, STM32_SDMMC_POWER_PWRCTRL_CYCLE);
  nxsig_usleep(1000);

  /* Put SDIO registers in their default, reset state */

  stm32_default(priv);

  /* Reset data */

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitmask   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */

  wd_cancel(&priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->buffer     = 0;      /* Address of current R/W buffer */
  priv->remaining  = 0;      /* Number of bytes remaining in the transfer */
  priv->xfrmask    = 0;      /* Interrupt enables for data transfer */

#ifdef HAVE_SDMMC_SDIO_MODE
  priv->sdiointmask = 0;     /* SDIO card in-band interrupt mask */
#endif

  priv->widebus    = false;

  /* Configure the SDIO peripheral */

  stm32_setpwrctrl(priv, STM32_SDMMC_POWER_PWRCTRL_OFF);
  nxsig_usleep(1000);
  stm32_setpwrctrl(priv, STM32_SDMMC_POWER_PWRCTRL_ON);

  stm32_setclkcr(priv, STM32_CLCKCR_INIT);

  leave_critical_section(flags);

  mcinfo("CLCKR: %08" PRIx32 " POWER: %08" PRIx32 "\n",
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

static sdio_capset_t stm32_capabilities(struct sdio_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  sdio_capset_t caps = 0;

  if (priv->onebit)
    {
      caps |= SDIO_CAPS_1BIT_ONLY;
    }

  caps |= SDIO_CAPS_DMABEFOREWRITE;

#if defined(CONFIG_STM32H7_SDMMC_IDMA)
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

static sdio_statset_t stm32_status(struct sdio_dev_s *dev)
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

static void stm32_widebus(struct sdio_dev_s *dev, bool wide)
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

static void stm32_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate)
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
      clckr = STM32_CLCKCR_INIT;
      break;

    /* Enable in MMC normal operation clocking */

    case CLOCK_MMC_TRANSFER:
      clckr = STM32_SDMMC_CLKCR_MMCXFR;
      break;

    /* SD normal operation clocking (wide 4-bit mode) */

    case CLOCK_SD_TRANSFER_4BIT:
      if (!priv->onebit)
        {
          clckr = STM32_SDMMC_CLCKR_SDWIDEXFR;
          break;
        }

    /* SD normal operation clocking (narrow 1-bit mode) */

    case CLOCK_SD_TRANSFER_1BIT:
      clckr = STM32_SDMMC_CLCKR_SDXFR;
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

static int stm32_attach(struct sdio_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  int ret;

  /* Attach the SDIO interrupt handler */

  ret = irq_attach(priv->nirq, stm32_sdmmc_interrupt, priv);
  if (ret == OK)
    {
      /* Disable all interrupts at the SDIO controller and clear
       * interrupt flags
       */

      sdmmc_putreg32(priv, STM32_SDMMC_MASK_RESET, STM32_SDMMC_MASK_OFFSET);
      sdmmc_putreg32(priv, STM32_SDMMC_ICR_ALLFLAGS, STM32_SDMMC_ICR_OFFSET);

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

static int stm32_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                         uint32_t arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t regval;
  uint32_t cmdidx;

  /* Clear CMDINDEX, CMDTRANS, WAITRESP, WAITINT, WAITPEND, and CPSMEN bits */

  regval = sdmmc_getreg32(priv, STM32_SDMMC_CMD_OFFSET);

  /* Make sure the register is clear */

  if (regval & STM32_SDMMC_CMD_CPSMEN)
    {
      sdmmc_putreg32(priv, 0, STM32_SDMMC_CMD_OFFSET);
    }

  /* Set the SDIO Argument value */

  sdmmc_putreg32(priv, arg, STM32_SDMMC_ARG_OFFSET);

  /* Clear CMDINDEX, WAITRESP, WAITINT, WAITPEND, and CPSMEN bits */

  regval &= ~(STM32_SDMMC_CMD_CMDINDEX_MASK | STM32_SDMMC_CMD_WAITRESP_MASK |
              STM32_SDMMC_CMD_CMDTRANS      |  STM32_SDMMC_CMD_CMDSTOP      |
              STM32_SDMMC_CMD_WAITRESP_MASK | STM32_SDMMC_CMD_WAITINT       |
              STM32_SDMMC_CMD_WAITPEND      | STM32_SDMMC_CMD_CPSMEN        |
              STM32_SDMMC_CMD_DTHOLD        | STM32_SDMMC_CMD_BOOTMODE      |
              STM32_SDMMC_CMD_BOOTEN        | STM32_SDMMC_CMD_SUSPEND);

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

  switch (cmd & MMCSD_DATAXFR_MASK)
    {
    case MMCSD_RDDATAXFR: /* Read block transfer */
    case MMCSD_WRDATAXFR: /* Write block transfer */
    case MMCSD_RDSTREAM:  /* MMC Read stream */
    case MMCSD_WRSTREAM:  /* MMC Write stream */
        regval |= STM32_SDMMC_CMD_CMDTRANS;
        break;

    case MMCSD_NODATAXFR:
    default:
      if ((cmd & MMCSD_STOPXFR) != 0)
        {
          regval |= STM32_SDMMC_CMD_CMDSTOP;
        }
      break;
    }

  /* Clear interrupts */

  sdmmc_putreg32(priv, STM32_SDMMC_CMDDONE_ICR | STM32_SDMMC_RESPDONE_ICR,
                 STM32_SDMMC_ICR_OFFSET);

  mcinfo("cmd: %08" PRIx32 " arg: %08" PRIx32 " regval: %08" PRIx32
         " enabled irq: %08" PRIx32 "\n",
         cmd, arg, regval, sdmmc_getreg32(priv, STM32_SDMMC_MASK_OFFSET));

  /* Write the SDIO CMD */

  sdmmc_putreg32(priv, regval, STM32_SDMMC_CMD_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: stm32_blocksetup
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

static void stm32_blocksetup(struct sdio_dev_s *dev,
                             unsigned int blocksize, unsigned int nblocks)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  priv->blocksize = blocksize;
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

#ifndef CONFIG_STM32H7_SDMMC_IDMA
static int stm32_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                           size_t nbytes)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  stm32_datadisable(priv);

  /* Initialize register sampling */

  stm32_sampleinit();
  stm32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt
   * handler
   */

  priv->buffer     = (uint32_t *)buffer;
  priv->remaining  = nbytes;
  priv->receivecnt = nbytes;

  /* Then set up the SDIO data path */

  stm32_dataconfig(priv, SDMMC_DTIMER_DATATIMEOUT_MS, nbytes, true);

  /* Workaround the FIFO data available issue */

  if (nbytes < FIFO_SIZE_IN_BYTES / 2)
    {
      /* Use the driver's buffer */

      sdmmc_putreg32(priv, (uintptr_t)priv->rxfifo,
                     STM32_SDMMC_IDMABASE0R_OFFSET);

      sdmmc_putreg32(priv, STM32_SDMMC_IDMACTRLR_IDMAEN,
                     STM32_SDMMC_IDMACTRLR_OFFSET);

      /* Ensure CPU will read DMA-ed data */

      up_invalidate_dcache((uintptr_t)priv->rxfifo,
                           (uintptr_t)priv->rxfifo + nbytes);
    }

  /* And enable interrupts */

  stm32_configxfrints(priv, STM32_SDMMC_RECV_MASK);
  stm32_sample(priv, SAMPLENDX_AFTER_SETUP);

  return OK;
}
#endif

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

#ifndef CONFIG_STM32H7_SDMMC_IDMA
static int stm32_sendsetup(struct sdio_dev_s *dev, const
                           uint8_t *buffer, size_t nbytes)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  stm32_datadisable(priv);

  /* Initialize register sampling */

  stm32_sampleinit();
  stm32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer     = (uint32_t *)buffer;
  priv->remaining  = nbytes;
  priv->receivecnt = 0;

  /* Then set up the SDIO data path */

  stm32_dataconfig(priv, SDMMC_DTIMER_DATATIMEOUT_MS, nbytes, false);

  /* Enable TX interrupts */

  stm32_configxfrints(priv, STM32_SDMMC_SEND_MASK);
  stm32_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}
#endif

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

static int stm32_cancel(struct sdio_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  /* Disable all transfer- and event- related interrupts */

  stm32_configxfrints(priv, 0);
  stm32_configwaitints(priv, 0, 0, 0);

  /* Send a stop command to DPSM */

  sdmmc_putreg32(priv, STM32_SDMMC_CMD_CMDSTOP,
                 STM32_SDMMC_CMD_OFFSET);

  /* If this was a IDMA transfer, make sure that IDMA is stopped */

  sdmmc_putreg32(priv, 0, STM32_SDMMC_IDMACTRLR_OFFSET);

  /* Clearing pending interrupt status on all transfer- and event- related
   * interrupts
   */

  sdmmc_putreg32(priv, STM32_SDMMC_WAITALL_ICR, STM32_SDMMC_ICR_OFFSET);

  /* Cancel any watchdog timeout */

  wd_cancel(&priv->waitwdog);

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

static int stm32_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
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
      case MMCSD_R4_RESPONSE:
      case MMCSD_R5_RESPONSE:
      case MMCSD_R6_RESPONSE:
        events  = STM32_SDMMC_RESPDONE_STA;
        timeout = SDMMC_LONGTIMEOUT;
        break;

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
          mcerr("ERROR: Timeout cmd: %08" PRIx32 " events: %08" PRIx32
                " STA: %08" PRIx32 "\n",
                cmd, events, sdmmc_getreg32(priv, STM32_SDMMC_STA_OFFSET));

          return -ETIMEDOUT;
        }
    }

  sdmmc_putreg32(priv, events, STM32_SDMMC_ICR_OFFSET);
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

static int stm32_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                              uint32_t *rshort)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
#if defined(CONFIG_DEBUG_FEATURES)
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

#if defined(CONFIG_DEBUG_FEATURES)
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

      regval = sdmmc_getreg32(priv, STM32_SDMMC_STA_OFFSET);
      if ((regval & STM32_SDMMC_STA_CTIMEOUT) != 0)
        {
          mcerr("ERROR: Command timeout: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & STM32_SDMMC_STA_CCRCFAIL) != 0)
        {
          mcerr("ERROR: CRC failure: %08" PRIx32 "\n", regval);
          ret = -EIO;
        }
#if defined(CONFIG_DEBUG_FEATURES)
    else
        {
          /* Check response received is of desired command */

          respcmd = sdmmc_getreg32(priv, STM32_SDMMC_RESPCMD_OFFSET);
          if ((uint8_t)(respcmd & STM32_SDMMC_RESPCMD_MASK) !=
              (cmd & MMCSD_CMDIDX_MASK))
            {
              mcerr("ERROR: RESCMD=%02" PRIx32 " CMD=%08" PRIx32
                    "\n", respcmd, cmd);
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

static int stm32_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
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

#if defined(CONFIG_DEBUG_FEATURES)
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

      regval = sdmmc_getreg32(priv, STM32_SDMMC_STA_OFFSET);
      if (regval & STM32_SDMMC_STA_CTIMEOUT)
        {
          mcerr("ERROR: Timeout STA: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & STM32_SDMMC_STA_CCRCFAIL)
        {
          mcerr("ERROR: CRC fail STA: %08" PRIx32 "\n", regval);
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

static int stm32_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
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

#if defined(CONFIG_DEBUG_FEATURES)
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
      /* Check if a timeout occurred (Apparently a CRC error can terminate
       * a good response)
       */

      regval = sdmmc_getreg32(priv, STM32_SDMMC_STA_OFFSET);
      if (regval & STM32_SDMMC_STA_CTIMEOUT)
        {
          mcerr("ERROR: Timeout STA: %08" PRIx32 "\n", regval);
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

static void stm32_waitenable(struct sdio_dev_s *dev,
                             sdio_eventset_t eventset, uint32_t timeout)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t waitmask = 0;

  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  stm32_configwaitints(priv, 0, 0, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  if ((eventset & SDIOWAIT_WRCOMPLETE) != 0)
    {
      /* Read pin to see if ready (true) skip timeout */

      if (stm32_gpioread(priv->d0_gpio))
        {
          eventset &= ~(SDIOWAIT_TIMEOUT | SDIOWAIT_WRCOMPLETE);
        }
    }
  else
#endif
    {
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
      if (ret < OK)
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

static sdio_eventset_t stm32_eventwait(struct sdio_dev_s *dev)
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
       * fired.  If Pin is ready and if ISR did NOT fire end the wait here.
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

      ret = nxsem_wait_uninterruptible(priv);
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

static void stm32_callbackenable(struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  mcinfo("eventset: %02" PRIx8 "\n", eventset);
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

static int stm32_registercallback(struct sdio_dev_s *dev,
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

#if defined(CONFIG_STM32H7_SDMMC_IDMA) && defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
static int stm32_dmapreflight(struct sdio_dev_s *dev,
                              const uint8_t *buffer, size_t buflen)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);

  /* IDMA must be possible to the buffer */

#if defined(CONFIG_STM32H7_SDMMC1)
  if (priv->base == STM32_SDMMC1_BASE)
    {
      /* For SDMMC1, IDMA cannot access SRAM123 or SRAM4. */

      if (((uintptr_t)buffer >= SRAM123_START &&
          (uintptr_t)buffer + buflen <= SRAM123_END) ||
          ((uintptr_t)buffer >= SRAM4_START &&
          (uintptr_t)buffer + buflen <= SRAM4_END))
        {
          mcerr("invalid IDMA address "
                "buffer:0x%08" PRIxPTR " end:0x%08" PRIxPTR "\n",
                (uintptr_t)buffer, (uintptr_t)(buffer + buflen - 1));
          return -EFAULT;
        }
    }
#endif

#if defined(CONFIG_ARMV7M_DCACHE) && !defined(CONFIG_ARMV7M_DCACHE_WRITETHROUGH)
  /* buffer alignment is required for DMA transfers with dcache in buffered
   * mode (not write-through) because a) arch_invalidate_dcache could lose
   * buffered writes and b) arch_flush_dcache could corrupt adjacent memory
   * if the maddr and the mend+1, the next next address are not on
   * ARMV7M_DCACHE_LINESIZE boundaries.
   */

  if (buffer != priv->rxfifo &&
      (((uintptr_t)buffer & (ARMV7M_DCACHE_LINESIZE - 1)) != 0 ||
      ((uintptr_t)(buffer + buflen) & (ARMV7M_DCACHE_LINESIZE - 1)) != 0))
    {
      mcerr("dcache unaligned "
            "buffer:%p end:%p\n",
            buffer, buffer + buflen - 1);
      return -EFAULT;
    }
#endif

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

#if defined(CONFIG_STM32H7_SDMMC_IDMA)
static int stm32_dmarecvsetup(struct sdio_dev_s *dev,
                              uint8_t *buffer, size_t buflen)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
#if defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
  DEBUGASSERT(stm32_dmapreflight(dev, buffer, buflen) == 0);
#endif

#if defined(CONFIG_ARMV7M_DCACHE)
  if (((uintptr_t)buffer & (ARMV7M_DCACHE_LINESIZE - 1)) != 0 ||
       (buflen & (ARMV7M_DCACHE_LINESIZE - 1)) != 0)
    {
      /* The read buffer is not cache-line aligned. Read to an internal
       * buffer instead.
       */

      up_invalidate_dcache((uintptr_t)priv->sdmmc_rxbuffer,
                           (uintptr_t)priv->sdmmc_rxbuffer +
                           priv->blocksize);

      priv->unaligned_rx = true;
    }
  else
    {
      up_invalidate_dcache((uintptr_t)buffer,
                           (uintptr_t)buffer + buflen);

      priv->unaligned_rx = false;
    }
#else

  /* IDMA access must be 32 bit aligned */

  priv->unaligned_rx = ((uintptr_t)buffer & 0x3) != 0;

#endif

  /* Reset the DPSM configuration */

  stm32_datadisable(priv);

  /* Initialize register sampling */

  stm32_sampleinit();
  stm32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer      = (uint32_t *)buffer;
  priv->remaining  = buflen;
  priv->receivecnt = buflen;

  /* Then set up the SDIO data path */

  stm32_dataconfig(priv, SDMMC_DTIMER_DATATIMEOUT_MS, buflen, true);

  /* Configure the RX DMA */

  if (priv->unaligned_rx)
    {
      sdmmc_putreg32(priv, (uintptr_t)priv->sdmmc_rxbuffer,
                     STM32_SDMMC_IDMABASE0R_OFFSET);
    }
  else
    {
      sdmmc_putreg32(priv, (uintptr_t)priv->buffer,
                     STM32_SDMMC_IDMABASE0R_OFFSET);
    }

  sdmmc_putreg32(priv, STM32_SDMMC_IDMACTRLR_IDMAEN,
                 STM32_SDMMC_IDMACTRLR_OFFSET);

  /* And enable interrupts */

  stm32_configxfrints(priv, STM32_SDMMC_DMARECV_MASK);

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

#if defined(CONFIG_STM32H7_SDMMC_IDMA)
static int stm32_dmasendsetup(struct sdio_dev_s *dev,
                              const uint8_t *buffer, size_t buflen)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
#if defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
  DEBUGASSERT(stm32_dmapreflight(dev, buffer, buflen) == 0);
#endif

  priv->unaligned_rx = false;

  /* Reset the DPSM configuration */

  stm32_datadisable(priv);

  /* Initialize register sampling */

  stm32_sampleinit();
  stm32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Flush cache to physical memory when not in DTCM memory */

#if defined(CONFIG_ARMV7M_DCACHE) && \
      !defined(CONFIG_ARMV7M_DCACHE_WRITETHROUGH)
  if ((uintptr_t)buffer < DTCM_START ||
      (uintptr_t)buffer + buflen > DTCM_END)
    {
      up_clean_dcache((uintptr_t)buffer, (uintptr_t)buffer + buflen);
    }
#endif

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer     = (uint32_t *)buffer;
  priv->remaining  = buflen;
  priv->receivecnt = 0;

  /* Then set up the SDIO data path */

  stm32_dataconfig(priv, SDMMC_DTIMER_DATATIMEOUT_MS, buflen, false);

  /* Configure the TX DMA */

  sdmmc_putreg32(priv, (uintptr_t)priv->buffer,
                 STM32_SDMMC_IDMABASE0R_OFFSET);

  sdmmc_putreg32(priv, STM32_SDMMC_IDMACTRLR_IDMAEN,
                 STM32_SDMMC_IDMACTRLR_OFFSET);

  /* Enable TX interrupts */

  stm32_configxfrints(priv, STM32_SDMMC_DMASEND_MASK);
  stm32_sample(priv, SAMPLENDX_AFTER_SETUP);
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
 * Returned Values:
 *   A reference to an SDIO interface structure.  NULL is returned on
 *   failures.
 *
 ****************************************************************************/

struct sdio_dev_s *sdio_initialize(int slotno)
{
  struct stm32_dev_s *priv = NULL;

#if defined(CONFIG_STM32H7_SDMMC1)
  if (slotno == 0)
    {
      /* Select SDMMC 1 */

      priv = &g_sdmmcdev1;

#if defined(CONFIG_SDMMC1_WIDTH_D1_ONLY)
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
      stm32_configgpio(SDMMC1_SDIO_PULL(GPIO_SDMMC1_D0));
#  ifndef CONFIG_SDMMC1_WIDTH_D1_ONLY
      stm32_configgpio(SDMMC1_SDIO_PULL(GPIO_SDMMC1_D1));
      stm32_configgpio(SDMMC1_SDIO_PULL(GPIO_SDMMC1_D2));
      stm32_configgpio(SDMMC1_SDIO_PULL(GPIO_SDMMC1_D3));
#  endif
      stm32_configgpio(GPIO_SDMMC1_CK);
      stm32_configgpio(SDMMC1_SDIO_PULL(GPIO_SDMMC1_CMD));
#endif
    }
  else
#endif
#if defined(CONFIG_STM32H7_SDMMC2)
  if (slotno == SDMMC2_SLOT)
    {
      /* Select SDMMC 2 */

      priv = &g_sdmmcdev2;

#  if defined(CONFIG_SDMMC2_WIDTH_D1_ONLY)
      priv->onebit = true;
#  else
      priv->onebit = false;
#  endif

      /* Configure GPIOs for 4-bit, wide-bus operation (the chip is capable
       * of 8-bit wide bus operation but D4-D7 are not configured).
       *
       * If bus is multiplexed then there is a custom bus configuration
       * utility in the scope of the board support package.
       */

#  ifndef CONFIG_SDIO_MUXBUS
      stm32_configgpio(SDMMC2_SDIO_PULL(GPIO_SDMMC2_D0));
#    ifndef CONFIG_SDMMC2_WIDTH_D1_ONLY
      stm32_configgpio(SDMMC2_SDIO_PULL(GPIO_SDMMC2_D1));
      stm32_configgpio(SDMMC2_SDIO_PULL(GPIO_SDMMC2_D2));
      stm32_configgpio(SDMMC2_SDIO_PULL(GPIO_SDMMC2_D3));
#    endif
      stm32_configgpio(GPIO_SDMMC2_CK);
      stm32_configgpio(SDMMC2_SDIO_PULL(GPIO_SDMMC2_CMD));
#  endif
    }
  else
#endif
    {
      mcerr("ERROR: Unsupported SDMMC slot: %d\n", slotno);
      return NULL;
    }

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

void sdio_mediachange(struct sdio_dev_s *dev, bool cardinslot)
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

  mcinfo("cdstatus OLD: %02" PRIx8 " NEW: %02" PRIx8 "\n",
         cdstatus, priv->cdstatus);

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

void sdio_wrprotect(struct sdio_dev_s *dev, bool wrprotect)
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

  mcinfo("cdstatus: %02" PRIx8 "\n", priv->cdstatus);
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

#ifdef HAVE_SDMMC_SDIO_MODE
void sdio_set_sdio_card_isr(struct sdio_dev_s *dev,
                            int (*func)(void *), void *arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  priv->do_sdio_card = func;

  if (func != NULL)
    {
      priv->sdiointmask = STM32_SDMMC_MASK_SDIOITIE;
      priv->do_sdio_arg = arg;
    }
  else
    {
      priv->sdiointmask = 0;
    }

  sdmmc_putreg32(priv, priv->xfrmask | priv->waitmask | priv->sdiointmask,
                 STM32_SDMMC_MASK_OFFSET);
}
#endif

#endif /* CONFIG_STM32H7_SDMMC1 || CONFIG_STM32H7_SDMMC2 */
