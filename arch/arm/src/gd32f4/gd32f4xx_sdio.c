/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_sdio.c
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
#include <stdio.h>

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
#include "gd32f4xx.h"
#include "gd32f4xx_sdio.h"

#ifdef CONFIG_GD32F4_SDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Required system configuration options:
 *
 *   CONFIG_ARCH_DMA - Enable architecture-specific DMA subsystem
 *     initialization.  Required if CONFIG_GD32F4_SDIO_DMA is enabled.
 *   CONFIG_GD32F4_DMA1 - Enable GD32 DMA1 support.  Required if
 *     CONFIG_GD32F4_SDIO_DMA is enabled
 *   CONFIG_SCHED_WORKQUEUE -- Callback support requires work queue support.
 *
 * Driver-specific configuration options:
 *
 *   CONFIG_GD32F4_SDIO_DMA - Enable SDIO.  This is a marginally optional.
 *     For most usages, SDIO will cause data overruns if used without DMA.
 *     NOTE the above system DMA configuration options.
 *   CONFIG_GD32F4_SDIO_WIDTH_D1_ONLY - This may be selected to force the
 *     driver operate with only a single data line (the default is to use
 *     all 4 SD data lines).
 *   CONFIG_SDM_DMAPRIO - SDIO DMA priority.  This can be selected if
 *     CONFIG_GD32F4_SDIO_DMA is enabled.
 *   CONFIG_SDIO_XFRDEBUG - Enables some very low-level debug output
 *     This also requires CONFIG_DEBUG_FS and CONFIG_DEBUG_INFO
 */

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE"
#endif

#ifdef CONFIG_GD32F4_SDIO_DMA
#  ifndef CONFIG_GD32_SDIO_DMAPRIO
#    if  defined(CONFIG_GD32F4_GD32F4XX)
#      define CONFIG_GD32_SDIO_DMAPRIO  DMA_PRIORITY_ULTRA_HIGH
#    else
#      error "Unknown GD32 DMA"
#    endif
#  endif
#  if defined(CONFIG_GD32F4_GD32F4XX)
#    if (CONFIG_GD32_SDIO_DMAPRIO & ~DMA_CHXCTL_PRIO_MASK) != 0
#      error "Illegal value for CONFIG_GD32_SDIO_DMAPRIO"
#    endif
#  else
#    error "Unknown GD32 DMA"
#  endif
#else
#  undef CONFIG_GD32_SDIO_DMAPRIO
#endif

#ifndef CONFIG_DEBUG_MEMCARD_INFO
#  undef CONFIG_SDIO_XFRDEBUG
#endif

#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
# error "GD32 not support mmcsd sdiowait wrcomple by D0 detect"
#endif

/* Enable the SDIO pull-up resistors if needed */

#ifdef CONFIG_GD32F4_SDIO_PULLUP
#  define SDIO_PULLUP_ENABLE GPIO_PUPD_PULLUP
#else
#  define SDIO_PULLUP_ENABLE 0
#endif

/* Friendly Clkctl bit re-definitions ***************************************/

#define SDIO_CLKCTL_RISINGEDGE     (0)
#define SDIO_CLKCTL_FALLINGEDGE    SDIO_CLKCTL_NEGEDGE

/* Use the default of the rising edge but allow a configuration,
 * that does not have the errata, to override the edge the SDIO
 * command and data is changed on.
 */

#if !defined(SDIO_CLKCTL_EDGE)
#  define SDIO_CLKCTL_EDGE SDIO_CLKCTL_RISINGEDGE
#endif

/* Mode dependent settings.  These depend on clock divisor settings that must
 * be defined in the board-specific board.h header file: SDIO_INIT_CLKDIV,
 * SDIO_MMCXFR_CLKDIV, and SDIO_SDXFR_CLKDIV.
 */

#define GD32_CLCKCTL_INIT         (SDIO_INIT_CLKDIV | SDIO_CLKCTL_EDGE | \
                                  SDIO_CLKCTL_WIDBUS_D1)
#define SDIO_CLKCTL_MMCXFR        (SDIO_MMCXFR_CLKDIV | SDIO_CLKCTL_EDGE | \
                                  SDIO_CLKCTL_WIDBUS_D1)
#define SDIO_CLKCTL_SDXFR         (SDIO_SDXFR_CLKDIV | SDIO_CLKCTL_EDGE | \
                                  SDIO_CLKCTL_WIDBUS_D1)
#define SDIO_CLKCTL_SDWIDEXFR     (SDIO_SDXFR_CLKDIV | SDIO_CLKCTL_EDGE | \
                                  SDIO_CLKCTL_WIDBUS_D4)

/* Timing */

#define SDIO_CMDTIMEOUT          (100000)
#define SDIO_LONGTIMEOUT         (0x7fffffff)

/* DTIMER setting */

/* Assuming Max timeout in bypass 48 Mhz */

#define IP_CLCK_FREQ               UINT32_C(48000000)
#define SDIO_DATATO_DATATIMEOUT_MS 250

/* SDIO DMA Channel/Stream selection.  For the case of the GD32 F4, there
 * are multiple DMA stream options that must be dis-ambiguated in the board.h
 * file.
 */

#if  defined(CONFIG_GD32F4_GD32F4XX)
#  define SDIO_DMACHAN           DMA_REQ_SDIO_1
#else
#  error "Unknown GD32 DMA"
#endif

/* FIFO sizes */

#define SDIO_HALFFIFO_WORDS      (8)
#define SDIO_HALFFIFO_BYTES      (8*4)

/* Data transfer interrupt bits */

#define SDIO_RECV_INTEN     (SDIO_INTEN_DTCRCERRIE | SDIO_INTEN_DTTMOUTIE | \
                            SDIO_INTEN_DTENDIE | SDIO_INTEN_RXOREIE | \
                            SDIO_INTEN_RFHIE | SDIO_INTEN_STBITEIE)
#define SDIO_SEND_INTEN     (SDIO_INTEN_DTCRCERRIE | SDIO_INTEN_DTTMOUTIE | \
                            SDIO_INTEN_DTENDIE | SDIO_INTEN_TXUREIE | \
                            SDIO_INTEN_TFHIE | SDIO_INTEN_STBITEIE)
#define SDIO_DMARECV_INTEN  (SDIO_INTEN_DTCRCERRIE | SDIO_INTEN_DTTMOUTIE | \
                            SDIO_INTEN_DTENDIE | SDIO_INTEN_RXOREIE | \
                            SDIO_INTEN_STBITEIE)
#define SDIO_DMASEND_INTEN  (SDIO_INTEN_DTCRCERRIE | SDIO_INTEN_DTTMOUTIE | \
                            SDIO_INTEN_DTENDIE | SDIO_INTEN_TXUREIE | \
                            SDIO_INTEN_STBITEIE)

/* Event waiting interrupt bits */

#define SDIO_CMDDONE_STAT   (SDIO_STAT_CMDSEND)
#define SDIO_RESPDONE_STAT  (SDIO_STAT_CMDTMOUT | SDIO_STAT_CCRCERR | \
                            SDIO_STAT_CMDRECV)
#define SDIO_XFRDONE_STAT   (0)

#define SDIO_CMDDONE_INTEN  (SDIO_INTEN_CMDSENDIE)
#define SDIO_RESPDONE_INTEN (SDIO_INTEN_CCRCERRIE | SDIO_INTEN_CMDTMOUTIE | \
                            SDIO_INTEN_CMDRECVIE)
#define SDIO_XFRDONE_INTEN  (0)

#define SDIO_CMDDONE_INTC   (SDIO_INTC_CMDSENDC | SDIO_INTC_DTBLKENDC)
#define SDIO_RESPDONE_INTC  (SDIO_INTC_CMDTMOUTC | SDIO_INTC_CCRCERRC | \
                            SDIO_INTC_CMDRECVC | SDIO_INTC_DTBLKENDC)
#define SDIO_XFRDONE_INTC   (SDIO_INTC_DTENDC | SDIO_INTC_DTCRCERRC | \
                            SDIO_INTC_DTTMOUTC | SDIO_INTC_RXOREC | \
                            SDIO_INTC_TXUREC | SDIO_INTC_STBITEC | \
                            SDIO_INTC_DTBLKENDC)

#define SDIO_WAITALL_INTC   (SDIO_CMDDONE_INTC | SDIO_RESPDONE_INTC | \
                            SDIO_XFRDONE_INTC | SDIO_INTC_DTBLKENDC)

/* Let's wait until we have both SDIO transfer complete and DMA complete. */

#define SDIO_XFRDONE_FLAG  (1)
#define SDIO_DMADONE_FLAG  (2)
#define SDIO_ALLDONE       (3)

#define GD32_SDIO_USE_DEFAULT_BLOCKSIZE (UINT8_MAX)

/****************************************************************************
 * Private Types
 ****************************************************************************/

  /* rcu  peripheral clock */

typedef enum
{
  ENABLE,
  DISABLE,
} periph_enum;

/* This structure defines the state of the GD32 SDIO interface */

struct gd32_dev_s
{
  struct sdio_dev_s  dev;             /* Standard, base SDIO interface */

  /* GD32-specific extensions */

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

#ifdef CONFIG_GD32_SDIO_CARD
  /* Interrupt at SDIO_D1 pin, only for SDIO cards */

  uint32_t           sdiointmask;            /* GD32 SDIO register mask */
  int               (*do_sdio_card)(void *); /* SDIO card ISR */
  void               *do_sdio_arg;           /* arg for SDIO card ISR */
#endif

  /* Fixed transfer block size support */

#ifdef CONFIG_SDIO_BLOCKSETUP
  uint8_t            block_size;
#endif

  /* DMA data transfer support */

  bool               widebus;         /* Required for DMA support */
#ifdef CONFIG_GD32F4_SDIO_DMA
  volatile uint8_t   xfrflags;        /* Used to synchronize SDIO and
                                       * DMA completion events */
  bool               dmamode;         /* true: DMA mode transfer */
  DMA_HANDLE         dma;             /* Handle for DMA channel */
#endif
};

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
struct gd32_sdioregs_s
{
  uint8_t  power;
  uint16_t clkctl;
  uint16_t datactl;
  uint32_t datato;
  uint32_t datalen;
  uint32_t datacnt;
  uint32_t stat;
  uint32_t inten;
  uint32_t fifocnt;
};

struct gd32_sampleregs_s
{
  struct gd32_sdioregs_s sdio;
#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_GD32F4_SDIO_DMA)
  struct gd32_dmaregs_s  dma;
#endif
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static inline void gd32_set_clkctl(uint32_t clkctl);
static void gd32_config_waitints(struct gd32_dev_s *priv, uint32_t waitmask,
              sdio_eventset_t waitevents, sdio_eventset_t wkupevents);
static void gd32_config_xfrints(struct gd32_dev_s *priv, uint32_t xfrmask);
static void gd32_set_pwrctl(uint32_t pwrctl);
static void gd32_sdio_clock_set(periph_enum type);

/* DMA Helpers **************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void gd32_sampleinit(void);
static void gd32_sdiosample(struct gd32_sdioregs_s *regs);
static void gd32_sample(struct gd32_dev_s *priv, int index);
static void gd32_sdiodump(struct gd32_sdioregs_s *regs, const char *msg);
static void gd32_dumpsample(struct gd32_dev_s *priv,
              struct gd32_sampleregs_s *regs, const char *msg);
static void gd32_dumpsamples(struct gd32_dev_s *priv);
#else
#  define   gd32_sampleinit()
#  define   gd32_sample(priv,index)
#  define   gd32_dumpsamples(priv)
#endif

#ifdef CONFIG_GD32F4_SDIO_DMA
static void gd32_dmacallback(DMA_HANDLE handle, uint16_t status, void *arg);
#endif

/* Data Transfer Helpers ****************************************************/

static uint8_t gd32_log2(uint16_t value);
static void gd32_data_config(uint32_t timeout, uint32_t dlen,
                             uint32_t dctl);
static void gd32_data_disable(void);
static void gd32_send_fifo(struct gd32_dev_s *priv);
static void gd32_recv_fifo(struct gd32_dev_s *priv);
static void gd32_event_timeout(wdparm_t arg);
static void gd32_end_wait(struct gd32_dev_s *priv,
              sdio_eventset_t wkupevent);
static void gd32_end_transfer(struct gd32_dev_s *priv,
              sdio_eventset_t wkupevent);

/* Interrupt Handling *******************************************************/

static int  gd32_interrupt(int irq, void *context, void *arg);

/* SDIO interface methods ***************************************************/

/* Initialization/setup */

static void gd32_reset(struct sdio_dev_s *dev);
static sdio_capset_t gd32_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t gd32_status(struct sdio_dev_s *dev);
static void gd32_widebus(struct sdio_dev_s *dev, bool enable);
static void gd32_clock(struct sdio_dev_s *dev,
              enum sdio_clock_e rate);
static int  gd32_attach(struct sdio_dev_s *dev);
static void sdio_deinit(void);

/* Command/Status/Data Transfer */

static int  gd32_send_cmd(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t arg);
#ifdef CONFIG_SDIO_BLOCKSETUP
static void gd32_block_setup(struct sdio_dev_s *dev,
              unsigned int blocklen, unsigned int nblocks);
#endif
static int  gd32_recv_setup(struct sdio_dev_s *dev, uint8_t *buffer,
              size_t nbytes);
static int  gd32_send_setup(struct sdio_dev_s *dev,
              const uint8_t *buffer, size_t nbytes);
static int  gd32_cancel(struct sdio_dev_s *dev);

static int  gd32_wait_response(struct sdio_dev_s *dev, uint32_t cmd);
static int  gd32_recv_shortcrc(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  gd32_recv_long(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t rlong[4]);
static int  gd32_recv_short(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);

/* EVENT handler */

static void gd32_wait_enable(struct sdio_dev_s *dev,
              sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t gd32_event_wait(struct sdio_dev_s *dev);
static void gd32_callback_enable(struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static int  gd32_register_callback(struct sdio_dev_s *dev,
              worker_t callback, void *arg);

/* DMA */

#ifdef CONFIG_GD32F4_SDIO_DMA
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
static int  gd32_dma_preflight(struct sdio_dev_s *dev,
              const uint8_t *buffer, size_t buflen);
#endif
static int  gd32_dmarecv_setup(struct sdio_dev_s *dev,
              uint8_t *buffer, size_t buflen);
static int  gd32_dmasend_setup(struct sdio_dev_s *dev,
              const uint8_t *buffer, size_t buflen);
#endif

/* Initialization/uninitialization/reset ************************************/

static void gd32_callback(void *arg);
static void gd32_default(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct gd32_dev_s g_sdiodev =
{
  .dev =
  {
#ifdef CONFIG_SDIO_MUXBUS
    .lock             = gd32_lock,
#endif
    .reset            = gd32_reset,
    .capabilities     = gd32_capabilities,
    .status           = gd32_status,
    .widebus          = gd32_widebus,
    .clock            = gd32_clock,
    .attach           = gd32_attach,
    .sendcmd          = gd32_send_cmd,
#ifdef CONFIG_SDIO_BLOCKSETUP
    .blocksetup       = gd32_block_setup,
#endif
    .recvsetup        = gd32_recv_setup,
    .sendsetup        = gd32_send_setup,
    .cancel           = gd32_cancel,
    .waitresponse     = gd32_wait_response,
    .recv_r1          = gd32_recv_shortcrc,
    .recv_r2          = gd32_recv_long,
    .recv_r3          = gd32_recv_short,
    .recv_r4          = gd32_recv_short,
    .recv_r5          = gd32_recv_shortcrc,
    .recv_r6          = gd32_recv_shortcrc,
    .recv_r7          = gd32_recv_short,
    .waitenable       = gd32_wait_enable,
    .eventwait        = gd32_event_wait,
    .callbackenable   = gd32_callback_enable,
    .registercallback = gd32_register_callback,
#ifdef CONFIG_SDIO_DMA
#ifdef CONFIG_GD32F4_SDIO_DMA
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
    .dmapreflight     = gd32_dma_preflight,
#endif
    .dmarecvsetup     = gd32_dmarecv_setup,
    .dmasendsetup     = gd32_dmasend_setup,
#else
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
    .dmapreflight     = NULL,
#endif
    .dmarecvsetup     = gd32_recv_setup,
    .dmasendsetup     = gd32_send_setup,
#endif
#endif
  },
  .waitsem = SEM_INITIALIZER(0),
};

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
static struct gd32_sampleregs_s g_sampleregs[DEBUG_NSAMPLES];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_sdio_clock_enable
 *
 * Description:
 * Enable SDIO clock
 ****************************************************************************/

static void gd32_sdio_clock_set(periph_enum type)
{
  uint32_t rcu_en;
  uint32_t regaddr;

  switch (type)
    {
    default:
      return;
    case ENABLE:
      rcu_en = RCU_APB2EN_SDIOEN;
      regaddr = GD32_RCU_APB2EN;
      modifyreg32(regaddr, 0, rcu_en);
      break;
    case DISABLE:
      rcu_en = RCU_APB2EN_SDIOEN;
      regaddr = GD32_RCU_APB2EN;
      modifyreg32(regaddr, rcu_en, 0);
      break;
    }
}

/****************************************************************************
 * Name: sdio_deinit
 *
 * Description:
 * deinitialize the SDIO
 ****************************************************************************/

static void sdio_deinit(void)
{
  uint32_t rcu_en;
  uint32_t regaddr;

  rcu_en = RCU_APB2RST_SDIORST;
  regaddr = GD32_RCU_APB2RST;
  modifyreg32(regaddr, 0, rcu_en);

  rcu_en  = RCU_APB2RST_SDIORST;
  regaddr = GD32_RCU_APB2RST;
  modifyreg32(regaddr, rcu_en, 0);
}

/****************************************************************************
 * Name: gd32_set_clkctl
 *
 * Description:
 *   Modify oft-changed bits in the CLKCTL register.  Only the following bit-
 *   fields are changed:
 *
 *   CLKDIV, PWRSAV, BYPASS, WIDBUS, NEGEDGE, and HWFC_EN
 *
 * Input Parameters:
 *   clkctl - A new CLKCTL setting for the above mentions bits (other bits
 *           are ignored.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void gd32_set_clkctl(uint32_t clkctl)
{
  uint32_t regval = getreg32(GD32_SDIO_CLKCTL);

  /* Clear CLKDIV, PWRSAV, BYPASS, WIDBUS, NEGEDGE, HWFC_EN bits */

  regval &= ~(SDIO_CLKCTL_CLKDIV_MASK | SDIO_CLKCTL_CLKPWRSAV |
              SDIO_CLKCTL_CLKBYP | SDIO_CLKCTL_WIDBUS_MASK |
              SDIO_CLKCTL_NEGEDGE | SDIO_CLKCTL_HWFC_EN |
              SDIO_CLKCTL_CLKEN);

  /* Replace with user provided settings */

  clkctl  &=  (SDIO_CLKCTL_CLKDIV_MASK | SDIO_CLKCTL_CLKPWRSAV |
              SDIO_CLKCTL_CLKBYP | SDIO_CLKCTL_WIDBUS_MASK |
              SDIO_CLKCTL_NEGEDGE | SDIO_CLKCTL_HWFC_EN |
              SDIO_CLKCTL_CLKEN);

  regval |=  clkctl;
  putreg32(regval, GD32_SDIO_CLKCTL);

  mcinfo("CLKCTL: %08" PRIx32 " PWR: %08" PRIx32 "\n",
         getreg32(GD32_SDIO_CLKCTL), getreg32(GD32_SDIO_PWRCTL));
}

/****************************************************************************
 * Name: gd32_config_waitints
 *
 * Description:
 *   Enable/disable SDIO interrupts needed to support the wait function
 *
 * Input Parameters:
 *   priv       - A reference to the SDIO device state structure
 *   waitmask   - The set of bits in the SDIO INTEN register to set
 *   waitevents - Waited for events
 *   wkupevent  - Wake-up events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gd32_config_waitints(struct gd32_dev_s *priv, uint32_t waitmask,
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
#ifdef CONFIG_GD32F4_SDIO_DMA
  priv->xfrflags   = 0;
#endif
#ifdef CONFIG_GD32_SDIO_CARD
  putreg32(priv->xfrmask | priv->waitmask | priv->sdiointmask,
           GD32_SDIO_INTEN);

#else
  putreg32(priv->xfrmask | priv->waitmask, GD32_SDIO_INTEN);
#endif

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: gd32_config_xfrints
 *
 * Description:
 *   Enable SDIO interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the SDIO device state structure
 *   xfrmask - The set of bits in the SDIO INTEN register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gd32_config_xfrints(struct gd32_dev_s *priv, uint32_t xfrmask)
{
  irqstate_t flags;

  flags = enter_critical_section();
  priv->xfrmask = xfrmask;
#ifdef CONFIG_GD32_SDIO_CARD
  putreg32(priv->xfrmask | priv->waitmask | priv->sdiointmask,
           GD32_SDIO_INTEN);
#else

  putreg32(priv->xfrmask | priv->waitmask, GD32_SDIO_INTEN);
#endif
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: gd32_set_pwrctl
 *
 * Description:
 *   Change the PWRCTRL field of the SDIO POWER register to turn the SDIO
 *   ON or OFF
 *
 * Input Parameters:
 *   pwrctl - A new PWRCTRL setting
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gd32_set_pwrctl(uint32_t pwrctl)
{
  uint32_t regval;

  regval  = getreg32(GD32_SDIO_PWRCTL);
  regval &= ~SDIO_PWRCTL_MASK;
  regval |= pwrctl;
  putreg32(regval, GD32_SDIO_PWRCTL);
}

/****************************************************************************
 * Name: gd32_sampleinit
 *
 * Description:
 *   Setup prior to collecting DMA samples
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void gd32_sampleinit(void)
{
  memset(g_sampleregs, 0xff,
         DEBUG_NSAMPLES * sizeof(struct gd32_sampleregs_s));
}
#endif

/****************************************************************************
 * Name: gd32_sdiosample
 *
 * Description:
 *   Sample SDIO registers
 *
 * Input Parameters:
 *   regs - SDIO registers structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void gd32_sdiosample(struct gd32_sdioregs_s *regs)
{
  regs->power    = (uint8_t)getreg32(GD32_SDIO_PWRCTL);
  regs->clkctl   = (uint16_t)getreg32(GD32_SDIO_CLKCTL);
  regs->datactl  = (uint16_t)getreg32(GD32_SDIO_DATACTL);
  regs->datato   = getreg32(GD32_SDIO_DATATO);
  regs->datalen  = getreg32(GD32_SDIO_DATALEN);
  regs->datacnt  = getreg32(GD32_SDIO_DATACNT);
  regs->stat     = getreg32(GD32_SDIO_STAT);
  regs->inten    = getreg32(GD32_SDIO_INTEN);
  regs->fifocnt  = getreg32(GD32_SDIO_FIFOCNT);
}
#endif

/****************************************************************************
 * Name: gd32_sample
 *
 * Description:
 *   Sample SDIO/DMA registers
 *
 * Input Parameters:
 *   priv   - A reference to the SDIO device state structure
 *   index  - index of the sample registers
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void gd32_sample(struct gd32_dev_s *priv, int index)
{
  struct gd32_sampleregs_s *regs = &g_sampleregs[index];

#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_GD32F4_SDIO_DMA)
  if (priv->dmamode)
    {
      gd32_dma_sample(priv->dma, &regs->dma);
    }
#endif

  gd32_sdiosample(&regs->sdio);
}
#endif

/****************************************************************************
 * Name: gd32_sdiodump
 *
 * Description:
 *   Dump one register sample
 *
 * Input Parameters:
 *   regs - SDIO registers structure
 *   msg  - message data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void gd32_sdiodump(struct gd32_sdioregs_s *regs, const char *msg)
{
  mcinfo("SDIO Registers: %s\n", msg);
  mcinfo("  POWER[%08x]: %08x\n", GD32_SDIO_PWRCTL,  regs->power);
  mcinfo(" CLKCTL[%08x]: %08x\n", GD32_SDIO_CLKCTL,  regs->clkctl);
  mcinfo("DATACTL[%08x]: %08x\n", GD32_SDIO_DATACTL, regs->datactl);
  mcinfo(" DATATO[%08x]: %08x\n", GD32_SDIO_DATATO,  regs->datato);
  mcinfo("DATALEN[%08x]: %08x\n", GD32_SDIO_DATALEN, regs->datalen);
  mcinfo(" DCOUNT[%08x]: %08x\n", GD32_SDIO_DATACNT, regs->datacnt);
  mcinfo("   STAT[%08x]: %08x\n", GD32_SDIO_STAT,    regs->stat);
  mcinfo("  INTEN[%08x]: %08x\n", GD32_SDIO_INTEN,   regs->inten);
  mcinfo("FIFOCNT[%08x]: %08x\n", GD32_SDIO_FIFOCNT, regs->fifocnt);
}
#endif

/****************************************************************************
 * Name: gd32_dumpsample
 *
 * Description:
 *   Dump one register sample
 *
 * Input Parameters:
 *   priv - A reference to the SDIO device state structure
 *   regs - SDIO registers structure
 *   msg  - message data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void gd32_dumpsample(struct gd32_dev_s *priv,
                            struct gd32_sampleregs_s *regs,
                            const char *msg)
{
#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_GD32F4_SDIO_DMA)
  if (priv->dmamode)
    {
      gd32_dma_sample(priv->dma, &regs->dma, msg);
    }
#endif

  gd32_sdiodump(&regs->sdio, msg);
}
#endif

/****************************************************************************
 * Name: gd32_dumpsamples
 *
 * Description:
 *   Dump all sampled register data
 *
 * Input Parameters:
 *   priv - A reference to the SDIO device state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void gd32_dumpsamples(struct gd32_dev_s *priv)
{
  gd32_dumpsample(priv, &g_sampleregs[SAMPLENDX_BEFORE_SETUP],
                  "Before setup");

#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_GD32F4_SDIO_DMA)
  if (priv->dmamode)
    {
      gd32_dumpsample(priv, &g_sampleregs[SAMPLENDX_BEFORE_ENABLE],
                      "Before DMA enable");
    }
#endif

  gd32_dumpsample(priv, &g_sampleregs[SAMPLENDX_AFTER_SETUP],
                  "After setup");
  gd32_dumpsample(priv, &g_sampleregs[SAMPLENDX_END_TRANSFER],
                  "End of transfer");

#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_GD32F4_SDIO_DMA)
  if (priv->dmamode)
    {
      gd32_dumpsample(priv, &g_sampleregs[SAMPLENDX_DMA_CALLBACK],
                      "DMA Callback");
    }
#endif
}
#endif

/****************************************************************************
 * Name: gd32_dmacallback
 *
 * Description:
 *   Called when SDIO DMA completes
 *
 * Input Parameters:
 *   handle - DMA channel handle
 *   status - DMA transfer status
 *   arg    - The argument
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_GD32F4_SDIO_DMA
static void gd32_dmacallback(DMA_HANDLE handle, uint16_t status, void *arg)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)arg;
  DEBUGASSERT(priv->dmamode);
  sdio_eventset_t result;

  /* In the normal case, SDIO appears to handle the End-Of-Transfer interrupt
   * first with the End-Of-DMA event occurring significantly later.  On
   * transfer errors, however, the DMA error will occur before the End-of-
   * Transfer.
   */

  gd32_sample((struct gd32_dev_s *)arg, SAMPLENDX_DMA_CALLBACK);

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
      gd32_end_transfer(priv, result);
    }
}
#endif

/****************************************************************************
 * Name: gd32_log2
 *
 * Description:
 *   Take (approximate) log base 2 of the provided number (Only works if the
 *   provided number is a power of 2).
 *
 * Input Parameters:
 *   value - The provide number
 *
 * Returned Value:
 *   The log base 2 of the provided number
 *
 ****************************************************************************/

static uint8_t gd32_log2(uint16_t value)
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
 * Name: gd32_data_config
 *
 * Description:
 *   Configure the SDIO data path for the next data transfer
 *
 * Input Parameters:
 *   timeout - The timeout value to wait
 *   dlen    - Data length to transfer
 *   dctl    - Data transfer control
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gd32_data_config(uint32_t timeout, uint32_t dlen, uint32_t dctl)
{
  uint32_t clkdiv;
  uint32_t regval;
  uint32_t sdio_clk = IP_CLCK_FREQ;

  /* Enable data path using a timeout scaled to the SD_CLOCK (the card
   * clock).
   */

  regval = getreg32(GD32_SDIO_CLKCTL);
  clkdiv = (regval & SDIO_CLKCTL_CLKDIV_MASK) >> SDIO_CLKCTL_CLKDIV_SHIFT;
  if ((regval & SDIO_CLKCTL_CLKBYP) == 0)
    {
      sdio_clk = sdio_clk / (2 + clkdiv);
    }

  /*  Convert Timeout in Ms to SD_CLK counts */

  timeout  = timeout * (sdio_clk / 1000);

  putreg32(timeout, GD32_SDIO_DATATO);  /* Set DTIMER */
  putreg32(dlen,    GD32_SDIO_DATALEN); /* Set DATALEN */

  /* Configure DATACTL DTDIR, DTMODE, and DBLOCKSIZE fields and set the DTEN
   * field
   */

  regval = getreg32(GD32_SDIO_DATACTL);
  regval &= ~(SDIO_DATACTL_DATADIR | SDIO_DATACTL_TRANSMOD |
              SDIO_DATACTL_DBLOCKSIZE_MASK);
  dctl   &= (SDIO_DATACTL_DATADIR | SDIO_DATACTL_TRANSMOD |
             SDIO_DATACTL_DBLOCKSIZE_MASK);
  regval |= (dctl | SDIO_DATACTL_DATAEN | SDIO_DATACTL_IOEN);
  putreg32(regval, GD32_SDIO_DATACTL);
}

/****************************************************************************
 * Name: gd32_data_disable
 *
 * Description:
 *   Disable the SDIO data path setup by gd32_data_config() and
 *   disable DMA.
 *
 ****************************************************************************/

static void gd32_data_disable(void)
{
  uint32_t regval;

  /* Disable the data path  */

  /* Reset DTIMER */

  putreg32(UINT32_MAX, GD32_SDIO_DATATO);

  /* Reset DLEN */

  putreg32(0, GD32_SDIO_DATALEN);

  /* Reset DATACTL DTEN, DTDIR, DTMODE, DMAEN, and DBLOCKSIZE fields */

  regval  = getreg32(GD32_SDIO_DATACTL);
  regval &= ~(SDIO_DATACTL_DATAEN | SDIO_DATACTL_DATADIR |
              SDIO_DATACTL_TRANSMOD | SDIO_DATACTL_DMAEN |
              SDIO_DATACTL_DBLOCKSIZE_MASK);
  putreg32(regval, GD32_SDIO_DATACTL);
}

/****************************************************************************
 * Name: gd32_send_fifo
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

static void gd32_send_fifo(struct gd32_dev_s *priv)
{
  union
  {
    uint32_t w;
    uint8_t  b[4];
  } data;

  /* Loop while there is more data to be sent and the RX FIFO is not full */

  while (priv->remaining > 0 &&
         (getreg32(GD32_SDIO_STAT) & SDIO_STAT_TFF) == 0)
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

      putreg32(data.w, GD32_SDIO_FIFO);
    }
}

/****************************************************************************
 * Name: gd32_recv_fifo
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

static void gd32_recv_fifo(struct gd32_dev_s *priv)
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
         (getreg32(GD32_SDIO_STAT) & SDIO_STAT_RXDTVAL) != 0)
    {
      /* Read the next word from the RX FIFO */

      data.w = getreg32(GD32_SDIO_FIFO);
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
 * Name: gd32_event_timeout
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

static void gd32_event_timeout(wdparm_t arg)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)arg;

  /* There is always race conditions with timer expirations. */

  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0 ||
              priv->wkupevent != 0);

  mcinfo("stat: %08" PRIx32 " enabled irq: %08" PRIx32 "\n",
         getreg32(GD32_SDIO_STAT),
         getreg32(GD32_SDIO_INTEN));

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. wake up any waiting threads */

    gd32_end_wait(priv, SDIOWAIT_TIMEOUT);

      mcerr("Timeout: remaining: %d\n", priv->remaining);
    }
}

/****************************************************************************
 * Name: gd32_end_wait
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

static void gd32_end_wait(struct gd32_dev_s *priv,
                          sdio_eventset_t wkupevent)
{
  /* Cancel the watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Disable event-related interrupts */

  gd32_config_waitints(priv, 0, 0, wkupevent);

  /* Wake up the waiting thread */

  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: gd32_end_transfer
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

static void gd32_end_transfer(struct gd32_dev_s *priv,
                              sdio_eventset_t wkupevent)
{
  /* Disable all transfer related interrupts */

  gd32_config_xfrints(priv, 0);

  /* Clearing pending interrupt status on all transfer related interrupts */

  putreg32(SDIO_XFRDONE_INTC, GD32_SDIO_INTC);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_GD32F4_SDIO_DMA
  if (priv->dmamode)
    {
      /* DMA debug instrumentation */

      gd32_sample(priv, SAMPLENDX_END_TRANSFER);

      /* Make sure that the DMA is stopped (it will be stopped automatically
       * on normal transfers, but not necessarily when the transfer
       * terminates on an error condition).
       */

      gd32_dma_stop(priv->dma);
    }
#endif

  /* Mark the transfer finished */

  priv->remaining = 0;

  /* Is a thread wait for these data transfer complete events? */

  if ((priv->waitevents & wkupevent) != 0)
    {
      /* Yes.. wake up any waiting threads */

      gd32_end_wait(priv, wkupevent);
    }
}

/****************************************************************************
 * Name: gd32_interrupt
 *
 * Description:
 *   SDIO interrupt handler
 *
 * Input Parameters:
 *   irq     - The IRQ number
 *   context - The interrupt context
 *   arg     - The argument
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int gd32_interrupt(int irq, void *context, void *arg)
{
  struct gd32_dev_s *priv = &g_sdiodev;
  uint32_t enabled;
  uint32_t pending;

  /* Loop while there are pending interrupts.  Check the SDIO status
   * register.  Mask out all bits that don't correspond to enabled
   * interrupts.  (This depends on the fact that bits are ordered
   * the same in both the STA and INTEN register).  If there are non-zero
   * bits remaining, then we have work to do here.
   */

  while ((enabled = getreg32(GD32_SDIO_STAT) &
                    getreg32(GD32_SDIO_INTEN)) != 0)
    {
      /* Handle in progress, interrupt driven data transfers ****************/

      pending  = enabled & priv->xfrmask;
      if (pending != 0)
        {
#ifdef CONFIG_GD32F4_SDIO_DMA
          if (!priv->dmamode)
#endif
            {
              /* Is the RX FIFO half full or more?  Is so then we must be
               * processing a receive transaction.
               */

              if ((pending & SDIO_STAT_RFH) != 0)
                {
                  /* Receive data from the RX FIFO */

                  gd32_recv_fifo(priv);
                }

              /* Otherwise, Is the transmit FIFO half empty or less?  If so
               * we must be processing a send transaction.  NOTE:  We can't
               * be processing both!
               */

              else if ((pending & SDIO_STAT_TFH) != 0)
                {
                  /* Send data via the TX FIFO */

                  gd32_send_fifo(priv);
                }
            }

          /* Handle data end events */

          if ((pending & SDIO_STAT_DTEND) != 0)
            {
              /* Handle any data remaining the RX FIFO.  If the RX FIFO is
               * less than half full at the end of the transfer, then no
               * half-full interrupt will be received.
               */

              /* Was this transfer performed in DMA mode? */

#ifdef CONFIG_GD32F4_SDIO_DMA
              if (priv->dmamode)
                {
                  /* Yes.. Terminate the transfers only if the DMA has also
                   * finished.
                   */

                  priv->xfrflags |= SDIO_XFRDONE_FLAG;
                  if (priv->xfrflags == SDIO_ALLDONE)
                    {
                      gd32_end_transfer(priv, SDIOWAIT_TRANSFERDONE);
                    }

                  /* Otherwise, just disable further transfer interrupts and
                   * wait for the DMA complete event.
                   */

                  else
                    {
                      gd32_config_xfrints(priv, 0);
                    }
                }
              else
#endif
                {
                  /* Receive data from the RX FIFO */

                  gd32_recv_fifo(priv);

                  /* Then terminate the transfer */

                  gd32_end_transfer(priv, SDIOWAIT_TRANSFERDONE);
                }
            }

          /* Handle data block send/receive CRC failure */

          else if ((pending & SDIO_STAT_DTCRCERR) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Data block CRC failure, remaining: %d\n",
                    priv->remaining);
              gd32_end_transfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle data timeout error */

          else if ((pending & SDIO_STAT_DTTMOUT) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Data timeout, remaining: %d\n",
                     priv->remaining);
              gd32_end_transfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT);
            }

          /* Handle RX FIFO overrun error */

          else if ((pending & SDIO_STAT_RXORE) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: RX FIFO overrun, remaining: %d\n",
                    priv->remaining);
              gd32_end_transfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle TX FIFO underrun error */

          else if ((pending & SDIO_STAT_TXURE) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: TX FIFO underrun, remaining: %d\n",
                    priv->remaining);
              gd32_end_transfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle start bit error */

          else if ((pending & SDIO_STAT_STBITE) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Start bit, remaining: %d\n",
                    priv->remaining);
              gd32_end_transfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }
        }

      /* Handle wait events *************************************************/

      pending  = enabled & priv->waitmask;
      if (pending != 0)
        {
          /* Is this a response completion event? */

          if ((pending & SDIO_RESPDONE_STAT) != 0)
            {
              /* Yes.. Is their a thread waiting for response done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  putreg32(SDIO_RESPDONE_INTC | SDIO_CMDDONE_INTC,
                           GD32_SDIO_INTC);
                  gd32_end_wait(priv, SDIOWAIT_RESPONSEDONE);
                }
            }

          /* Is this a command completion event? */

          if ((pending & SDIO_CMDDONE_STAT) != 0)
            {
              /* Yes.. Is their a thread waiting for command done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  putreg32(SDIO_CMDDONE_INTC, GD32_SDIO_INTC);
                  gd32_end_wait(priv, SDIOWAIT_CMDDONE);
                }
            }
        }

#ifdef CONFIG_GD32_SDIO_CARD
      /* Handle SDIO card interrupt */

      pending = enabled & priv->sdiointmask;
      if (pending != 0)
        {
          putreg32(SDIO_STAT_SDIOINT, GD32_SDIO_INTC);

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
 * Name: gd32_reset
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

static void gd32_reset(struct sdio_dev_s *dev)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;
  irqstate_t flags;
  uint32_t regval;

  /* Disable clocking */

  flags = enter_critical_section();
  regval = getreg32(GD32_SDIO_CLKCTL);
  regval &= ~(1 << 8);
  putreg32(regval, GD32_SDIO_CLKCTL);
  gd32_set_pwrctl(SDIO_PWRCTL_OFF);

  /* Put SDIO registers in their default, reset state */

  gd32_default();

  /* Reset data */

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitmask   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */
#ifdef CONFIG_GD32F4_SDIO_DMA
  priv->xfrflags   = 0;      /* Used to synchronize SDIO and DMA completion events */
#endif

  wd_cancel(&priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->buffer     = 0;      /* Address of current R/W buffer */
  priv->remaining  = 0;      /* Number of bytes remaining in the transfer */
  priv->xfrmask    = 0;      /* Interrupt enables for data transfer */

#ifdef CONFIG_GD32_SDIO_CARD
  priv->sdiointmask = 0;     /* SDIO card in-band interrupt mask */
#endif

  /* DMA data transfer support */

  priv->widebus    = false;  /* Required for DMA support */
#ifdef CONFIG_GD32F4_SDIO_DMA
  priv->dmamode    = false;  /* true: DMA mode transfer */
#endif

  /* Configure the SDIO peripheral */

  gd32_set_clkctl(GD32_CLCKCTL_INIT | SDIO_CLKCTL_CLKEN);
  gd32_set_pwrctl(SDIO_PWRCTL_ON);

  leave_critical_section(flags);

  mcinfo("CLCKR: %08" PRIx32 " POWER: %08" PRIx32 "\n",
         getreg32(GD32_SDIO_CLKCTL), getreg32(GD32_SDIO_PWRCTL));
}

/****************************************************************************
 * Name: gd32_capabilities
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

static sdio_capset_t gd32_capabilities(struct sdio_dev_s *dev)
{
  sdio_capset_t caps = 0;

  caps |= SDIO_CAPS_4BIT_ONLY;

#ifdef CONFIG_GD32_SDIO_WIDTH_D1_ONLY
  caps = 0;
  caps |= SDIO_CAPS_1BIT_ONLY;
#endif
#ifdef CONFIG_GD32F4_SDIO_DMA
  caps = 0;
  caps |= SDIO_CAPS_DMASUPPORTED;
#endif

  return caps;
}

/****************************************************************************
 * Name: gd32_status
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see gd32_status_* defines)
 *
 ****************************************************************************/

static sdio_statset_t gd32_status(struct sdio_dev_s *dev)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;
  return priv->cdstatus;
}

/****************************************************************************
 * Name: gd32_widebus
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

static void gd32_widebus(struct sdio_dev_s *dev, bool wide)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;
  priv->widebus = wide;
}

/****************************************************************************
 * Name: gd32_clock
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

static void gd32_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  uint32_t clkctl;

    switch (rate)
      {
        /* Disable clocking (with default ID mode divisor) */

        default:
        case CLOCK_SDIO_DISABLED:
          clkctl = GD32_CLCKCTL_INIT;
          break;

        /* Enable in initial ID mode clocking (<400KHz) */

        case CLOCK_IDMODE:
          clkctl = (GD32_CLCKCTL_INIT | SDIO_CLKCTL_CLKEN);
          break;

        /* Enable in MMC normal operation clocking */

        case CLOCK_MMC_TRANSFER:
          clkctl = (SDIO_CLKCTL_MMCXFR | SDIO_CLKCTL_CLKEN);
          break;

        /* SD normal operation clocking (wide 4-bit mode) */

        case CLOCK_SD_TRANSFER_4BIT:
#ifndef CONFIG_GD32_SDIO_WIDTH_D1_ONLY
          clkctl = (SDIO_CLKCTL_SDWIDEXFR | SDIO_CLKCTL_CLKEN);
          break;
#endif

        /* SD normal operation clocking (narrow 1-bit mode) */

        case CLOCK_SD_TRANSFER_1BIT:
          clkctl = (SDIO_CLKCTL_SDXFR | SDIO_CLKCTL_CLKEN);
          break;
      }

    /* Set the new clock frequency along with the clock enable/disable bit */

    gd32_set_clkctl(clkctl);
}

/****************************************************************************
 * Name: gd32_attach
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

static int gd32_attach(struct sdio_dev_s *dev)
{
  int ret;

  /* Attach the SDIO interrupt handler */

  ret = irq_attach(GD32_IRQ_SDIO, gd32_interrupt, NULL);
  if (ret == OK)
    {
      /* Disable all interrupts at the SDIO controller and clear static
       * interrupt flags
       */

      putreg32(SDIO_INTEN_RESET,      GD32_SDIO_INTEN);
      putreg32(SDIO_INTC_STATICFLAGS,  GD32_SDIO_INTC);

      /* Enable SDIO interrupts at the NVIC.  They can now be enabled at
       * the SDIO controller as needed.
       */

      up_enable_irq(GD32_IRQ_SDIO);
    }

  return ret;
}

/****************************************************************************
 * Name: gd32_send_cmd
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

static int gd32_send_cmd(struct sdio_dev_s *dev, uint32_t cmd,
                         uint32_t arg)
{
  uint32_t regval;
  uint32_t cmdidx;

  /* Set the SDIO Argument value */

  putreg32(arg, GD32_SDIO_CMDAGMT);

  /* Clear CMDINDEX, WAITRESP, WAITINT, WAITPEND, and CPSMEN bits */

  regval = getreg32(GD32_SDIO_CMDCTL);
  regval &= ~(SDIO_CMDCTL_CMDINDEX_MASK | SDIO_CMDCTL_WAITRESP_MASK |
            SDIO_CMDCTL_INTWAIT | SDIO_CMDCTL_WAITDEND | SDIO_CMDCTL_CSMEN);

  /* Set WAITRESP bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      regval |= SDIO_CMDCTL_NORESPONSE;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
    case MMCSD_R7_RESPONSE:
      regval |= SDIO_CMDCTL_SHORTRESPONSE;
      break;

    case MMCSD_R2_RESPONSE:
      regval |= SDIO_CMDCTL_LONGRESPONSE;
      break;
    }

  /* Set CPSMEN and the command index */

  cmdidx  = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval |= cmdidx | SDIO_CMDCTL_CSMEN;

  mcinfo("cmd: %08" PRIx32 " arg: %08" PRIx32 " regval: %08" PRIx32
         " enabled irq: %08" PRIx32 "\n",
         cmd, arg, regval, getreg32(GD32_SDIO_INTEN));

  /* Write the SDIO CMD */

  putreg32(SDIO_RESPDONE_INTC | SDIO_CMDDONE_INTC, GD32_SDIO_INTC);
  putreg32(regval, GD32_SDIO_CMDCTL);
  return OK;
}

/****************************************************************************
 * Name: gd32_block_setup
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
static void gd32_block_setup(struct sdio_dev_s *dev,
                             unsigned int blocklen, unsigned int nblocks)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;

  /* Configure block size for next transfer */

  priv->block_size = gd32_log2(blocklen);
}
#endif

/****************************************************************************
 * Name: gd32_recv_setup
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

static int gd32_recv_setup(struct sdio_dev_s *dev, uint8_t *buffer,
                           size_t nbytes)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;
  uint32_t dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  gd32_data_disable();
  gd32_sampleinit();
  gd32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
#ifdef CONFIG_GD32F4_SDIO_DMA
  priv->dmamode   = false;
#endif

  /* Then set up the SDIO data path */

#ifdef CONFIG_SDIO_BLOCKSETUP
  if (priv->block_size != GD32_SDIO_USE_DEFAULT_BLOCKSIZE)
    {
      dblocksize = priv->block_size << SDIO_DATACTL_DBLOCKSIZE_SHIFT;
    }
  else
#endif
    {
      dblocksize = gd32_log2(nbytes) << SDIO_DATACTL_DBLOCKSIZE_SHIFT;
    }

  gd32_data_config(SDIO_DATATO_DATATIMEOUT_MS, nbytes,
                   dblocksize | SDIO_DATACTL_DATADIR);

  /* And enable interrupts */

  gd32_config_xfrints(priv, SDIO_RECV_INTEN);
  gd32_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}

/****************************************************************************
 * Name: gd32_send_setup
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

static int gd32_send_setup(struct sdio_dev_s *dev,
                           const uint8_t *buffer, size_t nbytes)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;
  uint32_t dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  gd32_data_disable();
  gd32_sampleinit();
  gd32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
#ifdef CONFIG_GD32F4_SDIO_DMA
  priv->dmamode   = false;
#endif

  /* Then set up the SDIO data path */

#ifdef CONFIG_SDIO_BLOCKSETUP
  if (priv->block_size != GD32_SDIO_USE_DEFAULT_BLOCKSIZE)
    {
      dblocksize = priv->block_size << SDIO_DATACTL_DBLOCKSIZE_SHIFT;
    }
  else
#endif
    {
      dblocksize = gd32_log2(nbytes) << SDIO_DATACTL_DBLOCKSIZE_SHIFT;
    }

  gd32_data_config(SDIO_DATATO_DATATIMEOUT_MS, nbytes, dblocksize);

  /* Enable TX interrupts */

  gd32_config_xfrints(priv, SDIO_SEND_INTEN);
  gd32_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}

/****************************************************************************
 * Name: gd32_cancel
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

static int gd32_cancel(struct sdio_dev_s *dev)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;

  /* Disable all transfer- and event- related interrupts */

  gd32_config_xfrints(priv, 0);
  gd32_config_waitints(priv, 0, 0, 0);

  /* Clearing pending interrupt status on all transfer- and event- related
   * interrupts
   */

  putreg32(SDIO_WAITALL_INTC, GD32_SDIO_INTC);

  /* Cancel any watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_GD32F4_SDIO_DMA
  if (priv->dmamode)
    {
      /* Make sure that the DMA is stopped (it will be stopped automatically
       * on normal transfers, but not necessarily when the transfer
       * terminates on an error condition.
       */

      gd32_dma_stop(priv->dma);
    }
#endif

  /* Mark no transfer in progress */

  priv->remaining = 0;
  return OK;
}

/****************************************************************************
 * Name: gd32_wait_response
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

static int gd32_wait_response(struct sdio_dev_s *dev, uint32_t cmd)
{
  int32_t timeout;
  uint32_t events;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      events  = SDIO_CMDDONE_STAT;
      timeout = SDIO_CMDTIMEOUT;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
      events  = SDIO_RESPDONE_STAT;
      timeout = SDIO_LONGTIMEOUT;
      break;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      events  = SDIO_RESPDONE_STAT;
      timeout = SDIO_CMDTIMEOUT;
      break;

    default:
      return -EINVAL;
    }

  /* Then wait for the response (or timeout) */

  while ((getreg32(GD32_SDIO_STAT) & events) == 0)
    {
      if (--timeout <= 0)
        {
          mcerr("ERROR: Timeout cmd: %08" PRIx32 " events: %08" PRIx32
                " STAT: %08" PRIx32 "\n",
                cmd, events, getreg32(GD32_SDIO_STAT));

          return -ETIMEDOUT;
        }
    }

  putreg32(SDIO_CMDDONE_INTC, GD32_SDIO_INTC);
  return OK;
}

/****************************************************************************
 * Name: gd32_recv*
 *
 * Description:
 *   Receive response to SDIO command.  Only the critical payload is
 *   returned -- that is 32 bits for 48 bit status and 128 bits for 136 bit
 *   status.  The driver implementation should verify the correctness of
 *   the remaining, non-returned bits (CRCs, CMD index, etc.).
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   cmd    - Response command
 *   rshort - Short response
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure.  Here a
 *   failure means only a faiure to obtain the requested response (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intacta and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int gd32_recv_shortcrc(struct sdio_dev_s *dev, uint32_t cmd,
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

      regval = getreg32(GD32_SDIO_STAT);
      if ((regval & SDIO_STAT_CMDTMOUT) != 0)
        {
          mcerr("ERROR: Command timeout: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & SDIO_STAT_CCRCERR) != 0)
        {
          mcerr("ERROR: CRC failure: %08" PRIx32 "\n", regval);
          ret = -EIO;
        }
#ifdef CONFIG_DEBUG_MEMCARD_INFO
      else
        {
          /* Check response received is of desired command */

          respcmd = getreg32(GD32_SDIO_RSPCMDIDX);
          if ((uint8_t)(respcmd & SDIO_RSPCMDIDX_MASK) !=
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

  putreg32(SDIO_RESPDONE_INTC | SDIO_CMDDONE_INTC, GD32_SDIO_INTC);
  *rshort = getreg32(GD32_SDIO_RESP0);
  return ret;
}

static int gd32_recv_long(struct sdio_dev_s *dev, uint32_t cmd,
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

      regval = getreg32(GD32_SDIO_STAT);
      if (regval & SDIO_STAT_CMDTMOUT)
        {
          mcerr("ERROR: Timeout STAT: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & SDIO_STAT_CCRCERR)
        {
          mcerr("ERROR: CRC fail STAT: %08" PRIx32 "\n", regval);
          ret = -EIO;
        }
    }

  /* Return the long response */

  putreg32(SDIO_RESPDONE_INTC | SDIO_CMDDONE_INTC, GD32_SDIO_INTC);
  if (rlong)
    {
      rlong[0] = getreg32(GD32_SDIO_RESP0);
      rlong[1] = getreg32(GD32_SDIO_RESP1);
      rlong[2] = getreg32(GD32_SDIO_RESP2);
      rlong[3] = getreg32(GD32_SDIO_RESP3);
    }

  return ret;
}

static int gd32_recv_short(struct sdio_dev_s *dev, uint32_t cmd,
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

      regval = getreg32(GD32_SDIO_STAT);
      if (regval & SDIO_STAT_CMDTMOUT)
        {
          mcerr("ERROR: Timeout STAT: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
    }

  putreg32(SDIO_RESPDONE_INTC | SDIO_CMDDONE_INTC, GD32_SDIO_INTC);
  if (rshort)
    {
      *rshort = getreg32(GD32_SDIO_RESP0);
    }

  return ret;
}

/****************************************************************************
 * Name: gd32_wait_enable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the SDIO_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling gd32_event_wait.  This is done in this way
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
 *   timeout - Maximum time in milliseconds to wait.  Zero means immediate
 *             timeout with no wait.  The timeout value is ignored if
 *             SDIOWAIT_TIMEOUT is not included in the waited-for eventset.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gd32_wait_enable(struct sdio_dev_s *dev,
                             sdio_eventset_t eventset, uint32_t timeout)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;
  uint32_t waitmask;

  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  gd32_config_waitints(priv, 0, 0, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

    {
      waitmask = 0;
      if ((eventset & SDIOWAIT_CMDDONE) != 0)
        {
          waitmask |= SDIO_CMDDONE_INTEN;
        }

      if ((eventset & SDIOWAIT_RESPONSEDONE) != 0)
        {
          waitmask |= SDIO_RESPDONE_INTEN;
        }

      if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
        {
          waitmask |= SDIO_XFRDONE_INTEN;
        }

      /* Enable event-related interrupts */

      putreg32(SDIO_WAITALL_INTC, GD32_SDIO_INTC);
    }

  gd32_config_waitints(priv, waitmask, eventset, 0);

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
                       gd32_event_timeout, (wdparm_t)priv);
      if (ret < 0)
        {
          mcerr("ERROR: wd_start failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: gd32_event_wait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDIO_WAITEVENTS are disabled when gd32_event_wait
 *   returns.  SDIO_WAITEVENTS must be called again before gd32_event_wait
 *   can be used again.
 *
 * Input Parameters:
 *   dev     - An instance of the SDIO device interface
 *
 * Returned Value:
 *   Event set containing the event(s) that ended the wait.  Should always
 *   be non-zero.  All events are disabled after the wait concludes.
 *
 ****************************************************************************/

static sdio_eventset_t gd32_event_wait(struct sdio_dev_s *dev)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;
  sdio_eventset_t wkupevent = 0;
  irqstate_t flags;
  int ret;

  /* There is a race condition here... the event may have completed before
   * we get here.  In this case waitevents will be zero, but wkupevents will
   * be non-zero (and, hopefully, the semaphore count will also be non-zero.
   */

  flags = enter_critical_section();

  /* A card ejected while in SDIOWAIT_WRCOMPLETE can lead to a
   * condition where there is no waitevents set and no wkupevent
   */

  DEBUGASSERT(priv->waitevents != 0 || priv->wkupevent != 0);

  /* Loop until the event (or the timeout occurs). Race conditions are
   * avoided by calling gd32_wait_enable prior to triggering the logic that
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
  gd32_config_waitints(priv, 0, 0, 0);
#ifdef CONFIG_GD32F4_SDIO_DMA
  priv->xfrflags   = 0;
#endif

  leave_critical_section(flags);
  gd32_dumpsamples(priv);
  return wkupevent;
}

/****************************************************************************
 * Name: gd32_callback_enable
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events.  This is part of the
 *   the SDIO callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in gd32_register_callback.
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

static void gd32_callback_enable(struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;

  mcinfo("eventset: %02x\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  gd32_callback(priv);
}

/****************************************************************************
 * Name: gd32_register_callback
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

static int gd32_register_callback(struct sdio_dev_s *dev,
                                  worker_t callback, void *arg)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;

  /* Disable callbacks and register this callback and is argument */

  mcinfo("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: gd32_dma_preflight
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

#if defined(CONFIG_GD32F4_SDIO_DMA) && defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
static int gd32_dma_preflight(struct sdio_dev_s *dev,
                              const uint8_t *buffer, size_t buflen)
{
#if !defined(CONFIG_GD32F4_GD32F4XX)
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);

  /* Wide bus operation is required for DMA */

  if (!priv->widebus)
    {
      return -EINVAL;
    }
#endif

  return 0;
}
#endif

/****************************************************************************
 * Name: gd32_dmarecv_setup
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

#ifdef CONFIG_GD32F4_SDIO_DMA
static int gd32_dmarecv_setup(struct sdio_dev_s *dev,
                              uint8_t *buffer, size_t buflen)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;
  uint32_t dblocksize;
  uint32_t regval;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
  DEBUGASSERT(gd32_dma_preflight(dev, buffer, buflen) == 0);
#endif

  /* Reset the DPSM configuration */

  gd32_data_disable();

  /* Initialize register sampling */

  gd32_sampleinit();
  gd32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->dmamode   = true;

  /* Then set up the SDIO data path */

#ifdef CONFIG_SDIO_BLOCKSETUP
  if (priv->block_size != GD32_SDIO_USE_DEFAULT_BLOCKSIZE)
    {
      dblocksize = priv->block_size << SDIO_DATACTL_DBLOCKSIZE_SHIFT;
    }
  else
#endif
    {
      dblocksize = gd32_log2(buflen) << SDIO_DATACTL_DBLOCKSIZE_SHIFT;
    }

  gd32_data_config(SDIO_DATATO_DATATIMEOUT_MS, buflen,
                   dblocksize | SDIO_DATACTL_DATADIR);

  /* Configure the RX DMA */

  gd32_config_xfrints(priv, SDIO_DMARECV_MASK);

  regval = getreg32(GD32_SDIO_DATACTL);
  regval |= (1 << 3);
  putreg32(regval, GD32_SDIO_DATACTL);

  dma_single_data_parameter_struct dma_init_struct;

  dma_init_struct.memory0_addr = (uint32_t)priv->buffer;
  dma_init_struct.number = buflen / 4;
  dma_init_struct.periph_addr = GD32_SDIO_FIFO;
  dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
  dma_init_struct.periph_memory_width = (uint8_t)DMA_WIDTH_32BITS_SELECT;
  dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  dma_init_struct.priority = (uint8_t)DMA_PRIO_ULTRA_HIGHSELECT;
  dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_DISABLE;

  gd32_dma_setup(priv->dma, &dma_init_struct, 1);

  /* Start the DMA */

  gd32_sample(priv, SAMPLENDX_BEFORE_ENABLE);
  gd32_dma_start(priv->dma, gd32_dmacallback, priv, SDIO_DMA_INTEN);
  gd32_sample(priv, SAMPLENDX_AFTER_SETUP);

  return OK;
}
#endif

/****************************************************************************
 * Name: gd32_dmasend_setup
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

#ifdef CONFIG_GD32F4_SDIO_DMA
static int gd32_dmasend_setup(struct sdio_dev_s *dev,
                              const uint8_t *buffer, size_t buflen)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;
  uint32_t dblocksize;
  uint32_t regval;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
  DEBUGASSERT(gd32_dma_preflight(dev, buffer, buflen) == 0);
#endif

  /* Reset the DPSM configuration */

  gd32_data_disable();

  /* Initialize register sampling */

  gd32_sampleinit();
  gd32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->dmamode   = true;

  /* Then set up the SDIO data path */

#ifdef CONFIG_SDIO_BLOCKSETUP
  if (priv->block_size != GD32_SDIO_USE_DEFAULT_BLOCKSIZE)
    {
      dblocksize = priv->block_size << SDIO_DATACTL_DBLOCKSIZE_SHIFT;
    }
  else
#endif
    {
      dblocksize = gd32_log2(buflen) << SDIO_DATACTL_DBLOCKSIZE_SHIFT;
    }

  gd32_data_config(SDIO_DATATO_DATATIMEOUT_MS, buflen, dblocksize);

  /* Configure the TX DMA */

  dma_single_data_parameter_struct dma_init_struct;

  dma_init_struct.memory0_addr = (uint32_t)priv->buffer;
  dma_init_struct.number = buflen ;
  dma_init_struct.periph_addr = GD32_SDIO_FIFO;
  dma_init_struct.direction = DMA_MEMORY_TO_PERIPH;
  dma_init_struct.periph_memory_width = (uint8_t)DMA_WIDTH_8BITS_SELECT;
  dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  dma_init_struct.priority = (uint8_t)DMA_PRIO_ULTRA_HIGHSELECT;
  dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_DISABLE;

  gd32_dma_setup(priv->dma, &dma_init_struct, 1);
  gd32_sample(priv, SAMPLENDX_BEFORE_ENABLE);

  /* Start the DMA */

  regval = getreg32(GD32_SDIO_DATACTL);
  regval |= (1 << 3);
  putreg32(regval, GD32_SDIO_DATACTL);
  gd32_dma_start(priv->dma, gd32_dmacallback, priv, SDIO_DMA_INTEN);
  gd32_sample(priv, SAMPLENDX_AFTER_SETUP);

  /* Enable TX interrupts */

  gd32_config_xfrints(priv, SDIO_DMASEND_MASK);

  return OK;
}
#endif

/****************************************************************************
 * Name: gd32_callback
 *
 * Description:
 *   Perform callback.
 *
 * Input Parameters:
 *   arg - arg to be passed to the function.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function does not execute in the context of an interrupt handler.
 *   It may be invoked on any user thread or scheduled on the work thread
 *   from an interrupt handler.
 *
 ****************************************************************************/

static void gd32_callback(void *arg)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)arg;

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
 * Name: gd32_default
 *
 * Description:
 *   Restore SDIO registers to their default, reset values
 *
 ****************************************************************************/

static void gd32_default(void)
{
  putreg32(SDIO_PWRCTL_RESET,  GD32_SDIO_PWRCTL);
  putreg32(SDIO_CLKCTL_RESET,  GD32_SDIO_CLKCTL);
  putreg32(SDIO_CMDAGMT_RESET, GD32_SDIO_CMDAGMT);
  putreg32(SDIO_CMDCTL_RESET,  GD32_SDIO_CMDCTL);
  putreg32(SDIO_DATATO_RESET,  GD32_SDIO_DATATO);
  putreg32(SDIO_DATALEN_RESET, GD32_SDIO_DATALEN);
  putreg32(SDIO_DATACTL_RESET, GD32_SDIO_DATACTL);
  putreg32(SDIO_INTC_RESET,    GD32_SDIO_INTC);
  putreg32(SDIO_INTEN_RESET,   GD32_SDIO_INTEN);
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

struct sdio_dev_s *sdio_initialize(int slotno)
{
  /* There is only one slot */

  struct gd32_dev_s *priv = &g_sdiodev;

  /* Allocate a DMA channel */

#ifdef CONFIG_GD32F4_SDIO_DMA
  priv->dma = gd32_dma_channel_alloc(SDIO_DMACHAN);
  DEBUGASSERT(priv->dma);
#endif

  /* configure the GPIO, deinitialize the SDIO */

  gd32_sdio_clock_set(ENABLE);

  gd32_gpio_config(GPIO_SDIO_CMD_PIN);
  gd32_gpio_config(GPIO_SDIO_CLK_PIN);
  gd32_gpio_config(GPIO_SDIO_DAT0_PIN);
  gd32_gpio_config(GPIO_SDIO_DAT1_PIN);
  gd32_gpio_config(GPIO_SDIO_DAT2_PIN);
  gd32_gpio_config(GPIO_SDIO_DAT3_PIN);

  sdio_deinit();

  /* Reset the card and assure that it is in the initial, unconfigured
   * state.
   */

  gd32_reset(&priv->dev);

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

void sdio_mediachange(struct sdio_dev_s *dev, bool cardinslot)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;
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
      gd32_callback(priv);
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
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;
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
 *   dev       - An instance of the SDIO driver device state structure.
 *   func      - callback function.
 *   arg       - arg to be passed to the function.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_GD32_SDIO_CARD
void sdio_set_sdio_card_isr(struct sdio_dev_s *dev,
                            int (*func)(void *), void *arg)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev;

  priv->do_sdio_card = func;

  if (func != NULL)
    {
      priv->sdiointmask = SDIO_STAT_SDIOINT;
      priv->do_sdio_arg = arg;
    }
  else
    {
      priv->sdiointmask = 0;
    }

  putreg32(priv->xfrmask | priv->waitmask | priv->sdiointmask,
           GD32_SDIO_INTEN);
}
#endif

#endif /* CONFIG_GD32_SDIO */
