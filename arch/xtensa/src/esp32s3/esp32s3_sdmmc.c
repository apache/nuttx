/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_sdmmc.c
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
#if defined(CONFIG_ESP32S3_SDMMC_DMA) && defined(CONFIG_ESP32S3_SPIRAM)
#include <nuttx/kmalloc.h>
#endif

#include "xtensa.h"
#include "esp32s3_gpio.h"
#include "esp32s3_irq.h"
#include "hardware/esp32s3_sdmmc.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "hardware/esp32s3_soc.h"

#ifdef CONFIG_ESP32S3_SDMMC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MCI_DMADES0_OWN         (1UL << 31)
#define MCI_DMADES0_CH          (1 << 4)
#define MCI_DMADES0_FS          (1 << 3)
#define MCI_DMADES0_LD          (1 << 2)
#define MCI_DMADES0_DIC         (1 << 1)
#define MCI_DMADES1_MAXTR       4096
#define MCI_DMADES1_BS1(x)      (x)

#define GPIO_MATRIX_CONST_ONE_INPUT   (0x38)
#define GPIO_MATRIX_CONST_ZERO_INPUT  (0x3C)

/* Configuration ************************************************************/

/* Required system configuration options:
 *
 *   CONFIG_ARCH_DMA - Enable architecture-specific DMA subsystem
 *     initialization.  Required if CONFIG_ESP32S3_SDMMC_DMA is enabled.
 *   CONFIG_ESP32S3_DMA2 - Enable ESP32S3 DMA2 support.  Required if
 *     CONFIG_ESP32S3_SDMMC_DMA is enabled
 *   CONFIG_SCHED_WORKQUEUE -- Callback support requires work queue support.
 *
 * Driver-specific configuration options:
 *
 *   CONFIG_SDIO_MUXBUS - Setting this configuration enables some locking
 *     APIs to manage concurrent accesses on the SDIO bus.  This is not
 *     needed for the simple case of a single SD card, for example.
 *   CONFIG_ESP32S3_SDMMC_DMA - Enable SDIO.  This is a marginally optional.
 *     For most usages, SDIO will cause data overruns if used without DMA.
 *     NOTE the above system DMA configuration options.
 *   CONFIG_SDIO_WIDTH_D1_ONLY - This may be selected to force the
 *     driver operate with only a single data line (the default is to use
 *     all 4 SD data lines).
 *   CONFIG_SDIO_XFRDEBUG - Enables some very low-level debug output
 *     This also requires CONFIG_DEBUG_FS and CONFIG_DEBUG_INFO
 */

/* Timing : 100mS short timeout, 2 seconds for long one */

#define SDCARD_CMDTIMEOUT       MSEC2TICK(100)
#define SDCARD_LONGTIMEOUT      MSEC2TICK(2000)

/* FIFO size in bytes */

#define ESP32S3_TXFIFO_SIZE       (ESP32S3_TXFIFO_DEPTH | ESP32S3_TXFIFO_WIDTH)
#define ESP32S3_RXFIFO_SIZE       (ESP32S3_RXFIFO_DEPTH | ESP32S3_RXFIFO_WIDTH)

/* Number of DMA Descriptors */

#define ESP32S3_MULTIBLOCK_LIMIT  128
#define NUM_DMA_DESCRIPTORS       (1 + (ESP32S3_MULTIBLOCK_LIMIT * 512 / MCI_DMADES1_MAXTR))

#if (CONFIG_MMCSD_MULTIBLOCK_LIMIT == 0 || \
     CONFIG_MMCSD_MULTIBLOCK_LIMIT > ESP32S3_MULTIBLOCK_LIMIT)
#error "CONFIG_MMCSD_MULTIBLOCK_LIMIT is too big"
#endif

/* Data transfer interrupt mask bits */

#define SDCARD_RECV_MASK        (SDMMC_INT_DTO  | SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                                 SDMMC_INT_EBE  | SDMMC_INT_RXDR | SDMMC_INT_SBE)
#define SDCARD_SEND_MASK        (SDMMC_INT_DTO  | SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                                 SDMMC_INT_EBE  | SDMMC_INT_TXDR | SDMMC_INT_SBE)

#define SDCARD_DMARECV_MASK     (SDMMC_INT_DTO  | SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                                 SDMMC_INT_SBE  | SDMMC_INT_EBE)
#define SDCARD_DMASEND_MASK     (SDMMC_INT_DTO  | SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                                 SDMMC_INT_EBE)

#define SDCARD_DMAERROR_MASK    (SDMMC_IDINTEN_FBE | SDMMC_IDINTEN_DU | \
                                 SDMMC_IDINTEN_AIS)

#define SDCARD_TRANSFER_ALL     (SDMMC_INT_DTO  | SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                                 SDMMC_INT_EBE  | SDMMC_INT_TXDR | SDMMC_INT_RXDR | \
                                 SDMMC_INT_SBE)

/* Event waiting interrupt mask bits */

#define SDCARD_INT_RESPERR      (SDMMC_INT_RE   | SDMMC_INT_RCRC | SDMMC_INT_RTO)

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
#  define SDCARD_INT_CDET        SDMMC_INT_CDET
#else
#  define SDCARD_INT_CDET        0
#endif

#define SDCARD_CMDDONE_STA      (SDMMC_INT_CDONE)

#define SDCARD_CMDDONE_MASK     (SDMMC_INT_CDONE)
#define SDCARD_RESPDONE_MASK    (SDMMC_INT_CDONE | SDCARD_INT_RESPERR)
#define SDCARD_XFRDONE_MASK     (0)  /* Handled by transfer masks */

#define SDCARD_CMDDONE_CLEAR    (SDMMC_INT_CDONE)
#define SDCARD_RESPDONE_CLEAR   (SDMMC_INT_CDONE | SDCARD_INT_RESPERR)

#define SDCARD_XFRDONE_CLEAR    (SDCARD_TRANSFER_ALL)

#define SDCARD_WAITALL_CLEAR    (SDCARD_CMDDONE_CLEAR | SDCARD_RESPDONE_CLEAR | \
                                 SDCARD_XFRDONE_CLEAR)

/* Let's wait until we have both SD card transfer complete and DMA
 * complete.
 */

#define SDCARD_XFRDONE_FLAG     (1)
#define SDCARD_DMADONE_FLAG     (2)
#define SDCARD_ALLDONE          (3)

#define BOARD_SDMMC_FREQUENCY   (160 * 1000 * 1000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sdmmc_dma_s
{
  volatile uint32_t des0;        /* Control and status */
  volatile uint32_t des1;        /* Buffer size(s) */
  volatile uint32_t des2;        /* Buffer address pointer 1 */
  volatile uint32_t des3;        /* Buffer address pointer 2 */
};

/**
 * This structure lists pin numbers (if SOC_SDMMC_USE_IOMUX is set)
 * or GPIO Matrix signal numbers (if SOC_SDMMC_USE_GPIO_MATRIX is set)
 * for the SD bus signals. Field names match SD bus signal names.
 */

typedef struct
{
    uint8_t clk;
    uint8_t cmd;
    uint8_t d0;
    uint8_t d1;
    uint8_t d2;
    uint8_t d3;
    uint8_t d4;
    uint8_t d5;
    uint8_t d6;
    uint8_t d7;
} sdmmc_slot_io_info_t;

/**
 * Common SDMMC slot info,
 * doesn't depend on SOC_SDMMC_USE_{IOMUX,GPIO_MATRIX}
 */

typedef struct
{
    uint8_t card_detect;    /* !< Card detect signal in GPIO Matrix */
    uint8_t write_protect;  /* !< Write protect signal in GPIO Matrix */
    uint8_t card_int;       /* !< Card interrupt signal in GPIO Matrix */
} sdmmc_slot_info_t;

/* This structure defines the state of the ESP32S3 SDIO interface */

struct esp32s3_dev_s
{
  struct sdio_dev_s  dev;             /* Standard, base SDIO interface */

  /* ESP32S3-specific extensions */

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
#ifdef CONFIG_ESP32S3_SDMMC_DMA
  uint32_t           dmamask;         /* Interrupt enables for DMA transfer */
  volatile struct sdmmc_dma_s dma_desc[NUM_DMA_DESCRIPTORS];
#ifdef CONFIG_ESP32S3_SPIRAM
  uint8_t           *dma_buf;
  size_t            dma_buf_size;
#endif
#endif
  bool               wrdir;           /* True: Writing False: Reading */

  /* DMA data transfer support */

  int                slot;

  const sdmmc_slot_io_info_t *sdio_pins;
  const sdmmc_slot_info_t *slot_info;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* #define CONFIG_ESP32S3_SDMMC_REGDEBUG */

#ifdef CONFIG_ESP32S3_SDMMC_REGDEBUG
static uint32_t __esp32s3_getreg(const char *func, uint32_t addr);
static void __esp32s3_putreg(const char *func, uint32_t val, uint32_t addr);

#  define esp32s3_getreg(addr)     __esp32s3_getreg(__func__, addr)
#  define esp32s3_putreg(val,addr) __esp32s3_putreg(__func__, val,addr)
#else
#  define esp32s3_getreg(addr)     getreg32(addr)
#  define esp32s3_putreg(val,addr) putreg32(val,addr)
#endif

/* Low-level helpers ********************************************************/

/* DMA Helpers **************************************************************/

/* Data Transfer Helpers ****************************************************/

static void esp32s3_eventtimeout(wdparm_t arg);
static void esp32s3_endwait(struct esp32s3_dev_s *priv,
                            sdio_eventset_t wkupevent);
static void esp32s3_endtransfer(struct esp32s3_dev_s *priv,
                                sdio_eventset_t wkupevent);

/* Interrupt Handling *******************************************************/

static int  esp32s3_interrupt(int irq, void *context, void *arg);

/* SDIO interface methods ***************************************************/

/* Mutual exclusion */

#ifdef CONFIG_SDIO_MUXBUS
static int esp32s3_lock(struct sdio_dev_s *dev, bool lock);
#endif

/* Initialization/setup */

static void esp32s3_reset(struct sdio_dev_s *dev);
static sdio_capset_t esp32s3_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t esp32s3_status(struct sdio_dev_s *dev);
static void esp32s3_widebus(struct sdio_dev_s *dev, bool enable);
static void esp32s3_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate);
static int  esp32s3_attach(struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int  esp32s3_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                            uint32_t arg);
#ifdef CONFIG_SDIO_BLOCKSETUP
static void esp32s3_blocksetup(struct sdio_dev_s *dev, unsigned int blocklen,
                               unsigned int nblocks);
#endif
static int  esp32s3_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                              size_t nbytes);
static int  esp32s3_sendsetup(struct sdio_dev_s *dev, const uint8_t *buffer,
                              size_t nbytes);
static int  esp32s3_cancel(struct sdio_dev_s *dev);

static int  esp32s3_waitresponse(struct sdio_dev_s *dev, uint32_t cmd);
static int  esp32s3_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                                 uint32_t *rshort);
static int  esp32s3_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t rlong[4]);
static int  esp32s3_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                              uint32_t *rshort);

/* EVENT handler */

static void esp32s3_waitenable(struct sdio_dev_s *dev,
                               sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t esp32s3_eventwait(struct sdio_dev_s *dev);
static void esp32s3_callbackenable(struct sdio_dev_s *dev,
                                   sdio_eventset_t eventset);
static int  esp32s3_registercallback(struct sdio_dev_s *dev,
                                     worker_t callback, void *arg);

/* DMA */

#ifdef CONFIG_ESP32S3_SDMMC_DMA
static int  esp32s3_dmarecvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                                 size_t buflen);
static int  esp32s3_dmasendsetup(struct sdio_dev_s *dev,
                                 const uint8_t *buffer, size_t buflen);
#endif

/* Initialization/uninitialization/reset ************************************/

static void esp32s3_callback(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct esp32s3_dev_s g_sdiodev =
{
  .dev =
  {
#ifdef CONFIG_SDIO_MUXBUS
    .lock             = esp32s3_lock,
#endif
    .reset            = esp32s3_reset,
    .capabilities     = esp32s3_capabilities,
    .status           = esp32s3_status,
    .widebus          = esp32s3_widebus,
    .clock            = esp32s3_clock,
    .attach           = esp32s3_attach,
    .sendcmd          = esp32s3_sendcmd,
#ifdef CONFIG_SDIO_BLOCKSETUP
    .blocksetup       = esp32s3_blocksetup,
#endif
    .recvsetup        = esp32s3_recvsetup,
    .sendsetup        = esp32s3_sendsetup,
    .cancel           = esp32s3_cancel,
    .waitresponse     = esp32s3_waitresponse,
    .recv_r1          = esp32s3_recvshortcrc,
    .recv_r2          = esp32s3_recvlong,
    .recv_r3          = esp32s3_recvshort,
    .recv_r4          = esp32s3_recvshort,
    .recv_r5          = esp32s3_recvshortcrc,
    .recv_r6          = esp32s3_recvshortcrc,
    .recv_r7          = esp32s3_recvshort,
    .waitenable       = esp32s3_waitenable,
    .eventwait        = esp32s3_eventwait,
    .callbackenable   = esp32s3_callbackenable,
    .registercallback = esp32s3_registercallback,
#ifdef CONFIG_ESP32S3_SDMMC_DMA
    .dmarecvsetup     = esp32s3_dmarecvsetup,
    .dmasendsetup     = esp32s3_dmasendsetup,
#endif
  },
  .waitsem = SEM_INITIALIZER(0),
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_getreg
 *
 * Description:
 *   This function may to used to intercept an monitor all register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   addr - The register address to read
 *
 * Returned Value:
 *   The value read from the register
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SDMMC_REGDEBUG
static uint32_t __esp32s3_getreg(const char *func, uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval   = 0;
  static uint32_t count    = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              mcerr("%s: ...\n", func);
            }

          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          mcerr("%s: [repeats %d more times]\n", func, count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  mcerr("%s: %08x->%08x\n", func, addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: esp32s3_putreg
 *
 * Description:
 *   This function may to used to intercept an monitor all register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   val - The value to write to the register
 *   addr - The register address to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SDMMC_REGDEBUG
static void __esp32s3_putreg(const char *func, uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  mcerr("%s: %08x<-%08x\n", func, addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: esp32s3_ciu_sendcmd
 *
 * Description:
 *   Function to send command to Card interface unit (CIU)
 *
 * Input Parameters:
 *   cmd - The command to be executed
 *   arg - The argument to use with the command.
 *
 * Returned Value:
 *   Returns zero on success.  One will be returned on a timeout.
 *
 ****************************************************************************/

static int esp32s3_ciu_sendcmd(uint32_t cmd, uint32_t arg)
{
  clock_t watchtime;

  watchtime = clock_systime_ticks();

  while ((esp32s3_getreg(ESP32S3_SDMMC_CMD) & SDMMC_CMD_STARTCMD) != 0)
    {
      if (watchtime - clock_systime_ticks() > SDCARD_CMDTIMEOUT)
        {
          mcerr("TMO Timed out (%08X)\n",
                esp32s3_getreg(ESP32S3_SDMMC_CMD));
          return 1;
        }
    }

  /* Set command arg reg */

  cmd |= SDMMC_CMD_STARTCMD | SDMMC_CMD_USE_HOLE;

  esp32s3_putreg(arg, ESP32S3_SDMMC_CMDARG);
  esp32s3_putreg(cmd, ESP32S3_SDMMC_CMD);

  mcinfo("cmd=0x%x arg=0x%x\n", cmd, arg);

  return 0;
}

static void configure_pin(uint8_t gpio_pin, uint8_t sdio_pin,
                          gpio_pinattr_t attr)
{
  esp32s3_configgpio(gpio_pin, attr);

  if (attr & INPUT)
    {
      esp32s3_gpio_matrix_in(gpio_pin, sdio_pin, false);
    }

  if (attr & OUTPUT)
    {
      esp32s3_gpio_matrix_out(gpio_pin, sdio_pin, false, false);
    }
}

/****************************************************************************
 * Name: esp32s3_enable_ints
 *
 * Description:
 *   Enable/disable SD card interrupts per functional settings.
 *
 * Input Parameters:
 *   priv - A reference to the SD card device state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32s3_enable_ints(struct esp32s3_dev_s *priv)
{
  uint32_t regval;

#ifdef CONFIG_ESP32S3_SDMMC_DMA
  mcinfo("waitmask=%04lx xfrmask=%04lx dmamask=%04lx RINTSTS=%08lx\n",
         (unsigned long)priv->waitmask, (unsigned long)priv->xfrmask,
         (unsigned long)priv->dmamask,
         (unsigned long)esp32s3_getreg(ESP32S3_SDMMC_RINTSTS));
#else
  mcinfo("waitmask=%04lx xfrmask=%04lx RINTSTS=%08lx\n",
         (unsigned long)priv->waitmask, (unsigned long)priv->xfrmask,
         (unsigned long)esp32s3_getreg(ESP32S3_SDMMC_RINTSTS));
#endif

  /* Enable SDMMC interrupts */

  regval = priv->xfrmask | priv->waitmask | SDCARD_INT_CDET;
  esp32s3_putreg(regval, ESP32S3_SDMMC_INTMASK);
}

/****************************************************************************
 * Name: esp32s3_disable_allints
 *
 * Description:
 *   Disable all SD card interrupts.
 *
 * Input Parameters:
 *   priv - A reference to the SD card device state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32s3_disable_allints(struct esp32s3_dev_s *priv)
{
#ifdef CONFIG_ESP32S3_SDMMC_DMA
  /* Disable DMA-related interrupts */

  priv->dmamask = 0;
#endif

  /* Disable all SDMMC interrupts (except card detect) */

  esp32s3_putreg(SDCARD_INT_CDET, ESP32S3_SDMMC_INTMASK);
  priv->waitmask = 0;
  priv->xfrmask  = 0;
}

/****************************************************************************
 * Name: esp32s3_config_waitints
 *
 * Description:
 *   Enable/disable SD card interrupts needed to support the wait function
 *
 * Input Parameters:
 *   priv       - A reference to the SD card device state structure
 *   waitmask   - The set of bits in the SD card INTMASK register to set
 *   waitevents - Waited for events
 *   wkupevent  - Wake-up events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32s3_config_waitints(struct esp32s3_dev_s *priv,
                                    uint32_t waitmask,
                                    sdio_eventset_t waitevents,
                                    sdio_eventset_t wkupevent)
{
  irqstate_t flags;

  mcinfo("waitevents=%04x wkupevent=%04x\n",
         (unsigned)waitevents, (unsigned)wkupevent);

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags            = enter_critical_section();
  priv->waitevents = waitevents;
  priv->wkupevent  = wkupevent;
  priv->waitmask   = waitmask;

  esp32s3_enable_ints(priv);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32s3_config_xfrints
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

static void esp32s3_config_xfrints(struct esp32s3_dev_s *priv,
                                   uint32_t xfrmask)
{
  irqstate_t flags;
  flags = enter_critical_section();

  priv->xfrmask = xfrmask;
  esp32s3_enable_ints(priv);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32s3_config_dmaints
 *
 * Description:
 *   Enable DMA transfer interrupts
 *
 * Input Parameters:
 *   priv    - A reference to the SD card device state structure
 *   dmamask - The set of bits in the SD card MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SDMMC_DMA
static void esp32s3_config_dmaints(struct esp32s3_dev_s *priv,
                                   uint32_t xfrmask, uint32_t dmamask)
{
  irqstate_t flags;
  flags = enter_critical_section();

  priv->xfrmask = xfrmask;
  priv->dmamask = dmamask;
  esp32s3_enable_ints(priv);

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: esp32s3_eventtimeout
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

static void esp32s3_eventtimeout(wdparm_t arg)
{
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)arg;

  /* There is always race conditions with timer expirations. */

  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0 ||
              priv->wkupevent != 0);

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. wake up any waiting threads */

      esp32s3_endwait(priv, SDIOWAIT_TIMEOUT);
      mcerr("Timeout: remaining: %d\n", priv->remaining);
    }
}

/****************************************************************************
 * Name: esp32s3_endwait
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

static void esp32s3_endwait(struct esp32s3_dev_s *priv,
                            sdio_eventset_t wkupevent)
{
  mcinfo("wkupevent=%04x\n", (unsigned)wkupevent);

  /* Cancel the watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Disable event-related interrupts */

  esp32s3_config_waitints(priv, 0, 0, wkupevent);

  /* Wake up the waiting thread */

  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: esp32s3_endtransfer
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

static void esp32s3_endtransfer(struct esp32s3_dev_s *priv,
                                sdio_eventset_t wkupevent)
{
  mcinfo("wkupevent=%04x\n", (unsigned)wkupevent);

  /* Disable all transfer related interrupts */

  esp32s3_config_xfrints(priv, 0);

  /* Clearing pending interrupt status on all transfer related interrupts */

  esp32s3_putreg(priv->waitmask, ESP32S3_SDMMC_RINTSTS);

  /* Mark the transfer finished */

  priv->remaining = 0;

  /* Is a thread wait for these data transfer complete events? */

  if ((priv->waitevents & wkupevent) != 0)
    {
      /* Yes.. wake up any waiting threads */

      esp32s3_endwait(priv, wkupevent);
    }
}

/****************************************************************************
 * Name: esp32s3_interrupt
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

static int esp32s3_interrupt(int irq, void *context, void *arg)
{
  struct esp32s3_dev_s *priv = arg;
  uint32_t enabled;
  uint32_t pending;

  /* Loop while there are pending interrupts.  Check the SD card status
   * register.  Mask out all bits that don't correspond to enabled
   * interrupts.  (This depends on the fact that bits are ordered
   * the same in both the STA and MASK register).  If there are non-zero
   * bits remaining, then we have work to do here.
   */

  while ((enabled = esp32s3_getreg(ESP32S3_SDMMC_MINTSTS)) != 0)
    {
      /* Clear pending status */

      esp32s3_putreg(enabled, ESP32S3_SDMMC_RINTSTS);

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
      /* Handle in card detection events ************************************/

      if ((enabled & SDMMC_INT_CDET) != 0)
        {
          sdio_statset_t cdstatus;

          /* Update card status */

          cdstatus = priv->cdstatus;
          if ((esp32s3_getreg(ESP32S3_SDMMC_CDETECT) &
              SDMMC_CDETECT_NOTPRESENT(priv->slot)) == 0)
            {
              priv->cdstatus |= SDIO_STATUS_PRESENT;

#ifdef CONFIG_MMCSD_HAVE_WRITEPROTECT
              if ((esp32s3_getreg(ESP32S3_SDMMC_WRTPRT) &
                  SDMMC_WRTPRT_PROTECTED(priv->slot)) != 0)
                {
                  priv->cdstatus |= SDIO_STATUS_WRPROTECTED;
                }
              else
#endif
                {
                  priv->cdstatus &= ~SDIO_STATUS_WRPROTECTED;
                }
            }
          else
            {
              priv->cdstatus &=
                ~(SDIO_STATUS_PRESENT | SDIO_STATUS_WRPROTECTED);
            }

          mcinfo("cdstatus OLD: %02x NEW: %02x\n", cdstatus, priv->cdstatus);

          /* Perform any requested callback if the status has changed */

          if (cdstatus != priv->cdstatus)
            {
              esp32s3_callback(priv);
            }
        }
#endif

      /* Handle data transfer events ****************************************/

      pending = enabled & priv->xfrmask;
      if (pending != 0)
        {
          /* Handle data request events */

          if ((pending & SDMMC_INT_TXDR) != 0)
            {
              uint32_t status;

              /* Transfer data to the TX FIFO */

              DEBUGASSERT(priv->wrdir);

              for (status = esp32s3_getreg(ESP32S3_SDMMC_STATUS);
                   (status & SDMMC_STATUS_FIFOFULL) == 0 &&
                   priv->remaining > 0;
                   status = esp32s3_getreg(ESP32S3_SDMMC_STATUS))
                {
                  esp32s3_putreg(*priv->buffer, ESP32S3_SDMMC_DATA);
                  priv->buffer++;
                  priv->remaining -= 4;
                }
            }
          else if ((pending & SDMMC_INT_RXDR) != 0)
            {
              uint32_t status;

              /* Transfer data from the RX FIFO */

              DEBUGASSERT(!priv->wrdir);

              for (status = esp32s3_getreg(ESP32S3_SDMMC_STATUS);
                   (status & SDMMC_STATUS_FIFOEMPTY) == 0 &&
                   priv->remaining > 0;
                   status = esp32s3_getreg(ESP32S3_SDMMC_STATUS))
                {
                  *priv->buffer = esp32s3_getreg(ESP32S3_SDMMC_DATA);
                  priv->buffer++;
                  priv->remaining -= 4;
                }
            }

          /* Check for transfer errors */

          /* Handle data block send/receive CRC failure */

          if ((pending & SDMMC_INT_DCRC) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Data CRC failure, pending=%08x remaining: %d\n",
                    pending, priv->remaining);

              esp32s3_endtransfer(priv,
                                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle data timeout error */

          else if ((pending & SDMMC_INT_DRTO) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Data timeout, pending=%08x remaining: %d\n",
                    pending, priv->remaining);

              esp32s3_endtransfer(priv,
                                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT);
            }

          /* Handle RX FIFO overrun error */

          else if ((pending & SDMMC_INT_FRUN) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: RX FIFO overrun, pending=%08x remaining: %d\n",
                    pending, priv->remaining);

              esp32s3_endtransfer(priv,
                                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle TX FIFO underrun error */

          else if ((pending & SDMMC_INT_FRUN) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: TX FIFO underrun, pending=%08x remaining: %d\n",
                    pending, priv->remaining);

              esp32s3_endtransfer(priv,
                                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle start bit error */

          else if ((pending & SDMMC_INT_SBE) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Start bit, pending=%08x remaining: %d\n",
                    pending, priv->remaining);

              esp32s3_endtransfer(priv,
                                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle data end events.  Note that RXDR may accompany DTO, DTO
           * will be set on received while there is still data in the FIFO.
           * So for the case of receiving, we don't actually even enable the
           * DTO interrupt.
           */

          else if ((pending & SDMMC_INT_DTO) != 0)
            {
              /* Finish the transfer */

              esp32s3_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
            }
        }

      /* Handle wait events *************************************************/

      pending = enabled & priv->waitmask;
      if (pending != 0)
        {
          /* Is this a response error event? */

          if ((pending & SDCARD_INT_RESPERR) != 0)
            {
              /* If response errors are enabled, then we must certainly be
               * waiting for a response.
               */

              DEBUGASSERT((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0);

              /* Wake the thread up */

              mcerr("ERROR: Response error, pending=%08x\n", pending);
              esp32s3_endwait(priv, SDIOWAIT_RESPONSEDONE | SDIOWAIT_ERROR);
            }

          /* Is this a command (plus response) completion event? */

          else if ((pending & SDMMC_INT_CDONE) != 0)
            {
              /* Yes.. Is their a thread waiting for response done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  esp32s3_endwait(priv, SDIOWAIT_RESPONSEDONE);
                }

              /* NO.. Is their a thread waiting for command done? */

              else if ((priv->waitevents & SDIOWAIT_CMDDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  esp32s3_endwait(priv, SDIOWAIT_CMDDONE);
                }
            }
        }
    }

#ifdef CONFIG_ESP32S3_SDMMC_DMA
  /* DMA error events *******************************************************/

  pending = esp32s3_getreg(ESP32S3_SDMMC_IDSTS);
  if ((pending & priv->dmamask) != 0)
    {
      mcerr("ERROR: IDTS=%08lx\n", (unsigned long)pending);

      /* Clear the pending interrupts */

      esp32s3_putreg(pending, ESP32S3_SDMMC_IDSTS);

      /* Abort the transfer */

      esp32s3_endtransfer(priv, SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: esp32s3_lock
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
static int esp32s3_lock(struct sdio_dev_s *dev, bool lock)
{
  /* Single SDIO instance so there is only one possibility.  The multiplex
   * bus is part of board support package.
   */

  /* FIXME: Implement the below function to support bus share:
   *
   * esp32s3_muxbus_sdio_lock(lock);
   */

  return OK;
}
#endif

/****************************************************************************
 * Name: esp32s3_reset
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

static void esp32s3_reset(struct sdio_dev_s *dev)
{
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)dev;
  irqstate_t flags;
  uint32_t regval;

  mcinfo("Resetting...\n");

  flags = enter_critical_section();

  /* Reset all blocks */

  esp32s3_putreg(SDMMC_CTRL_CNTLRRESET | SDMMC_CTRL_FIFORESET |
                 SDMMC_CTRL_DMARESET, ESP32S3_SDMMC_CTRL);

  while ((esp32s3_getreg(ESP32S3_SDMMC_CTRL) &
         (SDMMC_CTRL_CNTLRRESET | SDMMC_CTRL_FIFORESET |
          SDMMC_CTRL_DMARESET)) != 0)
    {
    }

  /* Select clock source and init phases */

  regval = esp32s3_getreg(ESP32S3_SDMMC_CLOCK);
  regval &= ~(SDMMC_CLOCK_CLK_SEL_MASK | SDMMC_CLOCK_PHASE_MASK);
  regval |= SDMMC_CLOCK_CLK_SEL_PLL160M;
  regval |= 1 << SDMMC_CLOCK_PHASE_DOUT_SHIFT;
  esp32s3_putreg(regval, ESP32S3_SDMMC_CLOCK);

  /* Select clock divider
   * Slot N selects clock divider N.
   */

  regval = esp32s3_getreg(ESP32S3_SDMMC_CLKSRC);
  regval &= ~SDMMC_CLKSRC_MASK(priv->slot);
  regval |= SDMMC_CLKSRC_CLKDIV(priv->slot, priv->slot);
  esp32s3_putreg(regval, ESP32S3_SDMMC_CLKSRC);

#ifdef CONFIG_ESP32S3_SDMMC_DMA
  esp32s3_putreg((uint32_t)&priv->dma_desc[0], ESP32S3_SDMMC_DBADDR);
#endif

  /* Reset data */

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitmask   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */

  wd_cancel(&priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->buffer     = 0;      /* Address of current R/W buffer */
  priv->remaining  = 0;      /* Number of bytes remaining in the transfer */
  priv->xfrmask    = 0;      /* Interrupt enables for data transfer */
#ifdef CONFIG_ESP32S3_SDMMC_DMA
  priv->dmamask    = 0;      /* Interrupt enables for DMA transfer */
#endif

  /* DMA data transfer support */

  priv->cdstatus   = 0;      /* Card status is unknown */

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32s3_capabilities
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

static sdio_capset_t esp32s3_capabilities(struct sdio_dev_s *dev)
{
  sdio_capset_t caps = 0;

  caps |= SDIO_CAPS_DMABEFOREWRITE;
  caps |= SDIO_CAPS_MMC_HS_MODE;

#ifdef CONFIG_SDIO_WIDTH_D1_ONLY
  caps |= SDIO_CAPS_1BIT_ONLY;
#endif
#ifdef CONFIG_ESP32S3_SDMMC_DMA
  caps |= SDIO_CAPS_DMASUPPORTED;
#endif

  return caps;
}

/****************************************************************************
 * Name: esp32s3_status
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see esp32s3_status_* defines)
 *
 ****************************************************************************/

static sdio_statset_t esp32s3_status(struct sdio_dev_s *dev)
{
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)dev;

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
  if ((esp32s3_getreg(ESP32S3_SDMMC_CDETECT) &
       SDMMC_CDETECT_NOTPRESENT(priv->slot)) == 0)
    {
      priv->cdstatus |= SDIO_STATUS_PRESENT;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_PRESENT;
    }
#endif

  mcinfo("cdstatus=%02x\n", priv->cdstatus);

  return priv->cdstatus;
}

/****************************************************************************
 * Name: esp32s3_widebus
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

static void esp32s3_widebus(struct sdio_dev_s *dev, bool wide)
{
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)dev;
  uint32_t regval;

  regval = esp32s3_getreg(ESP32S3_SDMMC_CTYPE);
  regval &= ~(SDMMC_CTYPE_WIDTH4_MASK(priv->slot));
  regval &= ~(SDMMC_CTYPE_WIDTH8_MASK(priv->slot));

#ifndef CONFIG_SDIO_WIDTH_D1_ONLY
  if (wide)
    {
      regval |= SDMMC_CTYPE_WIDTH4_MASK(priv->slot);

      configure_pin(CONFIG_ESP32S3_SDMMC_D1, priv->sdio_pins->d1,
                    INPUT | OUTPUT | PULLUP);
      configure_pin(CONFIG_ESP32S3_SDMMC_D2, priv->sdio_pins->d2,
                    INPUT | OUTPUT | PULLUP);
      configure_pin(CONFIG_ESP32S3_SDMMC_D3, priv->sdio_pins->d3,
                    INPUT | OUTPUT | PULLUP);
    }
#endif

  esp32s3_putreg(regval, ESP32S3_SDMMC_CTYPE);
}

static int sdmmc_host_clock_update_command(struct esp32s3_dev_s *priv)
{
  /* Clock update command
   * not a real command; just updates CIU registers
   */

  uint32_t cmd = SDMMC_CMD_UPDCLOCK | SDMMC_CMD_WAITPREV |
                 SDMMC_CMD_CARD_NUMBER(priv->slot);
  uint32_t regval;
  bool repeat = true;
  int timeout_ms = 100;
  int ret;

  while (repeat)
    {
      ret = esp32s3_ciu_sendcmd(cmd, 0);
      if (ret)
        {
          return ret;
        }

      while (timeout_ms)
        {
          regval = esp32s3_getreg(ESP32S3_SDMMC_RINTSTS);
          if (regval & SDMMC_INT_HLE)
            {
              esp32s3_putreg(SDMMC_INT_HLE, ESP32S3_SDMMC_RINTSTS);
              break;
            }

          if ((esp32s3_getreg(ESP32S3_SDMMC_CMD) & SDMMC_CMD_STARTCMD)
              == 0)
            {
              repeat = false;
              break;
            }

          timeout_ms--;
          up_mdelay(1);
        }
    }

  return timeout_ms > 0 ? OK : -ETIMEDOUT;
}

static void sdmmc_host_get_clk_dividers(uint32_t freq_khz, int *host_div,
                                        int *card_div)
{
  uint32_t clk_src_freq_hz = BOARD_SDMMC_FREQUENCY;

  /* Calculate new dividers */

  if (freq_khz >= 40 * 1000)
    {
      *host_div = 4;       /* 160 MHz / 4 = 40 MHz */
      *card_div = 0;
    }
  else if (freq_khz == 20 * 1000)
    {
      *host_div = 8;       /* 160 MHz / 8 = 20 MHz */
      *card_div = 0;
    }
  else if (freq_khz == 400)
    {
      *host_div = 10;      /* 160 MHz / 10 / (20 * 2) = 400 kHz */
      *card_div = 20;
    }
  else
    {
      /* for custom frequencies use maximum range of host divider (1-16),
       * find the closest <= div. combination
       * if exceeded, combine with the card divider to keep reasonable
       * precision (applies mainly to low frequencies)
       * effective frequency range: 400 kHz - 32 MHz (32.1 - 39.9 MHz cannot
       * be covered with given divider scheme)
       */

      *host_div = (clk_src_freq_hz) / (freq_khz * 1000);
      if (*host_div > 15)
        {
          *host_div = 2;
          *card_div = (clk_src_freq_hz / 2) / (2 * freq_khz * 1000);
          if (((clk_src_freq_hz / 2) % (2 * freq_khz * 1000)) > 0)
            {
              (*card_div)++;
            }
        }
      else if ((clk_src_freq_hz % (freq_khz * 1000)) > 0)
        {
          (*host_div)++;
        }
    }
}

static void sdmmc_host_set_clk_div(uint32_t slot, uint32_t host_div,
                                   uint32_t card_div)
{
  irqstate_t flags = enter_critical_section();

  /* Set frequency to 160MHz / div
   *
   * n: counter resets at div_factor_n.
   * l: negedge when counter equals div_factor_l.
   * h: posedge when counter equals div_factor_h.
   *
   * We set the duty cycle to 1/2
   */

  ASSERT(host_div > 1 && host_div <= 16);
  int l = host_div - 1;
  int h = host_div / 2 - 1;
  uint32_t regval;
  uint32_t divider;

  /* Get the divider which is selected in esp32s3_reset() */

  regval = esp32s3_getreg(ESP32S3_SDMMC_CLKSRC);
  divider = (regval & SDMMC_CLKSRC_MASK(slot)) >> SDMMC_CLKSRC_SHIFT(slot);

  /* Set card divider */

  regval = esp32s3_getreg(ESP32S3_SDMMC_CLKDIV);
  regval &= ~SDMMC_CLKDIV_MASK(divider);
  regval |= SDMMC_CLKDIV(divider, card_div);
  esp32s3_putreg(regval, ESP32S3_SDMMC_CLKDIV);

  /* Set host_div divider */

  regval = esp32s3_getreg(ESP32S3_SDMMC_CLOCK);
  regval &= ~SDMMC_CLOCK_DIV_FACTOR_MASK;
  regval |= (h << SDMMC_CLOCK_DIV_FACTOR_H_SHIFT)
            & SDMMC_CLOCK_DIV_FACTOR_H_MASK;
  regval |= (l << SDMMC_CLOCK_DIV_FACTOR_L_SHIFT)
            & SDMMC_CLOCK_DIV_FACTOR_L_MASK;
  regval |= (l << SDMMC_CLOCK_DIV_FACTOR_N_SHIFT)
            & SDMMC_CLOCK_DIV_FACTOR_N_MASK;
  esp32s3_putreg(regval, ESP32S3_SDMMC_CLOCK);

  leave_critical_section(flags);

  /* Wait for the clock to propagate */

  up_udelay(10);
}

/****************************************************************************
 * Name: esp32s3_clock
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

static void esp32s3_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  uint32_t freq_khz;
  uint32_t regval;
  bool clk_en = true;
  int host_div = 0;   /* clock divider of the host (SDMMC.clock) */
  int card_div = 0;   /* 1/2 of card clock divider (SDMMC.clkdiv) */
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)dev;

  switch (rate)
    {
      /* Disable clocking (with default ID mode divisor) */

      default:
      case CLOCK_SDIO_DISABLED:
        freq_khz = 400;
        clk_en = false;
        break;

      /* Enable in initial ID mode clocking (<400KHz) */

      case CLOCK_IDMODE:
        freq_khz = 400;
        break;

      /* Enable in MMC normal operation clocking */

      case CLOCK_MMC_TRANSFER:
        if (esp32s3_capabilities(dev) & SDIO_CAPS_MMC_HS_MODE)
          {
            freq_khz = 40 * 1000;
          }
        else
          {
            freq_khz = 20 * 1000;
          }
        break;

      /* SD normal operation clocking (wide 4-bit mode) */

      case CLOCK_SD_TRANSFER_4BIT:
#ifndef CONFIG_ESP32S3_SDMMC_WIDTH_D1_ONLY
        /* TODO: Use higher frequency */

        freq_khz = 20 * 1000;
        esp32s3_widebus(dev, true);
        break;
#endif

      /* SD normal operation clocking (narrow 1-bit mode) */

      case CLOCK_SD_TRANSFER_1BIT:

        /* TODO: Use higher frequency */

        freq_khz = 20 * 1000;
        esp32s3_widebus(dev, false);
        break;
    }

  /* Disable clock first */

  regval = esp32s3_getreg(ESP32S3_SDMMC_CLKENA);
  regval &= ~(SDMMC_CLKENA_ENABLE(priv->slot)
              | SDMMC_CLKENA_LOWPOWER(priv->slot));
  esp32s3_putreg(regval, ESP32S3_SDMMC_CLKENA);
  if (sdmmc_host_clock_update_command(priv) != OK)
  {
    mcerr("disabling clk failed\n");
    return;
  }

  /* Program card clock settings, send them to the CIU */

  sdmmc_host_get_clk_dividers(freq_khz, &host_div, &card_div);
  sdmmc_host_set_clk_div(priv->slot, host_div, card_div);
  if (sdmmc_host_clock_update_command(priv) != OK)
  {
    mcerr("setting clk div failed\n");
    return;
  }

  /* Re-enable clocks */

  if (clk_en)
    {
      regval = esp32s3_getreg(ESP32S3_SDMMC_CLKENA);
      regval |= SDMMC_CLKENA_ENABLE(priv->slot)
                | SDMMC_CLKENA_LOWPOWER(priv->slot);
      esp32s3_putreg(regval, ESP32S3_SDMMC_CLKENA);
      if (sdmmc_host_clock_update_command(priv) != OK)
        {
          mcerr("re-enabling clk failed\n");
          return;
        }
    }

  /* set data timeout to 100ms */

  if (freq_khz * 100 > (SDMMC_TMOUT_DATA_MASK >> SDMMC_TMOUT_DATA_SHIFT))
    {
      regval = SDMMC_TMOUT_DATA_MASK;
    }
  else
    {
      regval = freq_khz * 100 << SDMMC_TMOUT_DATA_SHIFT;
    }

  /* always set response timeout to highest value, it's small enough anyway */

  regval |= SDMMC_TMOUT_RESPONSE_MASK;
  esp32s3_putreg(regval, ESP32S3_SDMMC_TMOUT);
}

/****************************************************************************
 * Name: esp32s3_attach
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

static int esp32s3_attach(struct sdio_dev_s *dev)
{
  int ret;
  uint32_t regval;
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)dev;

  ret = esp32s3_setup_irq(this_cpu(), ESP32S3_PERIPH_SDIO_HOST,
                          1, ESP32S3_CPUINT_LEVEL);
  DEBUGASSERT(ret >= 0);

  /* Attach the SDIO interrupt handler */

  ret = irq_attach(ESP32S3_IRQ_SDIO_HOST, esp32s3_interrupt, dev);
  if (ret == OK)
    {
      /* Disable all interrupts at the SD card controller and clear static
       * interrupt flags
       */

      esp32s3_putreg(0, ESP32S3_SDMMC_INTMASK);
      esp32s3_putreg(SDMMC_INT_ALL(priv->slot), ESP32S3_SDMMC_RINTSTS);

      /* Enable Interrupts to happen when the INTMASK is activated */

      regval  = esp32s3_getreg(ESP32S3_SDMMC_CTRL);
      regval |= SDMMC_CTRL_INTENABLE;
      esp32s3_putreg(regval, ESP32S3_SDMMC_CTRL);

      /* Enable card detection interrupts */

      esp32s3_putreg(SDCARD_INT_CDET, ESP32S3_SDMMC_INTMASK);

      /* Enable SD card interrupts at the NVIC.  They can now be enabled at
       * the SD card controller as needed.
       */

      up_enable_irq(ESP32S3_IRQ_SDIO_HOST);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32s3_sendcmd
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

static int esp32s3_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t arg)
{
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)dev;
  uint32_t regval = 0;

  mcinfo("cmd=%04x arg=%04x\n", cmd, arg);

  if (cmd == MMCSD_CMD12)
    {
      regval |= SDMMC_CMD_STOPABORT;
    }
  else if (cmd == MMCSD_CMD0)
    {
      /* The CMD0 needs the SENDINIT CMD */

      regval |= SDMMC_CMD_SENDINIT;
    }
  else
    {
      regval |= SDMMC_CMD_WAITPREV;
    }

  /* Is this a Read/Write Transfer Command ? */

  if ((cmd & MMCSD_WRDATAXFR) == MMCSD_WRDATAXFR)
    {
      regval |= SDMMC_CMD_DATAXFREXPTD | SDMMC_CMD_WRITE;
    }
  else if ((cmd & MMCSD_RDDATAXFR) == MMCSD_RDDATAXFR)
    {
      regval |= SDMMC_CMD_DATAXFREXPTD;
    }

  /* Set WAITRESP bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      regval |= SDMMC_CMD_NORESPONSE;
      break;

    case MMCSD_R1B_RESPONSE:
      regval |= SDMMC_CMD_RESPCRC;
      regval |= SDMMC_CMD_WAITPREV;
      regval |= SDMMC_CMD_SHORTRESPONSE;
      break;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R4_RESPONSE:
      regval |= SDMMC_CMD_SHORTRESPONSE;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
    case MMCSD_R7_RESPONSE:
      regval |= SDMMC_CMD_RESPCRC;
      regval |= SDMMC_CMD_SHORTRESPONSE;
      break;

    case MMCSD_R2_RESPONSE:
      regval |= SDMMC_CMD_LONGRESPONSE;
      regval |= SDMMC_CMD_RESPCRC;
      break;
    }

  /* Set the command index */

  regval |= (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval |= SDMMC_CMD_CARD_NUMBER(priv->slot);

  /* Write the SD card CMD */

  esp32s3_ciu_sendcmd(regval, arg);

  return OK;
}

/****************************************************************************
 * Name: esp32s3_blocksetup
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
static void esp32s3_blocksetup(struct sdio_dev_s *dev, unsigned int blocklen,
                               unsigned int nblocks)
{
  /* Configure block size for next transfer */

  esp32s3_putreg(blocklen, ESP32S3_SDMMC_BLKSIZ);
  esp32s3_putreg(blocklen * nblocks, ESP32S3_SDMMC_BYTCNT);
}
#endif

/****************************************************************************
 * Name: esp32s3_recvsetup
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

static int esp32s3_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                             size_t nbytes)
{
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)dev;
#ifdef CONFIG_ESP32S3_SDMMC_DMA
  uint32_t regval;
#endif

  mcinfo("nbytes=%ld\n", (long) nbytes);

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
  priv->wrdir     = false;

  /* Configure the FIFO so that we will receive the RXDR interrupt whenever
   * there are more than 1 words (at least 8 bytes) in the RX FIFO.
   */

  esp32s3_putreg(SDMMC_FIFOTH_RXWMARK(1), ESP32S3_SDMMC_FIFOTH);

#ifdef CONFIG_ESP32S3_SDMMC_DMA
  /* Make sure that internal DMA is disabled */

  esp32s3_putreg(0, ESP32S3_SDMMC_BMOD);

  regval  = esp32s3_getreg(ESP32S3_SDMMC_CTRL);
  regval &= ~SDMMC_CTRL_INTDMA;
  esp32s3_putreg(regval, ESP32S3_SDMMC_CTRL);
#endif

  /* Flush ints before we start */

  esp32s3_putreg(SDCARD_TRANSFER_ALL, ESP32S3_SDMMC_RINTSTS);

  /* Configure the transfer interrupts */

  esp32s3_config_xfrints(priv, SDCARD_RECV_MASK);
  return OK;
}

/****************************************************************************
 * Name: esp32s3_sendsetup
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

static int esp32s3_sendsetup(struct sdio_dev_s *dev, const uint8_t *buffer,
                             size_t nbytes)
{
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)dev;
#ifdef CONFIG_ESP32S3_SDMMC_DMA
  uint32_t regval;
#endif

  mcinfo("nbytes=%ld\n", (long)nbytes);

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
  priv->wrdir     = true;

  /* Configure the FIFO so that we will receive the TXDR interrupt whenever
   * there the TX FIFO is at least half empty.
   */

  esp32s3_putreg(SDMMC_FIFOTH_TXWMARK(ESP32S3_TXFIFO_DEPTH / 2),
                 ESP32S3_SDMMC_FIFOTH);

#ifdef CONFIG_ESP32S3_SDMMC_DMA
  /* Make sure that internal DMA is disabled */

  esp32s3_putreg(0, ESP32S3_SDMMC_BMOD);

  regval  = esp32s3_getreg(ESP32S3_SDMMC_CTRL);
  regval &= ~SDMMC_CTRL_INTDMA;
  esp32s3_putreg(regval, ESP32S3_SDMMC_CTRL);
#endif

  /* Flush ints before we start */

  esp32s3_putreg(SDCARD_TRANSFER_ALL, ESP32S3_SDMMC_RINTSTS);

  /* Configure the transfer interrupts */

  esp32s3_config_xfrints(priv, SDCARD_SEND_MASK);
  return OK;
}

/****************************************************************************
 * Name: esp32s3_cancel
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

static int esp32s3_cancel(struct sdio_dev_s *dev)
{
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)dev;

  mcinfo("Cancelling..\n");

  /* Disable all transfer- and event- related interrupts */

  esp32s3_disable_allints(priv);

  /* Clearing pending interrupt status on all transfer- and event- related
   * interrupts
   */

  esp32s3_putreg(SDCARD_WAITALL_CLEAR, ESP32S3_SDMMC_RINTSTS);

  /* Cancel any watchdog timeout */

  wd_cancel(&priv->waitwdog);

#if defined(CONFIG_ESP32S3_SDMMC_DMA) && defined(CONFIG_ESP32S3_SPIRAM)
  if (!esp32s3_ptr_dma_capable(priv->buffer) && priv->dma_buf)
    {
      kmm_free(priv->dma_buf);
      priv->dma_buf = NULL;
    }
#endif

  /* Mark no transfer in progress */

  priv->remaining = 0;
  return OK;
}

/****************************************************************************
 * Name: esp32s3_waitresponse
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

static int esp32s3_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  int ret = OK;
  volatile int32_t timeout;
  clock_t watchtime;

  mcinfo("cmd=%04x\n", cmd);

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      timeout = SDCARD_CMDTIMEOUT;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
      timeout = SDCARD_LONGTIMEOUT;
      break;

    default:
      return -EINVAL;
    }

  watchtime = clock_systime_ticks();

  /* We should wait for CMDDONE, even if there is a response error. */

  while ((esp32s3_getreg(ESP32S3_SDMMC_RINTSTS) & SDMMC_INT_CDONE) == 0)
    {
      if (clock_systime_ticks() - watchtime > timeout)
        {
          mcerr("ERROR: Timeout cmd: %04x STA: %08x RINTSTS: %08x\n",
                cmd, esp32s3_getreg(ESP32S3_SDMMC_STATUS),
                esp32s3_getreg(ESP32S3_SDMMC_RINTSTS));

          ret = -ETIMEDOUT;
        }
    }

  /* Check if there is a response error. */

  if (esp32s3_getreg(ESP32S3_SDMMC_RINTSTS) & SDCARD_INT_RESPERR)
    {
      mcerr("ERROR: SDMMC failure cmd: %04x STA: %08x RINTSTS: %08x\n",
            cmd, esp32s3_getreg(ESP32S3_SDMMC_STATUS),
            esp32s3_getreg(ESP32S3_SDMMC_RINTSTS));
      ret = -EIO;
    }

  esp32s3_putreg(SDCARD_CMDDONE_CLEAR, ESP32S3_SDMMC_RINTSTS);
  return ret;
}

/****************************************************************************
 * Name: esp32s3_recvshortcrc
 *
 * Description:
 *   Receive response to SDIO command.  Only the critical payload is
 *   returned -- 32 bits for 48 bit status.  The driver implementation
 *   verifies the correctness of the remaining, non-returned bits (CRCs, CMD
 *   index, etc.).
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   Rx   - Buffer in which to receive the response
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure.  Here a
 *   failure means only a faiure to obtain the requested response (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intacta and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int esp32s3_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                                uint32_t *rshort)
{
  uint32_t regval;
  int ret = OK;

  mcinfo("cmd=%04x\n", cmd);

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
      mcerr("ERROR: Wrong response CMD=%04x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = esp32s3_getreg(ESP32S3_SDMMC_RINTSTS);
      if ((regval & SDMMC_INT_RTO) != 0)
        {
          mcerr("ERROR: Command timeout: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & SDMMC_INT_RCRC) != 0)
        {
          mcerr("ERROR: CRC failure: %08x\n", regval);
          ret = -EIO;
        }
    }

  /* Clear all pending message completion events and return the R1/R6
   * response.
   */

  esp32s3_putreg(SDCARD_RESPDONE_CLEAR | SDCARD_CMDDONE_CLEAR,
                 ESP32S3_SDMMC_RINTSTS);
  *rshort = esp32s3_getreg(ESP32S3_SDMMC_RESP0);
  mcinfo("CRC=%04x\n", *rshort);

  return ret;
}

/****************************************************************************
 * Name: esp32s3_recvlong
 *
 * Description:
 *   Receive response to SDIO command.  Only the critical payload is
 *   returned -- 128 bits for 136 bit status.  The driver implementation
 *   verifies the correctness of the remaining, non-returned bits (CRCs, CMD
 *   index, etc.).
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   Rx   - Buffer in which to receive the response
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure.  Here a
 *   failure means only a faiure to obtain the requested response (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intacta and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int esp32s3_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                            uint32_t rlong[4])
{
  uint32_t regval;
  int ret = OK;

  mcinfo("cmd=%04x\n", cmd);

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
      mcerr("ERROR: Wrong response CMD=%04x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = esp32s3_getreg(ESP32S3_SDMMC_RINTSTS);
      if (regval & SDMMC_INT_RTO)
        {
          mcerr("ERROR: Timeout STA: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & SDMMC_INT_RCRC)
        {
          mcerr("ERROR: CRC fail STA: %08x\n", regval);
          ret = -EIO;
        }
    }

  /* Return the long response */

  esp32s3_putreg(SDCARD_RESPDONE_CLEAR | SDCARD_CMDDONE_CLEAR,
                 ESP32S3_SDMMC_RINTSTS);
  if (rlong)
    {
      rlong[0] = esp32s3_getreg(ESP32S3_SDMMC_RESP3);
      rlong[1] = esp32s3_getreg(ESP32S3_SDMMC_RESP2);
      rlong[2] = esp32s3_getreg(ESP32S3_SDMMC_RESP1);
      rlong[3] = esp32s3_getreg(ESP32S3_SDMMC_RESP0);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32s3_recvshort
 *
 * Description:
 *   Receive response to SDIO command.  Only the critical payload is
 *   returned -- 32 bits for 48 bit status.  The driver implementation
 *   verifies the correctness of the remaining, non-returned bits (CMD
 *   index, etc., not including CRC).
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   Rx   - Buffer in which to receive the response
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure.  Here a
 *   failure means only a faiure to obtain the requested response (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intacta and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int esp32s3_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t *rshort)
{
  uint32_t regval;
  int ret = OK;

  mcinfo("cmd=%04x\n", cmd);

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
      mcerr("ERROR: Wrong response CMD=%04x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout occurred (Apparently a CRC error can terminate
       * a good response)
       */

      regval = esp32s3_getreg(ESP32S3_SDMMC_RINTSTS);
      if (regval & SDMMC_INT_RTO)
        {
          mcerr("ERROR: Timeout STA: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
    }

  esp32s3_putreg(SDCARD_RESPDONE_CLEAR | SDCARD_CMDDONE_CLEAR,
                 ESP32S3_SDMMC_RINTSTS);
  if (rshort)
    {
      *rshort = esp32s3_getreg(ESP32S3_SDMMC_RESP0);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32s3_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the SDIO_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling esp32s3_eventwait.  This is done in this way
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

static void esp32s3_waitenable(struct sdio_dev_s *dev,
                               sdio_eventset_t eventset, uint32_t timeout)
{
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)dev;
  uint32_t waitmask;

  mcinfo("eventset=%04x\n", (unsigned int)eventset);
  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  esp32s3_config_waitints(priv, 0, 0, 0);

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

  esp32s3_config_waitints(priv, waitmask, eventset, 0);

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
                       esp32s3_eventtimeout, (wdparm_t)priv);
      if (ret < 0)
        {
          mcerr("ERROR: wd_start failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: esp32s3_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDIO_WAITEVENTS are disabled when
 *   esp32s3_eventwait returns.  SDIO_WAITEVENTS must be called again before
 *   esp32s3_eventwait can be used again.
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

static sdio_eventset_t esp32s3_eventwait(struct sdio_dev_s *dev)
{
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)dev;
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
   * avoided by calling esp32s3_waitenable prior to triggering the logic that
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
          /* Task canceled.  Cancel the wdog -- assuming it was started and
           * return an SDIO error.
           */

          wd_cancel(&priv->waitwdog);
          wkupevent = SDIOWAIT_ERROR;
          goto out;
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

  /* Disable all transfer- and event- related interrupts */

  esp32s3_disable_allints(priv);

out:
  leave_critical_section(flags);

#if defined(CONFIG_ESP32S3_SDMMC_DMA) && defined(CONFIG_ESP32S3_SPIRAM)
  if (!esp32s3_ptr_dma_capable(priv->buffer) && priv->dma_buf)
    {
      if (!priv->wrdir && wkupevent == SDIOWAIT_TRANSFERDONE)
        {
          memcpy(priv->buffer, priv->dma_buf, priv->dma_buf_size);
        }

      kmm_free(priv->dma_buf);
      priv->dma_buf = NULL;
    }
#endif

  mcinfo("wkupevent=%04x\n", wkupevent);
  return wkupevent;
}

/****************************************************************************
 * Name: esp32s3_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events.  This is part of the
 *   the SDIO callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in esp32s3_registercallback.
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

static void esp32s3_callbackenable(struct sdio_dev_s *dev,
                                   sdio_eventset_t eventset)
{
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)dev;

  mcinfo("eventset: %02x\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  esp32s3_callback(priv);
}

/****************************************************************************
 * Name: esp32s3_registercallback
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

static int esp32s3_registercallback(struct sdio_dev_s *dev,
                                    worker_t callback, void *arg)
{
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)dev;

  /* Disable callbacks and register this callback and is argument */

  mcinfo("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

#ifdef CONFIG_ESP32S3_SDMMC_DMA
static int esp32s3_fill_dma_desc(struct esp32s3_dev_s *priv)
{
  uint32_t ctrl;
  uint32_t maxs;
  int i = 0;
  size_t buflen = priv->remaining;
  uint32_t buffer = (uint32_t)priv->buffer;

#ifdef CONFIG_ESP32S3_SPIRAM
  if (!esp32s3_ptr_dma_capable(priv->buffer))
    {
      priv->dma_buf = kmm_memalign(16, buflen);
      if (!priv->dma_buf)
        {
          return -ENOMEM;
        }

      priv->dma_buf_size = buflen;
      buffer = (uint32_t)priv->dma_buf;

      if (priv->wrdir)
        {
          memcpy(priv->dma_buf, priv->buffer, buflen);
        }
    }
#endif

  /* Setup DMA list */

  while (buflen > 0)
    {
      /* Limit size of the transfer to maximum buffer size */

      maxs = buflen;

      if (maxs > MCI_DMADES1_MAXTR)
        {
          maxs = MCI_DMADES1_MAXTR;
        }

      buflen -= maxs;

      /* Set buffer size */

      priv->dma_desc[i].des1 = MCI_DMADES1_BS1(maxs);

      /* Setup buffer address (chained) */

      priv->dma_desc[i].des2 = buffer + (i * MCI_DMADES1_MAXTR);

      /* Setup basic control */

      ctrl = MCI_DMADES0_OWN | MCI_DMADES0_CH;

      if (i == 0)
        {
          ctrl |= MCI_DMADES0_FS; /* First DMA buffer */
        }

      /* No more data?  Then this is the last descriptor */

      if (buflen == 0)
        {
          ctrl |= MCI_DMADES0_LD;
          priv->dma_desc[i].des3 = 0;
        }
      else
        {
          ctrl |= MCI_DMADES0_DIC;
          priv->dma_desc[i].des3 = (uint32_t)&priv->dma_desc[i + 1];
        }

      priv->dma_desc[i].des0 = ctrl;
      i++;
    }

  DEBUGASSERT(i < NUM_DMA_DESCRIPTORS);

  return 0;
}
#endif

/****************************************************************************
 * Name: esp32s3_dmarecvsetup
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

#ifdef CONFIG_ESP32S3_SDMMC_DMA
static int esp32s3_dmarecvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                                size_t buflen)
{
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)dev;
  uint32_t regval;

  /* Don't bother with DMA if the entire transfer will fit in the RX FIFO or
   * if we do not have a 4-bit wide bus.
   */

  DEBUGASSERT(priv != NULL);

  DEBUGASSERT(buffer != NULL && buflen > 0 && ((uint32_t)buffer & 3) == 0);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->wrdir     = false;

  /* Setup DMA list */

  if (esp32s3_fill_dma_desc(priv))
    {
      return -ENOMEM;
    }

  /* Flush ints before we start */

  esp32s3_putreg(SDCARD_TRANSFER_ALL, ESP32S3_SDMMC_RINTSTS);

  /* Enable internal DMA, burst size of 4, fixed burst */

  regval  = esp32s3_getreg(ESP32S3_SDMMC_CTRL);
  regval |= SDMMC_CTRL_INTDMA;
  esp32s3_putreg(regval, ESP32S3_SDMMC_CTRL);

  regval = SDMMC_BMOD_DE | SDMMC_BMOD_FB;
  esp32s3_putreg(regval, ESP32S3_SDMMC_BMOD);

  esp32s3_putreg(1, ESP32S3_SDMMC_PLDMND);

  /* Setup DMA error interrupts */

  esp32s3_config_dmaints(priv, SDCARD_DMARECV_MASK, SDCARD_DMAERROR_MASK);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp32s3_dmasendsetup
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

#ifdef CONFIG_ESP32S3_SDMMC_DMA
static int esp32s3_dmasendsetup(struct sdio_dev_s *dev,
                                const uint8_t *buffer, size_t buflen)
{
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)dev;
  uint32_t regval;

  /* Don't bother with DMA if the entire transfer will fit in the TX FIFO or
   * if we do not have a 4-bit wide bus.
   */

  DEBUGASSERT(priv != NULL);

  mcinfo("buflen=%lu\n", (unsigned long)buflen);
  DEBUGASSERT(buffer != NULL && buflen > 0 && ((uint32_t)buffer & 3) == 0);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->wrdir     = true;

  /* Setup DMA descriptor list */

  if (esp32s3_fill_dma_desc(priv))
    {
      return -ENOMEM;
    }

  /* Flush ints before we start */

  esp32s3_putreg(SDCARD_TRANSFER_ALL, ESP32S3_SDMMC_RINTSTS);

  /* Enable internal DMA, fixed burst */

  regval  = esp32s3_getreg(ESP32S3_SDMMC_CTRL);
  regval |= SDMMC_CTRL_INTDMA;
  esp32s3_putreg(regval, ESP32S3_SDMMC_CTRL);

  regval = SDMMC_BMOD_DE | SDMMC_BMOD_FB;
  esp32s3_putreg(regval, ESP32S3_SDMMC_BMOD);
  esp32s3_putreg(1, ESP32S3_SDMMC_PLDMND);

  /* Setup DMA error interrupts */

  esp32s3_config_dmaints(priv, SDCARD_DMASEND_MASK, SDCARD_DMAERROR_MASK);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp32s3_callback
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

static void esp32s3_callback(void *arg)
{
  struct esp32s3_dev_s *priv = (struct esp32s3_dev_s *)arg;

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

const static sdmmc_slot_info_t sdmmc_slot_info[] =
{
    {
      .card_detect = SDHOST_CARD_DETECT_N_1_IDX,
      .write_protect = SDHOST_CARD_WRITE_PRT_1_IDX,
      .card_int = SDHOST_CARD_INT_N_1_IDX,
    },

    {
      .card_detect = SDHOST_CARD_DETECT_N_2_IDX,
      .write_protect = SDHOST_CARD_WRITE_PRT_2_IDX,
      .card_int = SDHOST_CARD_INT_N_2_IDX,
    }
};

const static sdmmc_slot_io_info_t sdmmc_slot_gpio_sig[] =
{
    {
      .clk = SDHOST_CCLK_OUT_1_IDX,
      .cmd = SDHOST_CCMD_OUT_1_IDX,
      .d0 = SDHOST_CDATA_OUT_10_IDX,
      .d1 = SDHOST_CDATA_OUT_11_IDX,
      .d2 = SDHOST_CDATA_OUT_12_IDX,
      .d3 = SDHOST_CDATA_OUT_13_IDX,
      .d4 = SDHOST_CDATA_OUT_14_IDX,
      .d5 = SDHOST_CDATA_OUT_15_IDX,
      .d6 = SDHOST_CDATA_OUT_16_IDX,
      .d7 = SDHOST_CDATA_OUT_17_IDX,
    },

    {
      .clk = SDHOST_CCLK_OUT_2_IDX,
      .cmd = SDHOST_CCMD_OUT_2_IDX,
      .d0 = SDHOST_CDATA_OUT_20_IDX,
      .d1 = SDHOST_CDATA_OUT_21_IDX,
      .d2 = SDHOST_CDATA_OUT_22_IDX,
      .d3 = SDHOST_CDATA_OUT_23_IDX,
      .d4 = SDHOST_CDATA_OUT_24_IDX,
      .d5 = SDHOST_CDATA_OUT_25_IDX,
      .d6 = SDHOST_CDATA_OUT_26_IDX,
      .d7 = SDHOST_CDATA_OUT_27_IDX,
    }
};

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
  uint32_t regval;

  struct esp32s3_dev_s *priv = &g_sdiodev;

  priv->slot = slotno;
  priv->slot_info = &sdmmc_slot_info[slotno];
  priv->sdio_pins = &sdmmc_slot_gpio_sig[slotno];

  /* enable bus clock */

  regval = esp32s3_getreg(SYSTEM_PERIP_CLK_EN1_REG);
  regval |= SYSTEM_SDIO_HOST_CLK_EN;
  esp32s3_putreg(regval, SYSTEM_PERIP_CLK_EN1_REG);

  /* reset registers */

  regval = esp32s3_getreg(SYSTEM_PERIP_RST_EN1_REG);
  regval |= SYSTEM_SDIO_HOST_RST;
  esp32s3_putreg(regval, SYSTEM_PERIP_RST_EN1_REG);
  regval &= ~SYSTEM_SDIO_HOST_RST;
  esp32s3_putreg(regval, SYSTEM_PERIP_RST_EN1_REG);

  /* Reset */

  priv->dev.reset(&priv->dev);

  /* Pin configuration */

  configure_pin(CONFIG_ESP32S3_SDMMC_CLK, priv->sdio_pins->clk, OUTPUT);
  configure_pin(CONFIG_ESP32S3_SDMMC_CMD, priv->sdio_pins->cmd,
                INPUT | OUTPUT | PULLUP);
  configure_pin(CONFIG_ESP32S3_SDMMC_D0, priv->sdio_pins->d0,
                INPUT | OUTPUT | PULLUP);

  esp32s3_gpio_matrix_in(GPIO_MATRIX_CONST_ONE_INPUT,
                         priv->slot_info->card_int, false);
  esp32s3_gpio_matrix_in(GPIO_MATRIX_CONST_ZERO_INPUT,
                         priv->slot_info->card_detect, false);
  esp32s3_gpio_matrix_in(GPIO_MATRIX_CONST_ONE_INPUT,
                         priv->slot_info->write_protect, true);

  return &priv->dev;
}

#endif /* CONFIG_ESP32S3_SDMMC */
