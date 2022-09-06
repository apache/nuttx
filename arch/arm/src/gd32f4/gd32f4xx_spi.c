/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_spi.c
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
 * The external functions, gd32_spi0/1/2/3/4/5select and gd32_spi0/1/2/3/4/5
 * status must be provided by board-specific logic.  They are implementations
 * of the select and status methods of the SPI interface defined by
 * struct spi_ops_s (see include/nuttx/spi/spi.h).
 * All other methods (including gd32_spibus_initialize())  are provided by
 * common GD32F4 logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in gd32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide gd32_spi[n]select() and gd32_spi[n]status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to gd32_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by gd32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"

#include "chip.h"
#include "gd32f4xx.h"
#include "gd32f4xx_spi.h"

#if defined(CONFIG_GD32F4_SPI0) || defined(CONFIG_GD32F4_SPI1) || \
    defined(CONFIG_GD32F4_SPI2) || defined(CONFIG_GD32F4_SPI3) || \
    defined(CONFIG_GD32F4_SPI4) || defined(CONFIG_GD32F4_SPI5)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GD32_SPI_CLK_MAX       (30000000ul)
#define GD32_SPI_CLK_INIT      (400000)

/* SPI parameter initialization mask */

#define SPI_INIT_MASK          ((uint32_t)0x00003040u)  /* SPI parameter initialization mask */

/* SPI interrupts */

#ifdef CONFIG_GD32F4_SPI_INTERRUPT
#  error "Now SPI interrupt has not yet supported"
#endif

#ifdef CONFIG_GD32F4_SPI_DMA

#  error "Now SPI DMA has not ready"

/* SPI DMA priority */

#if defined(CONFIG_GD32F4_SPI_PRIQ)
#  define SPI_DMA_PRIO  CONFIG_GD32F4_SPI_PRIQ
#else 
#  define SPI_DMA_PRIO  DMA_PRIO_MEDIUM_SELECT
#endif

#  define SPI_DMA_BUFFER_MASK      (4 - 1)
#  define SPI_DMA_BUFEER_ALIGN     aligned_data(4)

#  if defined(CONFIG_GD32F4_SPI0_DMA_BUFFER) && CONFIG_GD32F4_SPI0_DMA_BUFFER > 0
#    define SPI0_DMA_BUFSIZE_ADJ   ((CONFIG_GD32F4_SPI0_DMA_BUFFER + SPI_DMA_BUFFER_MASK) & \
                                    ~SPI_DMA_BUFFER_MASK)
#  endif

#  if defined(CONFIG_GD32F4_SPI1_DMA_BUFFER) && CONFIG_GD32F4_SPI1_DMA_BUFFER > 0
#    define SPI1_DMA_BUFSIZE_ADJ   ((CONFIG_GD32F4_SPI1_DMA_BUFFER + SPI_DMA_BUFFER_MASK) & \
                                    ~SPI_DMA_BUFFER_MASK)
#  endif

#  if defined(CONFIG_GD32F4_SPI2_DMA_BUFFER) && CONFIG_GD32F4_SPI2_DMA_BUFFER > 0
#    define SPI2_DMA_BUFSIZE_ADJ   ((CONFIG_GD32F4_SPI2_DMA_BUFFER + SPI_DMA_BUFFER_MASK) & \
                                    ~SPI_DMA_BUFFER_MASK)
#  endif

#  if defined(CONFIG_GD32F4_SPI3_DMA_BUFFER) && CONFIG_GD32F4_SPI3_DMA_BUFFER > 0
#    define SPI3_DMA_BUFSIZE_ADJ   ((CONFIG_GD32F4_SPI3_DMA_BUFFER + SPI_DMA_BUFFER_MASK) & \
                                    ~SPI_DMA_BUFFER_MASK)
#  endif

#  if defined(CONFIG_GD32F4_SPI4_DMA_BUFFER) && CONFIG_GD32F4_SPI4_DMA_BUFFER > 0
#    define SPI4_DMA_BUFSIZE_ADJ   ((CONFIG_GD32F4_SPI4_DMA_BUFFER + SPI_DMA_BUFFER_MASK) & \
                                    ~SPI_DMA_BUFFER_MASK)
#  endif

#  if defined(CONFIG_GD32F4_SPI5_DMA_BUFFER) && CONFIG_GD32F4_SPI5_DMA_BUFFER > 0
#    define SPI5_DMA_BUFSIZE_ADJ   ((CONFIG_GD32F4_SPI5_DMA_BUFFER + SPI_DMA_BUFFER_MASK) & \
                                    ~SPI_DMA_BUFFER_MASK)
#  endif

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gd32_spidev_s
{
  struct spi_dev_s spidev;       /* Externally visible part of the SPI interface */
  uint32_t         spibase;      /* SPIn spibase address */
  uint32_t         spiclock;     /* Clocking for the SPI module */
  uint32_t         frequency;    /* Requested clock frequency */
  uint32_t         actual;       /* Actual clock frequency */
  mutex_t          lock;         /* Held while chip is selected for mutual exclusion */
  uint8_t          nbits;        /* Width of word in bits (8 to 16) */
  uint8_t          mode;         /* Mode 0,1,2,3 */
#ifdef CONFIG_GD32F4_SPI_INTERRUPT
  uint8_t          spiirq;       /* SPI IRQ number */
#endif
#ifdef CONFIG_GD32F4_SPI_DMA
  volatile uint16_t rxresult;     /* Result of the RX DMA */
  volatile uint16_t txresult;     /* Result of the RX DMA */
#ifdef CONFIG_SPI_TRIGGER
  bool             defertrig;    /* Flag indicating that trigger should be deferred */
  bool             trigarmed;    /* Flag indicating that the trigger is armed */
#endif
  uint8_t          rxch;         /* The RX DMA channel number */
  uint8_t          txch;         /* The TX DMA channel number */
  uint8_t          *rxbuf;       /* The RX DMA buffer */
  uint8_t          *txbuf;       /* The TX DMA buffer */
  size_t           buflen;       /* The DMA buffer length */
  DMA_HANDLE       rxdma;        /* DMA channel handle for RX transfers */
  DMA_HANDLE       txdma;        /* DMA channel handle for TX transfers */
  sem_t            rxsem;        /* Wait for RX DMA to complete */
  sem_t            txsem;        /* Wait for TX DMA to complete */
#endif
  bool             initialized;  /* Has SPI interface been initialized */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Clock */

static void            gd32_spi_reset(uint32_t spibase);
static void            gd32_spi_clock_enable(uint32_t spibase);
static void            gd32_spi_clock_disable(uint32_t spibase);

/* Helpers */

static inline uint32_t spi_getreg(struct gd32_spidev_s *priv,
                                  uint8_t offset);
static inline void     spi_putreg(struct gd32_spidev_s *priv,
                                  uint8_t offset, uint32_t value);
static inline uint16_t spi_getreg16(struct gd32_spidev_s *priv,
                                    uint8_t offset);

static inline uint16_t spi_readword(struct gd32_spidev_s *priv);
static inline void     spi_writeword(struct gd32_spidev_s *priv,
                                     uint16_t byte);

/* SPI methods */

static int             spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t        spi_setfrequency(struct spi_dev_s *dev,
                                        uint32_t frequency);
static void            spi_setmode(struct spi_dev_s *dev,
                                   enum spi_mode_e mode);
static void            spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int             spi_hwfeatures(struct spi_dev_s *dev,
                                      spi_hwfeatures_t features);
#endif
static uint32_t        spi_send(struct spi_dev_s *dev, uint32_t wd);
static void            spi_exchange(struct spi_dev_s *dev,
                                    const void *txbuffer,
                                    void *rxbuffer, size_t nwords);
#ifdef CONFIG_SPI_TRIGGER
static int             spi_trigger(struct spi_dev_s *dev);
#endif
#ifndef CONFIG_SPI_EXCHANGE
static void            spi_sndblock(struct spi_dev_s *dev,
                                    const void *txbuffer, size_t nwords);
static void            spi_recvblock(struct spi_dev_s *dev,
                                     void *rxbuffer, size_t nwords);
#endif

/* DMA support */

#ifdef CONFIG_GD32F4_SPI_DMA
static int             spi_dmarxwait(struct gd32_spidev_s *priv);
static int             spi_dmatxwait(struct gd32_spidev_s *priv);
static void            spi_dmarxcallback(DMA_HANDLE handle, uint16_t isr,
                                         void *arg);
static void            spi_dmatxcallback(DMA_HANDLE handle, uint16_t isr,
                                         void *arg);
static void            spi_dmarxsetup(struct gd32_spidev_s *priv,
                                      void *rxbuffer, void *rxdummy,
                                      size_t nwords);
static void            spi_dmatxsetup(struct gd32_spidev_s *priv,
                                      const void *txbuffer,
                                      const void *txdummy,
                                      size_t nwords);
static inline void     spi_dmarxstart(struct gd32_spidev_s *priv);
static inline void     spi_dmatxstart(struct gd32_spidev_s *priv);
#endif

/* Initialization */

static void            spi_bus_initialize(struct gd32_spidev_s *priv);
static void            gd32_spi_gpio_config(uint32_t spibase);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_GD32F4_SPI0
static const struct spi_ops_s g_spi0ops =
{
  .lock              = spi_lock,
  .select            = gd32_spi0select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = gd32_spi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = gd32_spi0cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = gd32_spi0register,  /* Provided externally */
#else
  .registercallback  = 0,                   /* Not implemented */
#endif
};

#if defined(SPI0_DMA_BUFSIZE_ADJ)
static uint8_t g_spi0_txbuf[SPI0_DMA_BUFSIZE_ADJ] SPI_DMA_BUFEER_ALIGN;
static uint8_t g_spi0_rxbuf[SPI0_DMA_BUFSIZE_ADJ] SPI_DMA_BUFEER_ALIGN;
#endif

static struct gd32_spidev_s g_spi0dev =
{
  .spidev   =
              {
               &g_spi0ops
              },
  .spibase  = GD32_SPI0,
  .spiclock = GD32_PCLK2_FREQUENCY,
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_GD32F4_SPI_INTERRUPT
  .spiirq   = GD32_IRQ_SPI0,
#endif
#ifdef CONFIG_GD32F4_SPI_DMA
#  ifdef CONFIG_GD32F4_SPI0_DMA
  .rxch     = DMA_CHANNEL_SPI0_RX,
  .txch     = DMA_CHANNEL_SPI0_TX,
#    if defined(SPI0_DMA_BUFSIZE_ADJ)
  .rxbuf    = g_spi0_rxbuf,
  .txbuf    = g_spi0_txbuf,
  .buflen   = SPI0_DMA_BUFSIZE_ADJ,
#    endif
#  else
  .rxch     = 0,
  .txch     = 0,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_GD32F4_SPI1
static const struct spi_ops_s g_spi1ops =
{
  .lock              = spi_lock,
  .select            = gd32_spi1select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = gd32_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = gd32_spi1cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = gd32_spi1register,  /* Provided externally */
#else
  .registercallback  = 0,                   /* Not implemented */
#endif
};

#if defined(SPI1_DMA_BUFSIZE_ADJ)
static uint8_t g_spi1_txbuf[SPI1_DMA_BUFSIZE_ADJ] SPI_DMA_BUFEER_ALIGN;
static uint8_t g_spi1_rxbuf[SPI1_DMA_BUFSIZE_ADJ] SPI_DMA_BUFEER_ALIGN;
#endif

static struct gd32_spidev_s g_spi1dev =
{
  .spidev   =
              {
               &g_spi1ops
              },
  .spibase  = GD32_SPI1,
  .spiclock = GD32_PCLK1_FREQUENCY,
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_GD32F4_SPI_INTERRUPT
  .spiirq   = GD32_IRQ_SPI1,
#endif
#ifdef CONFIG_GD32F4_SPI_DMA
#  ifdef CONFIG_GD32F4_SPI1_DMA
  .rxch     = DMA_CHANNEL_SPI1_RX,
  .txch     = DMA_CHANNEL_SPI1_TX,
#    if defined(SPI1_DMA_BUFSIZE_ADJ)
  .rxbuf    = g_spi1_rxbuf,
  .txbuf    = g_spi1_txbuf,
  .buflen   = SPI1_DMA_BUFSIZE_ADJ,
#    endif
#  else
  .rxch     = 0,
  .txch     = 0,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_GD32F4_SPI2
static const struct spi_ops_s g_spi2ops =
{
  .lock              = spi_lock,
  .select            = gd32_spi2select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = gd32_spi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = gd32_spi2cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = gd32_spi2register,  /* Provided externally */
#else
  .registercallback  = 0,                   /* Not implemented */
#endif
};

#if defined(SPI2_DMA_BUFSIZE_ADJ)
static uint8_t g_spi2_txbuf[SPI2_DMA_BUFSIZE_ADJ] SPI_DMA_BUFEER_ALIGN;
static uint8_t g_spi2_rxbuf[SPI2_DMA_BUFSIZE_ADJ] SPI_DMA_BUFEER_ALIGN;
#endif

static struct gd32_spidev_s g_spi2dev =
{
  .spidev   =
              {
               &g_spi2ops
              },
  .spibase  = GD32_SPI2,
  .spiclock = GD32_PCLK1_FREQUENCY,
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_GD32F4_SPI_INTERRUPT
  .spiirq   = GD32_IRQ_SPI2,
#endif
#ifdef CONFIG_GD32F4_SPI_DMA
#  ifdef CONFIG_GD32F4_SPI2_DMA
  .rxch     = DMA_CHANNEL_SPI2_RX,
  .txch     = DMA_CHANNEL_SPI2_TX,
#    if defined(SPI2_DMA_BUFSIZE_ADJ)
  .rxbuf    = g_spi2_rxbuf,
  .txbuf    = g_spi2_txbuf,
  .buflen   = SPI2_DMA_BUFSIZE_ADJ,
#    endif
#  else
  .rxch     = 0,
  .txch     = 0,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_GD32F4_SPI3
static const struct spi_ops_s g_spi3ops =
{
  .lock              = spi_lock,
  .select            = gd32_spi3select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = gd32_spi3status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = gd32_spi3cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = gd32_spi3register,  /* Provided externally */
#else
  .registercallback  = 0,                   /* Not implemented */
#endif
};

#if defined(SPI3_DMA_BUFSIZE_ADJ)
static uint8_t g_spi3_txbuf[SPI3_DMA_BUFSIZE_ADJ] SPI_DMA_BUFEER_ALIGN;
static uint8_t g_spi3_rxbuf[SPI3_DMA_BUFSIZE_ADJ] SPI_DMA_BUFEER_ALIGN;
#endif

static struct gd32_spidev_s g_spi3dev =
{
  .spidev   =
              {
               &g_spi3ops
              },
  .spibase  = GD32_SPI3,
  .spiclock = GD32_PCLK2_FREQUENCY,
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_GD32F4_SPI_INTERRUPT
  .spiirq   = GD32_IRQ_SPI3,
#endif
#ifdef CONFIG_GD32F4_SPI_DMA
#  ifdef CONFIG_GD32F4_SPI3_DMA
  .rxch     = DMA_CHANNEL_SPI3_RX,
  .txch     = DMA_CHANNEL_SPI3_TX,
#    if defined(SPI3_DMA_BUFSIZE_ADJ)
  .rxbuf    = g_spi3_rxbuf,
  .txbuf    = g_spi3_txbuf,
  .buflen   = SPI3_DMA_BUFSIZE_ADJ,
#    endif
#  else
  .rxch     = 0,
  .txch     = 0,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_GD32F4_SPI4
static const struct spi_ops_s g_spi4ops =
{
  .lock              = spi_lock,
  .select            = gd32_spi4select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = gd32_spi4status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = gd32_spi4cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = gd32_spi4register,  /* Provided externally */
#else
  .registercallback  = 0,                   /* Not implemented */
#endif
};

#if defined(SPI4_DMA_BUFSIZE_ADJ)
static uint8_t g_spi4_txbuf[SPI4_DMA_BUFSIZE_ADJ] SPI_DMA_BUFEER_ALIGN;
static uint8_t g_spi4_rxbuf[SPI4_DMA_BUFSIZE_ADJ] SPI_DMA_BUFEER_ALIGN;
#endif

static struct gd32_spidev_s g_spi4dev =
{
  .spidev   =
              {
               &g_spi4ops
              },
  .spibase  = GD32_SPI4,
  .spiclock = GD32_PCLK2_FREQUENCY,
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_GD32F4_SPI_INTERRUPT
  .spiirq   = GD32_IRQ_SPI4,
#endif
#ifdef CONFIG_GD32F4_SPI_DMA
#  ifdef CONFIG_GD32F4_SPI4_DMA
  .rxch     = DMA_CHANNEL_SPI4_RX,
  .txch     = DMA_CHANNEL_SPI4_TX,
#    if defined(SPI4_DMA_BUFSIZE_ADJ)
  .rxbuf    = g_spi4_rxbuf,
  .txbuf    = g_spi4_txbuf,
  .buflen   = SPI4_DMA_BUFSIZE_ADJ,
#    endif
#  else
  .rxch     = 0,
  .txch     = 0,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_GD32F4_SPI5
static const struct spi_ops_s g_spi5ops =
{
  .lock              = spi_lock,
  .select            = gd32_spi5select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = gd32_spi5status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = gd32_spi5cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = gd32_spi5register,  /* Provided externally */
#else
  .registercallback  = 0,                   /* Not implemented */
#endif
};

#if defined(SPI5_DMA_BUFSIZE_ADJ)
static uint8_t g_spi5_txbuf[SPI5_DMA_BUFSIZE_ADJ] SPI_DMA_BUFEER_ALIGN;
static uint8_t g_spi5_rxbuf[SPI5_DMA_BUFSIZE_ADJ] SPI_DMA_BUFEER_ALIGN;
#endif

static struct gd32_spidev_s g_spi5dev =
{
  .spidev   =
              {
               &g_spi5ops
              },
  .spibase  = GD32_SPI5,
  .spiclock = GD32_PCLK2_FREQUENCY,
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_GD32F5_SPI_INTERRUPT
  .spiirq   = GD32_IRQ_SPI5,
#endif
#ifdef CONFIG_GD32F4_SPI_DMA
#  ifdef CONFIG_GD32F4_SPI5_DMA
  .rxch     = DMA_CHANNEL_SPI5_RX,
  .txch     = DMA_CHANNEL_SPI5_TX,
#    if defined(SPI5_DMA_BUFSIZE_ADJ)
  .rxbuf    = g_spi5_rxbuf,
  .txbuf    = g_spi5_txbuf,
  .buflen   = SPI5_DMA_BUFSIZE_ADJ,
#    endif
#  else
  .rxch     = 0,
  .txch     = 0,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_spi_reset
 *
 * Description:
 *   Reset the SPI.
 *
 ****************************************************************************/

static void gd32_spi_reset(uint32_t spibase)
{
  uint32_t rcu_rst;
  uint32_t regaddr;

  /* Determine which SPI to configure */

  switch (spibase)
    {
    default:
      return;
#ifdef CONFIG_GD32F4_SPI0
    case GD32_SPI0:
      rcu_rst = RCU_APB2RST_SPI0RST;
      regaddr = GD32_RCU_APB2RST;
      break;
#endif
#ifdef CONFIG_GD32F4_SPI1
    case GD32_SPI1:
      rcu_rst = RCU_APB1RST_SPI1RST;
      regaddr = GD32_RCU_APB1RST;
      break;
#endif
#ifdef CONFIG_GD32F4_SPI2
    case GD32_SPI2:
      rcu_rst = RCU_APB1RST_SPI2RST;
      regaddr = GD32_RCU_APB1RST;
      break;
#endif
#ifdef CONFIG_GD32F4_SPI3
    case GD32_SPI3:
      rcu_rst = RCU_APB2RST_SPI3RST;
      regaddr = GD32_RCU_APB2RST;
      break;
#endif
#ifdef CONFIG_GD32F4_SPI4
    case GD32_SPI4:
      rcu_rst = RCU_APB2RST_SPI4RST;
      regaddr = GD32_RCU_APB2RST;
      break;
#endif
#ifdef CONFIG_GD32F4_SPI5
    case GD32_SPI5:
      rcu_rst = RCU_APB2RST_SPI5RST;
      regaddr = GD32_RCU_APB2RST;
      break;
#endif
    }

  /* Enable APB 1/2 reset for SPI */

  modifyreg32(regaddr, 0, rcu_rst);

  /* Disable APB 1/2 reset for SPI */

  modifyreg32(regaddr, rcu_rst, 0);
}

/****************************************************************************
 * Name: gd32_spi_clock_enable
 *
 * Description:
 *   Enable SPI clock
 ****************************************************************************/

static void gd32_spi_clock_enable(uint32_t spibase)
{
  uint32_t rcu_en;
  uint32_t regaddr;

  /* Determine which SPI to configure */

  switch (spibase)
    {
    default:
      return;
#ifdef CONFIG_GD32F4_SPI0
    case GD32_SPI0:
      rcu_en = RCU_APB2EN_SPI0EN;
      regaddr = GD32_RCU_APB2EN;
      break;
#endif
#ifdef CONFIG_GD32F4_SPI1
    case GD32_SPI1:
      rcu_en = RCU_APB1EN_SPI1EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_SPI2
    case GD32_SPI2:
      rcu_en = RCU_APB1EN_SPI2EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_SPI3
    case GD32_SPI3:
      rcu_en = RCU_APB2EN_SPI3EN;
      regaddr = GD32_RCU_APB2EN;
      break;
#endif
#ifdef CONFIG_GD32F4_SPI4
    case GD32_SPI4:
      rcu_en = RCU_APB2EN_SPI4EN;
      regaddr = GD32_RCU_APB2EN;
      break;
#endif
#ifdef CONFIG_GD32F4_SPI5
    case GD32_SPI5:
      rcu_en = RCU_APB2EN_SPI5EN;
      regaddr = GD32_RCU_APB2EN;
      break;
#endif
    }

  /* Enable APB 1/2 clock for SPI */

  modifyreg32(regaddr, 0, rcu_en);
}

/****************************************************************************
 * Name: gd32_spi_clock_disable
 *
 * Description:
 *   Dinable SPI clock
 ****************************************************************************/

static void gd32_spi_clock_disable(uint32_t spibase)
{
  uint32_t rcu_en;
  uint32_t regaddr;

  /* Determine which SPI to configure */

  switch (spibase)
    {
    default:
      return;
#ifdef CONFIG_GD32F4_SPI0
    case GD32_SPI0:
      rcu_en = RCU_APB2EN_SPI0EN;
      regaddr = GD32_RCU_APB2EN;
      break;
#endif
#ifdef CONFIG_GD32F4_SPI1
    case GD32_SPI1:
      rcu_en = RCU_APB1EN_SPI1EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_SPI2
    case GD32_SPI2:
      rcu_en = RCU_APB1EN_SPI2EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_SPI3
    case GD32_SPI3:
      rcu_en = RCU_APB2EN_SPI3EN;
      regaddr = GD32_RCU_APB2EN;
      break;
#endif
#ifdef CONFIG_GD32F4_SPI4
    case GD32_SPI4:
      rcu_en = RCU_APB2EN_SPI4EN;
      regaddr = GD32_RCU_APB2EN;
      break;
#endif
#ifdef CONFIG_GD32F4_SPI5
    case GD32_SPI5:
      rcu_en = RCU_APB2EN_SPI5EN;
      regaddr = GD32_RCU_APB2EN;
      break;
#endif
    }

  /* Disable APB 1/2 clock for SPI */

  modifyreg32(regaddr, rcu_en, 0);
}

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline uint32_t spi_getreg(struct gd32_spidev_s *priv,
                                  uint8_t offset)
{
  return getreg32(priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a 32-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 32-bit value to be written
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline void spi_putreg(struct gd32_spidev_s *priv,
                              uint8_t offset, uint32_t value)
{
  putreg32(value, priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_getreg16
 *
 * Description:
 *   Get the 16 bit contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ****************************************************************************/

static inline uint16_t spi_getreg16(struct gd32_spidev_s *priv,
                                    uint8_t offset)
{
  return getreg16(priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_modifyreg
 *
 * Description:
 *   Clear and set bits in the register
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_modifyreg(struct gd32_spidev_s *priv, uint32_t offset,
                          uint16_t setbits, uint16_t clrbits)
{
  uint32_t regval;
  regval = spi_getreg(priv, offset);
  regval &= ~clrbits;
  regval |= setbits;
  spi_putreg(priv, offset, regval);
}

/****************************************************************************
 * Name: spi_wait_status
 *
 * Description:
 *   Wait for bit to be set in status
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   status- bit to wait on.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_wait_status(struct gd32_spidev_s *priv,
                                   uint32_t offset, uint32_t status)
{
  while (status != (spi_getreg(priv, offset) & status));
}

/****************************************************************************
 * Name: spi_readword
 *
 * Description:
 *   Read one 16 bit word from SPI DATA
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   16 bit word as read
 *
 ****************************************************************************/

static inline uint16_t spi_readword(struct gd32_spidev_s *priv)
{
  /* Wait until the receive buffer is not empty */

  spi_wait_status(priv, GD32_SPI_STAT_OFFSET, SPI_STAT_RBNE);

  /* Return the data */

  return spi_getreg16(priv, GD32_SPI_DATA_OFFSET);
}

/****************************************************************************
 * Name: spi_writeword
 *
 * Description:
 *   Write 16 bit word from SPI DATA
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   word - 16 bit word to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_writeword(struct gd32_spidev_s *priv,
                                 uint16_t word)
{
  /* Wait until the transmit buffer is empty */

  spi_wait_status(priv, GD32_SPI_STAT_OFFSET, SPI_STAT_TBE);

  /* Then send the word */

  spi_putreg(priv, GD32_SPI_DATA_OFFSET, (uint32_t)word);
}

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI buses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI bus is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxmutex_lock(&priv->lock);
    }
  else
    {
      ret = nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t spi_setfrequency(struct spi_dev_s *dev,
                                 uint32_t frequency)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;

  uint16_t setbits;
  uint32_t actual;
  uint32_t plk_div;

  /* Check if the requested frequence is the same as the frequency
   * selection.
   */

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }

  /* Check if requested frequency reasonable */

  if (frequency > GD32_SPI_CLK_MAX)
    {
      frequency = GD32_SPI_CLK_MAX;
    }
  else if (frequency == 0)
    {
      frequency = GD32_SPI_CLK_INIT;
    }

  /* Configure SCK frequency.
   *
   *   SCK frequency = PCLK / plk_div, or plk_div = PCLK / frequency
   */

  plk_div = priv->spiclock / frequency;

  if (plk_div < 2)
    {
      plk_div = 2;
    }
  else if (plk_div > 256)
    {
      plk_div = 256;
    }

  if (plk_div > 128)
    {
      setbits = SPI_CTL0_PSC_256;
    }
  else if (plk_div > 64)
    {
      setbits = SPI_CTL0_PSC_128;
    }
  else if (plk_div > 32)
    {
      setbits = SPI_CTL0_PSC_64;
    }
  else if (plk_div > 16)
    {
      setbits = SPI_CTL0_PSC_32;
    }
  else if (plk_div > 8)
    {
      setbits = SPI_CTL0_PSC_16;
    }
  else if (plk_div > 4)
    {
      setbits = SPI_CTL0_PSC_8;
    }
  else if (plk_div > 2)
    {
      setbits = SPI_CTL0_PSC_4;
    }
  else
    {
      setbits = SPI_CTL0_PSC_2;
    }

  spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET, 0, SPI_CTL0_SPIEN);
  spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET, setbits, SPI_CTL0_PSC_MASK);
  spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET, SPI_CTL0_SPIEN, 0);

  actual = priv->spiclock >> ((setbits >> SPI_CTL0_PSC_SHIFT) + 1);

  /* Save the frequency selection so that subsequent reconfigurations
   * will be faster.
   */

  spiinfo("Frequency %" PRIu32 "->%" PRIu32 "\n", frequency, actual);

  priv->frequency = frequency;
  priv->actual    = actual;

  return priv->actual;
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;

  uint32_t regval0;
  uint32_t regval1;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set CTL0 appropriately */

      regval0 = spi_getreg(priv, GD32_SPI_CTL0_OFFSET);
      regval0 &= ~(SPI_CTL0_CKPH | SPI_CTL0_CKPL);
      regval1 = spi_getreg(priv, GD32_SPI_CTL1_OFFSET);
      regval1 &= ~ SPI_CTL1_TMOD;

      switch (mode)
        {
        case SPIDEV_MODE0: /* CKPL=0, CKPH=0 */
          break;

        case SPIDEV_MODE1: /* CKPL=0, CKPH=1 */
          regval0 |= SPI_CTL0_CKPH;
          break;

        case SPIDEV_MODE2: /* CKPL=1, CKPH=0 */
          regval0 |= SPI_CTL0_CKPL;
          break;

        case SPIDEV_MODE3: /* CKPL=1, CKPH=1 */
          regval0 |= (SPI_CTL0_CKPH | SPI_CTL0_CKPL);
          break;

#ifdef SPI_TMOD_EN    /* If MCU TI Synchronous Serial Frame Format enable */
        case SPIDEV_MODETI:
          regval0 = (SPI_CTL0_CKPH | SPI_CTL0_CKPL);
          regval1 | SPI_CTL1_TMOD;
          break;
#endif

        default:
          return;
        }

      spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET, 0, SPI_CTL0_SPIEN);
      spi_putreg(priv, GD32_SPI_CTL0_OFFSET, regval0);
      spi_putreg(priv, GD32_SPI_CTL1_OFFSET, regval1);
      spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET, SPI_CTL0_SPIEN, 0);

      /* Save the mode so that subsequent re-configurations will be
       * faster
       */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;

  uint32_t regval;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      /* Yes... Set the number of bits (16 or 8) */

      regval = spi_getreg(priv, GD32_SPI_CTL0_OFFSET);
      regval &= ~ SPI_CTL0_FF16;

      if (nbits == 16)
        {
          regval |= SPI_CTL0_FF16;
        }

      spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET, 0, SPI_CTL0_SPIEN);
      spi_putreg(priv, GD32_SPI_CTL0_OFFSET, regval);
      spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET, SPI_CTL0_SPIEN, 0);

      /* Save the selection so that subsequent re-configurations will be
       * faster.
       */

      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: spi_hwfeatures
 *
 * Description:
 *   Set hardware-specific feature flags.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   features - H/W feature flags
 *
 * Returned Value:
 *   Zero (OK) if the selected H/W features are enabled; A negated errno
 *   value if any H/W feature is not supportable.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int spi_hwfeatures(struct spi_dev_s *dev,
                          spi_hwfeatures_t features)
{
#if defined(CONFIG_SPI_BITORDER) || defined(CONFIG_SPI_TRIGGER)
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
#endif

#ifdef CONFIG_SPI_BITORDER
  uint16_t setbits;
  uint16_t clrbits;
  uint32_t regval;

  spiinfo("features=%08x\n", features);

  regval = spi_getreg(priv, GD32_SPI_CTL0_OFFSET);
  regval &= ~ SPI_CTL0_LF;

  /* Transfer data LSB first? */

  if ((features & HWFEAT_LSBFIRST) != 0)
    {
      regval |= SPI_CTL0_LF;
    }

  spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET, 0, SPI_CTL0_SPIEN);
  spi_putreg(priv, GD32_SPI_CTL0_OFFSET, regval);
  spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET, SPI_CTL0_SPIEN, 0);

  features &= ~HWFEAT_LSBFIRST;
#endif

  /* Other H/W features are not supported */

  return (features == 0) ? OK : -ENOSYS;
}
#endif

/****************************************************************************
 * Name: spi_send_data
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint32_t spi_send_data(struct gd32_spidev_s *priv, uint16_t wd)
{
  uint32_t regval;
  uint32_t ret;

  DEBUGASSERT(priv && priv->spibase);

  spi_writeword(priv, (uint32_t)(wd & 0xffff));
  ret = (uint32_t)spi_readword(priv);

  /* Check and clear any error flags
   * (Reading from the STAT clears the error flags)
   */

  regval = spi_getreg(priv, GD32_SPI_STAT_OFFSET);

  if ((regval&SPI_STAT_CRCERR) == SPI_STAT_CRCERR)
    {
      spi_modifyreg(priv, GD32_SPI_STAT_OFFSET, 0, SPI_STAT_CRCERR);
    }

  spiinfo("Sent: %02" PRIx16 " Return: %04" PRIx32
          " Status: %04" PRIx32 "\n", wd, ret, regval);
  UNUSED(regval);

  return ret;
}

/****************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;

  return (uint32_t)spi_send_data(priv, (uint16_t)wd);
}

/****************************************************************************
 * Name: spi_exchange (no DMA).  aka spi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 *   REVISIT:
 *   This function could be much more efficient by exploiting (1) RX and TX
 *   FIFOs and (2) the GD32F4 F3 data packing.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if !defined(CONFIG_GD32F4_SPI_DMA) || defined(CONFIG_GD32F4_SPI_DMATHRESHOLD)
#if !defined(CONFIG_GD32F4_SPI_DMA)
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
#else
static void spi_exchange_nodma(struct spi_dev_s *dev,
                               const void *txbuffer,
                               void *rxbuffer, size_t nwords)
#endif
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
  DEBUGASSERT(priv && priv->spibase);

  uint8_t         *brxptr = (uint8_t *)rxbuffer;
  const uint8_t   *btxptr = (uint8_t *)txbuffer;
  uint16_t        *wrxptr = (uint16_t *)rxbuffer;
  const uint16_t  *wtxptr = (const uint16_t *)txbuffer;
  uint8_t          byte;
  uint16_t         word;

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* 8- or 16-bit mode? */

  if (priv->nbits > 8)
    {
      /* 16-bit mode */

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (wtxptr)
            {
              word = *wtxptr++;
            }
          else
            {
              word = 0xffff;
            }

          /* Exchange one word */

          word = (uint16_t)spi_send(dev, (uint32_t)word);

          /* Is there a buffer to receive the return value? */

          if (wrxptr)
            {
              *wrxptr++ = word;
            }
        }
    }
  else
    {
      /* 8-bit mode */

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (btxptr)
            {
              byte = *btxptr++;
            }
          else
            {
              byte = 0xff;
            }

          /* Exchange one word */

          byte = (uint8_t)spi_send(dev, (uint32_t)byte);

          /* Is there a buffer to receive the return value? */

          if (brxptr)
            {
              *brxptr++ = byte;
            }
        }
    }
}
#endif /* !CONFIG_GD32F4_SPI_DMA || CONFIG_GD32F4_SPI_DMATHRESHOLD */

/****************************************************************************
 * Name: spi_exchange (with DMA capability)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_GD32F4_SPI_DMA
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
  void *xbuffer = rxbuffer;
  size_t nbytes;
  int ret;
  static uint16_t rxdummy = 0xffff;
  static const uint16_t txdummy = 0xffff;

  DEBUGASSERT(priv && priv->spibase);

  /* Convert the number of word to a number of bytes */

  nbytes = (priv->nbits > 8) ? nwords << 1 : nwords;

  if ((priv->rxdma == NULL) || (priv->txdma == NULL) ||
      up_interrupt_context()
#ifdef CONFIG_GD32F4_SPI_DMATHRESHOLD
  /* If this is a small SPI transfer, then let spi_exchange_nodma()
   * work do the.
   */

  || (nbytes <= CONFIG_GD32F4_SPI_DMATHRESHOLD)
#endif
      )
    {
      /* Invalid DMA channels, or interrupt context, fall
       * back to non-DMA method.
       */

      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }

    {
      spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n",
               txbuffer, rxbuffer, nwords);

      /* Setup DMAs */

      /* If this bus uses a in driver buffers we will incur 2 copies,
       * The copy cost is << less the non DMA transfer time and having
       * the buffer in the driver ensures DMA can be used. This is because
       * the API does not support passing the buffer extent so the only
       * extent is buffer + the transfer size. These can sizes be less than
       * the cache line size, and not aligned and typically greater then 4
       * bytes, which is about the break even point for the DMA IO overhead.
       */

      if (txbuffer && priv->txbuf)
        {
          if (nbytes > priv->buflen)
            {
              nbytes = priv->buflen;
            }

          memcpy(priv->txbuf, txbuffer, nbytes);
          txbuffer  = priv->txbuf;
          rxbuffer  = rxbuffer ? priv->rxbuf : rxbuffer;
        }

      spi_dmarxsetup(priv, rxbuffer, &rxdummy, nwords);
      spi_dmatxsetup(priv, txbuffer, &txdummy, nwords);

#ifdef CONFIG_SPI_TRIGGER
      /* Is deferred triggering in effect? */

      if (!priv->defertrig)
        {
          /* No.. Start the DMAs */

          spi_dmarxstart(priv);
          spi_dmatxstart(priv);
        }
      else
        {
          /* Yes.. indicated that we are ready to be started */

          priv->trigarmed = true;
        }
#else
      /* Start the DMAs */

      spi_dmarxstart(priv);
      spi_dmatxstart(priv);
#endif

      /* Then wait for each to complete */

      ret = spi_dmarxwait(priv);
      if (ret < 0)
        {
          ret = spi_dmatxwait(priv);
        }

      if (rxbuffer != NULL && priv->rxbuf != NULL && ret >= 0)
        {
          memcpy(xbuffer, priv->rxbuf, nbytes);
        }

#ifdef CONFIG_SPI_TRIGGER
      priv->trigarmed = false;
#endif
    }
}
#endif /* CONFIG_GD32F4_SPI_DMA */

/****************************************************************************
 * Name: spi_trigger
 *
 * Description:
 *   Trigger a previously configured DMA transfer.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   OK       - Trigger was fired
 *   ENOTSUP  - Trigger not fired due to lack of DMA support
 *   EIO      - Trigger not fired because not previously primed
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_TRIGGER
static int spi_trigger(struct spi_dev_s *dev)
{
#ifdef CONFIG_GD32F4_SPI_DMA
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;

  if (!priv->trigarmed)
    {
      return -EIO;
    }

  spi_dmarxstart(priv);
  spi_dmatxstart(priv);

  return OK;
#else
  return -ENOSYS;
#endif
}
#endif

/****************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of
 *              words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev,
                         const void *txbuffer,
                         size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return spi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that can be received in the buffer in
 *              number of words.  The wordsize is determined by the number
 *              of bits-per-word selected for the SPI interface.  If
 *              nbits <= 8, the data is packed into uint8_t's; if nbits >8,
 *              the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(struct spi_dev_s *dev,
                          void *rxbuffer,
                          size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

#ifdef CONFIG_GD32F4_SPI_DMA
static int spi_dmarxwait(struct gd32_spidev_s *priv)
{
  int ret;

  /* Take the semaphore (perhaps waiting).  If the result is zero, then the
   * DMA must not really have completed???
   */

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->rxsem);

      /* The only expected error is ECANCELED which would occur if the
       * calling thread were canceled.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (priv->rxresult == 0 && ret == OK);

  return ret;
}

/****************************************************************************
 * Name: spi_dmatxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

static int spi_dmatxwait(struct gd32_spidev_s *priv)
{
  int ret;

  /* Take the semaphore (perhaps waiting).  If the result is zero, then the
   * DMA must not really have completed???
   */

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->txsem);

      /* The only expected error is ECANCELED which would occur if the
       * calling thread were canceled.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (priv->txresult == 0 && ret == OK);

  return ret;
}

/****************************************************************************
 * Name: spi_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

static void spi_dmarxcallback(DMA_HANDLE handle, uint16_t isr, void *arg)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)arg;

  /* Wake-up the SPI driver */

  priv->rxresult = isr | 0x0080;  /* OR'ed with 0x0080 to assure non-zero */

  nxsem_post(&priv->txsem);
}

/****************************************************************************
 * Name: spi_dmatxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

static void spi_dmatxcallback(DMA_HANDLE handle, uint16_t isr, void *arg)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)arg;

  /* Wake-up the SPI driver */

  priv->txresult = isr | 0x0080;  /* OR'ed with 0x80 to assure non-zero */

  nxsem_post(&priv->txsem);
}

/****************************************************************************
 * Name: spi_dmarxsetup
 *
 * Description:
 *   Setup to perform RX DMA
 *
 ****************************************************************************/

static void spi_dmarxsetup(struct gd32_spidev_s *priv,
                           void *rxbuffer,
                           void *rxdummy, size_t nwords)
{
  uint32_t regval;
  dma_single_data_parameter_struct dma_init_struct;

  /* Enable SPI RX DMA */

  regval = spi_getreg(priv, GD32_SPI_CTL1_OFFSET);
  regval |= SPI_CTL1_DMAREN;
  spi_putreg(priv, GD32_SPI_CTL1_OFFSET, regval);

  /* 8- or 16-bit mode? */

  if (priv->nbits > 8)
    {
      /* 16-bit mode -- is there a buffer to receive data in? */

      if (rxbuffer)
        {
          dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
        }
      else
        {
          rxbuffer    = rxdummy;
          dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_DISABLE;
        }

        dma_init_struct.periph_memory_width = DMA_WIDTH_16BITS_SELECT;
    }
  else
    {
      /* 8-bit mode -- is there a buffer to receive data in? */

      if (rxbuffer)
        {
          dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
        }
      else
        {
          rxbuffer    = rxdummy;
          dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_DISABLE;
        }

        dma_init_struct.periph_memory_width = DMA_WIDTH_8BITS_SELECT;
    }

  dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
  dma_init_struct.memory0_addr = (uint32_t)rxbuffer;
  dma_init_struct.number = nwords;
  dma_init_struct.periph_addr = GD32_SPI_DATA(priv->spibase);
  dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma_init_struct.priority = SPI_DMA_PRIO;

  /* Configure the RX DMA */

  gd32_dma_setup(priv->rxdma, &dma_init_struct, 1);
}

/****************************************************************************
 * Name: spi_dmatxsetup
 *
 * Description:
 *   Setup to perform TX DMA
 *
 ****************************************************************************/

static void spi_dmatxsetup(struct gd32_spidev_s *priv,
                           const void *txbuffer,
                           const void *txdummy, size_t nwords)
{
  uint32_t regval;
  dma_single_data_parameter_struct dma_init_struct;

  /* Enable SPI TX DMA */

  regval = spi_getreg(priv, GD32_SPI_CTL1_OFFSET);
  regval |= SPI_CTL1_DMATEN;
  spi_putreg(priv, GD32_SPI_CTL1_OFFSET, regval);

  /* 8- or 16-bit mode? */

  if (priv->nbits > 8)
    {
      /* 16-bit mode -- is there a buffer to transfer data from? */

      if (txbuffer)
        {
          dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
        }
      else
        {
          txbuffer    = txdummy;
          dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_DISABLE;
        }

        dma_init_struct.periph_memory_width = DMA_WIDTH_16BITS_SELECT;
    }
  else
    {
      /* 8-bit mode -- is there a buffer to transfer data from? */

      if (txbuffer)
        {
          dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
        }
      else
        {
          txbuffer    = txdummy;
          dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_DISABLE;
        }

        dma_init_struct.periph_memory_width = DMA_WIDTH_8BITS_SELECT;
    }

  dma_init_struct.direction = DMA_MEMORY_TO_PERIPH;
  dma_init_struct.memory0_addr = (uint32_t)txbuffer;
  dma_init_struct.number = nwords;
  dma_init_struct.periph_addr = GD32_SPI_DATA(priv->spibase);
  dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma_init_struct.priority = SPI_DMA_PRIO;

  /* Setup the TX DMA */

  gd32_dma_setup(priv->txdma, &dma_init_struct, 1);
}

/****************************************************************************
 * Name: spi_dmarxstart
 *
 * Description:
 *   Start RX DMA
 *
 ****************************************************************************/

static inline void spi_dmarxstart(struct gd32_spidev_s *priv)
{
  priv->rxresult = 0;
  gd32_dma_start(priv->rxdma, spi_dmarxcallback, priv, SPI_DMA_INTEN);
}

/****************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start TX DMA
 *
 ****************************************************************************/

static inline void spi_dmatxstart(struct gd32_spidev_s *priv)
{
  priv->txresult = 0;
  gd32_dma_start(priv->txdma, spi_dmatxcallback, priv, SPI_DMA_INTEN);
}
#endif

/****************************************************************************
 * Name: spi_bus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus in its default state (Master, 8-bit,
 *   mode 0, etc.)
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_bus_initialize(struct gd32_spidev_s *priv)
{
  uint32_t regval;

  /* Configure CTL0. Default configuration:
   *   Mode 0:                     CKPH=0 and CKPL=0
   *   Master:                     SPI_MASTER
   *   8-bit:                      FF16=0
   *   MSB transmitted first:      LF=0
   *   NSS software mode:          SWNSSEN=1
   *   RECV and TRANS at duplex:   SPI_TRANSMODE_FULLDUPLEX
   *
   */

  regval = spi_getreg(priv, GD32_SPI_CTL0_OFFSET);
  regval &= SPI_INIT_MASK;
  regval |= (SPI_MASTER | SPI_CTL0_SWNSSEN);
  spi_putreg(priv, GD32_SPI_CTL0_OFFSET, regval);

  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((struct spi_dev_s *)priv, 400000);

#ifdef CONFIG_GD32F4_SPI_DMA
  if (priv->rxch && priv->txch)
    {
      if (priv->txdma == NULL && priv->rxdma == NULL)
        {
          /* Get DMA channels */

          priv->rxdma = gd32_dma_channel_alloc(priv->rxch);
          priv->txdma = gd32_dma_channel_alloc(priv->txch);
          DEBUGASSERT(priv->rxdma && priv->txdma);
        }
    }
  else
    {
      priv->rxdma = NULL;
      priv->txdma = NULL;
    }
#endif

  /* Enable SPI */

  spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET, SPI_CTL0_SPIEN, 0);
}

/****************************************************************************
 * Name: gd32_spi_gpio_config
 *
 * Description:
 *   Configure GPIO pins for SPI.
 *
 ****************************************************************************/

static void gd32_spi_gpio_config(uint32_t spibase)
{
  /* Determine which SPI to configure */

  switch (spibase)
    {
    default:
      break;
#ifdef CONFIG_GD32F4_SPI0
    case GD32_SPI0:

        /* Configure GPIO pins for SPI0: SCK, MISO, and MOSI */

        gd32_gpio_config(GPIO_SPI0_SCK_PIN);
        gd32_gpio_config(GPIO_SPI0_MISO_PIN);
        gd32_gpio_config(GPIO_SPI0_MOSI_PIN);
      break;
#endif
#ifdef CONFIG_GD32F4_SPI1
    case GD32_SPI1:

        /* Configure GPIO pins for SPI1: SCK, MISO, and MOSI */

        gd32_gpio_config(GPIO_SPI1_SCK_PIN);
        gd32_gpio_config(GPIO_SPI1_MISO_PIN);
        gd32_gpio_config(GPIO_SPI1_MOSI_PIN);
      break;
#endif
#ifdef CONFIG_GD32F4_SPI2
    case GD32_SPI2:

        /* Configure GPIO pins for SPI2: SCK, MISO, and MOSI */

        gd32_gpio_config(GPIO_SPI2_SCK_PIN);
        gd32_gpio_config(GPIO_SPI2_MISO_PIN);
        gd32_gpio_config(GPIO_SPI2_MOSI_PIN);
      break;
#endif
#ifdef CONFIG_GD32F4_SPI3
    case GD32_SPI3:

        /* Configure GPIO pins for SPI3: SCK, MISO, and MOSI */

        gd32_gpio_config(GPIO_SPI3_SCK_PIN);
        gd32_gpio_config(GPIO_SPI3_MISO_PIN);
        gd32_gpio_config(GPIO_SPI3_MOSI_PIN);
      break;
#endif
#ifdef CONFIG_GD32F4_SPI4
    case GD32_SPI4:

        /* Configure GPIO pins for SPI4: SCK, MISO, and MOSI */

        gd32_gpio_config(GPIO_SPI4_SCK_PIN);
        gd32_gpio_config(GPIO_SPI4_MISO_PIN);
        gd32_gpio_config(GPIO_SPI4_MOSI_PIN);
      break;
#endif
#ifdef CONFIG_GD32F4_SPI5
    case GD32_SPI5:

        /* Configure GPIO pins for SPI5: SCK, MISO, and MOSI */

        gd32_gpio_config(GPIO_SPI5_SCK_PIN);
        gd32_gpio_config(GPIO_SPI5_MISO_PIN);
        gd32_gpio_config(GPIO_SPI5_MOSI_PIN);
        gd32_gpio_config(GPIO_SPI5_IO2_PIN);
        gd32_gpio_config(GPIO_SPI5_IO3_PIN);
      break;
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *gd32_spibus_initialize(int port)
{
  struct gd32_spidev_s *priv = NULL;

  /* Configure multiplexed pins as connected on the board.  Chip select pins
   * must be configured by board-specific logic.  Most SPI pins multiple,
   * alternative pin selection.  Definitions in the board.h file must be\
   * provided to resolve the board-specific pin configuration like:
   *
   * #define PIN_SPI0_SCK PIN_SPI0_SCK_1
   */

  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_GD32F4_SPI0
  if (port == 0)
    {
      /* Select SPI0 */

      priv = &g_spi0dev;
    }
  else
#endif
#ifdef CONFIG_GD32F4_SPI1
  if (port == 1)
    {
      /* Select SPI1 */

      priv = &g_spi1dev;
    }
  else
#endif
#ifdef CONFIG_GD32F4_SPI2
  if (port == 2)
    {
      /* Select SPI2 */
    }
  else
#endif
#ifdef CONFIG_GD32F4_SPI3
  if (port == 3)
    {
      /* Select SPI3 */

      priv = &g_spi3dev;
    }
  else
#endif
#ifdef CONFIG_GD32F4_SPI4
  if (port == 4)
    {
      /* Select SPI4 */

      priv = &g_spi4dev;
    }
  else
#endif
#ifdef CONFIG_GD32F4_SPI5
  if (port == 5)
    {
      /* Select SPI5 */

      priv = &g_spi5dev;
    }
  else
#endif
    {
      spierr("ERROR: Unsupported SPI port: %d\n", port);
    }

  /* Check if the port is seclected and has not been configured */

  if ((priv != NULL) && (!priv->initialized))
    {
      /* Configure GPIO pins for SPI */

      gd32_spi_gpio_config(priv->spibase);

      /* Enable clock for SPI */

      gd32_spi_clock_enable(priv->spibase);

      /* Set up default configuration */

      spi_bus_initialize(priv);

      priv->initialized = true;
    }

  leave_critical_section(flags);

  return (struct spi_dev_s *)priv;
}

/****************************************************************************
 * Name: gd32_spibus_deinitialize
 *
 * Description:
 *   Deinitialize the selected SPI port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 ****************************************************************************/

void gd32_spibus_deinitialize(int port)
{
  struct gd32_spidev_s *priv = NULL;

#ifdef CONFIG_GD32F4_SPI0
  if (port == 0)
    {
      /* Select SPI0 */

      priv = &g_spi0dev;
    }
  else
#endif
#ifdef CONFIG_GD32F4_SPI1
  if (port == 1)
    {
      /* Select SPI1 */

      priv = &g_spi1dev;
    }
  else
#endif
#ifdef CONFIG_GD32F4_SPI2
  if (port == 2)
    {
      /* Select SPI2 */

      priv = &g_spi2dev;
    }
  else
#endif
#ifdef CONFIG_GD32F4_SPI3
  if (port == 3)
    {
      /* Select SPI3 */

      priv = &g_spi3dev;
    }
  else
#endif
#ifdef CONFIG_GD32F4_SPI4
  if (port == 4)
    {
      /* Select SPI4 */

      priv = &g_spi4dev;
    }
  else
#endif
#ifdef CONFIG_GD32F4_SPI5
  if (port == 5)
    {
      /* Select SPI5 */

      priv = &g_spi5dev;
    }
  else
#endif
    {
      spierr("ERROR: Unsupported SPI port: %d\n", port);
    }

  /* Check if the port is seclected and has been configured */

  if ((priv != NULL) && (priv->initialized))
    {
      gd32_spi_clock_disable(priv->spibase);

      gd32_spi_reset(priv->spibase);

      priv->initialized = false;
    }
}

#endif /* CONFIG_GD32F4_SPI0 || CONFIG_GD32F4_SPI1 || CONFIG_GD32F4_SPI2 ||
        * CONFIG_GD32F4_SPI3 || CONFIG_GD32F4_SPI4 || CONFIG_GD32F4_SPI5
        */
