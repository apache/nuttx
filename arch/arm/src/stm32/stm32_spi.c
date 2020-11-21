/************************************************************************************
 * arch/arm/src/stm32/stm32_spi.c
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
 ************************************************************************************/

/************************************************************************************
 * The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status must be
 * provided by board-specific logic.  They are implementations of the select
 * and status methods of the SPI interface defined by struct spi_ops_s (see
 * include/nuttx/spi/spi.h). All other methods (including stm32_spibus_initialize())
 * are provided by common STM32 logic.  To use this common SPI logic on your
 * board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_dma.h"
#include "stm32_spi.h"

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || \
    defined(CONFIG_STM32_SPI3) || defined(CONFIG_STM32_SPI4) || \
    defined(CONFIG_STM32_SPI5) || defined(CONFIG_STM32_SPI6)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

/* SPI interrupts */

#ifdef CONFIG_STM32_SPI_INTERRUPTS
#  error "Interrupt driven SPI not yet supported"
#endif

/* Can't have both interrupt driven SPI and SPI DMA */

#if defined(CONFIG_STM32_SPI_INTERRUPTS) && defined(CONFIG_STM32_SPI_DMA)
#  error "Cannot enable both interrupt mode and DMA mode for SPI"
#endif

/* SPI DMA priority */

#ifdef CONFIG_STM32_SPI_DMA

#  if defined(CONFIG_SPI_DMAPRIO)
#    define SPI_DMA_PRIO  CONFIG_SPI_DMAPRIO
#  elif defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32L15XX) || \
        defined(CONFIG_STM32_STM32F30XX)
#    define SPI_DMA_PRIO  DMA_CCR_PRIMED
#  elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#    define SPI_DMA_PRIO  DMA_SCR_PRIMED
#  else
#    error "Unknown STM32 DMA"
#  endif

#  if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32L15XX) || \
      defined(CONFIG_STM32_STM32F30XX)
#    if (SPI_DMA_PRIO & ~DMA_CCR_PL_MASK) != 0
#      error "Illegal value for CONFIG_SPI_DMAPRIO"
#    endif
#  elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#    if (SPI_DMA_PRIO & ~DMA_SCR_PL_MASK) != 0
#      error "Illegal value for CONFIG_SPI_DMAPRIO"
#    endif
#  else
#    error "Unknown STM32 DMA"
#  endif

/* DMA channel configuration */

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32L15XX) || \
    defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
#  define SPI_RXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_CCR_MSIZE_16BITS|DMA_CCR_PSIZE_16BITS|DMA_CCR_MINC            )
#  define SPI_RXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_8BITS |DMA_CCR_MINC            )
#  define SPI_RXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_16BITS                         )
#  define SPI_RXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_8BITS                          )
#  define SPI_TXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_CCR_MSIZE_16BITS|DMA_CCR_PSIZE_16BITS|DMA_CCR_MINC|DMA_CCR_DIR)
#  define SPI_TXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_8BITS |DMA_CCR_MINC|DMA_CCR_DIR)
#  define SPI_TXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_16BITS             |DMA_CCR_DIR)
#  define SPI_TXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_8BITS              |DMA_CCR_DIR)
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#  define SPI_RXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_SCR_MSIZE_16BITS|DMA_SCR_PSIZE_16BITS|DMA_SCR_MINC|DMA_SCR_DIR_P2M)
#  define SPI_RXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS |DMA_SCR_MINC|DMA_SCR_DIR_P2M)
#  define SPI_RXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_16BITS             |DMA_SCR_DIR_P2M)
#  define SPI_RXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS              |DMA_SCR_DIR_P2M)
#  define SPI_TXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_SCR_MSIZE_16BITS|DMA_SCR_PSIZE_16BITS|DMA_SCR_MINC|DMA_SCR_DIR_M2P)
#  define SPI_TXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS |DMA_SCR_MINC|DMA_SCR_DIR_M2P)
#  define SPI_TXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_16BITS             |DMA_SCR_DIR_M2P)
#  define SPI_TXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS              |DMA_SCR_DIR_M2P)
#else
#  error "Unknown STM32 DMA"
#endif

#  define SPIDMA_BUFFER_MASK   (4 - 1)
#  define SPIDMA_SIZE(b) (((b) + SPIDMA_BUFFER_MASK) & ~SPIDMA_BUFFER_MASK)
#  define SPIDMA_BUF_ALIGN   aligned_data(4)

#  if defined(CONFIG_STM32_SPI1_DMA_BUFFER) && \
            CONFIG_STM32_SPI1_DMA_BUFFER > 0
#    define SPI1_DMABUFSIZE_ADJUSTED SPIDMA_SIZE(CONFIG_STM32_SPI1_DMA_BUFFER)
#    define SPI1_DMABUFSIZE_ALGN SPIDMA_BUF_ALIGN
#  endif

#  if defined(CONFIG_STM32_SPI2_DMA_BUFFER) && \
            CONFIG_STM32_SPI2_DMA_BUFFER > 0
#    define SPI2_DMABUFSIZE_ADJUSTED SPIDMA_SIZE(CONFIG_STM32_SPI2_DMA_BUFFER)
#    define SPI2_DMABUFSIZE_ALGN SPIDMA_BUF_ALIGN
#  endif

#  if defined(CONFIG_STM32_SPI3_DMA_BUFFER) && \
            CONFIG_STM32_SPI3_DMA_BUFFER > 0
#    define SPI3_DMABUFSIZE_ADJUSTED SPIDMA_SIZE(CONFIG_STM32_SPI3_DMA_BUFFER)
#    define SPI3_DMABUFSIZE_ALGN SPIDMA_BUF_ALIGN
#  endif

#  if defined(CONFIG_STM32_SPI4_DMA_BUFFER) && \
            CONFIG_STM32_SPI4_DMA_BUFFER > 0
#    define SPI4_DMABUFSIZE_ADJUSTED SPIDMA_SIZE(CONFIG_STM32_SPI4_DMA_BUFFER)
#    define SPI4_DMABUFSIZE_ALGN SPIDMA_BUF_ALIGN
#  endif

#  if defined(CONFIG_STM32_SPI5_DMA_BUFFER) && \
            CONFIG_STM32_SPI5_DMA_BUFFER > 0
#    define SPI5_DMABUFSIZE_ADJUSTED SPIDMA_SIZE(CONFIG_STM32_SPI5_DMA_BUFFER)
#    define SPI5_DMABUFSIZE_ALGN SPIDMA_BUF_ALIGN
#  endif

#  if defined(CONFIG_STM32_SPI6_DMA_BUFFER) && \
            CONFIG_STM32_SPI6_DMA_BUFFER > 0
#    define SPI6_DMABUFSIZE_ADJUSTED SPIDMA_SIZE(CONFIG_STM32_SPI6_DMA_BUFFER)
#    define SPI6_DMABUFSIZE_ALGN SPIDMA_BUF_ALIGN
#  endif

#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct stm32_spidev_s
{
  struct spi_dev_s spidev;       /* Externally visible part of the SPI interface */
  uint32_t         spibase;      /* SPIn base address */
  uint32_t         spiclock;     /* Clocking for the SPI module */
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  uint8_t          spiirq;       /* SPI IRQ number */
#endif
#ifdef CONFIG_STM32_SPI_DMA
  volatile uint8_t rxresult;     /* Result of the RX DMA */
  volatile uint8_t txresult;     /* Result of the RX DMA */
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
  uint32_t         txccr;        /* DMA control register for TX transfers */
  uint32_t         rxccr;        /* DMA control register for RX transfers */
#endif
  bool             initialized;  /* Has SPI interface been initialized */
  sem_t            exclsem;      /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;    /* Requested clock frequency */
  uint32_t         actual;       /* Actual clock frequency */
  uint8_t          nbits;        /* Width of word in bits (4 through 16) */
  uint8_t          mode;         /* Mode 0,1,2,3 */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Helpers */

static inline uint16_t spi_getreg(FAR struct stm32_spidev_s *priv, uint8_t offset);
#if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
static inline uint8_t spi_getreg8(FAR struct stm32_spidev_s *priv, uint8_t offset);
#endif
static inline void spi_putreg(FAR struct stm32_spidev_s *priv, uint8_t offset,
                                 uint16_t value);
#if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
static inline void spi_putreg8(FAR struct stm32_spidev_s *priv, uint8_t offset,
                                 uint8_t value);
#endif
static inline uint16_t spi_readword(FAR struct stm32_spidev_s *priv);
static inline void spi_writeword(FAR struct stm32_spidev_s *priv, uint16_t byte);

/* DMA support */

#ifdef CONFIG_STM32_SPI_DMA
static int         spi_dmarxwait(FAR struct stm32_spidev_s *priv);
static int         spi_dmatxwait(FAR struct stm32_spidev_s *priv);
static inline void spi_dmarxwakeup(FAR struct stm32_spidev_s *priv);
static inline void spi_dmatxwakeup(FAR struct stm32_spidev_s *priv);
static void        spi_dmarxcallback(DMA_HANDLE handle, uint8_t isr, void *arg);
static void        spi_dmatxcallback(DMA_HANDLE handle, uint8_t isr, void *arg);
static void        spi_dmarxsetup(FAR struct stm32_spidev_s *priv,
                                  FAR void *rxbuffer,
                                  FAR void *rxdummy,
                                  size_t nwords);
static void        spi_dmatxsetup(FAR struct stm32_spidev_s *priv,
                                  FAR const void *txbuffer,
                                  FAR const void *txdummy,
                                  size_t nwords);
static inline void spi_dmarxstart(FAR struct stm32_spidev_s *priv);
static inline void spi_dmatxstart(FAR struct stm32_spidev_s *priv);
#endif

/* SPI methods */

static int         spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t    spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void        spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void        spi_setbits(FAR struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int         spi_hwfeatures(FAR struct spi_dev_s *dev,
                                  spi_hwfeatures_t features);
#endif
static uint32_t    spi_send(FAR struct spi_dev_s *dev, uint32_t wd);
static void        spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                                FAR void *rxbuffer, size_t nwords);
#ifdef CONFIG_SPI_TRIGGER
static int         spi_trigger(FAR struct spi_dev_s *dev);
#endif
#ifndef CONFIG_SPI_EXCHANGE
static void        spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                                size_t nwords);
static void        spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
                                 size_t nwords);
#endif

/* Initialization */

static void        spi_bus_initialize(FAR struct stm32_spidev_s *priv);

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI1
static const struct spi_ops_s g_sp1iops =
{
  .lock              = spi_lock,
  .select            = stm32_spi1select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = stm32_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = stm32_spi1cmddata,
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
  .registercallback  = stm32_spi1register,  /* Provided externally */
#else
  .registercallback  = 0,                   /* Not implemented */
#endif
};

#if defined(SPI1_DMABUFSIZE_ADJUSTED)
static uint8_t g_spi1_txbuf[SPI1_DMABUFSIZE_ADJUSTED] SPI1_DMABUFSIZE_ALGN;
static uint8_t g_spi1_rxbuf[SPI1_DMABUFSIZE_ADJUSTED] SPI1_DMABUFSIZE_ALGN;
#endif

static struct stm32_spidev_s g_spi1dev =
{
  .spidev   =
              {
               &g_sp1iops
              },
  .spibase  = STM32_SPI1_BASE,
  .spiclock = STM32_PCLK2_FREQUENCY,
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  .spiirq   = STM32_IRQ_SPI1,
#endif
#ifdef CONFIG_STM32_SPI_DMA
#  ifdef CONFIG_STM32_SPI1_DMA
  .rxch     = DMACHAN_SPI1_RX,
  .txch     = DMACHAN_SPI1_TX,
#if defined(SPI1_DMABUFSIZE_ADJUSTED)
  .rxbuf    = g_spi1_rxbuf,
  .txbuf    = g_spi1_txbuf,
  .buflen   = SPI1_DMABUFSIZE_ADJUSTED,
#    endif
#  else
  .rxch     = 0,
  .txch     = 0,
#  endif
#endif
};
#endif

#ifdef CONFIG_STM32_SPI2
static const struct spi_ops_s g_sp2iops =
{
  .lock              = spi_lock,
  .select            = stm32_spi2select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = stm32_spi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = stm32_spi2cmddata,
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
  .registercallback  = stm32_spi2register,  /* provided externally */
#else
  .registercallback  = 0,                   /* not implemented */
#endif
};

#if defined(SPI2_DMABUFSIZE_ADJUSTED)
static uint8_t g_spi2_txbuf[SPI2_DMABUFSIZE_ADJUSTED] SPI2_DMABUFSIZE_ALGN;
static uint8_t g_spi2_rxbuf[SPI2_DMABUFSIZE_ADJUSTED] SPI2_DMABUFSIZE_ALGN;
#endif

static struct stm32_spidev_s g_spi2dev =
{
  .spidev   =
              {
               &g_sp2iops
              },
  .spibase  = STM32_SPI2_BASE,
  .spiclock = STM32_PCLK1_FREQUENCY,
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  .spiirq   = STM32_IRQ_SPI2,
#endif
#ifdef CONFIG_STM32_SPI_DMA
#  ifdef CONFIG_STM32_SPI2_DMA
  .rxch     = DMACHAN_SPI2_RX,
  .txch     = DMACHAN_SPI2_TX,
#if defined(SPI2_DMABUFSIZE_ADJUSTED)
  .rxbuf    = g_spi2_rxbuf,
  .txbuf    = g_spi2_txbuf,
  .buflen   = SPI2_DMABUFSIZE_ADJUSTED,
#    endif
#  else
  .rxch     = 0,
  .txch     = 0,
#  endif
#endif
};
#endif

#ifdef CONFIG_STM32_SPI3
static const struct spi_ops_s g_sp3iops =
{
  .lock              = spi_lock,
  .select            = stm32_spi3select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = stm32_spi3status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = stm32_spi3cmddata,
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
  .registercallback  = stm32_spi3register,  /* provided externally */
#else
  .registercallback  = 0,                   /* not implemented */
#endif
};

#if defined(SPI3_DMABUFSIZE_ADJUSTED)
static uint8_t g_spi3_txbuf[SPI3_DMABUFSIZE_ADJUSTED] SPI3_DMABUFSIZE_ALGN;
static uint8_t g_spi3_rxbuf[SPI3_DMABUFSIZE_ADJUSTED] SPI3_DMABUFSIZE_ALGN;
#endif

static struct stm32_spidev_s g_spi3dev =
{
  .spidev   =
              {
               &g_sp3iops
              },
  .spibase  = STM32_SPI3_BASE,
  .spiclock = STM32_PCLK1_FREQUENCY,
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  .spiirq   = STM32_IRQ_SPI3,
#endif
#ifdef CONFIG_STM32_SPI_DMA
#  ifdef CONFIG_STM32_SPI3_DMA
  .rxch     = DMACHAN_SPI3_RX,
  .txch     = DMACHAN_SPI3_TX,
#if defined(SPI3_DMABUFSIZE_ADJUSTED)
  .rxbuf    = g_spi3_rxbuf,
  .txbuf    = g_spi3_txbuf,
  .buflen   = SPI3_DMABUFSIZE_ADJUSTED,
#    endif
#  else
  .rxch     = 0,
  .txch     = 0,
#  endif
#endif
};
#endif

#ifdef CONFIG_STM32_SPI4
static const struct spi_ops_s g_sp4iops =
{
  .lock              = spi_lock,
  .select            = stm32_spi4select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = stm32_spi4status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = stm32_spi4cmddata,
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
  .registercallback  = stm32_spi4register,  /* provided externally */
#else
  .registercallback  = 0,                   /* not implemented */
#endif
};

#if defined(SPI4_DMABUFSIZE_ADJUSTED)
static uint8_t g_spi4_txbuf[SPI4_DMABUFSIZE_ADJUSTED] SPI4_DMABUFSIZE_ALGN;
static uint8_t g_spi4_rxbuf[SPI4_DMABUFSIZE_ADJUSTED] SPI4_DMABUFSIZE_ALGN;
#endif

static struct stm32_spidev_s g_spi4dev =
{
  .spidev   =
              {
               &g_sp4iops
              },
  .spibase  = STM32_SPI4_BASE,
  .spiclock = STM32_PCLK2_FREQUENCY,
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  .spiirq   = STM32_IRQ_SPI4,
#endif
#ifdef CONFIG_STM32_SPI_DMA
#  ifdef CONFIG_STM32_SPI4_DMA
  .rxch     = DMACHAN_SPI4_RX,
  .txch     = DMACHAN_SPI4_TX,
#if defined(SPI4_DMABUFSIZE_ADJUSTED)
  .rxbuf    = g_spi4_rxbuf,
  .txbuf    = g_spi4_txbuf,
  .buflen   = SPI4_DMABUFSIZE_ADJUSTED,
#    endif
#  else
  .rxch     = 0,
  .txch     = 0,
#  endif
#endif
};
#endif

#ifdef CONFIG_STM32_SPI5
static const struct spi_ops_s g_sp5iops =
{
  .lock              = spi_lock,
  .select            = stm32_spi5select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = stm32_spi5status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = stm32_spi5cmddata,
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
  .registercallback  = stm32_spi5register,  /* provided externally */
#else
  .registercallback  = 0,                   /* not implemented */
#endif
};

#if defined(SPI5_DMABUFSIZE_ADJUSTED)
static uint8_t g_spi5_txbuf[SPI5_DMABUFSIZE_ADJUSTED] SPI5_DMABUFSIZE_ALGN;
static uint8_t g_spi5_rxbuf[SPI5_DMABUFSIZE_ADJUSTED] SPI5_DMABUFSIZE_ALGN;
#endif

static struct stm32_spidev_s g_spi5dev =
{
  .spidev   =
              {
               &g_sp5iops
              },
  .spibase  = STM32_SPI5_BASE,
  .spiclock = STM32_PCLK2_FREQUENCY,
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  .spiirq   = STM32_IRQ_SPI5,
#endif
#ifdef CONFIG_STM32_SPI_DMA
#  ifdef CONFIG_STM32_SPI5_DMA
  .rxch     = DMACHAN_SPI5_RX,
  .txch     = DMACHAN_SPI5_TX,
#if defined(SPI5_DMABUFSIZE_ADJUSTED)
  .rxbuf    = g_spi5_rxbuf,
  .txbuf    = g_spi5_txbuf,
  .buflen   = SPI5_DMABUFSIZE_ADJUSTED,
#    endif
#  else
  .rxch     = 0,
  .txch     = 0,
#  endif
#endif
};
#endif

#ifdef CONFIG_STM32_SPI6
static const struct spi_ops_s g_sp6iops =
{
  .lock              = spi_lock,
  .select            = stm32_spi6select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = stm32_spi6status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = stm32_spi6cmddata,
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
  .registercallback  = stm32_spi6register,  /* provided externally */
#else
  .registercallback  = 0,                   /* not implemented */
#endif
};

#if defined(SPI6_DMABUFSIZE_ADJUSTED)
static uint8_t g_spi6_txbuf[SPI6_DMABUFSIZE_ADJUSTED] SPI6_DMABUFSIZE_ALGN;
static uint8_t g_spi6_rxbuf[SPI6_DMABUFSIZE_ADJUSTED] SPI6_DMABUFSIZE_ALGN;
#endif

static struct stm32_spidev_s g_spi6dev =
{
  .spidev   =
              {
               &g_sp6iops
              },
  .spibase  = STM32_SPI6_BASE,
  .spiclock = STM32_PCLK2_FREQUENCY,
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  .spiirq   = STM32_IRQ_SPI6,
#endif
#ifdef CONFIG_STM32_SPI_DMA
#  ifdef CONFIG_STM32_SPI6_DMA
  .rxch     = DMACHAN_SPI6_RX,
  .txch     = DMACHAN_SPI6_TX,
#if defined(SPI6_DMABUFSIZE_ADJUSTED)
  .rxbuf    = g_spi6_rxbuf,
  .txbuf    = g_spi6_txbuf,
  .buflen   = SPI6_DMABUFSIZE_ADJUSTED,
#    endif
#  else
  .rxch     = 0,
  .txch     = 0,
#  endif
#endif
};
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
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
 *   The contents of the 16-bit register
 *
 ************************************************************************************/

static inline uint16_t spi_getreg(FAR struct stm32_spidev_s *priv, uint8_t offset)
{
  return getreg16(priv->spibase + offset);
}

/************************************************************************************
 * Name: spi_getreg8
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ************************************************************************************/

#if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
static inline uint8_t spi_getreg8(FAR struct stm32_spidev_s *priv, uint8_t offset)
{
  return getreg8(priv->spibase + offset);
}
#endif

/************************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a 16-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ************************************************************************************/

static inline void spi_putreg(FAR struct stm32_spidev_s *priv,
                              uint8_t offset,
                              uint16_t value)
{
  putreg16(value, priv->spibase + offset);
}

/************************************************************************************
 * Name: spi_putreg8
 *
 * Description:
 *   Write an 8-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ************************************************************************************/

#if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
static inline void spi_putreg8(FAR struct stm32_spidev_s *priv, uint8_t offset,
                                 uint8_t value)
{
  putreg8(value, priv->spibase + offset);
}
#endif

/************************************************************************************
 * Name: spi_readword
 *
 * Description:
 *   Read one byte from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   Byte as read
 *
 ************************************************************************************/

static inline uint16_t spi_readword(FAR struct stm32_spidev_s *priv)
{
  /* Wait until the receive buffer is not empty */

  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_RXNE) == 0)
    {
    }

  /* Then return the received byte */

#if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
  /* "When the data frame size fits into one byte (less than or equal to 8 bits),
   *  data packing is used automatically when any read or write 16-bit access is
   *  performed on the SPIx_DR register. The double data frame pattern is handled
   *  in parallel in this case. At first, the SPI operates using the pattern
   *  stored in the LSB of the accessed word, then with the other half stored in
   *  the MSB.... The receiver then has to access both data frames by a single
   *  16-bit read of SPIx_DR as a response to this single RXNE event. The RxFIFO
   *  threshold setting and the following read access must be always kept aligned
   *  at the receiver side, as data can be lost if it is not in line."
   */

  if (priv->nbits < 9)
    {
      return (uint16_t)spi_getreg8(priv, STM32_SPI_DR_OFFSET);
    }
  else
#endif
    {
      return spi_getreg(priv, STM32_SPI_DR_OFFSET);
    }
}

/************************************************************************************
 * Name: spi_writeword
 *
 * Description:
 *   Write one byte to SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   byte - Byte to send
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void spi_writeword(FAR struct stm32_spidev_s *priv, uint16_t word)
{
  /* Wait until the transmit buffer is empty */

  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_TXE) == 0)
    {
    }

  /* Then send the word */

#if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
  /* "When the data frame size fits into one byte (less than or equal to 8 bits),
   *  data packing is used automatically when any read or write 16-bit access is
   *  performed on the SPIx_DR register. The double data frame pattern is handled
   *  in parallel in this case. At first, the SPI operates using the pattern
   *  stored in the LSB of the accessed word, then with the other half stored in
   *  the MSB...
   *
   *  "A specific problem appears if an odd number of such "fit into one byte"
   *   data frames must be handled. On the transmitter side, writing the last
   *   data frame of any odd sequence with an 8-bit access to SPIx_DR is enough.
   *   ..."
   *
   * REVISIT: "...The receiver has to change the Rx_FIFO threshold level for the
   * last data frame received in the odd sequence of frames in order to generate
   * the RXNE event."
   */

  if (priv->nbits < 9)
    {
      spi_putreg8(priv, STM32_SPI_DR_OFFSET, (uint8_t)word);
    }
  else
#endif
    {
      spi_putreg(priv, STM32_SPI_DR_OFFSET, word);
    }
}

/************************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static int spi_dmarxwait(FAR struct stm32_spidev_s *priv)
{
  int ret;

  /* Take the semaphore (perhaps waiting).  If the result is zero, then the DMA
   * must not really have completed???
   */

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->rxsem);

      /* The only expected error is ECANCELED which would occur if the calling
       * thread were canceled.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (priv->rxresult == 0 && ret == OK);

  return ret;
}
#endif

/************************************************************************************
 * Name: spi_dmatxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static int spi_dmatxwait(FAR struct stm32_spidev_s *priv)
{
  int ret;

  /* Take the semaphore (perhaps waiting).  If the result is zero, then the DMA
   * must not really have completed???
   */

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->txsem);

      /* The only expected error is ECANCELED which would occur if the calling
       * thread were canceled.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (priv->txresult == 0 && ret == OK);

  return ret;
}
#endif

/************************************************************************************
 * Name: spi_dmarxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static inline void spi_dmarxwakeup(FAR struct stm32_spidev_s *priv)
{
  nxsem_post(&priv->rxsem);
}
#endif

/************************************************************************************
 * Name: spi_dmatxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static inline void spi_dmatxwakeup(FAR struct stm32_spidev_s *priv)
{
  nxsem_post(&priv->txsem);
}
#endif

/************************************************************************************
 * Name: spi_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static void spi_dmarxcallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)arg;

  /* Wake-up the SPI driver */

  priv->rxresult = isr | 0x080;  /* OR'ed with 0x80 to assure non-zero */
  spi_dmarxwakeup(priv);
}
#endif

/************************************************************************************
 * Name: spi_dmatxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static void spi_dmatxcallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)arg;

  /* Wake-up the SPI driver */

  priv->txresult = isr | 0x080;  /* OR'ed with 0x80 to assure non-zero */
  spi_dmatxwakeup(priv);
}
#endif

/************************************************************************************
 * Name: spi_dmarxsetup
 *
 * Description:
 *   Setup to perform RX DMA
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static void spi_dmarxsetup(FAR struct stm32_spidev_s *priv, FAR void *rxbuffer,
                           FAR void *rxdummy, size_t nwords)
{
  /* 8- or 16-bit mode? */

  if (priv->nbits > 8)
    {
      /* 16-bit mode -- is there a buffer to receive data in? */

      if (rxbuffer)
        {
          priv->rxccr = SPI_RXDMA16_CONFIG;
        }
      else
        {
          rxbuffer    = rxdummy;
          priv->rxccr = SPI_RXDMA16NULL_CONFIG;
        }
    }
  else
    {
      /* 8-bit mode -- is there a buffer to receive data in? */

      if (rxbuffer)
        {
          priv->rxccr = SPI_RXDMA8_CONFIG;
        }
      else
        {
          rxbuffer    = rxdummy;
          priv->rxccr = SPI_RXDMA8NULL_CONFIG;
        }
    }

  /* Configure the RX DMA */

  stm32_dmasetup(priv->rxdma, priv->spibase + STM32_SPI_DR_OFFSET,
                 (uint32_t)rxbuffer, nwords, priv->rxccr);
}
#endif

/************************************************************************************
 * Name: spi_dmatxsetup
 *
 * Description:
 *   Setup to perform TX DMA
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static void spi_dmatxsetup(FAR struct stm32_spidev_s *priv, FAR const void *txbuffer,
                           FAR const void *txdummy, size_t nwords)
{
  /* 8- or 16-bit mode? */

  if (priv->nbits > 8)
    {
      /* 16-bit mode -- is there a buffer to transfer data from? */

      if (txbuffer)
        {
          priv->txccr = SPI_TXDMA16_CONFIG;
        }
      else
        {
          txbuffer    = txdummy;
          priv->txccr = SPI_TXDMA16NULL_CONFIG;
        }
    }
  else
    {
      /* 8-bit mode -- is there a buffer to transfer data from? */

      if (txbuffer)
        {
          priv->txccr = SPI_TXDMA8_CONFIG;
        }
      else
        {
          txbuffer    = txdummy;
          priv->txccr = SPI_TXDMA8NULL_CONFIG;
        }
    }

  /* Setup the TX DMA */

  stm32_dmasetup(priv->txdma, priv->spibase + STM32_SPI_DR_OFFSET,
                 (uint32_t)txbuffer, nwords, priv->txccr);
}
#endif

/************************************************************************************
 * Name: spi_dmarxstart
 *
 * Description:
 *   Start RX DMA
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static inline void spi_dmarxstart(FAR struct stm32_spidev_s *priv)
{
  priv->rxresult = 0;
  stm32_dmastart(priv->rxdma, spi_dmarxcallback, priv, false);
}
#endif

/************************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start TX DMA
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static inline void spi_dmatxstart(FAR struct stm32_spidev_s *priv)
{
  priv->txresult = 0;
  stm32_dmastart(priv->txdma, spi_dmatxcallback, priv, false);
}
#endif

/************************************************************************************
 * Name: spi_modifycr1
 *
 * Description:
 *   Clear and set bits in the CR1 register
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_modifycr1(FAR struct stm32_spidev_s *priv, uint16_t setbits,
                          uint16_t clrbits)
{
  uint16_t cr1;
  cr1 = spi_getreg(priv, STM32_SPI_CR1_OFFSET);
  cr1 &= ~clrbits;
  cr1 |= setbits;
  spi_putreg(priv, STM32_SPI_CR1_OFFSET, cr1);
}

/************************************************************************************
 * Name: spi_modifycr2
 *
 * Description:
 *   Clear and set bits in the CR2 register
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F30XX) || \
    defined(CONFIG_STM32_STM32F37XX) || defined(CONFIG_STM32_STM32F4XXX) || \
    defined(CONFIG_STM32_SPI_DMA)
static void spi_modifycr2(FAR struct stm32_spidev_s *priv, uint16_t setbits,
                          uint16_t clrbits)
{
  uint16_t cr2;
  cr2  = spi_getreg(priv, STM32_SPI_CR2_OFFSET);
  cr2 &= ~clrbits;
  cr2 |= setbits;
  spi_putreg(priv, STM32_SPI_CR2_OFFSET, cr2);
}
#endif

/************************************************************************************
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
 ************************************************************************************/

static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxsem_wait_uninterruptible(&priv->exclsem);
    }
  else
    {
      ret = nxsem_post(&priv->exclsem);
    }

  return ret;
}

/************************************************************************************
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
 ************************************************************************************/

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  uint16_t setbits;
  uint32_t actual;

  /* Has the frequency changed? */

  if (frequency != priv->frequency)
    {
      /* Choices are limited by PCLK frequency with a set of divisors */

      if (frequency >= priv->spiclock >> 1)
        {
          /* More than fPCLK/2.  This is as fast as we can go */

          setbits = SPI_CR1_FPCLCKd2; /* 000: fPCLK/2 */
          actual = priv->spiclock >> 1;
        }
      else if (frequency >= priv->spiclock >> 2)
        {
          /* Between fPCLCK/2 and fPCLCK/4, pick the slower */

          setbits = SPI_CR1_FPCLCKd4; /* 001: fPCLK/4 */
          actual = priv->spiclock >> 2;
        }
      else if (frequency >= priv->spiclock >> 3)
        {
          /* Between fPCLCK/4 and fPCLCK/8, pick the slower */

          setbits = SPI_CR1_FPCLCKd8; /* 010: fPCLK/8 */
          actual = priv->spiclock >> 3;
        }
      else if (frequency >= priv->spiclock >> 4)
        {
          /* Between fPCLCK/8 and fPCLCK/16, pick the slower */

          setbits = SPI_CR1_FPCLCKd16; /* 011: fPCLK/16 */
          actual = priv->spiclock >> 4;
        }
      else if (frequency >= priv->spiclock >> 5)
        {
          /* Between fPCLCK/16 and fPCLCK/32, pick the slower */

          setbits = SPI_CR1_FPCLCKd32; /* 100: fPCLK/32 */
          actual = priv->spiclock >> 5;
        }
      else if (frequency >= priv->spiclock >> 6)
        {
          /* Between fPCLCK/32 and fPCLCK/64, pick the slower */

          setbits = SPI_CR1_FPCLCKd64; /*  101: fPCLK/64 */
          actual = priv->spiclock >> 6;
        }
      else if (frequency >= priv->spiclock >> 7)
        {
          /* Between fPCLCK/64 and fPCLCK/128, pick the slower */

          setbits = SPI_CR1_FPCLCKd128; /* 110: fPCLK/128 */
          actual = priv->spiclock >> 7;
        }
      else
        {
          /* Less than fPCLK/128.  This is as slow as we can go */

          setbits = SPI_CR1_FPCLCKd256; /* 111: fPCLK/256 */
          actual = priv->spiclock >> 8;
        }

      spi_modifycr1(priv, 0, SPI_CR1_SPE);
      spi_modifycr1(priv, setbits, SPI_CR1_BR_MASK);
      spi_modifycr1(priv, SPI_CR1_SPE, 0);

      /* Save the frequency selection so that subsequent reconfigurations will be
       * faster.
       */

      spiinfo("Frequency %" PRIu32 "->%" PRIu32 "\n", frequency, actual);

      priv->frequency = frequency;
      priv->actual    = actual;
    }

  return priv->actual;
}

/************************************************************************************
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
 ************************************************************************************/

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  uint16_t setbits;
  uint16_t clrbits;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set CR1 appropriately */

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          setbits = 0;
          clrbits = SPI_CR1_CPOL | SPI_CR1_CPHA;
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          setbits = SPI_CR1_CPHA;
          clrbits = SPI_CR1_CPOL;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          setbits = SPI_CR1_CPOL;
          clrbits = SPI_CR1_CPHA;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          setbits = SPI_CR1_CPOL | SPI_CR1_CPHA;
          clrbits = 0;
          break;

#ifdef SPI_CR2_FRF    /* If MCU supports TI Synchronous Serial Frame Format */
        case SPIDEV_MODETI:
          setbits = 0;
          clrbits = SPI_CR1_CPOL | SPI_CR1_CPHA;
          break;
#endif

        default:
          return;
        }

        spi_modifycr1(priv, 0, SPI_CR1_SPE);
        spi_modifycr1(priv, setbits, clrbits);
        spi_modifycr1(priv, SPI_CR1_SPE, 0);

#ifdef SPI_CR2_FRF    /* If MCU supports TI Synchronous Serial Frame Format */
      switch (mode)
        {
          case SPIDEV_MODE0:
          case SPIDEV_MODE1:
          case SPIDEV_MODE2:
          case SPIDEV_MODE3:
            setbits = 0;
            clrbits = SPI_CR2_FRF;
            break;

          case SPIDEV_MODETI:
            setbits = SPI_CR2_FRF;
            clrbits = 0;
            break;

          default:
            return;
        }

      spi_modifycr1(priv, 0, SPI_CR1_SPE);
      spi_modifycr2(priv, setbits, clrbits);
      spi_modifycr1(priv, SPI_CR1_SPE, 0);
#endif

        /* Save the mode so that subsequent re-configurations will be faster */

        priv->mode = mode;
    }
}

/************************************************************************************
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
 ************************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  uint16_t setbits;
  uint16_t clrbits;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
#if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
      /* Yes... Set CR2 appropriately */

      /* Set the number of bits (valid range 4-16) */

      if (nbits < 4 || nbits > 16)
        {
          spierr("ERROR: nbits out of range: %d\n", nbits);
          return;
        }

      clrbits = SPI_CR2_DS_MASK;
      setbits = SPI_CR2_DS(nbits);

      /* If nbits is <=8, then we are in byte mode and FRXTH must be set
       * (else, transaction will not complete).
       */

      if (nbits < 9)
        {
          setbits |= SPI_CR2_FRXTH; /* RX FIFO Threshold = 1 byte */
        }
      else
        {
          clrbits |= SPI_CR2_FRXTH; /* RX FIFO Threshold = 2 bytes */
        }

      spi_modifycr1(priv, 0, SPI_CR1_SPE);
      spi_modifycr2(priv, setbits, clrbits);
      spi_modifycr1(priv, SPI_CR1_SPE, 0);
#else
      /* Yes... Set CR1 appropriately */

      switch (nbits)
        {
        case 8:
          setbits = 0;
          clrbits = SPI_CR1_DFF;
          break;

        case 16:
          setbits = SPI_CR1_DFF;
          clrbits = 0;
          break;

        default:
          return;
        }

      spi_modifycr1(priv, 0, SPI_CR1_SPE);
      spi_modifycr1(priv, setbits, clrbits);
      spi_modifycr1(priv, SPI_CR1_SPE, 0);
#endif
      /* Save the selection so that subsequent re-configurations will be faster. */

      priv->nbits = nbits;
    }
}

/************************************************************************************
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
 ************************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int spi_hwfeatures(FAR struct spi_dev_s *dev, spi_hwfeatures_t features)
{
#if defined(CONFIG_SPI_BITORDER) || defined(CONFIG_SPI_TRIGGER)
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
#endif

#ifdef CONFIG_SPI_BITORDER
  uint16_t setbits;
  uint16_t clrbits;

  spiinfo("features=%08x\n", features);

  /* Transfer data LSB first? */

  if ((features & HWFEAT_LSBFIRST) != 0)
    {
      setbits = SPI_CR1_LSBFIRST;
      clrbits = 0;
    }
  else
    {
      setbits = 0;
      clrbits = SPI_CR1_LSBFIRST;
    }

  spi_modifycr1(priv, 0, SPI_CR1_SPE);
  spi_modifycr1(priv, setbits, clrbits);
  spi_modifycr1(priv, SPI_CR1_SPE, 0);

  features &= ~HWFEAT_LSBFIRST;
#endif

#ifdef CONFIG_SPI_TRIGGER
/* Turn deferred trigger mode on or off.  Only applicable for DMA mode. If a
 * transfer is deferred then the DMA will not actually be triggered until a
 * subsequent call to SPI_TRIGGER to set it off. The thread will be waiting
 * on the transfer completing as normal.
 */

  priv->defertrig = ((features & HWFEAT_TRIGGER) != 0);
  features &= ~HWFEAT_TRIGGER;
#endif

  /* Other H/W features are not supported */

  return (features == 0) ? OK : -ENOSYS;
}
#endif

/************************************************************************************
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
 ************************************************************************************/

static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  uint32_t regval;
  uint32_t ret;

  DEBUGASSERT(priv && priv->spibase);

  spi_writeword(priv, (uint32_t)(wd & 0xffff));
  ret = (uint32_t)spi_readword(priv);

  /* Check and clear any error flags (Reading from the SR clears the error flags) */

  regval = spi_getreg(priv, STM32_SPI_SR_OFFSET);

  spiinfo("Sent: %04" PRIx32 " Return: %04" PRIx32
          " Status: %02" PRIx32 "\n", wd, ret, regval);
  UNUSED(regval);

  return ret;
}

/************************************************************************************
 * Name: spi_exchange (no DMA).  aka spi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 *   REVISIT: This function could be much more efficient by exploiting (1) RX and TX
 *   FIFOs and (2) the STM32 F3 data packing.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchaned in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#if !defined(CONFIG_STM32_SPI_DMA) || defined(CONFIG_STM32_DMACAPABLE) || \
     defined(CONFIG_STM32_SPI_DMATHRESHOLD)
#if !defined(CONFIG_STM32_SPI_DMA)
static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
#else
static void spi_exchange_nodma(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                               FAR void *rxbuffer, size_t nwords)
#endif
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  DEBUGASSERT(priv && priv->spibase);

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* 8- or 16-bit mode? */

  if (priv->nbits > 8)
    {
      /* 16-bit mode */

      const uint16_t *src  = (const uint16_t *)txbuffer;
            uint16_t *dest = (uint16_t *)rxbuffer;
            uint16_t  word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = *src++;
            }
          else
            {
              word = 0xffff;
            }

          /* Exchange one word */

          word = (uint16_t)spi_send(dev, (uint32_t)word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = word;
            }
        }
    }
  else
    {
      /* 8-bit mode */

      const uint8_t *src  = (const uint8_t *)txbuffer;
            uint8_t *dest = (uint8_t *)rxbuffer;
            uint8_t  word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = *src++;
            }
          else
            {
              word = 0xff;
            }

          /* Exchange one word */

          word = (uint8_t)spi_send(dev, (uint32_t)word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = word;
            }
        }
    }
}
#endif /* !CONFIG_STM32_SPI_DMA || CONFIG_STM32_DMACAPABLE || CONFIG_STM32_SPI_DMATHRESHOLD */

/************************************************************************************
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
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  FAR void *xbuffer = rxbuffer;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Convert the number of word to a number of bytes */

  size_t nbytes = (priv->nbits > 8) ? nwords << 1 : nwords;

#ifdef CONFIG_STM32_SPI_DMATHRESHOLD
  /* If this is a small SPI transfer, then let spi_exchange_nodma() do the work. */

  if (nbytes <= CONFIG_STM32_SPI_DMATHRESHOLD)
    {
      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }
#endif

  if ((priv->rxdma == NULL) || (priv->txdma == NULL) ||
      up_interrupt_context())
    {
      /* Invalid DMA channels, or interrupt context, fall
       * back to non-DMA method.
       */

      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }

#ifdef CONFIG_STM32_DMACAPABLE
  if ((txbuffer && priv->txbuf == 0 &&
      !stm32_dmacapable((uintptr_t)txbuffer, nwords, priv->txccr)) ||
      (rxbuffer && priv->rxbuf == 0 &&
       !stm32_dmacapable((uintptr_t)rxbuffer, nwords, priv->rxccr)))
    {
      /* Unsupported memory region fall back to non-DMA method. */

      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
    }
  else
#endif
    {
      static uint16_t rxdummy = 0xffff;
      static const uint16_t txdummy = 0xffff;

      spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);
      DEBUGASSERT(priv && priv->spibase);

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
#endif /* CONFIG_STM32_SPI_DMA */

/************************************************************************************
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
 ************************************************************************************/

#ifdef CONFIG_SPI_TRIGGER
static int spi_trigger(FAR struct spi_dev_s *dev)
{
#ifdef CONFIG_STM32_SPI_DMA
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;

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

/************************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev,
                         FAR const void *txbuffer,
                         size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return spi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/************************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that can be received in the buffer in number
 *              of words.  The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(FAR struct spi_dev_s *dev,
                          FAR void *rxbuffer,
                          size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/************************************************************************************
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
 ************************************************************************************/

static void spi_bus_initialize(FAR struct stm32_spidev_s *priv)
{
  uint16_t setbits;
  uint16_t clrbits;

#if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
  /* Configure CR1 and CR2. Default configuration:
   *   Mode 0:                        CR1.CPHA=0 and CR1.CPOL=0
   *   Master:                        CR1.MSTR=1
   *   8-bit:                         CR2.DS=7
   *   MSB transmitted first:         CR1.LSBFIRST=0
   *   Replace NSS with SSI & SSI=1:  CR1.SSI=1 CR1.SSM=1 (prevents MODF error)
   *   Two lines full duplex:         CR1.BIDIMODE=0 CR1.BIDIOIE=(Don't care) and
   *                                  CR1.RXONLY=0
   */

  clrbits = SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_BR_MASK | SPI_CR1_LSBFIRST |
            SPI_CR1_RXONLY | SPI_CR1_CRCL | SPI_CR1_BIDIOE | SPI_CR1_BIDIMODE;
  setbits = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM;
  spi_modifycr1(priv, setbits, clrbits);

  clrbits = SPI_CR2_DS_MASK;
  setbits = SPI_CR2_DS_8BIT | SPI_CR2_FRXTH; /* FRXTH must be high in 8-bit mode */
  spi_modifycr2(priv, setbits, clrbits);
#else
  /* Configure CR1. Default configuration:
   *   Mode 0:                        CPHA=0 and CPOL=0
   *   Master:                        MSTR=1
   *   8-bit:                         DFF=0
   *   MSB transmitted first:         LSBFIRST=0
   *   Replace NSS with SSI & SSI=1:  SSI=1 SSM=1 (prevents MODF error)
   *   Two lines full duplex:         BIDIMODE=0 BIDIOIE=(Don't care) and RXONLY=0
   */

  clrbits = SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_BR_MASK | SPI_CR1_LSBFIRST |
            SPI_CR1_RXONLY | SPI_CR1_DFF | SPI_CR1_BIDIOE | SPI_CR1_BIDIMODE;
  setbits = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM;
  spi_modifycr1(priv, setbits, clrbits);
#endif

  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* CRCPOLY configuration */

  spi_putreg(priv, STM32_SPI_CRCPR_OFFSET, 7);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

  nxsem_init(&priv->exclsem, 0, 1);

#ifdef CONFIG_STM32_SPI_DMA
  /* Initialize the SPI semaphores that is used to wait for DMA completion.
   * This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  if (priv->rxch && priv->txch)
    {
      nxsem_init(&priv->rxsem, 0, 0);
      nxsem_init(&priv->txsem, 0, 0);

      nxsem_set_protocol(&priv->rxsem, SEM_PRIO_NONE);
      nxsem_set_protocol(&priv->txsem, SEM_PRIO_NONE);

      /* Get DMA channels.  NOTE: stm32_dmachannel() will always assign the DMA
       * channel.  If the channel is not available, then stm32_dmachannel() will
       * block and wait until the channel becomes available.  WARNING: If you have
       * another device sharing a DMA channel with SPI and the code never releases
       * that channel, then the call to stm32_dmachannel()  will hang forever in
       * this function!  Don't let your design do that!
       */

      priv->rxdma = stm32_dmachannel(priv->rxch);
      priv->txdma = stm32_dmachannel(priv->txch);
      DEBUGASSERT(priv->rxdma && priv->txdma);

      spi_modifycr2(priv, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN, 0);
    }
  else
    {
      priv->rxdma = NULL;
      priv->txdma = NULL;
    }
#endif

  /* Enable SPI */

  spi_modifycr1(priv, SPI_CR1_SPE, 0);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ************************************************************************************/

FAR struct spi_dev_s *stm32_spibus_initialize(int bus)
{
  FAR struct stm32_spidev_s *priv = NULL;

  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_STM32_SPI1
  if (bus == 1)
    {
      /* Select SPI1 */

      priv = &g_spi1dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI1 pins: SCK, MISO, and MOSI */

          stm32_configgpio(GPIO_SPI1_SCK);
          stm32_configgpio(GPIO_SPI1_MISO);
          stm32_configgpio(GPIO_SPI1_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          spi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
#ifdef CONFIG_STM32_SPI2
  if (bus == 2)
    {
      /* Select SPI2 */

      priv = &g_spi2dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI2 pins: SCK, MISO, and MOSI */

          stm32_configgpio(GPIO_SPI2_SCK);
          stm32_configgpio(GPIO_SPI2_MISO);
          stm32_configgpio(GPIO_SPI2_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          spi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
#ifdef CONFIG_STM32_SPI3
  if (bus == 3)
    {
      /* Select SPI3 */

      priv = &g_spi3dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI3 pins: SCK, MISO, and MOSI */

          stm32_configgpio(GPIO_SPI3_SCK);
          stm32_configgpio(GPIO_SPI3_MISO);
          stm32_configgpio(GPIO_SPI3_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          spi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
#ifdef CONFIG_STM32_SPI4
  if (bus == 4)
    {
      /* Select SPI4 */

      priv = &g_spi4dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI4 pins: SCK, MISO, and MOSI */

          stm32_configgpio(GPIO_SPI4_SCK);
          stm32_configgpio(GPIO_SPI4_MISO);
          stm32_configgpio(GPIO_SPI4_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          spi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
#ifdef CONFIG_STM32_SPI5
  if (bus == 5)
    {
      /* Select SPI5 */

      priv = &g_spi5dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI5 pins: SCK, MISO, and MOSI */

          stm32_configgpio(GPIO_SPI5_SCK);
          stm32_configgpio(GPIO_SPI5_MISO);
          stm32_configgpio(GPIO_SPI5_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          spi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
#ifdef CONFIG_STM32_SPI6
  if (bus == 6)
    {
      /* Select SPI6 */

      priv = &g_spi6dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI6 pins: SCK, MISO, and MOSI */

          stm32_configgpio(GPIO_SPI6_SCK);
          stm32_configgpio(GPIO_SPI6_MISO);
          stm32_configgpio(GPIO_SPI6_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          spi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
    {
      spierr("ERROR: Unsupported SPI bus: %d\n", bus);
    }

  leave_critical_section(flags);
  return (FAR struct spi_dev_s *)priv;
}

#endif /* CONFIG_STM32_SPI1 || CONFIG_STM32_SPI2 || CONFIG_STM32_SPI3 ||
        * CONFIG_STM32_SPI4 || CONFIG_STM32_SPI5 || CONFIG_STM32_SPI6
        */
