/****************************************************************************
 * arch/arm/src/stm32h7/stm32_spi.c
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
 * The external functions, stm32_spi1/2/3/4/5/6select and
 * stm32_spi1/2/3/4/5/6status must be provided by board-specific logic.  They
 * are implementations of the select and status methods of the SPI interface
 * defined by struct spi_ops_s (see include/nuttx/spi/spi.h).  All other
 * methods (including stm32_spibus_initialize()) are provided by common STM32
 * logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3/4/5/6select() and stm32_spi1/2/3/4/5/6status()
 *      functions in your board-specific logic.  These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to
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
#include <stddef.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/compiler.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>
#include <nuttx/power/pm.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "stm32_dma.h"

#if defined(CONFIG_STM32H7_SPI1) || defined(CONFIG_STM32H7_SPI2) || \
    defined(CONFIG_STM32H7_SPI3) || defined(CONFIG_STM32H7_SPI4) || \
    defined(CONFIG_STM32H7_SPI5) || defined(CONFIG_STM32H7_SPI6)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SPI interrupts */

#ifdef CONFIG_STM32H7_SPI_INTERRUPTS
#  error "Interrupt driven SPI not yet supported"
#endif

/* Can't have both interrupt driven SPI and SPI DMA */

#if defined(CONFIG_STM32H7_SPI_INTERRUPTS) && defined(CONFIG_STM32H7_SPI_DMA)
#  error "Cannot enable both interrupt mode and DMA mode for SPI"
#endif

/* SPI DMA priority */

#ifdef CONFIG_STM32H7_SPI_DMA

#  if defined(CONFIG_SPI_DMAPRIO)
#    define SPI_DMA_PRIO  CONFIG_SPI_DMAPRIO
#  elif defined(DMA_SCR_PRIMED)
#    define SPI_DMA_PRIO  DMA_SCR_PRILO
#  else
#    error "Unknown STM32 DMA"
#  endif

#  if (SPI_DMA_PRIO & ~DMA_SCR_PL_MASK) != 0
#    error "Illegal value for CONFIG_SPI_DMAPRIO"
#  endif

/* DMA channel configuration */

#  define SPI_RXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_SCR_MSIZE_16BITS|DMA_SCR_PSIZE_16BITS|DMA_SCR_MINC|DMA_SCR_DIR_P2M)
#  define SPI_RXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS |DMA_SCR_MINC|DMA_SCR_DIR_P2M)
#  define SPI_RXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_16BITS             |DMA_SCR_DIR_P2M)
#  define SPI_RXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS              |DMA_SCR_DIR_P2M)
#  define SPI_TXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_SCR_MSIZE_16BITS|DMA_SCR_PSIZE_16BITS|DMA_SCR_MINC|DMA_SCR_DIR_M2P)
#  define SPI_TXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS |DMA_SCR_MINC|DMA_SCR_DIR_M2P)
#  define SPI_TXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_16BITS             |DMA_SCR_DIR_M2P)
#  define SPI_TXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS              |DMA_SCR_DIR_M2P)

/* If built with CONFIG_ARMV7M_DCACHE Buffers need to be aligned and
 * multiples of ARMV7M_DCACHE_LINESIZE
 */

#  if defined(CONFIG_ARMV7M_DCACHE)
#    define SPIDMA_BUFFER_MASK   (ARMV7M_DCACHE_LINESIZE - 1)
#    define SPIDMA_SIZE(b) (((b) + SPIDMA_BUFFER_MASK) & ~SPIDMA_BUFFER_MASK)
#    define SPIDMA_BUF_ALIGN   aligned_data(ARMV7M_DCACHE_LINESIZE)
#  else
#    define SPIDMA_SIZE(b)  (b)
#    define SPIDMA_BUF_ALIGN
#  endif

#  if defined(CONFIG_STM32H7_SPI1_DMA_BUFFER) && \
            CONFIG_STM32H7_SPI1_DMA_BUFFER > 0
#    define SPI1_DMABUFSIZE_ADJUSTED SPIDMA_SIZE(CONFIG_STM32H7_SPI1_DMA_BUFFER)
#    define SPI1_DMABUFSIZE_ALGN SPIDMA_BUF_ALIGN
#  endif

#  if defined(CONFIG_STM32H7_SPI2_DMA_BUFFER) && \
            CONFIG_STM32H7_SPI2_DMA_BUFFER > 0
#    define SPI2_DMABUFSIZE_ADJUSTED SPIDMA_SIZE(CONFIG_STM32H7_SPI2_DMA_BUFFER)
#    define SPI2_DMABUFSIZE_ALGN SPIDMA_BUF_ALIGN
#  endif

#  if defined(CONFIG_STM32H7_SPI3_DMA_BUFFER) && \
            CONFIG_STM32H7_SPI3_DMA_BUFFER > 0
#    define SPI3_DMABUFSIZE_ADJUSTED SPIDMA_SIZE(CONFIG_STM32H7_SPI3_DMA_BUFFER)
#    define SPI3_DMABUFSIZE_ALGN SPIDMA_BUF_ALIGN
#  endif

#  if defined(CONFIG_STM32H7_SPI4_DMA_BUFFER) && \
            CONFIG_STM32H7_SPI4_DMA_BUFFER > 0
#    define SPI4_DMABUFSIZE_ADJUSTED SPIDMA_SIZE(CONFIG_STM32H7_SPI4_DMA_BUFFER)
#    define SPI4_DMABUFSIZE_ALGN SPIDMA_BUF_ALIGN
#  endif

#  if defined(CONFIG_STM32H7_SPI5_DMA_BUFFER) && \
            CONFIG_STM32H7_SPI5_DMA_BUFFER > 0
#    define SPI5_DMABUFSIZE_ADJUSTED SPIDMA_SIZE(CONFIG_STM32H7_SPI5_DMA_BUFFER)
#    define SPI5_DMABUFSIZE_ALGN SPIDMA_BUF_ALIGN
#  endif

#if defined(CONFIG_STM32H7_SPI6_DMA_BUFFER) && \
            CONFIG_STM32H7_SPI6_DMA_BUFFER > 0
#    define SPI6_DMABUFSIZE_ADJUSTED SPIDMA_SIZE(CONFIG_STM32H7_SPI6_DMA_BUFFER)
#    define SPI6_DMABUFSIZE_ALGN SPIDMA_BUF_ALIGN
#  endif

#endif

/* Kernel clock configuration
 * TODO:
 *  - support for all kernel clock configuration
 */

#if defined(CONFIG_STM32H7_SPI1) || defined(CONFIG_STM32H7_SPI2) || \
  defined(CONFIG_STM32H7_SPI3)
#  if STM32_RCC_D2CCIP1R_SPI123SRC == RCC_D2CCIP1R_SPI123SEL_PLL1
#    define SPI123_KERNEL_CLOCK_FREQ STM32_PLL1Q_FREQUENCY
#  elif STM32_RCC_D2CCIP1R_SPI123SRC == RCC_D2CCIP1R_SPI123SEL_PLL2
#    define SPI123_KERNEL_CLOCK_FREQ STM32_PLL2P_FREQUENCY
#  else
#    error Not supported yet
#  endif
#  if SPI123_KERNEL_CLOCK_FREQ > 200000000
#    error Not supported SPI123 frequency
#  endif
#endif /* SPI123 */

#if defined(CONFIG_STM32H7_SPI4) || defined(CONFIG_STM32H7_SPI5)
#  if STM32_RCC_D2CCIP1R_SPI45SRC == RCC_D2CCIP1R_SPI45SEL_APB
#    define SPI45_KERNEL_CLOCK_FREQ STM32_PCLK2_FREQUENCY
#  elif STM32_RCC_D2CCIP1R_SPI45SRC == RCC_D2CCIP1R_SPI45SEL_PLL2
#    define SPI45_KERNEL_CLOCK_FREQ STM32_PLL2Q_FREQUENCY
#  else
#    error Not supported yet
#  endif
#  if SPI45_KERNEL_CLOCK_FREQ > 100000000
#    error Not supported SPI45 frequency
#  endif
#endif /* SPI45 */

#if defined(CONFIG_STM32H7_SPI6)
#  if STM32_RCC_D3CCIPR_SPI6SRC == RCC_D3CCIPR_SPI6SEL_PCLK4
#    define SPI6_KERNEL_CLOCK_FREQ STM32_PCLK4_FREQUENCY
#  elif STM32_RCC_D3CCIPR_SPI6SRC == RCC_D3CCIPR_SPI6SEL_PLL2
#    define SPI6_KERNEL_CLOCK_FREQ STM32_PLL2Q_FREQUENCY
#  else
#    error Not supported yet
#  endif
#  if SPI6_KERNEL_CLOCK_FREQ > 100000000
#    error Not supported SPI6 frequency
#  endif
#endif /* SPI6 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum spi_config_e
{
  FULL_DUPLEX = 0,
  SIMPLEX_TX,
  SIMPLEX_RX,
  HALF_DUPLEX
};

struct stm32_spidev_s
{
  struct spi_dev_s spidev;       /* Externally visible part of the SPI interface */
  uint32_t         spibase;      /* SPIn base address */
  uint32_t         spiclock;     /* Clocking for the SPI module */
  uint8_t          spiirq;       /* SPI IRQ number */
#ifdef CONFIG_STM32H7_SPI_DMA
  volatile uint8_t rxresult;     /* Result of the RX DMA */
  volatile uint8_t txresult;     /* Result of the RX DMA */
#ifdef CONFIG_SPI_TRIGGER
  bool             defertrig;    /* Flag indicating that trigger should be deferred */
  bool             trigarmed;    /* Flag indicating that the trigger is armed */
#endif
  uint32_t         rxch;         /* The RX DMA channel number */
  uint32_t         txch;         /* The TX DMA channel number */
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
  mutex_t          lock;         /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;    /* Requested clock frequency */
  uint32_t         actual;       /* Actual clock frequency */
  int8_t           nbits;        /* Width of word in bits */
  uint8_t          mode;         /* Mode 0,1,2,3 */
#ifdef CONFIG_PM
  struct pm_callback_s pm_cb;    /* PM callbacks */
#endif
  enum spi_config_e config;      /* full/half duplex, simplex transmit/read only */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint32_t spi_getreg(struct stm32_spidev_s *priv,
                                  uint32_t offset);
static inline void spi_putreg(struct stm32_spidev_s *priv,
                              uint32_t offset, uint32_t value);
static inline uint32_t spi_readword(struct stm32_spidev_s *priv);
static inline void spi_writeword(struct stm32_spidev_s *priv,
                                 uint32_t byte);
#ifdef CONFIG_DEBUG_SPI_INFO
static inline void spi_dumpregs(struct stm32_spidev_s *priv);
#endif

/* DMA support */

#ifdef CONFIG_STM32H7_SPI_DMA
static int         spi_dmarxwait(struct stm32_spidev_s *priv);
static int         spi_dmatxwait(struct stm32_spidev_s *priv);
static inline void spi_dmarxwakeup(struct stm32_spidev_s *priv);
static void        spi_dmarxcallback(DMA_HANDLE handle, uint8_t isr,
                                     void *arg);
static void        spi_dmarxsetup(struct stm32_spidev_s *priv,
                                  void *rxbuffer,
                                  void *rxdummy,
                                  size_t nwords,
                                  stm32_dmacfg_t *dmacfg);
static void        spi_dmatxsetup(struct stm32_spidev_s *priv,
                                  const void *txbuffer,
                                  const void *txdummy,
                                  size_t nwords,
                                  stm32_dmacfg_t *dmacfg);
static inline void spi_dmarxstart(struct stm32_spidev_s *priv);
static inline void spi_dmatxstart(struct stm32_spidev_s *priv);
#endif

/* SPI methods */

static int         spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t    spi_setfrequency(struct spi_dev_s *dev,
                                    uint32_t frequency);
#ifdef CONFIG_SPI_DELAY_CONTROL
static int         spi_setdelay(struct spi_dev_s *dev, uint32_t startdelay,
                                uint32_t stopdelay, uint32_t csdelay,
                                uint32_t ifdelay);
#endif
static void        spi_setmode(struct spi_dev_s *dev,
                               enum spi_mode_e mode);
static void        spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int         spi_hwfeatures(struct spi_dev_s *dev,
                                  spi_hwfeatures_t features);
#endif
static uint32_t    spi_send(struct spi_dev_s *dev, uint32_t wd);
static void        spi_exchange(struct spi_dev_s *dev,
                                const void *txbuffer, void *rxbuffer,
                                size_t nwords);
#ifdef CONFIG_SPI_TRIGGER
static int         spi_trigger(struct spi_dev_s *dev);
#endif
#ifndef CONFIG_SPI_EXCHANGE
static void        spi_sndblock(struct spi_dev_s *dev,
                                const void *txbuffer, size_t nwords);
static void        spi_recvblock(struct spi_dev_s *dev,
                                 void *rxbuffer, size_t nwords);
#endif

/* Initialization */

static void        spi_bus_initialize(struct stm32_spidev_s *priv);

/* PM interface */

#ifdef CONFIG_PM
static int         spi_pm_prepare(struct pm_callback_s *cb, int domain,
                                  enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI1
static const struct spi_ops_s g_sp1iops =
{
  .lock              = spi_lock,
  .select            = stm32_spi1select,
  .setfrequency      = spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay          = spi_setdelay,
#endif
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
    .ops    = &g_sp1iops,
  },
  .spibase  = STM32_SPI1_BASE,
  .spiclock = SPI123_KERNEL_CLOCK_FREQ,
  .spiirq   = STM32_IRQ_SPI1,
#ifdef CONFIG_STM32H7_SPI1_DMA
  .rxch     = DMAMAP_SPI1_RX,
  .txch     = DMAMAP_SPI1_TX,
#  if defined(SPI1_DMABUFSIZE_ADJUSTED)
  .rxbuf    = g_spi1_rxbuf,
  .txbuf    = g_spi1_txbuf,
  .buflen   = SPI1_DMABUFSIZE_ADJUSTED,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_PM
  .pm_cb.prepare = spi_pm_prepare,
#endif
#ifdef CONFIG_STM32H7_SPI1_COMMTYPE
  .config   = CONFIG_STM32H7_SPI1_COMMTYPE,
#else
  .config   = FULL_DUPLEX,
#endif
};
#endif /* CONFIG_STM32H7_SPI1 */

#ifdef CONFIG_STM32H7_SPI2
static const struct spi_ops_s g_sp2iops =
{
  .lock              = spi_lock,
  .select            = stm32_spi2select,
  .setfrequency      = spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay          = spi_setdelay,
#endif
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
  .registercallback  = 0,  /* not implemented */
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
    .ops    = &g_sp2iops,
  },
  .spibase  = STM32_SPI2_BASE,
  .spiclock = SPI123_KERNEL_CLOCK_FREQ,
  .spiirq   = STM32_IRQ_SPI2,
#ifdef CONFIG_STM32H7_SPI2_DMA
  .rxch     = DMAMAP_SPI2_RX,
  .txch     = DMAMAP_SPI2_TX,
#  if defined(SPI2_DMABUFSIZE_ADJUSTED)
  .rxbuf    = g_spi2_rxbuf,
  .txbuf    = g_spi2_txbuf,
  .buflen   = SPI2_DMABUFSIZE_ADJUSTED,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_PM
  .pm_cb.prepare = spi_pm_prepare,
#endif
#ifdef CONFIG_STM32H7_SPI2_COMMTYPE
  .config   = CONFIG_STM32H7_SPI2_COMMTYPE,
#else
  .config   = FULL_DUPLEX,
#endif
};
#endif /* CONFIG_STM32H7_SPI2 */

#ifdef CONFIG_STM32H7_SPI3
static const struct spi_ops_s g_sp3iops =
{
  .lock              = spi_lock,
  .select            = stm32_spi3select,
  .setfrequency      = spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay          = spi_setdelay,
#endif
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
    .ops    = &g_sp3iops,
  },
  .spibase  = STM32_SPI3_BASE,
  .spiclock = SPI123_KERNEL_CLOCK_FREQ,
  .spiirq   = STM32_IRQ_SPI3,
#ifdef CONFIG_STM32H7_SPI3_DMA
  .rxch     = DMAMAP_SPI3_RX,
  .txch     = DMAMAP_SPI3_TX,
#  if defined(SPI3_DMABUFSIZE_ADJUSTED)
  .rxbuf    = g_spi3_rxbuf,
  .txbuf    = g_spi3_txbuf,
  .buflen   = SPI3_DMABUFSIZE_ADJUSTED,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_PM
  .pm_cb.prepare = spi_pm_prepare,
#endif
#ifdef CONFIG_STM32H7_SPI3_COMMTYPE
  .config   = CONFIG_STM32H7_SPI3_COMMTYPE,
#else
  .config   = FULL_DUPLEX,
#endif
};
#endif /* CONFIG_STM32H7_SPI3 */

#ifdef CONFIG_STM32H7_SPI4
static const struct spi_ops_s g_sp4iops =
{
  .lock              = spi_lock,
  .select            = stm32_spi4select,
  .setfrequency      = spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay          = spi_setdelay,
#endif
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
    .ops    = &g_sp4iops,
  },
  .spibase  = STM32_SPI4_BASE,
  .spiclock = SPI45_KERNEL_CLOCK_FREQ,
  .spiirq   = STM32_IRQ_SPI4,
#ifdef CONFIG_STM32H7_SPI4_DMA
  .rxch     = DMAMAP_SPI4_RX,
  .txch     = DMAMAP_SPI4_TX,
#  if defined(SPI4_DMABUFSIZE_ADJUSTED)
  .rxbuf    = g_spi4_rxbuf,
  .txbuf    = g_spi4_txbuf,
  .buflen   = SPI4_DMABUFSIZE_ADJUSTED,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_PM
  .pm_cb.prepare = spi_pm_prepare,
#endif
#ifdef CONFIG_STM32H7_SPI4_COMMTYPE
  .config   = CONFIG_STM32H7_SPI4_COMMTYPE,
#else
  .config   = FULL_DUPLEX,
#endif
};
#endif /* CONFIG_STM32H7_SPI4 */

#ifdef CONFIG_STM32H7_SPI5
static const struct spi_ops_s g_sp5iops =
{
  .lock              = spi_lock,
  .select            = stm32_spi5select,
  .setfrequency      = spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay          = spi_setdelay,
#endif
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
    .ops    = &g_sp5iops,
  },
  .spibase  = STM32_SPI5_BASE,
  .spiclock = SPI45_KERNEL_CLOCK_FREQ,
  .spiirq   = STM32_IRQ_SPI5,
#ifdef CONFIG_STM32H7_SPI5_DMA
  .rxch     = DMAMAP_SPI5_RX,
  .txch     = DMAMAP_SPI5_TX,
#  if defined(SPI5_DMABUFSIZE_ADJUSTED)
  .rxbuf    = g_spi5_rxbuf,
  .txbuf    = g_spi5_txbuf,
  .buflen   = SPI5_DMABUFSIZE_ADJUSTED,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_PM
  .pm_cb.prepare = spi_pm_prepare,
#endif
#ifdef CONFIG_STM32H7_SPI5_COMMTYPE
  .config   = CONFIG_STM32H7_SPI5_COMMTYPE,
#else
  .config   = FULL_DUPLEX,
#endif
};
#endif /* CONFIG_STM32H7_SPI5 */

#ifdef CONFIG_STM32H7_SPI6
static const struct spi_ops_s g_sp6iops =
{
  .lock              = spi_lock,
  .select            = stm32_spi6select,
  .setfrequency      = spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay          = spi_setdelay,
#endif
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = stm32_spi6status,
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
  .registercallback  = stm32_spi6register,  /* provided externally */
#else
  .registercallback  = 0,                   /* not implemented */
#endif
};

#if defined(SPI6_DMABUFSIZE_ADJUSTED)
static uint8_t g_spi6_txbuf[SPI6_DMABUFSIZE_ADJUSTED] SPI6_DMABUFSIZE_ALGN
                                                      locate_data(".sram4");
static uint8_t g_spi6_rxbuf[SPI6_DMABUFSIZE_ADJUSTED] SPI6_DMABUFSIZE_ALGN
                                                      locate_data(".sram4");
#endif
static struct stm32_spidev_s g_spi6dev =
{
  .spidev   =
  {
    .ops    = &g_sp6iops,
  },
  .spibase  = STM32_SPI6_BASE,
  .spiclock = SPI6_KERNEL_CLOCK_FREQ,
  .spiirq   = STM32_IRQ_SPI6,
#ifdef CONFIG_STM32H7_SPI6_DMA
  .rxch     = DMAMAP_SPI6_RX,
  .txch     = DMAMAP_SPI6_TX,
#  if defined(SPI6_DMABUFSIZE_ADJUSTED)
  .rxbuf    = g_spi6_rxbuf,
  .txbuf    = g_spi6_txbuf,
  .buflen   = SPI6_DMABUFSIZE_ADJUSTED,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_PM
  .pm_cb.prepare = spi_pm_prepare,
#endif
#ifdef CONFIG_STM32H7_SPI6_COMMTYPE
  .config   = CONFIG_STM32H7_SPI6_COMMTYPE,
#else
  .config   = FULL_DUPLEX,
#endif
};
#endif /* CONFIG_STM32H7_SPI6 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
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
 *   The contents of the 8-bit register
 *
 ****************************************************************************/

static inline uint8_t spi_getreg8(struct stm32_spidev_s *priv,
                                  uint32_t offset)
{
  return getreg8(priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_putreg8
 *
 * Description:
 *   Write a 8-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 8-bit value to be written
 *
 ****************************************************************************/

static inline void spi_putreg8(struct stm32_spidev_s *priv,
                               uint32_t offset, uint8_t value)
{
  putreg8(value, priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_getreg16
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
 ****************************************************************************/

static inline uint16_t spi_getreg16(struct stm32_spidev_s *priv,
                                    uint32_t offset)
{
  return getreg16(priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_putreg16
 *
 * Description:
 *   Write a 16-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 ****************************************************************************/

static inline void spi_putreg16(struct stm32_spidev_s *priv,
                               uint32_t offset, uint16_t value)
{
  putreg16(value, priv->spibase + offset);
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
 *   The contents of the 16-bit register
 *
 ****************************************************************************/

static inline uint32_t spi_getreg(struct stm32_spidev_s *priv,
                                  uint32_t offset)
{
  return getreg32(priv->spibase + offset);
}

/****************************************************************************
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
 ****************************************************************************/

static inline void spi_putreg(struct stm32_spidev_s *priv,
                              uint32_t offset, uint32_t value)
{
  putreg32(value, priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_modifyreg
 *
 * Description:
 *   Modify an SPI register
 *
 ****************************************************************************/

static void spi_modifyreg(struct stm32_spidev_s *priv, uint32_t offset,
                          uint32_t clrbits, uint32_t setbits)
{
  /* Only 32-bit registers */

  modifyreg32(priv->spibase + offset, clrbits, setbits);
}

/****************************************************************************
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
 ****************************************************************************/

static inline uint32_t spi_readword(struct stm32_spidev_s *priv)
{
  /* Can't receive in tx only mode */

  if (priv->config == SIMPLEX_TX)
    {
      return 0;
    }

  /* Wait until the receive buffer is not empty */

  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_RXP) == 0);

  /* Then return the received 16 bit word */

  return spi_getreg16(priv, STM32_SPI_RXDR_OFFSET);
}

/****************************************************************************
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
 ****************************************************************************/

static inline void spi_writeword(struct stm32_spidev_s *priv,
                                 uint32_t word)
{
  /* Can't transmit in rx only mode */

  if (priv->config == SIMPLEX_RX)
    {
      return;
    }

  /* Wait until the transmit buffer is empty */

  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_TXP) == 0);

  /* Then send the 16 bit word */

  spi_putreg16(priv, STM32_SPI_TXDR_OFFSET, word);
}

/****************************************************************************
 * Name: spi_readbyte
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
 ****************************************************************************/

static inline uint8_t spi_readbyte(struct stm32_spidev_s *priv)
{
  /* Can't receive in tx only mode */

  if (priv->config == SIMPLEX_TX)
    {
      return 0;
    }

  /* Wait until the receive buffer is not empty */

  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_RXP) == 0);

  /* Then return the received byte */

  return spi_getreg8(priv, STM32_SPI_RXDR_OFFSET);
}

/****************************************************************************
 * Name: spi_writebyte
 *
 * Description:
 *   Write one 8-bit frame to the SPI FIFO
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   byte - Byte to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_writebyte(struct stm32_spidev_s *priv,
                                 uint8_t byte)
{
  /* Can't transmit in rx only mode */

  if (priv->config == SIMPLEX_RX)
    {
      return;
    }

  /* Wait until the transmit buffer is empty */

  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_TXP) == 0);

  /* Then send the byte */

  spi_putreg8(priv, STM32_SPI_TXDR_OFFSET, byte);
}

/****************************************************************************
 * Name: spi_dumpregs
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SPI_INFO
static void spi_dumpregs(struct stm32_spidev_s *priv)
{
  spiinfo("CR1: 0x%08" PRIx32 " CFG1: 0x%08" PRIx32
          " CFG2: 0x%08" PRIx32 "\n",
          spi_getreg(priv, STM32_SPI_CR1_OFFSET),
          spi_getreg(priv, STM32_SPI_CFG1_OFFSET),
          spi_getreg(priv, STM32_SPI_CFG2_OFFSET));
  spiinfo("IER: 0x%08" PRIx32 " SR: 0x%08" PRIx32
          " I2SCFGR: 0x%08" PRIx32 "\n",
          spi_getreg(priv, STM32_SPI_IER_OFFSET),
          spi_getreg(priv, STM32_SPI_SR_OFFSET),
          spi_getreg(priv, STM32_SPI_I2SCFGR_OFFSET));
}
#endif

/****************************************************************************
 * Name: spi_interrupt
 *
 * Description:
 *   In DMA transfer mode, handle the transmission complete interrupt
 *
 ****************************************************************************/

static int spi_interrupt(int irq, void *context, void *arg)
{
  struct stm32_spidev_s *priv = (struct stm32_spidev_s *)arg;
  uint32_t sr = spi_getreg(priv, STM32_SPI_SR_OFFSET);

  if (sr & SPI_SR_TXC)
    {
      /* Disable interrupt. This needs to be done
       * before disabling SPI to avoid spurious TXC interrupt
       * (ERRATA: Q2.14.4 TXP interrupt occurring while SPI/I2Sdisabled")
       * There is no need to clear the interrupt since it is disabled and
       * SPI will be disabled after transmit as well.
       */

      spi_modifyreg(priv, STM32_SPI_IER_OFFSET, SPI_IER_EOTIE, 0);

      /* Set result and release wait semaphore */
#ifdef CONFIG_STM32H7_SPI_DMA
      priv->txresult = 0x80;
      nxsem_post(&priv->txsem);
#endif
    }

  return 0;
}

/****************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI_DMA
static int spi_dmarxwait(struct stm32_spidev_s *priv)
{
  int ret;

  /* Don't wait in tx only mode */

  if (priv->config == SIMPLEX_TX)
    {
      return OK;
    }

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
#endif

/****************************************************************************
 * Name: spi_dmatxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI_DMA
static int spi_dmatxwait(struct stm32_spidev_s *priv)
{
  int ret;

  /* Don't wait in rx only mode */

  if (priv->config == SIMPLEX_RX)
    {
      return OK;
    }

  /* Enable TXC interrupt. This needs to be done after starting TX-dma AND
   * when spi is enabled to avoid spurious TXC interrupt (ERRATA:
   * "2.14.4 TXP interrupt occurring while SPI/I2Sdisabled")
   * It is fine to enable this interrupt even if the transfer already did
   * complete; in this case the irq will just fire right away
   */

  spi_modifyreg(priv, STM32_SPI_IER_OFFSET, 0, SPI_IER_EOTIE);

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
#endif

/****************************************************************************
 * Name: spi_dmarxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI_DMA
static inline void spi_dmarxwakeup(struct stm32_spidev_s *priv)
{
  nxsem_post(&priv->rxsem);
}
#endif

/****************************************************************************
 * Name: spi_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI_DMA
static void spi_dmarxcallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
  struct stm32_spidev_s *priv = (struct stm32_spidev_s *)arg;

  /* Wake-up the SPI driver */

  priv->rxresult = isr | 0x080;  /* OR'ed with 0x80 to assure non-zero */
  spi_dmarxwakeup(priv);
}
#endif

/****************************************************************************
 * Name: spi_dmarxsetup
 *
 * Description:
 *   Setup to perform RX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI_DMA
static void spi_dmarxsetup(struct stm32_spidev_s *priv,
                           void *rxbuffer, void *rxdummy,
                           size_t nwords, stm32_dmacfg_t *dmacfg)
{
  /* Can't receive in tx only mode */

  if (priv->config == SIMPLEX_TX)
    {
      priv->rxccr = 0;
      return;
    }

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

  dmacfg->paddr = priv->spibase + STM32_SPI_RXDR_OFFSET;
  dmacfg->maddr = (uint32_t)rxbuffer;
  dmacfg->ndata = nwords;
  dmacfg->cfg1  = priv->rxccr;
  dmacfg->cfg2  = 0;
}
#endif

/****************************************************************************
 * Name: spi_dmatxsetup
 *
 * Description:
 *   Setup to perform TX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI_DMA
static void spi_dmatxsetup(struct stm32_spidev_s *priv,
                           const void *txbuffer, const void *txdummy,
                           size_t nwords, stm32_dmacfg_t *dmacfg)
{
  /* Can't transmit in rx only mode */

  if (priv->config == SIMPLEX_RX)
    {
      priv->txccr = 0;
      return;
    }

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

  dmacfg->paddr = priv->spibase + STM32_SPI_TXDR_OFFSET;
  dmacfg->maddr = (uint32_t)txbuffer;
  dmacfg->ndata = nwords;
  dmacfg->cfg1  = priv->txccr;
  dmacfg->cfg2  = 0;
}
#endif

/****************************************************************************
 * Name: spi_dmarxstart
 *
 * Description:
 *   Start RX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI_DMA
static void spi_dmarxstart(struct stm32_spidev_s *priv)
{
  /* Can't receive in tx only mode */

  if (priv->config == SIMPLEX_TX)
    {
      return;
    }

  priv->rxresult = 0;
  spi_modifyreg(priv, STM32_SPI_CFG1_OFFSET, 0, SPI_CFG1_RXDMAEN);
  stm32_dmastart(priv->rxdma, spi_dmarxcallback, priv, false);
}
#endif

/****************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start TX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI_DMA
static void spi_dmatxstart(struct stm32_spidev_s *priv)
{
  /* Can't transmit in rx only mode */

  if (priv->config == SIMPLEX_RX)
    {
      return;
    }

  priv->txresult = 0;
  stm32_dmastart(priv->txdma, NULL, priv, false);
  spi_modifyreg(priv, STM32_SPI_CFG1_OFFSET, 0, SPI_CFG1_TXDMAEN);
}
#endif

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
  struct stm32_spidev_s *priv = (struct stm32_spidev_s *)dev;
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
 * Name: spi_enable
 ****************************************************************************/

static inline int spi_enable(struct stm32_spidev_s *priv, bool state)
{
  int ret = OK;

  if (state == true)
    {
      /* Enable SPI */

      spi_modifyreg(priv, STM32_SPI_CR1_OFFSET, 0, SPI_CR1_SPE);
    }
  else
    {
      /* Disable SPI */

      spi_modifyreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SPE, 0);
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
  struct stm32_spidev_s *priv = (struct stm32_spidev_s *)dev;
  uint32_t setbits = 0;
  uint32_t actual  = 0;

  /* Limit to max possible (if STM32_SPI_CLK_MAX is defined in board.h) */

  if (frequency > STM32_SPI_CLK_MAX)
    {
      frequency = STM32_SPI_CLK_MAX;
    }

  /* Has the frequency changed? */

  if (frequency != priv->frequency)
    {
      /* Choices are limited by PCLK frequency with a set of divisors */

      if (frequency >= priv->spiclock >> 1)
        {
          /* More than fPCLK/2.  This is as fast as we can go */

          setbits = SPI_CFG1_MBR_FPCLKd2; /* 000: fPCLK/2 */
          actual = priv->spiclock >> 1;
        }
      else if (frequency >= priv->spiclock >> 2)
        {
          /* Between fPCLCK/2 and fPCLCK/4, pick the slower */

          setbits = SPI_CFG1_MBR_FPCLKd4; /* 001: fPCLK/4 */
          actual = priv->spiclock >> 2;
        }
      else if (frequency >= priv->spiclock >> 3)
        {
          /* Between fPCLCK/4 and fPCLCK/8, pick the slower */

          setbits = SPI_CFG1_MBR_FPCLKd8; /* 010: fPCLK/8 */
          actual = priv->spiclock >> 3;
        }
      else if (frequency >= priv->spiclock >> 4)
        {
          /* Between fPCLCK/8 and fPCLCK/16, pick the slower */

          setbits = SPI_CFG1_MBR_FPCLKd16; /* 011: fPCLK/16 */
          actual = priv->spiclock >> 4;
        }
      else if (frequency >= priv->spiclock >> 5)
        {
          /* Between fPCLCK/16 and fPCLCK/32, pick the slower */

          setbits = SPI_CFG1_MBR_FPCLKd32; /* 100: fPCLK/32 */
          actual = priv->spiclock >> 5;
        }
      else if (frequency >= priv->spiclock >> 6)
        {
          /* Between fPCLCK/32 and fPCLCK/64, pick the slower */

          setbits = SPI_CFG1_MBR_FPCLKd64; /*  101: fPCLK/64 */
          actual = priv->spiclock >> 6;
        }
      else if (frequency >= priv->spiclock >> 7)
        {
          /* Between fPCLCK/64 and fPCLCK/128, pick the slower */

          setbits = SPI_CFG1_MBR_FPCLKd128; /* 110: fPCLK/128 */
          actual = priv->spiclock >> 7;
        }
      else
        {
          /* Less than fPCLK/128.  This is as slow as we can go */

          setbits = SPI_CFG1_MBR_FPCLKd256; /* 111: fPCLK/256 */
          actual = priv->spiclock >> 8;
        }

      spi_enable(priv, false);
      spi_modifyreg(priv, STM32_SPI_CFG1_OFFSET, SPI_CFG1_MBR_MASK, setbits);
      spi_enable(priv, true);

      /* Save the frequency selection so that subsequent reconfigurations
       * will be faster.
       */

      spiinfo("Frequency %" PRId32 "->%" PRId32 "\n", frequency, actual);

      priv->frequency = frequency;
      priv->actual    = actual;
    }

  return priv->actual;
}

/****************************************************************************
 * Name: spi_setdelay
 *
 * Description:
 *   Set the SPI Delays in nanoseconds. Optional.
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *   startdelay - The delay between CS active and first CLK
 *   stopdelay  - The delay between last CLK and CS inactive
 *   csdelay    - The delay between CS inactive and CS active again
 *   ifdelay    - The delay between frames
 *
 * Returned Value:
 *   Returns zero (OK) on success; a negated errno value is return on any
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_DELAY_CONTROL
static int spi_setdelay(struct spi_dev_s *dev, uint32_t startdelay,
                        uint32_t stopdelay, uint32_t csdelay,
                        uint32_t ifdelay)
{
  struct stm32_spidev_s *priv = (struct stm32_spidev_s *)dev;
  uint32_t setbits  = 0;
  uint32_t clrbits  = SPI_CFG2_MSSI_MASK | SPI_CFG2_MIDI_MASK;
  uint32_t nsperclk = NSEC_PER_SEC / priv->actual;

  startdelay /= nsperclk;
  ifdelay    /= nsperclk;

  setbits = ((ifdelay << SPI_CFG2_MIDI_SHIFT) & SPI_CFG2_MIDI_MASK) |
            ((startdelay << SPI_CFG2_MSSI_SHIFT) & SPI_CFG2_MSSI_MASK);

  spi_enable(priv, false);

  /* Change SPI mode */

  spi_modifyreg(priv, STM32_SPI_CFG2_OFFSET, clrbits, setbits);

  /* Re-enable SPI */

  spi_enable(priv, true);
  return OK;
}
#endif

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
  struct stm32_spidev_s *priv = (struct stm32_spidev_s *)dev;
  uint32_t setbits = 0;
  uint32_t clrbits = 0;

  spiinfo("mode=%" PRIx32 "\n", (uint32_t) mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set CR1 appropriately */

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          setbits = 0;
          clrbits = SPI_CFG2_CPOL | SPI_CFG2_CPHA;
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          setbits = SPI_CFG2_CPHA;
          clrbits = SPI_CFG2_CPOL;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          setbits = SPI_CFG2_CPOL;
          clrbits = SPI_CFG2_CPHA;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          setbits = SPI_CFG2_CPOL | SPI_CFG2_CPHA;
          clrbits = 0;
          break;

        default:
          return;
        }

      spi_enable(priv, false);

      /* Change SPI mode */

      spi_modifyreg(priv, STM32_SPI_CFG2_OFFSET, clrbits, setbits);

      /* Re-enable SPI */

      spi_enable(priv, true);
      while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) &
             SPI_SR_RXPLVL_MASK) != 0)
        {
          /* Flush SPI read FIFO */

          spi_getreg(priv, STM32_SPI_RXDR_OFFSET);
        }

      /* Save the mode so that subsequent re-configurations will be faster */

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
  struct stm32_spidev_s *priv = (struct stm32_spidev_s *)dev;
  uint32_t setbits = 0;
  uint32_t clrbits = 0;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      /* Yes... Set CFG1 appropriately */

      /* Set the number of bits (valid range 4-32) */

      if (nbits < 4 || nbits > 32)
        {
          return;
        }

      clrbits = SPI_CFG1_DSIZE_MASK;
      setbits = SPI_CFG1_DSIZE_VAL(nbits);

      /* RX FIFO Threshold 1 Frame either 8 or 16 bits */

      setbits |= SPI_CFG1_FTHLV_1DATA;

      spi_enable(priv, false);
      spi_modifyreg(priv, STM32_SPI_CFG1_OFFSET, clrbits, setbits);
      spi_enable(priv, true);

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
  struct stm32_spidev_s *priv = (struct stm32_spidev_s *)dev;
#endif

#ifdef CONFIG_SPI_BITORDER
  uint32_t setbits = 0;
  uint32_t clrbits = 0;

  spiinfo("features=%08" PRIx8 "\n", features);

  /* Transfer data LSB first? */

  if ((features & HWFEAT_LSBFIRST) != 0)
    {
      setbits = SPI_CFG2_LSBFRST;
      clrbits = 0;
    }
  else
    {
      setbits = 0;
      clrbits = SPI_CFG2_LSBFRST;
    }

  spi_enable(priv, false);
  spi_modifyreg(priv, STM32_SPI_CFG2_OFFSET, clrbits, setbits);
  spi_enable(priv, true);

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
  struct stm32_spidev_s *priv = (struct stm32_spidev_s *)dev;
  uint32_t regval = 0;
  uint32_t ret    = 0;

  DEBUGASSERT(priv && priv->spibase);

  /* Clear suspend flag */

  spi_modifyreg(priv, STM32_SPI_IFCR_OFFSET, 0, SPI_IFCR_SUSPC);

  /* Master transfer start */

  if (priv->config != SIMPLEX_RX)
    {
      spi_modifyreg(priv, STM32_SPI_CR1_OFFSET, 0, SPI_CR1_CSTART);
    }

  /* According to the number of bits, access data register as word or byte
   * This is absolutely required because of packing. With nbits <=8 bit
   * frames, two bytes are received by a 16-bit read of the data register!
   */

  if (priv->nbits > 8)
    {
      spi_writeword(priv, (uint16_t)(wd & 0xffff));
      ret = spi_readword(priv);
    }
  else
    {
      spi_writebyte(priv, (uint8_t)(wd & 0xff));
      ret = (uint32_t)spi_readbyte(priv);
    }

  /* Check and clear any error flags (Reading from the SR clears the error
   * flags).
   */

  regval = spi_getreg(priv, STM32_SPI_SR_OFFSET);

  /* Suspend */

  spi_modifyreg(priv, STM32_SPI_CR1_OFFSET, 0, SPI_CR1_CSUSP);
  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_SUSP) == 0);

  /* Dump some info */

  if (priv->nbits > 8)
    {
      spiinfo("Sent: %04" PRIx32 " Return: %04" PRIx32 " Status: %02" PRIx32
              "\n", wd, ret, regval);
    }
  else
    {
      spiinfo("Sent: %02" PRIx32 " Return: %02" PRIx32 " Status: %02" PRIx32
              "\n", wd, ret, regval);
    }

  UNUSED(regval);
  return ret;
}

/****************************************************************************
 * Name: spi_exchange (no DMA).  aka spi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
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

#if !defined(CONFIG_STM32H7_SPI_DMA) || defined(CONFIG_STM32H7_DMACAPABLE) || \
     defined(CONFIG_STM32H7_SPI_DMATHRESHOLD)
#if !defined(CONFIG_STM32H7_SPI_DMA)
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
#else
static void spi_exchange_nodma(struct spi_dev_s *dev,
                               const void *txbuffer, void *rxbuffer,
                               size_t nwords)
#endif
{
  struct stm32_spidev_s *priv = (struct stm32_spidev_s *)dev;
  DEBUGASSERT(priv && priv->spibase);

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* Disable the DMA Requests */

  spi_modifyreg(priv, STM32_SPI_CFG1_OFFSET, SPI_CFG1_RXDMAEN |
                                             SPI_CFG1_TXDMAEN, 0);

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

#endif /* !CONFIG_STM32H7_SPI_DMA || CONFIG_STM32H7_DMACAPABLE ||
        * CONFIG_STM32H7_SPI_DMATHRESHOLD
        */

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

#ifdef CONFIG_STM32H7_SPI_DMA
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct stm32_spidev_s *priv = (struct stm32_spidev_s *)dev;
  stm32_dmacfg_t rxdmacfg;
  stm32_dmacfg_t txdmacfg;
  static uint8_t rxdummy[ARMV7M_DCACHE_LINESIZE]
    aligned_data(ARMV7M_DCACHE_LINESIZE);
  static const uint16_t txdummy = 0xffff;
  void *orig_rxbuffer = rxbuffer;

  DEBUGASSERT(priv != NULL);

  /* Convert the number of word to a number of bytes */

  size_t nbytes = (priv->nbits > 8) ? nwords << 1 : nwords;

#ifdef CONFIG_STM32H7_SPI_DMATHRESHOLD
  /* If this is a small SPI transfer, then let spi_exchange_nodma() do the
   * work.
   */

  if (nbytes <= CONFIG_STM32H7_SPI_DMATHRESHOLD)
    {
      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }
#endif

  if ((priv->config != SIMPLEX_TX && priv->rxdma == NULL) ||
      (priv->config != SIMPLEX_RX && priv->txdma == NULL) ||
      up_interrupt_context())
    {
      /* Invalid DMA channels, or interrupt context, fall
       * back to non-DMA method.
       */

      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }

  /* If this bus uses the driver's DMA aligned buffers (priv->txbuf != NULL)
   * we will incur 2 copies. However the copy cost is much less the non
   * DMA transfer time. Having the buffer in the driver ensures DMA can be
   * used. This is needed because the stm32_dmacapable API does not support
   * passing the buffer extent. So the only extent calculated is buffer
   * plus the transfer size. The sizes can be less than a cache line size,
   * and not aligned and are typically greater then 4 bytes, which is
   * about the break even point for the DMA IO overhead.
   */

  /* Does bus support in driver DMA buffering? */

  if (priv->txbuf)
    {
      if (nbytes > priv->buflen)
        {
          /* Buffer is too big for internal DMA buffer so fall back. */

          spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
          return;
        }

      /* First copy: If provided, copy caller's buffer to the internal DMA
       * txbuf
       */

      if (txbuffer)
        {
          memcpy(priv->txbuf, txbuffer, nbytes);

          /* Adjust pointers to internal DMA buffers */

          txbuffer  = priv->txbuf;
        }

      /* orig_rxbuffer holds the callers return buffer */

      rxbuffer  = rxbuffer ? priv->rxbuf : rxbuffer;
    }

  /* Setup the DMA configs using the internal or external set of buffers */

  spi_dmatxsetup(priv, txbuffer, &txdummy, nwords, &txdmacfg);
  spi_dmarxsetup(priv, rxbuffer, (uint16_t *)rxdummy, nwords, &rxdmacfg);

#ifdef CONFIG_STM32H7_DMACAPABLE

  /* Test for DMA capability of only callers buffers, internal buffers are
   * guaranteed capable.
   */

  if ((priv->config != SIMPLEX_RX && priv->txbuf == 0 &&
       !stm32_dmacapable(priv->txdma, &txdmacfg)) ||
      (priv->config != SIMPLEX_TX && priv->rxbuf == 0 &&
       !stm32_dmacapable(priv->rxdma, &rxdmacfg)))
    {
      /* Unsupported memory region fall back to non-DMA method. */

      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
    }
  else
#endif
    {
      /* When starting communication using DMA, to prevent DMA channel
       * management raising error events, these steps must be followed in
       * order:
       * 1. Enable DMA Rx buffer in the RXDMAEN bit in the SPI_CFG1 register,
       *    if DMA Rx is used.
       * 2. Enable DMA requests for Tx and Rx in DMA registers, if the DMA is
       *    used.
       * 3. Enable DMA Tx buffer in the TXDMAEN bit in the SPI_CFG1 register,
       *    if DMA Tx is used.
       * 4. Enable the SPI by setting the SPE bit.
       */

      /* Flush cache to physical memory */

      if (txbuffer)
        {
          up_clean_dcache((uintptr_t)txbuffer, (uintptr_t)txbuffer + nbytes);
        }

      if (rxbuffer)
        {
          up_invalidate_dcache((uintptr_t)rxbuffer,
                               (uintptr_t)rxbuffer + nbytes);
        }

      /* N.B. the H7 tri-states the clock output on SPI disable and
       * unfortunately the Chip Select (CS) is active.  So we keep
       * the device disabled for the minimum time, that meets the
       * setup sequence criteria.
       */

      spi_enable(priv, false);

      spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n",
              txbuffer, rxbuffer, nwords);
      DEBUGASSERT(priv->spibase != 0);

      /* Setup the DMA */

      if (priv->config != SIMPLEX_TX)
        {
          stm32_dmasetup(priv->rxdma, &rxdmacfg);
        }

      if (priv->config != SIMPLEX_RX)
        {
          stm32_dmasetup(priv->txdma, &txdmacfg);
        }

#ifdef CONFIG_SPI_TRIGGER
      /* Is deferred triggering in effect? */

      if (!priv->defertrig)
        {
          /* No.. Start the DMAs then the SPI */

          spi_dmarxstart(priv);
          spi_dmatxstart(priv);
          spi_enable(priv, true);
          spi_modifyreg(priv, STM32_SPI_CR1_OFFSET, 0, SPI_CR1_CSTART);
        }
      else
        {
          /* Yes.. indicated that we are ready to be started */

          priv->trigarmed = true;
        }
#else
      /* Start the DMAs then the SPI */

      spi_dmarxstart(priv);
      spi_dmatxstart(priv);
      spi_enable(priv, true);
      spi_modifyreg(priv, STM32_SPI_CR1_OFFSET, 0, SPI_CR1_CSTART);
#endif

      /* Then wait for each to complete */

      spi_dmarxwait(priv);
      spi_dmatxwait(priv);

      /* To close communication it is mandatory to follow these steps in
       * this order:
       * 1. Disable DMA request for Tx and Rx in the DMA registers, if the
       *    DMA issued.
       * 2. Disable the SPI by following the SPI disable procedure.
       * 3. Disable DMA Tx and Rx buffers by clearing the TXDMAEN and RXDMAEN
       *    bits in the SPI_CFG1 register, if DMA Tx and/or DMA Rx are used.
       */

      /* TX transmission must be completed before shutting down the SPI */

      DEBUGASSERT(priv->config != SIMPLEX_RX ?
                  (spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_TXC) : 1);

      spi_enable(priv, false);
      spi_modifyreg(priv, STM32_SPI_CFG1_OFFSET, SPI_CFG1_TXDMAEN |
                          SPI_CFG1_RXDMAEN, 0);
      spi_enable(priv, true);

      /* Second copy: Copy the DMA internal buffer to caller's buffer */

      if (orig_rxbuffer && priv->rxbuf)
        {
          memcpy(orig_rxbuffer, priv->rxbuf, nbytes);
        }
    }

#ifdef CONFIG_SPI_TRIGGER
      priv->trigarmed = false;
#endif
}
#endif /* CONFIG_STM32H7_SPI_DMA */

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
#ifdef CONFIG_STM32H7_SPI_DMA
  struct stm32_spidev_s *priv = (struct stm32_spidev_s *)dev;

  if (!priv->trigarmed)
    {
      return -EIO;
    }

  spi_dmarxstart(priv);
  spi_dmatxstart(priv);
  spi_enable(priv, true);
  spi_modifyreg(priv, STM32_SPI_CR1_OFFSET, 0, SPI_CR1_CSTART);
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
 *              words. The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface.  If nbits <= 8,
 *              the data is packed into uint8_t's; if nbits >8, the data is
 *              packed into uint16_t's
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
 *              number of words.  The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface.  If nbits <= 8,
 *              the data is packed into uint8_t's; if nbits >8, the data is
 *              packed into uint16_t's
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
 * Name: spi_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a
 *   warning that the system is about to enter into a new power state.  The
 *   driver should begin whatever operations that may be required to enter
 *   power state.  The driver may abort the state change mode by returning
 *   a non-zero value from the callback function.
 *
 * Input Parameters:
 *   cb      - Returned to the driver.  The driver version of the callback
 *             structure may include additional, driver-specific state
 *             data at the end of the structure.
 *   domain  - Identifies the activity domain of the state change
 *   pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   0 (OK) means the event was successfully processed and that the driver
 *   is prepared for the PM state change.  Non-zero means that the driver
 *   is not prepared to perform the tasks needed achieve this power setting
 *   and will cause the state change to be aborted.  NOTE:  The prepare
 *   method will also be recalled when reverting from lower back to higher
 *   power consumption modes (say because another driver refused a lower
 *   power state change).  Drivers are not permitted to return non-zero
 *   values when reverting back to higher power consumption modes!
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int spi_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate)
{
  struct stm32_spidev_s *priv =
      (struct stm32_spidev_s *)((char *)cb -
                                    offsetof(struct stm32_spidev_s, pm_cb));

  /* Logic to prepare for a reduced power state goes here. */

  switch (pmstate)
    {
    case PM_NORMAL:
    case PM_IDLE:
      break;

    case PM_STANDBY:
    case PM_SLEEP:

      /* Check if exclusive lock for SPI bus is held. */

      if (nxmutex_is_locked(&priv->lock))
        {
          /* Exclusive lock is held, do not allow entry to deeper PM
           * states.
           */

          return -EBUSY;
        }

      break;

    default:

      /* Should not get here */

      break;
    }

  return OK;
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

static void spi_bus_initialize(struct stm32_spidev_s *priv)
{
  uint32_t setbits = 0;
  uint32_t clrbits = 0;
#ifdef CONFIG_PM
  int ret;
#endif

  /* Configure CR1, CFG1 and CFG2. Default configuration:
   *   Mode 0:                        CFG2.CPHA=0 and CFG2.CPOL=0
   *   Master:                        CFG2.MSTR=1
   *   8-bit:                         CFG1.DSIZE=7
   *   MSB transmitted first:         CFG2.LSBFRST=0
   *   Replace NSS with SSI & SSI=1:  CR1.SSI=1 CFG2.SSM=1 (prevents MODF
   *                                  error)
   *   Two lines full duplex:         CFG2.COMM=0
   */

  /* CR1 */

  clrbits = SPI_CR1_SPE;
  setbits = SPI_CR1_SSI;

  spi_modifyreg(priv, STM32_SPI_CR1_OFFSET, clrbits, setbits);

  /* CFG1 */

  clrbits = SPI_CFG1_DSIZE_MASK;
  setbits = SPI_CFG1_DSIZE_8BIT | SPI_CFG1_FTHLV_1DATA; /* REVISIT: FTHLV */
  spi_modifyreg(priv, STM32_SPI_CFG1_OFFSET, clrbits, setbits);

  /* CFG2 */

  clrbits = SPI_CFG2_CPHA | SPI_CFG2_CPOL | SPI_CFG2_LSBFRST |
            SPI_CFG2_COMM_MASK;
  setbits = SPI_CFG2_AFCNTR | SPI_CFG2_MASTER | SPI_CFG2_SSM;

  switch (priv->config)
    {
    default:
    case FULL_DUPLEX:
      setbits |= SPI_CFG2_COMM_FULL;
      break;
    case SIMPLEX_TX:
      setbits |= SPI_CFG2_COMM_STX;
      break;
    case SIMPLEX_RX:
      setbits |= SPI_CFG2_COMM_SRX;
      break;
    case HALF_DUPLEX:
      setbits |= SPI_CFG2_COMM_HALF;
      break;
    }

  spi_modifyreg(priv, STM32_SPI_CFG2_OFFSET, clrbits, setbits);

  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((struct spi_dev_s *)priv, 400000);

  /* CRCPOLY configuration */

  spi_putreg(priv, STM32_SPI_CRCPOLY_OFFSET, 7);

#ifdef CONFIG_STM32H7_SPI_DMA
  /* Get DMA channels.  NOTE: stm32_dmachannel() will always assign the DMA
   * channel.  If the channel is not available, then stm32_dmachannel() will
   * block and wait until the channel becomes available.  WARNING: If you
   * have another device sharing a DMA channel with SPI and the code never
   * releases that channel, then the call to stm32_dmachannel()  will hang
   * forever in this function!  Don't let your design do that!
   */

  priv->rxdma = NULL;
  priv->txdma = NULL;
  if (priv->config != SIMPLEX_TX)
    {
      priv->rxdma = stm32_dmachannel(priv->rxch);
      DEBUGASSERT(priv->rxdma);
      spi_modifyreg(priv, STM32_SPI_CFG1_OFFSET, 0, SPI_CFG1_RXDMAEN);
    }

  if (priv->config != SIMPLEX_RX)
    {
      priv->txdma = stm32_dmachannel(priv->txch);
      DEBUGASSERT(priv->txdma);
      spi_modifyreg(priv, STM32_SPI_CFG1_OFFSET, 0, SPI_CFG1_TXDMAEN);
    }
#endif

  /* Attach the IRQ to the driver */

  if (irq_attach(priv->spiirq, spi_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      spierr("ERROR: Can't initialize spi irq\n");
      return;
    }

  /* Enable SPI interrupts */

  up_enable_irq(priv->spiirq);

  /* Enable SPI */

  spi_enable(priv, true);

#ifdef CONFIG_PM
  /* Register to receive power management callbacks */

  ret = pm_register(&priv->pm_cb);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
#endif

#ifdef CONFIG_DEBUG_SPI_INFO
  /* Dump registers after initialization */

  spi_dumpregs(priv);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
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
 ****************************************************************************/

struct spi_dev_s *stm32_spibus_initialize(int bus)
{
  struct stm32_spidev_s *priv = NULL;

  irqstate_t flags = enter_critical_section();
#ifdef CONFIG_STM32H7_SPI1
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
#ifdef CONFIG_STM32H7_SPI2
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
#ifdef CONFIG_STM32H7_SPI3
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
#ifdef CONFIG_STM32H7_SPI4
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
#ifdef CONFIG_STM32H7_SPI5
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
#ifdef CONFIG_STM32H7_SPI6
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
  return (struct spi_dev_s *)priv;
}

#endif /* CONFIG_STM32H7_SPI1 || CONFIG_STM32H7_SPI2 || CONFIG_STM32H7_SPI3 ||
        * CONFIG_STM32H7_SPI4 || CONFIG_STM32H7_SPI5 || CONFIG_STM32H7_SPI6
        */
