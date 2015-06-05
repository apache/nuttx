/****************************************************************************
 * arm/arm/src/efm32/efm32_spi.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2014 Bouteville Pierre-Noel. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bouteville Pierre-Noel <pnb990@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "chip/efm32_usart.h"
#include "efm32_config.h"
#include "efm32_dma.h"
#include "efm32_lowputc.h"
#include "efm32_spi.h"
#include "efm32_gpio.h"

#ifdef HAVE_SPI_DEVICE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* SPI DMA */

#ifndef CONFIG_EFM32_SPI_DMA_TIMEO_NSEC
#  define CONFIG_EFM32_SPI_DMA_TIMEO_NSEC 500
#endif

#ifndef CONFIG_EFM32_SPI_DMA_MINSIZE
#  define CONFIG_EFM32_SPI_DMA_MINSIZE 16
#endif

/* DMA definitions **********************************************************/

#define SPI_DMA8_CONFIG       (EFM32_DMA_XFERSIZE_BYTE| EFM32_DMA_MEMINCR)
#define SPI_DMA8NULL_CONFIG   (EFM32_DMA_XFERSIZE_BYTE | EFM32_DMA_NOINCR)
#define SPI_DMA16_CONFIG      (EFM32_DMA_XFERSIZE_HWORD | EFM32_DMA_MEMINCR)
#define SPI_DMA16NULL_CONFIG  (EFM32_DMA_XFERSIZE_HWORD | EFM32_DMA_NOINCR)

/* Debug ********************************************************************/
/* Check if SPI debug is enabled */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_SPI
#endif

#ifdef CONFIG_DEBUG_SPI
#  define spidbg lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type constant configuration one SPI peripheral */

struct efm32_spidev_s;
struct efm32_spiconfig_s
{
  uintptr_t base;           /* USART base address */
#ifdef CONFIG_EFM32_SPI_DMA
  dma_config_t rxconfig;    /* RX DMA configuration */
  dma_config_t txconfig;    /* TX DMA configuration */
#endif

  /* SPI-specific methods */

  void (*select)(struct spi_dev_s *dev, enum spi_dev_e devid,
                 bool selected);
  uint8_t (*status)(struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
  int (*cmddata)(struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
};

/* This type represents the state of one SPI peripheral */

struct efm32_spidev_s
{
  const struct spi_ops_s *spidev;         /* Externally visible SPI interface */
  const struct efm32_spiconfig_s *config; /* Constant SPI hardware configuration */

#ifdef CONFIG_EFM32_SPI_DMA
  WDOG_ID wdog;              /* Timer to catch hung DMA */
  volatile uint8_t rxresult; /* Result of the RX DMA */
  volatile uint8_t txresult; /* Result of the TX DMA */
  DMA_HANDLE rxdmach;        /* RX DMA channel handle */
  DMA_HANDLE txdmach;        /* TX DMA channel handle */
  sem_t rxdmasem;            /* Wait for RX DMA to complete */
  sem_t txdmasem;            /* Wait for TX DMA to complete */
#endif

#ifndef CONFIG_SPI_OWNBUS
  sem_t exclsem;             /* Supports mutually exclusive access */
  uint32_t frequency;        /* Requested clock frequency */
  uint32_t actual;           /* Actual clock frequency */
  uint8_t mode;              /* Mode 0,1,2,3 */
#endif

  uint8_t nbits;             /* Width of word in bits (4-16) */
  bool lsbfirst;             /* True: Bit order is LSB first */
  bool initialized;          /* True: Already initialized */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Low level SPI access */

static uint32_t  spi_getreg(const struct efm32_spiconfig_s *config,
                   unsigned int regoffset);
static void      spi_putreg(const struct efm32_spiconfig_s *config,
                   unsigned int regoffset, uint32_t regval);
static void      spi_rxflush(const struct efm32_spiconfig_s *config);
static void      spi_wait_status(const struct efm32_spiconfig_s *config,
                   uint32_t mask, uint32_t match);

/* DMA support */

#ifdef CONFIG_EFM32_SPI_DMA
static void      spi_dma_timeout(int argc, uint32_t arg1, ...);
static void      spi_dmarxwait(struct efm32_spidev_s *priv);
static void      spi_dmatxwait(struct efm32_spidev_s *priv);
static inline void spi_dmarxwakeup(struct efm32_spidev_s *priv);
static inline void spi_dmatxwakeup(struct efm32_spidev_s *priv);
static void      spi_dmarxcallback(DMA_HANDLE handle, uint8_t status,
                   void *arg);
static void      spi_dmatxcallback(DMA_HANDLE handle, uint8_t status,
                   void *arg);
static void      spi_dmarxsetup(struct efm32_spidev_s *priv,
                   void *rxbuffer, void *rxdummy, size_t nwords);
static void      spi_dmatxsetup(struct efm32_spidev_s *priv,
                   const void *txbuffer, const void *txdummy,
                   size_t nwords);
static inline void spi_dmarxstart(FAR struct efm32_spidev_s *priv);
static inline void spi_dmatxstart(FAR struct efm32_spidev_s *priv);
#endif

/* SPI methods */

#ifndef CONFIG_SPI_OWNBUS
static int       spi_lock(struct spi_dev_s *dev, bool lock);
#endif
static void      spi_select(struct spi_dev_s *dev, enum spi_dev_e devid,
                   bool selected);
static uint32_t  spi_setfrequency(struct spi_dev_s *dev,
                   uint32_t frequency);
static void      spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void      spi_setbits(struct spi_dev_s *dev, int nbits);
static uint8_t   spi_status(struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
static int       spi_cmddata(struct spi_dev_s *dev, enum spi_dev_e devid,
                   bool cmd);
#endif
static uint16_t  spi_send(struct spi_dev_s *dev, uint16_t wd);
static void      spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                   void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void      spi_sndblock(struct spi_dev_s *dev, const void *txbuffer,
                   size_t nwords);
static void      spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                   size_t nwords);
#endif

/* Initialization */

static int       spi_portinitialize(struct efm32_spidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Common SPI operations */

static const struct spi_ops_s g_spiops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = spi_select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = spi_cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,
};

#ifdef CONFIG_EFM32_USART0_ISSPI
/* Support for SPI on USART0 */

static struct efm32_spidev_s g_spi0dev;
static const struct efm32_spiconfig_s g_spi0config =
{
  .base              = EFM32_USART0_BASE,
#ifdef CONFIG_EFM32_SPI_DMA
  .rxconfig          = EFM32_DMA_SIGSEL(_DMA_CH_CTRL_SIGSEL_USART0RXDATAV) |
                       EFM32_DMA_SOURCSEL(_DMA_CH_CTRL_SOURCESEL_USART0) |
                       EFM32_DMA_SINGLE,
  .txconfig          = EFM32_DMA_SIGSEL(_DMA_CH_CTRL_SIGSEL_USART0TXBL) |
                       EFM32_DMA_SOURCSEL(_DMA_CH_CTRL_SOURCESEL_USART0)  |
                       EFM32_DMA_SINGLE,
#endif
  .select            = efm32_spi0_select,
  .status            = efm32_spi0_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = efm32_spi0_cmddata,
#endif
};
#endif /* CONFIG_EFM32_USART0_ISSPI */

#ifdef CONFIG_EFM32_USART1_ISSPI
/* Support for SPI on USART1 */

static struct efm32_spidev_s g_spi1dev;
static const struct efm32_spiconfig_s g_spi1config =
{
  .base              = EFM32_USART1_BASE,
#ifdef CONFIG_EFM32_SPI_DMA
  .rxconfig          = EFM32_DMA_SIGSEL(_DMA_CH_CTRL_SIGSEL_USART1RXDATAV) |
                       EFM32_DMA_SOURCSEL(_DMA_CH_CTRL_SOURCESEL_USART1) |
                       EFM32_DMA_SINGLE,
  .txconfig          = EFM32_DMA_SIGSEL(_DMA_CH_CTRL_SIGSEL_USART1TXBL) |
                       EFM32_DMA_SOURCSEL(_DMA_CH_CTRL_SOURCESEL_USART1)  |
                       EFM32_DMA_SINGLE,
#endif
  .select            = efm32_spi1_select,
  .status            = efm32_spi1_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = efm32_spi1_cmddata,
#endif
};
#endif /* CONFIG_EFM32_USART1_ISSPI */

#ifdef CONFIG_EFM32_USART2_ISSPI
/* Support for SPI on USART2 */

static struct efm32_spidev_s g_spi2dev;
static const struct efm32_spiconfig_s g_spi2config =
{
  .base              = EFM32_USART2_BASE,
#ifdef CONFIG_EFM32_SPI_DMA
  .rxconfig          = EFM32_DMA_SIGSEL(_DMA_CH_CTRL_SIGSEL_USART2RXDATAV) |
                       EFM32_DMA_SOURCSEL(_DMA_CH_CTRL_SOURCESEL_USART2) |
                       EFM32_DMA_SINGLE,
  .txconfig          = EFM32_DMA_SIGSEL(_DMA_CH_CTRL_SIGSEL_USART2TXBL) |
                       EFM32_DMA_SOURCSEL(_DMA_CH_CTRL_SOURCESEL_USART2)  |
                       EFM32_DMA_SINGLE,
#endif
  .select            = efm32_spi2_select,
  .status            = efm32_spi2_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = efm32_spi2_cmddata,
#endif
};
#endif /* CONFIG_EFM32_USART2_ISSPI */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Read the contents of one SPI register
 *
 * Input Parameters:
 *   config    - Device-specific configuration data
 *   regoffset - Offset to the SPI register
 *
 * Returned Value:
 *   Value read from the SPI register
 *
 ****************************************************************************/

static uint32_t spi_getreg(const struct efm32_spiconfig_s *config,
                           unsigned int regoffset)
{
  uintptr_t regaddr = config->base + regoffset;
  return getreg32(regaddr);
}

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a value to one SPI register
 *
 * Input Parameters:
 *   config    - Device-specific configuration data
 *   regoffset - Offset to the SPI register
 *   regval    - The value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_putreg(const struct efm32_spiconfig_s *config,
                       unsigned int regoffset, uint32_t regval)
{
  uintptr_t regaddr = config->base + regoffset;
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: spi_rxflush
 *
 * Description:
 *  Flush any garbage from the RX buffer
 *
 ****************************************************************************/

static void spi_rxflush(const struct efm32_spiconfig_s *config)
{
  /* Loop while data is available */

  while ((spi_getreg(config, EFM32_USART_STATUS_OFFSET) & USART_STATUS_RXDATAV) != 0)
    {
      /* Read and discard the data */

      (void)spi_getreg(config, EFM32_USART_RXDATA_OFFSET);
    }
}

/****************************************************************************
 * Name: spi_wait_status
 *
 * Description:
 *   Poll until the SPI status under the mask is equal to the mask value.
 *
 ****************************************************************************/

static void spi_wait_status(const struct efm32_spiconfig_s *config,
                            uint32_t mask, uint32_t match)
{
  while ((spi_getreg(config, EFM32_USART_STATUS_OFFSET) & mask) != match);
}

/****************************************************************************
 * Name: spi_dma_timeout
 *
 * Description:
 *  Invoked when a DMA timeout occurs
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_SPI_DMA
static void spi_dma_timeout(int argc, uint32_t arg1, ...)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)((uintptr_t)arg1);

  /* Mark DMA timeout error and wakeup form RX and TX waiters */

  DEBUGASSERT(priv->rxresult == EINPROGRESS || priv->txresult == EINPROGRESS);
  if (priv->rxresult == EINPROGRESS)
    {
      priv->rxresult = ETIMEDOUT;
      spi_dmarxwakeup(priv);
    }

  if (priv->txresult == EINPROGRESS)
    {
      priv->txresult = ETIMEDOUT;
      spi_dmatxwakeup(priv);
    }
}
#endif

/****************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for RX DMA to complete.
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_SPI_DMA
static void spi_dmarxwait(struct efm32_spidev_s *priv)
{
  irqstate_t flags;

  /* Take the semaphore (perhaps waiting). */

  flags = irqsave();
  while (sem_wait(&priv->rxdmasem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(errno == EINTR);
    }

  /* Cancel the timeout only if both the RX and TX transfers have completed */

  DEBUGASSERT(priv->rxresult != EINPROGRESS);
  if (priv->txresult != EINPROGRESS)
    {
      wd_cancel(priv->wdog);
    }

  irqrestore(flags);
}
#endif

/****************************************************************************
 * Name: spi_dmatxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_SPI_DMA
static void spi_dmatxwait(struct efm32_spidev_s *priv)
{
  irqstate_t flags;

  /* Take the semaphore (perhaps waiting). */

  flags = irqsave();
  while (sem_wait(&priv->txdmasem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(errno == EINTR);
    }

  /* Cancel the timeout only if both the RX and TX transfers have completed */

  DEBUGASSERT(priv->txresult != EINPROGRESS);
  if (priv->rxresult != EINPROGRESS)
    {
      wd_cancel(priv->wdog);
    }

  irqrestore(flags);
}
#endif

/****************************************************************************
 * Name: spi_dmarxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_SPI_DMA
static inline void spi_dmarxwakeup(struct efm32_spidev_s *priv)
{
  (void)sem_post(&priv->rxdmasem);
}
#endif

/****************************************************************************
 * Name: spi_dmatxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_SPI_DMA
static inline void spi_dmatxwakeup(struct efm32_spidev_s *priv)
{
  (void)sem_post(&priv->txdmasem);
}
#endif

/****************************************************************************
 * Name: spi_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_SPI_DMA
static void spi_dmarxcallback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)arg;
  DEBUGASSERT(priv && status != EINPROGRESS);

  /* Wake-up the SPI driver */

  priv->rxresult = status;
  spi_dmarxwakeup(priv);
}
#endif

/****************************************************************************
 * Name: spi_dmatxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_SPI_DMA
static void spi_dmatxcallback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)arg;
  DEBUGASSERT(priv && status != EINPROGRESS);

  /* Wake-up the SPI driver */

  priv->txresult = status;
  spi_dmatxwakeup(priv);
}
#endif

/****************************************************************************
 * Name: spi_dmarxsetup
 *
 * Description:
 *   Setup to perform RX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_SPI_DMA
static void spi_dmarxsetup(struct efm32_spidev_s *priv, void *rxbuffer,
                           void *rxdummy, size_t nwords)
{
  const struct efm32_spiconfig_s *config = priv->config;
  dma_config_t dmaconfig = config->rxconfig;
  size_t nbytes;

  /* 8- or 16-bit mode? */

  if (priv->nbits > 8)
    {
      /* 16-bit mode -- is there a buffer to receive data in? */

      if (rxbuffer)
        {
          dmaconfig |= SPI_DMA16_CONFIG;
        }
      else
        {
          rxbuffer   = rxdummy;
          dmaconfig |= SPI_DMA16NULL_CONFIG;
        }

      nbytes = nwords << 1;
    }
  else
    {
      /* 8-bit mode -- is there a buffer to receive data in? */

      if (rxbuffer)
        {
          dmaconfig |= SPI_DMA8_CONFIG;
        }
      else
        {
          rxbuffer   = rxdummy;
          dmaconfig |= SPI_DMA8NULL_CONFIG;
        }

      nbytes = nwords;
    }

  /* Configure the RX DMA */

  efm32_rxdmasetup(priv->rxdmach, config->base + EFM32_USART_RXDATA_OFFSET,
                   (uintptr_t)rxbuffer, nbytes, dmaconfig);
}
#endif

/****************************************************************************
 * Name: spi_dmatxsetup
 *
 * Description:
 *   Setup to perform TX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_SPI_DMA
static void spi_dmatxsetup(struct efm32_spidev_s *priv, const void *txbuffer,
                           const void *txdummy, size_t nwords)
{
  const struct efm32_spiconfig_s *config = priv->config;
  dma_config_t dmaconfig = config->txconfig;
  size_t nbytes;

  /* 8- or 16-bit mode? */

  if (priv->nbits > 8)
    {
      /* 16-bit mode -- is there a buffer to receive data in? */

      if (txbuffer)
        {
          dmaconfig |= SPI_DMA16_CONFIG;
        }
      else
        {
          txbuffer   = txdummy;
          dmaconfig |= SPI_DMA16NULL_CONFIG;
        }

      nbytes = nwords << 1;
    }
  else
    {
      /* 8-bit mode -- is there a buffer to receive data in? */

      if (txbuffer)
        {
          dmaconfig |= SPI_DMA8_CONFIG;
        }
      else
        {
          txbuffer   = txdummy;
          dmaconfig |= SPI_DMA8NULL_CONFIG;
        }

      nbytes = nwords;
    }

  /* Configure the RX DMA */

  efm32_txdmasetup(priv->txdmach, config->base + EFM32_USART_TXDATA_OFFSET,
                   (uintptr_t)txbuffer, nbytes, dmaconfig);
}
#endif

/****************************************************************************
 * Name: spi_dmarxstart
 *
 * Description:
 *   Start RX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_SPI_DMA
static void spi_dmarxstart(FAR struct efm32_spidev_s *priv)
{
  priv->rxresult = EINPROGRESS;
  efm32_dmastart(priv->rxdmach, spi_dmarxcallback, priv);
}
#endif

/****************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start TX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_SPI_DMA
static inline void spi_dmatxstart(FAR struct efm32_spidev_s *priv)
{
  priv->txresult = EINPROGRESS;
  efm32_dmastart(priv->txdmach, spi_dmatxcallback, priv);
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

#ifndef CONFIG_SPI_OWNBUS
static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;

  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      while (sem_wait(&priv->exclsem) != 0)
        {
          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          ASSERT(errno == EINTR);
        }
    }
  else
    {
      (void)sem_post(&priv->exclsem);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   Enable/disable the SPI chip select.
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   devid -    Identifies the device to select
 *   selected - true: slave selected, false: slave de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_select(struct spi_dev_s *dev, enum spi_dev_e devid,
                       bool selected)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;

  DEBUGASSERT(priv && priv->config);
  config = priv->config;
  DEBUGASSERT(config->select);

  /* Defer to the board chip select logic */

  config->select(dev, devid, selected);
}

/****************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev       - Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;
  uint32_t clkdiv;
  uint32_t actual;

  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  /* Has the frequency changed? */

#ifndef CONFIG_SPI_OWNBUS
  if (frequency == priv->frequency)
    {
      /* No... just return the actual frequency from the last calcualtion */

      actual = priv->actual;
    }
  else
#endif
    {
      /* We want to use integer division to avoid forcing in float division
       * utils, and yet keep rounding effect errors to a minimum.
       *
       * CLKDIV in synchronous mode is given by:
       *
       * CLKDIV = 256 * (fHFPERCLK/(2 * br) - 1)
       * or
       * CLKDIV = (256 * fHFPERCLK)/(2 * br) - 256 = (128 * fHFPERCLK)/br - 256
       *
       * The basic problem with integer division in the above formula is that
       * the dividend (128 * fHFPERCLK) may become higher than max 32 bit
       * integer. Yet, we want to evaluate dividend first before dividing in
       * order to get as small rounding effects as possible. We do not want
       * to make too harsh restrictions on max fHFPERCLK value either.
       *
       * One can possibly factorize 128 and br. However, since the last
       * 6 bits of CLKDIV are don't care, we can base our integer arithmetic
       * on the below formula without loosing any extra precision:
       *
       * CLKDIV / 64 = (2 * fHFPERCLK)/br - 4
       *
       * and calculate 1/64 of CLKDIV first. This allows for fHFPERCLK
       * up to 2GHz without overflowing a 32 bit value!
       */

      /* Calculate and set CLKDIV with fractional bits */

      clkdiv  = (2 * BOARD_HFPERCLK_FREQUENCY + (frequency - 1)) / frequency;
      clkdiv  = 64 * (clkdiv - 4);

      /* Make sure we don't use fractional bits by rounding CLKDIV up (and
       * thus reducing baudrate, not increasing baudrate above.
       * specified value).
       */

      clkdiv = (clkdiv + 0xc0) & 0xffffff00;

      /* Verify that resulting clock divider is within limits */

      DEBUGASSERT(clkdiv <= _USART_CLKDIV_MASK);

      clkdiv &= _USART_CLKDIV_DIV_MASK;
      spi_putreg(config, EFM32_USART_CLKDIV_OFFSET, clkdiv);

      /* The actual frequency is then given by:
       *
       * br = fHFPERCLK / (2 * (1 + CLKDIV / 256))
       *    = 128 * fHFPERCLK / (256 + CLKDIV)
       */

      actual = (BOARD_HFPERCLK_FREQUENCY << 7) / ( 256 + clkdiv);
      spivdbg("frequency=%u actual=%u\n", frequency, actual);

#ifndef CONFIG_SPI_OWNBUS
      /* Save the frequency selection so that subsequent reconfigurations
       * will be faster.
       */

      priv->frequency = frequency;
      priv->actual    = actual;
#endif
    }

  return actual;
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
 *   Returns void
 *
 ****************************************************************************/

static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;
  uint32_t setting;
  uint32_t regval;

  spivdbg("mode=%d\n", mode);

  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  /* Has the mode changed? */

#ifndef CONFIG_SPI_OWNBUS
  if (mode != priv->mode)
    {
#endif
      setting = 0;
      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          setting = USART_CTRL_CLKPOL_IDLELOW | USART_CTRL_CLKPHA_SAMPLELEADING;
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          setting = USART_CTRL_CLKPOL_IDLELOW | USART_CTRL_CLKPHA_SAMPLETRAILING;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          setting = USART_CTRL_CLKPOL_IDLEHIGH | USART_CTRL_CLKPHA_SAMPLELEADING;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          setting = USART_CTRL_CLKPOL_IDLEHIGH | USART_CTRL_CLKPHA_SAMPLETRAILING;
          break;

        default:
          return;
        }

      regval  = spi_getreg(config, EFM32_USART_CTRL_OFFSET);
      regval &= ~(_USART_CTRL_CLKPOL_MASK | _USART_CTRL_CLKPHA_MASK);
      regval |= setting;
      spi_putreg(config, EFM32_USART_CLKDIV_OFFSET, regval);

#ifndef CONFIG_SPI_OWNBUS
      /* Save the mode so that subsequent re-configurations will be faster */

        priv->mode = mode;
    }
#endif
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
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;
  uint32_t regval;
  uint32_t setting;
  bool lsbfirst;

  spivdbg("nbits=%d\n", nbits);

  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  /* Bit order is encoded by the sign of nbits */

  if (nbits < 0)
    {
      /* LSB first */

      lsbfirst = true;
      nbits    = -nbits;
    }
  else
    {
      /* MSH first */

      lsbfirst = false;
    }

  /* Has the number of bits or the bit order changed? */

  if (nbits != priv->nbits || lsbfirst != priv->lsbfirst)
    {
      /* Set the new bit order */

      regval = spi_getreg(config, EFM32_USART_CTRL_OFFSET);
      if (lsbfirst)
        {
          regval &= ~USART_CTRL_MSBF;
        }
      else
        {
          regval |= USART_CTRL_MSBF;
        }

      spi_putreg(config, EFM32_USART_CTRL_OFFSET, regval);

      /* Select the new number of bits */

      switch (nbits)
        {
        case  4:
          setting = USART_FRAME_DATABITS_FOUR;
          break;

        case  5:
          setting = USART_FRAME_DATABITS_FIVE;
          break;

        case  6:
          setting = USART_FRAME_DATABITS_SIX;
          break;

        case  7:
          setting = USART_FRAME_DATABITS_SEVEN;
          break;

        case  8:
          setting = USART_FRAME_DATABITS_EIGHT;
          break;

        case  9:
          setting = USART_FRAME_DATABITS_NINE;
          break;

        case  10:
          setting = USART_FRAME_DATABITS_TEN;
          break;

        case  11:
          setting = USART_FRAME_DATABITS_ELEVEN;
          break;

        case  12:
          setting = USART_FRAME_DATABITS_TWELVE;
          break;

        case  13:
          setting = USART_FRAME_DATABITS_THIRTEEN;
          break;

        case  14:
          setting = USART_FRAME_DATABITS_FOURTEEN;
          break;

        case  15:
          setting = USART_FRAME_DATABITS_FIFTEEN;
          break;

        case  16:
          setting = USART_FRAME_DATABITS_SIXTEEN;
          break;

        default:
          return;
        }

      regval  = spi_getreg(config, EFM32_USART_FRAME_OFFSET);
      regval &= _USART_FRAME_DATABITS_MASK;
      regval |= setting;
      spi_putreg(config, EFM32_USART_FRAME_OFFSET, regval);

      /* Save the selection so the subsequence re-configurations will be
       * faster
       */

      priv->nbits    = nbits;
      priv->lsbfirst = lsbfirst;
    }
}

/****************************************************************************
 * Name: spi_status
 *
 * Description:
 *   Get SPI/MMC status.  Optional.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   devid - Identifies the device to report status on
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines)
 *
 ****************************************************************************/

static uint8_t spi_status(struct spi_dev_s *dev, enum spi_dev_e devid)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;

  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  /* Defer to the board chip status logic */

  return config->status(dev, devid);
}

/****************************************************************************
 * Name: spi_cmddata
 *
 * Description:
 *   Some devices require and additional out-of-band bit to specify if the
 *   next word sent to the device is a command or data. This is typical, for
 *   example, in "9-bit" displays where the 9th bit is the CMD/DATA bit.
 *   This function provides selection of command or data.
 *
 *   This "latches" the CMD/DATA state.  It does not have to be called before
 *   every word is transferred; only when the CMD/DATA state changes.  This
 *   method is required if CONFIG_SPI_CMDDATA is selected in the NuttX
 *   configuration
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   cmd - TRUE: The following word is a command; FALSE: the following words
 *         are data.
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(struct spi_dev_s *dev, enum spi_dev_e devid,
                       bool cmd);
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;

  DEBUGASSERT(priv && priv->config);
  config = priv->config;
  DEBUGASSERT(config->cmddata);

  /* Defer to the board chip cmd/data logic */

  return config->cmddata(dev, devid, cmd);
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
 *   Response
 *
 ****************************************************************************/

static uint16_t spi_send(struct spi_dev_s *dev, uint16_t wd)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;
  uint16_t ret;

  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  /* Wait until there is space in the TX buffer */

  spi_wait_status(config, _USART_STATUS_TXBL_MASK, USART_STATUS_TXBL);

  /* Flush any unread data */

  spi_rxflush(config);

  /* Write the data */

  spi_putreg(config, EFM32_USART_TXDATA_OFFSET, (uint32_t)wd);

  /* Wait for receive data to be available */

  spi_wait_status(config, _USART_STATUS_RXDATAV_MASK, USART_STATUS_RXDATAV);
  ret = (uint16_t)spi_getreg(config, EFM32_USART_RXDATA_OFFSET);

  spivdbg("Sent: %04x Return: %04x \n", wd, ret);
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

#if defined(CONFIG_EFM32_SPI_DMA) && CONFIG_EFM32_SPI_DMA_MINSIZE > 0
static void spi_exchange_nodma(struct spi_dev_s *dev, const void *txbuffer,
                               void *rxbuffer, size_t nwords)
#else
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
#endif
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;
  size_t unrecvd;
  size_t unsent;

  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  spivdbg("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* Flush any unread data */

  spi_rxflush(config);

  /* 8- or 16-bit mode? */

  if (priv->nbits > 8)
    {
      /* 16-bit mode */

      const uint16_t *src  = (const uint16_t*)txbuffer;;
            uint16_t *dest = (uint16_t*)rxbuffer;
            uint16_t  word;

      unrecvd = nwords;
      unsent  = nwords;
      while (unrecvd > 0)
        {
          /* REVISIT:  Could this cause RX data overruns??? */
          /* Send data if there is space in the TX buffer. */

          if ((spi_getreg(config, EFM32_USART_STATUS_OFFSET) & USART_STATUS_TXBL) != 0 &&
              unsent > 0)
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

              /* Send one word */

              spi_putreg(config, EFM32_USART_TXDATA_OFFSET, (uint32_t)word);
              unsent--;
            }

          /* Receive data if there is data available */

          if ((spi_getreg(config, EFM32_USART_STATUS_OFFSET) & USART_STATUS_RXDATAV) != 0 &&
              unrecvd > 0)
            {
              /* Receive the data */

              word = (uint16_t)spi_getreg(config, EFM32_USART_RXDATA_OFFSET);
              unrecvd--;

              /* Is there a buffer to receive the return value? */

              if (dest)
                {
                  *dest++ = word;
                }
            }
        }
    }
  else
    {
      /* 8-bit mode */

      const uint8_t *src  = (const uint8_t*)txbuffer;;
            uint8_t *dest = (uint8_t*)rxbuffer;
            uint8_t  word;

      unrecvd = nwords;
      unsent  = nwords;
      while (unrecvd > 0)
        {
          /* REVISIT:  Could this cause RX data overruns??? */
          /* Send data if there is space in the TX buffer. */

          if ((spi_getreg(config, EFM32_USART_STATUS_OFFSET) & USART_STATUS_TXBL) != 0 &&
                 unsent > 0)
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

              /* Send one word */

              spi_putreg(config, EFM32_USART_TXDATA_OFFSET, (uint32_t)word);
              unsent--;
            }

          /* Receive data if there is data available */

          if ((spi_getreg(config, EFM32_USART_STATUS_OFFSET) & USART_STATUS_RXDATAV) != 0 &&
              unrecvd > 0)
            {
              /* Receive the data */

              word = (uint8_t)spi_getreg(config, EFM32_USART_RXDATA_OFFSET);
              unrecvd--;

              /* Is there a buffer to receive the return value? */

              if (dest)
                {
                  *dest++ = word;
                }
            }
        }
    }

  DEBUGASSERT(unsent == 0);
}

/*************************************************************************
 * Name: spi_exchange (with DMA capability)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - The length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_SPI_DMA
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  static uint16_t rxdummy = 0xffff;
  static const uint16_t txdummy = 0xffff;
  irqstate_t flags;
  uint64_t ticks;
  int ret;

  DEBUGASSERT(priv && priv->config);

#if CONFIG_EFM32_SPI_DMA_MINSIZE > 0
  if (nwords <= CONFIG_EFM32_SPI_DMA_MINSIZE)
    {
      /* Small transfer, fall back to non-DMA method. */

      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
    }
  else
#endif
    {
      spivdbg("txbuffer=%p rxbuffer=%p nwords=%d\n",
              txbuffer, rxbuffer, nwords);

      /* Pre-calculate the timeout value */

      ticks = (CONFIG_EFM32_SPI_DMA_TIMEO_NSEC * nwords) / NSEC_PER_TICK;
      if (ticks < 1)
        {
          ticks = 1;
        }

      /* Setup DMAs */

      spi_dmarxsetup(priv, rxbuffer, &rxdummy, nwords);
      spi_dmatxsetup(priv, txbuffer, &txdummy, nwords);

      /* Start the DMAs */

      flags = irqsave();
      spi_dmarxstart(priv);
      spi_dmatxstart(priv);

      /* Start a timer to catch hung DMA transfers.  Timeout will be canceled
       * when both RX and TX transfers complete.
       */

      ret = wd_start(priv->wdog, (int)ticks, spi_dma_timeout, 1, (uint32_t)priv);
      if (ret < 0)
        {
          spidbg("ERROR: Failed to start timeout\n");
        }

      /* Then wait for each to complete.  TX should complete first */

      spi_dmatxwait(priv);
      spi_dmarxwait(priv);
      irqrestore(flags);
    }
}
#endif /* CONFIG_EFM32_SPI_DMA */

/*************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - The length of data to send from the buffer in number of
 *              words.  The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface.  If nbits
 *              <= 8, the data is packed into uint8_t's; if nbits >8, the
 *              data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev, const void *txbuffer,
                         size_t nwords)
{
  spivdbg("txbuffer=%p nwords=%d\n", txbuffer, nwords);
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
 *   rxbuffer - A pointer to the buffer in which to recieve data
 *   nwords   - The length of data that can be received in the buffer in
 *              number of words.  The wordsize is determined by the number
 *              of bits-per-word selected for the SPI interface.  If nbits
 *              <= 8, the data is packed into uint8_t's; if nbits >8, the
 *              data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                          size_t nwords)
{
  spivdbg("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: spi_portinitialize
 *
 * Description:
 *   Initialize the selected SPI port in its default state (Master, 8-bit,
 *   mode 0, etc.)
 *
 * Input Parameter:
 *   priv - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int spi_portinitialize(struct efm32_spidev_s *priv)
{
  const struct efm32_spiconfig_s *config = priv->config;
  uint32_t regval;

  /* Initialize USART registers to HW reset state. */

  efm32_uart_reset(config->base);

  /* NOTES:
   *
   * 1. USART GPIO pins were configured in efm32_lowsetup().  Chip select
   *    pins must be configured by board specific logic before
   *    efm32_spi_initialize() is called.
   * 2. Clocking for the USART as also enabled in efm32_lowsetup();
   */

  /* Set bits for synchronous mode */

  regval = _USART_CTRL_RESETVALUE | USART_CTRL_SYNC | USART_CTRL_CLKPOL_IDLELOW |
            USART_CTRL_CLKPHA_SAMPLELEADING;

  /* MSB First, 8 bits */

  regval &= ~_USART_FRAME_DATABITS_MASK;
  regval |= USART_FRAME_DATABITS_EIGHT | USART_CTRL_MSBF;
  spi_putreg(config, EFM32_USART_CTRL_OFFSET, regval);

#ifndef CONFIG_SPI_OWNBUS
  priv->frequency = 0;
  priv->mode      = SPIDEV_MODE0;
#endif
  priv->nbits     = 8;
  priv->lsbfirst  = false;

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Master mode */

  spi_putreg(config, EFM32_USART_CMD_OFFSET, USART_CMD_MASTEREN);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

#ifndef CONFIG_SPI_OWNBUS
  sem_init(&priv->exclsem, 0, 1);
#endif

#ifdef CONFIG_EFM32_SPI_DMA
  /* Allocate two DMA channels... one for the RX and one for the TX side of
   * the transfer.
   */

  priv->rxdmach = efm32_dmachannel();
  if (!priv->rxdmach)
    {
      spidbg("ERROR: Failed to allocate the RX DMA channel for SPI port: %d\n",
             port);
      goto errout;
    }

  priv->txdmach = efm32_dmachannel();
  if (!priv->txdmach)
    {
      spidbg("ERROR: Failed to allocate the TX DMA channel for SPI port: %d\n",
             port);
      goto errout_with_rxdmach;
    }

  /* Allocate a timer to catch hung DMA transfers */

  priv->wdog = wd_create();
  if (!priv->wdog)
    {
      spidbg("ERROR: Failed to create a timer for SPI port: %d\n", port);
      goto errout_with_txdmach;
    }

  /* Initialized semaphores used to wait for DMA completion */

  (void)sem_init(&priv->rxdmasem, 0, 0);
  (void)sem_init(&priv->txdmasem, 0, 0);
#endif

  /* Enable SPI */

  spi_putreg(config, EFM32_USART_CMD_OFFSET, USART_CMD_RXEN | USART_CMD_TXEN);
  return OK;

#ifdef CONFIG_EFM32_SPI_DMA
errout_with_txdmach:
  efm32_dmafree(priv->txdmach);
  priv->txdmach = NULL;

errout_with_rxdmach:
  efm32_dmafree(priv->rxdmach);
  priv->rxdmach = NULL;

errout:
  return -EBUSY;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_spi_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameter:
 *   port - SPI port number to initialize.  One of {0,1,2}
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *efm32_spi_initialize(int port)
{
  const struct efm32_spiconfig_s *config;
  struct efm32_spidev_s *priv;
  irqstate_t flags;
  int ret;

#ifdef CONFIG_EFM32_USART0_ISSPI
  if (port == 0)
    {
      priv   = &g_spi0dev;
      config = &g_spi0config;
      if (!priv->initialized)
        {
          efm32_configgpio(BOARD_SPI0_CLK);
          efm32_configgpio(BOARD_SPI0_MOSI);
          efm32_configgpio(BOARD_SPI0_MISO);
        }
    }
  else
#endif
#ifdef CONFIG_EFM32_USART1_ISSPI
  if (port == 1)
    {
      priv   = &g_spi1dev;
      config = &g_spi1config;
      if (!priv->initialized)
        {
          efm32_configgpio(BOARD_SPI1_CLK);
          efm32_configgpio(BOARD_SPI1_MOSI);
          efm32_configgpio(BOARD_SPI1_MISO);
        }
    }
  else
#endif
#ifdef CONFIG_EFM32_USART2_ISSPI
  if (port == 2)
    {
      priv   = &g_spi2dev;
      config = &g_spi2config;
      if (!priv->initialized)
        {
          efm32_configgpio(BOARD_SPI2_CLK);
          efm32_configgpio(BOARD_SPI2_MOSI);
          efm32_configgpio(BOARD_SPI2_MISO);
        }
    }
  else
#endif
    {
      spidbg("ERROR: Unsupported SPI port: %d\n", port);
      return NULL;
    }

  /* Has this port been initialized yet? */

  if (!priv->initialized)
    {
      /* No, then initialize it now */

      flags = irqsave();

      /* Initialize the state structure */

      priv->spidev    = &g_spiops;
      priv->config     = config;

      /* Initialize the SPI device */

       ret = spi_portinitialize(priv);
       if (ret < 0)
         {
           spidbg("ERROR: Failed to initialize SPI port %d\n", port);
           irqrestore(flags);
           return NULL;
         }

       /* Now we are initialized */

       priv->initialized = true;
       irqrestore(flags);
    }

  return (struct spi_dev_s *)priv;
}

#endif /* HAVE_SPI_DEVICE */
