/************************************************************************************
 * arch/arm/src/max326/max326_spi.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

/************************************************************************************
 * The external functions, max326_spi1/2/3select and max326_spi1/2/3status must be
 * provided by board-specific logic.  They are implementations of the select
 * and status methods of the SPI interface defined by struct spi_ops_s (see
 * include/nuttx/spi/spi.h). All other methods (including max326_spibus_initialize())
 * are provided by common MAX326 logic.  To use this common SPI logic on your
 * board:
 *
 *   1. Provide logic in max326_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide max326_spi1/2/3select() and max326_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to max326_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by max326_spibus_initialize() may then be used to bind
 *      the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************c********************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "hardware/max326_pinmux.h"
#include "max326_clockconfig.h"
#include "max326_gpio.h"
#include "max326_dma.h"
#include "max326_spim.h"

#include <arch/board/board.h>

#if defined(CONFIG_MAX326XX_HAVE_SPIM)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/
/* SPI interrupts */

#ifdef CONFIG_MAX326_SPI_INTERRUPTS
#  error "Interrupt driven SPI not yet supported"
#endif

/* Can't have both interrupt driven SPI and SPI DMA */

#if defined(CONFIG_MAX326_SPI_INTERRUPTS) && defined(CONFIG_MAX326_SPI_DMA)
#  error "Cannot enable both interrupt mode and DMA mode for SPI"
#endif

/* SPI DMA priority */

#ifdef CONFIG_MAX326_SPI_DMA

#  if defined(CONFIG_SPI_DMAPRIO)
#    define SPI_DMA_PRIO  CONFIG_SPI_DMAPRIO
#  elif defined(CONFIG_MAX326_MAX326F10XX) || defined(CONFIG_MAX326_MAX326L15XX) || \
        defined(CONFIG_MAX326_MAX326F30XX)
#    define SPI_DMA_PRIO  DMA_CCR_PRIMED
#  elif defined(CONFIG_MAX326_MAX326F20XX) || defined(CONFIG_MAX326_MAX326F4XXX)
#    define SPI_DMA_PRIO  DMA_SCR_PRIMED
#  else
#    error "Unknown MAX326 DMA"
#  endif

#  if defined(CONFIG_MAX326_MAX326F10XX) || defined(CONFIG_MAX326_MAX326L15XX) || \
      defined(CONFIG_MAX326_MAX326F30XX)
#    if (SPI_DMA_PRIO & ~DMA_CCR_PL_MASK) != 0
#      error "Illegal value for CONFIG_SPI_DMAPRIO"
#    endif
#  elif defined(CONFIG_MAX326_MAX326F20XX) || defined(CONFIG_MAX326_MAX326F4XXX)
#    if (SPI_DMA_PRIO & ~DMA_SCR_PL_MASK) != 0
#      error "Illegal value for CONFIG_SPI_DMAPRIO"
#    endif
#  else
#    error "Unknown MAX326 DMA"
#  endif

#endif

/* DMA channel configuration */

#define SPI_RXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_SCR_MSIZE_16BITS|DMA_SCR_PSIZE_16BITS|DMA_SCR_MINC|DMA_SCR_DIR_P2M)
#define SPI_RXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS |DMA_SCR_MINC|DMA_SCR_DIR_P2M)
#define SPI_RXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_16BITS             |DMA_SCR_DIR_P2M)
#define SPI_RXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS              |DMA_SCR_DIR_P2M)
#define SPI_TXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_SCR_MSIZE_16BITS|DMA_SCR_PSIZE_16BITS|DMA_SCR_MINC|DMA_SCR_DIR_M2P)
#define SPI_TXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS |DMA_SCR_MINC|DMA_SCR_DIR_M2P)
#define SPI_TXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_16BITS             |DMA_SCR_DIR_M2P)
#define SPI_TXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS              |DMA_SCR_DIR_M2P)

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct max326_spidev_s
{
  struct spi_dev_s spidev;       /* Externally visible part of the SPI interface */
  uint32_t         spibase;      /* SPIn base address */
#ifdef CONFIG_MAX326_SPI_INTERRUPTS
  uint8_t          spiirq;       /* SPI IRQ number */
#endif
#ifdef CONFIG_MAX326_SPI_DMA
  volatile uint8_t rxresult;     /* Result of the RX DMA */
  volatile uint8_t txresult;     /* Result of the RX DMA */
  uint8_t          rxch;         /* The RX DMA channel number */
  uint8_t          txch;         /* The TX DMA channel number */
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

static inline uint32_t spi_getreg(FAR struct max326_spidev_s *priv,
                                  unsigned int offset);
static inline void spi_putreg(FAR struct max326_spidev_s *priv,
                              unsigned int offset, uint32_t value);
static inline uint16_t spi_readword(FAR struct max326_spidev_s *priv);
static inline void spi_writeword(FAR struct max326_spidev_s *priv, uint16_t byte);
static inline bool spi_16bitmode(FAR struct max326_spidev_s *priv);

/* DMA support */

#ifdef CONFIG_MAX326_SPI_DMA
static void        spi_dmarxwait(FAR struct max326_spidev_s *priv);
static void        spi_dmatxwait(FAR struct max326_spidev_s *priv);
static inline void spi_dmarxwakeup(FAR struct max326_spidev_s *priv);
static inline void spi_dmatxwakeup(FAR struct max326_spidev_s *priv);
static void        spi_dmarxcallback(DMA_HANDLE handle, uint8_t isr, void *arg);
static void        spi_dmatxcallback(DMA_HANDLE handle, uint8_t isr, void *arg);
static void        spi_dmarxsetup(FAR struct max326_spidev_s *priv,
                                  FAR void *rxbuffer, FAR void *rxdummy,
                                  size_t nwords);
static void        spi_dmatxsetup(FAR struct max326_spidev_s *priv,
                                  FAR const void *txbuffer, FAR const void *txdummy,
                                  size_t nwords);
static inline void spi_dmarxstart(FAR struct max326_spidev_s *priv);
static inline void spi_dmatxstart(FAR struct max326_spidev_s *priv);
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
static uint16_t    spi_send(FAR struct spi_dev_s *dev, uint16_t wd);
static void        spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                                FAR void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void        spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                                size_t nwords);
static void        spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
                                 size_t nwords);
#endif

/* Initialization */

static void        spi_bus_initialize(FAR struct max326_spidev_s *priv);

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_MAX326XX_SPIM0
static const struct spi_ops_s g_sp0iops =
{
  .lock              = spi_lock,
  .select            = max326_spi0select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = max326_spi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = max326_spi0cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = max326_spi0register,  /* Provided externally */
#else
  .registercallback  = 0,                   /* Not implemented */
#endif
};

static struct max326_spidev_s g_spi0dev =
{
  .spidev   = { &g_sp0iops },
  .spibase  = MAX326_SPI0_BASE,
#ifdef CONFIG_MAX326_SPI_INTERRUPTS
  .spiirq   = MAX326_IRQ_SPI,
#endif
#ifdef CONFIG_MAX326_SPI_DMA
#  ifdef CONFIG_MAX326XX_SPIM0_DMA
  .rxch     = DMACHAN_SPI0_RX,
  .txch     = DMACHAN_SPI0_TX,
#  else
  .rxch     = 0,
  .txch     = 0,
#  endif
#endif
};
#endif

#ifdef CONFIG_MAX326XX_SPIM1
static const struct spi_ops_s g_sp1iops =
{
  .lock              = spi_lock,
  .select            = max326_spi1select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = max326_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = max326_spi1cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = max326_spi1register,  /* Provided externally */
#else
  .registercallback  = 0,                   /* Not implemented */
#endif
};

static struct max326_spidev_s g_spi1dev =
{
  .spidev   = { &g_sp1iops },
  .spibase  = MAX326_SPIMSS_BASE,
#ifdef CONFIG_MAX326_SPI_INTERRUPTS
  .spiirq   = MAX326_IRQ_SPIMSS,
#endif
#ifdef CONFIG_MAX326_SPI_DMA
#  ifdef CONFIG_MAX326XX_SPIM1_DMA
  .rxch     = DMACHAN_SPI1_RX,
  .txch     = DMACHAN_SPI1_TX,
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

static inline uint32_t spi_getreg(FAR struct max326_spidev_s *priv,
                                  unsigned int offset)
{
  return getreg32(priv->spibase + offset);
}

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

static inline void spi_putreg(FAR struct max326_spidev_s *priv, unsigned int offset,
                              uint32_t value)
{
  putreg32(value, priv->spibase + offset);
}

/************************************************************************************
 * Name: spi_readword
 *
 * Description:
 *   Read one word from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   Word as read
 *
 ************************************************************************************/

static inline uint16_t spi_readword(FAR struct max326_spidev_s *priv)
{
  /* Wait until the receive buffer is not empty */

  while ((spi_getreg(priv, MAX326_SPI_SR_OFFSET) & SPI_SR_RXNE) == 0)
    {
    }

  /* Then return the received word */

  return spi_getreg(priv, MAX326_SPI_DR_OFFSET);
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

static inline void spi_writeword(FAR struct max326_spidev_s *priv, uint16_t word)
{
  /* Wait until the transmit buffer is empty */

  while ((spi_getreg(priv, MAX326_SPI_SR_OFFSET) & SPI_SR_TXE) == 0)
    {
    }

  /* Then send the word */

  spi_putreg(priv, MAX326_SPI_DR_OFFSET, word);
}

/************************************************************************************
 * Name: spi_16bitmode
 *
 * Description:
 *   Check if the SPI is operating in 16-bit mode
 *
 * Input Parameters:
 *   priv     - Device-specific state data
 *
 * Returned Value:
 *   true: 16-bit mode, false: 8-bit mode
 *
 ************************************************************************************/

static inline bool spi_16bitmode(FAR struct max326_spidev_s *priv)
{
  return ((spi_getreg(priv, MAX326_SPI_CR1_OFFSET) & SPI_CR1_DFF) != 0);
}

/************************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ************************************************************************************/

#ifdef CONFIG_MAX326_SPI_DMA
static void spi_dmarxwait(FAR struct max326_spidev_s *priv)
{
  int ret;

  /* Take the semaphore (perhaps waiting).  If the result is zero, then the DMA
   * must not really have completed???
   */

  do
    {
      ret = nxsem_wait(&priv->rxsem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR || priv->rxresult == 0);
}
#endif

/************************************************************************************
 * Name: spi_dmatxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ************************************************************************************/

#ifdef CONFIG_MAX326_SPI_DMA
static void spi_dmatxwait(FAR struct max326_spidev_s *priv)
{
  int ret;

  /* Take the semaphore (perhaps waiting).  If the result is zero, then the DMA
   * must not really have completed???
   */

  do
    {
      ret = nxsem_wait(&priv->txsem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR || priv->txresult == 0);
}
#endif

/************************************************************************************
 * Name: spi_dmarxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ************************************************************************************/

#ifdef CONFIG_MAX326_SPI_DMA
static inline void spi_dmarxwakeup(FAR struct max326_spidev_s *priv)
{
  (void)nxsem_post(&priv->rxsem);
}
#endif

/************************************************************************************
 * Name: spi_dmatxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ************************************************************************************/

#ifdef CONFIG_MAX326_SPI_DMA
static inline void spi_dmatxwakeup(FAR struct max326_spidev_s *priv)
{
  (void)nxsem_post(&priv->txsem);
}
#endif

/************************************************************************************
 * Name: spi_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ************************************************************************************/

#ifdef CONFIG_MAX326_SPI_DMA
static void spi_dmarxcallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
  FAR struct max326_spidev_s *priv = (FAR struct max326_spidev_s *)arg;

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

#ifdef CONFIG_MAX326_SPI_DMA
static void spi_dmatxcallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
  FAR struct max326_spidev_s *priv = (FAR struct max326_spidev_s *)arg;

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

#ifdef CONFIG_MAX326_SPI_DMA
static void spi_dmarxsetup(FAR struct max326_spidev_s *priv, FAR void *rxbuffer,
                           FAR void *rxdummy, size_t nwords)
{
  /* 8- or 16-bit mode? */

  if (spi_16bitmode(priv))
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

  max326_dmasetup(priv->rxdma, priv->spibase + MAX326_SPI_DR_OFFSET,
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

#ifdef CONFIG_MAX326_SPI_DMA
static void spi_dmatxsetup(FAR struct max326_spidev_s *priv,
                           FAR const void *txbuffer, FAR const void *txdummy,
                           size_t nwords)
{
  /* 8- or 16-bit mode? */

  if (spi_16bitmode(priv))
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

  max326_dmasetup(priv->txdma, priv->spibase + MAX326_SPI_DR_OFFSET,
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

#ifdef CONFIG_MAX326_SPI_DMA
static inline void spi_dmarxstart(FAR struct max326_spidev_s *priv)
{
  priv->rxresult = 0;
  max326_dmastart(priv->rxdma, spi_dmarxcallback, priv, false);
}
#endif

/************************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start TX DMA
 *
 ************************************************************************************/

#ifdef CONFIG_MAX326_SPI_DMA
static inline void spi_dmatxstart(FAR struct max326_spidev_s *priv)
{
  priv->txresult = 0;
  max326_dmastart(priv->txdma, spi_dmatxcallback, priv, false);
}
#endif

/************************************************************************************
 * Name: spi_modigy_ctrl0
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

static void spi_modigy_ctrl0(FAR struct max326_spidev_s *priv, uint32_t setbits,
                             uint32_t clrbits)
{
  uint32_t ctrl0;

  ctrl0  = spi_getreg(priv, MAX326_SPI_CTRL0_OFFSET);
  ctrl0 &= ~clrbits;
  ctrl0 |= setbits;
  spi_putreg(priv, MAX326_SPI_CTRL0_OFFSET, ctrl0);
}

/************************************************************************************
 * Name: spi_modigy_ctrl1
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

#ifdef CONFIG_MAX326_SPI_DMA
static void spi_modigy_ctrl1(FAR struct max326_spidev_s *priv, uint32_t setbits,
                             uint32_t clrbits)
{
  uint32_t ctrl1;

  ctrl1  = spi_getreg(priv, MAX326_SPI_CTRL1_OFFSET);
  ctrl1 &= ~clrbits;
  ctrl1 |= setbits;
  spi_putreg(priv, MAX326_SPI_CTRL1_OFFSET, ctrl1);
}
#endif

/************************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
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
  FAR struct max326_spidev_s *priv = (FAR struct max326_spidev_s *)dev;
  int ret;

  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      do
        {
          ret = nxsem_wait(&priv->exclsem);

          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          DEBUGASSERT(ret == OK || ret == -EINTR);
        }
      while (ret == -EINTR);
    }
  else
    {
      (void)nxsem_post(&priv->exclsem);
      ret = OK;
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
  FAR struct max326_spidev_s *priv = (FAR struct max326_spidev_s *)dev;

  /* Has the frequency changed? */

  if (frequency != priv->frequency)
    {
      uint32_t pclk;
      uint32_t tmpbaud;
      uint32_t setbits;
      uint32_t actual;
      uint32_t error;
      uint32_t regval;
      unsigned int tmpscale;
      unsigned int tmphigh;
      unsigned int scale;
      unsigned int high;

      /* The SPI clock derives from the PCLK:
       *
       *   Fspi = Fpclk / (1 << CLKCFG.scale)
       *
       * The SPI high time is determined by Tspi and CLKCFG.high:
       *
       *   Thi = Tspi * CLKCFG.high
       *
       * Similar the SPI low time id determined by Tspi and CLKCFG.low:
       *
       *   Tlow = Tspi * CLKCFG.low
       *
       * The BAUD is the given by:
       *
       *   Fbaud = 1 / (Thi + Tlow)
       *
       * If we assume that Thi == Tlow, then:
       *
       *   Thi   = Tspi * CLKCFG.high
       *   Fbaud = 1 / (2 * Thi)
       *
       * And
       *
       *   Fbaud       = Fpclk / CLKCFG.high / (1 << (CLKCFG.scale + 1)
       *   CLKCFG.high = Fpclk / (1 << (CLKCFG.scale + 1)) / Fbaud
       *
       * Example:  Fpclk = 48MHz, Fbaud=1MHz
       *
       *   scale  CLKCFG.high Resulting Fbaud
       *     0    24         1,000,0000 (exact)
       *     1    12         1,000,0000 (exact)
       *     2    6          1,000,0000 (exact)
       *     3    3          1,000,0000 (exact)
       *     4    1          1,500,0000
       *     5    0          Invalid
       *     6    0          Invalid
       *     7    0          Invalid
       *     8    0          Invalid
       *
       *   Fbaud / 2 = Fspi / CLKCFG.high
       *   Fbaud = 2 * Fspi / CLKCFG.high
       */

      /* There are only 9 possible values for CLKCFG.scale.  Let's try them
       * all and pick the best.
       */

      pclk  = max326_pclk_frequency();
      error = UINT32_MAX;

      for (tmpscale = 0; tmpscale < 9; tmpscale++)
        {
          tmphigh = (pclk / frequency) >> (tmpscale + 1);
          if (tmphigh > 0)
            {
              uint32_t tmperr;

              /* Calculate the frequency difference */

              tmpbaud = (pclk / tmphigh) >> (tmpscale + 1);
              if (tmpbaud > frequency)
                {
                  tmperr = tmpbaud - frequency;
                }
              else
                {
                  tmperr = frequency - tmpbaud;
                }

              /* Save the best result we find (or the lowest scale in case
               * of ties).
               */

              if (tmperr < error)
                {
                  error  = tmperr;
                  actual = tmpbaud
                  scale  = tmpscale;
                  high   = tmphigh;
                }
            }
        }

      regval  = spi_getreg(priv, MAX326_SPI_CLKCFG_OFFSET);
      regval &= ~(SPI_CLKCFG_LO_MASK | SPI_CLKCFG_HI_MASK | SPI_CLKCFG_SCALE_MASK);
      regval |= (SPI_CLKCFG_LO(high) | SPI_CLKCFG_HI(high) | SPI_CLKCFG_SCALE(scale));
      spi_putreg(priv, MAX326_SPI_CLKCFG_OFFSET, regval);

      /* Save the frequency selection so that subsequent reconfigurations will be
       * faster.
       */

      spiinfo("Frequency %d->%d\n", frequency, actual);

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
  FAR struct max326_spidev_s *priv = (FAR struct max326_spidev_s *)dev;
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
          clrbits = SPI_CTRL2_CLKPOL | SPI_CTRL2_CLKPHA;
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          setbits = SPI_CTRL2_CLKPHA;
          clrbits = SPI_CTRL2_CLKPOL;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          setbits = SPI_CTRL2_CLKPOL;
          clrbits = SPI_CTRL2_CLKPHA;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          setbits = SPI_CTRL2_CLKPOL | SPI_CTRL2_CLKPHA;
          clrbits = 0;
          break;

        default:
          return;
        }

        spi_modigy_ctrl2(priv, setbits, clrbits);

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
  FAR struct max326_spidev_s *priv = (FAR struct max326_spidev_s *)dev;
  uint16_t setbits;
  uint16_t clrbits;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
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

      spi_modigy_ctrl0(priv, setbits, clrbits);

      /* Save the selection so the subsequence re-configurations will be faster */

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
static int spi_hwfeatures(FAR struct spi_dev_s *dev, spi_hwfeatures_t features)
{
#ifdef CONFIG_SPI_BITORDER
  FAR struct max326_spidev_s *priv = (FAR struct max326_spidev_s *)dev;
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

  spi_modigy_ctrl0(priv, setbits, clrbits);

  /* Other H/W features are not supported */

  return ((features & ~HWFEAT_LSBFIRST) == 0) ? OK : -ENOSYS;
#else
  return -ENOSYS;
#endif
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

static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  FAR struct max326_spidev_s *priv = (FAR struct max326_spidev_s *)dev;
  uint32_t regval;
  uint16_t ret;

  DEBUGASSERT(priv && priv->spibase);

  spi_writeword(priv, wd);
  ret = spi_readword(priv);

  /* Check and clear any error flags (Reading from the SR clears the error flags) */

  regval = spi_getreg(priv, MAX326_SPI_SR_OFFSET);

  spiinfo("Sent: %04x Return: %04x Status: %02x\n", wd, ret, regval);
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
 *   FIFOs and (2) the MAX326 F3 data packing.
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

#if !defined(CONFIG_MAX326_SPI_DMA)
static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
#else
static void spi_exchange_nodma(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                               FAR void *rxbuffer, size_t nwords)
#endif
{
  FAR struct max326_spidev_s *priv = (FAR struct max326_spidev_s *)dev;
  DEBUGASSERT(priv && priv->spibase);

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* 8- or 16-bit mode? */

  if (spi_16bitmode(priv))
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

          word = spi_send(dev, word);

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

          word = (uint8_t)spi_send(dev, (uint16_t)word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = word;
            }
        }
    }
}

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
 ************************************************************************************/

#ifdef CONFIG_MAX326_SPI_DMA
static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  FAR struct max326_spidev_s *priv = (FAR struct max326_spidev_s *)dev;
  static uint16_t rxdummy = 0xffff;
  static const uint16_t txdummy = 0xffff;

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);
  DEBUGASSERT(priv && priv->spibase);

  /* Setup DMAs */

  spi_dmarxsetup(priv, rxbuffer, &rxdummy, nwords);
  spi_dmatxsetup(priv, txbuffer, &txdummy, nwords);

  /* Start the DMAs */

  spi_dmarxstart(priv);
  spi_dmatxstart(priv);

  /* Then wait for each to complete */

  spi_dmarxwait(priv);
  spi_dmatxwait(priv);
}
#endif /* CONFIG_MAX326_SPI_DMA */

/****************************************************************************
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
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
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
 *   rxbuffer - A pointer to the buffer in which to recieve data
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
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
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
 *   Initialize the selected SPI bus in its default state (Master, 8-bit, mode 0,
 *   etc.)
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_bus_initialize(FAR struct max326_spidev_s *priv)
{
  uint16_t setbits;
  uint16_t clrbits;

  /* Configure CR1. Default configuration:
   *   Mode 0:                        CPHA=0 and CPOL=0
   *   Master:                        MSTR=1
   *   8-bit:                         DFF=0
   *   MSB transmitted first:         LSBFIRST=0
   *   Replace NSS with SSI & SSI=1:  SSI=1 SSM=1 (prevents MODF error)
   *   Two lines full duplex:         BIDIMODE=0 BIDIOIE=(Don't care) and RXONLY=0
   */

  clrbits = SPI_CTRL2_CLKPHA | SPI_CTRL2_CLKPOL | SPI_CR1_BR_MASK | SPI_CR1_LSBFIRST |
            SPI_CR1_RXONLY | SPI_CR1_DFF | SPI_CR1_BIDIOE | SPI_CR1_BIDIMODE;
  setbits = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM;
  spi_modigy_ctrl0(priv, setbits, clrbits);

  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* CRCPOLY configuration */

  spi_putreg(priv, MAX326_SPI_CRCPR_OFFSET, 7);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

  nxsem_init(&priv->exclsem, 0, 1);

#ifdef CONFIG_MAX326_SPI_DMA
  /* Initialize the SPI semaphores that is used to wait for DMA completion.
   * This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  if (priv->rxch && priv->txch)
    {
      nxsem_init(&priv->rxsem, 0, 0);
      nxsem_init(&priv->txsem, 0, 0);

      nxsem_setprotocol(&priv->rxsem, SEM_PRIO_NONE);
      nxsem_setprotocol(&priv->txsem, SEM_PRIO_NONE);

      /* Get DMA channels.  NOTE: max326_dmachannel() will always assign the DMA
       * channel.  If the channel is not available, then max326_dmachannel() will
       * block and wait until the channel becomes available.  WARNING: If you have
       * another device sharing a DMA channel with SPI and the code never releases
       * that channel, then the call to max326_dmachannel()  will hang forever in
       * this function!  Don't let your design do that!
       */

      priv->rxdma = max326_dmachannel(priv->rxch);
      priv->txdma = max326_dmachannel(priv->txch);
      DEBUGASSERT(priv->rxdma && priv->txdma);

      spi_modigy_ctrl1(priv, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN, 0);
    }
  else
    {
      priv->rxdma = NULL;
      priv->txdma = NULL;
    }
#endif

  /* Enable SPI */

  spi_modigy_ctrl0(priv, SPI_CR1_SPE, 0);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: max326_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ************************************************************************************/

FAR struct spi_dev_s *max326_spibus_initialize(int bus)
{
  FAR struct max326_spidev_s *priv = NULL;

  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_MAX326XX_SPIM0
  if (bus == 0)
    {
      /* Select SPI0 */

      priv = &g_spi0dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI0 pins: SCK, MISO, and MOSI */

          max326_gpio_config(GPIO_SPI0_SCK);
          max326_gpio_config(GPIO_SPI0_MISO);
          max326_gpio_config(GPIO_SPI0_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          spi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
#ifdef CONFIG_MAX326XX_SPIM1
  if (bus == 1)
    {
      /* Select SPI1 */

      priv = &g_spi1dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI1 pins: SCK, MISO, and MOSI */

          max326_gpio_config(GPIO_SPI1_SCK);
          max326_gpio_config(GPIO_SPI1_MISO);
          max326_gpio_config(GPIO_SPI1_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          spi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
    {
      spierr("ERROR: Unsupported SPI bus: %d\n", bus);
      return NULL;
    }

  leave_critical_section(flags);
  return (FAR struct spi_dev_s *)priv;
}

#endif /* CONFIG_MAX326XX_HAVE_SPIM */
