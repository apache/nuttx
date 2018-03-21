/************************************************************************************
 * arm/arm/src/stm32l4/stm32l4_spi.c
 *
 *   Copyright (C) 2009-2013, 2016 Gregory Nutt. All rights reserved.
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
 * The external functions, stm32l4_spi1/2/3select and stm32l4_spi1/2/3status must be
 * provided by board-specific logic.  They are implementations of the select
 * and status methods of the SPI interface defined by struct spi_ops_s (see
 * include/nuttx/spi/spi.h). All other methods (including stm32l4_spibus_initialize())
 * are provided by common STM32 logic.  To use this common SPI logic on your
 * board:
 *
 *   1. Provide logic in stm32l4_board_initialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32l4_spi1/2/3select() and stm32l4_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to stm32l4_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by stm32l4_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************c********************************/

/* This driver is ported from the stm32 one, which only supports 8 and 16 bits
 * transfers. The STM32L4 family supports frame size from 4 to 16 bits, but we do not
 * support that yet. For the moment, we replace uses of the CR1_DFF bit with a check
 * of the CR2_DS[0..3] bits. If the value is SPI_CR2_DS_16BIT it means 16 bits, else 8 bits.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/power/pm.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "stm32l4.h"
#include "stm32l4_gpio.h"
#include "stm32l4_dma.h"
#include "stm32l4_spi.h"

#include <arch/board/board.h>

#if defined(CONFIG_STM32L4_SPI1) || defined(CONFIG_STM32L4_SPI2) || \
    defined(CONFIG_STM32L4_SPI3)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/
/* SPI interrupts */

#ifdef CONFIG_STM32L4_SPI_INTERRUPTS
#  error "Interrupt driven SPI not yet supported"
#endif

/* Can't have both interrupt driven SPI and SPI DMA */

#if defined(CONFIG_STM32L4_SPI_INTERRUPTS) && defined(CONFIG_STM32L4_SPI_DMA)
#  error "Cannot enable both interrupt mode and DMA mode for SPI"
#endif

/* SPI DMA priority */

#ifdef CONFIG_STM32L4_SPI_DMA

#  if defined(CONFIG_SPI_DMAPRIO)
#    define SPI_DMA_PRIO  CONFIG_SPI_DMAPRIO
#  else
#    define SPI_DMA_PRIO  DMA_CCR_PRIMED
#  endif

#  if (SPI_DMA_PRIO & ~DMA_CCR_PL_MASK) != 0
#    error "Illegal value for CONFIG_SPI_DMAPRIO"
#  endif

#endif

/* DMA channel configuration */

#define SPI_RXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_CCR_MSIZE_16BITS|DMA_CCR_PSIZE_16BITS|DMA_CCR_MINC            )
#define SPI_RXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_8BITS |DMA_CCR_MINC            )
#define SPI_RXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_16BITS                         )
#define SPI_RXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_8BITS                          )
#define SPI_TXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_CCR_MSIZE_16BITS|DMA_CCR_PSIZE_16BITS|DMA_CCR_MINC|DMA_CCR_DIR)
#define SPI_TXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_8BITS |DMA_CCR_MINC|DMA_CCR_DIR)
#define SPI_TXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_16BITS             |DMA_CCR_DIR)
#define SPI_TXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_8BITS              |DMA_CCR_DIR)

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct stm32l4_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  uint32_t         spibase;    /* SPIn base address */
  uint32_t         spiclock;   /* Clocking for the SPI module */
#ifdef CONFIG_STM32L4_SPI_INTERRUPTS
  uint8_t          spiirq;     /* SPI IRQ number */
#endif
#ifdef CONFIG_STM32L4_SPI_DMA
  volatile uint8_t rxresult;   /* Result of the RX DMA */
  volatile uint8_t txresult;   /* Result of the RX DMA */
  uint16_t         rxch;       /* The RX DMA channel number */
  uint16_t         txch;       /* The TX DMA channel number */
  DMA_HANDLE       rxdma;      /* DMA channel handle for RX transfers */
  DMA_HANDLE       txdma;      /* DMA channel handle for TX transfers */
  sem_t            rxsem;      /* Wait for RX DMA to complete */
  sem_t            txsem;      /* Wait for TX DMA to complete */
  uint32_t         txccr;      /* DMA control register for TX transfers */
  uint32_t         rxccr;      /* DMA control register for RX transfers */
#endif
  sem_t            exclsem;    /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          nbits;      /* Width of word in bits (4 through 16) */
  uint8_t          mode;       /* Mode 0,1,2,3 */
#ifdef CONFIG_PM
  struct pm_callback_s pm_cb;  /* PM callbacks */
#endif
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Helpers */

static inline uint16_t spi_getreg(FAR struct stm32l4_spidev_s *priv, uint8_t offset);
static inline void spi_putreg(FAR struct stm32l4_spidev_s *priv, uint8_t offset,
                                 uint16_t value);
static inline uint16_t spi_readword(FAR struct stm32l4_spidev_s *priv);
static inline void spi_writeword(FAR struct stm32l4_spidev_s *priv, uint16_t byte);
static inline bool spi_16bitmode(FAR struct stm32l4_spidev_s *priv);

/* DMA support */

#ifdef CONFIG_STM32L4_SPI_DMA
static void        spi_dmarxwait(FAR struct stm32l4_spidev_s *priv);
static void        spi_dmatxwait(FAR struct stm32l4_spidev_s *priv);
static inline void spi_dmarxwakeup(FAR struct stm32l4_spidev_s *priv);
static inline void spi_dmatxwakeup(FAR struct stm32l4_spidev_s *priv);
static void        spi_dmarxcallback(DMA_HANDLE handle, uint8_t isr, void *arg);
static void        spi_dmatxcallback(DMA_HANDLE handle, uint8_t isr, void *arg);
static void        spi_dmarxsetup(FAR struct stm32l4_spidev_s *priv,
                                  FAR void *rxbuffer, FAR void *rxdummy, size_t nwords);
static void        spi_dmatxsetup(FAR struct stm32l4_spidev_s *priv,
                                  FAR const void *txbuffer, FAR const void *txdummy, size_t nwords);
static inline void spi_dmarxstart(FAR struct stm32l4_spidev_s *priv);
static inline void spi_dmatxstart(FAR struct stm32l4_spidev_s *priv);
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

static void        spi_bus_initialize(FAR struct stm32l4_spidev_s *priv);

/* PM interface */

#ifdef CONFIG_PM
static int         spi_pm_prepare(FAR struct pm_callback_s *cb, int domain,
                                  enum pm_state_e pmstate);
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_STM32L4_SPI1
static const struct spi_ops_s g_spi1ops =
{
  .lock              = spi_lock,
  .select            = stm32l4_spi1select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = stm32l4_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = stm32l4_spi1cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = stm32l4_spi1register,  /* Provided externally */
#else
  .registercallback  = 0,                   /* Not implemented */
#endif
};

static struct stm32l4_spidev_s g_spi1dev =
{
  .spidev   = { &g_spi1ops },
  .spibase  = STM32L4_SPI1_BASE,
  .spiclock = STM32L4_PCLK2_FREQUENCY,
#ifdef CONFIG_STM32L4_SPI_INTERRUPTS
  .spiirq   = STM32L4_IRQ_SPI1,
#endif
#ifdef CONFIG_STM32L4_SPI_DMA
  /* lines must be configured in board.h */
  .rxch     = DMACHAN_SPI1_RX,
  .txch     = DMACHAN_SPI1_TX,
#endif
#ifdef CONFIG_PM
  .pm_cb.prepare = spi_pm_prepare,
#endif
};
#endif

#ifdef CONFIG_STM32L4_SPI2
static const struct spi_ops_s g_spi2ops =
{
  .lock              = spi_lock,
  .select            = stm32l4_spi2select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = stm32l4_spi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = stm32l4_spi2cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = stm32l4_spi2register,  /* provided externally */
#else
  .registercallback  = 0,  /* not implemented */
#endif
};

static struct stm32l4_spidev_s g_spi2dev =
{
  .spidev   = { &g_spi2ops },
  .spibase  = STM32L4_SPI2_BASE,
  .spiclock = STM32L4_PCLK1_FREQUENCY,
#ifdef CONFIG_STM32L4_SPI_INTERRUPTS
  .spiirq   = STM32L4_IRQ_SPI2,
#endif
#ifdef CONFIG_STM32L4_SPI_DMA
  .rxch     = DMACHAN_SPI2_RX,
  .txch     = DMACHAN_SPI2_TX,
#endif
#ifdef CONFIG_PM
  .pm_cb.prepare = spi_pm_prepare,
#endif
};
#endif

#ifdef CONFIG_STM32L4_SPI3
static const struct spi_ops_s g_spi3ops =
{
  .lock              = spi_lock,
  .select            = stm32l4_spi3select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = stm32l4_spi3status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = stm32l4_spi3cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = stm32l4_spi3register,  /* provided externally */
#else
  .registercallback  = 0,  /* not implemented */
#endif
};

static struct stm32l4_spidev_s g_spi3dev =
{
  .spidev   = { &g_spi3ops },
  .spibase  = STM32L4_SPI3_BASE,
  .spiclock = STM32L4_PCLK1_FREQUENCY,
#ifdef CONFIG_STM32L4_SPI_INTERRUPTS
  .spiirq   = STM32L4_IRQ_SPI3,
#endif
#ifdef CONFIG_STM32L4_SPI_DMA
  .rxch     = DMACHAN_SPI3_RX,
  .txch     = DMACHAN_SPI3_TX,
#endif
#ifdef CONFIG_PM
  .pm_cb.prepare = spi_pm_prepare,
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

static inline uint16_t spi_getreg(FAR struct stm32l4_spidev_s *priv, uint8_t offset)
{
  return getreg16(priv->spibase + offset);
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

static inline void spi_putreg(FAR struct stm32l4_spidev_s *priv, uint8_t offset,
                              uint16_t value)
{
  putreg16(value, priv->spibase + offset);
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
 *   The contents of the 8-bit register
 *
 ************************************************************************************/

static inline uint8_t spi_getreg8(FAR struct stm32l4_spidev_s *priv, uint8_t offset)
{
  return getreg8(priv->spibase + offset);
}

/************************************************************************************
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
 ************************************************************************************/

static inline void spi_putreg8(FAR struct stm32l4_spidev_s *priv, uint8_t offset,
                               uint8_t value)
{
  putreg8(value, priv->spibase + offset);
}

/************************************************************************************
 * Name: spi_readword
 *
 * Description:
 *   Read one word (TWO bytes!) from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   Word as read
 *
 ************************************************************************************/

static inline uint16_t spi_readword(FAR struct stm32l4_spidev_s *priv)
{
  /* Wait until the receive buffer is not empty */

  while ((spi_getreg(priv, STM32L4_SPI_SR_OFFSET) & SPI_SR_RXNE) == 0);

  /* Then return the received byte */

  return spi_getreg(priv, STM32L4_SPI_DR_OFFSET);
}

/************************************************************************************
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
 ************************************************************************************/

static inline uint8_t spi_readbyte(FAR struct stm32l4_spidev_s *priv)
{
  /* Wait until the receive buffer is not empty */

  while ((spi_getreg(priv, STM32L4_SPI_SR_OFFSET) & SPI_SR_RXNE) == 0);

  /* Then return the received byte */

  return spi_getreg8(priv, STM32L4_SPI_DR_OFFSET);
}

/************************************************************************************
 * Name: spi_writeword
 *
 * Description:
 *   Write one 16-bit frame to the SPI FIFO
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   byte - Word to send
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void spi_writeword(FAR struct stm32l4_spidev_s *priv, uint16_t word)
{
  /* Wait until the transmit buffer is empty */

  while ((spi_getreg(priv, STM32L4_SPI_SR_OFFSET) & SPI_SR_TXE) == 0);

  /* Then send the byte */

  spi_putreg(priv, STM32L4_SPI_DR_OFFSET, word);
}

/************************************************************************************
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
 ************************************************************************************/

static inline void spi_writebyte(FAR struct stm32l4_spidev_s *priv, uint8_t byte)
{
  /* Wait until the transmit buffer is empty */

  while ((spi_getreg(priv, STM32L4_SPI_SR_OFFSET) & SPI_SR_TXE) == 0);

  /* Then send the byte */

  spi_putreg8(priv, STM32L4_SPI_DR_OFFSET, byte);
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

static inline bool spi_16bitmode(FAR struct stm32l4_spidev_s *priv)
{
  return (priv->nbits > 8);
}

/************************************************************************************
 * Name: spi_dmarxwaitw
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32L4_SPI_DMA
static void spi_dmarxwait(FAR struct stm32l4_spidev_s *priv)
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

#ifdef CONFIG_STM32L4_SPI_DMA
static void spi_dmatxwait(FAR struct stm32l4_spidev_s *priv)
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

#ifdef CONFIG_STM32L4_SPI_DMA
static inline void spi_dmarxwakeup(FAR struct stm32l4_spidev_s *priv)
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

#ifdef CONFIG_STM32L4_SPI_DMA
static inline void spi_dmatxwakeup(FAR struct stm32l4_spidev_s *priv)
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

#ifdef CONFIG_STM32L4_SPI_DMA
static void spi_dmarxcallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
  FAR struct stm32l4_spidev_s *priv = (FAR struct stm32l4_spidev_s *)arg;

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

#ifdef CONFIG_STM32L4_SPI_DMA
static void spi_dmatxcallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
  FAR struct stm32l4_spidev_s *priv = (FAR struct stm32l4_spidev_s *)arg;

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

#ifdef CONFIG_STM32L4_SPI_DMA
static void spi_dmarxsetup(FAR struct stm32l4_spidev_s *priv, FAR void *rxbuffer,
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

  stm32l4_dmasetup(priv->rxdma, priv->spibase + STM32L4_SPI_DR_OFFSET,
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

#ifdef CONFIG_STM32L4_SPI_DMA
static void spi_dmatxsetup(FAR struct stm32l4_spidev_s *priv, FAR const void *txbuffer,
                           FAR const void *txdummy, size_t nwords)
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

  stm32l4_dmasetup(priv->txdma, priv->spibase + STM32L4_SPI_DR_OFFSET,
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

#ifdef CONFIG_STM32L4_SPI_DMA
static inline void spi_dmarxstart(FAR struct stm32l4_spidev_s *priv)
{
  priv->rxresult = 0;
  stm32l4_dmastart(priv->rxdma, spi_dmarxcallback, priv, false);
}
#endif

/************************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start TX DMA
 *
 ************************************************************************************/

#ifdef CONFIG_STM32L4_SPI_DMA
static inline void spi_dmatxstart(FAR struct stm32l4_spidev_s *priv)
{
  priv->txresult = 0;
  stm32l4_dmastart(priv->txdma, spi_dmatxcallback, priv, false);
}
#endif

/************************************************************************************
 * Name: spi_modifycr
 *
 * Description:
 *   Clear and set bits in the CR1 or CR2 register
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

static void spi_modifycr(uint32_t addr, FAR struct stm32l4_spidev_s *priv,
                         uint16_t setbits, uint16_t clrbits)
{
  uint16_t cr;

  cr  = spi_getreg(priv, addr);
  cr &= ~clrbits;
  cr |= setbits;
  spi_putreg(priv, addr, cr);
}

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
  FAR struct stm32l4_spidev_s *priv = (FAR struct stm32l4_spidev_s *)dev;
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
  FAR struct stm32l4_spidev_s *priv = (FAR struct stm32l4_spidev_s *)dev;
  uint16_t setbits;
  uint32_t actual;

  /* Limit to max possible (if STM32L4_SPI_CLK_MAX is defined in board.h) */

  if (frequency > STM32L4_SPI_CLK_MAX)
    {
      frequency = STM32L4_SPI_CLK_MAX;
    }

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

      spi_modifycr(STM32L4_SPI_CR1_OFFSET, priv, 0, SPI_CR1_SPE);
      spi_modifycr(STM32L4_SPI_CR1_OFFSET, priv, setbits, SPI_CR1_BR_MASK);
      spi_modifycr(STM32L4_SPI_CR1_OFFSET, priv, SPI_CR1_SPE, 0);

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
  FAR struct stm32l4_spidev_s *priv = (FAR struct stm32l4_spidev_s *)dev;
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

        default:
          return;
        }

        spi_modifycr(STM32L4_SPI_CR1_OFFSET, priv, 0, SPI_CR1_SPE);
        spi_modifycr(STM32L4_SPI_CR1_OFFSET, priv, setbits, clrbits);
        spi_modifycr(STM32L4_SPI_CR1_OFFSET, priv, SPI_CR1_SPE, 0);

        /* Save the mode so that subsequent re-configurations will be faster */

        priv->mode = mode;
    }
}

/************************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word. With STM32L4, this is not restricted to 8 or 16,
 *   but can be any value between 4 and 16.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested, negative value means LSB first.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct stm32l4_spidev_s *priv = (FAR struct stm32l4_spidev_s *)dev;
  uint16_t setbits;
  uint16_t clrbits;
  int savbits = nbits;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      /* Yes... Set CR2 appropriately */
      /* Set the number of bits (valid range 4-16) */

      if (nbits < 4 || nbits > 16)
        {
          spierr("ERROR: nbits out of range: %d\n", nbits);
          return;
        }

      clrbits = SPI_CR2_DS_MASK;
      setbits = SPI_CR2_DS_VAL(nbits);

      /* If nbits is <=8, then we are in byte mode and FRXTH shall be set
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

      spi_modifycr(STM32L4_SPI_CR1_OFFSET, priv, 0, SPI_CR1_SPE);
      spi_modifycr(STM32L4_SPI_CR2_OFFSET, priv, setbits, clrbits);
      spi_modifycr(STM32L4_SPI_CR1_OFFSET, priv, SPI_CR1_SPE, 0);

      /* Save the selection so the subsequence re-configurations will be faster */

      priv->nbits = savbits; // nbits has been clobbered... save the signed value.
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
  FAR struct stm32l4_spidev_s *priv = (FAR struct stm32l4_spidev_s *)dev;
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

  spi_modifycr(STM32L4_SPI_CR1_OFFSET, priv, 0, SPI_CR1_SPE);
  spi_modifycr(STM32L4_SPI_CR1_OFFSET, priv, setbits, clrbits);
  spi_modifycr(STM32L4_SPI_CR1_OFFSET, priv, SPI_CR1_SPE, 0);

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
  FAR struct stm32l4_spidev_s *priv = (FAR struct stm32l4_spidev_s *)dev;
  uint32_t regval;
  uint16_t ret;

  DEBUGASSERT(priv && priv->spibase);

  /* According to the number of bits, access data register as word or byte
   * This is absolutely required because of packing. With <=8 bit frames,
   * two bytes are received by a 16-bit read of the data register!
   */

  if (spi_16bitmode(priv))
    {
      spi_writeword(priv, wd);
      ret = spi_readword(priv);
    }
  else
    {
      spi_writebyte(priv, (uint8_t)(wd & 0xFF));
      ret = (uint16_t)spi_readbyte(priv);
    }

  /* Check and clear any error flags (Reading from the SR clears the error
   * flags).
   */

  regval = spi_getreg(priv, STM32L4_SPI_SR_OFFSET);

  if (spi_16bitmode(priv))
    {
      spiinfo("Sent: %04x Return: %04x Status: %02x\n", wd, ret, regval);
    }
  else
    {
      spiinfo("Sent: %02x Return: %02x Status: %02x\n", wd, ret, regval);
    }

  UNUSED(regval);
  return ret;
}

/************************************************************************************
 * Name: spi_exchange (no DMA).  aka spi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchaned in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#if !defined(CONFIG_STM32L4_SPI_DMA) || defined(CONFIG_STM32L4_DMACAPABLE)
#if !defined(CONFIG_STM32L4_SPI_DMA)
static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
#else
static void spi_exchange_nodma(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                               FAR void *rxbuffer, size_t nwords)
#endif
{
  FAR struct stm32l4_spidev_s *priv = (FAR struct stm32l4_spidev_s *)dev;
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
#endif /* !CONFIG_STM32L4_SPI_DMA || CONFIG_STM32L4_DMACAPABLE */

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
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_STM32L4_SPI_DMA
static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  FAR struct stm32l4_spidev_s *priv = (FAR struct stm32l4_spidev_s *)dev;

#ifdef CONFIG_STM32L4_DMACAPABLE
  if ((txbuffer && !stm32l4_dmacapable((uint32_t)txbuffer, nwords, priv->txccr)) ||
      (rxbuffer && !stm32l4_dmacapable((uint32_t)rxbuffer, nwords, priv->rxccr)))
    {
      /* Unsupported memory region, fall back to non-DMA method. */

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

      spi_dmarxsetup(priv, rxbuffer, &rxdummy, nwords);
      spi_dmatxsetup(priv, txbuffer, &txdummy, nwords);

      /* Start the DMAs */

      spi_dmarxstart(priv);
      spi_dmatxstart(priv);

      /* Then wait for each to complete */

      spi_dmarxwait(priv);
      spi_dmatxwait(priv);
    }
}
#endif /* CONFIG_STM32L4_SPI_DMA */

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
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer, size_t nwords)
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
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer, size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/************************************************************************************
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
 ************************************************************************************/

#ifdef CONFIG_PM
static int spi_pm_prepare(FAR struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate)
{
  struct stm32l4_spidev_s *priv =
      (struct stm32l4_spidev_s *)((char *)cb -
                                    offsetof(struct stm32l4_spidev_s, pm_cb));
  int sval;

  /* Logic to prepare for a reduced power state goes here. */

  switch (pmstate)
    {
    case PM_NORMAL:
    case PM_IDLE:
      break;

    case PM_STANDBY:
    case PM_SLEEP:
      /* Check if exclusive lock for SPI bus is held. */

      if (nxsem_getvalue(&priv->exclsem, &sval) < 0)
        {
          DEBUGASSERT(false);
          return -EINVAL;
        }

      if (sval <= 0)
        {
          /* Exclusive lock is held, do not allow entry to deeper PM states. */

          return -EBUSY;
        }

      break;
    }

  return OK;
}
#endif

/************************************************************************************
 * Name: spi_bus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus in its default state (Master, 8-bit, mode 0, etc.)
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_bus_initialize(FAR struct stm32l4_spidev_s *priv)
{
  uint16_t setbits;
  uint16_t clrbits;
#ifdef CONFIG_PM
  int ret;
#endif

  /* Configure CR1 and CR2. Default configuration:
   *   Mode 0:                        CR1.CPHA=0 and CR1.CPOL=0
   *   Master:                        CR1.MSTR=1
   *   8-bit:                         CR2.DS=7
   *   MSB tranmitted first:          CR1.LSBFIRST=0
   *   Replace NSS with SSI & SSI=1:  CR1.SSI=1 CR1.SSM=1 (prevents MODF error)
   *   Two lines full duplex:         CR1.BIDIMODE=0 CR1.BIDIOIE=(Don't care) and CR1.RXONLY=0
   */

  clrbits = SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_BR_MASK | SPI_CR1_LSBFIRST |
            SPI_CR1_RXONLY | SPI_CR1_BIDIOE | SPI_CR1_BIDIMODE;
  setbits = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM;
  spi_modifycr(STM32L4_SPI_CR1_OFFSET, priv, setbits, clrbits);

  clrbits = SPI_CR2_DS_MASK;
  setbits = SPI_CR2_DS_8BIT | SPI_CR2_FRXTH; /* FRXTH must be high in 8-bit mode */
  spi_modifycr(STM32L4_SPI_CR2_OFFSET, priv, setbits, clrbits);

  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* CRCPOLY configuration */

  spi_putreg(priv, STM32L4_SPI_CRCPR_OFFSET, 7);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

  nxsem_init(&priv->exclsem, 0, 1);

#ifdef CONFIG_STM32L4_SPI_DMA
  /* Initialize the SPI semaphores that is used to wait for DMA completion */

  nxsem_init(&priv->rxsem, 0, 0);
  nxsem_init(&priv->txsem, 0, 0);

  /* These semaphores are used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_setprotocol(&priv->rxsem, SEM_PRIO_NONE);
  nxsem_setprotocol(&priv->txsem, SEM_PRIO_NONE);

  /* Get DMA channels.  NOTE: stm32l4_dmachannel() will always assign the DMA channel.
   * if the channel is not available, then stm32l4_dmachannel() will block and wait
   * until the channel becomes available.  WARNING: If you have another device sharing
   * a DMA channel with SPI and the code never releases that channel, then the call
   * to stm32l4_dmachannel()  will hang forever in this function!  Don't let your
   * design do that!
   */

  priv->rxdma = stm32l4_dmachannel(priv->rxch);
  priv->txdma = stm32l4_dmachannel(priv->txch);
  DEBUGASSERT(priv->rxdma && priv->txdma);

  spi_modifycr(STM32L4_SPI_CR2_OFFSET, priv, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN, 0);
#endif

  /* Enable spi */

  spi_modifycr(STM32L4_SPI_CR1_OFFSET, priv, SPI_CR1_SPE, 0);

#ifdef CONFIG_PM
  /* Register to receive power management callbacks */

  ret = pm_register(&priv->pm_cb);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
#endif
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32l4_spibus_initialize
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

FAR struct spi_dev_s *stm32l4_spibus_initialize(int bus)
{
  FAR struct stm32l4_spidev_s *priv = NULL;

  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_STM32L4_SPI1
  if (bus == 1)
    {
      /* Select SPI1 */

      priv = &g_spi1dev;

      /* Only configure if the bus is not already configured */

      if ((spi_getreg(priv, STM32L4_SPI_CR1_OFFSET) & SPI_CR1_SPE) == 0)
        {
          /* Configure SPI1 pins: SCK, MISO, and MOSI */

          stm32l4_configgpio(GPIO_SPI1_SCK);
          stm32l4_configgpio(GPIO_SPI1_MISO);
          stm32l4_configgpio(GPIO_SPI1_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          spi_bus_initialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_STM32L4_SPI2
  if (bus == 2)
    {
      /* Select SPI2 */

      priv = &g_spi2dev;

      /* Only configure if the bus is not already configured */

      if ((spi_getreg(priv, STM32L4_SPI_CR1_OFFSET) & SPI_CR1_SPE) == 0)
        {
          /* Configure SPI2 pins: SCK, MISO, and MOSI */

          stm32l4_configgpio(GPIO_SPI2_SCK);
          stm32l4_configgpio(GPIO_SPI2_MISO);
          stm32l4_configgpio(GPIO_SPI2_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          spi_bus_initialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_STM32L4_SPI3
  if (bus == 3)
    {
      /* Select SPI3 */

      priv = &g_spi3dev;

      /* Only configure if the bus is not already configured */

      if ((spi_getreg(priv, STM32L4_SPI_CR1_OFFSET) & SPI_CR1_SPE) == 0)
        {
          /* Configure SPI3 pins: SCK, MISO, and MOSI */

          stm32l4_configgpio(GPIO_SPI3_SCK);
          stm32l4_configgpio(GPIO_SPI3_MISO);
          stm32l4_configgpio(GPIO_SPI3_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          spi_bus_initialize(priv);
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

#endif /* CONFIG_STM32L4_SPI1 || CONFIG_STM32L4_SPI2 || CONFIG_STM32L4_SPI3 */

