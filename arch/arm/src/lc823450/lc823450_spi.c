/****************************************************************************
 * arch/arm/src/lc823450/lc823450_spi.c
 *
 *   Copyright 2014,2015,2016,2017 Sony Video & Sound Products Inc.
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include "up_arch.h"
#include "chip.h"

#include "lc823450_syscontrol.h"
#include "lc823450_clockconfig.h"
#include "lc823450_dma.h"
#include "lc823450_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SPI_EXCHANGE
# error "SPI_EXCHANGE is not supported"
#endif

#ifndef MIN
#  define MIN(a, b) ((a) > (b) ? b : a)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lc823450_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
#ifndef CONFIG_SPI_OWNBUS
  sem_t            exclsem;    /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          nbits;      /* Width of word in bits (8 to 16) */
  uint8_t          mode;       /* Mode 0,1,2,3 */
#endif
#ifdef CONFIG_LC823450_SPI_DMA
  DMA_HANDLE       hdma;
  sem_t dma_wait;
#endif /* CONFIG_LC823450_SPI_DMA */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI methods */

#ifndef CONFIG_SPI_OWNBUS
static int      spi_lock(FAR struct spi_dev_s *dev, bool lock);
#endif
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void     spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void     spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t ch);
#ifndef CONFIG_SPI_EXCHANGE
static void     spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords);
static void     spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spiops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .select            = lc823450_spiselect,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lc823450_spicmddata,
#endif
#if 0
  .status            = lc823450_spistatus,
#endif
  .send              = spi_send,
#ifndef CONFIG_SPI_EXCHANGE
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
};

static struct lc823450_spidev_s g_spidev =
{
  .spidev            = { &g_spiops },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct lc823450_spidev_s *priv = (FAR struct lc823450_spidev_s *)dev;
  int ret;

  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      do
        {
         ret = nxsem_wait(&priv->exclsem);

          /* The only case that an error should occur here is if the wait was
           * awakened by a signal.
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
#endif

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

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
  FAR struct lc823450_spidev_s *priv = (FAR struct lc823450_spidev_s *)dev;
  unsigned long  sysclk = lc823450_get_systemfreq();
  uint32_t actual;
  int div;

#ifndef CONFIG_SPI_OWNBUS
  if (priv->frequency == frequency)
    {
      return priv->actual;
    }
#endif

  for (div = 0xFE; div >= 0; div--)
    {
      if (frequency >= sysclk / (4 * (256 - div)))
        {
          break;
        }
    }

  spiinfo("Frequency %d -> %d\n", frequency, sysclk / (4 * (256 - div)));

  actual = sysclk / (4 * (256 - div));
  putreg32(div, LC823450_SPI_BRG);

#ifndef CONFIG_SPI_OWNBUS
  priv->frequency = frequency;
  priv->actual    = actual;
#endif

  return actual;
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode. Optional.  See enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct lc823450_spidev_s *priv = (FAR struct lc823450_spidev_s *)dev;

#ifndef CONFIG_SPI_OWNBUS
  if (priv->mode == mode)
    {
      return;
    }
#endif

  switch (mode)
    {
      case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
        DEBUGASSERT(0);
        break;

      case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
        modifyreg32(LC823450_SPI_SMD, 0, SPI_SMD_PO);
        break;

      case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
        DEBUGASSERT(0);
        break;

      case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
        modifyreg32(LC823450_SPI_SMD, SPI_SMD_PO, 0);
        break;

      default:
        DEBUGASSERT(FALSE);
        return;
    }

#ifndef CONFIG_SPI_OWNBUS
  priv->mode = mode;
#endif
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct lc823450_spidev_s *priv = (FAR struct lc823450_spidev_s *)dev;

#ifndef CONFIG_SPI_OWNBUS
  if (priv->nbits == nbits)
    {
      return;
    }
#endif

  DEBUGASSERT(nbits == 8 || nbits == 16);

  modifyreg32(LC823450_SPI_SMD, SPI_SMD_CHL, nbits == 8 ? 0 : SPI_SMD_CHL);

#ifndef CONFIG_SPI_OWNBUS
  priv->nbits = nbits;
#endif
}

/****************************************************************************
 * Name: spi_dma_callback
 *
 ****************************************************************************/

#ifdef CONFIG_LC823450_SPI_DMA
static void spi_dma_callback(DMA_HANDLE hdma, void *arg, int result)
{
  sem_t *waitsem = (sem_t *)arg;
  nxsem_post(waitsem);
}
#endif /* CONFIG_LC823450_SPI_DMA */

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

#ifdef CONFIG_LC823450_SPI_DMA
static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  spi_sndblock(dev, &wd, 1);
  return 0;
}
#else /* CONFIG_LC823450_SPI_DMA */
static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  putreg16(wd, LC823450_SPI_STR);

  modifyreg32(LC823450_SPI_ISR, 0, SPI_ISR_SPIF);

  modifyreg32(LC823450_SPI_SMD, 0, SPI_SMD_SSTR);

  /* Wait for Tranfer done */

  while ((getreg32(LC823450_SPI_ISR) & SPI_ISR_SPIF) == 0)
    ;

  return getreg16(LC823450_SPI_SRR);
}
#endif

/*************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nwords - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits > 8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer,
                         size_t nwords)
{
  FAR struct lc823450_spidev_s *priv = (FAR struct lc823450_spidev_s *)dev;

#ifdef CONFIG_LC823450_SPI_DMA
  /* TODO: 16bit */

  while (nwords)
    {
      int len;

      len = MIN(nwords, LC823450_DMA_MAX_TRANSSIZE);
      lc823450_dmasetup(priv->hdma,
                        LC823450_DMA_SRCWIDTH_BYTE |
                        LC823450_DMA_DSTWIDTH_BYTE |
                        LC823450_DMA_SRCINC,
                        (uint32_t)buffer, /* LC823450_SPI_STR */
                        LC823450_SPI_TxFF, len);

      lc823450_dmastart(priv->hdma, spi_dma_callback, &priv->dma_wait);

      modifyreg32(LC823450_SPI_SMD, 0, SPI_SMD_WTR);

      while (nxsem_wait(&priv->dma_wait) < 0);
      nwords -= len;
      buffer += len;
    }

  /* Wait for FIFO empty */

  putreg32(SPI_ISR_TxEMP, LC823450_SPI_ISR);
  while ((getreg32(LC823450_SPI_ISR) & SPI_ISR_TxEMP) != 0)
    ;

  /* Wait for Tx reg empty */

  while ((getreg32(LC823450_SPI_SSR) & SPI_SSR_TFF) != 0)
    ;

  /* Wait for Shift reg empty */

  while ((getreg32(LC823450_SPI_SSR) & SPI_SSR_SHRF) != 0)
    ;

#else /* CONFIG_LC823450_SPI_DMA */
  int i;
  const uint8_t *buf = buffer;
  const uint16_t *buf16 = buffer;

  if (priv->nbits == 16)
    {
      for (i = 0; i < nwords; i++)
        {
          (void)spi_send(dev, *buf16++);
        }
    }
  else
    {
      for (i = 0; i < nwords; i++)
        {
          (void)spi_send(dev, *buf++);
        }
    }
#endif /* CONFIG_LC823450_SPI_DMA */
}
#endif

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to recieve data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords)
{
  FAR struct lc823450_spidev_s *priv = (FAR struct lc823450_spidev_s *)dev;
  int i;
  uint8_t *buf = buffer;
  uint16_t *buf16 = buffer;

  if (priv->nbits == 16)
    {
      for (i = 0; i < nwords; i++)
        {
          *buf16++ = spi_send(dev, 0xffff);
        }
    }
  else
    {
      for (i = 0; i < nwords; i++)
        {
          *buf++ = spi_send(dev,  0xffff);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port.
 *
 * Input Parameters:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *lc823450_spibus_initialize(int port)
{
  FAR struct lc823450_spidev_s *priv = &g_spidev;
  irqstate_t flags;

  DEBUGASSERT(port == 0);

  flags = enter_critical_section();

  if ((getreg32(MCLKCNTAPB) & MCLKCNTAPB_SPI_CLKEN) == 0)
    {
      /* SPI: clock / reset */

      modifyreg32(MCLKCNTAPB, 0, MCLKCNTAPB_SPI_CLKEN);
      modifyreg32(MRSTCNTAPB, 0, MRSTCNTAPB_SPI_RSTB);

      /* PORT5: clock / reset */

      modifyreg32(MCLKCNTAPB, 0, MCLKCNTAPB_PORT5_CLKEN);
      modifyreg32(MRSTCNTAPB, 0, MRSTCNTAPB_PORT5_RSTB);

#ifndef CONFIG_SPI_OWNBUS
      nxsem_init(&priv->exclsem, 0, 1);
#endif

      /* Initialize SPI mode. It must be done before starting SPI transfer */

      /* PO: SPI Mode3 (default) */

      spi_setmode(&priv->spidev, SPIDEV_MODE3);

      /* LM: MSB first, BGE: Internal clock */

      modifyreg32(LC823450_SPI_SMD, 0, SPI_SMD_LM | SPI_SMD_BGE);

      priv->frequency = 0;
      priv->actual = 0;

      lc823450_spiinitialize();

#ifdef CONFIG_LC823450_SPI_DMA
      nxsem_init(&priv->dma_wait, 0, 0);
      priv->hdma = lc823450_dmachannel(DMA_CHANNEL_SIOTX);
      lc823450_dmarequest(priv->hdma, DMA_REQUEST_SIOTX);

      /* TX request enable */

      putreg32(SPI_DREQ_DREQ_TX, LC823450_SPI_DREQ);

      /* use FIFO */

      putreg32(SPI_TxFF_EN | SPI_TxFF_WL8, LC823450_SPI_FFCTL);
#endif /* CONFIG_LC823450_SPI_DMA */
    }

  leave_critical_section(flags);

  return &priv->spidev;
}
