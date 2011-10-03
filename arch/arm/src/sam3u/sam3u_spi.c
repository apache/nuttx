/****************************************************************************
 * arch/arm/src/sam3u/sam3u_spi.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
#include <nuttx/spi.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "sam3u_internal.h"
#include "sam3u_pmc.h"
#include "sam3u_spi.h"

#ifdef CONFIG_SAM3U_SPI

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

#undef SPI_DEBUG     /* Define to enable debug */
#undef SPI_VERBOSE   /* Define to enable verbose debug */

#ifdef SPI_DEBUG
#  define spidbg  lldbg
#  ifdef SPI_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  undef SPI_VERBOSE
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/* SPI Clocking */

#warning "Missing logi"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam3u_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
#ifndef CONFIG_SPI_OWNBUS
  sem_t            exclsem;    /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          nbits;      /* Width of word in bits (8 to 16) */
  uint8_t          mode;       /* Mode 0,1,2,3 */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI methods */

#ifndef CONFIG_SPI_OWNBUS
static int      spi_lock(FAR struct spi_dev_s *dev, bool lock);
#endif
static void     spi_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void     spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void     spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t ch);
static void     spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords);
static void     spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spiops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = sam3u_spiselect,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = sam3u_spistatus,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = sam3u_spicmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
  .registercallback  = 0,                 /* Not implemented */
};

static struct sam3u_spidev_s g_spidev =
{
  .spidev            = { &g_spiops },
}; 

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
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
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct sam3u_spidev_s *priv = (FAR struct sam3u_spidev_s *)dev;

  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      while (sem_wait(&priv->exclsem) != 0)
        {
          /* The only case that an error should occur here is if the wait was awakened
           * by a signal.
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

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
  FAR struct sam3u_spidev_s *priv = (FAR struct sam3u_spidev_s *)dev;
  uint32_t actual;

  /* Check if the requested frequence is the same as the frequency selection */

#ifndef CONFIG_SPI_OWNBUS
  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }
#endif

  /* Configure SPI to a frequency as close as possible to the requested
   * frequency.
   */
 
#warning "Missing logic"

  /* Calculate the actual actual frequency that is used */

#warning "Missing logic"
  actual = 0;

  /* Save the frequency setting */

#ifndef CONFIG_SPI_OWNBUS
  priv->frequency = frequency;
  priv->actual    = actual;
#endif

  spidbg("Frequency %d->%d\n", frequency, actual);
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
  FAR struct sam3u_spidev_s *priv = (FAR struct sam3u_spidev_s *)dev;
  uint32_t regval;

  /* Has the mode changed? */

#ifndef CONFIG_SPI_OWNBUS
  if (mode != priv->mode)
    {
#endif
      /* Yes... Set the mode appropriately */
#warning "Missing logic"

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          break;
 
        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          break;
 
        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          break;
 
        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          break;
 
        default:
          DEBUGASSERT(FALSE);
          return;
        }

      /* Save the mode so that subsequent re-configurations will be faster */

#ifndef CONFIG_SPI_OWNBUS
      priv->mode = mode;
    }
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
  FAR struct sam3u_spidev_s *priv = (FAR struct sam3u_spidev_s *)dev;
  uint32_t regval;

  /* Has the number of bits changed? */

  DEBUGASSERT(priv && nbits > 7 && nbits < 17);
#ifndef CONFIG_SPI_OWNBUS
  if (nbits != priv->nbits)
    {
#endif
      /* Yes... Set number of bits appropriately */
#warning "Missing logic"

      /* Save the selection so the subsequence re-configurations will be faster */

#ifndef CONFIG_SPI_OWNBUS
      priv->nbits = nbits;
    }
#endif
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

static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  /* Write the data to transmitted to the SPI Data Register */
#warning "Missing logic"

  /* Wait for the data exchange to complete */
#warning "Missing logic"

 /* Clear any pending status and return the received data */
#warning "Missing logic"
  return 0;
}

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
 *            packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords)
{
  FAR uint8_t *ptr = (FAR uint8_t*)buffer;
  uint8_t data;

  spidbg("nwords: %d\n", nwords);
  while (nwords)
    {
      /* Write the data to transmitted to the SPI Data Register */

      data = *ptr++;
#warning "Missing logic"

      /* Wait for the data exchange to complete */
#warning "Missing logic"

     /* Clear any pending status */
#warning "Missing logic"
     nwords--;
    }
}

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

static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords)
{
  FAR uint8_t *ptr = (FAR uint8_t*)buffer;

  spidbg("nwords: %d\n", nwords);
  while (nwords)
    {
      /* Write some dummy data to the SPI Data Register in order to clock the
       * read data.
       */

#warning "Missing logic"

      /* Wait for the data exchange to complete */
#warning "Missing logic"

     /* Read the received data from the SPI Data Register */   
#warning "Missing logic"

     /* Clear any pending status */
#warning "Missing logic"

     /* Read the received data from the SPI Data Register */   

#warning "Missing logic"
     *ptr++ = 0;
     nwords--;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *up_spiinitialize(int port)
{
  FAR struct sam3u_spidev_s *priv = &g_spidev;
  irqstate_t flags;
  uint32_t regval;

  /* The SAM3U has only a single SPI port */

  DEBUGASSERT(port == 0);

  /* Apply power to the SPI block */

  flags = irqsave();
  regval = getreg32(SAM3U_PMC_PCER);
  regval |= (1 << SAM3U_PID_SPI);
#ifdef CONFIG_SAM3U_SPIINTERRUPT
  regval |= (1 << SAM3U_IRQ_SPI);
#endif
  putreg32(regval, SAM3U_PMC_PCER);

  /* Configure multiplexed pins as connected on the board. */

  sam3u_configgpio(GPIO_SPI0_MISO);
  sam3u_configgpio(GPIO_SPI0_MOSI);
  sam3u_configgpio(GPIO_SPI0_SPCK);
  sam3u_configgpio(GPIO_SPI0_NPCS2_PC14);

  /* Execute a software reset of the SPI twice */

  putreg32(SPI_CR_SWRST, SAM3U_SPI_CR);
  putreg32(SPI_CR_SWRST, SAM3U_SPI_CR);

  /* Configure clocking */
#warning "Missing logic - Check SPI MR register"

  irqrestore(flags);
  
  /* Configure 8-bit SPI mode and master mode */
#warning "Missing logic"

  /* Set the initial SPI configuration */

#ifndef CONFIG_SPI_OWNBUS
  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;
#endif

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

#ifndef CONFIG_SPI_OWNBUS
  sem_init(&priv->exclsem, 0, 1);
#endif
  return &priv->spidev;
}
#endif /* CONFIG_SAM3U_SPI */
