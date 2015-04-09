/****************************************************************************
 * drivers/spi/spi_bitbang.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_bitbang.h>

#ifdef CONFIG_SPI_BITBANG

 /****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* This file holds the static, device-independ portion of the generica SPI-
 * bit-bang driver.  The full driver consists of 5 files:
 *
 * 1. drivers/spi/spi_bitbang.c:  This file.  This file holds the basic
 *    SPI driver framework and not perform any direct bit-bang operations.
 *    Rather, it will could out to board-specific logic to perform the
 *    low level data transfers.
 * 2. include/nuttx/spi/spi_bitbang.h:  This header file provides the
 *    data types and function prototypes needed to utilize the logic in
 *    this file.
 * 3. configs/<board>/src/<file>:  The implementation of the low-level
 *    bit-bang logic resides in some file in the board source directory.
 *    This board-specific logic includes the bit-bang skeleton logic
 *    provided in include/nuttx/spi/spi_bitband.c.
 * 4. include/nuttx/spi/spi_bitband.c.  Despite the .c extension, this
 *    really an included file.  It is used in this way:  1) The board-
 *    specific logic in configs/<board>/src/<file> provides some definitions
 *    then 2) includes include/nuttx/spi/spi_bitband.c.  That file will
 *    then use those definitions to implement the low-level bit-bang
 *    logic.  the board-specific logic then calls spi_create_bitbang()
 *    in this file to instantiate the complete SPI driver.
 *
 *    See include/nuttx/spi/spi_bitband.c for more detailed usage
 *    information.
 */

/* Debug ********************************************************************/
/* Check if SPI debut is enabled (non-standard.. no support in
 * include/debug.h
 */

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

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI methods */

#ifndef CONFIG_SPI_OWNBUS
static int      spi_lock(FAR struct spi_dev_s *dev, bool lock);
#endif
static void     spi_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                  bool selected);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                  uint32_t frequency);
static void     spi_setmode(FAR struct spi_dev_s *dev,
                  enum spi_mode_e mode);
static void     spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t ch);
static void     spi_exchange(FAR struct spi_dev_s *dev,
                   FAR const void *txbuffer, FAR void *rxbuffer,
                   size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void     spi_sndblock(FAR struct spi_dev_s *dev,
                  FAR const void *buffer, size_t nwords);
static void     spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                  size_t nwords);
#endif
static uint8_t  spi_status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
static int      spi_cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                  bool cmd);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SPI driver operations */

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
  .registercallback  = 0,                 /* Not implemented */
};

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
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;

  spivdbg("lock=%d\n", lock);
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
 * Name: spi_select
 *
 * Description:
 *   Set/clear the chip select line for the selected device.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   devid    - Identifies the device to be selected
 *   selected - select or de-select device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                       bool selected)
{
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;

  spivdbg("devid=%d selected=%d\n", devid, selected);
  DEBUGASSERT(priv && priv->low->select);
  priv->low->select(priv, devid, selected);
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

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;
  uint32_t actual;

  DEBUGASSERT(priv && priv->low->setfrequency);
  actual = priv->low->setfrequency(priv, frequency);
  spivdbg("frequency=%d holdtime=%d actual=%d\n",
          frequency, priv->holdtime, actual);
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
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;

  DEBUGASSERT(priv && priv->low->setmode);
  priv->low->setmode(priv, mode);
  spivdbg("mode=%d exchange=%p\n", mode, priv->exchange);
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
#ifdef CONFIG_SPI_BITBANG_VARWIDTH
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;

  spivdbg("nbits=%d\n", nbits);
  DEBUGASSERT(priv && nbits > 0 && nbits <= 16);
  priv->nbits = nbits;
#else
  spivdbg("nbits=%d\n", nbits);
  DEBUGASSERT(nbits == 8);
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
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;
  DEBUGASSERT(priv && priv->low && priv->low->exchange);

  return priv->low->exchange(priv, wd);
}

/****************************************************************************
 * Name: spi_exchange
 *
 * Description:
 *   Exahange a block of data from SPI. Required.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to recieve data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_exchange(FAR struct spi_dev_s *dev,
                         FAR const void *txbuffer, FAR void *rxbuffer,
                         size_t nwords)
{
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;
  FAR const uint8_t *src = (FAR const uint8_t *)txbuffer;
  FAR uint8_t *dest = (FAR uint8_t *)rxbuffer;
  uint16_t dataout;
  uint16_t datain;

  spivdbg("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);
  DEBUGASSERT(priv && priv->low && priv->low->exchange);

  /* If there is no data source, send 0xff */

  if (!src)
    {
      dataout = 0xff;
    }

  /* Exchange each word */

  while (nwords-- > 0)
    {
      /* If there is source data, get the next word from the source */

      if (src)
        {
          dataout = (uint16_t)*src++;

#ifdef CONFIG_SPI_BITBANG_VARWIDTH
          if (priv->nbits > 8)
           {
#ifdef CONFIG_ENDIAN_BIG
             dataout <<= 8;
             dataout |= *src++;
#else
             dataout |= (uint16_t)(*src++) << 8;
#endif
           }
#endif
        }

      /* Exchange the word of data */

      datain = priv->low->exchange(priv, dataout);

      /* If there is a data sink, transfer the data to the receive buffer */

      if (dest)
        {
#ifdef CONFIG_SPI_BITBANG_VARWIDTH
          if (priv->nbits > 8)
            {
#ifdef CONFIG_ENDIAN_BIG
             *dest++ = (uint8_t)(datain >> 8);
             *dest++ = (uint8_t)datain;
#else
             *dest++ = (uint8_t)datain;
             *dest++ = (uint8_t)(datain >> 8);
#endif
            }
#else
          *dest++ = (uint8_t)datain;
#endif
        }
    }
}

/***************************************************************************
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

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords)
{
  /* spi_exchange can do this. */

  spi_exchange(dev, buffer, NULL, nwords);
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
  /* spi_exchange can do this. */

  spi_exchange(dev, NULL, buffer, nwords);
}
#endif

/****************************************************************************
 * Name: spi_status
 *
 * Description:
 *   Get status bits associated with the device associated with 'devid'
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   devid - Identifies the device of interest
 *
 * Returned Value:
 *   Bit encoded status byte
 *
 ****************************************************************************/

static uint8_t spi_status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;
  DEBUGASSERT(priv && priv->low && priv->low->status);

  return priv->low->status(priv, devid);
}

/****************************************************************************
 * Name: spi_cmddata
 *
 * Description:
 *   Control the SPI CMD/DATA like for the device associated with 'devid'
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   devid - Identifies the device of interest
 *   cmd   - True:CMD False:DATA
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                       bool cmd)
{
  FAR struct spi_bitbang_s *priv = (FAR struct spi_bitbang_s *)dev;
  DEBUGASSERTcmddata(priv && priv->low && priv->low->status);

  return priv->low->cmddata(priv, devid, cmd);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  spi_create_bitbang
 *
 * Descripton:
 *   Create an instance of the SPI bit-bang driver.
 *
 ****************************************************************************/

FAR struct spi_dev_s *spi_create_bitbang(FAR const struct spi_bitbang_ops_s *low)
{
  FAR struct spi_bitbang_s *priv;

  DEBUGASSERT(low);

  /* Allocate an instance of the SPI bit bang structure */

  priv = (FAR struct spi_bitbang_s *)zalloc(sizeof(struct spi_bitbang_s));
  if (!priv)
    {
      spidbg("Failed to allocate the device structure\n");
      return NULL;
    }

  /* Initialize the driver structure */

  priv->dev.ops = &g_spiops;
  priv->low     = low;
#ifdef CONFIG_SPI_BITBANG_VARWIDTH
  priv->nbits   = 8;
#endif

  sem_init(&priv->exclsem, 0, 1);

  /* Select an initial state of mode 0, 8-bits, and 400KHz */

  low->setmode(priv, SPIDEV_MODE0);
  low->setfrequency(priv, 400000);

  /* And return the initialized driver structure */

  return &priv->dev;
}

#endif /* CONFIG_SPI_BITBANG */
